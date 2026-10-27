//! Cooperative unreset-reversal robustness phase.
//!
//! For each reversal direction, runs up to [`ROBUSTNESS_CYCLES_PER_DIRECTION`] probe
//! cycles: an establish leg (integrator cleared) then a measured reversal leg
//! (integrator not cleared). Metrics fold into a per-direction [`CycleLedger`]. The
//! accepted point is read-only; the phase never mutates the retained Integral slot.

use crate::autotune::CommissionError;
use crate::autotune::CommissionStatus;
use crate::autotune::TmcMechanicalRpm;
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::velocity_measurement::{DyadicBlocker, StationarityWork};
use crate::autotune::velocity_measurement::{
    StationarityResult, VELOCITY_RATE_Q_SCALE, VelocityObservationSource, estimate_confidence,
};
use crate::commission_dispatch::robustness::{
    ROBUSTNESS_CAUSE_ANALYSIS_POLL_FAILED, ROBUSTNESS_CAUSE_ANALYSIS_TIMEOUT,
    ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW, ROBUSTNESS_CAUSE_DISPATCH_ABORT,
    ROBUSTNESS_CAUSE_INVALID_GAINS, ROBUSTNESS_CAUSE_IO, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED,
    ROBUSTNESS_CAUSE_ORIGIN_RECOVERY_FAULT, ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID,
    ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED, ROBUSTNESS_CAUSE_SAFETY_ARM_FAILED,
    ROBUSTNESS_CAUSE_SAFETY_FAULT, ROBUSTNESS_CAUSE_SCORING_FAILED, ROBUSTNESS_CAUSE_TAIL_REPEATED,
    ROBUSTNESS_CAUSE_UNEXPECTED_STATE,
};
use crate::commissioning::OuterGainRollbackSnapshot;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::OuterSafetyLimits;
use crate::commissioning::outer::state::VELOCITY_SWEEP_BLOCK_LEVELS;
use crate::commissioning::outer::state::VelocityObservationRing;
use crate::commissioning::outer::velocity::robustness_reporting::ROBUSTNESS_SCHEMA_REVISION;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_QUARTER_TURN, OuterResult,
    OuterSafetyFault, RobustnessOutcome, RobustnessReversalTerminal, StepResult, TuneProvenance,
};
use crate::tmc_register_values::MOTION_VELOCITY_ADVANCED_PI;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_VELOCITY_ACTUAL, PID_VELOCITY_OFFSET,
    PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use super::band_confirmation::{CONFIRMATION_BAND_LOWER_PERCENT, CONFIRMATION_BAND_UPPER_PERCENT};
use super::integral_reproduction::AcceptancePoint;
use super::peak_ratio::scored_peak_permille;
use super::recovery::{
    RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryStage, RungOriginRecoveryConfig,
    RungOriginRecoveryOutcome, RungOriginRecoveryPlan, RungOriginRecoveryResources,
    RungOriginRecoveryRuntime, RungOriginRecoveryStep, commanded_side_lower_rate_q,
    plan_rung_origin_recovery, step_rung_origin_recovery,
};
use super::robustness_cycles::{
    CycleLedger, CycleRecord, LedgerAction, ROBUSTNESS_CYCLES_PER_DIRECTION,
};
use super::robustness_evidence::{RobustnessDirectionSummary, RobustnessEvidence, summary_cause};
use super::robustness_late_tracking::{LateTracking, LateTrackingAccumulator};
use super::robustness_metrics::{
    PROBE_BAND_WINDOW_SAMPLES, REVERSAL_PEAK_REJECT_PERMILLE, ROBUSTNESS_BAND_DWELL_US,
    ROBUSTNESS_BAND_FRACTION_PCT, ROBUSTNESS_LEG_AMPLITUDE_PCT, ROBUSTNESS_MEASUREMENT_BUDGET_US,
    ReversalMetrics, ReversalOnlineAccumulator, RobustnessThresholds, TRIP_PEAK, dac_rms_q,
    reduce_plant_rate_q, robustness_thresholds_for, tripped_mask,
};
use super::robustness_reporting::CycleWire;
use super::sweep_analysis::poll_stationarity;
use super::sweep_analysis::{ScoredRestPoll, deficit_interval_q, poll_scored_rest_confirmation};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, SETTLE_ANALYSIS_BUDGET_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US,
};
use super::target_ramp::{RampRate, TargetRamp, next_ramp_tick_us};
use super::velocity_clamp::{VelocityCurrentObserver, VelocityLimitEvidenceCapability};
use super::{clear_velocity_error_sum, clear_velocity_offset};

const ROBUSTNESS_SETTLE_TAIL_US: u32 = 80_000;

const ROBUSTNESS_CONFIDENCE_FAMILY_SIZE: u16 = 3;

const DIRECTION_COUNT: usize = 2;

/// Borrowed single-leg scratch space for capture, stationarity analysis, and band-entry/IAE
/// accumulation.
#[derive(Debug)]
pub(crate) struct RobustnessReversalWorkspace {
    observations: VelocityObservationRing,
    blocker: DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    analysis: StationarityWork,
    capture: ReversalOnlineAccumulator,
    late_tracking: Option<LateTrackingAccumulator>,
    previous_encoder_raw: u32,
    previous_read_us: u64,
    current_observer: VelocityCurrentObserver,
    settle_displacement_counts: i8,
    settle_min_displacement_counts: i8,
    settle_max_displacement_counts: i8,
    last_target_rpm: i32,
}

impl RobustnessReversalWorkspace {
    /// Construct an empty workspace.
    pub(crate) const fn new() -> Self {
        Self {
            observations: VelocityObservationRing::new(),
            blocker: DyadicBlocker::new(),
            analysis: StationarityWork::new(),
            capture: ReversalOnlineAccumulator::new(),
            late_tracking: None,
            previous_encoder_raw: 0,
            previous_read_us: 0,
            current_observer: VelocityCurrentObserver::EMPTY,
            settle_displacement_counts: 0,
            settle_min_displacement_counts: 0,
            settle_max_displacement_counts: 0,
            last_target_rpm: 0,
        }
    }

    fn reset_current_observer_from_operational_limit<B: CommissioningBackend>(
        &mut self,
        backend: &B,
    ) {
        self.current_observer.reset(
            backend.current_limit(),
            VelocityLimitEvidenceCapability::BestEffortPolling,
        );
    }
}

impl defmt::Format for RobustnessReversalWorkspace {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "RobustnessReversalWorkspace");
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingCycle {
    /// Raw per-cycle metrics folded into the ledger.
    metrics: ReversalMetrics,
    /// Wire triple mirroring this cycle's ledger slot.
    wire: CycleWire,
    /// Scored reversal peak in permille of the leg target; 0 when not scored.
    peak_permille: u16,
    /// Whether the leg's stationary suffix was excluded from the settled-tail metrics.
    suffix_excluded: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
enum HeldRoute {
    Ledger(LedgerAction),
    ReenterEstablish,
}

/// Frozen accepted point plus the mutable per-direction robustness progress.
#[derive(Debug, defmt::Format)]
pub(crate) struct RobustnessReversalSession {
    point: AcceptancePoint,
    run_sequence: u32,
    ramp_rate: RampRate,
    run_started_us: u64,
    direction: usize,
    ledgers: [CycleLedger; DIRECTION_COUNT],
    /// Maximum scored reversal velocity peak per direction, in permille of the leg target.
    peak_rate_permille: [u16; DIRECTION_COUNT],
    /// Count of scored reversal legs per direction.
    peak_scored_cycles: [u8; DIRECTION_COUNT],
    /// Per-cycle wire triples, mirroring each ledger's filled slots.
    cycle_wires: [[Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT],
    /// The grace-retried (displaced) tail cycle wire triple, per direction.
    displaced_tail_wires: [Option<CycleWire>; DIRECTION_COUNT],
    /// Per-slot stationary-suffix exclusion, mirroring each ledger's filled slots.
    suffix_excluded: [[bool; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT],
    /// Slot cursor mirroring each ledger cursor.
    cycle_cursor: [usize; DIRECTION_COUNT],
    pending_forward_settle_us: u32,
    /// The conclusive cycle awaiting the between-cycles rest confirmation.
    pending_cycle: Option<PendingCycle>,
    held_route: Option<HeldRoute>,
    origin_recovered: bool,
    recovery_plan: Option<RungOriginRecoveryPlan>,
}

impl RobustnessReversalSession {
    /// Freeze the accepted point for a robustness-reversal run. Reads the point
    /// only; never mutates the candidate slot. Every leg target is ramped at `ramp_rate`.
    pub(crate) fn new(
        point: AcceptancePoint,
        run_sequence: u32,
        counts_per_rev: u32,
        ramp_rate: RampRate,
    ) -> Self {
        let (_, thresholds) = constructed_thresholds(&point, counts_per_rev);
        let iae_max_q = thresholds.iae_max_q;
        Self {
            point,
            run_sequence,
            ramp_rate,
            run_started_us: 0,
            direction: 0,
            ledgers: [CycleLedger::new(iae_max_q), CycleLedger::new(iae_max_q)],
            peak_rate_permille: [0; DIRECTION_COUNT],
            peak_scored_cycles: [0; DIRECTION_COUNT],
            cycle_wires: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT],
            displaced_tail_wires: [None; DIRECTION_COUNT],
            suffix_excluded: [[false; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT],
            cycle_cursor: [0; DIRECTION_COUNT],
            pending_forward_settle_us: 0,
            pending_cycle: None,
            held_route: None,
            origin_recovered: false,
            recovery_plan: None,
        }
    }
}

/// Cooperative state for the unreset-reversal robustness phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RobustnessReversalState {
    /// Arm the safety baseline before any motion.
    Setup,
    /// Apply the accepted gains, ramp mode, and clear the integrator (entry leg).
    EstablishWriteGain,
    /// Ramp to the signed establish-cruise target for the current direction.
    EstablishRamp {
        ramp: TargetRamp,
        next_tick_us: u64,
    },
    /// Collect the establish-cruise capture: search for band entry up to the
    /// budget, then sample a settled tail for the stationarity analysis.
    EstablishCapture {
        next_sample_us: u64,
        budget_deadline_us: u64,
        settle_deadline_us: Option<u64>,
    },
    /// Analyze the establish capture until it yields a Usable stationary settle.
    EstablishAnalyze {
        analysis_deadline_us: u64,
    },
    /// Ramp the commanded target through the reversal WITHOUT clearing the integrator.
    ReverseRamp {
        ramp: TargetRamp,
        next_tick_us: u64,
    },
    /// Collect the measured reversal capture until the budget deadline, regardless of when the
    /// band is entered.
    ReverseCapture {
        next_sample_us: u64,
        budget_deadline_us: u64,
    },
    /// Analyze the reversal capture for band-entry reconvergence.
    ReverseAnalyze {
        analysis_deadline_us: u64,
    },
    /// Ramp the target to zero, then open the between-legs rest window. `retry_available`
    /// tracks whether one rejected-rest re-open remains.
    ZeroRamp {
        ramp: TargetRamp,
        next_tick_us: u64,
        retry_available: bool,
    },
    /// Collect the zero-target rest capture until its window elapses.
    ZeroCapture {
        next_sample_us: u64,
        deadline_us: u64,
        retry_available: bool,
    },
    /// Confirm rest between legs via scored-rest analysis.
    ZeroAnalyze {
        analysis_deadline_us: u64,
        retry_available: bool,
    },
    RecoverOrigin(RungOriginRecoveryRuntime),
    /// Assemble evidence and emit the terminal.
    Assemble,
    /// The terminal has been emitted; further steps are inert.
    Done,
}

enum TickFault {
    /// Register I/O or encoder-domain failure.
    Io,
    /// Outer-safety envelope violation.
    Safety(OuterSafetyFault),
}

fn amplitude_rpm(point: &AcceptancePoint) -> i32 {
    point.target_velocity_rpm.saturating_abs()
}

fn leg_amplitude_rpm(point: &AcceptancePoint) -> i32 {
    leg_amplitude_for_rpm(point.target_velocity_rpm)
}

fn leg_amplitude_for_rpm(target_velocity_rpm: i32) -> i32 {
    let scaled = i64::from(target_velocity_rpm.saturating_abs())
        * i64::from(ROBUSTNESS_LEG_AMPLITUDE_PCT)
        + 50;
    i32::try_from(scaled / 100).unwrap_or(i32::MAX)
}

/// Total ramp time of one robustness cycle at `target_velocity_rpm`: the establish ramp from
/// rest, the reversal through zero and the ramp back to rest, each rounded up to whole ticks.
pub(crate) fn robustness_cycle_ramp_us(target_velocity_rpm: i32, rate: RampRate) -> u32 {
    let leg = leg_amplitude_for_rpm(target_velocity_rpm).unsigned_abs();
    let one_leg_us = rate.ramp_us_for_span_rpm(leg);
    one_leg_us
        .saturating_mul(2)
        .saturating_add(rate.ramp_us_for_span_rpm(leg.saturating_mul(2)))
}

fn reversal_ramp_us(point: &AcceptancePoint, rate: RampRate) -> u32 {
    rate.ramp_us_for_span_rpm(leg_amplitude_rpm(point).unsigned_abs().saturating_mul(2))
}

fn establish_target_rpm(point: &AcceptancePoint, direction: usize) -> i32 {
    let amplitude = leg_amplitude_rpm(point);
    if direction == 0 {
        amplitude
    } else {
        -amplitude
    }
}

fn reverse_target_rpm(point: &AcceptancePoint, direction: usize) -> i32 {
    -establish_target_rpm(point, direction)
}

fn reverse_target_rate_q(point: &AcceptancePoint, direction: usize, counts_per_rev: u32) -> i64 {
    target_rate_q(reverse_target_rpm(point, direction), counts_per_rev)
}

const ESTABLISH_LEG_WINDOW_US: u32 =
    ROBUSTNESS_MEASUREMENT_BUDGET_US + ROBUSTNESS_SETTLE_TAIL_US + SETTLE_ANALYSIS_BUDGET_US;

fn establish_rate_counts_per_ms(point: &AcceptancePoint, counts_per_rev: u32) -> u32 {
    let counts_per_minute = u64::from(leg_amplitude_rpm(point).unsigned_abs())
        .saturating_mul(u64::from(counts_per_rev));
    u32::try_from(counts_per_minute.div_ceil(60_000)).unwrap_or(u32::MAX)
}

fn establish_leg_travel_counts(rate_counts_per_ms: u32, ramp_us: u32) -> u32 {
    let window_ms =
        u64::from(ESTABLISH_LEG_WINDOW_US).div_ceil(1_000) + u64::from(ramp_us).div_ceil(1_000);
    u32::try_from(u64::from(rate_counts_per_ms).saturating_mul(window_ms)).unwrap_or(u32::MAX)
}

fn leg_travel_counts(point: &AcceptancePoint, counts_per_rev: u32, rate: RampRate) -> u32 {
    establish_leg_travel_counts(
        establish_rate_counts_per_ms(point, counts_per_rev),
        reversal_ramp_us(point, rate),
    )
}

fn remaining_headroom_counts(
    unwrapped_position: i32,
    negative_window: u32,
    positive_window: u32,
    direction: usize,
) -> u32 {
    let position = i64::from(unwrapped_position);
    let remaining = if direction == 0 {
        i64::from(positive_window) - position
    } else {
        position + i64::from(negative_window)
    };
    u32::try_from(remaining.max(0)).unwrap_or(u32::MAX)
}

fn cycle_fits(
    leg_travel: u32,
    unwrapped_position: i32,
    negative_window: u32,
    positive_window: u32,
    direction: usize,
) -> bool {
    [direction, 1 - direction].into_iter().all(|leg_direction| {
        remaining_headroom_counts(
            unwrapped_position,
            negative_window,
            positive_window,
            leg_direction,
        ) >= leg_travel
    })
}

fn target_rate_q(signed_target_rpm: i32, counts_per_rev: u32) -> i64 {
    let numerator = i128::from(signed_target_rpm)
        .saturating_mul(i128::from(counts_per_rev))
        .saturating_mul(i128::from(VELOCITY_RATE_Q_SCALE));
    let rate = numerator / 60;
    i64::try_from(rate).unwrap_or(if rate.is_negative() {
        i64::MIN
    } else {
        i64::MAX
    })
}

fn band_q_for(target_rate_q: i64) -> i64 {
    let magnitude = i128::from(target_rate_q.unsigned_abs());
    let band = magnitude * i128::from(ROBUSTNESS_BAND_FRACTION_PCT) / 100;
    i64::try_from(band).unwrap_or(i64::MAX)
}

fn reconvergence_ratio_ppm(reconvergence_time_us: u32, forward_settle_time_us: u32) -> u32 {
    if forward_settle_time_us == 0 {
        return 0;
    }
    let ratio = u64::from(reconvergence_time_us).saturating_mul(1_000_000)
        / u64::from(forward_settle_time_us);
    u32::try_from(ratio).unwrap_or(u32::MAX)
}

fn settled_rate_interval_q(
    suffix: &crate::autotune::velocity_measurement::StationarySuffix,
) -> Result<(i64, i64), CommissionError> {
    let mean = suffix
        .selection
        .exact_totals
        .rate_q()
        .map_err(|_| CommissionError::VelocityRateCalculationFailed)?;
    let confidence = estimate_confidence(suffix.selection, ROBUSTNESS_CONFIDENCE_FAMILY_SIZE)
        .map_err(|_| CommissionError::VelocityConfidenceEstimationFailed)?;
    let half_width = i64::try_from(confidence.half_width_rate_q)
        .map_err(|_| CommissionError::InvalidSchedule)?;
    Ok((
        mean.saturating_sub(half_width),
        mean.saturating_add(half_width),
    ))
}

struct LegScore<'a> {
    point: &'a AcceptancePoint,
    direction: usize,
    counts_per_rev: u32,
    forward_settle_time_us: u32,
    reconvergence_time_us: u32,
    recovery_iae: i64,
    late_tracking: LateTracking,
    suffix: Option<&'a crate::autotune::velocity_measurement::StationarySuffix>,
    ring: &'a VelocityObservationRing,
}

fn score_reversal_leg(input: LegScore<'_>) -> Result<ReversalMetrics, CommissionError> {
    let LegScore {
        point,
        direction,
        counts_per_rev,
        forward_settle_time_us,
        reconvergence_time_us,
        recovery_iae,
        late_tracking,
        suffix,
        ring,
    } = input;
    let signed_target_rpm = reverse_target_rpm(point, direction);
    let target_rate_q_value = reverse_target_rate_q(point, direction, counts_per_rev);
    let (settled_residual_q, dac_rms) = match suffix {
        Some(suffix) => {
            let (settled_low, settled_high) = settled_rate_interval_q(suffix)?;
            let (deficit_low, deficit_high) =
                deficit_interval_q(signed_target_rpm, counts_per_rev, settled_low, settled_high)?;
            let suffix_len = usize::from(suffix.suffix_len);
            let start_index = ring.observation_len().saturating_sub(suffix_len);
            (
                deficit_low.saturating_add(deficit_high) / 2,
                dac_rms_q(ring, start_index, suffix_len, target_rate_q_value),
            )
        }
        None => (0, 0),
    };
    Ok(ReversalMetrics {
        reconvergence_time_us,
        settled_residual_q,
        recovery_iae_q: recovery_iae,
        dac_rms_q: dac_rms,
        forward_settle_time_us,
        reconvergence_ratio_ppm: reconvergence_ratio_ppm(
            reconvergence_time_us,
            forward_settle_time_us,
        ),
        late_tracking,
    })
}

async fn observe_tick<B: CommissioningBackend>(
    workspace: &mut RobustnessReversalWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<(u64, u32, i32), TickFault> {
    let (read_us, elapsed_us, signed) = sample_tick(workspace, safety, backend).await?;
    workspace
        .observations
        .push(signed, elapsed_us as u16)
        .map_err(|_| TickFault::Io)?;
    Ok((read_us, elapsed_us, signed))
}

/// Reads one tick's encoder and runs the per-tick observation-gap and quarter-turn checks,
/// advancing the previous read without recording the observation.
async fn sample_tick<B: CommissioningBackend>(
    workspace: &mut RobustnessReversalWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<(u64, u32, i32), TickFault> {
    let encoder_raw = backend
        .read_capture_register(ABN_DECODER_COUNT)
        .await
        .map_err(|_| TickFault::Io)?;
    let read_us = backend.elapsed_us();
    let elapsed_us = match read_us.checked_sub(workspace.previous_read_us) {
        Some(value @ 1..=MAX_SAFETY_OBSERVATION_GAP_US) => value,
        _ => {
            return Err(TickFault::Safety(safety.fault(
                OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                0,
                read_us.saturating_sub(workspace.previous_read_us),
                0,
                0,
            )));
        }
    };
    let domain = EncoderDomain::new(safety.counts_per_rev()).map_err(|_| TickFault::Io)?;
    let delta = match domain.delta(
        domain.count(workspace.previous_encoder_raw),
        domain.count(encoder_raw),
    ) {
        Ok(delta) if delta.absolute_counts() < safety.quarter_turn_fault_counts => delta,
        _ => {
            return Err(TickFault::Safety(safety.fault(
                OUTER_SAFETY_FAULT_QUARTER_TURN,
                0,
                elapsed_us,
                0,
                0,
            )));
        }
    };
    let signed = delta.signed_counts();
    workspace.previous_encoder_raw = encoder_raw;
    workspace.previous_read_us = read_us;
    Ok((read_us, elapsed_us as u32, signed))
}

/// Advance the robustness-reversal phase by one cooperative step.
///
/// The caller owns `state`, `workspace`, and `session`; the engine runs outer safety
/// checks between ticks.
pub(crate) async fn step_robustness_reversal<B: CommissioningBackend>(
    state: &mut RobustnessReversalState,
    workspace: &mut RobustnessReversalWorkspace,
    session: &mut RobustnessReversalSession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> StepResult {
    let counts_per_rev = safety.counts_per_rev();
    match *state {
        RobustnessReversalState::Setup => {
            let baseline = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => {
                    return fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_IO,
                        counts_per_rev,
                    );
                }
            };
            if safety.set_encoder_baseline(baseline).is_err() {
                return fault_terminal(
                    session,
                    backend,
                    None,
                    ROBUSTNESS_CAUSE_SAFETY_ARM_FAILED,
                    counts_per_rev,
                );
            }
            safety.torque_cap = u16::from(backend.current_limit());
            let now_us = backend.elapsed_us();
            if safety.arm_safety_observation(now_us).is_err() {
                return fault_terminal(
                    session,
                    backend,
                    None,
                    ROBUSTNESS_CAUSE_SAFETY_ARM_FAILED,
                    counts_per_rev,
                );
            }
            session.run_started_us = now_us;
            workspace.previous_encoder_raw = baseline;
            workspace.previous_read_us = now_us;
            let plan = match robustness_recovery_plan(&session.point, safety) {
                Ok(plan) => plan,
                Err(error) => {
                    defmt::warn!("robustness recovery cannot be planned: {}", error);
                    return fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID,
                        counts_per_rev,
                    );
                }
            };
            let leg_travel = leg_travel_counts(&session.point, counts_per_rev, session.ramp_rate);
            let usable = safety
                .negative_position_window
                .min(safety.positive_position_window)
                .saturating_sub(plan.origin_band_counts);
            if leg_travel > usable {
                defmt::warn!(
                    "robustness schedule cannot fit from origin: leg_travel={} usable={} windows=-{}/+{} band={}",
                    leg_travel,
                    usable,
                    safety.negative_position_window,
                    safety.positive_position_window,
                    plan.origin_band_counts
                );
                return fault_terminal(
                    session,
                    backend,
                    None,
                    ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID,
                    counts_per_rev,
                );
            }
            workspace.reset_current_observer_from_operational_limit(backend);
            session.recovery_plan = Some(plan);
            *state = RobustnessReversalState::EstablishWriteGain;
            StepResult::ReadyNow
        }
        RobustnessReversalState::EstablishWriteGain => {
            let leg_travel = leg_travel_counts(&session.point, counts_per_rev, session.ramp_rate);
            if !cycle_fits(
                leg_travel,
                safety.unwrapped_position,
                safety.negative_position_window,
                safety.positive_position_window,
                session.direction,
            ) {
                defmt::warn!(
                    "robustness cycle does not fit: dir={} slot={} position={} remaining_establish={} remaining_reverse={} leg_travel={} origin_recovered={}",
                    session.direction,
                    session.cycle_cursor[session.direction],
                    safety.unwrapped_position,
                    remaining_headroom_counts(
                        safety.unwrapped_position,
                        safety.negative_position_window,
                        safety.positive_position_window,
                        session.direction,
                    ),
                    remaining_headroom_counts(
                        safety.unwrapped_position,
                        safety.negative_position_window,
                        safety.positive_position_window,
                        1 - session.direction,
                    ),
                    leg_travel,
                    session.origin_recovered
                );
                if session.origin_recovered {
                    return origin_not_recovered_terminal(session, backend, counts_per_rev);
                }
                session.held_route = Some(HeldRoute::ReenterEstablish);
                *state = zero_ramp_state(session, workspace.last_target_rpm);
                return StepResult::ReadyNow;
            }
            let (p, i) = match accepted_gains(&session.point) {
                Ok(pair) => pair,
                Err(_) => {
                    return fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_INVALID_GAINS,
                        counts_per_rev,
                    );
                }
            };
            if clear_velocity_offset(backend).await.is_err()
                || backend
                    .write_register(
                        PID_VELOCITY_P_VELOCITY_I,
                        PidGainPair::new(p, i).register_value(),
                    )
                    .await
                    .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_VELOCITY_ADVANCED_PI)
                    .await
                    .is_err()
                || clear_velocity_error_sum(backend).await.is_err()
            {
                return fault_terminal(session, backend, None, ROBUSTNESS_CAUSE_IO, counts_per_rev);
            }
            *state = RobustnessReversalState::EstablishRamp {
                ramp: TargetRamp::new(
                    workspace.last_target_rpm,
                    establish_target_rpm(&session.point, session.direction),
                    session.ramp_rate,
                ),
                next_tick_us: 0,
            };
            StepResult::ReadyNow
        }
        RobustnessReversalState::EstablishRamp {
            mut ramp,
            mut next_tick_us,
        } => {
            if !ramp.is_started() {
                session.origin_recovered = false;
            }
            let leg = LegRamp {
                ramp: &mut ramp,
                next_tick_us: &mut next_tick_us,
                opens_primitive: true,
            };
            match advance_leg_ramp(leg, session, workspace, safety, backend).await {
                Ok(LegRampTick::Pending(at_us)) => {
                    *state = RobustnessReversalState::EstablishRamp { ramp, next_tick_us };
                    StepResult::At(at_us)
                }
                Ok(LegRampTick::Ended(written_us)) => {
                    let target = establish_target_rpm(&session.point, session.direction);
                    let leg = LegCapture {
                        signed_target_rpm: target,
                        started_us: written_us,
                    };
                    match open_leg_capture(session, workspace, safety, backend, leg) {
                        Ok((next_sample_us, budget_deadline_us)) => {
                            *state = RobustnessReversalState::EstablishCapture {
                                next_sample_us,
                                budget_deadline_us,
                                settle_deadline_us: None,
                            };
                            StepResult::At(next_sample_us)
                        }
                        Err(step) => step,
                    }
                }
                Err(step) => step,
            }
        }
        RobustnessReversalState::EstablishCapture {
            next_sample_us,
            budget_deadline_us,
            settle_deadline_us,
        } => {
            if backend.elapsed_us() < next_sample_us {
                return StepResult::At(next_sample_us);
            }
            let (read_us, elapsed_us, delta) = match observe_tick(workspace, safety, backend).await
            {
                Ok(sample) => sample,
                Err(fault) => return tick_fault_terminal(session, backend, fault, counts_per_rev),
            };
            let advance = advance_capture(
                &mut workspace.capture,
                read_us,
                elapsed_us,
                delta,
                next_sample_us,
                budget_deadline_us,
                settle_deadline_us,
            );
            if let Some(entry) = workspace.capture.band_entry_us() {
                session.pending_forward_settle_us = entry;
            }
            match advance {
                CaptureAdvance::Continue {
                    next_sample_us,
                    settle_deadline_us,
                } => {
                    *state = RobustnessReversalState::EstablishCapture {
                        next_sample_us,
                        budget_deadline_us,
                        settle_deadline_us,
                    };
                    StepResult::At(next_sample_us)
                }
                CaptureAdvance::Converged => {
                    workspace.analysis.reset();
                    let Some(analysis_deadline_us) = analysis_deadline(read_us) else {
                        return fault_terminal(
                            session,
                            backend,
                            None,
                            ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
                            counts_per_rev,
                        );
                    };
                    *state = RobustnessReversalState::EstablishAnalyze {
                        analysis_deadline_us,
                    };
                    StepResult::ReadyNow
                }
                CaptureAdvance::Exhausted => {
                    let velocity_offset = backend
                        .read_register(PID_VELOCITY_OFFSET)
                        .await
                        .map(|value| value as i32)
                        .unwrap_or(i32::MIN);
                    let velocity_actual = backend
                        .read_capture_register(PID_VELOCITY_ACTUAL)
                        .await
                        .map(|value| value as i32)
                        .unwrap_or(i32::MIN);
                    defmt::info!(
                        "robustness_reversal establish leg exhausted: dir={} slot={} read_us={} budget_deadline_us={} elapsed_us={} retries_used={} vel_to_target_pct={} peak_band_excess={} velocity_offset={} velocity_actual={}",
                        session.direction,
                        session.cycle_cursor[session.direction],
                        read_us,
                        budget_deadline_us,
                        elapsed_us,
                        session.ledgers[session.direction].inconclusive_retries_used(),
                        workspace.capture.velocity_to_target_pct(),
                        workspace.capture.reversal_overshoot_counts(),
                        velocity_offset,
                        velocity_actual,
                    );
                    retry_or_finalize_inconclusive(state, session, workspace.last_target_rpm);
                    StepResult::ReadyNow
                }
            }
        }
        RobustnessReversalState::EstablishAnalyze {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= analysis_deadline_us {
                return fault_terminal(
                    session,
                    backend,
                    None,
                    ROBUSTNESS_CAUSE_ANALYSIS_TIMEOUT,
                    counts_per_rev,
                );
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let stationarity = match poll {
                Ok(None) => return StepResult::ReadyNow,
                Ok(Some((result, _, _))) => result,
                Err(_) => {
                    return fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_ANALYSIS_POLL_FAILED,
                        counts_per_rev,
                    );
                }
            };
            match stationarity {
                StationarityResult::Usable(_) => {
                    *state = RobustnessReversalState::ReverseRamp {
                        ramp: TargetRamp::new(
                            workspace.last_target_rpm,
                            reverse_target_rpm(&session.point, session.direction),
                            session.ramp_rate,
                        ),
                        next_tick_us: 0,
                    };
                    StepResult::ReadyNow
                }
                StationarityResult::Excluded(_) => {
                    retry_or_finalize_inconclusive(state, session, workspace.last_target_rpm);
                    StepResult::ReadyNow
                }
            }
        }
        RobustnessReversalState::ReverseRamp {
            mut ramp,
            mut next_tick_us,
        } => {
            let leg = LegRamp {
                ramp: &mut ramp,
                next_tick_us: &mut next_tick_us,
                opens_primitive: false,
            };
            match advance_leg_ramp(leg, session, workspace, safety, backend).await {
                Ok(LegRampTick::Pending(at_us)) => {
                    *state = RobustnessReversalState::ReverseRamp { ramp, next_tick_us };
                    StepResult::At(at_us)
                }
                Ok(LegRampTick::Ended(written_us)) => {
                    let target = reverse_target_rpm(&session.point, session.direction);
                    let leg = LegCapture {
                        signed_target_rpm: target,
                        started_us: written_us,
                    };
                    match open_leg_capture(session, workspace, safety, backend, leg) {
                        Ok((next_sample_us, budget_deadline_us)) => {
                            *state = RobustnessReversalState::ReverseCapture {
                                next_sample_us,
                                budget_deadline_us,
                            };
                            StepResult::At(next_sample_us)
                        }
                        Err(step) => step,
                    }
                }
                Err(step) => step,
            }
        }
        RobustnessReversalState::ReverseCapture {
            next_sample_us,
            budget_deadline_us,
        } => {
            if backend.elapsed_us() < next_sample_us {
                return StepResult::At(next_sample_us);
            }
            let (read_us, elapsed_us, delta) = match observe_tick(workspace, safety, backend).await
            {
                Ok(sample) => sample,
                Err(fault) => return tick_fault_terminal(session, backend, fault, counts_per_rev),
            };
            match advance_reverse_capture(
                workspace,
                (read_us, elapsed_us, delta),
                next_sample_us,
                budget_deadline_us,
            ) {
                CaptureAdvance::Continue { next_sample_us, .. } => {
                    *state = RobustnessReversalState::ReverseCapture {
                        next_sample_us,
                        budget_deadline_us,
                    };
                    StepResult::At(next_sample_us)
                }
                CaptureAdvance::Converged => {
                    workspace.analysis.reset();
                    let Some(analysis_deadline_us) = analysis_deadline(read_us) else {
                        return fault_terminal(
                            session,
                            backend,
                            None,
                            ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
                            counts_per_rev,
                        );
                    };
                    *state = RobustnessReversalState::ReverseAnalyze {
                        analysis_deadline_us,
                    };
                    StepResult::ReadyNow
                }
                CaptureAdvance::Exhausted => {
                    defmt::info!(
                        "robustness_reversal reverse leg exhausted: dir={} slot={} read_us={} budget_deadline_us={} elapsed_us={} retries_used={}",
                        session.direction,
                        session.cycle_cursor[session.direction],
                        read_us,
                        budget_deadline_us,
                        elapsed_us,
                        session.ledgers[session.direction].inconclusive_retries_used()
                    );
                    retry_or_finalize_inconclusive(state, session, workspace.last_target_rpm);
                    StepResult::ReadyNow
                }
            }
        }
        RobustnessReversalState::ReverseAnalyze {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= analysis_deadline_us {
                return fault_terminal(
                    session,
                    backend,
                    None,
                    ROBUSTNESS_CAUSE_ANALYSIS_TIMEOUT,
                    counts_per_rev,
                );
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let stationarity = match poll {
                Ok(None) => return StepResult::ReadyNow,
                Ok(Some((result, _, _))) => result,
                Err(_) => {
                    return fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_ANALYSIS_POLL_FAILED,
                        counts_per_rev,
                    );
                }
            };
            let suffix = match stationarity {
                StationarityResult::Usable(suffix) => Some(suffix),
                StationarityResult::Excluded(_) => None,
            };
            let late_tracking = workspace
                .late_tracking
                .as_ref()
                .and_then(LateTrackingAccumulator::result);
            let (Some(reconvergence_time_us), Some(late_tracking)) =
                (workspace.capture.band_entry_us(), late_tracking)
            else {
                retry_or_finalize_inconclusive(state, session, workspace.last_target_rpm);
                return StepResult::ReadyNow;
            };
            let metrics = match score_reversal_leg(LegScore {
                point: &session.point,
                direction: session.direction,
                counts_per_rev,
                forward_settle_time_us: session.pending_forward_settle_us,
                reconvergence_time_us,
                recovery_iae: workspace.capture.recovery_iae_q(),
                late_tracking,
                suffix: suffix.as_ref(),
                ring: &workspace.observations,
            }) {
                Ok(metrics) => metrics,
                Err(_) => {
                    return fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_SCORING_FAILED,
                        counts_per_rev,
                    );
                }
            };
            let overshoot = workspace.capture.reversal_overshoot_counts();
            let peak = scored_peak_permille(
                safety.primitive_peak_milli_counts_per_s(),
                leg_amplitude_rpm(&session.point).unsigned_abs(),
                counts_per_rev,
                REVERSAL_PEAK_REJECT_PERMILLE,
            );
            session.pending_cycle = Some(PendingCycle {
                metrics,
                wire: CycleWire {
                    reconvergence_ms: to_ms_saturating(metrics.reconvergence_time_us),
                    overshoot_counts: overshoot,
                    forward_settle_ms: to_ms_saturating(metrics.forward_settle_time_us),
                },
                peak_permille: peak,
                suffix_excluded: suffix.is_none(),
            });
            *state = zero_ramp_state(session, workspace.last_target_rpm);
            StepResult::ReadyNow
        }
        RobustnessReversalState::ZeroRamp {
            mut ramp,
            mut next_tick_us,
            retry_available,
        } => {
            let first = !ramp.is_started();
            if !first && backend.elapsed_us() < next_tick_us {
                return StepResult::At(next_tick_us);
            }
            if write_next_ramp_target(&mut ramp, workspace, backend)
                .await
                .is_err()
            {
                return fault_terminal(session, backend, None, ROBUSTNESS_CAUSE_IO, counts_per_rev);
            }
            if first {
                safety.restart_primitive_clock(backend.elapsed_us());
            }
            if !ramp.is_complete() {
                return match next_ramp_tick_us(&mut next_tick_us, first, backend.elapsed_us()) {
                    Ok(at_us) => {
                        *state = RobustnessReversalState::ZeroRamp {
                            ramp,
                            next_tick_us,
                            retry_available,
                        };
                        StepResult::At(at_us)
                    }
                    Err(_) => fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
                        counts_per_rev,
                    ),
                };
            }
            safety.enter_settle_phase();
            match open_zero_window(session, workspace, backend, counts_per_rev).await {
                Ok((next_sample_us, deadline_us)) => {
                    *state = RobustnessReversalState::ZeroCapture {
                        next_sample_us,
                        deadline_us,
                        retry_available,
                    };
                    StepResult::At(next_sample_us)
                }
                Err(step) => step,
            }
        }
        RobustnessReversalState::ZeroCapture {
            next_sample_us,
            deadline_us,
            retry_available,
        } => {
            if backend.elapsed_us() < next_sample_us {
                return StepResult::At(next_sample_us);
            }
            let read_us = match observe_tick(workspace, safety, backend).await {
                Ok((read_us, _, _)) => read_us,
                Err(fault) => return tick_fault_terminal(session, backend, fault, counts_per_rev),
            };
            if read_us >= deadline_us {
                let Some(analysis_deadline_us) = analysis_deadline(read_us) else {
                    return fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
                        counts_per_rev,
                    );
                };
                *state = RobustnessReversalState::ZeroAnalyze {
                    analysis_deadline_us,
                    retry_available,
                };
                StepResult::ReadyNow
            } else {
                let next = (next_sample_us + CAPTURE_PERIOD_US).min(deadline_us);
                *state = RobustnessReversalState::ZeroCapture {
                    next_sample_us: next,
                    deadline_us,
                    retry_available,
                };
                StepResult::At(next)
            }
        }
        RobustnessReversalState::ZeroAnalyze {
            analysis_deadline_us,
            retry_available,
        } => {
            if backend.elapsed_us() >= analysis_deadline_us {
                return fault_terminal(
                    session,
                    backend,
                    None,
                    ROBUSTNESS_CAUSE_ANALYSIS_TIMEOUT,
                    counts_per_rev,
                );
            }
            match poll_scored_rest_confirmation(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(ScoredRestPoll::Pending | ScoredRestPoll::Diagnostic(_)) => StepResult::ReadyNow,
                Ok(ScoredRestPoll::Complete(confirmation)) => {
                    if confirmation.confirmed_rest {
                        safety.end_primitive();
                        if let Some(pending) = session.pending_cycle.take() {
                            apply_pending_cycle(state, session, pending);
                            StepResult::ReadyNow
                        } else if session.held_route.is_some() {
                            enter_origin_recovery(state, session, safety, backend, counts_per_rev)
                        } else {
                            fault_terminal(
                                session,
                                backend,
                                None,
                                ROBUSTNESS_CAUSE_UNEXPECTED_STATE,
                                counts_per_rev,
                            )
                        }
                    } else if retry_available {
                        match open_zero_window(session, workspace, backend, counts_per_rev).await {
                            Ok((next_sample_us, deadline_us)) => {
                                *state = RobustnessReversalState::ZeroCapture {
                                    next_sample_us,
                                    deadline_us,
                                    retry_available: false,
                                };
                                StepResult::At(next_sample_us)
                            }
                            Err(step) => step,
                        }
                    } else if session.held_route.is_some() {
                        defmt::warn!(
                            "robustness between-cycles rest not confirmed: dir={} disposition={} recent_structural_code={} selected_suffix_len={} conditioned_rest_evidence={} route={} proceeding to origin recovery",
                            session.direction,
                            confirmation.disposition,
                            confirmation.recent_structural_code,
                            confirmation.selected_suffix_len,
                            confirmation.conditioned_rest_evidence,
                            session.held_route
                        );
                        safety.end_primitive();
                        enter_origin_recovery(state, session, safety, backend, counts_per_rev)
                    } else {
                        defmt::warn!(
                            "robustness between-cycles rest not confirmed: dir={} disposition={} recent_structural_code={} selected_suffix_len={} conditioned_rest_evidence={}",
                            session.direction,
                            confirmation.disposition,
                            confirmation.recent_structural_code,
                            confirmation.selected_suffix_len,
                            confirmation.conditioned_rest_evidence
                        );
                        rest_not_confirmed_terminal(session, backend, counts_per_rev)
                    }
                }
                Err(_) => fault_terminal(
                    session,
                    backend,
                    None,
                    ROBUSTNESS_CAUSE_ANALYSIS_POLL_FAILED,
                    counts_per_rev,
                ),
            }
        }
        RobustnessReversalState::RecoverOrigin(runtime) => {
            let mut runtime = runtime;
            let step = {
                let mut resources = RungOriginRecoveryResources {
                    safety: &mut *safety,
                    observations: &mut workspace.observations,
                    blocker: &mut workspace.blocker,
                    analysis: &mut workspace.analysis,
                    settle_displacement_counts: &mut workspace.settle_displacement_counts,
                    settle_min_displacement_counts: &mut workspace.settle_min_displacement_counts,
                    settle_max_displacement_counts: &mut workspace.settle_max_displacement_counts,
                    current_observer: &mut workspace.current_observer,
                };
                step_rung_origin_recovery(&mut runtime, &mut resources, backend).await
            };
            *state = RobustnessReversalState::RecoverOrigin(runtime);
            match step {
                RungOriginRecoveryStep::ReadyNow | RungOriginRecoveryStep::OriginMismatch(_) => {
                    StepResult::ReadyNow
                }
                RungOriginRecoveryStep::At(deadline_us) => StepResult::At(deadline_us),
                RungOriginRecoveryStep::RestSelection { decision, .. } => {
                    defmt::info!(
                        "robustness origin recovery rest selection: dir={} decision={}",
                        session.direction,
                        decision
                    );
                    StepResult::ReadyNow
                }
                RungOriginRecoveryStep::RestAnalysis(analysis) => {
                    defmt::info!(
                        "robustness origin recovery rest analysis: dir={} final_offset={} disposition={}",
                        session.direction,
                        analysis.final_offset_counts,
                        analysis.disposition
                    );
                    StepResult::ReadyNow
                }
                RungOriginRecoveryStep::Contained(report)
                    if report.outcome == RungOriginRecoveryOutcome::RecoveryCurrentHeadroom =>
                {
                    defmt::warn!(
                        "robustness origin recovery done: dir={} outcome={} final_offset={} runtime_us={}",
                        session.direction,
                        report.outcome,
                        report.final_offset_counts,
                        report.total_duration_us
                    );
                    origin_not_recovered_terminal(session, backend, counts_per_rev)
                }
                RungOriginRecoveryStep::Finished(report)
                | RungOriginRecoveryStep::Contained(report) => {
                    defmt::info!(
                        "robustness origin recovery done: dir={} outcome={} final_offset={} runtime_us={}",
                        session.direction,
                        report.outcome,
                        report.final_offset_counts,
                        report.total_duration_us
                    );
                    session.origin_recovered = true;
                    apply_held_route(state, session);
                    StepResult::ReadyNow
                }
                RungOriginRecoveryStep::RestUnconfirmed => {
                    defmt::warn!(
                        "robustness origin recovery rest unconfirmed: dir={} position={}",
                        session.direction,
                        safety.unwrapped_position
                    );
                    origin_not_recovered_terminal(session, backend, counts_per_rev)
                }
                RungOriginRecoveryStep::Fault {
                    safety: Some(report),
                    ..
                } => {
                    defmt::warn!("robustness safety fault (recovery): {}", report);
                    fault_terminal(
                        session,
                        backend,
                        Some(report),
                        ROBUSTNESS_CAUSE_ORIGIN_RECOVERY_FAULT,
                        counts_per_rev,
                    )
                }
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => {
                    defmt::warn!("robustness origin recovery fault: {}", error);
                    fault_terminal(
                        session,
                        backend,
                        None,
                        ROBUSTNESS_CAUSE_ORIGIN_RECOVERY_FAULT,
                        counts_per_rev,
                    )
                }
            }
        }
        RobustnessReversalState::Assemble => {
            *state = RobustnessReversalState::Done;
            complete_terminal(session, backend, counts_per_rev)
        }
        RobustnessReversalState::Done => StepResult::ReadyNow,
    }
}

fn accepted_gains(point: &AcceptancePoint) -> Result<(PidPGain, PidIGain), CommissionError> {
    let p = PidPGain::new(point.selected_p).map_err(|_| CommissionError::PidGainInvalid)?;
    let i = PidIGain::new(point.selected_i).map_err(|_| CommissionError::PidGainInvalid)?;
    Ok((p, i))
}

fn planned_velocity_mrev_s(point: &AcceptancePoint) -> Result<u32, CommissionError> {
    let target =
        u32::try_from(amplitude_rpm(point)).map_err(|_| CommissionError::InvalidSchedule)?;
    let velocity = target
        .checked_mul(1_000)
        .map(|value| value / 60)
        .ok_or(CommissionError::TargetRateInvalid)?;
    if velocity == 0 {
        return Err(CommissionError::InvalidSchedule);
    }
    Ok(velocity)
}

fn return_lower_rate_q(point: &AcceptancePoint, direction: VelocityDirection) -> Option<u64> {
    let [low_q, high_q] = match direction {
        VelocityDirection::Forward => point.selected_response_intervals_q[0],
        VelocityDirection::Reverse => point.selected_response_intervals_q[1],
    };
    commanded_side_lower_rate_q(direction, i64::from(low_q), i64::from(high_q))
}

fn robustness_recovery_plan(
    point: &AcceptancePoint,
    safety: &OuterSafetyLimits,
) -> Result<RungOriginRecoveryPlan, CommissionError> {
    if return_lower_rate_q(point, VelocityDirection::Forward).is_none()
        || return_lower_rate_q(point, VelocityDirection::Reverse).is_none()
    {
        return Err(CommissionError::MissingRateTableEntry);
    }
    plan_rung_origin_recovery(
        safety.budget,
        safety.counts_per_rev(),
        planned_velocity_mrev_s(point)?,
    )
    .map_err(|_| CommissionError::RecoveryPlanFailed)
}

fn return_direction_from_offset(start_offset_counts: i32) -> VelocityDirection {
    if start_offset_counts > 0 {
        VelocityDirection::Reverse
    } else {
        VelocityDirection::Forward
    }
}

fn build_origin_recovery(
    session: &RobustnessReversalSession,
    safety: &OuterSafetyLimits,
) -> Result<RungOriginRecoveryRuntime, CommissionError> {
    let plan = session
        .recovery_plan
        .ok_or(CommissionError::MissingCaptureState)?;
    let start_offset_counts = safety.unwrapped_position;
    let return_direction = return_direction_from_offset(start_offset_counts);
    let lower_rate_q = return_lower_rate_q(&session.point, return_direction)
        .ok_or(CommissionError::MissingRateTableEntry)?;
    let p = PidPGain::new(session.point.selected_p).map_err(|_| CommissionError::PidGainInvalid)?;
    RungOriginRecoveryRuntime::new(
        RungOriginRecoveryConfig {
            stage: RecoveryStage::Robustness,
            rung_index: u8::try_from(session.cycle_cursor[session.direction]).unwrap_or(u8::MAX),
            p,
            target_velocity_rpm: TmcMechanicalRpm::from_register(amplitude_rpm(&session.point)),
            planned_velocity_mrev_s: planned_velocity_mrev_s(&session.point)?,
            start_offset_counts,
            lower_rate_q: Some(lower_rate_q),
            authority_source: RecoveryBindingSource::SettledRate,
            current_contact_policy: RecoveryCurrentContactPolicy::StopImmediately,
        },
        plan,
    )
}

fn enter_origin_recovery<B: CommissioningBackend>(
    state: &mut RobustnessReversalState,
    session: &RobustnessReversalSession,
    safety: &OuterSafetyLimits,
    backend: &B,
    counts_per_rev: u32,
) -> StepResult {
    match build_origin_recovery(session, safety) {
        Ok(runtime) => {
            defmt::info!(
                "robustness origin recovery: dir={} slot={} route={} start_offset={}",
                session.direction,
                session.cycle_cursor[session.direction],
                session.held_route,
                safety.unwrapped_position
            );
            *state = RobustnessReversalState::RecoverOrigin(runtime);
            StepResult::ReadyNow
        }
        Err(error) => {
            defmt::warn!("robustness origin recovery cannot be built: {}", error);
            fault_terminal(
                session,
                backend,
                None,
                ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID,
                counts_per_rev,
            )
        }
    }
}

/// One up or reversal ramp in flight, with whether its first write opens the velocity-step
/// primitive.
struct LegRamp<'a> {
    ramp: &'a mut TargetRamp,
    next_tick_us: &'a mut u64,
    opens_primitive: bool,
}

/// Progress of a leg ramp after one tick.
enum LegRampTick {
    /// More ticks follow; the next one is due at this time.
    Pending(u64),
    /// The ramp reached its target; the time of the encoder baseline read after the ramp-end
    /// write.
    Ended(u64),
}

/// Reads a fresh encoder baseline for the next observed delta and returns its read time.
#[allow(clippy::result_large_err)]
async fn read_encoder_baseline<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    workspace: &mut RobustnessReversalWorkspace,
    backend: &mut B,
    counts_per_rev: u32,
) -> Result<u64, StepResult> {
    let Ok(baseline) = backend.read_capture_register(ABN_DECODER_COUNT).await else {
        return Err(fault_terminal(
            session,
            backend,
            None,
            ROBUSTNESS_CAUSE_IO,
            counts_per_rev,
        ));
    };
    workspace.previous_encoder_raw = baseline;
    workspace.previous_read_us = backend.elapsed_us();
    Ok(workspace.previous_read_us)
}

/// Runs one ramp tick: the first tick reads the encoder baseline, later ticks run the per-tick
/// safety checks, and every tick writes the next target.
#[allow(clippy::result_large_err)]
async fn advance_leg_ramp<B: CommissioningBackend>(
    leg: LegRamp<'_>,
    session: &RobustnessReversalSession,
    workspace: &mut RobustnessReversalWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<LegRampTick, StepResult> {
    let LegRamp {
        ramp,
        next_tick_us,
        opens_primitive,
    } = leg;
    let counts_per_rev = safety.counts_per_rev();
    let first = !ramp.is_started();
    if first {
        read_encoder_baseline(session, workspace, backend, counts_per_rev).await?;
    } else {
        if backend.elapsed_us() < *next_tick_us {
            return Ok(LegRampTick::Pending(*next_tick_us));
        }
        if let Err(fault) = sample_tick(workspace, safety, backend).await {
            return Err(tick_fault_terminal(session, backend, fault, counts_per_rev));
        }
    }
    if write_next_ramp_target(ramp, workspace, backend)
        .await
        .is_err()
    {
        return Err(fault_terminal(
            session,
            backend,
            None,
            ROBUSTNESS_CAUSE_IO,
            counts_per_rev,
        ));
    }
    let written_us = backend.elapsed_us();
    if first && !opens_primitive {
        safety.restart_primitive_clock(written_us);
    }
    if first && opens_primitive {
        safety.end_primitive();
        if safety.begin_velocity_step_primitive(written_us).is_err() {
            return Err(fault_terminal(
                session,
                backend,
                None,
                ROBUSTNESS_CAUSE_SAFETY_ARM_FAILED,
                counts_per_rev,
            ));
        }
    }
    if ramp.is_complete() {
        let baseline_us =
            read_encoder_baseline(session, workspace, backend, counts_per_rev).await?;
        return Ok(LegRampTick::Ended(baseline_us));
    }
    match next_ramp_tick_us(next_tick_us, first, written_us) {
        Ok(at_us) => Ok(LegRampTick::Pending(at_us)),
        Err(_) => Err(fault_terminal(
            session,
            backend,
            None,
            ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
            counts_per_rev,
        )),
    }
}

async fn write_next_ramp_target<B: CommissioningBackend>(
    ramp: &mut TargetRamp,
    workspace: &mut RobustnessReversalWorkspace,
    backend: &mut B,
) -> Result<(), CommissionError> {
    let Some(target) = ramp.next() else {
        return Ok(());
    };
    if target == workspace.last_target_rpm {
        return Ok(());
    }
    backend
        .write_register(PID_VELOCITY_TARGET, target as u32)
        .await
        .map_err(|_| CommissionError::Spi)?;
    workspace.last_target_rpm = target;
    Ok(())
}

/// A leg's final target and the time of the encoder baseline read after its ramp-end write.
struct LegCapture {
    signed_target_rpm: i32,
    started_us: u64,
}

/// Opens a leg's capture at the post-ramp encoder baseline read taken after the ramp-end write:
/// the leg peak, the band window, the sample schedule and the measurement budget all start from
/// that read at `leg.started_us`.
#[allow(clippy::result_large_err)]
fn open_leg_capture<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    workspace: &mut RobustnessReversalWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &B,
    leg: LegCapture,
) -> Result<(u64, u64), StepResult> {
    let LegCapture {
        signed_target_rpm,
        started_us,
    } = leg;
    let counts_per_rev = safety.counts_per_rev();
    safety.reset_primitive_peak();
    workspace.observations.clear();
    workspace.analysis.reset();
    let target_rate = target_rate_q(signed_target_rpm, counts_per_rev);
    let band = band_q_for(target_rate);
    workspace.capture.reset_with_band_window(
        target_rate,
        band,
        ROBUSTNESS_BAND_DWELL_US,
        PROBE_BAND_WINDOW_SAMPLES,
    );
    workspace.late_tracking = LateTrackingAccumulator::new(target_rate);
    let deadlines = started_us
        .checked_add(CAPTURE_PERIOD_US)
        .zip(started_us.checked_add(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US)));
    deadlines.ok_or_else(|| {
        fault_terminal(
            session,
            backend,
            None,
            ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
            counts_per_rev,
        )
    })
}

enum CaptureAdvance {
    /// Keep sampling; carry the (possibly newly set) settle-tail deadline.
    Continue {
        next_sample_us: u64,
        settle_deadline_us: Option<u64>,
    },
    /// Band entry reached and the settled tail is sampled: analyze it.
    Converged,
    /// No band entry within the full measurement budget: inconclusive.
    Exhausted,
}

fn advance_capture(
    accumulator: &mut ReversalOnlineAccumulator,
    read_us: u64,
    elapsed_us: u32,
    delta: i32,
    next_sample_us: u64,
    budget_deadline_us: u64,
    settle_deadline_us: Option<u64>,
) -> CaptureAdvance {
    accumulator.observe(delta, elapsed_us);
    let settle_deadline_us = settle_deadline_us.or_else(|| {
        accumulator
            .band_entry_us()
            .map(|_| read_us.saturating_add(u64::from(ROBUSTNESS_SETTLE_TAIL_US)))
    });
    match settle_deadline_us {
        Some(settle_deadline) if read_us >= settle_deadline => CaptureAdvance::Converged,
        None if read_us >= budget_deadline_us => CaptureAdvance::Exhausted,
        _ => {
            let active_deadline = settle_deadline_us.unwrap_or(budget_deadline_us);
            let next = (next_sample_us + CAPTURE_PERIOD_US).min(active_deadline);
            CaptureAdvance::Continue {
                next_sample_us: next,
                settle_deadline_us,
            }
        }
    }
}

/// Feeds one reverse-leg observation `(read_us, elapsed_us, delta)`. The leg is observed until
/// its budget deadline regardless of when it enters the band; it then converges if it entered the
/// band and is exhausted otherwise.
fn advance_reverse_capture(
    workspace: &mut RobustnessReversalWorkspace,
    (read_us, elapsed_us, delta): (u64, u32, i32),
    next_sample_us: u64,
    budget_deadline_us: u64,
) -> CaptureAdvance {
    workspace.capture.observe(delta, elapsed_us);
    if let Some(late_tracking) = workspace.late_tracking.as_mut() {
        let started_us =
            budget_deadline_us.saturating_sub(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US));
        let since_start_us = u32::try_from(read_us.saturating_sub(started_us)).unwrap_or(u32::MAX);
        late_tracking.observe(since_start_us, delta, elapsed_us);
    }
    if read_us < budget_deadline_us {
        return CaptureAdvance::Continue {
            next_sample_us: (next_sample_us + CAPTURE_PERIOD_US).min(budget_deadline_us),
            settle_deadline_us: None,
        };
    }
    if workspace.capture.band_entry_us().is_some() {
        CaptureAdvance::Converged
    } else {
        CaptureAdvance::Exhausted
    }
}

fn retry_or_finalize_inconclusive(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
    last_target_rpm: i32,
) {
    let action = record_cycle(session, CycleRecord::Inconclusive, None);
    session.held_route = Some(HeldRoute::Ledger(action));
    *state = zero_ramp_state(session, last_target_rpm);
}

fn zero_ramp_state(
    session: &RobustnessReversalSession,
    last_target_rpm: i32,
) -> RobustnessReversalState {
    RobustnessReversalState::ZeroRamp {
        ramp: TargetRamp::new(last_target_rpm, 0, session.ramp_rate),
        next_tick_us: 0,
        retry_available: true,
    }
}

fn record_cycle(
    session: &mut RobustnessReversalSession,
    record: CycleRecord,
    wire: Option<CycleWire>,
) -> LedgerAction {
    let direction = session.direction;
    let cursor = session.cycle_cursor[direction];
    let action = session.ledgers[direction].record(record);
    defmt::info!(
        "robustness reversal cycle: dir={} cursor={} record={} action={}",
        direction,
        cursor,
        record,
        action,
    );
    let filled =
        matches!(record, CycleRecord::Conclusive(_)) && !matches!(action, LedgerAction::RetryTail);
    if filled {
        session.cycle_wires[direction][cursor] = wire;
        debug_assert!(session.ledgers[direction].cycles()[cursor].is_some());
    } else if matches!(action, LedgerAction::RetryTail) {
        session.displaced_tail_wires[direction] = wire;
    }
    if !matches!(
        action,
        LedgerAction::RetryInconclusive | LedgerAction::RetryTail
    ) {
        session.cycle_cursor[direction] += 1;
    }
    action
}

fn route_action(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
    action: LedgerAction,
) {
    match action {
        LedgerAction::DirectionDone => {
            if session.direction + 1 < DIRECTION_COUNT {
                session.direction += 1;
                *state = RobustnessReversalState::EstablishWriteGain;
            } else {
                *state = RobustnessReversalState::Assemble;
            }
        }
        LedgerAction::NextCycle
        | LedgerAction::RetryInconclusive
        | LedgerAction::RetryTail
        | LedgerAction::RejectTail => {
            *state = RobustnessReversalState::EstablishWriteGain;
        }
    }
}

fn apply_pending_cycle(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
    pending: PendingCycle,
) {
    let direction = session.direction;
    let cursor = session.cycle_cursor[direction];
    let action = record_cycle(
        session,
        CycleRecord::Conclusive(pending.metrics),
        Some(pending.wire),
    );
    if action != LedgerAction::RetryTail {
        session.suffix_excluded[direction][cursor] = pending.suffix_excluded;
    }
    if pending.peak_permille != 0 && action != LedgerAction::RetryTail {
        session.peak_rate_permille[direction] =
            session.peak_rate_permille[direction].max(pending.peak_permille);
        session.peak_scored_cycles[direction] =
            session.peak_scored_cycles[direction].saturating_add(1);
    }
    route_action(state, session, action);
}

fn apply_held_route(state: &mut RobustnessReversalState, session: &mut RobustnessReversalSession) {
    match session.held_route.take() {
        Some(HeldRoute::Ledger(action)) => route_action(state, session, action),
        Some(HeldRoute::ReenterEstablish) | None => {
            *state = RobustnessReversalState::EstablishWriteGain;
        }
    }
}

#[allow(clippy::result_large_err)]
async fn open_zero_window<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    workspace: &mut RobustnessReversalWorkspace,
    backend: &mut B,
    counts_per_rev: u32,
) -> Result<(u64, u64), StepResult> {
    let baseline = match backend.read_capture_register(ABN_DECODER_COUNT).await {
        Ok(value) => value,
        Err(_) => {
            return Err(fault_terminal(
                session,
                backend,
                None,
                ROBUSTNESS_CAUSE_IO,
                counts_per_rev,
            ));
        }
    };
    let baseline_us = backend.elapsed_us();
    workspace.observations.clear();
    workspace.analysis.reset_for_scored_rest(false);
    workspace.previous_encoder_raw = baseline;
    workspace.previous_read_us = baseline_us;
    let Some(next_sample_us) = baseline_us.checked_add(CAPTURE_PERIOD_US) else {
        return Err(fault_terminal(
            session,
            backend,
            None,
            ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
            counts_per_rev,
        ));
    };
    let Some(deadline_us) = baseline_us.checked_add(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US))
    else {
        return Err(fault_terminal(
            session,
            backend,
            None,
            ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
            counts_per_rev,
        ));
    };
    Ok((next_sample_us, deadline_us))
}

const CAPTURE_PERIOD_US: u64 = VELOCITY_SWEEP_SAMPLE_PERIOD_US as u64;

fn analysis_deadline(read_us: u64) -> Option<u64> {
    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
}

const MICROS_PER_MILLI: u32 = 1_000;

fn to_ms_saturating(value_us: u32) -> u16 {
    let ms = value_us.div_ceil(MICROS_PER_MILLI);
    u16::try_from(ms).unwrap_or(u16::MAX)
}

fn constructed_thresholds(
    point: &AcceptancePoint,
    counts_per_rev: u32,
) -> (i64, RobustnessThresholds) {
    let plant_rate_q = reduce_plant_rate_q(&point.selected_response_intervals_q);
    let target = target_rate_q(amplitude_rpm(point), counts_per_rev);
    (
        plant_rate_q,
        robustness_thresholds_for(plant_rate_q, target),
    )
}

fn median_tripped(ledger: &CycleLedger, point: &AcceptancePoint, counts_per_rev: u32) -> u8 {
    let median_metrics = ReversalMetrics {
        reconvergence_time_us: u32::from(ledger.median_reconvergence_ms()).saturating_mul(1_000),
        settled_residual_q: i64::from(ledger.median_settled_residual_q()),
        recovery_iae_q: i64::from(ledger.median_recovery_iae_qs()).saturating_mul(1_000_000),
        dac_rms_q: 0,
        forward_settle_time_us: u32::from(ledger.median_forward_settle_ms()).saturating_mul(1_000),
        reconvergence_ratio_ppm: 0,
        late_tracking: LateTracking {
            ppm: ledger.median_late_tracking_ppm(),
            window_delta_counts: 0,
            window_elapsed_us: 0,
            window_start_us: 0,
        },
    };
    let (_, thresholds) = constructed_thresholds(point, counts_per_rev);
    tripped_mask(&median_metrics, &thresholds)
}

fn peak_tripped(peak_rate_permille: u16, peak_scored_cycles: u8) -> u8 {
    if peak_scored_cycles > 0 && peak_rate_permille >= REVERSAL_PEAK_REJECT_PERMILLE {
        TRIP_PEAK
    } else {
        0
    }
}

fn direction_tripped(
    session: &RobustnessReversalSession,
    direction: usize,
    counts_per_rev: u32,
) -> u8 {
    median_tripped(&session.ledgers[direction], &session.point, counts_per_rev)
        | peak_tripped(
            session.peak_rate_permille[direction],
            session.peak_scored_cycles[direction],
        )
}

fn direction_summary(
    session: &RobustnessReversalSession,
    direction: usize,
    counts_per_rev: u32,
) -> RobustnessDirectionSummary {
    let ledger = &session.ledgers[direction];
    let tripped = direction_tripped(session, direction, counts_per_rev);
    RobustnessDirectionSummary {
        median_reconvergence_ms: ledger.median_reconvergence_ms(),
        max_reconvergence_ms: ledger.max_reconvergence_ms(),
        median_forward_settle_ms: ledger.median_forward_settle_ms(),
        peak_rate_permille: session.peak_rate_permille[direction],
        valid_cycles: ledger.valid_cycles(),
        trip_count: u8::from(tripped != 0 || ledger.tail_rejected()),
        retry_count: ledger.inconclusive_retries_used(),
        inconclusive: !ledger.meets_floor(),
        peak_unscored: session.peak_scored_cycles[direction] == 0,
    }
}

fn assembled_evidence(
    session: &RobustnessReversalSession,
    counts_per_rev: u32,
) -> RobustnessEvidence {
    RobustnessEvidence {
        directions: [
            direction_summary(session, 0, counts_per_rev),
            direction_summary(session, 1, counts_per_rev),
        ],
        residual_medians_q: [
            session.ledgers[0].median_settled_residual_q(),
            session.ledgers[1].median_settled_residual_q(),
        ],
        iae_medians_qs: [
            session.ledgers[0].median_recovery_iae_qs(),
            session.ledgers[1].median_recovery_iae_qs(),
        ],
        dac_rms_medians_q: [
            session.ledgers[0].median_dac_rms_q(),
            session.ledgers[1].median_dac_rms_q(),
        ],
        cycles: session.cycle_wires,
        tail_counts: [
            session.ledgers[0].tail_count(),
            session.ledgers[1].tail_count(),
        ],
        displaced_tails: session.displaced_tail_wires,
        late_tracking_medians_ppm: [
            session.ledgers[0].median_late_tracking_ppm(),
            session.ledgers[1].median_late_tracking_ppm(),
        ],
        late_target_rate_q: [
            reverse_target_rate_q(&session.point, 0, counts_per_rev),
            reverse_target_rate_q(&session.point, 1, counts_per_rev),
        ],
        late_cycles: [
            session.ledgers[0].late_cycles(),
            session.ledgers[1].late_cycles(),
        ],
        late_suffix_excluded: session.suffix_excluded,
    }
}

fn complete_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    counts_per_rev: u32,
) -> StepResult {
    let evidence = assembled_evidence(session, counts_per_rev);
    let tripped_a = direction_tripped(session, 0, counts_per_rev);
    let tripped_b = direction_tripped(session, 1, counts_per_rev);
    let (plant_rate_q, thresholds) = constructed_thresholds(&session.point, counts_per_rev);
    let inconclusive_a = !session.ledgers[0].meets_floor();
    let inconclusive_b = !session.ledgers[1].meets_floor();
    debug_assert_eq!(
        session.ledgers[0].tail_rejected(),
        session.ledgers[0].tail_count() >= 2
    );
    debug_assert_eq!(
        session.ledgers[1].tail_rejected(),
        session.ledgers[1].tail_count() >= 2
    );
    let tail_rejected = session.ledgers[0].tail_rejected() || session.ledgers[1].tail_rejected();
    let any_trip = tripped_a != 0 || tripped_b != 0;
    let cause = if tail_rejected {
        ROBUSTNESS_CAUSE_TAIL_REPEATED
    } else {
        summary_cause(tripped_a, tripped_b, inconclusive_a, inconclusive_b)
    };
    let outcome = if tail_rejected || any_trip {
        RobustnessOutcome::Rejected
    } else if inconclusive_a || inconclusive_b {
        RobustnessOutcome::Inconclusive
    } else {
        RobustnessOutcome::Complete
    };
    StepResult::RobustnessReversalTerminal(RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence: session.run_sequence,
        outcome,
        cause,
        selected_p: session.point.selected_p,
        selected_i: session.point.selected_i,
        target_velocity_rpm: session.point.target_velocity_rpm,
        plant_rate_q,
        iae_max_q: thresholds.iae_max_q,
        evidence,
        run_started_us: session.run_started_us,
        run_completed_us: backend.elapsed_us(),
        safety_fault: None,
    })
}

fn failed_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    cause: u8,
    safety_fault: Option<OuterSafetyFault>,
    counts_per_rev: u32,
) -> StepResult {
    let (plant_rate_q, thresholds) = constructed_thresholds(&session.point, counts_per_rev);
    StepResult::RobustnessReversalTerminal(RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence: session.run_sequence,
        outcome: RobustnessOutcome::Failed,
        cause,
        selected_p: session.point.selected_p,
        selected_i: session.point.selected_i,
        target_velocity_rpm: session.point.target_velocity_rpm,
        plant_rate_q,
        iae_max_q: thresholds.iae_max_q,
        evidence: assembled_evidence(session, counts_per_rev),
        run_started_us: session.run_started_us,
        run_completed_us: backend.elapsed_us(),
        safety_fault,
    })
}

fn fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    safety_fault: Option<OuterSafetyFault>,
    internal_cause: u8,
    counts_per_rev: u32,
) -> StepResult {
    let cause = if safety_fault.is_some() {
        ROBUSTNESS_CAUSE_SAFETY_FAULT
    } else {
        internal_cause
    };
    failed_terminal(session, backend, cause, safety_fault, counts_per_rev)
}

fn rest_not_confirmed_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    counts_per_rev: u32,
) -> StepResult {
    failed_terminal(
        session,
        backend,
        ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED,
        None,
        counts_per_rev,
    )
}

fn origin_not_recovered_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    counts_per_rev: u32,
) -> StepResult {
    failed_terminal(
        session,
        backend,
        ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED,
        None,
        counts_per_rev,
    )
}

fn tick_fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    fault: TickFault,
    counts_per_rev: u32,
) -> StepResult {
    match fault {
        TickFault::Io => {
            fault_terminal(session, backend, None, ROBUSTNESS_CAUSE_IO, counts_per_rev)
        }
        TickFault::Safety(report) => {
            defmt::warn!("robustness safety fault (tick): {}", report);
            fault_terminal(
                session,
                backend,
                Some(report),
                ROBUSTNESS_CAUSE_IO,
                counts_per_rev,
            )
        }
    }
}

/// Build a `Failed` robustness terminal for a dispatch-level outer-safety abort.
///
/// Reconstructs the terminal from the live session without a backend, so outer dispatch can
/// report a pre-tick envelope trip as a robustness terminal.
pub(crate) fn robustness_reversal_fault_terminal(
    phase: &crate::commissioning::outer::OuterPhase,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    let crate::commissioning::outer::OuterPhase::RobustnessReversal {
        session, safety, ..
    } = phase
    else {
        return StepResult::OuterFailed(error);
    };
    let _ = error;
    if let Some(report) = &safety_fault {
        defmt::warn!("robustness safety fault (dispatch): {}", report);
    }
    let cause = if safety_fault.is_some() {
        ROBUSTNESS_CAUSE_SAFETY_FAULT
    } else {
        ROBUSTNESS_CAUSE_DISPATCH_ABORT
    };
    let (plant_rate_q, thresholds) =
        constructed_thresholds(&session.point, safety.counts_per_rev());
    StepResult::RobustnessReversalTerminal(RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence: session.run_sequence,
        outcome: RobustnessOutcome::Failed,
        cause,
        selected_p: session.point.selected_p,
        selected_i: session.point.selected_i,
        target_velocity_rpm: session.point.target_velocity_rpm,
        plant_rate_q,
        iae_max_q: thresholds.iae_max_q,
        evidence: assembled_evidence(session, safety.counts_per_rev()),
        run_started_us: session.run_started_us,
        run_completed_us: session.run_started_us,
        safety_fault,
    })
}

/// Assemble the deployable outer tune for a passed reversal-robustness gate.
///
/// Joins the terminal robustness-selected velocity gains with the entry snapshot position
/// gains, velocity limit, and biquad cutoffs. Mechanical-identification fields stay zero.
/// A passed gate is always reported as `Accepted`.
pub(crate) fn robustness_pass_outer_result(
    terminal: &RobustnessReversalTerminal,
    entry: &OuterGainRollbackSnapshot,
    point: &AcceptancePoint,
) -> OuterResult {
    let status = CommissionStatus::Accepted;
    let provenance = TuneProvenance {
        probed_velocity_mrev_s: terminal.target_velocity_rpm.unsigned_abs() * 1_000 / 60,
        d_eq_q: i32::try_from(terminal.plant_rate_q).unwrap_or(i32::MAX),
        confidence_q: point.confidence_q,
        band_lower_percent: CONFIRMATION_BAND_LOWER_PERCENT,
        band_upper_percent: CONFIRMATION_BAND_UPPER_PERCENT,
        band_position_q: point.band_position_q,
    };
    OuterResult {
        status,
        velocity_p: terminal.selected_p,
        velocity_i: terminal.selected_i,
        velocity_limit: entry.velocity_limit,
        position_p: entry.position_p.raw(),
        position_i: entry.position_i.raw(),
        velocity_filter_hz: entry.velocity_filter_hz,
        position_filter_hz: entry.position_filter_hz,
        flux_filter_hz: entry.flux_filter_hz,
        torque_filter_hz: entry.torque_filter_hz,
        warning_code: 0,
        provenance,
        position_tune: crate::commissioning::types::PositionTuneProvenance::default(),
        position_tune_outcome_code: 0,
        motor_disarmed: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::robustness::{
        ROBUSTNESS_CAUSE_IAE_EXCEEDED, ROBUSTNESS_CAUSE_LATE_TRACKING_EXCEEDED,
        ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED, ROBUSTNESS_CAUSE_PASS,
        ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED,
        ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED,
        ROBUSTNESS_CAUSE_REVERSAL_PEAK_EXCEEDED, ROBUSTNESS_CAUSE_TAIL_REPEATED,
    };
    use crate::commission_dispatch::test_support::{
        MockBackend, SpiOp, current_limit, sample_entry_snapshot,
    };
    use crate::commissioning::outer::OuterMotionBudget;
    use crate::commissioning::outer::check_outer_safety;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        IntegralPlanIdentity, IntegralRetentionSlot,
    };
    use crate::commissioning::outer::velocity::robustness_metrics::{
        ROBUSTNESS_MAX_RETRIES, ROBUSTNESS_THRESHOLDS, late_tracking_at,
    };
    use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;
    use crate::commissioning::types::OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY;
    use foci_tmc4671::registers::PID_ERROR_PID_VELOCITY_ERROR_SUM;

    const STEADY_DELTA: i32 = 10;
    const STEP_BUDGET: usize = 200_000;

    fn test_budget() -> OuterMotionBudget {
        OuterMotionBudget {
            requested_velocity_mrev_s: 6_000,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 2_000,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: 5_000,
            positive_position_headroom_mrev: 5_000,
            max_duration_ms: 60_000,
            homing_speed_mrev_s: 4_375,
            max_accel_mrev_s2: 200_000,
        }
    }

    fn test_safety() -> OuterSafetyLimits {
        let mut safety = OuterSafetyLimits::new(current_limit(2_448).raw(), test_budget());
        safety
            .configure_robustness_encoder_resolution(4_000)
            .expect("valid encoder resolution");
        safety.set_encoder_baseline(0).expect("baseline configured");
        safety
            .arm_safety_observation(0)
            .expect("safety observation armed");
        safety.torque_cap = current_limit(2_448).raw();
        safety
    }

    fn test_backend() -> MockBackend {
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);
        backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
        backend
    }

    const COUNTS_PER_REV: u32 = 4_000;

    fn test_point() -> AcceptancePoint {
        AcceptancePoint {
            identity: IntegralPlanIdentity {
                channel: 0,
                build_features: 0x1234,
                schema_revision: 1,
                plan_digest: 0xfeed_beef,
            },
            selected_p: 1_024,
            selected_i: 512,
            target_velocity_rpm: 176,
            selected_response_intervals_q: [[10, 20], [-20, -10]],
            confidence_q: 0,
            band_position_q: 0,
            qualifier_bits: 0,
            token: None,
        }
    }

    fn test_rate() -> RampRate {
        RampRate::from_accel(333_334).expect("the fixture acceleration gives a nonzero rate")
    }

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum CapturePhase {
        EstablishRamp,
        Establish,
        ReverseRamp,
        Reverse,
        ZeroRamp,
        Zero,
        Recover,
    }

    fn capture_phase(state: &RobustnessReversalState) -> Option<CapturePhase> {
        match state {
            RobustnessReversalState::EstablishRamp { .. } => Some(CapturePhase::EstablishRamp),
            RobustnessReversalState::ReverseRamp { .. } => Some(CapturePhase::ReverseRamp),
            RobustnessReversalState::ZeroRamp { .. } => Some(CapturePhase::ZeroRamp),
            RobustnessReversalState::EstablishCapture { .. } => Some(CapturePhase::Establish),
            RobustnessReversalState::ReverseCapture { .. } => Some(CapturePhase::Reverse),
            RobustnessReversalState::ZeroCapture { .. } => Some(CapturePhase::Zero),
            RobustnessReversalState::RecoverOrigin(_) => Some(CapturePhase::Recover),
            _ => None,
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Envelope {
        Untracked,
        Tracked,
    }

    struct RunParts {
        state: RobustnessReversalState,
        workspace: RobustnessReversalWorkspace,
        session: RobustnessReversalSession,
        safety: OuterSafetyLimits,
        backend: MockBackend,
        targets: heapless::Vec<i32, 512>,
        log_cursor: usize,
        recoveries: u32,
    }

    impl RunParts {
        fn new(point: AcceptancePoint) -> Self {
            Self {
                state: RobustnessReversalState::Setup,
                workspace: RobustnessReversalWorkspace::new(),
                session: RobustnessReversalSession::new(point, 7, COUNTS_PER_REV, test_rate()),
                safety: test_safety(),
                backend: test_backend(),
                targets: heapless::Vec::new(),
                log_cursor: 0,
                recoveries: 0,
            }
        }

        fn step(&mut self) -> StepResult {
            pollster::block_on(step_robustness_reversal(
                &mut self.state,
                &mut self.workspace,
                &mut self.session,
                &mut self.safety,
                &mut self.backend,
            ))
        }

        fn harvest_targets(&mut self) {
            let target_addr = PID_VELOCITY_TARGET.addr();
            for op in &self.backend.spi_log[self.log_cursor..] {
                if let SpiOp::Write(addr, value) = op
                    && *addr == target_addr
                {
                    self.targets
                        .push(*value as i32)
                        .expect("target log holds every write of one run");
                }
            }
            self.log_cursor = self.backend.spi_log.len();
        }
    }

    fn drive_parts(
        mut parts: RunParts,
        envelope: Envelope,
        mut motion: impl FnMut(CapturePhase, u32, i32) -> i32,
    ) -> (RobustnessReversalTerminal, RunParts) {
        let mut encoder_count: i32 = 0;
        let mut previous_phase = None;
        let mut was_recovering = false;
        let mut leg_tick: u32 = 0;

        for _ in 0..STEP_BUDGET {
            let result = parts.step();
            parts.harvest_targets();
            if envelope == Envelope::Tracked {
                parts.backend.spi_log.clear();
                parts.log_cursor = 0;
            }
            let recovering = matches!(parts.state, RobustnessReversalState::RecoverOrigin(_));
            if recovering && !was_recovering {
                parts.recoveries += 1;
            }
            was_recovering = recovering;
            let phase = capture_phase(&parts.state);
            if phase.is_some() && phase != previous_phase {
                leg_tick = 0;
            }
            previous_phase = phase;
            match result {
                StepResult::ReadyNow => {}
                StepResult::At(deadline) => {
                    parts.backend.elapsed_us_value = deadline;
                    let target = *parts
                        .backend
                        .registers
                        .get(&PID_VELOCITY_TARGET.addr())
                        .unwrap_or(&0) as i32;
                    let phase = phase.expect("At is scheduled only from a capture state");
                    encoder_count += motion(phase, leg_tick, target);
                    let raw = encoder_count.rem_euclid(4_000);
                    parts.backend.set_abn_decoder_count(raw);
                    leg_tick += 1;
                    if envelope == Envelope::Tracked {
                        parts.backend.mechanical_encoder_position_counts_value = raw as u32;
                        match pollster::block_on(check_outer_safety(
                            &mut parts.safety,
                            &mut parts.backend,
                        )) {
                            Ok(None) => {}
                            Ok(Some(fault)) => {
                                let StepResult::RobustnessReversalTerminal(terminal) =
                                    fault_terminal(
                                        &parts.session,
                                        &parts.backend,
                                        Some(fault),
                                        ROBUSTNESS_CAUSE_SAFETY_FAULT,
                                        COUNTS_PER_REV,
                                    )
                                else {
                                    unreachable!("fault_terminal builds a robustness terminal")
                                };
                                return (terminal, parts);
                            }
                            Err(error) => panic!("outer safety sample failed: {error:?}"),
                        }
                    }
                }
                StepResult::RobustnessReversalTerminal(terminal) => {
                    return (terminal, parts);
                }
                other => panic!("unexpected step result: {other:?}"),
            }
        }
        panic!("robustness reversal run exceeded its bounded step budget");
    }

    fn drive_scripted(
        point: AcceptancePoint,
        motion: impl FnMut(CapturePhase, u32, i32) -> i32,
    ) -> (
        RobustnessReversalTerminal,
        MockBackend,
        RobustnessReversalSession,
    ) {
        let (terminal, parts) = drive_parts(RunParts::new(point), Envelope::Untracked, motion);
        (terminal, parts.backend, parts.session)
    }

    fn recovery_point() -> AcceptancePoint {
        AcceptancePoint {
            selected_response_intervals_q: [[12_000_000, 13_000_000], [-13_000_000, -12_000_000]],
            ..test_point()
        }
    }

    /// Absolute windows wide enough that reverse-side drift never calls for an origin recovery,
    /// so a run sees only the recoveries its own script causes.
    fn drift_free_parts(point: AcceptancePoint) -> RunParts {
        let mut parts = RunParts::new(point);
        parts.safety.negative_position_window *= 4;
        parts.safety.positive_position_window *= 4;
        parts
    }

    #[test]
    fn robustness_recovery_plan_rejects_missing_return_lower_rate() {
        let mut point = test_point();
        point.selected_response_intervals_q = [[0, 0], [0, 0]];
        let safety = test_safety();

        let error = robustness_recovery_plan(&point, &safety).unwrap_err();

        assert_eq!(error, CommissionError::MissingRateTableEntry);
    }

    #[test]
    fn robustness_recovery_plan_reports_recovery_plan_failed_on_planner_rejection() {
        let mut safety = test_safety();
        safety.budget.settle_travel_reserve_mrev = safety.budget.max_stroke_travel_mrev;

        let result = robustness_recovery_plan(&test_point(), &safety);

        assert_eq!(result, Err(CommissionError::RecoveryPlanFailed));
    }

    #[test]
    fn build_origin_recovery_reports_missing_capture_state_without_recovery_plan() {
        let session = RobustnessReversalSession::new(test_point(), 0, COUNTS_PER_REV, test_rate());
        let safety = test_safety();

        let result = build_origin_recovery(&session, &safety);

        assert_eq!(result, Err(CommissionError::MissingCaptureState));
    }

    fn test_session_with_recovery_plan() -> RobustnessReversalSession {
        let point = recovery_point();
        let safety = test_safety();
        let plan = robustness_recovery_plan(&point, &safety).expect("recovery plan resolves");
        let mut session = RobustnessReversalSession::new(point, 0, COUNTS_PER_REV, test_rate());
        session.recovery_plan = Some(plan);
        session
    }

    #[test]
    fn build_origin_recovery_rejects_missing_return_lower_rate() {
        let mut session = test_session_with_recovery_plan();
        session.point.selected_response_intervals_q = [[0, 0], [0, 0]];
        let safety = test_safety();

        let error = build_origin_recovery(&session, &safety).unwrap_err();

        assert_eq!(error, CommissionError::MissingRateTableEntry);
    }

    fn drive(
        point: AcceptancePoint,
        noisy_reversal: bool,
    ) -> (
        RobustnessReversalTerminal,
        MockBackend,
        RobustnessReversalSession,
    ) {
        let mut jitter_high = false;
        drive_scripted(point, move |phase, _tick, target| {
            if target == 0 {
                return 0;
            }
            match phase {
                CapturePhase::Reverse if noisy_reversal => {
                    jitter_high = !jitter_high;
                    if jitter_high { 300 } else { -300 }
                }
                _ => target.signum() * STEADY_DELTA,
            }
        })
    }

    #[test]
    fn clean_reversal_completes_with_pass_cause() {
        let (terminal, _backend, _session) = drive(test_point(), false);
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(terminal.run_sequence, 7);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_eq!(direction.trip_count, 0);
            assert_eq!(direction.retry_count, 0);
            assert_eq!(
                direction.valid_cycles, ROBUSTNESS_CYCLES_PER_DIRECTION as u8,
                "a clean run fills every cycle slot"
            );
        }
    }

    #[test]
    fn establish_leg_settles_through_per_tick_ripple_around_target() {
        let mut high = false;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, _tick, target| {
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Establish => {
                        high = !high;
                        target.signum()
                            * if high {
                                STEADY_DELTA + 2
                            } else {
                                STEADY_DELTA - 2
                            }
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_eq!(direction.retry_count, 0, "ripple must not consume retries");
        }
    }

    #[test]
    fn reverse_leg_settles_through_per_tick_ripple_around_target() {
        let mut high = false;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, _tick, target| {
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Reverse => {
                        high = !high;
                        target.signum()
                            * if high {
                                STEADY_DELTA + 2
                            } else {
                                STEADY_DELTA - 2
                            }
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_eq!(direction.retry_count, 0, "ripple must not consume retries");
        }
    }

    #[test]
    fn noisy_reversal_exhausts_retries_and_reports_inconclusive() {
        let (terminal, _backend, _session) = drive(test_point(), true);
        assert_eq!(terminal.outcome, RobustnessOutcome::Inconclusive);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE);
        assert!(terminal.safety_fault.is_none());
        for direction in terminal.evidence.directions {
            assert!(direction.inconclusive);
            assert_eq!(direction.valid_cycles, 0, "no cycle ever reconverged");
            assert_eq!(direction.retry_count, ROBUSTNESS_MAX_RETRIES);
        }
    }

    #[test]
    fn never_ran_direction_reports_zero_valid_cycles() {
        let session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());

        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());

        for direction in evidence.directions {
            assert_eq!(direction.valid_cycles, 0);
            assert!(direction.inconclusive);
            assert_eq!(direction.trip_count, 0);
            assert_eq!(direction.median_reconvergence_ms, 0);
            assert_eq!(direction.max_reconvergence_ms, 0);
        }
        assert_eq!(evidence.residual_medians_q, [0, 0]);
        assert_eq!(evidence.iae_medians_qs, [0, 0]);
    }

    #[test]
    fn probe_legs_run_at_the_leg_fraction_of_the_amplitude() {
        let point = test_point();
        assert_eq!(point.target_velocity_rpm, 176);
        assert_eq!(establish_target_rpm(&point, 0), 150);
        assert_eq!(establish_target_rpm(&point, 1), -150);
        assert_eq!(reverse_target_rpm(&point, 0), -150);
        assert_eq!(establish_rate_counts_per_ms(&point, COUNTS_PER_REV), 10);
    }

    #[test]
    fn establish_leg_travel_covers_budget_settle_tail_and_analysis() {
        let rate = establish_rate_counts_per_ms(&test_point(), COUNTS_PER_REV);
        assert_eq!(rate, 10);
        assert_eq!(ESTABLISH_LEG_WINDOW_US, 1_016_000);
        assert_eq!(establish_leg_travel_counts(rate, 0), 10 * 1_016);
    }

    #[test]
    fn cycle_fits_exactly_at_the_remaining_headroom_boundary_on_both_sides() {
        let travel = establish_leg_travel_counts(12, 0);
        let leg = travel as i32;
        let window: u32 = 30_000;
        let edge = window as i32 - leg;
        assert!(cycle_fits(travel, edge, window, window, 0));
        assert!(!cycle_fits(travel, edge + 1, window, window, 0));
        assert!(cycle_fits(travel, -edge, window, window, 1));
        assert!(!cycle_fits(travel, -(edge + 1), window, window, 1));
        assert!(cycle_fits(travel, -edge, window, window, 0));
        assert!(!cycle_fits(travel, -(edge + 1), window, window, 0));
        assert!(cycle_fits(travel, edge, window, window, 1));
        assert!(!cycle_fits(travel, edge + 1, window, window, 1));
        assert_eq!(
            remaining_headroom_counts(window as i32 + 5, window, window, 0),
            0
        );
        assert_eq!(
            remaining_headroom_counts(-(window as i32) - 5, window, window, 1),
            0
        );
        assert!(!cycle_fits(travel, window as i32 + 5, window, window, 0));
    }

    #[test]
    fn assembled_evidence_saturates_extreme_cycle_values() {
        const HUGE_DURATION_US: u32 = 200_000_000;

        let mut session =
            RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());
        session.ledgers[0] = CycleLedger::new(i64::MAX);
        session.ledgers[1] = CycleLedger::new(i64::MAX);
        session.ledgers[0].record(CycleRecord::Conclusive(ReversalMetrics {
            reconvergence_time_us: HUGE_DURATION_US,
            settled_residual_q: i64::MAX,
            recovery_iae_q: i64::MAX,
            dac_rms_q: 0,
            forward_settle_time_us: HUGE_DURATION_US,
            reconvergence_ratio_ppm: 0,
            late_tracking: late_tracking_at(0),
        }));
        session.ledgers[1].record(CycleRecord::Conclusive(ReversalMetrics {
            reconvergence_time_us: HUGE_DURATION_US,
            settled_residual_q: i64::MIN,
            recovery_iae_q: i64::MIN,
            dac_rms_q: 0,
            forward_settle_time_us: HUGE_DURATION_US,
            reconvergence_ratio_ppm: 0,
            late_tracking: late_tracking_at(0),
        }));

        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());

        for direction in evidence.directions {
            assert_eq!(direction.median_reconvergence_ms, u16::MAX);
            assert_eq!(direction.max_reconvergence_ms, u16::MAX);
            assert_eq!(direction.median_forward_settle_ms, u16::MAX);
            assert_eq!(direction.valid_cycles, 1);
        }
        assert_eq!(evidence.residual_medians_q, [i32::MAX, i32::MIN]);
        assert_eq!(evidence.iae_medians_qs, [i32::MAX, i32::MIN]);
    }

    #[test]
    fn integrator_clears_only_on_establish_never_on_reversal() {
        let (_terminal, backend, _session) = drive(test_point(), false);
        assert_eq!(
            backend.pid_velocity_error_sum_clears,
            ROBUSTNESS_CYCLES_PER_DIRECTION * DIRECTION_COUNT,
            "clean run clears the integrator once per establish leg"
        );

        let clear_addr = PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr();
        let target_addr = PID_VELOCITY_TARGET.addr();
        let last_reversal = backend
            .spi_log
            .iter()
            .rposition(
                |op| matches!(op, SpiOp::Write(addr, value) if *addr == target_addr && *value != 0),
            )
            .expect("at least one nonzero velocity-target write");
        let stray_clear_after_reversal = backend.spi_log[last_reversal + 1..].iter().any(
            |op| matches!(op, SpiOp::Write(addr, value) if *addr == clear_addr && *value == 0),
        );
        assert!(
            !stray_clear_after_reversal,
            "no integrator clear may follow the measured reversal target write"
        );

        let (_terminal, noisy_backend, _noisy_session) = drive(test_point(), true);
        let attempts_per_direction =
            ROBUSTNESS_CYCLES_PER_DIRECTION + usize::from(ROBUSTNESS_MAX_RETRIES);
        assert_eq!(
            noisy_backend.pid_velocity_error_sum_clears,
            attempts_per_direction * DIRECTION_COUNT,
            "each establish attempt clears once; reversals never clear"
        );
    }

    #[test]
    fn establish_leg_clears_the_velocity_offset() {
        let (_terminal, backend, _session) = drive(test_point(), false);
        let offset_addr = PID_VELOCITY_OFFSET.addr();
        let offset_clears = backend
            .spi_log
            .iter()
            .filter(
                |op| matches!(op, SpiOp::Write(addr, value) if *addr == offset_addr && *value == 0),
            )
            .count();
        assert_eq!(
            offset_clears,
            ROBUSTNESS_CYCLES_PER_DIRECTION * DIRECTION_COUNT,
            "each establish leg zeroes the velocity offset"
        );
    }

    #[test]
    fn run_leaves_the_retained_acceptance_point_untouched() {
        for (noisy, expected) in [(false, false), (true, true)] {
            let point = test_point();
            let mut slot = IntegralRetentionSlot::new();
            slot.install_acceptance_point(point);
            let before = slot.acceptance_point().expect("point installed");

            let (terminal, _backend, _session) = drive(point, noisy);
            assert_eq!(
                terminal.outcome == RobustnessOutcome::Inconclusive,
                expected
            );

            assert_eq!(slot.acceptance_point().expect("point retained"), before);
        }
    }

    #[test]
    fn slow_reconvergence_trips_time_and_rejects() {
        let (terminal, _backend, _session) = drive_scripted(test_point(), |phase, tick, target| {
            if target == 0 {
                return 0;
            }
            match phase {
                CapturePhase::Reverse if tick < 620 => 0,
                _ => target.signum() * STEADY_DELTA,
            }
        });
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED);
        assert!(terminal.safety_fault.is_none());
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_ne!(
                direction.trip_count, 0,
                "slow reconvergence must trip the time gate"
            );
            assert!(
                u32::from(direction.median_reconvergence_ms) * 1_000
                    > ROBUSTNESS_THRESHOLDS.t_max_us,
                "reconvergence must exceed t_max to reach TRIP_TIME"
            );
            assert_eq!(direction.retry_count, 0);
        }
    }

    #[test]
    fn mixed_trip_and_inconclusive_yields_rejected_with_trip_cause() {
        let mut jitter_high = false;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| match phase {
                CapturePhase::Reverse if target < 0 => {
                    if tick < 620 {
                        0
                    } else {
                        target.signum() * STEADY_DELTA
                    }
                }
                CapturePhase::Reverse if target > 0 => {
                    jitter_high = !jitter_high;
                    if jitter_high { 300 } else { -300 }
                }
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED);
        assert!(terminal.safety_fault.is_none());
        let [dir0, dir1] = terminal.evidence.directions;
        assert!(!dir0.inconclusive, "the tripped direction is conclusive");
        assert_ne!(dir0.trip_count, 0, "direction 0 must trip the time gate");
        assert!(dir1.inconclusive, "direction 1 must be inconclusive");
        assert_eq!(
            dir1.trip_count, 0,
            "an inconclusive direction reports no trips"
        );
    }

    #[test]
    fn excluded_establish_settle_retries_and_counts() {
        let (terminal, _backend, _session) = drive_scripted(test_point(), |phase, tick, target| {
            if target == 0 {
                return 0;
            }
            match phase {
                CapturePhase::Establish => {
                    if tick < 80 {
                        target.signum() * STEADY_DELTA
                    } else {
                        target.signum() * STEADY_DELTA + (tick as i32 - 80) * 3
                    }
                }
                _ => target.signum() * STEADY_DELTA,
            }
        });
        assert_eq!(terminal.outcome, RobustnessOutcome::Inconclusive);
        for direction in terminal.evidence.directions {
            assert!(direction.inconclusive);
            assert_eq!(
                direction.retry_count, ROBUSTNESS_MAX_RETRIES,
                "an Excluded establish retries rather than proceeding"
            );
        }
    }

    #[test]
    fn rejected_between_legs_rest_reopens_one_window_then_proceeds() {
        let mut zero_windows = 0u32;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| match phase {
                CapturePhase::Zero => {
                    if tick == 0 {
                        zero_windows += 1;
                    }
                    if zero_windows == 1 { 8 } else { 0 }
                }
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert!(terminal.safety_fault.is_none());
    }

    #[test]
    fn rest_rejected_beyond_retry_faults() {
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), |phase, _tick, target| match phase {
                CapturePhase::Zero => 8,
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED);
        assert!(terminal.safety_fault.is_none());
    }

    fn wire_reconvergence_median(
        cycles: &[Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION],
    ) -> u16 {
        let mut values = [0u16; ROBUSTNESS_CYCLES_PER_DIRECTION];
        for (slot, cycle) in cycles.iter().enumerate() {
            values[slot] = cycle.expect("filled cycle").reconvergence_ms;
        }
        values.sort_unstable();
        values[ROBUSTNESS_CYCLES_PER_DIRECTION / 2]
    }

    #[test]
    fn five_clean_cycles_per_direction_complete_with_median_summaries() {
        const STALLS_MS: [u32; ROBUSTNESS_CYCLES_PER_DIRECTION] = [60, 110, 80, 130, 70];
        let mut attempt = [0usize; DIRECTION_COUNT];
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| match phase {
                CapturePhase::Reverse => {
                    let dir = if target < 0 { 0 } else { 1 };
                    if tick == 0 {
                        attempt[dir] += 1;
                    }
                    let index = (attempt[dir] - 1).min(ROBUSTNESS_CYCLES_PER_DIRECTION - 1);
                    if tick < STALLS_MS[index] {
                        0
                    } else {
                        target.signum() * STEADY_DELTA
                    }
                }
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for (index, summary) in terminal.evidence.directions.iter().enumerate() {
            assert_eq!(summary.valid_cycles, ROBUSTNESS_CYCLES_PER_DIRECTION as u8);
            let expected = wire_reconvergence_median(&terminal.evidence.cycles[index]);
            assert_eq!(summary.median_reconvergence_ms, expected);
            let mut sorted = [0u16; ROBUSTNESS_CYCLES_PER_DIRECTION];
            for (slot, cycle) in terminal.evidence.cycles[index].iter().enumerate() {
                sorted[slot] = cycle.expect("filled cycle").reconvergence_ms;
            }
            sorted.sort_unstable();
            for pair in sorted.windows(2) {
                assert_ne!(pair[0], pair[1], "each cycle's reconvergence is distinct");
            }
        }
    }

    #[test]
    fn slow_median_rejects_with_time_cause() {
        const STALLS_MS: [u32; ROBUSTNESS_CYCLES_PER_DIRECTION] = [580, 615, 605, 595, 610];
        let mut attempt = [0usize; DIRECTION_COUNT];
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| match phase {
                CapturePhase::Reverse => {
                    let dir = if target < 0 { 0 } else { 1 };
                    if tick == 0 {
                        attempt[dir] += 1;
                    }
                    let index = (attempt[dir] - 1).min(ROBUSTNESS_CYCLES_PER_DIRECTION - 1);
                    if tick < STALLS_MS[index] {
                        0
                    } else {
                        target.signum() * STEADY_DELTA
                    }
                }
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });

        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED);
        for summary in terminal.evidence.directions {
            assert!(!summary.inconclusive);
            assert_ne!(summary.trip_count, 0);
            assert!(
                u32::from(summary.median_reconvergence_ms) * 1_000 > ROBUSTNESS_THRESHOLDS.t_max_us,
                "the median must exceed t_max to trip the time gate"
            );
        }
    }

    fn plant_identified_point() -> AcceptancePoint {
        AcceptancePoint {
            selected_response_intervals_q: [[50_000, 60_000], [-60_000, -50_000]],
            ..test_point()
        }
    }

    fn iae_metrics(recovery_iae_q: i64) -> ReversalMetrics {
        ReversalMetrics {
            reconvergence_time_us: 50_000,
            settled_residual_q: 0,
            recovery_iae_q,
            dac_rms_q: 0,
            forward_settle_time_us: 10_000,
            reconvergence_ratio_ppm: 0,
            late_tracking: late_tracking_at(0),
        }
    }

    fn complete_with_uniform_iae(
        point: AcceptancePoint,
        recovery_iae_q: i64,
    ) -> RobustnessReversalTerminal {
        let backend = test_backend();
        let mut session = RobustnessReversalSession::new(point, 7, COUNTS_PER_REV, test_rate());
        for dir in 0..DIRECTION_COUNT {
            for _ in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
                session.ledgers[dir].record(CycleRecord::Conclusive(iae_metrics(recovery_iae_q)));
            }
        }
        let StepResult::RobustnessReversalTerminal(terminal) =
            complete_terminal(&session, &backend, test_safety().counts_per_rev())
        else {
            panic!("complete_terminal must yield a robustness terminal");
        };
        terminal
    }

    #[test]
    fn median_iae_exceeding_plant_derived_bound_trips_and_rejects() {
        let terminal = complete_with_uniform_iae(plant_identified_point(), 5_000_000_000);
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_TAIL_REPEATED);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_ne!(direction.trip_count, 0, "the excess median IAE must trip");
        }
    }

    #[test]
    fn median_iae_within_plant_derived_bound_completes() {
        let terminal = complete_with_uniform_iae(plant_identified_point(), 4_000_000_000);
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_eq!(direction.trip_count, 0);
        }
    }

    #[test]
    fn robustness_pass_provenance_carries_confidence_and_band_position() {
        let point = AcceptancePoint {
            confidence_q: 40_000,
            band_position_q: 20_000,
            ..test_point()
        };
        let terminal = complete_with_uniform_iae(point, NON_TAIL_IAE_Q);
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        let entry = sample_entry_snapshot();

        let result = robustness_pass_outer_result(&terminal, &entry, &point);

        assert_eq!(result.provenance.confidence_q, 40_000);
        assert_eq!(result.provenance.band_position_q, 20_000);
    }

    const NON_TAIL_IAE_Q: i64 = 100_000;
    const TAIL_IAE_Q: i64 = 25_000_000_000_000;

    fn cycle_metrics(reconvergence_ms: u32) -> ReversalMetrics {
        cycle_metrics_with_iae(reconvergence_ms, NON_TAIL_IAE_Q)
    }

    fn cycle_metrics_with_iae(reconvergence_ms: u32, recovery_iae_q: i64) -> ReversalMetrics {
        ReversalMetrics {
            reconvergence_time_us: reconvergence_ms.saturating_mul(1_000),
            settled_residual_q: 0,
            recovery_iae_q,
            dac_rms_q: 0,
            forward_settle_time_us: 10_000,
            reconvergence_ratio_ppm: 0,
            late_tracking: late_tracking_at(0),
        }
    }

    #[test]
    fn single_tail_cycle_retries_then_passes() {
        const TAIL_MS: u32 = 900;
        const FAST_MS: u32 = 60;
        let backend = test_backend();
        let mut session =
            RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());
        for dir in 0..DIRECTION_COUNT {
            assert_eq!(
                session.ledgers[dir].record(CycleRecord::Conclusive(cycle_metrics_with_iae(
                    TAIL_MS, TAIL_IAE_Q,
                ))),
                LedgerAction::RetryTail
            );
            for _ in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
                session.ledgers[dir].record(CycleRecord::Conclusive(cycle_metrics(FAST_MS)));
            }
        }

        let StepResult::RobustnessReversalTerminal(terminal) =
            complete_terminal(&session, &backend, test_safety().counts_per_rev())
        else {
            panic!("complete_terminal must yield a robustness terminal");
        };
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for dir in 0..DIRECTION_COUNT {
            assert_eq!(
                session.ledgers[dir].tail_count(),
                1,
                "one grace-retried tail cycle per direction"
            );
        }
        for summary in terminal.evidence.directions {
            assert_eq!(summary.valid_cycles, ROBUSTNESS_CYCLES_PER_DIRECTION as u8);
            assert!(
                summary.max_reconvergence_ms >= TAIL_MS as u16,
                "max reconvergence reflects the displaced 900 ms tail"
            );
        }
    }

    #[test]
    fn second_tail_cycle_rejects_with_tail_cause_and_both_directions_measured() {
        const TAIL_MS: u32 = 900;
        const FAST_MS: u32 = 60;
        let backend = test_backend();
        let mut session =
            RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());
        assert_eq!(
            session.ledgers[0].record(CycleRecord::Conclusive(cycle_metrics_with_iae(
                TAIL_MS, TAIL_IAE_Q,
            ))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            session.ledgers[0].record(CycleRecord::Conclusive(cycle_metrics_with_iae(
                TAIL_MS, TAIL_IAE_Q,
            ))),
            LedgerAction::RejectTail
        );
        for _ in 0..(ROBUSTNESS_CYCLES_PER_DIRECTION - 1) {
            session.ledgers[0].record(CycleRecord::Conclusive(cycle_metrics(FAST_MS)));
        }
        for _ in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
            session.ledgers[1].record(CycleRecord::Conclusive(cycle_metrics(FAST_MS)));
        }

        let StepResult::RobustnessReversalTerminal(terminal) =
            complete_terminal(&session, &backend, test_safety().counts_per_rev())
        else {
            panic!("complete_terminal must yield a robustness terminal");
        };
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_TAIL_REPEATED);
        for summary in terminal.evidence.directions {
            assert!(
                summary.valid_cycles >= 4,
                "both directions complete their cycles despite the latch"
            );
        }
    }

    #[test]
    fn apply_pending_cycle_tail_arms_skip_grace_then_latch_and_continue() {
        const TAIL_MS: u32 = 900;
        let tail = cycle_metrics_with_iae(TAIL_MS, TAIL_IAE_Q);
        let tail_wire = CycleWire {
            reconvergence_ms: TAIL_MS as u16,
            overshoot_counts: 3,
            forward_settle_ms: 12,
        };
        let tail_cycle = PendingCycle {
            metrics: tail,
            wire: tail_wire,
            peak_permille: 0,
            suffix_excluded: false,
        };

        let mut state = RobustnessReversalState::EstablishWriteGain;
        let mut session =
            RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());
        assert_eq!(session.direction, 0);
        assert_eq!(session.cycle_cursor[0], 0);

        apply_pending_cycle(&mut state, &mut session, tail_cycle);
        assert_eq!(state, RobustnessReversalState::EstablishWriteGain);
        assert_eq!(session.direction, 0, "RetryTail retries the same direction");
        assert_eq!(session.cycle_cursor[0], 0, "RetryTail does not advance");
        assert_eq!(
            session.cycle_wires[0][0], None,
            "displaced tail is not filed"
        );
        assert_eq!(
            session.displaced_tail_wires[0],
            Some(tail_wire),
            "the displaced tail's full triple (overshoot + settle) is retained"
        );
        assert_eq!(session.ledgers[0].valid_cycles(), 0);
        assert!(!session.ledgers[0].tail_rejected());
        assert!(
            session.ledgers[0].max_reconvergence_ms() >= TAIL_MS as u16,
            "the ledger still carries the displaced tail in max"
        );

        apply_pending_cycle(&mut state, &mut session, tail_cycle);
        assert_eq!(
            state,
            RobustnessReversalState::EstablishWriteGain,
            "RejectTail continues the run, it does not short-circuit"
        );
        assert_eq!(session.direction, 0);
        assert!(
            session.ledgers[0].tail_rejected(),
            "the second tail latches"
        );
        assert_eq!(session.cycle_cursor[0], 1, "RejectTail advances one slot");
        assert_eq!(
            session.cycle_wires[0][0],
            Some(tail_wire),
            "RejectTail files the second tail's wire"
        );

        assert_eq!(session.ledgers[0].valid_cycles(), 1);
        for slot in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
            assert_eq!(
                session.cycle_wires[0][slot].is_some(),
                session.ledgers[0].cycles()[slot].is_some(),
                "wire slot {slot} mirrors the ledger slot",
            );
        }

        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());
        assert_eq!(
            evidence.tail_counts[0], 2,
            "grace tail plus the latched tail"
        );
        assert_eq!(evidence.displaced_tails[0], Some(tail_wire));
    }

    #[test]
    fn floor_breach_reports_inconclusive() {
        let mut attempt = [0usize; DIRECTION_COUNT];
        let mut jitter = false;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| match phase {
                CapturePhase::Reverse => {
                    let dir = if target < 0 { 0 } else { 1 };
                    if tick == 0 {
                        attempt[dir] += 1;
                    }
                    if attempt[dir] <= 6 {
                        jitter = !jitter;
                        if jitter { 300 } else { -300 }
                    } else {
                        target.signum() * STEADY_DELTA
                    }
                }
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });

        assert_eq!(terminal.outcome, RobustnessOutcome::Inconclusive);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE);
        for summary in terminal.evidence.directions {
            assert!(summary.inconclusive);
            assert_eq!(summary.valid_cycles, 3);
        }
    }

    #[test]
    fn safety_fault_mid_cycle_still_fails_with_envelope_cause() {
        use crate::commission_dispatch::robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;

        let (terminal, _backend, _session) =
            drive_scripted(test_point(), |phase, tick, target| match phase {
                CapturePhase::Reverse if tick == 0 => 1_500,
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });

        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_SAFETY_FAULT);
        assert!(terminal.safety_fault.is_some());
    }

    #[test]
    fn assembled_evidence_medians_convert_iae_before_median() {
        let mut session =
            RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());
        session.ledgers[0] = CycleLedger::new(i64::from(u32::MAX));
        let residuals_q = [10_i64, 40, 20, 30];
        let iae_us = [1_500_000_i64, 4_500_000, 2_500_000, 3_500_000];
        for (residual_q, iae) in residuals_q.into_iter().zip(iae_us) {
            session.ledgers[0].record(CycleRecord::Conclusive(ReversalMetrics {
                reconvergence_time_us: 50_000,
                settled_residual_q: residual_q,
                recovery_iae_q: iae,
                dac_rms_q: 0,
                forward_settle_time_us: 10_000,
                reconvergence_ratio_ppm: 0,
                late_tracking: late_tracking_at(0),
            }));
        }

        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());

        assert_eq!(evidence.residual_medians_q[0], 25);
        assert_eq!(evidence.iae_medians_qs[0], 2);
        assert_eq!(evidence.residual_medians_q[1], 0);
        assert_eq!(evidence.iae_medians_qs[1], 0);
    }

    #[test]
    fn assembled_evidence_dac_rms_median_reduces_filled_slots() {
        let mut session =
            RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());
        session.ledgers[0] = CycleLedger::new(i64::from(u32::MAX));
        let dac_rms_values = [100_i64, 400, 200, 300];
        for dac_rms_q in dac_rms_values {
            session.ledgers[0].record(CycleRecord::Conclusive(ReversalMetrics {
                reconvergence_time_us: 50_000,
                settled_residual_q: 0,
                recovery_iae_q: 0,
                dac_rms_q,
                forward_settle_time_us: 10_000,
                reconvergence_ratio_ppm: 0,
                late_tracking: late_tracking_at(0),
            }));
        }

        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());

        assert_eq!(evidence.dac_rms_medians_q[0], 250);
        assert_eq!(evidence.dac_rms_medians_q[1], 0);
    }

    fn burst_motion(
        burst_phase: CapturePhase,
        burst_delta: i32,
        burst_ticks: u32,
    ) -> impl FnMut(CapturePhase, u32, i32) -> i32 {
        move |phase, tick, target| {
            if target == 0 {
                0
            } else if phase == burst_phase && tick < burst_ticks {
                target.signum() * burst_delta
            } else {
                target.signum() * STEADY_DELTA
            }
        }
    }

    fn drive_tracked(
        motion: impl FnMut(CapturePhase, u32, i32) -> i32,
    ) -> (RobustnessReversalTerminal, RunParts) {
        drive_parts(RunParts::new(recovery_point()), Envelope::Tracked, motion)
    }

    #[test]
    fn overshoot_reaches_the_terminal_when_a_reverse_leg_outpaces_its_target() {
        let (terminal, parts) =
            drive_tracked(burst_motion(CapturePhase::Reverse, STEADY_DELTA * 2, 3));

        assert!(terminal.safety_fault.is_none());
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_REVERSAL_PEAK_EXCEEDED);
        for direction in 0..DIRECTION_COUNT {
            assert!(
                terminal.evidence.directions[direction].peak_rate_permille > 1000,
                "a genuine burst above the target rate must reach the wire"
            );
            let first_cycle = parts.session.cycle_wires[direction][0]
                .expect("a clean reversal run fills every cycle slot");
            assert!(
                first_cycle.overshoot_counts > 0,
                "the per-cycle evidence reply's own overshoot_counts field must \
                 carry the burst too, not just the terminal's aggregate peak"
            );
        }
    }

    #[test]
    fn a_reversal_peaking_one_band_under_the_cap_rejects_with_the_peak_cause() {
        let (terminal, _parts) =
            drive_tracked(burst_motion(CapturePhase::Reverse, STEADY_DELTA * 9 / 5, 5));

        assert!(terminal.safety_fault.is_none());
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_REVERSAL_PEAK_EXCEEDED);
        for direction in terminal.evidence.directions {
            assert!(direction.peak_rate_permille >= REVERSAL_PEAK_REJECT_PERMILLE);
            assert!(!direction.peak_unscored);
            assert_eq!(direction.trip_count, 1);
        }
    }

    #[test]
    fn a_retried_tail_reversal_does_not_count_toward_the_peak_gate() {
        let mut parts = RunParts::new(recovery_point());
        for ledger in &mut parts.session.ledgers {
            *ledger = CycleLedger::new(1_000_000_000);
        }
        let mut reverse_legs = 0_u32;
        let (terminal, parts) = drive_parts(parts, Envelope::Tracked, |phase, tick, target| {
            if target == 0 {
                return 0;
            }
            if phase == CapturePhase::Reverse && tick == 0 {
                reverse_legs += 1;
            }
            if phase == CapturePhase::Reverse && reverse_legs == 1 && tick < 5 {
                target.signum() * STEADY_DELTA * 3 / 2
            } else {
                target.signum() * STEADY_DELTA
            }
        });

        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.session.ledgers[0].tail_count(), 1);
        assert!(parts.session.displaced_tail_wires[0].is_some());
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in terminal.evidence.directions {
            assert!(
                direction.peak_rate_permille < REVERSAL_PEAK_REJECT_PERMILLE,
                "peak {} permille",
                direction.peak_rate_permille
            );
            assert!(!direction.peak_unscored);
            assert_eq!(direction.trip_count, 0);
        }
    }

    #[test]
    fn a_window_exactly_at_the_cap_is_retained_and_rejected_by_the_gate() {
        // 180 RPM plans 3000 mrev/s, so the moving cap is 3000 + 2000 = 5000 mrev/s, which at
        // 4000 counts per revolution is exactly 20 counts per tick over the 2-tick window.
        let point = AcceptancePoint {
            target_velocity_rpm: 180,
            ..recovery_point()
        };
        let mut parts = RunParts::new(point);
        parts
            .safety
            .configure_velocity_envelope(planned_velocity_mrev_s(&point).expect("planned"))
            .expect("envelope for the 180 RPM point");
        assert_eq!(
            parts.safety.moving_velocity_cap_milli_counts_per_s(),
            20_000_000
        );
        let (terminal, _parts) = drive_parts(
            parts,
            Envelope::Tracked,
            burst_motion(CapturePhase::Reverse, 20, 2),
        );

        assert!(terminal.safety_fault.is_none());
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_REVERSAL_PEAK_EXCEEDED);
    }

    #[test]
    fn the_gate_bound_sits_under_the_integer_cap_for_every_leg_target() {
        for rpm in 1..=10_000 {
            let point = AcceptancePoint {
                target_velocity_rpm: rpm,
                ..recovery_point()
            };
            let Ok(planned) = planned_velocity_mrev_s(&point) else {
                continue;
            };
            let leg = leg_amplitude_rpm(&point);
            if leg == 0 {
                continue;
            }
            let cap =
                OuterSafetyLimits::envelope_mrev_s(planned).expect("cap for planned velocity");
            let ratio_permille = u64::from(cap) * 60 / u64::try_from(leg).expect("positive leg");
            assert!(
                ratio_permille > u64::from(REVERSAL_PEAK_REJECT_PERMILLE),
                "rpm {rpm}: cap {cap} mrev/s, leg {leg} RPM, ratio {ratio_permille} permille"
            );
        }
    }

    #[test]
    fn a_reversal_under_the_bound_passes_and_reports_its_peak() {
        let (terminal, _parts) =
            drive_tracked(burst_motion(CapturePhase::Reverse, STEADY_DELTA * 6 / 5, 5));

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in terminal.evidence.directions {
            assert!(
                (1150..=1250).contains(&direction.peak_rate_permille),
                "peak {} permille",
                direction.peak_rate_permille
            );
            assert!(!direction.peak_unscored);
            assert_eq!(direction.trip_count, 0);
        }
    }

    #[test]
    fn the_establish_leg_peak_does_not_leak_into_the_reversal_peak() {
        let (terminal, _parts) = drive_tracked(burst_motion(
            CapturePhase::Establish,
            STEADY_DELTA * 8 / 5,
            5,
        ));

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in terminal.evidence.directions {
            assert!(direction.peak_rate_permille < REVERSAL_PEAK_REJECT_PERMILLE);
            assert!(!direction.peak_unscored);
            assert_eq!(direction.trip_count, 0);
        }
    }

    #[test]
    fn a_direction_with_no_valid_cycles_reports_unscored_and_no_peak_trip() {
        let (terminal, _parts) = drive_tracked(|phase, tick, target| {
            if target == 0 {
                0
            } else if phase == CapturePhase::Reverse && target > 0 {
                if tick < 5 {
                    target.signum() * STEADY_DELTA * 8 / 5
                } else {
                    target.signum() * STEADY_DELTA / 2
                }
            } else {
                target.signum() * STEADY_DELTA
            }
        });

        assert!(terminal.safety_fault.is_none());
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE);
        let unscored = terminal.evidence.directions[1];
        assert_eq!(unscored.valid_cycles, 0);
        assert!(unscored.peak_unscored);
        assert_eq!(unscored.peak_rate_permille, 0);
        assert_eq!(unscored.trip_count, 0);
    }

    #[test]
    fn a_peak_trip_outranks_an_iae_trip_in_the_same_direction() {
        let backend = test_backend();
        let mut session = RobustnessReversalSession::new(
            plant_identified_point(),
            7,
            COUNTS_PER_REV,
            test_rate(),
        );
        for dir in 0..DIRECTION_COUNT {
            session.ledgers[dir] = CycleLedger::new(i64::MAX);
            for _ in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
                session.ledgers[dir].record(CycleRecord::Conclusive(iae_metrics(5_000_000_000)));
            }
        }
        let terminal_of = |session: &RobustnessReversalSession| {
            let StepResult::RobustnessReversalTerminal(terminal) =
                complete_terminal(session, &backend, COUNTS_PER_REV)
            else {
                panic!("complete_terminal must yield a robustness terminal");
            };
            terminal
        };
        assert_eq!(terminal_of(&session).cause, ROBUSTNESS_CAUSE_IAE_EXCEEDED);

        session.peak_rate_permille[0] = REVERSAL_PEAK_REJECT_PERMILLE;
        session.peak_scored_cycles[0] = ROBUSTNESS_CYCLES_PER_DIRECTION as u8;
        let terminal = terminal_of(&session);

        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_REVERSAL_PEAK_EXCEEDED);
        assert_eq!(terminal.evidence.directions[0].trip_count, 1);
    }

    #[test]
    fn every_inconclusive_holds_its_route_and_rests_first() {
        let mut state = RobustnessReversalState::EstablishWriteGain;
        let mut session =
            RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV, test_rate());

        for _ in 0..ROBUSTNESS_MAX_RETRIES {
            retry_or_finalize_inconclusive(&mut state, &mut session, 150);
            assert_eq!(
                state,
                zero_ramp_state(&session, 150),
                "a granted retry rests before re-entering establish"
            );
            assert_eq!(
                session.held_route.take(),
                Some(HeldRoute::Ledger(LedgerAction::RetryInconclusive))
            );
            assert_eq!(
                session.cycle_cursor[0], 0,
                "a granted retry does not consume a slot"
            );
            state = RobustnessReversalState::EstablishWriteGain;
        }

        retry_or_finalize_inconclusive(&mut state, &mut session, 150);
        assert_eq!(state, zero_ramp_state(&session, 150));
        assert_eq!(
            session.held_route.take(),
            Some(HeldRoute::Ledger(LedgerAction::NextCycle))
        );
        assert_eq!(
            session.cycle_cursor[0], 1,
            "the exhausted inconclusive consumed the slot"
        );
        assert_eq!(session.ledgers[0].valid_cycles(), 0);
    }

    #[test]
    fn exhausted_establish_rests_before_its_retry() {
        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            RunParts::new(test_point()),
            Envelope::Untracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(
            &parts.targets[..9],
            &[60, 120, 150, 90, 30, 0, 60, 120, 150]
        );
        assert_eq!(terminal.evidence.directions[0].retry_count, 1);
        assert!(terminal.safety_fault.is_none());
        assert_ne!(terminal.outcome, RobustnessOutcome::Failed);
    }

    #[test]
    fn conclusive_cycles_keep_their_target_sequence() {
        let (terminal, parts) = drive_parts(
            RunParts::new(test_point()),
            Envelope::Untracked,
            |_phase, _tick, target| {
                if target == 0 {
                    0
                } else {
                    target.signum() * STEADY_DELTA
                }
            },
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        const FORWARD_CYCLE: [i32; 11] = [60, 120, 150, 90, 30, -30, -90, -150, -90, -30, 0];
        let mut expected: heapless::Vec<i32, 128> = heapless::Vec::new();
        for sign in [1, -1] {
            for _ in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
                for target in FORWARD_CYCLE {
                    expected
                        .push(sign * target)
                        .expect("expected sequence fits");
                }
            }
        }
        assert_eq!(parts.targets.as_slice(), expected.as_slice());
        assert_eq!(
            parts.backend.pid_velocity_error_sum_clears,
            ROBUSTNESS_CYCLES_PER_DIRECTION * DIRECTION_COUNT
        );
        for direction in terminal.evidence.directions {
            assert_eq!(
                direction.valid_cycles,
                ROBUSTNESS_CYCLES_PER_DIRECTION as u8
            );
            assert_eq!(direction.retry_count, 0);
        }
    }

    #[test]
    fn exhausted_inconclusive_that_cannot_rest_recovers_and_ends_by_the_ledger() {
        let mut jitter_high = false;
        let (terminal, parts) = drive_parts(
            RunParts::new(test_point()),
            Envelope::Untracked,
            move |phase, _tick, target| match phase {
                CapturePhase::Reverse => {
                    jitter_high = !jitter_high;
                    if jitter_high { 300 } else { -300 }
                }
                CapturePhase::Zero => 8,
                _ if target == 0 => 0,
                _ => target.signum() * STEADY_DELTA,
            },
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Inconclusive);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(
            parts.recoveries,
            u32::try_from(
                (ROBUSTNESS_CYCLES_PER_DIRECTION + usize::from(ROBUSTNESS_MAX_RETRIES))
                    * DIRECTION_COUNT
            )
            .expect("recovery count fits u32"),
            "every rejected rest still reaches a recovery: retries and slot-consumes alike"
        );
        assert!(parts.session.held_route.is_none());
        for direction in terminal.evidence.directions {
            assert_eq!(direction.retry_count, ROBUSTNESS_MAX_RETRIES);
            assert_eq!(direction.valid_cycles, 0);
        }
    }

    fn leg_travel(point: &AcceptancePoint) -> i32 {
        leg_travel_counts(point, COUNTS_PER_REV, test_rate()) as i32
    }

    fn nonzero_target_writes(backend: &MockBackend) -> usize {
        let target_addr = PID_VELOCITY_TARGET.addr();
        backend
            .spi_log
            .iter()
            .filter(
                |op| matches!(op, SpiOp::Write(addr, value) if *addr == target_addr && *value != 0),
            )
            .count()
    }

    #[test]
    fn precondition_failure_rests_and_holds_a_reentry_route() {
        let mut parts = RunParts::new(test_point());
        assert!(matches!(parts.step(), StepResult::ReadyNow));
        assert_eq!(parts.state, RobustnessReversalState::EstablishWriteGain);

        parts.safety.unwrapped_position =
            parts.safety.positive_position_window as i32 - leg_travel(&test_point()) + 1;
        assert!(matches!(parts.step(), StepResult::ReadyNow));

        assert_eq!(parts.state, zero_ramp_state(&parts.session, 0));
        assert_eq!(parts.session.held_route, Some(HeldRoute::ReenterEstablish));
        assert_eq!(nonzero_target_writes(&parts.backend), 0);
    }

    #[test]
    fn precondition_refuses_a_cycle_whose_reverse_leg_would_not_fit() {
        for direction in 0..DIRECTION_COUNT {
            let leg = leg_travel(&test_point());
            let mut parts = RunParts::new(test_point());
            assert!(matches!(parts.step(), StepResult::ReadyNow));
            parts.session.direction = direction;
            parts.safety.unwrapped_position = if direction == 0 {
                -(parts.safety.negative_position_window as i32 - leg + 1)
            } else {
                parts.safety.positive_position_window as i32 - leg + 1
            };
            assert!(matches!(parts.step(), StepResult::ReadyNow));

            assert_eq!(parts.state, zero_ramp_state(&parts.session, 0));
            assert_eq!(parts.session.held_route, Some(HeldRoute::ReenterEstablish));
            assert_eq!(nonzero_target_writes(&parts.backend), 0);
        }
    }

    #[test]
    fn precondition_passes_exactly_at_the_boundary_in_both_directions() {
        for direction in 0..DIRECTION_COUNT {
            let leg = leg_travel(&test_point());
            let fits = |parts: &RunParts| {
                if direction == 0 {
                    parts.safety.positive_position_window as i32 - leg
                } else {
                    -(parts.safety.negative_position_window as i32 - leg)
                }
            };

            let mut parts = RunParts::new(test_point());
            assert!(matches!(parts.step(), StepResult::ReadyNow));
            parts.session.direction = direction;
            parts.safety.unwrapped_position = fits(&parts);
            assert!(matches!(parts.step(), StepResult::ReadyNow));
            assert!(matches!(
                parts.state,
                RobustnessReversalState::EstablishRamp { .. }
            ));
            assert_eq!(parts.session.held_route, None);

            let mut parts = RunParts::new(test_point());
            assert!(matches!(parts.step(), StepResult::ReadyNow));
            parts.session.direction = direction;
            parts.safety.unwrapped_position = if direction == 0 {
                fits(&parts) + 1
            } else {
                fits(&parts) - 1
            };
            assert!(matches!(parts.step(), StepResult::ReadyNow));
            assert_eq!(parts.state, zero_ramp_state(&parts.session, 0));
            assert_eq!(parts.session.held_route, Some(HeldRoute::ReenterEstablish));
        }
    }

    #[test]
    fn precondition_failure_after_a_recovery_terminates_with_origin_cause() {
        let mut parts = RunParts::new(test_point());
        assert!(matches!(parts.step(), StepResult::ReadyNow));
        parts.session.origin_recovered = true;
        parts.safety.unwrapped_position =
            parts.safety.positive_position_window as i32 - leg_travel(&test_point()) + 1;

        let StepResult::RobustnessReversalTerminal(terminal) = parts.step() else {
            panic!("a refused leg right after a recovery must terminate");
        };

        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(nonzero_target_writes(&parts.backend), 0);
    }

    #[test]
    fn setup_rejects_a_schedule_that_cannot_fit_from_the_origin() {
        let mut parts = RunParts::new(test_point());
        parts.safety.positive_position_window = leg_travel(&test_point()) as u32;

        let StepResult::RobustnessReversalTerminal(terminal) = parts.step() else {
            panic!("an unplannable schedule must fail at setup");
        };

        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID);
        assert!(terminal.safety_fault.is_none());
        let target_addr = PID_VELOCITY_TARGET.addr();
        assert!(
            !parts
                .backend
                .spi_log
                .iter()
                .any(|op| matches!(op, SpiOp::Write(addr, _) if *addr == target_addr)),
            "setup rejection happens before any target write"
        );
    }

    #[test]
    fn origin_recovery_starts_from_drift_beyond_the_smaller_absolute_window() {
        let mut parts = RunParts::new(recovery_point());
        assert!(matches!(parts.step(), StepResult::ReadyNow));
        let leg = leg_travel(&recovery_point());
        let band = parts
            .session
            .recovery_plan
            .expect("setup plans the recovery")
            .origin_band_counts as i32;
        parts.session.direction = 1;
        parts.safety.negative_position_window = (2 * leg) as u32;
        parts.safety.positive_position_window = (leg + band) as u32;
        parts.safety.position_window = leg as u32;
        parts.safety.unwrapped_position = -(leg + 1);

        let (terminal, parts) = drive_parts(parts, Envelope::Tracked, |_phase, _tick, target| {
            if target == 0 {
                0
            } else {
                target.signum() * STEADY_DELTA
            }
        });

        assert!(parts.recoveries >= 1);
        assert_eq!(
            parts.targets[..2],
            [176, 0],
            "the recovery's return leg runs before any establish leg"
        );
        assert_ne!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_ne!(terminal.outcome, RobustnessOutcome::Failed);
    }

    #[test]
    fn exhausted_establish_unwinds_to_origin_before_its_retry() {
        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            drift_free_parts(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(
            &parts.targets[..11],
            &[60, 120, 150, 90, 30, 0, -176, 0, 60, 120, 150]
        );
        assert_eq!(parts.recoveries, 1);
        assert_eq!(terminal.evidence.directions[0].retry_count, 1);
        assert!(terminal.safety_fault.is_none());
        assert_ne!(terminal.outcome, RobustnessOutcome::Failed);
    }

    #[test]
    fn rejected_rest_after_exhausted_establish_proceeds_to_origin_recovery() {
        let mut establish_legs = 0u32;
        let mut zero_windows = 0u32;
        let (terminal, parts) = drive_parts(
            drift_free_parts(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                if phase == CapturePhase::Zero && tick == 0 {
                    zero_windows += 1;
                }
                match phase {
                    CapturePhase::Zero if zero_windows <= 2 => 8,
                    _ if target == 0 => 0,
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(
            &parts.targets[..11],
            &[60, 120, 150, 90, 30, 0, -176, 0, 60, 120, 150]
        );
        assert_eq!(parts.recoveries, 1);
        assert_eq!(terminal.evidence.directions[0].retry_count, 1);
        assert!(terminal.safety_fault.is_none());
        assert_ne!(terminal.outcome, RobustnessOutcome::Failed);
        assert_ne!(terminal.cause, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED);
    }

    #[test]
    fn rejected_rest_after_inconclusive_reverse_leg_proceeds_to_origin_recovery() {
        let mut reverse_legs = 0u32;
        let mut zero_windows = 0u32;
        let (terminal, parts) = drive_parts(
            drift_free_parts(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Reverse && tick == 0 {
                    reverse_legs += 1;
                }
                if phase == CapturePhase::Zero && tick == 0 {
                    zero_windows += 1;
                }
                match phase {
                    CapturePhase::Zero if zero_windows <= 2 => 8,
                    _ if target == 0 => 0,
                    CapturePhase::Reverse if reverse_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(
            &parts.targets[..11],
            &[60, 120, 150, 90, 30, -30, -90, -150, -90, -30, 0]
        );
        assert_eq!(
            parts.targets[11].abs(),
            176,
            "the recovery commands a return leg at the campaign amplitude"
        );
        assert_eq!(parts.targets[12], 0);
        assert_eq!(&parts.targets[13..16], &[60, 120, 150]);
        assert_eq!(parts.recoveries, 1);
        assert_eq!(terminal.evidence.directions[0].retry_count, 1);
        assert!(terminal.safety_fault.is_none());
        assert_ne!(terminal.cause, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED);
    }

    #[test]
    fn rejected_rest_then_unconfirmed_recovery_rest_terminates_with_origin_cause() {
        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            RunParts::new(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                match phase {
                    CapturePhase::Zero => 8,
                    CapturePhase::Recover if target == 0 => 8,
                    _ if target == 0 => 0,
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.recoveries, 1);
        assert_eq!(parts.targets.last(), Some(&0));
        assert_eq!(&parts.targets[..7], &[60, 120, 150, 90, 30, 0, -176]);
    }

    #[test]
    fn rejected_rest_with_unbuildable_recovery_faults_without_motion() {
        let mut establish_legs = 0u32;
        let mut parts = RunParts::new(recovery_point());
        assert!(matches!(parts.step(), StepResult::ReadyNow));
        assert_eq!(parts.state, RobustnessReversalState::EstablishWriteGain);
        parts.session.recovery_plan = None;
        let (terminal, parts) =
            drive_parts(parts, Envelope::Tracked, move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                match phase {
                    CapturePhase::Zero => 8,
                    _ if target == 0 => 0,
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.recoveries, 0);
        assert_eq!(parts.targets.as_slice(), &[60, 120, 150, 90, 30, 0]);
    }

    #[test]
    fn slot_consuming_inconclusive_unwinds_then_routes() {
        let mut jitter_high = false;
        let (terminal, parts) = drive_parts(
            RunParts::new(test_point()),
            Envelope::Untracked,
            move |phase, _tick, target| {
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Reverse => {
                        jitter_high = !jitter_high;
                        if jitter_high { 300 } else { -300 }
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Inconclusive);
        assert_eq!(
            parts.recoveries,
            u32::try_from(
                (ROBUSTNESS_CYCLES_PER_DIRECTION + usize::from(ROBUSTNESS_MAX_RETRIES))
                    * DIRECTION_COUNT
            )
            .expect("recovery count fits u32"),
            "one recovery per inconclusive leg, retries and slot-consumes alike"
        );
        assert!(parts.session.held_route.is_none());
        for direction in terminal.evidence.directions {
            assert_eq!(direction.retry_count, ROBUSTNESS_MAX_RETRIES);
            assert_eq!(direction.valid_cycles, 0);
        }
    }

    #[test]
    fn precondition_failure_then_centered_recovery_terminates_after_one_recovery() {
        let mut parts = RunParts::new(test_point());
        assert!(matches!(parts.step(), StepResult::ReadyNow));
        parts.safety.positive_position_window = leg_travel(&test_point()) as u32 - 1;

        let (terminal, parts) = drive_parts(parts, Envelope::Untracked, |_, _, _| 0);

        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.recoveries, 1);
        assert!(
            parts.targets.is_empty(),
            "the rest from rest is a zero-span ramp and commands nothing"
        );
    }

    #[test]
    fn contained_recovery_with_headroom_continues() {
        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            drift_free_parts(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                match phase {
                    CapturePhase::Recover => 0,
                    _ if target == 0 => 0,
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(
            &parts.targets[..11],
            &[60, 120, 150, 90, 30, 0, -176, 0, 60, 120, 150]
        );
        assert_eq!(parts.recoveries, 1);
        assert_eq!(terminal.evidence.directions[0].retry_count, 1);
        assert!(terminal.safety_fault.is_none());
        assert_ne!(terminal.outcome, RobustnessOutcome::Failed);
    }

    #[test]
    fn contained_recovery_without_headroom_terminates_with_origin_cause() {
        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            RunParts::new(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                match phase {
                    CapturePhase::Recover => 0,
                    _ if target == 0 => 0,
                    CapturePhase::Establish if establish_legs == 1 => target.signum() * 22,
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.recoveries, 1);
        assert_eq!(
            parts.targets.as_slice(),
            &[60, 120, 150, 90, 30, 0, -176, 0]
        );
    }

    #[test]
    fn current_clamp_contained_recovery_terminates_with_origin_cause() {
        let mut establish_legs = 0u32;
        let mut parts = RunParts::new(recovery_point());
        let _ = parts.backend.registers.insert(
            foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
                .data
                .addr(),
            2_448,
        );
        let (terminal, parts) =
            drive_parts(parts, Envelope::Tracked, move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.recoveries, 1);
        assert_eq!(
            parts.targets.as_slice(),
            &[60, 120, 150, 90, 30, 0, -176, 0]
        );
    }

    #[test]
    fn unconfirmed_recovery_rest_terminates_with_origin_cause() {
        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            RunParts::new(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                match phase {
                    CapturePhase::Recover if target == 0 => -1,
                    CapturePhase::Recover => target.signum() * STEADY_DELTA,
                    _ if target == 0 => 0,
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.recoveries, 1);
    }

    #[test]
    fn recovery_safety_fault_reports_envelope_cause() {
        use crate::commission_dispatch::robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;

        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            RunParts::new(recovery_point()),
            Envelope::Tracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Establish && tick == 0 {
                    establish_legs += 1;
                }
                match phase {
                    CapturePhase::Recover if target != 0 => -target.signum() * STEADY_DELTA,
                    _ if target == 0 => 0,
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_SAFETY_FAULT);
        assert_eq!(
            terminal.safety_fault.map(|fault| fault.reason),
            Some(OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY)
        );
        assert_eq!(parts.recoveries, 1);
    }

    #[test]
    fn accepted_gains_rejects_negative_selected_p_or_i() {
        let mut point = test_point();
        point.selected_p = 0x8000;
        let error = accepted_gains(&point).unwrap_err();
        assert_eq!(error, CommissionError::PidGainInvalid);

        let mut point = test_point();
        point.selected_i = 0x8000;
        let error = accepted_gains(&point).unwrap_err();
        assert_eq!(error, CommissionError::PidGainInvalid);
    }

    #[test]
    fn build_origin_recovery_rejects_negative_selected_p() {
        let mut session =
            RobustnessReversalSession::new(test_point(), 0, COUNTS_PER_REV, test_rate());
        session.recovery_plan = Some(
            plan_rung_origin_recovery(test_safety().budget, test_safety().counts_per_rev(), 1_000)
                .unwrap(),
        );
        session.point.selected_p = 0x8000;
        let safety = test_safety();
        let error = build_origin_recovery(&session, &safety).unwrap_err();
        assert_eq!(error, CommissionError::PidGainInvalid);
    }

    #[test]
    fn planned_velocity_mrev_s_rejects_overflowing_conversion() {
        let mut point = test_point();
        point.target_velocity_rpm = i32::MAX;
        let error = planned_velocity_mrev_s(&point).unwrap_err();
        assert_eq!(error, CommissionError::TargetRateInvalid);
    }

    fn point_at_180() -> AcceptancePoint {
        AcceptancePoint {
            target_velocity_rpm: 180,
            ..recovery_point()
        }
    }

    fn steady_motion(_phase: CapturePhase, _tick: u32, target: i32) -> i32 {
        target.signum() * STEADY_DELTA
    }

    #[test]
    fn establish_leg_ramps_from_rest_and_reversal_ramps_through_zero() {
        let (terminal, parts) = drive_parts(
            RunParts::new(point_at_180()),
            Envelope::Untracked,
            steady_motion,
        );

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(
            &parts.targets[..12],
            &[60, 120, 153, 93, 33, -27, -87, -147, -153, -93, -33, 0]
        );
        assert!(
            parts
                .targets
                .windows(2)
                .all(|pair| pair[0].abs_diff(pair[1]) <= 60),
            "no two consecutive targets are further apart than one tick's rate"
        );
    }

    #[test]
    fn reversal_ramp_starts_from_the_last_written_target_after_a_rest_window() {
        let (_terminal, parts) = drive_parts(
            RunParts::new(point_at_180()),
            Envelope::Untracked,
            steady_motion,
        );
        let cycle = 12;
        let direction_one = cycle * ROBUSTNESS_CYCLES_PER_DIRECTION;

        assert_eq!(&parts.targets[cycle..cycle + 3], &[60, 120, 153]);
        assert_eq!(
            &parts.targets[direction_one..direction_one + 9],
            &[-60, -120, -153, -93, -33, 27, 87, 147, 153]
        );
    }

    #[test]
    fn reconvergence_clock_and_iae_start_at_the_reversal_ramp_end() {
        const STALL_MS: u32 = 100;
        let (terminal, _backend, session) =
            drive_scripted(test_point(), |phase, tick, target| match phase {
                CapturePhase::Reverse if tick < STALL_MS => 0,
                CapturePhase::ReverseRamp => 0,
                _ => target.signum() * STEADY_DELTA,
            });

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        let expected_ms = STALL_MS as u16 + PROBE_BAND_WINDOW_SAMPLES as u16 - 1;
        let stalled_iae_q = i64::from(STALL_MS)
            * 1_000
            * target_rate_q(leg_amplitude_rpm(&test_point()), COUNTS_PER_REV);
        for direction in 0..DIRECTION_COUNT {
            for cycle in terminal.evidence.cycles[direction] {
                assert_eq!(cycle.expect("filled cycle").reconvergence_ms, expected_ms);
            }
            for metrics in session.ledgers[direction].cycles() {
                assert_eq!(metrics.expect("filled cycle").recovery_iae_q, stalled_iae_q);
            }
        }
    }

    #[test]
    fn reversal_peak_excludes_the_ramp_phase() {
        let mut cruise_sign = 0;
        let (terminal, _parts) = drive_tracked(move |phase, tick, target| match phase {
            CapturePhase::Establish => {
                cruise_sign = target.signum();
                target.signum() * STEADY_DELTA
            }
            CapturePhase::ReverseRamp => cruise_sign * STEADY_DELTA * 3 / 2,
            CapturePhase::Reverse if tick < 5 => target.signum() * STEADY_DELTA * 6 / 5,
            _ => target.signum() * STEADY_DELTA,
        });

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        for direction in terminal.evidence.directions {
            assert_eq!(direction.peak_rate_permille, 1200);
        }
    }

    struct DispatchedRun {
        phase: crate::commissioning::outer::OuterPhase,
        buf: crate::autotune::TransientBuffer,
        backend: MockBackend,
        position_counts: f64,
    }

    impl DispatchedRun {
        fn new(point: AcceptancePoint) -> Self {
            let mut safety = test_safety();
            safety
                .configure_velocity_envelope(planned_velocity_mrev_s(&point).expect("planned"))
                .expect("envelope for the point");
            Self {
                phase: crate::commissioning::outer::OuterPhase::RobustnessReversal {
                    state: RobustnessReversalState::Setup,
                    workspace: RobustnessReversalWorkspace::new(),
                    session: RobustnessReversalSession::new(point, 7, COUNTS_PER_REV, test_rate()),
                    safety,
                },
                buf: crate::autotune::TransientBuffer::new(),
                backend: test_backend(),
                position_counts: 0.0,
            }
        }

        fn written_target(&self) -> i32 {
            self.backend
                .registers
                .get(&PID_VELOCITY_TARGET.addr())
                .map_or(0, |value| *value as i32)
        }

        fn target_writes(&self) -> std::vec::Vec<i32> {
            self.backend
                .spi_log
                .iter()
                .filter_map(|operation| match operation {
                    SpiOp::Write(address, value) if *address == PID_VELOCITY_TARGET.addr() => {
                        Some(*value as i32)
                    }
                    _ => None,
                })
                .collect()
        }

        /// Steps through the outer dispatcher; on `At` the plant moves at
        /// `counts_per_ms(written target)` until the deadline.
        fn run_to_terminal(
            &mut self,
            mut counts_per_ms: impl FnMut(i32) -> f64,
        ) -> RobustnessReversalTerminal {
            for _ in 0..STEP_BUDGET {
                let mut trace_summary_seqs = crate::commissioning::outer::TraceSummarySeqs::new();
                let result =
                    pollster::block_on(crate::commissioning::outer::step_outer_with_trace(
                        &mut self.phase,
                        &mut self.buf,
                        &mut trace_summary_seqs,
                        &mut self.backend,
                    ));
                match result {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => {
                        let rate = counts_per_ms(self.written_target());
                        let elapsed_ms = deadline_us.saturating_sub(self.backend.elapsed_us_value)
                            as f64
                            / 1_000.0;
                        self.position_counts += rate * elapsed_ms;
                        self.backend.elapsed_us_value =
                            self.backend.elapsed_us_value.max(deadline_us);
                        let raw = (libm::floor(self.position_counts) as i32).rem_euclid(4_000);
                        self.backend.mechanical_encoder_position_counts_value = raw as u32;
                        self.backend.set_abn_decoder_count(raw);
                    }
                    StepResult::RobustnessReversalTerminal(terminal) => return terminal,
                    other => panic!("unexpected step result: {other:?}"),
                }
            }
            panic!("the dispatched robustness run did not end");
        }
    }

    fn tracking_counts_per_ms(target_rpm: i32) -> f64 {
        f64::from(target_rpm) * f64::from(COUNTS_PER_REV) / 60_000.0
    }

    fn run_dispatched_fault(trigger_write: i32) -> (DispatchedRun, RobustnessReversalTerminal) {
        let mut run = DispatchedRun::new(point_at_180());
        let mut jumped = false;
        let terminal = run.run_to_terminal(|target| {
            if target == trigger_write && !jumped {
                jumped = true;
                900.0
            } else {
                tracking_counts_per_ms(target)
            }
        });
        (run, terminal)
    }

    fn assert_velocity_fault_terminal(terminal: &RobustnessReversalTerminal) {
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(
            terminal.cause,
            crate::commission_dispatch::robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT
        );
        assert_eq!(
            terminal.safety_fault.map(|fault| fault.reason),
            Some(crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY)
        );
    }

    #[test]
    fn a_fault_mid_ramp_takes_the_fault_terminal_with_an_immediate_zero() {
        let (run, terminal) = run_dispatched_fault(120);

        assert_eq!(run.target_writes(), [60, 120, 0]);
        assert_velocity_fault_terminal(&terminal);
    }

    #[test]
    fn an_encoder_velocity_fault_during_the_reversal_ramp_is_caught_by_the_safety_sample() {
        let (run, terminal) = run_dispatched_fault(33);

        assert_eq!(run.target_writes(), [60, 120, 153, 93, 33, 0]);
        assert_velocity_fault_terminal(&terminal);
    }

    #[test]
    fn an_encoder_velocity_fault_during_the_zero_ramp_is_caught_by_the_safety_sample() {
        let (run, terminal) = run_dispatched_fault(-93);

        assert_eq!(
            run.target_writes(),
            [60, 120, 153, 93, 33, -27, -87, -147, -153, -93, 0]
        );
        assert_velocity_fault_terminal(&terminal);
    }

    fn step_parts_until(parts: &mut RunParts, done: impl Fn(&RobustnessReversalState) -> bool) {
        let mut encoder_count = 0;
        for _ in 0..STEP_BUDGET {
            if done(&parts.state) {
                return;
            }
            if let StepResult::At(deadline) = parts.step() {
                parts.backend.elapsed_us_value = parts.backend.elapsed_us_value.max(deadline);
                let target = *parts
                    .backend
                    .registers
                    .get(&PID_VELOCITY_TARGET.addr())
                    .unwrap_or(&0) as i32;
                encoder_count += target.signum() * STEADY_DELTA;
                parts
                    .backend
                    .set_abn_decoder_count(encoder_count.rem_euclid(4_000));
            }
        }
        panic!("the awaited state was never reached");
    }

    #[test]
    fn the_settle_phase_is_entered_at_the_zero_ramp_end() {
        let mut parts = RunParts::new(point_at_180());
        step_parts_until(&mut parts, |state| {
            matches!(state, RobustnessReversalState::ZeroRamp { .. })
        });
        step_parts_until(&mut parts, |state| {
            matches!(
                state,
                RobustnessReversalState::ZeroRamp { ramp, .. } if ramp.next_is_final()
            )
        });
        parts.harvest_targets();
        assert_eq!(parts.targets.last(), Some(&-33));
        assert!(!parts.safety.in_settle_phase());

        step_parts_until(&mut parts, |state| {
            matches!(state, RobustnessReversalState::ZeroCapture { .. })
        });

        parts.harvest_targets();
        assert_eq!(parts.targets.last(), Some(&0));
        assert!(parts.safety.in_settle_phase());
    }

    #[test]
    fn the_capture_budget_starts_at_the_post_ramp_baseline_read() {
        const SPI_STEP_US: u64 = 7;
        let mut parts = RunParts::new(point_at_180());
        parts.backend.spi_elapsed_step_us = SPI_STEP_US;
        step_parts_until(&mut parts, |state| {
            matches!(state, RobustnessReversalState::ReverseCapture { .. })
        });
        let post_write_read_us = parts.workspace.previous_read_us;

        let RobustnessReversalState::ReverseCapture {
            next_sample_us,
            budget_deadline_us,
            ..
        } = parts.state
        else {
            unreachable!()
        };

        parts.harvest_targets();
        assert_eq!(parts.targets.last(), Some(&-153));
        let target_addr = PID_VELOCITY_TARGET.addr();
        let last_target_write = parts
            .backend
            .spi_log
            .iter()
            .rposition(|op| matches!(op, SpiOp::Write(addr, _) if *addr == target_addr))
            .expect("the ramp wrote its target");
        assert_eq!(
            &parts.backend.spi_log[last_target_write + 1..],
            &[SpiOp::Read],
            "the capture baseline is read after the ramp-end write"
        );
        assert_eq!(next_sample_us, post_write_read_us + CAPTURE_PERIOD_US);
        assert_eq!(
            budget_deadline_us,
            post_write_read_us + u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US)
        );
        assert_eq!(parts.workspace.observations.observation_len(), 0);
    }

    #[test]
    fn the_first_scored_observation_starts_after_the_ramp_end_write() {
        const SPI_STEP_US: u64 = 7;
        const MOVED_DURING_WRITE: u32 = 5;
        let mut parts = RunParts::new(point_at_180());
        parts.backend.spi_elapsed_step_us = SPI_STEP_US;
        step_parts_until(&mut parts, |state| {
            matches!(
                state,
                RobustnessReversalState::ReverseRamp { ramp, .. }
                    if ramp.is_started() && ramp.next_is_final()
            )
        });
        let before_write = (parts.workspace.previous_encoder_raw + 4_000 - 10) % 4_000;
        let after_write = (before_write + 4_000 - MOVED_DURING_WRITE) % 4_000;
        let first_sample = (after_write + 4_000 - 10) % 4_000;
        parts
            .backend
            .set_abn_decoder_count_sequence(&[before_write, after_write, first_sample]);

        let StepResult::At(next_sample_us) = parts.step() else {
            panic!("the ramp-end tick schedules the first capture sample");
        };
        assert!(matches!(
            parts.state,
            RobustnessReversalState::ReverseCapture { .. }
        ));
        let baseline_read_us = parts.workspace.previous_read_us;
        parts.backend.elapsed_us_value = next_sample_us;
        let _ = parts.step();

        let observations: std::vec::Vec<_> = parts.workspace.observations.iter().collect();
        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].delta_counts, -10);
        assert_eq!(
            u64::from(observations[0].elapsed_us),
            next_sample_us + SPI_STEP_US - baseline_read_us
        );
        assert_eq!(next_sample_us, baseline_read_us + CAPTURE_PERIOD_US);
    }

    #[test]
    fn a_failed_baseline_read_after_the_ramp_end_write_takes_the_io_terminal() {
        let mut parts = RunParts::new(point_at_180());
        step_parts_until(&mut parts, |state| {
            matches!(
                state,
                RobustnessReversalState::ReverseRamp { ramp, .. }
                    if ramp.is_started() && ramp.next_is_final()
            )
        });
        parts.backend.fail_capture_read_at = Some(parts.backend.capture_register_reads + 2);

        let StepResult::RobustnessReversalTerminal(terminal) = parts.step() else {
            panic!("a failed baseline read must end the run");
        };

        parts.harvest_targets();
        assert_eq!(parts.targets.last(), Some(&-153));
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_IO);
        assert!(terminal.safety_fault.is_none());
    }

    #[test]
    fn the_establish_primitive_starts_at_the_first_ramp_write() {
        const SPI_STEP_US: u64 = 7;
        let mut parts = RunParts::new(point_at_180());
        parts.backend.spi_elapsed_step_us = SPI_STEP_US;
        step_parts_until(&mut parts, |state| {
            matches!(
                state,
                RobustnessReversalState::EstablishRamp { ramp, .. } if ramp.is_started()
            )
        });
        let baseline_read_us = parts.workspace.previous_read_us;

        parts.harvest_targets();
        assert_eq!(parts.targets.as_slice(), &[60]);
        assert_eq!(
            parts.safety.primitive_start_us,
            Some(baseline_read_us + SPI_STEP_US)
        );
    }

    #[test]
    fn cycle_fit_charges_the_reversal_ramp_at_leg_amplitude() {
        assert_eq!(
            establish_leg_travel_counts(12, 6_000),
            establish_leg_travel_counts(12, 0) + 12 * 6
        );
    }

    #[test]
    fn robustness_bound_constants_are_unchanged() {
        assert_eq!(REVERSAL_PEAK_REJECT_PERMILLE, 1714);
        assert_eq!(ROBUSTNESS_LEG_AMPLITUDE_PCT, 85);
        assert_eq!(ROBUSTNESS_THRESHOLDS.t_max_us, 600_000);
    }

    #[test]
    fn the_cycle_ramp_charges_every_leg_tick_rounded() {
        let rate = test_rate();
        assert_eq!(robustness_cycle_ramp_us(180, rate), 12_000);
        let slow = RampRate::from_milli_rpm_per_ms(100_000).expect("nonzero rate");
        assert_eq!(robustness_cycle_ramp_us(176, slow), 7_000);
    }

    #[test]
    fn the_primitive_clock_restarts_at_the_reverse_and_zero_ramp_first_writes() {
        const SPI_STEP_US: u64 = 7;
        let mut parts = RunParts::new(point_at_180());
        parts.backend.spi_elapsed_step_us = SPI_STEP_US;
        let mut driver = TickDriver::new(parts, 0);
        let mut on_tick = steady_tick;
        let ended = driver.run_until(&mut on_tick, |state| {
            matches!(state, RobustnessReversalState::ReverseRamp { ramp, .. } if ramp.is_started())
        });
        assert!(ended.is_none());
        let opened_position = driver.parts.safety.primitive_start_position;
        assert_eq!(
            driver.parts.safety.primitive_start_us,
            Some(driver.parts.workspace.previous_read_us + SPI_STEP_US),
            "the reverse ramp's first write restarts the clock"
        );

        let ended = driver.run_until(&mut on_tick, |state| {
            matches!(state, RobustnessReversalState::ZeroRamp { ramp, .. } if ramp.is_started())
        });

        assert!(ended.is_none());
        let RobustnessReversalState::ZeroRamp { next_tick_us, .. } = driver.parts.state else {
            unreachable!()
        };
        assert_eq!(
            driver.parts.safety.primitive_start_us,
            Some(next_tick_us - CAPTURE_PERIOD_US),
            "the zero ramp's first write restarts the clock"
        );
        assert_eq!(
            driver.parts.safety.primitive_start_position,
            opened_position
        );
    }

    struct TickContext {
        phase: CapturePhase,
        leg_tick: u32,
        target: i32,
        index: usize,
    }

    /// Steps a tracked run one tick at a time, running the outer safety sample after every
    /// scheduled tick and, on entry to each analysis state, `analysis_us` of further 1 ms samples
    /// while the plant keeps moving at its commanded sign.
    struct TickDriver {
        parts: RunParts,
        tracked: bool,
        analysis_us: u64,
        encoder_count: i32,
        previous_phase: Option<CapturePhase>,
        previous_state: RobustnessReversalState,
        leg_tick: u32,
        ticks: usize,
    }

    fn is_analysis(state: &RobustnessReversalState) -> bool {
        matches!(
            state,
            RobustnessReversalState::EstablishAnalyze { .. }
                | RobustnessReversalState::ReverseAnalyze { .. }
                | RobustnessReversalState::ZeroAnalyze { .. }
        )
    }

    impl TickDriver {
        fn new(parts: RunParts, analysis_us: u64) -> Self {
            let previous_state = parts.state;
            Self {
                parts,
                tracked: true,
                analysis_us,
                encoder_count: 0,
                previous_phase: None,
                previous_state,
                leg_tick: 0,
                ticks: 0,
            }
        }

        fn written_target(&self) -> i32 {
            *self
                .parts
                .backend
                .registers
                .get(&PID_VELOCITY_TARGET.addr())
                .unwrap_or(&0) as i32
        }

        fn sample(&mut self) -> Option<RobustnessReversalTerminal> {
            let raw = self.encoder_count.rem_euclid(4_000);
            self.parts.backend.set_abn_decoder_count(raw);
            self.parts.backend.mechanical_encoder_position_counts_value = raw as u32;
            if !self.tracked {
                return None;
            }
            match pollster::block_on(check_outer_safety(
                &mut self.parts.safety,
                &mut self.parts.backend,
            )) {
                Ok(None) => None,
                Ok(Some(fault)) => {
                    let StepResult::RobustnessReversalTerminal(terminal) = fault_terminal(
                        &self.parts.session,
                        &self.parts.backend,
                        Some(fault),
                        ROBUSTNESS_CAUSE_SAFETY_FAULT,
                        COUNTS_PER_REV,
                    ) else {
                        unreachable!("fault_terminal builds a robustness terminal")
                    };
                    Some(terminal)
                }
                Err(error) => panic!("outer safety sample failed: {error:?}"),
            }
        }

        fn run_analysis_time(&mut self) -> Option<RobustnessReversalTerminal> {
            for _ in 0..self.analysis_us / 1_000 {
                self.parts.backend.elapsed_us_value += 1_000;
                self.encoder_count += self.written_target().signum() * STEADY_DELTA;
                if let Some(terminal) = self.sample() {
                    return Some(terminal);
                }
            }
            None
        }

        /// Runs until the terminal, or `None` once `stop` holds after a step.
        fn run_until(
            &mut self,
            on_tick: &mut impl FnMut(&mut RunParts, TickContext) -> i32,
            stop: impl Fn(&RobustnessReversalState) -> bool,
        ) -> Option<RobustnessReversalTerminal> {
            for _ in 0..STEP_BUDGET {
                let result = self.parts.step();
                self.parts.harvest_targets();
                self.parts.backend.spi_log.clear();
                self.parts.log_cursor = 0;
                let state = self.parts.state;
                let recovering = matches!(state, RobustnessReversalState::RecoverOrigin(_));
                if recovering
                    && !matches!(
                        self.previous_state,
                        RobustnessReversalState::RecoverOrigin(_)
                    )
                {
                    self.parts.recoveries += 1;
                }
                let entered_analysis = is_analysis(&state)
                    && core::mem::discriminant(&state)
                        != core::mem::discriminant(&self.previous_state);
                self.previous_state = state;
                let phase = capture_phase(&state);
                if phase.is_some() && phase != self.previous_phase {
                    self.leg_tick = 0;
                }
                self.previous_phase = phase;
                match result {
                    StepResult::ReadyNow => {
                        if entered_analysis && let Some(terminal) = self.run_analysis_time() {
                            return Some(terminal);
                        }
                    }
                    StepResult::At(deadline) => {
                        self.parts.backend.elapsed_us_value = deadline;
                        let context = TickContext {
                            phase: phase.expect("At is scheduled only from a capture state"),
                            leg_tick: self.leg_tick,
                            target: self.written_target(),
                            index: self.ticks,
                        };
                        self.encoder_count += on_tick(&mut self.parts, context);
                        self.ticks += 1;
                        self.leg_tick += 1;
                        if let Some(terminal) = self.sample() {
                            return Some(terminal);
                        }
                    }
                    StepResult::RobustnessReversalTerminal(terminal) => return Some(terminal),
                    other => panic!("unexpected step result: {other:?}"),
                }
                if stop(&self.parts.state) {
                    return None;
                }
            }
            panic!("robustness reversal run exceeded its bounded step budget");
        }

        fn run(
            &mut self,
            mut on_tick: impl FnMut(&mut RunParts, TickContext) -> i32,
        ) -> RobustnessReversalTerminal {
            self.run_until(&mut on_tick, |_| false)
                .expect("an unbounded run ends in a terminal")
        }

        /// Index of the tick on which the run ended.
        fn last_tick(&self) -> usize {
            self.ticks - 1
        }
    }

    fn steady_tick(_parts: &mut RunParts, context: TickContext) -> i32 {
        context.target.signum() * STEADY_DELTA
    }

    #[derive(Clone, Copy, Debug)]
    enum InjectedFault {
        VelocityCap,
        PositionWindow,
        ObservationGap,
        QuarterTurn,
    }

    impl InjectedFault {
        const ALL: [Self; 4] = [
            Self::VelocityCap,
            Self::PositionWindow,
            Self::ObservationGap,
            Self::QuarterTurn,
        ];

        fn reason(self) -> u8 {
            use crate::commissioning::types::{
                OUTER_SAFETY_FAULT_POSITION, OUTER_SAFETY_FAULT_VELOCITY,
            };
            match self {
                Self::VelocityCap => OUTER_SAFETY_FAULT_VELOCITY,
                Self::PositionWindow => OUTER_SAFETY_FAULT_POSITION,
                Self::ObservationGap => OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                Self::QuarterTurn => OUTER_SAFETY_FAULT_QUARTER_TURN,
            }
        }

        fn inject(self, parts: &mut RunParts, context: &TickContext) -> i32 {
            let steady = context.target.signum() * STEADY_DELTA;
            let quarter_turn = parts.safety.quarter_turn_fault_counts as i32;
            match self {
                Self::VelocityCap => quarter_turn - 100,
                Self::PositionWindow => {
                    let reached = parts.safety.unwrapped_position + steady;
                    let window = reached.unsigned_abs() - 1;
                    parts.safety.negative_position_window = window;
                    parts.safety.positive_position_window = window;
                    steady
                }
                Self::ObservationGap => {
                    parts.backend.elapsed_us_value += MAX_SAFETY_OBSERVATION_GAP_US;
                    steady
                }
                Self::QuarterTurn => quarter_turn,
            }
        }
    }

    /// A run whose position stays off zero, so the absolute window can always be closed in front
    /// of it.
    fn fault_run_parts() -> RunParts {
        let mut parts = RunParts::new(test_point());
        parts.safety.unwrapped_position = STEADY_DELTA / 2;
        parts
    }

    /// Ticks of the first cycle on which the reverse ramp and the zero ramp take their first
    /// safety sample after restarting the primitive clock.
    fn restart_ticks() -> [usize; 2] {
        let mut reverse_ramp = None;
        let mut zero_ramp = None;
        let mut driver = TickDriver::new(fault_run_parts(), 0);
        let _ = driver.run(|parts, context| {
            if context.leg_tick == 0 {
                match context.phase {
                    CapturePhase::ReverseRamp => {
                        reverse_ramp = reverse_ramp.or(Some(context.index))
                    }
                    CapturePhase::ZeroRamp => zero_ramp = zero_ramp.or(Some(context.index)),
                    _ => {}
                }
            }
            steady_tick(parts, context)
        });
        [
            reverse_ramp.expect("the run reaches a reverse ramp"),
            zero_ramp.expect("the run reaches a zero ramp"),
        ]
    }

    fn detection_latency(fault: InjectedFault, at_tick: usize) -> usize {
        let mut driver = TickDriver::new(fault_run_parts(), 0);
        let terminal = driver.run(|parts, context| {
            if context.index == at_tick {
                fault.inject(parts, &context)
            } else {
                steady_tick(parts, context)
            }
        });
        assert_eq!(
            terminal.safety_fault.map(|report| report.reason),
            Some(fault.reason()),
            "{fault:?} injected at tick {at_tick}"
        );
        driver.last_tick() - at_tick
    }

    #[test]
    fn faults_around_each_primitive_clock_restart_are_detected_on_the_same_tick() {
        let restarts = restart_ticks();
        let mid_reverse_capture = (restarts[0] + restarts[1]) / 2;
        for fault in InjectedFault::ALL {
            let unrestarted = detection_latency(fault, mid_reverse_capture);
            assert_eq!(unrestarted, 0, "{fault:?} away from any restart");
            for restart in restarts {
                for at_tick in [restart - 1, restart, restart + 1] {
                    assert_eq!(
                        detection_latency(fault, at_tick),
                        unrestarted,
                        "{fault:?} injected at tick {at_tick} around the restart at {restart}"
                    );
                }
            }
        }
    }

    /// Drives an untracked campaign whose establish and rest legs track their targets while each
    /// reverse capture tick delivers `reverse(direction, attempt, tick, steady)`, where `attempt`
    /// counts that direction's reverse legs from 1.
    fn drive_reverse_script(
        mut reverse: impl FnMut(usize, usize, u32, i32) -> i32,
    ) -> (RobustnessReversalTerminal, RobustnessReversalSession) {
        let mut attempt = [0usize; DIRECTION_COUNT];
        let (terminal, _backend, session) =
            drive_scripted(test_point(), move |phase, tick, target| match phase {
                CapturePhase::Reverse => {
                    let direction = usize::from(target > 0);
                    if tick == 0 {
                        attempt[direction] += 1;
                    }
                    reverse(
                        direction,
                        attempt[direction],
                        tick,
                        target.signum() * STEADY_DELTA,
                    )
                }
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });
        (terminal, session)
    }

    const LATE_WINDOW_TICKS: core::ops::Range<u32> = 600..700;
    const BUDGET_TICKS: u32 = ROBUSTNESS_MEASUREMENT_BUDGET_US / 1_000;

    /// Maximum over the 16-tick windows inside the late window of `|rate - target| / target` in
    /// ppm, computed directly from the scripted per-tick deltas at 1 ms per tick.
    fn expected_late_ppm(delta_at: impl Fn(u32) -> i32) -> u32 {
        let target = i64::from(STEADY_DELTA) * 16;
        (LATE_WINDOW_TICKS.start..=LATE_WINDOW_TICKS.end - 16)
            .map(|first| {
                let sum: i64 = (first..first + 16)
                    .map(|tick| i64::from(delta_at(tick).abs()))
                    .sum();
                u32::try_from((sum - target).unsigned_abs() * 1_000_000 / target.unsigned_abs())
                    .expect("ppm fits u32")
            })
            .max()
            .expect("the late window holds at least one window")
    }

    #[test]
    fn reverse_leg_observes_to_the_budget_deadline_after_band_entry() {
        let mut reverse_ticks = std::vec::Vec::new();
        let (terminal, _session) = drive_reverse_script(|_, _, tick, steady| {
            if tick == 0 {
                reverse_ticks.push(0u32);
            }
            *reverse_ticks.last_mut().expect("a leg started") += 1;
            steady
        });

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(
            reverse_ticks.len(),
            ROBUSTNESS_CYCLES_PER_DIRECTION * DIRECTION_COUNT
        );
        assert!(
            reverse_ticks.iter().all(|&ticks| ticks == BUDGET_TICKS),
            "every reverse leg samples to its budget deadline: {reverse_ticks:?}"
        );
    }

    #[test]
    fn a_reverse_leg_without_band_entry_by_the_deadline_is_inconclusive_and_retried() {
        let mut reverse_ticks = std::vec::Vec::new();
        let (terminal, parts) = drive_parts(
            RunParts::new(test_point()),
            Envelope::Untracked,
            |phase, tick, target| match phase {
                CapturePhase::Reverse => {
                    if tick == 0 {
                        reverse_ticks.push(0u32);
                    }
                    *reverse_ticks.last_mut().expect("a leg started") += 1;
                    if reverse_ticks.len() == 1 {
                        0
                    } else {
                        target.signum() * STEADY_DELTA
                    }
                }
                _ => target.signum() * STEADY_DELTA,
            },
        );

        assert_eq!(reverse_ticks[0], BUDGET_TICKS);
        assert_eq!(
            &parts.targets[..14],
            &[
                60, 120, 150, 90, 30, -30, -90, -150, -90, -30, 0, 60, 120, 150
            ],
            "the inconclusive leg ramps to rest and retries its cycle"
        );
        assert_eq!(
            parts.recoveries, 1,
            "the inconclusive leg unwinds before its retry"
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.evidence.directions[0].retry_count, 1);
        assert_eq!(terminal.evidence.directions[0].valid_cycles, 5);
    }

    fn late_drift(tick: u32, steady: i32) -> i32 {
        if tick < LATE_WINDOW_TICKS.start {
            steady
        } else {
            steady + steady.signum() * ((tick - LATE_WINDOW_TICKS.start) / 4) as i32
        }
    }

    #[test]
    fn a_suffix_exclusion_does_not_discard_late_tracking() {
        let mut attempt = 0usize;
        let mut on_tick = |_parts: &mut RunParts, context: TickContext| {
            let steady = context.target.signum() * STEADY_DELTA;
            match context.phase {
                CapturePhase::Reverse => {
                    if context.leg_tick == 0 {
                        attempt += 1;
                    }
                    if attempt == 1 {
                        late_drift(context.leg_tick, steady)
                    } else {
                        steady
                    }
                }
                _ => steady,
            }
        };
        let mut driver = TickDriver::new(RunParts::new(test_point()), 0);
        driver.tracked = false;
        let ended = driver.run_until(&mut on_tick, |state| {
            matches!(state, RobustnessReversalState::ReverseAnalyze { .. })
        });
        assert!(ended.is_none());
        let mut analysis = StationarityWork::new();
        let mut blocker = DyadicBlocker::<VELOCITY_SWEEP_BLOCK_LEVELS>::new();
        let stationarity = loop {
            match poll_stationarity(
                &mut analysis,
                &driver.parts.workspace.observations,
                &mut blocker,
            ) {
                Ok(None) => {}
                Ok(Some((result, _, _))) => break result,
                Err(error) => panic!("stationarity poll failed: {error:?}"),
            }
        };
        assert!(
            matches!(stationarity, StationarityResult::Excluded(_)),
            "the drifting suffix is excluded"
        );

        let terminal = driver
            .run_until(&mut on_tick, |_| false)
            .expect("the run ends in a terminal");

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.evidence.directions[0].retry_count, 0);
        let first = driver.parts.session.ledgers[0].cycles()[0].expect("the excluded leg is kept");
        assert_eq!(first.settled_residual_q, 0);
        let mut excluded = [[false; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT];
        excluded[0][0] = true;
        assert_eq!(terminal.evidence.late_suffix_excluded, excluded);
        assert_eq!(
            first.late_tracking.ppm,
            expected_late_ppm(|tick| late_drift(tick, STEADY_DELTA))
        );
    }

    #[test]
    fn a_reverse_leg_without_a_late_window_is_inconclusive() {
        let mut driver = TickDriver::new(RunParts::new(test_point()), 0);
        driver.tracked = false;
        let mut on_tick = steady_tick;
        let ended = driver.run_until(&mut on_tick, |state| {
            matches!(state, RobustnessReversalState::ReverseAnalyze { .. })
        });
        assert!(ended.is_none());
        assert!(driver.parts.workspace.capture.band_entry_us().is_some());
        let target_q = target_rate_q(reverse_target_rpm(&test_point(), 0), COUNTS_PER_REV);
        driver.parts.workspace.late_tracking = LateTrackingAccumulator::new(target_q);

        let ended = driver.run_until(&mut on_tick, |state| {
            !matches!(state, RobustnessReversalState::ReverseAnalyze { .. })
        });

        assert!(ended.is_none());
        assert!(matches!(
            driver.parts.state,
            RobustnessReversalState::ZeroRamp { .. }
        ));
        assert_eq!(
            driver.parts.session.ledgers[0].inconclusive_retries_used(),
            1
        );
        assert_eq!(driver.parts.session.ledgers[0].valid_cycles(), 0);
        assert!(driver.parts.session.pending_cycle.is_none());
    }

    #[test]
    fn establish_leg_still_ends_at_band_entry_plus_the_settle_tail() {
        let mut legs: std::vec::Vec<(Option<u32>, u32)> = std::vec::Vec::new();
        let mut driver = TickDriver::new(RunParts::new(test_point()), 0);
        driver.tracked = false;
        let terminal = driver.run(|parts, context| {
            if context.phase == CapturePhase::Establish {
                if context.leg_tick == 0 {
                    legs.push((None, 0));
                }
                let leg = legs.last_mut().expect("a leg started");
                if leg.0.is_none() && parts.workspace.capture.band_entry_us().is_some() {
                    leg.0 = Some(context.leg_tick);
                }
                leg.1 += 1;
            }
            steady_tick(parts, context)
        });

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(
            legs.len(),
            ROBUSTNESS_CYCLES_PER_DIRECTION * DIRECTION_COUNT
        );
        for (detected_after, ticks) in legs {
            let detected_after = detected_after.expect("the establish leg enters the band");
            assert_eq!(ticks, detected_after + ROBUSTNESS_SETTLE_TAIL_US / 1_000);
            assert!(ticks < BUDGET_TICKS);
        }
    }

    #[test]
    fn a_clean_campaign_completes_inside_a_3000_ms_primitive_duration_cap() {
        const WORST_ANALYSIS_US: u64 = 230_000;
        assert!(WORST_ANALYSIS_US < u64::from(SETTLE_ANALYSIS_BUDGET_US));
        let mut parts = RunParts::new(recovery_point());
        parts.safety.budget.max_duration_ms = 3_000;
        let mut zero_windows = 0u32;
        let mut driver = TickDriver::new(parts, WORST_ANALYSIS_US);

        let terminal = driver.run(|parts, context| match context.phase {
            CapturePhase::Zero => {
                if context.leg_tick == 0 {
                    zero_windows += 1;
                }
                if zero_windows % 2 == 1 { 2 } else { 0 }
            }
            _ => steady_tick(parts, context),
        });

        assert!(
            terminal.safety_fault.is_none(),
            "{:?}",
            terminal.safety_fault
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        assert!(
            zero_windows as usize >= 2 * ROBUSTNESS_CYCLES_PER_DIRECTION * DIRECTION_COUNT,
            "every between-cycles rest is retried once: {zero_windows} windows"
        );
    }

    #[test]
    fn repeated_origin_recovery_is_tolerated_without_wedging() {
        let mut driver = TickDriver::new(RunParts::new(recovery_point()), 0);

        let terminal = driver.run(steady_tick);

        assert!(
            terminal.safety_fault.is_none(),
            "{:?}",
            terminal.safety_fault
        );
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        assert!(
            driver.parts.recoveries >= 2,
            "reverse-side drift recovers the origin more than once: {}",
            driver.parts.recoveries
        );
        for direction in terminal.evidence.directions {
            assert_eq!(
                direction.valid_cycles,
                ROBUSTNESS_CYCLES_PER_DIRECTION as u8
            );
        }
    }

    fn late_tracking_ppms(
        session: &RobustnessReversalSession,
        direction: usize,
    ) -> std::vec::Vec<u32> {
        session.ledgers[direction]
            .cycles()
            .iter()
            .flatten()
            .map(|metrics| metrics.late_tracking.ppm)
            .collect()
    }

    /// One count over the target in every 16th late-window tick: each 16-tick window runs at
    /// 161/160 of the target, 6,250 ppm.
    fn one_count_in_sixteen(tick: u32, steady: i32) -> i32 {
        if LATE_WINDOW_TICKS.contains(&tick) && tick.is_multiple_of(16) {
            steady + steady.signum()
        } else {
            steady
        }
    }

    /// The late window delivered at nine tenths of the target, 100,000 ppm.
    fn nine_tenths_late(tick: u32, steady: i32) -> i32 {
        if LATE_WINDOW_TICKS.contains(&tick) {
            steady * 9 / 10
        } else {
            steady
        }
    }

    #[test]
    fn late_tracking_does_not_depend_on_when_the_band_is_entered() {
        let run = |stall_ticks: u32| {
            let (terminal, session) = drive_reverse_script(move |_, _, tick, steady| {
                if tick < stall_ticks {
                    0
                } else {
                    one_count_in_sixteen(tick, steady)
                }
            });
            assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
            [
                late_tracking_ppms(&session, 0),
                late_tracking_ppms(&session, 1),
            ]
        };

        let early = run(60);
        let late = run(200);

        let expected = expected_late_ppm(|tick| one_count_in_sixteen(tick, STEADY_DELTA));
        assert_eq!(expected, 6_250);
        for ppms in early.iter().chain(late.iter()) {
            assert_eq!(
                ppms.as_slice(),
                &[expected; ROBUSTNESS_CYCLES_PER_DIRECTION]
            );
        }
        assert_eq!(early, late);
    }

    #[test]
    fn a_script_tracking_inside_the_limit_in_the_late_window_passes_end_to_end() {
        let (terminal, session) =
            drive_reverse_script(|_, _, tick, steady| one_count_in_sixteen(tick, steady));

        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for ledger in &session.ledgers {
            assert_eq!(ledger.median_late_tracking_ppm(), 6_250);
        }
    }

    #[test]
    fn a_script_exceeding_the_limit_in_the_late_window_rejects_with_cause_24() {
        let (terminal, session) =
            drive_reverse_script(|_, _, tick, steady| nine_tenths_late(tick, steady));

        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_LATE_TRACKING_EXCEEDED);
        assert_eq!(terminal.cause, 24);
        for ledger in &session.ledgers {
            assert!(ledger.median_late_tracking_ppm() > 50_000);
            assert_eq!(ledger.median_late_tracking_ppm(), 100_000);
        }
        for direction in terminal.evidence.directions {
            assert_ne!(direction.trip_count, 0);
        }
    }

    #[test]
    fn two_over_limit_cycles_per_direction_still_pass_and_three_reject() {
        let run = |outliers: usize| {
            drive_reverse_script(move |_, attempt, tick, steady| {
                if attempt <= outliers {
                    nine_tenths_late(tick, steady)
                } else {
                    steady
                }
            })
        };

        let (two, two_session) = run(2);
        let (three, three_session) = run(3);

        assert_eq!(two.outcome, RobustnessOutcome::Complete);
        assert_eq!(two.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in 0..DIRECTION_COUNT {
            assert_eq!(
                late_tracking_ppms(&two_session, direction),
                [100_000, 100_000, 0, 0, 0]
            );
        }
        assert_eq!(three.outcome, RobustnessOutcome::Rejected);
        assert_eq!(three.cause, ROBUSTNESS_CAUSE_LATE_TRACKING_EXCEEDED);
        for ledger in &three_session.ledgers {
            assert_eq!(ledger.median_late_tracking_ppm(), 100_000);
        }
    }

    #[test]
    fn a_late_departure_after_band_entry_is_rejected_while_the_residual_suffix_looks_fine() {
        let departure = |tick: u32, steady: i32| {
            if (640..656).contains(&tick) {
                steady * 9 / 10
            } else {
                steady
            }
        };
        let (terminal, session) =
            drive_reverse_script(move |_, _, tick, steady| departure(tick, steady));

        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_LATE_TRACKING_EXCEEDED);
        let expected = expected_late_ppm(|tick| departure(tick, STEADY_DELTA));
        assert_eq!(expected, 100_000);
        for ledger in &session.ledgers {
            assert_eq!(ledger.median_late_tracking_ppm(), expected);
            assert!(
                i64::from(ledger.median_settled_residual_q()).abs()
                    <= ROBUSTNESS_THRESHOLDS.r_max_q,
                "the retired residual gate would not have caught it"
            );
        }
    }

    #[test]
    fn the_medians_and_target_rate_match_the_cycle_ledgers() {
        let (terminal, session) = drive_reverse_script(|_, attempt, tick, steady| {
            if attempt <= 2 {
                nine_tenths_late(tick, steady)
            } else {
                one_count_in_sixteen(tick, steady)
            }
        });

        let evidence = terminal.evidence;
        for direction in 0..DIRECTION_COUNT {
            let ledger = &session.ledgers[direction];
            assert_eq!(
                evidence.late_tracking_medians_ppm[direction],
                ledger.median_late_tracking_ppm()
            );
            assert_eq!(evidence.late_tracking_medians_ppm[direction], 6_250);
            assert_eq!(evidence.late_cycles[direction], ledger.late_cycles());
            let ppms: std::vec::Vec<u32> = evidence.late_cycles[direction]
                .iter()
                .map(|slot| slot.expect("every slot filled").ppm)
                .collect();
            assert_eq!(ppms, [100_000, 100_000, 6_250, 6_250, 6_250]);
            assert_eq!(
                evidence.late_target_rate_q[direction],
                target_rate_q(reverse_target_rpm(&test_point(), direction), COUNTS_PER_REV)
            );
        }
        assert_ne!(evidence.late_target_rate_q[0], 0);
        assert_eq!(
            evidence.late_target_rate_q[0],
            -evidence.late_target_rate_q[1]
        );
        assert_eq!(
            evidence.late_suffix_excluded,
            [[false; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT]
        );
    }
}
