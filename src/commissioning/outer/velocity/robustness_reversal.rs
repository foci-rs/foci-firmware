//! Cooperative unreset-reversal robustness phase.
//!
//! For each reversal direction, runs up to [`ROBUSTNESS_CYCLES_PER_DIRECTION`] probe
//! cycles: an establish leg (integrator cleared) then a measured reversal leg
//! (integrator not cleared). Metrics fold into a per-direction [`CycleLedger`]. The
//! accepted point is read-only; the phase never advances
//! [`FixedGainAmplitudeProgress`](super::integral_reproduction::FixedGainAmplitudeProgress).

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
use super::robustness_metrics::{
    PROBE_BAND_WINDOW_SAMPLES, ROBUSTNESS_BAND_DWELL_US, ROBUSTNESS_BAND_FRACTION_PCT,
    ROBUSTNESS_LEG_AMPLITUDE_PCT, ROBUSTNESS_MEASUREMENT_BUDGET_US, ReversalMetrics,
    ReversalOnlineAccumulator, RobustnessThresholds, dac_rms_q, reduce_plant_rate_q,
    robustness_thresholds_for, tripped_mask,
};
use super::robustness_reporting::CycleWire;
use super::sweep_analysis::poll_stationarity;
use super::sweep_analysis::{ScoredRestPoll, deficit_interval_q, poll_scored_rest_confirmation};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, SETTLE_ANALYSIS_BUDGET_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US,
};
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
    previous_encoder_raw: u32,
    previous_read_us: u64,
    current_observer: VelocityCurrentObserver,
    settle_displacement_counts: i8,
    settle_min_displacement_counts: i8,
    settle_max_displacement_counts: i8,
}

impl RobustnessReversalWorkspace {
    /// Construct an empty workspace.
    pub(crate) const fn new() -> Self {
        Self {
            observations: VelocityObservationRing::new(),
            blocker: DyadicBlocker::new(),
            analysis: StationarityWork::new(),
            capture: ReversalOnlineAccumulator::new(),
            previous_encoder_raw: 0,
            previous_read_us: 0,
            current_observer: VelocityCurrentObserver::EMPTY,
            settle_displacement_counts: 0,
            settle_min_displacement_counts: 0,
            settle_max_displacement_counts: 0,
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
    run_started_us: u64,
    direction: usize,
    ledgers: [CycleLedger; DIRECTION_COUNT],
    /// Running max overshoot in counts across each direction's cycles.
    peak_overshoot_counts: [u16; DIRECTION_COUNT],
    /// Per-cycle wire triples, mirroring each ledger's filled slots.
    cycle_wires: [[Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT],
    /// The grace-retried (displaced) tail cycle wire triple, per direction.
    displaced_tail_wires: [Option<CycleWire>; DIRECTION_COUNT],
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
    /// only; never mutates the candidate slot or its fixed-gain-amplitude progress.
    pub(crate) fn new(point: AcceptancePoint, run_sequence: u32, counts_per_rev: u32) -> Self {
        let (_, thresholds) = constructed_thresholds(&point, counts_per_rev);
        let iae_max_q = thresholds.iae_max_q;
        Self {
            point,
            run_sequence,
            run_started_us: 0,
            direction: 0,
            ledgers: [CycleLedger::new(iae_max_q), CycleLedger::new(iae_max_q)],
            peak_overshoot_counts: [0; DIRECTION_COUNT],
            cycle_wires: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT],
            displaced_tail_wires: [None; DIRECTION_COUNT],
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
    /// Command the signed establish-cruise target for the current direction.
    EstablishCommandTarget,
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
    /// Reverse the commanded target WITHOUT clearing the integrator.
    ReverseCommandTarget,
    /// Collect the measured reversal capture: search for band entry up to the
    /// budget, then sample a settled tail for the stationarity analysis.
    ReverseCapture {
        next_sample_us: u64,
        budget_deadline_us: u64,
        settle_deadline_us: Option<u64>,
    },
    /// Analyze the reversal capture for band-entry reconvergence.
    ReverseAnalyze {
        analysis_deadline_us: u64,
    },
    /// Command zero and open the between-legs rest window. `retry_available`
    /// tracks whether one rejected-rest re-open remains.
    ZeroCommand {
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
    let scaled = i64::from(amplitude_rpm(point)) * i64::from(ROBUSTNESS_LEG_AMPLITUDE_PCT) + 50;
    i32::try_from(scaled / 100).unwrap_or(i32::MAX)
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

const ESTABLISH_LEG_WINDOW_US: u32 =
    ROBUSTNESS_MEASUREMENT_BUDGET_US + ROBUSTNESS_SETTLE_TAIL_US + SETTLE_ANALYSIS_BUDGET_US;

fn establish_rate_counts_per_ms(point: &AcceptancePoint, counts_per_rev: u32) -> u32 {
    let counts_per_minute = u64::from(leg_amplitude_rpm(point).unsigned_abs())
        .saturating_mul(u64::from(counts_per_rev));
    u32::try_from(counts_per_minute.div_ceil(60_000)).unwrap_or(u32::MAX)
}

fn establish_leg_travel_counts(rate_counts_per_ms: u32) -> u32 {
    let window_ms = u64::from(ESTABLISH_LEG_WINDOW_US).div_ceil(1_000);
    u32::try_from(u64::from(rate_counts_per_ms).saturating_mul(window_ms)).unwrap_or(u32::MAX)
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
    rate_counts_per_ms: u32,
    unwrapped_position: i32,
    negative_window: u32,
    positive_window: u32,
    direction: usize,
) -> bool {
    let leg_travel = establish_leg_travel_counts(rate_counts_per_ms);
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
    suffix: &'a crate::autotune::velocity_measurement::StationarySuffix,
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
        suffix,
        ring,
    } = input;
    let signed_target_rpm = reverse_target_rpm(point, direction);
    let (settled_low, settled_high) = settled_rate_interval_q(suffix)?;
    let (deficit_low, deficit_high) =
        deficit_interval_q(signed_target_rpm, counts_per_rev, settled_low, settled_high)?;
    let settled_residual_q = deficit_low.saturating_add(deficit_high) / 2;
    let target_rate_q_value = target_rate_q(signed_target_rpm, counts_per_rev);
    let suffix_len = usize::from(suffix.suffix_len);
    let start_index = ring.observation_len().saturating_sub(suffix_len);
    let dac_rms = dac_rms_q(ring, start_index, suffix_len, target_rate_q_value);
    let mut metrics = ReversalMetrics {
        reconvergence_time_us,
        settled_residual_q,
        recovery_iae_q: recovery_iae,
        dac_rms_q: dac_rms,
        forward_settle_time_us,
        reconvergence_ratio_ppm: 0,
    };
    metrics.reconvergence_ratio_ppm =
        reconvergence_ratio_ppm(reconvergence_time_us, forward_settle_time_us);
    Ok(metrics)
}

async fn observe_tick<B: CommissioningBackend>(
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
    workspace
        .observations
        .push(signed, elapsed_us as u16)
        .map_err(|_| TickFault::Io)?;
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
            let leg_travel = establish_leg_travel_counts(establish_rate_counts_per_ms(
                &session.point,
                counts_per_rev,
            ));
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
            let rate = establish_rate_counts_per_ms(&session.point, counts_per_rev);
            if !cycle_fits(
                rate,
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
                    establish_leg_travel_counts(rate),
                    session.origin_recovered
                );
                if session.origin_recovered {
                    return origin_not_recovered_terminal(session, backend, counts_per_rev);
                }
                session.held_route = Some(HeldRoute::ReenterEstablish);
                *state = RobustnessReversalState::ZeroCommand {
                    retry_available: true,
                };
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
            *state = RobustnessReversalState::EstablishCommandTarget;
            StepResult::ReadyNow
        }
        RobustnessReversalState::EstablishCommandTarget => {
            session.origin_recovered = false;
            let target = establish_target_rpm(&session.point, session.direction);
            match begin_capture(session, workspace, safety, backend, target, true).await {
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
                    retry_or_finalize_inconclusive(state, session);
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
                    *state = RobustnessReversalState::ReverseCommandTarget;
                    StepResult::ReadyNow
                }
                StationarityResult::Excluded(_) => {
                    retry_or_finalize_inconclusive(state, session);
                    StepResult::ReadyNow
                }
            }
        }
        RobustnessReversalState::ReverseCommandTarget => {
            let target = reverse_target_rpm(&session.point, session.direction);
            match begin_capture(session, workspace, safety, backend, target, false).await {
                Ok((next_sample_us, budget_deadline_us)) => {
                    *state = RobustnessReversalState::ReverseCapture {
                        next_sample_us,
                        budget_deadline_us,
                        settle_deadline_us: None,
                    };
                    StepResult::At(next_sample_us)
                }
                Err(step) => step,
            }
        }
        RobustnessReversalState::ReverseCapture {
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
            match advance_capture(
                &mut workspace.capture,
                read_us,
                elapsed_us,
                delta,
                next_sample_us,
                budget_deadline_us,
                settle_deadline_us,
            ) {
                CaptureAdvance::Continue {
                    next_sample_us,
                    settle_deadline_us,
                } => {
                    *state = RobustnessReversalState::ReverseCapture {
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
                    retry_or_finalize_inconclusive(state, session);
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
            let direction = session.direction;
            match stationarity {
                StationarityResult::Usable(suffix) => {
                    let Some(reconvergence_time_us) = workspace.capture.band_entry_us() else {
                        retry_or_finalize_inconclusive(state, session);
                        return StepResult::ReadyNow;
                    };
                    let metrics = match score_reversal_leg(LegScore {
                        point: &session.point,
                        direction,
                        counts_per_rev: safety.counts_per_rev(),
                        forward_settle_time_us: session.pending_forward_settle_us,
                        reconvergence_time_us,
                        recovery_iae: workspace.capture.recovery_iae_q(),
                        suffix: &suffix,
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
                    session.peak_overshoot_counts[direction] =
                        session.peak_overshoot_counts[direction].max(overshoot);
                    session.pending_cycle = Some(PendingCycle {
                        metrics,
                        wire: CycleWire {
                            reconvergence_ms: to_ms_saturating(metrics.reconvergence_time_us),
                            overshoot_counts: overshoot,
                            forward_settle_ms: to_ms_saturating(metrics.forward_settle_time_us),
                        },
                    });
                    *state = RobustnessReversalState::ZeroCommand {
                        retry_available: true,
                    };
                    StepResult::ReadyNow
                }
                StationarityResult::Excluded(_) => {
                    retry_or_finalize_inconclusive(state, session);
                    StepResult::ReadyNow
                }
            }
        }
        RobustnessReversalState::ZeroCommand { retry_available } => {
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return fault_terminal(session, backend, None, ROBUSTNESS_CAUSE_IO, counts_per_rev);
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
                            apply_cycle_record(
                                state,
                                session,
                                CycleRecord::Conclusive(pending.metrics),
                                Some(pending.wire),
                            );
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

#[allow(clippy::result_large_err)]
async fn begin_capture<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    workspace: &mut RobustnessReversalWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
    signed_target_rpm: i32,
    open_primitive: bool,
) -> Result<(u64, u64), StepResult> {
    let counts_per_rev = safety.counts_per_rev();
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
    if backend
        .write_register(PID_VELOCITY_TARGET, signed_target_rpm as u32)
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
    if open_primitive {
        safety.end_primitive();
        if safety.begin_primitive(baseline_us).is_err() {
            return Err(fault_terminal(
                session,
                backend,
                None,
                ROBUSTNESS_CAUSE_SAFETY_ARM_FAILED,
                counts_per_rev,
            ));
        }
    }
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
    let Some(budget_deadline_us) =
        baseline_us.checked_add(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US))
    else {
        return Err(fault_terminal(
            session,
            backend,
            None,
            ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW,
            counts_per_rev,
        ));
    };
    Ok((next_sample_us, budget_deadline_us))
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

fn retry_or_finalize_inconclusive(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
) {
    let action = record_cycle(session, CycleRecord::Inconclusive, None);
    session.held_route = Some(HeldRoute::Ledger(action));
    *state = RobustnessReversalState::ZeroCommand {
        retry_available: true,
    };
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

fn apply_cycle_record(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
    record: CycleRecord,
    wire: Option<CycleWire>,
) {
    let action = record_cycle(session, record, wire);
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
    };
    let (_, thresholds) = constructed_thresholds(point, counts_per_rev);
    tripped_mask(&median_metrics, &thresholds)
}

fn direction_summary(
    ledger: &CycleLedger,
    peak_overshoot_counts: u16,
    point: &AcceptancePoint,
    counts_per_rev: u32,
) -> RobustnessDirectionSummary {
    let tripped = median_tripped(ledger, point, counts_per_rev);
    RobustnessDirectionSummary {
        median_reconvergence_ms: ledger.median_reconvergence_ms(),
        max_reconvergence_ms: ledger.max_reconvergence_ms(),
        median_forward_settle_ms: ledger.median_forward_settle_ms(),
        overshoot_peak_counts: peak_overshoot_counts,
        valid_cycles: ledger.valid_cycles(),
        trip_count: u8::from(tripped != 0 || ledger.tail_rejected()),
        retry_count: ledger.inconclusive_retries_used(),
        inconclusive: !ledger.meets_floor(),
    }
}

fn assembled_evidence(
    session: &RobustnessReversalSession,
    counts_per_rev: u32,
) -> RobustnessEvidence {
    RobustnessEvidence {
        directions: [
            direction_summary(
                &session.ledgers[0],
                session.peak_overshoot_counts[0],
                &session.point,
                counts_per_rev,
            ),
            direction_summary(
                &session.ledgers[1],
                session.peak_overshoot_counts[1],
                &session.point,
                counts_per_rev,
            ),
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
    }
}

fn complete_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    counts_per_rev: u32,
) -> StepResult {
    let evidence = assembled_evidence(session, counts_per_rev);
    let tripped_a = median_tripped(&session.ledgers[0], &session.point, counts_per_rev);
    let tripped_b = median_tripped(&session.ledgers[1], &session.point, counts_per_rev);
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::robustness::{
        ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED, ROBUSTNESS_CAUSE_PASS,
        ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED,
        ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED,
        ROBUSTNESS_CAUSE_TAIL_REPEATED,
    };
    use crate::commission_dispatch::test_support::{
        MockBackend, SpiOp, current_limit, sample_entry_snapshot,
    };
    use crate::commissioning::outer::OuterMotionBudget;
    use crate::commissioning::outer::check_outer_safety;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        FixedGainAmplitudeProgress, IntegralPlanIdentity, IntegralRetentionSlot,
    };
    use crate::commissioning::outer::velocity::robustness_metrics::{
        ROBUSTNESS_MAX_RETRIES, ROBUSTNESS_THRESHOLDS,
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
            progress: FixedGainAmplitudeProgress::ReadyAscending,
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum CapturePhase {
        Establish,
        Reverse,
        Zero,
        Recover,
    }

    fn capture_phase(state: &RobustnessReversalState) -> Option<CapturePhase> {
        match state {
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
                session: RobustnessReversalSession::new(point, 7, COUNTS_PER_REV),
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
        let mut was_in_capture = false;
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
            if phase.is_some() && !was_in_capture {
                leg_tick = 0;
            }
            was_in_capture = phase.is_some();
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
        let session = RobustnessReversalSession::new(test_point(), 0, COUNTS_PER_REV);
        let safety = test_safety();

        let result = build_origin_recovery(&session, &safety);

        assert_eq!(result, Err(CommissionError::MissingCaptureState));
    }

    fn test_session_with_recovery_plan() -> RobustnessReversalSession {
        let point = recovery_point();
        let safety = test_safety();
        let plan = robustness_recovery_plan(&point, &safety).expect("recovery plan resolves");
        let mut session = RobustnessReversalSession::new(point, 0, COUNTS_PER_REV);
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
        let session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);

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
        assert_eq!(establish_leg_travel_counts(rate), 10 * 1_016);
    }

    #[test]
    fn cycle_fits_exactly_at_the_remaining_headroom_boundary_on_both_sides() {
        let leg = establish_leg_travel_counts(12) as i32;
        let window: u32 = 30_000;
        let edge = window as i32 - leg;
        assert!(cycle_fits(12, edge, window, window, 0));
        assert!(!cycle_fits(12, edge + 1, window, window, 0));
        assert!(cycle_fits(12, -edge, window, window, 1));
        assert!(!cycle_fits(12, -(edge + 1), window, window, 1));
        assert!(cycle_fits(12, -edge, window, window, 0));
        assert!(!cycle_fits(12, -(edge + 1), window, window, 0));
        assert!(cycle_fits(12, edge, window, window, 1));
        assert!(!cycle_fits(12, edge + 1, window, window, 1));
        assert_eq!(
            remaining_headroom_counts(window as i32 + 5, window, window, 0),
            0
        );
        assert_eq!(
            remaining_headroom_counts(-(window as i32) - 5, window, window, 1),
            0
        );
        assert!(!cycle_fits(12, window as i32 + 5, window, window, 0));
    }

    #[test]
    fn assembled_evidence_saturates_extreme_cycle_values() {
        const HUGE_DURATION_US: u32 = 200_000_000;

        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
        session.ledgers[0] = CycleLedger::new(i64::MAX);
        session.ledgers[1] = CycleLedger::new(i64::MAX);
        session.ledgers[0].record(CycleRecord::Conclusive(ReversalMetrics {
            reconvergence_time_us: HUGE_DURATION_US,
            settled_residual_q: i64::MAX,
            recovery_iae_q: i64::MAX,
            dac_rms_q: 0,
            forward_settle_time_us: HUGE_DURATION_US,
            reconvergence_ratio_ppm: 0,
        }));
        session.ledgers[1].record(CycleRecord::Conclusive(ReversalMetrics {
            reconvergence_time_us: HUGE_DURATION_US,
            settled_residual_q: i64::MIN,
            recovery_iae_q: i64::MIN,
            dac_rms_q: 0,
            forward_settle_time_us: HUGE_DURATION_US,
            reconvergence_ratio_ppm: 0,
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
    fn run_leaves_acceptance_matrix_progress_untouched() {
        for (noisy, expected) in [(false, false), (true, true)] {
            let point = test_point();
            let mut slot = IntegralRetentionSlot::new();
            slot.install_acceptance_point(point);
            let before = slot.acceptance_point().expect("point installed").progress;

            let (terminal, _backend, _session) = drive(point, noisy);
            assert_eq!(
                terminal.outcome == RobustnessOutcome::Inconclusive,
                expected
            );

            assert_eq!(
                slot.acceptance_point().expect("point retained").progress,
                before
            );
            assert_eq!(before, FixedGainAmplitudeProgress::ReadyAscending);
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
        }
    }

    fn complete_with_uniform_iae(
        point: AcceptancePoint,
        recovery_iae_q: i64,
    ) -> RobustnessReversalTerminal {
        let backend = test_backend();
        let mut session = RobustnessReversalSession::new(point, 7, COUNTS_PER_REV);
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
        }
    }

    #[test]
    fn single_tail_cycle_retries_then_passes() {
        const TAIL_MS: u32 = 900;
        const FAST_MS: u32 = 60;
        let backend = test_backend();
        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
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
        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
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
    fn apply_cycle_record_tail_arms_skip_grace_then_latch_and_continue() {
        const TAIL_MS: u32 = 900;
        let tail = cycle_metrics_with_iae(TAIL_MS, TAIL_IAE_Q);
        let tail_wire = CycleWire {
            reconvergence_ms: TAIL_MS as u16,
            overshoot_counts: 3,
            forward_settle_ms: 12,
        };

        let mut state = RobustnessReversalState::EstablishWriteGain;
        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
        assert_eq!(session.direction, 0);
        assert_eq!(session.cycle_cursor[0], 0);

        apply_cycle_record(
            &mut state,
            &mut session,
            CycleRecord::Conclusive(tail),
            Some(tail_wire),
        );
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

        apply_cycle_record(
            &mut state,
            &mut session,
            CycleRecord::Conclusive(tail),
            Some(tail_wire),
        );
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
        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
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
        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
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
            }));
        }

        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());

        assert_eq!(evidence.dac_rms_medians_q[0], 250);
        assert_eq!(evidence.dac_rms_medians_q[1], 0);
    }

    #[test]
    fn overshoot_reaches_the_terminal_when_a_reverse_leg_outpaces_its_target() {
        let (terminal, _backend, session) =
            drive_scripted(test_point(), |phase, leg_tick, target| {
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Reverse if leg_tick < 3 => target.signum() * STEADY_DELTA * 2,
                    _ => target.signum() * STEADY_DELTA,
                }
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        for direction in 0..DIRECTION_COUNT {
            assert!(
                terminal.evidence.directions[direction].overshoot_peak_counts > 0,
                "a genuine burst above the target rate must reach the wire"
            );
            let first_cycle = session.cycle_wires[direction][0]
                .expect("a clean reversal run fills every cycle slot");
            assert!(
                first_cycle.overshoot_counts > 0,
                "the per-cycle evidence reply's own overshoot_counts field must \
                 carry the burst too, not just the terminal's aggregate max"
            );
        }
    }

    #[test]
    fn every_inconclusive_holds_its_route_and_rests_first() {
        let mut state = RobustnessReversalState::EstablishWriteGain;
        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);

        for _ in 0..ROBUSTNESS_MAX_RETRIES {
            retry_or_finalize_inconclusive(&mut state, &mut session);
            assert_eq!(
                state,
                RobustnessReversalState::ZeroCommand {
                    retry_available: true
                },
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

        retry_or_finalize_inconclusive(&mut state, &mut session);
        assert_eq!(
            state,
            RobustnessReversalState::ZeroCommand {
                retry_available: true
            }
        );
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
        assert_eq!(&parts.targets[..3], &[150, 0, 150]);
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
        let mut expected: heapless::Vec<i32, 64> = heapless::Vec::new();
        for direction in 0..DIRECTION_COUNT {
            let establish = establish_target_rpm(&test_point(), direction);
            for _ in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
                expected
                    .extend_from_slice(&[establish, -establish, 0])
                    .expect("expected sequence fits");
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
        establish_leg_travel_counts(establish_rate_counts_per_ms(point, COUNTS_PER_REV)) as i32
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

        assert_eq!(
            parts.state,
            RobustnessReversalState::ZeroCommand {
                retry_available: true
            }
        );
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

            assert_eq!(
                parts.state,
                RobustnessReversalState::ZeroCommand {
                    retry_available: true
                }
            );
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
            assert_eq!(parts.state, RobustnessReversalState::EstablishCommandTarget);
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
            assert_eq!(
                parts.state,
                RobustnessReversalState::ZeroCommand {
                    retry_available: true
                }
            );
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
        parts.session.direction = 1;
        parts.safety.negative_position_window = (2 * leg) as u32;
        parts.safety.positive_position_window = leg as u32;
        parts.safety.position_window = leg as u32;
        parts.safety.unwrapped_position = -(leg + 1);

        let (terminal, parts) = drive_parts(parts, Envelope::Tracked, |_phase, _tick, target| {
            if target == 0 {
                0
            } else {
                target.signum() * STEADY_DELTA
            }
        });

        assert_eq!(parts.recoveries, 1);
        assert_ne!(terminal.cause, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED);
        assert!(terminal.safety_fault.is_none());
        assert_ne!(terminal.outcome, RobustnessOutcome::Failed);
    }

    #[test]
    fn exhausted_establish_unwinds_to_origin_before_its_retry() {
        let mut establish_legs = 0u32;
        let (terminal, parts) = drive_parts(
            RunParts::new(recovery_point()),
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
        assert_eq!(&parts.targets[..5], &[150, 0, -176, 0, 150]);
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
            RunParts::new(recovery_point()),
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
        assert_eq!(&parts.targets[..5], &[150, 0, -176, 0, 150]);
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
            RunParts::new(recovery_point()),
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
        assert_eq!(&parts.targets[..3], &[150, -150, 0]);
        assert_eq!(
            parts.targets[3].abs(),
            176,
            "the recovery commands a return leg at the campaign amplitude"
        );
        assert_eq!(parts.targets[4], 0);
        assert_eq!(parts.targets[5], 150);
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
        assert_eq!(&parts.targets[..3], &[150, 0, -176]);
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
        assert_eq!(parts.targets.as_slice(), &[150, 0]);
    }

    #[test]
    fn excluded_reverse_leg_unwinds_before_its_retry() {
        let mut reverse_legs = 0u32;
        let (terminal, parts) = drive_parts(
            RunParts::new(test_point()),
            Envelope::Untracked,
            move |phase, tick, target| {
                if phase == CapturePhase::Reverse && tick == 0 {
                    reverse_legs += 1;
                }
                if target == 0 {
                    return 0;
                }
                match phase {
                    CapturePhase::Reverse if reverse_legs == 1 => {
                        if tick < 80 {
                            target.signum() * STEADY_DELTA
                        } else {
                            target.signum() * STEADY_DELTA + (tick as i32 - 80) * 3
                        }
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(&parts.targets[..4], &[150, -150, 0, 150]);
        assert_eq!(parts.recoveries, 1);
        assert_eq!(terminal.evidence.directions[0].retry_count, 1);
        assert!(terminal.safety_fault.is_none());
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
        assert_eq!(
            parts.targets.as_slice(),
            &[0],
            "only the rest was commanded"
        );
    }

    #[test]
    fn contained_recovery_with_headroom_continues() {
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
                    CapturePhase::Establish if establish_legs == 1 => {
                        target.signum() * STEADY_DELTA / 2
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(&parts.targets[..5], &[150, 0, -176, 0, 150]);
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
        assert_eq!(parts.targets.as_slice(), &[150, 0, -176, 0]);
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
        assert_eq!(parts.targets.as_slice(), &[150, 0, -176, 0]);
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
        let mut session = RobustnessReversalSession::new(test_point(), 0, COUNTS_PER_REV);
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
}
