//! Cooperative unreset-reversal robustness phase.
//!
//! Measures reversal robustness of the matrix-accepted operating point. For
//! each reversal direction the phase runs up to
//! [`ROBUSTNESS_CYCLES_PER_DIRECTION`] probe cycles; each cycle establishes a
//! settled cruise with the velocity integrator cleared, then reverses the
//! commanded target **without** clearing the integrator and measures how the
//! loop reconverges. Clearing the integrator on the measured leg would
//! silently turn the robustness test back into a fresh identification run, so
//! the establish leg is the only leg that clears -- see the write-log
//! invariant test. Each cycle's metrics fold into a per-direction
//! [`CycleLedger`]; the gate compares each direction's median metric against
//! the absolute thresholds and rejects a second conclusive tail cycle.
//!
//! The phase reads the accepted point read-only; it never advances or closes
//! [`AcceptanceMatrixProgress`](super::integral_reproduction::AcceptanceMatrixProgress).

use crate::autotune::CommissionError;
use crate::autotune::CommissionStatus;
use crate::autotune::TmcMechanicalRpm;
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::velocity_measurement::{DyadicBlocker, StationarityWork};
use crate::autotune::velocity_measurement::{
    StationarityResult, VELOCITY_RATE_Q_SCALE, estimate_confidence,
};
use crate::commission_dispatch::robustness::{
    ROBUSTNESS_CAUSE_INTERNAL_FAULT, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED,
    ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED, ROBUSTNESS_CAUSE_SAFETY_FAULT,
    ROBUSTNESS_CAUSE_TAIL_REPEATED,
};
use crate::commissioning::EntrySnapshot;
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
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_VELOCITY_ACTUAL, PID_VELOCITY_OFFSET,
    PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use super::band_confirmation::{CONFIRMATION_BAND_LOWER_PERCENT, CONFIRMATION_BAND_UPPER_PERCENT};
use super::clear_velocity_integrator;
use super::current_guard::{VelocityCurrentObserver, VelocityLimitEvidenceCapability};
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
    ROBUSTNESS_BAND_DWELL_US, ROBUSTNESS_BAND_FRACTION_PCT, ROBUSTNESS_MEASUREMENT_BUDGET_US,
    ReversalMetrics, ReversalOnlineAccumulator, RobustnessThresholds, reduce_plant_rate_q,
    robustness_thresholds_for, tripped_mask,
};
use super::robustness_reporting::CycleWire;
use super::sweep_analysis::poll_stationarity;
use super::sweep_analysis::{ScoredRestPoll, deficit_interval_q, poll_scored_rest_confirmation};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, SETTLE_ANALYSIS_BUDGET_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US,
};

/// Outer velocity ramp mode: single-direction velocity ramp, I applied through
/// the gain pair. Matches the shared outer-velocity mode used by the Stage-C
/// sweep and the breakaway campaign.
const OUTER_VELOCITY_RAMP_MODE: u32 = 0x8000_0002;

/// Extra in-band capture after band entry, in microseconds, so the ring's
/// settled tail holds at least the 64-sample shortest stationary suffix for the
/// residual analysis. The 48 ms band-entry dwell alone falls short of 64
/// samples at the ~1 ms observation cadence, so band entry stops the search but
/// this tail keeps sampling the settled cruise before analysis.
const ROBUSTNESS_SETTLE_TAIL_US: u32 = 80_000;

/// Confidence family size for the settled-rate interval reconstruction, matching
/// the shared scored-rest reconstruction default.
const ROBUSTNESS_CONFIDENCE_FAMILY_SIZE: u16 = 3;

/// Number of reversal directions probed: forward->reverse and reverse->forward.
const DIRECTION_COUNT: usize = 2;

/// Borrowed single-leg scratch space: capture ring, stationarity analysis,
/// dyadic blocker, the online band-entry/IAE accumulator, and the running
/// encoder/time anchors. Never allocates on the heap; every field is
/// fixed-size.
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

    /// Arm the current observer against the operational limit: this phase
    /// installs no probe clamp of its own, so the limit already mirrored
    /// into `torque_cap` by setup is the one in force.
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

// The workspace holds measurement scratch buffers (observation ring, dyadic
// blocker, stationarity work) that do not implement `defmt::Format`, so it
// cannot derive it. `OuterPhase` requires every field be `Format`, so emit an
// opaque marker instead of the buffer contents, which carry no diagnostic value.
impl defmt::Format for RobustnessReversalWorkspace {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "RobustnessReversalWorkspace");
    }
}

/// A conclusive cycle measured at the reversal leg, held until the
/// between-cycles rest confirms and the ledger records it.
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
    /// The grace-retried (displaced) tail cycle's wire triple, per direction.
    /// It never fills a slot, so its overshoot and forward-settle would
    /// otherwise be lost -- only its reconvergence survives, in the ledger max.
    displaced_tail_wires: [Option<CycleWire>; DIRECTION_COUNT],
    /// Slot cursor mirroring each ledger's own cursor, so a filled wire lands
    /// in the same slot the ledger filled.
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
    /// only; never mutates the candidate slot or its acceptance-matrix progress.
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

/// Absolute establish-cruise magnitude, in signed TMC mechanical RPM.
fn amplitude_rpm(point: &AcceptancePoint) -> i32 {
    point.target_velocity_rpm.saturating_abs()
}

/// Signed establish-cruise target for `direction`. Index 0 establishes forward
/// (measures forward->reverse); index 1 establishes reverse.
fn establish_target_rpm(point: &AcceptancePoint, direction: usize) -> i32 {
    let amplitude = amplitude_rpm(point);
    if direction == 0 {
        amplitude
    } else {
        -amplitude
    }
}

/// Signed measured-leg target for `direction`: the reverse of the establish leg.
fn reverse_target_rpm(point: &AcceptancePoint, direction: usize) -> i32 {
    -establish_target_rpm(point, direction)
}

const ESTABLISH_LEG_WINDOW_US: u32 =
    ROBUSTNESS_MEASUREMENT_BUDGET_US + ROBUSTNESS_SETTLE_TAIL_US + SETTLE_ANALYSIS_BUDGET_US;

fn establish_rate_counts_per_ms(point: &AcceptancePoint, counts_per_rev: u32) -> u32 {
    let counts_per_minute =
        u64::from(amplitude_rpm(point).unsigned_abs()).saturating_mul(u64::from(counts_per_rev));
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

fn establish_leg_fits(
    rate_counts_per_ms: u32,
    unwrapped_position: i32,
    negative_window: u32,
    positive_window: u32,
    direction: usize,
) -> bool {
    remaining_headroom_counts(
        unwrapped_position,
        negative_window,
        positive_window,
        direction,
    ) >= establish_leg_travel_counts(rate_counts_per_ms)
}

/// Per-tick rate-Q value corresponding to a signed RPM target, in the same
/// fixed-point convention as [`per_tick_rate_q`]
/// (`VELOCITY_RATE_Q_SCALE`, counts per second scaled by 1000).
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

/// Settle-band half-width for `target_rate_q`, `|target| * PCT / 100`.
fn band_q_for(target_rate_q: i64) -> i64 {
    let magnitude = i128::from(target_rate_q.unsigned_abs());
    let band = magnitude * i128::from(ROBUSTNESS_BAND_FRACTION_PCT) / 100;
    i64::try_from(band).unwrap_or(i64::MAX)
}

/// Reconvergence time expressed as parts-per-million of the forward settle time,
/// guarding a zero denominator.
fn reconvergence_ratio_ppm(reconvergence_time_us: u32, forward_settle_time_us: u32) -> u32 {
    if forward_settle_time_us == 0 {
        return 0;
    }
    let ratio = u64::from(reconvergence_time_us).saturating_mul(1_000_000)
        / u64::from(forward_settle_time_us);
    u32::try_from(ratio).unwrap_or(u32::MAX)
}

/// Reconstruct the settled-rate confidence interval from a stationary suffix.
fn settled_rate_interval_q(
    suffix: &crate::autotune::velocity_measurement::StationarySuffix,
) -> Result<(i64, i64), CommissionError> {
    let mean = suffix
        .selection
        .exact_totals
        .rate_q()
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    let confidence = estimate_confidence(suffix.selection, ROBUSTNESS_CONFIDENCE_FAMILY_SIZE)
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    let half_width = i64::try_from(confidence.half_width_rate_q)
        .map_err(|_| CommissionError::InvalidSchedule)?;
    Ok((
        mean.saturating_sub(half_width),
        mean.saturating_add(half_width),
    ))
}

/// Grouped inputs for scoring one measured reversal leg. Bundles the borrowed
/// accepted point and settled-tail suffix with the online-accumulator metrics so
/// [`score_reversal_leg`] stays within the project's parameter budget.
///
/// `reconvergence_time_us` and `recovery_iae` come from the online accumulator
/// (the ring cannot retain the full transient); the suffix supplies only the
/// settled residual.
struct LegScore<'a> {
    point: &'a AcceptancePoint,
    direction: usize,
    counts_per_rev: u32,
    forward_settle_time_us: u32,
    reconvergence_time_us: u32,
    recovery_iae: i64,
    suffix: &'a crate::autotune::velocity_measurement::StationarySuffix,
}

/// Score one measured reversal leg into its raw per-cycle
/// [`ReversalMetrics`] from the online band-entry/IAE measurement and the
/// settled-tail suffix. The gate applies its thresholds to the per-direction
/// median of these metrics at assembly, not to a single leg.
fn score_reversal_leg(input: LegScore<'_>) -> Result<ReversalMetrics, CommissionError> {
    let LegScore {
        point,
        direction,
        counts_per_rev,
        forward_settle_time_us,
        reconvergence_time_us,
        recovery_iae,
        suffix,
    } = input;
    let signed_target_rpm = reverse_target_rpm(point, direction);
    let (settled_low, settled_high) = settled_rate_interval_q(suffix)?;
    let (deficit_low, deficit_high) =
        deficit_interval_q(signed_target_rpm, counts_per_rev, settled_low, settled_high)?;
    let settled_residual_q = deficit_low.saturating_add(deficit_high) / 2;
    let mut metrics = ReversalMetrics {
        reconvergence_time_us,
        settled_residual_q,
        recovery_iae_q: recovery_iae,
        forward_settle_time_us,
        reconvergence_ratio_ppm: 0,
    };
    metrics.reconvergence_ratio_ppm =
        reconvergence_ratio_ppm(reconvergence_time_us, forward_settle_time_us);
    Ok(metrics)
}

/// Sample one observation tick into the capture ring, enforcing the shared
/// gap and quarter-turn safety gates.
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
/// `state`, `workspace`, and `session` are caller-owned; none are allocated
/// here. The engine runs [`crate::commissioning::outer::check_outer_safety`]
/// against `safety` between ticks, exactly as it does for every other outer
/// phase; this stepper never performs that check itself.
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
                Err(_) => return fault_terminal(session, backend, None, counts_per_rev),
            };
            if safety.set_encoder_baseline(baseline).is_err() {
                return fault_terminal(session, backend, None, counts_per_rev);
            }
            // Arm the shared outer-safety envelope check for the ticks that
            // follow: `check_outer_safety` returns early while `torque_cap == 0`,
            // so the dispatch only begins enforcing the envelope once the encoder
            // baseline exists, exactly as the Stage-C sweep does in its own setup.
            safety.torque_cap = u16::from(backend.current_limit());
            let now_us = backend.elapsed_us();
            if safety.arm_safety_observation(now_us).is_err() {
                return fault_terminal(session, backend, None, counts_per_rev);
            }
            session.run_started_us = now_us;
            workspace.previous_encoder_raw = baseline;
            workspace.previous_read_us = now_us;
            let plan = match robustness_recovery_plan(&session.point, safety) {
                Ok(plan) => plan,
                Err(error) => {
                    defmt::warn!("robustness recovery cannot be planned: {}", error);
                    return fault_terminal(session, backend, None, counts_per_rev);
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
                return fault_terminal(session, backend, None, counts_per_rev);
            }
            workspace.reset_current_observer_from_operational_limit(backend);
            session.recovery_plan = Some(plan);
            *state = RobustnessReversalState::EstablishWriteGain;
            StepResult::ReadyNow
        }
        RobustnessReversalState::EstablishWriteGain => {
            let rate = establish_rate_counts_per_ms(&session.point, counts_per_rev);
            if !establish_leg_fits(
                rate,
                safety.unwrapped_position,
                safety.negative_position_window,
                safety.positive_position_window,
                session.direction,
            ) {
                defmt::warn!(
                    "robustness establish leg does not fit: dir={} slot={} position={} remaining={} leg_travel={} origin_recovered={}",
                    session.direction,
                    session.cycle_cursor[session.direction],
                    safety.unwrapped_position,
                    remaining_headroom_counts(
                        safety.unwrapped_position,
                        safety.negative_position_window,
                        safety.positive_position_window,
                        session.direction,
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
                Err(_) => return fault_terminal(session, backend, None, counts_per_rev),
            };
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, i).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_RAMP_MODE)
                    .await
                    .is_err()
                || clear_velocity_integrator(backend).await.is_err()
            {
                return fault_terminal(session, backend, None, counts_per_rev);
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
            // I2: forward_settle is the establish leg's actual band entry, not
            // the window duration. Latch it as soon as band entry is measured.
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
                        return fault_terminal(session, backend, None, counts_per_rev);
                    };
                    *state = RobustnessReversalState::EstablishAnalyze {
                        analysis_deadline_us,
                    };
                    StepResult::ReadyNow
                }
                CaptureAdvance::Exhausted => {
                    // I3: the establish leg never confirmed a settle within the
                    // budget; retry from a fresh entry rather than proceed.
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
                return fault_terminal(session, backend, None, counts_per_rev);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let stationarity = match poll {
                Ok(None) => return StepResult::ReadyNow,
                Ok(Some((result, _, _))) => result,
                Err(_) => return fault_terminal(session, backend, None, counts_per_rev),
            };
            match stationarity {
                // I3: the establish leg must confirm a Usable settle before the
                // reversal is measured against it.
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
            // Deliberately no clear_velocity_integrator here: the reversal is
            // measured against the integrator state established by the entry leg.
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
                        return fault_terminal(session, backend, None, counts_per_rev);
                    };
                    *state = RobustnessReversalState::ReverseAnalyze {
                        analysis_deadline_us,
                    };
                    StepResult::ReadyNow
                }
                CaptureAdvance::Exhausted => {
                    // I1: no band entry within the full budget is the
                    // inconclusive signal; retry from the establish leg.
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
                return fault_terminal(session, backend, None, counts_per_rev);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let stationarity = match poll {
                Ok(None) => return StepResult::ReadyNow,
                Ok(Some((result, _, _))) => result,
                Err(_) => return fault_terminal(session, backend, None, counts_per_rev),
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
                    }) {
                        Ok(metrics) => metrics,
                        Err(_) => return fault_terminal(session, backend, None, counts_per_rev),
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
                    // Settled tail not stationary: retry from the establish leg
                    // so every attempt begins from a freshly confirmed settle.
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
                return fault_terminal(session, backend, None, counts_per_rev);
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
                    return fault_terminal(session, backend, None, counts_per_rev);
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
                return fault_terminal(session, backend, None, counts_per_rev);
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
                            fault_terminal(session, backend, None, counts_per_rev)
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
                Err(_) => fault_terminal(session, backend, None, counts_per_rev),
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
                    fault_terminal(session, backend, Some(report), counts_per_rev)
                }
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => {
                    defmt::warn!("robustness origin recovery fault: {}", error);
                    fault_terminal(session, backend, None, counts_per_rev)
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

/// Validated accepted proportional/integral gain pair for the point.
fn accepted_gains(point: &AcceptancePoint) -> Result<(PidPGain, PidIGain), CommissionError> {
    let p = PidPGain::new(point.selected_p).map_err(|_| CommissionError::InvalidSchedule)?;
    let i = PidIGain::new(point.selected_i).map_err(|_| CommissionError::InvalidSchedule)?;
    Ok((p, i))
}

fn planned_velocity_mrev_s(point: &AcceptancePoint) -> Result<u32, CommissionError> {
    u32::try_from(amplitude_rpm(point))
        .ok()
        .and_then(|target| target.checked_mul(1_000))
        .map(|target| target / 60)
        .filter(|velocity| *velocity != 0)
        .ok_or(CommissionError::InvalidSchedule)
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
        return Err(CommissionError::InvalidSchedule);
    }
    plan_rung_origin_recovery(
        safety.budget,
        safety.counts_per_rev(),
        planned_velocity_mrev_s(point)?,
    )
    .map_err(|_| CommissionError::InvalidSchedule)
}

/// The return direction after an inconclusive or refused leg: back toward
/// the origin from wherever the axis currently sits, not the direction the
/// probe was establishing — an inconclusive reverse leg can leave the axis
/// on either side of the origin.
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
        .ok_or(CommissionError::InvalidSchedule)?;
    let start_offset_counts = safety.unwrapped_position;
    let return_direction = return_direction_from_offset(start_offset_counts);
    let lower_rate_q = return_lower_rate_q(&session.point, return_direction)
        .ok_or(CommissionError::InvalidSchedule)?;
    let p =
        PidPGain::new(session.point.selected_p).map_err(|_| CommissionError::InvalidSchedule)?;
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
            fault_terminal(session, backend, None, counts_per_rev)
        }
    }
}

/// Command a signed cruise target and open a fresh capture window.
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
        Err(_) => return Err(fault_terminal(session, backend, None, counts_per_rev)),
    };
    let baseline_us = backend.elapsed_us();
    if backend
        .write_register(PID_VELOCITY_TARGET, signed_target_rpm as u32)
        .await
        .is_err()
    {
        return Err(fault_terminal(session, backend, None, counts_per_rev));
    }
    if open_primitive {
        // The establish entry opens the bounded primitive; a retry re-opens after
        // the idempotent end. The reversal leg continues the open primitive and
        // never re-opens it.
        safety.end_primitive();
        if safety.begin_primitive(baseline_us).is_err() {
            return Err(fault_terminal(session, backend, None, counts_per_rev));
        }
    }
    workspace.observations.clear();
    workspace.analysis.reset();
    let target_rate = target_rate_q(signed_target_rpm, counts_per_rev);
    let band = band_q_for(target_rate);
    workspace
        .capture
        .reset(target_rate, band, ROBUSTNESS_BAND_DWELL_US);
    workspace.previous_encoder_raw = baseline;
    workspace.previous_read_us = baseline_us;
    let Some(next_sample_us) = baseline_us.checked_add(CAPTURE_PERIOD_US) else {
        return Err(fault_terminal(session, backend, None, counts_per_rev));
    };
    let Some(budget_deadline_us) =
        baseline_us.checked_add(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US))
    else {
        return Err(fault_terminal(session, backend, None, counts_per_rev));
    };
    Ok((next_sample_us, budget_deadline_us))
}

/// Outcome of folding one observation into a capture leg.
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

/// Fold one observation into the online accumulator and decide the next capture
/// transition.
///
/// Band entry stops the reconvergence search but opens a bounded settle-tail
/// window (`ROBUSTNESS_SETTLE_TAIL_US`) so the ring accumulates at least the
/// shortest stationary suffix before analysis. Without band entry the leg runs
/// to `budget_deadline_us` and reports [`CaptureAdvance::Exhausted`].
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

/// Fold one cycle record into the current direction's ledger, keep the
/// session's slot cursor in lockstep with the ledger's, and store the cycle's
/// wire triple when a slot fills. Returns the ledger's routing decision.
///
/// `wire` is `Some` for a conclusive cycle, `None` for an inconclusive one. A
/// retried cycle (an inconclusive retry or the first tail grace) neither fills
/// a slot nor advances the cursor; every other action advances past the current
/// slot, filling it only for a conclusive record.
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
        // The mirrored cursor must land on the slot the ledger just filled.
        debug_assert!(session.ledgers[direction].cycles()[cursor].is_some());
    } else if matches!(action, LedgerAction::RetryTail) {
        // The grace-retried tail never fills a slot; keep its triple so the
        // evidence reply can still report its overshoot and forward-settle,
        // which the ledger max cannot recover.
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

/// Route the state machine on a ledger action for a cycle that has already been
/// recorded (and, for a slot-consuming action, already rested). `DirectionDone`
/// advances to the next direction or to assembly; every other action re-enters
/// the establish leg of the current direction. `RejectTail` latches the ledger
/// rejection but keeps running so both directions complete their cycles -- the
/// verdict is applied at assembly.
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

/// Record a conclusive cycle and route immediately. The caller has already
/// performed the between-cycles rest, so this does not enter another one.
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

/// Open a fresh zero-target rest capture window without re-commanding the
/// target or re-entering the settle phase; the caller does that once per rest.
async fn open_zero_window<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    workspace: &mut RobustnessReversalWorkspace,
    backend: &mut B,
    counts_per_rev: u32,
) -> Result<(u64, u64), StepResult> {
    let baseline = match backend.read_capture_register(ABN_DECODER_COUNT).await {
        Ok(value) => value,
        Err(_) => return Err(fault_terminal(session, backend, None, counts_per_rev)),
    };
    let baseline_us = backend.elapsed_us();
    workspace.observations.clear();
    workspace.analysis.reset_for_scored_rest(false);
    workspace.previous_encoder_raw = baseline;
    workspace.previous_read_us = baseline_us;
    let Some(next_sample_us) = baseline_us.checked_add(CAPTURE_PERIOD_US) else {
        return Err(fault_terminal(session, backend, None, counts_per_rev));
    };
    let Some(deadline_us) = baseline_us.checked_add(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US))
    else {
        return Err(fault_terminal(session, backend, None, counts_per_rev));
    };
    Ok((next_sample_us, deadline_us))
}

/// Fixed 1 kHz capture cadence, in microseconds.
const CAPTURE_PERIOD_US: u64 = VELOCITY_SWEEP_SAMPLE_PERIOD_US as u64;

/// Bounded analysis deadline from a capture-completion timestamp.
fn analysis_deadline(read_us: u64) -> Option<u64> {
    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
}

/// Microseconds per millisecond, for the cycle's saturating ms conversion.
const MICROS_PER_MILLI: u32 = 1_000;

/// Convert a microsecond duration to milliseconds, rounding up, saturating
/// into `u16`. Mirrors `robustness_cycles::to_ms_saturating`.
fn to_ms_saturating(value_us: u32) -> u16 {
    let ms = value_us.div_ceil(MICROS_PER_MILLI);
    u16::try_from(ms).unwrap_or(u16::MAX)
}

/// Construct this run's plant-adaptive `(plant_rate_q, thresholds)` pair from
/// the accepted point's identified settled-rate response, shared by the gate
/// comparison ([`median_tripped`]) and terminal diagnostic reporting so the
/// reported `plant_rate_q`/`iae_max_q` can never drift from what the gate
/// actually compared against.
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

/// Tripped-threshold bitmask for one direction's median metrics.
///
/// Builds a synthetic [`ReversalMetrics`] from the ledger's per-direction
/// medians in the units [`tripped_mask`] expects (reconvergence and
/// forward-settle in microseconds, IAE back in rate-Q microseconds from the
/// ledger's rate-Q-seconds median) and gates it against thresholds built from
/// the accepted point's own identified plant response
/// ([`robustness_thresholds_for`]), rather than the fixed plant-independent
/// const. An empty ledger yields all-zero medians, which trip nothing.
fn median_tripped(ledger: &CycleLedger, point: &AcceptancePoint, counts_per_rev: u32) -> u8 {
    let median_metrics = ReversalMetrics {
        reconvergence_time_us: u32::from(ledger.median_reconvergence_ms()).saturating_mul(1_000),
        settled_residual_q: i64::from(ledger.median_settled_residual_q()),
        recovery_iae_q: i64::from(ledger.median_recovery_iae_qs()).saturating_mul(1_000_000),
        forward_settle_time_us: u32::from(ledger.median_forward_settle_ms()).saturating_mul(1_000),
        reconvergence_ratio_ppm: 0,
    };
    let (_, thresholds) = constructed_thresholds(point, counts_per_rev);
    tripped_mask(&median_metrics, &thresholds)
}

/// Reduce one direction's ledger into its schema-2 wire summary. `trip_count`
/// marks whether the direction contributed a rejection (a median trip or a
/// latched tail); `inconclusive` marks a direction below the valid-cycle floor.
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

/// Reduce both directions' ledgers into the schema-2 wire evidence.
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
        cycles: session.cycle_wires,
        tail_counts: [
            session.ledgers[0].tail_count(),
            session.ledgers[1].tail_count(),
        ],
        displaced_tails: session.displaced_tail_wires,
    }
}

/// Build the successful/rejected/inconclusive terminal from the ledgers.
///
/// A latched tail rejection in either direction outranks every median trip and
/// reports [`ROBUSTNESS_CAUSE_TAIL_REPEATED`]; otherwise a median trip in
/// either direction rejects with its threshold cause (time > residual > IAE), a
/// below-floor direction reports inconclusive, and a clean run passes.
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
    // A latched tail rejection means exactly the second (or later) conclusive
    // tail cycle occurred, so the tail count is at least two.
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

/// Build a `Failed` terminal stamping an explicit cause and safety fault.
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

/// Build a `Failed` terminal. A populated `safety_fault` is a real envelope
/// trip (`ROBUSTNESS_CAUSE_SAFETY_FAULT`); `None` is an internal, non-envelope
/// fault (`ROBUSTNESS_CAUSE_INTERNAL_FAULT`) so the host can disambiguate them.
fn fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    safety_fault: Option<OuterSafetyFault>,
    counts_per_rev: u32,
) -> StepResult {
    let cause = if safety_fault.is_some() {
        ROBUSTNESS_CAUSE_SAFETY_FAULT
    } else {
        ROBUSTNESS_CAUSE_INTERNAL_FAULT
    };
    failed_terminal(session, backend, cause, safety_fault, counts_per_rev)
}

/// Build a `Failed` terminal for the between-legs rest that was never confirmed,
/// even after the one bounded re-opened rest window. Distinct from the generic
/// `ROBUSTNESS_CAUSE_INTERNAL_FAULT` so the host can surface this specific,
/// operator-actionable condition.
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

/// Resolve a per-tick fault into the owning `Failed` terminal.
fn tick_fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    fault: TickFault,
    counts_per_rev: u32,
) -> StepResult {
    match fault {
        TickFault::Io => fault_terminal(session, backend, None, counts_per_rev),
        TickFault::Safety(report) => {
            defmt::warn!("robustness safety fault (tick): {}", report);
            fault_terminal(session, backend, Some(report), counts_per_rev)
        }
    }
}

/// Build a `Failed` robustness terminal for a dispatch-level outer-safety abort.
///
/// Mirrors `integral_fault_terminal`: it reconstructs the terminal from the live
/// phase session without a backend, so the shared outer dispatch can turn a
/// pre-tick envelope trip into the phase-appropriate robustness terminal instead
/// of a generic `OuterFailed`. `run_completed_us` mirrors `run_started_us`
/// because the abort preempts the phase before it records its own completion.
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
        ROBUSTNESS_CAUSE_INTERNAL_FAULT
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
/// Joins the robustness-selected velocity gains from `terminal` with the entry
/// snapshot's position gains, velocity limit, and biquad cutoffs, and fills the
/// firmware-authored provenance from the run's diagnostic evidence. The
/// mechanical-identification fields stay zero: the breakaway-seeded path that
/// reaches this gate never runs mechanical identification. A passed gate is
/// always reported as `Accepted`: the retained point's operating-point
/// qualifiers are measurement/capability provenance surfaced in the
/// acceptance evidence, not a status verdict, so they never gate the tune
/// status here.
pub(crate) fn robustness_pass_outer_result(
    terminal: &RobustnessReversalTerminal,
    entry: &EntrySnapshot,
    point: &AcceptancePoint,
) -> OuterResult {
    let status = CommissionStatus::Accepted;
    let provenance = TuneProvenance {
        probed_velocity_mrev_s: terminal.target_velocity_rpm.unsigned_abs() * 1_000 / 60,
        // Settled-rate-Q reduction of the accepted point's response intervals,
        // already computed as the gate's plant rate; the same encoder-rate-Q
        // domain the provenance field documents. Always non-negative.
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::robustness::{
        ROBUSTNESS_CAUSE_INTERNAL_FAULT, ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED,
        ROBUSTNESS_CAUSE_PASS, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE,
        ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED,
        ROBUSTNESS_CAUSE_TAIL_REPEATED,
    };
    use crate::commission_dispatch::test_support::{
        MockBackend, SpiOp, current_limit, sample_entry_snapshot,
    };
    use crate::commissioning::outer::OuterMotionBudget;
    use crate::commissioning::outer::check_outer_safety;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptanceMatrixProgress, StageCCandidateSlot, StageCPlanIdentity,
    };
    use crate::commissioning::outer::velocity::robustness_metrics::{
        ROBUSTNESS_MAX_RETRIES, ROBUSTNESS_THRESHOLDS,
    };
    use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;
    use crate::commissioning::types::OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY;
    use foci_tmc4671::registers::PID_ERROR_PID_VELOCITY_ERROR_SUM;

    const STEADY_DELTA: i32 = 12;
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
        safety.torque_cap = current_limit(2_448).raw() as u16;
        safety
    }

    fn test_backend() -> MockBackend {
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);
        backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
        backend
    }

    /// Encoder counts/rev used to build each test session's `iae_max_q`,
    /// matching `test_backend()`'s configured value.
    const COUNTS_PER_REV: u32 = 4_000;

    fn test_point() -> AcceptancePoint {
        AcceptancePoint {
            identity: StageCPlanIdentity {
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
            progress: AcceptanceMatrixProgress::ReadyAscending,
        }
    }

    /// Which capture leg is scheduling the current sample.
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
                    // Model the TMC4671 decoder register, which wraps within
                    // [0, counts_per_rev); feeding raw signed counts would alias
                    // against EncoderDomain's u32 modulo at the 2^32 boundary.
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

    /// Drive a run where establish and zero legs behave, and the measured
    /// reversal legs either settle cleanly (`noisy_reversal == false`) or jitter
    /// so wildly they never enter the band (`true`).
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

    /// A direction that never recorded any leg (a fault preempted the run
    /// before either leg was measured) must report `valid_cycles = 0`, not
    /// fabricate a cycle that never ran.
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
    fn establish_leg_travel_covers_budget_settle_tail_and_analysis() {
        let rate = establish_rate_counts_per_ms(&test_point(), COUNTS_PER_REV);
        assert_eq!(rate, 12);
        assert_eq!(ESTABLISH_LEG_WINDOW_US, 816_000);
        assert_eq!(establish_leg_travel_counts(rate), 12 * 816);
    }

    #[test]
    fn establish_leg_fits_exactly_at_the_remaining_headroom_boundary() {
        let leg = establish_leg_travel_counts(12) as i32;
        let window: u32 = 20_000;
        let edge = window as i32 - leg;
        assert!(establish_leg_fits(12, edge, window, window, 0));
        assert!(!establish_leg_fits(12, edge + 1, window, window, 0));
        assert!(establish_leg_fits(12, -edge, window, window, 1));
        assert!(!establish_leg_fits(12, -(edge + 1), window, window, 1));
        assert_eq!(
            remaining_headroom_counts(window as i32 + 5, window, window, 0),
            0
        );
        assert_eq!(
            remaining_headroom_counts(-(window as i32) - 5, window, window, 1),
            0
        );
        assert!(!establish_leg_fits(
            12,
            window as i32 + 5,
            window,
            window,
            0
        ));
    }

    /// A recorded cycle with extreme values must saturate, not wrap or panic,
    /// through both the ms conversion and the i64 -> i32 conversion the
    /// ledger's median accessors apply when reducing raw cycles into the
    /// schema-2 wire evidence. `iae_max_q` is set to `i64::MAX` so the
    /// extreme `recovery_iae_q` values themselves are never classed as tail
    /// (which would displace a cycle from a slot instead of filling one).
    #[test]
    fn assembled_evidence_saturates_extreme_cycle_values() {
        // 200 s = 200_000 ms, well past u16::MAX (65_535) milliseconds.
        const HUGE_DURATION_US: u32 = 200_000_000;

        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
        session.ledgers[0] = CycleLedger::new(i64::MAX);
        session.ledgers[1] = CycleLedger::new(i64::MAX);
        session.ledgers[0].record(CycleRecord::Conclusive(ReversalMetrics {
            reconvergence_time_us: HUGE_DURATION_US,
            settled_residual_q: i64::MAX,
            recovery_iae_q: i64::MAX,
            forward_settle_time_us: HUGE_DURATION_US,
            reconvergence_ratio_ppm: 0,
        }));
        session.ledgers[1].record(CycleRecord::Conclusive(ReversalMetrics {
            reconvergence_time_us: HUGE_DURATION_US,
            settled_residual_q: i64::MIN,
            recovery_iae_q: i64::MIN,
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

    /// The load-bearing invariant: the integrator is cleared exactly once per
    /// establish-cruise leg and never on the measured reversal leg. A stray
    /// clear on the reversal would silently re-run identification and make the
    /// gate vacuous.
    #[test]
    fn integrator_clears_only_on_establish_never_on_reversal() {
        // Clean run: one establish clear per cycle, no retries -> one clear per
        // cycle across both directions.
        let (_terminal, backend, _session) = drive(test_point(), false);
        assert_eq!(
            backend.pid_velocity_error_sum_clears,
            ROBUSTNESS_CYCLES_PER_DIRECTION * DIRECTION_COUNT,
            "clean run clears the integrator once per establish leg"
        );

        // The last commanded nonzero target is the final direction's reversal
        // target; no integrator clear may appear between it and the terminal.
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

        // Noisy run: every cycle slot is attempted at least once and the two
        // inconclusive retries add two more attempts, so each direction runs
        // ROBUSTNESS_CYCLES_PER_DIRECTION + ROBUSTNESS_MAX_RETRIES establish
        // legs -- still zero clears on any reversal leg.
        let (_terminal, noisy_backend, _noisy_session) = drive(test_point(), true);
        let attempts_per_direction =
            ROBUSTNESS_CYCLES_PER_DIRECTION + usize::from(ROBUSTNESS_MAX_RETRIES);
        assert_eq!(
            noisy_backend.pid_velocity_error_sum_clears,
            attempts_per_direction * DIRECTION_COUNT,
            "each establish attempt clears once; reversals never clear"
        );
    }

    // This test checks a slot the phase never receives (the session holds a
    // copy), so it is tautological; the real guarantee is the
    // `&StageCCandidateSlot` shared-ref signatures on the dispatch entry points.
    #[test]
    fn run_leaves_acceptance_matrix_progress_untouched() {
        for (noisy, expected) in [(false, false), (true, true)] {
            let point = test_point();
            let mut slot = StageCCandidateSlot::new();
            slot.install_acceptance_point(point);
            let before = slot.acceptance_point().expect("point installed").progress;

            let (terminal, _backend, _session) = drive(point, noisy);
            assert_eq!(
                terminal.outcome == RobustnessOutcome::Inconclusive,
                expected
            );

            // The phase holds only a copy of the point; the candidate slot's
            // acceptance-matrix progress is never advanced or closed.
            assert_eq!(
                slot.acceptance_point().expect("point retained").progress,
                before
            );
            assert_eq!(before, AcceptanceMatrixProgress::ReadyAscending);
        }
    }

    /// I1: a slow-but-clean reconvergence -- band entry between `t_max_us`
    /// and the budget-minus-dwell censoring point -- now yields a real
    /// `TRIP_TIME` + `Rejected` verdict. Under the old fixed 100 ms window
    /// this was physically unreachable: the window ended before band entry,
    /// aliasing the leg to Inconclusive.
    #[test]
    fn slow_reconvergence_trips_time_and_rejects() {
        let (terminal, _backend, _session) = drive_scripted(test_point(), |phase, tick, target| {
            if target == 0 {
                return 0;
            }
            match phase {
                // Stall past t_max but short of the budget-minus-dwell censoring
                // point, then converge in-band.
                CapturePhase::Reverse if tick < 440 => 0,
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

    /// I5 anti-aliasing, end to end: a run where one direction TRIPS a
    /// threshold and the other is INCONCLUSIVE must reject with the trip cause,
    /// never fold the reject and inconclusive verdicts into one another. This
    /// is the exact mixed regime the hardware characterization produces.
    #[test]
    fn mixed_trip_and_inconclusive_yields_rejected_with_trip_cause() {
        let mut jitter_high = false;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| {
                match phase {
                    // Direction 0 reversal (negative target): stall past t_max but
                    // short of the budget-minus-dwell censoring point, then
                    // converge in band -> a real TRIP_TIME rejection.
                    CapturePhase::Reverse if target < 0 => {
                        if tick < 440 {
                            0
                        } else {
                            target.signum() * STEADY_DELTA
                        }
                    }
                    // Direction 1 reversal (positive target): jitter so wildly it
                    // never enters the band -> inconclusive after the retry budget.
                    CapturePhase::Reverse if target > 0 => {
                        jitter_high = !jitter_high;
                        if jitter_high { 300 } else { -300 }
                    }
                    _ if target != 0 => target.signum() * STEADY_DELTA,
                    _ => 0,
                }
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

    /// I3: an establish leg whose settled tail fails stationarity (Excluded)
    /// must retry from a fresh entry, counting against the retry budget, rather
    /// than silently proceeding to measure the reversal against an unconfirmed
    /// settle.
    #[test]
    fn excluded_establish_settle_retries_and_counts() {
        let (terminal, _backend, _session) = drive_scripted(test_point(), |phase, tick, target| {
            if target == 0 {
                return 0;
            }
            match phase {
                CapturePhase::Establish => {
                    // In-band long enough to reach band entry, then a clear drift
                    // so the settled tail resolves a slope and is Excluded.
                    if tick < 50 {
                        target.signum() * STEADY_DELTA
                    } else {
                        target.signum() * STEADY_DELTA + (tick as i32 - 50) * 3
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

    /// I4: a rejected between-legs rest re-opens exactly one fresh bounded rest
    /// window; if that window confirms, the run proceeds normally.
    #[test]
    fn rejected_between_legs_rest_reopens_one_window_then_proceeds() {
        let mut zero_windows = 0u32;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| {
                match phase {
                    CapturePhase::Zero => {
                        if tick == 0 {
                            zero_windows += 1;
                        }
                        // Reject only the first rest window with sustained motion;
                        // the re-opened window is at rest and confirms.
                        if zero_windows == 1 { 8 } else { 0 }
                    }
                    _ if target != 0 => target.signum() * STEADY_DELTA,
                    _ => 0,
                }
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert!(terminal.safety_fault.is_none());
    }

    /// I4: a rest that is still rejected after the one bounded re-open faults
    /// the run with the dedicated rest-not-confirmed cause rather than folding
    /// into the generic internal-fault bucket or silently proceeding.
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

    /// Median of a fully filled direction's five per-cycle reconvergence wires.
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

    /// K=5 clean cycles per direction, each with a distinct reconvergence time,
    /// complete with the per-direction median reported on the summary matching
    /// the median of the cycle's own wire triples.
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
            // The five scripted stalls are distinct, so their wires are too.
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

    /// A per-direction reconvergence median above `t_max` rejects on the time
    /// cause even when individual cycles are fast.
    #[test]
    fn slow_median_rejects_with_time_cause() {
        const STALLS_MS: [u32; ROBUSTNESS_CYCLES_PER_DIRECTION] = [400, 445, 435, 415, 440];
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

    /// An accepted point whose identified plant response is well above the
    /// near-zero/degenerate cut-in, so its derived `iae_max_q` actually scales
    /// with the coefficient rather than falling open to the retired seed.
    fn plant_identified_point() -> AcceptancePoint {
        AcceptancePoint {
            selected_response_intervals_q: [[50_000, 60_000], [-60_000, -50_000]],
            ..test_point()
        }
    }

    /// Build a `ReversalMetrics` with a fixed reconvergence time and residual
    /// (both well under their carried-over seeds) and a caller-chosen IAE, so
    /// IAE tests can reason about the IAE gate alone.
    fn iae_metrics(recovery_iae_q: i64) -> ReversalMetrics {
        ReversalMetrics {
            reconvergence_time_us: 50_000,
            settled_residual_q: 0,
            recovery_iae_q,
            forward_settle_time_us: 10_000,
            reconvergence_ratio_ppm: 0,
        }
    }

    /// Fill both directions' ledgers with `ROBUSTNESS_CYCLES_PER_DIRECTION`
    /// identical conclusive cycles at the given IAE and drive the run to its
    /// terminal via `complete_terminal`, mirroring the tail-grace tests' direct
    /// ledger construction (a conclusive cycle's IAE cannot be driven through
    /// real capture motion without also entangling reconvergence timing).
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

    /// A median `recovery_iae_q` above the accepted point's plant-derived
    /// `iae_max_q` can only occur if at least 3 of 5 cycles individually
    /// exceed `iae_max_q`; the second such cycle latches a tail rejection
    /// before a third could ever be recorded, so a genuinely IAE-exceeding
    /// median always surfaces as `ROBUSTNESS_CAUSE_TAIL_REPEATED`, not a
    /// median-only IAE trip.
    #[test]
    fn median_iae_exceeding_plant_derived_bound_trips_and_rejects() {
        // plant_rate_q 60_000, non-degenerate: iae_max_q = 4_350_000_000;
        // 5_000_000_000 exceeds it.
        let terminal = complete_with_uniform_iae(plant_identified_point(), 5_000_000_000);
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_TAIL_REPEATED);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_ne!(direction.trip_count, 0, "the excess median IAE must trip");
        }
    }

    /// The same plant-derived bound as
    /// [`median_iae_exceeding_plant_derived_bound_trips_and_rejects`], but with
    /// a median `recovery_iae_q` under it: the run completes cleanly, proving
    /// the adaptive threshold does not reject on IAE alone.
    #[test]
    fn median_iae_within_plant_derived_bound_completes() {
        // Same plant as the trip case (iae_max_q = 4_350_000_000);
        // 4_000_000_000 stays under it.
        let terminal = complete_with_uniform_iae(plant_identified_point(), 4_000_000_000);
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_eq!(direction.trip_count, 0);
        }
    }

    /// The passed gate's tune provenance must report the accepted point's
    /// held-out confirmation confidence and band position, not stubs.
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

    /// `test_point()`'s degenerate plant response gives `iae_max_q =
    /// 23_466_666_000_000` (the seed bound, see `ROBUSTNESS_IAE_SEED_MAX_Q`).
    /// `NON_TAIL_IAE_Q` stays far under it; `TAIL_IAE_Q` sits far over it.
    const NON_TAIL_IAE_Q: i64 = 100_000;
    const TAIL_IAE_Q: i64 = 25_000_000_000_000;

    /// Build a `ReversalMetrics` with a given reconvergence time in ms and a
    /// non-tail `recovery_iae_q`; the other fields are fixed so tail/median
    /// tests can reason about reconvergence alone.
    fn cycle_metrics(reconvergence_ms: u32) -> ReversalMetrics {
        cycle_metrics_with_iae(reconvergence_ms, NON_TAIL_IAE_Q)
    }

    /// Build a `ReversalMetrics` with a given reconvergence time in ms and a
    /// caller-chosen `recovery_iae_q`, so tail tests can drive `is_tail()`
    /// directly.
    fn cycle_metrics_with_iae(reconvergence_ms: u32, recovery_iae_q: i64) -> ReversalMetrics {
        ReversalMetrics {
            reconvergence_time_us: reconvergence_ms.saturating_mul(1_000),
            settled_residual_q: 0,
            recovery_iae_q,
            forward_settle_time_us: 10_000,
            reconvergence_ratio_ppm: 0,
        }
    }

    /// A single tail cycle spends the one-shot grace, the fast replacement fills
    /// the slot, and the run still passes; the displaced tail still shows in max.
    ///
    /// A conclusive tail cycle's `recovery_iae_q` exceeds the accepted point's
    /// `iae_max_q`, which real capture motion would only produce on a genuine
    /// outlier -- the ledger is driven directly to exercise the tail grace and
    /// the assembly gate.
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

    /// A second conclusive tail cycle latches the tail rejection with the
    /// dedicated cause, and the run continues so both directions still measure
    /// their cycles (the latch is applied at assembly, not mid-run). Driven at
    /// the ledger for the same reason as
    /// [`single_tail_cycle_retries_then_passes`]: a genuine `recovery_iae_q`
    /// outlier above `iae_max_q`.
    #[test]
    fn second_tail_cycle_rejects_with_tail_cause_and_both_directions_measured() {
        const TAIL_MS: u32 = 900;
        const FAST_MS: u32 = 60;
        let backend = test_backend();
        let mut session = RobustnessReversalSession::new(test_point(), 7, COUNTS_PER_REV);
        // Direction 0: two tail cycles latch the rejection; the slot fills on the
        // second tail and the remaining four cycles fill normally.
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
        // Direction 1: fully clean.
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

    /// The tail arms of `apply_cycle_record` itself: `RetryTail` skips the wire
    /// and holds the cursor, `RejectTail` fills the slot, advances the cursor,
    /// latches the rejection, and still routes back into the same direction so
    /// the run does not short-circuit. This is the cause-10 routing glue; a
    /// tail is driven directly at the record boundary via a caller-chosen
    /// `recovery_iae_q` above the session's `iae_max_q`, rather than through
    /// real capture motion.
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

        // First tail -> RetryTail arm: the displaced tail is not written to a
        // slot, the cursor stays put, and the ledger keeps it only for max.
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

        // Second tail -> RejectTail arm: fills slot 0, advances the cursor by
        // one (mirroring the ledger), latches the rejection, and still routes
        // back into direction 0 rather than short-circuiting to Assemble.
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

        // The mirrored cursor tracks the ledger's own slot fills exactly.
        assert_eq!(session.ledgers[0].valid_cycles(), 1);
        for slot in 0..ROBUSTNESS_CYCLES_PER_DIRECTION {
            assert_eq!(
                session.cycle_wires[0][slot].is_some(),
                session.ledgers[0].cycles()[slot].is_some(),
                "wire slot {slot} mirrors the ledger slot",
            );
        }

        // The assembled evidence carries the tail count and the displaced
        // tail's full triple for the reply, not just its reconvergence via max.
        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());
        assert_eq!(
            evidence.tail_counts[0], 2,
            "grace tail plus the latched tail"
        );
        assert_eq!(evidence.displaced_tails[0], Some(tail_wire));
    }

    /// A direction that exhausts its inconclusive budget and leaves two slots
    /// unfilled falls below the valid-cycle floor and reports inconclusive.
    #[test]
    fn floor_breach_reports_inconclusive() {
        let mut attempt = [0usize; DIRECTION_COUNT];
        let mut jitter = false;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, tick, target| {
                match phase {
                    CapturePhase::Reverse => {
                        let dir = if target < 0 { 0 } else { 1 };
                        if tick == 0 {
                            attempt[dir] += 1;
                        }
                        // The first six reverse legs never enter the band: the
                        // first five spend the per-direction retry budget on slot
                        // 0, the sixth leaves slot 1 unfilled, leaving three valid
                        // cycles below the floor.
                        if attempt[dir] <= 6 {
                            jitter = !jitter;
                            if jitter { 300 } else { -300 }
                        } else {
                            target.signum() * STEADY_DELTA
                        }
                    }
                    _ if target != 0 => target.signum() * STEADY_DELTA,
                    _ => 0,
                }
            });

        assert_eq!(terminal.outcome, RobustnessOutcome::Inconclusive);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE);
        for summary in terminal.evidence.directions {
            assert!(summary.inconclusive);
            assert_eq!(summary.valid_cycles, 3);
        }
    }

    /// A safety-envelope violation mid-cycle still fails with the safety cause
    /// and a populated safety-fault context.
    #[test]
    fn safety_fault_mid_cycle_still_fails_with_envelope_cause() {
        use crate::commission_dispatch::robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;

        let (terminal, _backend, _session) = drive_scripted(test_point(), |phase, tick, target| {
            match phase {
                // A single quarter-turn jump on the first reverse tick trips the
                // shared outer-safety envelope.
                CapturePhase::Reverse if tick == 0 => 1_500,
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            }
        });

        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_SAFETY_FAULT);
        assert!(terminal.safety_fault.is_some());
    }

    /// Distinct per-cycle residual and recovery-IAE values reduce to the correct
    /// medians in the assembled evidence. The IAE case is even-count and chosen
    /// so converting each cycle to rate-Q seconds *before* the median (the
    /// ledger's order) differs from converting the median afterward.
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
                forward_settle_time_us: 10_000,
                reconvergence_ratio_ppm: 0,
            }));
        }

        let evidence = assembled_evidence(&session, test_safety().counts_per_rev());

        // Residual median (rate-Q, no unit change): median of [10,20,30,40] = 25.
        assert_eq!(evidence.residual_medians_q[0], 25);
        // IAE median converting each to seconds first: floors [1,2,3,4] -> 2,
        // not median-then-convert (3M us -> 3).
        assert_eq!(evidence.iae_medians_qs[0], 2);
        // Direction 1 never ran: all-zero medians.
        assert_eq!(evidence.residual_medians_q[1], 0);
        assert_eq!(evidence.iae_medians_qs[1], 0);
    }

    /// The call site reads `reversal_overshoot_counts()` directly -- no stroke
    /// or window reference enters into it at all (superseding
    /// `overshoot_reference_is_the_stroke_not_the_reopened_window`, whose
    /// stroke-relative premise this redesign removes). A reverse leg that
    /// genuinely runs ahead of its commanded target rate for a few ticks, even
    /// if it settles back onto the target rate before band entry, must reach
    /// the terminal's per-direction wire summary as a nonzero
    /// `overshoot_peak_counts` -- proving the new accumulator value actually
    /// propagates through `PendingCycle`/`session.peak_overshoot_counts` to the
    /// terminal, not just that the accumulator computes it correctly in
    /// isolation (Step 1's tests already cover that).
    #[test]
    fn overshoot_reaches_the_terminal_when_a_reverse_leg_outpaces_its_target() {
        let (terminal, _backend, session) =
            drive_scripted(test_point(), |phase, leg_tick, target| {
                if target == 0 {
                    return 0;
                }
                match phase {
                    // 2x steady rate for the first 3 ticks of every reverse leg,
                    // well under quarter_turn_fault_counts (1_000 at this test's
                    // 4_000 counts/rev) so it never trips the safety fault.
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
        assert_eq!(&parts.targets[..3], &[176, 0, 176]);
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
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_INTERNAL_FAULT);
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
        assert_eq!(&parts.targets[..5], &[176, 0, -176, 0, 176]);
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
        assert_eq!(&parts.targets[..5], &[176, 0, -176, 0, 176]);
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
        assert_eq!(&parts.targets[..3], &[176, -176, 0]);
        assert_eq!(
            parts.targets[3].abs(),
            176,
            "the recovery commands a return leg"
        );
        assert_eq!(parts.targets[4], 0);
        assert_eq!(parts.targets[5], 176);
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
        assert_eq!(&parts.targets[..3], &[176, 0, -176]);
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
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_INTERNAL_FAULT);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(parts.recoveries, 0);
        assert_eq!(parts.targets.as_slice(), &[176, 0]);
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
                        if tick < 50 {
                            target.signum() * STEADY_DELTA
                        } else {
                            target.signum() * STEADY_DELTA + (tick as i32 - 50) * 3
                        }
                    }
                    _ => target.signum() * STEADY_DELTA,
                }
            },
        );
        assert_eq!(&parts.targets[..4], &[176, -176, 0, 176]);
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
        assert_eq!(&parts.targets[..5], &[176, 0, -176, 0, 176]);
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
        assert_eq!(parts.targets.as_slice(), &[176, 0, -176, 0]);
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
        assert_eq!(parts.targets.as_slice(), &[176, 0, -176, 0]);
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
}
