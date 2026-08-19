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
use crate::autotune::velocity_measurement::{DyadicBlocker, StationarityWork};
use crate::autotune::velocity_measurement::{
    StationarityResult, VELOCITY_RATE_Q_SCALE, estimate_confidence,
};
use crate::commission_dispatch::robustness::{
    ROBUSTNESS_CAUSE_INTERNAL_FAULT, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED,
    ROBUSTNESS_CAUSE_SAFETY_FAULT, ROBUSTNESS_CAUSE_TAIL_REPEATED,
};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::OuterSafetyLimits;
use crate::commissioning::outer::state::VELOCITY_SWEEP_BLOCK_LEVELS;
use crate::commissioning::outer::state::VelocityObservationRing;
use crate::commissioning::outer::velocity::robustness_reporting::ROBUSTNESS_SCHEMA_REVISION;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault,
    RobustnessOutcome, RobustnessReversalTerminal, StepResult,
};
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use super::clear_velocity_integrator;
use super::integral_reproduction::AcceptancePoint;
use super::robustness_cycles::{
    CycleLedger, CycleRecord, LedgerAction, ROBUSTNESS_CYCLES_PER_DIRECTION,
};
use super::robustness_evidence::{RobustnessDirectionSummary, RobustnessEvidence, summary_cause};
use super::robustness_metrics::{
    ROBUSTNESS_BAND_DWELL_US, ROBUSTNESS_BAND_FRACTION_PCT, ROBUSTNESS_MEASUREMENT_BUDGET_US,
    ROBUSTNESS_THRESHOLDS, ReversalMetrics, ReversalOnlineAccumulator, RobustnessThresholds,
    overshoot_counts, reduce_plant_rate_q, robustness_thresholds_for, tripped_mask,
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
        }
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
}

impl RobustnessReversalSession {
    /// Freeze the accepted point for a robustness-reversal run. Reads the point
    /// only; never mutates the candidate slot or its acceptance-matrix progress.
    pub(crate) fn new(point: AcceptancePoint, run_sequence: u32) -> Self {
        let t_max_us = ROBUSTNESS_THRESHOLDS.t_max_us;
        Self {
            point,
            run_sequence,
            run_started_us: 0,
            direction: 0,
            ledgers: [CycleLedger::new(t_max_us), CycleLedger::new(t_max_us)],
            peak_overshoot_counts: [0; DIRECTION_COUNT],
            cycle_wires: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; DIRECTION_COUNT],
            displaced_tail_wires: [None; DIRECTION_COUNT],
            cycle_cursor: [0; DIRECTION_COUNT],
            pending_forward_settle_us: 0,
            pending_cycle: None,
        }
    }

    /// Recompute the probe velocity from the retained accepted point's target
    /// RPM, mirroring the conversion applied when the safety envelope was
    /// configured for this run.
    #[cfg(test)]
    pub(crate) fn probe_velocity_mrev_s_for_test(&self) -> u32 {
        self.point.target_velocity_rpm.unsigned_abs() * 1_000 / 60
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
    EstablishAnalyze { analysis_deadline_us: u64 },
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
    ReverseAnalyze { analysis_deadline_us: u64 },
    /// Command zero and open the between-legs rest window. `retry_available`
    /// tracks whether one rejected-rest re-open remains.
    ZeroCommand { retry_available: bool },
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
            *state = RobustnessReversalState::EstablishWriteGain;
            StepResult::ReadyNow
        }
        RobustnessReversalState::EstablishWriteGain => {
            // Start-of-cycle: clear the peak-excursion tracker so this cycle's
            // overshoot spans only its own establish and reverse legs.
            workspace.capture.reset_cycle_peak();
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
                    // Overshoot spans this cycle's establish and reverse legs
                    // (reset at EstablishWriteGain); fold it into the direction's
                    // running max and this cycle's wire triple. Measured against
                    // the stroke, not `position_window`: the reserve-open path
                    // widens the window to the machine-headroom clamp, and a
                    // window-relative reference would read a structural zero
                    // (any scored cycle stayed inside the window or it faulted).
                    let overshoot = overshoot_counts(
                        workspace.capture.peak_excursion_counts(),
                        safety.stroke_counts(),
                    );
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
                            // A conclusive cycle rested; record it into the
                            // ledger now and route the next cycle or direction.
                            apply_cycle_record(
                                state,
                                session,
                                CycleRecord::Conclusive(pending.metrics),
                                Some(pending.wire),
                            );
                        } else {
                            // The rested cycle was a slot-consuming inconclusive
                            // one, already recorded before the rest; perform the
                            // held routing now that the loop has come to rest.
                            route_after_consumed_slot(state, session);
                        }
                        StepResult::ReadyNow
                    } else if retry_available {
                        // I4: a rejected rest re-opens one fresh bounded rest
                        // window before faulting (integral_plan.rs precedent);
                        // it neither silently proceeds nor immediately kills the
                        // run.
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
                    } else {
                        // I4: the between-legs rest never confirmed even after the
                        // one bounded re-open. This is a specific, operator-
                        // actionable condition, not the generic internal-fault
                        // bucket, so it gets its own cause.
                        rest_not_confirmed_terminal(session, backend, counts_per_rev)
                    }
                }
                Err(_) => fault_terminal(session, backend, None, counts_per_rev),
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

/// Report an inconclusive cycle into the current direction's ledger and route.
///
/// A granted retry (`RetryInconclusive`) re-enters the establish leg
/// immediately, exactly as a retry did before any between-cycles rest existed.
/// A slot-consuming outcome (`NextCycle`/`DirectionDone`) instead brings the
/// machine to rest first: the probe just failed to converge, so the rest
/// confirms the loop can still come to a stop -- and faults the run with the
/// rest cause if it cannot -- before handing off to the next cycle or
/// direction. The record happens here so the ledger can classify the outcome;
/// the held routing runs from the rest-confirmed arm.
fn retry_or_finalize_inconclusive(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
) {
    match record_cycle(session, CycleRecord::Inconclusive, None) {
        LedgerAction::RetryInconclusive => {
            *state = RobustnessReversalState::EstablishWriteGain;
        }
        // NextCycle / DirectionDone consumed the slot; rest before the held
        // routing. An inconclusive record never yields a tail action, so any
        // other variant is treated as slot-consuming as a safe default.
        _ => {
            *state = RobustnessReversalState::ZeroCommand {
                retry_available: true,
            };
        }
    }
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

/// Route a slot-consuming inconclusive cycle after its rest confirmed. The
/// cursor already advanced when the slot was consumed, so its position selects
/// between the next cycle and finishing the direction.
fn route_after_consumed_slot(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
) {
    let action = if session.cycle_cursor[session.direction] >= ROBUSTNESS_CYCLES_PER_DIRECTION {
        LedgerAction::DirectionDone
    } else {
        LedgerAction::NextCycle
    };
    route_action(state, session, action);
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

/// Resolve a per-tick fault into the owning `Failed` terminal.
fn tick_fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    fault: TickFault,
    counts_per_rev: u32,
) -> StepResult {
    match fault {
        TickFault::Io => fault_terminal(session, backend, None, counts_per_rev),
        TickFault::Safety(report) => fault_terminal(session, backend, Some(report), counts_per_rev),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::robustness::{
        ROBUSTNESS_CAUSE_IAE_EXCEEDED, ROBUSTNESS_CAUSE_PASS,
        ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED,
        ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED, ROBUSTNESS_CAUSE_TAIL_REPEATED,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::outer::OuterMotionBudget;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptanceMatrixProgress, StageCCandidateSlot, StageCPlanIdentity,
    };
    use crate::commissioning::outer::velocity::robustness_metrics::ROBUSTNESS_MAX_RETRIES;
    use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;
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
            negative_position_headroom_mrev: 2_000,
            positive_position_headroom_mrev: 2_000,
            max_duration_ms: 60_000,
        }
    }

    fn test_safety() -> OuterSafetyLimits {
        let mut safety = OuterSafetyLimits::new(current_limit(2_448).raw(), test_budget());
        safety
            .configure_velocity_sweep_encoder_resolution(4_000)
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
    }

    /// Classify a post-step state as a capture leg, if any. `At` results are
    /// only ever scheduled from these states, so the driver reads the phase from
    /// the state the step transitioned into.
    fn capture_phase(state: &RobustnessReversalState) -> Option<CapturePhase> {
        match state {
            RobustnessReversalState::EstablishCapture { .. } => Some(CapturePhase::Establish),
            RobustnessReversalState::ReverseCapture { .. } => Some(CapturePhase::Reverse),
            RobustnessReversalState::ZeroCapture { .. } => Some(CapturePhase::Zero),
            _ => None,
        }
    }

    /// Drive one full run to its terminal against a scripted `MockBackend`.
    ///
    /// `motion` returns the per-tick encoder delta for the sample now being
    /// scheduled, given the capture phase, the tick index within the current
    /// leg (reset to 0 whenever a fresh capture leg begins, so retries restart
    /// the script), and the commanded velocity target.
    fn drive_scripted(
        point: AcceptancePoint,
        mut motion: impl FnMut(CapturePhase, u32, i32) -> i32,
    ) -> (
        RobustnessReversalTerminal,
        MockBackend,
        RobustnessReversalSession,
    ) {
        let mut state = RobustnessReversalState::Setup;
        let mut workspace = RobustnessReversalWorkspace::new();
        let mut session = RobustnessReversalSession::new(point, 7);
        let mut safety = test_safety();
        let mut backend = test_backend();

        let mut encoder_count: i32 = 0;
        let mut was_in_capture = false;
        let mut leg_tick: u32 = 0;

        for _ in 0..STEP_BUDGET {
            let result = pollster::block_on(step_robustness_reversal(
                &mut state,
                &mut workspace,
                &mut session,
                &mut safety,
                &mut backend,
            ));
            let phase = capture_phase(&state);
            if phase.is_some() && !was_in_capture {
                leg_tick = 0;
            }
            was_in_capture = phase.is_some();
            match result {
                StepResult::ReadyNow => {}
                StepResult::At(deadline) => {
                    backend.elapsed_us_value = deadline;
                    let target = *backend
                        .registers
                        .get(&PID_VELOCITY_TARGET.addr())
                        .unwrap_or(&0) as i32;
                    let phase = phase.expect("At is scheduled only from a capture state");
                    encoder_count += motion(phase, leg_tick, target);
                    // Model the TMC4671 decoder register, which wraps within
                    // [0, counts_per_rev); feeding raw signed counts would alias
                    // against EncoderDomain's u32 modulo at the 2^32 boundary.
                    backend.set_abn_decoder_count(encoder_count.rem_euclid(4_000));
                    leg_tick += 1;
                }
                StepResult::RobustnessReversalTerminal(terminal) => {
                    return (terminal, backend, session);
                }
                other => panic!("unexpected step result: {other:?}"),
            }
        }
        panic!("robustness reversal run exceeded its bounded step budget");
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
        let session = RobustnessReversalSession::new(test_point(), 7);

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

    /// A recorded cycle with extreme values must saturate, not wrap or panic,
    /// through both the ms conversion and the i64 -> i32 conversion the
    /// ledger's median accessors apply when reducing raw cycles into the
    /// schema-2 wire evidence. A very large `t_max` keeps the huge
    /// reconvergence from being classed as a tail (which would displace it
    /// from a slot instead of filling one).
    #[test]
    fn assembled_evidence_saturates_extreme_cycle_values() {
        // 200 s = 200_000 ms, well past u16::MAX (65_535) milliseconds.
        const HUGE_DURATION_US: u32 = 200_000_000;

        let mut session = RobustnessReversalSession::new(test_point(), 7);
        session.ledgers[0] = CycleLedger::new(u32::MAX);
        session.ledgers[1] = CycleLedger::new(u32::MAX);
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
        let mut session = RobustnessReversalSession::new(point, 7);
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
    /// `iae_max_q` trips `TRIP_IAE` and rejects, even though reconvergence
    /// time and settled residual stay comfortably under their own (unchanged)
    /// seeds.
    #[test]
    fn median_iae_exceeding_plant_derived_bound_trips_and_rejects() {
        // plant_rate_q 60_000, non-degenerate: iae_max_q = 4_350_000_000;
        // 5_000_000_000 exceeds it.
        let terminal = complete_with_uniform_iae(plant_identified_point(), 5_000_000_000);
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_IAE_EXCEEDED);
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

    /// Build a `ReversalMetrics` with a given reconvergence time in ms; the
    /// other fields are fixed so tail/median tests can reason about
    /// reconvergence alone.
    fn cycle_metrics(reconvergence_ms: u32) -> ReversalMetrics {
        ReversalMetrics {
            reconvergence_time_us: reconvergence_ms.saturating_mul(1_000),
            settled_residual_q: 0,
            recovery_iae_q: 0,
            forward_settle_time_us: 10_000,
            reconvergence_ratio_ppm: 0,
        }
    }

    /// A single tail cycle spends the one-shot grace, the fast replacement fills
    /// the slot, and the run still passes; the displaced tail still shows in max.
    ///
    /// A conclusive tail cycle's reconvergence (> `2 * t_max` = 860 ms) exceeds
    /// the 500 ms measurement budget, so it can never be measured through real
    /// capture motion -- the ledger is driven directly to exercise the tail
    /// grace and the assembly gate.
    #[test]
    fn single_tail_cycle_retries_then_passes() {
        const TAIL_MS: u32 = 900;
        const FAST_MS: u32 = 60;
        let backend = test_backend();
        let mut session = RobustnessReversalSession::new(test_point(), 7);
        for dir in 0..DIRECTION_COUNT {
            assert_eq!(
                session.ledgers[dir].record(CycleRecord::Conclusive(cycle_metrics(TAIL_MS))),
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
    /// the ledger for the same budget reason as
    /// [`single_tail_cycle_retries_then_passes`].
    #[test]
    fn second_tail_cycle_rejects_with_tail_cause_and_both_directions_measured() {
        const TAIL_MS: u32 = 900;
        const FAST_MS: u32 = 60;
        let backend = test_backend();
        let mut session = RobustnessReversalSession::new(test_point(), 7);
        // Direction 0: two tail cycles latch the rejection; the slot fills on the
        // second tail and the remaining four cycles fill normally.
        assert_eq!(
            session.ledgers[0].record(CycleRecord::Conclusive(cycle_metrics(TAIL_MS))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            session.ledgers[0].record(CycleRecord::Conclusive(cycle_metrics(TAIL_MS))),
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
    /// the run does not short-circuit. This is the cause-10 routing glue that no
    /// real-motion test can reach today (a tail's reconvergence exceeds the
    /// measurement budget), so it is driven at the record boundary directly.
    #[test]
    fn apply_cycle_record_tail_arms_skip_grace_then_latch_and_continue() {
        const TAIL_MS: u32 = 900;
        let tail = cycle_metrics(TAIL_MS);
        let tail_wire = CycleWire {
            reconvergence_ms: TAIL_MS as u16,
            overshoot_counts: 3,
            forward_settle_ms: 12,
        };

        let mut state = RobustnessReversalState::EstablishWriteGain;
        let mut session = RobustnessReversalSession::new(test_point(), 7);
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
                        // The first four reverse legs never enter the band: two
                        // spend the retry budget on slot 0, the next leaves slot 1
                        // unfilled, leaving three valid cycles below the floor.
                        if attempt[dir] <= 4 {
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
        let mut session = RobustnessReversalSession::new(test_point(), 7);
        session.ledgers[0] = CycleLedger::new(u32::MAX);
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

    /// Regression: overshoot is measured against the stroke, not the
    /// reserve-opened `position_window`. The reserve-open path widens
    /// `position_window` to the machine-headroom clamp; a window-relative
    /// reference would read a structural zero for every scored cycle (a cycle
    /// that reached scoring stayed inside the window or it faulted). With a
    /// budget whose window exceeds the stroke, the same peak excursion yields
    /// zero against the window but a real overshoot against the stroke.
    #[test]
    fn overshoot_reference_is_the_stroke_not_the_reopened_window() {
        let budget = OuterMotionBudget {
            requested_velocity_mrev_s: 6_000,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            // Stroke = 1_000 mrev * 4_000 cpr / 1_000 = 4_000 counts.
            max_stroke_travel_mrev: 1_000,
            settle_travel_reserve_mrev: 100,
            // Headroom windows clamp to 2 revs = 8_000 counts, past the stroke.
            negative_position_headroom_mrev: 2_000,
            positive_position_headroom_mrev: 2_000,
            max_duration_ms: 60_000,
        };
        let mut safety = OuterSafetyLimits::new(current_limit(2_448).raw(), budget);
        safety
            .configure_robustness_encoder_resolution(4_000)
            .expect("valid encoder resolution");

        let stroke = safety.stroke_counts();
        let window = safety.position_window;
        assert_eq!(stroke, 4_000, "stroke is the commanded travel allocation");
        assert!(
            window > stroke,
            "the reserve-open path widens the window past the stroke"
        );

        // A peak that overshot the stroke but stayed inside the reopened window.
        let peak = (stroke + window) / 2;
        assert_eq!(
            overshoot_counts(peak, window),
            0,
            "a window-relative reference structurally reads zero"
        );
        assert_eq!(
            overshoot_counts(peak, safety.stroke_counts()),
            (peak - stroke) as u16,
            "the stroke-relative reference reports the real overshoot"
        );
    }

    /// A granted inconclusive retry routes straight back to the establish leg
    /// with no rest, while the slot-consuming inconclusive that exhausts the
    /// budget rests first (enters `ZeroCommand`) so the loop's ability to stop
    /// is confirmed before the run moves on.
    #[test]
    fn inconclusive_retry_routes_immediately_but_slot_consume_rests() {
        let mut state = RobustnessReversalState::EstablishWriteGain;
        let mut session = RobustnessReversalSession::new(test_point(), 7);

        for _ in 0..ROBUSTNESS_MAX_RETRIES {
            retry_or_finalize_inconclusive(&mut state, &mut session);
            assert_eq!(
                state,
                RobustnessReversalState::EstablishWriteGain,
                "a granted retry re-enters establish with no rest"
            );
            assert_eq!(
                session.cycle_cursor[0], 0,
                "a granted retry does not consume a slot"
            );
        }

        // The retry budget is now spent; the next inconclusive consumes the slot
        // and must rest before the held routing.
        retry_or_finalize_inconclusive(&mut state, &mut session);
        assert!(
            matches!(
                state,
                RobustnessReversalState::ZeroCommand {
                    retry_available: true
                }
            ),
            "an exhausted-inconclusive slot-consume rests before routing"
        );
        assert_eq!(
            session.cycle_cursor[0], 1,
            "the exhausted inconclusive consumed the slot"
        );
        assert_eq!(session.ledgers[0].valid_cycles(), 0);
    }

    /// A run whose probe never converges AND whose between-cycles rest never
    /// settles faults with the dedicated rest cause -- not a soft Inconclusive.
    /// This is the schema-1 fault check the ledger rework must preserve for the
    /// exact misbehaving runs it exists to catch.
    #[test]
    fn exhausted_inconclusive_that_cannot_rest_faults_with_rest_cause() {
        let mut jitter_high = false;
        let (terminal, _backend, _session) =
            drive_scripted(test_point(), move |phase, _tick, target| match phase {
                // The reversal never enters the band: inconclusive cycles.
                CapturePhase::Reverse => {
                    jitter_high = !jitter_high;
                    if jitter_high { 300 } else { -300 }
                }
                // The between-cycles rest never settles: sustained drift.
                CapturePhase::Zero => 8,
                // The establish leg converges cleanly.
                _ => target.signum() * STEADY_DELTA,
            });

        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED);
        assert!(terminal.safety_fault.is_none());
    }
}
