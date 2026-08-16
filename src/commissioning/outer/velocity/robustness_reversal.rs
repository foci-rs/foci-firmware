//! Cooperative unreset-reversal robustness phase.
//!
//! Measures reversal robustness of the matrix-accepted operating point. For
//! each reversal direction the phase establishes a settled cruise with the
//! velocity integrator cleared, then reverses the commanded target **without**
//! clearing the integrator and measures how the loop reconverges. Clearing the
//! integrator on the measured leg would silently turn the robustness test back
//! into a fresh identification run, so the establish leg is the only leg that
//! clears -- see the write-log invariant test.
//!
//! The phase reads the accepted point read-only; it never advances or closes
//! [`AcceptanceMatrixProgress`](super::integral_reproduction::AcceptanceMatrixProgress).
//! Full engine/dispatch wiring is deferred to later work; this module
//! builds and host-tests the stepper against a mock backend.

use crate::autotune::CommissionError;
use crate::autotune::velocity_measurement::{DyadicBlocker, StationarityWork};
use crate::autotune::velocity_measurement::{
    StationarityResult, VELOCITY_RATE_Q_SCALE, estimate_confidence,
};
use crate::commission_dispatch::robustness::{
    ROBUSTNESS_CAUSE_INTERNAL_FAULT, ROBUSTNESS_CAUSE_SAFETY_FAULT,
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
use super::robustness_evidence::{RobustnessDirectionEvidence, RobustnessEvidence, summary_cause};
use super::robustness_metrics::{
    ROBUSTNESS_BAND_DWELL_US, ROBUSTNESS_BAND_FRACTION_PCT, ROBUSTNESS_MAX_RETRIES,
    ROBUSTNESS_MEASUREMENT_BUDGET_US, ROBUSTNESS_THRESHOLDS, ReversalMetrics,
    ReversalOnlineAccumulator, tripped_mask,
};
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

/// Frozen accepted point plus the mutable per-direction robustness progress.
#[derive(Debug, defmt::Format)]
pub(crate) struct RobustnessReversalSession {
    point: AcceptancePoint,
    run_sequence: u32,
    run_started_us: u64,
    direction: usize,
    retry_count: [u8; DIRECTION_COUNT],
    directions: [Option<RobustnessDirectionEvidence>; DIRECTION_COUNT],
    pending_forward_settle_us: u32,
}

impl RobustnessReversalSession {
    /// Freeze the accepted point for a robustness-reversal run. Reads the point
    /// only; never mutates the candidate slot or its acceptance-matrix progress.
    pub(crate) const fn new(point: AcceptancePoint, run_sequence: u32) -> Self {
        Self {
            point,
            run_sequence,
            run_started_us: 0,
            direction: 0,
            retry_count: [0; DIRECTION_COUNT],
            directions: [None; DIRECTION_COUNT],
            pending_forward_settle_us: 0,
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

/// Score one measured reversal leg from its online band-entry/IAE measurement
/// and the settled-tail suffix.
///
/// `reconvergence_time_us` and `recovery_iae` come from the online accumulator
/// (the ring cannot retain the full transient); the suffix supplies only the
/// settled residual.
#[allow(clippy::too_many_arguments)]
fn score_reversal_leg(
    point: &AcceptancePoint,
    direction: usize,
    counts_per_rev: u32,
    forward_settle_time_us: u32,
    reconvergence_time_us: u32,
    recovery_iae: i64,
    retry_count: u8,
    suffix: &crate::autotune::velocity_measurement::StationarySuffix,
) -> Result<RobustnessDirectionEvidence, CommissionError> {
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
    let tripped = tripped_mask(&metrics, &ROBUSTNESS_THRESHOLDS);
    Ok(RobustnessDirectionEvidence {
        metrics,
        tripped,
        retry_count,
        inconclusive: false,
    })
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
/// phase; this stepper never performs that check itself. The full engine
/// constructor and dispatch wiring is deferred to later work.
pub(crate) async fn step_robustness_reversal<B: CommissioningBackend>(
    state: &mut RobustnessReversalState,
    workspace: &mut RobustnessReversalWorkspace,
    session: &mut RobustnessReversalSession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> StepResult {
    match *state {
        RobustnessReversalState::Setup => {
            let baseline = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fault_terminal(session, backend, None),
            };
            if safety.set_encoder_baseline(baseline).is_err() {
                return fault_terminal(session, backend, None);
            }
            // Arm the shared outer-safety envelope check for the ticks that
            // follow: `check_outer_safety` returns early while `torque_cap == 0`,
            // so the dispatch only begins enforcing the envelope once the encoder
            // baseline exists, exactly as the Stage-C sweep does in its own setup.
            safety.torque_cap = u16::from(backend.current_limit());
            let now_us = backend.elapsed_us();
            if safety.arm_safety_observation(now_us).is_err() {
                return fault_terminal(session, backend, None);
            }
            session.run_started_us = now_us;
            workspace.previous_encoder_raw = baseline;
            workspace.previous_read_us = now_us;
            *state = RobustnessReversalState::EstablishWriteGain;
            StepResult::ReadyNow
        }
        RobustnessReversalState::EstablishWriteGain => {
            let (p, i) = match accepted_gains(&session.point) {
                Ok(pair) => pair,
                Err(_) => return fault_terminal(session, backend, None),
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
                return fault_terminal(session, backend, None);
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
                Err(fault) => return tick_fault_terminal(session, backend, fault),
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
                        return fault_terminal(session, backend, None);
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
                return fault_terminal(session, backend, None);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let stationarity = match poll {
                Ok(None) => return StepResult::ReadyNow,
                Ok(Some((result, _, _))) => result,
                Err(_) => return fault_terminal(session, backend, None),
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
                Err(fault) => return tick_fault_terminal(session, backend, fault),
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
                        return fault_terminal(session, backend, None);
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
                return fault_terminal(session, backend, None);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let stationarity = match poll {
                Ok(None) => return StepResult::ReadyNow,
                Ok(Some((result, _, _))) => result,
                Err(_) => return fault_terminal(session, backend, None),
            };
            let direction = session.direction;
            match stationarity {
                StationarityResult::Usable(suffix) => {
                    let Some(reconvergence_time_us) = workspace.capture.band_entry_us() else {
                        retry_or_finalize_inconclusive(state, session);
                        return StepResult::ReadyNow;
                    };
                    let evidence = match score_reversal_leg(
                        &session.point,
                        direction,
                        safety.counts_per_rev(),
                        session.pending_forward_settle_us,
                        reconvergence_time_us,
                        workspace.capture.recovery_iae_q(),
                        session.retry_count[direction],
                        &suffix,
                    ) {
                        Ok(evidence) => evidence,
                        Err(_) => return fault_terminal(session, backend, None),
                    };
                    session.directions[direction] = Some(evidence);
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
                return fault_terminal(session, backend, None);
            }
            safety.enter_settle_phase();
            match open_zero_window(session, workspace, backend).await {
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
                Err(fault) => return tick_fault_terminal(session, backend, fault),
            };
            if read_us >= deadline_us {
                let Some(analysis_deadline_us) = analysis_deadline(read_us) else {
                    return fault_terminal(session, backend, None);
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
                return fault_terminal(session, backend, None);
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
                        if session.direction + 1 < DIRECTION_COUNT {
                            session.direction += 1;
                            *state = RobustnessReversalState::EstablishWriteGain;
                        } else {
                            *state = RobustnessReversalState::Assemble;
                        }
                        StepResult::ReadyNow
                    } else if retry_available {
                        // I4: a rejected rest re-opens one fresh bounded rest
                        // window before faulting (integral_plan.rs precedent);
                        // it neither silently proceeds nor immediately kills the
                        // run.
                        match open_zero_window(session, workspace, backend).await {
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
                        fault_terminal(session, backend, None)
                    }
                }
                Err(_) => fault_terminal(session, backend, None),
            }
        }
        RobustnessReversalState::Assemble => {
            *state = RobustnessReversalState::Done;
            complete_terminal(session, backend)
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
    let baseline = match backend.read_capture_register(ABN_DECODER_COUNT).await {
        Ok(value) => value,
        Err(_) => return Err(fault_terminal(session, backend, None)),
    };
    let baseline_us = backend.elapsed_us();
    if backend
        .write_register(PID_VELOCITY_TARGET, signed_target_rpm as u32)
        .await
        .is_err()
    {
        return Err(fault_terminal(session, backend, None));
    }
    if open_primitive {
        // The establish entry opens the bounded primitive; a retry re-opens after
        // the idempotent end. The reversal leg continues the open primitive and
        // never re-opens it.
        safety.end_primitive();
        if safety.begin_primitive(baseline_us).is_err() {
            return Err(fault_terminal(session, backend, None));
        }
    }
    workspace.observations.clear();
    workspace.analysis.reset();
    let target_rate = target_rate_q(signed_target_rpm, safety.counts_per_rev());
    let band = band_q_for(target_rate);
    workspace
        .capture
        .reset(target_rate, band, ROBUSTNESS_BAND_DWELL_US);
    workspace.previous_encoder_raw = baseline;
    workspace.previous_read_us = baseline_us;
    let Some(next_sample_us) = baseline_us.checked_add(CAPTURE_PERIOD_US) else {
        return Err(fault_terminal(session, backend, None));
    };
    let Some(budget_deadline_us) =
        baseline_us.checked_add(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US))
    else {
        return Err(fault_terminal(session, backend, None));
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

/// Retry the current direction from the establish leg, or finalize it as
/// inconclusive once the bounded retry budget is exhausted.
///
/// Retrying always re-enters at [`RobustnessReversalState::EstablishWriteGain`]
/// so every attempt begins from a freshly confirmed direction-A settle with a
/// cleared integrator, never from an unsettled leg.
fn retry_or_finalize_inconclusive(
    state: &mut RobustnessReversalState,
    session: &mut RobustnessReversalSession,
) {
    let direction = session.direction;
    if session.retry_count[direction] < ROBUSTNESS_MAX_RETRIES {
        session.retry_count[direction] += 1;
        *state = RobustnessReversalState::EstablishWriteGain;
    } else {
        session.directions[direction] =
            Some(inconclusive_direction(session.retry_count[direction]));
        *state = RobustnessReversalState::ZeroCommand {
            retry_available: true,
        };
    }
}

/// Open a fresh zero-target rest capture window without re-commanding the
/// target or re-entering the settle phase; the caller does that once per rest.
async fn open_zero_window<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    workspace: &mut RobustnessReversalWorkspace,
    backend: &mut B,
) -> Result<(u64, u64), StepResult> {
    let baseline = match backend.read_capture_register(ABN_DECODER_COUNT).await {
        Ok(value) => value,
        Err(_) => return Err(fault_terminal(session, backend, None)),
    };
    let baseline_us = backend.elapsed_us();
    workspace.observations.clear();
    workspace.analysis.reset_for_scored_rest(false);
    workspace.previous_encoder_raw = baseline;
    workspace.previous_read_us = baseline_us;
    let Some(next_sample_us) = baseline_us.checked_add(CAPTURE_PERIOD_US) else {
        return Err(fault_terminal(session, backend, None));
    };
    let Some(deadline_us) = baseline_us.checked_add(u64::from(ROBUSTNESS_MEASUREMENT_BUDGET_US))
    else {
        return Err(fault_terminal(session, backend, None));
    };
    Ok((next_sample_us, deadline_us))
}

/// Fixed 1 kHz capture cadence, in microseconds.
const CAPTURE_PERIOD_US: u64 = VELOCITY_SWEEP_SAMPLE_PERIOD_US as u64;

/// Bounded analysis deadline from a capture-completion timestamp.
fn analysis_deadline(read_us: u64) -> Option<u64> {
    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
}

/// Zeroed metrics for an inconclusive direction after retries are exhausted.
fn inconclusive_direction(retry_count: u8) -> RobustnessDirectionEvidence {
    RobustnessDirectionEvidence {
        metrics: ReversalMetrics {
            reconvergence_time_us: 0,
            settled_residual_q: 0,
            recovery_iae_q: 0,
            forward_settle_time_us: 0,
            reconvergence_ratio_ppm: 0,
        },
        tripped: 0,
        retry_count,
        inconclusive: true,
    }
}

/// Fill any direction that never recorded evidence with a zeroed placeholder.
fn assembled_evidence(session: &RobustnessReversalSession) -> RobustnessEvidence {
    let zero = inconclusive_direction(0);
    RobustnessEvidence {
        directions: [
            session.directions[0].unwrap_or(zero),
            session.directions[1].unwrap_or(zero),
        ],
    }
}

/// Build the successful/rejected/inconclusive terminal from recorded evidence.
fn complete_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
) -> StepResult {
    let evidence = assembled_evidence(session);
    let any_trip = evidence.directions.iter().any(|d| d.tripped != 0);
    let any_inconclusive = evidence.directions.iter().any(|d| d.inconclusive);
    let outcome = if any_trip {
        RobustnessOutcome::Rejected
    } else if any_inconclusive {
        RobustnessOutcome::Inconclusive
    } else {
        RobustnessOutcome::Complete
    };
    let cause = summary_cause(&evidence);
    StepResult::RobustnessReversalTerminal(RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence: session.run_sequence,
        outcome,
        cause,
        selected_p: session.point.selected_p,
        selected_i: session.point.selected_i,
        target_velocity_rpm: session.point.target_velocity_rpm,
        evidence,
        run_started_us: session.run_started_us,
        run_completed_us: backend.elapsed_us(),
        safety_fault: None,
    })
}

/// Build a `Failed` terminal. A populated `safety_fault` is a real envelope
/// trip (`ROBUSTNESS_CAUSE_SAFETY_FAULT`); `None` is an internal, non-envelope
/// fault (`ROBUSTNESS_CAUSE_INTERNAL_FAULT`) so the host can disambiguate them.
fn fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    let cause = if safety_fault.is_some() {
        ROBUSTNESS_CAUSE_SAFETY_FAULT
    } else {
        ROBUSTNESS_CAUSE_INTERNAL_FAULT
    };
    StepResult::RobustnessReversalTerminal(RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence: session.run_sequence,
        outcome: RobustnessOutcome::Failed,
        cause,
        selected_p: session.point.selected_p,
        selected_i: session.point.selected_i,
        target_velocity_rpm: session.point.target_velocity_rpm,
        evidence: assembled_evidence(session),
        run_started_us: session.run_started_us,
        run_completed_us: backend.elapsed_us(),
        safety_fault,
    })
}

/// Resolve a per-tick fault into the owning `Failed` terminal.
fn tick_fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    fault: TickFault,
) -> StepResult {
    match fault {
        TickFault::Io => fault_terminal(session, backend, None),
        TickFault::Safety(report) => fault_terminal(session, backend, Some(report)),
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
    let crate::commissioning::outer::OuterPhase::RobustnessReversal { session, .. } = phase else {
        return StepResult::OuterFailed(error);
    };
    let _ = error;
    let cause = if safety_fault.is_some() {
        ROBUSTNESS_CAUSE_SAFETY_FAULT
    } else {
        ROBUSTNESS_CAUSE_INTERNAL_FAULT
    };
    StepResult::RobustnessReversalTerminal(RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence: session.run_sequence,
        outcome: RobustnessOutcome::Failed,
        cause,
        selected_p: session.point.selected_p,
        selected_i: session.point.selected_i,
        target_velocity_rpm: session.point.target_velocity_rpm,
        evidence: assembled_evidence(session),
        run_started_us: session.run_started_us,
        run_completed_us: session.run_started_us,
        safety_fault,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::robustness::{
        ROBUSTNESS_CAUSE_PASS, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE,
        ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::outer::OuterMotionBudget;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptanceMatrixProgress, StageCCandidateSlot, StageCPlanIdentity,
    };
    use crate::commissioning::outer::velocity::robustness_metrics::TRIP_TIME;
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
    ) -> (RobustnessReversalTerminal, MockBackend) {
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
                    return (terminal, backend);
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
    ) -> (RobustnessReversalTerminal, MockBackend) {
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
        let (terminal, _backend) = drive(test_point(), false);
        assert_eq!(terminal.outcome, RobustnessOutcome::Complete);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_PASS);
        assert!(terminal.safety_fault.is_none());
        assert_eq!(terminal.run_sequence, 7);
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_eq!(direction.tripped, 0);
            assert_eq!(direction.retry_count, 0);
        }
    }

    #[test]
    fn noisy_reversal_exhausts_retries_and_reports_inconclusive() {
        let (terminal, _backend) = drive(test_point(), true);
        assert_eq!(terminal.outcome, RobustnessOutcome::Inconclusive);
        assert_eq!(terminal.cause, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE);
        assert!(terminal.safety_fault.is_none());
        for direction in terminal.evidence.directions {
            assert!(direction.inconclusive);
            assert_eq!(direction.retry_count, ROBUSTNESS_MAX_RETRIES);
        }
    }

    /// The load-bearing invariant: the integrator is cleared exactly once per
    /// establish-cruise leg and never on the measured reversal leg. A stray
    /// clear on the reversal would silently re-run identification and make the
    /// gate vacuous.
    #[test]
    fn integrator_clears_only_on_establish_never_on_reversal() {
        // Clean run: one establish clear per direction, no retries -> two clears.
        let (_terminal, backend) = drive(test_point(), false);
        assert_eq!(
            backend.pid_velocity_error_sum_clears, 2,
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

        // Noisy run: one establish clear per attempt (1 + ROBUSTNESS_MAX_RETRIES
        // per direction), still zero clears on any reversal leg.
        let (_terminal, noisy_backend) = drive(test_point(), true);
        let attempts_per_direction = usize::from(ROBUSTNESS_MAX_RETRIES) + 1;
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

            let (terminal, _backend) = drive(point, noisy);
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
    /// (300 ms) and the 500 ms budget -- now yields a real `TRIP_TIME` +
    /// `Rejected` verdict. Under the old fixed 100 ms window this was
    /// physically unreachable: the window ended before band entry, aliasing the
    /// leg to Inconclusive.
    #[test]
    fn slow_reconvergence_trips_time_and_rejects() {
        let (terminal, _backend) = drive_scripted(test_point(), |phase, tick, target| {
            if target == 0 {
                return 0;
            }
            match phase {
                // Stall out of band well past t_max, then converge in-band.
                CapturePhase::Reverse if tick < 330 => 0,
                _ => target.signum() * STEADY_DELTA,
            }
        });
        assert_eq!(terminal.outcome, RobustnessOutcome::Rejected);
        assert!(terminal.safety_fault.is_none());
        for direction in terminal.evidence.directions {
            assert!(!direction.inconclusive);
            assert_ne!(
                direction.tripped & TRIP_TIME,
                0,
                "slow reconvergence must trip the time gate"
            );
            assert!(
                direction.metrics.reconvergence_time_us > ROBUSTNESS_THRESHOLDS.t_max_us,
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
        let (terminal, _backend) = drive_scripted(test_point(), move |phase, tick, target| {
            match phase {
                // Direction 0 reversal (negative target): stall past t_max, then
                // converge in band -> a real TRIP_TIME rejection.
                CapturePhase::Reverse if target < 0 => {
                    if tick < 330 {
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
        assert_ne!(
            dir0.tripped & TRIP_TIME,
            0,
            "direction 0 must trip the time gate"
        );
        assert!(dir1.inconclusive, "direction 1 must be inconclusive");
        assert_eq!(
            dir1.tripped, 0,
            "an inconclusive direction reports no trips"
        );
    }

    /// I3: an establish leg whose settled tail fails stationarity (Excluded)
    /// must retry from a fresh entry, counting against the retry budget, rather
    /// than silently proceeding to measure the reversal against an unconfirmed
    /// settle.
    #[test]
    fn excluded_establish_settle_retries_and_counts() {
        let (terminal, _backend) = drive_scripted(test_point(), |phase, tick, target| {
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
        let (terminal, _backend) = drive_scripted(test_point(), move |phase, tick, target| {
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
    /// the run rather than silently proceeding.
    #[test]
    fn rest_rejected_beyond_retry_faults() {
        let (terminal, _backend) =
            drive_scripted(test_point(), |phase, _tick, target| match phase {
                CapturePhase::Zero => 8,
                _ if target != 0 => target.signum() * STEADY_DELTA,
                _ => 0,
            });
        assert_eq!(terminal.outcome, RobustnessOutcome::Failed);
    }
}
