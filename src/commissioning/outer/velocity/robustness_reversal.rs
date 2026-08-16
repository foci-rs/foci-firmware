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
use crate::commission_dispatch::robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::OuterSafetyLimits;
use crate::commissioning::outer::state::VelocityObservationRing;
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocitySettleMetrics};
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
    ROBUSTNESS_THRESHOLDS, ReversalMetrics, band_entry_time_us, recovery_iae_q, tripped_mask,
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

/// Fixed moving-capture window for one leg, in microseconds. Chosen to collect
/// well over the 64-sample shortest stationary suffix at the ~1 ms observation
/// cadence while leaving room, together with the analysis budget, under
/// `ROBUSTNESS_MEASUREMENT_BUDGET_US` for the measured reversal leg.
const ROBUSTNESS_LEG_CAPTURE_US: u32 = 100_000;

/// Confidence family size for the settled-rate interval reconstruction, matching
/// the shared scored-rest reconstruction default.
const ROBUSTNESS_CONFIDENCE_FAMILY_SIZE: u16 = 3;

/// Number of reversal directions probed: forward->reverse and reverse->forward.
const DIRECTION_COUNT: usize = 2;

/// Borrowed single-leg scratch space: capture ring, stationarity analysis,
/// dyadic blocker, establish-leg settle metrics, and the running encoder/time
/// anchors. Never allocates on the heap; every field is fixed-size.
#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct RobustnessReversalWorkspace {
    observations: VelocityObservationRing,
    blocker: DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    analysis: StationarityWork,
    establish_settle: VelocitySettleMetrics,
    previous_encoder_raw: u32,
    previous_read_us: u64,
}

impl RobustnessReversalWorkspace {
    /// Construct an empty workspace.
    #[allow(dead_code)]
    pub(crate) const fn new() -> Self {
        Self {
            observations: VelocityObservationRing::new(),
            blocker: DyadicBlocker::new(),
            analysis: StationarityWork::new(),
            establish_settle: VelocitySettleMetrics::EMPTY,
            previous_encoder_raw: 0,
            previous_read_us: 0,
        }
    }
}

/// Frozen accepted point plus the mutable per-direction robustness progress.
#[allow(dead_code)]
#[derive(Debug)]
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
    #[allow(dead_code)]
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
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RobustnessReversalState {
    /// Arm the safety baseline before any motion.
    Setup,
    /// Apply the accepted gains, ramp mode, and clear the integrator (entry leg).
    EstablishWriteGain,
    /// Command the signed establish-cruise target for the current direction.
    EstablishCommandTarget,
    /// Collect the establish-cruise capture until its window elapses.
    EstablishCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    /// Analyze the establish capture until it yields a stationary settle.
    EstablishAnalyze { analysis_deadline_us: u64 },
    /// Reverse the commanded target WITHOUT clearing the integrator.
    ReverseCommandTarget,
    /// Collect the measured reversal capture until its window elapses.
    ReverseCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    /// Analyze the reversal capture for band-entry reconvergence.
    ReverseAnalyze { analysis_deadline_us: u64 },
    /// Command zero and open the between-legs rest window.
    ZeroCommand,
    /// Collect the zero-target rest capture until its window elapses.
    ZeroCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    /// Confirm rest between legs via scored-rest analysis.
    ZeroAnalyze { analysis_deadline_us: u64 },
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

/// Chronological observation index whose cumulative elapsed reaches
/// `band_entry_us`, so `recovery_iae_q` integrates `[0, index)` up to band entry.
fn band_entry_index(observations: &VelocityObservationRing, band_entry_us: u32) -> usize {
    use crate::autotune::velocity_measurement::VelocityObservationSource;
    let mut elapsed_before: u32 = 0;
    for index in 0..observations.observation_len() {
        if elapsed_before >= band_entry_us {
            return index;
        }
        let Some(sample) = observations.observation(index) else {
            return index;
        };
        elapsed_before = elapsed_before.saturating_add(sample.elapsed_us);
    }
    observations.observation_len()
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

/// Score one measured reversal leg from its completed capture and settled suffix.
fn score_reversal_leg(
    workspace: &RobustnessReversalWorkspace,
    point: &AcceptancePoint,
    direction: usize,
    counts_per_rev: u32,
    forward_settle_time_us: u32,
    retry_count: u8,
    suffix: &crate::autotune::velocity_measurement::StationarySuffix,
) -> Result<Option<RobustnessDirectionEvidence>, CommissionError> {
    let signed_target_rpm = reverse_target_rpm(point, direction);
    let target_rate = target_rate_q(signed_target_rpm, counts_per_rev);
    let band = band_q_for(target_rate);
    let Some(band_entry) = band_entry_time_us(
        &workspace.observations,
        target_rate,
        band,
        ROBUSTNESS_BAND_DWELL_US,
    ) else {
        // Stationary but never dwelled inside the settle band: inconclusive.
        return Ok(None);
    };
    let end_index = band_entry_index(&workspace.observations, band_entry);
    let recovery_iae = recovery_iae_q(&workspace.observations, target_rate, end_index);
    let (settled_low, settled_high) = settled_rate_interval_q(suffix)?;
    let (deficit_low, deficit_high) =
        deficit_interval_q(signed_target_rpm, counts_per_rev, settled_low, settled_high)?;
    let settled_residual_q = deficit_low.saturating_add(deficit_high) / 2;
    let mut metrics = ReversalMetrics {
        reconvergence_time_us: band_entry,
        settled_residual_q,
        recovery_iae_q: recovery_iae,
        forward_settle_time_us,
        reconvergence_ratio_ppm: 0,
    };
    metrics.reconvergence_ratio_ppm = reconvergence_ratio_ppm(band_entry, forward_settle_time_us);
    let tripped = tripped_mask(&metrics, &ROBUSTNESS_THRESHOLDS);
    Ok(Some(RobustnessDirectionEvidence {
        metrics,
        tripped,
        retry_count,
        inconclusive: false,
    }))
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
#[allow(dead_code)]
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
                Ok((next_sample_us, deadline_us)) => {
                    workspace.establish_settle = VelocitySettleMetrics::EMPTY;
                    *state = RobustnessReversalState::EstablishCapture {
                        next_sample_us,
                        deadline_us,
                    };
                    StepResult::At(next_sample_us)
                }
                Err(step) => step,
            }
        }
        RobustnessReversalState::EstablishCapture {
            next_sample_us,
            deadline_us,
        } => {
            if backend.elapsed_us() < next_sample_us {
                return StepResult::At(next_sample_us);
            }
            let (read_us, elapsed_us, delta) = match observe_tick(workspace, safety, backend).await
            {
                Ok(sample) => sample,
                Err(fault) => return tick_fault_terminal(session, backend, fault),
            };
            if workspace
                .establish_settle
                .observe(delta, elapsed_us)
                .is_err()
            {
                return fault_terminal(session, backend, None);
            }
            if read_us >= deadline_us {
                workspace.analysis.reset();
                let Some(analysis_deadline_us) = analysis_deadline(read_us) else {
                    return fault_terminal(session, backend, None);
                };
                *state = RobustnessReversalState::EstablishAnalyze {
                    analysis_deadline_us,
                };
                StepResult::ReadyNow
            } else {
                let next = (next_sample_us + CAPTURE_PERIOD_US).min(deadline_us);
                *state = RobustnessReversalState::EstablishCapture {
                    next_sample_us: next,
                    deadline_us,
                };
                StepResult::At(next)
            }
        }
        RobustnessReversalState::EstablishAnalyze {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= analysis_deadline_us {
                return fault_terminal(session, backend, None);
            }
            match poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(None) => StepResult::ReadyNow,
                Ok(Some(_)) => {
                    // The establish leg only needs a completed analysis to
                    // anchor its settle time; both Usable and Excluded settle
                    // metrics carry the same duration. The measured leg is the
                    // reversal, scored separately below.
                    session.pending_forward_settle_us = workspace.establish_settle.duration_us;
                    *state = RobustnessReversalState::ReverseCommandTarget;
                    StepResult::ReadyNow
                }
                Err(_) => fault_terminal(session, backend, None),
            }
        }
        RobustnessReversalState::ReverseCommandTarget => {
            let target = reverse_target_rpm(&session.point, session.direction);
            // Deliberately no clear_velocity_integrator here: the reversal is
            // measured against the integrator state established by the entry leg.
            match begin_capture(session, workspace, safety, backend, target, false).await {
                Ok((next_sample_us, deadline_us)) => {
                    *state = RobustnessReversalState::ReverseCapture {
                        next_sample_us,
                        deadline_us,
                    };
                    StepResult::At(next_sample_us)
                }
                Err(step) => step,
            }
        }
        RobustnessReversalState::ReverseCapture {
            next_sample_us,
            deadline_us,
        } => {
            if backend.elapsed_us() < next_sample_us {
                return StepResult::At(next_sample_us);
            }
            let (read_us, _elapsed_us, _delta) =
                match observe_tick(workspace, safety, backend).await {
                    Ok(sample) => sample,
                    Err(fault) => return tick_fault_terminal(session, backend, fault),
                };
            if read_us >= deadline_us {
                workspace.analysis.reset();
                let Some(analysis_deadline_us) = analysis_deadline(read_us) else {
                    return fault_terminal(session, backend, None);
                };
                *state = RobustnessReversalState::ReverseAnalyze {
                    analysis_deadline_us,
                };
                StepResult::ReadyNow
            } else {
                let next = (next_sample_us + CAPTURE_PERIOD_US).min(deadline_us);
                *state = RobustnessReversalState::ReverseCapture {
                    next_sample_us: next,
                    deadline_us,
                };
                StepResult::At(next)
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
            let scored = match stationarity {
                StationarityResult::Usable(suffix) => match score_reversal_leg(
                    workspace,
                    &session.point,
                    direction,
                    safety.counts_per_rev(),
                    session.pending_forward_settle_us,
                    session.retry_count[direction],
                    &suffix,
                ) {
                    Ok(evidence) => evidence,
                    Err(_) => return fault_terminal(session, backend, None),
                },
                StationarityResult::Excluded(_) => None,
            };
            match scored {
                Some(evidence) => {
                    session.directions[direction] = Some(evidence);
                    *state = RobustnessReversalState::ZeroCommand;
                    StepResult::ReadyNow
                }
                None => {
                    // Inconclusive: retry from the establish-cruise leg so every
                    // attempt begins from a freshly confirmed direction-A settle,
                    // never from the unsettled reversal state.
                    if session.retry_count[direction] < ROBUSTNESS_MAX_RETRIES {
                        session.retry_count[direction] += 1;
                        *state = RobustnessReversalState::EstablishWriteGain;
                    } else {
                        session.directions[direction] =
                            Some(inconclusive_direction(session.retry_count[direction]));
                        *state = RobustnessReversalState::ZeroCommand;
                    }
                    StepResult::ReadyNow
                }
            }
        }
        RobustnessReversalState::ZeroCommand => {
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return fault_terminal(session, backend, None);
            }
            safety.enter_settle_phase();
            let baseline = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fault_terminal(session, backend, None),
            };
            let baseline_us = backend.elapsed_us();
            workspace.observations.clear();
            workspace.analysis.reset_for_scored_rest(false);
            workspace.previous_encoder_raw = baseline;
            workspace.previous_read_us = baseline_us;
            let Some(next_sample_us) = baseline_us.checked_add(CAPTURE_PERIOD_US) else {
                return fault_terminal(session, backend, None);
            };
            let Some(deadline_us) = baseline_us.checked_add(u64::from(ROBUSTNESS_LEG_CAPTURE_US))
            else {
                return fault_terminal(session, backend, None);
            };
            *state = RobustnessReversalState::ZeroCapture {
                next_sample_us,
                deadline_us,
            };
            StepResult::At(next_sample_us)
        }
        RobustnessReversalState::ZeroCapture {
            next_sample_us,
            deadline_us,
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
                };
                StepResult::ReadyNow
            } else {
                let next = (next_sample_us + CAPTURE_PERIOD_US).min(deadline_us);
                *state = RobustnessReversalState::ZeroCapture {
                    next_sample_us: next,
                    deadline_us,
                };
                StepResult::At(next)
            }
        }
        RobustnessReversalState::ZeroAnalyze {
            analysis_deadline_us,
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
                Ok(ScoredRestPoll::Complete(_)) => {
                    safety.end_primitive();
                    if session.direction + 1 < DIRECTION_COUNT {
                        session.direction += 1;
                        *state = RobustnessReversalState::EstablishWriteGain;
                    } else {
                        *state = RobustnessReversalState::Assemble;
                    }
                    StepResult::ReadyNow
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
    workspace.previous_encoder_raw = baseline;
    workspace.previous_read_us = baseline_us;
    let Some(next_sample_us) = baseline_us.checked_add(CAPTURE_PERIOD_US) else {
        return Err(fault_terminal(session, backend, None));
    };
    let Some(deadline_us) = baseline_us.checked_add(u64::from(ROBUSTNESS_LEG_CAPTURE_US)) else {
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

/// Build a `Failed` terminal for an internal (non-safety) fault.
fn fault_terminal<B: CommissioningBackend>(
    session: &RobustnessReversalSession,
    backend: &B,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    StepResult::RobustnessReversalTerminal(RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence: session.run_sequence,
        outcome: RobustnessOutcome::Failed,
        cause: ROBUSTNESS_CAUSE_SAFETY_FAULT,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::robustness::{
        ROBUSTNESS_CAUSE_PASS, ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::outer::OuterMotionBudget;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptanceMatrixProgress, StageCCandidateSlot, StageCPlanIdentity,
    };
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

    /// Drive one full run to its terminal against a scripted `MockBackend`.
    ///
    /// `noisy_reversal` decides whether the measured reversal legs settle
    /// cleanly (`false`) or jitter so wildly they never enter the band
    /// (`true`). The establish legs always settle so every attempt begins from
    /// a confirmed direction-A settle; the zero-target rest window always holds
    /// still and confirms rest.
    fn drive(
        point: AcceptancePoint,
        noisy_reversal: bool,
    ) -> (RobustnessReversalTerminal, MockBackend) {
        let mut state = RobustnessReversalState::Setup;
        let mut workspace = RobustnessReversalWorkspace::new();
        let mut session = RobustnessReversalSession::new(point, 7);
        let mut safety = test_safety();
        let mut backend = test_backend();

        let mut encoder_count: i32 = 0;
        let mut jitter_high = false;

        for _ in 0..STEP_BUDGET {
            let in_reverse_capture =
                matches!(state, RobustnessReversalState::ReverseCapture { .. });
            let result = pollster::block_on(step_robustness_reversal(
                &mut state,
                &mut workspace,
                &mut session,
                &mut safety,
                &mut backend,
            ));
            match result {
                StepResult::ReadyNow => {}
                StepResult::At(deadline) => {
                    backend.elapsed_us_value = deadline;
                    let target = *backend
                        .registers
                        .get(&PID_VELOCITY_TARGET.addr())
                        .unwrap_or(&0) as i32;
                    if target != 0 {
                        if noisy_reversal && in_reverse_capture {
                            jitter_high = !jitter_high;
                            encoder_count += if jitter_high { 300 } else { -300 };
                        } else {
                            encoder_count += target.signum() * STEADY_DELTA;
                        }
                    }
                    // Model the TMC4671 decoder register, which wraps within
                    // [0, counts_per_rev); feeding raw signed counts would alias
                    // against EncoderDomain's u32 modulo at the 2^32 boundary.
                    backend.set_abn_decoder_count(encoder_count.rem_euclid(4_000));
                }
                StepResult::RobustnessReversalTerminal(terminal) => {
                    return (terminal, backend);
                }
                other => panic!("unexpected step result: {other:?}"),
            }
        }
        panic!("robustness reversal run exceeded its bounded step budget");
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
}
