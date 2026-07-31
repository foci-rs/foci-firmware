//! Breakaway probe-phase driver: two independent bisection searches locate the
//! per-direction breakaway gain through the existing guarded stroke and
//! origin-recovery primitives.
//!
//! This module drives motion. It reuses the lower-level primitives the trace-gated
//! velocity sweep (`sweep.rs`) is built from -- [`current_guard`](super::current_guard),
//! [`recovery`](super::recovery), and [`sweep_analysis`](super::sweep_analysis)'s
//! `poll_stationarity`/`poll_scored_rest_confirmation`/`assemble_velocity_observation_input`
//! -- and the sweep's own classifier via
//! [`classify_breakaway_observation`](super::breakaway_search::classify_breakaway_observation),
//! so the probe's fault surface is provably identical to the sweep's. It does not
//! copy or re-derive the sweep's rung/slot-folding state machine: a probe stroke is a
//! single direction, single observation, with no operating-region or handoff
//! bookkeeping.
//!
//! Nothing here decides how the driver is wired into [`super::super::OuterPhase`];
//! that is a later slice's job. This module exposes a self-contained, single-tick
//! cooperative stepper -- [`step_breakaway_probe`] -- that a caller (a future engine
//! phase, or a test harness) drives exactly like [`super::sweep::step_velocity_sweep_with_trace`]:
//! poll until [`BreakawayProbeStep::Done`], running the host-declared periodic safety
//! envelope check ([`crate::commissioning::outer::check_outer_safety`]) between ticks,
//! same as every other outer phase.
//!
//! Nothing outside this module's own tests calls into it yet -- wiring it into
//! [`super::super::OuterPhase`] is a later slice's job (see the module-level
//! docs above) -- so, matching [`super::current_guard`]'s own
//! `#![cfg_attr(not(test), allow(dead_code))]`, dead-code warnings are suppressed
//! outside test builds.
#![cfg_attr(not(test), allow(dead_code))]

use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use crate::autotune::velocity_measurement::{DyadicBlocker, StationarityWork, VelocityDirection};
use crate::autotune::{CommissionError, TmcMechanicalRpm};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::OuterSafetyLimits;
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocityObservationRing};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_CURRENT, OUTER_SAFETY_FAULT_OBSERVATION_GAP,
    OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault,
};

use super::breakaway_plan::{BreakawayProbePlan, PROBE_CONFIDENCE_FAMILY_SIZE};
use super::breakaway_search::{
    BreakawayMotionClass, BreakawaySearchCursor, BreakawaySearchError, BreakawaySearchStep,
    DirectionalBreakaway, ResolvedCampaignBreakaway, classify_breakaway_observation,
    resolve_campaign_breakaway,
};
use super::current_guard::{
    VelocityCurrentClampSession, VelocityCurrentGuardError, VelocityCurrentObserver,
    VelocityCurrentZeroPhase, VelocityLimitEvidenceCapability, begin_velocity_current_primitive,
    finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    read_velocity_pid_input_abs, read_velocity_pid_output_abs,
};
use super::recovery::{
    RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryRestSchedule, RecoveryStage,
    RungOriginRecoveryConfig, RungOriginRecoveryResources, RungOriginRecoveryRuntime,
    RungOriginRecoveryStep, commanded_side_lower_rate_q, plan_rung_origin_recovery,
    step_rung_origin_recovery,
};
use super::sweep::clear_velocity_integrator;
use super::sweep_analysis::{
    ScoredRestPoll, VelocityObservationInputRequest, assemble_velocity_observation_input,
    bounded_rest_displacement, direction_sign, poll_scored_rest_confirmation, poll_stationarity,
};
use super::sweep_evidence::VelocityObservationClass;
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US,
    SETTLE_ANALYSIS_BUDGET_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US, ZERO_SETTLE_US,
};

/// Outer velocity ramp mode. Matches the wire encoding
/// [`super::sweep::step_velocity_sweep_with_trace`] writes to `MODE_RAMP_MODE_MOTION`;
/// duplicated here because that constant is private to the `sweep` module.
const PROBE_VELOCITY_MODE: u32 = 0x8000_0002;

/// Borrowed single-stroke scratch space: capture ring, stationarity analysis, and
/// current-clamp evidence for one guarded stroke at a time.
///
/// Sized and shaped like the corresponding fields of
/// [`super::super::VelocitySweepData`] so a future caller can carve this out of the
/// same workspace rather than allocating a second one. This type never allocates on
/// the heap; every field is fixed-size.
#[derive(Debug, defmt::Format)]
pub(crate) struct BreakawayProbeWorkspace {
    previous_encoder_raw: u32,
    previous_read_us: u64,
    current_observer: VelocityCurrentObserver,
    settle_displacement_counts: i8,
    settle_min_displacement_counts: i8,
    settle_max_displacement_counts: i8,
    #[defmt(Debug2Format)]
    observations: VelocityObservationRing,
    #[defmt(Debug2Format)]
    blocker: DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    #[defmt(Debug2Format)]
    analysis: StationarityWork,
}

impl BreakawayProbeWorkspace {
    /// Construct an empty workspace.
    pub(crate) const fn new() -> Self {
        Self {
            previous_encoder_raw: 0,
            previous_read_us: 0,
            current_observer: VelocityCurrentObserver::EMPTY,
            settle_displacement_counts: 0,
            settle_min_displacement_counts: 0,
            settle_max_displacement_counts: 0,
            observations: VelocityObservationRing::new(),
            blocker: DyadicBlocker::new(),
            analysis: StationarityWork::new(),
        }
    }
}

/// Frozen campaign inputs and the mutable per-direction search progress.
///
/// Bundles the immutable [`BreakawayProbePlan`] together with the two independent
/// [`BreakawaySearchCursor`]s so [`step_breakaway_probe`] stays within the five
/// parameter guideline. Constructed once, before the first stroke, via
/// [`BreakawayProbeSession::new`].
#[derive(Debug, defmt::Format)]
pub(crate) struct BreakawayProbeSession {
    plan: BreakawayProbePlan,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    direction: VelocityDirection,
    cursor: BreakawaySearchCursor,
    pending_index: u8,
    forward_result: Option<DirectionalBreakaway>,
    pending_class: Option<Result<BreakawayMotionClass, BreakawaySearchError>>,
    pending_lower_rate_q: Option<u64>,
}

impl BreakawayProbeSession {
    /// Freeze the campaign inputs and start the forward-direction cursor.
    ///
    /// This never touches `backend`: constructing a session is pure. Motion is only
    /// reachable through [`step_breakaway_probe`], and only once its state has left
    /// [`BreakawayProbeState::PlannedProbe`].
    pub(crate) fn new(
        plan: BreakawayProbePlan,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
    ) -> Result<Self, BreakawaySearchError> {
        let (cursor, pending_index) = BreakawaySearchCursor::start(plan.search_count)?;
        Ok(Self {
            plan,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            direction: VelocityDirection::Forward,
            cursor,
            pending_index,
            forward_result: None,
            pending_class: None,
            pending_lower_rate_q: None,
        })
    }

    fn pending_gain(&self) -> Result<PidPGain, BreakawaySearchError> {
        if self.pending_index >= self.plan.search_count {
            return Err(BreakawaySearchError::InvalidRequest);
        }
        self.plan
            .search_gains
            .get(usize::from(self.pending_index))
            .copied()
            .ok_or(BreakawaySearchError::InvalidRequest)
    }
}

/// Single-tick cooperative stroke cycle for one probe observation.
///
/// Mirrors [`super::sweep::step_velocity_sweep_with_trace`]'s stroke states
/// (write gain, command target, capture, command zero, settle-capture, analyze) at
/// probe scale: one direction, one observation, no rung/slot folding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum BreakawayProbeState {
    /// The frozen plan and cursors are admitted; no stroke has been issued yet.
    PlannedProbe,
    /// Write the search-grid gain and the outer-velocity ramp mode.
    WriteGain,
    /// Arm the current clamp and command the signed velocity target.
    CommandTarget,
    /// Sample the commanded stroke until its fixed moving duration elapses.
    Capture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    /// Command zero and record the moving-phase current evidence boundary.
    CommandZero,
    /// Classify the moving-phase stationarity result.
    AnalyzeMoving {
        settle_deadline_us: u64,
        analysis_deadline_us: u64,
    },
    /// Read the settle baseline before sampling the zero-target rest window.
    BeginSettleCapture { settle_deadline_us: u64 },
    /// Sample the zero-target rest window.
    SettleCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    /// Confirm rest and record the zero-phase current evidence boundary.
    AnalyzeSettle { analysis_deadline_us: u64 },
    /// Return to the parked origin through the shared recovery primitive.
    Recover(RungOriginRecoveryRuntime),
    /// Fold the classified outcome into the active cursor and decide what is next.
    Advance,
}

/// One cooperative result from [`step_breakaway_probe`].
#[derive(Debug, defmt::Format)]
pub(crate) enum BreakawayProbeStep {
    /// Call again immediately; no external wait is needed.
    ReadyNow,
    /// Call again no earlier than this `elapsed_us` deadline.
    At(u64),
    /// The probe campaign concluded.
    Done(BreakawayProbeOutcome),
}

/// Terminal result of a complete probe campaign (both directions).
///
/// Every variant other than [`Self::BreakawayFound`] is a probe-owned terminal: the
/// caller must not advance toward discovery. The probe never asserts
/// `set_enable_pin` itself, matching every other outer velocity phase (sweep,
/// integral): arming happens once, earlier, in `MechanicalId`, and centralized
/// safe-state restoration on any outer-phase terminal happens at the surrounding
/// engine boundary (`commissioning::mod::restore_safe_state_via_backend`), not
/// per-phase. A caller wiring these terminals into that boundary gets safe-disarm
/// for free, the same way `sweep_fault_terminal` already does.
#[derive(Debug, defmt::Format)]
pub(crate) enum BreakawayProbeOutcome {
    /// Both directional breakaways resolved and agreed (or differed by exactly one
    /// grid step, resolved to the lower endpoint).
    BreakawayFound(ResolvedCampaignBreakaway),
    /// A bisection search reached the top of its grid without ever confirming
    /// commanded-side motion.
    NoMotionAtAuthority,
    /// The mandatory bracket-repeat observation contradicted the search's own prior
    /// read at that grid index.
    UnstableBreakaway,
    /// The two directional breakaways differed by more than one grid step.
    DirectionalBreakawayDisagreement,
    /// Target-independent stationarity evidence, or another controller-domain
    /// evidence field, was unusable for classification -- or the completed
    /// origin-recovery return could not confirm rest.
    ProbeEvidenceExcluded,
    /// The observation reached the inclusive usable-current boundary before a
    /// motion or inert verdict was reached.
    AuthorityFailure,
    /// An existing safety, current, travel, encoder, or register-I/O fault,
    /// returned unchanged.
    SafetyFault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

fn fault(error: CommissionError) -> BreakawayProbeStep {
    BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault {
        error,
        safety: None,
    })
}

fn safety_fault(safety: OuterSafetyFault, error: CommissionError) -> BreakawayProbeStep {
    BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault {
        error,
        safety: Some(safety),
    })
}

fn current_guard_fault(error: VelocityCurrentGuardError) -> BreakawayProbeStep {
    fault(match error {
        VelocityCurrentGuardError::InvalidLimit => CommissionError::InvalidSchedule,
        VelocityCurrentGuardError::RegisterIo => CommissionError::Spi,
        VelocityCurrentGuardError::ActionableStatus(_) => CommissionError::SafetyEnvelopeViolation,
    })
}

/// Map a resolved [`BreakawaySearchError`] to its owning probe outcome.
fn search_error_outcome(error: BreakawaySearchError) -> BreakawayProbeStep {
    match error {
        BreakawaySearchError::InvalidRequest | BreakawaySearchError::Arithmetic => {
            fault(CommissionError::InvalidSchedule)
        }
        BreakawaySearchError::SafetyFault(class) => fault(match class {
            VelocityObservationClass::WrongDirectionFault => {
                CommissionError::VelocityValidationFailed
            }
            VelocityObservationClass::HardClipFault => CommissionError::SafetyEnvelopeViolation,
            VelocityObservationClass::Eligible
            | VelocityObservationClass::Inert
            | VelocityObservationClass::DeficitUnresolved
            | VelocityObservationClass::CurrentHeadroom
            | VelocityObservationClass::StationarityExcluded(_)
            | VelocityObservationClass::InvalidEvidenceFault => {
                unreachable!(
                    "classify_breakaway_observation only wraps WrongDirectionFault or \
                     HardClipFault in BreakawaySearchError::SafetyFault"
                )
            }
        }),
        BreakawaySearchError::ProbeEvidenceExcluded => {
            BreakawayProbeStep::Done(BreakawayProbeOutcome::ProbeEvidenceExcluded)
        }
        BreakawaySearchError::AuthorityFailure => {
            BreakawayProbeStep::Done(BreakawayProbeOutcome::AuthorityFailure)
        }
        BreakawaySearchError::NoMotionAtAuthority => {
            BreakawayProbeStep::Done(BreakawayProbeOutcome::NoMotionAtAuthority)
        }
        BreakawaySearchError::UnstableBreakaway => {
            BreakawayProbeStep::Done(BreakawayProbeOutcome::UnstableBreakaway)
        }
        BreakawaySearchError::DirectionalBreakawayDisagreement => {
            BreakawayProbeStep::Done(BreakawayProbeOutcome::DirectionalBreakawayDisagreement)
        }
    }
}

/// Advance the probe by one cooperative step.
///
/// `workspace` is caller-owned scratch space (see [`BreakawayProbeWorkspace`]);
/// `session` is caller-owned campaign state (see [`BreakawayProbeSession`]). Neither
/// is allocated here. The caller is responsible for running
/// [`crate::commissioning::outer::check_outer_safety`] against `safety` between
/// ticks, exactly as `step_outer_with_trace` does for every other outer phase; this
/// function never performs that check itself, matching
/// [`super::sweep::step_velocity_sweep_with_trace`].
pub(crate) async fn step_breakaway_probe<B: CommissioningBackend>(
    state: &mut BreakawayProbeState,
    workspace: &mut BreakawayProbeWorkspace,
    session: &mut BreakawayProbeSession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> BreakawayProbeStep {
    match state {
        BreakawayProbeState::PlannedProbe => {
            *state = BreakawayProbeState::WriteGain;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::WriteGain => {
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, PROBE_VELOCITY_MODE)
                    .await
                    .is_err()
                || clear_velocity_integrator(backend).await.is_err()
            {
                return fault(CommissionError::Spi);
            }
            *state = BreakawayProbeState::CommandTarget;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::CommandTarget => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            let target = match session
                .target_velocity_rpm
                .with_direction(direction_sign(session.direction))
            {
                Ok(target) => target.raw(),
                Err(_) => return fault(CommissionError::InvalidSchedule),
            };
            let hard_limit = u16::from(backend.current_limit());
            let mut clamp_session =
                match VelocityCurrentClampSession::try_new(u32::from(hard_limit)) {
                    Ok(session) => session,
                    Err(error) => return current_guard_fault(error),
                };
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return current_guard_fault(error);
            }
            let clamp_limit = clamp_session.probe_limit();
            if let Err(error) = begin_velocity_current_primitive(
                &mut workspace.current_observer,
                backend,
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            )
            .await
            {
                return current_guard_fault(error);
            }
            if backend
                .write_register(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
                || safety.begin_primitive(backend.elapsed_us()).is_err()
            {
                return fault(CommissionError::Spi);
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fault(CommissionError::InvalidSchedule);
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(MOVING_STROKE_US)) else {
                return fault(CommissionError::InvalidSchedule);
            };
            workspace.observations.clear();
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            *state = BreakawayProbeState::Capture {
                next_sample_us,
                deadline_us,
            };
            BreakawayProbeStep::At(next_sample_us)
        }
        BreakawayProbeState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BreakawayProbeStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend).await {
                Ok(read_us) => read_us,
                Err(result) => return result,
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_moving_output(value),
                Err(error) => return current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                *state = BreakawayProbeState::CommandZero;
                BreakawayProbeStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BreakawayProbeStep::At(*next_sample_us)
                    }
                    Err(error) => fault(error),
                }
            }
        }
        BreakawayProbeState::CommandZero => {
            let boundary =
                finish_velocity_current_moving_phase(&mut workspace.current_observer, backend)
                    .await;
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            if let Err(error) = boundary {
                return current_guard_fault(error);
            }
            if clear_velocity_integrator(backend).await.is_err() {
                return fault(CommissionError::Spi);
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) = now_us.checked_add(u64::from(ZERO_SETTLE_US)) else {
                return fault(CommissionError::InvalidSchedule);
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
            else {
                return fault(CommissionError::InvalidSchedule);
            };
            workspace.analysis.reset_with_structured_diagnostics();
            *state = BreakawayProbeState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return fault(CommissionError::AnalysisOverrun);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let (stationarity, drift_ratio_ppm) = match poll {
                Ok(Some((result, _rejection, drift_ratio_ppm))) => (result, drift_ratio_ppm),
                Ok(None) => return BreakawayProbeStep::ReadyNow,
                Err(error) => return fault(error),
            };
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            let input = match assemble_velocity_observation_input(VelocityObservationInputRequest {
                direction: session.direction,
                p,
                stationarity,
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: u16::from(backend.current_limit()),
                initial_lead_in: false,
                confidence_family_size: PROBE_CONFIDENCE_FAMILY_SIZE,
                counts_per_rev: safety.counts_per_rev(),
            }) {
                Ok(input) => input,
                Err(error) => return fault(error),
            };
            session.pending_lower_rate_q = commanded_side_lower_rate_q(
                input.direction,
                input.settled_rate_low_q,
                input.settled_rate_high_q,
            );
            session.pending_class = Some(classify_breakaway_observation(input));
            workspace.analysis.reset();
            workspace.observations.clear();
            *state = BreakawayProbeState::BeginSettleCapture {
                settle_deadline_us: *settle_deadline_us,
            };
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::BeginSettleCapture { settle_deadline_us } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return fault(CommissionError::AnalysisOverrun);
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fault(CommissionError::InvalidSchedule);
            };
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            *state = BreakawayProbeState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
            };
            BreakawayProbeStep::At(next_sample_us)
        }
        BreakawayProbeState::SettleCapture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BreakawayProbeStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend).await {
                Ok(read_us) => read_us,
                Err(result) => return result,
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_zero_output(value),
                Err(error) => return current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return fault(CommissionError::InvalidSchedule);
                };
                workspace
                    .analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        workspace.settle_min_displacement_counts,
                        workspace.settle_max_displacement_counts,
                    ));
                *state = BreakawayProbeState::AnalyzeSettle {
                    analysis_deadline_us,
                };
                BreakawayProbeStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BreakawayProbeStep::At(*next_sample_us)
                    }
                    Err(error) => fault(error),
                }
            }
        }
        BreakawayProbeState::AnalyzeSettle {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return fault(CommissionError::AnalysisOverrun);
            }
            let confirmation = match poll_scored_rest_confirmation(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(ScoredRestPoll::Pending | ScoredRestPoll::Diagnostic(_)) => {
                    return BreakawayProbeStep::ReadyNow;
                }
                Ok(ScoredRestPoll::Complete(confirmation)) => confirmation,
                Err(error) => return fault(error),
            };
            if let Err(error) =
                finish_velocity_current_zero_phase(&mut workspace.current_observer, backend).await
            {
                return current_guard_fault(error);
            }
            if workspace
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .is_err()
            {
                return fault(CommissionError::InvalidSchedule);
            }
            workspace.analysis.reset();
            workspace.observations.clear();
            safety.end_primitive();
            if !confirmation.confirmed_rest {
                return BreakawayProbeStep::Done(BreakawayProbeOutcome::ProbeEvidenceExcluded);
            }
            let recovery_plan = match plan_rung_origin_recovery(
                safety.budget,
                safety.counts_per_rev(),
                session.planned_velocity_mrev_s,
            ) {
                Ok(plan) => plan,
                Err(_) => return fault(CommissionError::InvalidSchedule),
            };
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            let hard_limit = u16::from(backend.current_limit());
            let clamp_limit = match PidTorqueFluxLimit::new(u32::from(hard_limit)) {
                Ok(limit) => limit,
                Err(_) => return fault(CommissionError::InvalidSchedule),
            };
            workspace.current_observer.reset(
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            );
            let (lower_rate_q, authority_source) = match session.pending_lower_rate_q {
                Some(rate) => (Some(rate), RecoveryBindingSource::SettledRate),
                None => (None, RecoveryBindingSource::MissingRate),
            };
            let runtime = match RungOriginRecoveryRuntime::new(
                RungOriginRecoveryConfig {
                    stage: RecoveryStage::Proportional,
                    rung_index: session.pending_index,
                    p,
                    target_velocity_rpm: session.target_velocity_rpm,
                    planned_velocity_mrev_s: session.planned_velocity_mrev_s,
                    start_offset_counts: safety.unwrapped_position,
                    lower_rate_q,
                    authority_source,
                    current_contact_policy: RecoveryCurrentContactPolicy::StopImmediately,
                    rest_schedule: RecoveryRestSchedule::ImmediateScored,
                },
                recovery_plan,
            ) {
                Ok(runtime) => runtime,
                Err(error) => return fault(error),
            };
            *state = BreakawayProbeState::Recover(runtime);
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::Recover(runtime) => {
            let mut resources = RungOriginRecoveryResources {
                safety,
                observations: &mut workspace.observations,
                blocker: &mut workspace.blocker,
                analysis: &mut workspace.analysis,
                settle_displacement_counts: &mut workspace.settle_displacement_counts,
                settle_min_displacement_counts: &mut workspace.settle_min_displacement_counts,
                settle_max_displacement_counts: &mut workspace.settle_max_displacement_counts,
                current_observer: &mut workspace.current_observer,
            };
            match step_rung_origin_recovery(runtime, &mut resources, backend).await {
                RungOriginRecoveryStep::ReadyNow => BreakawayProbeStep::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => BreakawayProbeStep::At(deadline_us),
                RungOriginRecoveryStep::RestSelection { .. }
                | RungOriginRecoveryStep::RestAnalysis(_) => BreakawayProbeStep::ReadyNow,
                RungOriginRecoveryStep::Finished(_) | RungOriginRecoveryStep::Contained(_) => {
                    *state = BreakawayProbeState::Advance;
                    BreakawayProbeStep::ReadyNow
                }
                RungOriginRecoveryStep::RestUnconfirmed => {
                    BreakawayProbeStep::Done(BreakawayProbeOutcome::ProbeEvidenceExcluded)
                }
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: Some(fault_report),
                } => safety_fault(fault_report, error),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => fault(error),
            }
        }
        BreakawayProbeState::Advance => {
            let class_result = session
                .pending_class
                .take()
                .expect("classified before recovery started");
            match session.cursor.advance(class_result) {
                Ok(BreakawaySearchStep::Probe(next_index)) => {
                    session.pending_index = next_index;
                    *state = BreakawayProbeState::WriteGain;
                    BreakawayProbeStep::ReadyNow
                }
                Ok(BreakawaySearchStep::Resolved(directional)) => match session.direction {
                    VelocityDirection::Forward => {
                        session.forward_result = Some(directional);
                        session.direction = VelocityDirection::Reverse;
                        match BreakawaySearchCursor::start(session.plan.search_count) {
                            Ok((cursor, pending_index)) => {
                                session.cursor = cursor;
                                session.pending_index = pending_index;
                                *state = BreakawayProbeState::WriteGain;
                                BreakawayProbeStep::ReadyNow
                            }
                            Err(error) => search_error_outcome(error),
                        }
                    }
                    VelocityDirection::Reverse => {
                        let forward = session
                            .forward_result
                            .take()
                            .expect("forward direction resolves before reverse");
                        match resolve_campaign_breakaway(forward, directional) {
                            Ok(resolved) => BreakawayProbeStep::Done(
                                BreakawayProbeOutcome::BreakawayFound(resolved),
                            ),
                            Err(error) => search_error_outcome(error),
                        }
                    }
                },
                Err(error) => search_error_outcome(error),
            }
        }
    }
}

/// Sample one 1 kHz tick shared by the moving-capture and settle-capture states.
///
/// On success, returns the sample's `elapsed_us` timestamp so the caller can drive
/// its own deadline arithmetic. Every failure here -- register I/O, the observation
/// gap fault, or the quarter-turn fault -- is resolved to the exact
/// [`BreakawayProbeStep`] the caller must return.
///
/// This mirrors the per-tick fault prologue `step_velocity_sweep_with_trace` inlines
/// twice, at `sweep.rs:867` (`Capture`) and `sweep.rs:1150` (`SettleCapture`) --
/// same gap window, same quarter-turn test, same current-input check, same fault
/// constructors, same order. The sweep has no shared helper for this today (it is
/// duplicated there, not merely here), and `integral_sweep.rs` carries its own third
/// copy. Deliberately deferred rather than force-extracted in this fix round: a
/// shared primitive would need to preserve the sweep's interleaved trace-observation
/// recording (`record_velocity_trace_observation`, diagnostic reads, stroke-index
/// bookkeeping) byte-for-byte across two call sites plus `integral_sweep.rs`, which
/// is real surgery on the sweep hot path and deserves its own dedicated,
/// behavior-preservation-tested slice rather than a rushed one here. Numeric drift
/// between the three copies is not possible -- they close over the same shared
/// constants (`MAX_SAFETY_OBSERVATION_GAP_US`, the safety/current-guard fault
/// constructors) and fields (`OuterSafetyLimits`, `VelocityCurrentObserver`) -- so a
/// documented follow-up is a safe deferral, not a live risk.
async fn sample_one_tick<B: CommissioningBackend>(
    workspace: &mut BreakawayProbeWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<u64, BreakawayProbeStep> {
    let encoder_raw = backend
        .read_capture_register(ABN_DECODER_COUNT)
        .await
        .map_err(|_| fault(CommissionError::Spi))?;
    let read_us = backend.elapsed_us();
    let elapsed_us = match read_us.checked_sub(workspace.previous_read_us) {
        Some(value @ 1..=MAX_SAFETY_OBSERVATION_GAP_US) => value,
        _ => {
            let fault_report = safety.fault(
                OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                0,
                read_us.saturating_sub(workspace.previous_read_us),
                0,
                0,
            );
            return Err(safety_fault(
                fault_report,
                CommissionError::SafetyEnvelopeViolation,
            ));
        }
    };
    let domain = EncoderDomain::new(safety.counts_per_rev())
        .map_err(|_| fault(CommissionError::InvalidSchedule))?;
    let delta = match domain.delta(
        domain.count(workspace.previous_encoder_raw),
        domain.count(encoder_raw),
    ) {
        Ok(delta) if delta.absolute_counts() < safety.quarter_turn_fault_counts => delta,
        _ => {
            let fault_report = safety.fault(OUTER_SAFETY_FAULT_QUARTER_TURN, 0, elapsed_us, 0, 0);
            return Err(safety_fault(
                fault_report,
                CommissionError::SafetyEnvelopeViolation,
            ));
        }
    };
    let torque_demand = read_velocity_pid_input_abs(backend)
        .await
        .map_err(current_guard_fault)?;
    if torque_demand > workspace.current_observer.clamp_limit() {
        let fault_report = safety.fault(OUTER_SAFETY_FAULT_CURRENT, 0, elapsed_us, 0, 0);
        return Err(safety_fault(
            fault_report,
            CommissionError::SafetyEnvelopeViolation,
        ));
    }
    workspace.settle_displacement_counts = i32::from(workspace.settle_displacement_counts)
        .saturating_add(delta.signed_counts())
        .clamp(-2, 2) as i8;
    workspace.settle_min_displacement_counts = workspace
        .settle_min_displacement_counts
        .min(workspace.settle_displacement_counts);
    workspace.settle_max_displacement_counts = workspace
        .settle_max_displacement_counts
        .max(workspace.settle_displacement_counts);
    workspace
        .observations
        .push(delta.signed_counts(), elapsed_us as u16)
        .map_err(|_| fault(CommissionError::InvalidSchedule))?;
    workspace.previous_encoder_raw = encoder_raw;
    workspace.previous_read_us = read_us;
    Ok(read_us)
}

#[cfg(test)]
#[path = "breakaway_campaign_tests.rs"]
mod tests;
