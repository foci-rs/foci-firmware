//! Breakaway campaign drivers: a deterministic upward excursion probe, additive
//! discovery, and held-out confirmation over shared guarded-stroke primitives.
//!
//! This module drives motion. It reuses the lower-level primitives the trace-gated
//! velocity sweep (`sweep.rs`) is built from -- [`current_guard`](super::current_guard),
//! [`recovery`](super::recovery), and [`sweep_analysis`](super::sweep_analysis)'s
//! `poll_stationarity`/`poll_scored_rest_confirmation`/`assemble_probe_observation_input`
//! -- plus the probe-local physical-excursion accumulator. It does not
//! copy or re-derive the sweep's rung/slot-folding state machine: a probe stroke is a
//! single direction, single observation, with no operating-region or handoff
//! bookkeeping.
//!
//! The live [`BreakawayCampaign`] wires all three phases into
//! [`super::super::OuterPhase`]. Each phase exposes a single-tick
//! cooperative stepper -- [`step_breakaway_probe`] -- that a caller (a future engine
//! phase, or a test harness) drives exactly like [`super::sweep::step_velocity_sweep_with_trace`]:
//! poll until [`BreakawayProbeStep::Done`], running the host-declared periodic safety
//! envelope check ([`crate::commissioning::outer::check_outer_safety`]) between ticks,
//! same as every other outer phase.
//!
//! This module also drives the additive discovery phase --
//! [`step_additive_discovery`] -- which walks the resolved
//! [`super::breakaway_plan::AdditiveDiscoveryPlan`] upward, one rung at a time, with
//! velocity I held at zero. It reuses the probe's own primitives directly: the same
//! [`sample_one_tick`] per-tick safety guard, the same
//! `assemble_velocity_observation_input`, the same origin-recovery calls, and the
//! same safety short-circuit (a `WrongDirectionFault`/`HardClipFault` or
//! `InvalidEvidenceFault` observation goes straight to its fault terminal with no
//! settle/recovery stroke). It does not re-derive the probe's guarded-stroke
//! sequence; it composes [`BreakawayProbeWorkspace`] into its own workspace and
//! writes a parallel state machine over the same primitives, since a discovery
//! stroke additionally folds observations into per-rung, per-direction consensus
//! (`depth_three_consensus`) that the probe has no use for.
//!
//! This module also drives the held-out confirmation phase --
//! [`step_band_confirmation`], entered via [`begin_band_confirmation`] -- once a
//! finished additive discovery campaign has nominated a candidate and its
//! confirmation plan has been frozen. It reuses the probe/discovery primitives
//! directly: the same [`sample_one_tick`] per-tick safety guard, the same
//! `assemble_velocity_observation_input`, the same origin-recovery calls, the
//! same safety short-circuit, and discovery's own [`DirectionAccumulator`] for
//! folding four fresh strokes into a
//! [`super::sweep_consensus::DirectionalConsensus`]. It runs a FIXED
//! eight-stroke block (`CONFIRMATION_SLOT_ORDER`: four forward, four reverse) at
//! the frozen candidate gain -- no bisection cursor, no ladder walk, and no
//! early stop on current headroom, because the confirmation contract requires
//! all eight fresh observations regardless. Its only accept result,
//! [`BandConfirmationOutcome::Accepted`], is the sole thing a later slice may
//! use to authorize Stage C; every other outcome is a confirmation-owned
//! terminal.
//!

use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use crate::autotune::velocity_measurement::{
    DyadicBlocker, StationarityExclusion, StationarityResult, StationarityWork,
    VELOCITY_RATE_Q_SCALE, VelocityDirection, confidence_multiplier_95_df15,
};
use crate::autotune::{CommissionError, EquivalentTorqueTargetInterval, TmcMechanicalRpm};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::state::{OuterPhase, VelocityIntegralData, VelocityIntegralState};
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocityObservationRing};
use crate::commissioning::outer::{OuterSafetyLimits, fail_outer, fail_outer_with_safety};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::StepResult;
use crate::commissioning::types::{
    BreakawayCampaignEvent, BreakawayCeilingSource, OUTER_SAFETY_FAULT_CURRENT,
    OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault,
};
use crate::diagnostic::BreakawayCheckpoint;

use super::band_confirmation::{
    BandConfirmationAcceptance, BandConfirmationFailureCause, BandConfirmationPlan,
    CONFIRMATION_BAND_LOWER_PERCENT, CONFIRMATION_BAND_UPPER_PERCENT, CONFIRMATION_FAMILY_SIZE,
    CONFIRMATION_SLOT_ORDER, ConfirmationDirectionalInput, DiscoveryObservationAtom,
    DiscoveryRungNominationBounds, DiscoveryRungNominationInput, NominatedConfirmationCandidate,
    TargetBandConfirmationInconclusive, TargetBandUnconfirmableFromDiscovery,
    evaluate_band_confirmation, nominate_confirmation_candidate_from_bounds,
    plan_band_confirmation, rung_nomination_bounds, rung_response_bounds_percent,
    validate_fixed_contract,
};
use super::breakaway_excursion::{ProbeExcursionCapture, ProbeExcursionError, ProbeMotionClass};
use super::breakaway_plan::{
    AdditiveDiscoveryPlan, AdditiveDiscoveryRequest, AdditiveLadderPlanError,
    BreakawayProbeContext, GridFloorOrigin, PROBE_MAX_OBSERVATIONS, UpwardBreakawayProbePlan,
    plan_additive_discovery_ladder, plan_upward_breakaway_probe, resolve_grid_floor,
};
use super::breakaway_reporting::{
    BreakawayPhase, BreakawayTerminalCause, ProbeDecisionReportInput, campaign_terminal_report,
    confirmation_evaluated_terminal_report, confirmation_partial_evidence_terminal_report,
    confirmation_plan_report, confirmation_zero_stroke_cause,
    confirmation_zero_stroke_terminal_report, discovery_plan_report, discovery_terminal_cause,
    discovery_terminal_report, probe_plan_report, probe_resolved_report, probe_terminal_cause,
    probe_terminal_report, rung_zero_diagnostic_report,
};
use super::breakaway_search::{
    BreakawayMotionClass, BreakawaySearchError, ProbeStrokeRole, UpwardBreakawayCursor,
    UpwardProbeStep,
};
use super::combined_plan::combined_workflow_geometry;
use super::current_guard::{
    VelocityCurrentClampSession, VelocityCurrentGuardError, VelocityCurrentObserver,
    VelocityCurrentZeroPhase, VelocityLimitEvidenceCapability, begin_velocity_current_primitive,
    finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    read_velocity_pid_input_abs, read_velocity_pid_output_abs,
};
use super::integral_plan::{
    AcquisitionContext, CommissioningWorkflowPlan, VelocityIntegralPlan,
    bind_breakaway_stage_c_authority, breakaway_workflow_envelope_ms,
    plan_breakaway_velocity_integral_response,
};
use super::recovery::{
    RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryRestSchedule, RecoveryStage,
    RungOriginRecoveryConfig, RungOriginRecoveryResources, RungOriginRecoveryRuntime,
    RungOriginRecoveryStep, commanded_side_lower_rate_q, plan_rung_origin_recovery,
    step_rung_origin_recovery,
};
use super::sweep::clear_velocity_integrator;
use super::sweep_analysis::{
    ScoredRestPoll, VelocityObservationInputRequest, assemble_probe_observation_input,
    assemble_velocity_observation_input, bounded_rest_displacement, direction_sign,
    poll_scored_rest_confirmation, poll_stationarity,
};
use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, depth_three_consensus,
};
use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, classify_velocity_observation,
};
use super::sweep_plan::current_gain_span;
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US,
    OBSERVATIONS_PER_DIRECTION_PER_RUNG, SETTLE_ANALYSIS_BUDGET_US,
    VELOCITY_SWEEP_SAMPLE_PERIOD_US, VelocitySweepRequest, ZERO_SETTLE_US,
    plan_combined_velocity_sweep,
};

/// Outer velocity ramp mode. Matches the wire encoding
/// [`super::sweep::step_velocity_sweep_with_trace`] writes to `MODE_RAMP_MODE_MOTION`;
/// duplicated here because that constant is private to the `sweep` module. Shared by
/// the probe, the additive discovery driver, and the held-out confirmation driver:
/// all three command a single-direction velocity ramp with I held at zero.
const OUTER_VELOCITY_RAMP_MODE: u32 = 0x8000_0002;

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
    excursion: Option<ProbeExcursionCapture>,
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
            excursion: None,
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

/// Frozen campaign inputs and the mutable upward-walk progress.
///
/// Bundles the immutable [`UpwardBreakawayProbePlan`] and one
/// [`UpwardBreakawayCursor`] so [`step_breakaway_probe`] stays within the five
/// parameter guideline. Constructed before the first stroke via
/// [`BreakawayProbeSession::new`].
#[derive(Debug, defmt::Format)]
pub(crate) struct BreakawayProbeSession {
    plan: UpwardBreakawayProbePlan,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    cursor: UpwardBreakawayCursor,
    pending_index: u8,
    pending_role: ProbeStrokeRole,
    pending_result: Option<Result<bool, BreakawayProbeOutcome>>,
    pending_lower_rate_q: Option<u64>,
    pending_observation: Option<CollectedObservation>,
}

impl BreakawayProbeSession {
    /// Freeze the campaign inputs and start the forward-direction cursor.
    ///
    /// This never touches `backend`: constructing a session is pure. Motion is only
    /// reachable through [`step_breakaway_probe`], and only once its state has left
    /// [`BreakawayProbeState::PlannedProbe`].
    pub(crate) fn new(
        plan: UpwardBreakawayProbePlan,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
    ) -> Result<Self, BreakawaySearchError> {
        let (cursor, first) = UpwardBreakawayCursor::start(plan.search_count)?;
        let UpwardProbeStep::Probe {
            rung_index: pending_index,
            role: pending_role,
        } = first
        else {
            return Err(BreakawaySearchError::InvalidRequest);
        };
        Ok(Self {
            plan,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            cursor,
            pending_index,
            pending_role,
            pending_result: None,
            pending_lower_rate_q: None,
            pending_observation: None,
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
    /// Validate the frozen physical context and arm the campaign safety baseline.
    SetupSafety,
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
    /// Finalize excursion and comparison-only stationarity diagnostics.
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
    /// Emit the recovered observation's decision before mutating the cursor.
    EmitDecision,
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
    /// One raw observation is ready for immediate trace emission.
    Observation(CollectedObservation),
    /// One recovered observation is ready to commit to replay.
    Decision {
        /// Direction of the committed observation.
        direction: VelocityDirection,
        /// Zero-based probe-grid index.
        rung_index: u8,
        /// Disposition: 0 no motion, 1 moved.
        disposition: u8,
    },
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
    /// The first rung with repeatable motion in both directions.
    BreakawayFound(ResolvedUpwardBreakaway),
    /// The authority rung was exhausted without repeatable motion.
    NoRepeatableMotionAtAuthority,
    /// Excursion timing or arithmetic was not trustworthy enough to advance.
    ExcursionEvidenceInvalid,
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

/// Resolved upward-probe result retained for reporting and discovery planning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct ResolvedUpwardBreakaway {
    /// Zero-based resolved rung in the realized probe grid.
    pub(crate) breakaway_index: u8,
    /// Total observations consumed across the upward walk.
    pub(crate) observations: u8,
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

fn checkpoint_breakaway_operation(checkpoint: BreakawayCheckpoint) {
    crate::diagnostic::checkpoint_breakaway_campaign(checkpoint);
}

/// Map a resolved [`BreakawaySearchError`] to its owning probe outcome.
fn search_error_outcome(error: BreakawaySearchError) -> BreakawayProbeStep {
    match error {
        BreakawaySearchError::InvalidRequest | BreakawaySearchError::Arithmetic => {
            fault(CommissionError::InvalidSchedule)
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
            *state = BreakawayProbeState::SetupSafety;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::SetupSafety => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::SafetyConfigured,
            );
            let frozen = session.plan.context;
            if backend.mechanical_encoder_quadrature_counts_per_rev()
                != frozen.encoder_counts_per_rev
                || u16::from(backend.current_limit()) != frozen.hard_torque_limit
            {
                return fault(CommissionError::InvalidSchedule);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeSafetyEncoderRead);
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fault(CommissionError::Spi),
            };
            if safety.set_encoder_baseline(encoder_raw).is_err() {
                return fault(CommissionError::EncoderFault);
            }
            safety.torque_cap = frozen.hard_torque_limit;
            if let Err(error) = safety.arm_safety_observation(backend.elapsed_us()) {
                return fault(error);
            }
            *state = BreakawayProbeState::WriteGain;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::WriteGain => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::GainProgramming,
            );
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeGainWrite);
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeRampModeWrite);
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_RAMP_MODE)
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeGainIntegratorClear);
            if clear_velocity_integrator(backend).await.is_err() {
                return fault(CommissionError::Spi);
            }
            *state = BreakawayProbeState::CommandTarget;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::CommandTarget => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::TargetCommand,
            );
            let target = match session
                .target_velocity_rpm
                .with_direction(direction_sign(session.pending_role.direction()))
            {
                Ok(target) => target.raw(),
                Err(_) => return fault(CommissionError::InvalidSchedule),
            };
            let hard_limit = session.plan.context.hard_torque_limit;
            let mut clamp_session =
                match VelocityCurrentClampSession::try_new(u32::from(hard_limit)) {
                    Ok(session) => session,
                    Err(error) => return current_guard_fault(error),
                };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeClampInstall);
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return current_guard_fault(error);
            }
            let clamp_limit = clamp_session.probe_limit();
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeCurrentPrimitiveStart);
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
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeTargetBaselineRead);
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            workspace.excursion = Some(ProbeExcursionCapture::new(encoder_raw, baseline_us));
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeTargetWrite);
            if backend
                .write_register_verified(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            if let Err(error) = safety.begin_primitive(backend.elapsed_us()) {
                return fault(error);
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
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::Capture,
            );
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
            let read_us = match sample_one_tick(
                workspace,
                safety,
                backend,
                Some(BreakawayCheckpoint::ProbeMovingEncoderSample),
                Some(BreakawayCheckpoint::ProbeMovingCurrentInputSample),
                true,
            )
            .await
            {
                Ok(read_us) => read_us,
                Err(result) => return result,
            };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeMovingCurrentOutputSample);
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
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeMovingBoundary);
            let boundary =
                finish_velocity_current_moving_phase(&mut workspace.current_observer, backend)
                    .await;
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeZeroTargetWrite);
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
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeZeroIntegratorClear);
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
            let (stationarity, rejected_post_fit, drift_ratio_ppm) = match poll {
                Ok(Some((result, rejection, drift_ratio_ppm))) => {
                    (result, rejection, drift_ratio_ppm)
                }
                Ok(None) => return BreakawayProbeStep::ReadyNow,
                Err(error) => return fault(error),
            };
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            let input = match assemble_probe_observation_input(VelocityObservationInputRequest {
                direction: session.pending_role.direction(),
                p,
                stationarity,
                rejected_post_fit,
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: session.plan.context.hard_torque_limit,
                initial_lead_in: false,
                // Comparison-only diagnostics retain the stationarity work path's
                // family. Physical excursion alone decides probe motion.
                confidence_family_size: 3,
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
            let capture = match workspace.excursion.take() {
                Some(capture) => capture,
                None => return fault(CommissionError::InvalidSchedule),
            };
            let wire_fields = capture.wire_fields();
            let excursion = capture.finish(
                session.pending_role.direction(),
                session.plan.motion_threshold_counts,
                session.plan.max_capture_interval_us,
            );
            session.pending_result = Some(
                if input.predicted_torque_target_abs >= input.hard_torque_limit {
                    Err(BreakawayProbeOutcome::SafetyFault {
                        error: CommissionError::SafetyEnvelopeViolation,
                        safety: None,
                    })
                } else {
                    match excursion {
                        Err(error) => {
                            checkpoint_breakaway_operation(match error {
                                ProbeExcursionError::CaptureInterval => {
                                    BreakawayCheckpoint::ProbeExcursionIntervalInvalid
                                }
                                ProbeExcursionError::Arithmetic => {
                                    BreakawayCheckpoint::ProbeExcursionArithmeticInvalid
                                }
                            });
                            Err(BreakawayProbeOutcome::ExcursionEvidenceInvalid)
                        }
                        Ok(super::breakaway_excursion::ProbeExcursionEvidence {
                            motion: ProbeMotionClass::WrongDirection,
                            ..
                        }) => Err(BreakawayProbeOutcome::SafetyFault {
                            error: CommissionError::VelocityValidationFailed,
                            safety: None,
                        }),
                        Ok(evidence)
                            if matches!(evidence.motion, ProbeMotionClass::Moved)
                                && input.predicted_torque_target_abs
                                    >= input.usable_torque_limit =>
                        {
                            Err(BreakawayProbeOutcome::AuthorityFailure)
                        }
                        Ok(evidence) => Ok(matches!(evidence.motion, ProbeMotionClass::Moved)),
                    }
                },
            );
            let target_rate_q =
                match discovery_target_rate_q(session.target_velocity_rpm, safety.counts_per_rev())
                {
                    Ok(value) => value,
                    Err(error) => return fault(error),
                };
            let collected = CollectedObservation {
                atom: DiscoveryObservationAtom {
                    mean_rate_q: input.settled_rate_mean_q,
                    variance_of_mean_q2: input.variance_of_mean_q2,
                    target_rate_q,
                    direction: session.pending_role.direction(),
                },
                rung_index: session.pending_index,
                slot_index: session.pending_role.slot_index(),
                classifier_inputs: observation_classifier_inputs(
                    &input,
                    workspace.current_observer.moving_pid_output_peak(),
                ),
                stroke_min_displacement_counts: wire_fields.map_or(0, |fields| fields.0),
                stroke_max_displacement_counts: wire_fields.map_or(0, |fields| fields.1),
                capture_max_interval_us: wire_fields.map_or(0, |fields| fields.2),
            };
            session.pending_observation = Some(collected);
            workspace.analysis.reset();
            workspace.observations.clear();
            if session.pending_result.as_ref().is_some_and(Result::is_err) {
                safety.end_primitive();
            }
            *state = match session.pending_result.as_ref() {
                Some(Ok(_)) => BreakawayProbeState::BeginSettleCapture {
                    settle_deadline_us: *settle_deadline_us,
                },
                Some(Err(_)) => BreakawayProbeState::Advance,
                None => return fault(CommissionError::InvalidSchedule),
            };
            BreakawayProbeStep::Observation(collected)
        }
        BreakawayProbeState::BeginSettleCapture { settle_deadline_us } => {
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeSettleBaselineRead);
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
            let read_us = match sample_one_tick(
                workspace,
                safety,
                backend,
                Some(BreakawayCheckpoint::ProbeSettleEncoderSample),
                Some(BreakawayCheckpoint::ProbeSettleCurrentInputSample),
                false,
            )
            .await
            {
                Ok(read_us) => read_us,
                Err(result) => return result,
            };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeSettleCurrentOutputSample);
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
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeZeroBoundary);
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
                return fault(CommissionError::VelocityValidationFailed);
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
            let hard_limit = session.plan.context.hard_torque_limit;
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
                    *state = if session.pending_result.as_ref().is_some_and(Result::is_ok) {
                        BreakawayProbeState::EmitDecision
                    } else {
                        BreakawayProbeState::Advance
                    };
                    BreakawayProbeStep::ReadyNow
                }
                RungOriginRecoveryStep::RestUnconfirmed => {
                    fault(CommissionError::VelocityValidationFailed)
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
        BreakawayProbeState::EmitDecision => {
            let moved = session
                .pending_result
                .as_ref()
                .and_then(|result| result.as_ref().ok())
                .copied()
                .expect("only a motion verdict reaches decision emission");
            let observation = session
                .pending_observation
                .expect("raw observation retained until cursor commit");
            *state = BreakawayProbeState::Advance;
            BreakawayProbeStep::Decision {
                direction: observation.atom.direction,
                rung_index: observation.rung_index,
                disposition: u8::from(moved),
            }
        }
        BreakawayProbeState::Advance => {
            session
                .pending_observation
                .take()
                .expect("raw observation retained until cursor consumption");
            let result = session
                .pending_result
                .take()
                .expect("classified before recovery started");
            let moved = match result {
                Ok(moved) => moved,
                Err(outcome) => return BreakawayProbeStep::Done(outcome),
            };
            match session.cursor.advance(moved) {
                Ok(UpwardProbeStep::Probe { rung_index, role }) => {
                    session.pending_index = rung_index;
                    session.pending_role = role;
                    *state = BreakawayProbeState::WriteGain;
                    BreakawayProbeStep::ReadyNow
                }
                Ok(UpwardProbeStep::Resolved {
                    rung_index,
                    observations,
                }) => BreakawayProbeStep::Done(BreakawayProbeOutcome::BreakawayFound(
                    ResolvedUpwardBreakaway {
                        breakaway_index: rung_index,
                        observations,
                    },
                )),
                Ok(UpwardProbeStep::NoRepeatableMotionAtAuthority { .. }) => {
                    BreakawayProbeStep::Done(BreakawayProbeOutcome::NoRepeatableMotionAtAuthority)
                }
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
    encoder_checkpoint: Option<BreakawayCheckpoint>,
    current_input_checkpoint: Option<BreakawayCheckpoint>,
    record_excursion: bool,
) -> Result<u64, BreakawayProbeStep> {
    if let Some(checkpoint) = encoder_checkpoint {
        checkpoint_breakaway_operation(checkpoint);
    }
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
    if record_excursion {
        let capture = workspace
            .excursion
            .as_mut()
            .ok_or_else(|| fault(CommissionError::InvalidSchedule))?;
        capture
            .observe(domain, encoder_raw, read_us)
            .map_err(|_| fault(CommissionError::InvalidSchedule))?;
    }
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
    if let Some(checkpoint) = current_input_checkpoint {
        checkpoint_breakaway_operation(checkpoint);
    }
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

// ============================================================================
// Additive discovery driver
// ============================================================================

/// Maximum in-band discovery rungs retained for later nomination.
///
/// Bounded at [`MAX_SWEEP_RUNGS`] because, in the worst case, every rung on the
/// ladder could independently satisfy its own 70-80% containment; retaining fewer
/// would silently drop candidate evidence that 6d/6e's nomination screen is
/// entitled to see. This driver is not yet wired into
/// [`super::super::OuterPhase`]/`CommissioningEngine` (see the module-level docs),
/// so it is not subject to `size_regression_gates` until that wiring lands.
const MAX_IN_BAND_DISCOVERY_RUNGS: usize = MAX_SWEEP_RUNGS;

const EMPTY_DISCOVERY_RUNG_BOUNDS: DiscoveryRungNominationBounds = DiscoveryRungNominationBounds {
    p: PidPGain::ZERO,
    lower_percent: 0.0,
    upper_percent: 0.0,
    nominatable: false,
};

/// Every discovery rung whose own response was contained in 70-80%, collected in
/// ascending rung order for [`nominate_confirmation_candidate_from_bounds`].
///
/// Stores each rung's compact [`DiscoveryRungNominationBounds`] -- the
/// candidate `p`, its enveloped nomination interval, and whether it is
/// nominatable at all -- rather than the raw observation atoms nomination no
/// longer needs once [`rung_nomination_bounds`] has folded them at collection
/// time. At [`MAX_IN_BAND_DISCOVERY_RUNGS`] worst case this is roughly
/// [`MAX_SWEEP_RUNGS`] times `size_of::<DiscoveryRungNominationBounds>()`, a
/// few hundred bytes rather than the raw-atom representation's ~14 KB.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct CollectedInBandRungs {
    rungs: [DiscoveryRungNominationBounds; MAX_IN_BAND_DISCOVERY_RUNGS],
    count: u8,
}

impl CollectedInBandRungs {
    const fn new() -> Self {
        Self {
            rungs: [EMPTY_DISCOVERY_RUNG_BOUNDS; MAX_IN_BAND_DISCOVERY_RUNGS],
            count: 0,
        }
    }

    /// Every collected in-band rung's nomination bounds, in ascending rung order.
    pub(crate) fn as_slice(&self) -> &[DiscoveryRungNominationBounds] {
        &self.rungs[..usize::from(self.count)]
    }

    /// Number of collected in-band rungs.
    pub(crate) const fn count(&self) -> u8 {
        self.count
    }

    fn push(&mut self, bounds: DiscoveryRungNominationBounds) -> Result<(), CommissionError> {
        let slot = self
            .rungs
            .get_mut(usize::from(self.count))
            .ok_or(CommissionError::InvalidSchedule)?;
        *slot = bounds;
        self.count += 1;
        Ok(())
    }
}

/// Per-direction accumulator for one rung's four observations, folded into a
/// [`DirectionalConsensus`] once the direction's full slot schedule is collected.
#[derive(Debug, defmt::Format)]
struct DirectionAccumulator {
    classes: [VelocityObservationClass; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    intervals: [Option<EquivalentTorqueTargetInterval>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    atoms: [Option<DiscoveryObservationAtom>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
}

impl DirectionAccumulator {
    const fn new() -> Self {
        Self {
            classes: [VelocityObservationClass::InvalidEvidenceFault;
                CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            intervals: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            atoms: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        }
    }

    fn record(
        &mut self,
        slot: u8,
        classification: VelocityObservationClass,
        interval: Option<EquivalentTorqueTargetInterval>,
        atom: DiscoveryObservationAtom,
    ) {
        let index = usize::from(slot);
        self.classes[index] = classification;
        self.intervals[index] = interval;
        self.atoms[index] = Some(atom);
    }

    /// The mask of slots this accumulator has actually recorded, derived from
    /// which `atoms` entries are `Some`. On a complete rung this is
    /// `FULL_OBSERVATION_MASK`; on a partial/mid-block failure terminal (a
    /// `ConfirmationEvidenceExcluded` or `SafetyFault` reached after some but
    /// not all four strokes ran) it reports exactly the slots gathered.
    fn collected_mask(&self) -> u8 {
        self.atoms
            .iter()
            .enumerate()
            .filter(|(_, atom)| atom.is_some())
            .fold(0u8, |mask, (index, _)| mask | (1u8 << index))
    }

    /// Fold this direction's collected observations into their depth-three
    /// consensus.
    ///
    /// The reported `collected_mask` reflects the slots actually recorded, not
    /// an assumed-full rung. On a complete rung every slot is present, so
    /// `collected_mask == FULL_OBSERVATION_MASK` and the fold is unchanged. On
    /// a partial terminal the honest partial mask flows to
    /// [`depth_three_consensus`], which gates completeness on it (a
    /// non-full `collected_mask` yields an `IncompleteRung` consensus), so the
    /// wire record carries a truthful mask instead of falsely claiming a full
    /// rung's worth of evidence was gathered.
    fn consensus(&self) -> DirectionalConsensus {
        depth_three_consensus(self.collected_mask(), self.classes, self.intervals)
    }
}

/// One classified stroke's evidence, held between [`AdditiveDiscoveryState::AnalyzeMoving`]
/// and [`AdditiveDiscoveryState::Advance`] the same way
/// [`BreakawayProbeSession::pending_class`](BreakawayProbeSession) holds the probe's
/// classification across origin-recovery.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingObservationAtom {
    mean_rate_q: i64,
    variance_of_mean_q2: u128,
    target_rate_q: i64,
}

impl PendingObservationAtom {
    fn with_direction(self, direction: VelocityDirection) -> DiscoveryObservationAtom {
        DiscoveryObservationAtom {
            mean_rate_q: self.mean_rate_q,
            variance_of_mean_q2: self.variance_of_mean_q2,
            target_rate_q: self.target_rate_q,
            direction,
        }
    }
}

/// One discovery stroke retained until recovery finishes and the cursor consumes it.
///
/// Direction is owned by [`AdditiveDiscoverySession`] throughout this interval,
/// so the compact pending atom does not duplicate it in retained RAM.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingDiscoveryRecord {
    classification: VelocityObservationClass,
    interval: Option<EquivalentTorqueTargetInterval>,
    atom: PendingObservationAtom,
    classifier_inputs: ObservationClassifierInputs,
}

/// Frozen campaign inputs and the mutable ladder-walk progress.
///
/// Bundles the immutable [`AdditiveDiscoveryPlan`] together with the ladder-walk
/// cursor (rung, direction, slot), the per-rung accumulators, and the collected
/// in-band evidence, mirroring [`BreakawayProbeSession`]'s role for the probe.
#[derive(Debug, defmt::Format)]
pub(crate) struct AdditiveDiscoverySession {
    plan: AdditiveDiscoveryPlan,
    own_multiplier: f32,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    rung_index: u8,
    direction: VelocityDirection,
    slot: u8,
    forward: DirectionAccumulator,
    reverse: DirectionAccumulator,
    in_band: CollectedInBandRungs,
    pending_lower_rate_q: Option<u64>,
    pending_headroom_stop: bool,
    pending_record: Option<PendingDiscoveryRecord>,
    /// The frozen planning request, retained (it is tiny and `Copy`) so the
    /// planned-report can be assembled at report time via
    /// [`discovery_plan_report`], mirroring how [`BreakawayProbeSession`]
    /// retains its own plan for reporting.
    request: AdditiveDiscoveryRequest,
    /// Whether the resolved floor was clamped to the search grid's origin.
    floor_origin_clamped: bool,
    /// Rung zero's own per-direction motion classification, retained
    /// specifically for [`rung_zero_diagnostic_report`] once discovery walks
    /// past rung zero (spec section 6: reported purely as evidence).
    rung_zero_classification: Option<(BreakawayMotionClass, BreakawayMotionClass)>,
}

impl AdditiveDiscoverySession {
    /// Freeze the resolved additive ladder from `request` and start at rung zero,
    /// forward direction, slot zero.
    ///
    /// This never touches a backend: constructing a session is pure, and motion is
    /// only reachable through [`step_additive_discovery`], matching
    /// [`BreakawayProbeSession::new`]. On
    /// [`AdditiveLadderPlanError::InsufficientAdditiveSpan`] or
    /// [`AdditiveLadderPlanError::AdditiveLadderResolutionBudgetExceeded`], the
    /// caller receives the matching discovery-owned terminal directly, with no
    /// session and no motion. Every other planning failure collapses to the
    /// existing generic invalid-schedule fault, mirroring
    /// [`search_error_outcome`]'s `InvalidRequest | Arithmetic` arm.
    ///
    /// `AdditiveDiscoveryOutcome` is deliberately large (it carries
    /// [`CollectedInBandRungs`], sized at [`MAX_SWEEP_RUNGS`] worst case); returning
    /// it as this constructor's error is the same terminal-unification tradeoff
    /// already accepted elsewhere in this crate (see the `large_enum_variant`
    /// precedents in `commission_dispatch::mod` and `commissioning::schedule`),
    /// not an oversight.
    #[allow(clippy::result_large_err)]
    pub(crate) fn new(
        request: AdditiveDiscoveryRequest,
        floor_origin_clamped: bool,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
    ) -> Result<Self, AdditiveDiscoveryOutcome> {
        let plan = plan_additive_discovery_ladder(request)
            .map_err(additive_discovery_plan_error_outcome)?;
        let own_multiplier = confidence_multiplier_95_df15(plan.confidence_family_size)
            .expect("plan_additive_discovery_ladder only returns audited family sizes");
        Ok(Self {
            plan,
            own_multiplier,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            rung_index: 0,
            direction: VelocityDirection::Forward,
            slot: 0,
            forward: DirectionAccumulator::new(),
            reverse: DirectionAccumulator::new(),
            in_band: CollectedInBandRungs::new(),
            pending_lower_rate_q: None,
            pending_headroom_stop: false,
            pending_record: None,
            request,
            floor_origin_clamped,
            rung_zero_classification: None,
        })
    }

    fn current_gain(&self) -> Result<PidPGain, CommissionError> {
        if self.rung_index >= self.plan.rung_count {
            return Err(CommissionError::InvalidSchedule);
        }
        self.plan
            .gains
            .get(usize::from(self.rung_index))
            .copied()
            .ok_or(CommissionError::InvalidSchedule)
    }
}

/// Map a resolved [`AdditiveLadderPlanError`] to its owning discovery outcome.
///
/// Only the two errors with a dedicated discovery-owned terminal
/// (`InsufficientAdditiveSpan`, `AdditiveLadderResolutionBudgetExceeded`) get one;
/// every other planning failure is a structural precondition violation, not a
/// discovery-time result, so it collapses to the same generic invalid-schedule
/// fault [`search_error_outcome`] uses for the probe's `InvalidRequest | Arithmetic`.
fn additive_discovery_plan_error_outcome(
    error: AdditiveLadderPlanError,
) -> AdditiveDiscoveryOutcome {
    match error {
        AdditiveLadderPlanError::InsufficientAdditiveSpan => {
            AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        }
        AdditiveLadderPlanError::AdditiveLadderResolutionBudgetExceeded => {
            AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded
        }
        AdditiveLadderPlanError::InvalidRequest
        | AdditiveLadderPlanError::Arithmetic
        | AdditiveLadderPlanError::ConfidenceFamilyTooLarge
        | AdditiveLadderPlanError::PrimitiveDurationExceeded
        | AdditiveLadderPlanError::WorkflowEnvelopeExceeded => {
            AdditiveDiscoveryOutcome::SafetyFault {
                error: CommissionError::InvalidSchedule,
                safety: None,
            }
        }
    }
}

/// Single-tick cooperative stroke cycle for one discovery observation.
///
/// Mirrors [`BreakawayProbeState`]'s stroke states exactly (write gain, command
/// target, capture, command zero, analyze moving, settle-capture, analyze settle,
/// recover, advance): a discovery stroke and a probe stroke are the same guarded
/// primitive. The two states are kept separate rather than shared because
/// `Advance` means different things for each driver: the probe folds a
/// physical-motion verdict into the upward cursor, while discovery folds one into a
/// per-rung, per-direction consensus accumulator and a ladder-wide stop rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum AdditiveDiscoveryState {
    /// The frozen plan is admitted; no stroke has been issued yet.
    PlannedDiscovery,
    /// Write the current rung's gain and the outer-velocity ramp mode.
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
    BeginSettleCapture {
        settle_deadline_us: u64,
        retry_available: bool,
    },
    /// Sample the zero-target rest window.
    SettleCapture {
        next_sample_us: u64,
        deadline_us: u64,
        retry_available: bool,
    },
    /// Confirm rest and record the zero-phase current evidence boundary.
    AnalyzeSettle {
        analysis_deadline_us: u64,
        retry_available: bool,
    },
    /// Return to the parked origin through the shared recovery primitive.
    Recover(RungOriginRecoveryRuntime),
    /// Fold the classified observation into the active accumulator and decide
    /// what is next: another slot, another direction, another rung, or a
    /// terminal.
    Advance,
    /// The walk selected the target band; stream one collected in-band rung's
    /// margin per tick from the retained `session.in_band` before concluding.
    /// `cursor` is the next collected index to stream; the walk is complete, so
    /// the collected set no longer changes. Only reached with a non-empty
    /// in-band set.
    StreamMargins { cursor: u8 },
}

/// One cooperative result from [`step_additive_discovery`].
///
/// `Done`'s payload is deliberately much larger than `ReadyNow`/`At`: see
/// [`AdditiveDiscoveryOutcome`]'s own doc comment for why that size is accepted
/// rather than boxed (this crate has no heap).
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum AdditiveDiscoveryStep {
    /// Call again immediately; no external wait is needed.
    ReadyNow,
    /// Call again no earlier than this `elapsed_us` deadline.
    At(u64),
    /// A stroke just completed; its raw settled-rate evidence is ready to stream
    /// before the atom is folded into the rung's nomination bounds. Call again
    /// immediately; the fold happens on the following tick.
    Observation(CollectedObservation),
    /// One collected in-band rung's margin is ready to stream, identified by its
    /// index within the retained collected set. The campaign layer owns wire
    /// identity: it attaches the run/evidence sequences and the plan digest and
    /// builds the record (a non-nominatable index yields no record). Call again
    /// immediately; the next index streams on the following tick.
    Margin { collected_index: u8 },
    /// The discovery campaign concluded.
    Done(AdditiveDiscoveryOutcome),
}

/// One collected observation's raw evidence, surfaced from a driver on the tick
/// its stroke completes so [`step_breakaway_campaign`] can stream a
/// [`crate::commissioning::types::BreakawayRawObservation`] before the atom is
/// folded (discovery) or judged (confirmation). Carries only the driver-local
/// coordinates and the raw atom; the owning plan digest and the run/evidence
/// sequences are attached by the campaign, exactly as the `*_advance` helpers
/// attach them to terminal records.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct CollectedObservation {
    /// The raw settled-rate atom (mean, family-free variance, target, direction).
    pub(crate) atom: DiscoveryObservationAtom,
    /// Probe or discovery rung index; zero for confirmation.
    pub(crate) rung_index: u8,
    /// Probe direction-local slot, discovery within-direction slot, or fixed
    /// `0..=7` confirmation schedule slot.
    pub(crate) slot_index: u8,
    /// The frozen inputs `classify_velocity_observation` and
    /// `depth_three_consensus` consume, streamed so an offline decoder derives
    /// the consensus masks itself instead of trusting firmware's decision.
    pub(crate) classifier_inputs: ObservationClassifierInputs,
    /// Probe-only minimum start-relative displacement; zero in later phases.
    pub(crate) stroke_min_displacement_counts: i32,
    /// Probe-only maximum start-relative displacement; zero in later phases.
    pub(crate) stroke_max_displacement_counts: i32,
    /// Probe-only maximum capture interval; zero in later phases.
    pub(crate) capture_max_interval_us: u32,
}

/// The exact inputs the depth-three consensus decision consumes for one stroke,
/// captured at classification time so they can ride the streamed raw-observation
/// record. This is deliberately the frozen classify/consensus input set, not
/// every capture diagnostic: `foci-trace` ports
/// [`classify_velocity_observation`] and the depth-three consensus over these,
/// reproducing `collected_mask`/`eligible_mask`/`included_mask` independently
/// Firmware's own masks are never authoritative replay inputs.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct ObservationClassifierInputs {
    /// Stationarity outcome: `0` usable, otherwise the exclusion reason code
    /// (see [`stationarity_wire_code`]).
    pub(crate) stationarity_code: u8,
    /// Inclusive lower settled-rate confidence edge.
    pub(crate) settled_rate_low_q: i64,
    /// Inclusive upper settled-rate confidence edge.
    pub(crate) settled_rate_high_q: i64,
    /// Inclusive lower signed-deficit confidence edge.
    pub(crate) deficit_low_q: i64,
    /// Inclusive upper signed-deficit confidence edge.
    pub(crate) deficit_high_q: i64,
    /// Predicted absolute velocity-controller torque target. Already folded
    /// with the measured peak: `max(P * rpm / 256, moving_pid_output_peak)`.
    pub(crate) predicted_torque_target_abs: u16,
    /// Measured peak velocity-controller output for this stroke's moving
    /// window, carried alongside `predicted_torque_target_abs` so the `max`
    /// above stays decomposable offline for clamp-evidence analysis.
    pub(crate) moving_pid_output_peak: u16,
    /// Inclusive current-headroom terminus.
    pub(crate) usable_torque_limit: u16,
    /// Hard controller-output clip boundary.
    pub(crate) hard_torque_limit: u16,
    /// Whether a representable controller-domain disturbance interval exists.
    pub(crate) disturbance_present: bool,
    /// Lower disturbance-interval bound in equivalent torque-target Q16.16 counts
    /// (`0` when absent).
    pub(crate) disturbance_low_q16: i32,
    /// Upper disturbance-interval bound in the same units (`0` when absent).
    pub(crate) disturbance_high_q16: i32,
    /// Drift ratio in parts per million, or `u32::MAX` when unavailable.
    pub(crate) drift_ratio_ppm: u32,
    /// Post-fit lag-one state: 0 unavailable, 1 failed, 2 passed.
    pub(crate) post_fit_lag_one_state: u8,
}

/// Map a [`StationarityResult`] to its compact wire code: `0` usable, `1..=6`
/// the precise exclusion reason. The reason is carried (not just usable/excluded)
/// so an offline decoder reproduces the exact `StationarityExcluded` class
/// variant, matching [`classify_velocity_observation`].
fn stationarity_wire_code(result: StationarityResult) -> u8 {
    match result {
        StationarityResult::Usable(_) => 0,
        StationarityResult::Excluded(StationarityExclusion::NoBlockingPlateau) => 1,
        StationarityResult::Excluded(StationarityExclusion::TooFewDecorrelatedBlocks) => 2,
        StationarityResult::Excluded(StationarityExclusion::ResolvedSlope) => 3,
        StationarityResult::Excluded(StationarityExclusion::StructuredResidual) => 4,
        StationarityResult::Excluded(StationarityExclusion::Arithmetic) => 5,
        StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift) => 6,
    }
}

/// Capture the frozen classify/consensus inputs from a built
/// [`VelocityObservationInput`] at classification time.
fn observation_classifier_inputs(
    input: &VelocityObservationInput,
    moving_pid_output_peak: u16,
) -> ObservationClassifierInputs {
    let (disturbance_present, disturbance_low_q16, disturbance_high_q16) = match input.disturbance {
        Some((_, interval)) => (true, interval.low().raw(), interval.high().raw()),
        None => (false, 0, 0),
    };
    ObservationClassifierInputs {
        stationarity_code: stationarity_wire_code(input.stationarity),
        settled_rate_low_q: input.settled_rate_low_q,
        settled_rate_high_q: input.settled_rate_high_q,
        deficit_low_q: input.deficit_low_q,
        deficit_high_q: input.deficit_high_q,
        predicted_torque_target_abs: input.predicted_torque_target_abs,
        moving_pid_output_peak,
        usable_torque_limit: input.usable_torque_limit,
        hard_torque_limit: input.hard_torque_limit,
        disturbance_present,
        disturbance_low_q16,
        disturbance_high_q16,
        drift_ratio_ppm: input.drift_ratio_ppm,
        post_fit_lag_one_state: input.post_fit_lag_one_state,
    }
}

/// Terminal result of a complete additive discovery campaign.
///
/// Every variant other than [`Self::TargetBandSelected`] and
/// [`Self::TargetBandSkipped`] is unambiguously a non-nominating terminal; even
/// those two never authorize Stage C by themselves (spec section 6, "Discovery
/// does not authorize Stage C") -- they only classify and collect evidence for
/// 6d/6e's nomination screen and held-out confirmation. Matching every other
/// outer velocity phase, this driver never asserts `set_enable_pin` itself and
/// centralized safe-state restoration on any terminal happens at the surrounding
/// engine boundary, not per-phase.
///
/// `TargetBandSelected` carries [`CollectedInBandRungs`], sized at
/// [`MAX_SWEEP_RUNGS`] because, in the worst case, every rung on the ladder
/// could independently satisfy its own containment and no candidate evidence
/// may be silently dropped. Since 6e-1, each collected rung is its compact
/// [`DiscoveryRungNominationBounds`] rather than its raw observation atoms, so
/// this variant is now a few hundred bytes rather than ~14 KB.
/// `large_enum_variant` is accepted here the same way it already is at the
/// `commission_dispatch::mod` and `commissioning::schedule` terminal types,
/// rather than boxed -- this crate has no heap.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum AdditiveDiscoveryOutcome {
    /// At least one rung's own response was contained in 70-80%. Carries every
    /// in-band rung collected before the ladder stopped, in ascending order,
    /// ready for
    /// [`nominate_confirmation_candidate_from_bounds`](super::band_confirmation::nominate_confirmation_candidate_from_bounds).
    /// Reached either by completing the ladder (ceiling, both-directions-above-80,
    /// or budget exhaustion) with at least one in-band rung already found, or by
    /// current authority binding after an in-band rung was already found -- current
    /// headroom above an already-safe candidate ends exploration without
    /// invalidating that candidate.
    TargetBandSelected(CollectedInBandRungs),
    /// The ladder satisfied its resolution constraint at plan time, but no rung's
    /// own response was ever contained in 70-80%: responses crossed from below
    /// 70% to above 80% between adjacent rungs (or the ladder ended without ever
    /// entering the band), before any in-band rung was found. A discovery-time
    /// RESULT, distinct from the plan-time refusal
    /// [`Self::AdditiveLadderResolutionBudgetExceeded`].
    TargetBandSkipped,
    /// Current authority bound before any rung's own response was contained in
    /// 70-80%.
    CurrentHeadroomBeforeBand,
    /// No candidate rung count could hold the minimum structural span (plan-time,
    /// no motion).
    InsufficientAdditiveSpan,
    /// Every surviving planning candidate's first additive gap exceeded the
    /// provisional resolution guard (plan-time, no motion).
    AdditiveLadderResolutionBudgetExceeded,
    /// Target-independent stationarity evidence, or another controller-domain
    /// evidence field, was unusable for classification -- or the completed
    /// origin-recovery return could not confirm rest.
    DiscoveryEvidenceExcluded { error: CommissionError },
    /// An existing safety, current, travel, encoder, or register-I/O fault,
    /// returned unchanged.
    SafetyFault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

fn discovery_fault(error: CommissionError) -> AdditiveDiscoveryStep {
    AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::SafetyFault {
        error,
        safety: None,
    })
}

fn discovery_safety_fault(
    safety: OuterSafetyFault,
    error: CommissionError,
) -> AdditiveDiscoveryStep {
    AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::SafetyFault {
        error,
        safety: Some(safety),
    })
}

fn discovery_current_guard_fault(error: VelocityCurrentGuardError) -> AdditiveDiscoveryStep {
    discovery_fault(match error {
        VelocityCurrentGuardError::InvalidLimit => CommissionError::InvalidSchedule,
        VelocityCurrentGuardError::RegisterIo => CommissionError::Spi,
        VelocityCurrentGuardError::ActionableStatus(_) => CommissionError::SafetyEnvelopeViolation,
    })
}

/// Adapt [`sample_one_tick`]'s probe-flavored fault into a discovery step.
///
/// `sample_one_tick` only ever fails through its own `fault`/`safety_fault`
/// constructors, both of which resolve to
/// `BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault { .. })`; no other
/// probe outcome variant is reachable from it. Reusing the function directly
/// (rather than re-deriving its per-tick safety prologue a fourth time) means this
/// thin, total re-wrap is the only seam between the two drivers' fault shapes.
fn discovery_step_from_probe_fault(step: BreakawayProbeStep) -> AdditiveDiscoveryStep {
    match step {
        BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault { error, safety }) => {
            AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::SafetyFault { error, safety })
        }
        _ => unreachable!(
            "sample_one_tick only ever returns a SafetyFault-shaped Done, see its own docs"
        ),
    }
}

/// Positive commanded-magnitude target rate shared by both directions, matching
/// the existing `acceptance_target_rate_q` conversion in `integral_sweep.rs`.
fn discovery_target_rate_q(
    target_velocity_rpm: TmcMechanicalRpm,
    counts_per_rev: u32,
) -> Result<i64, CommissionError> {
    i64::from(target_velocity_rpm.raw().unsigned_abs())
        .checked_mul(i64::from(counts_per_rev))
        .and_then(|value| value.checked_mul(VELOCITY_RATE_Q_SCALE))
        .map(|value| value / 60)
        .ok_or(CommissionError::InvalidSchedule)
}

/// Discovery's own containment multiplier is applied with NO confirmation-style
/// inflation on top of it (`c_sigma = 1.0`, versus confirmation's
/// [`super::band_confirmation::NOMINATION_SCREEN_C_SIGMA`]) and with NO outward
/// endpoint rounding (versus confirmation's outward rounding at the exact 70/80
/// boundary).
///
/// This is a deliberate, documented divergence from confirmation, not
/// undocumented drift between two copies of the same math: both callers share
/// one implementation
/// ([`rung_response_bounds_percent`](super::band_confirmation::rung_response_bounds_percent)),
/// parameterized exactly by these two values. Discovery is the *permissive
/// collector* -- it only decides which rungs to hand to 6d/6e's nomination
/// screen, and it never authorizes Stage C itself, so silently excluding a
/// borderline-viable rung here (which discovery's own `Ambiguous` verdict would
/// do) is worse than collecting one that nomination's own conservative screen
/// rejects later. Confirmation is the *conservative gate* that actually
/// authorizes Stage C, so it inflates for a future held-out family-eight test
/// and defends the exact band boundary against floating-point rounding noise.
/// Neither behavior may drift out of sync with the other by accident: both are
/// two call sites into the same parameterized function.
const DISCOVERY_CONTAINMENT_C_SIGMA: f32 = 1.0;

/// One rung's classification against the 70-80% target band, judged entirely on
/// its own containment -- no interval-separation or response-model requirement
/// against neighboring rungs (spec section 6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RungBandVerdict {
    /// This rung's own response interval is wholly contained in 70-80%.
    InBand,
    /// This rung's own response interval is wholly above 80%: both directions are
    /// confidently past the band.
    ConfidentlyAboveBand,
    /// Neither of the above: still below the band, straddling it without full
    /// containment, or not judgeable (loss of consensus other than current
    /// headroom). Not a stop condition by itself; the ladder continues.
    Ambiguous,
}

/// Reduce one direction's four per-slot classifications to a single
/// moving/inert diagnostic verdict, mirroring
/// the discovery classification-to-motion mapping (`Eligible`/`DeficitUnresolved` => moving,
/// `Inert` => inert) but folded across a whole rung's four-slot consensus
/// schedule instead of a single probe observation: moving if any slot shows
/// motion, inert otherwise. Used only for the rung-zero diagnostic (spec
/// section 6), which is evidence, not a stop-rule input.
fn direction_motion_class(
    classes: [VelocityObservationClass; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
) -> BreakawayMotionClass {
    let moved = classes.iter().any(|class| {
        matches!(
            class,
            VelocityObservationClass::Eligible | VelocityObservationClass::DeficitUnresolved
        )
    });
    if moved {
        BreakawayMotionClass::Moving
    } else {
        BreakawayMotionClass::Inert
    }
}

fn classify_rung_band(rung: &DiscoveryRungNominationInput, own_multiplier: f32) -> RungBandVerdict {
    match rung_response_bounds_percent(rung, own_multiplier, DISCOVERY_CONTAINMENT_C_SIGMA, false) {
        Some((lower, upper))
            if lower >= f32::from(CONFIRMATION_BAND_LOWER_PERCENT)
                && upper <= f32::from(CONFIRMATION_BAND_UPPER_PERCENT) =>
        {
            RungBandVerdict::InBand
        }
        Some((lower, _upper)) if lower > f32::from(CONFIRMATION_BAND_UPPER_PERCENT) => {
            RungBandVerdict::ConfidentlyAboveBand
        }
        Some(_) | None => RungBandVerdict::Ambiguous,
    }
}

/// Conclude the discovery walk once a stop rule fires.
///
/// A non-empty in-band set means the target band was selected: enter
/// [`AdditiveDiscoveryState::StreamMargins`] to stream each collected rung's
/// margin on its own tick before the `TargetBandSelected` outcome (which
/// `StreamMargins` itself produces once the collected set is exhausted). An
/// empty in-band set terminates immediately with `empty_outcome`
/// (`TargetBandSkipped` or `CurrentHeadroomBeforeBand`). The walk is over at
/// this point, so the retained collected set no longer changes.
fn finish_discovery_walk(
    state: &mut AdditiveDiscoveryState,
    in_band_count: u8,
    empty_outcome: AdditiveDiscoveryOutcome,
) -> AdditiveDiscoveryStep {
    if in_band_count > 0 {
        *state = AdditiveDiscoveryState::StreamMargins { cursor: 0 };
        AdditiveDiscoveryStep::ReadyNow
    } else {
        AdditiveDiscoveryStep::Done(empty_outcome)
    }
}

/// Advance the additive discovery campaign by one cooperative step.
///
/// `workspace` is caller-owned scratch space; `session` is caller-owned campaign
/// state (see [`AdditiveDiscoverySession`]). Neither is allocated here. The caller
/// is responsible for running [`crate::commissioning::outer::check_outer_safety`]
/// against `safety` between ticks, exactly as `step_outer_with_trace` does for
/// every other outer phase; this function never performs that check itself,
/// matching [`step_breakaway_probe`].
pub(crate) async fn step_additive_discovery<B: CommissioningBackend>(
    state: &mut AdditiveDiscoveryState,
    workspace: &mut BreakawayProbeWorkspace,
    session: &mut AdditiveDiscoverySession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> AdditiveDiscoveryStep {
    match state {
        AdditiveDiscoveryState::PlannedDiscovery => {
            *state = AdditiveDiscoveryState::WriteGain;
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::WriteGain => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::GainProgramming,
            );
            let p = match session.current_gain() {
                Ok(p) => p,
                Err(error) => return discovery_fault(error),
            };
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_RAMP_MODE)
                    .await
                    .is_err()
                || clear_velocity_integrator(backend).await.is_err()
            {
                return discovery_fault(CommissionError::Spi);
            }
            *state = AdditiveDiscoveryState::CommandTarget;
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::CommandTarget => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::TargetCommand,
            );
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return discovery_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            let target = match session
                .target_velocity_rpm
                .with_direction(direction_sign(session.direction))
            {
                Ok(target) => target.raw(),
                Err(_) => return discovery_fault(CommissionError::InvalidSchedule),
            };
            let hard_limit = safety.torque_cap;
            let mut clamp_session =
                match VelocityCurrentClampSession::try_new(u32::from(hard_limit)) {
                    Ok(session) => session,
                    Err(error) => return discovery_current_guard_fault(error),
                };
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return discovery_current_guard_fault(error);
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
                return discovery_current_guard_fault(error);
            }
            if backend
                .write_register_verified(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                return discovery_fault(CommissionError::Spi);
            }
            if let Err(error) = safety.begin_primitive(backend.elapsed_us()) {
                return discovery_fault(error);
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(MOVING_STROKE_US)) else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            workspace.observations.clear();
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::Capture,
            );
            *state = AdditiveDiscoveryState::Capture {
                next_sample_us,
                deadline_us,
            };
            AdditiveDiscoveryStep::At(next_sample_us)
        }
        AdditiveDiscoveryState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return AdditiveDiscoveryStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return discovery_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_moving_output(value),
                Err(error) => return discovery_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                *state = AdditiveDiscoveryState::CommandZero;
                AdditiveDiscoveryStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        AdditiveDiscoveryStep::At(*next_sample_us)
                    }
                    Err(error) => discovery_fault(error),
                }
            }
        }
        AdditiveDiscoveryState::CommandZero => {
            let boundary =
                finish_velocity_current_moving_phase(&mut workspace.current_observer, backend)
                    .await;
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return discovery_fault(CommissionError::Spi);
            }
            if let Err(error) = boundary {
                return discovery_current_guard_fault(error);
            }
            if clear_velocity_integrator(backend).await.is_err() {
                return discovery_fault(CommissionError::Spi);
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) = now_us.checked_add(u64::from(ZERO_SETTLE_US)) else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
            else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            workspace.analysis.reset_with_structured_diagnostics();
            *state = AdditiveDiscoveryState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return discovery_fault(CommissionError::AnalysisOverrun);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let (stationarity, drift_ratio_ppm) = match poll {
                Ok(Some((result, _rejection, drift_ratio_ppm))) => (result, drift_ratio_ppm),
                Ok(None) => return AdditiveDiscoveryStep::ReadyNow,
                Err(error) => return discovery_fault(error),
            };
            let p = match session.current_gain() {
                Ok(p) => p,
                Err(error) => return discovery_fault(error),
            };
            let input = match assemble_velocity_observation_input(VelocityObservationInputRequest {
                direction: session.direction,
                p,
                stationarity,
                rejected_post_fit: None,
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: safety.torque_cap,
                initial_lead_in: false,
                confidence_family_size: session.plan.confidence_family_size,
                counts_per_rev: safety.counts_per_rev(),
            }) {
                Ok(input) => input,
                Err(error) => return discovery_fault(error),
            };
            // Mirror the probe's own AnalyzeMoving short-circuit (this file, above)
            // and sweep.rs's AnalyzeMoving `terminal_error` match (sweep.rs:1076):
            // wrong-direction and hard-clip mean the direction-to-motion assumption
            // origin-recovery relies on is untrustworthy, or current is already
            // clipped, so recovery must not run another energized stroke on top of
            // this observation. Invalid controller-domain evidence is likewise an
            // immediate terminal here. `CurrentHeadroom` is deliberately NOT a
            // short-circuit: discovery still runs settle + recovery for it, then
            // stops the campaign in `Advance` once recovery completes, exactly the
            // "current authority binds" stop condition from spec section 6.
            let classification = classify_velocity_observation(input).classification;
            match classification {
                VelocityObservationClass::WrongDirectionFault => {
                    return discovery_fault(CommissionError::VelocityValidationFailed);
                }
                VelocityObservationClass::HardClipFault => {
                    return discovery_fault(CommissionError::SafetyEnvelopeViolation);
                }
                VelocityObservationClass::InvalidEvidenceFault => {
                    return AdditiveDiscoveryStep::Done(
                        AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded {
                            error: CommissionError::VelocityValidationFailed,
                        },
                    );
                }
                VelocityObservationClass::CurrentHeadroom => {
                    session.pending_headroom_stop = true;
                }
                VelocityObservationClass::Eligible
                | VelocityObservationClass::Inert
                | VelocityObservationClass::DeficitUnresolved
                | VelocityObservationClass::StationarityExcluded(_) => {}
            }
            session.pending_lower_rate_q = commanded_side_lower_rate_q(
                input.direction,
                input.settled_rate_low_q,
                input.settled_rate_high_q,
            );
            let variance_of_mean_q2 = match stationarity {
                StationarityResult::Usable(suffix) => suffix.selection.variance_of_mean_q2,
                StationarityResult::Excluded(_) => 0,
            };
            let target_rate_q =
                match discovery_target_rate_q(session.target_velocity_rpm, safety.counts_per_rev())
                {
                    Ok(value) => value,
                    Err(error) => return discovery_fault(error),
                };
            session.pending_record = Some(PendingDiscoveryRecord {
                classification,
                interval: input.disturbance.map(|(_, interval)| interval),
                atom: PendingObservationAtom {
                    mean_rate_q: input.settled_rate_mean_q,
                    variance_of_mean_q2,
                    target_rate_q,
                },
                classifier_inputs: observation_classifier_inputs(
                    &input,
                    workspace.current_observer.moving_pid_output_peak(),
                ),
            });
            workspace.analysis.reset();
            workspace.observations.clear();
            *state = AdditiveDiscoveryState::BeginSettleCapture {
                settle_deadline_us: *settle_deadline_us,
                retry_available: true,
            };
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::BeginSettleCapture {
            settle_deadline_us,
            retry_available,
        } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return discovery_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return discovery_fault(CommissionError::AnalysisOverrun);
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            *state = AdditiveDiscoveryState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
                retry_available: *retry_available,
            };
            AdditiveDiscoveryStep::At(next_sample_us)
        }
        AdditiveDiscoveryState::SettleCapture {
            next_sample_us,
            deadline_us,
            retry_available,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return AdditiveDiscoveryStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return discovery_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_zero_output(value),
                Err(error) => return discovery_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return discovery_fault(CommissionError::InvalidSchedule);
                };
                workspace
                    .analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        workspace.settle_min_displacement_counts,
                        workspace.settle_max_displacement_counts,
                    ));
                *state = AdditiveDiscoveryState::AnalyzeSettle {
                    analysis_deadline_us,
                    retry_available: *retry_available,
                };
                AdditiveDiscoveryStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        AdditiveDiscoveryStep::At(*next_sample_us)
                    }
                    Err(error) => discovery_fault(error),
                }
            }
        }
        AdditiveDiscoveryState::AnalyzeSettle {
            analysis_deadline_us,
            retry_available,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return discovery_fault(CommissionError::AnalysisOverrun);
            }
            let confirmation = match poll_scored_rest_confirmation(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(ScoredRestPoll::Pending | ScoredRestPoll::Diagnostic(_)) => {
                    return AdditiveDiscoveryStep::ReadyNow;
                }
                Ok(ScoredRestPoll::Complete(confirmation)) => confirmation,
                Err(error) => return discovery_fault(error),
            };
            if let Err(error) =
                finish_velocity_current_zero_phase(&mut workspace.current_observer, backend).await
            {
                return discovery_current_guard_fault(error);
            }
            if workspace
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .is_err()
            {
                return discovery_fault(CommissionError::InvalidSchedule);
            }
            if !confirmation.confirmed_rest && *retry_available {
                workspace.analysis.reset();
                workspace.observations.clear();
                let Some(settle_deadline_us) =
                    backend.elapsed_us().checked_add(u64::from(ZERO_SETTLE_US))
                else {
                    return discovery_fault(CommissionError::InvalidSchedule);
                };
                *state = AdditiveDiscoveryState::BeginSettleCapture {
                    settle_deadline_us,
                    retry_available: false,
                };
                return AdditiveDiscoveryStep::ReadyNow;
            }
            workspace.analysis.reset();
            workspace.observations.clear();
            safety.end_primitive();
            if !confirmation.confirmed_rest {
                return AdditiveDiscoveryStep::Done(
                    AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded {
                        error: CommissionError::VelocityRestNotConfirmed,
                    },
                );
            }
            let recovery_plan = match plan_rung_origin_recovery(
                safety.budget,
                safety.counts_per_rev(),
                session.planned_velocity_mrev_s,
            ) {
                Ok(plan) => plan,
                Err(_) => return discovery_fault(CommissionError::InvalidSchedule),
            };
            let p = match session.current_gain() {
                Ok(p) => p,
                Err(error) => return discovery_fault(error),
            };
            let hard_limit = safety.torque_cap;
            let clamp_limit = match PidTorqueFluxLimit::new(u32::from(hard_limit)) {
                Ok(limit) => limit,
                Err(_) => return discovery_fault(CommissionError::InvalidSchedule),
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
                    rung_index: session.rung_index,
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
                Err(error) => return discovery_fault(error),
            };
            *state = AdditiveDiscoveryState::Recover(runtime);
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::Recover(runtime) => {
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
                RungOriginRecoveryStep::ReadyNow => AdditiveDiscoveryStep::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => AdditiveDiscoveryStep::At(deadline_us),
                RungOriginRecoveryStep::RestSelection { .. }
                | RungOriginRecoveryStep::RestAnalysis(_) => AdditiveDiscoveryStep::ReadyNow,
                RungOriginRecoveryStep::Finished(_) | RungOriginRecoveryStep::Contained(_) => {
                    // The stroke is complete and classified: stream its raw
                    // settled-rate atom now, on the collection tick, before the
                    // `Advance` state folds it into this rung's nomination
                    // bounds and drops the atom. `session.rung_index`/`slot`
                    // still name this observation's coordinates -- `Advance`
                    // advances them only after it records.
                    *state = AdditiveDiscoveryState::Advance;
                    match session.pending_record.as_ref() {
                        Some(record) => AdditiveDiscoveryStep::Observation(CollectedObservation {
                            atom: record.atom.with_direction(session.direction),
                            rung_index: session.rung_index,
                            slot_index: session.slot,
                            classifier_inputs: record.classifier_inputs,
                            stroke_min_displacement_counts: 0,
                            stroke_max_displacement_counts: 0,
                            capture_max_interval_us: 0,
                        }),
                        None => AdditiveDiscoveryStep::ReadyNow,
                    }
                }
                RungOriginRecoveryStep::RestUnconfirmed => AdditiveDiscoveryStep::Done(
                    AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded {
                        error: CommissionError::VelocityRestNotConfirmed,
                    },
                ),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: Some(fault_report),
                } => discovery_safety_fault(fault_report, error),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => discovery_fault(error),
            }
        }
        AdditiveDiscoveryState::Advance => {
            let record = session
                .pending_record
                .take()
                .expect("classified before recovery started");
            let atom = record.atom.with_direction(session.direction);
            match session.direction {
                VelocityDirection::Forward => session.forward.record(
                    session.slot,
                    record.classification,
                    record.interval,
                    atom,
                ),
                VelocityDirection::Reverse => session.reverse.record(
                    session.slot,
                    record.classification,
                    record.interval,
                    atom,
                ),
            }

            if session.pending_headroom_stop {
                return finish_discovery_walk(
                    state,
                    session.in_band.count(),
                    AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand,
                );
            }

            let Some(next_slot) = session.slot.checked_add(1) else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            if usize::from(next_slot) < CONSENSUS_OBSERVATIONS_PER_DIRECTION {
                session.slot = next_slot;
                *state = AdditiveDiscoveryState::WriteGain;
                return AdditiveDiscoveryStep::ReadyNow;
            }

            match session.direction {
                VelocityDirection::Forward => {
                    session.direction = VelocityDirection::Reverse;
                    session.slot = 0;
                    *state = AdditiveDiscoveryState::WriteGain;
                    AdditiveDiscoveryStep::ReadyNow
                }
                VelocityDirection::Reverse => {
                    let p = match session.current_gain() {
                        Ok(p) => p,
                        Err(error) => return discovery_fault(error),
                    };
                    let rung_input = DiscoveryRungNominationInput {
                        p,
                        forward_consensus: session.forward.consensus(),
                        forward_observations: session.forward.atoms,
                        reverse_consensus: session.reverse.consensus(),
                        reverse_observations: session.reverse.atoms,
                    };
                    let verdict = classify_rung_band(&rung_input, session.own_multiplier);
                    // Retain rung zero's own per-direction motion classification before
                    // its accumulators reset below, regardless of `verdict`: rung zero
                    // (the diagnostic floor, expected inert) is never pushed into
                    // `in_band` unless it happens to classify `InBand`, so without this
                    // capture its evidence would be silently discarded. Every other
                    // rung's handling is unchanged.
                    if session.rung_index == 0 {
                        session.rung_zero_classification = Some((
                            direction_motion_class(session.forward.classes),
                            direction_motion_class(session.reverse.classes),
                        ));
                    }
                    if verdict == RungBandVerdict::InBand {
                        // Compute the NOMINATION interval and eligibility now, while
                        // the raw atoms are still fresh, and retain only the compact
                        // result: nomination never needs the atoms again after this
                        // point. This is a SEPARATE computation from `classify_rung_band`
                        // above -- discovery's own stop-rule containment check, unchanged
                        // (`DISCOVERY_CONTAINMENT_C_SIGMA`, no rounding) -- not a
                        // replacement for it.
                        let bounds = rung_nomination_bounds(&rung_input);
                        if session.in_band.push(bounds).is_err() {
                            return discovery_fault(CommissionError::InvalidSchedule);
                        }
                    }
                    if verdict == RungBandVerdict::ConfidentlyAboveBand {
                        return finish_discovery_walk(
                            state,
                            session.in_band.count(),
                            AdditiveDiscoveryOutcome::TargetBandSkipped,
                        );
                    }

                    session.forward = DirectionAccumulator::new();
                    session.reverse = DirectionAccumulator::new();

                    let Some(next_rung) = session.rung_index.checked_add(1) else {
                        return discovery_fault(CommissionError::InvalidSchedule);
                    };
                    if usize::from(next_rung) >= usize::from(session.plan.rung_count) {
                        return finish_discovery_walk(
                            state,
                            session.in_band.count(),
                            AdditiveDiscoveryOutcome::TargetBandSkipped,
                        );
                    }
                    session.rung_index = next_rung;
                    session.direction = VelocityDirection::Forward;
                    session.slot = 0;
                    *state = AdditiveDiscoveryState::WriteGain;
                    AdditiveDiscoveryStep::ReadyNow
                }
            }
        }
        AdditiveDiscoveryState::StreamMargins { cursor } => {
            let index = *cursor;
            if usize::from(index) < usize::from(session.in_band.count()) {
                *cursor = index.saturating_add(1);
                AdditiveDiscoveryStep::Margin {
                    collected_index: index,
                }
            } else {
                AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::TargetBandSelected(
                    session.in_band,
                ))
            }
        }
    }
}

// ============================================================================
// Held-out confirmation driver
// ============================================================================

/// One classified confirmation stroke's evidence, held between
/// [`BandConfirmationState::AnalyzeMoving`] and [`BandConfirmationState::Advance`],
/// mirroring [`PendingDiscoveryRecord`]'s role for discovery.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingConfirmationRecord {
    classification: VelocityObservationClass,
    interval: Option<EquivalentTorqueTargetInterval>,
    atom: PendingObservationAtom,
    classifier_inputs: ObservationClassifierInputs,
}

/// Terminal result of the held-out confirmation campaign.
///
/// [`Self::Accepted`] is the ONLY variant that may authorize Stage C (a later
/// slice's job to route); every other variant is a confirmation-owned
/// terminal and must not. Matching every other outer velocity phase, this
/// driver never asserts `set_enable_pin` itself, and centralized safe-state
/// restoration on any terminal happens at the surrounding engine boundary,
/// not per-phase.
#[derive(Debug, defmt::Format)]
pub(crate) enum BandConfirmationOutcome {
    /// All eight held-out strokes were collected with complete
    /// `collected == eligible == included == 0b1111` masks in both directions
    /// and every fresh interval contained in 70-80%. Carries the confirmed
    /// gain and the frozen plan for a later slice's Stage-C construction.
    Accepted {
        /// The held-out acceptance itself.
        acceptance: BandConfirmationAcceptance,
        /// The frozen confirmation plan the acceptance was judged against.
        plan: BandConfirmationPlan,
    },
    /// No discovery rung's own margin reached zero (spec section 7); zero
    /// confirmation strokes were spent.
    TargetBandUnconfirmableFromDiscovery,
    /// Current authority bound before discovery ever collected an in-band
    /// rung (spec section 9). Consumed directly from
    /// [`AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand`] rather than
    /// re-derived here -- 6c has already fully classified this condition.
    /// Zero confirmation strokes were spent.
    TargetBandNotReachedAtAuthority,
    /// The held-out family-eight block failed strict acceptance; `cause`
    /// reports why.
    TargetBandConfirmationInconclusive(BandConfirmationFailureCause),
    /// Target-independent stationarity evidence, or another controller-domain
    /// evidence field, was unusable for classification -- or the completed
    /// origin-recovery return could not confirm rest.
    ConfirmationEvidenceExcluded,
    /// An existing safety, current, travel, encoder, or register-I/O fault,
    /// returned unchanged.
    SafetyFault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

/// Outcome of routing a finished discovery campaign into confirmation.
///
/// `Ready` is far larger than `Done`: it embeds [`BandConfirmationSession`],
/// which carries two [`DirectionAccumulator`]s worth of per-slot evidence.
/// `large_enum_variant` is accepted here the same way it already is at
/// [`AdditiveDiscoveryStep`] and the other terminal types in this crate,
/// rather than boxed -- this crate has no heap.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BandConfirmationEntry {
    /// The confirmation plan is frozen; step the returned session from
    /// [`BandConfirmationState::PlannedConfirmation`] via
    /// [`step_band_confirmation`] to run the eight-stroke block.
    Ready(BandConfirmationSession),
    /// A confirmation-owned terminal was reached before any confirmation
    /// stroke; zero strokes were spent.
    Done(BandConfirmationOutcome),
}

/// Nominate a candidate from a finished discovery campaign and freeze the
/// held-out confirmation plan. Touches no backend and spends zero
/// confirmation strokes; motion is only reachable afterward, through
/// [`step_band_confirmation`].
///
/// Confirmation may only begin from the two discovery outcomes that leave a
/// meaningful confirmation question open:
/// [`AdditiveDiscoveryOutcome::TargetBandSelected`] (screen the collected
/// rungs through [`nominate_confirmation_candidate_from_bounds`]) and
/// [`AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand`] (consumed directly
/// as [`BandConfirmationOutcome::TargetBandNotReachedAtAuthority`], spec
/// section 9 -- 6c has already fully classified this condition, so it is not
/// re-derived here). Every other discovery outcome is itself already a
/// discovery-owned terminal that ends the whole outer campaign before
/// confirmation is ever reached; handing one to this function is a caller
/// (routing) defect, not a confirmation result, and is reported as
/// `InvalidSchedule` rather than panicking.
pub(crate) fn begin_band_confirmation(
    discovery_outcome: AdditiveDiscoveryOutcome,
    discovery_plan_digest: u64,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
) -> BandConfirmationEntry {
    let rungs = match discovery_outcome {
        AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand => {
            return BandConfirmationEntry::Done(
                BandConfirmationOutcome::TargetBandNotReachedAtAuthority,
            );
        }
        AdditiveDiscoveryOutcome::TargetBandSelected(rungs) => rungs,
        AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded
        | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { .. } => {
            return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
                error: CommissionError::InvalidSchedule,
                safety: None,
            });
        }
        AdditiveDiscoveryOutcome::SafetyFault { error, safety } => {
            return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
                error,
                safety,
            });
        }
    };
    let candidate = match nominate_confirmation_candidate_from_bounds(rungs.as_slice()) {
        Ok(candidate) => candidate,
        Err(TargetBandUnconfirmableFromDiscovery) => {
            return BandConfirmationEntry::Done(
                BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery,
            );
        }
    };
    let plan = match plan_band_confirmation(discovery_plan_digest, candidate.p) {
        Ok(plan) => plan,
        Err(_) => {
            return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
                error: CommissionError::InvalidSchedule,
                safety: None,
            });
        }
    };
    // Single canonical contract check, run before the fixed eight-stroke block
    // drives any motion. The same `validate_fixed_contract` gates Stage-C
    // construction, so a plan that reaches confirmation and a plan that reaches
    // Stage C are held to one identical, non-drifting contract.
    if validate_fixed_contract(&plan).is_err() {
        return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
            error: CommissionError::InvalidSchedule,
            safety: None,
        });
    }
    BandConfirmationEntry::Ready(BandConfirmationSession::new(
        plan,
        candidate.margin_percent,
        target_velocity_rpm,
        planned_velocity_mrev_s,
    ))
}

/// Frozen confirmation plan and the mutable eight-stroke walk progress.
///
/// Bundles the immutable [`BandConfirmationPlan`] together with the
/// fixed-schedule stroke index and the two per-direction accumulators,
/// mirroring [`AdditiveDiscoverySession`]'s role for discovery. Constructed
/// only via [`begin_band_confirmation`], after nomination and plan-freeze have
/// already happened, so [`step_band_confirmation`] is the only path to
/// motion.
#[derive(Debug, defmt::Format)]
pub(crate) struct BandConfirmationSession {
    plan: BandConfirmationPlan,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    stroke_index: u8,
    forward: DirectionAccumulator,
    reverse: DirectionAccumulator,
    pending_lower_rate_q: Option<u64>,
    pending_record: Option<PendingConfirmationRecord>,
    /// The nominated candidate's `M_i = min(L_i - 70, 80 - U_i)`, retained
    /// (rather than the whole [`NominatedConfirmationCandidate`], which
    /// `begin_band_confirmation` otherwise fully consumes) so
    /// [`confirmation_plan_report`] can be assembled at report time.
    nominated_margin_percent: f32,
}

impl BandConfirmationSession {
    fn new(
        plan: BandConfirmationPlan,
        nominated_margin_percent: f32,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
    ) -> Self {
        Self {
            plan,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            stroke_index: 0,
            forward: DirectionAccumulator::new(),
            reverse: DirectionAccumulator::new(),
            pending_lower_rate_q: None,
            pending_record: None,
            nominated_margin_percent,
        }
    }

    /// Commanded direction for the stroke about to run (or just completed),
    /// read from the fixed [`CONFIRMATION_SLOT_ORDER`] schedule.
    fn current_direction(&self) -> VelocityDirection {
        CONFIRMATION_SLOT_ORDER[usize::from(self.stroke_index)]
    }

    /// This stroke's position within its own direction's four-slot schedule.
    ///
    /// `CONFIRMATION_SLOT_ORDER` is four forward strokes followed by four
    /// reverse strokes, so `stroke_index % 4` is always the correct
    /// within-direction slot -- matching both
    /// `CONFIRMATION_OBSERVATIONS_PER_DIRECTION` and
    /// `CONSENSUS_OBSERVATIONS_PER_DIRECTION`, both 4.
    fn slot_within_direction(&self) -> u8 {
        self.stroke_index % 4
    }
}

/// Single-tick cooperative stroke cycle for one confirmation observation.
///
/// Mirrors [`AdditiveDiscoveryState`]/[`BreakawayProbeState`]'s stroke states
/// exactly (write gain, command target, capture, command zero, analyze
/// moving, settle-capture, analyze settle, recover, advance): a confirmation
/// stroke is the same guarded primitive, at a frozen gain and a fixed
/// eight-slot schedule instead of a bisection cursor or a ladder walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum BandConfirmationState {
    /// The frozen plan is admitted; no stroke has been issued yet.
    PlannedConfirmation,
    /// Write the frozen candidate gain and the outer-velocity ramp mode.
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
    /// Fold the classified observation into the active accumulator and decide
    /// what is next: another stroke, or the held-out judgement.
    Advance,
}

/// One cooperative result from [`step_band_confirmation`].
#[derive(Debug, defmt::Format)]
pub(crate) enum BandConfirmationStep {
    /// Call again immediately; no external wait is needed.
    ReadyNow,
    /// Call again no earlier than this `elapsed_us` deadline.
    At(u64),
    /// A stroke just completed; its raw settled-rate evidence is ready to stream
    /// before the atom is folded and judged. Call again immediately; the fold
    /// happens on the following tick.
    Observation(CollectedObservation),
    /// The confirmation campaign concluded.
    Done(BandConfirmationOutcome),
}

fn confirmation_fault(error: CommissionError) -> BandConfirmationStep {
    BandConfirmationStep::Done(BandConfirmationOutcome::SafetyFault {
        error,
        safety: None,
    })
}

fn confirmation_safety_fault(
    safety: OuterSafetyFault,
    error: CommissionError,
) -> BandConfirmationStep {
    BandConfirmationStep::Done(BandConfirmationOutcome::SafetyFault {
        error,
        safety: Some(safety),
    })
}

fn confirmation_current_guard_fault(error: VelocityCurrentGuardError) -> BandConfirmationStep {
    confirmation_fault(match error {
        VelocityCurrentGuardError::InvalidLimit => CommissionError::InvalidSchedule,
        VelocityCurrentGuardError::RegisterIo => CommissionError::Spi,
        VelocityCurrentGuardError::ActionableStatus(_) => CommissionError::SafetyEnvelopeViolation,
    })
}

/// Adapt [`sample_one_tick`]'s probe-flavored fault into a confirmation step.
///
/// Mirrors [`discovery_step_from_probe_fault`]: `sample_one_tick` only ever
/// fails through its own `fault`/`safety_fault` constructors, both of which
/// resolve to `BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault { .. })`;
/// no other probe outcome variant is reachable from it.
fn confirmation_step_from_probe_fault(step: BreakawayProbeStep) -> BandConfirmationStep {
    match step {
        BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault { error, safety }) => {
            BandConfirmationStep::Done(BandConfirmationOutcome::SafetyFault { error, safety })
        }
        _ => unreachable!(
            "sample_one_tick only ever returns a SafetyFault-shaped Done, see its own docs"
        ),
    }
}

/// Map [`evaluate_band_confirmation`]'s result onto this driver's own outcome
/// type, attaching `plan` to an acceptance.
///
/// Extracted as its own pure function (taking the already-built directional
/// inputs and the already-evaluated result, rather than a session) so a test
/// can exercise this exact mapping directly against crafted
/// [`ConfirmationDirectionalInput`] fixtures -- mirroring
/// `band_confirmation_tests.rs`'s own fixture style -- without needing to
/// reproduce every [`BandConfirmationFailureCause`] via a physically
/// simulated encoder script: `evaluate_band_confirmation`'s own statistical
/// math is exhaustively covered there and is not re-derived here.
fn confirmation_evaluation_outcome(
    result: Result<BandConfirmationAcceptance, TargetBandConfirmationInconclusive>,
    plan: BandConfirmationPlan,
) -> BandConfirmationOutcome {
    match result {
        Ok(acceptance) => BandConfirmationOutcome::Accepted { acceptance, plan },
        Err(TargetBandConfirmationInconclusive(cause)) => {
            BandConfirmationOutcome::TargetBandConfirmationInconclusive(cause)
        }
    }
}

/// Fold this session's two completed direction accumulators into the held-out
/// confirmation judgement.
fn evaluate_confirmation_session(session: &BandConfirmationSession) -> BandConfirmationOutcome {
    let forward = ConfirmationDirectionalInput {
        consensus: session.forward.consensus(),
        classes: session.forward.classes,
        observations: session.forward.atoms,
    };
    let reverse = ConfirmationDirectionalInput {
        consensus: session.reverse.consensus(),
        classes: session.reverse.classes,
        observations: session.reverse.atoms,
    };
    confirmation_evaluation_outcome(
        evaluate_band_confirmation(session.plan.candidate_p, forward, reverse),
        session.plan,
    )
}

/// Advance the confirmation campaign by one cooperative step.
///
/// `workspace` is caller-owned scratch space; `session` is caller-owned
/// campaign state (see [`BandConfirmationSession`]). Neither is allocated
/// here. The caller is responsible for running
/// [`crate::commissioning::outer::check_outer_safety`] against `safety`
/// between ticks, exactly as `step_outer_with_trace` does for every other
/// outer phase; this function never performs that check itself, matching
/// [`step_breakaway_probe`] and [`step_additive_discovery`].
pub(crate) async fn step_band_confirmation<B: CommissioningBackend>(
    state: &mut BandConfirmationState,
    workspace: &mut BreakawayProbeWorkspace,
    session: &mut BandConfirmationSession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> BandConfirmationStep {
    match state {
        BandConfirmationState::PlannedConfirmation => {
            *state = BandConfirmationState::WriteGain;
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::WriteGain => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::GainProgramming,
            );
            let p = session.plan.candidate_p;
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_RAMP_MODE)
                    .await
                    .is_err()
                || clear_velocity_integrator(backend).await.is_err()
            {
                return confirmation_fault(CommissionError::Spi);
            }
            *state = BandConfirmationState::CommandTarget;
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::CommandTarget => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::TargetCommand,
            );
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return confirmation_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            let target = match session
                .target_velocity_rpm
                .with_direction(direction_sign(session.current_direction()))
            {
                Ok(target) => target.raw(),
                Err(_) => return confirmation_fault(CommissionError::InvalidSchedule),
            };
            let hard_limit = safety.torque_cap;
            let mut clamp_session =
                match VelocityCurrentClampSession::try_new(u32::from(hard_limit)) {
                    Ok(session) => session,
                    Err(error) => return confirmation_current_guard_fault(error),
                };
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return confirmation_current_guard_fault(error);
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
                return confirmation_current_guard_fault(error);
            }
            if backend
                .write_register_verified(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                return confirmation_fault(CommissionError::Spi);
            }
            if let Err(error) = safety.begin_primitive(backend.elapsed_us()) {
                return confirmation_fault(error);
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(MOVING_STROKE_US)) else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            workspace.observations.clear();
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::Capture,
            );
            *state = BandConfirmationState::Capture {
                next_sample_us,
                deadline_us,
            };
            BandConfirmationStep::At(next_sample_us)
        }
        BandConfirmationState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BandConfirmationStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return confirmation_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_moving_output(value),
                Err(error) => return confirmation_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                *state = BandConfirmationState::CommandZero;
                BandConfirmationStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BandConfirmationStep::At(*next_sample_us)
                    }
                    Err(error) => confirmation_fault(error),
                }
            }
        }
        BandConfirmationState::CommandZero => {
            let boundary =
                finish_velocity_current_moving_phase(&mut workspace.current_observer, backend)
                    .await;
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return confirmation_fault(CommissionError::Spi);
            }
            if let Err(error) = boundary {
                return confirmation_current_guard_fault(error);
            }
            if clear_velocity_integrator(backend).await.is_err() {
                return confirmation_fault(CommissionError::Spi);
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) = now_us.checked_add(u64::from(ZERO_SETTLE_US)) else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
            else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            workspace.analysis.reset_with_structured_diagnostics();
            *state = BandConfirmationState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return confirmation_fault(CommissionError::AnalysisOverrun);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let (stationarity, drift_ratio_ppm) = match poll {
                Ok(Some((result, _rejection, drift_ratio_ppm))) => (result, drift_ratio_ppm),
                Ok(None) => return BandConfirmationStep::ReadyNow,
                Err(error) => return confirmation_fault(error),
            };
            let p = session.plan.candidate_p;
            let input = match assemble_velocity_observation_input(VelocityObservationInputRequest {
                direction: session.current_direction(),
                p,
                stationarity,
                rejected_post_fit: None,
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: safety.torque_cap,
                initial_lead_in: false,
                confidence_family_size: CONFIRMATION_FAMILY_SIZE,
                counts_per_rev: safety.counts_per_rev(),
            }) {
                Ok(input) => input,
                Err(error) => return confirmation_fault(error),
            };
            // Mirrors the probe's and discovery's own AnalyzeMoving short-circuit
            // (this file, above): wrong-direction and hard-clip mean the
            // direction-to-motion assumption origin-recovery relies on is
            // untrustworthy, or current is already clipped, so recovery must not
            // run another energized stroke on top of this observation. Invalid
            // controller-domain evidence is likewise an immediate terminal here.
            // `CurrentHeadroom` is deliberately NOT a short-circuit and does NOT
            // stop the campaign early (unlike discovery): the confirmation
            // contract requires the fixed eight-stroke block to run to
            // completion regardless, and `evaluate_band_confirmation` itself
            // reports incomplete-mask exclusions traced to current headroom as
            // `BandConfirmationFailureCause::CurrentAuthority`.
            let classification = classify_velocity_observation(input).classification;
            match classification {
                VelocityObservationClass::WrongDirectionFault => {
                    return confirmation_fault(CommissionError::VelocityValidationFailed);
                }
                VelocityObservationClass::HardClipFault => {
                    return confirmation_fault(CommissionError::SafetyEnvelopeViolation);
                }
                VelocityObservationClass::InvalidEvidenceFault => {
                    return BandConfirmationStep::Done(
                        BandConfirmationOutcome::ConfirmationEvidenceExcluded,
                    );
                }
                VelocityObservationClass::Eligible
                | VelocityObservationClass::Inert
                | VelocityObservationClass::DeficitUnresolved
                | VelocityObservationClass::CurrentHeadroom
                | VelocityObservationClass::StationarityExcluded(_) => {}
            }
            session.pending_lower_rate_q = commanded_side_lower_rate_q(
                input.direction,
                input.settled_rate_low_q,
                input.settled_rate_high_q,
            );
            let variance_of_mean_q2 = match stationarity {
                StationarityResult::Usable(suffix) => suffix.selection.variance_of_mean_q2,
                StationarityResult::Excluded(_) => 0,
            };
            let target_rate_q =
                match discovery_target_rate_q(session.target_velocity_rpm, safety.counts_per_rev())
                {
                    Ok(value) => value,
                    Err(error) => return confirmation_fault(error),
                };
            session.pending_record = Some(PendingConfirmationRecord {
                classification,
                interval: input.disturbance.map(|(_, interval)| interval),
                atom: PendingObservationAtom {
                    mean_rate_q: input.settled_rate_mean_q,
                    variance_of_mean_q2,
                    target_rate_q,
                },
                classifier_inputs: observation_classifier_inputs(
                    &input,
                    workspace.current_observer.moving_pid_output_peak(),
                ),
            });
            workspace.analysis.reset();
            workspace.observations.clear();
            *state = BandConfirmationState::BeginSettleCapture {
                settle_deadline_us: *settle_deadline_us,
            };
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::BeginSettleCapture { settle_deadline_us } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return confirmation_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return confirmation_fault(CommissionError::AnalysisOverrun);
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            *state = BandConfirmationState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
            };
            BandConfirmationStep::At(next_sample_us)
        }
        BandConfirmationState::SettleCapture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BandConfirmationStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return confirmation_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_zero_output(value),
                Err(error) => return confirmation_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return confirmation_fault(CommissionError::InvalidSchedule);
                };
                workspace
                    .analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        workspace.settle_min_displacement_counts,
                        workspace.settle_max_displacement_counts,
                    ));
                *state = BandConfirmationState::AnalyzeSettle {
                    analysis_deadline_us,
                };
                BandConfirmationStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BandConfirmationStep::At(*next_sample_us)
                    }
                    Err(error) => confirmation_fault(error),
                }
            }
        }
        BandConfirmationState::AnalyzeSettle {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return confirmation_fault(CommissionError::AnalysisOverrun);
            }
            let confirmation = match poll_scored_rest_confirmation(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(ScoredRestPoll::Pending | ScoredRestPoll::Diagnostic(_)) => {
                    return BandConfirmationStep::ReadyNow;
                }
                Ok(ScoredRestPoll::Complete(confirmation)) => confirmation,
                Err(error) => return confirmation_fault(error),
            };
            if let Err(error) =
                finish_velocity_current_zero_phase(&mut workspace.current_observer, backend).await
            {
                return confirmation_current_guard_fault(error);
            }
            if workspace
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .is_err()
            {
                return confirmation_fault(CommissionError::InvalidSchedule);
            }
            workspace.analysis.reset();
            workspace.observations.clear();
            safety.end_primitive();
            if !confirmation.confirmed_rest {
                return BandConfirmationStep::Done(
                    BandConfirmationOutcome::ConfirmationEvidenceExcluded,
                );
            }
            let recovery_plan = match plan_rung_origin_recovery(
                safety.budget,
                safety.counts_per_rev(),
                session.planned_velocity_mrev_s,
            ) {
                Ok(plan) => plan,
                Err(_) => return confirmation_fault(CommissionError::InvalidSchedule),
            };
            let p = session.plan.candidate_p;
            let hard_limit = safety.torque_cap;
            let clamp_limit = match PidTorqueFluxLimit::new(u32::from(hard_limit)) {
                Ok(limit) => limit,
                Err(_) => return confirmation_fault(CommissionError::InvalidSchedule),
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
                    rung_index: session.stroke_index,
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
                Err(error) => return confirmation_fault(error),
            };
            *state = BandConfirmationState::Recover(runtime);
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::Recover(runtime) => {
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
                RungOriginRecoveryStep::ReadyNow => BandConfirmationStep::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => BandConfirmationStep::At(deadline_us),
                RungOriginRecoveryStep::RestSelection { .. }
                | RungOriginRecoveryStep::RestAnalysis(_) => BandConfirmationStep::ReadyNow,
                RungOriginRecoveryStep::Finished(_) | RungOriginRecoveryStep::Contained(_) => {
                    // The held-out stroke is complete and classified: stream its
                    // raw settled-rate atom now, on the collection tick, before
                    // the `Advance` state folds it and the block is judged for
                    // [70, 80] containment. The fixed schedule slot is the
                    // within-direction slot for forward strokes and `+ 4` for
                    // reverse, matching `confirmation_observation_report`'s own
                    // slot numbering.
                    let slot_index = match session.current_direction() {
                        VelocityDirection::Forward => session.slot_within_direction(),
                        VelocityDirection::Reverse => session.slot_within_direction() + 4,
                    };
                    *state = BandConfirmationState::Advance;
                    match session.pending_record.as_ref() {
                        Some(record) => BandConfirmationStep::Observation(CollectedObservation {
                            atom: record.atom.with_direction(session.current_direction()),
                            rung_index: 0,
                            slot_index,
                            classifier_inputs: record.classifier_inputs,
                            stroke_min_displacement_counts: 0,
                            stroke_max_displacement_counts: 0,
                            capture_max_interval_us: 0,
                        }),
                        None => BandConfirmationStep::ReadyNow,
                    }
                }
                RungOriginRecoveryStep::RestUnconfirmed => BandConfirmationStep::Done(
                    BandConfirmationOutcome::ConfirmationEvidenceExcluded,
                ),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: Some(fault_report),
                } => confirmation_safety_fault(fault_report, error),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => confirmation_fault(error),
            }
        }
        BandConfirmationState::Advance => {
            let record = session
                .pending_record
                .take()
                .expect("classified before recovery started");
            let atom = record.atom.with_direction(session.current_direction());
            match session.current_direction() {
                VelocityDirection::Forward => session.forward.record(
                    session.slot_within_direction(),
                    record.classification,
                    record.interval,
                    atom,
                ),
                VelocityDirection::Reverse => session.reverse.record(
                    session.slot_within_direction(),
                    record.classification,
                    record.interval,
                    atom,
                ),
            }

            let Some(next_stroke) = session.stroke_index.checked_add(1) else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            if usize::from(next_stroke) < CONFIRMATION_SLOT_ORDER.len() {
                session.stroke_index = next_stroke;
                *state = BandConfirmationState::WriteGain;
                return BandConfirmationStep::ReadyNow;
            }

            BandConfirmationStep::Done(evaluate_confirmation_session(session))
        }
    }
}

// ============================================================================
// Campaign orchestrator
// ============================================================================

/// Frozen physical and envelope inputs the campaign retains across every phase.
///
/// Computed once at [`BreakawayCampaign::new`], before any probe motion: the
/// current-representable `(p_start, p_top)` gain span (shared with the classic
/// sweep via [`current_gain_span`]), the physical acquisition parameters the
/// Stage-C tail needs, and the reserved command-level workflow envelope. The
/// probe outcome supplies only the resolved breakaway grid index; every other
/// discovery-plan input is derived from these retained values, matching the
/// spec's plan-time authority-ceiling rule (`authority_ceiling = p_top`).
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct BreakawayCampaignInputs {
    p_start: PidPGain,
    p_top: PidPGain,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    hard_torque_limit: u16,
    usable_torque_limit: u16,
    limit_evidence_capability: VelocityLimitEvidenceCapability,
    observations_per_direction: u16,
    max_duration_ms: u16,
    workflow_envelope_ms: u32,
    channel: u8,
    run_sequence: u32,
    /// Plan-time PWM carrier frequency, in Hz, frozen from the command request.
    /// Cross-checked against the live PWM frequency at Stage-C construction so
    /// confirmation evidence cannot be admitted under a different timebase.
    pwm_hz: u32,
    /// Plan-time encoder counts per mechanical revolution, frozen from the
    /// command request. Cross-checked against the live resolution at Stage-C
    /// construction.
    encoder_counts_per_rev: u32,
    /// `sweep_plan::CEILING_BINDING_CURRENT_LIMIT` or
    /// `sweep_plan::CEILING_BINDING_REPRESENTABILITY_CLAMP`: which `p_top`
    /// candidate bound the plan-time authority ceiling this campaign's probe
    /// and discovery ladders are both anchored against.
    ceiling_binding_source: u8,
}

/// Frozen command request the campaign is planned from.
///
/// Every field is a command-admission input; `nominal_workflow_ms` and
/// `maximum_workflow_ms` are the worst-case sums reserved for the whole campaign
/// (probe, discovery, confirmation, and the Stage-C continuation) before probe
/// motion, so every phase plan must fit inside `maximum_workflow_ms`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BreakawayCampaignRequest {
    /// Frozen signed-target magnitude in mechanical RPM register units.
    pub(crate) target_velocity_rpm: TmcMechanicalRpm,
    /// Frozen encoder-domain velocity amplitude.
    pub(crate) planned_velocity_mrev_s: u32,
    /// Configured hard torque-target limit.
    pub(crate) hard_torque_limit: u16,
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    /// Host-declared per-primitive safety cap for each discovery rung.
    pub(crate) max_duration_ms: u16,
    /// Conservative fixed-schedule campaign duration.
    pub(crate) nominal_workflow_ms: u32,
    /// Reserved maximum campaign duration; every phase plan must fit inside it.
    pub(crate) maximum_workflow_ms: u32,
    /// Physical firmware channel bound into the Stage-C authority.
    pub(crate) channel: u8,
    /// Trace/run sequence carried into the Stage-C continuation.
    pub(crate) run_sequence: u32,
    /// PWM carrier frequency the campaign is planned under, in Hz. Bound into
    /// the probe plan's digest as part of its acquisition context.
    pub(crate) pwm_hz: u32,
    /// Encoder counts per mechanical revolution the campaign is planned under.
    /// Bound into the probe plan's digest as part of its acquisition context.
    pub(crate) encoder_counts_per_rev: u32,
}

/// The active campaign phase: exactly one committed driver's state plus its
/// session at a time, so the largest retained footprint is one phase's session,
/// never all three.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BreakawayCampaignPhase {
    /// The upward physical-excursion probe locating the campaign breakaway.
    Probe {
        state: BreakawayProbeState,
        session: BreakawayProbeSession,
    },
    /// The additive discovery ladder walking upward from the resolved breakaway.
    Discovery {
        state: AdditiveDiscoveryState,
        session: AdditiveDiscoverySession,
    },
    /// The held-out eight-stroke confirmation block at the nominated candidate.
    Confirmation {
        state: BandConfirmationState,
        session: BandConfirmationSession,
    },
}

/// Frozen campaign inputs plus the active phase.
///
/// `pending_event`/`pending_done` defer a not-yet-returned evidence record or
/// terminal by exactly one [`step_breakaway_campaign`] tick: a phase's
/// terminal transition (or the campaign's own accept) can need to report more
/// than one wire record before the next state is actually reachable (for
/// example a discovery terminal's rung-margin/rung-zero bundle, followed by a
/// zero-stroke confirmation terminal it routes straight into, followed by the
/// campaign's own closure record). Each field holds at most the ONE item not
/// yet returned; [`step_breakaway_campaign`] drains `pending_event` before
/// `pending_done`, and both before touching the active phase driver again.
#[derive(Debug, defmt::Format)]
pub(crate) struct BreakawayCampaign {
    inputs: BreakawayCampaignInputs,
    phase: BreakawayCampaignPhase,
    evidence_sequence: u16,
    pending_event: Option<crate::commissioning::types::BreakawayCampaignTerminal>,
    pending_done: Option<BreakawayCampaignOutcome>,
}

impl BreakawayCampaign {
    /// Plan the campaign from a command request and start at the probe's
    /// [`BreakawayProbeState::PlannedProbe`].
    ///
    /// This never touches a backend: planning is pure, and motion is only
    /// reachable through [`step_breakaway_campaign`], and only once the probe's
    /// state has left `PlannedProbe`. The command-level workflow envelope is
    /// reserved here, before the probe's first stroke, via
    /// [`CommissioningWorkflowPlan::new_breakaway`]; a malformed envelope
    /// refuses the whole campaign without motion.
    pub(crate) fn new(request: BreakawayCampaignRequest) -> Result<Self, CommissionError> {
        let (p_start, p_top, usable_torque_limit, ceiling_binding_source) =
            current_gain_span(request.target_velocity_rpm, request.hard_torque_limit)
                .map_err(|_| CommissionError::InvalidSchedule)?;
        // Reserve the command-level envelope before any probe motion; every
        // phase plan must later fit inside `maximum_workflow_ms`. Its digest is
        // bound into the probe plan below, so the whole downstream digest chain
        // is pinned to this command-level envelope.
        let workflow = CommissioningWorkflowPlan::new_breakaway(
            request.run_sequence,
            request.nominal_workflow_ms,
            request.maximum_workflow_ms,
        )
        .map_err(|_| CommissionError::InvalidSchedule)?;
        let probe_context = BreakawayProbeContext {
            workflow_digest: workflow.digest(),
            target_velocity_rpm: request.target_velocity_rpm.raw(),
            hard_torque_limit: request.hard_torque_limit,
            usable_torque_limit,
            pwm_hz: request.pwm_hz,
            encoder_counts_per_rev: request.encoder_counts_per_rev,
        };
        let probe_plan = plan_upward_breakaway_probe(p_start, p_top, probe_context)
            .map_err(|_| CommissionError::InvalidSchedule)?;
        let session = BreakawayProbeSession::new(
            probe_plan,
            request.target_velocity_rpm,
            request.planned_velocity_mrev_s,
        )
        .map_err(|_| CommissionError::InvalidSchedule)?;
        Ok(Self {
            inputs: BreakawayCampaignInputs {
                p_start,
                p_top,
                target_velocity_rpm: request.target_velocity_rpm,
                planned_velocity_mrev_s: request.planned_velocity_mrev_s,
                hard_torque_limit: request.hard_torque_limit,
                usable_torque_limit,
                limit_evidence_capability: request.limit_evidence_capability,
                observations_per_direction: CONSENSUS_OBSERVATIONS_PER_DIRECTION as u16,
                max_duration_ms: request.max_duration_ms,
                workflow_envelope_ms: request.maximum_workflow_ms,
                channel: request.channel,
                run_sequence: request.run_sequence,
                pwm_hz: request.pwm_hz,
                encoder_counts_per_rev: request.encoder_counts_per_rev,
                ceiling_binding_source,
            },
            phase: BreakawayCampaignPhase::Probe {
                state: BreakawayProbeState::PlannedProbe,
                session,
            },
            evidence_sequence: 0,
            pending_event: None,
            pending_done: None,
        })
    }

    /// Trace/run sequence the Stage-C continuation is emitted under.
    pub(crate) const fn run_sequence(&self) -> u32 {
        self.inputs.run_sequence
    }

    /// Number of recovery slots the campaign reserves before probe motion.
    ///
    /// The implemented state machines recover after every observation, not once
    /// per rung, so the reservation must cover every observation across probe,
    /// discovery, and confirmation: every possible upward-probe stroke, every
    /// discovery rung driven in both directions at
    /// [`OBSERVATIONS_PER_DIRECTION_PER_RUNG`], and the confirmation family. Each
    /// slot is reserved at the plan's per-slot maximum, bounding the recovery term
    /// of [`breakaway_workflow_envelope_ms`]. This evaluates to 392
    /// (`128 + 32 * 2 * 4 + 8`); a unit test asserts both the concrete value and
    /// that it tracks the derivation constants.
    pub(crate) const RECOVERY_SLOT_COUNT: u32 = PROBE_MAX_OBSERVATIONS as u32
        + MAX_SWEEP_RUNGS as u32 * 2 * OBSERVATIONS_PER_DIRECTION_PER_RUNG as u32
        + CONFIRMATION_FAMILY_SIZE as u32;

    /// Hard torque-target limit the campaign energizes the motor with.
    ///
    /// This is the operational torque limit that safe-state restoration must
    /// restore when a campaign phase is torn down while still armed. It is the
    /// same frozen limit the Stage-C hand-off carries into the confirmed plan
    /// (`hard_torque_limit` in [`BreakawayCampaignInputs`]).
    pub(crate) const fn hard_torque_limit(&self) -> u16 {
        self.inputs.hard_torque_limit
    }

    /// Encoder counts per mechanical revolution frozen before campaign motion.
    pub(crate) const fn encoder_counts_per_rev(&self) -> u32 {
        self.inputs.encoder_counts_per_rev
    }

    /// Planned encoder-domain velocity amplitude frozen before campaign motion.
    pub(crate) const fn planned_velocity_mrev_s(&self) -> u32 {
        self.inputs.planned_velocity_mrev_s
    }

    /// The active campaign phase, for engine-adapter routing and test scripting.
    pub(crate) const fn phase(&self) -> &BreakawayCampaignPhase {
        &self.phase
    }
}

/// Derive the command-level campaign plan and its worst-case workflow envelope
/// from the same host sweep request the combined workflow admits.
///
/// This is pure command admission: it commands no motion and touches no backend.
/// It reuses [`plan_combined_velocity_sweep`] to derive the shared target
/// velocity, planned amplitude, and per-slot recovery reservations, and
/// [`combined_workflow_geometry`] for the worst-case Stage-C continuation, then
/// sums the whole pre-motion envelope with [`breakaway_workflow_envelope_ms`]
/// (392 recovery slots per [`BreakawayCampaign::RECOVERY_SLOT_COUNT`]). The
/// returned [`CommissioningWorkflowPlan`] carries the exact envelope the
/// campaign reserves, so disclosing it before construction matches the envelope
/// [`BreakawayCampaign::new`] validates. Any planning failure is a zero-motion
/// refusal reported as [`CommissionError::InvalidSchedule`].
pub(crate) fn plan_breakaway_campaign_command(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    hard_torque_limit: u16,
    limit_evidence_capability: u32,
    pwm_hz: u32,
    channel: u8,
    run_sequence: u32,
) -> Result<(BreakawayCampaign, CommissioningWorkflowPlan), CommissionError> {
    let plan = plan_combined_velocity_sweep(
        request,
        encoder_counts_per_rev,
        hard_torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    let geometry =
        combined_workflow_geometry(plan, pwm_hz).map_err(|_| CommissionError::InvalidSchedule)?;
    let stage_c_nominal_us = u64::from(geometry.i_nominal_ms)
        .checked_mul(1_000)
        .ok_or(CommissionError::InvalidSchedule)?;
    let stage_c_maximum_us = u64::from(geometry.i_maximum_ms)
        .checked_mul(1_000)
        .ok_or(CommissionError::InvalidSchedule)?;
    let nominal_workflow_ms = breakaway_workflow_envelope_ms(
        BreakawayCampaign::RECOVERY_SLOT_COUNT,
        plan.recovery_nominal_slot_us,
        stage_c_nominal_us,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    let maximum_workflow_ms = breakaway_workflow_envelope_ms(
        BreakawayCampaign::RECOVERY_SLOT_COUNT,
        plan.recovery_maximum_slot_us,
        stage_c_maximum_us,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    let workflow = CommissioningWorkflowPlan::new_breakaway(
        run_sequence,
        nominal_workflow_ms,
        maximum_workflow_ms,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    let campaign = BreakawayCampaign::new(BreakawayCampaignRequest {
        target_velocity_rpm: plan.target_velocity_rpm,
        planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
        hard_torque_limit,
        limit_evidence_capability: plan.limit_evidence_capability,
        max_duration_ms: request.max_duration_ms,
        nominal_workflow_ms,
        maximum_workflow_ms,
        channel,
        run_sequence,
        pwm_hz,
        encoder_counts_per_rev,
    })?;
    Ok((campaign, workflow))
}

/// Terminal result of a complete breakaway campaign.
///
/// [`Self::StageCPlanReady`] is the ONLY variant that authorizes Stage C, and it
/// is reachable only from an accepted held-out confirmation whose confirmed P
/// also passed the 6a plan-time safety admission. Every other terminal --
/// including an accepted confirmation whose P the 6a constructor refuses WITHOUT
/// motion -- is a [`Self::Terminal`] that carries NO Stage-C plan, so no
/// failure or inconclusive path can begin Stage C.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BreakawayCampaignOutcome {
    /// The confirmed candidate P was accepted and admitted by 6a plan-time
    /// safety; the exact Stage-C plan built through the token route is ready to
    /// hand off to the `VelocityIntegral` phase.
    StageCPlanReady(VelocityIntegralPlan),
    /// A non-accept campaign terminal, or an accept whose confirmed P the 6a
    /// constructor refused WITHOUT motion. Never carries a Stage-C plan.
    Terminal {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

/// One cooperative result from [`step_breakaway_campaign`].
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BreakawayCampaignStep {
    /// Call again immediately; no external wait is needed.
    ReadyNow,
    /// Call again no earlier than this `elapsed_us` deadline.
    At(u64),
    /// One evidence record is ready for wire emission; call again immediately.
    Event(BreakawayCampaignEvent),
    /// The campaign concluded.
    Done(BreakawayCampaignOutcome),
}

/// Internal transition control: either a live per-tick step passed straight
/// through, or one evidence record to emit now plus what happens next.
///
/// Every phase transition (probe success, and every probe/discovery/
/// confirmation terminal) reports at least its own phase-owned record before
/// the campaign either enters the next phase or closes, so `Emit` -- not a
/// bare `EnterPhase` -- is the only transition variant; see
/// [`EmitContinuation`]. Both variants are large -- one carries a `Done`-sized
/// outcome, the other an event plus a phase or a second deferred event -- so
/// there is no small variant to shrink toward; boxing is unavailable in this
/// heapless crate, matching the sibling terminal enums here.
#[allow(clippy::large_enum_variant)]
enum CampaignAdvance {
    /// Pass a non-terminal, non-eventful driver result straight through.
    Step(BreakawayCampaignStep),
    /// Emit `now` immediately; `next` says what happens starting the
    /// following tick.
    Emit {
        now: BreakawayCampaignEvent,
        next: EmitContinuation,
    },
}

/// What [`step_breakaway_campaign`] does after an [`CampaignAdvance::Emit`]'s
/// `now` event has been returned.
///
/// `Terminal` carries the campaign closure record as the bare
/// [`crate::commissioning::types::BreakawayCampaignTerminal`] struct, not the
/// full [`BreakawayCampaignEvent`] enum: this value is staged in
/// [`BreakawayCampaign::pending_event`], which -- unlike `now`, always
/// returned immediately and never retained -- is embedded in [`OuterPhase`]
/// for a whole extra tick. Sizing that field to `BreakawayCampaignEvent`'s
/// largest variant (the batch discovery/confirmation terminals, which only
/// ever appear as an immediate `now`) would blow `OuterPhase`'s stack budget
/// for a record that is always the small campaign-terminal shape in practice.
#[allow(clippy::large_enum_variant)]
enum EmitContinuation {
    /// Enter this phase; its own `Planned*` interception reports the new
    /// phase's plan-freeze event(s) on the following tick, before its first
    /// stroke.
    EnterPhase(BreakawayCampaignPhase),
    /// Emit the campaign's own closure record on the following tick, then
    /// return `done` the tick after that.
    Terminal {
        campaign_terminal: crate::commissioning::types::BreakawayCampaignTerminal,
        done: BreakawayCampaignOutcome,
    },
}

/// Map a non-`BreakawayFound` probe outcome to its campaign terminal.
fn probe_terminal(outcome: BreakawayProbeOutcome) -> BreakawayCampaignOutcome {
    match outcome {
        BreakawayProbeOutcome::BreakawayFound(_) => {
            unreachable!("BreakawayFound is advanced to discovery by the orchestrator")
        }
        BreakawayProbeOutcome::SafetyFault { error, safety } => {
            BreakawayCampaignOutcome::Terminal { error, safety }
        }
        BreakawayProbeOutcome::NoRepeatableMotionAtAuthority
        | BreakawayProbeOutcome::ExcursionEvidenceInvalid
        | BreakawayProbeOutcome::AuthorityFailure => BreakawayCampaignOutcome::Terminal {
            error: CommissionError::VelocityValidationFailed,
            safety: None,
        },
    }
}

/// Map a non-`Accepted` confirmation outcome to its campaign terminal.
fn confirmation_terminal(outcome: BandConfirmationOutcome) -> BreakawayCampaignOutcome {
    match outcome {
        BandConfirmationOutcome::Accepted { .. } => {
            unreachable!("Accepted is routed to Stage C by the orchestrator")
        }
        BandConfirmationOutcome::SafetyFault { error, safety } => {
            BreakawayCampaignOutcome::Terminal { error, safety }
        }
        BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery
        | BandConfirmationOutcome::TargetBandNotReachedAtAuthority
        | BandConfirmationOutcome::TargetBandConfirmationInconclusive(_)
        | BandConfirmationOutcome::ConfirmationEvidenceExcluded => {
            BreakawayCampaignOutcome::Terminal {
                error: CommissionError::VelocityValidationFailed,
                safety: None,
            }
        }
    }
}

/// Hand out the next monotonic evidence sequence number and advance the
/// counter, mirroring `VelocityTraceIntegrity::next_evidence_sequence` for
/// the breakaway campaign, which has no integrity tracker of its own.
fn next_sequence(evidence_sequence: &mut u16) -> u16 {
    let sequence = *evidence_sequence;
    *evidence_sequence = evidence_sequence.wrapping_add(1);
    sequence
}

/// Build the streamed raw-observation event for a just-collected stroke.
///
/// Attaches the owning plan digest, run sequence, and a fresh evidence sequence
/// to the driver-local [`CollectedObservation`], exactly as the `*_advance`
/// helpers attach them to terminal records. The event is returned immediately
/// as a bare [`BreakawayCampaignStep::Event`]; it is never staged in
/// `pending_event`, so streaming it retains no state.
fn raw_observation_step(
    plan_digest: u64,
    phase: super::breakaway_reporting::BreakawayPhase,
    collected: CollectedObservation,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let sequence = next_sequence(evidence_sequence);
    let report = super::breakaway_reporting::raw_observation_report(
        plan_digest,
        phase,
        &collected,
        run_sequence,
        sequence,
    );
    BreakawayCampaignStep::Event(BreakawayCampaignEvent::RawObservation(report))
}

fn probe_decision_step(
    plan_digest: u64,
    input: ProbeDecisionReportInput,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let sequence = next_sequence(evidence_sequence);
    let report = super::breakaway_reporting::probe_decision_report(
        plan_digest,
        input,
        run_sequence,
        sequence,
    );
    BreakawayCampaignStep::Event(BreakawayCampaignEvent::ProbeDecision(report))
}

/// Build the streamed margin event for one collected in-band rung, mirroring
/// [`raw_observation_step`]: the discovery driver names the collected index, and
/// this attaches the wire identity (plan digest, run sequence, a fresh evidence
/// sequence) and builds the record.
///
/// A non-nominatable in-band rung legitimately produces no record; that index
/// consumes no evidence sequence and simply advances the stream on the next
/// tick.
fn margin_step(
    plan_digest: u64,
    in_band: &[DiscoveryRungNominationBounds],
    collected_index: u8,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let nominatable = in_band
        .get(usize::from(collected_index))
        .is_some_and(|bounds| bounds.nominatable);
    if !nominatable {
        return BreakawayCampaignStep::ReadyNow;
    }
    let sequence = next_sequence(evidence_sequence);
    match super::breakaway_reporting::discovery_rung_margin_report(
        in_band,
        collected_index,
        plan_digest,
        run_sequence,
        sequence,
    ) {
        Some(report) => {
            BreakawayCampaignStep::Event(BreakawayCampaignEvent::DiscoveryRungMargin(report))
        }
        None => BreakawayCampaignStep::ReadyNow,
    }
}

/// Build the campaign's own closure record.
fn campaign_terminal_event(
    phase: BreakawayPhase,
    terminal_cause: super::breakaway_reporting::BreakawayTerminalCause,
    accepted: bool,
    stage_c_plan_digest: u64,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> crate::commissioning::types::BreakawayCampaignTerminal {
    let sequence = next_sequence(evidence_sequence);
    campaign_terminal_report(
        phase,
        terminal_cause,
        accepted,
        stage_c_plan_digest,
        inputs.run_sequence,
        sequence,
    )
}

/// Report a breakaway-campaign closure record for a fault raised by the
/// caller's own global outer-safety check -- an invalid motion budget, a
/// tripped safety envelope, or a safety-check I/O error -- rather than by any
/// of the three drivers' own internal state machines, while a breakaway
/// campaign phase is active.
///
/// Mirrors [`super::sweep::sweep_fault_terminal`] and
/// [`super::integral_sweep::integral_fault_terminal`]: a mid-campaign safety
/// fault must still produce a campaign-closure record before the phase fails,
/// because offline replay (foci-trace `velocity_measurement.py`) requires
/// exactly one closure record per campaign. Unlike those two report-only
/// phases, the campaign streams multiple wire records per run and cannot
/// return its closure record directly from here without dropping whatever
/// record the caller's next tick would otherwise have produced, so this
/// reuses the campaign's own `pending_event`/`pending_done` deferral -- the
/// same mechanism [`discovery_construction_fault`] and the other internal
/// terminal builders use -- rather than a new bespoke `StepResult` path: the
/// closure event drains on the next tick, and the `OuterFailed`/
/// `FailedWithSafety` transition follows on the tick after that.
///
/// The reported `terminal_cause` reuses the owning phase's own SafetyFault
/// cause code (`ProbeSafetyFault`/`DiscoverySafetyFault`/
/// `ConfirmationSafetyFault`), the same code that phase's own internal safety
/// short-circuit would have reported, so a host decoder sees one safety-fault
/// taxonomy regardless of whether the fault originated inside the driver or
/// at the surrounding dispatch boundary.
///
/// Falls back to a bare `OuterFailed` if `phase` is not a breakaway campaign,
/// matching every other fault-terminal reporter's defensive fallback.
pub(crate) fn breakaway_campaign_fault_terminal(
    phase: &mut OuterPhase,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    let OuterPhase::BreakawayCampaign { campaign, .. } = phase else {
        return StepResult::OuterFailed(error);
    };
    let (owning_phase, terminal_cause) = match campaign.phase() {
        BreakawayCampaignPhase::Probe { .. } => (
            BreakawayPhase::Probe,
            super::breakaway_reporting::BreakawayTerminalCause::ProbeSafetyFault,
        ),
        BreakawayCampaignPhase::Discovery { .. } => (
            BreakawayPhase::Discovery,
            super::breakaway_reporting::BreakawayTerminalCause::DiscoverySafetyFault,
        ),
        BreakawayCampaignPhase::Confirmation { .. } => (
            BreakawayPhase::Confirmation,
            super::breakaway_reporting::BreakawayTerminalCause::ConfirmationSafetyFault,
        ),
    };
    let campaign_terminal = campaign_terminal_event(
        owning_phase,
        terminal_cause,
        false,
        0,
        &campaign.inputs,
        &mut campaign.evidence_sequence,
    );
    campaign.pending_event = Some(campaign_terminal);
    campaign.pending_done = Some(BreakawayCampaignOutcome::Terminal {
        error,
        safety: safety_fault,
    });
    StepResult::ReadyNow
}

/// True when a breakaway campaign is holding a deferred terminal record or
/// outcome that still needs to drain through [`step_breakaway_campaign`].
///
/// The outer dispatcher consults this before re-running its own outer-safety
/// check. A *persistent* fault (an invalid motion budget, an elapsed-duration
/// violation) re-fires that check every tick; without this guard the dispatcher
/// would call [`breakaway_campaign_fault_terminal`] again on each of those
/// ticks, overwriting `pending_event`/`pending_done` before the campaign driver
/// ever runs to drain them -- so the closure record would never reach the wire
/// and the phase would never fail out. Draining first lets the deferred closure
/// emit and the `OuterFailed`/`FailedWithSafety` transition follow.
pub(crate) fn breakaway_campaign_pending_terminal(phase: &OuterPhase) -> bool {
    matches!(
        phase,
        OuterPhase::BreakawayCampaign { campaign, .. }
            if campaign.pending_event.is_some() || campaign.pending_done.is_some()
    )
}

/// A discovery plan-time construction failure never freezes a discovery plan
/// (`plan_additive_discovery_ladder` refused before `AdditiveDiscoveryPlan`
/// existed), so there is no discovery-owned terminal record to report -- only
/// the campaign's own closure record, with `BreakawayPhase::Discovery` and
/// [`super::breakaway_reporting::BreakawayTerminalCause::DiscoverySafetyFault`],
/// matching [`discovery_terminal_cause`]'s own mapping for
/// [`AdditiveDiscoveryOutcome::SafetyFault`].
fn discovery_construction_fault(
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> EmitContinuation {
    let campaign_terminal = campaign_terminal_event(
        BreakawayPhase::Discovery,
        super::breakaway_reporting::BreakawayTerminalCause::DiscoverySafetyFault,
        false,
        0,
        inputs,
        evidence_sequence,
    );
    EmitContinuation::Terminal {
        campaign_terminal,
        done: BreakawayCampaignOutcome::Terminal {
            error: CommissionError::InvalidSchedule,
            safety: None,
        },
    }
}

/// Route a finished discovery outcome into confirmation, or into its terminal.
///
/// Every discovery outcome -- whether a discovery-run `Done` or a discovery
/// plan-time construction error -- is routed through the single
/// [`begin_band_confirmation`] gate, so the discovery-to-confirmation mapping
/// (including the impossible-outcome-to-`InvalidSchedule` safety fault) has
/// exactly one implementation. `discovery_plan_digest` is only consulted on the
/// `Ready` path; terminal outcomes ignore it.
///
/// The zero-stroke confirmation outcomes reachable here
/// ([`BandConfirmationOutcome::TargetBandNotReachedAtAuthority`],
/// [`BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery`], and the
/// impossible-outcome safety fault) never freeze a confirmation plan, so --
/// mirroring [`discovery_construction_fault`] -- only the campaign's own
/// closure record is reported, not a zero-stroke confirmation terminal.
fn route_discovery_outcome(
    outcome: AdditiveDiscoveryOutcome,
    discovery_plan_digest: u64,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> EmitContinuation {
    match begin_band_confirmation(
        outcome,
        discovery_plan_digest,
        inputs.target_velocity_rpm,
        inputs.planned_velocity_mrev_s,
    ) {
        BandConfirmationEntry::Ready(session) => {
            EmitContinuation::EnterPhase(BreakawayCampaignPhase::Confirmation {
                state: BandConfirmationState::PlannedConfirmation,
                session,
            })
        }
        BandConfirmationEntry::Done(outcome) => {
            let cause = confirmation_zero_stroke_cause(&outcome).0;
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::Confirmation,
                cause,
                false,
                0,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: confirmation_terminal(outcome),
            }
        }
    }
}

/// Route an accepted held-out confirmation through the ONE authorized Stage-C
/// chain: [`bind_breakaway_stage_c_authority`] -> `ConfirmedAcquisitionDigest`
/// token -> [`plan_breakaway_velocity_integral_response`], which builds the
/// `StageCAuthority` internally from the token.
///
/// This is the sole breakaway path from a confirmed acceptance to a Stage-C
/// plan. It is reachable only from [`BandConfirmationOutcome::Accepted`]; it
/// takes the acceptance and its frozen plan by value, so no failure or
/// inconclusive confirmation terminal can even be expressed as an argument, and
/// no other campaign terminal mints a confirmation token. The 6a
/// constructor still applies its own plan-time safety admission and MAY refuse
/// Stage C WITHOUT motion; that refusal is returned here as a
/// [`BreakawayCampaignOutcome::Terminal`], never bypassed. This function touches
/// no backend excitation and commands no motion; it only reads the frozen PWM
/// timebase and encoder resolution.
fn route_confirmed_accept<B: CommissioningBackend>(
    acceptance: BandConfirmationAcceptance,
    plan: BandConfirmationPlan,
    inputs: &BreakawayCampaignInputs,
    safety: &OuterSafetyLimits,
    backend: &B,
) -> BreakawayCampaignOutcome {
    let token = match bind_breakaway_stage_c_authority(plan.plan_digest, plan, Ok(acceptance)) {
        Ok(token) => token,
        Err(_) => {
            return BreakawayCampaignOutcome::Terminal {
                error: CommissionError::InvalidSchedule,
                safety: None,
            };
        }
    };
    // The gate builds the `StageCAuthority` itself from the token, so no
    // pre-built authority is handed across. `channel` and the plan-time
    // PWM/encoder freeze travel in the acquisition context; the gate rejects
    // if the plan-time physical parameters diverge from the live ones passed
    // below.
    let context = AcquisitionContext {
        planned_velocity_mrev_s: inputs.planned_velocity_mrev_s,
        target_velocity_rpm: inputs.target_velocity_rpm.raw(),
        hard_torque_limit: inputs.hard_torque_limit,
        usable_torque_limit: inputs.usable_torque_limit,
        limit_evidence_capability: inputs.limit_evidence_capability,
        channel: inputs.channel,
        plan_time_pwm_hz: inputs.pwm_hz,
        plan_time_encoder_counts_per_rev: inputs.encoder_counts_per_rev,
    };
    match plan_breakaway_velocity_integral_response(
        token,
        plan,
        context,
        safety.budget,
        backend.tmc_config().pwm_freq_hz(),
        safety.counts_per_rev(),
    ) {
        Ok(stage_c_plan) => BreakawayCampaignOutcome::StageCPlanReady(stage_c_plan),
        Err(_) => BreakawayCampaignOutcome::Terminal {
            error: CommissionError::VelocityValidationFailed,
            safety: None,
        },
    }
}

/// Build the additive discovery session from a resolved probe breakaway.
///
/// `authority_ceiling = p_top` (the retained plan-time ceiling), per the spec's
/// plan-time authority rule: the Stage-C transition-authority guard is applied
/// only at Stage C, by [`plan_breakaway_velocity_integral_response`], never as a
/// pre-sweep discovery ceiling. Dedicated plan-time discovery refusals retain
/// their discovery-owned terminal causes; structural failures retain the generic
/// discovery safety fault.
fn enter_discovery(
    resolved: &ResolvedUpwardBreakaway,
    probe_plan: &UpwardBreakawayProbePlan,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> EmitContinuation {
    let (floor, floor_origin) = match resolve_grid_floor(
        inputs.p_start,
        inputs.p_top,
        probe_plan.search_count,
        resolved.breakaway_index,
    ) {
        Ok(resolved_floor) => resolved_floor,
        Err(_) => return discovery_construction_fault(inputs, evidence_sequence),
    };
    let Some(breakaway) = probe_plan
        .search_gains
        .get(usize::from(resolved.breakaway_index))
        .copied()
    else {
        return discovery_construction_fault(inputs, evidence_sequence);
    };
    let request = AdditiveDiscoveryRequest {
        floor,
        breakaway,
        authority_ceiling: inputs.p_top,
        observations_per_direction: inputs.observations_per_direction,
        max_duration_ms: inputs.max_duration_ms,
        workflow_envelope_ms: inputs.workflow_envelope_ms,
        probe_plan_digest: probe_plan.plan_digest,
    };
    let floor_origin_clamped = matches!(floor_origin, GridFloorOrigin::ClampedAtBreakaway);
    match AdditiveDiscoverySession::new(
        request,
        floor_origin_clamped,
        inputs.target_velocity_rpm,
        inputs.planned_velocity_mrev_s,
    ) {
        Ok(session) => EmitContinuation::EnterPhase(BreakawayCampaignPhase::Discovery {
            state: AdditiveDiscoveryState::PlannedDiscovery,
            session,
        }),
        Err(outcome) => {
            let (cause, error, safety) = match outcome {
                AdditiveDiscoveryOutcome::InsufficientAdditiveSpan => (
                    BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
                    CommissionError::InvalidSchedule,
                    None,
                ),
                AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => (
                    BreakawayTerminalCause::DiscoveryAdditiveLadderResolutionBudgetExceeded,
                    CommissionError::InvalidSchedule,
                    None,
                ),
                AdditiveDiscoveryOutcome::SafetyFault { error, safety } => {
                    (BreakawayTerminalCause::DiscoverySafetyFault, error, safety)
                }
                AdditiveDiscoveryOutcome::TargetBandSelected(_)
                | AdditiveDiscoveryOutcome::TargetBandSkipped
                | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
                | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { .. } => (
                    BreakawayTerminalCause::DiscoverySafetyFault,
                    CommissionError::InvalidSchedule,
                    None,
                ),
            };
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::Discovery,
                cause,
                false,
                0,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal { error, safety },
            }
        }
    }
}

/// Build the campaign's probe-success advance: report the resolved rung now,
/// then enter discovery (or report its
/// construction failure) starting the following tick.
fn probe_success_advance(
    resolved: ResolvedUpwardBreakaway,
    probe_plan: &UpwardBreakawayProbePlan,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let report = probe_resolved_report(
        probe_plan,
        resolved.breakaway_index,
        resolved.observations,
        inputs.run_sequence,
        sequence,
    );
    let now = BreakawayCampaignEvent::ProbeResolved(report);
    let next = enter_discovery(&resolved, probe_plan, inputs, evidence_sequence);
    CampaignAdvance::Emit { now, next }
}

/// Build the campaign's probe-failure advance: report the probe's own
/// terminal now, then the campaign's closure record starting the following
/// tick.
fn probe_failure_advance(
    outcome: BreakawayProbeOutcome,
    probe_plan: &UpwardBreakawayProbePlan,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let terminal = probe_terminal_report(probe_plan, &outcome, inputs.run_sequence, sequence);
    let cause = probe_terminal_cause(&outcome).0;
    let now = BreakawayCampaignEvent::ProbeTerminal(terminal);
    let campaign_terminal = campaign_terminal_event(
        BreakawayPhase::Probe,
        cause,
        false,
        0,
        inputs,
        evidence_sequence,
    );
    CampaignAdvance::Emit {
        now,
        next: EmitContinuation::Terminal {
            campaign_terminal,
            done: probe_terminal(outcome),
        },
    }
}

/// Build the campaign's discovery-terminal advance: report every collected
/// in-band rung's margin, the rung-zero diagnostic (when available), and the
/// discovery terminal itself now, then route into confirmation -- or, for a
/// zero-stroke confirmation outcome, the campaign's closure record -- starting
/// the following tick.
fn discovery_done_advance(
    outcome: AdditiveDiscoveryOutcome,
    session: &AdditiveDiscoverySession,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let terminal =
        discovery_terminal_report(&session.plan, &outcome, inputs.run_sequence, sequence);
    let rung_zero = session.rung_zero_classification.map(|(forward, reverse)| {
        rung_zero_diagnostic_report(
            &session.plan,
            forward,
            reverse,
            inputs.run_sequence,
            sequence,
        )
    });
    // Per-rung margins already streamed one per tick from `StreamMargins` before
    // this terminal, so the terminal event carries only the rung-zero diagnostic
    // and the terminal record (two frames, no burst).
    let now = BreakawayCampaignEvent::DiscoveryTerminal {
        terminal,
        rung_zero,
    };
    let plan_digest = session.plan.plan_digest;
    // Every discovery outcome other than a nominating success or the
    // spec-section-9 current-headroom reclassification is a discovery-OWNED
    // terminal: keep its own specific discovery cause on the campaign's
    // closure record too, rather than the generic `SafetyFault`
    // `begin_band_confirmation`'s impossible-outcome gate collapses every one
    // of them to.
    let next = match outcome {
        AdditiveDiscoveryOutcome::TargetBandSelected(_)
        | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand => {
            route_discovery_outcome(outcome, plan_digest, inputs, evidence_sequence)
        }
        AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => {
            let cause = discovery_terminal_cause(&outcome).0;
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::Discovery,
                cause,
                false,
                0,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal {
                    error: CommissionError::InvalidSchedule,
                    safety: None,
                },
            }
        }
        AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { error } => {
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::Discovery,
                super::breakaway_reporting::BreakawayTerminalCause::DiscoveryEvidenceExcluded,
                false,
                0,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal {
                    error,
                    safety: None,
                },
            }
        }
        AdditiveDiscoveryOutcome::SafetyFault { error, safety } => {
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::Discovery,
                super::breakaway_reporting::BreakawayTerminalCause::DiscoverySafetyFault,
                false,
                0,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal { error, safety },
            }
        }
    };
    CampaignAdvance::Emit { now, next }
}

/// One direction's fresh confirmation evidence, assembled from a live
/// session's accumulator -- the same construction
/// [`evaluate_confirmation_session`] uses to judge acceptance.
fn confirmation_directional_input(
    session: &BandConfirmationSession,
    direction: VelocityDirection,
) -> ConfirmationDirectionalInput {
    let accumulator = match direction {
        VelocityDirection::Forward => &session.forward,
        VelocityDirection::Reverse => &session.reverse,
    };
    ConfirmationDirectionalInput {
        consensus: accumulator.consensus(),
        classes: accumulator.classes,
        observations: accumulator.atoms,
    }
}

/// No observations collected: reported for a confirmation terminal whose
/// masks are all zero (see [`confirmation_zero_stroke_terminal_report`]), so
/// the event's own fan-out reports zero strokes too, never a partial session.
const EMPTY_CONFIRMATION_DIRECTIONAL_INPUT: ConfirmationDirectionalInput =
    ConfirmationDirectionalInput {
        consensus: DirectionalConsensus {
            classification: super::sweep_consensus::DirectionalConsensusClass::IncompleteRung,
            components: [[0; 2]; 2],
            component_count: 0,
            collected_mask: 0,
            eligible_mask: 0,
            included_mask: 0,
        },
        classes: [VelocityObservationClass::InvalidEvidenceFault;
            CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };

/// Build the campaign's confirmation-terminal advance: report every fresh
/// held-out stroke's response interval and the confirmation terminal itself
/// now, then the campaign's closure record starting the following tick,
/// routing an acceptance through the Stage-C token chain.
fn confirmation_done_advance<B: CommissioningBackend>(
    outcome: BandConfirmationOutcome,
    session: &BandConfirmationSession,
    inputs: &BreakawayCampaignInputs,
    safety: &OuterSafetyLimits,
    backend: &B,
    evidence_sequence: &mut u16,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let (cause, has_safety_fault) = confirmation_zero_stroke_cause(&outcome);
    let (terminal, forward, reverse) = match &outcome {
        BandConfirmationOutcome::Accepted { acceptance, .. } => {
            let forward = confirmation_directional_input(session, VelocityDirection::Forward);
            let reverse = confirmation_directional_input(session, VelocityDirection::Reverse);
            let terminal = confirmation_evaluated_terminal_report(
                &session.plan,
                &forward,
                &reverse,
                Some(acceptance.candidate_p().raw()),
                None,
                inputs.run_sequence,
                sequence,
            );
            (terminal, forward, reverse)
        }
        BandConfirmationOutcome::TargetBandConfirmationInconclusive(failure_cause) => {
            let forward = confirmation_directional_input(session, VelocityDirection::Forward);
            let reverse = confirmation_directional_input(session, VelocityDirection::Reverse);
            let terminal = confirmation_evaluated_terminal_report(
                &session.plan,
                &forward,
                &reverse,
                None,
                Some(*failure_cause),
                inputs.run_sequence,
                sequence,
            );
            (terminal, forward, reverse)
        }
        BandConfirmationOutcome::ConfirmationEvidenceExcluded
        | BandConfirmationOutcome::SafetyFault { .. } => {
            // Unlike the genuinely zero-stroke outcomes below, both exclusion
            // and a safety fault can strike after earlier strokes already
            // folded evidence into the session's accumulators --
            // `confirmation_fault`/`confirmation_safety_fault` are reachable
            // from every stroke state, including `Advance`'s successors --
            // so report that partial evidence rather than the empty fixture.
            // Without this, a mid-block safety fault would falsely claim no
            // confirmation work happened, exactly the evidence loss fixed for
            // `ConfirmationEvidenceExcluded`. `has_safety_fault` is threaded
            // through so a `SafetyFault`'s retained-evidence flag survives
            // here the same way it does on the zero-stroke path.
            let forward = confirmation_directional_input(session, VelocityDirection::Forward);
            let reverse = confirmation_directional_input(session, VelocityDirection::Reverse);
            let terminal = confirmation_partial_evidence_terminal_report(
                &session.plan,
                &forward,
                &reverse,
                cause,
                has_safety_fault,
                inputs.run_sequence,
                sequence,
            );
            (terminal, forward, reverse)
        }
        BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery
        | BandConfirmationOutcome::TargetBandNotReachedAtAuthority => {
            let terminal = confirmation_zero_stroke_terminal_report(
                session.plan.plan_digest,
                session.plan.discovery_plan_digest,
                session.plan.family_size,
                &outcome,
                inputs.run_sequence,
                sequence,
            );
            (
                terminal,
                EMPTY_CONFIRMATION_DIRECTIONAL_INPUT,
                EMPTY_CONFIRMATION_DIRECTIONAL_INPUT,
            )
        }
    };
    let now = BreakawayCampaignEvent::ConfirmationTerminal {
        terminal,
        forward,
        reverse,
    };
    let done = match outcome {
        BandConfirmationOutcome::Accepted { acceptance, plan } => {
            route_confirmed_accept(acceptance, plan, inputs, safety, backend)
        }
        other => confirmation_terminal(other),
    };
    let (accepted, stage_c_plan_digest) = match &done {
        BreakawayCampaignOutcome::StageCPlanReady(plan) => (true, plan.digest()),
        BreakawayCampaignOutcome::Terminal { .. } => (false, 0),
    };
    let campaign_terminal = campaign_terminal_event(
        BreakawayPhase::Confirmation,
        cause,
        accepted,
        stage_c_plan_digest,
        inputs,
        evidence_sequence,
    );
    CampaignAdvance::Emit {
        now,
        next: EmitContinuation::Terminal {
            campaign_terminal,
            done,
        },
    }
}

/// If the active phase is at its `Planned*` state (frozen but no stroke
/// issued yet), report that phase's plan-freeze event(s) and advance past the
/// interception point, mirroring
/// [`super::sweep::step_velocity_sweep_with_trace`]'s `Setup` arm: mutate the
/// state and return the event in the same tick. The probe's next tick performs
/// its motion-free safety setup; discovery and confirmation continue into their
/// first stroke. Returns `None` once the active phase has left its `Planned*`
/// state, so the caller falls through to the normal per-tick driver dispatch.
fn emit_phase_planned_event(campaign: &mut BreakawayCampaign) -> Option<BreakawayCampaignStep> {
    let BreakawayCampaign {
        inputs,
        phase,
        evidence_sequence,
        ..
    } = campaign;
    match phase {
        BreakawayCampaignPhase::Probe { state, session } => {
            if !matches!(*state, BreakawayProbeState::PlannedProbe) {
                return None;
            }
            let sequence = next_sequence(evidence_sequence);
            let report = probe_plan_report(
                &session.plan,
                inputs.p_start,
                inputs.p_top,
                inputs.run_sequence,
                sequence,
            );
            *state = BreakawayProbeState::SetupSafety;
            Some(BreakawayCampaignStep::Event(
                BreakawayCampaignEvent::ProbePlanned(report),
            ))
        }
        BreakawayCampaignPhase::Discovery { state, session } => {
            if !matches!(*state, AdditiveDiscoveryState::PlannedDiscovery) {
                return None;
            }
            let sequence = next_sequence(evidence_sequence);
            let planned = discovery_plan_report(
                &session.request,
                session.floor_origin_clamped,
                &session.plan,
                inputs.run_sequence,
                sequence,
            );
            let ceiling_source = BreakawayCeilingSource {
                run_sequence: inputs.run_sequence,
                evidence_sequence: sequence,
                plan_digest: session.plan.plan_digest,
                binding_source: inputs.ceiling_binding_source,
            };
            *state = AdditiveDiscoveryState::WriteGain;
            Some(BreakawayCampaignStep::Event(
                BreakawayCampaignEvent::DiscoveryPlanned {
                    planned,
                    ceiling_source,
                },
            ))
        }
        BreakawayCampaignPhase::Confirmation { state, session } => {
            if !matches!(*state, BandConfirmationState::PlannedConfirmation) {
                return None;
            }
            let sequence = next_sequence(evidence_sequence);
            let nomination = NominatedConfirmationCandidate {
                p: session.plan.candidate_p,
                margin_percent: session.nominated_margin_percent,
            };
            let planned =
                confirmation_plan_report(&session.plan, &nomination, inputs.run_sequence, sequence);
            *state = BandConfirmationState::WriteGain;
            Some(BreakawayCampaignStep::Event(
                BreakawayCampaignEvent::ConfirmationPlanned(planned),
            ))
        }
    }
}

fn checkpoint_event_routing(step: BreakawayCampaignStep) -> BreakawayCampaignStep {
    if matches!(&step, BreakawayCampaignStep::Event(_)) {
        crate::diagnostic::checkpoint_breakaway_campaign(
            crate::diagnostic::BreakawayCheckpoint::EventRouting,
        );
    }
    step
}

/// Advance the breakaway campaign by one cooperative step.
///
/// Sequences the three committed drivers -- probe, additive discovery, held-out
/// confirmation -- through the ONE borrowed `workspace`, then routes an accepted
/// confirmation through the Stage-C token chain. `safety` and `workspace` are
/// caller-owned; neither is allocated here. The caller runs
/// [`crate::commissioning::outer::check_outer_safety`] against `safety` between
/// ticks, exactly as it does for every other outer phase; this function never
/// performs that check itself, matching the three underlying drivers.
///
/// Each MOVING state is reachable only after its owning `Planned` event: a new
/// phase is always entered at its `PlannedProbe`/`PlannedDiscovery`/
/// `PlannedConfirmation` state, and entering a phase returns `ReadyNow` without
/// issuing that phase's first stroke, so the phase plan is always frozen before
/// its first stroke.
///
/// Most evidence emission is layered on top of this same sequencing rather than
/// woven into the three drivers' own per-tick state machines: the frozen plan
/// is available in full at phase entry (just before the first stroke) and the
/// terminal evidence just after the driver's last stroke, so
/// [`emit_phase_planned_event`] intercepts the former and the `*_advance`
/// helpers (called from the `Done` arms below) build the latter, deferring
/// anything beyond one record per tick through
/// `campaign.pending_event`/`campaign.pending_done` (drained at the top of
/// this function, before the active phase driver runs again).
///
/// The one exception is the raw per-observation evidence: the discovery and
/// confirmation drivers surface an [`AdditiveDiscoveryStep::Observation`]/
/// [`BandConfirmationStep::Observation`] on the tick each stroke completes, and
/// [`raw_observation_step`] turns it into a streamed
/// [`BreakawayCampaignEvent::RawObservation`] right then -- before the atom is
/// folded and reduced to bounds. This is deliberately at collection time, one
/// observation per tick, never bundled into a terminal-time burst, so an
/// offline auditor sees the raw inputs even though the firmware retains only
/// the reduced bounds.
pub(crate) async fn step_breakaway_campaign<B: CommissioningBackend>(
    campaign: &mut BreakawayCampaign,
    workspace: &mut BreakawayProbeWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> BreakawayCampaignStep {
    if let Some(campaign_terminal) = campaign.pending_event.take() {
        return checkpoint_event_routing(BreakawayCampaignStep::Event(
            BreakawayCampaignEvent::CampaignTerminal(campaign_terminal),
        ));
    }
    if let Some(outcome) = campaign.pending_done.take() {
        return BreakawayCampaignStep::Done(outcome);
    }
    if let Some(step) = emit_phase_planned_event(campaign) {
        return checkpoint_event_routing(step);
    }

    let advance = {
        let BreakawayCampaign {
            inputs,
            phase,
            evidence_sequence,
            ..
        } = &mut *campaign;
        match phase {
            BreakawayCampaignPhase::Probe { state, session } => {
                match step_breakaway_probe(state, workspace, session, safety, backend).await {
                    BreakawayProbeStep::ReadyNow => {
                        CampaignAdvance::Step(BreakawayCampaignStep::ReadyNow)
                    }
                    BreakawayProbeStep::At(deadline_us) => {
                        CampaignAdvance::Step(BreakawayCampaignStep::At(deadline_us))
                    }
                    BreakawayProbeStep::Observation(collected) => {
                        CampaignAdvance::Step(raw_observation_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::Probe,
                            collected,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    BreakawayProbeStep::Decision {
                        direction,
                        rung_index,
                        disposition,
                    } => CampaignAdvance::Step(probe_decision_step(
                        session.plan.plan_digest,
                        ProbeDecisionReportInput {
                            direction,
                            rung_index,
                            disposition,
                        },
                        inputs.run_sequence,
                        evidence_sequence,
                    )),
                    BreakawayProbeStep::Done(BreakawayProbeOutcome::BreakawayFound(resolved)) => {
                        probe_success_advance(resolved, &session.plan, inputs, evidence_sequence)
                    }
                    BreakawayProbeStep::Done(outcome) => {
                        probe_failure_advance(outcome, &session.plan, inputs, evidence_sequence)
                    }
                }
            }
            BreakawayCampaignPhase::Discovery { state, session } => {
                match step_additive_discovery(state, workspace, session, safety, backend).await {
                    AdditiveDiscoveryStep::ReadyNow => {
                        CampaignAdvance::Step(BreakawayCampaignStep::ReadyNow)
                    }
                    AdditiveDiscoveryStep::At(deadline_us) => {
                        CampaignAdvance::Step(BreakawayCampaignStep::At(deadline_us))
                    }
                    AdditiveDiscoveryStep::Observation(collected) => {
                        CampaignAdvance::Step(raw_observation_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::Discovery,
                            collected,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    AdditiveDiscoveryStep::Margin { collected_index } => {
                        CampaignAdvance::Step(margin_step(
                            session.plan.plan_digest,
                            session.in_band.as_slice(),
                            collected_index,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    AdditiveDiscoveryStep::Done(outcome) => {
                        discovery_done_advance(outcome, session, inputs, evidence_sequence)
                    }
                }
            }
            BreakawayCampaignPhase::Confirmation { state, session } => {
                match step_band_confirmation(state, workspace, session, safety, backend).await {
                    BandConfirmationStep::ReadyNow => {
                        CampaignAdvance::Step(BreakawayCampaignStep::ReadyNow)
                    }
                    BandConfirmationStep::At(deadline_us) => {
                        CampaignAdvance::Step(BreakawayCampaignStep::At(deadline_us))
                    }
                    BandConfirmationStep::Observation(collected) => {
                        CampaignAdvance::Step(raw_observation_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::Confirmation,
                            collected,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    BandConfirmationStep::Done(outcome) => confirmation_done_advance(
                        outcome,
                        session,
                        inputs,
                        safety,
                        backend,
                        evidence_sequence,
                    ),
                }
            }
        }
    };
    let step = match advance {
        CampaignAdvance::Step(step) => step,
        CampaignAdvance::Emit { now, next } => {
            match next {
                EmitContinuation::EnterPhase(next_phase) => campaign.phase = next_phase,
                EmitContinuation::Terminal {
                    campaign_terminal,
                    done,
                } => {
                    campaign.pending_event = Some(campaign_terminal);
                    campaign.pending_done = Some(done);
                }
            }
            BreakawayCampaignStep::Event(now)
        }
    };
    checkpoint_event_routing(step)
}

/// Engine adapter: step the embedded breakaway campaign by one cooperative tick,
/// then translate its terminal into the engine's [`StepResult`].
///
/// On an accepted, admitted confirmation this performs the Confirmed -> Stage C
/// hand-off in place, mirroring
/// [`crate::commissioning::CommissioningEngine::begin_stage_c_continuation_with_plan`]:
/// it replaces the [`OuterPhase::BreakawayCampaign`] with the existing
/// [`OuterPhase::VelocityIntegral`] variant carrying the exact Stage-C plan, so
/// Stage-C execution state lives in that variant, off the campaign variant's
/// retained budget. Every non-accept campaign terminal -- including an accept
/// the 6a constructor refused WITHOUT motion -- fails the outer phase and never
/// begins Stage C, so no failure or inconclusive path can reach a Stage-C
/// stroke. Safe-state restoration on the resulting `Failed`/`FailedWithSafety`
/// terminal happens at the surrounding engine/dispatch boundary, as it does for
/// every other outer phase.
pub(crate) async fn step_breakaway_campaign_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
) -> StepResult {
    let step = {
        let OuterPhase::BreakawayCampaign {
            campaign,
            workspace,
            safety,
        } = phase
        else {
            unreachable!("step_breakaway_campaign_with_trace requires a breakaway campaign phase")
        };
        step_breakaway_campaign(campaign, workspace, safety, backend).await
    };
    match step {
        BreakawayCampaignStep::ReadyNow => StepResult::ReadyNow,
        BreakawayCampaignStep::At(deadline_us) => StepResult::At(deadline_us),
        BreakawayCampaignStep::Event(event) => StepResult::BreakawayCampaignEvent(event),
        BreakawayCampaignStep::Done(BreakawayCampaignOutcome::StageCPlanReady(plan)) => {
            let old =
                core::mem::replace(phase, OuterPhase::Failed(CommissionError::InvalidSchedule));
            let OuterPhase::BreakawayCampaign {
                campaign, safety, ..
            } = old
            else {
                unreachable!("phase was matched as a breakaway campaign above")
            };
            *phase = OuterPhase::VelocityIntegral {
                state: VelocityIntegralState::Setup,
                data: VelocityIntegralData::new(campaign.run_sequence(), plan.execution_plan()),
                safety,
            };
            crate::diagnostic::complete_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::StageCHandedOff,
            );
            StepResult::ReadyNow
        }
        BreakawayCampaignStep::Done(BreakawayCampaignOutcome::Terminal {
            error,
            safety: Some(fault),
        }) => {
            let result = fail_outer_with_safety(phase, error, fault);
            crate::diagnostic::complete_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
            );
            result
        }
        BreakawayCampaignStep::Done(BreakawayCampaignOutcome::Terminal {
            error,
            safety: None,
        }) => {
            let result = fail_outer(phase, error);
            crate::diagnostic::complete_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
            );
            result
        }
    }
}

#[cfg(test)]
#[path = "breakaway_campaign_tests.rs"]
mod tests;
