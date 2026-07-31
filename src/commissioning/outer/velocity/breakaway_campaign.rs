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
//! Nothing outside this module's own tests calls into either driver yet -- wiring
//! either into [`super::super::OuterPhase`] is a later slice's job (see the
//! module-level docs above) -- so, matching [`super::current_guard`]'s own
//! `#![cfg_attr(not(test), allow(dead_code))]`, dead-code warnings are suppressed
//! outside test builds.
#![cfg_attr(not(test), allow(dead_code))]

use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use crate::autotune::velocity_measurement::{
    DyadicBlocker, StationarityResult, StationarityWork, VELOCITY_RATE_Q_SCALE, VelocityDirection,
    confidence_multiplier_95_df15,
};
use crate::autotune::{CommissionError, EquivalentTorqueTargetInterval, TmcMechanicalRpm};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::OuterSafetyLimits;
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocityObservationRing};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_CURRENT, OUTER_SAFETY_FAULT_OBSERVATION_GAP,
    OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault,
};

use super::band_confirmation::{
    CONFIRMATION_BAND_LOWER_PERCENT, CONFIRMATION_BAND_UPPER_PERCENT, DiscoveryObservationAtom,
    DiscoveryRungNominationInput, rung_response_bounds_percent,
};
use super::breakaway_plan::{
    AdditiveDiscoveryPlan, AdditiveDiscoveryRequest, AdditiveLadderPlanError, BreakawayProbePlan,
    PROBE_CONFIDENCE_FAMILY_SIZE, plan_additive_discovery_ladder,
};
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
use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    FULL_OBSERVATION_MASK, depth_three_consensus,
};
use super::sweep_evidence::{VelocityObservationClass, classify_velocity_observation};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US,
    SETTLE_ANALYSIS_BUDGET_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US, ZERO_SETTLE_US,
};

/// Outer velocity ramp mode. Matches the wire encoding
/// [`super::sweep::step_velocity_sweep_with_trace`] writes to `MODE_RAMP_MODE_MOTION`;
/// duplicated here because that constant is private to the `sweep` module. Shared by
/// both the probe and the additive discovery driver: both command a single-direction
/// velocity ramp with I held at zero.
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
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_RAMP_MODE)
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
            // Mirror sweep.rs's AnalyzeMoving `terminal_error` match (sweep.rs:1076):
            // wrong-direction and hard-clip mean the direction-to-motion assumption
            // origin-recovery relies on is untrustworthy, or that current is already
            // clipped, so recovery must not run another energized stroke on top of
            // this observation. Invalid controller-domain evidence is likewise an
            // immediate terminal there. `classify_breakaway_observation`'s own
            // `BreakawaySearchError::ProbeEvidenceExcluded` collapses
            // `StationarityExcluded` and `InvalidEvidenceFault` into one variant, so
            // the raw classification is inspected here to keep that distinction:
            // sweep continues normally (settle + recovery) on `StationarityExcluded`,
            // only short-circuiting on `InvalidEvidenceFault`. `CurrentHeadroom` is
            // deliberately untouched: the sweep also runs settle + recovery for it.
            match classify_velocity_observation(input).classification {
                VelocityObservationClass::WrongDirectionFault => {
                    return fault(CommissionError::VelocityValidationFailed);
                }
                VelocityObservationClass::HardClipFault => {
                    return fault(CommissionError::SafetyEnvelopeViolation);
                }
                VelocityObservationClass::InvalidEvidenceFault => {
                    return BreakawayProbeStep::Done(BreakawayProbeOutcome::ProbeEvidenceExcluded);
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

const EMPTY_DIRECTIONAL_CONSENSUS: DirectionalConsensus = DirectionalConsensus {
    classification: DirectionalConsensusClass::IncompleteRung,
    components: [[0; 2]; 2],
    component_count: 0,
    collected_mask: 0,
    eligible_mask: 0,
    included_mask: 0,
};

const EMPTY_DISCOVERY_RUNG: DiscoveryRungNominationInput = DiscoveryRungNominationInput {
    p: PidPGain::ZERO,
    forward_consensus: EMPTY_DIRECTIONAL_CONSENSUS,
    forward_observations: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    reverse_consensus: EMPTY_DIRECTIONAL_CONSENSUS,
    reverse_observations: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
};

/// Every discovery rung whose own response was contained in 70-80%, collected in
/// ascending rung order for 6d/6e's later
/// `nominate_confirmation_candidate`(super::band_confirmation::nominate_confirmation_candidate) call.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct CollectedInBandRungs {
    rungs: [DiscoveryRungNominationInput; MAX_IN_BAND_DISCOVERY_RUNGS],
    count: u8,
}

impl CollectedInBandRungs {
    const fn new() -> Self {
        Self {
            rungs: [EMPTY_DISCOVERY_RUNG; MAX_IN_BAND_DISCOVERY_RUNGS],
            count: 0,
        }
    }

    /// Every collected in-band rung, in ascending rung order.
    pub(crate) fn as_slice(&self) -> &[DiscoveryRungNominationInput] {
        &self.rungs[..usize::from(self.count)]
    }

    /// Number of collected in-band rungs.
    pub(crate) const fn count(&self) -> u8 {
        self.count
    }

    fn push(&mut self, rung: DiscoveryRungNominationInput) -> Result<(), CommissionError> {
        let slot = self
            .rungs
            .get_mut(usize::from(self.count))
            .ok_or(CommissionError::InvalidSchedule)?;
        *slot = rung;
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

    /// Fold this direction's four collected observations into their depth-three
    /// consensus. Always called after every slot has been recorded, so the
    /// collected mask is always complete.
    fn consensus(&self) -> DirectionalConsensus {
        depth_three_consensus(FULL_OBSERVATION_MASK, self.classes, self.intervals)
    }
}

/// One classified stroke's evidence, held between [`AdditiveDiscoveryState::AnalyzeMoving`]
/// and [`AdditiveDiscoveryState::Advance`] the same way
/// [`BreakawayProbeSession::pending_class`](BreakawayProbeSession) holds the probe's
/// classification across origin-recovery.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingDiscoveryRecord {
    classification: VelocityObservationClass,
    interval: Option<EquivalentTorqueTargetInterval>,
    atom: DiscoveryObservationAtom,
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
/// classification into a bisection cursor, while discovery folds one into a
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
    /// what is next: another slot, another direction, another rung, or a
    /// terminal.
    Advance,
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
    /// The discovery campaign concluded.
    Done(AdditiveDiscoveryOutcome),
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
/// `TargetBandSelected` is far larger than every other variant: it embeds
/// [`CollectedInBandRungs`], sized at [`MAX_SWEEP_RUNGS`] because, in the worst
/// case, every rung on the ladder could independently satisfy its own
/// containment and no candidate evidence may be silently dropped. This driver is
/// not yet wired into [`super::super::OuterPhase`]/`CommissioningEngine` (see the
/// module-level docs), so today's size has no retained-state cost; a later
/// wiring slice may revisit the representation once it is. `large_enum_variant`
/// is accepted here the same way it already is at the `commission_dispatch::mod`
/// and `commissioning::schedule` terminal types, rather than boxed -- this crate
/// has no heap.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum AdditiveDiscoveryOutcome {
    /// At least one rung's own response was contained in 70-80%. Carries every
    /// in-band rung collected before the ladder stopped, in ascending order,
    /// ready for [`nominate_confirmation_candidate`](super::band_confirmation::nominate_confirmation_candidate).
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
    DiscoveryEvidenceExcluded,
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
            let hard_limit = u16::from(backend.current_limit());
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
                .write_register(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
                || safety.begin_primitive(backend.elapsed_us()).is_err()
            {
                return discovery_fault(CommissionError::Spi);
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
            let read_us = match sample_one_tick(workspace, safety, backend).await {
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
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: u16::from(backend.current_limit()),
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
                        AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded,
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
                atom: DiscoveryObservationAtom {
                    mean_rate_q: input.settled_rate_mean_q,
                    variance_of_mean_q2,
                    target_rate_q,
                    direction: session.direction,
                },
            });
            workspace.analysis.reset();
            workspace.observations.clear();
            *state = AdditiveDiscoveryState::BeginSettleCapture {
                settle_deadline_us: *settle_deadline_us,
            };
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::BeginSettleCapture { settle_deadline_us } => {
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
            };
            AdditiveDiscoveryStep::At(next_sample_us)
        }
        AdditiveDiscoveryState::SettleCapture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return AdditiveDiscoveryStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend).await {
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
            workspace.analysis.reset();
            workspace.observations.clear();
            safety.end_primitive();
            if !confirmation.confirmed_rest {
                return AdditiveDiscoveryStep::Done(
                    AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded,
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
            let hard_limit = u16::from(backend.current_limit());
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
                    *state = AdditiveDiscoveryState::Advance;
                    AdditiveDiscoveryStep::ReadyNow
                }
                RungOriginRecoveryStep::RestUnconfirmed => {
                    AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded)
                }
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
            match session.direction {
                VelocityDirection::Forward => session.forward.record(
                    session.slot,
                    record.classification,
                    record.interval,
                    record.atom,
                ),
                VelocityDirection::Reverse => session.reverse.record(
                    session.slot,
                    record.classification,
                    record.interval,
                    record.atom,
                ),
            }

            if session.pending_headroom_stop {
                return AdditiveDiscoveryStep::Done(if session.in_band.count() > 0 {
                    AdditiveDiscoveryOutcome::TargetBandSelected(session.in_band)
                } else {
                    AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
                });
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
                    if verdict == RungBandVerdict::InBand
                        && session.in_band.push(rung_input).is_err()
                    {
                        return discovery_fault(CommissionError::InvalidSchedule);
                    }
                    if verdict == RungBandVerdict::ConfidentlyAboveBand {
                        return AdditiveDiscoveryStep::Done(if session.in_band.count() > 0 {
                            AdditiveDiscoveryOutcome::TargetBandSelected(session.in_band)
                        } else {
                            AdditiveDiscoveryOutcome::TargetBandSkipped
                        });
                    }

                    session.forward = DirectionAccumulator::new();
                    session.reverse = DirectionAccumulator::new();

                    let Some(next_rung) = session.rung_index.checked_add(1) else {
                        return discovery_fault(CommissionError::InvalidSchedule);
                    };
                    if usize::from(next_rung) >= usize::from(session.plan.rung_count) {
                        return AdditiveDiscoveryStep::Done(if session.in_band.count() > 0 {
                            AdditiveDiscoveryOutcome::TargetBandSelected(session.in_band)
                        } else {
                            AdditiveDiscoveryOutcome::TargetBandSkipped
                        });
                    }
                    session.rung_index = next_rung;
                    session.direction = VelocityDirection::Forward;
                    session.slot = 0;
                    *state = AdditiveDiscoveryState::WriteGain;
                    AdditiveDiscoveryStep::ReadyNow
                }
            }
        }
    }
}

#[cfg(test)]
#[path = "breakaway_campaign_tests.rs"]
mod tests;
