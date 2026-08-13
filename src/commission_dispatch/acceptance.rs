//! Firmware-owned admission and retained lifecycle for velocity confidence matrices.

use foci_tmc4671::pid::PidPGain;

use crate::commission_dispatch::stage_c::StageCPreflightConfig;
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::acceptance_plan::{
    AcceptanceMatrixOrder, AcceptanceMatrixPlan, AcceptanceMatrixPlanError, plan_acceptance_matrix,
};
use crate::commissioning::outer::velocity::combined_plan::{
    BREAKAWAY_STAGE_C_SCHEMA_REVISION, COMBINED_STAGE_C_SCHEMA_REVISION, CombinedSelectedResponse,
};
use crate::commissioning::outer::velocity::integral_plan::{
    AcquisitionContext, VelocityIntegralPlan, VelocityIntegralPlanError,
    plan_combined_velocity_integral_response, plan_retained_breakaway_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, AcceptancePoint, StageCCandidateSlot,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepRequest, plan_combined_velocity_sweep,
};
use crate::commissioning::types::{
    AcceptanceMatrixCause, AcceptanceMatrixOutcome, AcceptanceMatrixTerminal,
};

pub use crate::tmc::AutotuneAction;

use crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder;

impl AutotuneAction {
    /// Return the immutable amplitude order for a matrix action.
    pub const fn matrix_order(self) -> Option<AcceptanceMatrixOrder> {
        match self {
            Self::Combined
            | Self::CombinedMirrored
            | Self::CombinedPaired
            | Self::BreakawaySeeded
            | Self::StageCResume => None,
            Self::MatrixAscending | Self::MatrixAscendingMirrored => {
                Some(AcceptanceMatrixOrder::Ascending)
            }
            Self::MatrixDescending | Self::MatrixDescendingMirrored => {
                Some(AcceptanceMatrixOrder::Descending)
            }
        }
    }

    /// Return the immutable observation slot order for a matrix action.
    ///
    /// Amplitude order and slot order are independent dimensions that share one
    /// wire byte; this keeps them separate in the type system.
    pub const fn matrix_slot_order(self) -> Option<IntegralSlotOrder> {
        match self {
            Self::Combined
            | Self::CombinedMirrored
            | Self::CombinedPaired
            | Self::BreakawaySeeded
            | Self::StageCResume => None,
            Self::MatrixAscending | Self::MatrixDescending => Some(IntegralSlotOrder::ForwardFirst),
            Self::MatrixAscendingMirrored | Self::MatrixDescendingMirrored => {
                Some(IntegralSlotOrder::ReverseFirst)
            }
        }
    }
}

/// Exact matrix plan accepted before any SPI write or enable assertion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcceptanceMatrixPreflight {
    /// Sole immutable matrix plan used for disclosure and execution.
    pub plan: AcceptanceMatrixPlan,
    /// Caller-assigned trace sequence.
    pub run_sequence: u32,
}

/// Definitive zero-motion matrix admission rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AcceptanceMatrixPreflightRejection {
    /// No target-reaching combined operating point is retained.
    MissingAcceptancePoint,
    /// Current controller geometry does not reconstruct the retained identity.
    AcceptancePointIdentityMismatch,
    /// The action does not match the retained two-run progress.
    MatrixOrderMismatch,
    /// Exact plan arithmetic or geometry was invalid.
    InvalidPlan,
}

/// Reconstruct the combined plan whose terminal installed the retained point.
pub fn reconstruct_combined_plan(
    point: crate::commissioning::outer::velocity::integral_reproduction::AcceptancePoint,
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_limit: u16,
    limit_evidence_capability: u32,
    pwm_hz: u32,
) -> Result<VelocityIntegralPlan, AcceptanceMatrixPreflightRejection> {
    let stage_b = plan_combined_velocity_sweep(
        request,
        encoder_counts_per_rev,
        torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| AcceptanceMatrixPreflightRejection::InvalidPlan)?;
    let p = PidPGain::new(point.selected_p)
        .map_err(|_| AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)?;
    let selected =
        CombinedSelectedResponse::from_retained_intervals(p, point.selected_response_intervals_q)
            .map_err(|_| AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)?;
    let motion_budget = OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
    };
    let current = plan_combined_velocity_integral_response(
        selected,
        stage_b.execution_plan(stage_b.digest()),
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    )
    .map_err(|error| match error {
        VelocityIntegralPlanError::Arithmetic
        | VelocityIntegralPlanError::InvalidRequest
        | VelocityIntegralPlanError::ConfidenceFamilyTooLarge
        | VelocityIntegralPlanError::InsufficientStructuralSpan => {
            AcceptanceMatrixPreflightRejection::InvalidPlan
        }
        _ => AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch,
    })?;
    if current.digest() != point.identity.plan_digest
        || current.schema_revision != point.identity.schema_revision
        || current.final_p != point.selected_p
        || current.target_velocity_rpm != point.target_velocity_rpm
    {
        return Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch);
    }
    Ok(current)
}

/// Reconstruct the breakaway plan whose confirmed token installed the retained point.
///
/// Breakaway carries no retained combined-selected-response plan, and its
/// I-ladder is deterministic (PWM frequency plus fixed constants), so the
/// plan is rebuilt directly from the point's carried `token`, `selected_p`,
/// and `selected_response_intervals_q` via
/// [`plan_retained_breakaway_velocity_integral_response`] -- the same builder
/// the Stage-C resume path already uses -- rather than from any retained full
/// plan. `point.token` is `Some` only for a genuine breakaway acceptance
/// point (see [`AcceptancePoint::token`]), so a point carrying none is
/// refused before any reconstruction is attempted. Parallel to
/// [`reconstruct_combined_plan`]: same inputs, same digest verification,
/// same rejection on any divergence.
pub fn reconstruct_breakaway_plan(
    point: AcceptancePoint,
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_limit: u16,
    limit_evidence_capability: u32,
    pwm_hz: u32,
) -> Result<VelocityIntegralPlan, AcceptanceMatrixPreflightRejection> {
    let token = point
        .token
        .ok_or(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)?;
    // Breakaway has no Stage-B phase of its own; the campaign that produced
    // this point derived its physical acquisition context (planned velocity,
    // target RPM, torque limits, limit-evidence capability) from exactly this
    // same `plan_combined_velocity_sweep` reconstruction, so this reproduces
    // it identically. The plan-time physical context guard inside the
    // breakaway builder compares against the live `pwm_hz`/
    // `encoder_counts_per_rev` passed below; there is no separately retained
    // plan-time value here, so any live drift is instead caught by the
    // digest check at the end of this function.
    let stage_b = plan_combined_velocity_sweep(
        request,
        encoder_counts_per_rev,
        torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| AcceptanceMatrixPreflightRejection::InvalidPlan)?;
    let mut context = AcquisitionContext::from(stage_b.execution_plan(stage_b.digest()));
    context.channel = point.identity.channel;
    context.plan_time_pwm_hz = pwm_hz;
    context.plan_time_encoder_counts_per_rev = encoder_counts_per_rev;
    let motion_budget = OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
    };
    let current = plan_retained_breakaway_velocity_integral_response(
        token,
        point.selected_p,
        context,
        point.selected_response_intervals_q,
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    )
    .map_err(|error| match error {
        VelocityIntegralPlanError::Arithmetic
        | VelocityIntegralPlanError::InvalidRequest
        | VelocityIntegralPlanError::ConfidenceFamilyTooLarge
        | VelocityIntegralPlanError::InsufficientStructuralSpan => {
            AcceptanceMatrixPreflightRejection::InvalidPlan
        }
        _ => AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch,
    })?;
    if current.digest() != point.identity.plan_digest
        || current.schema_revision != BREAKAWAY_STAGE_C_SCHEMA_REVISION
        || current.final_p != point.selected_p
        || current.target_velocity_rpm != point.target_velocity_rpm
    {
        return Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch);
    }
    Ok(current)
}

/// Validate a retained point and derive one immutable matrix without mutating retention.
///
/// Reconstructs the plan the retained point identifies before checking it:
/// combined points reproduce theirs from the request via
/// [`reconstruct_combined_plan`]; breakaway points rebuild theirs
/// deterministically from the carried confirmation token via
/// [`reconstruct_breakaway_plan`]. Either way, no full plan is retained --
/// only the compact [`AcceptancePoint`] the caller reads from
/// `stage_c_candidate`.
pub fn preflight_acceptance_matrix(
    action: AutotuneAction,
    request: VelocitySweepRequest,
    config: &StageCPreflightConfig,
    run_sequence: u32,
    stage_c_candidate: &StageCCandidateSlot,
) -> Result<AcceptanceMatrixPreflight, AcceptanceMatrixPreflightRejection> {
    let point = stage_c_candidate
        .acceptance_point()
        .ok_or(AcceptanceMatrixPreflightRejection::MissingAcceptancePoint)?;
    let order = action
        .matrix_order()
        .ok_or(AcceptanceMatrixPreflightRejection::MatrixOrderMismatch)?;
    let slot_order = action
        .matrix_slot_order()
        .ok_or(AcceptanceMatrixPreflightRejection::MatrixOrderMismatch)?;
    let expected_progress = match order {
        AcceptanceMatrixOrder::Ascending => AcceptanceMatrixProgress::ReadyAscending,
        AcceptanceMatrixOrder::Descending => AcceptanceMatrixProgress::ReadyDescending,
    };
    if point.progress != expected_progress {
        return Err(AcceptanceMatrixPreflightRejection::MatrixOrderMismatch);
    }
    let current_config = match point.identity.schema_revision {
        COMBINED_STAGE_C_SCHEMA_REVISION => reconstruct_combined_plan(
            point,
            request,
            config.encoder_counts_per_rev,
            config.torque_limit,
            config.limit_evidence_capability,
            config.pwm_hz,
        )?,
        BREAKAWAY_STAGE_C_SCHEMA_REVISION => reconstruct_breakaway_plan(
            point,
            request,
            config.encoder_counts_per_rev,
            config.torque_limit,
            config.limit_evidence_capability,
            config.pwm_hz,
        )?,
        _ => return Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch),
    };
    let plan = plan_acceptance_matrix(point, request, order, slot_order, current_config).map_err(
        |error| match error {
            AcceptanceMatrixPlanError::IdentityMismatch => {
                AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch
            }
            AcceptanceMatrixPlanError::UnsupportedTargets
            | AcceptanceMatrixPlanError::Arithmetic => {
                AcceptanceMatrixPreflightRejection::InvalidPlan
            }
        },
    )?;
    Ok(AcceptanceMatrixPreflight { plan, run_sequence })
}

/// Apply one active matrix terminal after disarm-first restoration.
pub fn apply_acceptance_matrix_terminal(
    mut terminal: AcceptanceMatrixTerminal,
    order: AcceptanceMatrixOrder,
    expected_plan_digest: u64,
    stage_c_candidate: &mut StageCCandidateSlot,
) -> AcceptanceMatrixTerminal {
    let Some(point) = stage_c_candidate.acceptance_point() else {
        terminal.outcome = AcceptanceMatrixOutcome::Fault;
        terminal.cause = AcceptanceMatrixCause::EvidenceIntegrity;
        return terminal;
    };
    let expected_progress = match order {
        AcceptanceMatrixOrder::Ascending => AcceptanceMatrixProgress::ReadyAscending,
        AcceptanceMatrixOrder::Descending => AcceptanceMatrixProgress::ReadyDescending,
    };
    if point.identity.plan_digest != terminal.acceptance_plan_digest
        || terminal.plan_digest != expected_plan_digest
        || point.progress != expected_progress
    {
        stage_c_candidate.observe_fault();
        terminal.outcome = AcceptanceMatrixOutcome::Fault;
        terminal.cause = AcceptanceMatrixCause::EvidenceIntegrity;
        return terminal;
    }

    let transition_ok = match terminal.outcome {
        AcceptanceMatrixOutcome::Complete => match order {
            AcceptanceMatrixOrder::Ascending => stage_c_candidate.advance_matrix_progress(),
            AcceptanceMatrixOrder::Descending => stage_c_candidate.close_matrix_progress(),
        },
        AcceptanceMatrixOutcome::Inconclusive
            if terminal.cause == AcceptanceMatrixCause::VelocityRestNotConfirmed =>
        {
            true
        }
        AcceptanceMatrixOutcome::Inconclusive => stage_c_candidate.close_matrix_progress(),
        AcceptanceMatrixOutcome::Fault => {
            stage_c_candidate.observe_fault();
            true
        }
        AcceptanceMatrixOutcome::Failed => true,
    };
    if !transition_ok {
        stage_c_candidate.observe_fault();
        terminal.outcome = AcceptanceMatrixOutcome::Fault;
        terminal.cause = AcceptanceMatrixCause::EvidenceIntegrity;
    }
    terminal
}

/// Build one definitive zero-motion matrix terminal from admission policy.
pub fn failed_acceptance_matrix_terminal(
    run_sequence: u32,
    rejection: AcceptanceMatrixPreflightRejection,
    stage_c_candidate: &StageCCandidateSlot,
) -> AcceptanceMatrixTerminal {
    let cause = match rejection {
        AcceptanceMatrixPreflightRejection::MissingAcceptancePoint => {
            AcceptanceMatrixCause::MissingAcceptancePoint
        }
        AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch => {
            AcceptanceMatrixCause::AcceptancePointIdentityMismatch
        }
        AcceptanceMatrixPreflightRejection::MatrixOrderMismatch => {
            AcceptanceMatrixCause::MatrixOrderMismatch
        }
        AcceptanceMatrixPreflightRejection::InvalidPlan => AcceptanceMatrixCause::EvidenceIntegrity,
    };
    AcceptanceMatrixTerminal {
        schema_revision:
            crate::commissioning::outer::velocity::acceptance_plan::ACCEPTANCE_MATRIX_SCHEMA_REVISION,
        run_sequence,
        evidence_sequence: 0,
        acceptance_plan_digest: stage_c_candidate
            .acceptance_point()
            .map_or(0, |point| point.identity.plan_digest),
        plan_digest: 0,
        digest: 0,
        outcome: AcceptanceMatrixOutcome::Failed,
        cause,
        attempted_masks: [0; 2],
        eligible_masks: [0; 2],
        current_terminus_masks: [0; 2],
        unattempted_masks: [0; 2],
        expected_observations: 0,
        emitted_observations: 0,
        expected_amplitudes: 0,
        emitted_amplitudes: 0,
        qualifier_bits: stage_c_candidate
            .acceptance_point()
            .map_or(0, |point| point.qualifier_bits),
        run_started_us: 0,
        run_completed_us: 0,
        safety_fault: None,
    }
}

#[cfg(test)]
pub(crate) fn test_acceptance_point(
    request: VelocitySweepRequest,
    progress: AcceptanceMatrixProgress,
) -> crate::commissioning::outer::velocity::integral_reproduction::AcceptancePoint {
    use crate::commissioning::outer::velocity::combined_plan::select_proportional_response;
    use crate::commissioning::outer::velocity::operating_region::{
        BoundRounding, CompactSignedRate,
    };
    use crate::commissioning::outer::velocity::reproduction::BUILD_FEATURES;
    use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
    use crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport;

    let report = |low_q, high_q| CompactDirectionalRungReport {
        p: PidPGain::new(1_024).unwrap(),
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0; 2]; 2],
        pooled: None,
        settled_rate_bounds_q: [
            CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
            CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
        ],
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
        operable_count: 4,
        flags: 0,
    };
    let stage_b = plan_combined_velocity_sweep(
        request,
        4_000,
        2_816,
        crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let target_rpm = stage_b.target_velocity_rpm.raw();
    let target_rate_q = i64::from(target_rpm) * 4_000 * 1_000 / 60;
    let response_low = target_rate_q * 75 / 100;
    let response_high = target_rate_q * 76 / 100;
    let selected = select_proportional_response(
        report(response_low, response_high),
        report(-response_high, -response_low),
        target_rpm,
        4_000,
    )
    .unwrap()
    .unwrap();
    let plan = plan_combined_velocity_integral_response(
        selected,
        stage_b.execution_plan(stage_b.digest()),
        OuterMotionBudget {
            requested_velocity_mrev_s: request.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: request.requested_velocity_source,
            max_stroke_travel_mrev: request.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: request.negative_position_headroom_mrev,
            positive_position_headroom_mrev: request.positive_position_headroom_mrev,
            max_duration_ms: request.max_duration_ms,
        },
        25_000,
        4_000,
    )
    .unwrap();
    crate::commissioning::outer::velocity::integral_reproduction::AcceptancePoint {
        identity:
            crate::commissioning::outer::velocity::integral_reproduction::StageCPlanIdentity {
                channel: 0,
                build_features: BUILD_FEATURES,
                schema_revision: plan.schema_revision,
                plan_digest: plan.digest(),
            },
        selected_p: plan.final_p,
        selected_i: 1_024,
        target_velocity_rpm: plan.target_velocity_rpm,
        selected_response_intervals_q: plan.execution_plan().selected_response_intervals_q,
        qualifier_bits: 0,
        token: None,
        progress,
    }
}
