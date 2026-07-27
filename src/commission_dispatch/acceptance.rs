//! Firmware-owned admission and retained lifecycle for velocity confidence matrices.

use foci_tmc4671::pid::PidPGain;

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::acceptance_plan::{
    AcceptanceMatrixOrder, AcceptanceMatrixPlan, AcceptanceMatrixPlanError, plan_acceptance_matrix,
};
use crate::commissioning::outer::velocity::combined_plan::CombinedSelectedResponse;
use crate::commissioning::outer::velocity::integral_plan::{
    VelocityIntegralPlan, VelocityIntegralPlanError, plan_combined_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, StageCCandidateSlot,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepRequest, plan_combined_velocity_sweep,
};
use crate::commissioning::types::{
    AcceptanceMatrixCause, AcceptanceMatrixOutcome, AcceptanceMatrixTerminal,
};

pub use crate::tmc::AutotuneAction;

impl AutotuneAction {
    /// Return the immutable matrix order for a matrix action.
    pub const fn matrix_order(self) -> Option<AcceptanceMatrixOrder> {
        match self {
            Self::Combined => None,
            Self::MatrixAscending => Some(AcceptanceMatrixOrder::Ascending),
            Self::MatrixDescending => Some(AcceptanceMatrixOrder::Descending),
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

/// Validate a retained point and derive one immutable matrix without mutating retention.
pub fn preflight_acceptance_matrix(
    action: AutotuneAction,
    request: VelocitySweepRequest,
    current_config: VelocityIntegralPlan,
    run_sequence: u32,
    stage_c_candidate: &StageCCandidateSlot,
) -> Result<AcceptanceMatrixPreflight, AcceptanceMatrixPreflightRejection> {
    let point = stage_c_candidate
        .acceptance_point()
        .ok_or(AcceptanceMatrixPreflightRejection::MissingAcceptancePoint)?;
    let order = action
        .matrix_order()
        .ok_or(AcceptanceMatrixPreflightRejection::MatrixOrderMismatch)?;
    let expected_progress = match order {
        AcceptanceMatrixOrder::Ascending => AcceptanceMatrixProgress::ReadyAscending,
        AcceptanceMatrixOrder::Descending => AcceptanceMatrixProgress::ReadyDescending,
    };
    if point.progress != expected_progress {
        return Err(AcceptanceMatrixPreflightRejection::MatrixOrderMismatch);
    }
    let plan =
        plan_acceptance_matrix(point, request, order, current_config).map_err(
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
