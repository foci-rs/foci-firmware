use super::acceptance::{
    AcceptanceMatrixPreflightRejection, AutotuneAction, apply_acceptance_matrix_terminal,
    preflight_acceptance_matrix, reconstruct_combined_plan,
};
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use crate::commissioning::outer::velocity::acceptance_plan::AcceptanceMatrixOrder;
use crate::commissioning::outer::velocity::combined_plan::{
    CombinedSelectedResponse, select_proportional_response,
};
use crate::commissioning::outer::velocity::integral_plan::{
    CommissioningWorkflowPlan, StageCPlanShape, VelocityIntegralPlan,
    plan_combined_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, AcceptancePoint, StageCCandidateSlot, StageCPlanIdentity,
};
use crate::commissioning::outer::velocity::operating_region::{BoundRounding, CompactSignedRate};
use crate::commissioning::outer::velocity::reproduction::STAGE_B_BUILD_REVISION;
use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
use crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport;
use crate::commissioning::outer::velocity::sweep_plan::{
    RequestedVelocitySource, VelocitySweepRequest, plan_combined_velocity_sweep,
};
use crate::commissioning::types::{
    AcceptanceMatrixCause, AcceptanceMatrixOutcome, AcceptanceMatrixTerminal,
};
use foci_tmc4671::pid::PidPGain;

const CHANNEL: u8 = 0;
const COUNTS_PER_REV: u32 = 4_000;
const PWM_HZ: u32 = 25_000;
const TORQUE_LIMIT: u16 = 2_816;

fn request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 4_394,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    }
}

fn selected_response() -> CombinedSelectedResponse {
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
    select_proportional_response(
        report(13_300_000, 13_500_000),
        report(-13_500_000, -13_300_000),
        263,
        COUNTS_PER_REV,
    )
    .unwrap()
    .unwrap()
}

fn combined_plan() -> VelocityIntegralPlan {
    let request = request();
    let stage_b = plan_combined_velocity_sweep(
        request,
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    plan_combined_velocity_integral_response(
        selected_response(),
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
        PWM_HZ,
        COUNTS_PER_REV,
    )
    .unwrap()
}

fn point(plan: VelocityIntegralPlan, progress: AcceptanceMatrixProgress) -> AcceptancePoint {
    AcceptancePoint {
        identity: StageCPlanIdentity {
            channel: CHANNEL,
            build_revision: STAGE_B_BUILD_REVISION,
            schema_revision: plan.schema_revision,
            plan_digest: plan.digest(),
        },
        selected_p: plan.final_p,
        selected_i: 1_024,
        target_velocity_rpm: plan.target_velocity_rpm,
        selected_response_intervals_q: plan.execution_plan().selected_response_intervals_q,
        qualifier_bits: 0x0123,
        progress,
    }
}

fn terminal(plan_digest: u64, outcome: AcceptanceMatrixOutcome) -> AcceptanceMatrixTerminal {
    AcceptanceMatrixTerminal {
        schema_revision: 1,
        run_sequence: 9,
        evidence_sequence: 1,
        acceptance_plan_digest: combined_plan().digest(),
        plan_digest,
        digest: 0x1234,
        outcome,
        cause: AcceptanceMatrixCause::None,
        attempted_masks: [0b1_1111; 2],
        eligible_masks: [0b1_1111; 2],
        current_terminus_masks: [0; 2],
        unattempted_masks: [0; 2],
        expected_observations: 40,
        emitted_observations: 40,
        expected_amplitudes: 5,
        emitted_amplitudes: 5,
        qualifier_bits: 0x0123,
        run_started_us: 1,
        run_completed_us: 2,
        safety_fault: None,
    }
}

#[test]
fn action_codes_and_invalid_values_are_exact() {
    assert_eq!(AutotuneAction::Combined as u8, 0);
    assert_eq!(AutotuneAction::MatrixAscending as u8, 1);
    assert_eq!(AutotuneAction::MatrixDescending as u8, 2);
    assert_eq!(AutotuneAction::from_u8(0), Some(AutotuneAction::Combined));
    assert_eq!(AutotuneAction::from_u8(3), None);
    let ascending = CommissioningWorkflowPlan::new_acceptance_matrix(
        1,
        AcceptanceMatrixOrder::Ascending,
        60_541,
        66_456,
    )
    .unwrap();
    let descending = CommissioningWorkflowPlan::new_acceptance_matrix(
        2,
        AcceptanceMatrixOrder::Descending,
        60_541,
        66_456,
    )
    .unwrap();
    assert_eq!(ascending.shape, StageCPlanShape::AcceptanceMatrixAscending);
    assert_eq!(
        descending.shape,
        StageCPlanShape::AcceptanceMatrixDescending
    );
}

#[test]
fn preflight_requires_matching_point_and_order_without_mutation() {
    let current = combined_plan();
    let mut empty = StageCCandidateSlot::new();
    assert_eq!(
        preflight_acceptance_matrix(
            AutotuneAction::MatrixAscending,
            request(),
            current,
            7,
            &empty,
        ),
        Err(AcceptanceMatrixPreflightRejection::MissingAcceptancePoint)
    );

    let mut stale = point(current, AcceptanceMatrixProgress::ReadyAscending);
    stale.identity.plan_digest ^= 1;
    empty.install_acceptance_point(stale);
    assert_eq!(
        preflight_acceptance_matrix(
            AutotuneAction::MatrixAscending,
            request(),
            current,
            7,
            &empty,
        ),
        Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)
    );
    assert_eq!(empty.acceptance_point(), Some(stale));

    let ready_descending = point(current, AcceptanceMatrixProgress::ReadyDescending);
    empty.install_acceptance_point(ready_descending);
    assert_eq!(
        preflight_acceptance_matrix(
            AutotuneAction::MatrixAscending,
            request(),
            current,
            7,
            &empty,
        ),
        Err(AcceptanceMatrixPreflightRejection::MatrixOrderMismatch)
    );
    assert_eq!(empty.acceptance_point(), Some(ready_descending));
}

#[test]
fn preflight_admits_only_the_expected_matrix_direction() {
    let current = combined_plan();
    let mut slot = StageCCandidateSlot::new();
    slot.install_acceptance_point(point(current, AcceptanceMatrixProgress::ReadyAscending));
    let ascending = preflight_acceptance_matrix(
        AutotuneAction::MatrixAscending,
        request(),
        current,
        7,
        &slot,
    )
    .unwrap();
    assert_eq!(ascending.plan.order, AcceptanceMatrixOrder::Ascending);

    slot.advance_matrix_progress();
    let descending = preflight_acceptance_matrix(
        AutotuneAction::MatrixDescending,
        request(),
        current,
        8,
        &slot,
    )
    .unwrap();
    assert_eq!(descending.plan.order, AcceptanceMatrixOrder::Descending);
}

#[test]
fn retained_response_reconstructs_the_exact_combined_identity() {
    let current = combined_plan();
    let retained = point(current, AcceptanceMatrixProgress::ReadyAscending);
    assert_eq!(
        reconstruct_combined_plan(
            retained,
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            PWM_HZ,
        )
        .unwrap()
        .digest(),
        retained.identity.plan_digest
    );

    let mut changed = request();
    changed.max_duration_ms += 1;
    assert_eq!(
        reconstruct_combined_plan(
            retained,
            changed,
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            PWM_HZ,
        ),
        Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)
    );
}

#[test]
fn safe_terminals_advance_or_close_and_fault_clears() {
    let current = combined_plan();
    let mut slot = StageCCandidateSlot::new();
    slot.install_acceptance_point(point(current, AcceptanceMatrixProgress::ReadyAscending));
    let ascending = preflight_acceptance_matrix(
        AutotuneAction::MatrixAscending,
        request(),
        current,
        7,
        &slot,
    )
    .unwrap();
    apply_acceptance_matrix_terminal(
        terminal(ascending.plan.digest(), AcceptanceMatrixOutcome::Complete),
        AcceptanceMatrixOrder::Ascending,
        ascending.plan.digest(),
        &mut slot,
    );
    assert_eq!(
        slot.acceptance_point().unwrap().progress,
        AcceptanceMatrixProgress::ReadyDescending
    );

    let descending = preflight_acceptance_matrix(
        AutotuneAction::MatrixDescending,
        request(),
        current,
        8,
        &slot,
    )
    .unwrap();
    apply_acceptance_matrix_terminal(
        terminal(
            descending.plan.digest(),
            AcceptanceMatrixOutcome::Inconclusive,
        ),
        AcceptanceMatrixOrder::Descending,
        descending.plan.digest(),
        &mut slot,
    );
    assert_eq!(
        slot.acceptance_point().unwrap().progress,
        AcceptanceMatrixProgress::Closed
    );

    slot.install_acceptance_point(point(current, AcceptanceMatrixProgress::ReadyAscending));
    apply_acceptance_matrix_terminal(
        terminal(ascending.plan.digest(), AcceptanceMatrixOutcome::Fault),
        AcceptanceMatrixOrder::Ascending,
        ascending.plan.digest(),
        &mut slot,
    );
    assert_eq!(slot.acceptance_point(), None);
}
