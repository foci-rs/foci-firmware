use super::acceptance_plan::{
    ACCEPTANCE_MATRIX_AMPLITUDES, ACCEPTANCE_MATRIX_CAPTURE_ALLOWANCE_S,
    ACCEPTANCE_MATRIX_SCHEMA_REVISION, AcceptanceMatrixOrder, AcceptanceMatrixPlanError,
    derive_acceptance_targets, plan_acceptance_matrix,
};
use super::combined_plan::select_proportional_response;
use super::integral_plan::{
    VelocityIntegralAcquisitionKind, plan_combined_velocity_integral_response,
};
use super::integral_reproduction::{AcceptanceMatrixProgress, AcceptancePoint, StageCPlanIdentity};
use super::operating_region::{BoundRounding, CompactSignedRate};
use super::reproduction::STAGE_B_BUILD_REVISION;
use super::sweep_consensus::DirectionalConsensusClass;
use super::sweep_evidence::CompactDirectionalRungReport;
use super::sweep_plan::{
    RequestedVelocitySource, VelocitySweepRequest, plan_combined_velocity_sweep,
};
use crate::commissioning::outer::OuterMotionBudget;
use foci_tmc4671::pid::PidPGain;

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

fn selected_response() -> super::combined_plan::CombinedSelectedResponse {
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
        report(13_217_792, 13_672_960),
        report(-13_308_416, -12_827_648),
        263,
        COUNTS_PER_REV,
    )
    .unwrap()
    .unwrap()
}

fn combined_plan() -> super::integral_plan::VelocityIntegralPlan {
    let request = request();
    let stage_b = plan_combined_velocity_sweep(
        request,
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
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

fn point(plan: super::integral_plan::VelocityIntegralPlan) -> AcceptancePoint {
    AcceptancePoint {
        identity: StageCPlanIdentity {
            channel: 0,
            build_revision: STAGE_B_BUILD_REVISION,
            schema_revision: plan.schema_revision,
            plan_digest: plan.digest(),
        },
        selected_p: plan.final_p,
        selected_i: 1_024,
        target_velocity_rpm: plan.target_velocity_rpm,
        selected_response_intervals_q: plan.execution_plan().selected_response_intervals_q,
        qualifier_bits: 0x0123,
        progress: AcceptanceMatrixProgress::ReadyAscending,
    }
}

#[test]
fn live_matrix_geometry_and_orders_are_exact() {
    assert_eq!(ACCEPTANCE_MATRIX_SCHEMA_REVISION, 4);
    assert_eq!(AcceptanceMatrixOrder::Ascending as u8, 1);
    assert_eq!(AcceptanceMatrixOrder::Descending as u8, 2);
    assert_eq!(ACCEPTANCE_MATRIX_AMPLITUDES, 5);
    assert_eq!(ACCEPTANCE_MATRIX_CAPTURE_ALLOWANCE_S, 110);
    let current = combined_plan();
    let ascending = plan_acceptance_matrix(
        point(current),
        request(),
        AcceptanceMatrixOrder::Ascending,
        current,
    )
    .unwrap();
    assert_eq!(ascending.target_rpm, [16, 33, 66, 132, 263]);
    let executed_mrev_s_milli = ascending
        .target_rpm
        .map(|rpm| (u64::try_from(rpm).unwrap() * 1_000_000).div_ceil(60));
    let stroke_travel_mrev_milli = ascending
        .target_rpm
        .map(|rpm| (u64::try_from(rpm).unwrap() * 512_000).div_ceil(60));
    assert_eq!(
        executed_mrev_s_milli,
        [266_667, 550_000, 1_100_000, 2_200_000, 4_383_334]
    );
    assert_eq!(
        stroke_travel_mrev_milli,
        [136_534, 281_600, 563_200, 1_126_400, 2_244_267]
    );
    assert!(
        stroke_travel_mrev_milli
            .into_iter()
            .all(|travel| travel <= 2_250_000)
    );
    assert_eq!(ascending.amplitude_count, 5);
    assert_eq!(ascending.expected_observations, 40);
    assert_eq!(ascending.family_size, 20);
    assert_eq!(ascending.recovery_slot_count, 5);
    assert_eq!(ascending.recovery_lower_rate_q, [13_217_792, 12_827_648]);
    assert_eq!(ascending.nominal_workflow_ms, 63_041);
    assert_eq!(ascending.maximum_workflow_ms, 66_456);
    assert_eq!(
        (
            super::combined_plan::COMBINED_STAGE_B_SCHEMA_REVISION,
            super::combined_plan::COMBINED_STAGE_C_SCHEMA_REVISION,
            ACCEPTANCE_MATRIX_SCHEMA_REVISION,
        ),
        (13, 12, 4)
    );
    assert_eq!([160 + 23, 120 + 15, 40 + 5], [183, 135, 45]);
    assert_eq!((183 + 135 + 45) * 78, 28_314);

    let descending = plan_acceptance_matrix(
        point(current),
        request(),
        AcceptanceMatrixOrder::Descending,
        current,
    )
    .unwrap();
    assert_eq!(descending.target_rpm, [263, 132, 66, 33, 16]);
    assert_eq!(
        descending
            .target_rpm
            .map(|rpm| (u64::try_from(rpm).unwrap() * 512_000).div_ceil(60)),
        [2_244_267, 1_126_400, 563_200, 281_600, 136_534]
    );
    assert_ne!(ascending.digest(), descending.digest());
}

#[test]
fn historical_schema_three_keeps_the_stabilized_recovery_digest_input() {
    let current = combined_plan();
    let mut historical = plan_acceptance_matrix(
        point(current),
        request(),
        AcceptanceMatrixOrder::Ascending,
        current,
    )
    .unwrap();
    historical.schema_revision = 3;

    assert_eq!(historical.digest(), 0xb3c5_9858_d412_178a);
}

#[test]
fn target_derivation_rounds_half_up_and_rejects_unsupported_geometry() {
    assert_eq!(derive_acceptance_targets(263), Ok([16, 33, 66, 132, 263]));
    assert_eq!(derive_acceptance_targets(257), Ok([16, 32, 64, 129, 257]));
    assert_eq!(
        derive_acceptance_targets(7),
        Err(AcceptanceMatrixPlanError::UnsupportedTargets)
    );
    assert_eq!(
        derive_acceptance_targets(8),
        Err(AcceptanceMatrixPlanError::UnsupportedTargets)
    );
    assert_eq!(
        derive_acceptance_targets(i32::MAX),
        Err(AcceptanceMatrixPlanError::Arithmetic)
    );
}

#[test]
fn compact_execution_plan_carries_fixed_i_and_literal_targets() {
    let current = combined_plan();
    let ascending = plan_acceptance_matrix(
        point(current),
        request(),
        AcceptanceMatrixOrder::Ascending,
        current,
    )
    .unwrap();
    let execution = ascending.execution_plan();

    assert_eq!(
        execution.acquisition_kind,
        VelocityIntegralAcquisitionKind::AcceptanceAscending
    );
    assert_eq!(execution.i_start, 1_024);
    assert_eq!(execution.matrix_targets_rpm, [16, 33, 66, 132, 263]);
    assert_eq!(execution.positive_rung_count, 5);
    assert_eq!(execution.family_size, 20);
    assert_eq!(execution.target_velocity_rpm, 263);
    assert_eq!(execution.planned_velocity_mrev_s, 4_394);
    assert_eq!(execution.recovery_lower_rate_q, [13_217_792, 12_827_648]);
    assert_eq!(
        execution.selected_response_intervals_q,
        [
            [
                ascending.acceptance_identity.plan_digest as u32 as i32,
                (ascending.acceptance_identity.plan_digest >> 32) as u32 as i32,
            ],
            [i32::from(ascending.qualifier_bits), 0],
        ]
    );
    assert_eq!(core::mem::size_of_val(&execution), 112);
}

#[test]
fn matrix_plan_rejects_identity_and_request_drift() {
    let current = combined_plan();
    let mut stale = point(current);
    stale.identity.plan_digest ^= 1;
    assert_eq!(
        plan_acceptance_matrix(stale, request(), AcceptanceMatrixOrder::Ascending, current),
        Err(AcceptanceMatrixPlanError::IdentityMismatch)
    );

    let mut changed_request = request();
    changed_request.max_duration_ms += 1;
    assert_eq!(
        plan_acceptance_matrix(
            point(current),
            changed_request,
            AcceptanceMatrixOrder::Ascending,
            current,
        ),
        Err(AcceptanceMatrixPlanError::IdentityMismatch)
    );
}

#[test]
fn matrix_plan_rejects_wrong_sided_recovery_authority() {
    let current = combined_plan();
    for intervals in [
        [[0, 13_672_960], [-13_308_416, -12_827_648]],
        [[13_217_792, 13_672_960], [-13_308_416, 0]],
    ] {
        let mut invalid = point(current);
        invalid.selected_response_intervals_q = intervals;
        assert_eq!(
            plan_acceptance_matrix(
                invalid,
                request(),
                AcceptanceMatrixOrder::Ascending,
                current,
            ),
            Err(AcceptanceMatrixPlanError::Arithmetic)
        );
    }
}

#[test]
fn matrix_digest_binds_every_explicit_plan_dimension() {
    let current = combined_plan();
    let base = plan_acceptance_matrix(
        point(current),
        request(),
        AcceptanceMatrixOrder::Ascending,
        current,
    )
    .unwrap();
    let mutate = |change: fn(&mut super::acceptance_plan::AcceptanceMatrixPlan)| {
        let mut changed = base;
        change(&mut changed);
        assert_ne!(base.digest(), changed.digest());
    };
    mutate(|plan| plan.selected_p += 1);
    mutate(|plan| plan.selected_i += 1);
    mutate(|plan| plan.target_rpm[0] += 1);
    mutate(|plan| plan.pwm_hz += 1);
    mutate(|plan| plan.encoder_counts_per_rev += 1);
    mutate(|plan| plan.moving_stroke_us += 1);
    mutate(|plan| plan.zero_settle_us += 1);
    mutate(|plan| plan.analysis_budget_us += 1);
    mutate(|plan| plan.family_size += 1);
    mutate(|plan| plan.hard_torque_limit += 1);
    mutate(|plan| plan.usable_torque_limit += 1);
    mutate(|plan| plan.recovery_origin_band_counts += 1);
    mutate(|plan| plan.recovery_lower_rate_q[0] += 1);
    mutate(|plan| plan.recovery_lower_rate_q[1] += 1);
    mutate(|plan| plan.recovery_maximum_slot_us += 1);
    mutate(|plan| plan.nominal_workflow_ms += 1);
    mutate(|plan| plan.maximum_workflow_ms += 1);
    mutate(|plan| plan.schema_revision += 1);
}
