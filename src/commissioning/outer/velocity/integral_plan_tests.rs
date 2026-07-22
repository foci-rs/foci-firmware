use super::integral_plan::{
    CommissioningWorkflowPlan, STAGE_C_EVIDENCE_SCHEMA_REVISION, STAGE_C_MAX_POSITIVE_RUNGS,
    STAGE_C_OBSERVATIONS_PER_RUNG, StageCAuthority, StageCPlanShape, VelocityIntegralPlanError,
    build_integral_ladder, confidence_family_size, plan_velocity_integral_response,
    velocity_i_tau_us,
};
use super::sweep_plan::{
    MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US, RequestedVelocitySource, SETTLE_ANALYSIS_BUDGET_US,
    VelocitySweepRequest, ZERO_SETTLE_US, plan_velocity_sweep,
};
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};

fn stage_b_request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    }
}

fn authority(final_p: u16) -> StageCAuthority {
    StageCAuthority {
        channel: 0,
        stage_b_plan_digest: 0x0123_4567_89ab_cdef,
        final_p,
        directional_memberships: [0x0038_0000, 0x0038_0000],
        joint_membership: 0x0038_0000,
        directional_pooled_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
    }
}

fn stage_b_plan() -> super::sweep_plan::VelocitySweepPlan {
    plan_velocity_sweep(stage_b_request(), 4_000, 2_816).unwrap()
}

fn motion_budget() -> OuterMotionBudget {
    OuterMotionBudget {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    }
}

#[test]
fn current_geometry_derives_the_exact_octave_ladder() {
    assert_eq!(velocity_i_tau_us(5, 25_000), Some(524_288));
    assert_eq!(velocity_i_tau_us(80, 25_000), Some(32_768));

    let ladder = build_integral_ladder(
        25_000,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )
    .unwrap();

    assert_eq!(ladder.i_start, 5);
    assert_eq!(ladder.i_top_cap, 81);
    assert_eq!(ladder.rungs.as_slice(), &[5, 10, 20, 40, 80]);
}

#[test]
fn top_cap_is_not_appended_as_a_non_octave_rung() {
    let ladder = build_integral_ladder(25_000, 512_000, 64_000, 512).unwrap();
    assert_eq!(ladder.i_top_cap, 81);
    assert_eq!(ladder.rungs.last(), Some(&80));
    assert!(!ladder.rungs.contains(&81));
}

#[test]
fn confidence_family_counts_both_anchors() {
    assert_eq!(STAGE_C_MAX_POSITIVE_RUNGS, 30);
    assert_eq!(STAGE_C_OBSERVATIONS_PER_RUNG, 8);
    assert_eq!(confidence_family_size(5), Ok(28));
    assert_eq!(confidence_family_size(30), Ok(128));
    assert_eq!(
        confidence_family_size(31),
        Err(VelocityIntegralPlanError::ConfidenceFamilyTooLarge)
    );
}

#[test]
fn stage_c_plan_freezes_geometry_and_conservative_duration() {
    let stage_b = stage_b_plan();
    let mut authority = authority(1_448);
    authority.stage_b_plan_digest = stage_b.digest();
    let plan = plan_velocity_integral_response(
        authority,
        stage_b.execution_plan(stage_b.digest()),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();

    assert_eq!(plan.schema_revision, STAGE_C_EVIDENCE_SCHEMA_REVISION);
    assert_eq!(plan.final_p, 1_448);
    assert_eq!(plan.positive_rungs(), &[5, 10, 20, 40, 80]);
    assert_eq!(plan.family_size, 28);
    assert_eq!(plan.total_rung_count, 7);
    assert_eq!(plan.expected_observations, 56);
    assert_eq!(plan.moving_stroke_us, 512_000);
    assert_eq!(plan.zero_settle_us, 500_000);
    assert_eq!(plan.analysis_budget_us, 236_000);
    assert_eq!(plan.recovery_origin_band_counts, 1_000);
    assert_eq!(plan.recovery_nominal_slot_us, 1_816_958);
    assert_eq!(plan.recovery_maximum_slot_us, 3_000_000);
    assert_eq!(plan.recovery_slot_count, 7);
    assert_eq!(plan.nominal_workflow_ms, 82_607);
    assert_eq!(plan.maximum_workflow_ms, 90_888);
    let base_us = u32::from(plan.total_rung_count)
        * STAGE_C_OBSERVATIONS_PER_RUNG
        * (MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US);
    let recovery_us = u32::from(plan.total_rung_count) * plan.recovery_nominal_slot_us;
    assert_eq!(
        plan.nominal_workflow_ms,
        (base_us + recovery_us).div_ceil(1_000)
    );
}

#[test]
fn workflow_shape_does_not_change_either_stage_identity() {
    let stage_b = stage_b_plan();
    let stage_b_digest = stage_b.digest();
    let mut authority = authority(1_448);
    authority.stage_b_plan_digest = stage_b_digest;
    let stage_c = plan_velocity_integral_response(
        authority,
        stage_b.execution_plan(stage_b_digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();
    let stage_c_digest = stage_c.digest();

    let candidate =
        CommissioningWorkflowPlan::new(7, StageCPlanShape::StageB, stage_b.maximum_workflow_ms, 0)
            .unwrap();
    let composite = CommissioningWorkflowPlan::new(
        8,
        StageCPlanShape::StageBThenStageC,
        stage_b.maximum_workflow_ms,
        stage_c.maximum_workflow_ms,
    )
    .unwrap();
    let resume = CommissioningWorkflowPlan::new(
        9,
        StageCPlanShape::StageCResume,
        0,
        stage_c.maximum_workflow_ms,
    )
    .unwrap();

    assert_eq!(candidate.maximum_workflow_ms, 298_632);
    assert_eq!(composite.maximum_workflow_ms, 389_520);
    assert_eq!(resume.maximum_workflow_ms, 90_888);
    assert_ne!(candidate.digest(), composite.digest());
    assert_ne!(composite.digest(), resume.digest());
    assert_eq!(StageCPlanShape::StageB as u8, 0);
    assert_eq!(StageCPlanShape::StageBThenStageC as u8, 1);
    assert_eq!(StageCPlanShape::StageCResume as u8, 2);
    assert_eq!(stage_b.digest(), stage_b_digest);
    assert_eq!(stage_c.digest(), stage_c_digest);
    assert_eq!(stage_c.execution_plan().plan_digest, stage_c_digest);
}

#[test]
fn invalid_geometry_and_missing_authority_reject_before_motion() {
    assert_eq!(velocity_i_tau_us(0, 25_000), None);
    assert_eq!(velocity_i_tau_us(5, 0), None);
    assert_eq!(
        build_integral_ladder(25_000, 512_000, 64_000, 10),
        Err(VelocityIntegralPlanError::InsufficientStructuralSpan)
    );
    assert_eq!(
        build_integral_ladder(1, 1_000_000, 2, u16::MAX),
        Err(VelocityIntegralPlanError::Arithmetic)
    );

    let stage_b = stage_b_plan();
    let digest = stage_b.digest();
    assert_eq!(
        plan_velocity_integral_response(
            authority(0),
            stage_b.execution_plan(digest),
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MissingAuthority)
    );
    assert_eq!(
        plan_velocity_integral_response(
            authority(1_448),
            super::sweep_plan::VelocitySweepExecutionPlan::EMPTY,
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MissingStageBPlan)
    );
    let mut mismatched = authority(1_448);
    mismatched.stage_b_plan_digest = digest.wrapping_add(1);
    assert_eq!(
        plan_velocity_integral_response(
            mismatched,
            stage_b.execution_plan(digest),
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MismatchedAuthority)
    );
}
