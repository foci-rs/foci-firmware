use super::integral_plan::{
    CommissioningWorkflowPlan, STAGE_C_EVIDENCE_SCHEMA_REVISION, STAGE_C_MAX_POSITIVE_RUNGS,
    STAGE_C_MIN_POSITIVE_I, STAGE_C_OBSERVATIONS_PER_RUNG, StageCAuthority, StageCPlanShape,
    VelocityIntegralPlanError, build_integral_ladder, confidence_family_size,
    integral_workflow_maximum_ms, plan_velocity_integral_response, recovery_quantization_exposed,
    response_horizon, velocity_i_tau_us,
};
use super::sweep_plan::{
    MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US, RequestedVelocitySource, SETTLE_ANALYSIS_BUDGET_US,
    VelocitySweepRequest, ZERO_SETTLE_US, plan_velocity_sweep,
};
use crate::commissioning::outer::velocity::reproduction::StageBDirectionalValidity;
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};
use crate::commissioning::types::ResponseHorizonClass;

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

fn authority() -> StageCAuthority {
    StageCAuthority {
        channel: 0,
        stage_b_plan_digest: 0x0123_4567_89ab_cdef,
        directional_memberships: [0x0007_0000, 0x0007_0000],
        stage_b_joint_membership: 0x0007_0000,
        joint_membership: 0x0007_0000,
        directional_validity: [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2],
        reduced_margin: false,
        directional_pooled_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
    }
}

fn stage_b_plan() -> super::sweep_plan::VelocitySweepPlan {
    plan_velocity_sweep(
        stage_b_request(),
        4_000,
        2_816,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap()
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
fn native_q4_12_geometry_derives_the_fixed_prefix_and_octaves() {
    assert_eq!(STAGE_C_EVIDENCE_SCHEMA_REVISION, 7);
    assert_eq!(STAGE_C_MIN_POSITIVE_I, 1);
    assert_eq!(velocity_i_tau_us(1, 25_000), Some(41_943_040));
    assert_eq!(velocity_i_tau_us(1_024, 25_000), Some(40_960));

    let ladder = build_integral_ladder(
        25_000,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )
    .unwrap();

    assert_eq!(ladder.i_start, 1);
    assert_eq!(ladder.i_top_cap, 1_310);
    assert_eq!(
        ladder.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );

    let doubled_pwm = build_integral_ladder(50_000, 512_000, 64_000, 32_767).unwrap();
    assert_eq!(doubled_pwm.i_start, 1);
    assert_eq!(doubled_pwm.i_top_cap, 655);
    assert_eq!(
        doubled_pwm.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512]
    );

    let shorter_stroke = build_integral_ladder(25_000, 256_000, 64_000, 32_767).unwrap();
    assert_eq!(shorter_stroke.i_start, 1);
    assert_eq!(shorter_stroke.i_top_cap, 1_310);
    assert_eq!(
        shorter_stroke.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );
}

#[test]
fn response_horizon_classifies_anchors_and_the_64_to_128_boundary() {
    assert_eq!(
        response_horizon(0, 25_000, 512_000),
        Some((0, ResponseHorizonClass::NotApplicable))
    );
    assert_eq!(
        response_horizon(64, 25_000, 512_000),
        Some((655_360, ResponseHorizonClass::SubHorizon))
    );
    assert_eq!(
        response_horizon(128, 25_000, 512_000),
        Some((327_680, ResponseHorizonClass::HorizonResolved))
    );
}

#[test]
fn recovery_quantization_exposure_is_exact_and_checked() {
    assert_eq!(
        recovery_quantization_exposed(724, 264, 4_000, 25_000, 2_534),
        Ok(false)
    );
    assert_eq!(
        recovery_quantization_exposed(1_024, 264, 4_000, 25_000, 2_534),
        Ok(true)
    );
    assert_eq!(recovery_quantization_exposed(256, 0, 1, 1, 60), Ok(false));
    assert_eq!(
        recovery_quantization_exposed(1, i32::MIN, 1, 1, 1),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
}

#[test]
fn integral_accumulation_ratio_tracks_pwm_and_stroke_geometry() {
    fn stroke_cycles(pwm_hz: u32, stroke_us: u32) -> u64 {
        u64::from(pwm_hz) * u64::from(stroke_us) / 1_000_000
    }

    let current_cycles = stroke_cycles(25_000, 512_000);
    assert_eq!(current_cycles, 12_800);
    assert_eq!(
        (
            current_cycles,
            crate::autotune::ADVANCED_PI_I_Q4_12_NUMERATOR
        ),
        (12_800, 1_048_576)
    );
    assert_eq!(
        [1_u16, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
            .map(|gain| current_cycles * u64::from(gain)),
        [
            12_800, 25_600, 38_400, 51_200, 102_400, 204_800, 409_600, 819_200, 1_638_400,
            3_276_800, 6_553_600, 13_107_200,
        ]
    );
    assert_eq!(stroke_cycles(50_000, 512_000), 25_600);
    assert_eq!(stroke_cycles(25_000, 256_000), 6_400);
}

#[test]
fn top_cap_is_not_appended_as_a_non_octave_rung() {
    let ladder = build_integral_ladder(25_000, 512_000, 64_000, 8_192).unwrap();
    assert_eq!(ladder.i_top_cap, 1_310);
    assert_eq!(ladder.rungs.last(), Some(&1_024));
    assert!(!ladder.rungs.contains(&1_310));
}

#[test]
fn q4_12_deployment_bridge_is_evidence_not_native_geometry() {
    let deployment_bridge = [16_u16, 32, 64, 128, 256, 512, 1_024];
    let current =
        build_integral_ladder(25_000, MOVING_STROKE_US, MIN_SETTLE_CAPTURE_US, 32_767).unwrap();

    assert_eq!(
        current.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );
    assert_ne!(current.rungs.as_slice(), deployment_bridge);
}

#[test]
fn confidence_family_counts_both_anchors() {
    assert_eq!(STAGE_C_MAX_POSITIVE_RUNGS, 30);
    assert_eq!(STAGE_C_OBSERVATIONS_PER_RUNG, 8);
    assert_eq!(confidence_family_size(12), Ok(56));
    assert_eq!(confidence_family_size(30), Ok(128));
    assert_eq!(
        confidence_family_size(31),
        Err(VelocityIntegralPlanError::ConfidenceFamilyTooLarge)
    );
}

#[test]
fn preflight_duration_reserves_the_fixed_cleanup_window() {
    assert_eq!(integral_workflow_maximum_ms(25_000, 3_000), Ok(182_512));
}

#[test]
fn stage_c_plan_freezes_geometry_and_conservative_duration() {
    let stage_b = stage_b_plan();
    let mut authority = authority();
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
    assert_eq!(plan.final_p, 512);
    assert_eq!(
        plan.positive_rungs(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );
    assert_eq!(plan.family_size, 56);
    assert_eq!(plan.total_rung_count, 14);
    assert_eq!(plan.expected_observations, 112);
    assert_eq!(plan.moving_stroke_us, 512_000);
    assert_eq!(plan.zero_settle_us, 500_000);
    assert_eq!(plan.analysis_budget_us, 236_000);
    assert_eq!(plan.recovery_origin_band_counts, 1_000);
    assert_eq!(plan.recovery_nominal_slot_us, 1_816_958);
    assert_eq!(plan.recovery_maximum_slot_us, 3_000_000);
    assert_eq!(plan.recovery_slot_count, 14);
    assert_eq!(plan.nominal_workflow_ms, 165_950);
    assert_eq!(plan.maximum_workflow_ms, 182_512);
    let base_us = u32::from(plan.total_rung_count)
        * STAGE_C_OBSERVATIONS_PER_RUNG
        * (MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US);
    let recovery_us = u32::from(plan.total_rung_count) * plan.recovery_nominal_slot_us;
    assert_eq!(
        plan.nominal_workflow_ms,
        (base_us + recovery_us + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US).div_ceil(1_000)
    );
}

#[test]
fn workflow_shape_does_not_change_either_stage_identity() {
    let stage_b = stage_b_plan();
    let stage_b_digest = stage_b.digest();
    let mut authority = authority();
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
    assert_eq!(composite.maximum_workflow_ms, 481_144);
    assert_eq!(resume.maximum_workflow_ms, 182_512);
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
        build_integral_ladder(25_000, 512_000, 64_000, 2),
        Err(VelocityIntegralPlanError::InsufficientStructuralSpan)
    );
    assert_eq!(
        build_integral_ladder(25_000, 512_000, 64_000, 160)
            .unwrap()
            .rungs
            .as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128]
    );
    assert_eq!(
        build_integral_ladder(1, 1_000_000, 2, u16::MAX),
        Err(VelocityIntegralPlanError::Arithmetic)
    );

    let stage_b = stage_b_plan();
    let digest = stage_b.digest();
    let mut missing = authority();
    missing.stage_b_plan_digest = digest;
    missing.stage_b_joint_membership = 0;
    missing.joint_membership = 0;
    assert_eq!(
        plan_velocity_integral_response(
            missing,
            stage_b.execution_plan(digest),
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MissingAuthority)
    );
    assert_eq!(
        plan_velocity_integral_response(
            authority(),
            super::sweep_plan::VelocitySweepExecutionPlan::EMPTY,
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MissingStageBPlan)
    );
    let mut mismatched = authority();
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

#[test]
fn stage_c_accepts_local_authority_without_model() {
    let stage_b = stage_b_plan();
    let digest = stage_b.digest();
    let mut local = authority();
    local.stage_b_plan_digest = digest;
    local.stage_b_joint_membership = 0x0003_0000;
    local.joint_membership = 0x0003_0000;
    local.directional_memberships = [0x0003_0000, 0];
    local.directional_pooled_intervals = [[0, 0]; 2];
    local.directional_validity = [
        StageBDirectionalValidity::Fragment,
        StageBDirectionalValidity::NoRegion,
    ];
    local.reduced_margin = true;

    let plan = plan_velocity_integral_response(
        local,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("joint operability authorizes Stage C without a cross-P model");

    assert!(!plan.authority.model_interval_available(0));
    assert!(!plan.authority.model_interval_available(1));
    assert!(plan.authority.reduced_margin);
    assert_eq!(
        plan.authority.directional_validity,
        [
            StageBDirectionalValidity::Fragment,
            StageBDirectionalValidity::NoRegion,
        ]
    );
}

#[test]
fn stage_c_model_interval_availability_is_typed_and_hashed() {
    let availability = [
        (StageBDirectionalValidity::NoRegion, false),
        (StageBDirectionalValidity::Fragment, false),
        (StageBDirectionalValidity::RegionNotReproduced, false),
        (
            StageBDirectionalValidity::ReproducedRegionCoversFinalP,
            true,
        ),
        (StageBDirectionalValidity::ReproducedRegionTransferred, true),
    ];
    let mut value = authority();
    for (validity, expected) in availability {
        value.directional_validity[0] = validity;
        assert_eq!(value.model_interval_available(0), expected);
    }
    assert!(!value.model_interval_available(2));

    let stage_b = stage_b_plan();
    let digest = stage_b.digest();
    let mut baseline = authority();
    baseline.stage_b_plan_digest = digest;
    let baseline = plan_velocity_integral_response(
        baseline,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();

    let mut transferred = authority();
    transferred.stage_b_plan_digest = digest;
    transferred.directional_memberships[0] = 0x001c_0000;
    let transferred = plan_velocity_integral_response(
        transferred,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();
    assert_ne!(baseline.digest(), transferred.digest());

    let mut reduced = authority();
    reduced.stage_b_plan_digest = digest;
    reduced.stage_b_joint_membership = 0x0003_0000;
    reduced.joint_membership = 0x0003_0000;
    let reduced = plan_velocity_integral_response(
        reduced,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();
    assert_ne!(baseline.digest(), reduced.digest());
}
