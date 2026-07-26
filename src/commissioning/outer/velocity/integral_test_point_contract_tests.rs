use super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use super::integral_plan::{
    StageCAuthority, VelocityIntegralPlanError, plan_velocity_integral_response,
    transition_capable_stage_c_gain,
};
use super::reproduction::StageBDirectionalValidity;
use super::sweep_plan::{RequestedVelocitySource, VelocitySweepRequest, plan_velocity_sweep};
use crate::commissioning::outer::OuterMotionBudget;

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

fn budget() -> OuterMotionBudget {
    let request = request();
    OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
    }
}

fn authority(plan_digest: u64, source: u32) -> StageCAuthority {
    StageCAuthority {
        channel: 0,
        stage_b_plan_digest: plan_digest,
        directional_memberships: [0x001c_0000; 2],
        stage_b_joint_membership: source,
        joint_membership: source,
        directional_validity: [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2],
        reduced_margin: false,
        directional_pooled_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
    }
}

#[test]
fn stage_c_test_point_selection_uses_the_transition_capable_lower_prefix() {
    let stage_b = plan_velocity_sweep(
        request(),
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    assert_eq!(stage_b.target_velocity_rpm.raw(), 264);
    assert_eq!(stage_b.usable_torque_limit, 2_534);
    assert_eq!(stage_b.gains[18].raw(), 724);
    assert_eq!(stage_b.gains[19].raw(), 1_024);
    assert_eq!(stage_b.gains[20].raw(), 1_448);

    let digest = stage_b.digest();
    let mut authority = authority(digest, 0x001c_0000);
    authority.directional_memberships[1] = 0x0038_0000;
    let plan = plan_velocity_integral_response(
        authority,
        stage_b.execution_plan(digest),
        budget(),
        25_000,
        4_000,
    )
    .unwrap();

    assert_eq!(plan.final_p, 724);
    assert_eq!(plan.schema_revision, 7);
    assert_eq!(plan.authority.stage_b_joint_membership, 0x001c_0000);
    assert_eq!(plan.authority.joint_membership, 0x0004_0000);
    assert!(plan.authority.reduced_margin);
    assert!(plan.probe_constrained_test_point);
    assert!(plan.execution_plan().probe_constrained_test_point);
    assert_eq!(
        plan.authority.directional_validity,
        [
            StageBDirectionalValidity::ReproducedRegionCoversFinalP,
            StageBDirectionalValidity::ReproducedRegionTransferred,
        ]
    );
}

#[test]
fn stage_c_transition_capable_predicate_is_exact_at_quantum_and_clamp_boundaries() {
    for (inputs, expected) in [
        ((16_384, 264, 100, 1_000, 512_000, 55_377), false),
        ((16_384, 264, 100, 1_000, 512_000, 55_378), true),
        ((835, 374, 4_000, 25_000, 512_000, 2_534), true),
        ((835, 375, 4_000, 25_000, 512_000, 2_534), false),
        ((834, 376, 4_000, 25_000, 512_000, 2_534), false),
        ((724, 264, 4_000, 25_000, 512_001, 2_534), true),
        ((1_024, 132, 4_000, 25_000, 512_000, 2_534), true),
        ((1_448, 132, 4_000, 25_000, 512_000, 2_534), false),
    ] {
        assert_eq!(
            transition_capable_stage_c_gain(
                inputs.0, inputs.1, inputs.2, inputs.3, inputs.4, inputs.5,
            ),
            Ok(expected)
        );
    }
    for invalid in [
        transition_capable_stage_c_gain(0, 264, 4_000, 25_000, 512_000, 2_534),
        transition_capable_stage_c_gain(724, 264, 0, 25_000, 512_000, 2_534),
        transition_capable_stage_c_gain(724, 264, 4_000, 0, 512_000, 2_534),
        transition_capable_stage_c_gain(724, 264, 4_000, 25_000, 0, 2_534),
        transition_capable_stage_c_gain(724, 264, 4_000, 25_000, 512_000, 0),
    ] {
        assert_eq!(invalid, Err(VelocityIntegralPlanError::InvalidRequest));
    }
    assert_eq!(
        transition_capable_stage_c_gain(1, i32::MIN, 1, 1, 1, 1),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
    assert_eq!(
        transition_capable_stage_c_gain(u16::MAX, i32::MAX, u32::MAX, u32::MAX, u32::MAX, u16::MAX,),
        Ok(false)
    );
}

#[test]
fn stage_c_keeps_an_unchanged_lower_component_without_probe_constraint() {
    let stage_b = plan_velocity_sweep(
        request(),
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    assert_eq!(stage_b.gains[15].raw(), 256);
    assert_eq!(stage_b.gains[16].raw(), 362);
    let digest = stage_b.digest();
    let mut authority = authority(digest, 0x0001_8000);
    authority.directional_memberships = [0x0007_0000, 0x0003_0000];
    authority.directional_validity = [
        StageBDirectionalValidity::RegionNotReproduced,
        StageBDirectionalValidity::Fragment,
    ];
    authority.directional_pooled_intervals = [[0, 0]; 2];

    let plan = plan_velocity_integral_response(
        authority,
        stage_b.execution_plan(digest),
        budget(),
        25_000,
        4_000,
    )
    .unwrap();

    assert_eq!(plan.authority.stage_b_joint_membership, 0x0001_8000);
    assert_eq!(plan.authority.joint_membership, 0x0001_8000);
    assert_eq!(plan.final_p, 256);
    assert!(plan.authority.reduced_margin);
    assert!(!plan.probe_constrained_test_point);
    assert_eq!(
        plan.authority.directional_validity,
        [
            StageBDirectionalValidity::RegionNotReproduced,
            StageBDirectionalValidity::Fragment,
        ]
    );
}

#[test]
fn stage_c_digest_retains_source_filtered_selection_and_constraint() {
    let stage_b = plan_velocity_sweep(
        request(),
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let digest = stage_b.digest();
    let wide = plan_velocity_integral_response(
        authority(digest, 0x001c_0000),
        stage_b.execution_plan(digest),
        budget(),
        25_000,
        4_000,
    )
    .unwrap();
    let narrow = plan_velocity_integral_response(
        authority(digest, 0x000c_0000),
        stage_b.execution_plan(digest),
        budget(),
        25_000,
        4_000,
    )
    .unwrap();

    assert_eq!(wide.authority.joint_membership, 0x0004_0000);
    assert_eq!(narrow.authority.joint_membership, 0x0004_0000);
    assert_ne!(wide.digest(), narrow.digest());

    let baseline = wide.digest();
    let mut changed = wide;
    changed.authority.joint_membership = 0x000c_0000;
    assert_ne!(baseline, changed.digest());
    changed = wide;
    changed.final_p = 1_024;
    assert_ne!(baseline, changed.digest());
    changed = wide;
    changed.probe_constrained_test_point = false;
    assert_ne!(baseline, changed.digest());
}

#[test]
fn stage_c_reports_a_digested_failure_when_filtering_removes_every_rung() {
    let stage_b = plan_velocity_sweep(
        request(),
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let digest = stage_b.digest();
    let failure = plan_velocity_integral_response(
        authority(digest, 0x0018_0000),
        stage_b.execution_plan(digest),
        budget(),
        25_000,
        4_000,
    )
    .unwrap_err();

    let VelocityIntegralPlanError::NoTransitionCapableOperatingPoint(failure) = failure else {
        panic!("empty transition-capable membership must retain its exact failure identity");
    };
    assert_ne!(failure.plan_digest, 0);
    assert!(failure.probe_constrained_test_point);
}
