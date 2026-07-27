use super::combined_plan::{
    COMBINED_STAGE_B_SCHEMA_REVISION, COMBINED_STAGE_C_SCHEMA_REVISION, CombinedPlanError,
    CombinedRateInterval, CombinedTargetStatus, build_combined_integral_ladder,
    combined_workflow_geometry, integral_response_reaches_target,
    proportional_response_in_initial_band,
};
use super::integral_plan::{
    CommissioningWorkflowPlan, STAGE_C_EVIDENCE_SCHEMA_REVISION, StageCPlanShape,
    build_integral_ladder,
};
use super::sweep_plan::{
    MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US, RequestedVelocitySource, VelocitySweepRequest,
    plan_velocity_sweep,
};
use crate::commissioning::outer::OUTER_MAX_VELOCITY_I;

#[test]
fn corrected_y_intervals_select_p1024_without_model_membership() {
    let candidate_p724 = [
        CombinedRateInterval::new(11_562_496, 11_719_168).unwrap(),
        CombinedRateInterval::new(-11_218_944, -11_187_200).unwrap(),
    ];
    let candidate_p1024 = [
        CombinedRateInterval::new(13_359_104, 13_492_224).unwrap(),
        CombinedRateInterval::new(-12_968_960, -12_921_856).unwrap(),
    ];
    let reproduction_p724 = [
        CombinedRateInterval::new(11_593_728, 11_750_400).unwrap(),
        CombinedRateInterval::new(-11_359_744, -11_328_000).unwrap(),
    ];
    let reproduction_p1024 = [
        CombinedRateInterval::new(13_390_336, 13_539_328).unwrap(),
        CombinedRateInterval::new(-13_000_192, -12_937_216).unwrap(),
    ];

    assert!(!proportional_response_in_initial_band(264, 4_000, candidate_p724).unwrap());
    assert!(proportional_response_in_initial_band(264, 4_000, candidate_p1024).unwrap());
    assert!(!proportional_response_in_initial_band(264, 4_000, reproduction_p724).unwrap());
    assert!(proportional_response_in_initial_band(264, 4_000, reproduction_p1024).unwrap());
}

#[test]
fn response_boundaries_are_closed_signed_and_checked() {
    let target_low = 12_320_000;
    let target_high = 14_080_000;
    let touching = [
        CombinedRateInterval::new(target_low, target_high).unwrap(),
        CombinedRateInterval::new(-target_high, -target_low).unwrap(),
    ];
    assert!(proportional_response_in_initial_band(264, 4_000, touching).unwrap());

    let below = [
        CombinedRateInterval::new(target_low - 1, target_high).unwrap(),
        touching[1],
    ];
    assert!(!proportional_response_in_initial_band(264, 4_000, below).unwrap());

    let wrong_reverse_sign = [
        touching[0],
        CombinedRateInterval::new(1, target_low).unwrap(),
    ];
    assert_eq!(
        proportional_response_in_initial_band(264, 4_000, wrong_reverse_sign),
        Err(CombinedPlanError::InvalidInterval)
    );
    assert_eq!(
        CombinedRateInterval::new(1, 0),
        Err(CombinedPlanError::InvalidInterval)
    );
}

#[test]
fn integral_target_uses_the_lower_response_endpoint_and_preserves_overshoot() {
    let exact_target = [
        CombinedRateInterval::new(17_248_000, 17_700_000).unwrap(),
        CombinedRateInterval::new(-18_000_000, -17_248_000).unwrap(),
    ];
    assert!(integral_response_reaches_target(264, 4_000, exact_target).unwrap());

    let below = [
        CombinedRateInterval::new(17_247_999, 17_700_000).unwrap(),
        exact_target[1],
    ];
    assert!(!integral_response_reaches_target(264, 4_000, below).unwrap());

    let overflow = [
        exact_target[0],
        CombinedRateInterval::new(i64::MIN, -17_248_000).unwrap(),
    ];
    assert_eq!(
        integral_response_reaches_target(264, 4_000, overflow),
        Err(CombinedPlanError::Arithmetic)
    );
}

#[test]
fn combined_geometry_appends_the_exact_cap_without_changing_history() {
    assert_eq!(COMBINED_STAGE_B_SCHEMA_REVISION, 9);
    assert_eq!(COMBINED_STAGE_C_SCHEMA_REVISION, 8);
    assert_eq!(STAGE_C_EVIDENCE_SCHEMA_REVISION, 7);
    assert_eq!(StageCPlanShape::CombinedPThenI as u8, 3);

    let historical = build_integral_ladder(
        25_000,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )
    .unwrap();
    assert_eq!(historical.rungs.last(), Some(&1_024));

    let combined = build_combined_integral_ladder(
        25_000,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )
    .unwrap();
    assert_eq!(combined.i_top_cap, 1_310);
    assert_eq!(
        combined.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024, 1_310]
    );
}

#[test]
fn combined_workflow_uses_exact_unrounded_phase_durations() {
    let stage_b = plan_velocity_sweep(
        VelocitySweepRequest {
            requested_velocity_mrev_s: 6_666,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 2_500,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 7_025,
            positive_position_headroom_mrev: 7_025,
            max_duration_ms: 3_000,
        },
        4_000,
        2_816,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let geometry = combined_workflow_geometry(stage_b, 25_000).unwrap();

    assert_eq!(geometry.positive_i_rungs, 13);
    assert_eq!(geometry.total_i_rungs, 15);
    assert_eq!(geometry.i_observations, 120);
    assert_eq!(geometry.i_family_size, 60);
    assert_eq!(geometry.i_recovery_slots, 15);
    assert_eq!(geometry.i_nominal_ms, 177_751);
    assert_eq!(geometry.i_maximum_ms, 195_496);
    assert_eq!(geometry.combined_nominal_ms, 449_173);
    assert_eq!(geometry.combined_maximum_ms, 494_128);

    let workflow = CommissioningWorkflowPlan::new_combined(
        17,
        geometry.combined_nominal_ms,
        geometry.combined_maximum_ms,
    )
    .unwrap();
    assert_eq!(workflow.shape, StageCPlanShape::CombinedPThenI);
    assert_eq!(workflow.nominal_workflow_ms, 449_173);
    assert_eq!(workflow.maximum_workflow_ms, 494_128);
}

#[test]
fn combined_target_status_keeps_distinct_reportable_termini() {
    assert_ne!(
        CombinedTargetStatus::TargetReached,
        CombinedTargetStatus::TargetReachedSparse
    );
    assert_ne!(
        CombinedTargetStatus::CurrentHeadroom,
        CombinedTargetStatus::TargetNotReachedAtCap
    );
}
