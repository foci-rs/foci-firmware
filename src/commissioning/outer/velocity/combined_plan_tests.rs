use super::combined_plan::{
    BREAKAWAY_STAGE_B_SCHEMA_REVISION, BREAKAWAY_STAGE_C_SCHEMA_REVISION, CombinedPlanError,
    CombinedRateInterval, CombinedTargetStatus, STAGE_B_PLAN_SCHEMA_REVISION,
    STAGE_C_PLAN_SCHEMA_REVISION, build_combined_integral_ladder, integral_deficit_reaches_target,
    integral_response_reaches_target, proportional_response_in_initial_band,
};
use super::integral_plan::{STAGE_C_EVIDENCE_SCHEMA_REVISION, build_integral_ladder};
use super::sweep_plan::{MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US};
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
    assert!(!proportional_response_in_initial_band(263, 4_000, reproduction_p724).unwrap());
    assert!(proportional_response_in_initial_band(263, 4_000, reproduction_p1024).unwrap());
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
fn signed_deficit_target_uses_the_conservative_directional_endpoints() {
    let below = [
        CombinedRateInterval::new(300_000, 400_000).unwrap(),
        CombinedRateInterval::new(-400_000, -300_000).unwrap(),
    ];
    assert!(!integral_deficit_reaches_target(264, 4_000, below).unwrap());

    let reached = [
        CombinedRateInterval::new(-100_000, 340_000).unwrap(),
        CombinedRateInterval::new(-340_000, 100_000).unwrap(),
    ];
    assert!(integral_deficit_reaches_target(264, 4_000, reached).unwrap());
}

#[test]
fn combined_geometry_appends_the_exact_cap_without_changing_history() {
    assert_eq!(STAGE_B_PLAN_SCHEMA_REVISION, 14);
    assert_eq!(STAGE_C_PLAN_SCHEMA_REVISION, 13);
    assert_eq!(STAGE_C_EVIDENCE_SCHEMA_REVISION, 17);
    // The breakaway campaign is additive: it must not renumber or otherwise
    // disturb any historical Stage-B or Stage-C schema revision.
    assert_eq!(BREAKAWAY_STAGE_B_SCHEMA_REVISION, 17);
    assert_eq!(BREAKAWAY_STAGE_C_SCHEMA_REVISION, 16);
    assert_ne!(
        BREAKAWAY_STAGE_B_SCHEMA_REVISION,
        STAGE_B_PLAN_SCHEMA_REVISION
    );
    assert_ne!(
        BREAKAWAY_STAGE_C_SCHEMA_REVISION,
        STAGE_C_PLAN_SCHEMA_REVISION
    );

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
