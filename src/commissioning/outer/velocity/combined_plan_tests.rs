use super::combined_plan::{
    BREAKAWAY_INTEGRAL_SCHEMA_REVISION, BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION,
    CombinedRateInterval, CombinedTargetStatus, INTEGRAL_PLAN_SCHEMA_REVISION,
    PROPORTIONAL_PLAN_SCHEMA_REVISION, build_combined_integral_ladder,
    integral_deficit_reaches_target,
};
use super::integral_plan::{INTEGRAL_EVIDENCE_SCHEMA_REVISION, build_integral_ladder};
use super::sweep_plan::{MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US};
use crate::commissioning::outer::OUTER_MAX_VELOCITY_I;

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
    assert_eq!(PROPORTIONAL_PLAN_SCHEMA_REVISION, 14);
    assert_eq!(INTEGRAL_PLAN_SCHEMA_REVISION, 13);
    assert_eq!(INTEGRAL_EVIDENCE_SCHEMA_REVISION, 17);
    // The breakaway campaign is additive: it must not renumber or otherwise
    // disturb any historical Proportional or Integral schema revision.
    assert_eq!(BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION, 17);
    assert_eq!(BREAKAWAY_INTEGRAL_SCHEMA_REVISION, 18);
    assert_ne!(
        BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION,
        PROPORTIONAL_PLAN_SCHEMA_REVISION
    );
    assert_ne!(
        BREAKAWAY_INTEGRAL_SCHEMA_REVISION,
        INTEGRAL_PLAN_SCHEMA_REVISION
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
