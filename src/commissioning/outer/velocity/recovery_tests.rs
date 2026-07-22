use super::recovery::{
    commanded_side_lower_rate_q, plan_rung_origin_recovery, recovery_return_distance_counts,
    recovery_runtime_us,
};
use super::sweep_plan::RequestedVelocitySource;
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::commissioning::outer::OuterMotionBudget;

const RATE_Q: u128 = 1_000;

fn live_budget() -> OuterMotionBudget {
    OuterMotionBudget {
        requested_velocity_mrev_s: 4_394,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 28_100,
        positive_position_headroom_mrev: 28_100,
        max_duration_ms: 3_000,
    }
}

#[test]
fn live_geometry_derives_band_and_slots_from_the_budget() {
    let budget = live_budget();
    let primitive_cap_us = u32::from(budget.max_duration_ms) * 1_000;

    let plan = plan_rung_origin_recovery(budget, 4_000, 4_394).unwrap();

    assert_eq!(plan.origin_band_counts, 1_000);
    assert_eq!(plan.nominal_slot_us, 1_816_958);
    assert_eq!(plan.maximum_slot_us, primitive_cap_us);
}

#[test]
fn return_distance_stops_at_the_inclusive_origin_band() {
    assert_eq!(recovery_return_distance_counts(8_458, 1_000), Some(7_458));
    assert_eq!(recovery_return_distance_counts(-8_458, 1_000), Some(7_458));
    assert_eq!(recovery_return_distance_counts(1_000, 1_000), Some(0));
    assert_eq!(recovery_return_distance_counts(-1_000, 1_000), Some(0));
    assert_eq!(recovery_return_distance_counts(i32::MIN, 1_000), None);
}

#[test]
fn direction_authority_uses_the_smallest_commanded_side_magnitude() {
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Forward, 12_000, 18_000),
        Some(12_000)
    );
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Reverse, -18_000, -11_000),
        Some(11_000)
    );
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Forward, 0, 18_000),
        None
    );
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Reverse, -18_000, 0),
        None
    );
}

#[test]
fn runtime_uses_outward_rate_rounding_and_fixed_allowances() {
    let runtime = recovery_runtime_us(7_458, 14_000_000, 3_000_000).unwrap();
    let measured = (7_458_u128 * 1_000_000 * RATE_Q).div_ceil(14_000_000);

    assert_eq!(runtime, u32::try_from(measured).unwrap() + 1_248_000);
    assert!(recovery_runtime_us(7_458, 3_150_000, 3_000_000).is_err());
    assert!(recovery_runtime_us(1, 0, 3_000_000).is_err());
}

#[test]
fn proportional_decay_stalls_outside_the_declared_band() {
    let disturbance_rate_q = 3_150_000_u128;
    let proportional_horizon_us = 512_000_u128;
    let stalled_offset =
        (disturbance_rate_q * proportional_horizon_us).div_ceil(1_000_000 * RATE_Q);

    assert_eq!(stalled_offset, 1_613);
    assert!(stalled_offset > 1_000);
}

#[test]
fn invalid_geometry_rejects_without_saturation() {
    let mut budget = live_budget();
    budget.settle_travel_reserve_mrev = budget.max_stroke_travel_mrev;
    assert!(plan_rung_origin_recovery(budget, 4_000, 4_394).is_err());

    let budget = live_budget();
    assert!(plan_rung_origin_recovery(budget, 0, 4_394).is_err());
    assert!(plan_rung_origin_recovery(budget, 4_000, 0).is_err());

    let mut too_short = live_budget();
    too_short.max_duration_ms = 1_816;
    assert!(plan_rung_origin_recovery(too_short, 4_000, 4_394).is_err());
}
