//! Dispatch-side duration envelope for the two-bound position-P sweep.

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::position_tune::selection::next_rung;
use crate::commissioning::outer::position_tune::{
    RAMPED_STROKE_REPEATS, RESTORE_WAIT_US, STANDSTILL_CAPTURE_COUNT, STANDSTILL_HOLD_US,
    STROKE_LEAD_US, stroke_request,
};
use crate::commissioning::stimulus::trapezoid::plan_stroke;
use foci_tmc4671::pid::PidPGain;

const MICROS_PER_MILLI: u32 = 1_000;

const NOMINAL_RUNG_COUNT: u32 = 3;

fn worst_case_rung_count() -> u32 {
    let mut rungs = 1_u32;
    let mut current = PidPGain::new(1).expect("raw 1 is a representable P gain");
    while let Some(next) = next_rung(current) {
        current = next;
        rungs = rungs.saturating_add(1);
    }
    rungs
}

fn per_rung_duration_ms(
    budget: &OuterMotionBudget,
    steps_per_rev: u32,
    step_timer_hz: u32,
) -> Option<u32> {
    let homing_stroke_us = plan_stroke(&stroke_request(
        budget.homing_speed_mrev_s,
        budget,
        steps_per_rev,
        step_timer_hz,
    ))
    .ok()?
    .stroke_us();
    let motion_stroke_us = plan_stroke(&stroke_request(
        budget.machine_velocity_ceiling_mrev_s,
        budget,
        steps_per_rev,
        step_timer_hz,
    ))
    .ok()?
    .stroke_us();

    let repeats = u64::from(u32::from(RAMPED_STROKE_REPEATS));
    let strokes_us = u64::from(homing_stroke_us)
        .saturating_mul(repeats)
        .saturating_add(u64::from(motion_stroke_us).saturating_mul(repeats));
    let leads_us = repeats
        .saturating_mul(2)
        .saturating_mul(u64::from(STROKE_LEAD_US));
    let standstill_us = u64::from(u32::from(STANDSTILL_CAPTURE_COUNT))
        .saturating_mul(u64::from(STANDSTILL_HOLD_US));
    let total_us = strokes_us
        .saturating_add(leads_us)
        .saturating_add(u64::from(RESTORE_WAIT_US))
        .saturating_add(standstill_us);
    u32::try_from(total_us.div_ceil(u64::from(MICROS_PER_MILLI))).ok()
}

fn scale_rungs(rungs: u32, per_rung_ms: u32) -> u32 {
    u32::try_from(u64::from(rungs).saturating_mul(u64::from(per_rung_ms))).unwrap_or(u32::MAX)
}

pub(crate) fn position_tune_worst_case_duration_ms(
    budget: &OuterMotionBudget,
    steps_per_rev: u32,
    step_timer_hz: u32,
) -> u32 {
    match per_rung_duration_ms(budget, steps_per_rev, step_timer_hz) {
        Some(per_rung_ms) => scale_rungs(worst_case_rung_count(), per_rung_ms),
        None => u32::MAX,
    }
}

pub(crate) fn position_tune_nominal_duration_ms(
    budget: &OuterMotionBudget,
    steps_per_rev: u32,
    step_timer_hz: u32,
) -> u32 {
    match per_rung_duration_ms(budget, steps_per_rev, step_timer_hz) {
        Some(per_rung_ms) => scale_rungs(NOMINAL_RUNG_COUNT, per_rung_ms),
        None => u32::MAX,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_STEPS_PER_REV: u32 = 65_536 / 8;
    const TEST_STEP_TIMER_HZ: u32 = 260_000_000;

    #[test]
    fn worst_case_rung_count_matches_the_representable_half_octave_ladder() {
        assert_eq!(worst_case_rung_count(), 31);
    }

    #[test]
    fn per_rung_duration_covers_both_stroke_speeds_and_every_standstill_capture() {
        let budget = OuterMotionBudget::firmware_caps();
        let homing_us = plan_stroke(&stroke_request(
            budget.homing_speed_mrev_s,
            &budget,
            TEST_STEPS_PER_REV,
            TEST_STEP_TIMER_HZ,
        ))
        .expect("firmware-cap homing speed plans a stroke")
        .stroke_us();
        let motion_us = plan_stroke(&stroke_request(
            budget.machine_velocity_ceiling_mrev_s,
            &budget,
            TEST_STEPS_PER_REV,
            TEST_STEP_TIMER_HZ,
        ))
        .expect("firmware-cap motion speed plans a stroke")
        .stroke_us();
        let repeats = u64::from(u32::from(RAMPED_STROKE_REPEATS));
        let expected_us = u64::from(homing_us) * repeats
            + u64::from(motion_us) * repeats
            + repeats * 2 * u64::from(STROKE_LEAD_US)
            + u64::from(RESTORE_WAIT_US)
            + u64::from(u32::from(STANDSTILL_CAPTURE_COUNT)) * u64::from(STANDSTILL_HOLD_US);
        let expected_ms =
            u32::try_from(expected_us.div_ceil(1_000)).expect("fits a u32 millisecond count");
        assert_ne!(
            expected_us % 1_000,
            0,
            "fixture must have a nonzero microsecond remainder to prove ceiling rounding"
        );

        assert_eq!(
            per_rung_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ),
            Some(expected_ms)
        );
    }

    #[test]
    fn worst_case_duration_is_the_full_ladder_at_the_per_rung_cost() {
        let budget = OuterMotionBudget::firmware_caps();
        let per_rung_ms = per_rung_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ)
            .expect("firmware caps plan both strokes");

        assert_eq!(
            position_tune_worst_case_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ),
            31 * per_rung_ms
        );
    }

    #[test]
    fn nominal_duration_is_three_rungs_at_the_per_rung_cost() {
        let budget = OuterMotionBudget::firmware_caps();
        let per_rung_ms = per_rung_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ)
            .expect("firmware caps plan both strokes");

        assert_eq!(
            position_tune_nominal_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ),
            3 * per_rung_ms
        );
    }

    #[test]
    fn nominal_duration_fits_inside_the_worst_case_envelope() {
        let budget = OuterMotionBudget::firmware_caps();
        let nominal =
            position_tune_nominal_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ);
        let worst =
            position_tune_worst_case_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ);

        assert!(nominal <= worst);
        assert!(nominal > 0);
        assert!(worst > 0);
    }

    #[test]
    fn an_unplannable_budget_yields_u32_max_for_the_worst_case() {
        let budget = OuterMotionBudget::test_budget(2_000, 6_000, 3_000, 0, 200_000);

        assert_eq!(
            position_tune_worst_case_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ),
            u32::MAX
        );
    }
}
