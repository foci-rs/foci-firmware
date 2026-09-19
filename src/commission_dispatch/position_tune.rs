//! Dispatch-side duration envelope for the two-bound position-P sweep.
//!
//! The host arms a fixed 5 s plan-wait timeout until a shape-specific
//! workflow plan replaces it with that shape's own worst case. A position-P
//! sweep's per-rung cost is data-dependent -- it comes from the trapezoid
//! planner's stroke durations, which follow the runtime motion budget rather
//! than a fixed capture window -- over a data-dependent rung count, so the
//! dispatch discloses this envelope before any motion, exactly as the
//! reversal-robustness gate does for its K-cycle probe.

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::position_tune::selection::next_rung;
use crate::commissioning::outer::position_tune::{
    RAMPED_STROKE_REPEATS, RESTORE_WAIT_US, STANDSTILL_CAPTURE_COUNT, STANDSTILL_HOLD_US,
    stroke_request,
};
use crate::commissioning::stimulus::trapezoid::plan_stroke;
use foci_tmc4671::pid::PidPGain;

/// Microseconds per millisecond, for the envelope's unit conversion.
const MICROS_PER_MILLI: u32 = 1_000;

/// Rungs a clean run measures before a stop condition trips.
///
/// A clean-run estimate, not a bound -- a well-conditioned axis trips the
/// ceiling within a rung or two of the identified floor. The host arms its
/// run timeout off [`position_tune_worst_case_duration_ms`], never off
/// this.
const NOMINAL_RUNG_COUNT: u32 = 3;

/// Rungs the sweep can measure before `next_rung` runs out of representable
/// gains.
///
/// Counts the floor rung plus every half-octave step reachable from the
/// smallest representable gain, so the envelope tracks `next_rung`'s own step
/// ratio and `PidPGain::MAX` rather than a hardcoded guess. Raw 1 is the
/// smallest gain any floor can produce: `compute_position_floor` rounds up,
/// so it never returns a smaller nonzero value.
fn worst_case_rung_count() -> u32 {
    let mut rungs = 1_u32;
    let mut current = PidPGain::new(1).expect("raw 1 is a representable P gain");
    while let Some(next) = next_rung(current) {
        current = next;
        rungs = rungs.saturating_add(1);
    }
    rungs
}

/// Wall-clock cost of one rung, in milliseconds: its three homing-speed
/// stroke repeats, its three motion-speed stroke repeats, the feedforward
/// restore wait, and its standstill-dither captures.
///
/// `None` when either leg's stroke cannot be planned from `budget` -- an
/// unplannable budget has no per-rung cost to report, so the caller falls
/// back to `u32::MAX` for the worst case rather than understating the host's
/// timeout; the phase itself fails fast on `ConfigurationInvalid` the moment
/// it tries to plan the same stroke.
///
/// Every intermediate is carried in `u64` before the final millisecond
/// narrowing: `StrokeProfile::stroke_us` alone can approach `u32::MAX`, and
/// this sums six such strokes plus the restore wait and standstill holds, so
/// the running total can exceed `u32::MAX` well before the rung multiplier
/// in `position_tune_worst_case_duration_ms` is even applied.
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
    let standstill_us = u64::from(u32::from(STANDSTILL_CAPTURE_COUNT))
        .saturating_mul(u64::from(STANDSTILL_HOLD_US));
    let total_us = strokes_us
        .saturating_add(u64::from(RESTORE_WAIT_US))
        .saturating_add(standstill_us);
    u32::try_from(total_us / u64::from(MICROS_PER_MILLI)).ok()
}

/// `rungs * per_rung_ms`, widened through `u64` and saturated back to `u32`
/// so neither the per-rung cost nor the rung count alone can wrap the
/// result.
fn scale_rungs(rungs: u32, per_rung_ms: u32) -> u32 {
    u32::try_from(u64::from(rungs).saturating_mul(u64::from(per_rung_ms))).unwrap_or(u32::MAX)
}

/// Worst-case wall-clock duration of a full position-P sweep, in
/// milliseconds: the host's `maximum_workflow_ms`.
///
/// Bounds the sweep at its structural maximum -- every rung `next_rung` can
/// reach from the smallest representable gain -- times the per-rung stroke
/// cost planned from `budget`. Neither a ceiling trip nor `PidPGain::MAX`
/// exhaustion is guaranteed before the ladder runs out, so this is the only
/// duration the host can safely arm against. An unplannable `budget` reports
/// `u32::MAX`: the host arms a long timeout and the phase fails fast on
/// `ConfigurationInvalid` instead of a false understatement.
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

/// Clean-run duration estimate of a position-P sweep, in milliseconds: the
/// host's `nominal_workflow_ms`.
///
/// A ceiling trip or `PidPGain::MAX` exhaustion stops the walk within
/// [`NOMINAL_RUNG_COUNT`] rungs of the floor, so no further rung is measured.
/// This is only a disclosure hint; the host arms its run timeout off
/// [`position_tune_worst_case_duration_ms`]. Mirrors that function's
/// unplannable-budget fallback so `nominal <= worst_case` holds in every
/// case, including this one.
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

    /// 65_536 / 8: the same physical step width `position_tune/mod.rs`'s own
    /// fixture backend configures.
    const TEST_STEPS_PER_REV: u32 = 65_536 / 8;
    /// The `MockBackend` default step-timer frequency.
    const TEST_STEP_TIMER_HZ: u32 = 260_000_000;

    #[test]
    fn worst_case_rung_count_matches_the_representable_half_octave_ladder() {
        // 1 -> 2 -> 3 -> 4 -> 5 -> 7 -> ... -> 29_700, the last rung below
        // `PidPGain::MAX` (0x7fff): 30 half-octave steps, so 31 measured
        // rungs counting the floor itself.
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
            + u64::from(RESTORE_WAIT_US)
            + u64::from(u32::from(STANDSTILL_CAPTURE_COUNT)) * u64::from(STANDSTILL_HOLD_US);
        let expected_ms = u32::try_from(expected_us / 1_000).expect("fits a u32 millisecond count");

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
        // A zero homing speed can never plan a stroke leg, regardless of
        // steps-per-rev or step-timer frequency.
        let budget = OuterMotionBudget::test_budget(2_000, 6_000, 3_000, 0, 200_000);

        assert_eq!(
            position_tune_worst_case_duration_ms(&budget, TEST_STEPS_PER_REV, TEST_STEP_TIMER_HZ),
            u32::MAX
        );
    }
}
