//! Dispatch-side duration envelope for the two-bound position-P sweep.
//!
//! The host arms a fixed 5 s plan-wait timeout until a shape-specific
//! workflow plan replaces it with that shape's own worst case. A position-P
//! sweep spends about 2 seconds per half-octave rung (three ramped-stroke
//! repeats plus five standstill holds) over a data-dependent rung count, so
//! the dispatch discloses this envelope before any motion, exactly as the
//! reversal-robustness gate does for its K-cycle probe.

use crate::commissioning::outer::position_tune::selection::next_rung;
use crate::commissioning::outer::position_tune::{
    P_ONLY_CAPTURE_US, RAMPED_STROKE_CAPTURE_US, RAMPED_STROKE_REPEATS, STANDSTILL_CAPTURE_COUNT,
    STANDSTILL_HOLD_US,
};
use foci_tmc4671::pid::PidPGain;

/// Microseconds per millisecond, for the envelope's unit conversion.
const MICROS_PER_MILLI: u32 = 1_000;

/// Rungs a clean run measures before a stop condition trips: the floor rung
/// plus two more.
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

/// Wall-clock cost of one rung: its three ramped-stroke repeats plus its
/// standstill-dither captures.
fn per_rung_duration_ms() -> u32 {
    let ramped_stroke_us =
        RAMPED_STROKE_CAPTURE_US.saturating_mul(u32::from(RAMPED_STROKE_REPEATS));
    let standstill_us = u32::from(STANDSTILL_CAPTURE_COUNT).saturating_mul(STANDSTILL_HOLD_US);
    ramped_stroke_us.saturating_add(standstill_us) / MICROS_PER_MILLI
}

/// Worst-case wall-clock duration of a full position-P sweep, in
/// milliseconds: the host's `maximum_workflow_ms`.
///
/// Bounds the sweep at its structural maximum -- every rung `next_rung` can
/// reach from the smallest representable gain -- on top of the one-time
/// P-only floor capture. Neither stop condition is guaranteed to trip before
/// the ladder runs out, so this is the only duration the host can safely arm
/// against.
pub(crate) fn position_tune_worst_case_duration_ms() -> u32 {
    let floor_capture_ms = u64::from(P_ONLY_CAPTURE_US / MICROS_PER_MILLI);
    let sweep_ms = u64::from(worst_case_rung_count()) * u64::from(per_rung_duration_ms());
    u32::try_from(floor_capture_ms + sweep_ms).unwrap_or(u32::MAX)
}

/// Clean-run duration estimate of a position-P sweep, in milliseconds: the
/// host's `nominal_workflow_ms`.
///
/// A stop condition trips within [`NOMINAL_RUNG_COUNT`] rungs of the floor,
/// so no further rung is measured. This is only a disclosure hint; the host
/// arms its run timeout off [`position_tune_worst_case_duration_ms`].
pub(crate) fn position_tune_nominal_duration_ms() -> u32 {
    let floor_capture_ms = u64::from(P_ONLY_CAPTURE_US / MICROS_PER_MILLI);
    let sweep_ms = u64::from(NOMINAL_RUNG_COUNT) * u64::from(per_rung_duration_ms());
    u32::try_from(floor_capture_ms + sweep_ms).unwrap_or(u32::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn worst_case_rung_count_matches_the_representable_half_octave_ladder() {
        // 1 -> 2 -> 3 -> 4 -> 5 -> 7 -> ... -> 29_700, the last rung below
        // `PidPGain::MAX` (0x7fff): 30 half-octave steps, so 31 measured
        // rungs counting the floor itself.
        assert_eq!(worst_case_rung_count(), 31);
    }

    #[test]
    fn per_rung_duration_covers_all_three_strokes_and_every_standstill_capture() {
        // Three 512 ms ramped-stroke repeats (1_536 ms) plus five 100 ms
        // standstill holds (500 ms).
        assert_eq!(per_rung_duration_ms(), 2_036);
    }

    #[test]
    fn worst_case_duration_bounds_the_full_ladder_plus_the_floor_capture() {
        // 4 ms floor capture plus 31 rungs at 2_036 ms.
        assert_eq!(position_tune_worst_case_duration_ms(), 63_120);
    }

    #[test]
    fn nominal_duration_estimates_a_three_rung_clean_run() {
        // 4 ms floor capture plus 3 rungs at 2_036 ms.
        assert_eq!(position_tune_nominal_duration_ms(), 6_112);
    }

    #[test]
    fn nominal_duration_fits_inside_the_worst_case_envelope() {
        // `CommissioningWorkflowPlan::new_position_tune` rejects a nominal
        // that exceeds its maximum, and both are compile-time fixed.
        assert!(position_tune_nominal_duration_ms() <= position_tune_worst_case_duration_ms());
        assert!(position_tune_nominal_duration_ms() > 0);
    }
}
