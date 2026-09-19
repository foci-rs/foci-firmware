//! Per-leg metrics of a step-driven stroke's position-error capture.

use crate::autotune::{PositionTuneOutcome, TRANSIENT_LEN};
use crate::commissioning::stimulus::trapezoid::StrokeProfile;

#[allow(dead_code)]
const MICROS_PER_MILLI: u32 = 1_000;

/// The emitted stroke's boundaries, captured once per stroke from the
/// profile so the capture windows follow the schedule the ISR runs.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct StrokeTiming {
    pub ramp_end_us: u32,
    pub cruise_end_us: u32,
    pub leg_us: u32,
    pub stroke_us: u32,
}

impl StrokeTiming {
    #[allow(dead_code)]
    pub(crate) fn from_profile(profile: &StrokeProfile) -> Self {
        Self {
            ramp_end_us: profile.ramp_end_us(),
            cruise_end_us: profile.cruise_end_us(),
            leg_us: profile.leg_us(),
            stroke_us: profile.stroke_us(),
        }
    }
}

/// Smallest whole-millisecond sample period that fits `stroke_us` into
/// the capture buffer.
#[allow(dead_code)]
pub(crate) fn sample_period_us(stroke_us: u32) -> u32 {
    let samples = TRANSIENT_LEN as u32;
    let raw = stroke_us.div_ceil(samples);
    raw.div_ceil(MICROS_PER_MILLI).max(1) * MICROS_PER_MILLI
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct LegWindow {
    pub start: usize,
    pub cruise_start: usize,
    pub cruise_end: usize,
    pub end: usize,
}

#[allow(dead_code)]
pub(crate) fn leg_windows(
    timing: &StrokeTiming,
    sample_period_us: u32,
    captured: usize,
) -> [LegWindow; 2] {
    let idx = |us: u32| (us / sample_period_us) as usize;
    let window = |offset_us: u32| {
        let clamp = |v: usize| v.min(captured);
        LegWindow {
            start: clamp(idx(offset_us)),
            cruise_start: clamp(idx(offset_us + timing.ramp_end_us)),
            cruise_end: clamp(idx(offset_us + timing.cruise_end_us)),
            end: clamp(idx(offset_us + timing.leg_us)),
        }
    };
    [window(0), window(timing.leg_us)]
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub(crate) struct LegMetrics {
    pub cruise_mean_abs_units: u32,
    pub peak_abs_units: u32,
    pub overshoot_units: u32,
    pub cruise_rms_units: u32,
}

#[allow(dead_code)]
pub(crate) fn leg_metrics(
    samples: &[i32],
    window: &LegWindow,
) -> Result<LegMetrics, PositionTuneOutcome> {
    let leg = samples.get(window.start..window.end).unwrap_or(&[]);
    let cruise = samples
        .get(window.cruise_start..window.cruise_end)
        .unwrap_or(&[]);
    if leg.is_empty() || cruise.is_empty() {
        return Err(PositionTuneOutcome::CaptureEmpty);
    }
    let peak_abs = leg.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
    let cruise_abs_sum: u64 = cruise.iter().map(|s| u64::from(s.unsigned_abs())).sum();
    let cruise_mean_abs = u32::try_from(cruise_abs_sum / cruise.len() as u64).unwrap_or(u32::MAX);
    // RMS is about the signed mean: it measures ripple around the lead.
    let cruise_sum: i64 = cruise.iter().map(|&s| i64::from(s)).sum();
    let cruise_mean = cruise_sum / cruise.len() as i64;
    // Widened to i128/u128: an i32 sample can sit up to ~2^32 away from
    // cruise_mean, and squaring that difference is ~2^64 -- one past what a
    // u64 accumulator (let alone the running sum over up to TRANSIENT_LEN
    // samples) can represent. i128 keeps the square and the summed total
    // far below its range for every representable i32 input.
    let squares: u128 = cruise
        .iter()
        .map(|&s| {
            let diff = i128::from(s) - i128::from(cruise_mean);
            (diff * diff) as u128
        })
        .sum();
    let cruise_rms = u32::try_from((squares / cruise.len() as u128).isqrt()).unwrap_or(u32::MAX);

    // Steady-state reference for overshoot: the mean of the cruise window's
    // last 10% (at least 1 sample), same as the sibling `RampedStrokeMetrics`
    // reduction in `position_tune/mod.rs` -- steady-state must be measured
    // after the loop has settled, not blended with a window that may still
    // be transient. But the cruise window's own *final* sample sits right at
    // the reversal boundary, exactly where a reversal-adjacent overshoot
    // spike lands, so it is trimmed out of the tail average before that
    // average is taken (leaving at least 1 sample): an un-trimmed tail would
    // let the spike itself pull its own reference toward it and undercount
    // the overshoot it is meant to measure.
    let tail_len = (cruise.len() / 10).max(1);
    let tail = &cruise[cruise.len() - tail_len..];
    let trim = tail.len().saturating_sub(1).min(2);
    let trimmed_tail = &tail[..tail.len() - trim];
    let steady =
        trimmed_tail.iter().map(|&s| i64::from(s)).sum::<i64>() / trimmed_tail.len() as i64;
    let approaching_from_below = leg
        .iter()
        .find(|&&s| i64::from(s) != steady)
        .map(|&s| i64::from(s) < steady);
    let overshoot = match approaching_from_below {
        None => 0,
        Some(from_below) => leg
            .iter()
            .map(|&s| {
                let excess = if from_below {
                    i64::from(s) - steady
                } else {
                    steady - i64::from(s)
                };
                excess.max(0)
            })
            .max()
            .unwrap_or(0),
    };
    Ok(LegMetrics {
        cruise_mean_abs_units: cruise_mean_abs,
        peak_abs_units: peak_abs,
        overshoot_units: u32::try_from(overshoot).unwrap_or(u32::MAX),
        cruise_rms_units: cruise_rms,
    })
}

#[allow(dead_code)]
pub(crate) fn worst_of(a: &LegMetrics, b: &LegMetrics) -> LegMetrics {
    LegMetrics {
        cruise_mean_abs_units: a.cruise_mean_abs_units.max(b.cruise_mean_abs_units),
        peak_abs_units: a.peak_abs_units.max(b.peak_abs_units),
        overshoot_units: a.overshoot_units.max(b.overshoot_units),
        cruise_rms_units: a.cruise_rms_units.max(b.cruise_rms_units),
    }
}

#[allow(dead_code)]
pub(crate) fn median3(mut values: [u32; 3]) -> u32 {
    values.sort_unstable();
    values[1]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Boundaries as a 1b profile would report them for the 400 mm/s
    /// worked example (values rounded for the test; only the structure
    /// matters here).
    fn fast_timing() -> StrokeTiming {
        StrokeTiming {
            ramp_end_us: 50_000,
            cruise_end_us: 150_000,
            leg_us: 200_000,
            stroke_us: 400_000,
        }
    }

    #[test]
    fn sample_period_fits_the_stroke_into_the_buffer() {
        assert_eq!(sample_period_us(400_000), 2_000);
        assert_eq!(sample_period_us(728_000), 3_000);
        assert_eq!(sample_period_us(100_000), 1_000);
        assert_eq!(sample_period_us(256_000), 1_000);
        assert_eq!(sample_period_us(256_001), 2_000);
    }

    #[test]
    fn leg_windows_follow_the_plan_timing() {
        let [forward, reverse] = leg_windows(&fast_timing(), 2_000, 200);
        assert_eq!(
            (
                forward.start,
                forward.cruise_start,
                forward.cruise_end,
                forward.end
            ),
            (0, 25, 75, 100)
        );
        assert_eq!(
            (
                reverse.start,
                reverse.cruise_start,
                reverse.cruise_end,
                reverse.end
            ),
            (100, 125, 175, 200)
        );
    }

    #[test]
    fn leg_windows_clamp_to_the_capture_length() {
        let [_, reverse] = leg_windows(&fast_timing(), 2_000, 150);
        assert_eq!(reverse.end, 150);
        assert!(reverse.cruise_end <= 150);
    }

    #[test]
    fn cruise_mean_is_the_mean_of_absolute_errors_not_the_absolute_mean() {
        // +30 and -30 alternating: the signed mean is 0, the mean absolute
        // error is 30, and the bound must see 30.
        let mut samples = std::vec![0_i32; 40];
        for (i, s) in samples.iter_mut().enumerate() {
            *s = if i % 2 == 0 { 30 } else { -30 };
        }
        let window = LegWindow {
            start: 0,
            cruise_start: 0,
            cruise_end: 40,
            end: 40,
        };
        let m = leg_metrics(&samples, &window).unwrap();
        assert_eq!(m.cruise_mean_abs_units, 30);
    }

    #[test]
    fn leg_metrics_measure_mean_peak_overshoot_and_rms() {
        // Approach from below to a steady +40, one overshoot to +52 at the
        // reversal end, a peak of |-90| during the ramp.
        let mut samples = std::vec![0_i32; 100];
        for (i, s) in samples.iter_mut().enumerate().take(25) {
            *s = -(i as i32) * 4;
        } // ramp: down to -96
        for s in samples.iter_mut().take(75).skip(25) {
            *s = 40;
        } // cruise at +40
        samples[74] = 52; // overshoot past steady
        for s in samples.iter_mut().skip(75) {
            *s = 10;
        } // decel
        let window = LegWindow {
            start: 0,
            cruise_start: 25,
            cruise_end: 75,
            end: 100,
        };
        let m = leg_metrics(&samples, &window).unwrap();
        assert_eq!(m.peak_abs_units, 96);
        assert_eq!(m.cruise_mean_abs_units, 40);
        assert_eq!(m.overshoot_units, 12);
        assert!(m.cruise_rms_units <= 2);
    }

    #[test]
    fn cruise_rms_does_not_overflow_a_u64_accumulator() {
        // Half the window at i32::MIN, half at i32::MAX: both are values a
        // caller can legally pass. cruise_mean truncates to 0, so each
        // squared deviation is ~2^62 and the summed total across all 256
        // samples is ~1.18e21 -- about 64x past u64::MAX. A `u64` squares
        // accumulator panics (debug) or silently wraps (release) on this
        // input; the widened i128/u128 accumulator does not.
        let mut samples = std::vec![0_i32; 256];
        for (i, s) in samples.iter_mut().enumerate() {
            *s = if i < 128 { i32::MIN } else { i32::MAX };
        }
        let window = LegWindow {
            start: 0,
            cruise_start: 0,
            cruise_end: 256,
            end: 256,
        };
        let m = leg_metrics(&samples, &window).unwrap();
        assert_eq!(m.cruise_rms_units, 2_147_483_647);
    }

    #[test]
    fn empty_window_is_a_reported_failure() {
        let window = LegWindow {
            start: 5,
            cruise_start: 5,
            cruise_end: 5,
            end: 5,
        };
        assert_eq!(
            leg_metrics(&[1, 2, 3, 4, 5, 6], &window),
            Err(PositionTuneOutcome::CaptureEmpty)
        );
    }

    #[test]
    fn median3_and_worst_of() {
        assert_eq!(median3([5, 1, 3]), 3);
        assert_eq!(median3([2, 2, 9]), 2);
        let a = LegMetrics {
            cruise_mean_abs_units: 1,
            peak_abs_units: 9,
            overshoot_units: 3,
            cruise_rms_units: 0,
        };
        let b = LegMetrics {
            cruise_mean_abs_units: 4,
            peak_abs_units: 2,
            overshoot_units: 3,
            cruise_rms_units: 7,
        };
        assert_eq!(
            worst_of(&a, &b),
            LegMetrics {
                cruise_mean_abs_units: 4,
                peak_abs_units: 9,
                overshoot_units: 3,
                cruise_rms_units: 7
            }
        );
    }
}
