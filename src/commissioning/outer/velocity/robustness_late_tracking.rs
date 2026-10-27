//! Fixed-time late-tracking measurement for the reversal robustness gate.
//!
//! Over the validation interval, the worst 16-observation window rate is compared with the
//! target rate; the result is reported in parts per million of the target.

use super::robustness_metrics::per_tick_rate_q;

/// Start of the validation interval, microseconds after the post-ramp baseline read.
pub(crate) const LATE_WINDOW_START_US: u32 = 600_000;
/// End of the validation interval, microseconds after the post-ramp baseline read.
pub(crate) const LATE_WINDOW_END_US: u32 = 700_000;
/// Number of consecutive observations in one evaluation window.
pub(crate) const LATE_WINDOW_OBSERVATIONS: usize = 16;
/// Largest admissible late-tracking deviation, in parts per million of the target rate.
pub(crate) const LATE_TRACKING_MAX_PPM: u32 = 50_000;

/// The worst late-tracking window and the sums that produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct LateTracking {
    /// Deviation of the window rate from the target, in parts per million of the target.
    pub ppm: u32,
    /// Summed displacement over the window, in counts.
    pub window_delta_counts: i32,
    /// Summed elapsed time over the window, in microseconds.
    pub window_elapsed_us: u32,
    /// Start of the window's first observation, microseconds after the baseline read.
    pub window_start_us: u32,
}

#[derive(Debug, Clone, Copy)]
struct Observation {
    start_us: u32,
    delta_counts: i32,
    elapsed_us: u32,
}

const EMPTY: Observation = Observation {
    start_us: 0,
    delta_counts: 0,
    elapsed_us: 0,
};

/// Streaming maximum of the 16-observation window deviation over the validation interval.
#[derive(Debug, Clone)]
pub(crate) struct LateTrackingAccumulator {
    target_rate_q: i64,
    ring: [Observation; LATE_WINDOW_OBSERVATIONS],
    len: usize,
    head: usize,
    sum_delta: i64,
    sum_elapsed: u64,
    best: Option<LateTracking>,
}

impl LateTrackingAccumulator {
    /// Create an accumulator for `target_rate_q`; `None` when the target is zero.
    pub(crate) fn new(target_rate_q: i64) -> Option<Self> {
        if target_rate_q == 0 {
            return None;
        }
        Some(Self {
            target_rate_q,
            ring: [EMPTY; LATE_WINDOW_OBSERVATIONS],
            len: 0,
            head: 0,
            sum_delta: 0,
            sum_elapsed: 0,
            best: None,
        })
    }

    /// Feed the observation spanning `(end_us - elapsed_us, end_us]`.
    ///
    /// An observation that is not wholly inside the validation interval breaks the run of
    /// consecutive observations.
    pub(crate) fn observe(&mut self, end_us: u32, delta_counts: i32, elapsed_us: u32) {
        let start_us = end_us.saturating_sub(elapsed_us);
        if start_us < LATE_WINDOW_START_US || end_us > LATE_WINDOW_END_US {
            self.len = 0;
            self.head = 0;
            self.sum_delta = 0;
            self.sum_elapsed = 0;
            return;
        }
        if self.len == LATE_WINDOW_OBSERVATIONS {
            let oldest = self.ring[self.head];
            self.sum_delta -= i64::from(oldest.delta_counts);
            self.sum_elapsed -= u64::from(oldest.elapsed_us);
        } else {
            self.len += 1;
        }
        self.ring[self.head] = Observation {
            start_us,
            delta_counts,
            elapsed_us,
        };
        self.head = (self.head + 1) % LATE_WINDOW_OBSERVATIONS;
        self.sum_delta += i64::from(delta_counts);
        self.sum_elapsed += u64::from(elapsed_us);
        if self.len == LATE_WINDOW_OBSERVATIONS {
            self.evaluate_window();
        }
    }

    fn evaluate_window(&mut self) {
        let delta = saturate_i32(self.sum_delta);
        let elapsed = u32::try_from(self.sum_elapsed).unwrap_or(u32::MAX);
        let ppm = ppm_for(per_tick_rate_q(delta, elapsed), self.target_rate_q);
        if self.best.is_none_or(|best| ppm > best.ppm) {
            self.best = Some(LateTracking {
                ppm,
                window_delta_counts: delta,
                window_elapsed_us: elapsed,
                window_start_us: self.ring[self.head].start_us,
            });
        }
    }

    /// The worst window seen so far; `None` until a full window lies inside the interval.
    pub(crate) fn result(&self) -> Option<LateTracking> {
        self.best
    }
}

fn saturate_i32(value: i64) -> i32 {
    i32::try_from(value).unwrap_or(if value < 0 { i32::MIN } else { i32::MAX })
}

fn ppm_for(rate_q: i64, target_rate_q: i64) -> u32 {
    let diff = (i128::from(rate_q) - i128::from(target_rate_q)).unsigned_abs();
    let target = i128::from(target_rate_q).unsigned_abs();
    let ppm = diff.saturating_mul(1_000_000) / target;
    u32::try_from(ppm).unwrap_or(u32::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::outer::velocity::robustness_metrics::per_tick_rate_q;

    fn target() -> i64 {
        per_tick_rate_q(10, 1_000)
    }

    fn steady(acc: &mut LateTrackingAccumulator, off_target: &[(u32, i32)]) {
        let mut end = 1_000;
        while end <= 800_000 {
            let delta = off_target
                .iter()
                .find(|(t, _)| *t == end)
                .map_or(10, |(_, d)| *d);
            acc.observe(end, delta, 1_000);
            end += 1_000;
        }
    }

    #[test]
    fn a_window_counts_only_when_all_16_observations_lie_inside_the_interval() {
        let mut acc = LateTrackingAccumulator::new(target()).unwrap();
        steady(&mut acc, &[]);
        let clean = acc.result().unwrap();
        assert_eq!(clean.ppm, 0);
        assert_eq!(clean.window_start_us, 600_000);
        assert_eq!(clean.window_elapsed_us, 16_000);

        let mut acc = LateTrackingAccumulator::new(target()).unwrap();
        steady(&mut acc, &[(650_000, 20)]);
        let hit = acc.result().unwrap();
        assert_eq!(hit.ppm, 62_500);
        assert_eq!(hit.window_delta_counts, 170);
        assert_eq!(hit.window_elapsed_us, 16_000);
        assert_eq!(hit.window_start_us, 634_000);
    }

    #[test]
    fn the_last_window_ends_exactly_at_the_interval_end() {
        let mut acc = LateTrackingAccumulator::new(target()).unwrap();
        steady(&mut acc, &[(700_000, 30)]);
        let last = acc.result().unwrap();
        assert_eq!(last.window_start_us, 684_000);
        assert_eq!(last.window_delta_counts, 180);
        assert_eq!(last.ppm, 125_000);
    }

    #[test]
    fn an_observation_straddling_either_bound_is_excluded() {
        let mut acc = LateTrackingAccumulator::new(target()).unwrap();
        acc.observe(601_000, 9_999, 1_500);
        let mut end = 601_000;
        while end < 698_500 {
            end += 1_500;
            acc.observe(end, 15, 1_500);
        }
        acc.observe(699_000, 5, 500);
        acc.observe(700_500, 9_999, 1_500);
        let result = acc.result().unwrap();
        assert_eq!(result.ppm, 0);
        assert_eq!(result.window_start_us, 601_000);
        assert_eq!(result.window_delta_counts, 240);
        assert_eq!(result.window_elapsed_us, 24_000);
    }

    #[test]
    fn the_maximum_over_all_windows_is_returned_with_its_own_audit_tuple() {
        let mut acc = LateTrackingAccumulator::new(target()).unwrap();
        steady(&mut acc, &[(620_000, 11), (680_000, 30)]);
        let result = acc.result().unwrap();
        assert_eq!(result.ppm, 125_000);
        assert_eq!(result.window_delta_counts, 180);
        assert_eq!(result.window_elapsed_us, 16_000);
        assert_eq!(result.window_start_us, 664_000);
    }

    #[test]
    fn fewer_than_16_contained_observations_returns_none() {
        let mut acc = LateTrackingAccumulator::new(target()).unwrap();
        assert_eq!(acc.result(), None);
        let mut end = 601_000;
        while end <= 615_000 {
            acc.observe(end, 10, 1_000);
            end += 1_000;
        }
        assert_eq!(acc.result(), None);
    }

    #[test]
    fn zero_target_rate_is_rejected_by_new() {
        assert!(LateTrackingAccumulator::new(0).is_none());
    }

    #[test]
    fn ppm_truncates_toward_zero_and_survives_extreme_inputs() {
        assert_eq!(ppm_for(100_000_000 + 4_999_999, 100_000_000), 49_999);
        assert_eq!(ppm_for(100_000_000 - 4_999_999, 100_000_000), 49_999);
        assert_eq!(ppm_for(i64::MIN, i64::MAX), 2_000_000);
        assert_eq!(ppm_for(i64::MAX, i64::MIN), 1_999_999);
        assert_eq!(ppm_for(i64::MAX, i64::MAX), 0);
        assert_eq!(ppm_for(i64::MAX, 1), u32::MAX);

        let mut acc = LateTrackingAccumulator::new(i64::MAX).unwrap();
        for i in 0..16 {
            acc.observe(601_000 + i * 1_000, i32::MAX, 1_000);
        }
        let result = acc.result().unwrap();
        assert_eq!(result.window_delta_counts, i32::MAX);
    }
}
