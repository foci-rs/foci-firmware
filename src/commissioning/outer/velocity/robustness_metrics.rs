//! Pure derivation of reversal-robustness metrics from raw velocity observations.
//!
//! Deterministic transforms over per-tick `{delta_counts, elapsed_us}` samples; band-entry
//! timing and recovery IAE are computed on the derived per-tick rate, independent of the
//! stationarity suffix selector.

use crate::autotune::velocity_measurement::VELOCITY_RATE_Q_SCALE;
use crate::autotune::velocity_measurement::VelocityObservationSource;

use super::sweep_plan::{ENVELOPE_PROPORTIONAL_DEN, ENVELOPE_PROPORTIONAL_NUM};

const MICROS_PER_SECOND: i128 = 1_000_000;

const OVERSHOOT_SCALE_Q: i64 = 1_000_000 * VELOCITY_RATE_Q_SCALE;

/// Reconvergence and recovery metrics for one reversal-robustness probe.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct ReversalMetrics {
    /// Elapsed time from reversal onset to the start of the sustained
    /// band-entry dwell.
    pub reconvergence_time_us: u32,
    /// Residual per-tick rate error, in rate-Q units, once settled.
    pub settled_residual_q: i64,
    /// Integral of absolute rate error over the recovery window, in
    /// rate-Q-microsecond units.
    pub recovery_iae_q: i64,
    /// Demeaned first-difference RMS of settled per-tick rate error, in raw rate-Q units;
    /// `0` unless the leg reached the settled tail.
    pub dac_rms_q: i64,
    /// Elapsed time from the initial (forward) approach to its own settle.
    pub forward_settle_time_us: u32,
    /// `reconvergence_time_us` expressed as parts-per-million of
    /// `forward_settle_time_us`. Populated by the calling phase, not here.
    pub reconvergence_ratio_ppm: u32,
}

/// Derive the per-tick rate in rate-Q units (`VELOCITY_RATE_Q_SCALE`).
///
/// Returns `0` for a zero-elapsed sample.
pub(crate) fn per_tick_rate_q(delta_counts: i32, elapsed_us: u32) -> i64 {
    if elapsed_us == 0 {
        return 0;
    }
    let numerator = i128::from(delta_counts)
        .saturating_mul(MICROS_PER_SECOND)
        .saturating_mul(i128::from(VELOCITY_RATE_Q_SCALE));
    let rate = numerator / i128::from(elapsed_us);
    i64::try_from(rate).unwrap_or(if rate.is_negative() {
        i64::MIN
    } else {
        i64::MAX
    })
}

/// Return the elapsed time (from the start of `obs`) at the first sample of a dwell of at
/// least `dwell_us` inside `[target_rate_q - band_q, target_rate_q + band_q]`.
///
/// Returns `None` if no such dwell exists (the inconclusive signal). Reference scanner for
/// [`ReversalOnlineAccumulator`].
#[cfg(test)]
pub(crate) fn band_entry_time_us(
    obs: &impl VelocityObservationSource,
    target_rate_q: i64,
    band_q: i64,
    dwell_us: u32,
) -> Option<u32> {
    let lower = target_rate_q.saturating_sub(band_q);
    let upper = target_rate_q.saturating_add(band_q);

    let mut elapsed_before_sample_us: u32 = 0;
    let mut run_start_us: Option<u32> = None;
    let mut run_duration_us: u32 = 0;

    for index in 0..obs.observation_len() {
        let Some(sample) = obs.observation(index) else {
            break;
        };
        let rate = per_tick_rate_q(sample.delta_counts, sample.elapsed_us);
        if rate >= lower && rate <= upper {
            let start = run_start_us.get_or_insert(elapsed_before_sample_us);
            run_duration_us = run_duration_us.saturating_add(sample.elapsed_us);
            if run_duration_us >= dwell_us {
                return Some(*start);
            }
        } else {
            run_start_us = None;
            run_duration_us = 0;
        }
        elapsed_before_sample_us = elapsed_before_sample_us.saturating_add(sample.elapsed_us);
    }
    None
}

/// Integral of absolute per-tick rate error against `target_rate_q` over `obs[0..end_index]`,
/// weighted by elapsed time: `sum(|rate_i - target_rate_q| * elapsed_us_i)`.
///
/// Reference scanner for [`ReversalOnlineAccumulator`].
#[cfg(test)]
pub(crate) fn recovery_iae_q(
    obs: &impl VelocityObservationSource,
    target_rate_q: i64,
    end_index: usize,
) -> i64 {
    let mut iae_q: i64 = 0;
    for index in 0..end_index {
        let Some(sample) = obs.observation(index) else {
            break;
        };
        let rate = per_tick_rate_q(sample.delta_counts, sample.elapsed_us);
        let error_q = rate.saturating_sub(target_rate_q).unsigned_abs();
        let error_q = i64::try_from(error_q).unwrap_or(i64::MAX);
        let contribution_q = error_q.saturating_mul(i64::from(sample.elapsed_us));
        iae_q = iae_q.saturating_add(contribution_q);
    }
    iae_q
}

/// Demeaned first-difference RMS of the per-tick rate error over
/// `[start_index, start_index + len)` of `source`.
///
/// Demeans the window's rate error against `target_rate_q`,
/// then takes the RMS of the demeaned series' first difference. Missing
/// indices (past what `source` holds) are skipped, not treated as zero.
/// Returns 0 for fewer than 2 available samples.
pub(crate) fn dac_rms_q<S: VelocityObservationSource + ?Sized>(
    source: &S,
    start_index: usize,
    len: usize,
    target_rate_q: i64,
) -> i64 {
    let mut sum_q: i64 = 0;
    let mut count: i64 = 0;
    for offset in 0..len {
        let Some(observation) = source.observation(start_index + offset) else {
            continue;
        };
        let error_q = per_tick_rate_q(observation.delta_counts, observation.elapsed_us)
            .saturating_sub(target_rate_q);
        sum_q = sum_q.saturating_add(error_q);
        count += 1;
    }
    if count < 2 {
        return 0;
    }
    let mean_q = sum_q / count;

    let mut sq_diff_sum_f: f32 = 0.0;
    let mut diff_count: i64 = 0;
    let mut prev_ac_q: Option<i64> = None;
    for offset in 0..len {
        let Some(observation) = source.observation(start_index + offset) else {
            continue;
        };
        let error_q = per_tick_rate_q(observation.delta_counts, observation.elapsed_us)
            .saturating_sub(target_rate_q);
        let ac_q = error_q.saturating_sub(mean_q);
        if let Some(prev_ac_q) = prev_ac_q {
            let diff_q = (ac_q.saturating_sub(prev_ac_q)) as f32;
            sq_diff_sum_f += diff_q * diff_q;
            diff_count += 1;
        }
        prev_ac_q = Some(ac_q);
    }
    if diff_count == 0 {
        return 0;
    }
    let mean_sq_diff_f = sq_diff_sum_f / (diff_count as f32);
    libm::sqrtf(mean_sq_diff_f) as i64
}

/// Samples in a probe leg band-membership window.
pub(crate) const PROBE_BAND_WINDOW_SAMPLES: usize = 16;

/// Online band-entry and recovery-IAE accumulator for one capture leg.
///
/// Folds each observation in as it arrives, yielding the band-entry time and recovery IAE
/// of [`band_entry_time_us`] and [`recovery_iae_q`] without retaining the full stream.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub(crate) struct ReversalOnlineAccumulator {
    target_rate_q: i64,
    lower_q: i64,
    upper_q: i64,
    dwell_us: u32,
    band_window_samples: usize,
    band_window: [(i32, u32); PROBE_BAND_WINDOW_SAMPLES],
    band_window_len: usize,
    band_window_head: usize,
    band_window_delta: i64,
    band_window_elapsed: u64,
    elapsed_before_us: u32,
    iae_running_q: i64,
    run_start_us: Option<u32>,
    run_iae_at_start_q: i64,
    run_duration_us: u32,
    band_entry_us: Option<u32>,
    band_entry_iae_q: i64,
    actual_position_counts: i32,
    expected_position_q: i64,
    peak_excess_q: i64,
}

impl ReversalOnlineAccumulator {
    /// Construct an idle accumulator with a zero band around zero.
    pub(crate) const fn new() -> Self {
        Self {
            target_rate_q: 0,
            lower_q: 0,
            upper_q: 0,
            dwell_us: 0,
            band_window_samples: 1,
            band_window: [(0, 0); PROBE_BAND_WINDOW_SAMPLES],
            band_window_len: 0,
            band_window_head: 0,
            band_window_delta: 0,
            band_window_elapsed: 0,
            elapsed_before_us: 0,
            iae_running_q: 0,
            run_start_us: None,
            run_iae_at_start_q: 0,
            run_duration_us: 0,
            band_entry_us: None,
            band_entry_iae_q: 0,
            actual_position_counts: 0,
            expected_position_q: 0,
            peak_excess_q: 0,
        }
    }

    /// Re-arm for a fresh leg targeting `target_rate_q` with a symmetric `band_q` half-width
    /// and a `dwell_us` residence requirement, clearing the leg-scoped overshoot tracking.
    ///
    /// Band membership is judged on the mean rate of the last `band_window_samples`
    /// observations (at most [`PROBE_BAND_WINDOW_SAMPLES`]); IAE and overshoot use the raw
    /// per-tick rate.
    pub(crate) fn reset_with_band_window(
        &mut self,
        target_rate_q: i64,
        band_q: i64,
        dwell_us: u32,
        band_window_samples: usize,
    ) {
        self.target_rate_q = target_rate_q;
        self.lower_q = target_rate_q.saturating_sub(band_q);
        self.upper_q = target_rate_q.saturating_add(band_q);
        self.dwell_us = dwell_us;
        self.band_window_samples = band_window_samples.clamp(1, PROBE_BAND_WINDOW_SAMPLES);
        self.band_window_len = 0;
        self.band_window_head = 0;
        self.band_window_delta = 0;
        self.band_window_elapsed = 0;
        self.elapsed_before_us = 0;
        self.iae_running_q = 0;
        self.run_start_us = None;
        self.run_iae_at_start_q = 0;
        self.run_duration_us = 0;
        self.band_entry_us = None;
        self.band_entry_iae_q = 0;
        self.actual_position_counts = 0;
        self.expected_position_q = 0;
        self.peak_excess_q = 0;
    }

    /// Fold one observation into the running band-entry and IAE state.
    ///
    /// Mirrors [`band_entry_time_us`]/[`recovery_iae_q`]; once band entry is recorded,
    /// further observations are ignored.
    pub(crate) fn observe(&mut self, delta_counts: i32, elapsed_us: u32) {
        self.expected_position_q = self
            .expected_position_q
            .saturating_add(self.target_rate_q.saturating_mul(i64::from(elapsed_us)));
        self.actual_position_counts = self.actual_position_counts.saturating_add(delta_counts);
        let excess_q = i64::from(self.actual_position_counts)
            .saturating_mul(OVERSHOOT_SCALE_Q)
            .saturating_sub(self.expected_position_q)
            .saturating_mul(self.target_rate_q.signum());
        self.peak_excess_q = self.peak_excess_q.max(excess_q.max(0));
        if self.band_entry_us.is_some() {
            return;
        }
        let rate = per_tick_rate_q(delta_counts, elapsed_us);
        let in_band = self
            .windowed_rate_q(delta_counts, elapsed_us)
            .is_some_and(|windowed| windowed >= self.lower_q && windowed <= self.upper_q);
        if in_band {
            if self.run_start_us.is_none() {
                self.run_start_us = Some(self.elapsed_before_us);
                self.run_iae_at_start_q = self.iae_running_q;
                self.run_duration_us = 0;
            }
            self.run_duration_us = self.run_duration_us.saturating_add(elapsed_us);
            if self.run_duration_us >= self.dwell_us {
                self.band_entry_us = self.run_start_us;
                self.band_entry_iae_q = self.run_iae_at_start_q;
            }
        } else {
            self.run_start_us = None;
            self.run_duration_us = 0;
        }
        let error_q = rate.saturating_sub(self.target_rate_q).unsigned_abs();
        let error_q = i64::try_from(error_q).unwrap_or(i64::MAX);
        let contribution_q = error_q.saturating_mul(i64::from(elapsed_us));
        self.iae_running_q = self.iae_running_q.saturating_add(contribution_q);
        self.elapsed_before_us = self.elapsed_before_us.saturating_add(elapsed_us);
    }

    fn windowed_rate_q(&mut self, delta_counts: i32, elapsed_us: u32) -> Option<i64> {
        if self.band_window_samples == 1 {
            return Some(per_tick_rate_q(delta_counts, elapsed_us));
        }
        if self.band_window_len == self.band_window_samples {
            let (old_delta, old_elapsed) = self.band_window[self.band_window_head];
            self.band_window_delta -= i64::from(old_delta);
            self.band_window_elapsed -= u64::from(old_elapsed);
        } else {
            self.band_window_len += 1;
        }
        self.band_window[self.band_window_head] = (delta_counts, elapsed_us);
        self.band_window_head = (self.band_window_head + 1) % self.band_window_samples;
        self.band_window_delta += i64::from(delta_counts);
        self.band_window_elapsed += u64::from(elapsed_us);
        if self.band_window_len < self.band_window_samples {
            return None;
        }
        let delta = i32::try_from(self.band_window_delta).unwrap_or(i32::MAX);
        let elapsed = u32::try_from(self.band_window_elapsed).unwrap_or(u32::MAX);
        Some(per_tick_rate_q(delta, elapsed))
    }

    /// Band-entry time (elapsed-us from leg start), or `None` while the leg has
    /// not yet sustained a full `dwell_us` inside the band.
    pub(crate) fn band_entry_us(&self) -> Option<u32> {
        self.band_entry_us
    }

    /// Recovery IAE integrated up to band entry. Meaningful only once
    /// [`Self::band_entry_us`] returns `Some`.
    pub(crate) fn recovery_iae_q(&self) -> i64 {
        self.band_entry_iae_q
    }

    /// Peak positive excess of actual displacement over the ideal constant-target-rate
    /// trajectory since the last `reset()`.
    pub(crate) fn reversal_overshoot_counts(&self) -> u16 {
        u16::try_from(self.peak_excess_q / OVERSHOOT_SCALE_Q).unwrap_or(u16::MAX)
    }

    /// Whole-leg mean velocity-to-target ratio in percent; zero before any target
    /// displacement accrues.
    pub(crate) fn velocity_to_target_pct(&self) -> i32 {
        if self.expected_position_q == 0 {
            return 0;
        }
        let actual_q = i128::from(self.actual_position_counts) * i128::from(OVERSHOOT_SCALE_Q);
        let ratio = actual_q.saturating_mul(100) / i128::from(self.expected_position_q);
        i32::try_from(ratio).unwrap_or(i32::MAX)
    }
}

/// Absolute per-direction gate thresholds for one reversal-robustness probe.
///
/// Compared directly against `ReversalMetrics`, never against another candidate.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub(crate) struct RobustnessThresholds {
    /// Maximum allowed `reconvergence_time_us`.
    pub t_max_us: u32,
    /// Maximum allowed `|settled_residual_q|`.
    pub r_max_q: i64,
    /// Maximum allowed `recovery_iae_q`.
    pub iae_max_q: i64,
}

/// Conservative seed thresholds, deliberately loose. The gate compares the per-direction
/// median of each metric against these bounds.
///
/// `r_max_q` and `iae_max_q` are frozen literals derived offline from a 176 rpm reference
/// target at 4000 counts per rev; they are never recomputed at runtime.
pub(crate) const ROBUSTNESS_THRESHOLDS: RobustnessThresholds = RobustnessThresholds {
    t_max_us: ROBUSTNESS_MEASUREMENT_BUDGET_US - ROBUSTNESS_TRIP_TIME_HEADROOM_US,
    r_max_q: 1_173_333,
    iae_max_q: 2 * (2 * 11_733_333) * 500_000,
};

/// Bit set in `tripped_mask` when `reconvergence_time_us` exceeds `t_max_us`.
pub(crate) const TRIP_TIME: u8 = 1 << 0;
/// Bit set in `tripped_mask` when `|settled_residual_q|` exceeds `r_max_q`.
pub(crate) const TRIP_RESIDUAL: u8 = 1 << 1;
/// Bit set in `tripped_mask` when `recovery_iae_q` exceeds `iae_max_q`.
pub(crate) const TRIP_IAE: u8 = 1 << 2;
/// Bit set in a direction's tripped mask when its maximum scored reversal velocity peak
/// reaches [`REVERSAL_PEAK_REJECT_PERMILLE`].
pub(crate) const TRIP_PEAK: u8 = 1 << 3;

/// Reversal velocity peak, in permille of the leg target, at which a direction is rejected:
/// the outer-safety envelope fraction.
pub(crate) const REVERSAL_PEAK_REJECT_PERMILLE: u16 =
    (1000 * ENVELOPE_PROPORTIONAL_NUM / ENVELOPE_PROPORTIONAL_DEN) as u16;

/// Settle-band half-width, as a percentage of `|target_rate_q|`.
pub(crate) const ROBUSTNESS_BAND_FRACTION_PCT: u32 = 5;

/// Minimum sustained dwell inside the settle band, in microseconds.
/// About 48 ticks at the 1 ms observation cadence, under the shortest stationary suffix.
pub(crate) const ROBUSTNESS_BAND_DWELL_US: u32 = 48_000;

/// Overall measurement budget for one probe leg, in microseconds; must exceed `t_max_us`.
/// A leg that does not reach band entry within it is inconclusive.
pub(crate) const ROBUSTNESS_MEASUREMENT_BUDGET_US: u32 = 700_000;

/// Fixed integration window the live IAE bound is derived over.
pub(crate) const ROBUSTNESS_IAE_REFERENCE_WINDOW_US: u32 = 500_000;

/// Gap between the measurement budget and the reconvergence trip line.
pub(crate) const ROBUSTNESS_TRIP_TIME_HEADROOM_US: u32 = 100_000;

/// Probe-leg cruise target as a percentage of the campaign amplitude.
///
/// Legs at 85% put the cap at 1.76x of the leg target while the gate rejects at 1.5x, so a
/// reversal rejected by the gate is still 15% or more below the cap.
pub(crate) const ROBUSTNESS_LEG_AMPLITUDE_PCT: i32 = 85;

/// Bounded retry count for an inconclusive probe leg.
pub(crate) const ROBUSTNESS_MAX_RETRIES: u8 = 4;

/// Number of `ROBUSTNESS_MEASUREMENT_BUDGET_US`-sized legs run per probe cycle: establish,
/// reversal, rest window, and one re-opened rest window.
///
/// The worst-case per-cycle motion envelope is this count times the budget; analysis time
/// is not charged.
pub(crate) const ROBUSTNESS_LEGS_PER_CYCLE: u32 = 4;

/// Degenerate cut-in divisor for [`robustness_thresholds_for`]: a `plant_rate_q` below
/// `target_rate_q / PLANT_RATE_DEGENERATE_MIN_DIVISOR` fails open to
/// [`ROBUSTNESS_IAE_SEED_MAX_Q`].
pub(crate) const PLANT_RATE_DEGENERATE_MIN_DIVISOR: i64 = 256;

/// Loose IAE seed used as the degenerate fail-open value and as the calibration anchor for
/// [`ROBUSTNESS_IAE_COEFF_NUM`]/[`ROBUSTNESS_IAE_COEFF_DEN`].
pub(crate) const ROBUSTNESS_IAE_SEED_MAX_Q: i64 = 23_466_666_000_000;

/// Calibration coefficient (numerator) for `iae_max_q` in [`robustness_thresholds_for`]:
/// `iae_max_q = plant_rate_q * NUM * REFERENCE_WINDOW_US / DEN`.
///
/// `plant_rate_q` must derive from breakaway plant identification, never a candidate own
/// reversal measurement.
pub(crate) const ROBUSTNESS_IAE_COEFF_NUM: i64 = 29;
/// Calibration coefficient (denominator). See [`ROBUSTNESS_IAE_COEFF_NUM`].
pub(crate) const ROBUSTNESS_IAE_COEFF_DEN: i64 = 200;

/// Reduce the two reversal-direction response intervals into one worse-of plant rate in
/// rate-Q units: per direction `max(|low|, |high|)`, pooled with `max`.
pub(crate) fn reduce_plant_rate_q(intervals: &[[i32; 2]; 2]) -> i64 {
    let dir0 = intervals[0][0]
        .unsigned_abs()
        .max(intervals[0][1].unsigned_abs());
    let dir1 = intervals[1][0]
        .unsigned_abs()
        .max(intervals[1][1].unsigned_abs());
    i64::from(dir0.max(dir1))
}

/// Construct plant-adaptive robustness thresholds from the identified settled-rate response
/// (`plant_rate_q`, see [`reduce_plant_rate_q`]) and the reference target rate.
///
/// `t_max_us` and `r_max_q` carry over from [`ROBUSTNESS_THRESHOLDS`]; only `iae_max_q`
/// adapts. See [`ROBUSTNESS_IAE_COEFF_NUM`] for the `plant_rate_q` provenance requirement.
pub(crate) fn robustness_thresholds_for(
    plant_rate_q: i64,
    target_rate_q: i64,
) -> RobustnessThresholds {
    let iae_max_q = if plant_rate_q < target_rate_q / PLANT_RATE_DEGENERATE_MIN_DIVISOR {
        ROBUSTNESS_IAE_SEED_MAX_Q
    } else {
        let scaled = i128::from(ROBUSTNESS_IAE_COEFF_NUM)
            .saturating_mul(i128::from(plant_rate_q))
            .saturating_mul(i128::from(ROBUSTNESS_IAE_REFERENCE_WINDOW_US))
            / i128::from(ROBUSTNESS_IAE_COEFF_DEN);
        i64::try_from(scaled).unwrap_or(if scaled.is_negative() {
            i64::MIN
        } else {
            i64::MAX
        })
    };
    RobustnessThresholds {
        t_max_us: ROBUSTNESS_THRESHOLDS.t_max_us,
        r_max_q: ROBUSTNESS_THRESHOLDS.r_max_q,
        iae_max_q,
    }
}

/// Compute the tripped bitmask for `m` against `t`: which of time, residual,
/// and IAE gates failed, independently of one another.
pub(crate) fn tripped_mask(m: &ReversalMetrics, t: &RobustnessThresholds) -> u8 {
    let mut mask = 0u8;
    if m.reconvergence_time_us > t.t_max_us {
        mask |= TRIP_TIME;
    }
    if m.settled_residual_q.unsigned_abs() > t.r_max_q.unsigned_abs() {
        mask |= TRIP_RESIDUAL;
    }
    if m.recovery_iae_q > t.iae_max_q {
        mask |= TRIP_IAE;
    }
    mask
}

/// OR the tripped masks of both reversal directions: a candidate fails if
/// either direction trips any gate.
pub(crate) fn worse_of(a: u8, b: u8) -> u8 {
    a | b
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::outer::state::VelocityObservationRing;

    #[test]
    fn the_reversal_peak_bound_is_the_envelope_fraction() {
        assert_eq!(REVERSAL_PEAK_REJECT_PERMILLE, 1500);
    }

    #[test]
    fn band_entry_reports_first_stable_dwell_and_ignores_earlier_excursions() {
        let mut ring = VelocityObservationRing::new();
        let target = per_tick_rate_q(100, 1_000);
        for _ in 0..5 {
            ring.push(400, 1_000).unwrap();
        }
        for _ in 0..15 {
            ring.push(100, 1_000).unwrap();
        }
        let band = per_tick_rate_q(1, 1_000);
        let t = band_entry_time_us(&ring, target, band, 3_000).expect("enters band");
        assert_eq!(t, 5_000);
    }

    #[test]
    fn band_entry_is_none_when_never_stable() {
        let mut ring = VelocityObservationRing::new();
        let target = per_tick_rate_q(100, 1_000);
        for i in 0..20 {
            ring.push(if i % 2 == 0 { 100 } else { 400 }, 1_000)
                .unwrap();
        }
        let band = per_tick_rate_q(1, 1_000);
        assert_eq!(band_entry_time_us(&ring, target, band, 3_000), None);
    }

    #[test]
    fn recovery_iae_sums_absolute_error_times_elapsed() {
        let mut ring = VelocityObservationRing::new();
        let target = per_tick_rate_q(100, 1_000);
        ring.push(120, 1_000).unwrap();
        ring.push(80, 1_000).unwrap();
        let iae = recovery_iae_q(&ring, target, 2);
        let per_err = (per_tick_rate_q(20, 1_000)).abs() * 1_000;
        assert_eq!(iae, per_err * 2);
    }

    #[test]
    fn band_entry_ignores_the_stationarity_suffix_selector() {
        let mut ring = VelocityObservationRing::new();
        let target = per_tick_rate_q(100, 1_000);
        for _ in 0..8 {
            ring.push(500, 1_000).unwrap();
        }
        for _ in 0..64 {
            ring.push(100, 1_000).unwrap();
        }
        let band = per_tick_rate_q(1, 1_000);
        assert!(band_entry_time_us(&ring, target, band, 3_000).is_some());
    }

    fn assert_online_matches_pure(samples: &[(i32, u32)], target: i64, band: i64, dwell_us: u32) {
        let mut ring = VelocityObservationRing::new();
        let mut online = ReversalOnlineAccumulator::new();
        online.reset_with_band_window(target, band, dwell_us, 1);
        for &(delta, elapsed) in samples {
            ring.push(delta, elapsed as u16).unwrap();
            online.observe(delta, elapsed);
        }
        let pure_entry = band_entry_time_us(&ring, target, band, dwell_us);
        assert_eq!(online.band_entry_us(), pure_entry);
        if let Some(entry) = pure_entry {
            let mut elapsed_before: u32 = 0;
            let mut end_index = ring.observation_len();
            for index in 0..ring.observation_len() {
                if elapsed_before >= entry {
                    end_index = index;
                    break;
                }
                elapsed_before =
                    elapsed_before.saturating_add(ring.observation(index).unwrap().elapsed_us);
            }
            assert_eq!(
                online.recovery_iae_q(),
                recovery_iae_q(&ring, target, end_index)
            );
        }
    }

    #[test]
    fn online_accumulator_matches_pure_scanners_for_transient_then_settle() {
        let target = per_tick_rate_q(100, 1_000);
        let band = per_tick_rate_q(1, 1_000);
        let samples: [(i32, u32); 110] =
            core::array::from_fn(|i| (if i < 30 { 400 } else { 100 }, 1_000));
        assert_online_matches_pure(&samples, target, band, 48_000);
    }

    #[test]
    fn windowed_band_membership_admits_fast_ripple_and_rejects_slow_hunting() {
        let target = per_tick_rate_q(12, 1_000);
        let band = target * 5 / 100;
        let ripple: [(i32, u32); 200] =
            core::array::from_fn(|i| (if i % 2 == 0 { 10 } else { 14 }, 1_000));

        let mut raw = ReversalOnlineAccumulator::new();
        raw.reset_with_band_window(target, band, 48_000, 1);
        for &(delta, elapsed) in &ripple {
            raw.observe(delta, elapsed);
        }
        assert_eq!(raw.band_entry_us(), None);

        let mut windowed = ReversalOnlineAccumulator::new();
        windowed.reset_with_band_window(target, band, 48_000, PROBE_BAND_WINDOW_SAMPLES);
        for &(delta, elapsed) in &ripple {
            windowed.observe(delta, elapsed);
        }
        assert_eq!(
            windowed.band_entry_us(),
            Some((PROBE_BAND_WINDOW_SAMPLES as u32 - 1) * 1_000)
        );

        let hunting: [(i32, u32); 400] =
            core::array::from_fn(|i| (if (i / 50) % 2 == 0 { 10 } else { 14 }, 1_000));
        let mut slow = ReversalOnlineAccumulator::new();
        slow.reset_with_band_window(target, band, 48_000, PROBE_BAND_WINDOW_SAMPLES);
        for &(delta, elapsed) in &hunting {
            slow.observe(delta, elapsed);
        }
        assert_eq!(slow.band_entry_us(), None);

        let mut off_target = ReversalOnlineAccumulator::new();
        off_target.reset_with_band_window(target, band, 48_000, PROBE_BAND_WINDOW_SAMPLES);
        for _ in 0..200 {
            off_target.observe(10, 1_000);
        }
        assert_eq!(off_target.band_entry_us(), None);
    }

    #[test]
    fn online_accumulator_matches_pure_scanners_when_band_never_reached() {
        let target = per_tick_rate_q(100, 1_000);
        let band = per_tick_rate_q(1, 1_000);
        let samples: [(i32, u32); 20] =
            core::array::from_fn(|i| (if i % 2 == 0 { 100 } else { 400 }, 1_000));
        assert_online_matches_pure(&samples, target, band, 3_000);
        let mut online = ReversalOnlineAccumulator::new();
        online.reset_with_band_window(target, band, 3_000, 1);
        for &(delta, elapsed) in &samples {
            online.observe(delta, elapsed);
        }
        assert_eq!(online.band_entry_us(), None);
    }

    #[test]
    fn tripped_mask_flags_each_threshold_independently() {
        let t = RobustnessThresholds {
            t_max_us: 100_000,
            r_max_q: 500,
            iae_max_q: 1_000_000,
        };
        let ok = ReversalMetrics {
            reconvergence_time_us: 50_000,
            settled_residual_q: 100,
            recovery_iae_q: 500_000,
            dac_rms_q: 0,
            forward_settle_time_us: 20_000,
            reconvergence_ratio_ppm: 0,
        };
        assert_eq!(tripped_mask(&ok, &t), 0);
        let slow = ReversalMetrics {
            reconvergence_time_us: 200_000,
            ..ok
        };
        assert_eq!(tripped_mask(&slow, &t), TRIP_TIME);
        let bad = ReversalMetrics {
            reconvergence_time_us: 200_000,
            settled_residual_q: -900,
            recovery_iae_q: 2_000_000,
            ..ok
        };
        assert_eq!(tripped_mask(&bad, &t), TRIP_TIME | TRIP_RESIDUAL | TRIP_IAE);
    }

    #[test]
    fn residual_gate_uses_absolute_value() {
        let t = RobustnessThresholds {
            t_max_us: u32::MAX,
            r_max_q: 500,
            iae_max_q: i64::MAX,
        };
        let neg = ReversalMetrics {
            reconvergence_time_us: 0,
            settled_residual_q: -600,
            recovery_iae_q: 0,
            dac_rms_q: 0,
            forward_settle_time_us: 0,
            reconvergence_ratio_ppm: 0,
        };
        assert_eq!(tripped_mask(&neg, &t), TRIP_RESIDUAL);
    }

    #[test]
    fn worse_of_ors_both_directions() {
        assert_eq!(worse_of(TRIP_TIME, TRIP_IAE), TRIP_TIME | TRIP_IAE);
        assert_eq!(worse_of(0, 0), 0);
    }

    #[test]
    fn overshoot_is_zero_when_leg_tracks_target_rate_exactly() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(100, 1_000);
        online.reset_with_band_window(target, 0, 0, 1);
        for _ in 0..10 {
            online.observe(100, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    #[test]
    fn overshoot_registers_a_mid_transient_burst_even_after_settling_back() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(100, 1_000);
        online.reset_with_band_window(target, 0, 0, 1);
        for _ in 0..3 {
            online.observe(200, 1_000);
        }
        for _ in 0..10 {
            online.observe(100, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 300);
    }

    #[test]
    fn overshoot_stays_zero_when_leg_runs_behind_target_rate() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(100, 1_000);
        online.reset_with_band_window(target, 0, 0, 1);
        for _ in 0..10 {
            online.observe(50, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    #[test]
    fn overshoot_sign_symmetry_holds_for_a_reverse_target_burst() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(-100, 1_000);
        online.reset_with_band_window(target, 0, 0, 1);
        for _ in 0..3 {
            online.observe(-200, 1_000);
        }
        for _ in 0..10 {
            online.observe(-100, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 300);
    }

    #[test]
    fn overshoot_sign_symmetry_holds_for_a_reverse_target_running_behind() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(-100, 1_000);
        online.reset_with_band_window(target, 0, 0, 1);
        for _ in 0..10 {
            online.observe(-50, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    #[test]
    fn reset_clears_the_excess_tracker_for_a_fresh_leg() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(100, 1_000);
        online.reset_with_band_window(target, 0, 0, 1);
        for _ in 0..3 {
            online.observe(200, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 300);

        online.reset_with_band_window(target, 0, 0, 1);
        assert_eq!(
            online.reversal_overshoot_counts(),
            0,
            "reset() must clear peak_excess_q, not carry it into the new leg"
        );
        online.observe(100, 1_000);
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    #[test]
    fn t_max_is_a_live_range_coarse_catch() {
        assert_eq!(ROBUSTNESS_THRESHOLDS.t_max_us, 600_000);
        const { assert!(ROBUSTNESS_THRESHOLDS.t_max_us > 450_000) };
        const {
            assert!(
                ROBUSTNESS_THRESHOLDS.t_max_us
                    < ROBUSTNESS_MEASUREMENT_BUDGET_US - ROBUSTNESS_BAND_DWELL_US
            )
        };
    }

    #[test]
    fn iae_bound_is_independent_of_the_measurement_budget() {
        const LIVE_PLANT_RATE_Q: i64 = 16_912_328;
        const LIVE_TARGET_RATE_Q: i64 = 21_466_666;
        const LIVE_IAE_MAX_QS: i64 = 1_226_143;
        let thresholds = robustness_thresholds_for(LIVE_PLANT_RATE_Q, LIVE_TARGET_RATE_Q);
        assert_eq!(thresholds.iae_max_q / 1_000_000, LIVE_IAE_MAX_QS);
        const { assert!(ROBUSTNESS_MEASUREMENT_BUDGET_US != ROBUSTNESS_IAE_REFERENCE_WINDOW_US) };
    }

    #[test]
    fn tail_threshold_stays_dormant_above_budget() {
        const { assert!(2 * ROBUSTNESS_THRESHOLDS.t_max_us > ROBUSTNESS_MEASUREMENT_BUDGET_US) };
    }

    #[test]
    fn reduce_plant_rate_q_takes_worse_of_max_bound() {
        let intervals = [[-3, 5], [7, -2]];
        assert_eq!(reduce_plant_rate_q(&intervals), 7);
    }

    #[test]
    fn iae_max_q_scales_with_plant_rate_above_the_degenerate_cut_in() {
        let target = 11_733_333;
        let a = robustness_thresholds_for(1_000_000, target).iae_max_q;
        let b = robustness_thresholds_for(2_000_000, target).iae_max_q;
        assert!(a > 0 && b > a);
        assert!((b - 2 * a).abs() <= 1);
    }

    #[test]
    fn degenerate_or_near_zero_plant_fails_open_to_the_retired_seed() {
        let target = 11_733_333;
        let below_cut_in = target / (PLANT_RATE_DEGENERATE_MIN_DIVISOR * 2);
        for plant in [0_i64, -5, below_cut_in] {
            assert_eq!(
                robustness_thresholds_for(plant, target).iae_max_q,
                ROBUSTNESS_IAE_SEED_MAX_Q
            );
        }
    }

    #[test]
    fn calibrated_coefficient_accepts_reference_gains_and_rejects_the_weak_anchor() {
        use crate::commissioning::outer::velocity::robustness_reporting::IAE_WIRE_TO_INTERNAL_US_PER_S;
        const PLANT_RATE_Q: i64 = 16_300_855;
        const TARGET_RATE_Q: i64 = 11_733_333;
        let accept_ceiling_q = 774_513 * IAE_WIRE_TO_INTERNAL_US_PER_S;
        let reject_floor_q = 1_527_833 * IAE_WIRE_TO_INTERNAL_US_PER_S;

        let t = robustness_thresholds_for(PLANT_RATE_Q, TARGET_RATE_Q);
        assert!(t.iae_max_q > accept_ceiling_q);
        assert!(t.iae_max_q < reject_floor_q);
        assert!(t.iae_max_q >= accept_ceiling_q * 3 / 2);
        assert!(reject_floor_q * 4 / 5 >= t.iae_max_q);

        let accept = ReversalMetrics {
            reconvergence_time_us: 260_000,
            settled_residual_q: 0,
            recovery_iae_q: accept_ceiling_q,
            dac_rms_q: 0,
            forward_settle_time_us: 240_000,
            reconvergence_ratio_ppm: 0,
        };
        assert_eq!(tripped_mask(&accept, &t) & TRIP_IAE, 0);
        let reject = ReversalMetrics {
            recovery_iae_q: reject_floor_q,
            ..accept
        };
        assert_eq!(tripped_mask(&reject, &t) & TRIP_IAE, TRIP_IAE);
    }

    #[test]
    fn thresholds_for_keeps_time_and_residual_seeds() {
        let t = robustness_thresholds_for(5_000, 11_733_333);
        assert_eq!(t.t_max_us, ROBUSTNESS_THRESHOLDS.t_max_us);
        assert_eq!(t.r_max_q, ROBUSTNESS_THRESHOLDS.r_max_q);
    }

    use crate::autotune::velocity_measurement::RawVelocityObservation;

    fn observation(delta_counts: i32, elapsed_us: u32) -> RawVelocityObservation {
        RawVelocityObservation::new(delta_counts, elapsed_us).expect("non-zero elapsed_us")
    }

    #[test]
    fn dac_rms_q_is_zero_for_a_flat_window() {
        let target_rate_q = per_tick_rate_q(100, 1_000);
        let samples = [observation(100, 1_000); 8];
        assert_eq!(dac_rms_q(&samples[..], 0, samples.len(), target_rate_q), 0);
    }

    #[test]
    fn dac_rms_q_is_nonzero_for_alternating_error() {
        let target_rate_q = per_tick_rate_q(100, 1_000);
        let high = observation(110, 1_000);
        let low = observation(90, 1_000);
        let samples = [high, low, high, low, high, low, high, low];
        let d = per_tick_rate_q(110, 1_000) - target_rate_q;
        assert_eq!(
            dac_rms_q(&samples[..], 0, samples.len(), target_rate_q),
            2 * d
        );
    }

    #[test]
    fn dac_rms_q_reads_only_the_requested_window() {
        let target_rate_q = per_tick_rate_q(100, 1_000);
        let transient = observation(9_999, 1_000);
        let flat = observation(100, 1_000);
        let samples = [transient, transient, flat, flat, flat, flat];
        assert_eq!(dac_rms_q(&samples[..], 2, 4, target_rate_q), 0);
    }

    #[test]
    fn dac_rms_q_is_sign_symmetric_for_forward_and_reverse_targets() {
        let forward_target_q = per_tick_rate_q(100, 1_000);
        let reverse_target_q = per_tick_rate_q(-100, 1_000);
        let forward_samples = [
            observation(110, 1_000),
            observation(90, 1_000),
            observation(110, 1_000),
            observation(90, 1_000),
        ];
        let reverse_samples = [
            observation(-110, 1_000),
            observation(-90, 1_000),
            observation(-110, 1_000),
            observation(-90, 1_000),
        ];
        assert_eq!(
            dac_rms_q(&forward_samples[..], 0, 4, forward_target_q),
            dac_rms_q(&reverse_samples[..], 0, 4, reverse_target_q)
        );
    }

    #[test]
    fn dac_rms_q_does_not_panic_on_a_short_or_empty_range() {
        let target_rate_q = per_tick_rate_q(100, 1_000);
        let samples = [observation(100, 1_000)];
        assert_eq!(dac_rms_q(&samples[..], 0, 0, target_rate_q), 0);
        assert_eq!(dac_rms_q(&samples[..], 0, 1, target_rate_q), 0);
        assert_eq!(dac_rms_q(&samples[..], 0, 5, target_rate_q), 0);
    }

    #[test]
    fn dac_rms_q_matches_an_independently_computed_reference() {
        let target_rate_q = 0_i64;
        let elapsed_us = 1_000_u32;
        const DELTA_COUNTS: [i32; 5] = [2, 10, 4, 8, 6];
        let samples: [RawVelocityObservation; 5] = DELTA_COUNTS.map(|d| observation(d, elapsed_us));
        let k = per_tick_rate_q(1, elapsed_us) as f64;

        let errors: [f64; 5] = DELTA_COUNTS.map(|d| d as f64);
        let mean: f64 = errors.iter().sum::<f64>() / errors.len() as f64;
        let ac: [f64; 5] = errors.map(|e| e - mean);
        let mut sq_diff_sum = 0.0_f64;
        for i in 1..ac.len() {
            let diff = ac[i] - ac[i - 1];
            sq_diff_sum += diff * diff;
        }
        let mean_sq_diff = sq_diff_sum / (ac.len() - 1) as f64;
        let expected = mean_sq_diff.sqrt() * k;

        let actual = dac_rms_q(&samples[..], 0, samples.len(), target_rate_q) as f64;
        let tolerance = expected.abs() * 0.01;
        assert!(
            (actual - expected).abs() <= tolerance,
            "dac_rms_q={actual}, independent reference={expected}, tolerance={tolerance}"
        );
    }
}
