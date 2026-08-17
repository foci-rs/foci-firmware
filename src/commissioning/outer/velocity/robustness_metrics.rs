//! Pure derivation of reversal-robustness metrics from raw velocity observations.
//!
//! Every function here is a deterministic transform over `{delta_counts,
//! elapsed_us}` samples pulled from a `VelocityObservationSource` (the
//! commissioning outer-loop tail ring in production, `[RawVelocityObservation]`
//! or a stub in tests). Nothing here touches hardware, state machines, or the
//! stationarity suffix selector: band-entry timing and recovery IAE are
//! computed directly on the derived per-tick rate, so their outcome is
//! independent of which (if any) suffix the selector would have chosen.

use crate::autotune::velocity_measurement::VELOCITY_RATE_Q_SCALE;
#[cfg(test)]
use crate::autotune::velocity_measurement::VelocityObservationSource;

const MICROS_PER_SECOND: i128 = 1_000_000;

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
    /// Elapsed time from the initial (forward) approach to its own settle.
    pub forward_settle_time_us: u32,
    /// `reconvergence_time_us` expressed as parts-per-million of
    /// `forward_settle_time_us`. Populated by the calling phase, not here.
    pub reconvergence_ratio_ppm: u32,
}

/// Derive the per-tick rate, in the same rate-Q fixed-point convention as
/// `sweep_analysis::deficit_interval_q`'s settled-rate arguments
/// (`VELOCITY_RATE_Q_SCALE`, counts per second scaled by 1000).
///
/// Returns `0` for a degenerate zero-elapsed sample rather than dividing by
/// zero; a valid `VelocityObservationSource` never yields one.
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

/// Return the elapsed time (from the start of `obs`) at which a sustained
/// dwell inside `[target_rate_q - band_q, target_rate_q + band_q]` begins,
/// where the dwell lasts at least `dwell_us`.
///
/// Returns the elapsed-us at the *first sample of that dwell*, not the
/// sample where the dwell requirement is finally satisfied, so the result
/// reads as "how long after reversal onset did the rate settle". Returns
/// `None` if no such dwell exists anywhere in `obs` (the inconclusive
/// signal).
///
/// Reference scanner: production folds samples online via
/// [`ReversalOnlineAccumulator`]; this batch form exists only to validate that
/// accumulator against an intact sample stream, so it is `#[cfg(test)]`.
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

/// Integral of absolute per-tick rate error against `target_rate_q` over
/// `obs[0..end_index]`, weighted by each sample's elapsed time:
/// `sum(|rate_i - target_rate_q| * elapsed_us_i)`.
///
/// Reference scanner (see [`band_entry_time_us`]): `#[cfg(test)]`, used only to
/// validate [`ReversalOnlineAccumulator`] against an intact sample stream.
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

/// Online band-entry and recovery-IAE accumulator for one capture leg.
///
/// The [`VelocityObservationRing`](crate::commissioning::outer::state::VelocityObservationRing)
/// retains only its most recent 256 samples, so a capture that runs up to the
/// full [`ROBUSTNESS_MEASUREMENT_BUDGET_US`] budget cannot be scanned after the
/// fact -- the early transient has already been overwritten. This accumulator
/// therefore folds each observation in as it arrives, deriving exactly the same
/// band-entry time and recovery IAE that [`band_entry_time_us`] and
/// [`recovery_iae_q`] would compute over the intact sample stream. The ring is
/// left to serve only the settled tail (stationarity + residual).
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub(crate) struct ReversalOnlineAccumulator {
    target_rate_q: i64,
    lower_q: i64,
    upper_q: i64,
    dwell_us: u32,
    elapsed_before_us: u32,
    iae_running_q: i64,
    run_start_us: Option<u32>,
    run_iae_at_start_q: i64,
    run_duration_us: u32,
    band_entry_us: Option<u32>,
    band_entry_iae_q: i64,
    signed_position: i32,
    peak_excursion_counts: u32,
}

impl ReversalOnlineAccumulator {
    /// Construct an idle accumulator with a zero band around zero.
    pub(crate) const fn new() -> Self {
        Self {
            target_rate_q: 0,
            lower_q: 0,
            upper_q: 0,
            dwell_us: 0,
            elapsed_before_us: 0,
            iae_running_q: 0,
            run_start_us: None,
            run_iae_at_start_q: 0,
            run_duration_us: 0,
            band_entry_us: None,
            band_entry_iae_q: 0,
            signed_position: 0,
            peak_excursion_counts: 0,
        }
    }

    /// Re-arm the accumulator for a fresh leg targeting `target_rate_q` with a
    /// symmetric `band_q` half-width and a `dwell_us` sustained-residence
    /// requirement. Clears every running band-entry/IAE counter, but leaves
    /// the peak-excursion tracking untouched: `reset` is called once per
    /// capture (establish and reverse legs both call it), while the peak
    /// spans both legs of one cycle. Use [`Self::reset_cycle_peak`] to clear
    /// the peak between cycles.
    pub(crate) fn reset(&mut self, target_rate_q: i64, band_q: i64, dwell_us: u32) {
        self.target_rate_q = target_rate_q;
        self.lower_q = target_rate_q.saturating_sub(band_q);
        self.upper_q = target_rate_q.saturating_add(band_q);
        self.dwell_us = dwell_us;
        self.elapsed_before_us = 0;
        self.iae_running_q = 0;
        self.run_start_us = None;
        self.run_iae_at_start_q = 0;
        self.run_duration_us = 0;
        self.band_entry_us = None;
        self.band_entry_iae_q = 0;
    }

    /// Clear the peak-excursion tracking for a new cycle. Separate from
    /// [`Self::reset`], which runs once per capture (twice per cycle) and
    /// must not disturb a peak that spans both the establish and reverse
    /// legs.
    pub(crate) fn reset_cycle_peak(&mut self) {
        self.signed_position = 0;
        self.peak_excursion_counts = 0;
    }

    /// Fold one observation into the running band-entry and IAE state.
    ///
    /// Mirrors [`band_entry_time_us`]/[`recovery_iae_q`] exactly: band entry is
    /// recorded at the *first* sample of the dwell that first satisfies
    /// `dwell_us`, and the recovery IAE integrates `|rate - target| * elapsed`
    /// over every sample strictly before that dwell start. Once band entry is
    /// recorded, further observations are ignored so the IAE stays frozen at
    /// the transient it measured.
    pub(crate) fn observe(&mut self, delta_counts: i32, elapsed_us: u32) {
        self.signed_position = self.signed_position.saturating_add(delta_counts);
        self.peak_excursion_counts = self
            .peak_excursion_counts
            .max(self.signed_position.unsigned_abs());
        if self.band_entry_us.is_some() {
            return;
        }
        let rate = per_tick_rate_q(delta_counts, elapsed_us);
        if rate >= self.lower_q && rate <= self.upper_q {
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

    /// Peak absolute local excursion (`|signed_position|`) observed since the
    /// last [`Self::reset_cycle_peak`]. Spans the establish and reverse
    /// captures of one cycle -- [`Self::reset`] runs once per capture and
    /// does not clear it -- while the zero-target rest window between
    /// cycles is excluded: its braking travel belongs to the settle
    /// reserve, not the reversal transient this metric measures.
    pub(crate) fn peak_excursion_counts(&self) -> u32 {
        self.peak_excursion_counts
    }
}

/// Absolute per-direction gate thresholds for one reversal-robustness probe.
///
/// Compared directly against `ReversalMetrics`, never against another
/// candidate's own measurement, so the gate cannot be gamed by an
/// over-integrated candidate that is merely "less bad" than its peers.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub(crate) struct RobustnessThresholds {
    /// Maximum allowed `reconvergence_time_us`.
    pub t_max_us: u32,
    /// Maximum allowed `|settled_residual_q|`.
    pub r_max_q: i64,
    /// Maximum allowed `recovery_iae_q`.
    pub iae_max_q: i64,
}

/// Conservative, pre-characterization seed thresholds. Deliberately loose:
/// the gate must never false-reject a good candidate before the hardware
/// characterization pass tightens these values. The gate compares the
/// per-direction median of each metric against these bounds, so a single
/// tail cycle cannot by itself trip a threshold.
///
/// `r_max_q` is the deficit-Q equivalent of 10% of a reference acceptance
/// target rate (176 rpm at `REFERENCE_ENCODER_COUNTS_PER_REV` = 4000 counts
/// per rev, `commissioning/outer/limits.rs`), computed offline against
/// `VELOCITY_RATE_Q_SCALE` and frozen as a literal -- never recomputed at
/// runtime, which would silently reintroduce a self-relative gate:
///
/// ```text
/// target_rate_q = target_velocity_rpm * counts_per_rev * VELOCITY_RATE_Q_SCALE / 60
///               = 176 * 4_000 * 1_000 / 60
///               = 704_000_000 / 60
///               = 11_733_333 (truncated)
/// r_max_q = target_rate_q * 10 / 100 = 1_173_333 (truncated)
/// ```
///
/// `iae_max_q` is a loose *transient* envelope, not a settled-residual one:
/// the recovery window integrates the reversal transient, whose error starts
/// near twice the target rate. The seed allows a 2x margin over that peak
/// error (`2 * target_rate_q`) sustained across the full measurement budget
/// (`ROBUSTNESS_MEASUREMENT_BUDGET_US`), in the same rate-Q-microsecond unit
/// as the internal `recovery_iae_q` the gate compares:
///
/// ```text
/// iae_max_q = 2 (margin) * (2 * target_rate_q) * ROBUSTNESS_MEASUREMENT_BUDGET_US
///           = 2 * (2 * 11_733_333) * 500_000
///           = 23_466_666_000_000 (rate-Q microseconds)
/// ```
pub(crate) const ROBUSTNESS_THRESHOLDS: RobustnessThresholds = RobustnessThresholds {
    // Loose seed, pending hardware characterization.
    t_max_us: 300_000,
    // Loose seed (10% of target), pending hardware characterization.
    r_max_q: 1_173_333,
    // Loose seed: 2x margin over the full-budget peak-error transient,
    // pending hardware characterization.
    iae_max_q: 2 * (2 * 11_733_333) * 500_000,
};

/// Bit set in `tripped_mask` when `reconvergence_time_us` exceeds `t_max_us`.
pub(crate) const TRIP_TIME: u8 = 1 << 0;
/// Bit set in `tripped_mask` when `|settled_residual_q|` exceeds `r_max_q`.
pub(crate) const TRIP_RESIDUAL: u8 = 1 << 1;
/// Bit set in `tripped_mask` when `recovery_iae_q` exceeds `iae_max_q`.
pub(crate) const TRIP_IAE: u8 = 1 << 2;

/// Settle-band half-width, as a percentage of `|target_rate_q|`. The phase
/// derives `band_q = |target_rate_q| * ROBUSTNESS_BAND_FRACTION_PCT / 100`
/// at runtime. This is a settle-detection tolerance proportional to target,
/// not a self-relative gate -- the ratio concern is confined to `t_max_us`,
/// which stays absolute.
pub(crate) const ROBUSTNESS_BAND_FRACTION_PCT: u32 = 5;

/// Minimum sustained dwell inside the settle band, in microseconds.
/// ~48 ticks at the ~1 ms observation cadence (416 samples over 415,488 us
/// in the scored-rest captures, `docs/logbook/2026-07-28.md`), under the
/// 64-sample shortest stationary suffix (`VELOCITY_SUFFIX_LENGTHS[0]`).
pub(crate) const ROBUSTNESS_BAND_DWELL_US: u32 = 48_000;

/// Overall measurement budget for one reversal-robustness probe leg, in
/// microseconds. Matches the existing 500 ms zero-target rest window
/// budget; must exceed `t_max_us`. A leg that does not reach band entry
/// within this budget is inconclusive.
pub(crate) const ROBUSTNESS_MEASUREMENT_BUDGET_US: u32 = 500_000;

/// Bounded retry count for an inconclusive probe leg.
pub(crate) const ROBUSTNESS_MAX_RETRIES: u8 = 2;

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

/// Stroke-relative overshoot, in counts: `peak_excursion` saturating-minus
/// `stroke_counts`, `0` when the peak stayed within the per-primitive
/// stroke allocation. Evidence-only -- no threshold gates this value.
pub(crate) fn overshoot_counts(peak_excursion: u32, stroke_counts: u32) -> u16 {
    let overshoot = peak_excursion.saturating_sub(stroke_counts);
    u16::try_from(overshoot).unwrap_or(u16::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::outer::state::VelocityObservationRing;

    #[test]
    fn band_entry_reports_first_stable_dwell_and_ignores_earlier_excursions() {
        // 20 ticks at 1000us: first 5 far from target, then within band for the rest.
        let mut ring = VelocityObservationRing::new();
        let target = per_tick_rate_q(100, 1_000); // rate for 100 counts/1ms
        for _ in 0..5 {
            ring.push(400, 1_000).unwrap();
        } // way over
        for _ in 0..15 {
            ring.push(100, 1_000).unwrap();
        } // on target
        // band tolerant of 1 count, dwell 3ms
        let band = per_tick_rate_q(1, 1_000);
        let t = band_entry_time_us(&ring, target, band, 3_000).expect("enters band");
        assert_eq!(t, 5_000); // reversal-to-band-entry = 5 ticks * 1000us
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
        ring.push(120, 1_000).unwrap(); // +20 counts error
        ring.push(80, 1_000).unwrap(); // -20 counts error
        let iae = recovery_iae_q(&ring, target, 2);
        let per_err = (per_tick_rate_q(20, 1_000)).abs() * 1_000;
        assert_eq!(iae, per_err * 2);
    }

    #[test]
    fn band_entry_ignores_the_stationarity_suffix_selector() {
        // A ring the suffix selector would exclude (a large early excursion
        // dominates the long suffixes) but whose derived-rate band entry is
        // clean once settled. band_entry_time_us takes only the ring + target +
        // band + dwell -- no StationarityWork/suffix input -- so the selector
        // outcome structurally cannot affect the reconvergence-time metric.
        let mut ring = VelocityObservationRing::new();
        let target = per_tick_rate_q(100, 1_000);
        for _ in 0..8 {
            ring.push(500, 1_000).unwrap();
        } // large early excursion
        for _ in 0..64 {
            ring.push(100, 1_000).unwrap();
        } // clean band residence
        let band = per_tick_rate_q(1, 1_000);
        assert!(band_entry_time_us(&ring, target, band, 3_000).is_some());
    }

    /// Feed identical sample streams through the online accumulator and the pure
    /// scanners; the accumulator must reproduce band-entry time and recovery IAE
    /// exactly, since production folds samples online (the 256-sample ring
    /// cannot retain a full 500 ms transient).
    fn assert_online_matches_pure(samples: &[(i32, u32)], target: i64, band: i64, dwell_us: u32) {
        let mut ring = VelocityObservationRing::new();
        let mut online = ReversalOnlineAccumulator::new();
        online.reset(target, band, dwell_us);
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
        // 30 ticks far out of band, then a sustained on-target dwell.
        let samples: [(i32, u32); 110] =
            core::array::from_fn(|i| (if i < 30 { 400 } else { 100 }, 1_000));
        assert_online_matches_pure(&samples, target, band, 48_000);
    }

    #[test]
    fn online_accumulator_matches_pure_scanners_when_band_never_reached() {
        let target = per_tick_rate_q(100, 1_000);
        let band = per_tick_rate_q(1, 1_000);
        let samples: [(i32, u32); 20] =
            core::array::from_fn(|i| (if i % 2 == 0 { 100 } else { 400 }, 1_000));
        assert_online_matches_pure(&samples, target, band, 3_000);
        let mut online = ReversalOnlineAccumulator::new();
        online.reset(target, band, 3_000);
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
    fn accumulator_tracks_peak_absolute_excursion() {
        let mut online = ReversalOnlineAccumulator::new();
        online.reset(0, 0, 0);
        for _ in 0..5 {
            online.observe(100, 1_000);
        }
        for _ in 0..2 {
            online.observe(-30, 1_000);
        }
        assert_eq!(online.peak_excursion_counts(), 500);
    }

    #[test]
    fn overshoot_is_zero_within_stroke_and_saturates_beyond() {
        assert_eq!(overshoot_counts(11_990, 12_000), 0);
        assert_eq!(overshoot_counts(12_018, 12_000), 18);
        assert_eq!(overshoot_counts(u32::MAX, 0), u16::MAX);
    }

    #[test]
    fn capture_reset_preserves_the_peak_across_legs() {
        let mut online = ReversalOnlineAccumulator::new();
        online.reset(0, 0, 0);
        for _ in 0..5 {
            online.observe(100, 1_000);
        }
        assert_eq!(online.peak_excursion_counts(), 500);
        online.reset(0, 0, 0);
        online.observe(-10, 1_000);
        assert_eq!(online.peak_excursion_counts(), 500);
    }

    #[test]
    fn reset_cycle_peak_clears_it() {
        let mut online = ReversalOnlineAccumulator::new();
        online.reset(0, 0, 0);
        for _ in 0..5 {
            online.observe(100, 1_000);
        }
        assert_eq!(online.peak_excursion_counts(), 500);
        online.reset_cycle_peak();
        assert_eq!(online.peak_excursion_counts(), 0);
        online.observe(10, 1_000);
        assert_eq!(online.peak_excursion_counts(), 10);
    }
}
