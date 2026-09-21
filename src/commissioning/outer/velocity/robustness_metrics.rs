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
use crate::autotune::velocity_measurement::VelocityObservationSource;

const MICROS_PER_SECOND: i128 = 1_000_000;

/// `SCALE` in the spec's Algorithm section: multiplying `target_rate_q *
/// elapsed_us` for one tick yields `expected_delta_counts * SCALE`, by the
/// same identity `per_tick_rate_q` divides out. See
/// `docs/specs/2026-08-22-foc-274-overshoot-metric-redesign.md`, "Units".
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
    /// Demeaned first-difference RMS of settled per-tick rate error, in raw
    /// rate-Q units (not IAE's rate-Q-seconds). Meaningful only when this
    /// leg reached the settled tail; `0` otherwise.
    pub dac_rms_q: i64,
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

/// Demeaned first-difference RMS of the per-tick rate error over
/// `[start_index, start_index + len)` of `source`.
///
/// Mirrors `docs/validation/foc-250-2026-08-18-p-sweep-stepper-x/analyze_sweep.py`'s
/// `over_gain_metrics`: demean the window's rate error against `target_rate_q`,
/// then take the RMS of the demeaned series' first difference. Missing
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

    /// Re-arm the accumulator for a fresh leg targeting `target_rate_q` with a
    /// symmetric `band_q` half-width and a `dwell_us` sustained-residence
    /// requirement. Also clears the leg-scoped overshoot-excess tracking
    /// (`actual_position_counts`, `expected_position_q`, `peak_excess_q`):
    /// unlike the old whole-cycle peak this design replaces, the excess
    /// tracker is scoped to one leg, so the reverse leg's own `reset()` call
    /// is the reversal-command instant it anchors at. See
    /// `docs/specs/2026-08-22-foc-274-overshoot-metric-redesign.md`.
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
        self.actual_position_counts = 0;
        self.expected_position_q = 0;
        self.peak_excess_q = 0;
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

    /// Peak positive excess of actual displacement over the ideal
    /// constant-target-rate trajectory, observed since the last `reset()`
    /// -- i.e. scoped to one leg. Zero for a leg that never ran ahead of its
    /// commanded target rate. See
    /// `docs/specs/2026-08-22-foc-274-overshoot-metric-redesign.md`.
    pub(crate) fn reversal_overshoot_counts(&self) -> u16 {
        u16::try_from(self.peak_excess_q / OVERSHOOT_SCALE_Q).unwrap_or(u16::MAX)
    }

    /// Whole-leg mean velocity-to-target ratio in percent: accumulated signed
    /// displacement as a fraction of the ideal constant-target-rate
    /// displacement. Around 100 tracked the commanded rate; below 100 ran
    /// behind (sat below the band); above 100 with a nonzero
    /// [`Self::reversal_overshoot_counts`] ran ahead (hunted across). Zero
    /// before any target displacement accrues.
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
/// Structural censoring: a conclusive tail cycle cannot be measured while the
/// tail threshold (`2 * t_max` = 860 ms) exceeds the measurement budget
/// (`ROBUSTNESS_MEASUREMENT_BUDGET_US` = 500 ms) -- band entry past the budget
/// reads as inconclusive, never as a slow conclusive cycle. The tail-reject
/// path is therefore dormant until the tail threshold is pinned below the
/// budget.
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
    // Reconvergence time becomes a coarse live-range catch, above the observed
    // reference conclusive max and below the budget-minus-dwell censoring point;
    // beyond-budget slowness surfaces as inconclusive, and the tail path stays
    // dormant above the measurement budget.
    t_max_us: 430_000,
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

/// Bounded retry count for an inconclusive probe leg. Band entry is stochastic
/// and occasionally misses within a single leg's budget; each retry re-runs a
/// fresh leg within that same budget, so a slow cycle gets four independent
/// attempts before its slot is consumed inconclusive. Prefer more retries over a
/// longer per-leg budget: extending the budget lengthens each leg's spin and
/// risks tripping the outer-safety travel envelope, whereas a retry does not.
pub(crate) const ROBUSTNESS_MAX_RETRIES: u8 = 4;

/// Number of `ROBUSTNESS_MEASUREMENT_BUDGET_US`-sized legs run per probe
/// cycle: the establish settle (`begin_capture` in the establish state), the
/// reversal measurement (`begin_capture` again in the reverse state), the
/// between-legs zero-target rest window `open_zero_window` opens, and the one
/// bounded re-opened rest window a rejected rest is granted before the run
/// faults. All four legs in `robustness_reversal` budget their deadline off
/// this same constant, so the worst-case per-cycle envelope is this leg count
/// times `ROBUSTNESS_MEASUREMENT_BUDGET_US`.
///
/// The per-leg settle-analysis window (`SETTLE_ANALYSIS_BUDGET_US`) is
/// deliberately not charged: it is a bounded post-capture analysis, not a
/// motion budget, and this envelope bounds motion time. Folding it in would
/// still fit the u16 field but overstate the moving envelope.
pub(crate) const ROBUSTNESS_LEGS_PER_CYCLE: u32 = 4;

/// Near-zero/degenerate cut-in divisor for [`robustness_thresholds_for`]: a
/// `plant_rate_q` below `target_rate_q / PLANT_RATE_DEGENERATE_MIN_DIVISOR`
/// (a documented small fraction of target, ~0.4%) fails open to
/// [`ROBUSTNESS_IAE_SEED_MAX_Q`] instead of scaling the coefficient down
/// toward a reject-everything threshold.
pub(crate) const PLANT_RATE_DEGENERATE_MIN_DIVISOR: i64 = 256;

/// The retired loose IAE seed (see [`ROBUSTNESS_THRESHOLDS`]'s doc comment
/// for its derivation). Kept as both the near-zero/degenerate fail-open
/// value and the dormant-equivalent anchor that
/// [`ROBUSTNESS_IAE_COEFF_NUM`]/[`ROBUSTNESS_IAE_COEFF_DEN`] are calibrated
/// against.
pub(crate) const ROBUSTNESS_IAE_SEED_MAX_Q: i64 = 23_466_666_000_000;

/// Calibration coefficient (numerator) for [`robustness_thresholds_for`]'s
/// `iae_max_q`: `iae_max_q = plant_rate_q * NUM * BUDGET_US / DEN`. Split as a
/// rational so the calibration is exact.
///
/// Candidate-independence invariant: the `plant_rate_q` this scales must derive
/// from breakaway plant identification, never a candidate's own reversal
/// measurement, or a candidate could loosen its own gate by degrading its
/// measured response. See [`RobustnessThresholds`] for the same invariant.
pub(crate) const ROBUSTNESS_IAE_COEFF_NUM: i64 = 29;
/// Calibration coefficient (denominator). See [`ROBUSTNESS_IAE_COEFF_NUM`].
pub(crate) const ROBUSTNESS_IAE_COEFF_DEN: i64 = 200;

/// Reduce the two reversal-direction response intervals from the accepted
/// point's identified settled-rate response into one worse-of plant rate,
/// in rate-Q units: per direction `max(|low|, |high|)`, pooled across the
/// two directions with worse-of `max`.
pub(crate) fn reduce_plant_rate_q(intervals: &[[i32; 2]; 2]) -> i64 {
    let dir0 = intervals[0][0]
        .unsigned_abs()
        .max(intervals[0][1].unsigned_abs());
    let dir1 = intervals[1][0]
        .unsigned_abs()
        .max(intervals[1][1].unsigned_abs());
    i64::from(dir0.max(dir1))
}

/// Construct plant-adaptive robustness thresholds from the accepted point's
/// identified settled-rate response (`plant_rate_q`, see
/// [`reduce_plant_rate_q`]) and the reference target rate. `t_max_us` and
/// `r_max_q` are carried over unchanged from [`ROBUSTNESS_THRESHOLDS`]; only
/// `iae_max_q` adapts to the identified plant.
///
/// Candidate-independence invariant: `plant_rate_q` must derive from
/// breakaway plant identification, never a candidate's own measurement --
/// see [`ROBUSTNESS_IAE_COEFF_NUM`].
pub(crate) fn robustness_thresholds_for(
    plant_rate_q: i64,
    target_rate_q: i64,
) -> RobustnessThresholds {
    let iae_max_q = if plant_rate_q < target_rate_q / PLANT_RATE_DEGENERATE_MIN_DIVISOR {
        ROBUSTNESS_IAE_SEED_MAX_Q
    } else {
        let scaled = i128::from(ROBUSTNESS_IAE_COEFF_NUM)
            .saturating_mul(i128::from(plant_rate_q))
            .saturating_mul(i128::from(ROBUSTNESS_MEASUREMENT_BUDGET_US))
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
        let target = per_tick_rate_q(100, 1_000); // 100 counts per 1ms tick
        online.reset(target, 0, 0);
        for _ in 0..10 {
            online.observe(100, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    #[test]
    fn overshoot_registers_a_mid_transient_burst_even_after_settling_back() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(100, 1_000);
        online.reset(target, 0, 0);
        for _ in 0..3 {
            // 2x target rate for 3 ticks: +100 counts of excess per tick.
            online.observe(200, 1_000);
        }
        for _ in 0..10 {
            // Settles back onto the target rate exactly -- this does not erase
            // the excess already reached, it only stops it from growing
            // further, since actual and expected now advance in lockstep. This
            // is the case a single-point-at-band-entry comparison would miss.
            online.observe(100, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 300);
    }

    #[test]
    fn overshoot_stays_zero_when_leg_runs_behind_target_rate() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(100, 1_000);
        online.reset(target, 0, 0);
        for _ in 0..10 {
            // Half the target rate, never ahead of the ideal trajectory.
            online.observe(50, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    /// Reverse-target (negative `target_rate_q`) counterparts of the three
    /// cases above. `overshoot_is_zero_when_leg_tracks_target_rate_exactly`'s
    /// negative counterpart is deliberately omitted: with actual and expected
    /// exactly equal, `excess_q` is `0` before the sign projection is even
    /// applied, so that case cannot distinguish a correct sign projection from
    /// an omitted or broken one at any target sign -- it is structurally
    /// incapable of catching a sign bug, so a negative-target duplicate would
    /// add no coverage. The burst and behind-target cases below are not
    /// redundant with that reasoning: the sign projection is a no-op for a
    /// positive target (`signum() == 1`), so the *positive*-target burst/behind
    /// tests above cannot distinguish "sign applied" from "sign omitted"
    /// either -- only a *negative*-target case exercises a projection that
    /// actually changes the sign of `excess_q` before clamping. A sign-omitted
    /// implementation would report `300` here as `0`
    /// (`overshoot_sign_symmetry_burst_...`) and `0` here as a spurious nonzero
    /// value (`overshoot_sign_symmetry_behind_...`) -- so both are needed to
    /// close the gap the positive-only tests leave open.
    #[test]
    fn overshoot_sign_symmetry_holds_for_a_reverse_target_burst() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(-100, 1_000);
        online.reset(target, 0, 0);
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
        online.reset(target, 0, 0);
        for _ in 0..10 {
            // Half the target magnitude, same (negative) direction -- never
            // ahead of the ideal trajectory. A sign-omitted implementation
            // computes `(actual*SCALE - expected_q)`, which is positive here
            // (actual is *less* negative than expected) and would wrongly clamp
            // to a nonzero value instead of 0.
            online.observe(-50, 1_000);
        }
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    /// The excess tracker is leg-scoped: `reset()` must clear it, unlike the
    /// old whole-cycle peak this design replaces (which was cleared only once
    /// per cycle, by the now-deleted `reset_cycle_peak()`). Without this test,
    /// a future change that accidentally drops the three new clearing lines
    /// from `reset()` would leave every other test in this file passing (they
    /// each call `reset()` only once, at the start), while production code
    /// would silently carry a burst's excess across into the next leg.
    #[test]
    fn reset_clears_the_excess_tracker_for_a_fresh_leg() {
        let mut online = ReversalOnlineAccumulator::new();
        let target = per_tick_rate_q(100, 1_000);
        online.reset(target, 0, 0);
        for _ in 0..3 {
            online.observe(200, 1_000); // burst: reaches overshoot 300
        }
        assert_eq!(online.reversal_overshoot_counts(), 300);

        online.reset(target, 0, 0); // new leg
        assert_eq!(
            online.reversal_overshoot_counts(),
            0,
            "reset() must clear peak_excess_q, not carry it into the new leg"
        );
        online.observe(100, 1_000); // exactly on target: only this leg's sample counts
        assert_eq!(online.reversal_overshoot_counts(), 0);
    }

    #[test]
    fn t_max_is_a_live_range_coarse_catch() {
        // Above the observed reference conclusive max (388 ms) and below the
        // ~452 ms conclusive-time censoring point (budget 500 ms - dwell 48 ms):
        // a conclusive leg slower than the budget cannot reach this threshold and
        // reads as inconclusive instead, so the catch is deliberately coarse.
        assert_eq!(ROBUSTNESS_THRESHOLDS.t_max_us, 430_000);
        const { assert!(ROBUSTNESS_THRESHOLDS.t_max_us > 388_000) };
        const {
            assert!(
                ROBUSTNESS_THRESHOLDS.t_max_us
                    < ROBUSTNESS_MEASUREMENT_BUDGET_US - ROBUSTNESS_BAND_DWELL_US
            )
        };
    }

    #[test]
    fn tail_threshold_stays_dormant_above_budget() {
        // 2 * t_max exceeds the measurement budget, so a conclusive tail cycle
        // cannot be measured (band entry past budget reads inconclusive). The
        // cause-10 tail-latch path stays dormant, unchanged by the t_max move.
        const { assert!(2 * ROBUSTNESS_THRESHOLDS.t_max_us > ROBUSTNESS_MEASUREMENT_BUDGET_US) };
    }

    #[test]
    fn reduce_plant_rate_q_takes_worse_of_max_bound() {
        // dir0 bounds |-3|,|5| -> 5 ; dir1 bounds |7|,|-2| -> 7 ; worse-of -> 7
        let intervals = [[-3, 5], [7, -2]];
        assert_eq!(reduce_plant_rate_q(&intervals), 7);
    }

    #[test]
    fn iae_max_q_scales_with_plant_rate_above_the_degenerate_cut_in() {
        let target = 11_733_333;
        // Both plant rates are well above the near-zero cut-in (target/256), so
        // both take the coefficient path rather than the fail-open seed.
        let a = robustness_thresholds_for(1_000_000, target).iae_max_q;
        let b = robustness_thresholds_for(2_000_000, target).iae_max_q;
        assert!(a > 0 && b > a);
        assert!((b - 2 * a).abs() <= 1); // ~linear in plant_rate_q (integer rounding)
    }

    #[test]
    fn degenerate_or_near_zero_plant_fails_open_to_the_retired_seed() {
        // <= 0 AND any positive value below the near-zero cut-in (target/256) must
        // fail open to the retired seed, independent of the coefficient -- never a
        // reject-everything iae_max_q ~ 0.
        let target = 11_733_333;
        let below_cut_in = target / (PLANT_RATE_DEGENERATE_MIN_DIVISOR * 2); // target/512
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
        // Measured accept ceiling and reject floor (median recovery IAE, in
        // wire rate-Q-seconds); the gate compares in internal
        // rate-Q-microseconds, hence the scale.
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
        // Error alternates +d/-d around the target every tick: after demeaning
        // (mean 0 for an even-length alternating series), each first difference
        // has magnitude 2d, so dac_rms_q = 2d exactly.
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
        // Range extends past what the source has -- observation() returns None
        // for the missing indices; must not panic, must not count them.
        assert_eq!(dac_rms_q(&samples[..], 0, 5, target_rate_q), 0);
    }

    /// Cross-checks `dac_rms_q` against `analyze_sweep.py`'s `over_gain_metrics`
    /// definition via a genuinely independent reference: a non-alternating,
    /// non-symmetric error sequence run through a separate demean ->
    /// first-difference -> RMS calculation, compared against `dac_rms_q`'s
    /// output. A 1% relative tolerance absorbs `f32`-vs-`f64` rounding at
    /// realistic rate-Q magnitudes without masking a materially wrong formula.
    #[test]
    fn dac_rms_q_matches_an_independently_computed_reference() {
        let target_rate_q = 0_i64;
        let elapsed_us = 1_000_u32;
        // Non-alternating, non-symmetric-mean error sequence (in units of K =
        // per_tick_rate_q(1, elapsed_us)): [2, 10, 4, 8, 6].
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
        // 1% relative tolerance: generous enough to absorb f32-vs-f64 rounding
        // at this magnitude, tight enough to still catch a materially wrong
        // formula (a missing demean step, wrong denominator, etc. would miss
        // by far more than 1%).
        let tolerance = expected.abs() * 0.01;
        assert!(
            (actual - expected).abs() <= tolerance,
            "dac_rms_q={actual}, independent reference={expected}, tolerance={tolerance}"
        );
    }
}
