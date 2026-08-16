//! Pure derivation of reversal-robustness metrics from raw velocity observations.
//!
//! Every function here is a deterministic transform over `{delta_counts,
//! elapsed_us}` samples pulled from a `VelocityObservationSource` (the
//! commissioning outer-loop tail ring in production, `[RawVelocityObservation]`
//! or a stub in tests). Nothing here touches hardware, state machines, or the
//! stationarity suffix selector: band-entry timing and recovery IAE are
//! computed directly on the derived per-tick rate, so their outcome is
//! independent of which (if any) suffix the selector would have chosen.

use crate::autotune::velocity_measurement::{VELOCITY_RATE_Q_SCALE, VelocityObservationSource};

#[allow(dead_code)]
const MICROS_PER_SECOND: i128 = 1_000_000;

/// Reconvergence and recovery metrics for one reversal-robustness probe.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
#[allow(dead_code)]
pub(crate) struct ReversalMetrics {
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
#[allow(dead_code)]
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
#[allow(dead_code)]
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
#[allow(dead_code)]
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
}
