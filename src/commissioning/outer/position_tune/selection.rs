use foci_tmc4671::pid::PidPGain;

/// Step to the next half-octave rung: `raw * 181 / 128` (`181/128 ~=
/// 1.4141`, the closest 8-bit-denominator rational approximation of
/// `sqrt(2)`), matching the outer velocity autotune's own rung convention.
/// Returns `None` when the next step would exceed `PidPGain::MAX`.
pub fn next_rung(current: PidPGain) -> Option<PidPGain> {
    let scaled = u32::from(current.raw()).checked_mul(181)? / 128;
    // Guarantee forward progress even at very small raw values, where
    // integer rounding could otherwise repeat the same rung.
    let next_raw = scaled.max(u32::from(current.raw()) + 1);
    u16::try_from(next_raw)
        .ok()
        .and_then(|raw| PidPGain::new(raw).ok())
}

/// One rung's full evaluated evidence, carried by `RungResolution::Accept`
/// so Task 6's reporting layer has the lag and ceiling-ratio evidence
/// without needing to re-derive it from the raw captures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RungOutcome {
    pub position_p: PidPGain,
    /// Time-domain tracking lag in microseconds (see this module's
    /// Interfaces note on the required conversion from the raw
    /// position-error capture).
    pub lag_us: u32,
    /// The accepted rung's 5 per-capture ceiling ratios (milli-ratios, see
    /// `ceiling::ceiling_trips`), kept for the reported diagnostic median.
    pub ceiling_ratios_milli: [i64; 5],
}

/// Index (0..3) of the ramped-stroke repeat whose `lag_us` is the median of
/// three repeats captured at one rung. A tie between two repeats' `lag_us`
/// resolves to whichever was captured first (the lower index).
pub fn median_repeat_index(lags: [u32; 3]) -> usize {
    let mut sorted = lags;
    sorted.sort_unstable();
    let median_value = sorted[1];
    lags.iter()
        .position(|&lag| lag == median_value)
        .expect("the median of three values is always one of the three")
}

/// True when `current_lag_us` should replace `running_best_lag_us` as the
/// walk's running-best-seen rung: strict improvement only -- an exact tie
/// (`==`) must NOT replace the running best.
pub fn is_new_running_best(current_lag_us: u32, running_best_lag_us: Option<u32>) -> bool {
    match running_best_lag_us {
        None => true,
        Some(best_lag_us) => current_lag_us < best_lag_us,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn median_repeat_index_picks_the_middle_value() {
        assert_eq!(median_repeat_index([1_000, 500, 1_500]), 0);
    }

    #[test]
    fn median_repeat_index_breaks_exact_ties_toward_the_earlier_repeat() {
        assert_eq!(median_repeat_index([1_000, 1_500, 1_000]), 0);
        assert_eq!(median_repeat_index([500, 1_000, 1_000]), 1);
    }

    #[test]
    fn next_rung_steps_by_half_octave() {
        let rung = PidPGain::new(100).unwrap();
        let next = next_rung(rung).expect("representable next rung");
        // Half-octave: multiply by 2^0.5 ~= 1.414 -- integer approximation.
        assert!(next.raw() > 100 && next.raw() <= 150, "got {}", next.raw());
    }

    #[test]
    fn next_rung_returns_none_past_max() {
        let rung = PidPGain::new(0x7000).unwrap();
        // Repeatedly stepping must eventually saturate to None rather than
        // overflow or wrap.
        let mut current = rung;
        let mut steps = 0;
        while let Some(next) = next_rung(current) {
            current = next;
            steps += 1;
            assert!(steps < 20, "did not terminate within 20 half-octave steps");
        }
    }

    #[test]
    fn is_new_running_best_accepts_the_first_rung_unconditionally() {
        assert!(is_new_running_best(999_999, None));
    }

    #[test]
    fn is_new_running_best_requires_strict_improvement() {
        assert!(is_new_running_best(900, Some(1_000)));
        assert!(!is_new_running_best(1_000, Some(1_000)));
        assert!(!is_new_running_best(1_100, Some(1_000)));
    }
}
