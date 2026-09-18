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

/// True when `current_lag_us` improves on `previous_lag_us` by less than
/// 5% -- the floor-confirming plateau stop condition (spec, Selection).
pub fn is_lag_plateau(previous_lag_us: u32, current_lag_us: u32) -> bool {
    if current_lag_us >= previous_lag_us {
        return true; // no improvement at all is certainly a plateau
    }
    let improvement = previous_lag_us - current_lag_us;
    // improvement / previous < 5%  <=>  improvement * 20 < previous
    improvement.saturating_mul(20) < previous_lag_us
}

/// One rung's full evaluated evidence, carried by `SweepDecision::Accept`
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

/// Outcome of evaluating one rung's stop conditions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepDecision {
    /// Neither stop condition tripped; continue to the next rung.
    Continue,
    /// A stop condition tripped; this outcome (this rung, or the previous
    /// one for the lag-plateau case) is the accepted result.
    Accept(RungOutcome),
    /// The ceiling tripped at the floor rung itself -- report conflict.
    Conflict,
}

/// Resolve one rung's evaluated stop conditions into a `SweepDecision`.
///
/// `is_first_rung` is true only for the floor rung itself. **`accepted_outcome`
/// must be the PREVIOUS rung's `RungOutcome` on every `Accept` path, for
/// both stop conditions** — the spec's Selection section accepts "the
/// previous rung" on a lag plateau and "the last rung below that margin"
/// on a ceiling trip; both mean the rung one step before the condition
/// that just tripped, never the rung that tripped it. This function does
/// not know or check which physical rung its argument describes — it only
/// decides which `SweepDecision` variant applies and passes the argument
/// through unchanged, so passing the current (tripping) rung's outcome
/// here is a caller bug this function cannot catch.
pub fn resolve_rung(
    is_first_rung: bool,
    ceiling_tripped: bool,
    lag_plateaued: bool,
    accepted_outcome: RungOutcome,
) -> SweepDecision {
    if ceiling_tripped && is_first_rung {
        return SweepDecision::Conflict;
    }
    if ceiling_tripped || lag_plateaued {
        return SweepDecision::Accept(accepted_outcome);
    }
    SweepDecision::Continue
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn lag_plateau_accepts_previous_rung() {
        // 5% tolerance: an improvement of less than 5% versus the previous
        // rung's lag counts as a plateau.
        let previous_lag_us = 1000_u32;
        let current_lag_us = 970_u32; // 3% improvement -- plateau.
        assert!(is_lag_plateau(previous_lag_us, current_lag_us));

        let current_lag_us_improved = 900_u32; // 10% improvement -- not a plateau.
        assert!(!is_lag_plateau(previous_lag_us, current_lag_us_improved));
    }

    #[test]
    fn conflict_when_ceiling_trips_at_floor_rung() {
        // The ceiling ratio check tripping on the very first rung tested
        // (the floor itself) is the coarse-PPR-encoder conflict case.
        let decision = resolve_rung(
            /* is_first_rung */ true,
            /* ceiling_tripped */ true,
            /* lag_plateaued */ false,
            RungOutcome {
                position_p: PidPGain::new(63).unwrap(),
                lag_us: 500,
                ceiling_ratios_milli: [2_100, 2_050, 900, 2_200, 2_150],
            },
        );
        assert!(matches!(decision, SweepDecision::Conflict));
    }

    #[test]
    fn accept_when_ceiling_trips_after_floor() {
        // `resolve_rung` itself doesn't know which physical rung
        // `accepted_outcome` describes -- it only decides which
        // `SweepDecision` variant applies and passes the argument through
        // unchanged. It is the *caller's* responsibility (Task 6's driving
        // loop) to pass the PREVIOUS rung's outcome here on a ceiling trip,
        // never the tripping rung's own outcome -- see Task 6, Step 5's
        // calling-convention note for why, and its own test for the case
        // where getting this backwards would matter.
        let previous_rung_outcome = RungOutcome {
            position_p: PidPGain::new(150).unwrap(),
            lag_us: 300,
            ceiling_ratios_milli: [900, 950, 980, 920, 910],
        };
        let decision = resolve_rung(false, true, false, previous_rung_outcome);
        assert!(matches!(
            decision,
            SweepDecision::Accept(outcome) if outcome.position_p.raw() == 150
        ));
    }
}
