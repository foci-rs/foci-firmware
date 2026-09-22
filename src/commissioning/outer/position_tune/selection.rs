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

/// One rung's evaluated evidence for `decide`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RungMeasurement {
    pub position_p: PidPGain,
    pub homing_peak_abs_units: u32,
    pub motion_cruise_mean_abs_units: u32,
    pub motion_overshoot_units: u32,
    pub motion_cruise_rms_units: u32,
    pub ceiling_ratios_milli: [i64; 5],
}

/// Which guard tripped when `decide` reports `RungDecision::Conflict`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum ConflictKind {
    OvershootRatio,
    OvershootOverBound,
    Dither,
}

/// What to do with the current rung, from `decide`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RungDecision {
    Accept,
    Advance,
    Conflict(ConflictKind),
}

/// Reversal overshoot may not grow past this multiple of the running
/// minimum overshoot seen so far without tripping a conflict.
pub(crate) const OVERSHOOT_RATIO_LIMIT: u32 = 3;

/// Decide what to do with one rung's measurement against the tracking
/// bound and the dither/overshoot-ratio guards.
///
/// Order matches the design spec's `Selection` pseudocode exactly: the
/// dither guard, then the overshoot-ratio guard (unconditional -- a
/// resonance-transition rung conflicts on the ratio alone, whether or not
/// tracking would separately pass or fail), then tracking (`Advance` on
/// either bound miss), and only once tracking passes does the plain
/// overshoot-vs-bound check decide `Accept` vs `Conflict`. `min.max(1)`
/// keeps a zero running minimum from making any nonzero overshoot a ratio
/// trip; the ratio guard is about growth, and a rung with zero overshoot has
/// nothing to grow from.
pub(crate) fn decide(
    m: &RungMeasurement,
    bound_units: u32,
    min_overshoot_so_far: Option<u32>,
    ceiling_tripped: bool,
) -> RungDecision {
    if ceiling_tripped {
        return RungDecision::Conflict(ConflictKind::Dither);
    }
    if let Some(min) = min_overshoot_so_far
        && m.motion_overshoot_units > min.max(1).saturating_mul(OVERSHOOT_RATIO_LIMIT)
    {
        return RungDecision::Conflict(ConflictKind::OvershootRatio);
    }
    let homing_ok = m.homing_peak_abs_units <= bound_units;
    let motion_ok = m.motion_cruise_mean_abs_units <= bound_units;
    if !(homing_ok && motion_ok) {
        return RungDecision::Advance;
    }
    if m.motion_overshoot_units > bound_units {
        return RungDecision::Conflict(ConflictKind::OvershootOverBound);
    }
    RungDecision::Accept
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

    fn measurement(p: u16, homing: u32, cruise: u32, overshoot: u32) -> RungMeasurement {
        RungMeasurement {
            position_p: PidPGain::new(p).unwrap(),
            homing_peak_abs_units: homing,
            motion_cruise_mean_abs_units: cruise,
            motion_overshoot_units: overshoot,
            motion_cruise_rms_units: 0,
            ceiling_ratios_milli: [0; 5],
        }
    }

    #[test]
    fn accepts_the_first_rung_inside_both_bounds() {
        assert_eq!(
            decide(&measurement(165, 300, 200, 50), 409, None, false),
            RungDecision::Accept
        );
    }

    #[test]
    fn advances_while_either_metric_is_outside_the_bound() {
        assert_eq!(
            decide(&measurement(100, 600, 200, 50), 409, None, false),
            RungDecision::Advance
        );
        assert_eq!(
            decide(&measurement(100, 300, 500, 50), 409, None, false),
            RungDecision::Advance
        );
    }

    #[test]
    fn overshoot_over_the_bound_with_tracking_met_and_ratio_clear_is_a_conflict() {
        // min_overshoot_so_far = 200 keeps the ratio threshold (3 * 200 =
        // 600) above this rung's overshoot (500), so only the plain
        // bound guard is exercised, isolating `OvershootOverBound` from
        // `OvershootRatio`.
        assert_eq!(
            decide(&measurement(300, 300, 200, 500), 409, Some(200), false),
            RungDecision::Conflict(ConflictKind::OvershootOverBound)
        );
    }

    #[test]
    fn overshoot_ratio_trips_before_tracking_is_ever_checked() {
        // Homing tracking would independently fail too (600 > the 409
        // bound), but the spec's ratio guard is unconditional and runs
        // first: a rung whose motion overshoot has grown past 3x the
        // running minimum (500 > 3*50 = 150) conflicts on the ratio alone,
        // regardless of what tracking would have decided.
        assert_eq!(
            decide(&measurement(300, 600, 200, 500), 409, Some(50), false),
            RungDecision::Conflict(ConflictKind::OvershootRatio)
        );
    }

    #[test]
    fn overshoot_ratio_trips_only_after_the_first_rung() {
        assert_eq!(
            decide(&measurement(600, 300, 200, 200), 409, None, false),
            RungDecision::Accept
        );
        assert_eq!(
            decide(&measurement(600, 300, 200, 200), 409, Some(50), false),
            RungDecision::Conflict(ConflictKind::OvershootRatio)
        );
        assert_eq!(
            decide(&measurement(600, 300, 200, 149), 409, Some(50), false),
            RungDecision::Accept
        );
    }

    #[test]
    fn dither_trip_is_a_conflict_whatever_the_metrics_say() {
        assert_eq!(
            decide(&measurement(165, 300, 200, 50), 409, None, true),
            RungDecision::Conflict(ConflictKind::Dither)
        );
    }

    #[test]
    fn a_quartered_ppr_dither_trip_still_reaches_conflict() {
        let velocity_p = PidPGain::new(1434).unwrap();
        let position_p = PidPGain::new(627).unwrap();
        let k_q16 =
            super::super::ceiling::compute_k(250, velocity_p).expect("valid k at a quartered PPR");
        let predicted = k_q16.saturating_mul(i64::from(position_p.raw())) / 65_536;
        let predicted_i32 = i32::try_from(predicted).expect("fits i32 for this fixture");
        let captures: [i32; 5] = [
            predicted_i32 * 3,
            predicted_i32 * 3,
            predicted_i32 / 2,
            predicted_i32 * 3,
            predicted_i32 / 2,
        ];
        let (ceiling_tripped, ratios) =
            super::super::ceiling::ceiling_trips(&captures, k_q16, position_p);
        assert!(ceiling_tripped);
        let m = RungMeasurement {
            ceiling_ratios_milli: ratios,
            ..measurement(627, 300, 200, 50)
        };
        assert_eq!(
            decide(&m, 409, None, ceiling_tripped),
            RungDecision::Conflict(ConflictKind::Dither)
        );
    }
}
