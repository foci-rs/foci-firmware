use crate::autotune::PositionTuneOutcome;
use foci_tmc4671::pid::PidPGain;

/// `k = (65536 / (4*PPR)) * (velocity_p/256) / 256`, scaled to a Q16.16
/// fixed-point integer so `k * position_p_raw` (Task-caller side) lands in
/// raw `PID_TORQUE_FLUX_TARGET` register counts directly.
///
/// See the spec's Ceiling section: `k` is the expected quantization-only
/// torque-command dither per unit of position P, in the same domain as the
/// measured `PID_TORQUE_FLUX_TARGET` excursion.
pub fn compute_k(ppr: u32, velocity_p: PidPGain) -> Result<i64, PositionTuneOutcome> {
    if ppr == 0 || velocity_p.raw() == 0 {
        return Err(PositionTuneOutcome::InvalidCeilingConfiguration);
    }
    let numerator = 65_536_i64
        .checked_mul(i64::from(velocity_p.raw()))
        .ok_or(PositionTuneOutcome::InvalidCeilingConfiguration)?;
    let denominator = 4_i64
        .checked_mul(i64::from(ppr))
        .and_then(|v| v.checked_mul(256))
        .and_then(|v| v.checked_mul(256))
        .ok_or(PositionTuneOutcome::InvalidCeilingConfiguration)?;
    if denominator == 0 {
        return Err(PositionTuneOutcome::InvalidCeilingConfiguration);
    }
    // Keep k in Q16.16 (scaled by 65536) so the caller's `k * position_p /
    // 65536` recovers the real dither prediction without losing precision
    // for small PPR/velocity_p combinations.
    let k_q16 = numerator
        .checked_mul(65_536)
        .and_then(|v| v.checked_div(denominator))
        .ok_or(PositionTuneOutcome::InvalidCeilingConfiguration)?;
    // A sufficiently large PPR combined with a small nonzero velocity_p can
    // quantize k_q16 to zero even though neither input was itself zero --
    // treat that the same as an invalid configuration, not a valid k of 0
    // (which ceiling_trips would otherwise read as an extreme, always-tripping
    // prediction rather than a rejected configuration).
    if k_q16 <= 0 {
        return Err(PositionTuneOutcome::InvalidCeilingConfiguration);
    }
    Ok(k_q16)
}

/// At least 3 of `dither_captures`' 5 individual ratios exceeding a 2x
/// margin trips the ceiling. Returns the trip decision plus all 5 ratios
/// (each scaled by 1000, i.e. a milli-ratio, so `1000` means `r_i,j == 1.0`)
/// for the caller to report the median as a diagnostic.
///
/// The majority vote is over these 5 individual ratios, never a
/// rung-level median computed first -- a median collapsed to one number
/// before the threshold check cannot itself satisfy a "3 of 5" condition
/// (spec, Ceiling section).
pub fn ceiling_trips(
    dither_captures: &[i32; 5],
    k_q16: i64,
    position_p: PidPGain,
) -> (bool, [i64; 5]) {
    let predicted_q16 = k_q16.saturating_mul(i64::from(position_p.raw()));
    let mut ratios_milli = [0_i64; 5];
    let mut over_margin_count = 0_u8;
    for (idx, &measured) in dither_captures.iter().enumerate() {
        // ratio_milli = measured / (predicted_q16/65536) * 1000
        //             = measured * 65536 * 1000 / predicted_q16
        let ratio_milli = if predicted_q16 == 0 {
            i64::MAX // degenerate predicted value; treat as an extreme trip
        } else {
            i64::from(measured)
                .saturating_mul(65_536)
                .saturating_mul(1_000)
                / predicted_q16
        };
        ratios_milli[idx] = ratio_milli;
        if ratio_milli > 2_000 {
            over_margin_count += 1;
        }
    }
    (over_margin_count >= 3, ratios_milli)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compute_k_rejects_zero_ppr() {
        let velocity_p = PidPGain::new(755).unwrap();
        assert_eq!(
            compute_k(0, velocity_p),
            Err(PositionTuneOutcome::InvalidCeilingConfiguration)
        );
    }

    #[test]
    fn compute_k_rejects_zero_velocity_p() {
        assert_eq!(
            compute_k(1000, PidPGain::ZERO),
            Err(PositionTuneOutcome::InvalidCeilingConfiguration)
        );
    }

    #[test]
    fn compute_k_rejects_quantization_to_zero() {
        // Neither input is zero, but numerator*65536 (4_294_967_296) is far
        // smaller than the denominator (4 * 1_000_000_000 * 256 * 256 =
        // 262_144_000_000_000), so k_q16 truncates to 0.
        let velocity_p = PidPGain::new(1).unwrap();
        assert_eq!(
            compute_k(1_000_000_000, velocity_p),
            Err(PositionTuneOutcome::InvalidCeilingConfiguration)
        );
    }

    #[test]
    fn ceiling_trips_on_three_of_five_over_margin() {
        let k_q16 = 65_536; // k = 1.0 for a simple fixture
        let position_p = PidPGain::new(100).unwrap();
        // Predicted (k*P_i) = 100. 2x margin trips above 200.
        // 3 of 5 captures over margin -> trips.
        let captures: [i32; 5] = [90, 210, 220, 95, 230];
        let (trips, _ratios) = ceiling_trips(&captures, k_q16, position_p);
        assert!(trips);
    }

    #[test]
    fn ceiling_does_not_trip_on_two_of_five_over_margin() {
        let k_q16 = 65_536;
        let position_p = PidPGain::new(100).unwrap();
        let captures: [i32; 5] = [90, 210, 95, 95, 230];
        let (trips, _ratios) = ceiling_trips(&captures, k_q16, position_p);
        assert!(!trips);
    }

    #[test]
    fn ceiling_predicts_squeal_at_historical_squeal_gains() {
        // 2026-07-11 logbook: position_p=627, velocity_p=1434 squealed
        // while holding. k*P_i at these gains should predict a dither
        // value the 2x-margin check would flag as a real limit cycle if
        // the measured captures matched the prediction closely (i.e. this
        // test checks the *predicted* k*P_i magnitude order, not a live
        // capture -- there is no live capture from 2026-07-11 to replay).
        let velocity_p = PidPGain::new(1434).unwrap();
        let position_p = PidPGain::new(627).unwrap();
        let k_q16 = compute_k(1000, velocity_p).expect("valid k");
        let predicted = k_q16.saturating_mul(i64::from(position_p.raw())) / 65_536;
        // The historical squeal case's predicted dither is far larger than
        // the historical silent case's (below) -- this test guards the
        // formula's relative ordering, which is what the ceiling ratio
        // check actually depends on, rather than asserting one absolute
        // count value pulled from an ear-judged historical note.
        let velocity_p_silent = PidPGain::new(500).unwrap();
        let position_p_silent = PidPGain::new(100).unwrap();
        let k_silent_q16 = compute_k(1000, velocity_p_silent).expect("valid k");
        let predicted_silent =
            k_silent_q16.saturating_mul(i64::from(position_p_silent.raw())) / 65_536;
        assert!(
            predicted > predicted_silent * 10,
            "squeal-gain prediction ({predicted}) should be far larger than \
             silent-gain prediction ({predicted_silent})"
        );
    }

    #[test]
    fn ceiling_does_not_trip_at_production_gains_with_matching_captures() {
        // Production (100, 755) and (100, 812): if the 5 standstill
        // captures match the pure-quantization prediction closely (as
        // production's real quiet-but-audible behavior implies), the
        // ceiling must not trip -- a false trip here would make the sweep
        // reject the already-validated production gain.
        for velocity_p_raw in [755_u16, 812] {
            let velocity_p = PidPGain::new(velocity_p_raw).unwrap();
            let position_p = PidPGain::new(100).unwrap();
            let k_q16 = compute_k(1000, velocity_p).expect("valid k");
            let predicted = k_q16.saturating_mul(i64::from(position_p.raw())) / 65_536;
            let predicted_i32 = i32::try_from(predicted).expect("fits i32 for this fixture");
            // Captures matching the prediction within noise (+/-10%).
            let captures: [i32; 5] = [
                predicted_i32,
                predicted_i32 * 105 / 100,
                predicted_i32 * 95 / 100,
                predicted_i32 * 102 / 100,
                predicted_i32 * 98 / 100,
            ];
            let (trips, _ratios) = ceiling_trips(&captures, k_q16, position_p);
            assert!(
                !trips,
                "velocity_p={velocity_p_raw} should not trip at matching captures"
            );
        }
    }
}
