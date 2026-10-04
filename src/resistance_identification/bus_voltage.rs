//! Bus-voltage sensing and ladder scaling for resistance identification.

use super::profile::{REFERENCE_BUS_MILLIVOLTS, ResistanceProfile};

/// Lowest bus voltage the sweep accepts as a plausible reading.
pub const BUS_MILLIVOLTS_MIN: u32 = 18_000;
/// Highest bus voltage the sweep accepts as a plausible reading.
pub const BUS_MILLIVOLTS_MAX: u32 = 60_000;

/// What the runner needs to turn a raw `ADC_VM` count into bus millivolts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BusVoltageSense {
    /// Raw `ADC_VM` count at zero bus voltage, captured in the `VDD/2` input-stage mode.
    pub offset_raw: u16,
    /// Milli-counts of `ADC_VM` per volt at the bus, after the board's divider.
    pub counts_per_volt_milli: u32,
}

/// Milli-counts of `ADC_VM` per bus volt for a divider of `high_ohms` over `low_ohms`
/// feeding an ADC with `center_counts` over `reference_mv`.
///
/// Rounded to nearest; `u64` intermediates because the numerator is about 4.9e13 for
/// the shipped boards.
pub const fn counts_per_volt_milli(
    center_counts: u32,
    low_ohms: u32,
    high_ohms: u32,
    reference_mv: u32,
) -> u32 {
    let numerator = center_counts as u64 * 1_000_000 * low_ohms as u64;
    let denominator = reference_mv as u64 * (high_ohms as u64 + low_ohms as u64);
    if denominator == 0 {
        return 0;
    }
    ((numerator + denominator / 2) / denominator) as u32
}

/// Bus millivolts for a raw `ADC_VM` reading, or `None` when the reading is at or below
/// the offset or the sense scale is zero.
pub fn bus_millivolts(raw: u16, sense: BusVoltageSense) -> Option<u32> {
    if raw <= sense.offset_raw || sense.counts_per_volt_milli == 0 {
        return None;
    }
    let delta = u64::from(raw - sense.offset_raw);
    let millivolts = delta * 1_000_000 / u64::from(sense.counts_per_volt_milli);
    u32::try_from(millivolts).ok()
}

/// Whether a bus reading is inside the range the sweep supports.
pub const fn bus_millivolts_in_range(v_bus_mv: u32) -> bool {
    v_bus_mv >= BUS_MILLIVOLTS_MIN && v_bus_mv <= BUS_MILLIVOLTS_MAX
}

fn scale_ud(ud: i16, v_bus_mv: u32) -> i16 {
    let magnitude = u64::from(ud.unsigned_abs()) * u64::from(REFERENCE_BUS_MILLIVOLTS);
    let divisor = u64::from(v_bus_mv.max(1));
    let scaled = ((magnitude + divisor / 2) / divisor).min(i16::MAX as u64) as i16;
    if ud < 0 { -scaled } else { scaled }
}

/// The profile with every UD point and the signed anchor scaled by the reference bus
/// over `v_bus_mv`, rounded to nearest. All current-domain fields are returned as-is.
pub fn scale_profile_to_bus(profile: &ResistanceProfile, v_bus_mv: u32) -> ResistanceProfile {
    let mut scaled = *profile;
    scaled.ud_points = profile.ud_points.map(|ud| scale_ud(ud, v_bus_mv));
    scaled.signed_anchor_ud = scale_ud(profile.signed_anchor_ud, v_bus_mv);
    scaled
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resistance_identification::profile::{REFERENCE_BUS_MILLIVOLTS, ResistanceProfile};

    const BOARD_SENSE: BusVoltageSense = BusVoltageSense {
        offset_raw: 33_671,
        counts_per_volt_milli: counts_per_volt_milli(32_767, 1_500, 71_500, 2_500),
    };

    #[test]
    fn board_constants_give_the_spec_milli_counts_per_volt() {
        assert_eq!(counts_per_volt_milli(32_767, 1_500, 71_500, 2_500), 269_318);
    }

    #[test]
    fn tester_reading_converts_to_fifty_point_three_volts() {
        assert_eq!(bus_millivolts(47_217, BOARD_SENSE), Some(50_297));
    }

    #[test]
    fn bus_millivolts_rejects_raw_at_or_below_the_offset() {
        assert_eq!(bus_millivolts(33_671, BOARD_SENSE), None);
        assert_eq!(bus_millivolts(0, BOARD_SENSE), None);
        let zero_scale = BusVoltageSense {
            offset_raw: 0,
            counts_per_volt_milli: 0,
        };
        assert_eq!(bus_millivolts(40_000, zero_scale), None);
    }

    #[test]
    fn range_check_matches_the_spec_bounds() {
        assert!(!bus_millivolts_in_range(17_999));
        assert!(bus_millivolts_in_range(18_000));
        assert!(bus_millivolts_in_range(24_000));
        assert!(bus_millivolts_in_range(60_000));
        assert!(!bus_millivolts_in_range(60_001));
    }

    #[test]
    fn scaling_at_the_rig_voltage_and_at_reference() {
        let standard = ResistanceProfile::standard();

        let at_reference = scale_profile_to_bus(&standard, REFERENCE_BUS_MILLIVOLTS);
        assert_eq!(at_reference, standard);

        let rig = scale_profile_to_bus(&standard, 48_300);
        assert_eq!(rig.ud_points, [509, 636, 795, 954, 1113, 1272, 1431, 1590]);
        assert_eq!(rig.signed_anchor_ud, 954);
    }

    #[test]
    fn scaling_at_twenty_four_volts_doubles_every_point_and_the_anchor() {
        let standard = ResistanceProfile::standard();
        let low_bus = scale_profile_to_bus(&standard, 24_000);
        assert_eq!(
            low_bus.ud_points,
            [1024, 1280, 1600, 1920, 2240, 2560, 2880, 3200]
        );
        assert_eq!(low_bus.signed_anchor_ud, 1920);
        assert_eq!(low_bus.linear_current_threshold_count, 750);
        assert_eq!(
            low_bus.max_expected_current_count,
            standard.max_expected_current_count
        );
        assert_eq!(low_bus.settle_ms, standard.settle_ms);
    }

    #[test]
    fn scaling_rounds_to_nearest() {
        let standard = ResistanceProfile::standard();
        let scaled = scale_profile_to_bus(&standard, 50_297);
        assert_eq!(scaled.ud_points[7], 1527);
        assert_eq!(scaled.ud_points[0], 489);
    }
}
