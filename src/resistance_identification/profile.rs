use super::types::ElectricalAxis;

/// Standard positive UD point candidates for adaptive resistance identification.
pub const STANDARD_UD_POINTS: [i16; 8] = [512, 640, 800, 960, 1120, 1280, 1440, 1600];

// STATUS_FLAGS bit 31 is reserved in the TMC4671 status layout. It may
// appear in sampled flags, but it is not an actionable resistance-run
// fault. See `autotune::current::current_loop_actionable_status_mask`'s
// identical exclusion of this bit.
const RESISTANCE_RUN_RESERVED_STATUS_MASK: u32 = 1 << 31;

/// Documented actionable mask for resistance-run `STATUS_FLAGS` warnings.
///
/// Deliberately broader than `autotune::current::current_loop_actionable_status_mask`,
/// which also excludes several bits that are specifically not meaningful for
/// closed-loop current validation (PID output clipping, inactive-encoder
/// bits). Those distinctions do not apply to the resistance run's open-loop
/// excitation, so only the one bit documented as hardware-reserved is
/// excluded here.
pub const fn resistance_run_actionable_status_mask() -> u32 {
    !RESISTANCE_RUN_RESERVED_STATUS_MASK
}

/// Profile constants for one resistance-identification run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ResistanceProfile {
    pub axes: [ElectricalAxis; 2],
    pub ud_points: [i16; 8],
    pub signed_anchor_ud: i16,
    pub linear_current_threshold_count: u16,
    pub min_selected_points: u8,
    pub electrical_axis_mismatch_permille: u16,
    pub signed_asymmetry_warn_permille: u16,
    pub thermal_drift_warn_permille: u16,
    pub thermal_drift_fail_permille: u16,
    pub high_residual_warn_permille: u16,
    pub high_residual_fail_permille: u16,
    pub settle_ms: u16,
    pub zero_settle_ms: u16,
    pub sample_interval_ms: u16,
    pub pwm_maxcnt: u16,
    pub bbm_h: u8,
    pub bbm_l: u8,
    pub dsadc_mdec_a: u16,
    pub dsadc_mdec_b: u16,
    pub pwm_sv_chop: u32,
    pub max_expected_current_count: u16,
    pub encoder_move_warn_counts: u16,
    pub status_flags_warn_mask: u32,
    pub max_window_exposure: u64,
    pub max_total_exposure: u64,
    pub scale_metadata_validated: bool,
}

impl ResistanceProfile {
    pub const fn standard() -> Self {
        Self {
            axes: [ElectricalAxis::PHASE0, ElectricalAxis::PHASE90],
            ud_points: STANDARD_UD_POINTS,
            signed_anchor_ud: 960,
            linear_current_threshold_count: 750,
            min_selected_points: 4,
            electrical_axis_mismatch_permille: 150,
            signed_asymmetry_warn_permille: 100,
            thermal_drift_warn_permille: 20,
            thermal_drift_fail_permille: 50,
            high_residual_warn_permille: 25,
            high_residual_fail_permille: 75,
            settle_ms: 500,
            zero_settle_ms: 250,
            sample_interval_ms: 5,
            pwm_maxcnt: 3999,
            bbm_h: 9,
            bbm_l: 9,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            pwm_sv_chop: 7,
            max_expected_current_count: 1600,
            encoder_move_warn_counts: 128,
            status_flags_warn_mask: resistance_run_actionable_status_mask(),
            max_window_exposure: 30_000_000_000,
            max_total_exposure: 100_000_000_000,
            scale_metadata_validated: true,
        }
    }

    pub const fn energized_windows_per_axis(&self) -> u8 {
        self.ud_points.len() as u8 + 2
    }

    pub fn max_abs_ud(&self) -> i16 {
        let mut max = self.signed_anchor_ud.abs();
        for point in self.ud_points {
            let abs = point.abs();
            if abs > max {
                max = abs;
            }
        }
        max
    }

    pub fn exposure_per_axis(&self) -> u64 {
        let mut total = 0u64;
        for ud in self.ud_points {
            let abs = u64::from(ud.unsigned_abs());
            total = total.saturating_add(abs * abs * u64::from(self.settle_ms));
        }
        let anchor = u64::from(self.signed_anchor_ud.unsigned_abs());
        total.saturating_add(2 * anchor * anchor * u64::from(self.settle_ms))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_profile_counts_repeat_and_negative_anchor_in_exposure() {
        let profile = ResistanceProfile::standard();

        assert_eq!(
            profile.ud_points,
            [512, 640, 800, 960, 1120, 1280, 1440, 1600]
        );
        assert_eq!(profile.signed_anchor_ud, 960);
        assert_eq!(profile.energized_windows_per_axis(), 10);

        let expected_anchor_exposure = 2
            * u64::from(profile.signed_anchor_ud.unsigned_abs()).pow(2)
            * u64::from(profile.settle_ms);
        let expected = profile
            .ud_points
            .iter()
            .map(|ud| u64::from(ud.unsigned_abs()).pow(2) * u64::from(profile.settle_ms))
            .sum::<u64>()
            + expected_anchor_exposure;
        assert_eq!(profile.exposure_per_axis(), expected);
    }

    #[test]
    fn standard_profile_rejects_fixture_peak_current_without_rating() {
        let profile = ResistanceProfile::standard();

        assert_eq!(profile.max_abs_ud(), 1600);
        assert!(profile.max_expected_current_count >= 1400);
        assert_eq!(profile.encoder_move_warn_counts, 128);
        assert_ne!(profile.status_flags_warn_mask, 0);
    }

    #[test]
    fn standard_profile_status_flags_warn_mask_excludes_only_the_reserved_bit() {
        let profile = ResistanceProfile::standard();

        // STATUS_FLAGS bit 31 is reserved in the TMC4671 status layout (see
        // `autotune::current::current_loop_actionable_status_mask`'s
        // identical exclusion) and must never itself trigger a warning.
        // Every other bit remains actionable for the resistance-run warning
        // classification, unlike the hard-fail current-loop mask which also
        // excludes several current-loop-specific bits not relevant here.
        assert_eq!(profile.status_flags_warn_mask, !(1u32 << 31));
    }
}
