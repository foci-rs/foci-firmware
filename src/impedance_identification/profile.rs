//! Standard impedance-identification profile defaults.

use super::types::{ImpedanceInjectionPoint, IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT};

/// Temporary OpenFFBoard fail mask used until the unsafe diagnostic status
/// policy is audited into named TMC4671 status bits.
pub const OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK: u32 = 0x7000_4480;

/// OpenFFBoard warning mask starts at zero until a reviewed warning-only status
/// policy exists for impedance identification.
pub const OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK: u32 = 0x0000_0000;

/// Profile constants for one impedance-identification run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceProfile {
    pub profile_version: u16,
    pub injection_points: [ImpedanceInjectionPoint; IMPEDANCE_PROFILE_POINTS],
    pub sample_count: u16,
    pub max_sample_count: u16,
    pub sample_interval_us: u16,
    pub min_signal_count: u16,
    pub max_expected_current_count: u16,
    pub encoder_move_warn_counts: u16,
    pub encoder_move_fail_counts: u16,
    pub residual_warn_permille: u16,
    pub residual_fail_permille: u16,
    pub repeatability_warn_permille: u16,
    pub repeatability_fail_permille: u16,
    pub scalar_2f_mismatch_warn_permille: u16,
    pub scalar_2f_mismatch_fail_permille: u16,
    pub status_flags_fail_mask: u32,
    pub status_flags_warn_mask: u32,
    pub max_window_exposure_budget: u64,
    pub max_total_exposure_budget: u64,
    pub scale_metadata_validated: bool,
    pub openloop_acceleration_rpm_per_s: u32,
}

impl ImpedanceProfile {
    /// Build the starting OpenFFBoard impedance-identification profile.
    ///
    /// The acceleration default is intentionally conservative: 3000 electrical
    /// rpm/s ramps to the profile's highest 50 Hz injection point
    /// (3000 electrical rpm) in about one second without depending on any raw
    /// TMC clock conversion.
    pub const fn standard_openffboard() -> Self {
        Self {
            profile_version: 1,
            injection_points: [
                ImpedanceInjectionPoint {
                    ud_abs: 640,
                    frequency_millihz: 25_000,
                    settle_ms: 250,
                },
                ImpedanceInjectionPoint {
                    ud_abs: 960,
                    frequency_millihz: 50_000,
                    settle_ms: 250,
                },
            ],
            sample_count: IMPEDANCE_SAMPLE_COUNT as u16,
            max_sample_count: IMPEDANCE_SAMPLE_COUNT as u16,
            sample_interval_us: 250,
            min_signal_count: 96,
            max_expected_current_count: 1600,
            encoder_move_warn_counts: 64,
            encoder_move_fail_counts: 128,
            residual_warn_permille: 100,
            residual_fail_permille: 200,
            repeatability_warn_permille: 100,
            repeatability_fail_permille: 200,
            scalar_2f_mismatch_warn_permille: 100,
            scalar_2f_mismatch_fail_permille: 200,
            status_flags_fail_mask: OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK,
            status_flags_warn_mask: OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK,
            max_window_exposure_budget: 250_000_000,
            max_total_exposure_budget: 500_000_000,
            scale_metadata_validated: false,
            openloop_acceleration_rpm_per_s: 3_000,
        }
    }

    /// Largest requested UD magnitude in the profile.
    pub fn max_abs_ud(&self) -> u16 {
        let mut max = 0u16;
        for point in self.injection_points {
            if point.ud_abs > max {
                max = point.ud_abs;
            }
        }
        max
    }

    /// Largest per-window exposure (`ud^2 * settle_ms`) in the profile.
    pub fn max_window_exposure(&self) -> u64 {
        let mut max = 0u64;
        for point in self.injection_points {
            let exposure = u64::from(point.ud_abs) * u64::from(point.ud_abs) * u64::from(point.settle_ms);
            if exposure > max {
                max = exposure;
            }
        }
        max
    }

    /// Total exposure budget consumed by all injection windows.
    pub fn max_total_exposure(&self) -> u64 {
        let mut total = 0u64;
        for point in self.injection_points {
            let exposure = u64::from(point.ud_abs) * u64::from(point.ud_abs) * u64::from(point.settle_ms);
            total = total.saturating_add(exposure);
        }
        total
    }
}
