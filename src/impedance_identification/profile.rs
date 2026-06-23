//! Standard impedance-identification profile defaults.

use super::types::{IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceInjectionPoint};

/// Temporary OpenFFBoard fail mask used until the unsafe diagnostic status
/// policy is audited into named TMC4671 status bits.
pub const OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK: u32 = 0x7000_4480;

/// OpenFFBoard warning mask starts at zero until a reviewed warning-only status
/// policy exists for impedance identification.
pub const OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK: u32 = 0x0000_0000;

const OPENFFBOARD_IMPEDANCE_OPENLOOP_ACCELERATION_RPM_PER_S: u32 = 3_000;
const MICROS_PER_MILLISECOND: u64 = 1_000;

pub(crate) const fn frequency_millihz_to_openloop_velocity_rpm(frequency_millihz: u64) -> i32 {
    let rpm = ((frequency_millihz as u128) * 60) / 1_000;
    if rpm > i32::MAX as u128 {
        i32::MAX
    } else {
        rpm as i32
    }
}

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
}

impl ImpedanceProfile {
    /// Build the starting OpenFFBoard impedance-identification profile.
    ///
    /// The per-point acceleration default is intentionally conservative:
    /// 3000 electrical rpm/s ramps to the profile's highest 50 Hz injection
    /// point (3000 electrical rpm) in about one second without depending on
    /// any raw TMC clock conversion.
    pub const fn standard_openffboard() -> Self {
        Self {
            profile_version: 1,
            injection_points: [
                ImpedanceInjectionPoint {
                    ud_abs: 640,
                    frequency_millihz: 25_000,
                    openloop_velocity_rpm: frequency_millihz_to_openloop_velocity_rpm(25_000),
                    openloop_acceleration_rpm_per_s:
                        OPENFFBOARD_IMPEDANCE_OPENLOOP_ACCELERATION_RPM_PER_S,
                    settle_ms: 250,
                },
                ImpedanceInjectionPoint {
                    ud_abs: 960,
                    frequency_millihz: 50_000,
                    openloop_velocity_rpm: frequency_millihz_to_openloop_velocity_rpm(50_000),
                    openloop_acceleration_rpm_per_s:
                        OPENFFBOARD_IMPEDANCE_OPENLOOP_ACCELERATION_RPM_PER_S,
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
            // Allows the largest 960-count UD point through its 250 ms AC settle
            // plus the existing supported 128-sample cadence test at 375 us,
            // conservatively rounded to 48 ms.
            max_window_exposure_budget: 280_000_000,
            // Allows both standard points through DC settle and AC settle/capture
            // windows at the existing supported 375 us cadence:
            // sum(ud^2 * (250 ms + 250 ms + 48 ms)).
            max_total_exposure_budget: 735_000_000,
            scale_metadata_validated: false,
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

    /// Largest per-window exposure (`ud^2 * nonzero_ms`) in the profile.
    pub fn max_window_exposure(&self) -> u64 {
        let mut max = 0u64;
        for point in self.injection_points {
            let dc_exposure = exposure_for_ms(point.ud_abs, dc_nonzero_ms(point));
            if dc_exposure > max {
                max = dc_exposure;
            }

            let ac_exposure = exposure_for_ms(point.ud_abs, self.ac_nonzero_ms(point));
            if ac_exposure > max {
                max = ac_exposure;
            }
        }
        max
    }

    /// Total exposure budget consumed by all injection windows.
    pub fn max_total_exposure(&self) -> u64 {
        let mut total = 0u64;
        for point in self.injection_points {
            let dc_exposure = exposure_for_ms(point.ud_abs, dc_nonzero_ms(point));
            let ac_exposure = exposure_for_ms(point.ud_abs, self.ac_nonzero_ms(point));
            total = total.saturating_add(dc_exposure).saturating_add(ac_exposure);
        }
        total
    }

    fn capture_duration_ms(&self) -> u64 {
        let capture_us =
            u64::from(self.sample_count).saturating_mul(u64::from(self.sample_interval_us));
        capture_us.saturating_add(MICROS_PER_MILLISECOND - 1) / MICROS_PER_MILLISECOND
    }

    fn ac_nonzero_ms(&self, point: ImpedanceInjectionPoint) -> u64 {
        u64::from(point.settle_ms).saturating_add(self.capture_duration_ms())
    }
}

fn dc_nonzero_ms(point: ImpedanceInjectionPoint) -> u64 {
    u64::from(point.settle_ms)
}

fn exposure_for_ms(ud_abs: u16, duration_ms: u64) -> u64 {
    let ud = u64::from(ud_abs);
    ud.saturating_mul(ud).saturating_mul(duration_ms)
}
