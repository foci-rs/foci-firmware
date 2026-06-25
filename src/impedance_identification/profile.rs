//! Standard impedance-identification profile defaults.

use super::types::{IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceInjectionPoint};

const STATUS_PID_V_OUTPUT_LIMIT: u32 = 1 << 7;
const STATUS_PID_ID_ERRSUM_LIMIT: u32 = 1 << 10;
const STATUS_PID_IQ_ERRSUM_LIMIT: u32 = 1 << 14;
const STATUS_ADC_I_CLIPPED: u32 = 1 << 26;
const STATUS_ENC_N: u32 = 1 << 28;
const STATUS_ENC_2_N: u32 = 1 << 29;
const STATUS_AENC_N: u32 = 1 << 30;

/// OpenFFBoard impedance status bits that abort the diagnostic immediately.
pub const OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK: u32 = STATUS_ADC_I_CLIPPED;

/// OpenFFBoard impedance status bits that are reported as non-fatal evidence.
pub const OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK: u32 = STATUS_ENC_N
    | STATUS_ENC_2_N
    | STATUS_AENC_N
    | STATUS_PID_V_OUTPUT_LIMIT
    | STATUS_PID_ID_ERRSUM_LIMIT
    | STATUS_PID_IQ_ERRSUM_LIMIT;

const OPENFFBOARD_IMPEDANCE_OPENLOOP_ACCELERATION_RPM_PER_S: u32 = 24_000;
const MICROS_PER_MILLISECOND: u64 = 1_000;
const POINT_OBSERVATION_OVERHEAD_MS: u64 = 2;

/// Conservative per-sample overhead charged to the exposure budget on top of the
/// requested sample interval. It accounts for the raw ADC, transformed-current,
/// sampled open-loop phase, and status SPI register reads plus the safety gate
/// checks the runner performs between sample delays while voltage is applied,
/// which consume real energized time the requested delays do not model.
const PER_SAMPLE_OVERHEAD_US: u16 = 500;

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
    pub pwm_maxcnt: u16,
    pub bbm_h: u8,
    pub bbm_l: u8,
    pub dsadc_mdec_a: u16,
    pub dsadc_mdec_b: u16,
    pub pwm_sv_chop: u32,
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
    /// The standard points use one frequency with distinct voltage amplitudes
    /// so the repeatability check compares like-for-like normalized currents.
    /// The per-point acceleration default keeps the ramp to the profile's
    /// 25 Hz injection point inside the 250 ms settle window without increasing
    /// energized exposure.
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
                    frequency_millihz: 25_000,
                    openloop_velocity_rpm: frequency_millihz_to_openloop_velocity_rpm(25_000),
                    openloop_acceleration_rpm_per_s:
                        OPENFFBOARD_IMPEDANCE_OPENLOOP_ACCELERATION_RPM_PER_S,
                    settle_ms: 250,
                },
            ],
            sample_count: IMPEDANCE_SAMPLE_COUNT as u16,
            max_sample_count: IMPEDANCE_SAMPLE_COUNT as u16,
            sample_interval_us: 1000,
            pwm_maxcnt: 3999,
            bbm_h: 9,
            bbm_l: 9,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            pwm_sv_chop: 7,
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
            // Allows the largest 960-count UD point through DC settle, AC settle,
            // and the standard 128-sample capture. The capture window counts the
            // 128 ms of requested delay plus a conservative per-sample read/check
            // overhead, 192 ms at the 1 ms firmware cadence, plus 2 ms for the
            // point-local observation reads:
            // 960^2 * (250 ms + 250 ms + 2 ms + 192 ms).
            max_window_exposure_budget: 640_000_000,
            // Allows both standard points through DC settle and AC settle/capture
            // windows at the 1 ms firmware cadence, including the per-sample
            // overhead in the capture window and the point-local observation
            // reads:
            // sum(ud^2 * (250 ms + 250 ms + 2 ms + 192 ms)).
            max_total_exposure_budget: 925_000_000,
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
            let exposure = exposure_for_ms(point.ud_abs, self.point_nonzero_ms(point));
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
            total =
                total.saturating_add(exposure_for_ms(point.ud_abs, self.point_nonzero_ms(point)));
        }
        total
    }

    fn capture_duration_ms(&self) -> u64 {
        let per_sample_us =
            u64::from(self.sample_interval_us).saturating_add(u64::from(PER_SAMPLE_OVERHEAD_US));
        let capture_us = u64::from(self.sample_count).saturating_mul(per_sample_us);
        capture_us.saturating_add(MICROS_PER_MILLISECOND - 1) / MICROS_PER_MILLISECOND
    }

    fn ac_nonzero_ms(&self, point: ImpedanceInjectionPoint) -> u64 {
        u64::from(point.settle_ms)
            .saturating_add(POINT_OBSERVATION_OVERHEAD_MS)
            .saturating_add(self.capture_duration_ms())
    }

    fn point_nonzero_ms(&self, point: ImpedanceInjectionPoint) -> u64 {
        u64::from(point.settle_ms).saturating_add(self.ac_nonzero_ms(point))
    }
}

fn exposure_for_ms(ud_abs: u16, duration_ms: u64) -> u64 {
    let ud = u64::from(ud_abs);
    ud.saturating_mul(ud).saturating_mul(duration_ms)
}
