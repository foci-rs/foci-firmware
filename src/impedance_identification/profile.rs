//! Held-angle rotating-carrier impedance-identification profile defaults.

use super::types::{IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedancePoint};

const MICROS_PER_MILLISECOND: u64 = 1_000;
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

/// Conservative per-sample overhead charged to the exposure budget on top of the
/// requested sample interval. It accounts for the raw ADC, transformed-current,
/// encoder, and status SPI reads plus safety gates between sample delays.
pub(crate) const IMPEDANCE_PER_SAMPLE_OVERHEAD_US: u16 = 500;

/// Profile constants for one held-angle rotating-carrier impedance-identification run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceProfile {
    /// Carrier probe-angle windows sampled by this profile.
    pub angle_points: [ImpedancePoint; IMPEDANCE_PROFILE_POINTS],
    /// Number of samples captured for each carrier cycle.
    pub samples_per_cycle: u16,
    /// Number of carrier cycles captured at each probe angle.
    pub cycles_per_angle: u16,
    /// Number of samples captured at each probe angle.
    pub sample_count: u16,
    /// Maximum accepted sample count for this profile.
    pub max_sample_count: u16,
    /// Requested interval between samples in microseconds.
    pub sample_interval_us: u16,
    /// DC UD magnitude used to hold the rotor at the fixed external electrical angle.
    pub hold_ud_abs: u16,
    /// Settling time after applying the fixed held-angle DC voltage.
    pub hold_settle_ms: u16,
    /// Fixed delay after each carrier-voltage write before sampling current.
    pub carrier_settle_us: u16,
    /// Initial carrier cycles excluded from lock-in accumulation at each angle.
    pub discard_cycles: u16,
    /// Expected `PWM_MAXCNT` profile value.
    pub pwm_maxcnt: u16,
    /// Expected high-side break-before-make clocks.
    pub bbm_h: u8,
    /// Expected low-side break-before-make clocks.
    pub bbm_l: u8,
    /// Expected DSADC decimation for channel A.
    pub dsadc_mdec_a: u16,
    /// Expected DSADC decimation for channel B.
    pub dsadc_mdec_b: u16,
    /// Expected `PWM_SV_CHOP` profile value.
    pub pwm_sv_chop: u32,
    /// Minimum normalized signal count per UD millivolt-equivalent.
    pub min_signal_count_per_ud_milli: u16,
    /// Minimum saliency separation needed to determine the electrical axis.
    pub saliency_axis_determinability_count_per_ud_milli: u16,
    /// Minimum accepted angular coverage in permille.
    pub angle_coverage_fail_permille: u16,
    /// Maximum expected current count before aborting.
    pub max_expected_current_count: u16,
    /// Encoder movement count that emits a warning.
    pub encoder_move_warn_counts: u16,
    /// Encoder movement count that fails the run.
    pub encoder_move_fail_counts: u16,
    /// Encoder-angle invalid count that fails the run.
    pub encoder_angle_invalid_counts: u16,
    /// Residual warning threshold in permille.
    pub residual_warn_permille: u16,
    /// Residual failure threshold in permille.
    pub residual_fail_permille: u16,
    /// Repeatability warning threshold in permille.
    pub repeatability_warn_permille: u16,
    /// Repeatability failure threshold in permille.
    pub repeatability_fail_permille: u16,
    /// STATUS_FLAGS bits that fail the diagnostic immediately.
    pub status_flags_fail_mask: u32,
    /// STATUS_FLAGS bits that are reported as warning evidence.
    pub status_flags_warn_mask: u32,
    /// Maximum allowed exposure for one probe-angle window.
    pub max_window_exposure_budget: u64,
    /// Maximum allowed exposure for all probe-angle windows.
    pub max_total_exposure_budget: u64,
    /// Whether host scale metadata has been physically validated.
    pub scale_metadata_validated: bool,
}

impl ImpedanceProfile {
    /// Build the starting OpenFFBoard held-angle rotating-carrier impedance profile.
    pub const fn standard_openffboard() -> Self {
        Self {
            // Roles: warmup zero, reference zero, sweep, return-settle zero,
            // post-sweep zero measurement, stability zero check.
            angle_points: [
                ImpedancePoint {
                    angle_electrical_counts: 0,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 0,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 8192,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 16384,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 24576,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 32768,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 40960,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 0,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 0,
                    ud_abs: 464,
                },
                ImpedancePoint {
                    angle_electrical_counts: 0,
                    ud_abs: 464,
                },
            ],
            samples_per_cycle: 20,
            cycles_per_angle: 8,
            sample_count: IMPEDANCE_SAMPLE_COUNT as u16,
            max_sample_count: IMPEDANCE_SAMPLE_COUNT as u16,
            sample_interval_us: 2_000,
            hold_ud_abs: 640,
            hold_settle_ms: 80,
            carrier_settle_us: 1_000,
            discard_cycles: 1,
            pwm_maxcnt: 3999,
            bbm_h: 9,
            bbm_l: 9,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            pwm_sv_chop: 7,
            min_signal_count_per_ud_milli: 32,
            saliency_axis_determinability_count_per_ud_milli: 16,
            angle_coverage_fail_permille: 850,
            max_expected_current_count: 1600,
            encoder_move_warn_counts: 64,
            encoder_move_fail_counts: 128,
            encoder_angle_invalid_counts: 32,
            residual_warn_permille: 250,
            residual_fail_permille: 750,
            repeatability_warn_permille: 100,
            repeatability_fail_permille: 200,
            status_flags_fail_mask: OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK,
            status_flags_warn_mask: OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK,
            max_window_exposure_budget: 586_000_000,
            max_total_exposure_budget: 5_851_000_000,
            scale_metadata_validated: false,
        }
    }

    /// Largest requested UD magnitude in the profile.
    pub fn max_abs_ud(&self) -> u16 {
        let mut max = self.hold_ud_abs;
        for point in self.angle_points {
            let high = self.hold_ud_abs.saturating_add(point.ud_abs);
            if high > max {
                max = high;
            }
        }
        max
    }

    /// Square-carrier frequency in millihertz, derived from the real sample cadence.
    pub const fn carrier_frequency_millihz(&self) -> u32 {
        let carrier_period_us =
            (self.sample_interval_us as u64).saturating_mul(self.samples_per_cycle as u64);
        if carrier_period_us == 0 {
            return 0;
        }

        (1_000_000_000u64 / carrier_period_us) as u32
    }

    /// Nominal carrier-active duration for each angle window, in milliseconds.
    pub const fn nominal_carrier_window_ms(&self) -> u64 {
        let carrier_frequency_millihz = self.carrier_frequency_millihz();
        if carrier_frequency_millihz == 0 {
            return u64::MAX;
        }

        (self.cycles_per_angle as u64).saturating_mul(1_000_000)
            / (carrier_frequency_millihz as u64)
    }

    /// Budgeted energized duration for each angle window, in milliseconds.
    pub const fn energized_window_ms(&self) -> u64 {
        let per_sample_us = (self.sample_interval_us as u64)
            .saturating_add(IMPEDANCE_PER_SAMPLE_OVERHEAD_US as u64);
        let capture_us = (self.sample_count as u64).saturating_mul(per_sample_us);
        let capture_ms =
            capture_us.saturating_add(MICROS_PER_MILLISECOND - 1) / MICROS_PER_MILLISECOND;
        capture_ms.saturating_add(self.hold_settle_ms as u64)
    }

    /// Largest per-window exposure (`ud^2 * energized_ms`) in the profile.
    pub fn max_window_exposure(&self) -> u64 {
        let mut max = 0u64;
        for point in self.angle_points {
            let exposure = exposure_for_ms(
                self.hold_ud_abs.saturating_add(point.ud_abs),
                self.energized_window_ms(),
            );
            if exposure > max {
                max = exposure;
            }
        }
        max
    }

    /// Total exposure budget consumed by all probe-angle windows.
    pub fn max_total_exposure(&self) -> u64 {
        let mut total = 0u64;
        for point in self.angle_points {
            total = total.saturating_add(exposure_for_ms(
                self.hold_ud_abs.saturating_add(point.ud_abs),
                self.energized_window_ms(),
            ));
        }
        total
    }
}

const fn exposure_for_ms(ud_abs: u16, duration_ms: u64) -> u64 {
    let ud = ud_abs as u64;
    ud.saturating_mul(ud).saturating_mul(duration_ms)
}
