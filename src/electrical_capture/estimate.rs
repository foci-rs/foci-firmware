use super::{ElectricalCaptureSample, R_SENSITIVITY_MAX_L_SHIFT_PERMILLE};

const MICROS_PER_SECOND: i128 = 1_000_000;
const TAU_MILLIRADIANS: u128 = 6_283;

/// Inclusive-exclusive sample range used for estimator fitting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FitWindow {
    /// First sample index to include.
    pub start_index: usize,
    /// One past the last sample index to include.
    pub end_index: usize,
}

/// Least-squares current ramp fit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct LinearFit {
    /// Fitted primary-current slope in current counts per second.
    pub slope_count_per_s: i64,
    /// Mean primary current in current counts.
    pub mean_current_count: i32,
    /// Fitted intercept in current counts at the first fitted timestamp.
    pub intercept_count: i32,
    /// Root-mean-square fit residual in current counts.
    pub residual_rms_count: u32,
    /// Number of samples included in the fit.
    pub sample_count: u16,
    /// Elapsed capture time covered by the fit.
    pub elapsed_us: u32,
}

/// Resistance drop model used to estimate inductance sensitivity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RDropModel {
    /// R drop in voltage-count millis per current count.
    pub r_count_slope_milli: i32,
    /// Applied d-axis voltage count.
    pub ud_count: i16,
    /// Symmetric R perturbation in permille.
    pub perturb_permille: u16,
}

/// Inductance sensitivity evidence under an R perturbation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceEvidence {
    /// Nominal inductance estimate in microseconds.
    pub l_nominal_us: u32,
    /// L shift when R is perturbed downward, in permille of nominal L.
    pub l_shift_minus_permille: u16,
    /// L shift when R is perturbed upward, in permille of nominal L.
    pub l_shift_plus_permille: u16,
}

impl InductanceEvidence {
    /// Return true when both R-perturbation shifts satisfy the 3% structural gate.
    pub fn passes_3_percent_gate(&self) -> bool {
        self.l_shift_minus_permille <= R_SENSITIVITY_MAX_L_SHIFT_PERMILLE
            && self.l_shift_plus_permille <= R_SENSITIVITY_MAX_L_SHIFT_PERMILLE
    }
}

/// Current delta threshold for theta-onset detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OnsetThreshold {
    /// Minimum absolute primary-current delta from the first sample.
    pub delta_count: u16,
}

/// Estimate electrical frequency from the current ramp slope and mean current.
pub fn effective_frequency_hz(
    di_dt_count_per_s: i64,
    mean_current_count: i32,
    current_noise_floor: u16,
) -> Option<u32> {
    let abs_mean = i64::from(mean_current_count).abs();
    if abs_mean <= i64::from(current_noise_floor) {
        return None;
    }

    let abs_slope = u128::from(di_dt_count_per_s.unsigned_abs());
    let denominator = u128::try_from(abs_mean).ok()? * TAU_MILLIRADIANS;
    if denominator == 0 {
        return None;
    }

    let hz = abs_slope * 1_000 / denominator;
    u32::try_from(hz).ok()
}

/// Fit a primary-current slope over the requested sample window.
pub fn fit_current_slope(
    samples: &[ElectricalCaptureSample],
    window: FitWindow,
) -> Option<LinearFit> {
    if window.start_index >= window.end_index || window.end_index > samples.len() {
        return None;
    }

    let fit_samples = &samples[window.start_index..window.end_index];
    if fit_samples.len() < 2 || fit_samples.len() > usize::from(u16::MAX) {
        return None;
    }

    let first_timestamp_us = fit_samples.first()?.timestamp_us;
    let last_timestamp_us = fit_samples.last()?.timestamp_us;
    let elapsed_us = last_timestamp_us.checked_sub(first_timestamp_us)?;
    if elapsed_us == 0 || elapsed_us > u64::from(u32::MAX) {
        return None;
    }

    let mut sum_x = 0_i128;
    let mut sum_x2 = 0_i128;
    let mut sum_y = 0_i128;
    let mut sum_xy = 0_i128;

    for sample in fit_samples {
        let x = i128::from(sample.timestamp_us.checked_sub(first_timestamp_us)?);
        let y = i128::from(sample.current_primary);

        sum_x += x;
        sum_x2 += x * x;
        sum_y += y;
        sum_xy += x * y;
    }

    let sample_count = i128::try_from(fit_samples.len()).ok()?;
    let slope_numerator = sample_count * sum_xy - sum_x * sum_y;
    let slope_denominator = sample_count * sum_x2 - sum_x * sum_x;
    if slope_denominator == 0 {
        return None;
    }

    let slope_count_per_s = (slope_numerator * MICROS_PER_SECOND) / slope_denominator;
    let intercept_micro_count =
        (sum_y * MICROS_PER_SECOND - slope_count_per_s * sum_x) / sample_count;

    let mut residual_square_sum = 0_u128;
    for sample in fit_samples {
        let x = i128::from(sample.timestamp_us.checked_sub(first_timestamp_us)?);
        let observed_micro_count = i128::from(sample.current_primary) * MICROS_PER_SECOND;
        let fitted_micro_count = intercept_micro_count + slope_count_per_s * x;
        let residual = observed_micro_count - fitted_micro_count;
        let residual_abs = residual.unsigned_abs();
        residual_square_sum =
            residual_square_sum.saturating_add(residual_abs.saturating_mul(residual_abs));
    }

    let residual_mean_square = residual_square_sum / u128::try_from(fit_samples.len()).ok()?;
    let residual_rms_micro_count = integer_sqrt(residual_mean_square);
    let residual_rms_count = residual_rms_micro_count / u128::try_from(MICROS_PER_SECOND).ok()?;

    Some(LinearFit {
        slope_count_per_s: i64::try_from(slope_count_per_s).ok()?,
        mean_current_count: i32::try_from(sum_y / sample_count).ok()?,
        intercept_count: i32::try_from(intercept_micro_count / MICROS_PER_SECOND).ok()?,
        residual_rms_count: u32::try_from(residual_rms_count).ok()?,
        sample_count: u16::try_from(fit_samples.len()).ok()?,
        elapsed_us: u32::try_from(elapsed_us).ok()?,
    })
}

/// Estimate nominal inductance and sensitivity to R-model perturbation.
pub fn inductance_sensitivity(fit: LinearFit, model: RDropModel) -> Option<InductanceEvidence> {
    let nominal = inductance_us_for_r_scale_permille(fit, model, 1_000)?;
    if nominal == 0 {
        return None;
    }

    let perturb = i32::from(model.perturb_permille);
    let minus = inductance_us_for_r_scale_permille(fit, model, 1_000_i32.checked_sub(perturb)?)?;
    let plus = inductance_us_for_r_scale_permille(fit, model, 1_000_i32.checked_add(perturb)?)?;

    Some(InductanceEvidence {
        l_nominal_us: u32::try_from(nominal).ok()?,
        l_shift_minus_permille: shift_permille(nominal, minus)?,
        l_shift_plus_permille: shift_permille(nominal, plus)?,
    })
}

/// Detect the first timestamp where primary current departs from the initial sample.
pub fn detect_theta_onset_us(
    samples: &[ElectricalCaptureSample],
    threshold: OnsetThreshold,
) -> Option<u64> {
    let first = samples.first()?;
    let baseline = i32::from(first.current_primary);
    let threshold = i32::from(threshold.delta_count);

    samples.iter().find_map(|sample| {
        let delta = (i32::from(sample.current_primary) - baseline).abs();
        (delta >= threshold).then_some(sample.timestamp_us)
    })
}

fn inductance_us_for_r_scale_permille(
    fit: LinearFit,
    model: RDropModel,
    r_scale_permille: i32,
) -> Option<u64> {
    let slope = fit.slope_count_per_s.unsigned_abs();
    if slope == 0 {
        return None;
    }

    let r_drop_count = i64::from(fit.mean_current_count)
        .checked_mul(i64::from(model.r_count_slope_milli))?
        .checked_mul(i64::from(r_scale_permille))?
        / 1_000_000;
    let effective_voltage = i64::from(model.ud_count).checked_sub(r_drop_count)?;
    effective_voltage
        .unsigned_abs()
        .checked_mul(1_000_000)?
        .checked_div(slope)
}

fn shift_permille(nominal: u64, perturbed: u64) -> Option<u16> {
    let delta = nominal.abs_diff(perturbed);
    let shift = div_ceil_u64(delta.checked_mul(1_000)?, nominal);
    u16::try_from(shift).ok()
}

fn div_ceil_u64(numerator: u64, denominator: u64) -> u64 {
    if numerator == 0 {
        0
    } else {
        1 + ((numerator - 1) / denominator)
    }
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }

    let mut low = 1_u128;
    let mut high = value;
    let mut result = 1_u128;

    while low <= high {
        let mid = low + ((high - low) / 2);
        if mid <= value / mid {
            result = mid;
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn effective_frequency_uses_slope_over_mean_current() {
        let hz = effective_frequency_hz(628_300, 100, 5);
        assert_eq!(hz, Some(1_000));
    }

    #[test]
    fn effective_frequency_rejects_noise_floor_window() {
        assert_eq!(effective_frequency_hz(10_000, 2, 5), None);
    }

    #[test]
    fn linear_fit_returns_slope_and_mean_current() {
        let samples = [
            ElectricalCaptureSample::new(0, 0, 1024, 10, 0, 100, 0),
            ElectricalCaptureSample::new(100, 0, 1024, 20, 0, 100, 0),
            ElectricalCaptureSample::new(200, 0, 1024, 30, 0, 100, 0),
            ElectricalCaptureSample::new(300, 0, 1024, 40, 0, 100, 0),
        ];

        let fit = fit_current_slope(
            &samples,
            FitWindow {
                start_index: 0,
                end_index: 4,
            },
        )
        .expect("linear samples should fit");

        assert_eq!(fit.slope_count_per_s, 100_000);
        assert_eq!(fit.mean_current_count, 25);
        assert_eq!(fit.sample_count, 4);
    }

    #[test]
    fn linear_fit_preserves_sub_unity_counts_per_us_slope() {
        let samples = [
            ElectricalCaptureSample::new(0, 0, 1024, 0, 0, 100, 0),
            ElectricalCaptureSample::new(250, 0, 1024, 1, 0, 100, 0),
            ElectricalCaptureSample::new(500, 0, 1024, 2, 0, 100, 0),
            ElectricalCaptureSample::new(750, 0, 1024, 3, 0, 100, 0),
        ];

        let fit = fit_current_slope(
            &samples,
            FitWindow {
                start_index: 0,
                end_index: 4,
            },
        )
        .expect("linear samples should fit");

        assert_eq!(fit.slope_count_per_s, 4_000);
        assert_eq!(fit.residual_rms_count, 0);
    }

    #[test]
    fn r_sensitivity_reports_l_shift_permille() {
        let fit = LinearFit {
            slope_count_per_s: 400_000,
            mean_current_count: 100,
            intercept_count: 10,
            residual_rms_count: 0,
            sample_count: 4,
            elapsed_us: 300,
        };

        let evidence = inductance_sensitivity(
            fit,
            RDropModel {
                r_count_slope_milli: 1_000,
                ud_count: 2_000,
                perturb_permille: 200,
            },
        )
        .expect("nonzero slope should produce evidence");

        assert_eq!(evidence.l_nominal_us, 4_750);
        assert_eq!(evidence.l_shift_minus_permille, 11);
        assert_eq!(evidence.l_shift_plus_permille, 11);
        assert!(evidence.passes_3_percent_gate());
    }

    #[test]
    fn theta_onset_uses_same_capture_samples() {
        let samples = [
            ElectricalCaptureSample::new(0, 0, 1024, 0, 0, 100, 0),
            ElectricalCaptureSample::new(40, 0, 1024, 2, 0, 100, 0),
            ElectricalCaptureSample::new(80, 0, 1024, 8, 0, 100, 0),
            ElectricalCaptureSample::new(120, 0, 1024, 20, 0, 100, 0),
        ];

        let onset = detect_theta_onset_us(&samples, OnsetThreshold { delta_count: 8 });
        assert_eq!(onset, Some(80));
    }
}
