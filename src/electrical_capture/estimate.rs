use super::{ElectricalCaptureSample, R_SENSITIVITY_MAX_L_SHIFT_PERMILLE};

const MICROS_PER_SECOND: i128 = 1_000_000;
const TAU_MILLIRADIANS: u128 = 6_283;
const MAX_FIT_TIMESTAMP_OFFSET_US: u64 = u32::MAX as u64;
const ELECTRICAL_TURN_COUNTS: u32 = 65_536;
const SALIENCY_Q15_SCALE: i128 = 32_767;
const MIN_SALIENCY_SAMPLE_COUNT: u16 = 8;

const COS_2THETA_Q15: [i16; 64] = [
    32767, 32137, 30273, 27245, 23170, 18204, 12539, 6393, 0, -6393, -12539, -18204, -23170,
    -27245, -30273, -32137, -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6393, 0, 6393,
    12539, 18204, 23170, 27245, 30273, 32137, 32767, 32137, 30273, 27245, 23170, 18204, 12539,
    6393, 0, -6393, -12539, -18204, -23170, -27245, -30273, -32137, -32767, -32137, -30273, -27245,
    -23170, -18204, -12539, -6393, 0, 6393, 12539, 18204, 23170, 27245, 30273, 32137,
];

const SIN_2THETA_Q15: [i16; 64] = [
    0, 6393, 12539, 18204, 23170, 27245, 30273, 32137, 32767, 32137, 30273, 27245, 23170, 18204,
    12539, 6393, 0, -6393, -12539, -18204, -23170, -27245, -30273, -32137, -32767, -32137, -30273,
    -27245, -23170, -18204, -12539, -6393, 0, 6393, 12539, 18204, 23170, 27245, 30273, 32137,
    32767, 32137, 30273, 27245, 23170, 18204, 12539, 6393, 0, -6393, -12539, -18204, -23170,
    -27245, -30273, -32137, -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6393,
];

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
    /// Fitted intercept in micro-counts at the first fitted timestamp.
    pub intercept_micro_count: i64,
    /// Root-mean-square fit residual in current counts.
    pub residual_rms_count: u32,
    /// Root-mean-square fit residual in micro-counts.
    pub residual_rms_micro_count: u64,
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

/// Averaged rotating-frame AC current evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AcFrameEvidence {
    /// Number of post-settle AC samples accumulated.
    pub sample_count: u16,
    /// Elapsed AC averaging window in microseconds.
    pub ac_elapsed_us: u32,
    /// Realized electrical frequency in millihertz.
    pub realized_frequency_millihz: u32,
    /// Measured open-loop phi delta over the averaging window.
    pub openloop_phi_delta_counts: u32,
    /// Mean d-axis current in milli-counts.
    pub id_mean_milli_count: i32,
    /// Mean q-axis current in milli-counts.
    pub iq_mean_milli_count: i32,
    /// RMS d-axis spread around the mean in milli-counts.
    pub id_rms_milli_count: u32,
    /// RMS q-axis spread around the mean in milli-counts.
    pub iq_rms_milli_count: u32,
}

/// Count-space AC reactance evidence for the rotating-frame estimator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AcReactanceEvidence {
    /// R-free quadrature reactance ratio in milli count-ratio units.
    pub x_quad_count_ratio_milli: u32,
    /// Nominal magnitude-branch reactance ratio in milli count-ratio units.
    pub x_mag_nominal_count_ratio_milli: u32,
    /// Magnitude-branch shift with R scaled down by 20%, in permille.
    pub x_mag_shift_minus_permille: u16,
    /// Magnitude-branch shift with R scaled up by 20%, in permille.
    pub x_mag_shift_plus_permille: u16,
    /// Nominal magnitude-vs-quadrature disagreement in permille.
    pub x_mag_vs_quad_permille: u16,
}

/// Saliency evidence could not be extracted from this capture.
pub const AC_SALIENCY_NOT_DETERMINABLE: u8 = 0;
/// Saliency evidence was extracted and passed basic sanity checks.
pub const AC_SALIENCY_DETERMINABLE: u8 = 1;

/// Count-space Ld/Lq reactance evidence extracted from the rotating capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AcSaliencyEvidence {
    /// `AC_SALIENCY_*` status code.
    pub status: u8,
    /// Accepted angle-averaged reactance ratio in milli count-ratio units.
    pub x_average_count_ratio_milli: u32,
    /// D-axis reactance ratio in milli count-ratio units, valid when determinable.
    pub x_d_count_ratio_milli: u32,
    /// Q-axis reactance ratio in milli count-ratio units, valid when determinable.
    pub x_q_count_ratio_milli: u32,
    /// `abs(Ld - Lq) / average(Ld, Lq)` in permille, valid when determinable.
    pub saliency_permille: u16,
}

impl AcSaliencyEvidence {
    /// Build explicit not-determinable evidence without inventing Ld/Lq values.
    pub const fn not_determinable(x_average_count_ratio_milli: u32) -> Self {
        Self {
            status: AC_SALIENCY_NOT_DETERMINABLE,
            x_average_count_ratio_milli,
            x_d_count_ratio_milli: 0,
            x_q_count_ratio_milli: 0,
            saliency_permille: 0,
        }
    }

    /// Return true when the accepted average is bracketed by the two axes.
    pub fn brackets_average(&self) -> bool {
        if self.status != AC_SALIENCY_DETERMINABLE {
            return false;
        }
        let low = self.x_d_count_ratio_milli.min(self.x_q_count_ratio_milli);
        let high = self.x_d_count_ratio_milli.max(self.x_q_count_ratio_milli);
        low <= self.x_average_count_ratio_milli && self.x_average_count_ratio_milli <= high
    }
}

/// Inputs to the second-harmonic saliency extraction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct SaliencySecondHarmonicInput {
    /// Accepted angle-averaged reactance ratio in milli count-ratio units.
    pub x_average_count_ratio_milli: u32,
    /// Normalized `2 * mean((x - avg) * cos(2theta))` coefficient.
    pub cos2_milli: i32,
    /// Normalized `2 * mean((x - avg) * sin(2theta))` coefficient.
    pub sin2_milli: i32,
    /// Number of samples that fed the harmonic correlation.
    pub sample_count: u16,
    /// Phi-domain window width used for the correlation.
    pub phi_delta_counts: u32,
}

/// Phi-domain second-harmonic accumulator for saliency evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcSaliencyAccumulator {
    sample_count: u32,
    weight_sum: u32,
    sum_x_milli: i128,
    sum_cos_q15: i128,
    sum_sin_q15: i128,
    sum_x_cos2: i128,
    sum_x_sin2: i128,
}

impl AcSaliencyAccumulator {
    /// Construct an empty saliency accumulator.
    pub const fn new() -> Self {
        Self {
            sample_count: 0,
            weight_sum: 0,
            sum_x_milli: 0,
            sum_cos_q15: 0,
            sum_sin_q15: 0,
            sum_x_cos2: 0,
            sum_x_sin2: 0,
        }
    }

    /// Add one weighted rotating-frame current sample to the 2theta correlation.
    pub fn push_weighted_sample(
        &mut self,
        openloop_phi: u16,
        sample_weight_counts: u32,
        ud_count: i16,
        id_count: i16,
        iq_count: i16,
    ) -> Option<()> {
        if sample_weight_counts == 0 {
            return None;
        }

        let x_milli = i128::from(sample_quadrature_reactance_milli(
            ud_count, id_count, iq_count,
        )?);
        let weight = i128::from(sample_weight_counts);
        let cos = i128::from(cos_2theta_q15(openloop_phi));
        let sin = i128::from(sin_2theta_q15(openloop_phi));

        self.sample_count = self.sample_count.checked_add(1)?;
        self.weight_sum = self.weight_sum.checked_add(sample_weight_counts)?;
        self.sum_x_milli = self.sum_x_milli.checked_add(x_milli.checked_mul(weight)?)?;
        self.sum_cos_q15 = self.sum_cos_q15.checked_add(cos.checked_mul(weight)?)?;
        self.sum_sin_q15 = self.sum_sin_q15.checked_add(sin.checked_mul(weight)?)?;
        self.sum_x_cos2 = self
            .sum_x_cos2
            .checked_add(x_milli.checked_mul(cos)?.checked_mul(weight)?)?;
        self.sum_x_sin2 = self
            .sum_x_sin2
            .checked_add(x_milli.checked_mul(sin)?.checked_mul(weight)?)?;
        Some(())
    }

    /// Finish the phi-domain correlation.
    pub fn finish(&self, phi_delta_counts: u32) -> Option<SaliencySecondHarmonicInput> {
        if self.weight_sum == 0
            || self.weight_sum != phi_delta_counts
            || phi_delta_counts == 0
            || !phi_delta_counts.is_multiple_of(ELECTRICAL_TURN_COUNTS)
            || self.sample_count < u32::from(MIN_SALIENCY_SAMPLE_COUNT)
            || self.sample_count > u32::from(u16::MAX)
        {
            return None;
        }

        let weight_sum = i128::from(self.weight_sum);
        let x_average = self.sum_x_milli.checked_div(weight_sum)?;
        let centered_cos = self
            .sum_x_cos2
            .checked_sub(x_average.checked_mul(self.sum_cos_q15)?)?;
        let centered_sin = self
            .sum_x_sin2
            .checked_sub(x_average.checked_mul(self.sum_sin_q15)?)?;
        let denominator = weight_sum.checked_mul(SALIENCY_Q15_SCALE)?;
        let cos2 = centered_cos.checked_mul(2)?.checked_div(denominator)?;
        let sin2 = centered_sin.checked_mul(2)?.checked_div(denominator)?;

        Some(SaliencySecondHarmonicInput {
            x_average_count_ratio_milli: u32::try_from(x_average).ok()?,
            cos2_milli: i32::try_from(cos2).ok()?,
            sin2_milli: i32::try_from(sin2).ok()?,
            sample_count: u16::try_from(self.sample_count).ok()?,
            phi_delta_counts,
        })
    }
}

impl Default for AcSaliencyAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl defmt::Format for AcSaliencyAccumulator {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "AcSaliencyAccumulator {{ sample_count: {}, weight_sum: {} }}",
            self.sample_count,
            self.weight_sum
        );
    }
}

/// Extract Ld/Lq reactance evidence from normalized 2theta correlation terms.
pub fn extract_saliency_from_second_harmonic(
    input: SaliencySecondHarmonicInput,
) -> Option<AcSaliencyEvidence> {
    if input.sample_count < MIN_SALIENCY_SAMPLE_COUNT
        || input.phi_delta_counts == 0
        || !input
            .phi_delta_counts
            .is_multiple_of(ELECTRICAL_TURN_COUNTS)
        || input.x_average_count_ratio_milli == 0
    {
        return None;
    }

    let cos = i128::from(input.cos2_milli);
    let sin = i128::from(input.sin2_milli);
    let amplitude_sq =
        u128::try_from(cos.checked_mul(cos)?.checked_add(sin.checked_mul(sin)?)?).ok()?;
    let amplitude = integer_sqrt(amplitude_sq);
    let average = u128::from(input.x_average_count_ratio_milli);
    if amplitude > average {
        return None;
    }

    let x_d = average.checked_add(amplitude)?;
    let x_q = average.checked_sub(amplitude)?;
    let saliency_permille = amplitude.checked_mul(2_000)?.checked_div(average)?;

    Some(AcSaliencyEvidence {
        status: AC_SALIENCY_DETERMINABLE,
        x_average_count_ratio_milli: input.x_average_count_ratio_milli,
        x_d_count_ratio_milli: u32::try_from(x_d).ok()?,
        x_q_count_ratio_milli: u32::try_from(x_q).ok()?,
        saliency_permille: u16::try_from(saliency_permille).ok()?,
    })
}

/// One weighted interval for a phi-locked AC accumulation window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PhiLockedAcInterval {
    /// Forward phi movement observed since the previous sample.
    pub observed_phi_delta_counts: u32,
    /// Phi-delta weight this sample contributes to the target window.
    pub sample_weight_counts: u32,
    /// Accumulated phi movement after applying this interval.
    pub accumulated_phi_delta_counts: u32,
    /// True when this interval closes the requested phi window.
    pub complete: bool,
}

/// Streaming accumulator for post-settle rotating-frame AC samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcFrameAccumulator {
    sample_count: u32,
    weight_sum: u32,
    sum_id: i64,
    sum_iq: i64,
    sum_id_sq: i128,
    sum_iq_sq: i128,
}

impl AcFrameAccumulator {
    /// Construct an empty accumulator.
    pub const fn new() -> Self {
        Self {
            sample_count: 0,
            weight_sum: 0,
            sum_id: 0,
            sum_iq: 0,
            sum_id_sq: 0,
            sum_iq_sq: 0,
        }
    }

    /// Add one `(Id, Iq)` rotating-frame current sample.
    pub fn push(&mut self, id_count: i16, iq_count: i16) -> Option<()> {
        self.push_weighted(id_count, iq_count, 1)
    }

    /// Add one `(Id, Iq)` rotating-frame current sample with a phi-delta weight.
    pub fn push_weighted(&mut self, id_count: i16, iq_count: i16, weight: u32) -> Option<()> {
        if weight == 0 {
            return None;
        }

        self.sample_count = self.sample_count.checked_add(1)?;
        self.weight_sum = self.weight_sum.checked_add(weight)?;

        let weight_i64 = i64::from(weight);
        self.sum_id = self
            .sum_id
            .checked_add(i64::from(id_count).checked_mul(weight_i64)?)?;
        self.sum_iq = self
            .sum_iq
            .checked_add(i64::from(iq_count).checked_mul(weight_i64)?)?;

        let id = i128::from(id_count);
        let iq = i128::from(iq_count);
        let weight_i128 = i128::from(weight);
        self.sum_id_sq = self
            .sum_id_sq
            .checked_add(id.checked_mul(id)?.checked_mul(weight_i128)?)?;
        self.sum_iq_sq = self
            .sum_iq_sq
            .checked_add(iq.checked_mul(iq)?.checked_mul(weight_i128)?)?;
        Some(())
    }

    /// Finish the accumulation and return averaged AC evidence.
    pub fn finish(
        self,
        ac_elapsed_us: u32,
        realized_frequency_millihz: u32,
        openloop_phi_delta_counts: u32,
    ) -> Option<AcFrameEvidence> {
        if self.sample_count == 0 || self.sample_count > u32::from(u16::MAX) || self.weight_sum == 0
        {
            return None;
        }

        Some(AcFrameEvidence {
            sample_count: u16::try_from(self.sample_count).ok()?,
            ac_elapsed_us,
            realized_frequency_millihz,
            openloop_phi_delta_counts,
            id_mean_milli_count: mean_milli_count(self.sum_id, self.weight_sum)?,
            iq_mean_milli_count: mean_milli_count(self.sum_iq, self.weight_sum)?,
            id_rms_milli_count: rms_milli_count(self.sum_id, self.sum_id_sq, self.weight_sum)?,
            iq_rms_milli_count: rms_milli_count(self.sum_iq, self.sum_iq_sq, self.weight_sum)?,
        })
    }
}

impl Default for AcFrameAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl defmt::Format for AcFrameAccumulator {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "AcFrameAccumulator {{ sample_count: {}, weight_sum: {} }}",
            self.sample_count,
            self.weight_sum
        );
    }
}

/// Estimate electrical frequency from the current ramp slope and mean current.
///
/// Returns `None` when the mean current is at or below the supplied noise floor
/// or when the computed frequency does not fit in `u32`.
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
///
/// Returns `None` for invalid windows, windows with fewer than two samples,
/// non-monotonic timestamps, timestamp offsets outside the supported arithmetic
/// domain, degenerate timing, or checked-arithmetic overflow.
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
    if elapsed_us == 0 || elapsed_us > MAX_FIT_TIMESTAMP_OFFSET_US {
        return None;
    }

    let mut sum_x = 0_i128;
    let mut sum_x2 = 0_i128;
    let mut sum_y = 0_i128;
    let mut sum_xy = 0_i128;
    let mut previous_timestamp_us = first_timestamp_us;

    for sample in fit_samples {
        if sample.timestamp_us < previous_timestamp_us {
            return None;
        }
        previous_timestamp_us = sample.timestamp_us;

        let offset_us = sample.timestamp_us.checked_sub(first_timestamp_us)?;
        if offset_us > MAX_FIT_TIMESTAMP_OFFSET_US {
            return None;
        }

        let x = i128::from(offset_us);
        let y = i128::from(sample.current_primary);

        sum_x = sum_x.checked_add(x)?;
        sum_x2 = sum_x2.checked_add(x.checked_mul(x)?)?;
        sum_y = sum_y.checked_add(y)?;
        sum_xy = sum_xy.checked_add(x.checked_mul(y)?)?;
    }

    let sample_count = i128::try_from(fit_samples.len()).ok()?;
    let slope_numerator = sample_count
        .checked_mul(sum_xy)?
        .checked_sub(sum_x.checked_mul(sum_y)?)?;
    let slope_denominator = sample_count
        .checked_mul(sum_x2)?
        .checked_sub(sum_x.checked_mul(sum_x)?)?;
    if slope_denominator == 0 {
        return None;
    }

    let slope_count_per_s = slope_numerator.checked_mul(MICROS_PER_SECOND)? / slope_denominator;
    let intercept_micro_count = sum_y
        .checked_mul(MICROS_PER_SECOND)?
        .checked_sub(slope_count_per_s.checked_mul(sum_x)?)?
        / sample_count;

    let mut residual_square_sum = 0_u128;
    for sample in fit_samples {
        let x = i128::from(sample.timestamp_us.checked_sub(first_timestamp_us)?);
        let observed_micro_count =
            i128::from(sample.current_primary).checked_mul(MICROS_PER_SECOND)?;
        let fitted_micro_count =
            intercept_micro_count.checked_add(slope_count_per_s.checked_mul(x)?)?;
        let residual = observed_micro_count.checked_sub(fitted_micro_count)?;
        let residual_abs = residual.unsigned_abs();
        residual_square_sum =
            residual_square_sum.checked_add(residual_abs.checked_mul(residual_abs)?)?;
    }

    let residual_mean_square = residual_square_sum / u128::try_from(fit_samples.len()).ok()?;
    let residual_rms_micro_count = integer_sqrt(residual_mean_square);

    Some(LinearFit {
        slope_count_per_s: i64::try_from(slope_count_per_s).ok()?,
        mean_current_count: i32::try_from(sum_y / sample_count).ok()?,
        intercept_count: round_micro_count_to_count(intercept_micro_count)?,
        intercept_micro_count: i64::try_from(intercept_micro_count).ok()?,
        residual_rms_count: round_micro_residual_to_count(residual_rms_micro_count)?,
        residual_rms_micro_count: u64::try_from(residual_rms_micro_count).ok()?,
        sample_count: u16::try_from(fit_samples.len()).ok()?,
        elapsed_us: u32::try_from(elapsed_us).ok()?,
    })
}

/// Estimate nominal inductance and sensitivity to R-model perturbation.
///
/// Returns `None` when the slope or nominal inductance is zero, the R
/// perturbation exceeds 1000 permille, or checked arithmetic/conversions fail.
pub fn inductance_sensitivity(fit: LinearFit, model: RDropModel) -> Option<InductanceEvidence> {
    if model.perturb_permille > 1_000 {
        return None;
    }

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
///
/// Returns `None` when there are no samples or no sample reaches the threshold.
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

/// Compute rotating-frame AC reactance evidence from averaged current.
pub fn rotating_frame_reactance(
    evidence: AcFrameEvidence,
    ud_count: i16,
    r_count_slope_milli: i32,
) -> Option<AcReactanceEvidence> {
    let u_count = i128::from(ud_count).unsigned_abs();
    let id = i128::from(evidence.id_mean_milli_count);
    let iq = i128::from(evidence.iq_mean_milli_count);
    let id_sq = id.checked_mul(id)?;
    let iq_sq = iq.checked_mul(iq)?;
    let current_mag_sq = u128::try_from(id_sq.checked_add(iq_sq)?).ok()?;
    if u_count == 0 || current_mag_sq == 0 {
        return None;
    }

    let x_quad = u_count
        .checked_mul(iq.unsigned_abs())?
        .checked_mul(1_000_000)?
        .checked_div(current_mag_sq)?;
    if x_quad == 0 {
        return None;
    }

    let current_mag = integer_sqrt(current_mag_sq);
    if current_mag == 0 {
        return None;
    }

    let z_milli = u_count.checked_mul(1_000_000)?.checked_div(current_mag)?;
    let nominal = magnitude_reactance_milli(z_milli, r_count_slope_milli, 1_000)?;
    if nominal == 0 {
        return None;
    }

    let minus = magnitude_reactance_milli(z_milli, r_count_slope_milli, 800)?;
    let plus = magnitude_reactance_milli(z_milli, r_count_slope_milli, 1_200)?;
    let x_quad = u64::try_from(x_quad).ok()?;

    Some(AcReactanceEvidence {
        x_quad_count_ratio_milli: u32::try_from(x_quad).ok()?,
        x_mag_nominal_count_ratio_milli: u32::try_from(nominal).ok()?,
        x_mag_shift_minus_permille: shift_permille(nominal, minus)?,
        x_mag_shift_plus_permille: shift_permille(nominal, plus)?,
        x_mag_vs_quad_permille: shift_permille(x_quad, nominal)?,
    })
}

fn sample_quadrature_reactance_milli(ud_count: i16, id_count: i16, iq_count: i16) -> Option<u32> {
    let u_count = u128::from(ud_count.unsigned_abs());
    let id = i128::from(id_count);
    let iq = i128::from(iq_count);
    let id_sq = id.checked_mul(id)?;
    let iq_sq = iq.checked_mul(iq)?;
    let current_mag_sq = u128::try_from(id_sq.checked_add(iq_sq)?).ok()?;
    if u_count == 0 || current_mag_sq == 0 {
        return None;
    }

    let x_milli = u_count
        .checked_mul(iq.unsigned_abs())?
        .checked_mul(1_000)?
        .checked_div(current_mag_sq)?;
    u32::try_from(x_milli).ok()
}

fn cos_2theta_q15(openloop_phi: u16) -> i16 {
    COS_2THETA_Q15[usize::from(openloop_phi >> 10)]
}

fn sin_2theta_q15(openloop_phi: u16) -> i16 {
    SIN_2THETA_Q15[usize::from(openloop_phi >> 10)]
}

/// Return the forward 16-bit electrical-angle delta from `previous` to `current`.
pub fn phi_delta_counts_forward(previous: u16, current: u16) -> u32 {
    u32::from(current.wrapping_sub(previous))
}

/// Compute one sample's contribution to a phi-locked AC averaging window.
pub fn phi_locked_ac_interval(
    accumulated_phi_delta_counts: u32,
    target_phi_delta_counts: u32,
    previous_phi: u16,
    current_phi: u16,
) -> Option<PhiLockedAcInterval> {
    if accumulated_phi_delta_counts >= target_phi_delta_counts {
        return None;
    }

    let observed_phi_delta_counts = phi_delta_counts_forward(previous_phi, current_phi);
    let remaining_phi_delta_counts =
        target_phi_delta_counts.checked_sub(accumulated_phi_delta_counts)?;
    let sample_weight_counts = observed_phi_delta_counts.min(remaining_phi_delta_counts);
    let accumulated_phi_delta_counts =
        accumulated_phi_delta_counts.checked_add(sample_weight_counts)?;

    Some(PhiLockedAcInterval {
        observed_phi_delta_counts,
        sample_weight_counts,
        accumulated_phi_delta_counts,
        complete: accumulated_phi_delta_counts >= target_phi_delta_counts,
    })
}

/// Interpolate the elapsed time where a phi-locked interval reached its target.
pub fn interpolate_phi_window_elapsed_us(
    previous_elapsed_us: u32,
    current_elapsed_us: u32,
    interval: PhiLockedAcInterval,
) -> Option<u32> {
    if interval.observed_phi_delta_counts == 0 {
        return None;
    }
    let elapsed_delta_us = current_elapsed_us.checked_sub(previous_elapsed_us)?;
    let elapsed = u128::from(previous_elapsed_us).checked_add(
        u128::from(elapsed_delta_us)
            .checked_mul(u128::from(interval.sample_weight_counts))?
            .checked_add(u128::from(interval.observed_phi_delta_counts / 2))?
            .checked_div(u128::from(interval.observed_phi_delta_counts))?,
    )?;
    u32::try_from(elapsed).ok()
}

/// Compute realized electrical frequency from measured phi movement and time.
pub fn realized_frequency_millihz(phi_delta_counts: u32, elapsed_us: u32) -> Option<u32> {
    if elapsed_us == 0 {
        return None;
    }
    let numerator = u128::from(phi_delta_counts).checked_mul(1_000_000_000)?;
    let denominator = u128::from(u16::MAX as u32 + 1).checked_mul(u128::from(elapsed_us))?;
    let millihz = numerator
        .checked_add(denominator.checked_div(2)?)?
        .checked_div(denominator)?;
    u32::try_from(millihz).ok()
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

fn mean_milli_count(sum: i64, count: u32) -> Option<i32> {
    let mean = i128::from(sum)
        .checked_mul(1_000)?
        .checked_div(i128::from(count))?;
    i32::try_from(mean).ok()
}

fn rms_milli_count(sum: i64, sum_sq: i128, count: u32) -> Option<u32> {
    let count_i128 = i128::from(count);
    let numerator = count_i128
        .checked_mul(sum_sq)?
        .checked_sub(i128::from(sum).checked_mul(i128::from(sum))?)?;
    let numerator = u128::try_from(numerator).ok()?;
    let denominator = u128::from(count).checked_mul(u128::from(count))?;
    let mean_square_milli = numerator.checked_mul(1_000_000)?.checked_div(denominator)?;
    u32::try_from(integer_sqrt(mean_square_milli)).ok()
}

fn magnitude_reactance_milli(
    z_milli: u128,
    r_count_slope_milli: i32,
    r_scale_permille: i32,
) -> Option<u64> {
    if r_count_slope_milli < 0 || r_scale_permille < 0 {
        return None;
    }

    let r_milli = u128::try_from(r_count_slope_milli)
        .ok()?
        .checked_mul(u128::try_from(r_scale_permille).ok()?)?
        .checked_div(1_000)?;
    let z_sq = z_milli.checked_mul(z_milli)?;
    let r_sq = r_milli.checked_mul(r_milli)?;
    let x_sq = z_sq.checked_sub(r_sq)?;
    u64::try_from(integer_sqrt(x_sq)).ok()
}

fn div_ceil_u64(numerator: u64, denominator: u64) -> u64 {
    if numerator == 0 {
        0
    } else {
        1 + ((numerator - 1) / denominator)
    }
}

fn round_micro_count_to_count(micro_count: i128) -> Option<i32> {
    let rounded = if micro_count >= 0 {
        (micro_count + (MICROS_PER_SECOND / 2)) / MICROS_PER_SECOND
    } else {
        (micro_count - (MICROS_PER_SECOND / 2)) / MICROS_PER_SECOND
    };

    i32::try_from(rounded).ok()
}

fn round_micro_residual_to_count(micro_count: u128) -> Option<u32> {
    let rounded = (micro_count + u128::try_from(MICROS_PER_SECOND / 2).ok()?)
        / u128::try_from(MICROS_PER_SECOND).ok()?;

    u32::try_from(rounded).ok()
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
    fn linear_fit_preserves_fractional_residual_evidence() {
        let samples = [
            ElectricalCaptureSample::new(0, 0, 1024, 0, 0, 100, 0),
            ElectricalCaptureSample::new(100, 0, 1024, 1, 0, 100, 0),
            ElectricalCaptureSample::new(200, 0, 1024, 1, 0, 100, 0),
            ElectricalCaptureSample::new(300, 0, 1024, 2, 0, 100, 0),
        ];

        let fit = fit_current_slope(
            &samples,
            FitWindow {
                start_index: 0,
                end_index: 4,
            },
        )
        .expect("fractional residual samples should fit");

        assert_eq!(fit.intercept_count, 0);
        assert_eq!(fit.intercept_micro_count, 100_000);
        assert_eq!(fit.residual_rms_count, 0);
        assert!(fit.residual_rms_micro_count > 0);
    }

    #[test]
    fn linear_fit_rejects_out_of_order_timestamps() {
        let samples = [
            ElectricalCaptureSample::new(0, 0, 1024, 0, 0, 100, 0),
            ElectricalCaptureSample::new(200, 0, 1024, 1, 0, 100, 0),
            ElectricalCaptureSample::new(100, 0, 1024, 2, 0, 100, 0),
        ];

        assert_eq!(
            fit_current_slope(
                &samples,
                FitWindow {
                    start_index: 0,
                    end_index: 3,
                },
            ),
            None
        );
    }

    #[test]
    fn linear_fit_rejects_oversized_intermediate_timestamp() {
        let samples = [
            ElectricalCaptureSample::new(0, 0, 1024, 0, 0, 100, 0),
            ElectricalCaptureSample::new(u64::MAX, 0, 1024, 1, 0, 100, 0),
            ElectricalCaptureSample::new(1, 0, 1024, 2, 0, 100, 0),
        ];

        assert_eq!(
            fit_current_slope(
                &samples,
                FitWindow {
                    start_index: 0,
                    end_index: 3,
                },
            ),
            None
        );
    }

    #[test]
    fn r_sensitivity_reports_l_shift_permille() {
        let fit = LinearFit {
            slope_count_per_s: 400_000,
            mean_current_count: 100,
            intercept_count: 10,
            intercept_micro_count: 10_000_000,
            residual_rms_count: 0,
            residual_rms_micro_count: 0,
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
    fn r_sensitivity_accepts_full_negative_perturbation_boundary() {
        let fit = LinearFit {
            slope_count_per_s: 400_000,
            mean_current_count: 100,
            intercept_count: 10,
            intercept_micro_count: 10_000_000,
            residual_rms_count: 0,
            residual_rms_micro_count: 0,
            sample_count: 4,
            elapsed_us: 300,
        };

        assert!(
            inductance_sensitivity(
                fit,
                RDropModel {
                    r_count_slope_milli: 1_000,
                    ud_count: 2_000,
                    perturb_permille: 1_000,
                },
            )
            .is_some()
        );
    }

    #[test]
    fn r_sensitivity_rejects_over_full_negative_perturbation() {
        let fit = LinearFit {
            slope_count_per_s: 400_000,
            mean_current_count: 100,
            intercept_count: 10,
            intercept_micro_count: 10_000_000,
            residual_rms_count: 0,
            residual_rms_micro_count: 0,
            sample_count: 4,
            elapsed_us: 300,
        };

        assert_eq!(
            inductance_sensitivity(
                fit,
                RDropModel {
                    r_count_slope_milli: 1_000,
                    ud_count: 2_000,
                    perturb_permille: 1_001,
                },
            ),
            None
        );
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

    #[test]
    fn ac_accumulator_reports_milli_count_means_and_rms() {
        let mut acc = AcFrameAccumulator::new();
        acc.push(9, -101).expect("first sample");
        acc.push(11, -99).expect("second sample");

        let evidence = acc.finish(8_000, 1_000_000, 524_288).expect("evidence");

        assert_eq!(evidence.sample_count, 2);
        assert_eq!(evidence.id_mean_milli_count, 10_000);
        assert_eq!(evidence.iq_mean_milli_count, -100_000);
        assert_eq!(evidence.id_rms_milli_count, 1_000);
        assert_eq!(evidence.iq_rms_milli_count, 1_000);
        assert_eq!(evidence.ac_elapsed_us, 8_000);
        assert_eq!(evidence.realized_frequency_millihz, 1_000_000);
        assert_eq!(evidence.openloop_phi_delta_counts, 524_288);
    }

    #[test]
    fn ac_accumulator_weights_samples_by_phi_delta() {
        let mut acc = AcFrameAccumulator::new();
        acc.push_weighted(0, 0, 1).expect("first interval");
        acc.push_weighted(10, -20, 3).expect("second interval");

        let evidence = acc.finish(8_000, 1_000_000, 4).expect("evidence");

        assert_eq!(evidence.sample_count, 2);
        assert_eq!(evidence.id_mean_milli_count, 7_500);
        assert_eq!(evidence.iq_mean_milli_count, -15_000);
        assert_eq!(evidence.id_rms_milli_count, 4_330);
        assert_eq!(evidence.iq_rms_milli_count, 8_660);
        assert_eq!(evidence.openloop_phi_delta_counts, 4);
    }

    #[test]
    fn ac_accumulator_rejects_empty_window() {
        let acc = AcFrameAccumulator::new();
        assert_eq!(acc.finish(8_000, 1_000_000, 524_288), None);
    }

    #[test]
    fn rotating_frame_reactance_reports_r_free_primary_and_magnitude_gate() {
        let evidence = AcFrameEvidence {
            sample_count: 200,
            ac_elapsed_us: 8_000,
            realized_frequency_millihz: 1_000_000,
            openloop_phi_delta_counts: 524_288,
            id_mean_milli_count: 10_000,
            iq_mean_milli_count: -100_000,
            id_rms_milli_count: 300,
            iq_rms_milli_count: 500,
        };

        let reactance = rotating_frame_reactance(evidence, 768, 500).expect("reactance");

        assert_eq!(reactance.x_quad_count_ratio_milli, 7_603);
        assert!(reactance.x_mag_nominal_count_ratio_milli >= 7_615);
        assert!(reactance.x_mag_nominal_count_ratio_milli <= 7_635);
        assert!(reactance.x_mag_shift_minus_permille <= 3);
        assert!(reactance.x_mag_shift_plus_permille <= 3);
        assert!(reactance.x_mag_vs_quad_permille <= 3);
    }

    #[test]
    fn rotating_frame_reactance_rejects_zero_current_magnitude() {
        let evidence = AcFrameEvidence {
            sample_count: 1,
            ac_elapsed_us: 8_000,
            realized_frequency_millihz: 1_000_000,
            openloop_phi_delta_counts: 524_288,
            id_mean_milli_count: 0,
            iq_mean_milli_count: 0,
            id_rms_milli_count: 0,
            iq_rms_milli_count: 0,
        };

        assert_eq!(rotating_frame_reactance(evidence, 768, 500), None);
    }

    #[test]
    fn saliency_reactance_brackets_average_when_determinable() {
        let evidence = AcSaliencyEvidence {
            status: AC_SALIENCY_DETERMINABLE,
            x_average_count_ratio_milli: 8_600,
            x_d_count_ratio_milli: 9_200,
            x_q_count_ratio_milli: 8_000,
            saliency_permille: 140,
        };

        assert!(evidence.brackets_average());
    }

    #[test]
    fn saliency_reactance_rejects_average_outside_ld_lq() {
        let evidence = AcSaliencyEvidence {
            status: AC_SALIENCY_DETERMINABLE,
            x_average_count_ratio_milli: 9_500,
            x_d_count_ratio_milli: 9_200,
            x_q_count_ratio_milli: 8_000,
            saliency_permille: 140,
        };

        assert!(!evidence.brackets_average());
    }

    #[test]
    fn saliency_reactance_allows_not_determinable_without_fake_axes() {
        let evidence = AcSaliencyEvidence::not_determinable(8_600);

        assert_eq!(evidence.status, AC_SALIENCY_NOT_DETERMINABLE);
        assert_eq!(evidence.x_average_count_ratio_milli, 8_600);
        assert_eq!(evidence.x_d_count_ratio_milli, 0);
        assert_eq!(evidence.x_q_count_ratio_milli, 0);
        assert!(!evidence.brackets_average());
    }

    #[test]
    fn saliency_second_harmonic_extractor_brackets_average() {
        let evidence = extract_saliency_from_second_harmonic(SaliencySecondHarmonicInput {
            x_average_count_ratio_milli: 8_600,
            cos2_milli: 600,
            sin2_milli: 800,
            sample_count: 104,
            phi_delta_counts: 8 * 65_536,
        })
        .expect("determinable saliency");

        assert_eq!(evidence.status, AC_SALIENCY_DETERMINABLE);
        assert_eq!(evidence.x_average_count_ratio_milli, 8_600);
        assert_eq!(evidence.x_d_count_ratio_milli, 9_600);
        assert_eq!(evidence.x_q_count_ratio_milli, 7_600);
        assert_eq!(evidence.saliency_permille, 232);
        assert!(evidence.brackets_average());
    }

    #[test]
    fn saliency_second_harmonic_requires_integer_turn_coverage() {
        let evidence = extract_saliency_from_second_harmonic(SaliencySecondHarmonicInput {
            x_average_count_ratio_milli: 8_600,
            cos2_milli: 600,
            sin2_milli: 800,
            sample_count: 104,
            phi_delta_counts: 7 * 65_536 + 10_000,
        });

        assert!(evidence.is_none());
    }

    #[test]
    fn saliency_accumulator_extracts_second_harmonic_from_phi_samples() {
        let mut acc = AcSaliencyAccumulator::new();
        let ud_count = 768;
        let iq_by_quarter = [-78, -91, -78, -91];

        for _turn in 0..8 {
            for (quarter, iq_count) in iq_by_quarter.iter().copied().enumerate() {
                let phi = u16::try_from(quarter).unwrap().wrapping_mul(16_384);
                acc.push_weighted_sample(phi, 16_384, ud_count, 0, iq_count)
                    .expect("sample accepted");
            }
        }

        let input = acc
            .finish(8 * 65_536)
            .expect("determinable second-harmonic input");
        let evidence = extract_saliency_from_second_harmonic(input).expect("saliency evidence");

        assert_eq!(input.sample_count, 32);
        assert_eq!(input.x_average_count_ratio_milli, 9_142);
        assert!(input.cos2_milli >= 1_405);
        assert!(input.cos2_milli <= 1_410);
        assert_eq!(input.sin2_milli, 0);
        assert!(evidence.brackets_average());
        assert!(evidence.x_d_count_ratio_milli > evidence.x_q_count_ratio_milli);
    }

    #[test]
    fn phi_locked_ac_interval_truncates_final_wrap_interval() {
        let interval = phi_locked_ac_interval(400, 512, 0xFF00, 0x0100).expect("window interval");

        assert_eq!(interval.observed_phi_delta_counts, 512);
        assert_eq!(interval.sample_weight_counts, 112);
        assert_eq!(interval.accumulated_phi_delta_counts, 512);
        assert!(interval.complete);
        assert_eq!(
            interpolate_phi_window_elapsed_us(1_000, 1_100, interval),
            Some(1_022)
        );
    }

    #[test]
    fn phi_locked_ac_interval_rejects_completed_window() {
        assert_eq!(phi_locked_ac_interval(512, 512, 0, 1), None);
    }

    #[test]
    fn realized_frequency_uses_phi_delta_and_elapsed_time() {
        assert_eq!(realized_frequency_millihz(524_288, 8_000), Some(1_000_000));
        assert_eq!(realized_frequency_millihz(524_288, 0), None);
    }
}
