use super::{
    AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC,
    CommissionProfile, CurrentGainSynthesis, CurrentGainSynthesisError, CurrentGainSynthesisInput,
    CurrentGains, CurrentLoopEvidence, ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel,
    Permille, TransientBuffer, ValidationResult,
};
#[cfg(test)]
use super::{
    CURRENT_AXIS_TORQUE, CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, CURRENT_GAINS_SOURCE_DEFAULT,
    CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED, CURRENT_GAINS_TIER_DEFAULT,
    CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, CURRENT_GAINS_TIER_NONE,
    CURRENT_LOOP_FAILURE_FLUX_VALIDATION, CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS,
    CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN, CURRENT_LOOP_FAILURE_HOLD_SAMPLE,
    CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS, CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID,
    CURRENT_LOOP_FAILURE_MOTION, CURRENT_LOOP_FAILURE_NONE,
    CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED,
    CURRENT_LOOP_FAILURE_SATURATION, CURRENT_LOOP_FAILURE_SPI, CURRENT_LOOP_FAILURE_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT,
    CURRENT_VALIDATION_AXIS_FLUX, CURRENT_VALIDATION_AXIS_TORQUE,
    CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS,
};
use foci_tmc4671::pid::PidGain;

/// Raw settled-flux current-validation reads retained for audit.
pub const CURRENT_VALIDATION_SETTLED_MAX_SAMPLES: usize = 8;

/// Advanced-PI current-loop I zero scale.
///
/// With `CONFIG_ADVANCED_PI_REPRESENT=0`, current-loop I is Q8.8 and the
/// advanced PI integrator input is divided by another 256, so the effective
/// integral-zero term is `I_reg / 65536` per PWM sample.
pub const ADVANCED_PI_I_ZERO_SCALE: f32 = 65_536.0;

/// Stability detune applied to the pole-cancelling current-loop I zero.
///
/// `DETUNE=1` is true pole cancellation, but on the OpenFFBoard printer
/// cascade it would produce `I ~= 1216`, roughly 3x beyond the observed
/// sustained-hold squeal edge. The 2026-06-28 long-dwell current-loop artifact
/// showed `DETUNE ~= 3.2` (`I=384`) near the instability boundary, while
/// `DETUNE=8` (`I ~= 152-159`) closed the flux step cleanly in about 80-100 ms.
///
/// This is a fixed Phase-1 cascade stability margin, not a profile tuning knob.
/// A future sustained-hold gate must catch hardware where this margin is not
/// enough before any profile-dependent detune is introduced.
pub const CURRENT_I_STABILITY_DETUNE: f32 = 8.0;

/// Detuned current-loop I zero factor.
///
/// Commissioning stores `tau_e_us = L / R`, so Phase 1 uses
/// `CURRENT_I_DETUNED_ZERO_FACTOR / (tau_e_seconds * f_pwm)`, where this factor
/// is the advanced-PI zero scale divided by the fixed stability detune.
pub const CURRENT_I_DETUNED_ZERO_FACTOR: f32 =
    ADVANCED_PI_I_ZERO_SCALE / CURRENT_I_STABILITY_DETUNE;

/// Samples used to smooth current validation settling/ringing checks.
///
/// Current captures include PWM/ADC ripple around the regulated value. A short
/// trailing average keeps the validation focused on the controller envelope
/// while preserving raw samples for rise time and overshoot.
const CURRENT_METRIC_SMOOTHING_SAMPLES: usize = 8;

/// Metrics from a current-loop step-response validation test.
///
/// Extracted from the [`TransientBuffer`] after a torque step.
/// Used by [`classify_current_validation`] to determine pass/marginal/fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentValidation {
    /// Time from 10% to 90% of target, in microseconds.
    pub rise_time_us: u32,
    /// Peak overshoot above target, in 0.1% units (150 = 15.0%).
    pub overshoot_permille: Permille,
    /// Time to settle within +/-5% of target, in microseconds.
    pub settling_time_us: u32,
    /// Zero-crossings of (actual - target) after peak.
    pub ringing_count: u8,
    /// Samples where value hit the saturation limit.
    pub saturation_samples: u16,
    /// |fwd_tail - rev_tail| / avg, in 0.1% units (permille).
    /// Set to 0 by extract_step_metrics (requires two captures).
    pub symmetry_permille: Permille,
    /// Robust tail level: median of last 32 samples (or fewer if buffer is
    /// shorter). Used to compute steady-state signed-current symmetry between
    /// forward and reverse captures.
    pub tail_level: u32,
    /// |tail_level - target| / target, in 0.1% units (permille).
    /// Measures whether the current loop actually reached the commanded
    /// target. Extraction sets this to 0 for empty captures and nonzero
    /// targets, or to 100% for a non-empty zero-target capture. The caller
    /// computes it from the combined (worst-case) tail_level and commanded
    /// target.
    pub target_error_permille: Permille,
}

/// Axis sample passed validation.
pub const CURRENT_AXIS_VALIDATION_PASS: u8 = 0;
/// Same-axis current response had the opposite sign from the target.
pub const CURRENT_AXIS_VALIDATION_WRONG_SIGN: u8 = 1;
/// Same-axis current response was below the minimum expected response.
pub const CURRENT_AXIS_VALIDATION_LOW_RESPONSE: u8 = 2;
/// Same-axis current response exceeded the maximum expected response.
pub const CURRENT_AXIS_VALIDATION_HIGH_RESPONSE: u8 = 3;
/// Opposite-axis current response exceeded the allowed coupling limit.
pub const CURRENT_AXIS_VALIDATION_CROSS_AXIS: u8 = 4;
/// Voltage output exceeded the configured output limit.
pub const CURRENT_AXIS_VALIDATION_SATURATION: u8 = 5;
/// Encoder motion exceeded the allowed current-validation movement.
pub const CURRENT_AXIS_VALIDATION_MOTION: u8 = 6;
/// TMC4671 status flags contained an actionable fault bit.
pub const CURRENT_AXIS_VALIDATION_STATUS_FLAGS: u8 = 7;
/// Current target was zero, so response ratios cannot be classified.
pub const CURRENT_AXIS_VALIDATION_ZERO_TARGET: u8 = 8;

// Current validation directly gates sustained current-loop voltage from
// INTERIM_FOC_UQ_UD_LIMITED. The TMC PID output bits latch on step-edge clipping,
// so they stay as telemetry but are not the pass/fail saturation signal here.
const CURRENT_AXIS_POSITION_VELOCITY_STATUS_MASK: u32 =
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 6) | (1 << 7);
const CURRENT_AXIS_PID_OUTPUT_STATUS_MASK: u32 = (1 << 11) | (1 << 15);
const CURRENT_AXIS_ENCODER_PRESENCE_STATUS_MASK: u32 = (1 << 28) | (1 << 29) | (1 << 30);
// STATUS_FLAGS bit 31 is reserved in the TMC4671 status layout. It may appear
// in sampled flags, but it is not an actionable current-validation fault.
const CURRENT_AXIS_RESERVED_STATUS_MASK: u32 = 1 << 31;
const CURRENT_AXIS_NON_ACTIONABLE_STATUS_MASK: u32 = CURRENT_AXIS_POSITION_VELOCITY_STATUS_MASK
    | CURRENT_AXIS_PID_OUTPUT_STATUS_MASK
    | CURRENT_AXIS_ENCODER_PRESENCE_STATUS_MASK
    | CURRENT_AXIS_RESERVED_STATUS_MASK;

/// Status flags that fail current-loop validation and sustained-hold checks.
pub const fn current_loop_actionable_status_mask() -> u32 {
    !CURRENT_AXIS_NON_ACTIONABLE_STATUS_MASK
}

/// Retry action for one failed current-axis short-sample validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CurrentAxisRetryDecision {
    /// Retry with lower lambda and therefore higher bandwidth.
    RetryHigherBandwidth,
    /// Retry with higher lambda and therefore lower bandwidth.
    RetryLowerBandwidth,
    /// The axis failure is retryable, but the section retry has already been used.
    RetryBudgetExhausted,
    /// Do not retry this failure.
    Fail,
}

/// Raw current-loop sample for one current axis at one delay.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentAxisSample {
    /// Signed current target for the sampled axis.
    pub target: i16,
    /// Signed measured current on the commanded axis.
    pub same_axis_actual: i16,
    /// Signed measured current on the opposite current axis.
    pub cross_axis_actual: i16,
    /// Signed limited voltage output for the commanded axis.
    pub output_limited: i16,
    /// Configured voltage output limit used for normalization.
    pub output_limit: u16,
    /// Encoder movement observed during the sample window.
    pub encoder_delta_counts: i32,
    /// TMC4671 status flags captured at the sample.
    pub status_flags: u32,
}

/// Current-loop short-sample validation gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentAxisGate {
    /// Minimum allowed same-axis response, in permille of target.
    pub min_response_permille: u16,
    /// Maximum allowed same-axis response, in permille of target.
    pub max_response_permille: u16,
    /// Minimum absolute same-axis current error before the relative response
    /// gate can become a hard fault.
    pub min_response_fault_delta_count: u16,
    /// Maximum allowed opposite-axis response, in permille of target.
    pub max_cross_axis_permille: u16,
    /// Minimum absolute opposite-axis current count before the relative
    /// cross-axis gate can become a hard fault.
    pub min_cross_axis_fault_count: u16,
    /// Maximum allowed voltage output, in permille of output limit.
    pub max_voltage_output_permille: u16,
    /// Maximum allowed encoder movement during the sample.
    pub max_encoder_delta_counts: u16,
    /// Status flags that should fail validation when present.
    pub actionable_status_mask: u32,
}

impl CurrentAxisGate {
    /// Build a commissioning current-validation sample gate.
    pub const fn commissioning_short_sample(
        min_response_permille: u16,
        max_response_permille: u16,
        max_encoder_delta_counts: u16,
    ) -> Self {
        Self {
            min_response_permille,
            max_response_permille,
            min_response_fault_delta_count: 32,
            max_cross_axis_permille: 150,
            min_cross_axis_fault_count: 32,
            max_voltage_output_permille: 900,
            max_encoder_delta_counts,
            actionable_status_mask: current_loop_actionable_status_mask(),
        }
    }
}

/// Classified current-loop short-sample result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentAxisGateResult {
    /// `CURRENT_AXIS_VALIDATION_*` status code.
    pub status: u8,
    /// Same-axis current response normalized to target, in permille.
    pub response_permille: u16,
    /// Opposite-axis current response normalized to target, in permille.
    pub cross_axis_permille: u16,
    /// Voltage output normalized to output limit, in permille.
    pub voltage_output_permille: u16,
    /// Absolute encoder movement observed during the sample.
    pub encoder_delta_counts: u16,
    /// Gate limit copied into evidence for host-side audit.
    pub max_cross_axis_permille: u16,
}

/// Classify one current-axis short sample against a fixed gate.
pub fn classify_current_axis_sample(
    sample: &CurrentAxisSample,
    gate: &CurrentAxisGate,
) -> CurrentAxisGateResult {
    if sample.target == 0 {
        return CurrentAxisGateResult {
            status: CURRENT_AXIS_VALIDATION_ZERO_TARGET,
            response_permille: 0,
            cross_axis_permille: 0,
            voltage_output_permille: 0,
            encoder_delta_counts: 0,
            max_cross_axis_permille: gate.max_cross_axis_permille,
        };
    }

    let target_abs = sample.target.unsigned_abs().max(1) as u32;
    let same_axis_count = sample.same_axis_actual.unsigned_abs();
    let response_permille =
        Permille::checked_ratio(u64::from(same_axis_count), u64::from(target_abs))
            .expect("nonzero current-axis target")
            .raw();
    let response_error_count = same_axis_count.abs_diff(sample.target.unsigned_abs());
    let cross_axis_count = sample.cross_axis_actual.unsigned_abs();
    let cross_axis_permille =
        Permille::checked_ratio(u64::from(cross_axis_count), u64::from(target_abs))
            .expect("nonzero current-axis target")
            .raw();
    let output_limit = sample.output_limit.max(1) as u32;
    let voltage_output_permille = Permille::checked_ratio(
        u64::from(sample.output_limited.unsigned_abs()),
        u64::from(output_limit),
    )
    .expect("nonzero current-axis output limit")
    .raw();
    let encoder_delta_counts = sample
        .encoder_delta_counts
        .unsigned_abs()
        .min(u16::MAX as u32) as u16;

    let status = if (sample.target > 0 && sample.same_axis_actual < 0)
        || (sample.target < 0 && sample.same_axis_actual > 0)
    {
        CURRENT_AXIS_VALIDATION_WRONG_SIGN
    } else if sample.status_flags & gate.actionable_status_mask != 0 {
        CURRENT_AXIS_VALIDATION_STATUS_FLAGS
    } else if voltage_output_permille > gate.max_voltage_output_permille {
        CURRENT_AXIS_VALIDATION_SATURATION
    } else if encoder_delta_counts > gate.max_encoder_delta_counts {
        CURRENT_AXIS_VALIDATION_MOTION
    } else if cross_axis_count > gate.min_cross_axis_fault_count
        && cross_axis_permille > gate.max_cross_axis_permille
    {
        CURRENT_AXIS_VALIDATION_CROSS_AXIS
    } else if response_error_count > gate.min_response_fault_delta_count
        && response_permille < gate.min_response_permille
    {
        CURRENT_AXIS_VALIDATION_LOW_RESPONSE
    } else if response_error_count > gate.min_response_fault_delta_count
        && response_permille > gate.max_response_permille
    {
        CURRENT_AXIS_VALIDATION_HIGH_RESPONSE
    } else {
        CURRENT_AXIS_VALIDATION_PASS
    };

    CurrentAxisGateResult {
        status,
        response_permille,
        cross_axis_permille,
        voltage_output_permille,
        encoder_delta_counts,
        max_cross_axis_permille: gate.max_cross_axis_permille,
    }
}

impl CurrentGainSynthesisInput {
    /// Build symmetric current-loop synthesis input from a measured electrical model.
    pub const fn symmetric(model: ElectricalModel, profile: CommissionProfile) -> Self {
        Self {
            model,
            profile,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            high_inductance_axis: CURRENT_AXIS_FLUX,
        }
    }
}

const CURRENT_P_Q8_8_SCALE: f32 = 256.0;
const MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ: f32 = 2_000.0;

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn valid_physical_current_bandwidth_hz(value: f32) -> bool {
    finite_positive(value) && value <= MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ
}

fn pid_gain_from_float(value: f32) -> Result<PidGain, CurrentGainSynthesisError> {
    if !finite_positive(value) {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }
    let rounded = (value + 0.5).max(1.0) as u64;
    if rounded > u64::from(PidGain::MAX.raw()) {
        return Err(CurrentGainSynthesisError::GainOutOfRange {
            attempted_raw: rounded,
        });
    }
    PidGain::new(rounded as u16).map_err(|error| CurrentGainSynthesisError::GainOutOfRange {
        attempted_raw: u64::from(error.raw()),
    })
}

fn physical_current_p_raw_from_l_count_micro(
    l_count_micro: u32,
    bandwidth_hz: f32,
) -> Result<PidGain, CurrentGainSynthesisError> {
    if l_count_micro == 0 || !valid_physical_current_bandwidth_hz(bandwidth_hz) {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }
    let l_count_seconds = l_count_micro as f32 * 1e-6;
    let p_float =
        2.0 * core::f32::consts::PI * bandwidth_hz * l_count_seconds * CURRENT_P_Q8_8_SCALE;
    pid_gain_from_float(p_float)
}

#[cfg(test)]
fn physical_current_bandwidth_hz_for_p_raw(l_count_micro: u32, p_raw: u16) -> Option<f32> {
    if l_count_micro == 0 || p_raw == 0 {
        return None;
    }
    let l_count_seconds = l_count_micro as f32 * 1e-6;
    Some(p_raw as f32 / (2.0 * core::f32::consts::PI * l_count_seconds * CURRENT_P_Q8_8_SCALE))
}

fn bandwidth_period_us(bandwidth_hz: f32) -> Option<u32> {
    if !valid_physical_current_bandwidth_hz(bandwidth_hz) {
        return None;
    }
    let period_us = 1_000_000.0 / (2.0 * core::f32::consts::PI * bandwidth_hz);
    if period_us > u32::MAX as f32 {
        return None;
    }
    Some((period_us + 0.5).max(1.0) as u32)
}

fn physical_current_gains_checked(
    model: &ElectricalModel,
    bandwidth_hz: f32,
    config: &crate::config::TmcConfig,
) -> Result<CurrentGains, CurrentGainSynthesisError> {
    if model.r_count_milli == 0 || model.l_count_micro == 0 || model.tau_e_us == 0 {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }
    let p_reg = physical_current_p_raw_from_l_count_micro(model.l_count_micro, bandwidth_hz)?;
    let tau_s = model.tau_e_us as f32 * 1e-6;
    let f_pwm = config.pwm_freq_hz() as f32;
    let i_reg = pid_gain_from_float(CURRENT_I_DETUNED_ZERO_FACTOR / (tau_s * f_pwm))?;

    Ok(CurrentGains {
        flux_p: p_reg,
        flux_i: i_reg,
        torque_p: p_reg,
        torque_i: i_reg,
        effective_bandwidth_hz: (bandwidth_hz + 0.5).max(1.0) as u32,
        lambda_us: bandwidth_period_us(bandwidth_hz)
            .ok_or(CurrentGainSynthesisError::InvalidElectricalModel)?,
    })
}

/// Synthesize production current-loop gains and gain-source evidence.
pub fn synthesize_current_gains(
    input: &CurrentGainSynthesisInput,
    config: &crate::config::TmcConfig,
) -> Result<CurrentGainSynthesis, CurrentGainSynthesisError> {
    if input.model.r_count_milli == 0
        || input.model.l_count_micro == 0
        || input.model.theta_e_us == 0
    {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }

    match input.axis_split_source {
        AXIS_SPLIT_SOURCE_NONE => {
            let gains = physical_current_gains_checked(&input.model, config.bandwidth_hz, config)?;
            Ok(CurrentGainSynthesis {
                gains,
                evidence: CurrentLoopEvidence {
                    gains_tier: CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC,
                    candidate_gains_tier: CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC,
                    ..CurrentLoopEvidence::measured_symmetric()
                },
                flux_tau_us: input.model.tau_e_us,
                torque_tau_us: input.model.tau_e_us,
            })
        }
        axis_split_source => {
            Err(CurrentGainSynthesisError::UnsupportedSynthesisPolicy { axis_split_source })
        }
    }
}

/// Number of tail samples used for the robust tail-level statistic.
const TAIL_LEVEL_WINDOW: usize = 32;

/// Compute the median of a slice of u32 values.
///
/// Uses a stack-allocated scratch buffer (max 32 elements). Returns 0 for
/// empty slices. For even-length slices, returns the mean of the two middle
/// values.
fn median_u32(values: &[u32]) -> u32 {
    let len = values.len();
    if len == 0 {
        return 0;
    }
    let mut scratch = [0u32; TAIL_LEVEL_WINDOW];
    let n = len.min(TAIL_LEVEL_WINDOW);
    scratch[..n].copy_from_slice(&values[..n]);
    scratch[..n].sort_unstable();
    if n % 2 == 1 {
        scratch[n / 2]
    } else {
        (scratch[n / 2 - 1] + scratch[n / 2]) / 2
    }
}

fn trailing_average(samples: &[u32], idx: usize, window_len: usize) -> u32 {
    let start = idx.saturating_add(1).saturating_sub(window_len);
    let window = &samples[start..=idx];
    let sum: u64 = window.iter().map(|&sample| sample as u64).sum();
    (sum / window.len() as u64) as u32
}

/// Estimate current-response onset from actual voltage-step-relative offsets.
///
/// This preserves the existing low-rate delay evidence only; it is not the
/// high-rate transport-delay estimator tracked separately elsewhere.
pub fn estimate_electrical_delay(
    samples: &[u32],
    offsets_us: &[u16],
) -> Result<u32, crate::autotune::CommissionError> {
    use crate::autotune::CommissionError;

    if samples.is_empty()
        || samples.len() != offsets_us.len()
        || offsets_us[0] == 0
        || offsets_us.windows(2).any(|pair| pair[0] >= pair[1])
    {
        return Err(CommissionError::DelayTimingRejected);
    }
    let peak = samples.iter().copied().max().unwrap_or(0);
    if peak == 0 {
        return Err(CommissionError::ElectricalModelNoSamples);
    }
    let onset_threshold = peak.div_ceil(10);
    samples
        .iter()
        .zip(offsets_us)
        .find_map(|(&sample, &offset_us)| {
            (sample >= onset_threshold).then_some(u32::from(offset_us))
        })
        .ok_or(CommissionError::ElectricalModelNoSamples)
}

/// Extract step-response metrics from a transient buffer capture.
///
/// Analyzes a captured step response to measure rise time, overshoot,
/// settling time, ringing, and saturation. These metrics are then
/// classified by [`classify_current_validation`].
///
/// # Parameters
/// - `buf`: captured step response samples (raw ADC or encoder values)
/// - `target`: the target value that was commanded (step amplitude)
/// - `pidout_limit`: saturation threshold (0 to skip saturation check)
///
/// Returns metrics with `symmetry_permille = 0`. Symmetry requires two captures
/// (forward + reverse); the caller computes it via [`compute_tail_symmetry_permille`].
/// Timing fields are durations within the capture, not absolute times since
/// the command step. Callers that schedule the first sample after one sample
/// period should keep that origin outside this buffer.
pub fn extract_step_metrics(
    buf: &TransientBuffer,
    target: u32,
    pidout_limit: u32,
) -> CurrentValidation {
    let samples = buf.samples();
    if samples.is_empty() {
        return CurrentValidation {
            rise_time_us: 0,
            overshoot_permille: Permille::ZERO,
            settling_time_us: 0,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
            tail_level: 0,
            target_error_permille: Permille::ZERO,
        };
    }
    if target == 0 {
        return CurrentValidation {
            rise_time_us: 0,
            overshoot_permille: Permille::MAX,
            settling_time_us: 0,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
            tail_level: 0,
            target_error_permille: Permille::HUNDRED_PERCENT,
        };
    }

    let period = buf.sample_period_us() as u32;
    let target_f = target as f32;
    let threshold_10 = (target_f * 0.10) as u32;
    let threshold_90 = (target_f * 0.90) as u32;
    // Current validation samples quantized ADC values. At low targets, a
    // strict 5% band can truncate to zero counts, which would require every
    // sample to equal the target exactly. Allow at least one count so a small,
    // otherwise stable response can settle.
    let settle_band = ((target_f * 0.05) as u32).max(1);

    // These are relative crossing offsets; rise_time subtracts them, so the
    // absolute capture origin does not affect the result.
    let mut t_10: Option<u32> = None;
    let mut t_90: Option<u32> = None;
    let mut peak: u32 = 0;
    let mut peak_idx: usize = 0;
    let mut saturation_count: u16 = 0;

    for (i, &s) in samples.iter().enumerate() {
        if t_10.is_none() && s >= threshold_10 {
            t_10 = Some(crate::sample_timing::sample_offset_us(i, period));
        }
        if t_90.is_none() && s >= threshold_90 {
            t_90 = Some(crate::sample_timing::sample_offset_us(i, period));
        }
        if s > peak {
            peak = s;
            peak_idx = i;
        }
        if pidout_limit > 0 && s >= pidout_limit {
            saturation_count += 1;
        }
    }

    let rise_time_us = match (t_10, t_90) {
        (Some(a), Some(b)) => b.saturating_sub(a),
        _ => 0,
    };

    // Overshoot: (peak - target) / target in 0.1% units
    let overshoot_permille = if peak > target {
        Permille::checked_ratio(u64::from(peak - target), u64::from(target))
            .unwrap_or(Permille::MAX)
    } else {
        Permille::ZERO
    };

    // Settling: last smoothed sample where |actual - target| > 5% band.
    // Use (index + 1) * period because settling time is the duration through
    // the period containing the last out-of-band sample, not its offset
    // from the capture origin.
    let mut last_outside: Option<usize> = None;
    for (i, _) in samples.iter().enumerate() {
        let smoothed = trailing_average(samples, i, CURRENT_METRIC_SMOOTHING_SAMPLES);
        let diff = smoothed.abs_diff(target);
        if diff > settle_band {
            last_outside = Some(i);
        }
    }
    let settling_time_us = match last_outside {
        Some(idx) => crate::sample_timing::sample_elapsed_us(idx, period),
        None => 0,
    };

    // Ringing: crossings between smoothed excursions outside the settle band.
    // Samples inside the accepted band are ignored so ADC noise around the
    // target does not look like sustained oscillation.
    let mut ringing_count: u8 = 0;
    if peak_idx + 1 < samples.len() {
        let upper = target.saturating_add(settle_band);
        let lower = target.saturating_sub(settle_band);
        let mut above: Option<bool> = None;
        for i in peak_idx + 1..samples.len() {
            let smoothed = trailing_average(samples, i, CURRENT_METRIC_SMOOTHING_SAMPLES);
            let now_above = if smoothed > upper {
                Some(true)
            } else if smoothed < lower {
                Some(false)
            } else {
                None
            };
            if let Some(now) = now_above {
                if let Some(prev) = above
                    && now != prev
                {
                    ringing_count = ringing_count.saturating_add(1);
                }
                above = Some(now);
            }
        }
    }

    // Tail level: median of last TAIL_LEVEL_WINDOW samples.
    let tail_start = samples.len().saturating_sub(TAIL_LEVEL_WINDOW);
    let tail_level = median_u32(&samples[tail_start..]);

    CurrentValidation {
        rise_time_us,
        overshoot_permille,
        settling_time_us,
        ringing_count,
        saturation_samples: saturation_count,
        symmetry_permille: Permille::ZERO,
        tail_level,
        target_error_permille: Permille::ZERO,
    }
}

/// Classify current-loop validation metrics against thresholds.
///
/// Thresholds (starting points, subject to hardware validation):
///
/// | Metric | Pass | Marginal | Fail |
/// |--------|------|----------|------|
/// | Overshoot | <= 15% | > 15-30% | > 30% |
/// | Ringing | 0-1 | 2-3 | > 3 |
/// | Settling | <= 5*settle_ref | > 5-10*settle_ref | > 10*settle_ref |
/// | Symmetry | <= 10% | > 10-25% | > 25% |
/// | Saturation | <= 10% | > 10-25% | > 25% |
/// | Target error | <= 10% | > 10-50% | > 50% |
///
/// `total_samples` is the total number of capture samples across both
/// directions, used for computing the saturation percentage.
/// Returns the worst classification across all metrics.
pub fn classify_current_validation(
    metrics: &CurrentValidation,
    settle_reference_us: u32,
    total_samples: u16,
) -> ValidationResult {
    let mut worst = ValidationResult::Pass;

    // Overshoot (0.1% units: 150 = 15.0%, 300 = 30.0%)
    if metrics.overshoot_permille > Permille::new(300) {
        return ValidationResult::Fail;
    } else if metrics.overshoot_permille > Permille::new(150) {
        worst = ValidationResult::Marginal;
    }

    // Ringing
    if metrics.ringing_count > 3 {
        return ValidationResult::Fail;
    } else if metrics.ringing_count > 1 {
        worst = ValidationResult::Marginal;
    }

    // Settling (relative to the dominant current-loop reference).
    let settle_5x = 5 * settle_reference_us;
    let settle_10x = 10 * settle_reference_us;
    if metrics.settling_time_us > settle_10x {
        return ValidationResult::Fail;
    } else if metrics.settling_time_us > settle_5x {
        worst = ValidationResult::Marginal;
    }

    // Symmetry (0.1% units: 100 = 10.0%, 250 = 25.0%)
    // Tail-level symmetry is much less noisy than the old rise-time metric,
    // so tighter thresholds are appropriate.
    if metrics.symmetry_permille > Permille::new(250) {
        return ValidationResult::Fail;
    } else if metrics.symmetry_permille > Permille::new(100) {
        worst = ValidationResult::Marginal;
    }

    // Saturation (fraction of total samples): pass <=10%, marginal >10-25%, fail >25%.
    let saturation_permille =
        super::metrics::saturation_permille(metrics.saturation_samples, total_samples);
    if let Some(saturation_permille) = saturation_permille {
        if saturation_permille > Permille::new(250) {
            return ValidationResult::Fail;
        } else if saturation_permille > Permille::new(100) {
            worst = ValidationResult::Marginal;
        }
    }

    // Target attainment: did the current loop actually reach the commanded
    // target? Catches zero-current, wrong-sign, and under-target responses
    // that would otherwise pass all other checks.
    // 0.1% units: 500 = 50.0%, 100 = 10.0%
    if metrics.target_error_permille > Permille::new(500) {
        return ValidationResult::Fail;
    } else if metrics.target_error_permille > Permille::new(100) {
        worst = ValidationResult::Marginal;
    }

    worst
}

/// Return whether current validation should retry with adjusted gains.
pub fn should_retry_current_validation(result: ValidationResult, attempt: u8) -> bool {
    attempt == 0 && result != ValidationResult::Pass
}

/// Return the retry policy decision for one current-axis validation failure.
pub fn current_axis_retry_decision(
    axis_status: u8,
    section_attempt: u8,
    status_fault: bool,
    saturated: bool,
    motion_fault: bool,
) -> CurrentAxisRetryDecision {
    if status_fault || saturated || motion_fault {
        return CurrentAxisRetryDecision::Fail;
    }

    let retryable = axis_status == CURRENT_AXIS_VALIDATION_LOW_RESPONSE
        || axis_status == CURRENT_AXIS_VALIDATION_HIGH_RESPONSE;
    if !retryable {
        return CurrentAxisRetryDecision::Fail;
    }

    if section_attempt > 0 {
        return CurrentAxisRetryDecision::RetryBudgetExhausted;
    }

    if axis_status == CURRENT_AXIS_VALIDATION_LOW_RESPONSE {
        CurrentAxisRetryDecision::RetryHigherBandwidth
    } else {
        CurrentAxisRetryDecision::RetryLowerBandwidth
    }
}

/// Return whether a marginal current validation can be accepted without retry.
///
/// This is intentionally narrow: it only accepts the case where all metrics
/// except bidirectional rise-time symmetry are already in the pass range, and
/// the computed retry would not change the gains. That covers measurement
/// variability in the symmetry metric without hiding overshoot, ringing,
/// settling, or saturation issues.
pub fn should_accept_current_validation_without_retry(
    metrics: &CurrentValidation,
    settle_reference_us: u32,
    total_samples: u16,
    previous: CurrentGains,
    retry: CurrentGains,
) -> bool {
    if previous != retry {
        return false;
    }
    if metrics.overshoot_permille > Permille::new(150) || metrics.ringing_count > 1 {
        return false;
    }
    if metrics.settling_time_us > 5 * settle_reference_us {
        return false;
    }
    if super::metrics::saturation_permille(metrics.saturation_samples, total_samples)
        .is_some_and(|saturation_permille| saturation_permille > Permille::new(100))
    {
        return false;
    }

    metrics.symmetry_permille > Permille::new(100)
        && metrics.symmetry_permille <= Permille::new(250)
}

fn retry_low_response_lambda_permille(_profile: CommissionProfile) -> u16 {
    // Keep the retry section-wide but bounded. A full 0.5x lambda retry
    // doubled P on OpenFFBoard and produced sustained holding squeal.
    750
}

fn retry_high_response_lambda_permille(_profile: CommissionProfile) -> u16 {
    2000
}

fn retry_min_lambda_permille(_profile: CommissionProfile) -> u16 {
    250
}

fn retry_max_lambda_permille(_profile: CommissionProfile) -> u16 {
    4000
}

fn scale_lambda_permille(lambda_us: u32, permille: u16) -> u32 {
    ((lambda_us.max(1) as u64 * permille.max(1) as u64) / 1000).min(u32::MAX as u64) as u32
}

fn scale_gain_for_lambda(
    gain: PidGain,
    previous_lambda_us: u32,
    next_lambda_us: u32,
) -> Result<PidGain, CurrentGainSynthesisError> {
    if previous_lambda_us == 0 || next_lambda_us == 0 {
        return Ok(gain);
    }

    let raw = (u64::from(gain.raw()) * u64::from(previous_lambda_us)) / u64::from(next_lambda_us);
    if raw > u64::from(PidGain::MAX.raw()) {
        return Err(CurrentGainSynthesisError::GainOutOfRange { attempted_raw: raw });
    }
    PidGain::new(raw.max(1) as u16).map_err(|error| CurrentGainSynthesisError::GainOutOfRange {
        attempted_raw: u64::from(error.raw()),
    })
}

fn scale_bandwidth_for_lambda(
    bandwidth_hz: u32,
    previous_lambda_us: u32,
    next_lambda_us: u32,
) -> u32 {
    if previous_lambda_us == 0 || next_lambda_us == 0 || bandwidth_hz == 0 {
        return bandwidth_hz;
    }

    ((bandwidth_hz as u64 * previous_lambda_us as u64) / next_lambda_us as u64).min(u32::MAX as u64)
        as u32
}

fn cap_higher_retry_at_physical_limit(
    decision: CurrentAxisRetryDecision,
    bandwidth_hz: u32,
    lambda_us: u32,
) -> (u32, u32) {
    if decision != CurrentAxisRetryDecision::RetryHigherBandwidth
        || bandwidth_hz <= MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ as u32
    {
        return (bandwidth_hz, lambda_us);
    }

    let capped_bandwidth_hz = MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ as u32;
    let capped_lambda_us =
        bandwidth_period_us(MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ).unwrap_or(lambda_us);

    (capped_bandwidth_hz, capped_lambda_us)
}

/// Adjust current-loop gains according to the section retry policy.
///
/// A retry changes the controller bandwidth for the whole current loop, not
/// just the axis that first failed the short-sample gate. P tracks the new
/// bandwidth; I stays fixed so retry does not move the tau-derived integrator
/// zero.
pub fn retry_current_loop_gains(
    previous: CurrentGains,
    decision: CurrentAxisRetryDecision,
    profile: CommissionProfile,
) -> Result<CurrentGains, CurrentGainSynthesisError> {
    let lambda_permille = match decision {
        CurrentAxisRetryDecision::RetryHigherBandwidth => {
            retry_low_response_lambda_permille(profile)
        }
        CurrentAxisRetryDecision::RetryLowerBandwidth => {
            retry_high_response_lambda_permille(profile)
        }
        CurrentAxisRetryDecision::RetryBudgetExhausted | CurrentAxisRetryDecision::Fail => {
            return Ok(previous);
        }
    };
    let min_lambda = scale_lambda_permille(previous.lambda_us, retry_min_lambda_permille(profile));
    let max_lambda = scale_lambda_permille(previous.lambda_us, retry_max_lambda_permille(profile));
    let next_lambda = scale_lambda_permille(previous.lambda_us, lambda_permille)
        .clamp(min_lambda.min(max_lambda), min_lambda.max(max_lambda));
    let next_bandwidth = scale_bandwidth_for_lambda(
        previous.effective_bandwidth_hz,
        previous.lambda_us,
        next_lambda,
    );
    let (next_bandwidth, next_lambda) =
        cap_higher_retry_at_physical_limit(decision, next_bandwidth, next_lambda);
    let flux_p = scale_gain_for_lambda(previous.flux_p, previous.lambda_us, next_lambda)?;
    let torque_p = scale_gain_for_lambda(previous.torque_p, previous.lambda_us, next_lambda)?;
    let mut next = previous;

    next.lambda_us = next_lambda;
    next.effective_bandwidth_hz = next_bandwidth;
    next.flux_p = flux_p;
    next.torque_p = torque_p;

    Ok(next)
}

/// Compute symmetry metric from forward and reverse tail levels.
///
/// Returns `|fwd - rev| / avg(fwd, rev)` in 0.1% units (permille).
/// This measures steady-state signed-current symmetry — whether both
/// directions produce the same actual current magnitude. Returns 0 if
/// both values are zero.
pub fn compute_tail_symmetry_permille(fwd_tail: u32, rev_tail: u32) -> Permille {
    super::metrics::normalized_difference(fwd_tail, rev_tail)
}

/// Compute target-attainment error from a tail level and commanded target.
///
/// Returns `|tail_level - target| / target` in 0.1% units (permille).
/// Returns 1000 (100%) if `target` is zero to avoid division by zero.
pub fn compute_target_error_permille(tail_level: u32, target: u32) -> Permille {
    if target == 0 {
        return Permille::HUNDRED_PERCENT;
    }
    Permille::checked_ratio(u64::from(tail_level.abs_diff(target)), u64::from(target))
        .expect("target checked nonzero")
}

#[cfg(test)]
mod tests {
    use super::super::test_support::{autotune_test_config, test_electrical_model};
    use super::*;

    fn gain(raw: u16) -> PidGain {
        PidGain::new(raw).expect("test PID gain must be valid")
    }

    #[test]
    fn current_loop_evidence_defaults_to_failed_none() {
        let evidence = CurrentLoopEvidence::failed(CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_FAILED);
        assert_eq!(evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(evidence.gains_tier, CURRENT_GAINS_TIER_NONE);
        assert_eq!(evidence.failure_reason, CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS);
        assert_eq!(evidence.measured_axis_split_permille, 1000);
        assert_eq!(evidence.applied_axis_split_permille, 1000);
        assert_eq!(evidence.axis_split_clamped, 0);
        assert_eq!(evidence.current_validation_axes, 0);
        assert_eq!(evidence.retry_budget_exhausted, 0);
    }

    #[test]
    fn current_loop_evidence_records_measured_symmetric_defaults() {
        let evidence = CurrentLoopEvidence::measured_symmetric();

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_MEASURED);
        assert_eq!(evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(evidence.gains_tier, CURRENT_GAINS_TIER_MEASURED_SYMMETRIC);
        assert_eq!(evidence.failure_reason, CURRENT_LOOP_FAILURE_NONE);
        assert_eq!(evidence.measured_axis_split_permille, 1000);
        assert_eq!(evidence.applied_axis_split_permille, 1000);
        assert_eq!(evidence.axis_split_clamped, 0);
        assert_eq!(evidence.current_validation_axes, 0);
        assert_eq!(evidence.retry_budget_exhausted, 0);
    }

    #[test]
    fn current_loop_evidence_records_default_fallback() {
        let evidence = CurrentLoopEvidence::default_gains(
            CURRENT_LOOP_FAILURE_TORQUE_VALIDATION,
            true,
            CURRENT_VALIDATION_AXIS_TORQUE,
        );

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_DEFAULT);
        assert_eq!(evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(evidence.gains_tier, CURRENT_GAINS_TIER_DEFAULT);
        assert_eq!(
            evidence.failure_reason,
            CURRENT_LOOP_FAILURE_TORQUE_VALIDATION
        );
        assert_eq!(evidence.measured_axis_split_permille, 1000);
        assert_eq!(evidence.applied_axis_split_permille, 1000);
        assert_eq!(evidence.axis_split_clamped, 0);
        assert_eq!(
            evidence.current_validation_axes,
            CURRENT_VALIDATION_AXIS_TORQUE
        );
        assert_eq!(evidence.retry_budget_exhausted, 1);
    }

    #[test]
    fn current_loop_evidence_pins_gain_source_split_and_tier_codes() {
        assert_eq!(CURRENT_GAINS_SOURCE_FAILED, 0);
        assert_eq!(CURRENT_GAINS_SOURCE_MEASURED, 1);
        assert_eq!(CURRENT_GAINS_SOURCE_DEFAULT, 2);

        assert_eq!(AXIS_SPLIT_SOURCE_NONE, 0);

        assert_eq!(CURRENT_GAINS_TIER_NONE, 0);
        assert_eq!(CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, 1);
        assert_eq!(CURRENT_GAINS_TIER_DEFAULT, 3);
        assert_eq!(CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC, 4);
    }

    #[test]
    fn current_loop_evidence_pins_axis_and_sample_count_codes() {
        assert_eq!(CURRENT_VALIDATION_AXIS_FLUX, 1);
        assert_eq!(CURRENT_VALIDATION_AXIS_TORQUE, 2);

        assert_eq!(CURRENT_AXIS_FLUX, 0);
        assert_eq!(CURRENT_AXIS_TORQUE, 1);

        assert_eq!(CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, 4);
        assert_eq!(CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT, 3);
        assert_eq!(CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS, 4);
    }

    #[test]
    fn current_loop_evidence_pins_failure_reason_codes() {
        assert_eq!(CURRENT_LOOP_FAILURE_NONE, 0);
        assert_eq!(CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, 1);
        assert_eq!(CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID, 2);
        assert_eq!(CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS, 3);
        assert_eq!(CURRENT_LOOP_FAILURE_FLUX_VALIDATION, 4);
        assert_eq!(CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, 5);
        assert_eq!(CURRENT_LOOP_FAILURE_SATURATION, 6);
        assert_eq!(CURRENT_LOOP_FAILURE_MOTION, 7);
        assert_eq!(CURRENT_LOOP_FAILURE_STATUS_FLAGS, 8);
        assert_eq!(CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED, 9);
        assert_eq!(CURRENT_LOOP_FAILURE_SPI, 10);
        assert_eq!(CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN, 11);
        assert_eq!(CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS, 12);
        assert_eq!(CURRENT_LOOP_FAILURE_HOLD_SAMPLE, 13);
        assert_eq!(crate::autotune::CURRENT_LOOP_FAILURE_RESPONSE_MAGNITUDE, 14);
        assert_eq!(
            crate::autotune::CURRENT_LOOP_FAILURE_CROSS_AXIS_COUPLING,
            15
        );
        assert_eq!(crate::autotune::CURRENT_LOOP_FAILURE_WRONG_SIGN, 16);
        assert_eq!(crate::autotune::CURRENT_LOOP_FAILURE_CLOSED_LOOP_ENTRY, 17);
    }

    #[test]
    fn physical_current_p_uses_count_space_l_and_bandwidth() {
        assert_eq!(
            physical_current_p_raw_from_l_count_micro(1_346, 600.0)
                .expect("valid physical gain")
                .raw(),
            1_299
        );
        assert_eq!(
            physical_current_p_raw_from_l_count_micro(1_346, 800.0)
                .expect("valid physical gain")
                .raw(),
            1_732
        );
        assert_eq!(
            physical_current_p_raw_from_l_count_micro(1_366, 650.0)
                .expect("valid physical gain")
                .raw(),
            1_428
        );
    }

    #[test]
    fn physical_current_p_anchor_matches_foc108_evidence() {
        let x_bw_for_p1422 =
            physical_current_bandwidth_hz_for_p_raw(1_346, 1_422).expect("valid X anchor");
        let y_bw_for_p1422 =
            physical_current_bandwidth_hz_for_p_raw(1_366, 1_422).expect("valid Y anchor");

        assert!(
            (656.0..=657.5).contains(&x_bw_for_p1422),
            "X anchor {x_bw_for_p1422}"
        );
        assert!(
            (646.0..=648.5).contains(&y_bw_for_p1422),
            "Y anchor {y_bw_for_p1422}"
        );
    }

    #[test]
    fn physical_current_p_rejects_invalid_inputs() {
        assert!(physical_current_p_raw_from_l_count_micro(0, 600.0).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 0.0).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, f32::NAN).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 2_000.0).is_ok());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 2_001.0).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 12_000.0).is_err());
    }

    #[test]
    fn retry_policy_allows_one_retry_for_response_magnitude_only() {
        let decision = current_axis_retry_decision(
            CURRENT_AXIS_VALIDATION_LOW_RESPONSE,
            0,
            false,
            false,
            false,
        );

        assert_eq!(decision, CurrentAxisRetryDecision::RetryHigherBandwidth);
    }

    #[test]
    fn retry_policy_rejects_saturation_without_retry() {
        let decision =
            current_axis_retry_decision(CURRENT_AXIS_VALIDATION_SATURATION, 0, false, true, false);

        assert_eq!(decision, CurrentAxisRetryDecision::Fail);
    }

    #[test]
    fn retry_policy_exhausts_section_budget() {
        let decision = current_axis_retry_decision(
            CURRENT_AXIS_VALIDATION_LOW_RESPONSE,
            1,
            false,
            false,
            false,
        );

        assert_eq!(decision, CurrentAxisRetryDecision::RetryBudgetExhausted);
    }

    #[test]
    fn gain_conversion_accepts_maximum_and_rejects_first_negative_encoding() {
        assert_eq!(
            pid_gain_from_float(32_767.0).unwrap(),
            foci_tmc4671::pid::PidGain::MAX
        );
        assert!(matches!(
            pid_gain_from_float(32_768.0),
            Err(CurrentGainSynthesisError::GainOutOfRange {
                attempted_raw: 32_768,
            })
        ));
    }

    #[test]
    fn higher_bandwidth_retry_reports_gain_overflow_without_saturation() {
        let previous = CurrentGains {
            flux_p: foci_tmc4671::pid::PidGain::MAX,
            flux_i: foci_tmc4671::pid::PidGain::new(10).unwrap(),
            torque_p: foci_tmc4671::pid::PidGain::MAX,
            torque_i: foci_tmc4671::pid::PidGain::new(10).unwrap(),
            effective_bandwidth_hz: 100,
            lambda_us: 1_000,
        };

        assert!(matches!(
            retry_current_loop_gains(
                previous,
                CurrentAxisRetryDecision::RetryHigherBandwidth,
                CommissionProfile::Balanced,
            ),
            Err(CurrentGainSynthesisError::GainOutOfRange { .. })
        ));
    }

    #[test]
    fn retry_policy_scales_bandwidth_and_p_but_keeps_i() {
        let previous = CurrentGains {
            flux_p: gain(120),
            flux_i: gain(23),
            torque_p: gain(220),
            torque_i: gain(19),
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };

        let retry = retry_current_loop_gains(
            previous,
            CurrentAxisRetryDecision::RetryHigherBandwidth,
            CommissionProfile::Balanced,
        )
        .expect("retry gains must remain valid");

        assert!(retry.flux_p.raw() > previous.flux_p.raw());
        assert!(retry.torque_p.raw() > previous.torque_p.raw());
        assert_eq!(retry.flux_i, previous.flux_i);
        assert_eq!(retry.torque_i, previous.torque_i);
        // These metadata expectations track the retry constants. If hardware
        // tuning moves those constants, update the expected metadata with them.
        assert_eq!(retry.effective_bandwidth_hz, 193);
        assert_eq!(retry.lambda_us, 700);
    }

    #[test]
    fn retry_policy_caps_higher_bandwidth_at_physical_limit() {
        let physical_limit_lambda =
            bandwidth_period_us(MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ).expect("valid limit");
        let previous = CurrentGains {
            flux_p: gain(1500),
            flux_i: gain(23),
            torque_p: gain(1600),
            torque_i: gain(19),
            effective_bandwidth_hz: MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ as u32,
            lambda_us: physical_limit_lambda,
        };

        let retry = retry_current_loop_gains(
            previous,
            CurrentAxisRetryDecision::RetryHigherBandwidth,
            CommissionProfile::Balanced,
        )
        .expect("capped retry gains must remain valid");

        assert_eq!(
            retry.effective_bandwidth_hz,
            previous.effective_bandwidth_hz
        );
        assert_eq!(retry.lambda_us, previous.lambda_us);
        assert_eq!(retry.flux_p, previous.flux_p);
        assert_eq!(retry.torque_p, previous.torque_p);
        assert_eq!(retry.flux_i, previous.flux_i);
        assert_eq!(retry.torque_i, previous.torque_i);
    }

    #[test]
    fn retry_policy_lower_bandwidth_keeps_i() {
        let previous = CurrentGains {
            flux_p: gain(120),
            flux_i: gain(23),
            torque_p: gain(220),
            torque_i: gain(19),
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };

        let retry = retry_current_loop_gains(
            previous,
            CurrentAxisRetryDecision::RetryLowerBandwidth,
            CommissionProfile::Balanced,
        )
        .expect("retry gains must remain valid");

        assert!(retry.flux_p.raw() < previous.flux_p.raw());
        assert!(retry.torque_p.raw() < previous.torque_p.raw());
        assert_eq!(retry.flux_i, previous.flux_i);
        assert_eq!(retry.torque_i, previous.torque_i);
        // These metadata expectations track the retry constants. If hardware
        // tuning moves those constants, update the expected metadata with them.
        assert_eq!(retry.effective_bandwidth_hz, 72);
        assert_eq!(retry.lambda_us, 1868);
    }

    #[test]
    fn physical_synthesis_uses_configured_bandwidth_for_symmetric_p() {
        let model = ElectricalModel {
            r_count_milli: 888,
            l_count_micro: 1_346,
            tau_e_us: 1_515,
            theta_e_us: 160,
        };
        let mut config = autotune_test_config();
        config.bandwidth_hz = 600.0;
        let input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);

        let output = synthesize_current_gains(&input, &config).expect("physical gains");

        assert_eq!(output.gains.flux_p.raw(), 1_299);
        assert_eq!(output.gains.torque_p.raw(), 1_299);
        assert_eq!(output.gains.effective_bandwidth_hz, 600);
        assert_eq!(
            output.evidence.gains_tier,
            CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC
        );
        assert_eq!(
            output.evidence.candidate_gains_tier,
            CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC
        );
        assert_eq!(output.flux_tau_us, model.tau_e_us);
        assert_eq!(output.torque_tau_us, model.tau_e_us);
    }

    #[test]
    fn physical_synthesis_keeps_i_on_detuned_tau_path() {
        let model = ElectricalModel {
            r_count_milli: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_count_micro: 1_366,
            tau_e_us: 1_366,
            theta_e_us: 160,
        };
        let mut config = autotune_test_config();
        config.bandwidth_hz = 650.0;
        let input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);

        let output = synthesize_current_gains(&input, &config).expect("physical gains");

        let tau_s = model.tau_e_us as f32 * 1e-6;
        let expected_i = (CURRENT_I_DETUNED_ZERO_FACTOR / (tau_s * config.pwm_freq_hz() as f32)
            + 0.5)
            .clamp(1.0, PidGain::MAX.raw() as f32) as u16;

        assert_eq!(output.gains.flux_i.raw(), expected_i);
        assert_eq!(output.gains.torque_i.raw(), expected_i);
        assert_eq!(output.gains.flux_p.raw(), 1_428);
    }

    #[test]
    fn phase1_symmetric_synthesis_reports_physical_symmetric() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);

        let output = synthesize_current_gains(&input, &config).unwrap();

        assert_eq!(output.evidence.gains_source, CURRENT_GAINS_SOURCE_MEASURED);
        assert_eq!(output.evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(
            output.evidence.gains_tier,
            CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC
        );
        assert_eq!(output.evidence.failure_reason, CURRENT_LOOP_FAILURE_NONE);
        assert_eq!(output.gains.flux_p, output.gains.torque_p);
        assert_eq!(output.gains.flux_i, output.gains.torque_i);
        assert_eq!(output.flux_tau_us, model.tau_e_us);
        assert_eq!(output.torque_tau_us, model.tau_e_us);
    }

    #[test]
    fn physical_synthesis_rejects_unvalidated_axis_split() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput {
            model,
            profile: CommissionProfile::Balanced,
            measured_axis_split_permille: 1_200,
            applied_axis_split_permille: 1_100,
            axis_split_source: 1,
            high_inductance_axis: CURRENT_AXIS_FLUX,
        };

        assert!(matches!(
            synthesize_current_gains(&input, &config),
            Err(CurrentGainSynthesisError::UnsupportedSynthesisPolicy {
                axis_split_source: 1
            })
        ));
    }

    #[test]
    fn split_synthesis_rejects_invalid_axis_split_source() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let mut input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);
        input.axis_split_source = 0xff;

        assert!(matches!(
            synthesize_current_gains(&input, &config),
            Err(CurrentGainSynthesisError::UnsupportedSynthesisPolicy { .. })
        ));
    }

    // --- Step metrics extraction ---

    /// Build a synthetic first-order step response for testing.
    /// Fills 256 samples at 100us intervals.
    fn synthetic_step_response(overshoot_percent: f32) -> TransientBuffer {
        let mut buf = TransientBuffer::new();
        buf.reset(100);
        let target = 1000.0f32;
        for i in 0..256u32 {
            let t = i as f32;
            let base = target * (1.0 - (-t / 10.0).exp());
            let overshoot_term = if overshoot_percent > 0.0 {
                target * overshoot_percent / 100.0 * (-t / 20.0).exp() * (t / 5.0).sin()
            } else {
                0.0
            };
            buf.push((base + overshoot_term).max(0.0) as u32);
        }
        buf
    }

    #[test]
    fn step_metrics_clean_response() {
        let buf = synthetic_step_response(0.0);
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert!(
            metrics.overshoot_permille < Permille::new(20),
            "overshoot {} should be <2%",
            metrics.overshoot_permille.raw()
        );
        assert!(
            metrics.ringing_count <= 1,
            "ringing {} should be <=1",
            metrics.ringing_count
        );
        assert!(metrics.rise_time_us > 0, "rise time must be nonzero");
    }

    /// Phase 1 timing invariants — pin the current sample-timing
    /// convention so a later measurement redesign updates these tests
    /// deliberately. See `docs/specs/2026-04-30-inner-commissioning-stability.md`
    /// §5.

    #[test]
    fn step_metrics_t10_at_or_before_t90() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &v in &[0u32, 50, 200, 600, 950, 1000, 1000, 1000] {
            buf.push(v);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        // rise_time = t90 - t10; nonzero rise_time implies t90 >= t10.
        assert!(metrics.rise_time_us > 0);
    }

    #[test]
    fn delay_timestamp_estimator_uses_actual_jittered_offsets() {
        let samples = [0, 20, 100, 900, 1_000];
        let offsets_us = [160, 350, 515, 720, 900];

        assert_eq!(estimate_electrical_delay(&samples, &offsets_us), Ok(515));
    }

    #[test]
    fn delay_timestamp_estimator_rejects_invalid_offset_domains() {
        use crate::autotune::CommissionError;

        assert_eq!(
            estimate_electrical_delay(&[], &[]),
            Err(CommissionError::DelayTimingRejected)
        );
        assert_eq!(
            estimate_electrical_delay(&[0, 100], &[160]),
            Err(CommissionError::DelayTimingRejected)
        );
        assert_eq!(
            estimate_electrical_delay(&[0, 100], &[160, 160]),
            Err(CommissionError::DelayTimingRejected)
        );
    }

    #[test]
    fn step_metrics_rise_time_invariant_under_capture_origin_offset() {
        // Adding zero-valued samples *before* the meaningful step must not
        // change rise_time, because t90 - t10 cancels the capture-origin
        // offset.
        let mut early = TransientBuffer::new();
        early.reset(160);
        for &v in &[0u32, 50, 200, 600, 950, 1000, 1000, 1000] {
            early.push(v);
        }
        let mut delayed = TransientBuffer::new();
        delayed.reset(160);
        for &v in &[0u32, 0, 0, 50, 200, 600, 950, 1000, 1000, 1000] {
            delayed.push(v);
        }
        let early_m = extract_step_metrics(&early, 1000, 0);
        let delayed_m = extract_step_metrics(&delayed, 1000, 0);
        assert_eq!(early_m.rise_time_us, delayed_m.rise_time_us);
    }

    #[test]
    fn step_metrics_settling_time_uses_last_outside_plus_one_period() {
        // Settling time uses smoothed (8-sample trailing average) values, so
        // pin the formula by inserting one zero at index 15. The trailing
        // average then stays out-of-band through index 22 and recovers at
        // index 23 — last_outside == 22, settling == (22 + 1) * 160 = 3680us.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for i in 0..24u32 {
            if i == 15 {
                buf.push(0);
            } else {
                buf.push(1000);
            }
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.settling_time_us, 23 * 160);
    }

    #[test]
    fn step_metrics_zero_target_stays_bounded() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for _ in 0..16 {
            buf.push(100);
        }
        let metrics = extract_step_metrics(&buf, 0, 0);
        assert_eq!(metrics.rise_time_us, 0);
        assert_eq!(metrics.settling_time_us, 0);
    }

    #[test]
    fn step_metrics_low_target_stays_bounded() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for _ in 0..16 {
            buf.push(0);
        }
        let metrics = extract_step_metrics(&buf, 5, 0);
        // settle_band is clamped to >=1 count; an all-zero capture cannot
        // reach the band but the metric must stay bounded, not panic.
        assert!(metrics.settling_time_us <= 16 * 160);
    }

    #[test]
    fn step_metrics_with_overshoot() {
        let buf = synthetic_step_response(100.0);
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert!(
            metrics.overshoot_permille > Permille::new(100),
            "should detect significant overshoot: {}",
            metrics.overshoot_permille.raw()
        );
    }

    #[test]
    fn step_metrics_ignore_in_band_noise_for_ringing() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &sample in &[0, 120, 500, 900, 1220, 1100, 1040] {
            buf.push(sample);
        }
        for i in 0..96 {
            let sample = if i % 2 == 0 { 960 } else { 1040 };
            buf.push(sample);
        }

        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.ringing_count, 0);
    }

    #[test]
    fn current_metrics_ignore_tail_ripple_after_settling() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &sample in &[53, 98, 180, 260, 330, 390, 416, 370] {
            buf.push(sample);
        }
        for i in 0..248 {
            let sample = if i % 2 == 0 { 345 } else { 416 };
            buf.push(sample);
        }

        let metrics = extract_step_metrics(&buf, 367, 0);

        assert!(
            metrics.settling_time_us < 10 * 1868,
            "tail ripple around target should not force full-window settling"
        );
        assert_eq!(metrics.ringing_count, 0);
        assert_eq!(
            classify_current_validation(&metrics, 1868, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn step_metrics_empty_buffer() {
        let buf = TransientBuffer::new();
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.rise_time_us, 0);
        assert_eq!(metrics.overshoot_permille, Permille::ZERO);
        assert_eq!(metrics.tail_level, 0);
    }

    #[test]
    fn step_metrics_tail_level_from_last_32_samples() {
        // Build a buffer that rises quickly to ~950, then stays there.
        // The tail_level (median of last 32 samples) should be ~950.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        // 10 rising samples
        for &sample in &[0u32, 100, 300, 500, 700, 850, 920, 940, 945, 948] {
            buf.push(sample);
        }
        // 246 steady-state samples at 950
        for _ in 0..246 {
            buf.push(950);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(
            metrics.tail_level, 950,
            "tail_level should be median of last 32 samples"
        );
    }

    #[test]
    fn step_metrics_tail_level_resists_outliers() {
        // Last 32 samples: 30 at 950, 2 outliers at 0. Median should be 950.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        // Fill first 224 samples as steady state
        for _ in 0..224 {
            buf.push(950);
        }
        // Last 32 samples: 2 outliers, then 30 at 950
        buf.push(0);
        buf.push(0);
        for _ in 0..30 {
            buf.push(950);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.tail_level, 950, "median should resist outliers");
    }

    #[test]
    fn step_metrics_tail_level_short_buffer() {
        // Buffer with only 10 samples — tail_level uses all of them.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &sample in &[100u32, 200, 300, 400, 500, 600, 700, 800, 900, 950] {
            buf.push(sample);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        // Median of [100,200,300,400,500,600,700,800,900,950] = (500+600)/2 = 550
        assert_eq!(
            metrics.tail_level, 550,
            "tail_level should use all samples when buffer < 32"
        );
    }

    // --- Validation classification ---

    #[test]
    fn current_sample_gate_accepts_good_flux_sample() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
        assert_eq!(gate.response_permille, 960);
        assert!(gate.cross_axis_permille <= gate.max_cross_axis_permille);
    }

    #[test]
    fn current_sample_gate_rejects_zero_target() {
        let sample = CurrentAxisSample {
            target: 0,
            same_axis_actual: 0,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_ZERO_TARGET);
    }

    #[test]
    fn current_sample_gate_rejects_wrong_sign() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: -20,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_WRONG_SIGN);
    }

    #[test]
    fn current_sample_gate_saturates_response_permille_before_classifying() {
        let sample = CurrentAxisSample {
            target: 1,
            same_axis_actual: 132,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.response_permille, u16::MAX);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_HIGH_RESPONSE);
    }

    #[test]
    fn current_sample_gate_saturates_cross_axis_and_voltage_permille() {
        let sample = CurrentAxisSample {
            target: 1,
            same_axis_actual: 1,
            cross_axis_actual: 132,
            output_limited: 132,
            output_limit: 1,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.cross_axis_permille, u16::MAX);
        assert_eq!(gate.voltage_output_permille, u16::MAX);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_SATURATION);
    }

    #[test]
    fn current_sample_gate_accepts_low_target_cross_axis_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 128,
            cross_axis_actual: 20,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(850, 1150, 4),
        );

        assert_eq!(gate.cross_axis_permille, 156);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_rejects_low_target_cross_axis_above_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 128,
            cross_axis_actual: 33,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(850, 1150, 4),
        );

        assert_eq!(gate.cross_axis_permille, 257);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_CROSS_AXIS);
    }

    #[test]
    fn current_sample_gate_accepts_low_target_response_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 151,
            cross_axis_actual: 0,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(850, 1150, 4),
        );

        assert_eq!(gate.response_permille, 1179);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_rejects_low_target_response_above_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 161,
            cross_axis_actual: 0,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(850, 1150, 4),
        );

        assert_eq!(gate.response_permille, 1257);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_HIGH_RESPONSE);
    }

    #[test]
    fn current_sample_gate_rejects_low_target_response_below_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 95,
            cross_axis_actual: 0,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(850, 1150, 4),
        );

        assert_eq!(gate.response_permille, 742);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
    }

    #[test]
    fn current_sample_gate_rejects_zero_actual_as_low_response() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 0,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.response_permille, 0);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
    }

    #[test]
    fn current_sample_gate_ignores_encoder_presence_status_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x7000_0000,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_ignores_reserved_status_bit_31() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0xf000_0080,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_ignores_outer_cascade_status_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x0000_00dd,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_rejects_current_loop_status_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 1 << 10,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_STATUS_FLAGS);
    }

    #[test]
    fn current_sample_gate_ignores_latched_current_pid_output_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: (1 << 11) | (1 << 15),
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisGate::commissioning_short_sample(700, 1250, 4),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }
}
