use super::{
    AXIS_SPLIT_SOURCE_IMPEDANCE, AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE,
    CommissionProfile, CurrentGainSynthesis, CurrentGainSynthesisInput, CurrentGains,
    CurrentLoopEvidence, ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel, TransientBuffer,
    ValidationResult,
};
#[cfg(test)]
use super::{
    CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, CURRENT_GAINS_SOURCE_DEFAULT,
    CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED, CURRENT_GAINS_TIER_DEFAULT,
    CURRENT_GAINS_TIER_MEASURED_SPLIT, CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
    CURRENT_GAINS_TIER_NONE, CURRENT_LOOP_FAILURE_FLUX_VALIDATION,
    CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS, CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID,
    CURRENT_LOOP_FAILURE_MOTION, CURRENT_LOOP_FAILURE_NONE,
    CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED,
    CURRENT_LOOP_FAILURE_SATURATION, CURRENT_LOOP_FAILURE_SPI, CURRENT_LOOP_FAILURE_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT,
    CURRENT_VALIDATION_AXIS_FLUX, CURRENT_VALIDATION_AXIS_TORQUE,
    CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS,
};

/// Current-loop SIMC integrator damping factor.
///
/// The documented advanced-PI zero placement is correct in isolation, but on
/// the target hardware the raw `4*(lambda+theta)` SIMC integral time winds up
/// during the current-step rise. Use a slower integral time while keeping the
/// proportional SIMC damping unchanged.
const CURRENT_SIMC_TI_MULTIPLIER: f32 = 11.0;

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
    pub overshoot_pct: u16,
    /// Time to settle within +/-5% of target, in microseconds.
    pub settling_time_us: u32,
    /// Zero-crossings of (actual - target) after peak.
    pub ringing_count: u8,
    /// Samples where value hit the saturation limit.
    pub saturation_samples: u16,
    /// |fwd_tail - rev_tail| / avg, in 0.1% units (permille).
    /// Set to 0 by extract_step_metrics (requires two captures).
    pub symmetry_pct: u16,
    /// Robust tail level: median of last 32 samples (or fewer if buffer is
    /// shorter). Used to compute steady-state signed-current symmetry between
    /// forward and reverse captures.
    pub tail_level: u32,
    /// |tail_level - target| / target, in 0.1% units (permille).
    /// Measures whether the current loop actually reached the commanded
    /// target. Set to 0 by extract_step_metrics; the caller computes it
    /// from the combined (worst-case) tail_level and commanded target.
    pub target_error_pct: u16,
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

const CURRENT_AXIS_NON_ACTIONABLE_STATUS_MASK: u32 = 0x7000_0000;

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
    /// Maximum allowed opposite-axis response, in permille of target.
    pub max_cross_axis_permille: u16,
    /// Maximum allowed voltage output, in permille of output limit.
    pub max_voltage_output_permille: u16,
    /// Maximum allowed encoder movement during the sample.
    pub max_encoder_delta_counts: u16,
    /// Status flags that should fail validation when present.
    pub actionable_status_mask: u32,
}

impl CurrentAxisGate {
    /// OpenFFBoard v1 flux-axis 1 ms sample gate.
    pub const fn openffboard_flux_1ms() -> Self {
        Self::openffboard(250, 1250)
    }

    /// OpenFFBoard v1 flux-axis 2 ms sample gate.
    pub const fn openffboard_flux_2ms() -> Self {
        Self::openffboard(700, 1250)
    }

    /// OpenFFBoard v1 flux-axis 5 ms sample gate.
    pub const fn openffboard_flux_5ms() -> Self {
        Self::openffboard(850, 1150)
    }

    /// OpenFFBoard v1 torque-axis 1 ms sample gate.
    pub const fn openffboard_torque_1ms() -> Self {
        Self::openffboard(250, 1250)
    }

    /// OpenFFBoard v1 torque-axis 2 ms sample gate.
    pub const fn openffboard_torque_2ms() -> Self {
        Self::openffboard(700, 1250)
    }

    const fn openffboard(min_response_permille: u16, max_response_permille: u16) -> Self {
        Self {
            min_response_permille,
            max_response_permille,
            max_cross_axis_permille: 150,
            max_voltage_output_permille: 900,
            max_encoder_delta_counts: 4,
            actionable_status_mask: !CURRENT_AXIS_NON_ACTIONABLE_STATUS_MASK,
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
    let response_permille = ((sample.same_axis_actual.unsigned_abs() as u32 * 1000) / target_abs)
        .min(u16::MAX as u32) as u16;
    let cross_axis_permille = ((sample.cross_axis_actual.unsigned_abs() as u32 * 1000) / target_abs)
        .min(u16::MAX as u32) as u16;
    let output_limit = sample.output_limit.max(1) as u32;
    let voltage_output_permille = ((sample.output_limited.unsigned_abs() as u32 * 1000)
        / output_limit)
        .min(u16::MAX as u32) as u16;
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
    } else if cross_axis_permille > gate.max_cross_axis_permille {
        CURRENT_AXIS_VALIDATION_CROSS_AXIS
    } else if response_permille < gate.min_response_permille {
        CURRENT_AXIS_VALIDATION_LOW_RESPONSE
    } else if response_permille > gate.max_response_permille {
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

    /// Build impedance-split current-loop synthesis input from measured axis asymmetry.
    pub const fn with_impedance_split(
        model: ElectricalModel,
        profile: CommissionProfile,
        measured_axis_split_permille: u16,
        applied_axis_split_permille: u16,
        high_inductance_axis: u8,
    ) -> Self {
        Self {
            model,
            profile,
            measured_axis_split_permille,
            applied_axis_split_permille,
            axis_split_source: AXIS_SPLIT_SOURCE_IMPEDANCE,
            high_inductance_axis,
        }
    }
}

/// Compute the SIMC lambda (closed-loop time constant) for a given profile.
///
/// All profiles guarantee `lambda >= theta_e` (SIMC stability boundary).
fn simc_lambda_us(profile: CommissionProfile, tau_e_us: u32, theta_e_us: u32) -> u32 {
    let lambda = match profile {
        CommissionProfile::Conservative => (4 * theta_e_us).max(tau_e_us),
        CommissionProfile::Balanced => (2 * theta_e_us).max(tau_e_us / 2),
        CommissionProfile::Stiff => {
            let from_theta = theta_e_us;
            let from_tau = tau_e_us / 4;
            if from_theta > from_tau {
                from_theta
            } else {
                from_tau
            }
        }
    };
    // Enforce stability boundary: lambda >= theta_e
    if lambda >= theta_e_us {
        lambda
    } else {
        theta_e_us
    }
}

/// Compute SIMC PI gains for the current loop, returning `None` if the
/// electrical model is invalid (zero R, zero L, or zero theta).
pub fn simc_current_gains_checked(
    model: &ElectricalModel,
    profile: CommissionProfile,
    config: &crate::config::TmcConfig,
) -> Option<CurrentGains> {
    let lambda_us = simc_lambda_us(profile, model.tau_e_us, model.theta_e_us);
    simc_current_gains_with_lambda_checked(model, lambda_us, config)
}

/// Compute SIMC PI gains for the current loop using an explicit lambda.
pub fn simc_current_gains_with_lambda_checked(
    model: &ElectricalModel,
    lambda_us: u32,
    config: &crate::config::TmcConfig,
) -> Option<CurrentGains> {
    if model.r_int == 0 || model.l_int == 0 || model.theta_e_us == 0 {
        return None;
    }

    let lambda_us = lambda_us.max(model.theta_e_us);

    // SIMC PI tuning law for FOPDT plant G(s) = K / (tau*s+1) * e^(-theta*s):
    //   Kp = tau / (K * (lambda + theta))
    //   Ti = max(tau, CURRENT_SIMC_TI_MULTIPLIER * (lambda + theta))
    //   Ki = Kp / Ti
    //
    // For current loop: K = 1/R_int (DC gain: current per unit voltage).
    // All in internal units — no board-specific scaling needed.

    let tau = model.tau_e_us as f32;
    let theta = model.theta_e_us as f32;
    let lambda = lambda_us as f32;
    let r_int = model.r_int as f32 / ELECTRICAL_MODEL_RESISTANCE_SCALE as f32;
    let k = 1.0 / r_int; // Plant DC gain

    let kp = tau / (k * (lambda + theta));
    let ti = {
        let candidate = CURRENT_SIMC_TI_MULTIPLIER * (lambda + theta);
        if tau > candidate { tau } else { candidate }
    };
    let ti_s = ti * 1e-6; // Convert Ti from microseconds to seconds

    // Convert to the TMC4671 current-loop register format.
    //
    // OpenFFBoard leaves CONFIG_ADVANCED_PI_REPRESENT at its default 0, so the
    // current-loop I register field is raw Q8.8. In advanced PI mode the
    // integrator is series/interacting:
    //   x[k+1] = x[k] + Kp * (I/65536) * e[k]
    // so the raw Q8.8 I register places the PI zero as I/65536 per PWM sample.
    // A continuous Ti maps to I ~= 65536 / (Ti * f_pwm).
    let f_pwm = config.pwm_freq_hz() as f32;
    let p_reg = (kp * 256.0).clamp(1.0, u16::MAX as f32) as u16;
    let i_reg = (65_536.0 / (ti_s * f_pwm)).clamp(1.0, u16::MAX as f32) as u16;

    // Effective bandwidth = 1 / (2*pi*(lambda+theta)) in Hz
    let bw_hz = 1_000_000.0 / (2.0 * core::f32::consts::PI * (lambda + theta));

    Some(CurrentGains {
        flux_p: p_reg,
        flux_i: i_reg,
        torque_p: p_reg,
        torque_i: i_reg,
        effective_bandwidth_hz: bw_hz as u32,
        lambda_us,
    })
}

/// Synthesize production current-loop gains and gain-source evidence.
pub fn synthesize_current_gains(
    input: &CurrentGainSynthesisInput,
    config: &crate::config::TmcConfig,
) -> Option<CurrentGainSynthesis> {
    if input.model.r_int == 0 || input.model.l_int == 0 || input.model.theta_e_us == 0 {
        return None;
    }

    match input.axis_split_source {
        AXIS_SPLIT_SOURCE_NONE => {
            let lambda_us =
                simc_lambda_us(input.profile, input.model.tau_e_us, input.model.theta_e_us);
            let gains = simc_current_gains_with_lambda_checked(&input.model, lambda_us, config)?;
            return Some(CurrentGainSynthesis {
                gains,
                evidence: CurrentLoopEvidence::measured_symmetric(),
                flux_tau_us: input.model.tau_e_us,
                torque_tau_us: input.model.tau_e_us,
            });
        }
        AXIS_SPLIT_SOURCE_IMPEDANCE => {}
        _ => return None,
    }

    let applied_axis_split_permille = input.applied_axis_split_permille.max(1000);
    let split = applied_axis_split_permille as u64;
    let low_tau_us = input.model.tau_e_us;
    let high_tau_us = ((input.model.tau_e_us as u64 * split) / 1000).min(u32::MAX as u64) as u32;
    let (flux_tau_us, torque_tau_us) = match input.high_inductance_axis {
        CURRENT_AXIS_FLUX => (high_tau_us, low_tau_us),
        CURRENT_AXIS_TORQUE => (low_tau_us, high_tau_us),
        _ => return None,
    };

    let flux_model = axis_model(input.model, flux_tau_us);
    let torque_model = axis_model(input.model, torque_tau_us);
    let flux_lambda_us = simc_lambda_us(input.profile, flux_tau_us, input.model.theta_e_us);
    let torque_lambda_us = simc_lambda_us(input.profile, torque_tau_us, input.model.theta_e_us);
    let flux_gains = simc_current_gains_with_lambda_checked(&flux_model, flux_lambda_us, config)?;
    let torque_gains =
        simc_current_gains_with_lambda_checked(&torque_model, torque_lambda_us, config)?;

    Some(CurrentGainSynthesis {
        gains: CurrentGains {
            flux_p: flux_gains.flux_p,
            flux_i: flux_gains.flux_i,
            torque_p: torque_gains.torque_p,
            torque_i: torque_gains.torque_i,
            effective_bandwidth_hz: flux_gains
                .effective_bandwidth_hz
                .min(torque_gains.effective_bandwidth_hz),
            lambda_us: flux_gains.lambda_us.max(torque_gains.lambda_us),
        },
        evidence: CurrentLoopEvidence::measured_split(
            input.measured_axis_split_permille,
            applied_axis_split_permille,
            applied_axis_split_permille < input.measured_axis_split_permille,
        ),
        flux_tau_us,
        torque_tau_us,
    })
}

fn axis_model(model: ElectricalModel, tau_e_us: u32) -> ElectricalModel {
    let l_int = ((model.r_int as u64 * tau_e_us as u64) / ELECTRICAL_MODEL_RESISTANCE_SCALE as u64)
        .min(u32::MAX as u64) as u32;
    ElectricalModel {
        l_int,
        tau_e_us,
        ..model
    }
}

/// Compute SIMC PI gains for the current loop.
///
/// Returns default conservative gains if the electrical model is invalid.
pub fn simc_current_gains(
    model: &ElectricalModel,
    profile: CommissionProfile,
    config: &crate::config::TmcConfig,
) -> CurrentGains {
    simc_current_gains_checked(model, profile, config).unwrap_or(CurrentGains {
        flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
        flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
        torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
        torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
        effective_bandwidth_hz: 0,
        lambda_us: 0,
    })
}

/// Compute SIMC current-loop gains with an explicit lambda, falling back to defaults.
pub fn simc_current_gains_with_lambda(
    model: &ElectricalModel,
    lambda_us: u32,
    config: &crate::config::TmcConfig,
) -> CurrentGains {
    simc_current_gains_with_lambda_checked(model, lambda_us, config).unwrap_or(CurrentGains {
        flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
        flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
        torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
        torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
        effective_bandwidth_hz: 0,
        lambda_us: 0,
    })
}

/// Settling reference for current-loop validation, in microseconds.
///
/// Advanced PI uses the `I` register as the controller zero location. When the
/// current-loop integrator is intentionally damped, electrical tau is no longer
/// the dominant settling reference; the configured PI zero time is.
pub fn current_settling_reference_us(
    gains: &CurrentGains,
    config: &crate::config::TmcConfig,
) -> u32 {
    let i_reg = gains.flux_i.min(gains.torque_i).max(1) as u64;
    let f_pwm = config.pwm_freq_hz().max(1) as u64;
    (65_536_000_000u64 / (i_reg * f_pwm)).min(u32::MAX as u64) as u32
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
/// Returns metrics with `symmetry_pct = 0`. Symmetry requires two captures
/// (forward + reverse); the caller computes it via [`compute_tail_symmetry_pct`].
/// Timing fields are durations within the capture, not absolute times since
/// the command step. Callers that schedule the first sample after one sample
/// period should keep that origin outside this buffer.
pub fn extract_step_metrics(
    buf: &TransientBuffer,
    target: u32,
    pidout_limit: u32,
) -> CurrentValidation {
    let samples = buf.samples();
    if samples.is_empty() || target == 0 {
        return CurrentValidation {
            rise_time_us: 0,
            overshoot_pct: 0,
            settling_time_us: 0,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 0,
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
    let overshoot_pct = if peak > target {
        ((peak - target) as u64 * 1000 / target as u64) as u16
    } else {
        0
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
        overshoot_pct,
        settling_time_us,
        ringing_count,
        saturation_samples: saturation_count,
        symmetry_pct: 0,
        tail_level,
        target_error_pct: 0,
    }
}

/// Classify current-loop validation metrics against thresholds.
///
/// Thresholds (starting points, subject to hardware validation):
///
/// | Metric | Pass | Marginal | Fail |
/// |--------|------|----------|------|
/// | Overshoot | < 15% | 15-30% | > 30% |
/// | Ringing | 0-1 | 2-3 | > 3 |
/// | Settling | < 5*settle_ref | 5-10*settle_ref | > 10*settle_ref |
/// | Symmetry | <= 10% | 10-25% | > 25% |
/// | Saturation | < 10% | 10-25% | > 25% |
/// | Target error | <= 10% | 10-50% | > 50% |
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
    if metrics.overshoot_pct > 300 {
        return ValidationResult::Fail;
    } else if metrics.overshoot_pct > 150 {
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
    if metrics.symmetry_pct > 250 {
        return ValidationResult::Fail;
    } else if metrics.symmetry_pct > 100 {
        worst = ValidationResult::Marginal;
    }

    // Saturation (fraction of total samples): pass <10%, marginal 10-25%, fail >25%
    if total_samples > 0 {
        let sat_pct = (metrics.saturation_samples as u32 * 100) / total_samples as u32;
        if sat_pct > 25 {
            return ValidationResult::Fail;
        } else if sat_pct > 10 {
            worst = ValidationResult::Marginal;
        }
    }

    // Target attainment: did the current loop actually reach the commanded
    // target? Catches zero-current, wrong-sign, and under-target responses
    // that would otherwise pass all other checks.
    // 0.1% units: 500 = 50.0%, 100 = 10.0%
    if metrics.target_error_pct > 500 {
        return ValidationResult::Fail;
    } else if metrics.target_error_pct > 100 {
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
    if metrics.overshoot_pct > 150 || metrics.ringing_count > 1 {
        return false;
    }
    if metrics.settling_time_us > 5 * settle_reference_us {
        return false;
    }
    if total_samples > 0 {
        let sat_pct = (metrics.saturation_samples as u32 * 100) / total_samples as u32;
        if sat_pct > 10 {
            return false;
        }
    }

    metrics.symmetry_pct > 100 && metrics.symmetry_pct <= 250
}

fn retry_low_response_lambda_permille(_profile: CommissionProfile) -> u16 {
    500
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

fn scale_gain_for_lambda(gain: u16, previous_lambda_us: u32, next_lambda_us: u32) -> u16 {
    if previous_lambda_us == 0 || next_lambda_us == 0 {
        return gain;
    }

    ((gain as u64 * previous_lambda_us as u64) / next_lambda_us as u64).clamp(1, u16::MAX as u64)
        as u16
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

/// Adjust current-loop gains for one failed current axis according to retry policy.
pub fn retry_current_axis_gains(
    previous: CurrentGains,
    axis: u8,
    decision: CurrentAxisRetryDecision,
    profile: CommissionProfile,
) -> CurrentGains {
    let lambda_permille = match decision {
        CurrentAxisRetryDecision::RetryHigherBandwidth => {
            retry_low_response_lambda_permille(profile)
        }
        CurrentAxisRetryDecision::RetryLowerBandwidth => {
            retry_high_response_lambda_permille(profile)
        }
        CurrentAxisRetryDecision::RetryBudgetExhausted | CurrentAxisRetryDecision::Fail => {
            return previous;
        }
    };
    let min_lambda = scale_lambda_permille(previous.lambda_us, retry_min_lambda_permille(profile));
    let max_lambda = scale_lambda_permille(previous.lambda_us, retry_max_lambda_permille(profile));
    let next_lambda = scale_lambda_permille(previous.lambda_us, lambda_permille)
        .clamp(min_lambda.min(max_lambda), min_lambda.max(max_lambda));
    let mut next = previous;
    // Higher-bandwidth retry only speeds up the failed axis. The unchanged
    // axis still bounds the aggregate current-loop metadata, so keeping the
    // previous lower-bandwidth metadata remains conservative.
    if decision == CurrentAxisRetryDecision::RetryLowerBandwidth {
        next.lambda_us = previous.lambda_us.max(next_lambda);
        next.effective_bandwidth_hz =
            previous
                .effective_bandwidth_hz
                .min(scale_bandwidth_for_lambda(
                    previous.effective_bandwidth_hz,
                    previous.lambda_us,
                    next_lambda,
                ));
    }

    match axis {
        CURRENT_AXIS_FLUX => {
            next.flux_p = scale_gain_for_lambda(previous.flux_p, previous.lambda_us, next_lambda);
            next.flux_i = scale_gain_for_lambda(previous.flux_i, previous.lambda_us, next_lambda);
        }
        CURRENT_AXIS_TORQUE => {
            next.torque_p =
                scale_gain_for_lambda(previous.torque_p, previous.lambda_us, next_lambda);
            next.torque_i =
                scale_gain_for_lambda(previous.torque_i, previous.lambda_us, next_lambda);
        }
        _ => return previous,
    }

    next
}

/// Adjust current-loop integrator gain for a validation retry while retaining P damping.
pub fn current_validation_retry_gains(
    previous: CurrentGains,
    target: u32,
    settle_band: u32,
    forward_tail_level: u32,
    reverse_tail_level: u32,
    overshoot_pct: u16,
) -> CurrentGains {
    let min_tail = forward_tail_level.min(reverse_tail_level);
    let max_tail = forward_tail_level.max(reverse_tail_level);
    let upper = target.saturating_add(settle_band);
    let lower = target.saturating_sub(settle_band);
    let i = if overshoot_pct > 150 || max_tail > upper {
        (previous.flux_i / 2).max(1)
    } else if min_tail < lower {
        previous.flux_i.saturating_add((previous.flux_i / 2).max(1))
    } else {
        previous.flux_i
    };
    CurrentGains {
        flux_p: previous.flux_p,
        flux_i: i,
        torque_p: previous.torque_p,
        torque_i: i,
        effective_bandwidth_hz: previous.effective_bandwidth_hz,
        lambda_us: previous.lambda_us,
    }
}

/// Compute symmetry metric from forward and reverse tail levels.
///
/// Returns `|fwd - rev| / avg(fwd, rev)` in 0.1% units (permille).
/// This measures steady-state signed-current symmetry — whether both
/// directions produce the same actual current magnitude. Returns 0 if
/// both values are zero.
pub fn compute_tail_symmetry_pct(fwd_tail: u32, rev_tail: u32) -> u16 {
    let avg = (fwd_tail as u64 + rev_tail as u64) / 2;
    if avg == 0 {
        return 0;
    }
    let diff = fwd_tail.abs_diff(rev_tail);
    (diff as u64 * 1000 / avg) as u16
}

/// Compute target-attainment error from a tail level and commanded target.
///
/// Returns `|tail_level - target| / target` in 0.1% units (permille).
/// Returns 1000 (100%) if `target` is zero to avoid division by zero.
pub fn compute_target_error_pct(tail_level: u32, target: u32) -> u16 {
    if target == 0 {
        return 1000;
    }
    let diff = tail_level.abs_diff(target);
    (diff as u64 * 1000 / target as u64) as u16
}

#[cfg(test)]
mod tests {
    use super::super::test_support::{autotune_test_config, test_electrical_model};
    use super::*;

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
    fn current_loop_evidence_records_measured_split() {
        let evidence = CurrentLoopEvidence::measured_split(1850, 1500, true);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_MEASURED);
        assert_eq!(evidence.axis_split_source, AXIS_SPLIT_SOURCE_IMPEDANCE);
        assert_eq!(evidence.gains_tier, CURRENT_GAINS_TIER_MEASURED_SPLIT);
        assert_eq!(evidence.failure_reason, CURRENT_LOOP_FAILURE_NONE);
        assert_eq!(evidence.measured_axis_split_permille, 1850);
        assert_eq!(evidence.applied_axis_split_permille, 1500);
        assert_eq!(evidence.axis_split_clamped, 1);
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
        assert_eq!(AXIS_SPLIT_SOURCE_IMPEDANCE, 1);

        assert_eq!(CURRENT_GAINS_TIER_NONE, 0);
        assert_eq!(CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, 1);
        assert_eq!(CURRENT_GAINS_TIER_MEASURED_SPLIT, 2);
        assert_eq!(CURRENT_GAINS_TIER_DEFAULT, 3);
    }

    #[test]
    fn current_loop_evidence_pins_axis_and_sample_count_codes() {
        assert_eq!(CURRENT_VALIDATION_AXIS_FLUX, 1);
        assert_eq!(CURRENT_VALIDATION_AXIS_TORQUE, 2);

        assert_eq!(CURRENT_AXIS_FLUX, 0);
        assert_eq!(CURRENT_AXIS_TORQUE, 1);

        assert_eq!(CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, 3);
        assert_eq!(CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT, 2);
        assert_eq!(CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS, 3);
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
    fn retry_policy_low_response_retry_preserves_conservative_metadata() {
        let previous = CurrentGains {
            flux_p: 120,
            flux_i: 60,
            torque_p: 220,
            torque_i: 110,
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };

        let retry = retry_current_axis_gains(
            previous,
            CURRENT_AXIS_FLUX,
            CurrentAxisRetryDecision::RetryHigherBandwidth,
            CommissionProfile::Balanced,
        );

        assert!(retry.flux_p > previous.flux_p);
        assert!(retry.flux_i > previous.flux_i);
        assert_eq!(retry.torque_p, previous.torque_p);
        assert_eq!(retry.torque_i, previous.torque_i);
        assert_eq!(
            retry.effective_bandwidth_hz,
            previous.effective_bandwidth_hz
        );
        assert_eq!(retry.lambda_us, previous.lambda_us);
    }

    #[test]
    fn retry_policy_high_response_retry_reports_slower_aggregate_metadata() {
        let previous = CurrentGains {
            flux_p: 120,
            flux_i: 60,
            torque_p: 220,
            torque_i: 110,
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };

        let retry = retry_current_axis_gains(
            previous,
            CURRENT_AXIS_TORQUE,
            CurrentAxisRetryDecision::RetryLowerBandwidth,
            CommissionProfile::Balanced,
        );

        assert_eq!(retry.flux_p, previous.flux_p);
        assert_eq!(retry.flux_i, previous.flux_i);
        assert!(retry.torque_p < previous.torque_p);
        assert!(retry.torque_i < previous.torque_i);
        assert_eq!(retry.effective_bandwidth_hz, 72);
        assert_eq!(retry.lambda_us, 1868);
    }

    #[test]
    fn split_synthesis_uses_measured_symmetric_without_impedance() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);

        let output = synthesize_current_gains(&input, &config).unwrap();

        assert_eq!(
            output.evidence.gains_tier,
            CURRENT_GAINS_TIER_MEASURED_SYMMETRIC
        );
        assert_eq!(output.evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(output.gains.flux_p, output.gains.torque_p);
        assert_eq!(output.gains.flux_i, output.gains.torque_i);
    }

    #[test]
    fn split_synthesis_scales_flux_when_flux_is_high_inductance() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::with_impedance_split(
            model,
            CommissionProfile::Balanced,
            1800,
            1800,
            CURRENT_AXIS_FLUX,
        );

        let output = synthesize_current_gains(&input, &config).unwrap();

        assert_eq!(
            output.evidence.gains_tier,
            CURRENT_GAINS_TIER_MEASURED_SPLIT
        );
        assert_eq!(output.evidence.measured_axis_split_permille, 1800);
        assert_eq!(output.evidence.applied_axis_split_permille, 1800);
        assert_eq!(output.evidence.axis_split_clamped, 0);
        assert!(output.flux_tau_us > output.torque_tau_us);
        assert!(output.gains.flux_p > output.gains.torque_p);
    }

    #[test]
    fn split_synthesis_scales_torque_when_torque_is_high_inductance() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::with_impedance_split(
            model,
            CommissionProfile::Balanced,
            1800,
            1800,
            CURRENT_AXIS_TORQUE,
        );

        let output = synthesize_current_gains(&input, &config).unwrap();

        assert!(output.torque_tau_us > output.flux_tau_us);
        assert!(output.gains.torque_p > output.gains.flux_p);
    }

    #[test]
    fn split_synthesis_reports_clamped_axis_ratio() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::with_impedance_split(
            model,
            CommissionProfile::Balanced,
            2400,
            1500,
            CURRENT_AXIS_FLUX,
        );

        let output = synthesize_current_gains(&input, &config).unwrap();

        assert_eq!(output.evidence.measured_axis_split_permille, 2400);
        assert_eq!(output.evidence.applied_axis_split_permille, 1500);
        assert_eq!(output.evidence.axis_split_clamped, 1);
    }

    #[test]
    fn split_synthesis_rejects_invalid_axis_split_source() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let mut input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);
        input.axis_split_source = 0xff;

        assert!(synthesize_current_gains(&input, &config).is_none());
    }

    #[test]
    fn split_synthesis_rejects_invalid_high_inductance_axis() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::with_impedance_split(
            model,
            CommissionProfile::Balanced,
            1800,
            1800,
            0xff,
        );

        assert!(synthesize_current_gains(&input, &config).is_none());
    }

    #[test]
    fn split_synthesis_floors_under_unity_applied_split_in_evidence_and_math() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::with_impedance_split(
            model,
            CommissionProfile::Balanced,
            900,
            900,
            CURRENT_AXIS_FLUX,
        );
        let symmetric_input =
            CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);

        let output = synthesize_current_gains(&input, &config).unwrap();
        let symmetric = synthesize_current_gains(&symmetric_input, &config).unwrap();

        assert_eq!(output.evidence.measured_axis_split_permille, 900);
        assert_eq!(output.evidence.applied_axis_split_permille, 1000);
        assert_eq!(output.flux_tau_us, model.tau_e_us);
        assert_eq!(output.torque_tau_us, model.tau_e_us);
        assert_eq!(output.gains, symmetric.gains);
    }

    // --- SIMC current gains ---

    #[test]
    fn simc_balanced_gains_nonzero() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        assert!(gains.flux_p > 0, "flux_p must be nonzero");
        assert!(gains.flux_i > 0, "flux_i must be nonzero");
        assert_eq!(gains.flux_p, gains.torque_p, "flux/torque P must match");
        assert_eq!(gains.flux_i, gains.torque_i, "flux/torque I must match");
        assert!(gains.lambda_us > 0, "lambda must be nonzero");
        assert!(
            gains.effective_bandwidth_hz > 0,
            "bandwidth must be nonzero"
        );
    }

    #[test]
    fn simc_delay_dominated_plant_stiff_below_balanced() {
        // Delay-dominated: theta is large relative to tau.
        // Stiff = max(theta, tau/4) = max(700, 655) = 700
        // Balanced = 2*theta = 1400
        // So stiff < balanced here.
        let model = test_electrical_model(); // theta=700, tau=2622
        let config = autotune_test_config();
        let balanced = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        let stiff = simc_current_gains(&model, CommissionProfile::Stiff, &config);
        assert!(
            stiff.lambda_us < balanced.lambda_us,
            "delay-dominated: stiff lambda {} should be < balanced {}",
            stiff.lambda_us,
            balanced.lambda_us
        );
    }

    #[test]
    fn simc_tau_dominated_plant_profiles_remain_ordered() {
        // Tau-dominated current plants still need tau-based lambda floors:
        // Conservative >= Balanced >= Stiff.
        let model = ElectricalModel {
            r_int: 1125,
            l_int: 2950,
            tau_e_us: 4000,
            theta_e_us: 100,
            r_mohm: 1125,
            l_uh: 450,
            tau_e_crosscheck_us: 3800,
        };
        let config = autotune_test_config();
        let conservative = simc_current_gains(&model, CommissionProfile::Conservative, &config);
        let balanced = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        let stiff = simc_current_gains(&model, CommissionProfile::Stiff, &config);
        assert_eq!(conservative.lambda_us, 4000);
        assert_eq!(balanced.lambda_us, 2000);
        assert_eq!(stiff.lambda_us, 1000);
    }

    #[test]
    fn simc_invariants_hold_for_all_profiles() {
        // Universal SIMC invariants that hold regardless of plant shape:
        // 1. conservative >= balanced >= stiff
        // 2. conservative >= tau, balanced >= tau/2, stiff >= tau/4
        // 3. all profiles >= theta (SIMC stability boundary)
        for model in [
            test_electrical_model(),
            ElectricalModel {
                tau_e_us: 4000,
                theta_e_us: 100,
                ..test_electrical_model()
            },
            ElectricalModel {
                tau_e_us: 200,
                theta_e_us: 500,
                ..test_electrical_model()
            },
        ] {
            let config = autotune_test_config();
            let con = simc_current_gains(&model, CommissionProfile::Conservative, &config);
            let bal = simc_current_gains(&model, CommissionProfile::Balanced, &config);
            let stf = simc_current_gains(&model, CommissionProfile::Stiff, &config);

            assert!(
                con.lambda_us >= bal.lambda_us,
                "tau={} theta={}: conservative {} must be >= balanced {}",
                model.tau_e_us,
                model.theta_e_us,
                con.lambda_us,
                bal.lambda_us
            );
            assert!(
                bal.lambda_us >= stf.lambda_us,
                "tau={} theta={}: balanced {} must be >= stiff {}",
                model.tau_e_us,
                model.theta_e_us,
                bal.lambda_us,
                stf.lambda_us
            );
            assert!(
                con.lambda_us >= model.tau_e_us,
                "tau={} theta={}: conservative {} must be >= tau",
                model.tau_e_us,
                model.theta_e_us,
                con.lambda_us
            );
            assert!(
                bal.lambda_us >= model.tau_e_us / 2,
                "tau={} theta={}: balanced {} must be >= tau/2={}",
                model.tau_e_us,
                model.theta_e_us,
                bal.lambda_us,
                model.tau_e_us / 2
            );
            assert!(
                stf.lambda_us >= model.theta_e_us,
                "tau={} theta={}: stiff {} must be >= theta {}",
                model.tau_e_us,
                model.theta_e_us,
                stf.lambda_us,
                model.theta_e_us
            );
            assert!(
                stf.lambda_us >= model.tau_e_us / 4,
                "tau={} theta={}: stiff {} must be >= tau/4={}",
                model.tau_e_us,
                model.theta_e_us,
                stf.lambda_us,
                model.tau_e_us / 4
            );
            for (name, gains) in [("conservative", &con), ("balanced", &bal), ("stiff", &stf)] {
                assert!(
                    gains.lambda_us >= model.theta_e_us,
                    "tau={} theta={}: {} lambda {} must satisfy stability floor theta {}",
                    model.tau_e_us,
                    model.theta_e_us,
                    name,
                    gains.lambda_us,
                    model.theta_e_us
                );
            }
        }
    }

    #[test]
    fn simc_zero_r_returns_none() {
        let mut model = test_electrical_model();
        model.r_int = 0;
        let config = autotune_test_config();
        assert!(simc_current_gains_checked(&model, CommissionProfile::Balanced, &config).is_none());
    }

    #[test]
    fn simc_i_gain_not_trivial() {
        // With typical motor parameters, the I gain should be significantly
        // above the minimum clamp of 1. A value of 1 indicates a units bug.
        let model = test_electrical_model();
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        assert!(
            gains.flux_i > 10,
            "flux_i={} is suspiciously low (units bug?)",
            gains.flux_i
        );
    }

    #[test]
    fn simc_current_i_gain_uses_advanced_pi_zero_parameter() {
        let model = ElectricalModel {
            r_int: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_int: 1863,
            tau_e_us: 1863,
            theta_e_us: 160,
            r_mohm: 1000,
            l_uh: 1863,
            tau_e_crosscheck_us: 1863,
        };
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);

        // Balanced current-loop lambda is floored at tau/2 for robustness:
        // max(2*theta=320us, tau/2=931us). The damped SIMC integrator time is
        // max(tau, CURRENT_SIMC_TI_MULTIPLIER*(lambda+theta)). With the
        // default Q8.8 representation, TMC4671 advanced PI places the series
        // PI zero as I/65536 per PWM sample.
        assert_eq!(gains.lambda_us, model.tau_e_us / 2);
        let ti_s =
            CURRENT_SIMC_TI_MULTIPLIER as f64 * (gains.lambda_us + model.theta_e_us) as f64 * 1e-6;
        let expected_i = (65_536.0 / (ti_s * config.pwm_freq_hz() as f64)) as u16;

        assert_eq!(gains.flux_i, expected_i);
        assert_eq!(gains.torque_i, expected_i);
    }

    #[test]
    fn simc_current_i_gain_starts_near_observed_settling_bracket() {
        let model = ElectricalModel {
            r_int: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_int: 1868,
            tau_e_us: 1868,
            theta_e_us: 160,
            r_mohm: 1000,
            l_uh: 1868,
            tau_e_crosscheck_us: 1868,
        };
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);

        assert!(
            (215..=225).contains(&gains.flux_i),
            "initial I should start near the measured stable retry value, got {}",
            gains.flux_i
        );
        assert_eq!(gains.flux_i, gains.torque_i);
    }

    #[test]
    fn current_settling_reference_uses_advanced_pi_zero_time() {
        let gains = CurrentGains {
            flux_p: 436,
            flux_i: 220,
            torque_p: 436,
            torque_i: 220,
            effective_bandwidth_hz: 146,
            lambda_us: 923,
        };
        let config = autotune_test_config();

        assert_eq!(current_settling_reference_us(&gains, &config), 11915);
    }

    #[test]
    fn simc_retry_lambda_preserves_measured_theta() {
        let model = ElectricalModel {
            r_int: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_int: 1917,
            tau_e_us: 1917,
            theta_e_us: 160,
            r_mohm: 1000,
            l_uh: 1917,
            tau_e_crosscheck_us: 1917,
        };
        let config = autotune_test_config();
        let gains = simc_current_gains_with_lambda(&model, 1436, &config);

        let p_expected =
            (1917.0_f64 / (1436.0_f64 + 160.0_f64) * 256.0_f64).clamp(1.0, u16::MAX as f64) as u16;
        let ti_s = CURRENT_SIMC_TI_MULTIPLIER as f64 * (1436.0 + 160.0) * 1e-6;
        let i_expected =
            (65_536.0 / (ti_s * config.pwm_freq_hz() as f64)).clamp(1.0, u16::MAX as f64) as u16;

        assert_eq!(gains.lambda_us, 1436);
        assert_eq!(gains.flux_p, p_expected);
        assert_eq!(gains.flux_i, i_expected);
        assert!(gains.flux_p > 227, "retry P should not use inflated theta");
        assert!(gains.flux_i > 76, "retry I should not use inflated theta");
    }

    // --- Step metrics extraction ---

    /// Build a synthetic first-order step response for testing.
    /// Fills 256 samples at 100us intervals.
    fn synthetic_step_response(overshoot_pct: f32) -> TransientBuffer {
        let mut buf = TransientBuffer::new();
        buf.reset(100);
        let target = 1000.0f32;
        for i in 0..256u32 {
            let t = i as f32;
            let base = target * (1.0 - (-t / 10.0).exp());
            let overshoot_term = if overshoot_pct > 0.0 {
                target * overshoot_pct / 100.0 * (-t / 20.0).exp() * (t / 5.0).sin()
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
            metrics.overshoot_pct < 20,
            "overshoot {} should be <2%",
            metrics.overshoot_pct
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
            metrics.overshoot_pct > 100,
            "should detect significant overshoot: {}",
            metrics.overshoot_pct
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
        assert_eq!(metrics.overshoot_pct, 0);
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

    #[test]
    fn simc_dimensional_consistency() {
        // Compute gains using the microsecond-based function, then verify
        // against a manual calculation using seconds throughout. The final
        // Q8.8 register values must match, proving no units conversion error.
        let model = test_electrical_model();
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);

        // Manual calculation in SI (seconds):
        let tau_s = model.tau_e_us as f64 * 1e-6;
        let theta_s = model.theta_e_us as f64 * 1e-6;
        let lambda_s = ((2 * model.theta_e_us).max(model.tau_e_us / 2)) as f64 * 1e-6;
        let r_int = model.r_int as f64 / ELECTRICAL_MODEL_RESISTANCE_SCALE as f64;
        let k = 1.0 / r_int;
        let f_pwm = config.pwm_freq_hz() as f64;

        let kp_si = tau_s / (k * (lambda_s + theta_s));
        let ti_s = {
            let candidate = CURRENT_SIMC_TI_MULTIPLIER as f64 * (lambda_s + theta_s);
            if tau_s > candidate { tau_s } else { candidate }
        };
        let p_reg_expected = (kp_si * 256.0).clamp(1.0, u16::MAX as f64) as u16;
        let i_reg_expected = (65_536.0 / (ti_s * f_pwm)).clamp(1.0, u16::MAX as f64) as u16;

        assert_eq!(
            gains.flux_p, p_reg_expected,
            "P gain mismatch: got {} expected {}",
            gains.flux_p, p_reg_expected
        );
        // Allow +/- 1 for f32 vs f64 rounding
        let i_diff = (gains.flux_i as i32 - i_reg_expected as i32).unsigned_abs();
        assert!(
            i_diff <= 1,
            "I gain mismatch: got {} expected {} (diff {})",
            gains.flux_i,
            i_reg_expected,
            i_diff
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
        let gate = classify_current_axis_sample(&sample, &CurrentAxisGate::openffboard_flux_2ms());

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
        let gate = classify_current_axis_sample(&sample, &CurrentAxisGate::openffboard_flux_2ms());

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
        let gate = classify_current_axis_sample(&sample, &CurrentAxisGate::openffboard_flux_2ms());

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
        let gate = classify_current_axis_sample(&sample, &CurrentAxisGate::openffboard_flux_2ms());

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
        let gate = classify_current_axis_sample(&sample, &CurrentAxisGate::openffboard_flux_2ms());

        assert_eq!(gate.cross_axis_permille, u16::MAX);
        assert_eq!(gate.voltage_output_permille, u16::MAX);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_SATURATION);
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
        let gate = classify_current_axis_sample(&sample, &CurrentAxisGate::openffboard_flux_2ms());

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
        let gate = classify_current_axis_sample(&sample, &CurrentAxisGate::openffboard_flux_2ms());

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn classify_pass() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 100, // 10.0%
            settling_time_us: 2000,
            ringing_count: 1,
            saturation_samples: 5,
            symmetry_pct: 100, // 10.0%
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn classify_marginal_overshoot() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 200, // 20.0%
            settling_time_us: 2000,
            ringing_count: 1,
            saturation_samples: 5,
            symmetry_pct: 100,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_fail_overshoot() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 350, // 35.0%
            settling_time_us: 2000,
            ringing_count: 1,
            saturation_samples: 5,
            symmetry_pct: 100,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_fail_ringing() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 4,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_marginal_settling() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 7000, // 7x tau=1000
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_current_validation_settling_uses_controller_reference() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 141,
            settling_time_us: 39360,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 83,
            tail_level: 0,
            target_error_pct: 0,
        };

        assert_eq!(
            classify_current_validation(&metrics, 11915, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn accept_noop_retry_when_marginal_is_symmetry_only() {
        let previous = CurrentGains {
            flux_p: 437,
            flux_i: 217,
            torque_p: 437,
            torque_i: 217,
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };
        let retry = current_validation_retry_gains(previous, 367, 18, 351, 358, 119);
        // 180 permille: Marginal under tail-level thresholds (Pass <= 100, Fail > 250).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 119,
            settling_time_us: 40480,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 180,
            tail_level: 0,
            target_error_pct: 0,
        };

        assert_eq!(
            classify_current_validation(&metrics, 12080, 512),
            ValidationResult::Marginal
        );
        assert!(should_accept_current_validation_without_retry(
            &metrics, 12080, 512, previous, retry,
        ));
    }

    #[test]
    fn classify_marginal_symmetry_at_150_permille() {
        // With tail-level symmetry thresholds (Pass <= 100, Marginal <= 250),
        // 150 permille should be Marginal (not Pass as with old 200 threshold).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 150,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_fail_symmetry_at_300_permille() {
        // With tail-level thresholds (Fail > 250), 300 should be Fail
        // (not Marginal as with old 400 threshold).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 300,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_pass_symmetry_at_boundary() {
        // Exactly 100 permille should still pass.
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 100,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Pass
        );
    }

    // --- Target attainment ---

    #[test]
    fn classify_fails_when_both_tails_zero() {
        // Zero current in both directions: target_error = 1000‰ (100% off).
        let metrics = CurrentValidation {
            rise_time_us: 0,
            overshoot_pct: 0,
            settling_time_us: 0,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 1000, // 100% off target
        };
        assert_eq!(
            classify_current_validation(&metrics, 12000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_fails_when_tails_below_lower_band() {
        // tail_level=200 vs target=367 → error ≈ 455‰ (45.5% off).
        // |200 - 367| / 367 * 1000 = 455
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 0,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 20,
            tail_level: 200,
            target_error_pct: 455,
        };
        assert_eq!(
            classify_current_validation(&metrics, 12000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_passes_when_tails_in_band() {
        // tail_level=360 vs target=367 → error ≈ 19‰ (1.9% off).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 20,
            tail_level: 360,
            target_error_pct: 19,
        };
        assert_eq!(
            classify_current_validation(&metrics, 12000, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn current_validation_retries_first_failed_attempt() {
        assert!(should_retry_current_validation(ValidationResult::Fail, 0));
        assert!(should_retry_current_validation(
            ValidationResult::Marginal,
            0
        ));
        assert!(!should_retry_current_validation(ValidationResult::Pass, 0));
        assert!(!should_retry_current_validation(ValidationResult::Fail, 1));
    }

    #[test]
    fn current_validation_retry_reduces_integrator_when_tail_high() {
        let previous = CurrentGains {
            flux_p: 437,
            flux_i: 592,
            torque_p: 437,
            torque_i: 592,
            effective_bandwidth_hz: 143,
            lambda_us: 947,
        };

        let retry = current_validation_retry_gains(previous, 367, 18, 372, 403, 294);

        assert_eq!(retry.flux_p, previous.flux_p);
        assert_eq!(retry.torque_p, previous.torque_p);
        assert_eq!(retry.flux_i, 296);
        assert_eq!(retry.torque_i, 296);
        assert_eq!(retry.lambda_us, previous.lambda_us);
        assert_eq!(
            retry.effective_bandwidth_hz,
            previous.effective_bandwidth_hz
        );
    }

    #[test]
    fn current_validation_retry_increases_integrator_when_tail_low() {
        let previous = CurrentGains {
            flux_p: 436,
            flux_i: 151,
            torque_p: 436,
            torque_i: 151,
            effective_bandwidth_hz: 146,
            lambda_us: 923,
        };

        let retry = current_validation_retry_gains(previous, 367, 18, 354, 321, 35);

        assert_eq!(retry.flux_p, previous.flux_p);
        assert_eq!(retry.torque_p, previous.torque_p);
        assert_eq!(retry.flux_i, 226);
        assert_eq!(retry.torque_i, 226);
        assert_eq!(retry.lambda_us, previous.lambda_us);
        assert_eq!(
            retry.effective_bandwidth_hz,
            previous.effective_bandwidth_hz
        );
    }

    // --- Tail-level symmetry ---

    #[test]
    fn tail_symmetry_identical_is_zero() {
        assert_eq!(compute_tail_symmetry_pct(367, 367), 0);
    }

    #[test]
    fn tail_symmetry_small_mismatch() {
        // |351 - 358| / avg(351, 358) = 7 / 354.5 ≈ 19.7‰ → 19 in permille
        let result = compute_tail_symmetry_pct(351, 358);
        assert!(
            (15..=25).contains(&result),
            "tail symmetry {result} expected ~20‰"
        );
    }

    #[test]
    fn tail_symmetry_large_mismatch() {
        // |200 - 400| / avg(200, 400) = 200 / 300 ≈ 666.7‰
        let result = compute_tail_symmetry_pct(200, 400);
        assert!(
            (660..=670).contains(&result),
            "tail symmetry {result} expected ~667‰"
        );
    }

    #[test]
    fn tail_symmetry_zero_avg() {
        assert_eq!(compute_tail_symmetry_pct(0, 0), 0);
    }
}
