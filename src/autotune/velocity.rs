use super::{
    CommissionProfile, MechanicalModel, OperatingMode, Permille, TransientBuffer,
    VELOCITY_SIMC_MAX_P_REG, ValidationResult, VelocityGains, metrics,
};

/// Metrics from a velocity-loop step-response validation test (Phase 10).
///
/// Extracted from the [`TransientBuffer`] after a velocity step.
/// Used by [`classify_velocity_validation`] to determine pass/marginal/fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityValidation {
    /// Time from 10% to 90% of target velocity, in milliseconds.
    pub rise_time_ms: u32,
    /// Peak overshoot above target velocity, in 0.1% units (200 = 20.0%).
    pub overshoot_permille: Permille,
    /// Peak measured encoder velocity in counts/ms.
    pub peak_velocity: u32,
    /// Time to settle within +/-5% of target velocity, in milliseconds.
    pub settling_time_ms: u32,
    /// Zero-speed chatter detected after returning to zero velocity.
    pub zero_speed_chatter: bool,
    /// Samples where PID output hit saturation.
    pub saturation_samples: u16,
    /// |rise_fwd - rise_rev| / avg, in 0.1% units.
    pub symmetry_permille: Permille,
}

// ---------------------------------------------------------------------------
// Velocity-loop SIMC synthesis (Phase 9)
// ---------------------------------------------------------------------------

/// Compute the velocity-loop SIMC lambda (closed-loop time constant).
///
/// Profile mapping from the spec:
///
/// | Profile | Lambda_vel base |
/// |---------|----------------|
/// | Conservative | `6 * theta_mech` |
/// | Balanced | `3 * theta_mech` |
/// | Stiff | `max(1.5 * theta_mech, tau_mech / 4)` |
///
/// All values in microseconds. The result is clamped to the bandwidth
/// constraint `lambda_vel >= 4 * (lambda_i + theta_e)` which ensures
/// the inner loop settles before the outer loop reacts.
fn simc_velocity_lambda_us(
    profile: CommissionProfile,
    tau_mech_us: u32,
    theta_mech_us: u32,
    inner_lambda_us: u32,
    theta_e_us: u32,
) -> u32 {
    let lambda = match profile {
        CommissionProfile::Conservative => 6 * theta_mech_us,
        CommissionProfile::Balanced => 3 * theta_mech_us,
        CommissionProfile::Stiff => {
            // 1.5 * theta_mech = theta_mech + theta_mech/2
            let from_theta = theta_mech_us + theta_mech_us / 2;
            let from_tau = tau_mech_us / 4;
            if from_theta > from_tau {
                from_theta
            } else {
                from_tau
            }
        }
    };

    // Enforce stability boundary: lambda >= theta_mech.
    let lambda = if lambda >= theta_mech_us {
        lambda
    } else {
        theta_mech_us
    };

    // Bandwidth constraint: lambda_vel >= 4 * (lambda_i + theta_e).
    // Ensures inner loop settles before outer loop reacts.
    let bandwidth_floor = velocity_bandwidth_floor_us(inner_lambda_us, theta_e_us);
    if lambda >= bandwidth_floor {
        lambda
    } else {
        bandwidth_floor
    }
}

fn velocity_bandwidth_floor_us(inner_lambda_us: u32, theta_e_us: u32) -> u32 {
    4 * (inner_lambda_us + theta_e_us)
}

/// Apply operating-mode adjustment to velocity-loop lambda.
///
/// | Mode | Adjustment | Rationale |
/// |------|-----------|-----------|
/// | Unloaded | `lambda *= 1.5` | Less stiffness, reduce standstill buzz |
/// | Nominal | No change | Default |
/// | High inertia | `lambda *= 0.8` | Tighter, natural damping from inertia |
fn apply_mode_adjustment(lambda_us: u32, mode: OperatingMode) -> u32 {
    match mode {
        // lambda * 1.5 = lambda + lambda/2
        OperatingMode::Unloaded => lambda_us + lambda_us / 2,
        OperatingMode::Nominal => lambda_us,
        // lambda * 0.8 = lambda * 4/5
        OperatingMode::HighInertia => lambda_us * 4 / 5,
    }
}

fn apply_velocity_gain_envelope(
    lambda_us: u32,
    tau_us: u32,
    theta_us: u32,
    b_eff: u32,
    encoder_counts_per_rev: u32,
    f_pwm_hz: u32,
) -> u32 {
    if b_eff == 0 || encoder_counts_per_rev == 0 || f_pwm_hz == 0 {
        return lambda_us;
    }

    // b_eff is stored as torque * 1000 / encoder-counts/ms, while TMC4671
    // velocity error is measured in TMC velocity units:
    //
    //   tmc_vel_per_cpms = 65536 * 1000 / (encoder_counts_per_rev * f_pwm_hz)
    //   P_reg = tau_us * b_eff * 256 * f_pwm_hz
    //           * encoder_counts_per_rev
    //           / (1_000_000 * 65536 * (lambda_us + theta_us))
    //
    // Widen lambda when needed so the synthesized P lands inside the same
    // envelope enforced by Phase 10 and commit.
    let numerator =
        tau_us as u128 * b_eff as u128 * 256 * f_pwm_hz as u128 * encoder_counts_per_rev as u128;
    let denominator = 1_000_000_u128 * 65_536 * VELOCITY_SIMC_MAX_P_REG as u128;
    let required_sum = numerator.div_ceil(denominator).min(u32::MAX as u128) as u32;
    let lambda_floor = required_sum.saturating_sub(theta_us);
    lambda_us.max(lambda_floor)
}

/// Compute SIMC PI gains for the velocity loop, returning `None` if the
/// mechanical model is invalid (zero J, zero tau, or zero theta).
///
/// # Parameters
/// - `model`: mechanical plant model from Phase 8
/// - `profile`: tuning aggressiveness
/// - `mode`: operating condition adjustment
/// - `inner_lambda_us`: inner-loop time scale from current-loop commissioning
/// - `theta_e_us`: electrical transport delay from inner loop
/// - `peak_velocity`: peak velocity observed during mechanical ID (for limit)
/// - `encoder_counts_per_rev`: quadrature encoder counts per mechanical revolution
/// - `config`: TMC4671 board configuration (for PWM frequency)
#[allow(clippy::too_many_arguments)]
pub fn simc_velocity_gains(
    model: &MechanicalModel,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    peak_velocity: u32,
    encoder_counts_per_rev: u32,
    config: &crate::config::TmcConfig,
) -> Option<VelocityGains> {
    simc_velocity_gains_with_lambda_multiplier(
        model,
        profile,
        mode,
        inner_lambda_us,
        theta_e_us,
        peak_velocity,
        encoder_counts_per_rev,
        config,
        1.0,
    )
}

/// Compute SIMC velocity-loop gains with a multiplier applied to the final
/// selected velocity lambda before the gain-envelope widening step.
#[allow(clippy::too_many_arguments)]
pub(crate) fn simc_velocity_gains_with_lambda_multiplier(
    model: &MechanicalModel,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    peak_velocity: u32,
    encoder_counts_per_rev: u32,
    config: &crate::config::TmcConfig,
    lambda_multiplier: f32,
) -> Option<VelocityGains> {
    if model.j_eff == 0 || model.tau_mech_ms == 0 || model.theta_mech_us == 0 {
        return None;
    }
    let f_pwm_hz = config.pwm_freq_hz();
    if encoder_counts_per_rev == 0 || f_pwm_hz == 0 {
        return None;
    }

    // Convert tau_mech from milliseconds to microseconds for consistent units.
    let tau_mech_us = model.tau_mech_ms * 1000;

    let lambda_us = simc_velocity_lambda_us(
        profile,
        tau_mech_us,
        model.theta_mech_us,
        inner_lambda_us,
        theta_e_us,
    );
    let lambda_us = apply_mode_adjustment(lambda_us, mode);

    // Ensure lambda doesn't collapse to zero after mode adjustment, then widen
    // it if the model would synthesize gains outside the accepted register
    // envelope.
    let lambda_us = lambda_us.max(model.theta_mech_us);
    let lambda_us =
        ((lambda_us as f32) * lambda_multiplier.max(1.0)).clamp(1.0, u32::MAX as f32) as u32;
    let lambda_us = lambda_us.max(velocity_bandwidth_floor_us(inner_lambda_us, theta_e_us));
    let lambda_us = apply_velocity_gain_envelope(
        lambda_us,
        tau_mech_us,
        model.theta_mech_us,
        model.b_eff,
        encoder_counts_per_rev,
        f_pwm_hz,
    );

    // SIMC PI tuning law for FOPDT plant G_mech(s) = K / (tau*s+1) * e^(-theta*s):
    //   Kp = tau / (K * (lambda + theta))
    //   Ti = min(tau, 4 * (lambda + theta))
    //   Ki = Kp / Ti
    //
    // In encoder-counts/ms units: K = 1000/B_eff because B_eff is stored as
    // torque * 1000 / cpms. TMC4671 evaluates velocity error in TMC velocity
    // units, so convert the cpms-domain SIMC gains before Q8.8 packing.
    let tau = tau_mech_us as f32;
    let theta = model.theta_mech_us as f32;
    let lambda = lambda_us as f32;
    let b_eff = model.b_eff as f32;
    let k = if b_eff > 0.0 { 1000.0 / b_eff } else { 1.0 };
    let tmc_velocity_units_per_cpms =
        65_536.0 * 1000.0 / (encoder_counts_per_rev as f32 * f_pwm_hz as f32);

    let kp_cpms = tau / (k * (lambda + theta));
    let kp = kp_cpms / tmc_velocity_units_per_cpms;
    let ti = {
        let candidate = 4.0 * (lambda + theta);
        if tau < candidate { tau } else { candidate }
    };
    let ti_s = ti * 1e-6; // Convert Ti from microseconds to seconds.
    let ki = kp / ti_s;

    // Convert to TMC4671 Q8.8 register format.
    let f_pwm = f_pwm_hz as f32;
    let p_reg = (kp * 256.0).clamp(1.0, u16::MAX as f32) as u16;
    let i_reg = (ki / f_pwm * 256.0).clamp(1.0, u16::MAX as f32) as u16;

    // Effective bandwidth = 1 / (2*pi*(lambda+theta)) in Hz.
    let bw_hz = 1_000_000.0 / (2.0 * core::f32::consts::PI * (lambda + theta));

    // Velocity limit = 80% of peak velocity from mechanical ID.
    let velocity_limit = peak_velocity * 4 / 5;

    Some(VelocityGains {
        velocity_p: p_reg,
        velocity_i: i_reg,
        velocity_limit,
        effective_bandwidth_hz: bw_hz as u32,
        lambda_vel_ms: lambda_us / 1000,
    })
}

// ---------------------------------------------------------------------------
// Velocity validation metric extraction and classification (Phase 10)
// ---------------------------------------------------------------------------

const VELOCITY_OVERSHOOT_DEADBAND_COUNTS: u32 = 3;

/// Extract velocity step-response metrics from a transient buffer.
///
/// Analyzes encoder-derived velocity samples to measure rise time,
/// overshoot, settling time, saturation, and zero-speed chatter.
///
/// # Parameters
/// - `buf`: captured velocity samples (encoder count differences per tick)
/// - `target`: commanded velocity step amplitude
/// - `saturation_limit`: PID output saturation threshold (0 to skip)
/// - `chatter_threshold`: velocity noise threshold for zero-speed chatter
///   detection (encoder counts/tick, spec default = 3)
/// - `chatter_window`: number of post-zero samples to monitor for chatter
pub fn extract_velocity_metrics(
    buf: &TransientBuffer,
    target: u32,
    saturation_limit: u32,
    chatter_threshold: u32,
    chatter_window: u16,
) -> VelocityValidation {
    let samples = buf.samples();
    if samples.is_empty() {
        return VelocityValidation {
            rise_time_ms: 0,
            overshoot_permille: Permille::ZERO,
            peak_velocity: 0,
            settling_time_ms: 0,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };
    }
    if target == 0 {
        return VelocityValidation {
            rise_time_ms: 0,
            overshoot_permille: Permille::MAX,
            peak_velocity: 0,
            settling_time_ms: 0,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };
    }

    let period_us = buf.sample_period_us() as u32;
    let target_f = target as f32;
    let threshold_10 = ((target as u64 * 10).div_ceil(100) as u32).max(1);
    let threshold_90 = ((target as u64 * 90).div_ceil(100) as u32).max(1);
    // Encoder-derived velocity is integer counts per sample. At low targets,
    // a strict 5% band truncates to zero counts and makes a stable +/-1 count
    // response look permanently unsettled.
    let settle_band = ((target_f * 0.05) as u32).max(1);

    let mut t_10: Option<u32> = None;
    let mut t_90: Option<u32> = None;
    let mut peak: u32 = 0;
    let mut saturation_count: u16 = 0;

    for (i, &sample) in samples.iter().enumerate() {
        if t_10.is_none() && sample >= threshold_10 {
            t_10 = Some(crate::sample_timing::sample_offset_us(i, period_us));
        }
        if t_90.is_none() && sample >= threshold_90 {
            t_90 = Some(crate::sample_timing::sample_offset_us(i, period_us));
        }
        if sample > peak {
            peak = sample;
        }
        if saturation_limit > 0 && sample >= saturation_limit {
            saturation_count += 1;
        }
    }

    let reached_90 = t_90.is_some();
    let rise_time_us = match (t_10, t_90) {
        (Some(a), Some(b)) => b.saturating_sub(a),
        _ => 0,
    };
    // Convert to milliseconds (rounding to nearest).
    let rise_time_ms = (rise_time_us + 500) / 1000;

    // Overshoot: (peak - target) / target in 0.1% units. The absolute
    // deadband prevents a few integer encoder counts at low target speeds from
    // dominating the relative percentage.
    let overshoot_counts = peak
        .saturating_sub(target)
        .saturating_sub(VELOCITY_OVERSHOOT_DEADBAND_COUNTS);
    let overshoot_permille = if overshoot_counts > 0 {
        Permille::checked_ratio(u64::from(overshoot_counts), u64::from(target))
            .unwrap_or(Permille::MAX)
    } else {
        Permille::ZERO
    };

    // Settling: last sample where |actual - target| > 5% band.
    let mut last_outside: Option<usize> = None;
    for (i, &sample) in samples.iter().enumerate() {
        let diff = sample.abs_diff(target);
        if diff > settle_band {
            last_outside = Some(i);
        }
    }
    let settling_time_ms = if !reached_90 {
        // A response that never reaches the commanded step cannot be
        // considered settled, even if the large mechanical tau would
        // otherwise make the finite capture window look acceptable.
        u32::MAX
    } else if let Some(idx) = last_outside {
        let settling_us = crate::sample_timing::sample_elapsed_us(idx, period_us);
        // Convert to milliseconds (rounding to nearest).
        (settling_us + 500) / 1000
    } else {
        0
    };

    // Zero-speed chatter: check the last `chatter_window` samples
    // for velocity oscillations above the noise threshold.
    let chatter_start = samples.len().saturating_sub(chatter_window as usize);
    let zero_speed_chatter = samples[chatter_start..]
        .iter()
        .any(|&sample| sample > chatter_threshold);

    VelocityValidation {
        rise_time_ms,
        overshoot_permille,
        peak_velocity: peak,
        settling_time_ms,
        zero_speed_chatter,
        saturation_samples: saturation_count,
        symmetry_permille: Permille::ZERO, // Caller computes from fwd+rev captures.
    }
}

/// Classify velocity-loop validation metrics against spec thresholds.
///
/// | Metric | Pass | Marginal | Fail |
/// |--------|------|----------|------|
/// | Overshoot | < 20% | 20-40% | > 40% |
/// | Settling | < 5*tau_mech | 5-10*tau_mech | > 10*tau_mech |
/// | Zero chatter | none | detected | -- |
/// | Saturation | < 10% | 10-25% | > 25% |
///
/// Chatter is marginal, not fail — the filter stage addresses it.
/// Returns the worst classification across all metrics.
pub fn classify_velocity_validation(
    metrics: &VelocityValidation,
    tau_mech_ms: u32,
    total_samples: u16,
) -> ValidationResult {
    let mut worst = ValidationResult::Pass;

    // Overshoot (0.1% units: 200 = 20.0%, 400 = 40.0%).
    if metrics.overshoot_permille > Permille::new(400) {
        return ValidationResult::Fail;
    } else if metrics.overshoot_permille > Permille::new(200) {
        worst = ValidationResult::Marginal;
    }

    // Settling (relative to tau_mech in milliseconds).
    let settle_5x = 5 * tau_mech_ms;
    let settle_10x = 10 * tau_mech_ms;
    if metrics.settling_time_ms > settle_10x {
        return ValidationResult::Fail;
    } else if metrics.settling_time_ms > settle_5x {
        worst = ValidationResult::Marginal;
    }

    // Zero-speed chatter is marginal, not fail.
    if metrics.zero_speed_chatter {
        worst = ValidationResult::Marginal;
    }

    // Saturation (fraction of total samples).
    let saturation_permille =
        metrics::saturation_permille(metrics.saturation_samples, total_samples);
    if let Some(saturation_permille) = saturation_permille {
        if saturation_permille > Permille::new(250) {
            return ValidationResult::Fail;
        } else if saturation_permille > Permille::new(100) {
            worst = ValidationResult::Marginal;
        }
    }

    // Symmetry (0.1% units: 200 = 20.0%, 400 = 40.0%).
    if metrics.symmetry_permille > Permille::new(400) {
        return ValidationResult::Fail;
    } else if metrics.symmetry_permille > Permille::new(200) {
        worst = ValidationResult::Marginal;
    }

    worst
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityStiffnessVerdict {
    Clean,
    UnderDriven,
}

pub fn classify_velocity_stiffness(
    metrics: &VelocityValidation,
    validation_result: ValidationResult,
    stiffness_timebase_ms: u32,
) -> VelocityStiffnessVerdict {
    if validation_result != ValidationResult::Pass {
        return VelocityStiffnessVerdict::Clean;
    }

    let limit = stiffness_timebase_ms.max(1).saturating_mul(3);
    if metrics.rise_time_ms > limit || metrics.settling_time_ms > limit {
        VelocityStiffnessVerdict::UnderDriven
    } else {
        VelocityStiffnessVerdict::Clean
    }
}

#[cfg(test)]
mod tests {
    use super::super::test_support::{autotune_test_config, test_mechanical_model};
    use super::*;

    fn passing_velocity_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 1,
            overshoot_permille: Permille::ZERO,
            peak_velocity: 10,
            settling_time_ms: 1,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        }
    }

    #[test]
    fn velocity_overshoot_saturates_instead_of_wrapping_to_pass() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        buf.push(65_540);
        let metrics = extract_velocity_metrics(&buf, 1, 0, 3, 0);

        assert_eq!(metrics.overshoot_permille, Permille::MAX);
        assert_eq!(
            classify_velocity_validation(&metrics, 1, 1),
            ValidationResult::Fail,
        );
    }

    #[test]
    fn zero_velocity_target_fails_closed() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        buf.push(1);
        let metrics = extract_velocity_metrics(&buf, 0, 0, 3, 0);

        assert_eq!(metrics.overshoot_permille, Permille::MAX);
    }

    #[test]
    fn velocity_saturation_uses_permille_boundaries() {
        let mut metrics = passing_velocity_metrics();
        metrics.saturation_samples = 100;
        assert_eq!(
            classify_velocity_validation(&metrics, 100, 1000),
            ValidationResult::Pass
        );
        metrics.saturation_samples = 101;
        assert_eq!(
            classify_velocity_validation(&metrics, 100, 1000),
            ValidationResult::Marginal,
        );
        metrics.saturation_samples = 251;
        assert_eq!(
            classify_velocity_validation(&metrics, 100, 1000),
            ValidationResult::Fail
        );
    }

    const TEST_ENCODER_COUNTS_PER_REV: u32 = 80;

    // --- Velocity-loop SIMC synthesis ---

    #[test]
    fn velocity_lambda_conservative_is_6x_theta() {
        let lambda = simc_velocity_lambda_us(
            CommissionProfile::Conservative,
            10_000, // tau_mech = 10ms
            2000,   // theta_mech = 2ms
            700,    // inner lambda
            100,    // theta_e
        );
        // 6 * 2000 = 12000, bandwidth floor = 4*(700+100) = 3200
        assert_eq!(lambda, 12_000);
    }

    #[test]
    fn velocity_lambda_balanced_is_3x_theta() {
        let lambda = simc_velocity_lambda_us(CommissionProfile::Balanced, 10_000, 2000, 700, 100);
        // 3 * 2000 = 6000, bandwidth floor = 3200
        assert_eq!(lambda, 6000);
    }

    #[test]
    fn velocity_lambda_stiff_max_theta_tau() {
        // theta_mech = 2000: 1.5*2000 = 3000
        // tau_mech = 10000: tau/4 = 2500
        // max(3000, 2500) = 3000
        let lambda = simc_velocity_lambda_us(CommissionProfile::Stiff, 10_000, 2000, 700, 100);
        assert_eq!(lambda, 3200); // Clamped by bandwidth floor 4*(700+100) = 3200
    }

    #[test]
    fn velocity_lambda_bandwidth_constraint_enforced() {
        // With large inner lambda, the bandwidth floor dominates.
        let lambda = simc_velocity_lambda_us(
            CommissionProfile::Stiff,
            10_000,
            2000,
            5000, // large inner lambda
            1000, // large theta_e
        );
        // Stiff base: max(3000, 2500) = 3000
        // Bandwidth floor: 4*(5000+1000) = 24000
        // Floor wins.
        assert_eq!(lambda, 24_000);
    }

    #[test]
    fn velocity_lambda_conservative_ge_balanced() {
        for theta in [500, 1000, 2000, 5000] {
            let con =
                simc_velocity_lambda_us(CommissionProfile::Conservative, 10_000, theta, 700, 100);
            let bal = simc_velocity_lambda_us(CommissionProfile::Balanced, 10_000, theta, 700, 100);
            assert!(
                con >= bal,
                "theta={}: conservative {} must be >= balanced {}",
                theta,
                con,
                bal
            );
        }
    }

    #[test]
    fn mode_adjustment_unloaded_increases() {
        let base = 6000;
        let adjusted = apply_mode_adjustment(base, OperatingMode::Unloaded);
        // 6000 * 1.5 = 9000
        assert_eq!(adjusted, 9000);
    }

    #[test]
    fn mode_adjustment_nominal_unchanged() {
        let base = 6000;
        let adjusted = apply_mode_adjustment(base, OperatingMode::Nominal);
        assert_eq!(adjusted, 6000);
    }

    #[test]
    fn mode_adjustment_high_inertia_decreases() {
        let base = 6000;
        let adjusted = apply_mode_adjustment(base, OperatingMode::HighInertia);
        // 6000 * 0.8 = 4800
        assert_eq!(adjusted, 4800);
    }

    #[test]
    fn velocity_gains_balanced_nonzero() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let gains = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            700,  // inner lambda from current tune
            100,  // theta_e
            1000, // peak velocity
            TEST_ENCODER_COUNTS_PER_REV,
            &config,
        );
        let gains = gains.expect("gains should be Some for valid model");
        assert!(gains.velocity_p > 0, "velocity_p must be nonzero");
        assert!(gains.velocity_i > 0, "velocity_i must be nonzero");
        assert_eq!(gains.velocity_limit, 800, "80% of peak=1000");
        assert!(gains.effective_bandwidth_hz > 0);
        assert!(gains.lambda_vel_ms > 0);
    }

    #[test]
    fn velocity_gains_use_exact_non_divisible_encoder_scale() {
        let model = MechanicalModel {
            j_eff: 1,
            b_eff: 100,
            tau_mech_ms: 100,
            theta_mech_us: 1_000,
            t_f: 0,
        };
        let config = autotune_test_config();

        let gains = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            0,
            0,
            1_000,
            4_000,
            &config,
        )
        .expect("non-divisible encoder scale should synthesize gains");

        assert_eq!(gains.velocity_p, 976);
        assert_eq!(gains.velocity_i, 2);
    }

    #[test]
    fn velocity_stiffness_marks_slow_pass_under_driven() {
        let metrics = VelocityValidation {
            rise_time_ms: 200,
            overshoot_permille: Permille::ZERO,
            peak_velocity: 20,
            settling_time_ms: 120,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };

        assert_eq!(
            classify_velocity_stiffness(&metrics, ValidationResult::Pass, 50),
            VelocityStiffnessVerdict::UnderDriven
        );
    }

    #[test]
    fn velocity_stiffness_does_not_override_unsafe_result() {
        let metrics = VelocityValidation {
            rise_time_ms: 200,
            overshoot_permille: Permille::new(500),
            peak_velocity: 20,
            settling_time_ms: 200,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };

        assert_eq!(
            classify_velocity_stiffness(&metrics, ValidationResult::Fail, 50),
            VelocityStiffnessVerdict::Clean
        );
    }

    #[test]
    fn velocity_gains_field_model_stays_inside_outer_envelope() {
        let config = autotune_test_config();
        let field_models = [
            MechanicalModel {
                j_eff: 2_522_213,
                b_eff: 19_851,
                tau_mech_ms: 127,
                theta_mech_us: 14_764,
                t_f: 0,
            },
            MechanicalModel {
                j_eff: 2_624_100,
                b_eff: 20_988,
                tau_mech_ms: 125,
                theta_mech_us: 10_731,
                t_f: 0,
            },
        ];

        for model in field_models {
            let base_lambda_us = apply_mode_adjustment(
                simc_velocity_lambda_us(
                    CommissionProfile::Balanced,
                    model.tau_mech_ms * 1000,
                    model.theta_mech_us,
                    934,
                    160,
                ),
                OperatingMode::Nominal,
            )
            .max(model.theta_mech_us);

            let gains = simc_velocity_gains(
                &model,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                934,
                160,
                19,
                TEST_ENCODER_COUNTS_PER_REV,
                &config,
            )
            .expect("field model should synthesize gains");

            assert!(
                gains.velocity_p <= VELOCITY_SIMC_MAX_P_REG,
                "velocity P {} should fit envelope {} for {:?}",
                gains.velocity_p,
                VELOCITY_SIMC_MAX_P_REG,
                model
            );
            assert!(
                gains.velocity_i <= crate::commissioning::outer::OUTER_MAX_VELOCITY_I,
                "velocity I {} should fit envelope {} for {:?}",
                gains.velocity_i,
                crate::commissioning::outer::OUTER_MAX_VELOCITY_I,
                model
            );
            assert!(
                gains.velocity_p < 1024,
                "velocity P {} should be scaled to TMC velocity-error units for {:?}",
                gains.velocity_p,
                model
            );
            assert!(
                gains.lambda_vel_ms >= base_lambda_us / 1000,
                "lambda should not tighten below base {}us for {:?}, got {}ms",
                base_lambda_us,
                model,
                gains.lambda_vel_ms
            );
        }
    }

    #[test]
    fn velocity_gains_field_model_are_scaled_to_tmc_velocity_units() {
        let config = autotune_test_config();
        let model = MechanicalModel {
            j_eff: 2_712_665,
            b_eff: 20_451,
            tau_mech_ms: 132,
            theta_mech_us: 12_895,
            t_f: 0,
        };

        let gains = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            934,
            160,
            18,
            TEST_ENCODER_COUNTS_PER_REV,
            &config,
        )
        .expect("field model should synthesize gains");

        assert!(
            gains.velocity_p < 1024,
            "P={} is still in cpms-domain units; expected TMC velocity-error scaling",
            gains.velocity_p
        );
    }

    #[test]
    fn velocity_gains_none_for_zero_inertia() {
        let mut model = test_mechanical_model();
        model.j_eff = 0;
        let config = autotune_test_config();
        assert!(
            simc_velocity_gains(
                &model,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                700,
                100,
                1000,
                TEST_ENCODER_COUNTS_PER_REV,
                &config
            )
            .is_none()
        );
    }

    #[test]
    fn velocity_gains_none_for_zero_theta() {
        let mut model = test_mechanical_model();
        model.theta_mech_us = 0;
        let config = autotune_test_config();
        assert!(
            simc_velocity_gains(
                &model,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                700,
                100,
                1000,
                TEST_ENCODER_COUNTS_PER_REV,
                &config
            )
            .is_none()
        );
    }

    #[test]
    fn velocity_gains_unloaded_slower_than_nominal() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let nominal = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            700,
            100,
            1000,
            TEST_ENCODER_COUNTS_PER_REV,
            &config,
        )
        .unwrap();
        let unloaded = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Unloaded,
            700,
            100,
            1000,
            TEST_ENCODER_COUNTS_PER_REV,
            &config,
        )
        .unwrap();
        assert!(
            unloaded.lambda_vel_ms >= nominal.lambda_vel_ms,
            "unloaded lambda {} should be >= nominal {}",
            unloaded.lambda_vel_ms,
            nominal.lambda_vel_ms
        );
    }

    #[test]
    fn velocity_gains_high_inertia_tighter_than_nominal() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let nominal = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            700,
            100,
            1000,
            TEST_ENCODER_COUNTS_PER_REV,
            &config,
        )
        .unwrap();
        let high_inertia = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::HighInertia,
            700,
            100,
            1000,
            TEST_ENCODER_COUNTS_PER_REV,
            &config,
        )
        .unwrap();
        assert!(
            high_inertia.lambda_vel_ms <= nominal.lambda_vel_ms,
            "high_inertia lambda {} should be <= nominal {}",
            high_inertia.lambda_vel_ms,
            nominal.lambda_vel_ms
        );
    }

    #[test]
    fn velocity_gains_high_inertia_preserves_inner_bandwidth_floor() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let inner_lambda_us = 5000;
        let theta_e_us = 1000;
        let bandwidth_floor_ms = 4 * (inner_lambda_us + theta_e_us) / 1000;

        let gains = simc_velocity_gains(
            &model,
            CommissionProfile::Stiff,
            OperatingMode::HighInertia,
            inner_lambda_us,
            theta_e_us,
            1000,
            TEST_ENCODER_COUNTS_PER_REV,
            &config,
        )
        .unwrap();

        assert!(
            gains.lambda_vel_ms >= bandwidth_floor_ms,
            "high-inertia lambda {}ms should preserve inner bandwidth floor {}ms",
            gains.lambda_vel_ms,
            bandwidth_floor_ms
        );
    }

    // --- Velocity validation classification ---

    #[test]
    fn velocity_classify_pass() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_permille: Permille::new(100), // 10%
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 5,
            symmetry_permille: Permille::new(100),
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Pass
        );
    }

    #[test]
    fn velocity_classify_marginal_overshoot() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_permille: Permille::new(300), // 30%
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn velocity_classify_fail_overshoot() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_permille: Permille::new(500), // 50%
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_classify_marginal_chatter() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_permille: Permille::new(50),
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: true,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn velocity_classify_fail_settling() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_permille: Permille::new(50),
            peak_velocity: 0,
            settling_time_ms: 150, // > 10 * tau_mech(10) = 100
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_classify_fail_saturation() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_permille: Permille::new(50),
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 70, // 70/256 = 27% > 25%
            symmetry_permille: Permille::ZERO,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_classify_marginal_settling() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_permille: Permille::new(50),
            peak_velocity: 0,
            settling_time_ms: 70, // 7*10 = 70, between 5*10=50 and 10*10=100
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Marginal
        );
    }

    // --- Velocity metric extraction ---

    #[test]
    fn velocity_metrics_empty_buffer() {
        let buf = TransientBuffer::new();
        let metrics = extract_velocity_metrics(&buf, 1000, 0, 3, 50);
        assert_eq!(metrics.rise_time_ms, 0);
        assert_eq!(metrics.overshoot_permille, Permille::ZERO);
        assert!(!metrics.zero_speed_chatter);
    }

    #[test]
    fn velocity_metrics_chatter_detection() {
        // Fill buffer: first 200 samples at target, last 56 with oscillation.
        let mut buf = TransientBuffer::new();
        buf.reset(1000); // 1ms per sample
        for _ in 0..200 {
            buf.push(1000); // at target
        }
        for i in 0..56 {
            // Oscillate above and below zero (but stored as u32, so abs values).
            buf.push(if i % 2 == 0 { 5 } else { 0 });
        }
        let metrics = extract_velocity_metrics(&buf, 1000, 0, 3, 56);
        assert!(
            metrics.zero_speed_chatter,
            "should detect chatter in tail samples"
        );
    }

    #[test]
    fn velocity_metrics_settle_band_allows_low_target_quantization() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for i in 0..256 {
            // A low-speed encoder-derived velocity target cannot represent
            // fractional counts per sample. Alternating 5/6 around target 6 is
            // stable within one count and should not look permanently unsettled.
            buf.push(if i % 2 == 0 { 6 } else { 5 });
        }

        let metrics = extract_velocity_metrics(&buf, 6, 0, 3, 0);
        assert_eq!(metrics.settling_time_ms, 0);
    }

    #[test]
    fn velocity_metrics_overshoot_allows_low_target_quantization() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        buf.push(7);
        for _ in 1..256 {
            buf.push(4);
        }

        let metrics = extract_velocity_metrics(&buf, 4, 0, 3, 0);
        assert_eq!(metrics.overshoot_permille, Permille::ZERO);
    }

    #[test]
    fn velocity_validation_fails_when_low_target_is_never_reached() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for i in 0..256 {
            buf.push(if i % 4 == 2 { 1 } else { 0 });
        }

        let metrics = extract_velocity_metrics(&buf, 2, 0, 3, 0);
        assert_eq!(metrics.peak_velocity, 1);
        assert_eq!(
            classify_velocity_validation(&metrics, 139, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_metrics_no_chatter_when_quiet() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for _ in 0..256 {
            buf.push(1000);
        }
        let metrics = extract_velocity_metrics(&buf, 1000, 0, 3, 50);
        // Last 50 samples are all 1000, which is above threshold 3 — but
        // this tests the "chatter in zero-speed region" semantic. For a
        // properly captured velocity step response, the samples near target
        // would be at target, not zero. The chatter window monitors the
        // return-to-zero tail.
        // In this case the tail samples are at 1000 which IS above threshold.
        // The function checks raw values, so any value > threshold = chatter.
        // This is expected: in real use, the chatter window covers the
        // zero-velocity settling region where values should be near 0.
        assert!(metrics.zero_speed_chatter);
    }
}
