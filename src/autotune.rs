//! Shared types and analysis helpers for the FOCI commissioning workflow.
//!
//! This module provides:
//! - [`TransientBuffer`]: Fixed-size ring buffer for step-response capture
//! - [`CommissionProfile`]: Tuning aggressiveness selection
//! - [`OperatingMode`]: Load condition selection
//! - [`CommissionError`]: Error variants for commissioning phase failures
//! - [`CommissionStatus`]: Overall commissioning outcome
//! - [`PhaseId`]: Identifies each commissioning phase for progress reporting
//!
//! All types are `no_std` compatible. No heap allocation is used.

/// Number of samples in the step-response transient capture buffer.
///
/// Sized to fit one complete transient capture (256 × 4 bytes = 1,024 bytes).
/// Only one capture is active at a time; the buffer is reused across phases.
pub const TRANSIENT_LEN: usize = 256;

/// Fixed-size ring buffer for raw step-response capture.
///
/// Statically allocated and reused across commissioning phases. After each
/// capture phase completes, scalar metrics are extracted and the buffer is
/// reset for the next phase. Raw transient streaming is deferred to Phase 6.
///
/// Memory: 1,024 bytes for samples, plus 4 bytes of metadata.
#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransientBuffer {
    samples: [u32; TRANSIENT_LEN],
    /// Number of samples captured so far (always <= `TRANSIENT_LEN`).
    len: u16,
    /// Microseconds between consecutive samples.
    sample_period_us: u16,
}

impl Default for TransientBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl TransientBuffer {
    /// Create an empty buffer.
    ///
    /// Sample period defaults to 0. Call [`reset`](Self::reset) before
    /// each capture to set the period for that measurement.
    pub const fn new() -> Self {
        Self {
            samples: [0u32; TRANSIENT_LEN],
            len: 0,
            sample_period_us: 0,
        }
    }

    /// Reset the buffer for a new capture, optionally changing the sample period.
    ///
    /// # Parameters
    /// - `sample_period_us`: microseconds between consecutive samples for the
    ///   new capture
    pub fn reset(&mut self, sample_period_us: u16) {
        self.len = 0;
        self.sample_period_us = sample_period_us;
    }

    /// Append one sample to the buffer.
    ///
    /// Returns `true` if the sample was stored. Returns `false` if the buffer
    /// is already full; the sample is discarded.
    ///
    /// # Parameters
    /// - `value`: raw ADC or encoder count to store
    pub fn push(&mut self, value: u32) -> bool {
        let idx = self.len as usize;
        if idx >= TRANSIENT_LEN {
            return false;
        }
        self.samples[idx] = value;
        self.len += 1;
        true
    }

    /// Number of samples currently stored.
    pub fn len(&self) -> u16 {
        self.len
    }

    /// Returns `true` if no samples have been captured yet.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` if the buffer holds `TRANSIENT_LEN` samples.
    pub fn is_full(&self) -> bool {
        self.len as usize >= TRANSIENT_LEN
    }

    /// Microseconds between consecutive samples.
    pub fn sample_period_us(&self) -> u16 {
        self.sample_period_us
    }

    /// Slice of the captured samples (length may be less than `TRANSIENT_LEN`).
    pub fn samples(&self) -> &[u32] {
        &self.samples[..self.len as usize]
    }
}

impl core::fmt::Debug for TransientBuffer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TransientBuffer")
            .field("len", &self.len)
            .field("sample_period_us", &self.sample_period_us)
            .finish()
    }
}

/// Tuning aggressiveness profile for SIMC gain synthesis.
///
/// Controls the closed-loop time constant lambda in the SIMC PI tuning law.
/// A more aggressive profile yields faster response but requires a more
/// accurate plant model and leaves less stability margin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommissionProfile {
    /// `lambda = 4 * theta` — slow, very robust, minimal audible noise.
    Conservative,
    /// `lambda = 2 * theta` — good default for most motors and print setups.
    Balanced,
    /// `lambda = max(theta, tau/4)` — fastest safe response; may be audible.
    Stiff,
}

impl CommissionProfile {
    /// Decode from the wire representation used in the Klipper command.
    ///
    /// | Value | Profile |
    /// |-------|---------|
    /// | 0 | `Conservative` |
    /// | 1 (default) | `Balanced` |
    /// | 2 | `Stiff` |
    ///
    /// Unknown values map to `Balanced`.
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Conservative,
            1 => Self::Balanced,
            2 => Self::Stiff,
            _ => Self::Balanced,
        }
    }

    /// Encode to the wire representation.
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Conservative => 0,
            Self::Balanced => 1,
            Self::Stiff => 2,
        }
    }
}

/// Load condition present during mechanical identification.
///
/// Adjusts velocity-loop lambda after SIMC synthesis to account for the
/// expected operating environment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OperatingMode {
    /// Motor running without a significant load (e.g., belt axis with no
    /// toolhead). `lambda_vel *= 1.5` — reduces standstill buzz.
    Unloaded,
    /// Typical printing load; no lambda adjustment applied.
    Nominal,
    /// Heavy toolhead or high-inertia carriage. `lambda_vel *= 0.8` — tighter
    /// tuning; natural damping from the load helps stability.
    HighInertia,
}

impl OperatingMode {
    /// Decode from the wire representation used in the Klipper command.
    ///
    /// | Value | Mode |
    /// |-------|------|
    /// | 0 (default) | `Unloaded` |
    /// | 1 | `Nominal` |
    /// | 2 | `HighInertia` |
    ///
    /// Unknown values map to `Unloaded`.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Nominal,
            2 => Self::HighInertia,
            _ => Self::Unloaded,
        }
    }

    /// Encode to the wire representation.
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Unloaded => 0,
            Self::Nominal => 1,
            Self::HighInertia => 2,
        }
    }
}

/// Error variants for commissioning phase failures.
///
/// Each variant maps to a unique non-zero `status_code()` used in the
/// `foci_commission_phase` Klipper message. The Python module converts
/// the code back to a human-readable string for display.
///
/// Codes are stable across firmware versions. Adding new variants must
/// use new codes; existing codes must not be renumbered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommissionError {
    /// Motor was already enabled when commissioning was requested. Code 1.
    MotorEnabled,
    /// No current was detected in the motor windings during the coil check. Code 2.
    NoCurrent,
    /// SPI communication with the TMC4671 failed. Code 3.
    Spi,
    /// ADC calibration produced offsets outside the expected range. Code 4.
    AdcFault,
    /// Coil connectivity check failed (open circuit or short). Code 5.
    CoilFault,
    /// Phase wiring check: current appeared on the wrong ADC channel. Code 6.
    PhaseFault,
    /// Encoder was not detected or direction could not be determined. Code 7.
    EncoderFault,
    /// Electrical identification (R, L) failed to converge. Code 8.
    ElectricalIdFailed,
    /// Current-loop step-response validation failed after retry. Code 9.
    CurrentValidationFailed,
    /// Mechanical identification produced zero or negative inertia. Code 10.
    MechanicalIdFailed,
    /// Velocity-loop step-response validation failed after retry. Code 11.
    VelocityValidationFailed,
    /// Position-loop P ramp-up produced oscillation at minimum gain. Code 12.
    PositionTuneFailed,
    /// Encoder alignment was not confirmed before outer commissioning. Code 13.
    EncoderNotAligned,
    /// Hardware shutdown was requested (M112 / e-stop) while commissioning. Code 14.
    Shutdown,
    /// A commissioning run was already in progress. Code 15.
    AlreadyRunning,
    /// The commissioning command queue was full. Code 16.
    QueueFull,
}

impl CommissionError {
    /// Non-zero status code for the Klipper protocol message.
    ///
    /// Codes are stable across firmware versions. Adding new variants must
    /// use new codes; existing codes must not be renumbered.
    pub fn status_code(self) -> u8 {
        match self {
            Self::MotorEnabled => 1,
            Self::NoCurrent => 2,
            Self::Spi => 3,
            Self::AdcFault => 4,
            Self::CoilFault => 5,
            Self::PhaseFault => 6,
            Self::EncoderFault => 7,
            Self::ElectricalIdFailed => 8,
            Self::CurrentValidationFailed => 9,
            Self::MechanicalIdFailed => 10,
            Self::VelocityValidationFailed => 11,
            Self::PositionTuneFailed => 12,
            Self::EncoderNotAligned => 13,
            Self::Shutdown => 14,
            Self::AlreadyRunning => 15,
            Self::QueueFull => 16,
        }
    }
}

/// Overall outcome of a commissioning run.
///
/// Reported in the `foci_commission_inner_result` or
/// `foci_commission_outer_result` message. The Python module uses this to
/// decide whether to persist the resulting gains.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommissionStatus {
    /// All phases passed; gains are reliable.
    Accepted,
    /// All phases passed but at least one used fallback / conservative gains.
    /// Gains are safe to use; the user should review the warnings.
    AcceptedWithWarnings,
    /// Inner commissioning completed successfully; outer commissioning was not
    /// run (e.g., mechanical ID failed). Current-loop gains are valid.
    InnerOnly,
    /// Commissioning failed; no gains should be persisted.
    Failed(CommissionError),
}

/// Identifies a single commissioning phase for progress reporting.
///
/// Sent in the `foci_commission_phase` message on each phase transition.
/// The Python module translates the `u8` code to a human-readable label.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhaseId {
    /// Inner phase 1 — ADC offset calibration.
    AdcCalibrate,
    /// Inner phase 2 — Coil connectivity check.
    CoilCheck,
    /// Inner phase 3 — Phase wiring validation.
    PhaseWiringCheck,
    /// Inner phase 4 — Encoder presence and direction detection.
    EncoderCheck,
    /// Inner phase 5 — Electrical identification (R, L, theta_e).
    ElectricalId,
    /// Inner phase 6 — Current-loop SIMC gain synthesis.
    CurrentTune,
    /// Inner phase 7 — Current-loop step-response validation.
    CurrentValidate,
    /// Inner commissioning completed.
    InnerDone,
    /// Outer phase 8 — Mechanical identification (J_eff, B_eff, tau_mech).
    MechanicalId,
    /// Outer phase 9 — Velocity-loop SIMC gain synthesis.
    VelocityTune,
    /// Outer phase 10 — Velocity-loop step-response validation.
    VelocityValidate,
    /// Outer phase 11 — Position-loop P ramp-up.
    PositionTune,
    /// Outer phase 12 — Filter selection based on validation metrics.
    FilterTune,
    /// Outer phase 13 — Assemble and transmit final results to host.
    Commit,
    /// Outer commissioning completed.
    OuterDone,
}

impl PhaseId {
    /// Encode to the wire representation used in the Klipper protocol message.
    ///
    /// Values are stable; new variants must use new codes.
    pub fn to_u8(self) -> u8 {
        match self {
            Self::AdcCalibrate => 1,
            Self::CoilCheck => 2,
            Self::PhaseWiringCheck => 3,
            Self::EncoderCheck => 4,
            Self::ElectricalId => 5,
            Self::CurrentTune => 6,
            Self::CurrentValidate => 7,
            Self::InnerDone => 8,
            Self::MechanicalId => 9,
            Self::VelocityTune => 10,
            Self::VelocityValidate => 11,
            Self::PositionTune => 12,
            Self::FilterTune => 13,
            Self::Commit => 14,
            Self::OuterDone => 15,
        }
    }
}

/// First-order-plus-dead-time (FOPDT) current plant model.
///
/// Estimated during electrical identification (Phase 5). Contains
/// both internal-unit values (used for gain computation) and physical
/// values (milliohms, microhenries — for host display and persistence).
///
/// Plant model: `G(s) = K / (tau_e * s + 1) * e^(-theta_e * s)`
/// where `K = 1 / R_int` (DC gain in internal units).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ElectricalModel {
    /// Winding resistance in internal (ADC/DAC) units.
    /// R_int = V_excitation / I_steady_state.
    pub r_int: u32,
    /// Winding inductance in internal units.
    /// L_int = V * dt_s / delta_I.
    pub l_int: u32,
    /// Electrical time constant L/R in microseconds.
    pub tau_e_us: u32,
    /// Effective transport delay in microseconds (from transient onset).
    pub theta_e_us: u32,
    /// Physical resistance in milliohms (for display/persistence).
    pub r_mohm: u32,
    /// Physical inductance in microhenries (for display/persistence).
    pub l_uh: u32,
    /// Tau from two-point transient fit in microseconds.
    /// Compared against L/R tau for plant model validation.
    pub tau_e_crosscheck_us: u32,
}

/// Computed PI gains for the TMC4671 current (flux/torque) loops.
///
/// Produced by [`simc_current_gains`] from an [`ElectricalModel`] and
/// a [`CommissionProfile`]. Includes the effective bandwidth and lambda
/// for diagnostics and downstream use (velocity loop bandwidth constraint).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentGains {
    /// Flux P gain — TMC4671 Q8.8 register value.
    pub flux_p: u16,
    /// Flux I gain — TMC4671 Q8.8 register value.
    pub flux_i: u16,
    /// Torque P gain — TMC4671 Q8.8 register value.
    pub torque_p: u16,
    /// Torque I gain — TMC4671 Q8.8 register value.
    pub torque_i: u16,
    /// Effective closed-loop bandwidth in Hz.
    pub effective_bandwidth_hz: u32,
    /// Closed-loop time constant lambda in microseconds.
    pub lambda_us: u32,
}

/// Metrics from a current-loop step-response validation test.
///
/// Extracted from the [`TransientBuffer`] after a torque step.
/// Used by [`classify_current_validation`] to determine pass/marginal/fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// |rise_fwd - rise_rev| / avg, in 0.1% units.
    /// Set to 0 by extract_step_metrics (requires two captures).
    pub symmetry_pct: u16,
}

/// First-order mechanical plant model estimated during Phase 8.
///
/// Represents `G_mech(s) = 1 / (J_eff * s + B_eff)` where J_eff includes
/// carriage mass, belt compliance, and reflected rotor inertia. Used by
/// Phase 9 (velocity-loop SIMC synthesis) to compute velocity gains.
///
/// All values are in internal units derived from encoder counts and
/// torque DAC values. Physical conversion happens on the host side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MechanicalModel {
    /// Effective inertia in internal units.
    /// J_eff = torque / initial_acceleration (from first few samples).
    pub j_eff: u32,
    /// Effective damping in internal units.
    /// B_eff = torque / steady_state_velocity, or from deceleration curve.
    pub b_eff: u32,
    /// Mechanical time constant J_eff / B_eff in milliseconds.
    pub tau_mech_ms: u32,
    /// Effective mechanical delay in microseconds.
    /// Time from torque step to first measurable velocity change.
    pub theta_mech_us: u32,
    /// Static friction estimate in torque units. 0 if not measured.
    pub t_f: u16,
}

/// Compute the SIMC lambda (closed-loop time constant) for a given profile.
///
/// All profiles guarantee `lambda >= theta_e` (SIMC stability boundary).
fn simc_lambda_us(profile: CommissionProfile, tau_e_us: u32, theta_e_us: u32) -> u32 {
    let lambda = match profile {
        CommissionProfile::Conservative => 4 * theta_e_us,
        CommissionProfile::Balanced => 2 * theta_e_us,
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
    if model.r_int == 0 || model.l_int == 0 || model.theta_e_us == 0 {
        return None;
    }

    let lambda_us = simc_lambda_us(profile, model.tau_e_us, model.theta_e_us);

    // SIMC PI tuning law for FOPDT plant G(s) = K / (tau*s+1) * e^(-theta*s):
    //   Kp = tau / (K * (lambda + theta))
    //   Ti = min(tau, 4 * (lambda + theta))
    //   Ki = Kp / Ti
    //
    // For current loop: K = 1/R_int (DC gain: current per unit voltage).
    // All in internal units — no board-specific scaling needed.

    let tau = model.tau_e_us as f32;
    let theta = model.theta_e_us as f32;
    let lambda = lambda_us as f32;
    let r_int = model.r_int as f32;
    let k = 1.0 / r_int; // Plant DC gain

    let kp = tau / (k * (lambda + theta));
    let ti = {
        let candidate = 4.0 * (lambda + theta);
        if tau < candidate {
            tau
        } else {
            candidate
        }
    };
    let ti_s = ti * 1e-6; // Convert Ti from microseconds to seconds
    let ki = kp / ti_s;

    // Convert to TMC4671 Q8.8 register format
    let f_pwm = config.pwm_freq_hz() as f32;
    let p_reg = (kp * 256.0).clamp(1.0, u16::MAX as f32) as u16;
    let i_reg = (ki / f_pwm * 256.0).clamp(1.0, u16::MAX as f32) as u16;

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
/// (forward + reverse); the caller computes it via [`compute_symmetry_pct`].
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
        };
    }

    let period = buf.sample_period_us() as u32;
    let target_f = target as f32;
    let threshold_10 = (target_f * 0.10) as u32;
    let threshold_90 = (target_f * 0.90) as u32;
    let settle_band = (target_f * 0.05) as u32;

    let mut t_10: Option<u32> = None;
    let mut t_90: Option<u32> = None;
    let mut peak: u32 = 0;
    let mut peak_idx: usize = 0;
    let mut saturation_count: u16 = 0;

    for (i, &s) in samples.iter().enumerate() {
        if t_10.is_none() && s >= threshold_10 {
            t_10 = Some(i as u32 * period);
        }
        if t_90.is_none() && s >= threshold_90 {
            t_90 = Some(i as u32 * period);
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

    // Settling: last sample where |actual - target| > 5% band
    let mut last_outside: usize = 0;
    for (i, &s) in samples.iter().enumerate() {
        let diff = s.abs_diff(target);
        if diff > settle_band {
            last_outside = i;
        }
    }
    let settling_time_us = if last_outside > 0 {
        (last_outside as u32 + 1) * period
    } else {
        0
    };

    // Ringing: zero-crossings of (actual - target) after peak
    let mut ringing_count: u8 = 0;
    if peak_idx + 1 < samples.len() {
        let mut above = samples[peak_idx] > target;
        for &s in &samples[peak_idx + 1..] {
            let now_above = s > target;
            if now_above != above {
                ringing_count = ringing_count.saturating_add(1);
                above = now_above;
            }
        }
    }

    CurrentValidation {
        rise_time_us,
        overshoot_pct,
        settling_time_us,
        ringing_count,
        saturation_samples: saturation_count,
        symmetry_pct: 0,
    }
}

/// Outcome of a validation threshold check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ValidationResult {
    /// All metrics within normal limits.
    Pass,
    /// One or more metrics in the marginal range — retry with more conservative gains.
    Marginal,
    /// One or more metrics exceed hard limits — fall back to defaults.
    Fail,
}

/// Classify current-loop validation metrics against thresholds.
///
/// Thresholds (starting points, subject to hardware validation):
///
/// | Metric | Pass | Marginal | Fail |
/// |--------|------|----------|------|
/// | Overshoot | < 15% | 15-30% | > 30% |
/// | Ringing | 0-1 | 2-3 | > 3 |
/// | Settling | < 5*tau_e | 5-10*tau_e | > 10*tau_e |
/// | Symmetry | < 20% | 20-40% | > 40% |
/// | Saturation | < 10% | 10-25% | > 25% |
///
/// `total_samples` is the total number of capture samples across both
/// directions, used for computing the saturation percentage.
/// Returns the worst classification across all metrics.
pub fn classify_current_validation(
    metrics: &CurrentValidation,
    tau_e_us: u32,
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

    // Settling (relative to tau_e)
    let settle_5x = 5 * tau_e_us;
    let settle_10x = 10 * tau_e_us;
    if metrics.settling_time_us > settle_10x {
        return ValidationResult::Fail;
    } else if metrics.settling_time_us > settle_5x {
        worst = ValidationResult::Marginal;
    }

    // Symmetry (0.1% units: 200 = 20.0%, 400 = 40.0%)
    if metrics.symmetry_pct > 400 {
        return ValidationResult::Fail;
    } else if metrics.symmetry_pct > 200 {
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

    worst
}

/// Compute symmetry metric from forward and reverse rise times.
///
/// Returns `|a - b| / avg(a, b)` in 0.1% units (200 = 20.0%).
/// Returns 0 if both values are zero.
pub fn compute_symmetry_pct(rise_fwd: u32, rise_rev: u32) -> u16 {
    let avg = (rise_fwd as u64 + rise_rev as u64) / 2;
    if avg == 0 {
        return 0;
    }
    let diff = rise_fwd.abs_diff(rise_rev);
    (diff as u64 * 1000 / avg) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- TransientBuffer ---

    #[test]
    fn transient_buffer_push_and_read() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);
        assert_eq!(buf.sample_period_us(), 160);

        assert!(buf.push(10));
        assert!(buf.push(20));
        assert!(buf.push(30));

        assert!(!buf.is_empty());
        assert_eq!(buf.len(), 3);
        let s = buf.samples();
        assert_eq!(s, &[10u32, 20, 30]);
    }

    #[test]
    fn transient_buffer_full() {
        let mut buf = TransientBuffer::new();
        buf.reset(100);
        for i in 0u32..TRANSIENT_LEN as u32 {
            assert!(buf.push(i), "push {i} should succeed");
        }
        assert!(buf.is_full());
        assert_eq!(buf.len() as usize, TRANSIENT_LEN);

        // One more push must be rejected.
        assert!(!buf.push(9999), "push to full buffer must return false");
        // Length must not change.
        assert_eq!(buf.len() as usize, TRANSIENT_LEN);
    }

    #[test]
    fn transient_buffer_reset_clears() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for i in 0u32..10 {
            buf.push(i);
        }
        assert_eq!(buf.len(), 10);

        buf.reset(200);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);
        assert_eq!(buf.sample_period_us(), 200);
        assert_eq!(buf.samples(), &[] as &[u32]);
    }

    // --- CommissionProfile ---

    #[test]
    fn commission_profile_roundtrip() {
        let profiles = [
            CommissionProfile::Conservative,
            CommissionProfile::Balanced,
            CommissionProfile::Stiff,
        ];
        for p in profiles {
            let code = p.to_u8();
            let decoded = CommissionProfile::from_u8(code);
            assert_eq!(p, decoded, "roundtrip failed for {p:?}");
        }
    }

    #[test]
    fn commission_profile_unknown_maps_to_balanced() {
        assert_eq!(CommissionProfile::from_u8(3), CommissionProfile::Balanced);
        assert_eq!(CommissionProfile::from_u8(255), CommissionProfile::Balanced);
    }

    // --- OperatingMode ---

    #[test]
    fn operating_mode_roundtrip() {
        let modes = [
            OperatingMode::Unloaded,
            OperatingMode::Nominal,
            OperatingMode::HighInertia,
        ];
        for m in modes {
            let code = m.to_u8();
            let decoded = OperatingMode::from_u8(code);
            assert_eq!(m, decoded, "roundtrip failed for {m:?}");
        }
    }

    #[test]
    fn operating_mode_unknown_maps_to_unloaded() {
        assert_eq!(OperatingMode::from_u8(3), OperatingMode::Unloaded);
        assert_eq!(OperatingMode::from_u8(255), OperatingMode::Unloaded);
    }

    // --- CommissionError ---

    #[test]
    fn commission_error_status_codes_unique() {
        let errors = [
            CommissionError::MotorEnabled,
            CommissionError::NoCurrent,
            CommissionError::Spi,
            CommissionError::AdcFault,
            CommissionError::CoilFault,
            CommissionError::PhaseFault,
            CommissionError::EncoderFault,
            CommissionError::ElectricalIdFailed,
            CommissionError::CurrentValidationFailed,
            CommissionError::MechanicalIdFailed,
            CommissionError::VelocityValidationFailed,
            CommissionError::PositionTuneFailed,
            CommissionError::EncoderNotAligned,
            CommissionError::Shutdown,
            CommissionError::AlreadyRunning,
            CommissionError::QueueFull,
        ];
        let mut seen = [false; 256];
        for e in errors {
            let code = e.status_code() as usize;
            assert_ne!(code, 0, "{e:?} must not use code 0 (reserved for success)");
            assert!(!seen[code], "duplicate status_code {code} for {e:?}");
            seen[code] = true;
        }
    }

    // --- PhaseId ---

    #[test]
    fn phase_id_to_u8_unique() {
        let phases = [
            PhaseId::AdcCalibrate,
            PhaseId::CoilCheck,
            PhaseId::PhaseWiringCheck,
            PhaseId::EncoderCheck,
            PhaseId::ElectricalId,
            PhaseId::CurrentTune,
            PhaseId::CurrentValidate,
            PhaseId::InnerDone,
            PhaseId::MechanicalId,
            PhaseId::VelocityTune,
            PhaseId::VelocityValidate,
            PhaseId::PositionTune,
            PhaseId::FilterTune,
            PhaseId::Commit,
            PhaseId::OuterDone,
        ];
        let mut seen = [false; 256];
        for p in phases {
            let code = p.to_u8() as usize;
            assert_ne!(code, 0, "{p:?} must not use code 0");
            assert!(!seen[code], "duplicate to_u8 code {code} for {p:?}");
            seen[code] = true;
        }
    }

    // --- ElectricalModel ---

    #[test]
    fn electrical_model_tau_from_l_over_r() {
        let model = ElectricalModel {
            r_int: 512,
            l_int: 256,
            tau_e_us: 50,
            theta_e_us: 10,
            r_mohm: 1125,
            l_uh: 450,
            tau_e_crosscheck_us: 48,
        };
        assert_eq!(model.r_int, 512);
        assert_eq!(model.tau_e_us, 50);
        assert_eq!(model.theta_e_us, 10);
    }

    // --- SIMC current gains ---

    fn autotune_test_config() -> crate::config::TmcConfig {
        crate::config::TmcConfig {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec: 331,
            excitation_voltage: 512,
            didt_cycles: 5000,
            bandwidth_hz: 150.0,
        }
    }

    fn test_electrical_model() -> ElectricalModel {
        ElectricalModel {
            r_int: 1125,
            l_int: 2950,
            tau_e_us: 2622,
            theta_e_us: 700,
            r_mohm: 1125,
            l_uh: 450,
            tau_e_crosscheck_us: 2500,
        }
    }

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
    fn simc_tau_dominated_plant_stiff_above_balanced() {
        // Tau-dominated: tau >> theta, so tau/4 dominates the Stiff formula.
        // Stiff = max(theta, tau/4) = max(100, 1000) = 1000
        // Balanced = 2*theta = 200
        // So stiff > balanced here.
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
        let balanced = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        let stiff = simc_current_gains(&model, CommissionProfile::Stiff, &config);
        assert!(
            stiff.lambda_us > balanced.lambda_us,
            "tau-dominated: stiff lambda {} should be > balanced {}",
            stiff.lambda_us,
            balanced.lambda_us
        );
    }

    #[test]
    fn simc_invariants_hold_for_all_profiles() {
        // Universal SIMC invariants that hold regardless of plant shape:
        // 1. conservative >= balanced (4*theta >= 2*theta always)
        // 2. stiff >= theta (stability floor)
        // 3. stiff >= tau/4 (stiff formula is max(theta, tau/4))
        // 4. all profiles >= theta (SIMC stability boundary)
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
    fn step_metrics_empty_buffer() {
        let buf = TransientBuffer::new();
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.rise_time_us, 0);
        assert_eq!(metrics.overshoot_pct, 0);
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
        let lambda_s = (2.0 * model.theta_e_us as f64) * 1e-6; // Balanced = 2*theta
        let r_int = model.r_int as f64;
        let k = 1.0 / r_int;
        let f_pwm = config.pwm_freq_hz() as f64;

        let kp_si = tau_s / (k * (lambda_s + theta_s));
        let ti_s = {
            let candidate = 4.0 * (lambda_s + theta_s);
            if tau_s < candidate {
                tau_s
            } else {
                candidate
            }
        };
        let ki_si = kp_si / ti_s; // 1/second — no conversion needed
        let p_reg_expected = (kp_si * 256.0).clamp(1.0, u16::MAX as f64) as u16;
        let i_reg_expected = (ki_si / f_pwm * 256.0).clamp(1.0, u16::MAX as f64) as u16;

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
    fn classify_pass() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 100, // 10.0%
            settling_time_us: 2000,
            ringing_count: 1,
            saturation_samples: 5,
            symmetry_pct: 100, // 10.0%
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
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Marginal
        );
    }

    // --- Symmetry ---

    #[test]
    fn compute_symmetry_identical() {
        assert_eq!(compute_symmetry_pct(500, 500), 0);
    }

    #[test]
    fn compute_symmetry_20_percent() {
        // |600 - 500| / 550 ≈ 18.2% → 181 in 0.1% units
        let result = compute_symmetry_pct(600, 500);
        assert!(
            result >= 170 && result <= 190,
            "symmetry {result} expected ~181"
        );
    }

    #[test]
    fn compute_symmetry_zero_avg() {
        assert_eq!(compute_symmetry_pct(0, 0), 0);
    }
}
