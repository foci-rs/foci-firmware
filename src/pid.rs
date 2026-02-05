//! PID gain computation for TMC4671 inner current loops.
//!
//! Computes flux and torque PI gains from measured motor resistance
//! and inductance. All math is integer-only (no_std compatible).

/// Validated motor parameters from R/L measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorParams {
    /// Winding resistance in milliohms (average of both coils).
    pub resistance_mohm: u32,
    /// Winding inductance in microhenries (average of both coils).
    pub inductance_uh: u32,
}

/// Validation error for motor parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorParamError {
    /// Resistance is zero (open circuit or measurement failure).
    ZeroResistance,
    /// Resistance exceeds 50 ohms (50_000 milliohms).
    ResistanceTooHigh,
    /// Inductance is zero (no inductance or measurement failure).
    ZeroInductance,
    /// Inductance exceeds 50 mH (50_000 microhenries).
    InductanceTooHigh,
    /// Coil A and B resistance differ by more than 50%.
    ResistanceAsymmetry,
    /// Coil A and B inductance differ by more than 50%.
    InductanceAsymmetry,
}

/// Raw step-response measurement data in TMC4671 internal units.
///
/// These values encode the board-specific ADC and voltage scaling
/// implicitly — the PI controller, ADC, and voltage output all operate
/// in the same s16 space, so gain computation from these values needs
/// no board-specific constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawMeasurement {
    /// Voltage step applied via UQ_UD_EXT (PIDOUT units).
    pub voltage: u16,
    /// Steady-state current response, coil A (ADC units, unsigned abs).
    pub current_a: u16,
    /// Steady-state current response, coil B (ADC units, unsigned abs).
    pub current_b: u16,
    /// Time constant tau for coil A in sample periods (1ms each).
    pub tau_samples_a: u16,
    /// Time constant tau for coil B in sample periods (1ms each).
    pub tau_samples_b: u16,
}

/// Computed PI gains for TMC4671 flux and torque controllers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InnerLoopGains {
    /// Flux P gain — raw TMC4671 register value (Q8.8 format).
    pub flux_p: u16,
    /// Flux I gain — raw TMC4671 register value (Q8.8 format, advanced mode).
    pub flux_i: u16,
    /// Torque P gain — raw TMC4671 register value (Q8.8 format).
    pub torque_p: u16,
    /// Torque I gain — raw TMC4671 register value (Q8.8 format, advanced mode).
    pub torque_i: u16,
}

/// TMC4671 PWM frequency derived from clock and PWM_MAXCNT.
/// f_pwm = tmc_clock / (pwm_maxcnt + 1)
/// OpenFFBoard: 25_000_000 / 4000 = 6250 Hz
pub const DEFAULT_PWM_FREQ_HZ: u32 = 6250;

/// Target closed-loop bandwidth as fraction of PWM frequency.
/// 1/10th is a conservative, stable choice.
const BANDWIDTH_DIVISOR: u32 = 10;

/// Conservative default gains for when no R/L data is available.
/// These are intentionally low — stable but sluggish.
pub const DEFAULT_INNER_GAINS: InnerLoopGains = InnerLoopGains {
    flux_p: 256,   // 1.0 in Q8.8
    flux_i: 26,    // ~0.1 in Q8.8 (matches firmware hardcoded value)
    torque_p: 256, // 1.0 in Q8.8
    torque_i: 26,  // ~0.1 in Q8.8 (matches firmware hardcoded value)
};

/// Validate and average per-coil R/L measurements.
///
/// # Parameters
/// - `r_a`, `r_b`: per-coil resistance in milliohms
/// - `l_a`, `l_b`: per-coil inductance in microhenries
///
/// # Returns
/// Averaged `MotorParams` or validation error.
pub fn validate_motor_params(
    r_a: u16,
    r_b: u16,
    l_a: u16,
    l_b: u16,
) -> Result<MotorParams, MotorParamError> {
    if r_a == 0 || r_b == 0 {
        return Err(MotorParamError::ZeroResistance);
    }
    if r_a > 50_000 || r_b > 50_000 {
        return Err(MotorParamError::ResistanceTooHigh);
    }
    if l_a == 0 || l_b == 0 {
        return Err(MotorParamError::ZeroInductance);
    }
    if l_a > 50_000 || l_b > 50_000 {
        return Err(MotorParamError::InductanceTooHigh);
    }

    // Check asymmetry: |a - b| > max(a, b) / 2
    let r_max = r_a.max(r_b) as u32;
    let r_diff = (r_a as i32 - r_b as i32).unsigned_abs();
    if r_diff > r_max / 2 {
        return Err(MotorParamError::ResistanceAsymmetry);
    }

    let l_max = l_a.max(l_b) as u32;
    let l_diff = (l_a as i32 - l_b as i32).unsigned_abs();
    if l_diff > l_max / 2 {
        return Err(MotorParamError::InductanceAsymmetry);
    }

    Ok(MotorParams {
        resistance_mohm: (r_a as u32 + r_b as u32) / 2,
        inductance_uh: (l_a as u32 + l_b as u32) / 2,
    })
}

/// Compute inner loop PI gains from motor parameters.
///
/// Uses standard PI tuning for RL load:
///   P = L * bandwidth (proportional to inductance)
///   I = R * bandwidth (proportional to resistance)
///
/// Bandwidth = f_pwm / BANDWIDTH_DIVISOR (default: 625 Hz = 3927 rad/s)
///
/// The TMC4671 PI controller in advanced mode (MODE_PID_TYPE=1) uses
/// Q8.8 format for both P and I gains (register value / 256).
/// Section 4.7.6 describes Q0.15 for I in standard mode, but the
/// firmware uses advanced mode with default ADVANCED_PI_REPRESENT
/// (sub-register 62), where all gains are Q8.8.
///
/// # Parameters
/// - `params`: validated motor parameters (R in milliohms, L in microhenries)
/// - `pwm_freq_hz`: TMC4671 PWM frequency (default 6250 Hz for OpenFFBoard)
///
/// # Panics (debug only)
/// Debug-asserts that `params` fields are within `validate_motor_params` bounds
/// and `pwm_freq_hz > 0`. In release builds, out-of-range inputs may produce
/// incorrect results due to integer overflow.
pub fn compute_inner_gains(params: &MotorParams, pwm_freq_hz: u32) -> InnerLoopGains {
    debug_assert!(pwm_freq_hz > 0, "PWM frequency must be non-zero");
    debug_assert!(
        params.resistance_mohm <= 50_000,
        "resistance out of validated range"
    );
    debug_assert!(
        params.inductance_uh <= 50_000,
        "inductance out of validated range"
    );

    // Target bandwidth in Hz
    let bw_hz = pwm_freq_hz / BANDWIDTH_DIVISOR;

    // P gain = L * 2 * pi * bw_hz
    // Units: L is in uH, bw in Hz
    // P_real = L_uH * 1e-6 * 2 * pi * bw_hz
    // P_q8_8 = P_real * 256
    //
    // To keep integer math: P_q8_8 = L_uH * bw_hz * 256 * 2 * pi / 1_000_000
    // Approximate 2*pi as 6283/1000:
    // P_q8_8 = L_uH * bw_hz * 256 * 6283 / 1_000_000_000
    //
    // To avoid overflow with u64:
    let p_num = params.inductance_uh as u64 * bw_hz as u64 * 256 * 6283;
    let p_raw = (p_num / 1_000_000_000) as u32;
    let flux_p = p_raw.min(u16::MAX as u32) as u16;

    // I gain = R * 2 * pi * bw_hz
    // Units: R is in milliohms, bw in Hz
    // I_real = R_mohm * 1e-3 * 2 * pi * bw_hz
    // I_q8_8 = I_real * 256
    //
    // In advanced PID mode (MODE_PID_TYPE=1) with default
    // ADVANCED_PI_REPRESENT, I gains are Q8.8 (not Q0.15).
    // I_q8_8 = R_mohm * bw_hz * 256 * 6283 / 1_000_000_000
    let i_num = params.resistance_mohm as u64 * bw_hz as u64 * 256 * 6283;
    let i_raw = (i_num / 1_000_000_000) as u32;
    let flux_i = i_raw.min(u16::MAX as u32) as u16;

    // Torque gains are identical to flux for 2-phase stepper (FOC2)
    InnerLoopGains {
        flux_p,
        flux_i,
        torque_p: flux_p,
        torque_i: flux_i,
    }
}

/// Compute inner loop PI gains from raw step-response measurement.
///
/// Uses the TMC4671's internal-unit R/L values directly, avoiding
/// board-specific scaling factors. The advanced PI controller
/// (MODE_PID_TYPE=1, Table 22) with P 8.8 / I 8.8 representation:
///
/// ```text
/// output = (P_reg/256) * error + integrator
/// integrator += (P_reg/256 * I_reg/256 * error) / 256
/// ```
///
/// The I parameter acts as a ratio relative to P (inverse integral
/// time constant), not an independent integrator gain.
///
/// # Strategy
///
/// P gain scales linearly with internal inductance (L_int = V/I * tau).
/// Higher inductance requires more voltage per unit current change rate.
///
/// I gain is a conservative fraction — slow integrator to eliminate
/// steady-state error without aggressive dynamics. Scaled with R/L
/// ratio (the plant's natural frequency).
///
/// Reference calibration point: OpenFFBoard v1.2.4, test motor
/// (R~1.2 ohm, L~1.5 mH), working gains P=404, I=26,
/// raw measurement V=512, I=455, tau=1.
pub fn compute_inner_gains_from_raw(raw: &RawMeasurement) -> InnerLoopGains {
    // Average the two coils (both should be similar after validation)
    let current_avg = (raw.current_a as u32 + raw.current_b as u32) / 2;
    let tau_avg = (raw.tau_samples_a as u32 + raw.tau_samples_b as u32) / 2;

    if current_avg == 0 || tau_avg == 0 {
        return DEFAULT_INNER_GAINS;
    }

    // Internal-unit R and L:
    //   R_int = voltage / current  (dimensionless ratio in s16 space)
    //   L_int = R_int * tau        (ratio * sample periods)
    //
    // Reference point (OpenFFBoard test motor):
    //   V=512, I=455, tau=1 → R_int_ref = 512/455 ≈ 1.125, L_int_ref ≈ 1.125
    //   Working gains: P=404, I=26
    //
    // Scaling:
    //   P_new = P_ref * L_int_new / L_int_ref
    //         = 404 * (V_new/I_new * tau_new) / (512/455 * 1)
    //         = 404 * V_new * tau_new * 455 / (I_new * 512)
    //
    //   I_new = I_ref * (R_int_new/L_int_new) / (R_int_ref/L_int_ref)
    //         = I_ref * (1/tau_new) / (1/tau_ref)
    //         = I_ref * tau_ref / tau_new
    //         = 26 * 1 / tau_new
    //         = 26 / tau_new

    // P = 404 * voltage * tau_avg * 455 / (current_avg * 512)
    let p_num = 404u64 * raw.voltage as u64 * tau_avg as u64 * 455;
    let p_den = current_avg as u64 * 512;
    let p_raw = (p_num / p_den) as u32;
    let flux_p = p_raw.min(u16::MAX as u32) as u16;

    // I = 26 / tau_avg (tau_ref = 1 for reference motor)
    // For tau=1 (fast rise, low inductance motor): I=26 (matches reference)
    // For tau=5 (slower rise, higher inductance): I=5 (slower integrator)
    let flux_i = (26u32 / tau_avg).max(1).min(u16::MAX as u32) as u16;

    // Torque gains identical to flux for 2-phase stepper (FOC2)
    InnerLoopGains {
        flux_p,
        flux_i,
        torque_p: flux_p,
        torque_i: flux_i,
    }
}

/// Pack P and I gains into a TMC4671 PID register value.
///
/// Register layout (datasheet section 7.2, page 96):
///   bits[15:0] = I gain
///   bits[31:16] = P gain
///
/// Note: the datasheet types these fields as s16, but PI gains are
/// always non-negative. This function treats them as u16.
pub fn pack_pid_register(p: u16, i: u16) -> u32 {
    ((p as u32) << 16) | (i as u32)
}

/// Unpack P and I gains from a TMC4671 PID register value.
///
/// Returns (p_gain, i_gain).
pub fn unpack_pid_register(reg: u32) -> (u16, u16) {
    let p = (reg >> 16) as u16;
    let i = (reg & 0xFFFF) as u16;
    (p, i)
}

impl InnerLoopGains {
    /// Pack gains into TMC4671 register values.
    ///
    /// Returns (flux_register, torque_register) ready for
    /// `PID_FLUX_P_FLUX_I` (0x54) and `PID_TORQUE_P_TORQUE_I` (0x56).
    pub fn to_registers(&self) -> (u32, u32) {
        let flux = pack_pid_register(self.flux_p, self.flux_i);
        let torque = pack_pid_register(self.torque_p, self.torque_i);
        (flux, torque)
    }

    /// Unpack gains from TMC4671 register values.
    pub fn from_registers(flux_reg: u32, torque_reg: u32) -> Self {
        let (flux_p, flux_i) = unpack_pid_register(flux_reg);
        let (torque_p, torque_i) = unpack_pid_register(torque_reg);
        Self {
            flux_p,
            flux_i,
            torque_p,
            torque_i,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validate_typical_stepper() {
        // 17HS19-2004S1: ~1.2 ohm, ~1.5 mH per coil
        let result = validate_motor_params(1200, 1210, 1500, 1480);
        assert!(result.is_ok());
        let params = result.unwrap();
        assert_eq!(params.resistance_mohm, 1205);
        assert_eq!(params.inductance_uh, 1490);
    }

    #[test]
    fn reject_zero_resistance() {
        let result = validate_motor_params(0, 1200, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ZeroResistance));
    }

    #[test]
    fn reject_zero_inductance() {
        let result = validate_motor_params(1200, 1200, 0, 1500);
        assert_eq!(result, Err(MotorParamError::ZeroInductance));
    }

    #[test]
    fn reject_zero_resistance_coil_b() {
        let result = validate_motor_params(1200, 0, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ZeroResistance));
    }

    #[test]
    fn reject_zero_inductance_coil_b() {
        let result = validate_motor_params(1200, 1200, 1500, 0);
        assert_eq!(result, Err(MotorParamError::ZeroInductance));
    }

    #[test]
    fn reject_high_resistance() {
        let result = validate_motor_params(65535, 65535, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ResistanceTooHigh));
    }

    #[test]
    fn reject_high_inductance() {
        let result = validate_motor_params(1200, 1200, 65535, 65535);
        assert_eq!(result, Err(MotorParamError::InductanceTooHigh));
    }

    #[test]
    fn reject_resistance_asymmetry() {
        // 1200 vs 500 — differ by >50% of max
        let result = validate_motor_params(1200, 500, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ResistanceAsymmetry));
    }

    #[test]
    fn reject_inductance_asymmetry() {
        let result = validate_motor_params(1200, 1200, 1500, 600);
        assert_eq!(result, Err(MotorParamError::InductanceAsymmetry));
    }

    #[test]
    fn accept_moderate_asymmetry() {
        // 1200 vs 900 — differ by 25%, within 50% threshold
        let result = validate_motor_params(1200, 900, 1500, 1200);
        assert!(result.is_ok());
    }

    #[test]
    fn compute_gains_typical_stepper() {
        // 17HS19-2004S1: R=1200 mOhm, L=1500 uH
        let params = MotorParams {
            resistance_mohm: 1200,
            inductance_uh: 1500,
        };
        let gains = compute_inner_gains(&params, 6250);

        // P = L * 2*pi*bw * 256 (Q8.8)
        // bw = 625 Hz
        // P = 1500e-6 * 2*pi*625 * 256 = 1500 * 625 * 256 * 6283 / 1e9
        // = 1,508,304,000,000 / 1e9 = 1508 (approx)
        assert!(
            gains.flux_p > 1000 && gains.flux_p < 2000,
            "flux_p={} expected ~1508",
            gains.flux_p
        );

        // I = R * 2*pi*bw * 256 (Q8.8, advanced mode)
        // I = 1200e-3 * 2*pi*625 * 256 = 1200 * 625 * 256 * 6283 / 1e9
        // = 1,205,568,000,000 / 1e9 = ~1205
        assert!(
            gains.flux_i > 900 && gains.flux_i < 1500,
            "flux_i={} expected ~1205",
            gains.flux_i
        );

        // Torque == flux for FOC2
        assert_eq!(gains.flux_p, gains.torque_p);
        assert_eq!(gains.flux_i, gains.torque_i);
    }

    #[test]
    fn compute_gains_low_rl() {
        // Very low R/L motor
        let params = MotorParams {
            resistance_mohm: 100,
            inductance_uh: 100,
        };
        let gains = compute_inner_gains(&params, 6250);
        assert!(
            gains.flux_p > 0,
            "P gain should be non-zero for any valid motor"
        );
        assert!(
            gains.flux_i > 0,
            "I gain should be non-zero for any valid motor"
        );
    }

    #[test]
    fn compute_gains_high_rl() {
        // High R/L motor (large stepper)
        let params = MotorParams {
            resistance_mohm: 10000,
            inductance_uh: 20000,
        };
        let gains = compute_inner_gains(&params, 6250);
        // Should clamp to u16::MAX, not overflow
        assert!(gains.flux_p <= u16::MAX);
        assert!(gains.flux_i <= u16::MAX);
    }

    #[test]
    fn compute_gains_different_pwm_freq() {
        let params = MotorParams {
            resistance_mohm: 1200,
            inductance_uh: 1500,
        };
        let gains_6250 = compute_inner_gains(&params, 6250);
        let gains_12500 = compute_inner_gains(&params, 12500);

        // Higher PWM freq = higher bandwidth = higher gains
        assert!(gains_12500.flux_p > gains_6250.flux_p);
    }

    #[test]
    fn default_gains_are_valid() {
        // Defaults should be reasonable non-zero values
        assert!(DEFAULT_INNER_GAINS.flux_p > 0);
        assert!(DEFAULT_INNER_GAINS.flux_i > 0);
        assert!(DEFAULT_INNER_GAINS.torque_p > 0);
        assert!(DEFAULT_INNER_GAINS.torque_i > 0);
    }

    #[test]
    fn raw_gains_reference_motor() {
        // OpenFFBoard test motor: V=512, I=455, tau=1
        // Known-working gains: P=404, I=26
        let raw = RawMeasurement {
            voltage: 512,
            current_a: 455,
            current_b: 455,
            tau_samples_a: 1,
            tau_samples_b: 1,
        };
        let gains = compute_inner_gains_from_raw(&raw);

        // Should produce gains close to the known-working values
        assert!(
            gains.flux_p > 350 && gains.flux_p < 460,
            "flux_p={} expected ~404",
            gains.flux_p
        );
        assert_eq!(gains.flux_i, 26, "flux_i should match reference I=26");
        assert_eq!(gains.flux_p, gains.torque_p);
        assert_eq!(gains.flux_i, gains.torque_i);
    }

    #[test]
    fn raw_gains_higher_inductance() {
        // Same current but longer tau = higher inductance → higher P
        let raw = RawMeasurement {
            voltage: 512,
            current_a: 455,
            current_b: 455,
            tau_samples_a: 3,
            tau_samples_b: 3,
        };
        let gains = compute_inner_gains_from_raw(&raw);

        // P should scale with tau (3x inductance → ~3x P)
        assert!(
            gains.flux_p > 1000 && gains.flux_p < 1400,
            "flux_p={} expected ~1212 (404*3)",
            gains.flux_p
        );
        // I should decrease with tau (slower integrator for higher inductance)
        assert!(
            gains.flux_i < 26,
            "flux_i={} should be less than reference 26",
            gains.flux_i
        );
    }

    #[test]
    fn raw_gains_higher_resistance() {
        // Higher R = lower current for same voltage → higher R_int
        let raw = RawMeasurement {
            voltage: 512,
            current_a: 200,
            current_b: 200,
            tau_samples_a: 1,
            tau_samples_b: 1,
        };
        let gains = compute_inner_gains_from_raw(&raw);

        // P scales with V/I ratio (higher R → higher P for same tau)
        assert!(
            gains.flux_p > gains.flux_i,
            "P should be larger than I for any real motor"
        );
        // I stays at 26 since tau=1 (same as reference)
        assert_eq!(gains.flux_i, 26);
    }

    #[test]
    fn raw_gains_zero_current_returns_defaults() {
        let raw = RawMeasurement {
            voltage: 512,
            current_a: 0,
            current_b: 0,
            tau_samples_a: 1,
            tau_samples_b: 1,
        };
        let gains = compute_inner_gains_from_raw(&raw);
        assert_eq!(gains, DEFAULT_INNER_GAINS);
    }

    #[test]
    fn raw_gains_zero_tau_returns_defaults() {
        let raw = RawMeasurement {
            voltage: 512,
            current_a: 455,
            current_b: 455,
            tau_samples_a: 0,
            tau_samples_b: 0,
        };
        let gains = compute_inner_gains_from_raw(&raw);
        assert_eq!(gains, DEFAULT_INNER_GAINS);
    }

    #[test]
    fn pack_unpack_roundtrip() {
        let gains = InnerLoopGains {
            flux_p: 1508,
            flux_i: 32768,
            torque_p: 1508,
            torque_i: 32768,
        };
        let (flux_reg, torque_reg) = gains.to_registers();
        let restored = InnerLoopGains::from_registers(flux_reg, torque_reg);
        assert_eq!(gains, restored);
    }

    #[test]
    fn pack_register_layout() {
        // P=0x1234 in bits[31:16], I=0x5678 in bits[15:0]
        let reg = pack_pid_register(0x1234, 0x5678);
        assert_eq!(reg, 0x12345678);
    }

    #[test]
    fn unpack_register_layout() {
        let (p, i) = unpack_pid_register(0x12345678);
        assert_eq!(p, 0x1234);
        assert_eq!(i, 0x5678);
    }
}
