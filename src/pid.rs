//! PID gain computation for TMC4671 inner current loops.
//!
//! Computes flux and torque PI gains from di/dt measurement in the
//! TMC4671's internal unit space. Uses pole-zero cancellation with
//! a 50 Hz bandwidth validated on hardware.

/// Motor R and L measured in TMC4671 internal units.
///
/// These values are dimensionless ratios in the TMC4671's s16 ADC/DAC
/// space. Because the PI controller, ADC, and voltage output all
/// operate in this same space, gain computation from internal-unit
/// values needs no board-specific scaling constants.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InternalMeasurement {
    /// Voltage applied during measurement (PIDOUT units).
    pub voltage: u16,
    /// Steady-state current per coil (ADC units, unsigned absolute).
    /// Average of coil A and coil B.
    pub current_avg: u16,
    /// Average |delta_I| per cycle during square-wave excitation
    /// (ADC units). Averaged over both coils.
    pub delta_i_avg: u16,
    /// Actual time per di/dt cycle in microseconds, measured via DWT.
    /// Includes SPI transfer overhead (~50us write + ~25us read).
    pub dt_us: u32,
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

/// Conservative default gains for when no R/L data is available.
/// These are intentionally low — stable but sluggish.
pub const DEFAULT_INNER_GAINS: InnerLoopGains = InnerLoopGains {
    flux_p: 256,   // 1.0 in Q8.8
    flux_i: 26,    // ~0.1 in Q8.8 (matches firmware hardcoded value)
    torque_p: 256, // 1.0 in Q8.8
    torque_i: 26,  // ~0.1 in Q8.8 (matches firmware hardcoded value)
};

/// Compute PI gains from internal-unit measurements.
///
/// Uses the standard pole-zero cancellation formula directly in the
/// TMC4671's internal unit space:
///
///   R_int = voltage / current_avg
///   L_int = voltage / (delta_i_avg * f_pwm)
///   P = L_int * omega_bw * 256         (Q8.8)
///
/// The f_pwm terms cancel in the P formula:
///   P = V / (dI * f) * 2*pi*f/20 * 256 = V * pi / (dI * 10) * 256
///
/// Because all quantities are in the same internal-unit space, no
/// board-specific scaling constants are needed.
///
/// # Parameters
/// - `meas`: internal-unit measurement from R and L measurement
/// - `pwm_freq_hz`: TMC4671 PWM frequency (6250 Hz for OpenFFBoard)
///
/// # Returns
/// Computed gains, or `DEFAULT_INNER_GAINS` if measurement data is invalid.
/// Absolute target bandwidth for the current loop in Hz.
/// 50 Hz (314 rad/s) — validated on hardware via sweep (50/62.5 Hz).
/// 50 Hz produces silent idle with minimal vibration; 62.5 Hz is
/// slightly audible with stronger vibration. Move quality is equal.
const BANDWIDTH_HZ: f32 = 50.0;

pub fn compute_gains_from_internal(meas: &InternalMeasurement, pwm_freq_hz: u32) -> InnerLoopGains {
    if meas.current_avg == 0 || meas.delta_i_avg == 0 || meas.dt_us == 0 {
        return DEFAULT_INNER_GAINS;
    }

    // L_int = V * dt / dI (internal units, dt in seconds)
    let v = meas.voltage as f32;
    let di = meas.delta_i_avg as f32;
    let i_ss = meas.current_avg as f32;
    let dt_s = meas.dt_us as f32 / 1_000_000.0;
    let l_int = v * dt_s / di;

    // R_int = V / I_ss (internal units, resistance in ADC/DAC space)
    let r_int = v / i_ss;

    // omega_bw = 2 * pi * bandwidth_hz (absolute, not relative to f_pwm)
    let omega_bw = 2.0 * core::f32::consts::PI * BANDWIDTH_HZ;

    // P = L_int * omega_bw * 256 (Q8.8)
    let p_real = l_int * omega_bw;
    let p_reg = (p_real * 256.0).clamp(1.0, u16::MAX as f32) as u16;

    // I gain: pole-zero cancellation places the PI zero at the plant
    // pole (1/tau = R/L). In the TMC4671's advanced PI structure where
    // the integrator is scaled by P, the effective Ki = P * I / 256^2
    // per sample. For the zero at R/L:
    //   I = R_int * 256 / (L_int * f_pwm)
    let f_pwm = pwm_freq_hz as f32;
    let i_real = r_int * 256.0 / (l_int * f_pwm);
    let i_reg = i_real.clamp(1.0, u16::MAX as f32) as u16;

    InnerLoopGains {
        flux_p: p_reg,
        flux_i: i_reg,
        torque_p: p_reg,
        torque_i: i_reg,
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
    fn default_gains_are_valid() {
        assert!(DEFAULT_INNER_GAINS.flux_p > 0);
        assert!(DEFAULT_INNER_GAINS.flux_i > 0);
        assert!(DEFAULT_INNER_GAINS.torque_p > 0);
        assert!(DEFAULT_INNER_GAINS.torque_i > 0);
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

    #[test]
    fn internal_gains_zero_current_returns_defaults() {
        let meas = InternalMeasurement {
            voltage: 512,
            current_avg: 0,
            delta_i_avg: 100,
            dt_us: 98,
        };
        assert_eq!(
            compute_gains_from_internal(&meas, 6250),
            DEFAULT_INNER_GAINS
        );
    }

    #[test]
    fn internal_gains_zero_delta_returns_defaults() {
        let meas = InternalMeasurement {
            voltage: 512,
            current_avg: 455,
            delta_i_avg: 0,
            dt_us: 98,
        };
        assert_eq!(
            compute_gains_from_internal(&meas, 6250),
            DEFAULT_INNER_GAINS
        );
    }

    #[test]
    fn internal_gains_p_scales_with_voltage_over_delta_i() {
        let low_l = InternalMeasurement {
            voltage: 512,
            current_avg: 455,
            delta_i_avg: 200,
            dt_us: 98,
        };
        let high_l = InternalMeasurement {
            voltage: 512,
            current_avg: 455,
            delta_i_avg: 50,
            dt_us: 98,
        };
        let gains_low = compute_gains_from_internal(&low_l, 6250);
        let gains_high = compute_gains_from_internal(&high_l, 6250);
        assert!(
            gains_high.flux_p > gains_low.flux_p,
            "higher L (lower dI) should give higher P: low={} high={}",
            gains_low.flux_p,
            gains_high.flux_p,
        );
    }

    #[test]
    fn internal_gains_p_independent_of_pwm_freq() {
        // Bandwidth is absolute (50 Hz), so P does not depend on f_pwm.
        // I depends on f_pwm (pole-zero cancellation: I = R*256/(L*f_pwm)).
        let meas = InternalMeasurement {
            voltage: 512,
            current_avg: 455,
            delta_i_avg: 100,
            dt_us: 98,
        };
        let gains_6250 = compute_gains_from_internal(&meas, 6250);
        let gains_25000 = compute_gains_from_internal(&meas, 25000);
        assert_eq!(
            gains_6250.flux_p, gains_25000.flux_p,
            "P should not depend on f_pwm"
        );
        assert!(
            gains_6250.flux_i > gains_25000.flux_i,
            "I should decrease with higher f_pwm: 6250Hz={} 25kHz={}",
            gains_6250.flux_i,
            gains_25000.flux_i,
        );
    }

    #[test]
    fn internal_gains_i_matches_reference() {
        // Test motor: V=512, I_ss=300, dI=17, dt=98us
        // R_int = 512/300 = 1.707, L_int = 512*98e-6/17 = 0.00295
        // I = R_int * 256 / (L_int * 6250) = 1.707 * 256 / (0.00295 * 6250) = 23.7
        let meas = InternalMeasurement {
            voltage: 512,
            current_avg: 300,
            delta_i_avg: 17,
            dt_us: 98,
        };
        let gains = compute_gains_from_internal(&meas, 6250);
        // Should be close to the empirically validated I=26
        assert!(
            gains.flux_i >= 20 && gains.flux_i <= 30,
            "flux_i={} expected ~24 (near empirical 26)",
            gains.flux_i,
        );
        assert_eq!(gains.flux_i, gains.torque_i);
    }
}
