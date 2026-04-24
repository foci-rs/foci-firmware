//! Board-specific TMC4671 hardware configuration.

/// Board-specific TMC4671 hardware configuration.
///
/// Constructed as a `const` by each firmware crate. Passed to all
/// foci-firmware functions that interact with TMC4671 registers.
/// Changing `pwm_maxcnt` is the only edit needed to switch PWM frequency.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct TmcConfig {
    /// PWM_MAXCNT register value.
    pub pwm_maxcnt: u16,
    /// TMC4671 external oscillator frequency in Hz (typically 25 MHz).
    pub tmc_clock_hz: u32,
    /// Break-before-make dead-time, high-side (PWM clock cycles, 10ns each).
    pub bbm_h: u8,
    /// Break-before-make dead-time, low-side (PWM clock cycles, 10ns each).
    pub bbm_l: u8,
    /// DSADC decimation parameter (both ADC groups).
    pub dsadc_mdec: u16,
    /// Excitation voltage for R/L measurement (PIDOUT units).
    pub excitation_voltage: u16,
    /// Number of di/dt toggle cycles per coil for L measurement.
    pub didt_cycles: u32,
    /// Current loop bandwidth in Hz (empirically validated per PWM frequency).
    pub bandwidth_hz: f32,
}

impl TmcConfig {
    /// PWM switching frequency in Hz (datasheet section 4.9.3):
    /// `fPWM = 4 * fCLK / (PWM_MAXCNT + 1)`
    pub const fn pwm_freq_hz(&self) -> u32 {
        4 * self.tmc_clock_hz / (self.pwm_maxcnt as u32 + 1)
    }

    /// PWM period in nanoseconds.
    /// `T = (PWM_MAXCNT + 1) * 10ns` (100 MHz internal counter)
    pub const fn pwm_period_ns(&self) -> u32 {
        (self.pwm_maxcnt as u32 + 1) * 10
    }

    /// BBM register value: `(bbm_h << 8) | bbm_l`.
    pub const fn bbm_register(&self) -> u32 {
        ((self.bbm_h as u32) << 8) | (self.bbm_l as u32)
    }

    /// DSADC_MDEC_B_MDEC_A register value (same MDEC for both groups).
    pub const fn dsadc_mdec_register(&self) -> u32 {
        ((self.dsadc_mdec as u32) << 16) | (self.dsadc_mdec as u32)
    }
}

#[cfg(test)]
impl TmcConfig {
    /// Minimal valid configuration for host-side unit tests.
    ///
    /// Uses OpenFFBoard v1.2.4 hardware constants (25 MHz clock, 4 kHz PWM)
    /// so computed values match the reference board. Tests that need different
    /// parameters should construct `TmcConfig` directly.
    pub fn default_for_test() -> Self {
        Self {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec: 331,
            excitation_voltage: 512,
            didt_cycles: 5000,
            bandwidth_hz: 50.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn openffboard_config() -> TmcConfig {
        TmcConfig {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec: 331,
            excitation_voltage: 512,
            didt_cycles: 5000,
            bandwidth_hz: 50.0,
        }
    }

    #[test]
    fn pwm_freq_hz_datasheet_formula() {
        let cfg = openffboard_config();
        assert_eq!(cfg.pwm_freq_hz(), 25_000);
    }

    #[test]
    fn pwm_period_ns_matches_freq() {
        let cfg = openffboard_config();
        assert_eq!(cfg.pwm_period_ns(), 40_000);
    }

    #[test]
    fn bbm_register_packs_correctly() {
        let cfg = openffboard_config();
        assert_eq!(cfg.bbm_register(), (10 << 8) | 10);
    }

    #[test]
    fn dsadc_mdec_register_packs_correctly() {
        let cfg = openffboard_config();
        assert_eq!(cfg.dsadc_mdec_register(), (331 << 16) | 331);
    }

    #[test]
    fn pwm_freq_hz_at_maxcnt_999() {
        let cfg = TmcConfig {
            pwm_maxcnt: 999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec: 331,
            excitation_voltage: 512,
            didt_cycles: 5000,
            bandwidth_hz: 50.0,
        };
        assert_eq!(cfg.pwm_freq_hz(), 100_000);
    }

    #[test]
    fn pwm_freq_hz_at_maxcnt_1999() {
        let cfg = TmcConfig {
            pwm_maxcnt: 1999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec: 331,
            excitation_voltage: 512,
            didt_cycles: 5000,
            bandwidth_hz: 50.0,
        };
        assert_eq!(cfg.pwm_freq_hz(), 50_000);
    }
}
