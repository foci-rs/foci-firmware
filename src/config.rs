//! Board-specific TMC4671 hardware configuration.

/// Return the DSADC MDEC value for a TMC4671 PWM period.
///
/// Datasheet section 4.11.2 defines the DSADC sample timing as:
/// `MDEC = round(tPWM / (3 * 40ns) - 2)`.
///
/// `PWM_MAXCNT` counts 10ns ticks, so this is equivalent to:
/// `round((PWM_MAXCNT + 1) / 12) - 2`.
pub const fn dsadc_mdec_from_pwm_maxcnt(pwm_maxcnt: u16) -> u16 {
    let pwm_ticks = pwm_maxcnt as u32 + 1;
    let rounded_div_12 = (pwm_ticks + 6) / 12;
    if rounded_div_12 <= 2 {
        0
    } else {
        (rounded_div_12 - 2) as u16
    }
}

const CURRENT_FILTER_RATIO_TO_BANDWIDTH: f32 = 4.0;
const CURRENT_FILTER_FLOOR_HZ: u32 = 1000;
const CURRENT_FILTER_MAX_HZ: u32 = 6000;

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
    /// DSADC decimation parameter for ADC group A.
    pub dsadc_mdec_a: u16,
    /// DSADC decimation parameter for ADC group B.
    pub dsadc_mdec_b: u16,
    /// ADC_I_SELECT register value for phase-current routing.
    pub adc_i_select: u32,
    /// Excitation voltage for R/L measurement (PIDOUT units).
    pub excitation_voltage: u16,
    /// Current loop bandwidth in Hz (empirically validated per PWM frequency).
    pub bandwidth_hz: f32,
    /// Board-derived effective electrical delay for current-loop synthesis.
    pub electrical_model_theta_e_us: u32,
    /// Maximum encoder movement allowed during flux-axis current validation.
    pub current_validation_flux_max_encoder_delta_counts: u16,
    /// Maximum encoder movement allowed during torque-axis current validation.
    pub current_validation_torque_max_encoder_delta_counts: u16,
    /// Maximum expected resistance-profile current in TMC current counts.
    pub resistance_max_expected_current_count: u16,
    /// True when board scale metadata is hardware-validated for physical display.
    pub resistance_scale_metadata_validated: bool,
    /// Board-specific milliamps-per-LSB scale for `PID_TORQUE_FLUX_LIMITS`.
    pub current_scale_ma_per_lsb: f32,
}

/// Which physical encoder drives a TMC4671 selection register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FeedbackSource {
    /// ABN (quadrature) encoder.
    Abn,
    /// Analog or Hall-effect encoder, read through the AENC input.
    Analog,
}

/// A channel's steady-state feedback wiring, mirroring the TMC4671's two
/// independent selection registers (`PHI_E_SELECTION`, `POSITION_SELECTION`).
///
/// Describes board wiring, not live register state: commissioning routines
/// (e.g. electrical identification's open-loop phase) may transiently write a
/// selection register to a value other than what's declared here. See
/// `current_loop_actionable_status_mask`'s "Known limitation" doc comment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FeedbackConfig {
    /// Source driving `PHI_E_SELECTION` (commutation angle) at steady state.
    pub phi_e_source: FeedbackSource,
    /// Source driving `POSITION_SELECTION` (mechanical position) at steady state.
    pub position_source: FeedbackSource,
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

    /// Default torque/flux current-measurement filter cutoff in Hz.
    ///
    /// The cutoff tracks the current-loop bandwidth so retuning the loop keeps
    /// roughly the same filter-to-crossover ratio. The upper bound stays below
    /// the 25 kHz PWM Nyquist region and follows lower PWM frequencies by
    /// capping at one quarter of the PWM frequency.
    pub fn default_current_filter_hz(&self) -> u16 {
        self.default_current_filter_hz_for_bandwidth(self.bandwidth_hz)
    }

    /// Default torque/flux current-measurement filter cutoff for an accepted
    /// current-loop bandwidth in Hz.
    pub fn default_current_filter_hz_for_bandwidth(&self, bandwidth_hz: f32) -> u16 {
        let derived = if bandwidth_hz.is_finite() && bandwidth_hz > 0.0 {
            (bandwidth_hz * CURRENT_FILTER_RATIO_TO_BANDWIDTH + 0.5) as u32
        } else {
            0
        };
        let with_floor = derived.max(CURRENT_FILTER_FLOOR_HZ);
        let pwm_margin = self.pwm_freq_hz() / 4;
        with_floor.min(CURRENT_FILTER_MAX_HZ).min(pwm_margin) as u16
    }

    /// BBM register value: `(bbm_h << 8) | bbm_l`.
    pub const fn bbm_register(&self) -> u32 {
        ((self.bbm_h as u32) << 8) | (self.bbm_l as u32)
    }

    /// DSADC_MDEC_B_MDEC_A register value: group A in bits 15:0, group B in bits 31:16.
    pub const fn dsadc_mdec_register(&self) -> u32 {
        ((self.dsadc_mdec_b as u32) << 16) | (self.dsadc_mdec_a as u32)
    }

    /// Build the board's standard resistance-identification profile.
    pub const fn resistance_profile(
        &self,
        scale_metadata_validated: bool,
    ) -> crate::resistance_identification::ResistanceProfile {
        let mut profile = crate::resistance_identification::ResistanceProfile::standard();
        profile.pwm_maxcnt = self.pwm_maxcnt;
        profile.bbm_h = self.bbm_h;
        profile.bbm_l = self.bbm_l;
        profile.dsadc_mdec_a = self.dsadc_mdec_a;
        profile.dsadc_mdec_b = self.dsadc_mdec_b;
        profile.max_expected_current_count = self.resistance_max_expected_current_count;
        profile.scale_metadata_validated = scale_metadata_validated;
        profile
    }
}

#[cfg(test)]
impl TmcConfig {
    /// Minimal valid configuration for host-side unit tests.
    ///
    /// Uses representative 25 MHz TMC clock and 25 kHz PWM parameters.
    /// Tests that need different parameters should construct `TmcConfig`
    /// directly.
    pub fn default_for_test() -> Self {
        Self {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: 0x1800_0100,
            excitation_voltage: 512,
            bandwidth_hz: 50.0,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
        }
    }
}

#[cfg(test)]
impl FeedbackConfig {
    /// ABN on both selectors — every board's configuration today.
    pub const fn abn_abn() -> Self {
        Self {
            phi_e_source: FeedbackSource::Abn,
            position_source: FeedbackSource::Abn,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> TmcConfig {
        TmcConfig {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: 0x1800_0100,
            excitation_voltage: 512,
            bandwidth_hz: 50.0,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
        }
    }

    #[test]
    fn pwm_freq_hz_datasheet_formula() {
        let cfg = test_config();
        assert_eq!(cfg.pwm_freq_hz(), 25_000);
    }

    #[test]
    fn current_filter_default_tracks_bandwidth_and_caps_at_six_khz() {
        let mut cfg = test_config();

        cfg.bandwidth_hz = 400.0;
        assert_eq!(cfg.default_current_filter_hz(), 1600);

        cfg.bandwidth_hz = 1200.0;
        assert_eq!(cfg.default_current_filter_hz(), 4800);

        cfg.bandwidth_hz = 1600.0;
        assert_eq!(cfg.default_current_filter_hz(), 6000);

        cfg.bandwidth_hz = 2000.0;
        assert_eq!(cfg.default_current_filter_hz(), 6000);
    }

    #[test]
    fn current_filter_default_has_floor_and_pwm_margin() {
        let mut cfg = test_config();

        cfg.bandwidth_hz = 50.0;
        assert_eq!(cfg.default_current_filter_hz(), 1000);

        cfg.bandwidth_hz = 1600.0;
        cfg.pwm_maxcnt = 9999;
        assert_eq!(cfg.default_current_filter_hz(), 2500);
    }

    #[test]
    fn pwm_period_ns_matches_freq() {
        let cfg = test_config();
        assert_eq!(cfg.pwm_period_ns(), 40_000);
    }

    #[test]
    fn bbm_register_packs_correctly() {
        let cfg = test_config();
        assert_eq!(cfg.bbm_register(), (10 << 8) | 10);
    }

    #[test]
    fn dsadc_mdec_register_packs_correctly() {
        let cfg = test_config();
        assert_eq!(cfg.dsadc_mdec_register(), (331 << 16) | 331);
    }

    #[test]
    fn dsadc_mdec_register_packs_group_b_high_group_a_low() {
        let cfg = TmcConfig {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 40,
            bbm_l: 40,
            dsadc_mdec_a: 660,
            dsadc_mdec_b: 331,
            adc_i_select: 0x1800_0100,
            excitation_voltage: 512,
            bandwidth_hz: 150.0,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
        };
        assert_eq!(cfg.dsadc_mdec_register(), (331 << 16) | 660);
    }

    #[test]
    fn pwm_freq_hz_at_maxcnt_999() {
        let cfg = TmcConfig {
            pwm_maxcnt: 999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: 0x1800_0100,
            excitation_voltage: 512,
            bandwidth_hz: 50.0,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
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
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: 0x1800_0100,
            excitation_voltage: 512,
            bandwidth_hz: 50.0,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
        };
        assert_eq!(cfg.pwm_freq_hz(), 50_000);
    }

    #[test]
    fn dsadc_mdec_from_pwm_maxcnt_uses_datasheet_formula() {
        assert_eq!(dsadc_mdec_from_pwm_maxcnt(3999), 331);
        assert_eq!(dsadc_mdec_from_pwm_maxcnt(2047), 169);
        assert_eq!(dsadc_mdec_from_pwm_maxcnt(1023), 83);
    }

    #[test]
    fn openffboard_resistance_profile_matches_power_stage() {
        let cfg = TmcConfig::default_for_test();
        let profile = cfg.resistance_profile(true);

        assert_eq!(profile.pwm_maxcnt, cfg.pwm_maxcnt);
        assert_eq!(profile.bbm_h, cfg.bbm_h);
        assert_eq!(profile.bbm_l, cfg.bbm_l);
        assert_eq!(profile.dsadc_mdec_a, cfg.dsadc_mdec_a);
        assert_eq!(profile.dsadc_mdec_b, cfg.dsadc_mdec_b);
    }

    #[test]
    fn feedback_config_abn_abn_is_abn_on_both_selectors() {
        let config = FeedbackConfig::abn_abn();
        assert_eq!(config.phi_e_source, FeedbackSource::Abn);
        assert_eq!(config.position_source, FeedbackSource::Abn);
    }
}
