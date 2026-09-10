use super::ElectricalModel;

pub(super) fn autotune_test_config() -> crate::config::TmcConfig {
    crate::config::TmcConfig {
        pwm_maxcnt: 3999,
        tmc_clock_hz: 25_000_000,
        bbm_h: 10,
        bbm_l: 10,
        dsadc_mdec_a: 331,
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
    }
}

pub(super) fn test_electrical_model() -> ElectricalModel {
    ElectricalModel {
        r_count_milli: 1125,
        l_count_micro: 2950,
        tau_e_us: 2622,
        theta_e_us: 700,
    }
}
