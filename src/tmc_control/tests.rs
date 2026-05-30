use super::*;
use crate::config::TmcConfig;
use crate::pid::InnerLoopGains;

#[test]
fn runtime_starts_disarmed_and_uncalibrated() {
    let runtime = TmcControlRuntime::new_for_channel(0);

    assert_eq!(runtime.channel(), 0);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert_eq!(runtime.stored_gains(), None);
    assert_eq!(runtime.pending_commission_kind(), None);
}

#[test]
fn runtime_stores_inner_gains_without_spi_side_effects() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let gains = InnerLoopGains {
        flux_p: 11,
        flux_i: 12,
        torque_p: 13,
        torque_i: 14,
    };

    runtime.set_stored_gains(gains);

    assert_eq!(runtime.stored_gains(), Some(gains));
}

#[test]
fn runtime_config_carries_board_constants() {
    let tmc = TmcConfig::default_for_test();
    let config = TmcControlConfig::new(84_000_000, tmc.pwm_freq_hz(), 0.03125, tmc);

    assert_eq!(config.clock_freq, 84_000_000);
    assert_eq!(config.pwm_freq_hz, 25_000);
    assert_eq!(config.current_scale_ma_per_lsb, 0.03125);
    assert_eq!(config.tmc, tmc);
}

#[test]
fn state_refs_carry_channel_state_without_ownership() {
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let refs = TmcControlStateRefs::new(&tmc, &stepper);

    assert!(core::ptr::eq(refs.tmc, &tmc));
    assert!(core::ptr::eq(refs.stepper, &stepper));
}
