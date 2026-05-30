use super::*;
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
