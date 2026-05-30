use super::*;
use crate::config::TmcConfig;
use crate::pid::InnerLoopGains;
use crate::tmc::{TmcRequest, TmcRequestKind};
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

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

#[test]
fn dispatch_state_view_mutates_runtime_owned_flags() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc: &'static crate::state::tmc::TmcChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::tmc::TmcChannelState::new()),
    );
    let stepper: &'static crate::state::stepper::StepperChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::stepper::StepperChannelState::new()),
    );

    {
        let dispatch = runtime.dispatch_state(tmc, stepper);
        *dispatch.calibrated = true;
        *dispatch.motor_enabled = true;
    }

    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());

    runtime.set_calibrated_for_test(false);
    runtime.set_motor_enabled_for_test(false);
}

#[test]
fn config_reset_runtime_state_clears_owned_flags_and_defaults() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_velocity_scale_for_test(1234);
    runtime.set_slow_control_accum_us_for_test(750);

    runtime.reset_runtime_state();

    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_motor_enabled());
    assert_eq!(runtime.velocity_scale_for_test(), 0);
    assert_eq!(runtime.slow_control_accum_us_for_test(), 0);
}

#[test]
fn config_reset_channel_state_restores_known_defaults() {
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();

    tmc.current_limit_reg.store(99, Relaxed);
    tmc.velocity_limit_cfg.store(123, Relaxed);
    stepper.position.store(44, Relaxed);

    TmcControlRuntime::reset_channel_state(&tmc, &stepper);

    assert_eq!(tmc.current_limit_reg.load(Relaxed), 0);
    assert_eq!(tmc.velocity_limit_cfg.load(Relaxed), 500_000);
    assert_eq!(stepper.position.load(Relaxed), 0);
}

#[test]
fn shutdown_signal_emits_one_stop_action_until_reset() {
    let shutdown_active = AtomicBool::new(true);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(false);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    let first = runtime.poll_shutdown(&signals);
    let second = runtime.poll_shutdown(&signals);

    assert!(
        first
            .iter()
            .any(|a| matches!(a, BoardAction::Stepper(StepperAction::StopNow)))
    );
    assert!(second.is_empty());

    shutdown_active.store(false, Relaxed);
    runtime.poll_shutdown(&signals);
}

#[test]
fn emergency_disable_signal_is_consumed_by_runtime() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(false);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    disable_signal.signal(());
    let actions = runtime.poll_emergency_disable(&signals, 1 << 26);

    assert!(
        actions
            .iter()
            .any(|a| matches!(a, BoardAction::Shutdown(ShutdownReason::AdcCurrentClipped)))
    );
    assert!(disable_signal.try_take().is_none());
}

#[test]
fn diagnostic_requests_are_delegated() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let req = TmcRequest::new(
        0,
        TmcRequestKind::VoltageStepTest {
            oid: 9,
            uq_ext: 10,
            ud_ext: -10,
            sample_delay_ms: 3,
        },
    );

    let actions = runtime.classify_request(req);

    assert!(
        actions
            .iter()
            .any(|a| matches!(a, BoardAction::DelegatedDiagnostic(_)))
    );
}

#[test]
fn invalid_channel_request_emits_shutdown_action() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let req = TmcRequest::new(1, TmcRequestKind::SetEnable(false));

    let actions = runtime.classify_request(req);

    assert!(
        actions
            .iter()
            .any(|a| matches!(a, BoardAction::Shutdown(ShutdownReason::InvalidTmcChannel)))
    );
}

#[test]
fn commission_tick_without_engine_is_empty() {
    let runtime = TmcControlRuntime::new_for_channel(0);
    assert!(runtime.pending_commission_kind().is_none());
    assert!(!runtime.is_commission_active());
}

#[test]
fn take_commission_engine_is_none_when_idle() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    assert!(runtime.take_commission_engine().is_none());
    assert!(!runtime.is_commission_active());
}

#[test]
fn clear_calibrated_resets_flag() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.clear_calibrated();
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_before_calibration_without_gate_is_noop() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    tmc.auto_calibrate_on_enable_allowed.store(false, Relaxed);

    let actions = runtime.prepare_enable_request(&tmc);

    assert!(actions.is_empty());
    assert!(!runtime.is_motor_enabled());
}

#[test]
fn enable_after_calibration_emits_arm_check_action() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    runtime.set_calibrated_for_test(true);

    let actions = runtime.prepare_enable_request(&tmc);

    assert!(actions.iter().any(|a| matches!(
        a,
        BoardAction::Stepper(StepperAction::ArmCheck {
            post_action: crate::commission_dispatch::StepperPostAction::ArmPreservePosition,
        })
    )));
}

#[test]
fn disable_request_stops_and_clears_runtime_flags() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let actions = runtime.prepare_disable_request();

    assert!(
        actions
            .iter()
            .any(|a| matches!(a, BoardAction::Stepper(StepperAction::StopNow)))
    );
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn slow_control_tick_fires_every_millisecond_under_fast_period() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    assert!(!runtime.update_slow_control_for_test(250));
    assert!(!runtime.update_slow_control_for_test(250));
    assert!(!runtime.update_slow_control_for_test(250));
    assert!(runtime.update_slow_control_for_test(250));
    assert_eq!(runtime.slow_control_accum_us_for_test(), 0);
}

#[test]
fn slow_control_tick_fires_immediately_for_default_period() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    assert!(runtime.update_slow_control_for_test(1_000));
    assert_eq!(runtime.slow_control_accum_us_for_test(), 0);
}

#[test]
fn command_velocity_reference_zeroes_when_reset_is_requested() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, true);

    assert_eq!(reference, 0);
}

#[test]
fn command_velocity_reference_can_use_single_consumed_reset_latch() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_velocity_scale_for_test(84_000_000);
    tmc.feedforward_reset.store(true, Relaxed);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reset = tmc.feedforward_reset.swap(false, Relaxed);
    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, reset);

    assert!(reset);
    assert_eq!(reference, 0);
    assert!(!tmc.feedforward_reset.load(Relaxed));
}

#[test]
fn command_velocity_reference_uses_motion_when_reset_false() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(
        reference,
        crate::tmc::compute_command_velocity_reference(84_000_000, 2_800, false)
    );
}

#[test]
fn command_velocity_reference_zeroes_when_runtime_inactive() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(reference, 0);
}

#[test]
fn velocity_scale_accessors_round_trip() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    assert_eq!(runtime.velocity_scale(), 0);
    runtime.set_velocity_scale(123_456);
    assert_eq!(runtime.velocity_scale(), 123_456);
}
