use super::*;
use crate::commission_dispatch::test_support::MockBackend;
use crate::commissioning::{CommissioningEngine, types::StepResult};
use crate::config::TmcConfig;
use crate::pid::InnerLoopGains;
use crate::tmc::{TmcRequest, TmcRequestKind};
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_hal_async::spi::{ErrorKind, ErrorType, Operation, SpiDevice};
use foci_tmc4671::registers::{ABN_DECODER_PHI_E_PHI_M_OFFSET, PID_POSITION_TARGET};
use foci_tmc4671::spi::Tmc4671Spi;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockSpiError;

impl embedded_hal_async::spi::Error for MockSpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

#[derive(Default)]
struct RuntimeMockSpi {
    writes: std::vec::Vec<(u8, u32)>,
    write_attempts: std::vec::Vec<(u8, u32)>,
    read_requests: std::vec::Vec<u8>,
    reads: std::collections::VecDeque<u32>,
    fail_writes_to: Option<u8>,
}

impl RuntimeMockSpi {
    fn with_read(mut self, value: u32) -> Self {
        self.reads.push_back(value);
        self
    }

    fn fail_writes_to(mut self, addr: u8) -> Self {
        self.fail_writes_to = Some(addr);
        self
    }

    fn attempted_write_to(&self, register: foci_tmc4671::registers::Register) -> bool {
        self.write_attempts
            .iter()
            .any(|(addr, _)| *addr == register.addr())
    }

    fn read_requested(&self, register: foci_tmc4671::registers::Register) -> bool {
        self.read_requests
            .iter()
            .any(|addr| *addr == register.addr())
    }
}

impl ErrorType for RuntimeMockSpi {
    type Error = MockSpiError;
}

impl SpiDevice<u8> for RuntimeMockSpi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                Operation::Write(buf) if buf.len() == 5 => {
                    let addr = buf[0] & 0x7f;
                    let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    self.write_attempts.push((addr, value));
                    if self.fail_writes_to == Some(addr) {
                        return Err(MockSpiError);
                    }
                    self.writes.push((addr, value));
                }
                Operation::Write(buf) if buf.len() == 1 => {
                    self.read_requests.push(buf[0]);
                }
                Operation::Read(buf) => {
                    let value = self.reads.pop_front().unwrap_or(0);
                    buf.copy_from_slice(&value.to_be_bytes());
                }
                Operation::Write(_) | Operation::DelayNs(_) => {}
                Operation::Transfer(read, write) => {
                    read.copy_from_slice(write);
                }
                Operation::TransferInPlace(_) => {}
            }
        }
        Ok(())
    }
}

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

fn commissioning_runtime_config() -> TmcControlConfig {
    let tmc = TmcConfig::default_for_test();
    TmcControlConfig::new(84_000_000, tmc.pwm_freq_hz(), 0.03125, tmc)
}

#[test]
fn step_commissioning_returns_none_without_engine() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();

        let result = runtime.step_commissioning(&mut backend).await;

        assert!(result.is_none());
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn drive_commissioning_result_returns_none_without_engine() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();

        let action = runtime
            .drive_commissioning_result(
                StepResult::Continue,
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
                &config,
            )
            .await;

        assert!(action.is_none());
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn tick_commissioning_returns_none_without_engine() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();

        let action = runtime
            .tick_commissioning(
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
                &config,
            )
            .await;

        assert!(action.is_none());
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn split_commissioning_tick_maps_continue_without_consuming_engine() {
    pollster::block_on(async {
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();

        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

        let result = runtime
            .step_commissioning(&mut backend)
            .await
            .expect("engine should be active");

        assert!(matches!(result, StepResult::Continue));
        assert!(runtime.is_commission_active());

        let action = runtime
            .drive_commissioning_result(result, &mut backend, tmc, stepper, &config)
            .await
            .expect("engine should still be active for dispatch");

        assert!(matches!(action, CommissionTickAction::Continue));
        assert!(runtime.is_commission_active());
    });
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
fn queue_flush_signal_resets_runtime_and_channel_state() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    tmc.current_limit_reg.store(88, Relaxed);
    stepper.position.store(55, Relaxed);
    queue_flush.store(true, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert!(!queue_flush.load(Relaxed));
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_motor_enabled());
    assert_eq!(tmc.current_limit_reg.load(Relaxed), 0);
    assert_eq!(stepper.position.load(Relaxed), 0);
    assert!(!fault_active.load(Relaxed));
    assert!(
        actions
            .iter()
            .any(|a| { matches!(a, BoardAction::Stepper(StepperAction::StopNow)) })
    );
}

#[test]
fn pid_error_clear_request_loads_hold_settle_countdown() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();

    tmc.pid_error_sums_clear.store(true, Relaxed);
    runtime.poll_pid_error_clear_request(&tmc);

    assert_eq!(
        runtime.pid_error_clear_countdown_for_test(),
        crate::tmc_control::HOLD_SETTLE_DELAY_MS
    );
}

#[test]
fn shutdown_quiesce_clears_safe_state_without_config_reset() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let gains = InnerLoopGains {
        flux_p: 11,
        flux_i: 12,
        torque_p: 13,
        torque_i: 14,
    };

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(crate::commissioning::CommissioningEngine::new_calibrate(0));
    runtime.set_stored_gains(gains);
    runtime.set_velocity_scale(123_456);
    runtime.set_feedforward_was_active(true);
    runtime.set_pid_error_clear_countdown_ms(9);

    let actions = runtime.prepare_shutdown_quiesce();

    assert!(
        actions
            .iter()
            .any(|a| matches!(a, BoardAction::Stepper(StepperAction::StopNow)))
    );
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_commission_active());
    assert!(!runtime.feedforward_was_active());
    assert_eq!(runtime.pid_error_clear_countdown_ms(), 0);
    assert_eq!(runtime.stored_gains(), Some(gains));
    assert_eq!(runtime.velocity_scale(), 123_456);
}

#[test]
fn feedforward_and_pid_settle_accessors_round_trip() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    assert!(!runtime.feedforward_was_active());
    runtime.set_feedforward_was_active(true);
    assert!(runtime.feedforward_was_active());

    assert_eq!(runtime.pid_error_clear_countdown_ms(), 0);
    runtime.set_pid_error_clear_countdown_ms(5);
    assert_eq!(runtime.pid_error_clear_countdown_ms(), 5);
}

#[test]
fn dispatch_state_remains_usable_after_shutdown_quiesce() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc: &'static crate::state::tmc::TmcChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::tmc::TmcChannelState::new()),
    );
    let stepper: &'static crate::state::stepper::StepperChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::stepper::StepperChannelState::new()),
    );

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.prepare_shutdown_quiesce();

    {
        let dispatch = runtime.dispatch_state(tmc, stepper);
        *dispatch.calibrated = true;
        *dispatch.motor_enabled = true;
    }

    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());
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

#[test]
fn phase_advance_state_tracks_restore_to_base_register() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);

    runtime.mark_phase_advance_restored_for_test();

    assert!(!runtime.phase_advance_active_for_test());
    assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
}

#[test]
fn phase_advance_restore_now_writes_base_and_clears_active_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let restored = runtime.restore_phase_advance_now(&mut tmc).await;

        assert!(restored);
        assert!(!runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr(), 0x1111_2222)]
        );
    });
}

#[test]
fn phase_advance_restore_now_failure_preserves_retry_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let raw_spi =
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);

        let restored = runtime.restore_phase_advance_now(&mut tmc).await;

        assert!(!restored);
        assert!(runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x3333_4444);
        let raw_spi = tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr(), 0x1111_2222)]
        );
        assert!(raw_spi.writes.is_empty());
    });
}

#[test]
fn phase_advance_tick_reset_gate_skips_abn_snapshot_and_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.phase_advance.store(true, Relaxed);
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state.step_width_value.store(16, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                123,
                false,
                true,
            )
            .await;

        assert!(!runtime.phase_advance_active_for_test());
        let raw_spi = tmc.into_inner();
        assert!(!raw_spi.read_requested(ABN_DECODER_PHI_E_PHI_M_OFFSET));
        assert!(!raw_spi.attempted_write_to(ABN_DECODER_PHI_E_PHI_M_OFFSET));
    });
}

#[test]
fn position_lead_tick_writes_target_and_tracks_applied_scaled() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state.position_lead_gain_permille.store(1000, Relaxed);
        tmc_state.position_lead_max_counts.store(20, Relaxed);
        tmc_state.step_width_value.store(16, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let lead_counts =
            crate::tmc::compute_position_lead_counts(84_000_000, 2_800, false, 1000, 20);
        let (expected_target, expected_applied_scaled) =
            crate::tmc::compute_position_lead_target(1_000, 0, lead_counts, 16);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                0,
                false,
                false,
            )
            .await;

        assert_eq!(
            runtime.position_lead_applied_scaled_for_test(),
            expected_applied_scaled
        );
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_TARGET.addr(), expected_target)]
        );
    });
}
