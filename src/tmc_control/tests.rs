use super::*;
use crate::commission_dispatch::test_support::MockBackend;
use crate::commissioning::{CommissioningEngine, types::StepResult};
use crate::config::TmcConfig;
use crate::pid::{InnerLoopGains, OuterLoopGains};
use crate::tmc::{TmcRequest, TmcRequestKind};
use crate::tmc_control::dispatch::{SharedTmcDispatch, apply_shared_tmc_request};
use crate::tmc_control::helpers::{
    apply_live_inner_gains, apply_live_outer_gains, refresh_velocity_scale_from_state,
};
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_hal_async::spi::{ErrorKind, ErrorType, Operation, SpiDevice};
use foci_tmc4671::registers::{
    ABN_DECODER_MODE, ABN_DECODER_PHI_E_PHI_M_OFFSET, ABN_DECODER_PPR,
    PID_ERROR_PID_POSITION_ERROR_SUM, PID_FLUX_P_FLUX_I, PID_POSITION_ACTUAL,
    PID_POSITION_P_POSITION_I, PID_POSITION_TARGET, PID_TORQUE_FLUX_LIMITS, PID_TORQUE_P_TORQUE_I,
    PID_VELOCITY_P_VELOCITY_I, PIDOUT_UQ_UD_LIMITS, STATUS_FLAGS,
};
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

#[test]
fn request_outcome_constructors_encode_tail_shutdown_and_refresh() {
    let fall_through = TmcControlRequestOutcome::fall_through();
    assert_eq!(fall_through.tail, TmcControlRequestTail::FallThrough);
    assert_eq!(fall_through.shutdown, None);
    assert!(!fall_through.should_refresh_velocity_scale);

    let skip = TmcControlRequestOutcome::skip_tick_tail();
    assert_eq!(skip.tail, TmcControlRequestTail::SkipTickTail);
    assert_eq!(skip.shutdown, None);
    assert!(!skip.should_refresh_velocity_scale);

    let shutdown = TmcControlRequestOutcome::shutdown(
        ShutdownReason::RequestValidationFailed,
        TmcControlRequestTail::FallThrough,
    );
    assert_eq!(shutdown.tail, TmcControlRequestTail::FallThrough);
    assert_eq!(
        shutdown.shutdown,
        Some(ShutdownReason::RequestValidationFailed)
    );
    assert!(!shutdown.should_refresh_velocity_scale);

    let refresh =
        TmcControlRequestOutcome::refresh_velocity_scale(TmcControlRequestTail::SkipTickTail);
    assert_eq!(refresh.tail, TmcControlRequestTail::SkipTickTail);
    assert_eq!(refresh.shutdown, None);
    assert!(refresh.should_refresh_velocity_scale);

    let both = TmcControlRequestOutcome::shutdown_then_refresh_velocity_scale(
        ShutdownReason::InvalidTmcChannel,
        TmcControlRequestTail::FallThrough,
    );
    assert_eq!(both.tail, TmcControlRequestTail::FallThrough);
    assert_eq!(both.shutdown, Some(ShutdownReason::InvalidTmcChannel));
    assert!(both.should_refresh_velocity_scale);
}

#[test]
fn refresh_velocity_scale_reads_step_width_from_state() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc_state = crate::state::tmc::TmcChannelState::new();
    tmc_state.step_width_value.store(-3200, Relaxed);

    refresh_velocity_scale_from_state(&mut runtime, &tmc_state, 84_000_000, 25_000);

    assert_eq!(
        runtime.velocity_scale(),
        crate::tmc::compute_velocity_scale_factor(3200, 84_000_000, 25_000)
    );
}

#[test]
fn apply_live_inner_gains_stores_gains_and_writes_flux_then_torque() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let gains = InnerLoopGains {
            flux_p: 11,
            flux_i: 12,
            torque_p: 13,
            torque_i: 14,
        };

        let status = apply_live_inner_gains(&mut runtime, &mut tmc, gains).await;
        let spi = tmc.into_inner();

        assert!(status.all_ok());
        assert_eq!(runtime.stored_gains(), Some(gains));
        assert_eq!(
            spi.write_attempts,
            std::vec![
                (
                    PID_FLUX_P_FLUX_I.addr(),
                    crate::pid::pack_pid_register(11, 12)
                ),
                (
                    PID_TORQUE_P_TORQUE_I.addr(),
                    crate::pid::pack_pid_register(13, 14)
                ),
            ]
        );
    });
}

#[test]
fn apply_live_inner_gains_attempts_second_write_after_first_failure() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PID_FLUX_P_FLUX_I.addr()));
        let gains = InnerLoopGains {
            flux_p: 21,
            flux_i: 22,
            torque_p: 23,
            torque_i: 24,
        };

        let status = apply_live_inner_gains(&mut runtime, &mut tmc, gains).await;
        let spi = tmc.into_inner();

        assert!(!status.all_ok());
        assert!(!status.first_write_ok);
        assert!(status.second_write_ok);
        assert_eq!(runtime.stored_gains(), Some(gains));
        assert!(spi.attempted_write_to(PID_FLUX_P_FLUX_I));
        assert!(spi.attempted_write_to(PID_TORQUE_P_TORQUE_I));
    });
}

#[test]
fn apply_live_outer_gains_updates_state_and_writes_velocity_then_position() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let gains = OuterLoopGains {
            position_p: 101,
            position_i: 102,
            velocity_p: 103,
            velocity_i: 104,
        };

        let status = apply_live_outer_gains(&tmc_state, &mut tmc, gains).await;
        let spi = tmc.into_inner();

        assert!(status.all_ok());
        assert_eq!(tmc_state.position_p_value.load(Relaxed), 101);
        assert_eq!(tmc_state.position_i_value.load(Relaxed), 102);
        assert_eq!(tmc_state.velocity_p_value.load(Relaxed), 103);
        assert_eq!(tmc_state.velocity_i_value.load(Relaxed), 104);
        assert_eq!(
            spi.write_attempts,
            std::vec![
                (
                    PID_VELOCITY_P_VELOCITY_I.addr(),
                    crate::pid::pack_pid_register(103, 104)
                ),
                (
                    PID_POSITION_P_POSITION_I.addr(),
                    crate::pid::pack_pid_register(101, 102)
                ),
            ]
        );
    });
}

#[test]
fn apply_live_outer_gains_attempts_both_writes_on_partial_failure() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_POSITION_P_POSITION_I.addr()),
        );
        let gains = OuterLoopGains {
            position_p: 201,
            position_i: 202,
            velocity_p: 203,
            velocity_i: 204,
        };

        let status = apply_live_outer_gains(&tmc_state, &mut tmc, gains).await;
        let spi = tmc.into_inner();

        assert!(!status.all_ok());
        assert!(status.first_write_ok);
        assert!(!status.second_write_ok);
        assert!(spi.attempted_write_to(PID_VELOCITY_P_VELOCITY_I));
        assert!(spi.attempted_write_to(PID_POSITION_P_POSITION_I));
        assert_eq!(tmc_state.position_p_value.load(Relaxed), 201);
        assert_eq!(tmc_state.velocity_p_value.load(Relaxed), 203);
    });
}

#[test]
fn shared_dispatch_set_current_writes_current_limit_and_updates_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let expected_limit = ((1155.0_f32 * core::f32::consts::SQRT_2) / 1.155_f32) as u32;

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetCurrent { run_ma: 1155 },
            &mut tmc,
            &tmc_state,
            1.155,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert_eq!(
            spi.write_attempts,
            std::vec![(PID_TORQUE_FLUX_LIMITS.addr(), expected_limit)]
        );
        assert_eq!(tmc_state.current_limit_reg.load(Relaxed), expected_limit);
    });
}

#[test]
fn shared_dispatch_set_encoder_writes_quadrature_ppr() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetEncoder { ppr: 1000 },
            &mut tmc,
            &tmc_state,
            1.155,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert_eq!(
            spi.write_attempts,
            std::vec![(ABN_DECODER_PPR.addr(), 4000)]
        );
    });
}

#[test]
fn shared_dispatch_set_encoder_rejects_zero_ppr() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetEncoder { ppr: 0 },
            &mut tmc,
            &tmc_state,
            1.155,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Err(()));
        assert!(spi.write_attempts.is_empty());
    });
}

#[test]
fn shared_dispatch_set_encoder_dir_writes_mode_and_marks_reset() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetEncoderDir { invert: true },
            &mut tmc,
            &tmc_state,
            1.155,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert_eq!(
            spi.write_attempts,
            std::vec![(ABN_DECODER_MODE.addr(), 1 << 12)]
        );
        assert!(tmc_state.encoder_reversed.load(Relaxed));
        assert!(tmc_state.feedforward_reset.load(Relaxed));
    });
}

#[test]
fn shared_dispatch_set_voltage_limit_propagates_write_failure() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PIDOUT_UQ_UD_LIMITS.addr()));

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetVoltageLimit {
                voltage_limit: 32_000,
            },
            &mut tmc,
            &tmc_state,
            1.155,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Err(()));
        assert!(spi.attempted_write_to(PIDOUT_UQ_UD_LIMITS));
    });
}

#[test]
fn shared_dispatch_silent_noop_returns_handled_without_spi() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetVelocityFilter { filter_hz: 200 },
            &mut tmc,
            &tmc_state,
            1.155,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert!(spi.write_attempts.is_empty());
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn shared_dispatch_main_shell_request_returns_unhandled_without_spi() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::Calibrate { oid: 7 },
            &mut tmc,
            &tmc_state,
            1.155,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Unhandled));
        assert!(spi.write_attempts.is_empty());
        assert!(spi.read_requests.is_empty());
    });
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
fn pid_error_settle_tick_waits_for_inactive_timer_then_clears() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        runtime.set_motor_enabled_for_test(true);
        tmc_state.step_width_value.store(16, Relaxed);
        tmc_state.pid_error_sums_clear.store(true, Relaxed);
        stepper_state.timer_active_mirror.store(true, Relaxed);

        let mut busy_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_pid_error_settle_tick(
                &mut busy_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
            )
            .await;

        assert!(!tmc_state.pid_error_sums_clear.load(Relaxed));
        assert_eq!(
            runtime.pid_error_clear_countdown_for_test(),
            crate::tmc_control::HOLD_SETTLE_DELAY_MS
        );
        assert!(busy_tmc.into_inner().writes.is_empty());

        stepper_state.timer_active_mirror.store(false, Relaxed);
        runtime.set_pid_error_clear_countdown_ms(1);
        let mut settle_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000).with_read(1_000));

        runtime
            .apply_pid_error_settle_tick(
                &mut settle_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
            )
            .await;

        assert_eq!(runtime.pid_error_clear_countdown_for_test(), 0);
        let raw_spi = settle_tmc.into_inner();
        assert!(raw_spi.attempted_write_to(PID_POSITION_ACTUAL));
        assert!(raw_spi.attempted_write_to(STATUS_FLAGS));
    });
}

#[test]
fn homing_channel_tick_suppresses_position_i_without_global_advance() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.position_p_value.store(123, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                1,
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_P_POSITION_I.addr(), 123u32 << 16)]
        );
    });
}

#[test]
fn homing_channel_tick_preserves_openffboard_ungated_suppression() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.position_p_value.store(77, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                1,
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_P_POSITION_I.addr(), 77u32 << 16)]
        );
    });
}

#[test]
fn homing_channel_tick_restores_position_i_and_clears_integrator() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.position_p_value.store(12, Relaxed);
        tmc_state.position_i_value.store(34, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                3,
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        let writes = tmc.into_inner().writes;
        assert!(writes.contains(&(PID_POSITION_P_POSITION_I.addr(), (12u32 << 16) | 34)));
        assert!(writes.contains(&(
            PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
            PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
        )));
        assert!(writes.contains(&(PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0)));
    });
}

#[test]
fn homing_channel_tick_preserves_openffboard_ungated_restore() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.position_p_value.store(7, Relaxed);
        tmc_state.position_i_value.store(9, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                3,
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        let writes = tmc.into_inner().writes;
        assert!(writes.contains(&(PID_POSITION_P_POSITION_I.addr(), (7u32 << 16) | 9)));
        assert!(writes.contains(&(
            PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
            PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
        )));
        assert!(writes.contains(&(PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0)));
    });
}

#[test]
fn homing_channel_tick_restore_failure_blocks_advance() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let raw_spi =
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                1,
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::RetryCurrentState);
        assert!(runtime.phase_advance_active_for_test());
        assert!(
            !tmc.into_inner()
                .attempted_write_to(PID_POSITION_P_POSITION_I)
        );
    });
}

#[test]
fn advance_homing_state_after_channels_waits_for_all_channels() {
    let homing_state = core::sync::atomic::AtomicU8::new(1);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);

    TmcControlRuntime::advance_homing_state_after_channels(homing, 1, false);
    assert_eq!(homing_state.load(Relaxed), 1);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 1, true);
    assert_eq!(homing_state.load(Relaxed), 2);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 2, true);
    assert_eq!(homing_state.load(Relaxed), 2);

    homing_triggered.store(0, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 2, true);
    assert_eq!(homing_state.load(Relaxed), 3);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(homing_state.load(Relaxed), 0);
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
fn minimum_control_period_us_selects_fastest_runtime() {
    let mut runtime0 = TmcControlRuntime::new_for_channel(0);
    let mut runtime1 = TmcControlRuntime::new_for_channel(1);
    let tmc0 = crate::state::tmc::TmcChannelState::new();
    let tmc1 = crate::state::tmc::TmcChannelState::new();
    let stepper0 = crate::state::stepper::StepperChannelState::new();
    let stepper1 = crate::state::stepper::StepperChannelState::new();

    runtime0.set_motor_enabled_for_test(true);
    runtime0.set_velocity_scale_for_test(84_000_000);
    runtime1.set_motor_enabled_for_test(true);
    runtime1.set_velocity_scale_for_test(84_000_000);
    tmc0.velocity_transient_feedforward.store(true, Relaxed);
    tmc0.velocity_transient_rate_hz.store(10_000, Relaxed);
    tmc1.velocity_transient_feedforward.store(true, Relaxed);
    tmc1.velocity_transient_rate_hz.store(2_000, Relaxed);

    let runtimes = [runtime0, runtime1];
    let refs = [
        TmcControlStateRefs::new(&tmc0, &stepper0),
        TmcControlStateRefs::new(&tmc1, &stepper1),
    ];

    assert_eq!(
        TmcControlRuntime::minimum_control_period_us(&runtimes, &refs),
        100
    );
}

#[test]
fn minimum_control_period_us_defaults_to_one_millisecond_when_empty() {
    let runtimes: [TmcControlRuntime; 0] = [];
    let refs: [TmcControlStateRefs<'_>; 0] = [];

    assert_eq!(
        TmcControlRuntime::minimum_control_period_us(&runtimes, &refs),
        1_000
    );
}

#[test]
fn shared_slow_control_accumulator_carries_overshoot() {
    let mut accum_us = 0u16;

    assert!(!TmcControlRuntime::update_slow_control_accumulator(
        &mut accum_us,
        400
    ));
    assert_eq!(accum_us, 400);
    assert!(TmcControlRuntime::update_slow_control_accumulator(
        &mut accum_us,
        700
    ));
    assert_eq!(accum_us, 100);
    assert!(!TmcControlRuntime::update_slow_control_accumulator(
        &mut accum_us,
        800
    ));
    assert_eq!(accum_us, 900);
}

#[test]
fn shared_slow_control_accumulator_fires_immediately_at_default_period() {
    let mut accum_us = 750u16;

    assert!(TmcControlRuntime::update_slow_control_accumulator(
        &mut accum_us,
        1_000
    ));
    assert_eq!(accum_us, 0);
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
