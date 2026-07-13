use super::*;
use crate::commission_dispatch::test_support::{MockBackend, accepted_commission_result};
use crate::commission_dispatch::{PostActions, ReplyPayload, StepperPostAction};
use crate::commissioning::CommissioningEngine;
use crate::config::TmcConfig;
use crate::pid::{InnerLoopGains, OuterLoopGains};
use crate::tmc::{TmcRequest, TmcRequestKind};
use crate::tmc_control::dispatch::{SharedTmcDispatch, apply_shared_tmc_request};
use crate::tmc_control::helpers::{
    LiveGainWriteStatus, apply_live_inner_gains, apply_live_outer_gains, apply_motion_scale,
    refresh_velocity_scale_from_state,
};
use crate::tmc_control::request::live_gain_failure_outcome;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_hal_async::spi::{ErrorKind, ErrorType, Operation, SpiDevice};
use foci_tmc4671::registers::{
    ABN_DECODER_MODE, ABN_DECODER_PHI_E_PHI_M_OFFSET, ABN_DECODER_PPR, CONFIG_BIQUAD_T_ENABLE,
    PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM,
    PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_FLUX_P_FLUX_I, PID_POSITION_ACTUAL,
    PID_POSITION_P_POSITION_I, PID_POSITION_TARGET, PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET,
    PID_TORQUE_P_TORQUE_I, PID_VELOCITY_OFFSET, PID_VELOCITY_P_VELOCITY_I, PIDOUT_UQ_UD_LIMITS,
    STATUS_FLAGS, STEP_WIDTH,
};
use foci_tmc4671::spi::Tmc4671Spi;
use std::vec::Vec;

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
    failed_write_addresses: std::vec::Vec<u8>,
}

impl RuntimeMockSpi {
    fn with_read(mut self, value: u32) -> Self {
        self.reads.push_back(value);
        self
    }

    fn fail_writes_to(mut self, addr: u8) -> Self {
        self.failed_write_addresses.push(addr);
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
                    if self.failed_write_addresses.contains(&addr) {
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

fn assert_no_actions(actions: &BoardActions) {
    assert_eq!(actions.len(), 0, "expected no board actions");
}

fn assert_single_stop_now(actions: &BoardActions) {
    assert_eq!(actions.len(), 1, "expected one StopNow action");
    assert_stop_now_at(actions, 0);
}

fn assert_stop_now_at(actions: &BoardActions, index: usize) {
    match &actions[index] {
        BoardAction::Stepper(StepperAction::StopNow) => {}
        BoardAction::Stepper(_) => panic!("expected StopNow stepper action at index {index}"),
        BoardAction::Shutdown(_) => panic!("expected StopNow, got Shutdown at index {index}"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected StopNow, got SignalEmergencyDisable at index {index}")
        }
        BoardAction::Reply(_) => panic!("expected StopNow, got Reply at index {index}"),
        BoardAction::DelegatedDiagnostic(_) => {
            panic!("expected StopNow, got DelegatedDiagnostic at index {index}")
        }
    }
}

fn assert_shutdown_at(actions: &BoardActions, index: usize, reason: ShutdownReason) {
    match &actions[index] {
        BoardAction::Shutdown(actual) => assert_eq!(*actual, reason),
        BoardAction::Stepper(_) => panic!("expected Shutdown at index {index}, got Stepper"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected Shutdown at index {index}, got SignalEmergencyDisable")
        }
        BoardAction::Reply(_) => panic!("expected Shutdown at index {index}, got Reply"),
        BoardAction::DelegatedDiagnostic(_) => {
            panic!("expected Shutdown at index {index}, got DelegatedDiagnostic")
        }
    }
}

fn assert_single_shutdown(actions: &BoardActions, reason: ShutdownReason) {
    assert_eq!(actions.len(), 1, "expected one Shutdown action");
    assert_shutdown_at(actions, 0, reason);
}

fn assert_stop_then_shutdown(actions: &BoardActions, reason: ShutdownReason) {
    assert_eq!(actions.len(), 2, "expected StopNow then Shutdown actions");
    assert_stop_now_at(actions, 0);
    assert_shutdown_at(actions, 1, reason);
}

fn assert_single_signal_emergency_disable(actions: &BoardActions) {
    assert_eq!(
        actions.len(),
        1,
        "expected one SignalEmergencyDisable action"
    );
    match &actions[0] {
        BoardAction::SignalEmergencyDisable => {}
        BoardAction::Stepper(_) => panic!("expected SignalEmergencyDisable, got Stepper"),
        BoardAction::Shutdown(_) => panic!("expected SignalEmergencyDisable, got Shutdown"),
        BoardAction::Reply(_) => panic!("expected SignalEmergencyDisable, got Reply"),
        BoardAction::DelegatedDiagnostic(_) => {
            panic!("expected SignalEmergencyDisable, got DelegatedDiagnostic")
        }
    }
}

fn assert_single_arm_preserve_position(actions: &BoardActions) {
    assert_eq!(actions.len(), 1, "expected one ArmCheck action");
    match &actions[0] {
        BoardAction::Stepper(StepperAction::ArmCheck { post_action }) => {
            assert_eq!(
                *post_action,
                crate::commission_dispatch::StepperPostAction::ArmPreservePosition
            );
        }
        BoardAction::Stepper(_) => panic!("expected ArmCheck action"),
        BoardAction::Shutdown(_) => panic!("expected ArmCheck, got Shutdown"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected ArmCheck, got SignalEmergencyDisable")
        }
        BoardAction::Reply(_) => panic!("expected ArmCheck, got Reply"),
        BoardAction::DelegatedDiagnostic(_) => {
            panic!("expected ArmCheck, got DelegatedDiagnostic")
        }
    }
}

fn assert_single_delegated_request(actions: &BoardActions, expected: &TmcRequest) {
    assert_eq!(actions.len(), 1, "expected one DelegatedDiagnostic action");
    match &actions[0] {
        BoardAction::DelegatedDiagnostic(delegated) => {
            assert_eq!(&delegated.request, expected);
        }
        BoardAction::Stepper(_) => panic!("expected DelegatedDiagnostic, got Stepper"),
        BoardAction::Shutdown(_) => panic!("expected DelegatedDiagnostic, got Shutdown"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected DelegatedDiagnostic, got SignalEmergencyDisable")
        }
        BoardAction::Reply(_) => panic!("expected DelegatedDiagnostic, got Reply"),
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
    assert_eq!(
        TmcControlRequestOutcome::fall_through(),
        TmcControlRequestOutcome {
            tail: TmcControlRequestTail::FallThrough,
            shutdown: None,
            should_refresh_velocity_scale: false,
        }
    );
    assert_eq!(
        TmcControlRequestOutcome::skip_tick_tail(),
        TmcControlRequestOutcome {
            tail: TmcControlRequestTail::SkipTickTail,
            shutdown: None,
            should_refresh_velocity_scale: false,
        }
    );
    assert_eq!(
        TmcControlRequestOutcome::shutdown(
            ShutdownReason::RequestValidationFailed,
            TmcControlRequestTail::FallThrough,
        ),
        TmcControlRequestOutcome {
            tail: TmcControlRequestTail::FallThrough,
            shutdown: Some(ShutdownReason::RequestValidationFailed),
            should_refresh_velocity_scale: false,
        }
    );
    assert_eq!(
        TmcControlRequestOutcome::refresh_velocity_scale(TmcControlRequestTail::SkipTickTail),
        TmcControlRequestOutcome {
            tail: TmcControlRequestTail::SkipTickTail,
            shutdown: None,
            should_refresh_velocity_scale: true,
        }
    );
    assert_eq!(
        TmcControlRequestOutcome::shutdown_then_refresh_velocity_scale(
            ShutdownReason::InvalidTmcChannel,
            TmcControlRequestTail::FallThrough,
        ),
        TmcControlRequestOutcome {
            tail: TmcControlRequestTail::FallThrough,
            shutdown: Some(ShutdownReason::InvalidTmcChannel),
            should_refresh_velocity_scale: true,
        }
    );
}

#[test]
fn refresh_velocity_scale_caches_exact_planner_scale() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc_state = crate::state::tmc::tmc_channel0();
    let stepper_state = crate::state::stepper::stepper_channel0();
    tmc_state.reset();
    stepper_state.reset();
    let scale = tmc_state
        .configure_motion_scale(stepper_state, 1000, 3200)
        .unwrap();

    refresh_velocity_scale_from_state(&mut runtime, tmc_state, 84_000_000, 25_000);

    assert_eq!(
        runtime.planner_velocity_scale(),
        scale.planner_velocity_scale(84_000_000, 25_000)
    );
    tmc_state.reset();
    stepper_state.reset();
}

#[test]
fn apply_live_inner_gains_publishes_only_after_both_writes_succeed() {
    pollster::block_on(async {
        let previous = InnerLoopGains {
            flux_p: 1,
            flux_i: 2,
            torque_p: 3,
            torque_i: 4,
        };
        let requested = InnerLoopGains {
            flux_p: 11,
            flux_i: 12,
            torque_p: 13,
            torque_i: 14,
        };
        let cases = [
            (false, false, true, true),
            (true, false, false, true),
            (false, true, true, false),
            (true, true, false, false),
        ];

        for (fail_flux, fail_torque, expected_first_ok, expected_second_ok) in cases {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_stored_gains(previous);
            let mut spi = RuntimeMockSpi::default();
            if fail_flux {
                spi = spi.fail_writes_to(PID_FLUX_P_FLUX_I.addr());
            }
            if fail_torque {
                spi = spi.fail_writes_to(PID_TORQUE_P_TORQUE_I.addr());
            }
            let mut tmc = Tmc4671Spi::new(spi);

            let status = apply_live_inner_gains(&mut runtime, &mut tmc, requested).await;
            let spi = tmc.into_inner();

            assert_eq!(status.first_write_ok, expected_first_ok);
            assert_eq!(status.second_write_ok, expected_second_ok);
            assert_eq!(spi.write_attempts.len(), 2);
            assert_eq!(
                spi.write_attempts,
                std::vec![
                    (
                        PID_FLUX_P_FLUX_I.addr(),
                        crate::pid::pack_pid_register(11, 12),
                    ),
                    (
                        PID_TORQUE_P_TORQUE_I.addr(),
                        crate::pid::pack_pid_register(13, 14),
                    ),
                ],
            );
            assert_eq!(
                runtime.stored_gains(),
                if status.all_ok() {
                    Some(requested)
                } else {
                    Some(previous)
                },
            );
        }
    });
}

#[test]
fn apply_live_outer_gains_publishes_only_after_both_writes_succeed() {
    pollster::block_on(async {
        let previous = OuterLoopGains {
            position_p: 101,
            position_i: 102,
            velocity_p: 103,
            velocity_i: 104,
        };
        let requested = OuterLoopGains {
            position_p: 201,
            position_i: 202,
            velocity_p: 203,
            velocity_i: 204,
        };
        let cases = [
            (false, false, true, true),
            (true, false, false, true),
            (false, true, true, false),
            (true, true, false, false),
        ];

        for (fail_velocity, fail_position, expected_first_ok, expected_second_ok) in cases {
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            tmc_state
                .position_p_value
                .store(previous.position_p, Relaxed);
            tmc_state
                .position_i_value
                .store(previous.position_i, Relaxed);
            tmc_state
                .velocity_p_value
                .store(previous.velocity_p, Relaxed);
            tmc_state
                .velocity_i_value
                .store(previous.velocity_i, Relaxed);
            let mut spi = RuntimeMockSpi::default();
            if fail_velocity {
                spi = spi.fail_writes_to(PID_VELOCITY_P_VELOCITY_I.addr());
            }
            if fail_position {
                spi = spi.fail_writes_to(PID_POSITION_P_POSITION_I.addr());
            }
            let mut tmc = Tmc4671Spi::new(spi);

            let status = apply_live_outer_gains(&tmc_state, &mut tmc, requested).await;
            let spi = tmc.into_inner();
            let published = OuterLoopGains {
                position_p: tmc_state.position_p_value.load(Relaxed),
                position_i: tmc_state.position_i_value.load(Relaxed),
                velocity_p: tmc_state.velocity_p_value.load(Relaxed),
                velocity_i: tmc_state.velocity_i_value.load(Relaxed),
            };

            assert_eq!(status.first_write_ok, expected_first_ok);
            assert_eq!(status.second_write_ok, expected_second_ok);
            assert_eq!(spi.write_attempts.len(), 2);
            assert_eq!(
                spi.write_attempts,
                std::vec![
                    (
                        PID_VELOCITY_P_VELOCITY_I.addr(),
                        crate::pid::pack_pid_register(203, 204),
                    ),
                    (
                        PID_POSITION_P_POSITION_I.addr(),
                        crate::pid::pack_pid_register(201, 202),
                    ),
                ],
            );
            assert_eq!(
                published,
                if status.all_ok() { requested } else { previous },
            );
        }
    });
}

#[test]
fn live_gain_failure_outcome_maps_incomplete_pairs_to_shutdown() {
    let cases = [
        (true, true, None),
        (
            false,
            true,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
        (
            true,
            false,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
        (
            false,
            false,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
    ];

    for (first_write_ok, second_write_ok, expected) in cases {
        let status = LiveGainWriteStatus {
            first_write_ok,
            second_write_ok,
        };

        assert_eq!(live_gain_failure_outcome(status), expected);
    }
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
            3999,
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
fn apply_motion_scale_writes_encoder_then_physical_width_and_publishes_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let scale = crate::motion_scale::MotionScale::new(1000, 3200).unwrap();

        let result = apply_motion_scale(&mut tmc, &tmc_state, &stepper_state, scale, |_| {
            Ok::<(), core::convert::Infallible>(())
        })
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(()));
        assert_eq!(
            spi.write_attempts,
            std::vec![(ABN_DECODER_PPR.addr(), 4000), (STEP_WIDTH.addr(), 16)]
        );
        assert_eq!(tmc_state.encoder_counts_per_rev(), Some(4000));
        assert_eq!(stepper_state.planner_steps_per_rev(&tmc_state), Some(3200));
        assert_eq!(tmc_state.physical_step_width(), Some(16));
    });
}

#[test]
fn apply_motion_scale_spi_failures_leave_scale_unconfigured() {
    pollster::block_on(async {
        for failed_register in [ABN_DECODER_PPR.addr(), STEP_WIDTH.addr()] {
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            let stepper_state = crate::state::stepper::StepperChannelState::new();
            let mut tmc =
                Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(failed_register));
            let scale = crate::motion_scale::MotionScale::new(1000, 3200).unwrap();

            let expected = if failed_register == ABN_DECODER_PPR.addr() {
                crate::motion_scale::MotionScaleRejection::EncoderRegisterWriteFailed
            } else {
                crate::motion_scale::MotionScaleRejection::StepWidthRegisterWriteFailed
            };
            assert_eq!(
                apply_motion_scale(&mut tmc, &tmc_state, &stepper_state, scale, |_| {
                    Ok::<(), core::convert::Infallible>(())
                })
                .await,
                Err(expected)
            );
            assert!(!tmc_state.motion_scale_configured());
            assert_eq!(tmc_state.physical_step_width(), None);
        }
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
            3999,
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
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Err(()));
        assert!(spi.attempted_write_to(PIDOUT_UQ_UD_LIMITS));
    });
}

#[test]
fn shared_dispatch_set_torque_filter_writes_biquad_and_updates_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetTorqueFilter { filter_hz: 3000 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert!(spi.write_attempts.contains(&(
            CONFIG_BIQUAD_T_ENABLE.addr.addr(),
            CONFIG_BIQUAD_T_ENABLE.sub_addr as u32,
        )));
        assert!(
            spi.write_attempts
                .contains(&(CONFIG_BIQUAD_T_ENABLE.data.addr(), 1))
        );
        assert_eq!(tmc_state.torque_filter_hz.load(Relaxed), Some(3000));
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn shared_dispatch_set_torque_filter_zero_disables_filter() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.torque_filter_hz.store(Some(3000), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetTorqueFilter { filter_hz: 0 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert!(spi.write_attempts.contains(&(
            CONFIG_BIQUAD_T_ENABLE.addr.addr(),
            CONFIG_BIQUAD_T_ENABLE.sub_addr as u32,
        )));
        assert!(
            spi.write_attempts
                .contains(&(CONFIG_BIQUAD_T_ENABLE.data.addr(), 0))
        );
        assert_eq!(tmc_state.torque_filter_hz.load(Relaxed), Some(0));
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
            3999,
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
fn scheduled_step_preserves_legacy_deadline_without_consuming_engine() {
    pollster::block_on(async {
        let tmc = crate::state::tmc::tmc_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 42_000;
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));
        assert!(matches!(
            runtime
                .step_commissioning_scheduled(&mut backend)
                .await
                .expect("engine should be active"),
            crate::commissioning::schedule::CommissionPoll::At {
                timestamp_us: 43_000,
                event: None,
            }
        ));
    });
}

#[test]
fn scheduled_dispatch_preserves_wait_action() {
    pollster::block_on(async {
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

        let action = runtime
            .drive_commissioning_poll(
                crate::commissioning::schedule::CommissionPoll::At {
                    timestamp_us: 43_000,
                    event: None,
                },
                &mut backend,
                tmc,
                stepper,
                &config,
            )
            .await
            .expect("engine should be active");

        assert!(matches!(
            action,
            ScheduledCommissionTickAction::At {
                timestamp_us: 43_000,
                event: None,
            }
        ));
        assert!(runtime.is_commission_active());
    });
}

#[test]
fn scheduled_classification_is_synchronous_for_at_action() {
    let tmc = crate::state::tmc::tmc_channel0();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

    let action = runtime
        .classify_commission_poll(crate::commissioning::schedule::CommissionPoll::At {
            timestamp_us: 43_000,
            event: None,
        })
        .expect("engine should be active");

    assert!(matches!(
        action,
        ScheduledCommissionTickAction::At {
            timestamp_us: 43_000,
            event: None,
        }
    ));
}

#[test]
fn runtime_appends_board_diagnostic_to_active_commission() {
    let tmc = crate::state::tmc::tmc_channel0();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));
    let diagnostic = crate::selftest::CommissionDiagnostic {
        phase: crate::autotune::PhaseId::ElectricalId.to_u8(),
        code: 40,
        status: 2,
        value0: 100,
        value1: 491,
        value2: 1,
    };

    assert!(runtime.push_commission_diagnostic(diagnostic));

    let mut engine = runtime
        .take_commission_engine()
        .expect("commission engine remains active");
    assert_eq!(
        engine.take_diagnostics().iter().collect::<Vec<_>>(),
        [diagnostic]
    );
}

#[test]
fn runtime_rejects_board_diagnostic_without_active_commission() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    assert!(!runtime.push_commission_diagnostic(crate::selftest::CommissionDiagnostic::default()));
}

#[test]
fn scheduled_dispatch_preserves_phase_event_context_and_deadline() {
    pollster::block_on(async {
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_commission(
            37,
            crate::autotune::CommissionProfile::Balanced,
        ));
        let poll = crate::commissioning::schedule::adapt_step_result(
            crate::commissioning::types::StepResult::PhaseChanged(
                crate::autotune::PhaseId::ElectricalId,
            ),
            81_250,
            Some((37, crate::commissioning::types::EngineKind::Commission)),
        )
        .expect("phase change should produce a scheduled poll");

        assert!(matches!(
            &poll,
            crate::commissioning::schedule::CommissionPoll::At {
                timestamp_us: 82_250,
                event: Some(
                    crate::commissioning::schedule::CommissionEvent::PhaseChanged {
                        oid: 37,
                        phase_id: crate::autotune::PhaseId::ElectricalId,
                        engine_kind: crate::commissioning::types::EngineKind::Commission,
                    }
                ),
            }
        ));

        let action = runtime
            .drive_commissioning_poll(poll, &mut backend, tmc, stepper, &config)
            .await
            .expect("engine should be active");

        assert!(matches!(
            action,
            ScheduledCommissionTickAction::At {
                timestamp_us: 82_250,
                event: Some(
                    crate::commissioning::schedule::CommissionEvent::PhaseChanged {
                        oid: 37,
                        phase_id: crate::autotune::PhaseId::ElectricalId,
                        engine_kind: crate::commissioning::types::EngineKind::Commission,
                    }
                ),
            }
        ));
        assert!(runtime.is_commission_active());
    });
}

#[test]
fn scheduled_dispatch_preserves_terminal_reply_and_actions() {
    pollster::block_on(async {
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

        let action = runtime
            .drive_commissioning_poll(
                crate::commissioning::schedule::CommissionPoll::Terminal {
                    result: crate::commissioning::schedule::CommissionTerminal::SelftestDone,
                },
                &mut backend,
                tmc,
                stepper,
                &config,
            )
            .await
            .expect("terminal dispatch should produce an action");

        match action {
            ScheduledCommissionTickAction::Terminal {
                reply: ReplyPayload::Selftest { oid: 7, .. },
                actions,
            } => {
                assert_eq!(actions.stop_trace, PostActions::NONE.stop_trace);
                assert_eq!(actions.stepper, PostActions::NONE.stepper);
            }
            _ => panic!("expected Selftest terminal action"),
        }
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn scheduled_dispatch_preserves_accepted_commission_reply_and_actions() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;

        const MOTOR_SENTINEL: u32 = 0x0002_0032;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.reset();
        stepper.reset();
        tmc.current_limit_reg.store(0x0010_0010, Ordering::Relaxed);
        tmc.motor_sentinel.store(MOTOR_SENTINEL, Ordering::Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                MOTOR_SENTINEL,
            )
            .await
            .expect("test motor sentinel write should succeed");
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_commission(
            23,
            crate::autotune::CommissionProfile::Balanced,
        ));

        let action = runtime
            .drive_commissioning_poll(
                crate::commissioning::schedule::CommissionPoll::Terminal {
                    result: crate::commissioning::schedule::CommissionTerminal::CommissionDone(
                        accepted_commission_result(),
                    ),
                },
                &mut backend,
                tmc,
                stepper,
                &config,
            )
            .await
            .expect("terminal dispatch should produce an action");

        match action {
            ScheduledCommissionTickAction::Terminal {
                reply:
                    ReplyPayload::CommissionResult {
                        oid,
                        status,
                        result,
                    },
                actions,
            } => {
                assert_eq!(oid, 23);
                assert_eq!(status, 0);
                assert_eq!(result.inner.r_int, 1706);
                assert_eq!(result.inner.l_int, 1245);
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmZeroPosition);
            }
            _ => panic!("expected accepted CommissionResult terminal action"),
        }
        assert!(!runtime.is_commission_active());
        assert!(runtime.is_calibrated());
        assert!(runtime.is_motor_enabled());
        assert!(runtime.stored_gains().is_some());
        assert!(backend.enable_pin);
        assert!(!tmc.motor_armed.load(Ordering::Relaxed));
        assert!(!tmc.commission_in_flight.load(Ordering::Relaxed));

        tmc.reset();
        stepper.reset();
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
    runtime.set_planner_velocity_scale_for_test(1234);
    runtime.set_slow_control_accum_us_for_test(750);

    runtime.reset_runtime_state();

    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_motor_enabled());
    assert_eq!(runtime.planner_velocity_scale(), None);
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
fn queue_flush_without_signal_does_not_mutate_state() {
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

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert_no_actions(&actions);
    assert!(!queue_flush.load(Relaxed));
    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());
    assert_eq!(tmc.current_limit_reg.load(Relaxed), 88);
    assert_eq!(stepper.position.load(Relaxed), 55);
    assert!(fault_active.load(Relaxed));
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
    assert_single_stop_now(&actions);
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
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .expect("motion scale should configure");
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
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![
                (PID_POSITION_ACTUAL.addr(), 1_000),
                (
                    PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
                (STATUS_FLAGS.addr(), 0),
            ]
        );
    });
}

#[test]
fn slow_maintenance_before_status_cadence_returns_empty_without_status_read() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let homing_state = core::sync::atomic::AtomicU8::new(0);
        let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
        let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
        runtime.set_status_poll_counter_for_test(998);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1 << 26));

        let actions = runtime
            .apply_slow_maintenance_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                homing,
            )
            .await;

        assert_no_actions(&actions);
        assert!(!tmc.into_inner().read_requested(STATUS_FLAGS));
    });
}

#[test]
fn slow_maintenance_status_poll_non_fault_returns_empty() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let homing_state = core::sync::atomic::AtomicU8::new(0);
        let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
        let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
        runtime.set_status_poll_counter_for_test(999);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(0));

        let actions = runtime
            .apply_slow_maintenance_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                homing,
            )
            .await;

        assert_no_actions(&actions);
        assert!(tmc.into_inner().read_requested(STATUS_FLAGS));
    });
}

#[test]
fn slow_maintenance_status_fault_emits_emergency_disable() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let homing_state = core::sync::atomic::AtomicU8::new(0);
        let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
        let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
        runtime.set_status_poll_counter_for_test(999);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1 << 26));

        let actions = runtime
            .apply_slow_maintenance_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                homing,
            )
            .await;

        assert_single_signal_emergency_disable(&actions);
        assert!(tmc.into_inner().read_requested(STATUS_FLAGS));
    });
}

#[test]
fn slow_maintenance_homing_retry_skips_status_poll() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let homing_state = core::sync::atomic::AtomicU8::new(1);
        let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
        let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        runtime.set_status_poll_counter_for_test(999);
        let raw_spi =
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);

        let actions = runtime
            .apply_slow_maintenance_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                homing,
            )
            .await;

        assert_no_actions(&actions);
        assert_eq!(homing_state.load(Relaxed), 1);
        let raw_spi = tmc.into_inner();
        assert!(raw_spi.attempted_write_to(ABN_DECODER_PHI_E_PHI_M_OFFSET));
        assert!(!raw_spi.read_requested(STATUS_FLAGS));
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
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![
                (PID_POSITION_P_POSITION_I.addr(), (12u32 << 16) | 34),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
            ]
        );
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
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![
                (PID_POSITION_P_POSITION_I.addr(), (7u32 << 16) | 9),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
            ]
        );
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
    runtime.set_planner_velocity_scale_for_test(123_456);
    runtime.set_feedforward_was_active(true);
    runtime.set_pid_error_clear_countdown_ms(9);

    let actions = runtime.prepare_shutdown_quiesce();

    assert_single_stop_now(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_commission_active());
    assert!(!runtime.feedforward_was_active());
    assert_eq!(runtime.pid_error_clear_countdown_ms(), 0);
    assert_eq!(runtime.stored_gains(), Some(gains));
    assert_eq!(
        runtime.planner_velocity_scale(),
        Some(crate::motion_scale::PlannerVelocityScale::from_reduced(
            123_456, 1
        ))
    );
}

#[test]
fn shutdown_quiesce_clears_torque_flux_offset_activity() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.current_limit_reg.store(10_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(&mut active_tmc, refs, &config, false, false, true)
                .await,
            Ok(())
        );
        assert!(
            active_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET)
        );

        assert_single_stop_now(&runtime.prepare_shutdown_quiesce());
        tmc_state.accel_feedforward.store(false, Relaxed);
        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert!(
            !inactive_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET),
            "shutdown quiesce should clear the offset-active latch so the next inactive tick does not emit a cleanup write"
        );
    });
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
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(7));

    let first = runtime.poll_shutdown(&signals);

    assert_single_stop_now(&first);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_commission_active());

    let second = runtime.poll_shutdown(&signals);
    assert_no_actions(&second);

    shutdown_active.store(false, Relaxed);
    assert_no_actions(&runtime.poll_shutdown(&signals));

    shutdown_active.store(true, Relaxed);
    let third = runtime.poll_shutdown(&signals);
    assert_single_stop_now(&third);
}

#[test]
fn emergency_disable_without_signal_is_empty() {
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
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let actions = runtime.poll_emergency_disable(&signals, 1 << 26);

    assert_no_actions(&actions);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
    assert!(fault_active.load(Relaxed));
}

#[test]
fn emergency_disable_consumes_signal_and_orders_stop_before_shutdown() {
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
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    disable_signal.signal(());
    let clipped = runtime.poll_emergency_disable(&signals, 1 << 26);

    assert_stop_then_shutdown(&clipped, ShutdownReason::AdcCurrentClipped);
    assert!(disable_signal.try_take().is_none());
    assert!(!fault_active.load(Relaxed));
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    fault_active.store(true, Relaxed);
    disable_signal.signal(());
    let non_clipping_fault = runtime.poll_emergency_disable(&signals, 0);

    assert_stop_then_shutdown(&non_clipping_fault, ShutdownReason::TmcFault);
    assert!(disable_signal.try_take().is_none());
    assert!(!fault_active.load(Relaxed));
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn same_channel_regular_request_is_unhandled_without_mutating_runtime() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(8));
    let req = TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 750 });

    let actions = runtime.classify_request(req);

    assert_no_actions(&actions);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
    assert!(runtime.is_commission_active());
}

#[test]
fn wrong_channel_request_emits_only_invalid_channel_shutdown_without_mutation() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(9));
    let req = TmcRequest::new(1, TmcRequestKind::SetEnable(false));

    let actions = runtime.classify_request(req);

    assert_single_shutdown(&actions, ShutdownReason::InvalidTmcChannel);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
    assert!(runtime.is_commission_active());
}

#[test]
fn all_active_diagnostic_requests_are_delegated_without_mutation() {
    let requests = [
        crate::tmc::tmc_current_step_test(0, 10, crate::tmc::CurrentStepAxis::Torque, 111, 12, 13),
        crate::tmc::tmc_current_vector_step_test(0, 11, 112, -113, 14, 15),
        crate::tmc::tmc_current_torque_sample_test(0, 12, 114, -115, 16, 17),
        crate::tmc::tmc_position_torque_offset_sample_test(0, 13, 116, 18, 19),
        crate::tmc::tmc_voltage_step_test(0, 14, 120, -121, 20),
        crate::tmc::tmc_resistance_test(0, 15, 0),
    ];

    for req in requests {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_calibrated_for_test(true);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_commission_engine(CommissioningEngine::new_calibrate(10));
        let expected = req.clone();

        let actions = runtime.classify_request(req);

        assert_single_delegated_request(&actions, &expected);
        assert!(runtime.is_motor_enabled());
        assert!(runtime.is_calibrated());
        assert!(runtime.is_commission_active());
    }
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

    assert_no_actions(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_before_calibration_with_auto_calibrate_gate_still_does_not_energize() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    tmc.auto_calibrate_on_enable_allowed.store(true, Relaxed);

    let actions = runtime.prepare_enable_request(&tmc);

    assert_no_actions(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_after_calibration_emits_only_arm_preserve_position_check() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    runtime.set_calibrated_for_test(true);

    let actions = runtime.prepare_enable_request(&tmc);

    assert_single_arm_preserve_position(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
}

#[test]
fn enable_when_already_enabled_is_noop() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let actions = runtime.prepare_enable_request(&tmc);

    assert_no_actions(&actions);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
}

#[test]
fn disable_request_stops_and_clears_runtime_flags() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let actions = runtime.prepare_disable_request();

    assert_single_stop_now(&actions);
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
    runtime0.set_planner_velocity_scale_for_test(84_000_000);
    runtime1.set_motor_enabled_for_test(true);
    runtime1.set_planner_velocity_scale_for_test(84_000_000);
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
fn feedforward_tick_writes_velocity_offset_and_marks_active() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state.velocity_feedforward_multiplier.store(40, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let expected = crate::tmc::compute_planner_velocity_offset_scaled(
            crate::motion_scale::PlannerVelocityScale::from_reduced(84_000_000, 1),
            2_800,
            false,
            40,
        ) as u32;
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), expected)]
        );
        assert!(runtime.feedforward_was_active());
    });
}

#[test]
fn feedforward_tick_clears_velocity_offset_once_after_deactivation() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        runtime.set_feedforward_was_active(true);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut first_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            first_tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), 0)]
        );
        assert!(!runtime.feedforward_was_active());

        let mut second_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut second_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(second_tmc.into_inner().writes.is_empty());
    });
}

#[test]
fn feedforward_tick_retries_failed_velocity_offset_clear() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state.velocity_feedforward_multiplier.store(40, Relaxed);
        tmc_state
            .velocity_transient_feedforward
            .store(true, Relaxed);
        tmc_state
            .velocity_transient_lead_time_us
            .store(1_000, Relaxed);
        tmc_state
            .velocity_transient_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .velocity_transient_max_offset
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut active_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        let active_writes = active_tmc.into_inner().writes;
        assert_eq!(active_writes.len(), 1);
        let active_offset = active_writes[0].1 as i32;
        assert_ne!(active_offset, 0);
        assert!(runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        let active_transient_offset = runtime.trace_state().cached_velocity_transient_offset;
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, active_offset);
            assert_ne!(active_transient_offset, 0);
        }

        tmc_state.velocity_feedforward.store(false, Relaxed);
        tmc_state
            .velocity_transient_feedforward
            .store(false, Relaxed);
        let mut failed_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PID_VELOCITY_OFFSET.addr()));

        let failed_tick = runtime
            .apply_feedforward_tick(
                &mut failed_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                false,
            )
            .await;

        assert_eq!(failed_tick, Err(()));
        assert!(runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, active_offset);
            assert_eq!(
                runtime.trace_state().cached_velocity_transient_offset,
                active_transient_offset
            );
        }

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        assert_eq!(
            retry_tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), 0)]
        );
        assert!(!runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, 0);
            assert_eq!(runtime.trace_state().cached_velocity_transient_offset, 0);
        }

        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        assert!(inactive_tmc.into_inner().writes.is_empty());
    });
}

#[test]
fn feedforward_tick_retries_failed_torque_flux_offset_clear() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.current_limit_reg.store(10_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut active_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        let active_writes = active_tmc.into_inner().writes;
        assert_eq!(active_writes.len(), 1);
        assert_eq!(active_writes[0].0, PID_TORQUE_FLUX_OFFSET.addr());
        let active_offset = ((active_writes[0].1 >> 16) as u16) as i16 as i32;
        assert_ne!(active_offset, 0);
        assert!(runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        let (active_gain, active_flags) = {
            let trace = runtime.trace_state();
            assert_eq!(trace.cached_accel_torque_offset, active_offset);
            assert_ne!(trace.cached_accel_gain_permille, 0);
            assert_ne!(trace.cached_accel_flags, 0);
            (trace.cached_accel_gain_permille, trace.cached_accel_flags)
        };

        tmc_state.accel_feedforward.store(false, Relaxed);
        let mut failed_tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr()),
        );

        let failed_tick = runtime
            .apply_feedforward_tick(
                &mut failed_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                true,
            )
            .await;

        assert_eq!(failed_tick, Err(()));
        assert!(runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        {
            assert_eq!(
                runtime.trace_state().cached_accel_torque_offset,
                active_offset
            );
            assert_eq!(
                runtime.trace_state().cached_accel_gain_permille,
                active_gain
            );
            assert_eq!(runtime.trace_state().cached_accel_flags, active_flags);
        }

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert_eq!(
            retry_tmc.into_inner().writes,
            std::vec![(PID_TORQUE_FLUX_OFFSET.addr(), 0)]
        );
        assert!(!runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_accel_torque_offset, 0);
            assert_eq!(runtime.trace_state().cached_accel_gain_permille, 0);
            assert_eq!(runtime.trace_state().cached_accel_flags, 0);
        }

        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(
            !inactive_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET)
        );
    });
}

#[test]
fn feedforward_fast_tick_skips_torque_flux_offset_slow_work() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.current_limit_reg.store(10_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );

        assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_OFFSET));
    });
}

#[test]
fn command_velocity_reference_zeroes_when_reset_is_requested() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
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
    runtime.set_planner_velocity_scale_for_test(84_000_000);
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
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(
        reference,
        crate::tmc::compute_planner_command_velocity_reference(
            crate::motion_scale::PlannerVelocityScale::from_reduced(84_000_000, 1),
            2_800,
            false,
        )
    );
}

#[test]
fn command_velocity_reference_zeroes_when_runtime_inactive() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(reference, 0);
}

#[test]
fn planner_velocity_scale_accessors_round_trip() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    assert_eq!(runtime.planner_velocity_scale(), None);
    runtime.set_planner_velocity_scale_for_test(123_456);
    assert_eq!(
        runtime.planner_velocity_scale(),
        Some(crate::motion_scale::PlannerVelocityScale::from_reduced(
            123_456, 1
        ))
    );
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
fn phase_advance_restore_via_backend_writes_base_and_clears_active_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();

        let restored = runtime
            .restore_phase_advance_via_backend(&mut backend)
            .await;

        assert!(restored);
        assert!(!runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
        assert_eq!(
            backend
                .registers
                .get(&ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
            Some(&0x1111_2222)
        );
    });
}

#[test]
fn phase_advance_restore_via_backend_failure_preserves_retry_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());

        let restored = runtime
            .restore_phase_advance_via_backend(&mut backend)
            .await;

        assert!(!restored);
        assert!(runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x3333_4444);
        assert_eq!(
            backend
                .registers
                .get(&ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
            None
        );
    });
}

#[test]
fn phase_advance_tick_snapshots_base_and_writes_adjusted_offset() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.phase_advance.store(true, Relaxed);
        tmc_state.phase_advance_gain_ppm.store(1_000, Relaxed);
        tmc_state.phase_advance_max_counts.store(20, Relaxed);
        tmc_state.phase_advance_deadband.store(0, Relaxed);
        let base_register = 0x0100_2345;
        let command_velocity_reference = 10_000;
        let phase_output = crate::tmc::compute_phase_advance_counts(
            command_velocity_reference,
            crate::tmc::PhaseAdvanceConfig {
                enable: true,
                gain_ppm: 1_000,
                max_counts: 20,
                deadband_velocity: 0,
            },
        );
        let expected_register = crate::tmc::pack_abn_phi_offset(base_register, phase_output.counts);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(base_register));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                command_velocity_reference,
                false,
                false,
            )
            .await;

        let raw_spi = tmc.into_inner();
        assert!(raw_spi.read_requested(ABN_DECODER_PHI_E_PHI_M_OFFSET));
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr(), expected_register)]
        );
        assert!(runtime.phase_advance_active_for_test());
        assert_eq!(
            runtime.phase_advance_last_register_for_test(),
            expected_register
        );
    });
}

#[test]
fn phase_advance_tick_reset_gate_skips_abn_snapshot_and_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.phase_advance.store(true, Relaxed);
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .unwrap();
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
fn phase_and_position_tick_disabled_runtime_writes_nothing() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.phase_advance.store(true, Relaxed);
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state.position_lead_gain_permille.store(1000, Relaxed);
        tmc_state.position_lead_max_counts.store(20, Relaxed);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .unwrap();
        stepper_state.interval.store(2_800, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                123,
                false,
                false,
            )
            .await;

        let raw_spi = tmc.into_inner();
        assert!(raw_spi.read_requests.is_empty());
        assert!(raw_spi.write_attempts.is_empty());
        assert_eq!(runtime.position_lead_applied_units_for_test(), 0);
    });
}

#[test]
fn position_lead_tick_uses_planner_position_units() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state.position_lead_gain_permille.store(1000, Relaxed);
        tmc_state.position_lead_max_counts.store(20, Relaxed);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .unwrap();
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let lead_counts =
            crate::tmc::compute_position_lead_counts_from_reference(30_000, false, 1000, 20);
        let (expected_target, expected_applied_units) =
            crate::tmc::compute_position_lead_target(1_000, 0, lead_counts, 3_200);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                30_000,
                false,
                false,
            )
            .await;

        assert_eq!(
            runtime.position_lead_applied_units_for_test(),
            expected_applied_units
        );
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_TARGET.addr(), expected_target)]
        );
    });
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_returns_sample_before_incremented_control_tick() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 0);
    assert_eq!(tick.control_tick, 1);
    assert!(!tick.status_detail_due);
    assert!(!tick.metadata_due);
    assert!(!tick.phase_advance_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_marks_status_detail_due_at_100() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(99);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 99);
    assert_eq!(tick.control_tick, 100);
    assert!(tick.status_detail_due);
    assert!(!tick.metadata_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_marks_metadata_due_at_1000() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(999);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 999);
    assert_eq!(tick.control_tick, 1000);
    assert!(tick.status_detail_due);
    assert!(tick.metadata_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_consumes_phase_advance_one_shot_once() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_phase_advance_for_test(true, 0x01, 7);

    let first = runtime.advance_trace_tick();
    let second = runtime.advance_trace_tick();

    assert!(first.phase_advance_due);
    assert_eq!(first.phase_advance_flags, 0x01);
    assert_eq!(first.phase_advance_counts, 7);
    assert!(!second.phase_advance_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_emits_phase_advance_periodically_when_nonzero() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(9);
    runtime.set_trace_phase_advance_for_test(false, 0x02, 5);

    let due = runtime.advance_trace_tick();
    let not_due = runtime.advance_trace_tick();

    assert_eq!(due.control_tick, 10);
    assert!(due.phase_advance_due);
    assert_eq!(due.phase_advance_flags, 0x02);
    assert_eq!(due.phase_advance_counts, 5);
    assert_eq!(not_due.control_tick, 11);
    assert!(!not_due.phase_advance_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_does_not_emit_periodic_phase_advance_when_zero() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(9);
    runtime.set_trace_phase_advance_for_test(false, 0, 0);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.control_tick, 10);
    assert!(!tick.phase_advance_due);
}
