#![allow(unused_imports)]
//! Tests for drive_commission_result. Populated per-variant in Tasks 12-18.

use super::test_support::{MockBackend, SpiOp, fresh_state_locals};
use super::*;
use crate::autotune::CommissionProfile;

fn dispatch_state<'a>(
    calibrated: &'a mut bool,
    motor_enabled: &'a mut bool,
    stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
) -> DispatchState<'a> {
    DispatchState::new(
        crate::state::tmc::tmc_channel0(),
        crate::state::stepper::stepper_channel0(),
        calibrated,
        motor_enabled,
        stored_gains,
    )
}

fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
    crate::state::tmc::tmc_channel0()
}

fn stepper_state() -> &'static crate::state::stepper::StepperChannelState {
    crate::state::stepper::stepper_channel0()
}

#[test]
fn continue_returns_minimal_outcome() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_calibrate(0));
        let outcome = drive_commission_result(
            crate::commissioning::types::StepResult::Continue,
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;
        assert!(matches!(outcome, CommissionOutcome::Continue));
        assert!(backend.spi_log.is_empty());
        assert!(engine.is_some()); // engine NOT consumed on Continue
    });
}

#[test]
fn phase_changed_returns_oid_phase_kind() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_commission(
            7,
            CommissionProfile::Balanced,
        ));
        let outcome = drive_commission_result(
            crate::commissioning::types::StepResult::PhaseChanged(
                crate::autotune::PhaseId::ClosedLoopEntry,
            ),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;
        match outcome {
            CommissionOutcome::PhaseChanged {
                oid,
                phase_id,
                engine_kind,
            } => {
                assert_eq!(oid, 7);
                assert_eq!(phase_id, crate::autotune::PhaseId::ClosedLoopEntry);
                assert_eq!(
                    engine_kind,
                    crate::commissioning::types::EngineKind::Commission
                );
            }
            _ => panic!("expected PhaseChanged"),
        }
        assert!(engine.is_some()); // not consumed
    });
}

#[test]
fn selftest_done_zero_stages_emits_done_only() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        // Reset the in-flight guard to a known state before the test.
        tmc_state()
            .commission_in_flight
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_selftest(3));
        let outcome = drive_commission_result(
            crate::commissioning::types::StepResult::SelftestDone,
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;
        match outcome {
            CommissionOutcome::Terminal { reply, actions } => {
                assert!(!actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::None);
                match reply {
                    ReplyPayload::Selftest {
                        oid, done_status, ..
                    } => {
                        assert_eq!(oid, 3);
                        assert_eq!(done_status, 0);
                    }
                    _ => panic!("expected Selftest reply"),
                }
            }
            _ => panic!("expected Terminal"),
        }
        assert!(engine.is_none()); // consumed
        assert!(
            !tmc_state()
                .commission_in_flight
                .load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn inner_failed_commission_emits_phase_reply() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        tmc_state()
            .commission_in_flight
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_commission(
            5,
            CommissionProfile::Balanced,
        ));
        engine.as_mut().expect("engine").diagnostics_mut().push(
            crate::selftest::CommissionDiagnostic {
                phase: crate::autotune::PhaseId::ElectricalId.to_u8(),
                code: 28,
                status: 1,
                value0: 820,
                value1: 500,
                value2: 0,
            },
        );
        let err = crate::autotune::CommissionError::NoCurrent;
        let outcome = drive_commission_result(
            crate::commissioning::types::StepResult::InnerFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;
        match outcome {
            CommissionOutcome::Terminal { reply, actions } => {
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::None);
                match reply {
                    ReplyPayload::CommissionPhase {
                        oid,
                        phase,
                        status,
                        diagnostics,
                    } => {
                        assert_eq!(oid, 5);
                        assert_eq!(phase, 0);
                        assert_eq!(status, err.status_code());
                        let items: heapless::Vec<_, 4> = diagnostics.iter().collect();
                        assert_eq!(items.len(), 1);
                        assert_eq!(items[0].code, 28);
                        assert_eq!(items[0].value0, 820);
                    }
                    _ => panic!("expected CommissionPhase"),
                }
            }
            _ => panic!("expected Terminal"),
        }
        // restore_safe_state_via_backend cleared enable_pin via backend.set_enable_pin(false).
        assert!(!backend.enable_pin);
        // engine consumed
        assert!(engine.is_none());
        assert!(
            !tmc_state()
                .commission_in_flight
                .load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn inner_failed_calibrate_emits_calibrate_reply() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        tmc_state()
            .commission_in_flight
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_calibrate(2));
        let err = crate::autotune::CommissionError::NoCurrent;
        let outcome = drive_commission_result(
            crate::commissioning::types::StepResult::InnerFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;
        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::CalibrateResult { oid, status, .. },
                ..
            } => {
                assert_eq!(oid, 2);
                assert_eq!(status, err.status_code());
            }
            _ => panic!("expected CalibrateResult"),
        }
        assert!(engine.is_none());
        assert!(
            !tmc_state()
                .commission_in_flight
                .load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn inner_failed_selftest_emits_selftest_pair() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        tmc_state()
            .commission_in_flight
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_selftest(8));
        engine.as_mut().expect("engine").diagnostics_mut().push(
            crate::selftest::CommissionDiagnostic {
                phase: crate::autotune::PhaseId::ElectricalId.to_u8(),
                code: 27,
                status: 1,
                value0: 0,
                value1: 0,
                value2: 0,
            },
        );
        let err = crate::autotune::CommissionError::Spi;
        let outcome = drive_commission_result(
            crate::commissioning::types::StepResult::InnerFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;
        match outcome {
            CommissionOutcome::Terminal {
                reply:
                    ReplyPayload::Selftest {
                        oid,
                        done_status,
                        diagnostics,
                        ..
                    },
                ..
            } => {
                assert_eq!(oid, 8);
                assert_eq!(done_status, err.status_code());
                let items: heapless::Vec<_, 4> = diagnostics.iter().collect();
                assert_eq!(items.len(), 1);
                assert_eq!(items[0].code, 27);
            }
            _ => panic!("expected Selftest"),
        }
        assert!(engine.is_none());
        assert!(
            !tmc_state()
                .commission_in_flight
                .load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn calibrate_done_arms_after_gain_restore() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.assert_motor_armed_false_on_write = true;
        let (mut cal, mut en, _) = fresh_state_locals();
        let mut gains = Some(crate::pid::InnerLoopGains {
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
        });
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_calibrate(4));
        let cal_result = crate::commissioning::types::AdcCalResult {
            adc_i0: 1234,
            adc_i1: 5678,
            encoder_count: 999,
        };
        stepper_state().position.store(42, Ordering::Relaxed);
        tmc_state().motor_armed.store(true, Ordering::Relaxed);

        let outcome = drive_commission_result(
            StepResult::CalibrateDone(cal_result.clone()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        // Motor electrically enabled; board owns stepper arming.
        assert!(backend.enable_pin);
        assert!(*state.motor_enabled);
        assert!(*state.calibrated);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        // Dispatch must not mutate the firmware step epoch directly; the
        // returned board action owns the zero-before-arm ordering.
        assert_eq!(stepper_state().position.load(Ordering::Relaxed), 42);

        match outcome {
            CommissionOutcome::Terminal { reply, actions } => {
                assert!(!actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmZeroPosition);
                match reply {
                    ReplyPayload::CalibrateResult {
                        oid,
                        status,
                        result,
                    } => {
                        assert_eq!(oid, 4);
                        assert_eq!(status, 0);
                        assert_eq!(result.adc_i0, cal_result.adc_i0);
                        assert_eq!(result.encoder_count, cal_result.encoder_count);
                    }
                    _ => panic!("expected CalibrateResult"),
                }
            }
            _ => panic!("expected Terminal"),
        }

        // Cleanup statics for downstream tests.
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        stepper_state().position.store(0, Ordering::Relaxed);
    });
}

use crate::commissioning::types::OuterResult;

fn sample_outer_result_for_dispatch() -> OuterResult {
    OuterResult {
        status: crate::autotune::CommissionStatus::Accepted,
        velocity_p: 1152,
        velocity_i: 0,
        velocity_limit: 500_000,
        position_p: 640,
        position_i: 0,
        velocity_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
        torque_filter_hz: 0,
        j_eff: 0,
        b_eff: 0,
        warning_code: 0,
        mech_torque_step: 0,
        mech_fwd_accel: 0,
        mech_rev_accel: 0,
        mech_fwd_peak_velocity: 0,
        mech_rev_peak_velocity: 0,
        mech_fwd_travel: 0,
        mech_rev_travel: 0,
        synth_velocity_p: 0,
        synth_velocity_i: 0,
        synth_position_p: 0,
        synth_position_i: 0,
    }
}

#[test]
fn outer_done_success_arms_motor() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.assert_motor_armed_false_on_write = true;
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_commission(
            0,
            CommissionProfile::Balanced,
        ));
        let outer = sample_outer_result_for_dispatch();
        tmc_state().motor_armed.store(true, Ordering::Relaxed);

        let outcome = drive_commission_result(
            StepResult::OuterDone(outer.clone()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        // Atomics persisted
        assert_eq!(
            tmc_state().position_p_value.load(Ordering::Relaxed),
            outer.position_p
        );
        assert_eq!(
            tmc_state().velocity_limit_cfg.load(Ordering::Relaxed),
            outer.velocity_limit
        );

        // Motor electrically enabled; board owns stepper arming.
        assert!(backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, 0);
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmPreservePosition);
            }
            _ => panic!("expected TuneResult"),
        }

        tmc_state().motor_armed.store(false, Ordering::Relaxed);
    });
}

#[test]
fn outer_done_spi_failure_disarms() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr());
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_commission(
            0,
            CommissionProfile::Balanced,
        ));
        let outer = sample_outer_result_for_dispatch();

        let outcome = drive_commission_result(
            StepResult::OuterDone(outer),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        // Disarmed
        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, crate::autotune::CommissionError::Spi.status_code());
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected TuneResult error"),
        }
    });
}

#[test]
fn outer_done_filter_apply_failure_disarms() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE.data.addr());
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_commission(
            0,
            CommissionProfile::Balanced,
        ));
        let mut outer = sample_outer_result_for_dispatch();
        outer.velocity_filter_hz = 100;

        let outcome = drive_commission_result(
            StepResult::OuterDone(outer),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, crate::autotune::CommissionError::Spi.status_code());
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected TuneResult error"),
        }
    });
}

fn sample_commission_result() -> crate::commissioning::types::CommissionResult {
    use crate::autotune::CommissionStatus;
    use crate::commissioning::types::{CommissionResult, InnerResult};
    CommissionResult {
        inner: InnerResult {
            status: CommissionStatus::Accepted,
            r_mohm: 1700,
            l_uh: 3100,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            lambda_us: 100,
            theta_e_us: 50,
            ringing_count: 0,
            bandwidth_hz: 1000,
            tau_e_us: 730,
            tau_e_crosscheck_us: 730,
            tau_residual_permille: 0,
            inner_warning_flags: 0,
        },
        fallback_velocity_p: 1152,
        fallback_velocity_i: 0,
        fallback_position_p: 640,
        fallback_position_i: 0,
        fallback_velocity_limit: 500_000,
    }
}

#[test]
fn commission_done_finalize_path() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .current_limit_reg
            .store(0x0010_0010, Ordering::Relaxed);
        stepper_state().position.store(99, Ordering::Relaxed);
        let mut engine = Some(CommissioningEngine::new_commission(
            6,
            CommissionProfile::Balanced,
        ));
        let commission_result = sample_commission_result();

        let outcome = drive_commission_result(
            StepResult::CommissionDone(commission_result.clone()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(state.stored_gains.is_some());
        assert!(*state.calibrated);
        assert!(backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        // Dispatch must not mutate the firmware step epoch directly; the
        // returned board action owns the zero-before-arm ordering.
        assert_eq!(stepper_state().position.load(Ordering::Relaxed), 99);
        // PID_TORQUE_FLUX_LIMITS write expected: current limit re-apply.
        let writes: heapless::Vec<u8, 32> = backend
            .spi_log
            .iter()
            .filter_map(|op| {
                if let SpiOp::Write(a, _) = op {
                    Some(*a)
                } else {
                    None
                }
            })
            .collect();
        assert!(writes.contains(&foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr()));

        match outcome {
            CommissionOutcome::Terminal { actions, .. } => {
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmZeroPosition);
            }
            _ => panic!("expected Terminal"),
        }

        // Cleanup
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        stepper_state().position.store(0, Ordering::Relaxed);
        tmc_state().current_limit_reg.store(0, Ordering::Relaxed);
    });
}

#[test]
fn commission_done_current_limit_restore_failure_disarms() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr());
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .current_limit_reg
            .store(0x0010_0010, Ordering::Relaxed);
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        let mut engine = Some(CommissioningEngine::new_commission(
            6,
            CommissionProfile::Balanced,
        ));

        let outcome = drive_commission_result(
            StepResult::CommissionDone(sample_commission_result()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::CommissionResult { status, .. },
                actions,
            } => {
                assert_eq!(status, crate::autotune::CommissionError::Spi.status_code());
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected CommissionResult error"),
        }

        tmc_state().current_limit_reg.store(0, Ordering::Relaxed);
    });
}

fn sample_entry_snapshot() -> crate::commissioning::EntrySnapshot {
    crate::commissioning::EntrySnapshot {
        velocity_p: 1152,
        velocity_i: 0,
        position_p: 640,
        position_i: 0,
        velocity_limit: 500_000,
        voltage_limit: 16000,
        velocity_filter_hz: 0,
        torque_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
    }
}

fn sample_outer_params() -> crate::commissioning::OuterTuneParams {
    crate::commissioning::OuterTuneParams {
        profile: CommissionProfile::Balanced,
        mode: crate::autotune::OperatingMode::Nominal,
        inner_lambda_us: 100,
        theta_e_us: 50,
        current_ringing: false,
        current_bandwidth_hz: 1000,
        tau_e_us: 730,
        tau_e_crosscheck_us: 730,
        tau_residual_permille: 0,
        inner_warning_flags: 0,
    }
}

#[test]
fn outer_failed_safety_fault_full_disarm() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (_, _, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut en = true; // simulate motor was previously armed
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_outer(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
        ));
        let err = crate::autotune::CommissionError::Spi;

        let outcome = drive_commission_result(
            StepResult::OuterFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        // Safety fault skips stability check — no delays.
        assert!(backend.delay_us_log.is_empty());

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, err.status_code());
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected TuneResult"),
        }
    });
}

#[test]
fn outer_failed_soft_failure_holds_motor() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (_, _, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut en = true;
        stepper_state().position.store(99, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_outer(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
        ));
        // NoCurrent is not a safety-fault variant.
        let err = crate::autotune::CommissionError::NoCurrent;

        let outcome = drive_commission_result(
            StepResult::OuterFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        // Five-sample restore stability window.
        assert_eq!(backend.delay_us_log.len(), 5);
        assert!(backend.delay_us_log.iter().all(|d| *d == 5_000));

        // Motor still armed (soft failure, stable).
        assert!(*state.calibrated);
        assert!(backend.enable_pin);
        // Stepper position NOT zeroed (motor held position).
        assert_eq!(stepper_state().position.load(Ordering::Relaxed), 99);

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, err.status_code());
                assert_eq!(actions.stepper, StepperPostAction::ArmPreservePosition);
            }
            _ => panic!("expected TuneResult"),
        }

        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        stepper_state().position.store(0, Ordering::Relaxed);
    });
}

#[test]
fn outer_failed_unstable_drift_preserves_upstream_error() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        // Pre-load drifting position values (drift > 2 => unstable).
        backend.read_position_actual_sequence.push(100).unwrap();
        backend.read_position_actual_sequence.push(120).unwrap();

        let (_, _, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut en = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_outer(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
        ));
        let err = crate::autotune::CommissionError::NoCurrent;

        let outcome = drive_commission_result(
            StepResult::OuterFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                ..
            } => {
                // Drift > 2 in the restore stability check no longer
                // overrides the upstream error; the original NoCurrent
                // failure is preserved so the host sees the meaningful
                // root cause. Drift is still logged for diagnostics.
                assert_eq!(
                    status,
                    crate::autotune::CommissionError::NoCurrent.status_code()
                );
            }
            _ => panic!("expected TuneResult error"),
        }
        assert_eq!(backend.delay_us_log.len(), 5);
        assert!(backend.delay_us_log.iter().all(|d| *d == 5_000));

        tmc_state().motor_armed.store(false, Ordering::Relaxed);
    });
}

#[test]
fn outer_failed_restore_rejects_aliased_position_oscillation() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.step_width_value = 10;
        // First two samples match, so the old two-read check accepts this.
        // The full window spans 31 TMC position units: more than two
        // encoder counts at step_width=10.
        backend.read_position_actual_sequence.push(1000).unwrap();
        backend.read_position_actual_sequence.push(1000).unwrap();
        backend.read_position_actual_sequence.push(1031).unwrap();
        backend.read_position_actual_sequence.push(1001).unwrap();
        backend.read_position_actual_sequence.push(1000).unwrap();

        let (_, _, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut en = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_outer(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
        ));
        let err = crate::autotune::CommissionError::VelocityValidationFailed;

        let outcome = drive_commission_result(
            StepResult::OuterFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, err.status_code());
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected TuneResult"),
        }
    });
}

#[test]
fn outer_failed_restore_accepts_scaled_encoder_jitter() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.step_width_value = 819;
        backend.read_position_actual_sequence.push(1000).unwrap();
        backend.read_position_actual_sequence.push(1001).unwrap();
        backend.read_position_actual_sequence.push(1000).unwrap();
        backend.read_position_actual_sequence.push(999).unwrap();
        backend.read_position_actual_sequence.push(1000).unwrap();

        let (_, _, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut en = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_outer(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
        ));
        let err = crate::autotune::CommissionError::VelocityValidationFailed;

        let outcome = drive_commission_result(
            StepResult::OuterFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(*state.calibrated);
        assert!(*state.motor_enabled);
        assert!(backend.enable_pin);

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, err.status_code());
                assert_eq!(actions.stepper, StepperPostAction::ArmPreservePosition);
            }
            _ => panic!("expected TuneResult"),
        }

        tmc_state().motor_armed.store(false, Ordering::Relaxed);
    });
}

#[test]
fn outer_failed_restore_rejects_monotonic_creep() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.step_width_value = 10;
        backend.read_position_actual_sequence.push(1000).unwrap();
        backend.read_position_actual_sequence.push(1006).unwrap();
        backend.read_position_actual_sequence.push(1012).unwrap();
        backend.read_position_actual_sequence.push(1018).unwrap();
        backend.read_position_actual_sequence.push(1024).unwrap();

        let (_, _, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut en = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_outer(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
        ));
        let err = crate::autotune::CommissionError::VelocityValidationFailed;

        let outcome = drive_commission_result(
            StepResult::OuterFailed(err),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, err.status_code());
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected TuneResult"),
        }
    });
}
