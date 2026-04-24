#![allow(unused_imports)]
//! Tests for drive_commission_result. Populated per-variant in Tasks 12-18.

use super::test_support::{MockBackend, SpiOp, fresh_state_locals};
use super::*;
use crate::autotune::CommissionProfile;

#[test]
fn continue_returns_minimal_outcome() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        // Reset COMMISSION_IN_FLIGHT to a known state before the test.
        crate::state::tmc::COMMISSION_IN_FLIGHT.store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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
                assert!(!actions.stop_stepper);
                assert!(!actions.spawn_arm_check);
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
            !crate::state::tmc::COMMISSION_IN_FLIGHT.load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn inner_failed_commission_emits_phase_reply() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        crate::state::tmc::COMMISSION_IN_FLIGHT.store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        let mut engine = Some(CommissioningEngine::new_commission(
            5,
            CommissionProfile::Balanced,
        ));
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
                assert!(!actions.stop_stepper);
                assert!(!actions.spawn_arm_check);
                match reply {
                    ReplyPayload::CommissionPhase { oid, phase, status } => {
                        assert_eq!(oid, 5);
                        assert_eq!(phase, 0);
                        assert_eq!(status, err.status_code());
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
            !crate::state::tmc::COMMISSION_IN_FLIGHT.load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn inner_failed_calibrate_emits_calibrate_reply() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        crate::state::tmc::COMMISSION_IN_FLIGHT.store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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
            !crate::state::tmc::COMMISSION_IN_FLIGHT.load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn inner_failed_selftest_emits_selftest_pair() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        crate::state::tmc::COMMISSION_IN_FLIGHT.store(true, core::sync::atomic::Ordering::Relaxed);
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        let mut engine = Some(CommissioningEngine::new_selftest(8));
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
                        oid, done_status, ..
                    },
                ..
            } => {
                assert_eq!(oid, 8);
                assert_eq!(done_status, err.status_code());
            }
            _ => panic!("expected Selftest"),
        }
        assert!(engine.is_none());
        assert!(
            !crate::state::tmc::COMMISSION_IN_FLIGHT.load(core::sync::atomic::Ordering::Relaxed)
        );
    });
}

#[test]
fn calibrate_done_arms_after_gain_restore() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        gains = Some(crate::pid::InnerLoopGains {
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
        });
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        let mut engine = Some(CommissioningEngine::new_calibrate(4));
        let cal_result = crate::commissioning::types::AdcCalResult {
            adc_i0: 1234,
            adc_i1: 5678,
            encoder_count: 999,
        };
        crate::state::stepper::STEPPER_POSITION.store(42, Ordering::Relaxed);

        let outcome = drive_commission_result(
            StepResult::CalibrateDone(cal_result.clone()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        // Motor armed via electrical+for_steps.
        assert!(backend.enable_pin);
        assert!(*state.motor_enabled);
        assert!(*state.calibrated);
        assert!(crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));
        // STEPPER_POSITION is NOT zeroed by the dispatch — the board's
        // SetEnable(true) path owns the zero. Dispatch must not touch it,
        // to avoid racing with queued homing moves that start as soon as
        // MOTOR_ARMED is set.
        assert_eq!(
            crate::state::stepper::STEPPER_POSITION.load(Ordering::Relaxed),
            42
        );

        match outcome {
            CommissionOutcome::Terminal { reply, actions } => {
                assert!(actions.spawn_arm_check);
                assert!(!actions.stop_trace);
                assert!(!actions.stop_stepper);
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
        crate::state::tmc::MOTOR_ARMED.store(false, Ordering::Relaxed);
        crate::state::stepper::STEPPER_POSITION.store(0, Ordering::Relaxed);
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
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        let mut engine = Some(CommissioningEngine::new_commission(
            0,
            CommissionProfile::Balanced,
        ));
        let outer = sample_outer_result_for_dispatch();

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
            crate::state::tmc::POSITION_P_VALUE.load(Ordering::Relaxed),
            outer.position_p
        );
        assert_eq!(
            crate::state::tmc::VELOCITY_LIMIT_CFG.load(Ordering::Relaxed),
            outer.velocity_limit
        );

        // Motor armed
        assert!(backend.enable_pin);
        assert!(crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, 0);
                assert!(actions.stop_trace);
                assert!(!actions.stop_stepper);
                assert!(!actions.spawn_arm_check); // OuterDone doesn't spawn arm_check
            }
            _ => panic!("expected TuneResult"),
        }

        crate::state::tmc::MOTOR_ARMED.store(false, Ordering::Relaxed);
    });
}

#[test]
fn outer_done_spi_failure_disarms() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr());
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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
        assert!(!crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, crate::autotune::CommissionError::Spi.status_code());
                assert!(actions.stop_trace);
                assert!(actions.stop_stepper);
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
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        crate::state::tmc::CURRENT_LIMIT_REG.store(0x0010_0010, Ordering::Relaxed);
        crate::state::stepper::STEPPER_POSITION.store(99, Ordering::Relaxed);
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
        assert!(crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));
        // STEPPER_POSITION must NOT be zeroed by the dispatch — the board's
        // SetEnable(true) path owns the zero. See CalibrateDone test comment.
        assert_eq!(
            crate::state::stepper::STEPPER_POSITION.load(Ordering::Relaxed),
            99
        );
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
                assert!(actions.spawn_arm_check);
                assert!(!actions.stop_stepper);
            }
            _ => panic!("expected Terminal"),
        }

        // Cleanup
        crate::state::tmc::MOTOR_ARMED.store(false, Ordering::Relaxed);
        crate::state::stepper::STEPPER_POSITION.store(0, Ordering::Relaxed);
        crate::state::tmc::CURRENT_LIMIT_REG.store(0, Ordering::Relaxed);
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
    }
}

#[test]
fn outer_failed_safety_fault_full_disarm() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        cal = true;
        en = true; // simulate motor was previously armed
        crate::state::tmc::MOTOR_ARMED.store(true, Ordering::Relaxed);
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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
        assert!(!crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));
        // Safety fault skips stability check — no delays.
        assert!(backend.delay_us_log.is_empty());

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, err.status_code());
                assert!(actions.stop_stepper);
                assert!(actions.stop_trace);
            }
            _ => panic!("expected TuneResult"),
        }
    });
}

#[test]
fn outer_failed_soft_failure_holds_motor() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        cal = true;
        en = true;
        crate::state::stepper::STEPPER_POSITION.store(99, Ordering::Relaxed);
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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

        // Two delay_us(10_000) calls expected (one before each stability read).
        assert_eq!(backend.delay_us_log.len(), 2);
        assert!(backend.delay_us_log.iter().all(|d| *d == 10_000));

        // Motor still armed (soft failure, stable).
        assert!(*state.calibrated);
        assert!(backend.enable_pin);
        // STEPPER_POSITION NOT zeroed (motor held position).
        assert_eq!(
            crate::state::stepper::STEPPER_POSITION.load(Ordering::Relaxed),
            99
        );

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(status, err.status_code());
                assert!(!actions.stop_stepper);
            }
            _ => panic!("expected TuneResult"),
        }

        crate::state::tmc::MOTOR_ARMED.store(false, Ordering::Relaxed);
        crate::state::stepper::STEPPER_POSITION.store(0, Ordering::Relaxed);
    });
}

#[test]
fn outer_failed_unstable_drift_treats_as_failure() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().unwrap();
        let mut backend = MockBackend::new();
        // Pre-load drifting position values (drift > 2 => unstable).
        backend.read_position_actual_sequence.push(100).unwrap();
        backend.read_position_actual_sequence.push(120).unwrap();

        let (mut cal, mut en, mut gains) = fresh_state_locals();
        cal = true;
        en = true;
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
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
                assert_eq!(
                    status,
                    crate::autotune::CommissionError::CurrentValidationFailed.status_code()
                );
            }
            _ => panic!("expected TuneResult error"),
        }
        assert_eq!(backend.delay_us_log.len(), 2);

        crate::state::tmc::MOTOR_ARMED.store(false, Ordering::Relaxed);
    });
}
