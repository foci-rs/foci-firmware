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
