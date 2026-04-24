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
