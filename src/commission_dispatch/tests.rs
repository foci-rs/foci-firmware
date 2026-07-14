#![allow(unused_imports)]
//! Tests for drive_commission_result. Populated per-variant in Tasks 12-18.

use super::test_support::{
    MockBackend, SpiOp, accepted_commission_result as sample_commission_result, fresh_state_locals,
};
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PublishedControllerCache {
    stored_gains: Option<crate::pid::InnerLoopGains>,
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
    velocity_limit: u32,
    velocity_filter_hz: Option<u16>,
    torque_filter_hz: Option<u16>,
    position_filter_hz: Option<u16>,
    flux_filter_hz: Option<u16>,
}

fn load_published_controller_cache(state: &DispatchState<'_>) -> PublishedControllerCache {
    use core::sync::atomic::Ordering::Relaxed;

    PublishedControllerCache {
        stored_gains: *state.stored_gains,
        position_p: state.tmc.position_p_value.load(Relaxed),
        position_i: state.tmc.position_i_value.load(Relaxed),
        velocity_p: state.tmc.velocity_p_value.load(Relaxed),
        velocity_i: state.tmc.velocity_i_value.load(Relaxed),
        velocity_limit: state.tmc.velocity_limit_cfg.load(Relaxed),
        velocity_filter_hz: state.tmc.velocity_filter_hz.load(Relaxed),
        torque_filter_hz: state.tmc.torque_filter_hz.load(Relaxed),
        position_filter_hz: state.tmc.position_filter_hz.load(Relaxed),
        flux_filter_hz: state.tmc.flux_filter_hz.load(Relaxed),
    }
}

fn store_published_controller_cache(
    state: &mut DispatchState<'_>,
    cache: PublishedControllerCache,
) {
    use core::sync::atomic::Ordering::Relaxed;

    *state.stored_gains = cache.stored_gains;
    state.tmc.position_p_value.store(cache.position_p, Relaxed);
    state.tmc.position_i_value.store(cache.position_i, Relaxed);
    state.tmc.velocity_p_value.store(cache.velocity_p, Relaxed);
    state.tmc.velocity_i_value.store(cache.velocity_i, Relaxed);
    state
        .tmc
        .velocity_limit_cfg
        .store(cache.velocity_limit, Relaxed);
    state
        .tmc
        .velocity_filter_hz
        .store(cache.velocity_filter_hz, Relaxed);
    state
        .tmc
        .torque_filter_hz
        .store(cache.torque_filter_hz, Relaxed);
    state
        .tmc
        .position_filter_hz
        .store(cache.position_filter_hz, Relaxed);
    state
        .tmc
        .flux_filter_hz
        .store(cache.flux_filter_hz, Relaxed);
}

const PREVIOUS_CONTROLLER_CACHE: PublishedControllerCache = PublishedControllerCache {
    stored_gains: Some(crate::pid::InnerLoopGains {
        flux_p: 11,
        flux_i: 12,
        torque_p: 13,
        torque_i: 14,
    }),
    position_p: 21,
    position_i: 22,
    velocity_p: 23,
    velocity_i: 24,
    velocity_limit: 25_000,
    velocity_filter_hz: Some(31),
    torque_filter_hz: Some(32),
    position_filter_hz: Some(33),
    flux_filter_hz: Some(34),
};

fn stepper_state() -> &'static crate::state::stepper::StepperChannelState {
    crate::state::stepper::stepper_channel0()
}

const TEST_MOTOR_SENTINEL: u32 = 0x0002_0032;

#[test]
fn zero_outer_result_clears_outer_evidence_fields() {
    let result = zero_outer_result();

    assert_eq!(result.outer_evidence_flags, 0);
    assert_eq!(result.stiffness_timebase_ms, 0);
    assert_eq!(result.velocity_search_stop_reason, 0);
    assert_eq!(result.motion_budget_mrev, 0);
}

#[derive(Debug, PartialEq, Eq)]
enum ReplyEvent {
    Commission {
        oid: u8,
        status: u8,
    },
    ResistanceRun,
    ResistanceAxis {
        electrical_axis: u8,
    },
    CurrentValidationAxis {
        axis: u8,
        sample_index: u8,
        sample_delay_ms: u16,
        positive_response_permille: u16,
        positive_encoder_delta_counts: i32,
        negative_encoder_delta_counts: i32,
    },
    CurrentLoopRun {
        measured_axis_split_permille: u16,
        applied_axis_split_permille: u16,
        axis_split_clamped: u8,
    },
    CurrentLoopHold {
        hold_status: u8,
        sample_count: u8,
    },
    ClosedLoopEntry {
        entry_status: u8,
        drift_count: u16,
    },
}

#[derive(Default)]
struct ReplyRecorder {
    events: std::vec::Vec<ReplyEvent>,
}

impl CommissionReplySink for ReplyRecorder {
    fn foci_commission_detail(&mut self, _oid: u8, _event: crate::selftest::CommissionDiagnostic) {}

    fn foci_commission_phase(&mut self, _oid: u8, _phase: u8, _status: u8) {}

    fn foci_commission_result(
        &mut self,
        oid: u8,
        status: u8,
        _result: &crate::commissioning::types::CommissionResult,
    ) {
        self.events.push(ReplyEvent::Commission { oid, status });
    }

    fn foci_resistance_run(&mut self, _report: crate::tmc_diagnostics::ResistanceRunReport) {
        self.events.push(ReplyEvent::ResistanceRun);
    }

    fn foci_resistance_axis(&mut self, report: crate::tmc_diagnostics::ResistanceAxisReport) {
        self.events.push(ReplyEvent::ResistanceAxis {
            electrical_axis: report.electrical_axis,
        });
    }

    fn foci_encoder_alignment(&mut self, _report: crate::tmc_diagnostics::EncoderAlignmentReport) {}

    fn foci_adc_residual(&mut self, _report: crate::tmc_diagnostics::AdcResidualReport) {}

    fn foci_inductance_run(&mut self, _report: crate::tmc_diagnostics::InductanceRunReport) {}

    fn foci_inductance_frame(&mut self, _report: crate::tmc_diagnostics::InductanceFrameReport) {}

    fn foci_inductance_estimate(
        &mut self,
        _report: crate::tmc_diagnostics::InductanceEstimateReport,
    ) {
    }

    fn foci_current_loop_run(&mut self, report: crate::tmc_diagnostics::CurrentLoopRunReport) {
        self.events.push(ReplyEvent::CurrentLoopRun {
            measured_axis_split_permille: report.measured_axis_split_permille,
            applied_axis_split_permille: report.applied_axis_split_permille,
            axis_split_clamped: report.axis_split_clamped,
        });
    }

    fn foci_current_loop_hold(&mut self, report: crate::tmc_diagnostics::CurrentLoopHoldReport) {
        self.events.push(ReplyEvent::CurrentLoopHold {
            hold_status: report.hold_status,
            sample_count: report.sample_count,
        });
    }

    fn foci_closed_loop_entry(&mut self, report: crate::tmc_diagnostics::ClosedLoopEntryReport) {
        self.events.push(ReplyEvent::ClosedLoopEntry {
            entry_status: report.entry_status,
            drift_count: report.drift_count,
        });
    }

    fn foci_current_loop_filters(
        &mut self,
        _report: crate::tmc_diagnostics::CurrentLoopFiltersReport,
    ) {
    }

    fn foci_current_validation_axis(
        &mut self,
        report: crate::tmc_diagnostics::CurrentValidationAxisReport,
    ) {
        self.events.push(ReplyEvent::CurrentValidationAxis {
            axis: report.axis,
            sample_index: report.sample_index,
            sample_delay_ms: report.sample_delay_ms,
            positive_response_permille: report.positive_response_permille,
            positive_encoder_delta_counts: report.positive_encoder_delta_counts,
            negative_encoder_delta_counts: report.negative_encoder_delta_counts,
        });
    }

    fn foci_current_validation_settled_sample(
        &mut self,
        _report: crate::tmc_diagnostics::CurrentValidationSettledSampleReport,
    ) {
    }

    fn foci_current_validation_envelope(
        &mut self,
        _report: crate::tmc_diagnostics::CurrentValidationEnvelopeReport,
    ) {
    }

    fn foci_tune_result(
        &mut self,
        _oid: u8,
        _status: u8,
        _result: &crate::commissioning::types::OuterResult,
    ) {
    }

    fn foci_outer_safety_fault(
        &mut self,
        _oid: u8,
        _fault: crate::commissioning::types::OuterSafetyFault,
    ) {
    }

    fn foci_calibrate_result(
        &mut self,
        _oid: u8,
        _status: u8,
        _result: &crate::commissioning::types::AdcCalResult,
    ) {
    }

    fn foci_selftest_result(
        &mut self,
        _oid: u8,
        _stage: u8,
        _value: crate::selftest::SelftestStageValue,
    ) {
    }

    fn foci_selftest_done(&mut self, _oid: u8, _status: u8) {}
}

#[test]
fn restore_summary_packs_exact_fragments() {
    let mut backend = MockBackend::new();
    let mut seqs = crate::commissioning::outer::TraceSummarySeqs::new();

    record_restore_summary(
        &mut backend,
        &mut seqs,
        0,
        crate::autotune::CommissionError::VelocityValidationFailed.status_code(),
        17,
        1,
        crate::trace::events::TraceEventLevel::Warn,
    );

    assert_eq!(
        backend.trace_events[0].event_id,
        crate::trace::events::EVENT_COMMISSION_RESTORE_RESULT
    );
    assert_eq!(backend.trace_events[0].payload, [0, 0]);
    assert_eq!(
        backend.trace_events[1].event_id,
        crate::trace::events::EVENT_COMMISSION_RESTORE_ERROR_DRIFT
    );
    assert_eq!(
        backend.trace_events[1].payload,
        [
            0,
            crate::commissioning::outer::trace_pack_i16_pair(
                i32::from(crate::autotune::CommissionError::VelocityValidationFailed.status_code()),
                17,
            )
        ]
    );
    assert_eq!(
        backend.trace_events[2].event_id,
        crate::trace::events::EVENT_COMMISSION_RESTORE_DISPOSITION
    );
    assert_eq!(backend.trace_events[2].payload, [0, 1]);
}

async fn seed_motor_sentinel(backend: &mut MockBackend) {
    tmc_state()
        .motor_sentinel
        .store(TEST_MOTOR_SENTINEL, core::sync::atomic::Ordering::Relaxed);
    backend
        .write_register(
            foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
            TEST_MOTOR_SENTINEL,
        )
        .await
        .expect("test MOTOR_TYPE sentinel write should succeed");
}

fn clear_motor_sentinel() {
    tmc_state()
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);
}

#[test]
fn ready_now_returns_minimal_outcome() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_calibrate(0));
        let outcome = drive_commission_result(
            crate::commissioning::types::StepResult::ReadyNow,
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;
        assert!(matches!(outcome, CommissionOutcome::Continue));
        assert!(backend.spi_log.is_empty());
        assert!(engine.is_some()); // engine is not consumed on ReadyNow
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
fn calibrate_inner_failure_preserves_encoder_fault_status() {
    let status =
        calibration_status_from_commission_error(crate::autotune::CommissionError::EncoderFault);

    assert_eq!(status, 8);
    assert_ne!(
        status,
        crate::calibration::CalibrationError::InternalError.status_code()
    );
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
        engine.as_mut().expect("engine").diagnostics_mut().push(
            crate::selftest::CommissionDiagnostic {
                phase: crate::autotune::PhaseId::EncoderCheck.to_u8(),
                code: 2,
                status: 1,
                value0: 123,
                value1: 123,
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
                    ReplyPayload::CalibrateResult {
                        oid,
                        status,
                        diagnostics,
                        ..
                    },
                ..
            } => {
                assert_eq!(oid, 2);
                assert_eq!(
                    status,
                    crate::calibration::CalibrationError::SpiError.status_code()
                );
                let items: heapless::Vec<_, 4> = diagnostics.iter().collect();
                assert_eq!(items.len(), 1);
                assert_eq!(
                    items[0].phase,
                    crate::autotune::PhaseId::EncoderCheck.to_u8()
                );
                assert_eq!(items[0].code, 2);
                assert_eq!(items[0].value0, 123);
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
fn calibrate_done_preserves_step_epoch_after_gain_restore() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.assert_motor_armed_false_on_write = true;
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        seed_motor_sentinel(&mut backend).await;
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
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
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
        // Calibrate runs after M84 and before homing, so it must not create a
        // new MCU step-position origin behind Klipper's cached position.
        assert_eq!(stepper_state().position.load(Ordering::Relaxed), 42);

        match outcome {
            CommissionOutcome::Terminal { reply, actions } => {
                assert!(!actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmPreservePosition);
                match reply {
                    ReplyPayload::CalibrateResult {
                        oid,
                        status,
                        result,
                        diagnostics,
                    } => {
                        assert_eq!(oid, 4);
                        assert_eq!(status, 0);
                        assert_eq!(result.adc_i0, cal_result.adc_i0);
                        assert_eq!(result.encoder_count, cal_result.encoder_count);
                        assert_eq!(diagnostics.iter().count(), 0);
                    }
                    _ => panic!("expected CalibrateResult"),
                }
            }
            _ => panic!("expected Terminal"),
        }

        // Cleanup statics for downstream tests.
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        stepper_state().position.store(0, Ordering::Relaxed);
        clear_motor_sentinel();
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
        outer_evidence_flags: 0,
        stiffness_timebase_ms: 0,
        velocity_search_stop_reason: 0,
        motion_budget_mrev: 0,
    }
}

async fn restored_position_target(
    encoder_count: i32,
    encoder_ppr: u32,
    planner_steps_per_rev: u32,
) -> u32 {
    use foci_tmc4671::registers::PID_POSITION_TARGET;

    let mut backend = MockBackend::new();
    backend.set_abn_decoder_count(encoder_count);
    backend.mechanical_encoder_quadrature_counts_per_rev_value = encoder_ppr * 4;
    tmc_state()
        .configure_motion_scale(stepper_state(), encoder_ppr, planner_steps_per_rev)
        .expect("valid test motion scale");

    restore::restore_position_mode_after_outer(
        &mut backend,
        tmc_state(),
        &sample_outer_result_for_dispatch(),
    )
    .await
    .expect("restore succeeds");

    backend
        .spi_log
        .iter()
        .find_map(|op| match op {
            SpiOp::Write(addr, value) if *addr == PID_POSITION_TARGET.addr() => Some(*value),
            _ => None,
        })
        .expect("position target write")
}

#[test]
fn outer_restore_converts_positive_and_negative_encoder_counts_symmetrically() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;

        assert_eq!(restored_position_target(1_234, 1_000, 3_200).await, 20_218);
        assert_eq!(
            restored_position_target(-1_234, 1_000, 3_200).await,
            (-20_218_i32) as u32
        );
    });
}

#[test]
fn outer_restore_rounds_non_divisible_encoder_positions_to_nearest() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;

        assert_eq!(restored_position_target(7, 1_000, 3_200).await, 115);
        assert_eq!(
            restored_position_target(-7, 1_000, 3_200).await,
            (-115_i32) as u32
        );
    });
}

#[test]
fn outer_restore_preserves_signed_behavior_adjacent_to_encoder_wrap() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;

        assert_eq!(
            restored_position_target(i32::MAX, 16_384, 3_200).await,
            i32::MAX as u32
        );
        assert_eq!(
            restored_position_target(i32::MIN, 16_384, 3_200).await,
            i32::MIN as u32
        );
    });
}

#[test]
fn outer_restore_is_independent_of_planner_microsteps() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;

        let coarse = restored_position_target(1_234, 1_000, 3_200).await;
        let fine = restored_position_target(1_234, 1_000, 51_200).await;

        assert_eq!(coarse, 20_218);
        assert_eq!(fine, coarse);
    });
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
        let original = load_published_controller_cache(&state);
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        seed_motor_sentinel(&mut backend).await;
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
        assert_eq!(
            tmc_state().position_i_value.load(Ordering::Relaxed),
            outer.position_i
        );
        assert_eq!(
            tmc_state().velocity_p_value.load(Ordering::Relaxed),
            outer.velocity_p
        );
        assert_eq!(
            tmc_state().velocity_i_value.load(Ordering::Relaxed),
            outer.velocity_i
        );
        assert_eq!(
            tmc_state().velocity_filter_hz.load(Ordering::Relaxed),
            Some(outer.velocity_filter_hz)
        );
        assert_eq!(
            tmc_state().torque_filter_hz.load(Ordering::Relaxed),
            Some(outer.torque_filter_hz)
        );
        assert_eq!(
            tmc_state().position_filter_hz.load(Ordering::Relaxed),
            Some(outer.position_filter_hz)
        );
        assert_eq!(
            tmc_state().flux_filter_hz.load(Ordering::Relaxed),
            Some(outer.flux_filter_hz)
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
        store_published_controller_cache(&mut state, original);
        clear_motor_sentinel();
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
        let original = load_published_controller_cache(&state);
        store_published_controller_cache(&mut state, PREVIOUS_CONTROLLER_CACHE);

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
        assert_eq!(
            load_published_controller_cache(&state),
            PREVIOUS_CONTROLLER_CACHE
        );

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

        store_published_controller_cache(&mut state, original);
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
        let original = load_published_controller_cache(&state);
        let previous_cache = PublishedControllerCache {
            velocity_filter_hz: Some(0),
            ..PREVIOUS_CONTROLLER_CACHE
        };
        store_published_controller_cache(&mut state, previous_cache);

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
        assert_eq!(load_published_controller_cache(&state), previous_cache);

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

        store_published_controller_cache(&mut state, original);
    });
}

#[test]
fn outer_done_sentinel_failure_preserves_published_cache() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;

        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let mut engine = Some(CommissioningEngine::new_commission(
            0,
            CommissionProfile::Balanced,
        ));
        let original = load_published_controller_cache(&state);
        store_published_controller_cache(&mut state, PREVIOUS_CONTROLLER_CACHE);
        seed_motor_sentinel(&mut backend).await;
        backend.read_override = Some((
            foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr(),
            TEST_MOTOR_SENTINEL ^ 1,
        ));

        let outcome = drive_commission_result(
            StepResult::OuterDone(sample_outer_result_for_dispatch()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert_eq!(
            load_published_controller_cache(&state),
            PREVIOUS_CONTROLLER_CACHE
        );
        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult { status, .. },
                actions,
            } => {
                assert_eq!(
                    status,
                    crate::autotune::CommissionError::ChipResetDetected.status_code()
                );
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected TuneResult sentinel error"),
        }

        store_published_controller_cache(&mut state, original);
        clear_motor_sentinel();
    });
}

#[test]
fn commission_result_maps_current_loop_evidence() {
    let evidence = crate::autotune::CurrentLoopEvidence::measured_split(1840, 1500, true);
    let flux_validation = [
        crate::commissioning::types::CurrentValidationAxisEvidence {
            status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
            attempt: 0,
            target: 250,
            sample_delay_ms: 1,
            positive_response_permille: 720,
            negative_response_permille: 710,
            cross_axis_permille: 28,
            cross_axis_peak_permille: 28,
            voltage_output_permille: 390,
            encoder_delta_counts: 0,
            positive_encoder_delta_counts: 0,
            negative_encoder_delta_counts: 0,
            status_flags_or: 0,
        },
        crate::commissioning::types::CurrentValidationAxisEvidence {
            status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
            attempt: 0,
            target: 250,
            sample_delay_ms: 2,
            positive_response_permille: 960,
            negative_response_permille: 940,
            cross_axis_permille: 35,
            cross_axis_peak_permille: 35,
            voltage_output_permille: 420,
            encoder_delta_counts: 0,
            positive_encoder_delta_counts: 0,
            negative_encoder_delta_counts: 0,
            status_flags_or: 0,
        },
        crate::commissioning::types::CurrentValidationAxisEvidence {
            status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
            attempt: 0,
            target: 250,
            sample_delay_ms: 5,
            positive_response_permille: 1010,
            negative_response_permille: 1000,
            cross_axis_permille: 40,
            cross_axis_peak_permille: 40,
            voltage_output_permille: 430,
            encoder_delta_counts: 0,
            positive_encoder_delta_counts: 0,
            negative_encoder_delta_counts: 0,
            status_flags_or: 0,
        },
        crate::commissioning::types::CurrentValidationAxisEvidence {
            status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
            attempt: 0,
            target: 250,
            sample_delay_ms: 100,
            positive_response_permille: 1015,
            negative_response_permille: 1008,
            cross_axis_permille: 38,
            cross_axis_peak_permille: 38,
            voltage_output_permille: 410,
            encoder_delta_counts: 0,
            positive_encoder_delta_counts: 0,
            negative_encoder_delta_counts: 0,
            status_flags_or: 0,
        },
    ];
    let torque_validation = [
        crate::commissioning::types::CurrentValidationAxisEvidence {
            status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
            attempt: 0,
            target: 250,
            sample_delay_ms: 1,
            positive_response_permille: 610,
            negative_response_permille: 590,
            cross_axis_permille: 42,
            cross_axis_peak_permille: 42,
            voltage_output_permille: 510,
            encoder_delta_counts: 2,
            positive_encoder_delta_counts: 2,
            negative_encoder_delta_counts: -1,
            status_flags_or: 0,
        },
        crate::commissioning::types::CurrentValidationAxisEvidence {
            status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
            attempt: 0,
            target: 250,
            sample_delay_ms: 2,
            positive_response_permille: 780,
            negative_response_permille: 770,
            cross_axis_permille: 38,
            cross_axis_peak_permille: 38,
            voltage_output_permille: 530,
            encoder_delta_counts: 4,
            positive_encoder_delta_counts: 4,
            negative_encoder_delta_counts: -2,
            status_flags_or: 0,
        },
        crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY,
        crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY,
    ];
    let mut result = sample_commission_result();
    result.inner.current_gains_source = evidence.gains_source;
    result.inner.axis_split_source = evidence.axis_split_source;
    result.inner.gains_tier = evidence.gains_tier;
    result.inner.measured_axis_split_permille = evidence.measured_axis_split_permille;
    result.inner.applied_axis_split_permille = evidence.applied_axis_split_permille;
    result.inner.axis_split_clamped = evidence.axis_split_clamped;
    result.inner.current_validation_axes = crate::autotune::CURRENT_VALIDATION_AXIS_FLUX
        | crate::autotune::CURRENT_VALIDATION_AXIS_TORQUE;
    result.inner.retry_budget_exhausted = evidence.retry_budget_exhausted;
    result.inner.current_loop_failure_reason = evidence.failure_reason;
    result.inner.current_flux_validation_count = 4;
    result.inner.current_flux_validation = flux_validation;
    result.inner.current_torque_validation_count = 2;
    result.inner.current_torque_validation = torque_validation;

    let mut sink = ReplyRecorder::default();
    emit_terminal_reply(
        &mut sink,
        ReplyPayload::CommissionResult {
            oid: 6,
            status: 0,
            result,
        },
    );

    assert_eq!(
        sink.events,
        std::vec![
            ReplyEvent::ResistanceAxis { electrical_axis: 0 },
            ReplyEvent::ResistanceAxis { electrical_axis: 1 },
            ReplyEvent::ResistanceRun,
            ReplyEvent::CurrentValidationAxis {
                axis: crate::autotune::CURRENT_AXIS_FLUX,
                sample_index: 0,
                sample_delay_ms: 1,
                positive_response_permille: 720,
                positive_encoder_delta_counts: 0,
                negative_encoder_delta_counts: 0,
            },
            ReplyEvent::CurrentValidationAxis {
                axis: crate::autotune::CURRENT_AXIS_FLUX,
                sample_index: 1,
                sample_delay_ms: 2,
                positive_response_permille: 960,
                positive_encoder_delta_counts: 0,
                negative_encoder_delta_counts: 0,
            },
            ReplyEvent::CurrentValidationAxis {
                axis: crate::autotune::CURRENT_AXIS_FLUX,
                sample_index: 2,
                sample_delay_ms: 5,
                positive_response_permille: 1010,
                positive_encoder_delta_counts: 0,
                negative_encoder_delta_counts: 0,
            },
            ReplyEvent::CurrentValidationAxis {
                axis: crate::autotune::CURRENT_AXIS_FLUX,
                sample_index: 3,
                sample_delay_ms: 100,
                positive_response_permille: 1015,
                positive_encoder_delta_counts: 0,
                negative_encoder_delta_counts: 0,
            },
            ReplyEvent::CurrentValidationAxis {
                axis: crate::autotune::CURRENT_AXIS_TORQUE,
                sample_index: 0,
                sample_delay_ms: 1,
                positive_response_permille: 610,
                positive_encoder_delta_counts: 2,
                negative_encoder_delta_counts: -1,
            },
            ReplyEvent::CurrentValidationAxis {
                axis: crate::autotune::CURRENT_AXIS_TORQUE,
                sample_index: 1,
                sample_delay_ms: 2,
                positive_response_permille: 780,
                positive_encoder_delta_counts: 4,
                negative_encoder_delta_counts: -2,
            },
            ReplyEvent::CurrentLoopHold {
                hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
                sample_count: 0,
            },
            ReplyEvent::ClosedLoopEntry {
                entry_status: crate::autotune::CLOSED_LOOP_ENTRY_STATUS_NOT_RUN,
                drift_count: 0,
            },
            ReplyEvent::CurrentLoopRun {
                measured_axis_split_permille: 1840,
                applied_axis_split_permille: 1500,
                axis_split_clamped: 1,
            },
            ReplyEvent::Commission { oid: 6, status: 0 },
        ]
    );
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
        let original = load_published_controller_cache(&state);
        seed_motor_sentinel(&mut backend).await;

        let outcome = drive_commission_result(
            StepResult::CommissionDone(commission_result.clone()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert_eq!(
            *state.stored_gains,
            Some(crate::pid::InnerLoopGains {
                flux_p: commission_result.inner.flux_p,
                flux_i: commission_result.inner.flux_i,
                torque_p: commission_result.inner.torque_p,
                torque_i: commission_result.inner.torque_i,
            })
        );
        assert_eq!(
            state.tmc.position_p_value.load(Ordering::Relaxed),
            commission_result.fallback_position_p
        );
        assert_eq!(
            state.tmc.position_i_value.load(Ordering::Relaxed),
            commission_result.fallback_position_i
        );
        assert_eq!(
            state.tmc.velocity_p_value.load(Ordering::Relaxed),
            commission_result.fallback_velocity_p
        );
        assert_eq!(
            state.tmc.velocity_i_value.load(Ordering::Relaxed),
            commission_result.fallback_velocity_i
        );
        assert_eq!(
            state.tmc.velocity_limit_cfg.load(Ordering::Relaxed),
            commission_result.fallback_velocity_limit
        );
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
            CommissionOutcome::Terminal {
                reply:
                    ReplyPayload::CommissionResult {
                        result: reply_result,
                        ..
                    },
                actions,
            } => {
                assert_eq!(reply_result.inner.r_int, 1706);
                assert_eq!(reply_result.inner.l_int, 1245);
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmZeroPosition);
            }
            _ => panic!("expected CommissionResult"),
        }
        // Cleanup
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        store_published_controller_cache(&mut state, original);
        stepper_state().position.store(0, Ordering::Relaxed);
        tmc_state().current_limit_reg.store(0, Ordering::Relaxed);
        clear_motor_sentinel();
    });
}

#[test]
fn commission_done_failed_inner_status_skips_persist_and_arm() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .current_limit_reg
            .store(0x0010_0010, Ordering::Relaxed);
        let mut engine = Some(CommissioningEngine::new_commission(
            6,
            CommissionProfile::Balanced,
        ));
        let mut commission_result = sample_commission_result();
        commission_result.inner.status = crate::autotune::CommissionStatus::Failed(
            crate::autotune::CommissionError::CurrentValidationFailed,
        );
        commission_result.inner.current_gains_source = crate::autotune::CURRENT_GAINS_SOURCE_FAILED;
        commission_result.inner.gains_tier = crate::autotune::CURRENT_GAINS_TIER_NONE;
        commission_result.inner.current_loop_failure_reason =
            crate::autotune::CURRENT_LOOP_FAILURE_SATURATION;
        commission_result.inner.current_validation_axes =
            crate::autotune::CURRENT_VALIDATION_AXIS_FLUX;

        let outcome = drive_commission_result(
            StepResult::CommissionDone(commission_result),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert!(state.stored_gains.is_none());
        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        assert!(!backend.spi_log.iter().any(|op| {
            matches!(
                op,
                SpiOp::Write(addr, _) if *addr == foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr()
            )
        }));

        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::CommissionResult { status, result, .. },
                actions,
            } => {
                assert_eq!(
                    status,
                    crate::autotune::CommissionError::CurrentValidationFailed.status_code()
                );
                assert_eq!(
                    result.inner.current_loop_failure_reason,
                    crate::autotune::CURRENT_LOOP_FAILURE_SATURATION
                );
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected failed CommissionResult"),
        }

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
        let original = load_published_controller_cache(&state);
        store_published_controller_cache(&mut state, PREVIOUS_CONTROLLER_CACHE);

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
        assert_eq!(
            load_published_controller_cache(&state),
            PREVIOUS_CONTROLLER_CACHE
        );

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

        store_published_controller_cache(&mut state, original);
        tmc_state().current_limit_reg.store(0, Ordering::Relaxed);
    });
}

#[test]
fn commission_done_sentinel_failure_preserves_published_cache() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;

        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        state
            .tmc
            .current_limit_reg
            .store(0x0010_0010, Ordering::Relaxed);
        let mut engine = Some(CommissioningEngine::new_commission(
            6,
            CommissionProfile::Balanced,
        ));
        let original = load_published_controller_cache(&state);
        store_published_controller_cache(&mut state, PREVIOUS_CONTROLLER_CACHE);
        seed_motor_sentinel(&mut backend).await;
        backend.read_override = Some((
            foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr(),
            TEST_MOTOR_SENTINEL ^ 1,
        ));

        let outcome = drive_commission_result(
            StepResult::CommissionDone(sample_commission_result()),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
        )
        .await;

        assert_eq!(
            load_published_controller_cache(&state),
            PREVIOUS_CONTROLLER_CACHE
        );
        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!backend.enable_pin);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        match outcome {
            CommissionOutcome::Terminal {
                reply: ReplyPayload::CommissionResult { status, .. },
                actions,
            } => {
                assert_eq!(
                    status,
                    crate::autotune::CommissionError::ChipResetDetected.status_code()
                );
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::Stop);
            }
            _ => panic!("expected CommissionResult sentinel error"),
        }

        store_published_controller_cache(&mut state, original);
        state.tmc.current_limit_reg.store(0, Ordering::Relaxed);
        clear_motor_sentinel();
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
        inner_warning_flags: 0,
        max_travel_mrev: 1000,
        max_velocity_mrev_s: 6000,
        max_duration_ms: 3000,
        direction_mask: 3,
    }
}

#[test]
fn outer_failed_safety_envelope_violation_full_disarm() {
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
        let err = crate::autotune::CommissionError::SafetyEnvelopeViolation;

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
        seed_motor_sentinel(&mut backend).await;

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
        clear_motor_sentinel();
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
fn outer_failed_restore_uses_encoder_scale_for_position_jitter() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        // The full window spans 31 TMC position units, within two quadrature
        // encoder counts at 4000 counts/revolution (33 rounded position units).
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
        seed_motor_sentinel(&mut backend).await;

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

        tmc_state()
            .motor_armed
            .store(false, core::sync::atomic::Ordering::Relaxed);
        clear_motor_sentinel();
    });
}

#[test]
fn outer_failed_restore_accepts_scaled_encoder_jitter() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
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
        seed_motor_sentinel(&mut backend).await;

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
        clear_motor_sentinel();
    });
}

#[test]
fn outer_failed_restore_rejects_monotonic_creep() {
    pollster::block_on(async {
        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut backend = MockBackend::new();
        backend.mechanical_encoder_quadrature_counts_per_rev_value = 6_554;
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
