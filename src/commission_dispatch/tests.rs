#![allow(unused_imports)]
//! Tests for drive_commission_result. Populated per-variant in Tasks 12-18.

use super::test_support::{
    MockBackend, SpiOp, accepted_commission_result as sample_commission_result, fresh_state_locals,
};
use super::*;
use crate::autotune::CommissionProfile;

#[cfg(feature = "trace-velocity-sweep")]
fn stage_b_request() -> crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest {
    use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;

    crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest {
        requested_velocity_mrev_s: 2_929,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 500,
        settle_travel_reserve_mrev: 100,
        negative_position_headroom_mrev: 2_000,
        positive_position_headroom_mrev: 2_000,
        max_duration_ms: 2_000,
    }
}

#[cfg(feature = "trace-velocity-sweep")]
#[test]
fn stage_b_preflight_freezes_one_plan_and_rejects_mismatch_without_mutation() {
    use crate::commission_dispatch::stage_b::{
        StageBPreflightRejection, preflight_stage_b_request,
    };
    use crate::commissioning::outer::velocity::reproduction::{
        StageBCandidateSlot, StageBCandidateSummary,
    };

    let mut slot = StageBCandidateSlot::new();
    let first = preflight_stage_b_request(stage_b_request(), 4_000, 2_816, 0, 7, &mut slot)
        .expect("valid preflight");
    assert_eq!(first.plan.digest(), first.identity.plan_digest);
    assert_eq!(first.identity.channel, 0);
    assert_eq!(first.run_sequence, 7);

    let retained = StageBCandidateSummary {
        identity: first.identity,
        forward_membership: 0b0001_1100,
        reverse_membership: 0b0001_1100,
        joint_membership: 0b0001_1100,
        nominated_p: 724,
        p_start: first.plan.p_start.raw(),
        p_top: first.plan.p_top.raw(),
        rung_count: first.plan.rung_count,
        forward_interval: [10, 20],
        reverse_interval: [-20, -10],
        forward_common_interval: [12, 18],
        reverse_common_interval: [-18, -12],
        run_completed_us: 2,
        run_digest: 3,
    };
    assert!(matches!(
        slot.observe_sufficient(retained),
        crate::commissioning::outer::velocity::reproduction::StageBReproductionDecision::CompleteCandidate(_)
    ));

    let mut changed = stage_b_request();
    changed.requested_velocity_mrev_s += 1;
    let rejection = preflight_stage_b_request(changed, 4_000, 2_816, 0, 8, &mut slot)
        .expect_err("changed frozen plan must be rejected");
    assert!(matches!(
        rejection,
        StageBPreflightRejection::RejectedPlanMismatch { retained: old, requested }
            if old == first.identity && requested != first.identity
    ));
    assert_eq!(slot.candidate(), Some(retained));
}

#[cfg(feature = "trace-velocity-sweep")]
#[test]
fn stage_b_preflight_maps_invalid_plans_without_touching_retention() {
    use crate::commission_dispatch::stage_b::{
        StageBPreflightRejection, preflight_stage_b_request,
    };
    use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;

    let mut slot = StageBCandidateSlot::new();
    let mut invalid = stage_b_request();
    invalid.requested_velocity_mrev_s = 0;
    assert_eq!(
        preflight_stage_b_request(invalid, 4_000, 2_816, 0, 7, &mut slot),
        Err(StageBPreflightRejection::InvalidPlan)
    );
    assert_eq!(slot.candidate(), None);
}

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
    let outer = state.tmc.load_outer_gains(Relaxed).unwrap();

    PublishedControllerCache {
        stored_gains: *state.stored_gains,
        position_p: outer.position_p.raw(),
        position_i: outer.position_i.raw(),
        velocity_p: outer.velocity_p.raw(),
        velocity_i: outer.velocity_i.raw(),
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
    state.tmc.store_outer_gains(
        crate::pid::test_outer_gains(
            cache.position_p,
            cache.position_i,
            cache.velocity_p,
            cache.velocity_i,
        ),
        Relaxed,
    );
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
        flux_p: crate::pid::test_gain(11),
        flux_i: crate::pid::test_gain(12),
        torque_p: crate::pid::test_gain(13),
        torque_i: crate::pid::test_gain(14),
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
            flux_p: crate::pid::test_gain(100),
            flux_i: crate::pid::test_gain(10),
            torque_p: crate::pid::test_gain(200),
            torque_i: crate::pid::test_gain(20),
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
        let published = tmc_state().load_outer_gains(Ordering::Relaxed).unwrap();
        assert_eq!(published.position_p.raw(), outer.position_p);
        assert_eq!(
            tmc_state().velocity_limit_cfg.load(Ordering::Relaxed),
            outer.velocity_limit
        );
        assert_eq!(published.position_i.raw(), outer.position_i);
        assert_eq!(published.velocity_p.raw(), outer.velocity_p);
        assert_eq!(published.velocity_i.raw(), outer.velocity_i);
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
    let evidence = crate::autotune::CurrentLoopEvidence::measured_symmetric();
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
                measured_axis_split_permille: 1000,
                applied_axis_split_permille: 1000,
                axis_split_clamped: 0,
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
        tmc_state().store_current_limit(test_support::current_limit(0x10), Ordering::Relaxed);
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
                flux_p: crate::pid::test_gain(commission_result.inner.flux_p),
                flux_i: crate::pid::test_gain(commission_result.inner.flux_i),
                torque_p: crate::pid::test_gain(commission_result.inner.torque_p),
                torque_i: crate::pid::test_gain(commission_result.inner.torque_i),
            })
        );
        let published = state.tmc.load_outer_gains(Ordering::Relaxed).unwrap();
        assert_eq!(
            published.position_p.raw(),
            commission_result.fallback_position_p
        );
        assert_eq!(
            published.position_i.raw(),
            commission_result.fallback_position_i
        );
        assert_eq!(
            published.velocity_p.raw(),
            commission_result.fallback_velocity_p
        );
        assert_eq!(
            published.velocity_i.raw(),
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
                assert_eq!(reply_result.inner.r_count_milli, 1706);
                assert_eq!(reply_result.inner.l_count_micro, 1245);
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmZeroPosition);
            }
            _ => panic!("expected CommissionResult"),
        }
        // Cleanup
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        store_published_controller_cache(&mut state, original);
        stepper_state().position.store(0, Ordering::Relaxed);
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
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
        tmc_state().store_current_limit(test_support::current_limit(0x10), Ordering::Relaxed);
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

        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
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
        tmc_state().store_current_limit(test_support::current_limit(0x10), Ordering::Relaxed);
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
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
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
            .store_current_limit(test_support::current_limit(0x10), Ordering::Relaxed);
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
        state.tmc.store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
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
        inner_warning_flags: 0,
        requested_velocity_mrev_s: 6000,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source:
            crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 1000,
        settle_travel_reserve_mrev: 100,
        negative_position_headroom_mrev: 2000,
        positive_position_headroom_mrev: 2000,
        max_duration_ms: 3000,
    }
}

fn sample_velocity_sweep_terminal(
    outcome: crate::commissioning::types::VelocitySweepOutcome,
) -> crate::commissioning::types::VelocitySweepTerminal {
    crate::commissioning::types::VelocitySweepTerminal {
        run_sequence: 9,
        evidence_sequence: 0,
        plan_digest: 0x1234,
        outcome,
        cause: 4,
        recovery_unavailable: false,
        model_direction_mask: 0,
        selected_memberships: [0; 3],
        nominated_p: 0,
        selected_intervals: [[0; 2]; 2],
        selected_common_intervals: [[0; 2]; 2],
        p_start: 256,
        p_top: 1_024,
        rung_count: 5,
        coverage_mask: 0,
        directional_region_counts: [0; 2],
        directional_fragment_counts: [0; 2],
        expected_observations: 20,
        emitted_observations: 4,
        expected_rungs: 5,
        emitted_rungs: 1,
        digest: 0x0123_4567_89ab_cdef,
        run_started_us: 1_000,
        run_completed_us: 2_000,
        safety_fault: None,
    }
}

#[cfg(feature = "trace-velocity-sweep")]
#[test]
fn stage_b_dispatch_rewrites_complete_from_the_reproduced_core() {
    use crate::commission_dispatch::stage_b::apply_stage_b_terminal;
    use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
    use crate::commissioning::types::VelocitySweepOutcome;

    let mut slot = StageBCandidateSlot::new();
    let mut first = sample_velocity_sweep_terminal(VelocitySweepOutcome::CompleteCandidate);
    first.selected_memberships = [0b11100, 0b11100, 0b11100];
    first.nominated_p = 724;
    first.selected_intervals = [[10, 20], [-20, -10]];
    first.selected_common_intervals = [[12, 18], [-18, -12]];
    let first_dispatch = apply_stage_b_terminal(first, 0, &mut slot);
    assert_eq!(
        first_dispatch.terminal.outcome,
        VelocitySweepOutcome::CompleteCandidate
    );
    assert_eq!(first_dispatch.reproduction, None);
    assert!(slot.candidate().is_some());

    let mut second = first;
    second.run_sequence += 1;
    second.run_started_us += 10_000;
    second.run_completed_us += 10_000;
    second.digest ^= 0x55aa;
    second.selected_memberships = [0b11110, 0b11100, 0b11110];
    second.nominated_p = 512;
    second.selected_intervals = [[11, 21], [-19, -9]];
    second.selected_common_intervals = [[13, 19], [-17, -11]];
    let second_dispatch = apply_stage_b_terminal(second, 0, &mut slot);
    assert_eq!(
        second_dispatch.terminal.outcome,
        VelocitySweepOutcome::Complete
    );
    let reproduction = second_dispatch
        .reproduction
        .expect("a retained sufficient run emits reproduction evidence");
    assert_eq!(second_dispatch.terminal.selected_memberships, [0b11100; 3]);
    assert_eq!(second_dispatch.terminal.nominated_p, 724);
    assert_eq!(
        second_dispatch.terminal.selected_intervals,
        [[11, 20], [-19, -10]]
    );
    assert_eq!(
        second_dispatch.terminal.selected_common_intervals,
        [[13, 18], [-17, -12]]
    );
    assert_eq!(second_dispatch.terminal.coverage_mask, 3);
    assert_eq!(
        second_dispatch.terminal.evidence_sequence,
        second.evidence_sequence + 1
    );
    let mut expected_digest = second.digest;
    crate::commission_dispatch::stage_b::for_each_stage_b_reproduction_record(
        reproduction,
        |record| {
            expected_digest = crate::ankyra::velocity_sweep_encoding::extend_velocity_sweep_digest(
                expected_digest,
                &record,
            );
        },
    );
    assert_eq!(second_dispatch.terminal.digest, expected_digest);
    assert_eq!(slot.candidate(), None);
}

#[cfg(feature = "trace-velocity-sweep")]
#[test]
fn stage_b_dispatch_faults_and_clears_noncontiguous_evidence() {
    use crate::commission_dispatch::stage_b::{
        STAGE_B_CAUSE_EVIDENCE_INTEGRITY, apply_stage_b_terminal,
    };
    use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
    use crate::commissioning::types::VelocitySweepOutcome;

    let mut slot = StageBCandidateSlot::new();
    let mut first = sample_velocity_sweep_terminal(VelocitySweepOutcome::CompleteCandidate);
    first.selected_memberships = [0b11100; 3];
    first.nominated_p = 724;
    first.selected_intervals = [[10, 20], [-20, -10]];
    first.selected_common_intervals = [[12, 18], [-18, -12]];
    apply_stage_b_terminal(first, 0, &mut slot);

    let mut invalid = first;
    invalid.selected_memberships[0] = 0b10100;
    invalid.run_sequence += 1;
    invalid.digest ^= 1;
    let dispatch = apply_stage_b_terminal(invalid, 0, &mut slot);

    assert_eq!(dispatch.terminal.outcome, VelocitySweepOutcome::Fault);
    assert_eq!(dispatch.terminal.cause, STAGE_B_CAUSE_EVIDENCE_INTEGRITY);
    assert_eq!(dispatch.reproduction, None);
    assert_eq!(slot.candidate(), None);
}

#[cfg(feature = "trace-velocity-sweep")]
#[test]
fn stage_b_schema6_reproduction_records_are_canonical_and_ordered() {
    use crate::commission_dispatch::stage_b::{
        StageBReproductionReport, for_each_stage_b_reproduction_record,
    };
    use crate::commissioning::outer::velocity::operating_region::CoverageKind;
    use crate::commissioning::outer::velocity::reproduction::{
        STAGE_B_EVIDENCE_SCHEMA_REVISION, StageBCoreCoverage, StageBIntervalComparison,
        StageBMembershipComparison, StageBReproductionDelta,
    };

    let report = StageBReproductionReport {
        run_sequence: 0x1122_3344,
        evidence_sequence: 0x5566,
        outcome: 1,
        delta: StageBReproductionDelta {
            memberships: [
                StageBMembershipComparison {
                    previous: 0x0038_0000,
                    current: 0x003c_0000,
                    core: 0x0038_0000,
                    previous_only: 0,
                    current_only: 0x0004_0000,
                    low_delta: -1,
                    high_delta: 0,
                },
                StageBMembershipComparison {
                    previous: 0x003c_0000,
                    current: 0x0038_0000,
                    core: 0x0038_0000,
                    previous_only: 0x0004_0000,
                    current_only: 0,
                    low_delta: 1,
                    high_delta: 0,
                },
                StageBMembershipComparison {
                    previous: 0x0038_0000,
                    current: 0x003c_0000,
                    core: 0x0038_0000,
                    previous_only: 0,
                    current_only: 0x0004_0000,
                    low_delta: -1,
                    high_delta: 0,
                },
            ],
            intervals: [
                StageBIntervalComparison {
                    previous_pooled: [100, 120],
                    current_pooled: [110, 130],
                    pooled_overlap: [110, 120],
                    previous_common: [104, 116],
                    current_common: [108, 118],
                    conservative_common: [108, 116],
                    conservative_common_nonempty: true,
                },
                StageBIntervalComparison {
                    previous_pooled: [-130, -100],
                    current_pooled: [-120, -90],
                    pooled_overlap: [-120, -100],
                    previous_common: [-125, -115],
                    current_common: [-110, -100],
                    conservative_common: [-110, -115],
                    conservative_common_nonempty: false,
                },
            ],
            previous_provisional_p: 1_448,
            current_provisional_p: 1_024,
            final_p: 1_448,
            coverage: [
                StageBCoreCoverage {
                    kind: CoverageKind::FullCoverage,
                    signed_rung_distance: 0,
                    gain_ratio_num: 1,
                    gain_ratio_den: 1,
                },
                StageBCoreCoverage {
                    kind: CoverageKind::TransferredCoverage,
                    signed_rung_distance: 1,
                    gain_ratio_num: 1_448,
                    gain_ratio_den: 1_024,
                },
            ],
            reduced_margin: false,
            reason_mask: 0,
            previous_run_digest: 0x0123_4567_89ab_cdef,
            current_run_digest: 0xfedc_ba98_7654_3210,
        },
    };
    let mut records = std::vec::Vec::new();
    for_each_stage_b_reproduction_record(report, |record| records.push(record));

    assert_eq!(
        records
            .iter()
            .map(|record| record.as_bytes()[0])
            .collect::<std::vec::Vec<_>>(),
        (10_u8..=20).collect::<std::vec::Vec<_>>()
    );
    assert_eq!(
        &records[1].as_bytes()[16..20],
        &0x0038_0000_u32.to_le_bytes()
    );
    assert_eq!(
        &records[1].as_bytes()[24..28],
        &0x0004_0000_u32.to_le_bytes()
    );
    assert_eq!(records[1].as_bytes()[28] as i8, -1);
    assert_eq!(records[1].as_bytes()[29] as i8, 0);
    assert_eq!(records[1].as_bytes().len(), 30);
    assert_eq!(records[7].as_bytes()[32], 0);
    assert_eq!(
        &records[10].as_bytes()[7..11],
        &0x89ab_cdef_u32.to_le_bytes()
    );
    assert_eq!(
        &records[10].as_bytes()[11..15],
        &0x0123_4567_u32.to_le_bytes()
    );
    assert_eq!(STAGE_B_EVIDENCE_SCHEMA_REVISION, 6);

    let digest = records
        .iter()
        .fold(report.delta.current_run_digest, |digest, record| {
            crate::ankyra::velocity_sweep_encoding::extend_velocity_sweep_digest(digest, record)
        });
    assert_ne!(digest, report.delta.current_run_digest);
}

#[cfg(feature = "trace-velocity-sweep")]
#[test]
fn outer_safety_timing_terminal_order_standalone_stage_b() {
    pollster::block_on(async {
        use crate::commissioning::outer::velocity::reproduction::{
            StageBCandidateSlot, StageBCandidateSummary, StageBPlanIdentity,
        };
        use crate::commissioning::types::VelocitySweepOutcome;

        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        let mut terminal = sample_velocity_sweep_terminal(VelocitySweepOutcome::CompleteCandidate);
        terminal.selected_memberships = [0b11100, 0b11100, 0b11100];
        terminal.nominated_p = 724;
        terminal.selected_intervals = [[10, 20], [-20, -10]];
        let mut slot = StageBCandidateSlot::new();
        slot.observe_sufficient(StageBCandidateSummary {
            identity: StageBPlanIdentity {
                channel: 0,
                build_revision:
                    crate::commissioning::outer::velocity::reproduction::STAGE_B_BUILD_REVISION,
                schema_revision: crate::commissioning::outer::velocity::reproduction::STAGE_B_EVIDENCE_SCHEMA_REVISION,
                plan_digest: terminal.plan_digest,
            },
            forward_membership: terminal.selected_memberships[0],
            reverse_membership: terminal.selected_memberships[1],
            joint_membership: terminal.selected_memberships[2],
            nominated_p: terminal.nominated_p,
            p_start: terminal.p_start,
            p_top: terminal.p_top,
            rung_count: terminal.rung_count,
            forward_interval: terminal.selected_intervals[0],
            reverse_interval: terminal.selected_intervals[1],
            forward_common_interval: terminal.selected_common_intervals[0],
            reverse_common_interval: terminal.selected_common_intervals[1],
            run_completed_us: terminal.run_completed_us.saturating_sub(100),
            run_digest: terminal.digest ^ 0x55aa,
        });

        let mut backend = MockBackend::new();
        let (_, _, mut gains) = fresh_state_locals();
        let mut calibrated = true;
        let mut motor_enabled = true;
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        let mut engine = Some(CommissioningEngine::new_outer(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
        ));
        seed_motor_sentinel(&mut backend).await;

        let outcome = drive_commission_result_with_stage_b(
            StepResult::VelocitySweepTerminal(terminal),
            &mut engine,
            &mut backend,
            &mut state,
            4095,
            0,
            &mut slot,
        )
        .await;

        let CommissionOutcome::Terminal {
            reply: ReplyPayload::VelocitySweepTerminal { terminal, .. },
            ..
        } = outcome
        else {
            panic!("matching second run must terminate")
        };
        assert_eq!(terminal.outcome, VelocitySweepOutcome::Complete);
        let kinds: std::vec::Vec<u8> = backend
            .velocity_sweep_trace_records
            .iter()
            .map(|record| record.as_bytes()[0])
            .collect();
        assert_eq!(kinds, [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 30, 8]);
        assert!(
            backend
                .trace_emission_order
                .windows(3)
                .any(|window| window == [30, 144, 8])
        );
        let traced_terminal = backend
            .velocity_sweep_trace_records
            .last()
            .expect("authoritative terminal trace");
        assert_eq!(
            *traced_terminal,
            crate::commissioning::outer::velocity::sweep_reporting::encode_velocity_sweep_terminal(
                terminal,
                backend.mechanical_encoder_quadrature_counts_per_rev_value,
                backend.tmc_config.pwm_freq_hz(),
            )
        );
    });
}

#[cfg(feature = "trace-velocity-sweep")]
#[test]
fn stage_b_terminal_mismatch_replaces_candidate_and_insufficient_clears_it() {
    use crate::commission_dispatch::stage_b::{
        STAGE_B_CAUSE_REPRODUCTION_MISMATCH, apply_stage_b_terminal,
    };
    use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
    use crate::commissioning::types::VelocitySweepOutcome;

    let mut slot = StageBCandidateSlot::new();
    let mut first = sample_velocity_sweep_terminal(VelocitySweepOutcome::CompleteCandidate);
    first.selected_memberships = [0b11100, 0b11100, 0b11100];
    first.nominated_p = 724;
    first.selected_intervals = [[10, 20], [-20, -10]];
    apply_stage_b_terminal(first, 0, &mut slot);

    let mut mismatch = first;
    mismatch.run_sequence += 1;
    mismatch.selected_memberships[0] = 0b01110;
    mismatch.digest ^= 0xaa55;
    let mismatch_dispatch = apply_stage_b_terminal(mismatch, 0, &mut slot);
    assert_eq!(
        mismatch_dispatch.terminal.outcome,
        VelocitySweepOutcome::Inconclusive
    );
    assert_eq!(
        mismatch_dispatch.terminal.cause,
        STAGE_B_CAUSE_REPRODUCTION_MISMATCH
    );
    assert_eq!(
        slot.candidate()
            .expect("replacement candidate")
            .forward_membership,
        mismatch.selected_memberships[0]
    );
    assert_eq!(
        slot.candidate().expect("replacement candidate").run_digest,
        mismatch_dispatch.terminal.digest
    );

    let insufficient = sample_velocity_sweep_terminal(VelocitySweepOutcome::Inconclusive);
    let cleared = apply_stage_b_terminal(insufficient, 0, &mut slot);
    assert_eq!(cleared.reproduction, None);
    assert_eq!(slot.candidate(), None);
}

#[test]
fn every_velocity_sweep_terminal_disarms_without_rearming() {
    pollster::block_on(async {
        use crate::commissioning::types::VelocitySweepOutcome;
        use core::sync::atomic::Ordering;

        let _guard = test_support::SHARED_STATE_MUTEX.lock().await;
        for expected in [
            VelocitySweepOutcome::CompleteCandidate,
            VelocitySweepOutcome::Complete,
            VelocitySweepOutcome::Inconclusive,
            VelocitySweepOutcome::Fault,
        ] {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            let (_, _, mut gains) = fresh_state_locals();
            let mut calibrated = true;
            let mut motor_enabled = true;
            tmc_state().motor_armed.store(true, Ordering::Relaxed);
            let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
            let mut engine = Some(CommissioningEngine::new_outer(
                0,
                sample_outer_params(),
                sample_entry_snapshot(),
            ));
            seed_motor_sentinel(&mut backend).await;

            let outcome = drive_commission_result(
                StepResult::VelocitySweepTerminal(sample_velocity_sweep_terminal(expected)),
                &mut engine,
                &mut backend,
                &mut state,
                4095,
            )
            .await;

            assert!(engine.is_none());
            assert!(!*state.calibrated);
            assert!(!*state.motor_enabled);
            assert!(!backend.enable_pin);
            assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
            match outcome {
                CommissionOutcome::Terminal {
                    reply: ReplyPayload::VelocitySweepTerminal { terminal, .. },
                    actions,
                } => {
                    assert_eq!(terminal.outcome, expected);
                    assert_eq!(actions.stepper, StepperPostAction::Stop);
                    assert!(actions.stop_trace);
                }
                _ => panic!("expected velocity-sweep terminal"),
            }
            clear_motor_sentinel();
        }
    });
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
