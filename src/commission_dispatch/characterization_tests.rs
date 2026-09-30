use super::test_support::{
    MockBackend, SpiOp, accepted_commission_result, breakaway_campaign_engine, dispatch_state,
    fresh_state_locals, robustness_reversal_engine,
};
use super::*;
use crate::autotune::CommissionProfile;
use crate::commissioning::backend::CommissionMotorControl;
use crate::commissioning::schedule::CommissionTerminal;
use crate::commissioning::types::RobustnessOutcome;
use crate::tmc_control::lifecycle::LifecycleFlag;
use foci_firmware_macros::fixture_test;

const TEST_MOTOR_SENTINEL: u32 = 0x0002_0032;
const TEST_RUN_CURRENT_LIMIT: u16 = 0x1234;

fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
    crate::state::tmc::tmc_channel0()
}

async fn seed_motor_sentinel(backend: &mut MockBackend) {
    tmc_state()
        .motor_sentinel
        .store(TEST_MOTOR_SENTINEL, core::sync::atomic::Ordering::Relaxed);
    tmc_state().store_current_limit(
        foci_tmc4671::current_limit::PidTorqueFluxLimit::new(u32::from(TEST_RUN_CURRENT_LIMIT))
            .unwrap(),
        core::sync::atomic::Ordering::Relaxed,
    );
    backend
        .write_register(
            foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
            TEST_MOTOR_SENTINEL,
        )
        .await
        .expect("test MOTOR_TYPE sentinel write should succeed");
}

fn sample_outer_result() -> crate::commissioning::types::OuterResult {
    crate::commissioning::types::OuterResult {
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
        warning_code: 0,
        provenance: crate::commissioning::types::TuneProvenance::default(),
        position_tune: crate::commissioning::types::PositionTuneProvenance::default(),
        position_tune_outcome_code: 0,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OutcomeTag {
    Continue,
    PhaseChanged,
    Evidence,
    Terminal {
        stop_trace: bool,
        stepper: StepperPostAction,
        outcome_class: Option<u8>,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct OpTrace {
    events: std::vec::Vec<SpiOp>,
    outcome_tag: OutcomeTag,
    terminal_cause: Option<u16>,
    engine_consumed: bool,
}

fn integral_cause_code(cause: crate::commissioning::types::IntegralOutcomeCause) -> u16 {
    (u16::from(cause.namespace_code()) << 8) | u16::from(cause.raw_value())
}

fn terminal_shape(reply: &ReplyPayload) -> (Option<u8>, Option<u16>) {
    match reply {
        ReplyPayload::CommissionPhase { status, .. } => (None, Some(u16::from(*status))),
        ReplyPayload::CommissionResult { status, .. } => (None, Some(u16::from(*status))),
        ReplyPayload::TuneResult { status, .. } => (None, Some(u16::from(*status))),
        ReplyPayload::CalibrateResult { status, .. } => (None, Some(u16::from(*status))),
        ReplyPayload::Selftest { done_status, .. } => (None, Some(u16::from(*done_status))),
        ReplyPayload::VelocityIntegralTerminal { terminal, .. }
        | ReplyPayload::VelocityIntegralRefusal { terminal, .. } => (
            Some(terminal.outcome as u8),
            Some(integral_cause_code(terminal.cause)),
        ),
        ReplyPayload::RobustnessReversalTerminal { terminal, .. } => (
            Some(terminal.outcome as u8),
            Some(u16::from(terminal.cause)),
        ),
        _ => (None, None),
    }
}

async fn drive_and_trace(
    result: StepResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut MockBackend,
    state: &mut DispatchState<'_>,
) -> (CommissionOutcome, OpTrace) {
    let outcome = drive_commission_result(result, engine, backend, state, 4095).await;

    let (outcome_tag, terminal_cause) = match &outcome {
        CommissionOutcome::Continue => (OutcomeTag::Continue, None),
        CommissionOutcome::PhaseChanged { .. } => (OutcomeTag::PhaseChanged, None),
        CommissionOutcome::Evidence { .. } => (OutcomeTag::Evidence, None),
        CommissionOutcome::Terminal { reply, actions } => {
            let (outcome_class, cause) = terminal_shape(reply);
            (
                OutcomeTag::Terminal {
                    stop_trace: actions.stop_trace,
                    stepper: actions.stepper,
                    outcome_class,
                },
                cause,
            )
        }
    };

    if let CommissionOutcome::Terminal { actions, .. } = &outcome {
        let _ = apply_stepper_post_action_state(actions.stepper, state);
        if actions.stepper == StepperPostAction::ArmPreservePosition {
            backend.record_lifecycle(LifecycleFlag::MotorArmed, true);
        }
    }

    let trace = OpTrace {
        events: backend.spi_log.clone(),
        outcome_tag,
        terminal_cause,
        engine_consumed: engine.is_none(),
    };
    (outcome, trace)
}

async fn run_variant(
    result: StepResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut MockBackend,
    state: &mut DispatchState<'_>,
) -> OpTrace {
    drive_and_trace(result, engine, backend, state).await.1
}

fn index_of(events: &[SpiOp], target: SpiOp) -> Option<usize> {
    events.iter().position(|op| *op == target)
}

fn contains(events: &[SpiOp], target: SpiOp) -> bool {
    index_of(events, target).is_some()
}

#[fixture_test]
async fn calibrate_done_success_arms_electrical_before_steps() {
    let mut backend = MockBackend::new();
    let (mut cal, mut en, mut gains) = fresh_state_locals();
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_calibrate(0));
    seed_motor_sentinel(&mut backend).await;
    let cal_result = crate::commissioning::types::AdcCalResult {
        adc_i0: 0,
        adc_i1: 0,
        encoder_count: 0,
        encoder_alignment_electrical_residual_counts: 0,
        encoder_alignment_stability_counts: 0,
        encoder_alignment_movement_counts: 0,
        encoder_alignment_min_movement_counts: 0,
        encoder_alignment_counts_per_electrical_rev: 0,
        adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
        adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
    };

    let trace = run_variant(
        StepResult::CalibrateDone(cal_result),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: false,
            stepper: StepperPostAction::ArmPreservePosition,
            outcome_class: None,
        }
    );
    assert_eq!(trace.terminal_cause, Some(0));

    let enabled_index = index_of(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: true,
        },
    )
    .expect("arm_motor_electrical must record MotorEnabled(true)");
    let armed_index = index_of(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: true,
        },
    )
    .expect("apply_stepper_post_action_state must record MotorArmed(true)");
    assert!(
        enabled_index < armed_index,
        "arm_motor_electrical must complete before arm_motor_for_steps runs"
    );
    assert!(contains(&trace.events, SpiOp::EnablePin(true)));
}

#[fixture_test]
async fn calibrate_done_gain_restore_failure_disarms_without_arming() {
    let mut backend = MockBackend::new();
    backend.fail_writes_remaining = Some((foci_tmc4671::registers::PID_FLUX_P_FLUX_I.addr(), 1));
    let (mut cal, mut en, _) = fresh_state_locals();
    let mut gains = Some(crate::pid::InnerLoopGains {
        flux_p: crate::pid::p_gain(100),
        flux_i: crate::pid::i_gain(10),
        torque_p: crate::pid::p_gain(200),
        torque_i: crate::pid::i_gain(20),
    });
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_calibrate(0));
    seed_motor_sentinel(&mut backend).await;
    let cal_result = crate::commissioning::types::AdcCalResult {
        adc_i0: 0,
        adc_i1: 0,
        encoder_count: 0,
        encoder_alignment_electrical_residual_counts: 0,
        encoder_alignment_stability_counts: 0,
        encoder_alignment_movement_counts: 0,
        encoder_alignment_min_movement_counts: 0,
        encoder_alignment_counts_per_electrical_rev: 0,
        adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
        adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
    };

    let trace = run_variant(
        StepResult::CalibrateDone(cal_result),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: false,
            stepper: StepperPostAction::Stop,
            outcome_class: None,
        }
    );
    assert_eq!(
        trace.terminal_cause,
        Some(u16::from(
            crate::calibration::CalibrationError::SpiError.status_code()
        ))
    );
    assert!(
        !contains(
            &trace.events,
            SpiOp::Lifecycle {
                flag: LifecycleFlag::MotorEnabled,
                value: true,
            },
        ),
        "a mid-restore failure must never reach arm_motor_electrical"
    );
    assert!(!backend.enable_pin);
}

#[fixture_test]
async fn calibrate_arm_pending_config_drain_failure_takes_the_existing_error_path() {
    let mut backend = MockBackend::new();
    let (mut cal, mut en, mut gains) = fresh_state_locals();
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_calibrate(0));
    seed_motor_sentinel(&mut backend).await;
    tmc_state().config_apply_pending.store(
        crate::state::tmc::PENDING_CURRENT,
        core::sync::atomic::Ordering::Relaxed,
    );
    backend.pending_config_result =
        Err(crate::tmc_control::pending_config::PendingConfigItem::Current);
    let cal_result = crate::commissioning::types::AdcCalResult {
        adc_i0: 0,
        adc_i1: 0,
        encoder_count: 0,
        encoder_alignment_electrical_residual_counts: 0,
        encoder_alignment_stability_counts: 0,
        encoder_alignment_movement_counts: 0,
        encoder_alignment_min_movement_counts: 0,
        encoder_alignment_counts_per_electrical_rev: 0,
        adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
        adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
    };

    let trace = run_variant(
        StepResult::CalibrateDone(cal_result),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: false,
            stepper: StepperPostAction::Stop,
            outcome_class: None,
        },
        "a refused arm must land in CalibrateDisarm, not panic or arm anyway"
    );
    assert_eq!(
        trace.terminal_cause,
        Some(u16::from(
            crate::calibration::CalibrationError::SpiError.status_code()
        ))
    );
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: true,
        },
    ));
    assert!(!backend.enable_pin);
}

#[fixture_test]
async fn inner_failed_calibrate_uses_safe_state_restore_not_disarm_motor() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_calibrate(2));

    let trace = run_variant(
        StepResult::InnerFailed(crate::autotune::CommissionError::EncoderFault),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::None,
            outcome_class: None,
        }
    );
    assert_eq!(
        trace.terminal_cause,
        Some(u16::from(
            crate::calibration::CalibrationError::EncoderFault.status_code()
        ))
    );
    assert!(!backend.enable_pin);
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: false,
        },
    ));
    assert!(*state.calibrated);
    assert!(*state.motor_enabled);
}

#[fixture_test]
async fn selftest_done_restores_gains_without_lifecycle_events() {
    let mut backend = MockBackend::new();
    let (mut cal, mut en, _) = fresh_state_locals();
    let mut gains = Some(crate::pid::InnerLoopGains {
        flux_p: crate::pid::p_gain(100),
        flux_i: crate::pid::i_gain(10),
        torque_p: crate::pid::p_gain(200),
        torque_i: crate::pid::i_gain(20),
    });
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_selftest(3));

    let (outcome, trace) = drive_and_trace(
        StepResult::SelftestDone,
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: false,
            stepper: StepperPostAction::None,
            outcome_class: None,
        }
    );
    assert_eq!(trace.terminal_cause, Some(0));
    match outcome {
        CommissionOutcome::Terminal {
            reply: ReplyPayload::Selftest { oid, .. },
            ..
        } => assert_eq!(oid, 3),
        _ => panic!("expected Selftest"),
    }
    assert!(trace.events.iter().any(|op| matches!(
        op,
        SpiOp::Write(addr, _) if *addr == foci_tmc4671::registers::PID_FLUX_P_FLUX_I.addr()
    )));
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: true,
        },
    ));
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: true,
        },
    ));
}

#[fixture_test]
async fn commission_done_success_arms_electrical_before_preserve_position_arm() {
    let mut backend = MockBackend::new();
    let (mut cal, mut en, mut gains) = fresh_state_locals();
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_commission(
        0,
        CommissionProfile::Balanced,
    ));
    seed_motor_sentinel(&mut backend).await;

    let trace = run_variant(
        StepResult::CommissionDone(accepted_commission_result()),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::ArmPreservePosition,
            outcome_class: None,
        }
    );
    let enabled_index = index_of(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: true,
        },
    )
    .expect("must record MotorEnabled(true)");
    let armed_index = index_of(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: true,
        },
    )
    .expect("must record MotorArmed(true)");
    assert!(enabled_index < armed_index);
}

#[fixture_test]
async fn commission_done_failed_status_skips_disarm_motor_and_lifecycle_events() {
    let mut backend = MockBackend::new();
    let (mut cal, mut en, mut gains) = fresh_state_locals();
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_commission(
        0,
        CommissionProfile::Balanced,
    ));
    let mut commission_result = accepted_commission_result();
    commission_result.inner.status = crate::autotune::CommissionStatus::Failed(
        crate::autotune::CommissionError::CurrentValidationFailed,
    );

    let trace = run_variant(
        StepResult::CommissionDone(commission_result),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
            outcome_class: None,
        }
    );
    assert_eq!(
        trace.terminal_cause,
        Some(u16::from(
            crate::autotune::CommissionError::CurrentValidationFailed.status_code()
        ))
    );
    assert!(trace.events.is_empty());
    assert!(!*state.calibrated);
    assert!(!*state.motor_enabled);
}

#[fixture_test]
async fn commission_done_arm_failure_disarms_without_reaching_arm_for_steps() {
    let mut backend = MockBackend::new();
    let (mut cal, mut en, mut gains) = fresh_state_locals();
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_commission(
        0,
        CommissionProfile::Balanced,
    ));

    let trace = run_variant(
        StepResult::CommissionDone(accepted_commission_result()),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
            outcome_class: None,
        }
    );
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: true,
        },
    ));
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: true,
        },
    ));
    assert!(!backend.enable_pin);
}

#[fixture_test]
async fn outer_done_success_disarms_without_arming() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_commission(
        0,
        CommissionProfile::Balanced,
    ));

    let trace = run_variant(
        StepResult::OuterDone(sample_outer_result()),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
            outcome_class: None,
        }
    );
    assert!(contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: false,
        },
    ));
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: true,
        },
    ));
    assert!(!backend.enable_pin);
}

#[fixture_test]
async fn outer_failed_safety_fault_disarms_without_restore_attempt() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(robustness_reversal_engine());
    let err = crate::autotune::CommissionError::SafetyEnvelopeViolation;

    let trace = run_variant(
        StepResult::OuterFailed(err),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
            outcome_class: None,
        }
    );
    assert_eq!(trace.terminal_cause, Some(u16::from(err.status_code())));
    assert!(backend.delay_us_log.is_empty());
    assert!(!backend.enable_pin);
}

#[fixture_test]
async fn outer_failed_soft_failure_arms_electrical_and_holds_position() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(robustness_reversal_engine());
    let err = crate::autotune::CommissionError::NoCurrent;
    seed_motor_sentinel(&mut backend).await;
    advance_clock_for_stability_check(&mut backend);

    let trace = run_variant(
        StepResult::OuterFailed(err),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::ArmPreservePosition,
            outcome_class: None,
        }
    );
    assert_eq!(trace.terminal_cause, Some(u16::from(err.status_code())));
    let enabled_index = index_of(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: true,
        },
    )
    .expect("soft failure must still arm electrically");
    let armed_index = index_of(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: true,
        },
    )
    .expect("must record MotorArmed(true)");
    assert!(enabled_index < armed_index);
}

#[fixture_test]
async fn robustness_reversal_terminal_never_arms_even_on_complete_outcome() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(robustness_reversal_engine());
    let mut terminal = robustness::failed_robustness_terminal(
        1,
        robustness::RobustnessPreflightRejection::NoAcceptedCandidate,
    );
    terminal.outcome = RobustnessOutcome::Complete;
    terminal.cause = robustness::ROBUSTNESS_CAUSE_PASS;

    let trace = run_variant(
        StepResult::RobustnessReversalTerminal(terminal),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
            outcome_class: Some(RobustnessOutcome::Complete as u8),
        }
    );
    assert!(!contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: true,
        },
    ));
    assert!(!backend.enable_pin);
}

#[fixture_test]
async fn robustness_reversal_terminal_restores_baseline_before_disarm_on_reject() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(robustness_reversal_engine());
    let mut terminal = robustness::failed_robustness_terminal(
        1,
        robustness::RobustnessPreflightRejection::NoAcceptedCandidate,
    );
    terminal.outcome = RobustnessOutcome::Rejected;
    terminal.cause = robustness::ROBUSTNESS_CAUSE_IAE_EXCEEDED;

    let trace = run_variant(
        StepResult::RobustnessReversalTerminal(terminal),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    let OutcomeTag::Terminal { outcome_class, .. } = trace.outcome_tag else {
        panic!("expected Terminal");
    };
    assert_eq!(outcome_class, Some(RobustnessOutcome::Rejected as u8));
    let velocity_gain_index = trace
        .events
        .iter()
        .position(|op| {
            matches!(
                op,
                SpiOp::Write(addr, _)
                    if *addr == foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
            )
        })
        .expect("baseline velocity gains must be restored before disarm");
    let disarmed_index = index_of(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: false,
        },
    )
    .expect("must record MotorArmed(false)");
    assert!(
        velocity_gain_index < disarmed_index,
        "the entry-snapshot baseline restore must precede disarm"
    );
}

#[fixture_test]
async fn velocity_integral_terminal_without_operational_limit_disarms_directly() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_commission(
        0,
        CommissionProfile::Balanced,
    ));

    let trace = run_variant(
        StepResult::VelocityIntegralTerminal(sample_velocity_integral_terminal()),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
            outcome_class: Some(
                crate::commissioning::types::VelocityIntegralOutcome::Inconclusive as u8
            ),
        }
    );
    assert!(contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: false,
        },
    ));
}

#[fixture_test]
async fn velocity_integral_terminal_restore_failure_skips_disarm_leaving_motor_armed() {
    let mut backend = MockBackend::new();
    backend.fail_on_write = Some(foci_tmc4671::registers::PID_VELOCITY_TARGET.addr());
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(breakaway_campaign_engine());

    let (outcome, trace) = drive_and_trace(
        StepResult::VelocityIntegralTerminal(sample_velocity_integral_terminal()),
        &mut engine,
        &mut backend,
        &mut state,
    )
    .await;

    assert!(trace.engine_consumed);
    assert_eq!(
        trace.outcome_tag,
        OutcomeTag::Terminal {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
            outcome_class: Some(crate::commissioning::types::VelocityIntegralOutcome::Fault as u8),
        }
    );
    match outcome {
        CommissionOutcome::Terminal {
            reply: ReplyPayload::VelocityIntegralTerminal { terminal, .. },
            ..
        } => {
            assert!(terminal.recovery_unavailable);
            assert!(terminal.safety_fault.is_none());
        }
        _ => panic!("expected VelocityIntegralTerminal"),
    }
    assert!(
        !contains(
            &trace.events,
            SpiOp::Lifecycle {
                flag: LifecycleFlag::MotorArmed,
                value: false,
            },
        ),
        "disarm_motor must never run when the velocity-workflow restore \
         fails before reaching it"
    );
}

fn sample_velocity_integral_terminal() -> crate::commissioning::types::VelocityIntegralTerminal {
    use crate::commissioning::outer::velocity::integral_evidence::{
        ExactRateInterval, IntegralRunEvidence,
    };
    use crate::commissioning::types::VelocityIntegralOutcome;

    let evidence = IntegralRunEvidence::new([
        ExactRateInterval::new(100, 120).unwrap(),
        ExactRateInterval::new(-120, -100).unwrap(),
    ]);
    crate::commissioning::types::VelocityIntegralTerminal {
        run_sequence: 1,
        evidence_sequence: 1,
        plan_digest: 0x1111_2222_3333_4444,
        digest: 0x5555_6666_7777_8888,
        outcome: VelocityIntegralOutcome::Inconclusive,
        cause: crate::commissioning::types::IntegralOutcomeCause::Engine(0),
        breakaway_acceptance: false,
        fixed_p: 0,
        candidate_i: None,
        recovery_unavailable: false,
        recovered_with_current_headroom: false,
        recovery_quantization_exposed: false,
        probe_constrained_test_point: false,
        rest_boundary_rung_plus_one: 0,
        rest_boundary_slot_plus_one: 0,
        evidence,
        expected_observations: 0,
        emitted_observations: 0,
        expected_rungs: 0,
        emitted_rungs: 0,
        run_started_us: 0,
        run_completed_us: 0,
        safety_fault: None,
        rest_rejection_after_sufficiency: false,
        rest_rejection_owner: None,
    }
}

struct TerminalVariant {
    name: &'static str,
    result: fn() -> StepResult,
    engine: fn() -> CommissioningEngine,
    prepare: fn(&mut MockBackend),
    seed_sentinel: bool,
    entry_calibrated: bool,
    entry_motor_enabled: bool,
    stored_gains: fn() -> Option<crate::pid::InnerLoopGains>,
    reaches_single_op_arm: bool,
}

fn no_gains() -> Option<crate::pid::InnerLoopGains> {
    None
}

fn saved_gains() -> Option<crate::pid::InnerLoopGains> {
    Some(crate::pid::InnerLoopGains {
        flux_p: crate::pid::p_gain(100),
        flux_i: crate::pid::i_gain(10),
        torque_p: crate::pid::p_gain(200),
        torque_i: crate::pid::i_gain(20),
    })
}

fn no_prepare(_: &mut MockBackend) {}

fn sample_adc_cal_result() -> crate::commissioning::types::AdcCalResult {
    zero_adc_cal_result()
}

fn calibrate_engine() -> CommissioningEngine {
    CommissioningEngine::new_calibrate(0)
}

fn selftest_engine() -> CommissioningEngine {
    CommissioningEngine::new_selftest(3)
}

fn commission_engine() -> CommissioningEngine {
    CommissioningEngine::new_commission(0, CommissionProfile::Balanced)
}

fn failed_commission_result() -> StepResult {
    let mut result = accepted_commission_result();
    result.inner.status = crate::autotune::CommissionStatus::Failed(
        crate::autotune::CommissionError::CurrentValidationFailed,
    );
    StepResult::CommissionDone(result)
}

fn rejected_robustness_terminal() -> StepResult {
    let mut terminal = robustness::failed_robustness_terminal(
        1,
        robustness::RobustnessPreflightRejection::NoAcceptedCandidate,
    );
    terminal.outcome = RobustnessOutcome::Rejected;
    terminal.cause = robustness::ROBUSTNESS_CAUSE_IAE_EXCEEDED;
    StepResult::RobustnessReversalTerminal(terminal)
}

fn complete_robustness_terminal() -> StepResult {
    let mut terminal = robustness::failed_robustness_terminal(
        1,
        robustness::RobustnessPreflightRejection::NoAcceptedCandidate,
    );
    terminal.outcome = RobustnessOutcome::Complete;
    terminal.cause = robustness::ROBUSTNESS_CAUSE_PASS;
    StepResult::RobustnessReversalTerminal(terminal)
}

fn fail_flux_gain_write(backend: &mut MockBackend) {
    backend.fail_writes_remaining = Some((foci_tmc4671::registers::PID_FLUX_P_FLUX_I.addr(), 1));
}

fn fail_velocity_target_write(backend: &mut MockBackend) {
    backend.fail_on_write = Some(foci_tmc4671::registers::PID_VELOCITY_TARGET.addr());
}

fn advance_clock_for_stability_check(backend: &mut MockBackend) {
    backend.set_elapsed_us_sequence(&[
        0, 5_000, 5_000, 10_000, 10_000, 15_000, 15_000, 20_000, 20_000, 25_000, 25_000,
    ]);
    backend.elapsed_us_value = 25_000;
}

fn all_terminal_variants() -> [TerminalVariant; 21] {
    const BASE: TerminalVariant = TerminalVariant {
        name: "",
        result: || StepResult::SelftestDone,
        engine: selftest_engine,
        prepare: no_prepare,
        seed_sentinel: false,
        entry_calibrated: false,
        entry_motor_enabled: false,
        stored_gains: no_gains,
        reaches_single_op_arm: false,
    };
    [
        TerminalVariant {
            name: "calibrate_done_success",
            result: || StepResult::CalibrateDone(sample_adc_cal_result()),
            engine: calibrate_engine,
            seed_sentinel: true,
            reaches_single_op_arm: true,
            ..BASE
        },
        TerminalVariant {
            name: "calibrate_done_gain_restore_failure",
            result: || StepResult::CalibrateDone(sample_adc_cal_result()),
            engine: calibrate_engine,
            prepare: fail_flux_gain_write,
            seed_sentinel: true,
            stored_gains: saved_gains,
            ..BASE
        },
        TerminalVariant {
            name: "calibrate_done_arm_failure",
            result: || StepResult::CalibrateDone(sample_adc_cal_result()),
            engine: calibrate_engine,
            ..BASE
        },
        TerminalVariant {
            name: "inner_failed_calibrate",
            result: || StepResult::InnerFailed(crate::autotune::CommissionError::EncoderFault),
            engine: || CommissioningEngine::new_calibrate(2),
            entry_calibrated: true,
            entry_motor_enabled: true,
            ..BASE
        },
        TerminalVariant {
            name: "inner_failed_selftest",
            result: || StepResult::InnerFailed(crate::autotune::CommissionError::Spi),
            engine: selftest_engine,
            stored_gains: saved_gains,
            ..BASE
        },
        TerminalVariant {
            name: "inner_failed_commission",
            result: || StepResult::InnerFailed(crate::autotune::CommissionError::NoCurrent),
            engine: commission_engine,
            ..BASE
        },
        TerminalVariant {
            name: "selftest_done",
            result: || StepResult::SelftestDone,
            engine: selftest_engine,
            stored_gains: saved_gains,
            ..BASE
        },
        TerminalVariant {
            name: "selftest_done_gain_restore_failure",
            result: || StepResult::SelftestDone,
            engine: selftest_engine,
            prepare: fail_flux_gain_write,
            stored_gains: saved_gains,
            ..BASE
        },
        TerminalVariant {
            name: "commission_done_success",
            result: || StepResult::CommissionDone(accepted_commission_result()),
            engine: commission_engine,
            seed_sentinel: true,
            reaches_single_op_arm: true,
            ..BASE
        },
        TerminalVariant {
            name: "commission_done_failed_status",
            result: failed_commission_result,
            engine: commission_engine,
            ..BASE
        },
        TerminalVariant {
            name: "commission_done_arm_failure",
            result: || StepResult::CommissionDone(accepted_commission_result()),
            engine: commission_engine,
            ..BASE
        },
        TerminalVariant {
            name: "outer_done_success",
            result: || StepResult::OuterDone(sample_outer_result()),
            engine: commission_engine,
            ..BASE
        },
        TerminalVariant {
            name: "outer_done_invalid_gains",
            result: || {
                let mut outer = sample_outer_result();
                outer.velocity_p = 0x8000;
                StepResult::OuterDone(outer)
            },
            engine: commission_engine,
            ..BASE
        },
        TerminalVariant {
            name: "outer_failed_safety_fault",
            result: || {
                StepResult::OuterFailed(crate::autotune::CommissionError::SafetyEnvelopeViolation)
            },
            engine: robustness_reversal_engine,
            entry_calibrated: true,
            entry_motor_enabled: true,
            reaches_single_op_arm: true,
            ..BASE
        },
        TerminalVariant {
            name: "outer_failed_soft_failure",
            result: || StepResult::OuterFailed(crate::autotune::CommissionError::NoCurrent),
            engine: robustness_reversal_engine,
            prepare: advance_clock_for_stability_check,
            seed_sentinel: true,
            entry_calibrated: true,
            entry_motor_enabled: true,
            reaches_single_op_arm: true,
            ..BASE
        },
        TerminalVariant {
            name: "outer_failed_safety_fault_velocity_limit",
            result: || StepResult::OuterFailed(crate::autotune::CommissionError::Shutdown),
            engine: breakaway_campaign_engine,
            seed_sentinel: true,
            entry_calibrated: true,
            entry_motor_enabled: true,
            ..BASE
        },
        TerminalVariant {
            name: "outer_failed_soft_failure_arm_failure",
            result: || StepResult::OuterFailed(crate::autotune::CommissionError::NoCurrent),
            engine: robustness_reversal_engine,
            prepare: advance_clock_for_stability_check,
            entry_calibrated: true,
            entry_motor_enabled: true,
            reaches_single_op_arm: true,
            ..BASE
        },
        TerminalVariant {
            name: "robustness_reversal_complete",
            result: complete_robustness_terminal,
            engine: robustness_reversal_engine,
            entry_calibrated: true,
            entry_motor_enabled: true,
            ..BASE
        },
        TerminalVariant {
            name: "robustness_reversal_rejected",
            result: rejected_robustness_terminal,
            engine: robustness_reversal_engine,
            entry_calibrated: true,
            entry_motor_enabled: true,
            reaches_single_op_arm: true,
            ..BASE
        },
        TerminalVariant {
            name: "velocity_integral_terminal",
            result: || StepResult::VelocityIntegralTerminal(sample_velocity_integral_terminal()),
            engine: commission_engine,
            entry_calibrated: true,
            entry_motor_enabled: true,
            ..BASE
        },
        TerminalVariant {
            name: "velocity_integral_terminal_restore_failure",
            result: || StepResult::VelocityIntegralTerminal(sample_velocity_integral_terminal()),
            engine: breakaway_campaign_engine,
            prepare: fail_velocity_target_write,
            entry_calibrated: true,
            entry_motor_enabled: true,
            ..BASE
        },
    ]
}

fn reset_channel0() {
    crate::state::tmc::tmc_channel0().reset();
    crate::state::stepper::stepper_channel0().reset();
}

async fn prepare_backend(variant: &TerminalVariant, backend: &mut MockBackend) {
    reset_channel0();
    (variant.prepare)(backend);
    if variant.seed_sentinel {
        seed_motor_sentinel(backend).await;
    }
}

#[derive(Debug, PartialEq, Eq)]
struct TraceSummary {
    events: std::vec::Vec<SpiOp>,
    outcome_tag: OutcomeTag,
    terminal_cause: Option<u16>,
    engine_consumed: bool,
}

fn trace_of(
    outcome: &CommissionOutcome,
    backend: &MockBackend,
    engine: &Option<CommissioningEngine>,
) -> TraceSummary {
    let (outcome_tag, terminal_cause) = match outcome {
        CommissionOutcome::Continue => (OutcomeTag::Continue, None),
        CommissionOutcome::PhaseChanged { .. } => (OutcomeTag::PhaseChanged, None),
        CommissionOutcome::Evidence { .. } => (OutcomeTag::Evidence, None),
        CommissionOutcome::Terminal { reply, actions } => {
            let (outcome_class, cause) = terminal_shape(reply);
            (
                OutcomeTag::Terminal {
                    stop_trace: actions.stop_trace,
                    stepper: actions.stepper,
                    outcome_class,
                },
                cause,
            )
        }
    };
    TraceSummary {
        events: backend.spi_log.to_vec(),
        outcome_tag,
        terminal_cause,
        engine_consumed: engine.is_none(),
    }
}

fn apply_post_actions(
    outcome: &CommissionOutcome,
    backend: &mut MockBackend,
    state: &mut DispatchState<'_>,
) {
    if let CommissionOutcome::Terminal { actions, .. } = outcome {
        let _ = apply_stepper_post_action_state(actions.stepper, state);
        if actions.stepper == StepperPostAction::ArmPreservePosition {
            backend.record_lifecycle(LifecycleFlag::MotorArmed, true);
        }
    }
}

async fn run_variant_via_driver(variant: &TerminalVariant) -> TraceSummary {
    let mut backend = MockBackend::new();
    prepare_backend(variant, &mut backend).await;
    let mut calibrated = variant.entry_calibrated;
    let mut motor_enabled = variant.entry_motor_enabled;
    let mut gains = (variant.stored_gains)();
    let mut engine = Some((variant.engine)());

    let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
    let outcome = drive_commission_result(
        (variant.result)(),
        &mut engine,
        &mut backend,
        &mut state,
        4095,
    )
    .await;
    apply_post_actions(&outcome, &mut backend, &mut state);
    trace_of(&outcome, &backend, &engine)
}

async fn run_variant_via_machine(variant: &TerminalVariant) -> TraceSummary {
    let mut backend = MockBackend::new();
    prepare_backend(variant, &mut backend).await;
    let mut calibrated = variant.entry_calibrated;
    let mut motor_enabled = variant.entry_motor_enabled;
    let mut gains = (variant.stored_gains)();
    let mut engine = Some((variant.engine)());

    let mut machine = TerminalRestoreMachine::start((variant.result)(), 0, 4095);
    let mut polls = 0usize;
    let outcome = loop {
        polls += 1;
        assert!(
            polls < 64,
            "{}: restore machine did not terminate",
            variant.name
        );
        assert!(
            machine.is_pending(),
            "{}: machine stopped early",
            variant.name
        );
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        match machine
            .poll_step(&mut engine, &mut backend, &mut state, None)
            .await
        {
            RestoreStep::Pending => {}
            RestoreStep::Done(outcome) => break outcome,
        }
    };
    assert!(
        !machine.is_pending(),
        "{}: machine still pending after Done",
        variant.name
    );
    let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
    apply_post_actions(&outcome, &mut backend, &mut state);
    trace_of(&outcome, &backend, &engine)
}

async fn assert_disarm_phase_one_op_per_poll(variant: &TerminalVariant) {
    let mut backend = MockBackend::new();
    prepare_backend(variant, &mut backend).await;
    let mut calibrated = variant.entry_calibrated;
    let mut motor_enabled = variant.entry_motor_enabled;
    let mut gains = (variant.stored_gains)();
    let mut engine = Some((variant.engine)());
    let mut machine = TerminalRestoreMachine::start((variant.result)(), 0, 4095);
    let mut previous_len = backend.spi_log.len();
    let mut previous_verified = backend.verified_write_calls;
    let mut previous_attributed = backend.raw_ops_attributed_to_verified_calls;
    let mut disarm_phase = false;
    let mut poll_index = 0usize;
    loop {
        poll_index += 1;
        assert!(
            poll_index < 128,
            "{}: machine did not terminate",
            variant.name
        );
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        let outcome = machine
            .poll_step(&mut engine, &mut backend, &mut state, None)
            .await;
        let delta = &backend.spi_log[previous_len..];
        if !disarm_phase && delta.iter().any(|op| matches!(op, SpiOp::EnablePin(false))) {
            disarm_phase = true;
        }
        if disarm_phase {
            let verified_delta = backend.verified_write_calls - previous_verified;
            let attributed_delta =
                backend.raw_ops_attributed_to_verified_calls - previous_attributed;
            let total_raw = delta
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                .count();
            let outside_delta = total_raw.saturating_sub(attributed_delta);
            let ok = match verified_delta {
                0 => outside_delta <= 1,
                1 => outside_delta == 0,
                _ => false,
            };
            assert!(
                ok,
                "{}: poll {} in the disarm phase: {} verified-write call(s), {} raw register \
                 op(s) unattributed to any verified call (of {} total) -- expected at most \
                 one logical register operation",
                variant.name, poll_index, verified_delta, outside_delta, total_raw
            );
        }
        previous_verified = backend.verified_write_calls;
        previous_attributed = backend.raw_ops_attributed_to_verified_calls;
        previous_len = backend.spi_log.len();
        if matches!(outcome, RestoreStep::Done(_)) {
            assert!(
                disarm_phase,
                "{}: run finished without ever entering a disarm phase",
                variant.name
            );
            break;
        }
    }
}

async fn assert_staged_disarm_phase_one_op_per_poll(
    name: &str,
    terminal: CommissionTerminal,
    engine_factory: fn() -> CommissioningEngine,
    prepare: fn(&mut MockBackend),
    entry_calibrated: bool,
    entry_motor_enabled: bool,
    stored_gains: fn() -> Option<crate::pid::InnerLoopGains>,
) {
    use crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;

    let mut backend = MockBackend::new();
    reset_channel0();
    prepare(&mut backend);
    let mut calibrated = entry_calibrated;
    let mut motor_enabled = entry_motor_enabled;
    let mut gains = stored_gains();
    let mut engine = Some(engine_factory());
    let mut slot = IntegralRetentionSlot::new();
    let mut machine = TerminalRestoreMachine::start_staged(terminal, 0, 4095);
    let mut previous_len = backend.spi_log.len();
    let mut previous_verified = backend.verified_write_calls;
    let mut previous_attributed = backend.raw_ops_attributed_to_verified_calls;
    let mut disarm_phase = false;
    let mut poll_index = 0usize;
    loop {
        poll_index += 1;
        assert!(poll_index < 128, "{name}: machine did not terminate");
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        let mut retention = CommissionRetention::new(&mut slot);
        let outcome = machine
            .poll_step(&mut engine, &mut backend, &mut state, Some(&mut retention))
            .await;
        let delta = &backend.spi_log[previous_len..];
        if !disarm_phase && delta.iter().any(|op| matches!(op, SpiOp::EnablePin(false))) {
            disarm_phase = true;
        }
        if disarm_phase {
            let verified_delta = backend.verified_write_calls - previous_verified;
            let attributed_delta =
                backend.raw_ops_attributed_to_verified_calls - previous_attributed;
            let total_raw = delta
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                .count();
            let outside_delta = total_raw.saturating_sub(attributed_delta);
            let ok = match verified_delta {
                0 => outside_delta <= 1,
                1 => outside_delta == 0,
                _ => false,
            };
            assert!(
                ok,
                "{name}: poll {poll_index} in the disarm phase: {verified_delta} verified-write \
                 call(s), {outside_delta} raw register op(s) unattributed to any verified call \
                 (of {total_raw} total) -- expected at most one logical register operation"
            );
        }
        previous_verified = backend.verified_write_calls;
        previous_attributed = backend.raw_ops_attributed_to_verified_calls;
        previous_len = backend.spi_log.len();
        if matches!(outcome, RestoreStep::Done(_)) {
            assert!(
                disarm_phase,
                "{name}: run finished without ever entering a disarm phase"
            );
            break;
        }
    }
}

async fn assert_single_op_arm_budget(variant: &TerminalVariant) {
    let mut backend = MockBackend::new();
    prepare_backend(variant, &mut backend).await;
    let mut calibrated = variant.entry_calibrated;
    let mut motor_enabled = variant.entry_motor_enabled;
    let mut gains = (variant.stored_gains)();
    let mut engine = Some((variant.engine)());
    let mut machine = TerminalRestoreMachine::start((variant.result)(), 0, 4095);
    let mut previous_len = backend.spi_log.len();
    let mut poll_index = 0usize;
    let mut saw_target_arm = false;
    loop {
        poll_index += 1;
        assert!(
            poll_index < 256,
            "{}: machine did not terminate",
            variant.name
        );
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        let outcome = machine
            .poll_step(&mut engine, &mut backend, &mut state, None)
            .await;
        if machine.entered_single_op_arm_this_poll() {
            saw_target_arm = true;
            let raw = backend.spi_log[previous_len..]
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                .count();
            assert!(
                raw <= 1,
                "{}: poll {} entered a Plan A target arm and issued {} plain \
                 register op(s), expected at most 1",
                variant.name,
                poll_index,
                raw
            );
        }
        previous_len = backend.spi_log.len();
        if matches!(outcome, RestoreStep::Done(_)) {
            break;
        }
    }
    if variant.reaches_single_op_arm {
        assert!(
            saw_target_arm,
            "{}: expected to reach a Plan A target arm but never did",
            variant.name
        );
    }
}

async fn assert_staged_single_op_arm_budget(
    name: &str,
    terminal: CommissionTerminal,
    engine_factory: fn() -> CommissioningEngine,
    prepare: fn(&mut MockBackend),
    entry_calibrated: bool,
    entry_motor_enabled: bool,
    stored_gains: fn() -> Option<crate::pid::InnerLoopGains>,
) {
    use crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;

    let mut backend = MockBackend::new();
    reset_channel0();
    prepare(&mut backend);
    let mut calibrated = entry_calibrated;
    let mut motor_enabled = entry_motor_enabled;
    let mut gains = stored_gains();
    let mut engine = Some(engine_factory());
    let mut slot = IntegralRetentionSlot::new();
    let mut machine = TerminalRestoreMachine::start_staged(terminal, 0, 4095);
    let mut previous_len = backend.spi_log.len();
    let mut poll_index = 0usize;
    let mut saw_target_arm = false;
    loop {
        poll_index += 1;
        assert!(poll_index < 256, "{name}: machine did not terminate");
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        let mut retention = CommissionRetention::new(&mut slot);
        let outcome = machine
            .poll_step(&mut engine, &mut backend, &mut state, Some(&mut retention))
            .await;
        if machine.entered_single_op_arm_this_poll() {
            saw_target_arm = true;
            let raw = backend.spi_log[previous_len..]
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                .count();
            assert!(
                raw <= 1,
                "{name}: poll {poll_index} entered a Plan A target arm and issued \
                 {raw} plain register op(s), expected at most 1"
            );
        }
        previous_len = backend.spi_log.len();
        if matches!(outcome, RestoreStep::Done(_)) {
            break;
        }
    }
    assert!(
        saw_target_arm,
        "{name}: expected to reach a Plan A target arm but never did"
    );
}

#[fixture_test]
async fn calibrate_done_reaches_apply_filters_one_op_per_poll() {
    for variant in all_terminal_variants()
        .iter()
        .filter(|v| v.name.starts_with("calibrate_done") && v.reaches_single_op_arm)
    {
        assert_single_op_arm_budget(variant).await;
    }
}

#[fixture_test]
async fn commission_done_reaches_arm_one_op_per_poll() {
    for variant in all_terminal_variants()
        .iter()
        .filter(|v| v.name.starts_with("commission_done") && v.reaches_single_op_arm)
    {
        assert_single_op_arm_budget(variant).await;
    }
}

#[fixture_test]
async fn robustness_restore_baseline_one_op_per_poll() {
    for variant in all_terminal_variants()
        .iter()
        .filter(|v| v.name.starts_with("robustness_reversal") && v.reaches_single_op_arm)
    {
        assert_single_op_arm_budget(variant).await;
    }
}

#[fixture_test]
async fn outer_failed_soft_failure_stability_arm_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "outer_failed_soft_failure")
        .expect("outer_failed_soft_failure variant must exist");
    assert_single_op_arm_budget(&variant).await;
}

#[fixture_test]
async fn outer_failed_soft_failure_arm_failure_stability_arm_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "outer_failed_soft_failure_arm_failure")
        .expect("outer_failed_soft_failure_arm_failure variant must exist");
    assert_single_op_arm_budget(&variant).await;
}

#[fixture_test]
async fn outer_failed_safety_fault_restore_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "outer_failed_safety_fault")
        .expect("outer_failed_safety_fault variant must exist");
    assert_single_op_arm_budget(&variant).await;
}

#[fixture_test]
async fn velocity_staged_disarm_velocity_phase_is_one_op_per_poll() {
    assert!(
        robustness_reversal_engine()
            .velocity_operational_torque_limit()
            .is_none(),
        "robustness_reversal_engine fixture must not carry an operational torque limit for this test"
    );
    assert_staged_disarm_phase_one_op_per_poll(
        "velocity_staged_disarm_velocity",
        CommissionTerminal::VelocityIntegral(sample_velocity_integral_terminal()),
        robustness_reversal_engine,
        no_prepare,
        true,
        true,
        no_gains,
    )
    .await;
}

#[fixture_test]
async fn velocity_staged_disarm_velocity_with_operational_limit_phase_is_one_op_per_poll() {
    assert!(
        breakaway_campaign_engine()
            .velocity_operational_torque_limit()
            .is_some(),
        "breakaway_campaign_engine fixture must carry an operational torque limit for this test"
    );
    assert_staged_disarm_phase_one_op_per_poll(
        "velocity_staged_disarm_velocity_with_operational_limit",
        CommissionTerminal::VelocityIntegral(sample_velocity_integral_terminal()),
        breakaway_campaign_engine,
        no_prepare,
        true,
        true,
        no_gains,
    )
    .await;
}

#[fixture_test]
async fn velocity_staged_restore_baseline_one_op_per_poll() {
    assert!(
        robustness_reversal_engine().entry_snapshot().is_some(),
        "robustness_reversal_engine fixture must carry an entry snapshot for this test \
         to reach VelocityStagedRestoreBaseline"
    );
    assert_staged_single_op_arm_budget(
        "velocity_staged_restore_baseline",
        CommissionTerminal::VelocityIntegral(sample_velocity_integral_terminal()),
        robustness_reversal_engine,
        no_prepare,
        true,
        true,
        no_gains,
    )
    .await;
}

#[fixture_test]
async fn machine_matches_async_trace_for_every_variant() {
    for variant in &all_terminal_variants() {
        let golden = run_variant_via_driver(variant).await;
        let via_machine = run_variant_via_machine(variant).await;
        assert_eq!(golden, via_machine, "variant {} diverged", variant.name);
    }
}

#[fixture_test]
async fn inner_failed_commission_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "inner_failed_commission")
        .expect("inner_failed_commission variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn calibrate_done_arm_failure_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "calibrate_done_arm_failure")
        .expect("calibrate_done_arm_failure variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn commission_done_arm_failure_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "commission_done_arm_failure")
        .expect("commission_done_arm_failure variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn outer_done_success_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "outer_done_success")
        .expect("outer_done_success variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn outer_failed_safety_fault_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "outer_failed_safety_fault")
        .expect("outer_failed_safety_fault variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn outer_failed_safety_fault_velocity_limit_disarm_phase_is_one_op_per_poll() {
    assert!(
        breakaway_campaign_engine()
            .velocity_operational_torque_limit()
            .is_some(),
        "breakaway_campaign_engine fixture must carry an operational torque limit for this test"
    );
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "outer_failed_safety_fault_velocity_limit")
        .expect("outer_failed_safety_fault_velocity_limit variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn outer_failed_soft_failure_arm_failure_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "outer_failed_soft_failure_arm_failure")
        .expect("outer_failed_soft_failure_arm_failure variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn robustness_reversal_complete_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "robustness_reversal_complete")
        .expect("robustness_reversal_complete variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn robustness_reversal_rejected_disarm_phase_is_one_op_per_poll() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "robustness_reversal_rejected")
        .expect("robustness_reversal_rejected variant must exist");
    assert_disarm_phase_one_op_per_poll(&variant).await;
}

async fn assert_gain_restore_phase_one_op_per_poll(variant: &TerminalVariant) {
    let mut backend = MockBackend::new();
    prepare_backend(variant, &mut backend).await;
    let mut calibrated = variant.entry_calibrated;
    let mut motor_enabled = variant.entry_motor_enabled;
    let mut gains = (variant.stored_gains)();
    let mut engine = Some((variant.engine)());
    let mut machine = TerminalRestoreMachine::start((variant.result)(), 0, 4095);
    let mut previous_len = backend.spi_log.len();
    let mut gain_phase = false;
    let mut gain_phase_ended = false;
    let pi_addr = foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
        .addr
        .addr();
    let mut poll_index = 0usize;
    loop {
        poll_index += 1;
        assert!(
            poll_index < 128,
            "{}: machine did not terminate",
            variant.name
        );
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        let outcome = machine
            .poll_step(&mut engine, &mut backend, &mut state, None)
            .await;
        let delta = &backend.spi_log[previous_len..];
        if !gain_phase
            && delta
                .iter()
                .any(|op| matches!(op, SpiOp::Write(addr, _) if *addr == pi_addr))
        {
            gain_phase = true;
        }
        if gain_phase && !gain_phase_ended {
            if delta.iter().any(|op| matches!(op, SpiOp::EnablePin(false))) {
                gain_phase_ended = true;
            } else {
                let raw_ops = delta
                    .iter()
                    .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                    .count();
                assert!(
                    raw_ops <= 1,
                    "{}: poll {} in the gain-restore phase logged {} raw ops, expected at most 1",
                    variant.name,
                    poll_index,
                    raw_ops
                );
            }
        }
        previous_len = backend.spi_log.len();
        if matches!(outcome, RestoreStep::Done(_)) {
            break;
        }
    }
    assert!(
        gain_phase,
        "{}: run finished without ever entering the gain-restore phase",
        variant.name
    );
}

#[fixture_test]
async fn selftest_done_gain_restore_polls_are_one_op_each() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "selftest_done_gain_restore_failure")
        .expect("selftest_done_gain_restore_failure variant must exist");
    assert_gain_restore_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn inner_failed_calibrate_gain_restore_polls_are_one_op_each() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "inner_failed_calibrate")
        .expect("inner_failed_calibrate variant must exist");
    assert_gain_restore_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn calibrate_done_gain_restore_failure_polls_are_one_op_each() {
    let variant = all_terminal_variants()
        .into_iter()
        .find(|v| v.name == "calibrate_done_gain_restore_failure")
        .expect("calibrate_done_gain_restore_failure variant must exist");
    assert_gain_restore_phase_one_op_per_poll(&variant).await;
}

#[fixture_test]
async fn outer_failed_stability_stale_gap_falls_back_to_spi_disarm() {
    let mut backend = MockBackend::new();
    seed_motor_sentinel(&mut backend).await;
    backend.set_elapsed_us_sequence(&[0, 0, 0]);
    backend.elapsed_us_value = u64::from(super::OUTER_RESTORE_STABILITY_INTERVAL_US)
        + u64::from(super::OUTER_RESTORE_STABILITY_MAX_SAMPLE_GAP_US)
        + 1;
    backend.read_position_actual_sequence.push(1000).unwrap();

    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(robustness_reversal_engine());
    let err = crate::autotune::CommissionError::NoCurrent;

    let mut machine = TerminalRestoreMachine::start(StepResult::OuterFailed(err), 0, 4095);
    let mut polls = 0usize;
    let outcome = loop {
        polls += 1;
        assert!(polls < 64, "machine did not terminate");
        match machine
            .poll_step(&mut engine, &mut backend, &mut state, None)
            .await
        {
            RestoreStep::Pending => {}
            RestoreStep::Done(outcome) => break outcome,
        }
    };

    assert_eq!(
        backend.read_position_actual_pos, 0,
        "a stale gap must not attempt a PID_POSITION_ACTUAL read -- `spi_log`'s \
         plain SpiOp::Read count is not usable here, since OuterFailedRestoreEntryGains' \
         own PI-representation verification issues unrelated reads earlier in the same run"
    );
    match outcome {
        CommissionOutcome::Terminal {
            reply: ReplyPayload::TuneResult { status, .. },
            ..
        } => {
            assert_eq!(status, crate::autotune::CommissionError::Spi.status_code());
        }
        _ => panic!("expected TuneResult"),
    }
}
