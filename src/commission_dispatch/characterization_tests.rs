//! Characterization oracle for `drive_commission_result`'s terminal-variant
//! behavior. This is executable documentation: it pins the *current*
//! per-terminal ordering of SPI writes, GPIO/enable-pin transitions, motor
//! lifecycle flag transitions, fault/cause stamping, reply shape, and
//! `engine.take()` timing, so a later restructuring of this dispatch can be
//! proven behavior-preserving against these goldens rather than against
//! prose.
//!
//! Terminal-variant table, extracted from `drive_commission_result`
//! (mod.rs:473-1259):
//!
//! | `StepResult` (sub-case)                          | mandatory cleanup on failure                                                             | fault/cause stamped                                    | reply                        | `engine.take()` | arms? |
//! |---------------------------------------------------|-------------------------------------------------------------------------------------------|----------------------------------------------------------|-------------------------------|-------------------|-------|
//! | `VelocityIntegralTerminal`                         | `disarm_velocity_workflow`: no operational limit -> `disarm_motor` (always `Ok`); with a limit -> `PID_VELOCITY_TARGET=0`, clear velocity error sum, *then* `disarm_motor`, *then* restore the operational current-limit clamp | on `disarm_velocity_workflow` failure: outcome forced to `Fault`, `cause=Error(Spi)`, `recovery_unavailable=true`, `safety_fault=None` | `VelocityIntegralTerminal`   | always           | never |
//! | `AcceptanceMatrixTerminal`                         | `disarm_motor` unconditionally, before any terminal post-processing                       | terminal passed through unmodified                       | `AcceptanceMatrixTerminal`   | always           | never |
//! | `RobustnessReversalTerminal` (outcome != `Complete`) | restore `entry_snapshot()` to registers if present; `disarm_motor` always runs afterward   | restore failure or missing snapshot -> `Failed`/`ROBUSTNESS_CAUSE_SAFETY_FAULT` | `RobustnessReversalTerminal` | always           | never |
//! | `RobustnessReversalTerminal` (outcome == `Complete`) | **no** baseline restore attempted; `disarm_motor` still runs unconditionally              | terminal passed through unmodified                       | `RobustnessReversalTerminal` | always           | never (deploy/arm happens only in `drive_commission_result_with_stages`, out of this function's scope) |
//! | `SelftestDone`                                     | `restore_calibrate_gains`; failure is logged only, no disarm/arm, no lifecycle events      | none (`done_status=0` always)                             | `Selftest`                   | always           | never |
//! | `InnerFailed` (any `EngineKind`)                    | `restore_safe_state_via_backend` directly -- **not** `lifecycle::disarm_motor`, so `state.calibrated`/`state.motor_enabled` are left untouched even though the enable pin is lowered; `Calibrate`/`Selftest` kinds also attempt `restore_calibrate_gains` (log-only on failure) | `error.status_code()` mapped per `EngineKind`             | `CalibrateResult`/`Selftest`/`CommissionPhase` | always | never |
//! | `CalibrateDone` (gain-restore SPI failure)          | `disarm_motor`                                                                             | `PiRepresentation` -> `ChipResetDetected`, else `SpiError` | `CalibrateResult`            | always           | never |
//! | `CalibrateDone` (filter-apply SPI failure)          | `disarm_motor`                                                                             | `SpiError`                                                | `CalibrateResult`            | always           | never |
//! | `CalibrateDone` (`arm_motor_electrical` failure)    | `disarm_motor`                                                                             | `ChipResetDetected` or `SpiError` per error variant        | `CalibrateResult`            | always           | never |
//! | `CalibrateDone` (success)                           | none                                                                                       | `status=0`                                                | `CalibrateResult`            | always           | **yes** (`ArmPreservePosition`) |
//! | `CommissionDone` (`inner.status == Failed`)         | **direct** atomic/flag clears (`motor_armed`, `calibrated`, `motor_enabled`) -- **not** `disarm_motor`, so no SPI restore and no `EnablePin`/`Lifecycle` events are recorded at all | `inner.status`'s error code                               | `CommissionResult`           | always           | never |
//! | `CommissionDone` (invalid fallback outer gains)     | `disarm_motor`                                                                             | `CurrentValidationFailed`                                  | `CommissionResult`           | always           | never |
//! | `CommissionDone` (current-limit restore SPI failure) | `disarm_motor`                                                                            | `Spi`                                                      | `CommissionResult`           | always           | never |
//! | `CommissionDone` (`arm_motor_electrical` failure)   | `disarm_motor`                                                                             | error's status code                                        | `CommissionResult`           | always           | never |
//! | `CommissionDone` (success)                          | none                                                                                       | `inner.status`'s code (0/1/2)                              | `CommissionResult`           | always           | **yes** (`ArmZeroPosition`) |
//! | `OuterDone` (position-mode restore failure)         | `disarm_motor`                                                                             | mapped from `RestoreError`                                  | `TuneResult`                 | always           | never |
//! | `OuterDone` (filter-apply SPI failure)              | `disarm_motor`                                                                             | `Spi`                                                       | `TuneResult`                 | always           | never |
//! | `OuterDone` (`arm_motor_electrical` failure)        | `disarm_motor`                                                                             | error's status code                                          | `TuneResult`                 | always           | never |
//! | `OuterDone` (success)                                | none                                                                                       | `outer.status`'s code                                        | `TuneResult`                 | always           | **yes** (`ArmPreservePosition`) |
//! | `OuterFailed` (safety-fault error variant)          | restore skipped outright; `MODE_RAMP_MODE_MOTION=0` best-effort, then `disarm_motor`        | the safety-fault error itself                                 | `TuneResult`                 | always (`take_after_handled_stop` or plain `take`) | never |
//! | `OuterFailed` (entry-gain restore fails, or restore-stability read fails) | `MODE_RAMP_MODE_MOTION=0`, `disarm_motor`                          | restore error, or `Spi` on a stability-read failure             | `TuneResult`                 | always           | never |
//! | `OuterFailed` (restore succeeds but position drifts) | `MODE_RAMP_MODE_MOTION=0`, `disarm_motor`                                                 | the **original upstream** error is preserved, not overwritten  | `TuneResult`                 | always           | never |
//! | `OuterFailed` (restore succeeds, stable, `arm_motor_electrical` fails) | `disarm_motor`                                                              | arm error's status code                                        | `TuneResult`                 | always           | never |
//! | `OuterFailed` (restore succeeds, stable, arm succeeds -- soft failure) | none                                                                        | the original upstream error (still nonzero: `OuterFailed` always reports *some* failure) | `TuneResult` | always | **yes** (`ArmPreservePosition`) |
//!
//! Non-terminal arms (`ReadyNow`, `At`, `PhaseChanged`, `VelocityIntegralEvent`,
//! `AcceptanceMatrixEvent`, `BreakawayCampaignEvent`, `BreakawayStageCPlanReady`)
//! never touch `engine.take()`, never call `arm_motor_electrical`/`disarm_motor`,
//! and are the only arms safe to poll repeatedly: every terminal arm calls
//! `engine.take().expect(..)`, so re-driving the same already-consumed engine
//! slot through a terminal arm panics rather than being idempotent. Dispatch
//! is a run-exactly-once-per-terminal-result contract, not a poll loop that
//! tolerates replay.
//!
//! `arm_motor_for_steps` is never called from inside `drive_commission_result`
//! itself. It runs from the board's separate `apply_stepper_post_action_state`
//! call, driven by the `PostActions::stepper` this function returns
//! (`ArmZeroPosition`/`ArmPreservePosition`). `run_variant` below simulates
//! that second call so the full arm-electrical-before-arm-for-steps ordering
//! is observable in one trace.

use super::test_support::{
    MockBackend, SpiOp, accepted_commission_result, breakaway_campaign_engine, fresh_state_locals,
    robustness_reversal_engine,
};
use super::*;
use crate::autotune::CommissionProfile;
use crate::commissioning::backend::CommissionMotorControl;
use crate::commissioning::types::{
    AcceptanceMatrixCause, AcceptanceMatrixOutcome, AcceptanceMatrixTerminal, RobustnessOutcome,
};
use crate::tmc_control::lifecycle::LifecycleFlag;
use foci_firmware_macros::fixture_test;

const TEST_MOTOR_SENTINEL: u32 = 0x0002_0032;

fn dispatch_state<'a>(
    calibrated: &'a mut bool,
    motor_enabled: &'a mut bool,
    stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
) -> DispatchState<'a> {
    crate::state::tmc::tmc_channel0().mark_pi_representation_established();
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
    }
}

fn zero_acceptance_matrix_terminal() -> AcceptanceMatrixTerminal {
    AcceptanceMatrixTerminal {
        schema_revision: 1,
        run_sequence: 1,
        evidence_sequence: 0,
        acceptance_plan_digest: 0,
        plan_digest: 0,
        digest: 0,
        outcome: AcceptanceMatrixOutcome::Complete,
        cause: AcceptanceMatrixCause::None,
        attempted_masks: [0, 0],
        eligible_masks: [0, 0],
        current_terminus_masks: [0, 0],
        unattempted_masks: [0, 0],
        expected_observations: 0,
        emitted_observations: 0,
        expected_amplitudes: 0,
        emitted_amplitudes: 0,
        qualifier_bits: 0,
        run_started_us: 0,
        run_completed_us: 0,
        safety_fault: None,
    }
}

/// What kind of `CommissionOutcome` a run produced, independent of the full
/// `ReplyPayload` so an equivalence check can `assert_eq!` two traces
/// without needing `PartialEq`/`Debug` on `CommissionOutcome` itself (its
/// nested evidence types make that impractical).
///
/// `outcome_class` carries the reply's own semantic outcome discriminant
/// (`RobustnessOutcome`, `AcceptanceMatrixOutcome`, `VelocityIntegralOutcome`)
/// as its `u8` value, for the three reply families that separate "what
/// class of result" from "what caused it". It is `None` for the flat-status
/// replies (`CalibrateResult`, `CommissionResult`, `TuneResult`, `Selftest`,
/// `CommissionPhase`), where the status code alone is the whole story.
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

/// One characterization run's interleaved ordered event log, outcome shape,
/// terminal cause, and whether the dispatch consumed the engine slot.
///
/// This is the equivalence helper a future restructuring of
/// `drive_commission_result` is checked against: run the same `StepResult`
/// through both implementations and `assert_eq!` the two `OpTrace` values.
#[derive(Debug, Clone, PartialEq, Eq)]
struct OpTrace {
    events: heapless::Vec<SpiOp, 8_192>,
    outcome_tag: OutcomeTag,
    terminal_cause: Option<u16>,
    engine_consumed: bool,
}

/// Pack a `StageCCause`'s namespace and raw value into one `u16` so the two
/// namespaces (`Engine`/`Error`/`Dispatch`) never alias each other.
fn stage_c_cause_code(cause: crate::commissioning::types::StageCCause) -> u16 {
    (u16::from(cause.namespace_code()) << 8) | u16::from(cause.raw_value())
}

/// Extract `(outcome_class, terminal_cause)` from one terminal `ReplyPayload`.
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
            Some(stage_c_cause_code(terminal.cause)),
        ),
        ReplyPayload::AcceptanceMatrixTerminal { terminal, .. } => {
            (Some(terminal.outcome as u8), Some(terminal.cause as u16))
        }
        ReplyPayload::RobustnessReversalTerminal { terminal, .. } => (
            Some(terminal.outcome as u8),
            Some(u16::from(terminal.cause)),
        ),
        _ => (None, None),
    }
}

/// Drive one `StepResult` through `drive_commission_result`, then -- for a
/// terminal outcome -- simulate the board's follow-up
/// `apply_stepper_post_action_state` call so `arm_motor_for_steps` (which
/// never runs inside `drive_commission_result` itself) appears in the same
/// ordered trace. Returns both the raw `CommissionOutcome` (for goldens that
/// need to inspect reply fields `OpTrace` does not carry, e.g.
/// `recovery_unavailable`) and the derived `OpTrace`.
///
/// Takes `engine`/`backend`/`state` rather than constructing them from
/// `result` alone: `StepResult::InnerFailed` carries no `EngineKind`, so a
/// single-`StepResult`-argument helper cannot pick between a Calibrate,
/// Selftest, or Commission engine for it, and several goldens below need
/// backend failure injection (`fail_writes_remaining`, `fail_on_write`) that
/// no `StepResult` value encodes either. `run_variant` below is the
/// single-purpose oracle entry point that returns bare `OpTrace`; call this
/// directly only when a test needs the full `CommissionOutcome` too.
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
        if matches!(
            actions.stepper,
            StepperPostAction::ArmZeroPosition | StepperPostAction::ArmPreservePosition
        ) {
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

/// The equivalence-check entry point: run one `StepResult` and return only
/// the `OpTrace` a future implementation is diffed against via `assert_eq!`.
/// See [`drive_and_trace`] for the parameter-shape rationale and for goldens
/// that need the full `CommissionOutcome` as well.
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

// --- calibrate ---------------------------------------------------------

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
    // Forced-fail-mid-restore: a transient write failure inside
    // `restore_calibrate_gains` must divert straight to `disarm_motor`
    // without ever reaching `arm_motor_electrical`.
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

// --- ADC / inner-failed (calibrate) -------------------------------------

#[fixture_test]
async fn inner_failed_calibrate_uses_safe_state_restore_not_disarm_motor() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    // Pre-seed as if a prior arm had happened, to prove this path leaves
    // these flags untouched (it restores GPIO/registers directly, bypassing
    // `lifecycle::disarm_motor`).
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
    // GPIO restored low via restore_safe_state_via_backend...
    assert!(!backend.enable_pin);
    // ...but the dispatch-local flags are untouched: this path never calls
    // `lifecycle::disarm_motor`, so no Lifecycle events are recorded and the
    // caller-owned flags keep whatever value they held on entry.
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

// --- selftest --------------------------------------------------------------

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
    // restore_calibrate_gains writes the stored current-loop gains -- the
    // "restore, log-only on failure, no disarm/arm" row of the table.
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

// --- commission (Stage 1) ------------------------------------------------

#[fixture_test]
async fn commission_done_success_arms_electrical_before_zero_position_arm() {
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
            stepper: StepperPostAction::ArmZeroPosition,
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
    // The Failed-status branch clears flags directly, never through
    // `lifecycle::disarm_motor` -- so the interleaved log carries no SPI,
    // no EnablePin, and no Lifecycle events at all.
    assert!(trace.events.is_empty());
    assert!(!*state.calibrated);
    assert!(!*state.motor_enabled);
}

#[fixture_test]
async fn commission_done_arm_failure_disarms_without_reaching_arm_for_steps() {
    let mut backend = MockBackend::new();
    // No sentinel seeded, so arm_motor_electrical's read_register_verified
    // mismatches and fails.
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

// --- outer (Stage 2) ------------------------------------------------------

#[fixture_test]
async fn outer_done_success_arms_electrical_before_preserve_position_arm() {
    let mut backend = MockBackend::new();
    let (mut cal, mut en, mut gains) = fresh_state_locals();
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(CommissioningEngine::new_commission(
        0,
        CommissionProfile::Balanced,
    ));
    seed_motor_sentinel(&mut backend).await;

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
    // A safety fault skips the restore-stability check entirely -- no
    // stability-window register reads before the disarm.
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
    // The soft-failure path still reports the original upstream error even
    // though the motor stays armed.
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

// --- robustness ------------------------------------------------------------

#[fixture_test]
async fn robustness_reversal_terminal_never_arms_even_on_complete_outcome() {
    // Driven directly through `drive_commission_result` (not the
    // `_with_stages` wrapper): a Complete outcome never deploys/arms here --
    // that only happens in `drive_commission_result_with_stages`, which
    // reroutes a passed probe through the OuterDone arm.
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

// --- stage-c / acceptance matrix -------------------------------------------

#[fixture_test]
async fn acceptance_matrix_terminal_disarms_unconditionally() {
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
        StepResult::AcceptanceMatrixTerminal(zero_acceptance_matrix_terminal()),
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
            outcome_class: Some(AcceptanceMatrixOutcome::Complete as u8),
        }
    );
    assert!(contains(
        &trace.events,
        SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: false,
        },
    ));
    assert!(!backend.enable_pin);
}

// --- velocity-integral terminal ---------------------------------------------

#[fixture_test]
async fn velocity_integral_terminal_without_operational_limit_disarms_directly() {
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    // `new_commission` reports no active velocity workflow, so
    // `disarm_velocity_workflow` takes the plain `disarm_motor` branch.
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
            // The disarm path must not have rewritten a healthy terminal's
            // outcome away from the `Inconclusive` it was built with.
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
    // A restore failure on the very first velocity-workflow disarm write
    // aborts `disarm_and_restore_velocity_limit` before it ever reaches
    // `lifecycle::disarm_motor` -- so the motor stays electrically armed
    // even though the terminal is downgraded to Fault. This is a
    // preserved-as-is characterization finding, not something to silently
    // fix here.
    let mut backend = MockBackend::new();
    backend.fail_on_write = Some(foci_tmc4671::registers::PID_VELOCITY_TARGET.addr());
    let (_, _, mut gains) = fresh_state_locals();
    let mut cal = true;
    let mut en = true;
    let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
    let mut engine = Some(breakaway_campaign_engine());

    // `drive_and_trace`, not `run_variant`: this golden needs
    // `recovery_unavailable`/`safety_fault`, reply fields `OpTrace` does not
    // carry.
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
        cause: crate::commissioning::types::StageCCause::Engine(0),
        breakaway_acceptance: false,
        final_p: 0,
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
