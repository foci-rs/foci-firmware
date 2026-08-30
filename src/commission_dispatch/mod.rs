//! Board-agnostic post-processing for `CommissioningEngine::step()` results.
//!
//! Lifted from `openffboard-fw::tmc_control` per the design at
//! `docs/specs/2026-04-23-commission-dispatch-extraction-design.md`.
//!
//! Public surface: a single `drive_commission_result` function plus the
//! types it returns. The board calls this once per engine.step() result
//! and matches `CommissionOutcome` to fire RTIC primitives + emit replies.

// pub(crate) so sibling test modules (commissioning, diagnostics, ...) can
// import MockBackend + helpers via crate::commission_dispatch::test_support::*.
pub mod acceptance;
#[cfg(test)]
mod acceptance_tests;
#[cfg(test)]
mod characterization_tests;
#[cfg(test)]
mod manifest_completeness_tests;
pub mod post_actions;
pub mod replies;
mod restore;
pub mod robustness;
pub mod stage_c;
#[cfg(test)]
mod stage_c_tests;
#[cfg(test)]
pub(crate) mod test_support;
#[cfg(test)]
mod tests;

pub use post_actions::{StepperTaskDirective, apply_stepper_post_action_state};
pub use replies::{CommissionReplySink, emit_phase_reply, emit_terminal_reply};

use crate::autotune::PhaseId;
// CommissioningEngine lives in commissioning/mod.rs (the engine impl); the
// data types live in commissioning/types.rs.
use crate::commissioning::{
    CommissioningEngine,
    types::{
        AdcCalResult, CommissionResult, CommissioningBackend, EngineKind, OuterResult,
        OuterSafetyFault, StepResult,
    },
};
use crate::pid::{InnerLoopGains, OuterLoopGains};
use crate::selftest::SelftestResults;
use crate::state::stepper::StepperChannelState;
use crate::state::tmc::TmcChannelState;

/// Borrowed view of the per-engine task locals the dispatch needs to mutate.
///
/// The board's `tmc_control` task owns these as task locals; per-dispatch
/// it constructs a `DispatchState` so the firmware can mutate them in
/// place without copy-back.
pub struct DispatchState<'a> {
    /// Channel-local TMC runtime atomics for the active motor.
    pub tmc: &'static TmcChannelState,
    /// Channel-local stepper runtime atomics for the active motor.
    pub stepper: &'static StepperChannelState,
    pub calibrated: &'a mut bool,
    pub motor_enabled: &'a mut bool,
    pub stored_gains: &'a mut Option<InnerLoopGains>,
}

/// Mutable board-wide evidence retention passed through one dispatch call.
pub struct CommissionRetention<'a> {
    pub stage_c:
        &'a mut crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot,
}

impl<'a> CommissionRetention<'a> {
    /// Borrow the Stage-C RAM slot as one lifecycle context.
    pub fn new(
        stage_c: &'a mut crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot,
    ) -> Self {
        Self { stage_c }
    }
}

/// Stage-retention and immediate evidence emission for one terminal dispatch.
pub struct CommissionTerminalContext<'a, E> {
    /// Mutable Stage-C retained evidence for this dispatch.
    pub retention: CommissionRetention<'a>,
    /// Board-owned emitter used before returning the compact scheduler action.
    pub emit_evidence: E,
}

impl<'a, E> CommissionTerminalContext<'a, E> {
    /// Borrow the retained Stage-C slot and bind the board's reply emitter.
    pub fn new(
        stage_c: &'a mut crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot,
        emit_evidence: E,
    ) -> Self {
        Self {
            retention: CommissionRetention::new(stage_c),
            emit_evidence,
        }
    }
}

impl<'a> DispatchState<'a> {
    /// Build a dispatch-state view for one board-local motor channel.
    pub fn new(
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<InnerLoopGains>,
    ) -> Self {
        Self {
            tmc,
            stepper,
            calibrated,
            motor_enabled,
            stored_gains,
        }
    }
}

/// What the board should do after the dispatch returns.
#[allow(clippy::large_enum_variant)]
pub enum CommissionOutcome {
    /// Engine wants more time. Board: Mono::delay(1ms), continue.
    Continue,
    /// A phase transition occurred. Board: emit FociCommissionPhase reply,
    /// Mono::delay(1ms), continue.
    PhaseChanged {
        oid: u8,
        phase_id: PhaseId,
        engine_kind: EngineKind,
    },
    /// Nonterminal report-only evidence is ready for immediate emission.
    Evidence {
        /// Shared reply payload; boards provide only transport forwarding.
        reply: ReplyPayload,
    },
    /// Engine reached a terminal state. Firmware has already executed all
    /// SPI restore, atomic stores, filter re-apply, and arm/disarm motor.
    /// Board: execute PostActions in order, emit reply.
    Terminal {
        reply: ReplyPayload,
        actions: PostActions,
    },
}

/// Stepper-runtime transition the board performs after a terminal outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepperPostAction {
    /// Leave the stepper runtime unchanged.
    None,
    /// Zero the firmware step epoch, then arm step-timer processing.
    ArmZeroPosition,
    /// Preserve the firmware step epoch, then arm step-timer processing.
    ArmPreservePosition,
    /// Stop and drain step-timer processing.
    Stop,
}

/// RTIC-bound primitives the board fires after a Terminal outcome.
/// Order: stop_trace -> stepper action -> emit reply.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct PostActions {
    /// Stop the trace buffer. True for InnerFailed, OuterDone, OuterFailed,
    /// CommissionDone. False for CalibrateDone, SelftestDone.
    pub stop_trace: bool,
    /// Board-owned stepper runtime transition.
    pub stepper: StepperPostAction,
}

impl PostActions {
    pub const NONE: Self = Self {
        stop_trace: false,
        stepper: StepperPostAction::None,
    };
}

const OUTER_RESTORE_STABILITY_SAMPLES: usize = 5;
const OUTER_RESTORE_STABILITY_INTERVAL_US: u32 = 5_000;
const OUTER_RESTORE_STABILITY_ENCODER_COUNTS: u32 = 2;

#[derive(Debug, Clone, Copy, defmt::Format)]
struct RestoreStability {
    stable: bool,
    span_counts: u32,
}

async fn outer_restore_position_stable<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<RestoreStability, B::Error> {
    let mut first = 0i32;
    let mut min_delta = 0i32;
    let mut max_delta = 0i32;

    for index in 0..OUTER_RESTORE_STABILITY_SAMPLES {
        backend.delay_us(OUTER_RESTORE_STABILITY_INTERVAL_US).await;
        let position = backend
            .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
            .await? as i32;

        if index == 0 {
            first = position;
        }
        let delta = position.wrapping_sub(first);
        min_delta = min_delta.min(delta);
        max_delta = max_delta.max(delta);
    }

    let span = (max_delta as i64 - min_delta as i64).unsigned_abs();
    let encoder_counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
    let threshold = if encoder_counts_per_rev == 0 {
        1
    } else {
        ((u64::from(OUTER_RESTORE_STABILITY_ENCODER_COUNTS) * 65_536
            + u64::from(encoder_counts_per_rev) / 2)
            / u64::from(encoder_counts_per_rev))
        .max(1)
    };
    let span_counts = span.min(u32::MAX as u64) as u32;
    if span > threshold {
        defmt::error!(
            "OuterFailed restore: unstable span={} threshold={}",
            span,
            threshold,
        );
        Ok(RestoreStability {
            stable: false,
            span_counts,
        })
    } else {
        Ok(RestoreStability {
            stable: true,
            span_counts,
        })
    }
}

fn record_restore_summary<B: CommissioningBackend>(
    backend: &mut B,
    trace_summary_seqs: &mut crate::commissioning::outer::TraceSummarySeqs,
    restore_status: u8,
    upstream_status: u8,
    drift_counts: u32,
    disposition: i32,
    level: crate::trace::events::TraceEventLevel,
) {
    let summary_seq = trace_summary_seqs.next_restore();
    crate::commissioning::outer::trace_summary_event(
        backend,
        level,
        crate::trace::events::EVENT_COMMISSION_RESTORE_RESULT,
        summary_seq,
        i32::from(restore_status),
    );
    crate::commissioning::outer::trace_summary_event(
        backend,
        level,
        crate::trace::events::EVENT_COMMISSION_RESTORE_ERROR_DRIFT,
        summary_seq,
        crate::commissioning::outer::trace_pack_i16_pair(
            i32::from(upstream_status),
            drift_counts.min(i16::MAX as u32) as i32,
        ),
    );
    crate::commissioning::outer::trace_summary_event(
        backend,
        level,
        crate::trace::events::EVENT_COMMISSION_RESTORE_DISPOSITION,
        summary_seq,
        disposition,
    );
}

/// Wire-reply payload for the board to emit via `klipper_reply_from!`.
#[allow(clippy::large_enum_variant)]
pub enum ReplyPayload {
    /// Command-level motion shape and conservative host-timeout authority.
    CommissioningWorkflowPlan {
        oid: u8,
        plan: crate::commissioning::outer::velocity::integral_plan::CommissioningWorkflowPlan,
    },
    /// FociCommissionPhase: phase change reply (foci_commission failures
    /// during inner phases).
    CommissionPhase {
        oid: u8,
        phase: u8,
        status: u8,
        diagnostics: crate::selftest::CommissionDiagnostics,
    },
    /// FociCommissionResult: Stage 1 commission complete.
    CommissionResult {
        oid: u8,
        status: u8,
        result: CommissionResult,
    },
    /// FociTuneResult: Stage 2 tune complete (success or any failure mode).
    /// Zero-filled OuterResult on failure paths.
    TuneResult {
        oid: u8,
        status: u8,
        result: OuterResult,
        safety_fault: Option<OuterSafetyFault>,
    },
    /// FociCalibrateResult: ADC calibration complete.
    CalibrateResult {
        oid: u8,
        status: u8,
        result: AdcCalResult,
        diagnostics: crate::selftest::CommissionDiagnostics,
    },
    /// FociSelftestResult per stage + FociSelftestDone terminal.
    /// Board emits one Result message per populated stage, then Done.
    Selftest {
        oid: u8,
        results: SelftestResults,
        diagnostics: crate::selftest::CommissionDiagnostics,
        done_status: u8,
    },
    /// Complete exact Stage-C plan emitted before direct-resume motion.
    VelocityIntegralPlan {
        oid: u8,
        run_sequence: u32,
        plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
    },
    /// One streamed Stage-C observation or rung event.
    VelocityIntegralEvent {
        oid: u8,
        event: crate::commissioning::types::VelocityIntegralEvent,
    },
    /// Dedicated detailed Stage-C terminal and optional comparison.
    VelocityIntegralTerminal {
        oid: u8,
        terminal: crate::commissioning::types::VelocityIntegralTerminal,
        reproduction: Option<stage_c::StageCReproductionReport>,
        authority: Option<
            crate::commissioning::outer::velocity::integral_reproduction::StageCAuthoritySummary,
        >,
    },
    /// Stage-C terminal for a request refused before any exact plan existed.
    ///
    /// Distinct from [`ReplyPayload::VelocityIntegralTerminal`] because that one
    /// derives leading fragments from run evidence. A refusal has no run, so
    /// those fragments would describe motion that never happened, and the host
    /// reads anything ahead of the terminal as evidence preceding its plan.
    VelocityIntegralRefusal {
        oid: u8,
        terminal: crate::commissioning::types::VelocityIntegralTerminal,
    },
    /// Complete immutable fixed-I confidence-matrix plan disclosed before motion.
    AcceptanceMatrixPlan {
        oid: u8,
        run_sequence: u32,
        plan: crate::commissioning::outer::velocity::acceptance_plan::AcceptanceMatrixPlan,
    },
    /// Compact firmware-authored fixed-I confidence-matrix terminal.
    AcceptanceMatrixTerminal {
        oid: u8,
        terminal: crate::commissioning::types::AcceptanceMatrixTerminal,
    },
    /// Compact firmware-authored reversal-standstill robustness-gate terminal.
    RobustnessReversalTerminal {
        oid: u8,
        terminal: crate::commissioning::types::RobustnessReversalTerminal,
    },
    /// One streamed breakaway-campaign plan, terminal, or evidence bundle.
    BreakawayCampaignEvent {
        oid: u8,
        event: crate::commissioning::types::BreakawayCampaignEvent,
    },
}

// Reply values cross nested async commissioning dispatch. Bound their inline
// size so a future evidence field cannot recreate the F407 startup reset.
const _: () = assert!(core::mem::size_of::<ReplyPayload>() <= 1_400);

/// Helper: zero-filled OuterResult for failure-path TuneResult emission.
fn zero_outer_result() -> OuterResult {
    OuterResult {
        status: crate::autotune::CommissionStatus::Failed(crate::autotune::CommissionError::Spi),
        velocity_p: 0,
        velocity_i: 0,
        velocity_limit: 0,
        position_p: 0,
        position_i: 0,
        velocity_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
        torque_filter_hz: 0,
        warning_code: 0,
        provenance: crate::commissioning::types::TuneProvenance::default(),
    }
}

fn validate_outer_gains(
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) -> Result<OuterLoopGains, foci_tmc4671::pid::PidGainOutOfRange> {
    use foci_tmc4671::pid::{PidIGain, PidPGain};

    Ok(OuterLoopGains {
        position_p: PidPGain::new(position_p)?,
        position_i: PidIGain::new(position_i)?,
        velocity_p: PidPGain::new(velocity_p)?,
        velocity_i: PidIGain::new(velocity_i)?,
    })
}

/// Helper: zero-filled AdcCalResult for failure-path CalibrateResult emission.
fn zero_adc_cal_result() -> AdcCalResult {
    AdcCalResult {
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
    }
}

fn calibration_status_from_commission_error(error: crate::autotune::CommissionError) -> u8 {
    use crate::autotune::CommissionError;
    use crate::calibration::CalibrationError;

    let calibration_error = match error {
        CommissionError::Spi => CalibrationError::SpiError,
        CommissionError::ChipResetDetected => CalibrationError::ChipResetDetected,
        CommissionError::MotorEnabled => CalibrationError::AlreadyEnabled,
        CommissionError::EncoderFault => CalibrationError::EncoderFault,
        CommissionError::ClosedLoopEntryUnstable => CalibrationError::ClosedLoopEntryUnstable,
        _ => CalibrationError::InternalError,
    };
    calibration_error.status_code()
}

async fn disarm_velocity_workflow<B: CommissioningBackend>(
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> bool {
    let Some(operational_limit) = engine
        .as_ref()
        .and_then(CommissioningEngine::velocity_operational_torque_limit)
    else {
        crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
        return true;
    };
    let restored = restore::disarm_and_restore_velocity_limit(backend, state, operational_limit)
        .await
        .is_ok();
    engine
        .as_mut()
        .expect("velocity workflow disappeared during restoration")
        .mark_velocity_limit_restored();
    restored
}

/// Dispatch a single `StepResult` from `CommissioningEngine::step()` to the
/// appropriate outcome, executing any required SPI work, state mutations, and
/// atomic stores along the way.
///
/// # Parameters
/// - `result`: The `StepResult` returned by the most recent engine step.
/// - `engine`: Mutable reference to the `Option<CommissioningEngine>` task-local
///   slot. Terminal variants consume the engine (`engine.take()`); non-terminal
///   variants leave it in place.
/// - `backend`: SPI + GPIO backend for register access.
/// - `state`: Borrowed task-local mutable state (calibrated flag, motor-enabled
///   flag, stored PID gains).
/// - `pwm_maxcnt`: PWM period counter used when re-applying filters or limits
///   after a terminal outcome.
pub async fn drive_commission_result<B: CommissioningBackend>(
    result: StepResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    pwm_maxcnt: u16,
) -> CommissionOutcome {
    let _ = pwm_maxcnt; // suppress unused warning until all arms land
    match result {
        StepResult::ReadyNow | StepResult::At(_) => CommissionOutcome::Continue,
        StepResult::PhaseChanged(phase_id) => {
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on PhaseChanged");
            CommissionOutcome::PhaseChanged {
                oid: engine_ref.oid(),
                phase_id,
                engine_kind: engine_ref.kind(),
            }
        }
        StepResult::VelocityIntegralEvent(event) => {
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on VelocityIntegralEvent");
            CommissionOutcome::Evidence {
                reply: ReplyPayload::VelocityIntegralEvent {
                    oid: engine_ref.oid(),
                    event,
                },
            }
        }
        StepResult::VelocityIntegralTerminal(terminal) => {
            dispatch_velocity_integral_terminal(terminal, engine, backend, state).await
        }
        StepResult::AcceptanceMatrixEvent(_) => CommissionOutcome::Continue,
        StepResult::BreakawayCampaignEvent(event) => {
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on BreakawayCampaignEvent");
            CommissionOutcome::Evidence {
                reply: ReplyPayload::BreakawayCampaignEvent {
                    oid: engine_ref.oid(),
                    event,
                },
            }
        }
        // A breakaway campaign normally reaches Stage C on the scheduled-tick
        // path, where `adapt_step_result` carries the plan out as a
        // `CommissionEvent`. This arm covers the request-driven path so the
        // plan is reported either way. Trace records are not emitted here: the
        // engine's own trace hook already emits them for this result, and doing
        // it twice would duplicate the plan on the wire.
        StepResult::BreakawayStageCPlanReady { run_sequence, plan } => {
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on BreakawayStageCPlanReady");
            CommissionOutcome::Evidence {
                reply: ReplyPayload::VelocityIntegralPlan {
                    oid: engine_ref.oid(),
                    run_sequence,
                    plan,
                },
            }
        }
        StepResult::AcceptanceMatrixTerminal(terminal) => {
            dispatch_acceptance_matrix_terminal(terminal, engine, backend, state).await
        }
        StepResult::RobustnessReversalTerminal(terminal) => {
            dispatch_robustness_reversal_terminal(terminal, engine, backend, state).await
        }
        StepResult::SelftestDone => {
            let mut consumed = engine.take().expect("engine slot None on SelftestDone");
            let oid = consumed.oid();
            let results = consumed.take_selftest_results();
            let diagnostics = consumed.take_diagnostics();
            state
                .tmc
                .commission_in_flight
                .store(false, core::sync::atomic::Ordering::Relaxed);
            if restore::restore_calibrate_gains(backend, state.tmc, *state.stored_gains)
                .await
                .is_err()
            {
                defmt::error!("SelftestDone: saved gain restore SPI failed");
            }
            CommissionOutcome::Terminal {
                reply: ReplyPayload::Selftest {
                    oid,
                    results,
                    diagnostics,
                    done_status: 0,
                },
                actions: PostActions::NONE,
            }
        }
        StepResult::InnerFailed(error) => {
            let mut consumed = engine.take().expect("engine slot None on InnerFailed");
            let oid = consumed.oid();
            let kind = consumed.kind();
            let selftest_results = consumed.take_selftest_results();
            let diagnostics = consumed.take_diagnostics();
            state
                .tmc
                .commission_in_flight
                .store(false, core::sync::atomic::Ordering::Relaxed);
            crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
            if matches!(kind, EngineKind::Selftest | EngineKind::Calibrate)
                && restore::restore_calibrate_gains(backend, state.tmc, *state.stored_gains)
                    .await
                    .is_err()
            {
                defmt::error!("InnerFailed: saved gain restore SPI failed");
            }

            let reply = match kind {
                EngineKind::Calibrate => ReplyPayload::CalibrateResult {
                    oid,
                    status: calibration_status_from_commission_error(error),
                    result: zero_adc_cal_result(),
                    diagnostics,
                },
                EngineKind::Selftest => ReplyPayload::Selftest {
                    oid,
                    results: selftest_results,
                    diagnostics,
                    done_status: error.status_code(),
                },
                EngineKind::Commission | EngineKind::OuterTune => ReplyPayload::CommissionPhase {
                    oid,
                    phase: 0,
                    status: error.status_code(),
                    diagnostics,
                },
            };
            CommissionOutcome::Terminal {
                reply,
                actions: PostActions {
                    stop_trace: true,
                    stepper: StepperPostAction::None,
                },
            }
        }
        StepResult::CalibrateDone(cal_result) => {
            let consumed = engine.take().expect("engine slot None on CalibrateDone");
            let oid = consumed.oid();
            state
                .tmc
                .motor_armed
                .store(false, core::sync::atomic::Ordering::Relaxed);
            let restore_result =
                restore::restore_calibrate_gains(backend, state.tmc, *state.stored_gains).await;
            if let Err(restore_error) = restore_result {
                defmt::error!("CalibrateDone: saved gain restore SPI failed");
                crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                state
                    .tmc
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CalibrateResult {
                        oid,
                        status: match restore_error {
                            restore::RestoreError::PiRepresentation => {
                                crate::calibration::CalibrationError::ChipResetDetected
                                    .status_code()
                            }
                            restore::RestoreError::Spi(_)
                            | restore::RestoreError::InvalidGains
                            | restore::RestoreError::VelocityClamp
                            | restore::RestoreError::Readback => {
                                crate::calibration::CalibrationError::SpiError.status_code()
                            }
                        },
                        result: zero_adc_cal_result(),
                        diagnostics: crate::selftest::CommissionDiagnostics::default(),
                    },
                    actions: PostActions {
                        stop_trace: false,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            if crate::tmc_control::filters::apply_stored_filters_via_backend(
                backend, state.tmc, pwm_maxcnt,
            )
            .await
            .is_err()
            {
                defmt::error!("CalibrateDone: stored filter restore SPI failed");
                crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                state
                    .tmc
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CalibrateResult {
                        oid,
                        status: crate::calibration::CalibrationError::SpiError.status_code(),
                        result: zero_adc_cal_result(),
                        diagnostics: crate::selftest::CommissionDiagnostics::default(),
                    },
                    actions: PostActions {
                        stop_trace: false,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            *state.calibrated = true;
            if let Err(error) =
                crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
            {
                defmt::error!("CalibrateDone: arm sentinel verify failed: {}", error);
                crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                state
                    .tmc
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                let status = match error {
                    crate::autotune::CommissionError::ChipResetDetected => {
                        crate::calibration::CalibrationError::ChipResetDetected.status_code()
                    }
                    _ => crate::calibration::CalibrationError::SpiError.status_code(),
                };
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CalibrateResult {
                        oid,
                        status,
                        result: zero_adc_cal_result(),
                        diagnostics: crate::selftest::CommissionDiagnostics::default(),
                    },
                    actions: PostActions {
                        stop_trace: false,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            state
                .tmc
                .commission_in_flight
                .store(false, core::sync::atomic::Ordering::Relaxed);

            CommissionOutcome::Terminal {
                reply: ReplyPayload::CalibrateResult {
                    oid,
                    status: 0,
                    result: cal_result,
                    diagnostics: crate::selftest::CommissionDiagnostics::default(),
                },
                actions: PostActions {
                    stop_trace: false,
                    stepper: StepperPostAction::ArmPreservePosition,
                },
            }
        }
        StepResult::CommissionDone(commission_result) => {
            use core::sync::atomic::Ordering::Relaxed;
            let consumed = engine.take().expect("engine slot None on CommissionDone");
            let oid = consumed.oid();
            let inner = &commission_result.inner;
            let status = match inner.status {
                crate::autotune::CommissionStatus::Accepted => 0u8,
                crate::autotune::CommissionStatus::AcceptedWithWarnings => 1,
                crate::autotune::CommissionStatus::InnerOnly => 2,
                crate::autotune::CommissionStatus::Failed(e) => e.status_code(),
            };

            if matches!(inner.status, crate::autotune::CommissionStatus::Failed(_)) {
                state.tmc.motor_armed.store(false, Relaxed);
                *state.calibrated = false;
                *state.motor_enabled = false;
                state.tmc.commission_in_flight.store(false, Relaxed);
                defmt::error!("foci_commission failed: oid={} status={}", oid, status);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CommissionResult {
                        oid,
                        status,
                        result: commission_result,
                    },
                    actions: PostActions {
                        stop_trace: true,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            let fallback_outer_gains = match validate_outer_gains(
                commission_result.fallback_position_p,
                commission_result.fallback_position_i,
                commission_result.fallback_velocity_p,
                commission_result.fallback_velocity_i,
            ) {
                Ok(gains) => gains,
                Err(_) => {
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    return CommissionOutcome::Terminal {
                        reply: ReplyPayload::CommissionResult {
                            oid,
                            status: crate::autotune::CommissionError::CurrentValidationFailed
                                .status_code(),
                            result: commission_result,
                        },
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    };
                }
            };

            // The engine's ClosedLoopEntry phase already set PID gains, limits,
            // filters, and entered position mode. Re-apply current limit because
            // the mode change resets PID_TORQUE_FLUX_LIMITS.
            let current_limit = state.tmc.load_current_limit(Relaxed).register_value();
            if backend
                .write_register(
                    foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                    current_limit,
                )
                .await
                .is_err()
            {
                defmt::error!("CommissionDone: current-limit restore SPI failed");
                crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                state.tmc.commission_in_flight.store(false, Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CommissionResult {
                        oid,
                        status: crate::autotune::CommissionError::Spi.status_code(),
                        result: commission_result,
                    },
                    actions: PostActions {
                        stop_trace: true,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            *state.calibrated = true;
            if let Err(error) =
                crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
            {
                defmt::error!("CommissionDone: arm sentinel verify failed: {}", error);
                crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                state.tmc.commission_in_flight.store(false, Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CommissionResult {
                        oid,
                        status: error.status_code(),
                        result: commission_result,
                    },
                    actions: PostActions {
                        stop_trace: true,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            // Publish the completed result only after terminal restore and
            // sentinel verification have both succeeded.
            *state.stored_gains = Some(crate::pid::InnerLoopGains {
                flux_p: foci_tmc4671::pid::PidPGain::new(inner.flux_p)
                    .expect("commissioned flux P must be valid"),
                flux_i: foci_tmc4671::pid::PidIGain::new(inner.flux_i)
                    .expect("commissioned flux I must be valid"),
                torque_p: foci_tmc4671::pid::PidPGain::new(inner.torque_p)
                    .expect("commissioned torque P must be valid"),
                torque_i: foci_tmc4671::pid::PidIGain::new(inner.torque_i)
                    .expect("commissioned torque I must be valid"),
            });
            state.tmc.store_outer_gains(fallback_outer_gains, Relaxed);
            state
                .tmc
                .velocity_limit_cfg
                .store(commission_result.fallback_velocity_limit, Relaxed);

            state.tmc.commission_in_flight.store(false, Relaxed);

            defmt::info!("foci_commission complete: oid={} status={}", oid, status);
            CommissionOutcome::Terminal {
                reply: ReplyPayload::CommissionResult {
                    oid,
                    status,
                    result: commission_result,
                },
                actions: PostActions {
                    stop_trace: true,
                    stepper: StepperPostAction::ArmZeroPosition,
                },
            }
        }
        StepResult::OuterDone(outer) => {
            dispatch_outer_done(outer, engine, backend, state, pwm_maxcnt).await
        }
        StepResult::OuterFailed(error) => {
            use crate::autotune::CommissionError as CE;
            use core::sync::atomic::Ordering::Relaxed;
            let mut error = error;
            let velocity_workflow_restored = if engine
                .as_ref()
                .and_then(CommissioningEngine::velocity_operational_torque_limit)
                .is_some()
            {
                disarm_velocity_workflow(engine, backend, state).await
            } else {
                true
            };
            if !velocity_workflow_restored {
                error = CE::Spi;
            }
            let engine_ref = engine.as_ref().expect("engine slot None on OuterFailed");
            let oid = engine_ref.oid();
            let snapshot = engine_ref.entry_snapshot();
            let safety_fault = engine_ref.outer_safety_fault();

            let is_safety_fault =
                matches!(error, CE::Spi | CE::Shutdown | CE::SafetyEnvelopeViolation);

            let mut restore_drift_counts = 0u32;
            let restore_failure: Option<CE> = if is_safety_fault {
                // Safety faults skip restore — motor state unknown, disarm immediately.
                Some(error)
            } else if let Some(ref snap) = snapshot {
                match crate::commissioning::inner::restore_entry_gains(snap, backend, state.tmc)
                    .await
                {
                    Ok(()) => match outer_restore_position_stable(backend).await {
                        Ok(stability) => {
                            restore_drift_counts = stability.span_counts;
                            if stability.stable {
                                None
                            } else {
                                // Preserve the upstream error; the stability
                                // log is the secondary diagnostic. Overriding
                                // with CurrentValidationFailed would mask the
                                // meaningful failure (e.g.
                                // VelocityValidationFailed).
                                Some(error)
                            }
                        }
                        Err(_) => {
                            defmt::error!("OuterFailed restore: stability read SPI failed");
                            Some(CE::Spi)
                        }
                    },
                    Err(restore_err) => {
                        defmt::error!("OuterFailed: Stage 2 restore failed: {}", restore_err);
                        Some(restore_err)
                    }
                }
            } else {
                // No snapshot available — treat as unrecoverable.
                Some(CE::Spi)
            };
            let restore_status = restore_failure.map_or(0, |fault| fault.status_code());

            let (reply_status, actions) = if let Some(fault) = restore_failure {
                let _ = backend
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                defmt::error!("OuterFailed: safety fault ({}), motor disabled", fault);
                (
                    fault.status_code(),
                    PostActions {
                        stop_trace: true,
                        stepper: StepperPostAction::Stop,
                    },
                )
            } else {
                *state.calibrated = true;
                if let Err(arm_error) =
                    crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
                {
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    defmt::error!(
                        "OuterFailed: arm sentinel verify failed after soft failure: {}",
                        arm_error
                    );
                    (
                        arm_error.status_code(),
                        PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    )
                } else {
                    defmt::warn!(
                        "OuterFailed: soft failure ({}), entry gains restored",
                        error
                    );
                    (
                        error.status_code(),
                        PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::ArmPreservePosition,
                        },
                    )
                }
            };

            let disposition = match actions.stepper {
                StepperPostAction::ArmPreservePosition => 0,
                StepperPostAction::Stop if is_safety_fault => 2,
                StepperPostAction::Stop => 1,
                StepperPostAction::ArmZeroPosition | StepperPostAction::None => 0,
            };
            let restore_level = if restore_status == 0 {
                crate::trace::events::TraceEventLevel::Warn
            } else {
                crate::trace::events::TraceEventLevel::Error
            };
            record_restore_summary(
                backend,
                engine
                    .as_mut()
                    .expect("engine slot None before OuterFailed restore summary")
                    .trace_summary_seqs_mut(),
                restore_status,
                error.status_code(),
                restore_drift_counts,
                disposition,
                restore_level,
            );

            state.tmc.commission_in_flight.store(false, Relaxed);

            let consumed = if velocity_workflow_restored {
                CommissioningEngine::take_after_handled_stop(engine)
            } else {
                engine.take()
            };
            let _ = consumed.expect("engine slot None after OuterFailed safe restoration");

            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult {
                    oid,
                    status: reply_status,
                    result: zero_outer_result(),
                    safety_fault,
                },
                actions,
            }
        }
    }
}

/// Dispatch `StepResult::VelocityIntegralTerminal` directly, without routing
/// through the multi-variant [`drive_commission_result`] dispatcher.
///
/// Extracted so a caller that already knows the variant (the Stage-C staged
/// wrapper) embeds only this frame instead of the full dispatcher's; the
/// dispatcher still delegates here for its own callers, so behavior for every
/// existing call path is unchanged.
async fn dispatch_velocity_integral_terminal<B: CommissioningBackend>(
    mut terminal: crate::commissioning::types::VelocityIntegralTerminal,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> CommissionOutcome {
    use core::sync::atomic::Ordering::Relaxed;

    if let Some(record) = engine
        .as_ref()
        .and_then(|engine| engine.attach_stage_c_rest_boundary(&mut terminal))
    {
        #[cfg(feature = "trace")]
        backend.record_velocity_sweep_trace_record(record);
        #[cfg(not(feature = "trace"))]
        let _ = record;
    }
    if !disarm_velocity_workflow(engine, backend, state).await {
        terminal.outcome = crate::commissioning::types::VelocityIntegralOutcome::Fault;
        terminal.cause = crate::commissioning::types::StageCCause::Error(
            crate::autotune::CommissionError::Spi.status_code(),
        );
        terminal.recovery_unavailable = true;
        terminal.safety_fault = None;
    }
    let consumed = engine
        .take()
        .expect("engine slot is None on VelocityIntegralTerminal");
    let oid = consumed.oid();
    state.tmc.commission_in_flight.store(false, Relaxed);
    CommissionOutcome::Terminal {
        reply: ReplyPayload::VelocityIntegralTerminal {
            oid,
            terminal,
            reproduction: None,
            authority: None,
        },
        actions: PostActions {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
        },
    }
}

/// Dispatch `StepResult::AcceptanceMatrixTerminal` directly; see
/// [`dispatch_velocity_integral_terminal`] for why this is extracted.
async fn dispatch_acceptance_matrix_terminal<B: CommissioningBackend>(
    terminal: crate::commissioning::types::AcceptanceMatrixTerminal,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> CommissionOutcome {
    use core::sync::atomic::Ordering::Relaxed;

    let consumed = engine
        .take()
        .expect("engine slot is None on AcceptanceMatrixTerminal");
    let oid = consumed.oid();
    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
    state.tmc.commission_in_flight.store(false, Relaxed);
    CommissionOutcome::Terminal {
        reply: ReplyPayload::AcceptanceMatrixTerminal { oid, terminal },
        actions: PostActions {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
        },
    }
}

/// Dispatch `StepResult::RobustnessReversalTerminal` directly; see
/// [`dispatch_velocity_integral_terminal`] for why this is extracted.
async fn dispatch_robustness_reversal_terminal<B: CommissioningBackend>(
    mut terminal: crate::commissioning::types::RobustnessReversalTerminal,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> CommissionOutcome {
    use core::sync::atomic::Ordering::Relaxed;

    let consumed = engine
        .take()
        .expect("engine slot is None on RobustnessReversalTerminal");
    let oid = consumed.oid();
    // Every non-`Complete` outcome (`Rejected`, `Inconclusive`, `Failed`)
    // leaves the live registers holding the probed candidate; restore
    // the operator's Stage 2 baseline before disarm so a non-passing
    // probe never leaves the wrong gains armed. A `Complete` outcome
    // deploys instead (staged wrapper), so it never reaches here with
    // the entry snapshot still needed.
    if terminal.outcome != crate::commissioning::types::RobustnessOutcome::Complete {
        let restored = match consumed.entry_snapshot() {
            Some(snapshot) => {
                match restore::restore_entry_snapshot_to_registers(backend, &snapshot).await {
                    Ok(()) => true,
                    Err(err) => {
                        // `restore::RestoreError` is generic over the
                        // backend's SPI error type, which is not
                        // `defmt::Format`-bound here; log the variant
                        // instead of the whole error so a transport
                        // failure is still distinguishable from a
                        // readback mismatch or an invalid-gains
                        // rejection.
                        let variant = match err {
                            restore::RestoreError::Spi(_) => "spi",
                            restore::RestoreError::InvalidGains => "invalid_gains",
                            restore::RestoreError::PiRepresentation => "pi_representation",
                            restore::RestoreError::VelocityClamp => "velocity_clamp",
                            restore::RestoreError::Readback => "readback",
                        };
                        defmt::error!("robustness terminal: baseline restore failed: {}", variant);
                        false
                    }
                }
            }
            None => {
                defmt::error!("robustness terminal: no entry snapshot to restore baseline from");
                false
            }
        };
        if !restored {
            terminal.outcome = crate::commissioning::types::RobustnessOutcome::Failed;
            terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
        }
    }
    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
    state.tmc.commission_in_flight.store(false, Relaxed);
    CommissionOutcome::Terminal {
        reply: ReplyPayload::RobustnessReversalTerminal { oid, terminal },
        actions: PostActions {
            stop_trace: true,
            stepper: StepperPostAction::Stop,
        },
    }
}

/// Dispatch `StepResult::OuterDone` directly; see
/// [`dispatch_velocity_integral_terminal`] for why this is extracted.
async fn dispatch_outer_done<B: CommissioningBackend>(
    outer: OuterResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    pwm_maxcnt: u16,
) -> CommissionOutcome {
    use core::sync::atomic::Ordering::Relaxed;
    let consumed = engine.take().expect("engine slot None on OuterDone");
    let oid = consumed.oid();
    state.tmc.motor_armed.store(false, Relaxed);

    // SPI restore — enter position mode with the tuned gains.
    let restored_gains =
        match restore::restore_position_mode_after_outer(backend, state.tmc, &outer).await {
            Ok(gains) => gains,
            Err(error) => {
                let status = match error {
                    restore::RestoreError::Spi(_)
                    | restore::RestoreError::VelocityClamp
                    | restore::RestoreError::Readback => crate::autotune::CommissionError::Spi,
                    restore::RestoreError::PiRepresentation => {
                        crate::autotune::CommissionError::ChipResetDetected
                    }
                    restore::RestoreError::InvalidGains => {
                        crate::autotune::CommissionError::VelocityValidationFailed
                    }
                };
                defmt::error!("OuterDone: position-mode restore failed");
                crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                state.tmc.commission_in_flight.store(false, Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::TuneResult {
                        oid,
                        status: status.status_code(),
                        result: zero_outer_result(),
                        safety_fault: None,
                    },
                    actions: PostActions {
                        stop_trace: true,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }
        };

    if crate::tmc_control::filters::apply_outer_result_filters_via_backend(
        backend, &outer, pwm_maxcnt,
    )
    .await
    .is_err()
    {
        defmt::error!("OuterDone: candidate filter apply SPI failed");
        crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
        state.tmc.commission_in_flight.store(false, Relaxed);
        return CommissionOutcome::Terminal {
            reply: ReplyPayload::TuneResult {
                oid,
                status: crate::autotune::CommissionError::Spi.status_code(),
                result: zero_outer_result(),
                safety_fault: None,
            },
            actions: PostActions {
                stop_trace: true,
                stepper: StepperPostAction::Stop,
            },
        };
    }

    *state.calibrated = true;
    if let Err(error) = crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await {
        defmt::error!("OuterDone: arm sentinel verify failed: {}", error);
        crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
        state.tmc.commission_in_flight.store(false, Relaxed);
        return CommissionOutcome::Terminal {
            reply: ReplyPayload::TuneResult {
                oid,
                status: error.status_code(),
                result: outer,
                safety_fault: None,
            },
            actions: PostActions {
                stop_trace: true,
                stepper: StepperPostAction::Stop,
            },
        };
    }

    // Publish the completed result only after terminal restore,
    // candidate filter application, and sentinel verification succeed.
    state.tmc.store_outer_gains(restored_gains, Relaxed);
    state
        .tmc
        .velocity_limit_cfg
        .store(outer.velocity_limit, Relaxed);
    state
        .tmc
        .velocity_filter_hz
        .store(Some(outer.velocity_filter_hz), Relaxed);
    state
        .tmc
        .torque_filter_hz
        .store(Some(outer.torque_filter_hz), Relaxed);
    state
        .tmc
        .position_filter_hz
        .store(Some(outer.position_filter_hz), Relaxed);
    state
        .tmc
        .flux_filter_hz
        .store(Some(outer.flux_filter_hz), Relaxed);

    state.tmc.commission_in_flight.store(false, Relaxed);

    let status = match outer.status {
        crate::autotune::CommissionStatus::Accepted => 0u8,
        crate::autotune::CommissionStatus::AcceptedWithWarnings => 1,
        crate::autotune::CommissionStatus::InnerOnly => 2,
        crate::autotune::CommissionStatus::Failed(e) => e.status_code(),
    };
    CommissionOutcome::Terminal {
        reply: ReplyPayload::TuneResult {
            oid,
            status,
            result: outer,
            safety_fault: None,
        },
        actions: PostActions {
            stop_trace: true,
            stepper: StepperPostAction::ArmPreservePosition,
        },
    }
}

/// Dispatch one Stage-C terminal while applying its retained-evidence lifecycle.
pub async fn drive_velocity_integral_terminal_with_stages<B: CommissioningBackend>(
    mut terminal: crate::commissioning::types::VelocityIntegralTerminal,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    pwm_maxcnt: u16,
    channel: u8,
    retention: CommissionRetention<'_>,
) -> CommissionOutcome {
    let _ = pwm_maxcnt; // dispatch_velocity_integral_terminal does not need it
    if let Some(record) = engine
        .as_ref()
        .and_then(|engine| engine.attach_stage_c_rest_boundary(&mut terminal))
    {
        #[cfg(feature = "trace")]
        backend.record_velocity_sweep_trace_record(record);
        #[cfg(not(feature = "trace"))]
        let _ = record;
    }
    // Captured before disarm can tear the phase down. `Some` only for a
    // breakaway run, so it doubles as the `acceptance_source == Breakaway`
    // discriminator; the authority the promotion held arrives on `dispatch`.
    #[cfg(feature = "trace")]
    let promotion_source_diagnostic = engine.as_ref().map_or(
        (0, false),
        CommissioningEngine::stage_c_promotion_source_diagnostic,
    );
    let breakaway_execution_plan = engine
        .as_ref()
        .and_then(CommissioningEngine::breakaway_stage_c_execution_plan);
    #[cfg(feature = "trace")]
    let promotion_execution_plan_present = breakaway_execution_plan.is_some();
    let mut dispatch = stage_c::apply_stage_c_terminal(terminal, retention.stage_c);
    #[cfg(feature = "trace")]
    let promotion_authority_present = dispatch.authority.is_some();

    // A promoted breakaway `Complete` (never a first `CompleteCandidate`, which
    // is promoted only on the reproduced second run) installs the token-carrying
    // acceptance point. The token rides inside the point, so it is cleared
    // automatically with the point -- no separate retention.
    #[cfg(feature = "trace")]
    let mut promotion_installer_result: u8 = 0;
    if dispatch.terminal.outcome == crate::commissioning::types::VelocityIntegralOutcome::Complete
        && let (Some(execution_plan), Some(authority)) =
            (breakaway_execution_plan, dispatch.authority)
    {
        match stage_c::breakaway_acceptance_point(
            channel,
            &authority,
            execution_plan,
            &dispatch.terminal,
        ) {
            Ok(stage_c::AcceptanceInstall::Installed(point)) => {
                retention.stage_c.install_acceptance_point(point);
                #[cfg(feature = "trace")]
                {
                    promotion_installer_result = 1;
                }
            }
            Ok(stage_c::AcceptanceInstall::MissingTargetTerminus) => {
                dispatch.terminal.cause = crate::commissioning::types::StageCCause::Dispatch(
                    stage_c::STAGE_C_CAUSE_NO_ACCEPTANCE_TERMINUS,
                );
                #[cfg(feature = "trace")]
                {
                    promotion_installer_result = 2;
                }
            }
            Ok(stage_c::AcceptanceInstall::NotAcceptanceTerminal) => {
                #[cfg(feature = "trace")]
                {
                    promotion_installer_result = 3;
                }
            }
            Err(()) => {
                retention.stage_c.observe_fault();
                dispatch.terminal.outcome =
                    crate::commissioning::types::VelocityIntegralOutcome::Fault;
                dispatch.terminal.cause = crate::commissioning::types::StageCCause::Dispatch(
                    stage_c::STAGE_C_CAUSE_EVIDENCE_INTEGRITY,
                );
                dispatch.terminal.safety_fault = None;
                dispatch.reproduction = None;
                dispatch.authority = None;
                #[cfg(feature = "trace")]
                {
                    promotion_installer_result = 4;
                }
            }
        }
    }

    #[cfg(feature = "trace")]
    {
        let (acceptance_source_code, phase_live) = promotion_source_diagnostic;
        backend.record_velocity_sweep_trace_record(
            crate::commissioning::outer::velocity::integral_reporting::encode_stage_c_promotion_diagnostic(
                crate::commissioning::outer::velocity::integral_reporting::StageCPromotionDiagnostic {
                    channel,
                    outcome: crate::commissioning::outer::velocity::integral_reporting::outcome_code(
                        dispatch.terminal.outcome,
                    ),
                    acceptance_source_code,
                    phase_live,
                    breakaway_acceptance: dispatch.terminal.breakaway_acceptance,
                    execution_plan_present: promotion_execution_plan_present,
                    authority_present: promotion_authority_present,
                    rest_boundary_rung_plus_one: dispatch.terminal.rest_boundary_rung_plus_one,
                    rest_boundary_slot_plus_one: dispatch.terminal.rest_boundary_slot_plus_one,
                    installer_result: promotion_installer_result,
                },
            ),
        );
    }

    // Restore the operator's Stage 2 entry baseline before disarming.
    // `disarm_velocity_workflow` below restores the *current* limit (owned by
    // tmc_state, not the snapshot), so this must run first -- reversing the
    // order would let this restore overwrite the current-limit register the
    // disarm call is about to set.
    let baseline_restored = match engine
        .as_ref()
        .and_then(CommissioningEngine::entry_snapshot)
    {
        Some(snapshot) => restore::restore_entry_snapshot_to_registers(backend, &snapshot)
            .await
            .is_ok(),
        None => false,
    };

    // Every path -- `CompleteCandidate`, `Complete`, and every fault -- disarms
    // the workflow, forcing a fault terminal if the disarm SPI itself fails.
    if !disarm_velocity_workflow(engine, backend, state).await || !baseline_restored {
        retention.stage_c.observe_fault();
        dispatch.terminal.outcome = crate::commissioning::types::VelocityIntegralOutcome::Fault;
        dispatch.terminal.cause = crate::commissioning::types::StageCCause::Error(
            crate::autotune::CommissionError::Spi.status_code(),
        );
        dispatch.terminal.recovery_unavailable = true;
        dispatch.terminal.safety_fault = None;
        dispatch.reproduction = None;
        dispatch.authority = None;
    }
    #[cfg(feature = "trace")]
    engine
        .as_mut()
        .expect("engine slot is None on VelocityIntegralTerminal")
        .emit_stage_c_terminal_trace(backend, &dispatch.terminal, dispatch.reproduction);
    match dispatch_velocity_integral_terminal(dispatch.terminal, engine, backend, state).await {
        CommissionOutcome::Terminal {
            reply: ReplyPayload::VelocityIntegralTerminal { oid, terminal, .. },
            actions,
        } => CommissionOutcome::Terminal {
            reply: ReplyPayload::VelocityIntegralTerminal {
                oid,
                terminal,
                reproduction: dispatch.reproduction,
                authority: dispatch.authority,
            },
            actions,
        },
        _ => unreachable!("velocity-integral terminal produced a nonterminal outcome"),
    }
}

/// Dispatch one Stage-B result while applying its retained-evidence lifecycle.
pub async fn drive_commission_result_with_stages<B: CommissioningBackend>(
    result: StepResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    pwm_maxcnt: u16,
    channel: u8,
    retention: CommissionRetention<'_>,
) -> CommissionOutcome {
    match result {
        StepResult::AcceptanceMatrixTerminal(mut terminal) => {
            let matrix_context = engine.as_ref().and_then(|engine| {
                engine
                    .acceptance_matrix_order()
                    .zip(engine.acceptance_matrix_plan_digest())
            });
            if !disarm_velocity_workflow(engine, backend, state).await {
                retention.stage_c.observe_fault();
                terminal.outcome = crate::commissioning::types::AcceptanceMatrixOutcome::Fault;
                terminal.cause =
                    crate::commissioning::types::AcceptanceMatrixCause::EvidenceIntegrity;
                terminal.safety_fault = None;
            } else if let Some((order, plan_digest)) = matrix_context {
                terminal = acceptance::apply_acceptance_matrix_terminal(
                    terminal,
                    order,
                    plan_digest,
                    retention.stage_c,
                );
            } else {
                retention.stage_c.observe_fault();
                terminal.outcome = crate::commissioning::types::AcceptanceMatrixOutcome::Fault;
                terminal.cause =
                    crate::commissioning::types::AcceptanceMatrixCause::EvidenceIntegrity;
                terminal.safety_fault = None;
            }
            #[cfg(feature = "trace")]
            engine
                .as_mut()
                .expect("engine slot is None on AcceptanceMatrixTerminal")
                .emit_acceptance_matrix_terminal_trace(backend, terminal);
            dispatch_acceptance_matrix_terminal(terminal, engine, backend, state).await
        }
        StepResult::RobustnessReversalTerminal(terminal) => {
            // The robustness probe reads the retained accepted point read-only,
            // so the only staged policy is the identity check: it re-stamps the
            // terminal `Failed`/`EVIDENCE_INTEGRITY` if the retained point moved
            // underneath the probe.
            let mut terminal = robustness::apply_robustness_terminal(terminal, retention.stage_c);
            // A passed gate deploys the robustness-selected gains: join them with
            // the entry snapshot and route through the shared `OuterDone` arm so
            // the tune result and the host persist path light up. The join needs
            // both the live entry snapshot and the retained accepted point;
            // without either, fall back to the fault-terminal path rather than
            // publish an incomplete tune. Every non-pass outcome forwards the
            // fault terminal unchanged and deploys nothing.
            let deploy = (terminal.outcome
                == crate::commissioning::types::RobustnessOutcome::Complete)
                .then(|| {
                    engine
                        .as_ref()
                        .and_then(CommissioningEngine::entry_snapshot)
                        .zip(retention.stage_c.acceptance_point())
                })
                .flatten();
            match deploy {
                Some((entry, point)) => {
                    let result = crate::commissioning::outer::velocity::robustness_reversal::robustness_pass_outer_result(
                        &terminal,
                        &entry,
                        &point,
                    );
                    dispatch_outer_done(result, engine, backend, state, pwm_maxcnt).await
                }
                None => {
                    // A `Complete` outcome that could not be joined (missing
                    // entry snapshot or retained acceptance point) is not a
                    // deploy and not a genuine non-pass terminal either -- it
                    // is a firmware evidence-integrity gap. Stamp it `Failed`
                    // before forwarding so the generic handler's restore
                    // (gated on non-`Complete`) actually runs, and so the host
                    // sees the safety-fault cause rather than a stale pass.
                    if terminal.outcome == crate::commissioning::types::RobustnessOutcome::Complete
                    {
                        terminal.outcome = crate::commissioning::types::RobustnessOutcome::Failed;
                        terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
                    }
                    dispatch_robustness_reversal_terminal(terminal, engine, backend, state).await
                }
            }
        }
        StepResult::BreakawayStageCPlanReady { run_sequence, plan } => {
            stage_c::install_authority_from_plan(channel, retention.stage_c, plan);
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on BreakawayStageCPlanReady");
            CommissionOutcome::Evidence {
                reply: ReplyPayload::VelocityIntegralPlan {
                    oid: engine_ref.oid(),
                    run_sequence,
                    plan,
                },
            }
        }
        StepResult::VelocityIntegralTerminal(_) => {
            unreachable!("Stage-C terminals use their stage-owned dispatcher")
        }
        _ => unreachable!("non-velocity terminal reached retained velocity dispatch"),
    }
}

#[cfg(feature = "trace")]
pub(crate) fn emit_outer_safety_timing_before_failure<B: CommissioningBackend>(
    result: &StepResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
) {
    if !matches!(result, StepResult::OuterFailed(_)) {
        return;
    }
    let Some(run_sequence) = engine
        .as_ref()
        .and_then(CommissioningEngine::trace_velocity_run_sequence)
    else {
        return;
    };
    engine
        .as_mut()
        .expect("trace velocity engine disappeared before terminal timing")
        .emit_outer_safety_timing_trace(backend, run_sequence);
}
