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
pub mod post_actions;
pub mod replies;
mod restore;
#[cfg(test)]
pub(crate) mod test_support;
#[cfg(test)]
mod tests;

pub use post_actions::{StepperTaskDirective, apply_stepper_post_action_state};
pub use replies::{
    CommissionReplySink, emit_phase_reply, emit_terminal_reply, emit_velocity_sweep_event,
};

use crate::autotune::PhaseId;
// CommissioningEngine lives in commissioning/mod.rs (the engine impl); the
// data types live in commissioning/types.rs.
use crate::commissioning::{
    CommissioningEngine,
    types::{
        AdcCalResult, CommissionResult, CommissioningBackend, EngineKind, OuterInconclusive,
        OuterResult, OuterSafetyFault, StepResult, VelocitySweepEvent, VelocitySweepOutcome,
        VelocitySweepTerminal,
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
    /// One streamed velocity-sweep plan, observation, or rung report.
    VelocitySweepEvent { oid: u8, event: VelocitySweepEvent },
    /// Detailed report-only terminal, optionally followed by a generic inconclusive terminal.
    VelocitySweepTerminal {
        oid: u8,
        terminal: VelocitySweepTerminal,
        inconclusive: Option<OuterInconclusive>,
    },
}

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

fn validate_outer_gains(
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) -> Result<OuterLoopGains, foci_tmc4671::pid::PidGainOutOfRange> {
    use foci_tmc4671::pid::PidGain;

    Ok(OuterLoopGains {
        position_p: PidGain::new(position_p)?,
        position_i: PidGain::new(position_i)?,
        velocity_p: PidGain::new(velocity_p)?,
        velocity_i: PidGain::new(velocity_i)?,
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
        _ => CalibrationError::InternalError,
    };
    calibration_error.status_code()
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
        StepResult::VelocitySweepEvent(event) => {
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on VelocitySweepEvent");
            CommissionOutcome::Evidence {
                reply: ReplyPayload::VelocitySweepEvent {
                    oid: engine_ref.oid(),
                    event,
                },
            }
        }
        StepResult::VelocitySweepTerminal(terminal) => {
            use core::sync::atomic::Ordering::Relaxed;

            let consumed = engine
                .take()
                .expect("engine slot is None on VelocitySweepTerminal");
            let oid = consumed.oid();
            crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
            state.tmc.commission_in_flight.store(false, Relaxed);
            let inconclusive = (terminal.outcome == VelocitySweepOutcome::Inconclusive).then_some(
                OuterInconclusive {
                    phase: PhaseId::VelocityTune,
                    sufficient_direction_mask: terminal.model_direction_mask,
                    cause: terminal.cause,
                    digest_low: terminal.digest as u32,
                    digest_high: (terminal.digest >> 32) as u32,
                },
            );
            CommissionOutcome::Terminal {
                reply: ReplyPayload::VelocitySweepTerminal {
                    oid,
                    terminal,
                    inconclusive,
                },
                actions: PostActions {
                    stop_trace: true,
                    stepper: StepperPostAction::Stop,
                },
            }
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
            if restore_result.is_err() {
                defmt::error!("CalibrateDone: saved gain restore SPI failed");
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
                flux_p: foci_tmc4671::pid::PidGain::new(inner.flux_p)
                    .expect("commissioned flux P must be valid"),
                flux_i: foci_tmc4671::pid::PidGain::new(inner.flux_i)
                    .expect("commissioned flux I must be valid"),
                torque_p: foci_tmc4671::pid::PidGain::new(inner.torque_p)
                    .expect("commissioned torque P must be valid"),
                torque_i: foci_tmc4671::pid::PidGain::new(inner.torque_i)
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
            use core::sync::atomic::Ordering::Relaxed;
            let consumed = engine.take().expect("engine slot None on OuterDone");
            let oid = consumed.oid();
            state.tmc.motor_armed.store(false, Relaxed);

            // SPI restore — enter position mode with the tuned gains.
            let restored_gains = match restore::restore_position_mode_after_outer(
                backend, state.tmc, &outer,
            )
            .await
            {
                Ok(gains) => gains,
                Err(error) => {
                    let status = match error {
                        restore::RestoreError::Spi(_) => crate::autotune::CommissionError::Spi,
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
            if let Err(error) =
                crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
            {
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
        StepResult::OuterFailed(error) => {
            use crate::autotune::CommissionError as CE;
            use core::sync::atomic::Ordering::Relaxed;
            let mut consumed = engine.take().expect("engine slot None on OuterFailed");
            let oid = consumed.oid();
            let snapshot = consumed.entry_snapshot();
            let safety_fault = consumed.outer_safety_fault();

            let is_safety_fault =
                matches!(error, CE::Spi | CE::Shutdown | CE::SafetyEnvelopeViolation);

            let mut restore_drift_counts = 0u32;
            let restore_failure: Option<CE> = if is_safety_fault {
                // Safety faults skip restore — motor state unknown, disarm immediately.
                Some(error)
            } else if let Some(ref snap) = snapshot {
                match crate::commissioning::inner::restore_entry_gains(snap, backend).await {
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
                consumed.trace_summary_seqs_mut(),
                restore_status,
                error.status_code(),
                restore_drift_counts,
                disposition,
                restore_level,
            );

            state.tmc.commission_in_flight.store(false, Relaxed);

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
