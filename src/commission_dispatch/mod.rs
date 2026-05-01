//! Board-agnostic post-processing for `CommissioningEngine::step()` results.
//!
//! Lifted from `openffboard-fw::tmc_control` per the design at
//! `docs/specs/2026-04-23-commission-dispatch-extraction-design.md`.
//!
//! Public surface: a single `drive_commission_result` function plus the
//! types it returns. The board calls this once per engine.step() result
//! and matches `CommissionOutcome` to fire RTIC primitives + emit replies.

// pub(crate) so sibling test modules (tmc_ops, commissioning, …) can
// import MockBackend + helpers via crate::commission_dispatch::test_support::*.
#[cfg(test)]
pub(crate) mod test_support;
#[cfg(test)]
mod tests;

use crate::autotune::PhaseId;
// CommissioningEngine lives in commissioning/mod.rs (the engine impl); the
// data types live in commissioning/types.rs.
use crate::commissioning::{
    CommissioningEngine,
    types::{
        AdcCalResult, CommissionResult, CommissioningBackend, EngineKind, OuterResult, StepResult,
    },
};
use crate::pid::InnerLoopGains;
use crate::selftest::SelftestResults;

/// Borrowed view of the per-engine task locals the dispatch needs to mutate.
///
/// The board's `tmc_control` task owns these as task locals; per-dispatch
/// it constructs a `DispatchState` so the firmware can mutate them in
/// place without copy-back.
pub struct DispatchState<'a> {
    pub calibrated: &'a mut bool,
    pub motor_enabled: &'a mut bool,
    pub stored_gains: &'a mut Option<InnerLoopGains>,
}

/// What the board should do after the dispatch returns.
pub enum CommissionOutcome {
    /// Engine wants more time. Board: trace_tick, Mono::delay(1ms), continue.
    Continue,
    /// A phase transition occurred. Board: update trace_buf phase (and
    /// conditionally start trace at ClosedLoopEntry for Commission), emit
    /// FociCommissionPhase reply, Mono::delay(1ms), continue.
    PhaseChanged {
        oid: u8,
        phase_id: PhaseId,
        engine_kind: EngineKind,
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

/// Wire-reply payload for the board to emit via `klipper_reply_from!`.
pub enum ReplyPayload {
    /// FociCommissionPhase: phase change reply (foci_commission failures
    /// during inner phases).
    CommissionPhase { oid: u8, phase: u8, status: u8 },
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
    },
    /// FociCalibrateResult: ADC calibration complete.
    CalibrateResult {
        oid: u8,
        status: u8,
        result: AdcCalResult,
    },
    /// FociSelftestResult per stage + FociSelftestDone terminal.
    /// Board emits one Result message per populated stage, then Done.
    Selftest {
        oid: u8,
        results: SelftestResults,
        done_status: u8,
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
    }
}

/// Helper: zero-filled AdcCalResult for failure-path CalibrateResult emission.
fn zero_adc_cal_result() -> AdcCalResult {
    AdcCalResult {
        adc_i0: 0,
        adc_i1: 0,
        encoder_count: 0,
    }
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
        StepResult::Continue => CommissionOutcome::Continue,
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
        StepResult::SelftestDone => {
            let mut consumed = engine.take().expect("engine slot None on SelftestDone");
            let oid = consumed.oid();
            let results = consumed.take_selftest_results();
            crate::state::tmc::COMMISSION_IN_FLIGHT
                .store(false, core::sync::atomic::Ordering::Relaxed);
            CommissionOutcome::Terminal {
                reply: ReplyPayload::Selftest {
                    oid,
                    results,
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
            crate::state::tmc::COMMISSION_IN_FLIGHT
                .store(false, core::sync::atomic::Ordering::Relaxed);
            crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;

            let reply = match kind {
                EngineKind::Calibrate => ReplyPayload::CalibrateResult {
                    oid,
                    status: error.status_code(),
                    result: zero_adc_cal_result(),
                },
                EngineKind::Selftest => ReplyPayload::Selftest {
                    oid,
                    results: selftest_results,
                    done_status: error.status_code(),
                },
                EngineKind::Commission | EngineKind::OuterTune => ReplyPayload::CommissionPhase {
                    oid,
                    phase: 0,
                    status: error.status_code(),
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
            let restore_result =
                crate::tmc_ops::restore_calibrate_gains(backend, *state.stored_gains).await;
            if restore_result.is_err() {
                defmt::error!("CalibrateDone: saved gain restore SPI failed");
                crate::tmc_ops::disarm_motor(backend, state).await;
                crate::state::tmc::COMMISSION_IN_FLIGHT
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CalibrateResult {
                        oid,
                        status: crate::calibration::CalibrationError::SpiError.status_code(),
                        result: zero_adc_cal_result(),
                    },
                    actions: PostActions {
                        stop_trace: false,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            if crate::tmc_ops::apply_stored_filters_via_backend(backend, pwm_maxcnt)
                .await
                .is_err()
            {
                defmt::error!("CalibrateDone: stored filter restore SPI failed");
                crate::tmc_ops::disarm_motor(backend, state).await;
                crate::state::tmc::COMMISSION_IN_FLIGHT
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::CalibrateResult {
                        oid,
                        status: crate::calibration::CalibrationError::SpiError.status_code(),
                        result: zero_adc_cal_result(),
                    },
                    actions: PostActions {
                        stop_trace: false,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            *state.calibrated = true;
            crate::tmc_ops::arm_motor_electrical(backend, state);

            crate::state::tmc::COMMISSION_IN_FLIGHT
                .store(false, core::sync::atomic::Ordering::Relaxed);

            CommissionOutcome::Terminal {
                reply: ReplyPayload::CalibrateResult {
                    oid,
                    status: 0,
                    result: cal_result,
                },
                actions: PostActions {
                    stop_trace: false,
                    stepper: StepperPostAction::ArmZeroPosition,
                },
            }
        }
        StepResult::CommissionDone(commission_result) => {
            use core::sync::atomic::Ordering::Relaxed;
            let consumed = engine.take().expect("engine slot None on CommissionDone");
            let oid = consumed.oid();

            // The engine's ClosedLoopEntry phase already set PID gains, limits,
            // filters, and entered position mode. Re-apply current limit because
            // the mode change resets PID_TORQUE_FLUX_LIMITS.
            let current_limit = crate::state::tmc::CURRENT_LIMIT_REG.load(Relaxed);
            if backend
                .write_register(
                    foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                    current_limit,
                )
                .await
                .is_err()
            {
                defmt::error!("CommissionDone: current-limit restore SPI failed");
                crate::tmc_ops::disarm_motor(backend, state).await;
                crate::state::tmc::COMMISSION_IN_FLIGHT.store(false, Relaxed);
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

            // Persist inner gains so future calibrate uses them.
            let inner = &commission_result.inner;
            *state.stored_gains = Some(crate::pid::InnerLoopGains {
                flux_p: inner.flux_p,
                flux_i: inner.flux_i,
                torque_p: inner.torque_p,
                torque_i: inner.torque_i,
            });

            // Sync outer-gain atomics to the fallback values the engine wrote.
            crate::state::tmc::POSITION_P_VALUE
                .store(commission_result.fallback_position_p, Relaxed);
            crate::state::tmc::POSITION_I_VALUE
                .store(commission_result.fallback_position_i, Relaxed);
            crate::state::tmc::VELOCITY_P_VALUE
                .store(commission_result.fallback_velocity_p, Relaxed);
            crate::state::tmc::VELOCITY_I_VALUE
                .store(commission_result.fallback_velocity_i, Relaxed);
            crate::state::tmc::VELOCITY_LIMIT_CFG
                .store(commission_result.fallback_velocity_limit, Relaxed);

            *state.calibrated = true;
            crate::tmc_ops::arm_motor_electrical(backend, state);

            crate::state::tmc::COMMISSION_IN_FLIGHT.store(false, Relaxed);

            let status = match inner.status {
                crate::autotune::CommissionStatus::Accepted => 0u8,
                crate::autotune::CommissionStatus::AcceptedWithWarnings => 1,
                crate::autotune::CommissionStatus::InnerOnly => 2,
                crate::autotune::CommissionStatus::Failed(e) => e.status_code(),
            };
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

            // Persist outer gains to atomics for closed-loop entry.
            crate::state::tmc::POSITION_P_VALUE.store(outer.position_p, Relaxed);
            crate::state::tmc::POSITION_I_VALUE.store(outer.position_i, Relaxed);
            crate::state::tmc::VELOCITY_P_VALUE.store(outer.velocity_p, Relaxed);
            crate::state::tmc::VELOCITY_I_VALUE.store(outer.velocity_i, Relaxed);
            crate::state::tmc::VELOCITY_LIMIT_CFG.store(outer.velocity_limit, Relaxed);
            crate::state::tmc::VELOCITY_FILTER_HZ.store(outer.velocity_filter_hz, Relaxed);
            crate::state::tmc::POSITION_FILTER_HZ.store(outer.position_filter_hz, Relaxed);
            crate::state::tmc::FLUX_FILTER_HZ.store(outer.flux_filter_hz, Relaxed);
            crate::state::tmc::TORQUE_FILTER_HZ.store(outer.torque_filter_hz, Relaxed);

            // SPI restore — enter position mode with the tuned gains.
            if crate::tmc_ops::restore_position_mode_after_outer(backend, &outer)
                .await
                .is_err()
            {
                defmt::error!("OuterDone: position-mode restore SPI failed");
                crate::tmc_ops::disarm_motor(backend, state).await;
                crate::state::tmc::COMMISSION_IN_FLIGHT.store(false, Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::TuneResult {
                        oid,
                        status: crate::autotune::CommissionError::Spi.status_code(),
                        result: zero_outer_result(),
                    },
                    actions: PostActions {
                        stop_trace: true,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            if crate::tmc_ops::apply_stored_filters_via_backend(backend, pwm_maxcnt)
                .await
                .is_err()
            {
                defmt::error!("OuterDone: stored filter apply SPI failed");
                crate::tmc_ops::disarm_motor(backend, state).await;
                crate::state::tmc::COMMISSION_IN_FLIGHT.store(false, Relaxed);
                return CommissionOutcome::Terminal {
                    reply: ReplyPayload::TuneResult {
                        oid,
                        status: crate::autotune::CommissionError::Spi.status_code(),
                        result: zero_outer_result(),
                    },
                    actions: PostActions {
                        stop_trace: true,
                        stepper: StepperPostAction::Stop,
                    },
                };
            }

            *state.calibrated = true;
            crate::tmc_ops::arm_motor_electrical(backend, state);

            crate::state::tmc::COMMISSION_IN_FLIGHT.store(false, Relaxed);

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
            let consumed = engine.take().expect("engine slot None on OuterFailed");
            let oid = consumed.oid();
            let snapshot = consumed.entry_snapshot();

            let is_safety_fault =
                matches!(error, CE::Spi | CE::Shutdown | CE::SafetyEnvelopeViolation);

            let restore_failure: Option<CE> = if is_safety_fault {
                // Safety faults skip restore — motor state unknown, disarm immediately.
                Some(error)
            } else if let Some(ref snap) = snapshot {
                match crate::commissioning::inner::restore_entry_gains(snap, backend).await {
                    Ok(()) => {
                        // Two-read stability check: wait 10ms, read position, wait 10ms, read again.
                        backend.delay_us(10_000).await;
                        let pos1 = backend
                            .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                            .await;
                        backend.delay_us(10_000).await;
                        let pos2 = backend
                            .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                            .await;
                        match (pos1, pos2) {
                            (Ok(p1), Ok(p2)) => {
                                let p1_signed = p1 as i32;
                                let p2_signed = p2 as i32;
                                let drift = p2_signed.wrapping_sub(p1_signed).unsigned_abs();
                                if drift > 2 {
                                    defmt::error!(
                                        "OuterFailed restore: unstable p1={} p2={} drift={}",
                                        p1_signed,
                                        p2_signed,
                                        drift,
                                    );
                                    // Preserve the upstream error; the drift
                                    // log is the secondary diagnostic. Overriding
                                    // with CurrentValidationFailed would mask
                                    // the meaningful failure (e.g.
                                    // VelocityValidationFailed).
                                    Some(error)
                                } else {
                                    None
                                }
                            }
                            _ => {
                                defmt::error!("OuterFailed restore: stability read SPI failed");
                                Some(CE::Spi)
                            }
                        }
                    }
                    Err(restore_err) => {
                        defmt::error!("OuterFailed: Stage 2 restore failed: {}", restore_err);
                        Some(restore_err)
                    }
                }
            } else {
                // No snapshot available — treat as unrecoverable.
                Some(CE::Spi)
            };

            let (reply_status, actions) = if let Some(fault) = restore_failure {
                let _ = backend
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                crate::tmc_ops::disarm_motor(backend, state).await;
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
                crate::tmc_ops::arm_motor_electrical(backend, state);
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
            };

            crate::state::tmc::COMMISSION_IN_FLIGHT.store(false, Relaxed);

            CommissionOutcome::Terminal {
                reply: ReplyPayload::TuneResult {
                    oid,
                    status: reply_status,
                    result: zero_outer_result(),
                },
                actions,
            }
        }
    }
}
