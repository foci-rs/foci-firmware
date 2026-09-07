//! Poll-driven per-channel terminal restore.
//!
//! [`TerminalRestoreMachine`] is the same terminal dispatch the awaited
//! `drive_commission_result` chain used to run inline, re-expressed as an owned
//! state machine that performs at most one awaited backend operation per
//! [`TerminalRestoreMachine::poll_step`]. The machine holds only owned values;
//! the engine slot, dispatch view, backend, and velocity-integral retention are borrows
//! supplied per poll, so nothing borrowed lives across a tick boundary.
//!
//! Ordering, fault stamping, reply shape, and `engine.take()` policy are
//! reproduced exactly from the awaited chain; the characterization goldens in
//! `characterization_tests.rs` are the oracle.

use core::sync::atomic::Ordering::Relaxed;

use crate::autotune::CommissionError;
use crate::commission_dispatch::{
    CommissionOutcome, CommissionRetention, DispatchState, PostActions, ReplyPayload,
    StepperPostAction, disarm_velocity_workflow, fixed_gain_amplitude,
    outer_restore_position_stable, record_restore_summary, restore, robustness, stage_c,
    validate_outer_gains, zero_adc_cal_result, zero_commission_result, zero_outer_result,
};
use crate::commissioning::CommissioningEngine;
use crate::commissioning::schedule::CommissionTerminal;
use crate::commissioning::types::{
    AdcCalResult, CommissionResult, CommissioningBackend, EngineKind, FixedGainAmplitudeCause,
    FixedGainAmplitudeOutcome, FixedGainAmplitudeTerminal, OuterResult, RobustnessOutcome,
    RobustnessReversalTerminal, StageCCause, StepResult, VelocityIntegralOutcome,
    VelocityIntegralTerminal,
};
use crate::pid::OuterLoopGains;

/// One advance of a terminal restore.
#[allow(clippy::large_enum_variant)]
pub enum RestoreStep {
    /// More backend work remains; poll again on the next tick.
    Pending,
    /// The restore reached its terminal; the engine slot has been consumed.
    Done(CommissionOutcome),
}

/// Terminal payload copied out of the engine step result at `start`.
///
/// The variants differ widely in size because the reply payloads themselves do;
/// boxing them would need an allocator this firmware does not have.
#[allow(clippy::large_enum_variant)]
#[derive(Clone)]
enum TerminalPayload {
    Selftest,
    InnerFailed(CommissionError),
    CalibrateDone(AdcCalResult),
    CommissionDone(CommissionResult),
    OuterDone(OuterResult),
    OuterFailed,
    VelocityIntegral(VelocityIntegralTerminal),
    FixedGainAmplitude(FixedGainAmplitudeTerminal),
    RobustnessReversal(RobustnessReversalTerminal),
}

/// Owned cross-tick terminal state: the reply payload under construction plus
/// every intermediate result the remaining steps still need.
struct OwnedTerminalData {
    payload: TerminalPayload,
    /// Status this restore's `cancelled_reply` reports once it reaches
    /// `ForceSafeDisarm`. Defaults to `Shutdown`; `set_status_error` and
    /// `start_cancelled` are the only ways to override it.
    status_error: CommissionError,
    outer_error: CommissionError,
    outer_restore_failure: Option<CommissionError>,
    restore_drift_counts: u32,
    is_safety_fault: bool,
    velocity_workflow_restored: bool,
    baseline_restored: bool,
    use_handled_stop_take: bool,
    reproduction: Option<stage_c::StageCReproductionReport>,
    authority: Option<
        crate::commissioning::outer::velocity::integral_reproduction::StageCAuthoritySummary,
    >,
}

impl OwnedTerminalData {
    fn new(payload: TerminalPayload) -> Self {
        Self {
            payload,
            status_error: CommissionError::Shutdown,
            outer_error: CommissionError::Spi,
            outer_restore_failure: None,
            restore_drift_counts: 0,
            is_safety_fault: false,
            velocity_workflow_restored: true,
            baseline_restored: false,
            use_handled_stop_take: false,
            reproduction: None,
            authority: None,
        }
    }
}

/// Which awaited step runs next.
#[derive(Clone, Copy)]
enum RestoreCursor {
    SelftestRestoreGains,

    InnerFailedSafeState,
    InnerFailedRestoreGains,

    CalibrateRestoreGains,
    CalibrateApplyFilters,
    CalibrateArm,
    CalibrateDisarm(u8),

    CommissionClassify,
    CommissionWriteCurrentLimit(OuterLoopGains),
    CommissionArm(OuterLoopGains),
    CommissionDisarm(u8),

    OuterDoneRestorePosition,
    OuterDoneApplyFilters(OuterLoopGains),
    OuterDoneArm(OuterLoopGains),
    /// `keep_result` reproduces the arm-failure reply, which reports the tuned
    /// result rather than the zero-filled one every earlier failure uses.
    OuterDoneDisarm {
        status: u8,
        keep_result: bool,
    },

    OuterFailedStagedTiming,
    OuterFailedDisarmVelocity,
    OuterFailedClassify,
    OuterFailedRestoreEntryGains,
    OuterFailedSafetyRestore,
    OuterFailedStability,
    OuterFailedModeRamp,
    OuterFailedFaultDisarm,
    OuterFailedArm,
    OuterFailedArmDisarm(u8),

    FixedGainAmplitudeStagedDisarmVelocity,
    FixedGainAmplitudeDisarm,

    RobustnessStagedClassify,
    RobustnessRestoreBaseline,
    RobustnessDisarm,

    VelocityStagedPrologue,
    VelocityStagedRestoreBaseline,
    VelocityStagedDisarmVelocity,
    VelocityDisarmVelocity,

    ForceSafeDisarm,
    Finished,
}

/// Owned, borrow-free, per-channel terminal restore driven one backend
/// operation at a time.
pub struct TerminalRestoreMachine {
    channel: u8,
    pwm_maxcnt: u16,
    cursor: RestoreCursor,
    terminal: OwnedTerminalData,
    engine_consumed: bool,
}

impl TerminalRestoreMachine {
    /// Begin an unstaged terminal restore for one `StepResult`.
    ///
    /// # Panics
    ///
    /// Panics when `result` is not one of the terminal `StepResult` variants;
    /// non-terminal results are handled by the caller without a restore.
    pub fn start(result: StepResult, channel: u8, pwm_maxcnt: u16) -> Self {
        let (payload, cursor) = match result {
            StepResult::SelftestDone => (
                TerminalPayload::Selftest,
                RestoreCursor::SelftestRestoreGains,
            ),
            StepResult::InnerFailed(error) => (
                TerminalPayload::InnerFailed(error),
                RestoreCursor::InnerFailedSafeState,
            ),
            StepResult::CalibrateDone(result) => (
                TerminalPayload::CalibrateDone(result),
                RestoreCursor::CalibrateRestoreGains,
            ),
            StepResult::CommissionDone(result) => (
                TerminalPayload::CommissionDone(result),
                RestoreCursor::CommissionClassify,
            ),
            StepResult::OuterDone(result) => (
                TerminalPayload::OuterDone(result),
                RestoreCursor::OuterDoneRestorePosition,
            ),
            StepResult::OuterFailed(error) => {
                let mut data = OwnedTerminalData::new(TerminalPayload::OuterFailed);
                data.outer_error = error;
                return Self {
                    channel,
                    pwm_maxcnt,
                    cursor: RestoreCursor::OuterFailedDisarmVelocity,
                    terminal: data,
                    engine_consumed: false,
                };
            }
            StepResult::VelocityIntegralTerminal(terminal) => (
                TerminalPayload::VelocityIntegral(terminal),
                RestoreCursor::VelocityDisarmVelocity,
            ),
            StepResult::FixedGainAmplitudeTerminal(terminal) => (
                TerminalPayload::FixedGainAmplitude(terminal),
                RestoreCursor::FixedGainAmplitudeDisarm,
            ),
            StepResult::RobustnessReversalTerminal(terminal) => (
                TerminalPayload::RobustnessReversal(terminal),
                RestoreCursor::RobustnessRestoreBaseline,
            ),
            StepResult::ReadyNow
            | StepResult::At(_)
            | StepResult::PhaseChanged(_)
            | StepResult::VelocityIntegralEvent(_)
            | StepResult::FixedGainAmplitudeEvent(_)
            | StepResult::BreakawayCampaignEvent(_)
            | StepResult::BreakawayStageCPlanReady { .. } => {
                unreachable!("nonterminal step result cannot start a terminal restore")
            }
        };
        Self {
            channel,
            pwm_maxcnt,
            cursor,
            terminal: OwnedTerminalData::new(payload),
            engine_consumed: false,
        }
    }

    /// Begin a staged terminal restore, applying the retained-evidence policy
    /// the velocity-integral, fixed-gain-amplitude, and robustness terminals carry.
    pub fn start_staged(result: CommissionTerminal, channel: u8, pwm_maxcnt: u16) -> Self {
        match result {
            CommissionTerminal::VelocityIntegral(terminal) => Self {
                channel,
                pwm_maxcnt,
                cursor: RestoreCursor::VelocityStagedPrologue,
                terminal: OwnedTerminalData::new(TerminalPayload::VelocityIntegral(terminal)),
                engine_consumed: false,
            },
            CommissionTerminal::FixedGainAmplitude(terminal) => Self {
                channel,
                pwm_maxcnt,
                cursor: RestoreCursor::FixedGainAmplitudeStagedDisarmVelocity,
                terminal: OwnedTerminalData::new(TerminalPayload::FixedGainAmplitude(terminal)),
                engine_consumed: false,
            },
            CommissionTerminal::RobustnessReversal(terminal) => Self {
                channel,
                pwm_maxcnt,
                cursor: RestoreCursor::RobustnessStagedClassify,
                terminal: OwnedTerminalData::new(TerminalPayload::RobustnessReversal(terminal)),
                engine_consumed: false,
            },
            other => {
                let mut machine = Self::start(other.into_step_result(), channel, pwm_maxcnt);
                // Only the staged entry emits the outer safety-timing trace
                // record, matching the awaited chain this replaced.
                if matches!(machine.cursor, RestoreCursor::OuterFailedDisarmVelocity) {
                    machine.cursor = RestoreCursor::OuterFailedStagedTiming;
                }
                machine
            }
        }
    }

    /// Begin a synthetic terminal restore for a channel whose commissioning
    /// engine is still actively stepping (no natural `StepResult` terminal
    /// reached yet, so no restore machine exists). Builds a placeholder
    /// payload for the engine's kind, matching the zero-filled failure-path
    /// results `zero_outer_result()`/`zero_adc_cal_result()` already use, and
    /// jumps straight to `ForceSafeDisarm` — this restore never had real
    /// register state to restore, only the motor to de-energize.
    pub fn start_cancelled(engine_kind: EngineKind, channel: u8, pwm_maxcnt: u16) -> Self {
        let payload = match engine_kind {
            EngineKind::Commission => TerminalPayload::CommissionDone(zero_commission_result()),
            EngineKind::Calibrate => TerminalPayload::CalibrateDone(zero_adc_cal_result()),
            EngineKind::Selftest => TerminalPayload::Selftest,
            EngineKind::OuterTune => TerminalPayload::OuterFailed,
        };
        let mut terminal = OwnedTerminalData::new(payload);
        terminal.status_error = CommissionError::Cancelled;
        Self {
            channel,
            pwm_maxcnt,
            cursor: RestoreCursor::ForceSafeDisarm,
            terminal,
            engine_consumed: false,
        }
    }

    /// Override the status this restore's `cancelled_reply` will report,
    /// once it reaches `ForceSafeDisarm`. Existing callers that never call
    /// this keep reporting `CommissionError::Shutdown`, the pre-existing
    /// default `OwnedTerminalData::new` sets.
    pub(crate) fn set_status_error(&mut self, status_error: CommissionError) {
        self.terminal.status_error = status_error;
    }

    /// Return this machine's board-local TMC channel.
    pub const fn channel(&self) -> u8 {
        self.channel
    }

    /// Return whether the restore still has work left.
    pub const fn is_pending(&self) -> bool {
        !matches!(self.cursor, RestoreCursor::Finished)
    }

    /// Advance the restore by at most one awaited backend operation.
    ///
    /// Returns [`RestoreStep::Done`] once the terminal outcome is ready; the
    /// engine slot is consumed at that point and not before, so an interrupted
    /// teardown never leaves cleared bookkeeping behind a half-restored motor.
    ///
    /// # Panics
    ///
    /// Panics when polled after it already reported a terminal, or when a
    /// staged terminal is polled without velocity-integral retention.
    pub async fn poll_step<B: CommissioningBackend>(
        &mut self,
        engine: &mut Option<CommissioningEngine>,
        backend: &mut B,
        state: &mut DispatchState<'_>,
        retention: Option<&mut CommissionRetention<'_>>,
    ) -> RestoreStep {
        assert_eq!(
            self.channel, state.channel,
            "FATAL: terminal restore polled with another channel's dispatch state"
        );
        let mut retention = retention;
        loop {
            match self.cursor {
                RestoreCursor::Finished => {
                    unreachable!("terminal restore polled after it reported its terminal")
                }

                // --- selftest ------------------------------------------------
                RestoreCursor::SelftestRestoreGains => {
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    if restore::restore_calibrate_gains(backend, state.tmc, *state.stored_gains)
                        .await
                        .is_err()
                    {
                        defmt::error!("SelftestDone: saved gain restore SPI failed");
                    }
                    let mut consumed = self.take_engine(engine);
                    let oid = consumed.oid();
                    let results = consumed.take_selftest_results();
                    let diagnostics = consumed.take_diagnostics();
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::Selftest {
                            oid,
                            results,
                            diagnostics,
                            done_status: 0,
                        },
                        actions: PostActions::NONE,
                    });
                }

                // --- inner-failed --------------------------------------------
                RestoreCursor::InnerFailedSafeState => {
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
                    self.cursor = RestoreCursor::InnerFailedRestoreGains;
                    return RestoreStep::Pending;
                }
                RestoreCursor::InnerFailedRestoreGains => {
                    let kind = self.engine_kind(engine);
                    if matches!(kind, EngineKind::Selftest | EngineKind::Calibrate)
                        && restore::restore_calibrate_gains(backend, state.tmc, *state.stored_gains)
                            .await
                            .is_err()
                    {
                        defmt::error!("InnerFailed: saved gain restore SPI failed");
                    }
                    let TerminalPayload::InnerFailed(error) = self.terminal.payload.clone() else {
                        unreachable!("inner-failed cursor without an inner-failed payload")
                    };
                    let mut consumed = self.take_engine(engine);
                    let oid = consumed.oid();
                    let selftest_results = consumed.take_selftest_results();
                    let diagnostics = consumed.take_diagnostics();
                    let reply = match kind {
                        EngineKind::Calibrate => ReplyPayload::CalibrateResult {
                            oid,
                            status: super::calibration_status_from_commission_error(error),
                            result: zero_adc_cal_result(),
                            diagnostics,
                        },
                        EngineKind::Selftest => ReplyPayload::Selftest {
                            oid,
                            results: selftest_results,
                            diagnostics,
                            done_status: error.status_code(),
                        },
                        EngineKind::Commission | EngineKind::OuterTune => {
                            ReplyPayload::CommissionPhase {
                                oid,
                                phase: 0,
                                status: error.status_code(),
                                diagnostics,
                            }
                        }
                    };
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply,
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::None,
                        },
                    });
                }

                // --- calibrate -----------------------------------------------
                RestoreCursor::CalibrateRestoreGains => {
                    state.tmc.motor_armed.store(false, Relaxed);
                    match restore::restore_calibrate_gains(backend, state.tmc, *state.stored_gains)
                        .await
                    {
                        Ok(()) => {
                            self.cursor = RestoreCursor::CalibrateApplyFilters;
                        }
                        Err(restore_error) => {
                            defmt::error!("CalibrateDone: saved gain restore SPI failed");
                            let status = match restore_error {
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
                            };
                            self.cursor = RestoreCursor::CalibrateDisarm(status);
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CalibrateApplyFilters => {
                    if crate::tmc_control::filters::apply_stored_filters_via_backend(
                        backend,
                        state.tmc,
                        self.pwm_maxcnt,
                    )
                    .await
                    .is_err()
                    {
                        defmt::error!("CalibrateDone: stored filter restore SPI failed");
                        self.cursor = RestoreCursor::CalibrateDisarm(
                            crate::calibration::CalibrationError::SpiError.status_code(),
                        );
                    } else {
                        self.cursor = RestoreCursor::CalibrateArm;
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CalibrateArm => {
                    *state.calibrated = true;
                    match crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
                    {
                        Ok(()) => {
                            let TerminalPayload::CalibrateDone(cal_result) =
                                self.terminal.payload.clone()
                            else {
                                unreachable!("calibrate cursor without a calibrate payload")
                            };
                            state.tmc.commission_in_flight.store(false, Relaxed);
                            let consumed = self.take_engine(engine);
                            return RestoreStep::Done(CommissionOutcome::Terminal {
                                reply: ReplyPayload::CalibrateResult {
                                    oid: consumed.oid(),
                                    status: 0,
                                    result: cal_result,
                                    diagnostics: crate::selftest::CommissionDiagnostics::default(),
                                },
                                actions: PostActions {
                                    stop_trace: false,
                                    stepper: StepperPostAction::ArmPreservePosition,
                                },
                            });
                        }
                        Err(error) => {
                            defmt::error!("CalibrateDone: arm sentinel verify failed: {}", error);
                            let status = match error {
                                CommissionError::ChipResetDetected => {
                                    crate::calibration::CalibrationError::ChipResetDetected
                                        .status_code()
                                }
                                _ => crate::calibration::CalibrationError::SpiError.status_code(),
                            };
                            self.cursor = RestoreCursor::CalibrateDisarm(status);
                            return RestoreStep::Pending;
                        }
                    }
                }
                RestoreCursor::CalibrateDisarm(status) => {
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let consumed = self.take_engine(engine);
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::CalibrateResult {
                            oid: consumed.oid(),
                            status,
                            result: zero_adc_cal_result(),
                            diagnostics: crate::selftest::CommissionDiagnostics::default(),
                        },
                        actions: PostActions {
                            stop_trace: false,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }

                RestoreCursor::CommissionClassify => {
                    let TerminalPayload::CommissionDone(commission_result) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("commission cursor without a commission payload")
                    };
                    let status = commission_status_code(commission_result.inner.status);
                    if matches!(
                        commission_result.inner.status,
                        crate::autotune::CommissionStatus::Failed(_)
                    ) {
                        state.tmc.motor_armed.store(false, Relaxed);
                        *state.calibrated = false;
                        *state.motor_enabled = false;
                        state.tmc.commission_in_flight.store(false, Relaxed);
                        let consumed = self.take_engine(engine);
                        let oid = consumed.oid();
                        defmt::error!("foci_commission failed: oid={} status={}", oid, status);
                        return RestoreStep::Done(CommissionOutcome::Terminal {
                            reply: ReplyPayload::CommissionResult {
                                oid,
                                status,
                                result: commission_result,
                            },
                            actions: PostActions {
                                stop_trace: true,
                                stepper: StepperPostAction::Stop,
                            },
                        });
                    }

                    match validate_outer_gains(
                        commission_result.fallback_position_p,
                        commission_result.fallback_position_i,
                        commission_result.fallback_velocity_p,
                        commission_result.fallback_velocity_i,
                    ) {
                        Ok(gains) => {
                            self.cursor = RestoreCursor::CommissionWriteCurrentLimit(gains);
                        }
                        Err(_) => {
                            self.cursor = RestoreCursor::CommissionDisarm(
                                CommissionError::CurrentValidationFailed.status_code(),
                            );
                        }
                    }
                }
                RestoreCursor::CommissionWriteCurrentLimit(gains) => {
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
                        self.cursor =
                            RestoreCursor::CommissionDisarm(CommissionError::Spi.status_code());
                    } else {
                        self.cursor = RestoreCursor::CommissionArm(gains);
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CommissionArm(fallback_outer_gains) => {
                    *state.calibrated = true;
                    match crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
                    {
                        Err(error) => {
                            defmt::error!("CommissionDone: arm sentinel verify failed: {}", error);
                            self.cursor = RestoreCursor::CommissionDisarm(error.status_code());
                            return RestoreStep::Pending;
                        }
                        Ok(()) => {
                            let TerminalPayload::CommissionDone(commission_result) =
                                self.terminal.payload.clone()
                            else {
                                unreachable!("commission cursor without a commission payload")
                            };
                            let inner = &commission_result.inner;
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

                            let status = commission_status_code(inner.status);
                            let consumed = self.take_engine(engine);
                            let oid = consumed.oid();
                            defmt::info!("foci_commission complete: oid={} status={}", oid, status);
                            return RestoreStep::Done(CommissionOutcome::Terminal {
                                reply: ReplyPayload::CommissionResult {
                                    oid,
                                    status,
                                    result: commission_result,
                                },
                                actions: PostActions {
                                    stop_trace: true,
                                    stepper: StepperPostAction::ArmZeroPosition,
                                },
                            });
                        }
                    }
                }
                RestoreCursor::CommissionDisarm(status) => {
                    let TerminalPayload::CommissionDone(commission_result) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("commission cursor without a commission payload")
                    };
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let consumed = self.take_engine(engine);
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::CommissionResult {
                            oid: consumed.oid(),
                            status,
                            result: commission_result,
                        },
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }

                RestoreCursor::OuterDoneRestorePosition => {
                    let TerminalPayload::OuterDone(outer) = self.terminal.payload.clone() else {
                        unreachable!("outer-done cursor without an outer-done payload")
                    };
                    state.tmc.motor_armed.store(false, Relaxed);
                    match restore::restore_position_mode_after_outer(backend, state.tmc, &outer)
                        .await
                    {
                        Ok(gains) => {
                            self.cursor = RestoreCursor::OuterDoneApplyFilters(gains);
                        }
                        Err(error) => {
                            let status = match error {
                                restore::RestoreError::Spi(_)
                                | restore::RestoreError::VelocityClamp
                                | restore::RestoreError::Readback => CommissionError::Spi,
                                restore::RestoreError::PiRepresentation => {
                                    CommissionError::ChipResetDetected
                                }
                                restore::RestoreError::InvalidGains => {
                                    CommissionError::VelocityValidationFailed
                                }
                            };
                            defmt::error!("OuterDone: position-mode restore failed");
                            self.cursor = RestoreCursor::OuterDoneDisarm {
                                status: status.status_code(),
                                keep_result: false,
                            };
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterDoneApplyFilters(gains) => {
                    let TerminalPayload::OuterDone(outer) = self.terminal.payload.clone() else {
                        unreachable!("outer-done cursor without an outer-done payload")
                    };
                    if crate::tmc_control::filters::apply_outer_result_filters_via_backend(
                        backend,
                        &outer,
                        self.pwm_maxcnt,
                    )
                    .await
                    .is_err()
                    {
                        defmt::error!("OuterDone: candidate filter apply SPI failed");
                        self.cursor = RestoreCursor::OuterDoneDisarm {
                            status: CommissionError::Spi.status_code(),
                            keep_result: false,
                        };
                    } else {
                        self.cursor = RestoreCursor::OuterDoneArm(gains);
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterDoneArm(restored_gains) => {
                    *state.calibrated = true;
                    match crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
                    {
                        Err(error) => {
                            defmt::error!("OuterDone: arm sentinel verify failed: {}", error);
                            self.cursor = RestoreCursor::OuterDoneDisarm {
                                status: error.status_code(),
                                keep_result: true,
                            };
                            return RestoreStep::Pending;
                        }
                        Ok(()) => {
                            let TerminalPayload::OuterDone(outer) = self.terminal.payload.clone()
                            else {
                                unreachable!("outer-done cursor without an outer-done payload")
                            };
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
                            let status = commission_status_code(outer.status);
                            let consumed = self.take_engine(engine);
                            return RestoreStep::Done(CommissionOutcome::Terminal {
                                reply: ReplyPayload::TuneResult {
                                    oid: consumed.oid(),
                                    status,
                                    result: outer,
                                    safety_fault: None,
                                },
                                actions: PostActions {
                                    stop_trace: true,
                                    stepper: StepperPostAction::ArmPreservePosition,
                                },
                            });
                        }
                    }
                }
                RestoreCursor::OuterDoneDisarm {
                    status,
                    keep_result,
                } => {
                    let TerminalPayload::OuterDone(outer) = self.terminal.payload.clone() else {
                        unreachable!("outer-done cursor without an outer-done payload")
                    };
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let consumed = self.take_engine(engine);
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::TuneResult {
                            oid: consumed.oid(),
                            status,
                            result: if keep_result {
                                outer
                            } else {
                                zero_outer_result()
                            },
                            safety_fault: None,
                        },
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }

                // --- outer failed ---------------------------------------------
                RestoreCursor::OuterFailedStagedTiming => {
                    #[cfg(feature = "trace")]
                    if let Some(run_sequence) = engine
                        .as_ref()
                        .and_then(CommissioningEngine::trace_velocity_run_sequence)
                    {
                        engine
                            .as_mut()
                            .expect("trace velocity engine disappeared before terminal timing")
                            .emit_outer_safety_timing_trace(backend, run_sequence);
                    }
                    self.cursor = RestoreCursor::OuterFailedDisarmVelocity;
                }
                RestoreCursor::OuterFailedDisarmVelocity => {
                    if engine
                        .as_ref()
                        .and_then(CommissioningEngine::velocity_operational_torque_limit)
                        .is_some()
                    {
                        self.terminal.velocity_workflow_restored =
                            disarm_velocity_workflow(engine, backend, state).await;
                        self.cursor = RestoreCursor::OuterFailedClassify;
                        return RestoreStep::Pending;
                    }
                    self.terminal.velocity_workflow_restored = true;
                    self.cursor = RestoreCursor::OuterFailedClassify;
                }
                RestoreCursor::OuterFailedClassify => {
                    if !self.terminal.velocity_workflow_restored {
                        self.terminal.outer_error = CommissionError::Spi;
                    }
                    let error = self.terminal.outer_error;
                    self.terminal.is_safety_fault = matches!(
                        error,
                        CommissionError::Spi
                            | CommissionError::Shutdown
                            | CommissionError::SafetyEnvelopeViolation
                    );
                    if self.terminal.is_safety_fault {
                        self.terminal.outer_restore_failure = Some(error);
                        // A SafetyEnvelopeViolation is a motion-envelope trip on a
                        // healthy chip: still reload the operational gains/limits so
                        // the next enable/homing does not inherit the tune's
                        // mid-ladder gains (velocity I=0). Spi/Shutdown cannot
                        // reliably write, so they keep the bare disarm.
                        if matches!(error, CommissionError::SafetyEnvelopeViolation)
                            && self.entry_snapshot(engine).is_some()
                        {
                            self.cursor = RestoreCursor::OuterFailedSafetyRestore;
                        } else {
                            self.cursor = RestoreCursor::OuterFailedModeRamp;
                        }
                    } else if self.entry_snapshot(engine).is_some() {
                        self.cursor = RestoreCursor::OuterFailedRestoreEntryGains;
                    } else {
                        self.terminal.outer_restore_failure = Some(CommissionError::Spi);
                        self.cursor = RestoreCursor::OuterFailedModeRamp;
                    }
                }
                RestoreCursor::OuterFailedRestoreEntryGains => {
                    let snapshot = self
                        .entry_snapshot(engine)
                        .expect("entry snapshot disappeared during outer-failed restore");
                    match crate::commissioning::inner::restore_entry_gains(
                        &snapshot, backend, state.tmc,
                    )
                    .await
                    {
                        Ok(()) => {
                            self.cursor = RestoreCursor::OuterFailedStability;
                        }
                        Err(restore_err) => {
                            defmt::error!(
                                "OuterFailed: installed-tuning restore failed: {}",
                                restore_err
                            );
                            self.terminal.outer_restore_failure = Some(restore_err);
                            self.cursor = RestoreCursor::OuterFailedModeRamp;
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedSafetyRestore => {
                    let snapshot = self
                        .entry_snapshot(engine)
                        .expect("entry snapshot disappeared during safety-fault restore");
                    if let Err(restore_err) = crate::commissioning::inner::restore_entry_gains(
                        &snapshot, backend, state.tmc,
                    )
                    .await
                    {
                        defmt::error!(
                            "OuterFailed: safety-fault gain restore failed: {}",
                            restore_err
                        );
                    }
                    self.cursor = RestoreCursor::OuterFailedModeRamp;
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedStability => {
                    match outer_restore_position_stable(backend).await {
                        Ok(stability) => {
                            self.terminal.restore_drift_counts = stability.span_counts;
                            if stability.stable {
                                self.terminal.outer_restore_failure = None;
                                self.cursor = RestoreCursor::OuterFailedArm;
                            } else {
                                self.terminal.outer_restore_failure =
                                    Some(self.terminal.outer_error);
                                self.cursor = RestoreCursor::OuterFailedModeRamp;
                            }
                        }
                        Err(_) => {
                            defmt::error!("OuterFailed restore: stability read SPI failed");
                            self.terminal.outer_restore_failure = Some(CommissionError::Spi);
                            self.cursor = RestoreCursor::OuterFailedModeRamp;
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedModeRamp => {
                    let _ = backend
                        .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                        .await;
                    self.cursor = RestoreCursor::OuterFailedFaultDisarm;
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedFaultDisarm => {
                    let fault = self
                        .terminal
                        .outer_restore_failure
                        .expect("outer-failed fault disarm without a restore failure");
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    defmt::error!("OuterFailed: safety fault ({}), motor disabled", fault);
                    return RestoreStep::Done(self.finish_outer_failed(
                        engine,
                        backend,
                        state,
                        fault.status_code(),
                        PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    ));
                }
                RestoreCursor::OuterFailedArm => {
                    *state.calibrated = true;
                    match crate::tmc_control::lifecycle::arm_motor_electrical(backend, state).await
                    {
                        Err(arm_error) => {
                            self.cursor =
                                RestoreCursor::OuterFailedArmDisarm(arm_error.status_code());
                            defmt::error!(
                                "OuterFailed: arm sentinel verify failed after soft failure: {}",
                                arm_error
                            );
                            return RestoreStep::Pending;
                        }
                        Ok(()) => {
                            let error = self.terminal.outer_error;
                            defmt::warn!(
                                "OuterFailed: soft failure ({}), entry gains restored",
                                error
                            );
                            return RestoreStep::Done(self.finish_outer_failed(
                                engine,
                                backend,
                                state,
                                error.status_code(),
                                PostActions {
                                    stop_trace: true,
                                    stepper: StepperPostAction::ArmPreservePosition,
                                },
                            ));
                        }
                    }
                }
                RestoreCursor::OuterFailedArmDisarm(status) => {
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    return RestoreStep::Done(self.finish_outer_failed(
                        engine,
                        backend,
                        state,
                        status,
                        PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    ));
                }

                // --- fixed-gain amplitude ----------------------------------------
                RestoreCursor::FixedGainAmplitudeStagedDisarmVelocity => {
                    let TerminalPayload::FixedGainAmplitude(mut terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!(
                            "fixed-gain amplitude cursor without a fixed-gain amplitude payload"
                        )
                    };
                    let retention = retention
                        .take()
                        .expect("staged terminal restore requires Stage-C retention");
                    let amplitude_context = engine.as_ref().and_then(|engine| {
                        engine
                            .fixed_gain_amplitude_order()
                            .zip(engine.fixed_gain_amplitude_plan_digest())
                    });
                    if !disarm_velocity_workflow(engine, backend, state).await {
                        retention.stage_c.observe_fault();
                        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
                        terminal.cause = FixedGainAmplitudeCause::EvidenceIntegrity;
                        terminal.safety_fault = None;
                    } else if let Some((order, plan_digest)) = amplitude_context {
                        terminal = fixed_gain_amplitude::apply_fixed_gain_amplitude_terminal(
                            terminal,
                            order,
                            plan_digest,
                            retention.stage_c,
                        );
                    } else {
                        retention.stage_c.observe_fault();
                        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
                        terminal.cause = FixedGainAmplitudeCause::EvidenceIntegrity;
                        terminal.safety_fault = None;
                    }
                    self.terminal.payload = TerminalPayload::FixedGainAmplitude(terminal);
                    #[cfg(feature = "trace")]
                    engine
                        .as_mut()
                        .expect("engine slot is None on FixedGainAmplitudeTerminal")
                        .emit_fixed_gain_amplitude_terminal_trace(backend, terminal);
                    self.cursor = RestoreCursor::FixedGainAmplitudeDisarm;
                    return RestoreStep::Pending;
                }
                RestoreCursor::FixedGainAmplitudeDisarm => {
                    let TerminalPayload::FixedGainAmplitude(terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!(
                            "fixed-gain amplitude cursor without a fixed-gain amplitude payload"
                        )
                    };
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let consumed = self.take_engine(engine);
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::FixedGainAmplitudeTerminal {
                            oid: consumed.oid(),
                            terminal,
                        },
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }

                // --- robustness reversal --------------------------------------
                RestoreCursor::RobustnessStagedClassify => {
                    let TerminalPayload::RobustnessReversal(terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("robustness cursor without a robustness payload")
                    };
                    let retention = retention
                        .take()
                        .expect("staged terminal restore requires Stage-C retention");
                    let mut terminal =
                        robustness::apply_robustness_terminal(terminal, retention.stage_c);
                    let deploy = (terminal.outcome == RobustnessOutcome::Complete)
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
                            self.terminal.payload = TerminalPayload::OuterDone(result);
                            self.cursor = RestoreCursor::OuterDoneRestorePosition;
                        }
                        None => {
                            if terminal.outcome == RobustnessOutcome::Complete {
                                terminal.outcome = RobustnessOutcome::Failed;
                                terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
                            }
                            self.terminal.payload = TerminalPayload::RobustnessReversal(terminal);
                            self.cursor = RestoreCursor::RobustnessRestoreBaseline;
                        }
                    }
                }
                RestoreCursor::RobustnessRestoreBaseline => {
                    let TerminalPayload::RobustnessReversal(mut terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("robustness cursor without a robustness payload")
                    };
                    if terminal.outcome == RobustnessOutcome::Complete {
                        self.cursor = RestoreCursor::RobustnessDisarm;
                        continue;
                    }
                    let restored = match self.entry_snapshot(engine) {
                        Some(snapshot) => {
                            match restore::restore_entry_snapshot_to_registers(backend, &snapshot)
                                .await
                            {
                                Ok(()) => true,
                                Err(err) => {
                                    let variant = match err {
                                        restore::RestoreError::Spi(_) => "spi",
                                        restore::RestoreError::InvalidGains => "invalid_gains",
                                        restore::RestoreError::PiRepresentation => {
                                            "pi_representation"
                                        }
                                        restore::RestoreError::VelocityClamp => "velocity_clamp",
                                        restore::RestoreError::Readback => "readback",
                                    };
                                    defmt::error!(
                                        "robustness terminal: baseline restore failed: {}",
                                        variant
                                    );
                                    false
                                }
                            }
                        }
                        None => {
                            defmt::error!(
                                "robustness terminal: no entry snapshot to restore baseline from"
                            );
                            false
                        }
                    };
                    if !restored {
                        terminal.outcome = RobustnessOutcome::Failed;
                        terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
                    }
                    self.terminal.payload = TerminalPayload::RobustnessReversal(terminal);
                    self.cursor = RestoreCursor::RobustnessDisarm;
                    return RestoreStep::Pending;
                }
                RestoreCursor::RobustnessDisarm => {
                    let TerminalPayload::RobustnessReversal(terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("robustness cursor without a robustness payload")
                    };
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let consumed = self.take_engine(engine);
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::RobustnessReversalTerminal {
                            oid: consumed.oid(),
                            terminal,
                        },
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }

                RestoreCursor::VelocityStagedPrologue => {
                    let retention = retention
                        .take()
                        .expect("staged terminal restore requires Stage-C retention");
                    self.velocity_staged_prologue(engine, backend, retention);
                    if self.entry_snapshot(engine).is_some() {
                        self.cursor = RestoreCursor::VelocityStagedRestoreBaseline;
                    } else {
                        self.terminal.baseline_restored = false;
                        self.cursor = RestoreCursor::VelocityStagedDisarmVelocity;
                    }
                }
                RestoreCursor::VelocityStagedRestoreBaseline => {
                    let snapshot = self
                        .entry_snapshot(engine)
                        .expect("entry snapshot disappeared during Stage-C restore");
                    self.terminal.baseline_restored =
                        restore::restore_entry_snapshot_to_registers(backend, &snapshot)
                            .await
                            .is_ok();
                    self.cursor = RestoreCursor::VelocityStagedDisarmVelocity;
                    return RestoreStep::Pending;
                }
                RestoreCursor::VelocityStagedDisarmVelocity => {
                    let TerminalPayload::VelocityIntegral(mut terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("velocity cursor without a velocity payload")
                    };
                    let retention = retention
                        .take()
                        .expect("staged terminal restore requires Stage-C retention");
                    if !disarm_velocity_workflow(engine, backend, state).await
                        || !self.terminal.baseline_restored
                    {
                        retention.stage_c.observe_fault();
                        terminal.outcome = VelocityIntegralOutcome::Fault;
                        terminal.cause = StageCCause::Error(CommissionError::Spi.status_code());
                        terminal.recovery_unavailable = true;
                        terminal.safety_fault = None;
                        self.terminal.reproduction = None;
                        self.terminal.authority = None;
                    }
                    self.terminal.payload = TerminalPayload::VelocityIntegral(terminal);
                    #[cfg(feature = "trace")]
                    engine
                        .as_mut()
                        .expect("engine slot is None on VelocityIntegralTerminal")
                        .emit_stage_c_terminal_trace(
                            backend,
                            &terminal,
                            self.terminal.reproduction,
                        );
                    self.cursor = RestoreCursor::VelocityDisarmVelocity;
                    return RestoreStep::Pending;
                }
                RestoreCursor::VelocityDisarmVelocity => {
                    let TerminalPayload::VelocityIntegral(mut terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("velocity cursor without a velocity payload")
                    };
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
                        terminal.outcome = VelocityIntegralOutcome::Fault;
                        terminal.cause = StageCCause::Error(CommissionError::Spi.status_code());
                        terminal.recovery_unavailable = true;
                        terminal.safety_fault = None;
                    }
                    self.terminal.payload = TerminalPayload::VelocityIntegral(terminal);
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let consumed = self.take_engine(engine);
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::VelocityIntegralTerminal {
                            oid: consumed.oid(),
                            terminal,
                            reproduction: self.terminal.reproduction,
                            authority: self.terminal.authority,
                        },
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }

                // --- cancellation ----------------------------------------------
                RestoreCursor::ForceSafeDisarm => {
                    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let status_error = self.terminal.status_error;
                    let consumed = self.take_engine(engine);
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: self.cancelled_reply(consumed.oid(), status_error),
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }
            }
        }
    }

    /// Run the remaining mandatory disarm/safe-state steps and consume the
    /// machine, discarding whatever restore progress was in flight.
    ///
    /// This is the cancellation entry: it always de-energizes through
    /// [`crate::tmc_control::lifecycle::disarm_motor`], whatever cursor the
    /// restore had reached, so a torn-down restore can never leave the motor
    /// partly armed.
    pub async fn force_safe_terminal<B: CommissioningBackend>(
        &mut self,
        engine: &mut Option<CommissioningEngine>,
        backend: &mut B,
        state: &mut DispatchState<'_>,
    ) -> CommissionOutcome {
        self.cursor = RestoreCursor::ForceSafeDisarm;
        match self.poll_step(engine, backend, state, None).await {
            RestoreStep::Done(outcome) => outcome,
            RestoreStep::Pending => {
                unreachable!("forced safe terminal must complete in one step")
            }
        }
    }

    fn engine_kind(&self, engine: &Option<CommissioningEngine>) -> EngineKind {
        engine
            .as_ref()
            .expect("engine slot None during terminal restore")
            .kind()
    }

    fn entry_snapshot(
        &self,
        engine: &Option<CommissioningEngine>,
    ) -> Option<crate::commissioning::OuterGainRollbackSnapshot> {
        engine
            .as_ref()
            .and_then(CommissioningEngine::entry_snapshot)
    }

    fn take_engine(&mut self, engine: &mut Option<CommissioningEngine>) -> CommissioningEngine {
        self.engine_consumed = true;
        self.cursor = RestoreCursor::Finished;
        let consumed = if self.terminal.use_handled_stop_take {
            CommissioningEngine::take_after_handled_stop(engine)
        } else {
            engine.take()
        };
        consumed.expect("engine slot None at terminal restore completion")
    }

    fn finish_outer_failed<B: CommissioningBackend>(
        &mut self,
        engine: &mut Option<CommissioningEngine>,
        backend: &mut B,
        state: &mut DispatchState<'_>,
        reply_status: u8,
        actions: PostActions,
    ) -> CommissionOutcome {
        let restore_status = self
            .terminal
            .outer_restore_failure
            .map_or(0, |fault| fault.status_code());
        let disposition = match actions.stepper {
            StepperPostAction::ArmPreservePosition => 0,
            StepperPostAction::Stop if self.terminal.is_safety_fault => 2,
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
            self.terminal.outer_error.status_code(),
            self.terminal.restore_drift_counts,
            disposition,
            restore_level,
        );

        let (oid, safety_fault) = {
            let engine_ref = engine.as_ref().expect("engine slot None on OuterFailed");
            (engine_ref.oid(), engine_ref.outer_safety_fault())
        };
        state.tmc.commission_in_flight.store(false, Relaxed);
        self.terminal.use_handled_stop_take = self.terminal.velocity_workflow_restored;
        let _ = self.take_engine(engine);

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

    fn velocity_staged_prologue<B: CommissioningBackend>(
        &mut self,
        engine: &mut Option<CommissioningEngine>,
        backend: &mut B,
        retention: &mut CommissionRetention<'_>,
    ) {
        let TerminalPayload::VelocityIntegral(mut terminal) = self.terminal.payload.clone() else {
            unreachable!("velocity cursor without a velocity payload")
        };
        if let Some(record) = engine
            .as_ref()
            .and_then(|engine| engine.attach_stage_c_rest_boundary(&mut terminal))
        {
            #[cfg(feature = "trace")]
            backend.record_velocity_sweep_trace_record(record);
            #[cfg(not(feature = "trace"))]
            let _ = record;
        }
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

        #[cfg(feature = "trace")]
        let mut promotion_installer_result: u8 = 0;
        defmt::info!(
            "velocity-integral promotion check: outcome={} execution_plan_present={} authority_present={}",
            dispatch.terminal.outcome,
            breakaway_execution_plan.is_some(),
            dispatch.authority.is_some(),
        );
        if dispatch.terminal.outcome == VelocityIntegralOutcome::Complete
            && let (Some(execution_plan), Some(authority)) =
                (breakaway_execution_plan, dispatch.authority)
        {
            match stage_c::breakaway_acceptance_point(
                self.channel,
                &authority,
                execution_plan,
                &dispatch.terminal,
            ) {
                Ok(stage_c::AcceptanceInstall::Installed(point)) => {
                    defmt::info!("velocity-integral promotion: acceptance point installed");
                    retention.stage_c.install_acceptance_point(point);
                    #[cfg(feature = "trace")]
                    {
                        promotion_installer_result = 1;
                    }
                }
                Ok(stage_c::AcceptanceInstall::MissingTargetTerminus) => {
                    defmt::warn!("velocity-integral promotion: missing target terminus");
                    dispatch.terminal.cause =
                        StageCCause::Dispatch(stage_c::STAGE_C_CAUSE_NO_ACCEPTANCE_TERMINUS);
                    #[cfg(feature = "trace")]
                    {
                        promotion_installer_result = 2;
                    }
                }
                Ok(stage_c::AcceptanceInstall::NotAcceptanceTerminal) => {
                    defmt::info!("velocity-integral promotion: not an acceptance terminal");
                    #[cfg(feature = "trace")]
                    {
                        promotion_installer_result = 3;
                    }
                }
                Err(()) => {
                    defmt::warn!("velocity-integral promotion: evidence integrity fault");
                    retention.stage_c.observe_fault();
                    dispatch.terminal.outcome = VelocityIntegralOutcome::Fault;
                    dispatch.terminal.cause =
                        StageCCause::Dispatch(stage_c::STAGE_C_CAUSE_EVIDENCE_INTEGRITY);
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
                        channel: self.channel,
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
        #[cfg(not(feature = "trace"))]
        let _ = backend;

        self.terminal.payload = TerminalPayload::VelocityIntegral(dispatch.terminal);
        self.terminal.reproduction = dispatch.reproduction;
        self.terminal.authority = dispatch.authority;
    }

    /// Terminal reply for a restore torn down before it finished.
    fn cancelled_reply(&self, oid: u8, status_error: CommissionError) -> ReplyPayload {
        let status = status_error.status_code();
        match self.terminal.payload.clone() {
            TerminalPayload::Selftest => ReplyPayload::Selftest {
                oid,
                results: crate::selftest::SelftestResults::default(),
                diagnostics: crate::selftest::CommissionDiagnostics::default(),
                done_status: status,
            },
            TerminalPayload::InnerFailed(_) | TerminalPayload::CalibrateDone(_) => {
                ReplyPayload::CalibrateResult {
                    oid,
                    status: super::calibration_status_from_commission_error(status_error),
                    result: zero_adc_cal_result(),
                    diagnostics: crate::selftest::CommissionDiagnostics::default(),
                }
            }
            TerminalPayload::CommissionDone(result) => ReplyPayload::CommissionResult {
                oid,
                status,
                result,
            },
            TerminalPayload::OuterDone(_) | TerminalPayload::OuterFailed => {
                ReplyPayload::TuneResult {
                    oid,
                    status,
                    result: zero_outer_result(),
                    safety_fault: None,
                }
            }
            TerminalPayload::VelocityIntegral(mut terminal) => {
                terminal.outcome = VelocityIntegralOutcome::Fault;
                terminal.cause = StageCCause::Error(status);
                terminal.recovery_unavailable = true;
                terminal.safety_fault = None;
                ReplyPayload::VelocityIntegralTerminal {
                    oid,
                    terminal,
                    reproduction: None,
                    authority: None,
                }
            }
            TerminalPayload::FixedGainAmplitude(mut terminal) => {
                terminal.outcome = FixedGainAmplitudeOutcome::Fault;
                terminal.cause = FixedGainAmplitudeCause::EvidenceIntegrity;
                terminal.safety_fault = None;
                ReplyPayload::FixedGainAmplitudeTerminal { oid, terminal }
            }
            TerminalPayload::RobustnessReversal(mut terminal) => {
                terminal.outcome = RobustnessOutcome::Failed;
                terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
                ReplyPayload::RobustnessReversalTerminal { oid, terminal }
            }
        }
    }
}

fn commission_status_code(status: crate::autotune::CommissionStatus) -> u8 {
    match status {
        crate::autotune::CommissionStatus::Accepted => 0,
        crate::autotune::CommissionStatus::AcceptedWithWarnings => 1,
        crate::autotune::CommissionStatus::InnerOnly => 2,
        crate::autotune::CommissionStatus::Failed(error) => error.status_code(),
    }
}
