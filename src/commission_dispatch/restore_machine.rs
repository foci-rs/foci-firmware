//! Poll-driven per-channel terminal restore.

use core::sync::atomic::Ordering::Relaxed;

use crate::autotune::CommissionError;
use crate::commission_dispatch::{
    CommissionOutcome, CommissionRetention, DispatchState, PostActions, ReplyPayload,
    StepperPostAction, integral, record_restore_summary, restore, robustness, validate_outer_gains,
    zero_adc_cal_result, zero_commission_result, zero_outer_result,
};
use crate::commissioning::CommissioningEngine;
use crate::commissioning::schedule::CommissionTerminal;
use crate::commissioning::types::{
    AdcCalResult, CommissionResult, CommissioningBackend, EngineKind, IntegralOutcomeCause,
    OuterResult, RobustnessOutcome, RobustnessReversalTerminal, StepResult,
    VelocityIntegralOutcome, VelocityIntegralTerminal,
};
use crate::pid::OuterLoopGains;
use crate::tmc_control::lifecycle::{ArmMotorElectricalStep, LifecycleFlag};
use crate::tmc_register_values::MOTION_STOPPED;
use crate::tmc_safe_state::SafeStateRestoreStep;

/// One advance of a terminal restore.
#[allow(clippy::large_enum_variant)]
pub enum RestoreStep {
    /// More backend work remains; poll again on the next tick.
    Pending,
    /// The restore reached its terminal; the engine slot has been consumed.
    Done(CommissionOutcome),
}

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
    RobustnessReversal(RobustnessReversalTerminal),
}

struct OwnedTerminalData {
    payload: TerminalPayload,
    status_error: CommissionError,
    outer_error: CommissionError,
    outer_restore_failure: Option<CommissionError>,
    restore_drift_counts: u32,
    is_safety_fault: bool,
    velocity_workflow_restored: bool,
    baseline_restored: bool,
    use_handled_stop_take: bool,
    reproduction: Option<integral::IntegralReproductionReport>,
    authority: Option<
        crate::commissioning::outer::velocity::integral_reproduction::IntegralAuthoritySummary,
    >,
    robustness_evidence:
        Option<crate::commissioning::outer::velocity::robustness_evidence::RobustnessEvidence>,
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
            robustness_evidence: None,
        }
    }
}

#[derive(Clone, Copy)]
enum RestoreCursor {
    SelftestRestoreGains(Option<restore::CalibrateGainsRestoreStep>),

    InnerFailedSafeState(SafeStateRestoreStep),
    InnerFailedRestoreGains(Option<restore::CalibrateGainsRestoreStep>),

    CalibrateRestoreGains(Option<restore::CalibrateGainsRestoreStep>),
    CalibrateApplyFilters(Option<crate::tmc_control::filters::FilterCutoffsStep>),
    CalibrateArm(Option<ArmMotorElectricalStep>),
    CalibrateDisarm(SafeStateRestoreStep, u8),

    CommissionClassify,
    CommissionWriteCurrentLimit(OuterLoopGains),
    CommissionArm(OuterLoopGains, Option<ArmMotorElectricalStep>),
    CommissionDisarm(SafeStateRestoreStep, u8),

    OuterDoneDisarm {
        step: SafeStateRestoreStep,
        outcome: OuterDoneOutcome,
    },

    OuterFailedStagedTiming,
    OuterFailedDisarmVelocity(Option<VelocityDisarmStep>),
    OuterFailedClassify,
    OuterFailedRestoreEntryGains(Option<restore::RestoreEntryGainsStep>),
    OuterFailedSafetyRestore(Option<restore::RestoreEntryGainsStep>),
    OuterFailedStability(Option<restore::PositionStabilityStep>),
    OuterFailedModeRamp,
    OuterFailedFaultDisarm(SafeStateRestoreStep),
    OuterFailedArm(Option<ArmMotorElectricalStep>),
    OuterFailedArmDisarm(SafeStateRestoreStep, u8),

    RobustnessStagedClassify,
    RobustnessRestoreBaseline(Option<restore::EntrySnapshotRestoreStep>),
    RobustnessDisarm(SafeStateRestoreStep),

    VelocityStagedPrologue,
    VelocityStagedRestoreBaseline(Option<restore::EntrySnapshotRestoreStep>),
    VelocityStagedDisarmVelocity(Option<VelocityDisarmStep>),
    VelocityDisarmVelocity(Option<VelocityDisarmStep>),

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
    #[cfg(test)]
    entered_single_op_arm_this_poll: bool,
}

#[derive(Clone, Copy)]
enum OuterDoneOutcome {
    PublishTunedGains(OuterLoopGains),
    RejectInvalidGains,
}

fn outer_done_disarm_cursor(outer: &OuterResult) -> RestoreCursor {
    let outcome = match validate_outer_gains(
        outer.position_p,
        outer.position_i,
        outer.velocity_p,
        outer.velocity_i,
    ) {
        Ok(gains) => OuterDoneOutcome::PublishTunedGains(gains),
        Err(_) => {
            defmt::error!("OuterDone: tuned outer gains out of range");
            OuterDoneOutcome::RejectInvalidGains
        }
    };
    RestoreCursor::OuterDoneDisarm {
        step: SafeStateRestoreStep::default(),
        outcome,
    }
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
                RestoreCursor::SelftestRestoreGains(None),
            ),
            StepResult::InnerFailed(error) => (
                TerminalPayload::InnerFailed(error),
                RestoreCursor::InnerFailedSafeState(SafeStateRestoreStep::default()),
            ),
            StepResult::CalibrateDone(result) => (
                TerminalPayload::CalibrateDone(result),
                RestoreCursor::CalibrateRestoreGains(None),
            ),
            StepResult::CommissionDone(result) => (
                TerminalPayload::CommissionDone(result),
                RestoreCursor::CommissionClassify,
            ),
            StepResult::OuterDone(result) => {
                let cursor = outer_done_disarm_cursor(&result);
                (TerminalPayload::OuterDone(result), cursor)
            }
            StepResult::OuterFailed(error) => {
                let mut data = OwnedTerminalData::new(TerminalPayload::OuterFailed);
                data.outer_error = error;
                return Self {
                    channel,
                    pwm_maxcnt,
                    cursor: RestoreCursor::OuterFailedDisarmVelocity(None),
                    terminal: data,
                    engine_consumed: false,
                    #[cfg(test)]
                    entered_single_op_arm_this_poll: false,
                };
            }
            StepResult::VelocityIntegralTerminal(terminal) => (
                TerminalPayload::VelocityIntegral(terminal),
                RestoreCursor::VelocityDisarmVelocity(None),
            ),
            StepResult::RobustnessReversalTerminal(terminal) => (
                TerminalPayload::RobustnessReversal(terminal),
                RestoreCursor::RobustnessRestoreBaseline(None),
            ),
            StepResult::ReadyNow
            | StepResult::At(_)
            | StepResult::PhaseChanged(_)
            | StepResult::VelocityIntegralEvent(_)
            | StepResult::BreakawayCampaignEvent(_)
            | StepResult::BreakawayIntegralPlanReady { .. } => {
                unreachable!("nonterminal step result cannot start a terminal restore")
            }
        };
        Self {
            channel,
            pwm_maxcnt,
            cursor,
            terminal: OwnedTerminalData::new(payload),
            engine_consumed: false,
            #[cfg(test)]
            entered_single_op_arm_this_poll: false,
        }
    }

    /// Begin a staged terminal restore, applying the retained-evidence policy
    /// the velocity-integral and robustness terminals carry.
    pub fn start_staged(result: CommissionTerminal, channel: u8, pwm_maxcnt: u16) -> Self {
        match result {
            CommissionTerminal::VelocityIntegral(terminal) => Self {
                channel,
                pwm_maxcnt,
                cursor: RestoreCursor::VelocityStagedPrologue,
                terminal: OwnedTerminalData::new(TerminalPayload::VelocityIntegral(terminal)),
                engine_consumed: false,
                #[cfg(test)]
                entered_single_op_arm_this_poll: false,
            },
            CommissionTerminal::RobustnessReversal(terminal) => Self {
                channel,
                pwm_maxcnt,
                cursor: RestoreCursor::RobustnessStagedClassify,
                terminal: OwnedTerminalData::new(TerminalPayload::RobustnessReversal(terminal)),
                engine_consumed: false,
                #[cfg(test)]
                entered_single_op_arm_this_poll: false,
            },
            other => {
                let mut machine = Self::start(other.into_step_result(), channel, pwm_maxcnt);
                if matches!(machine.cursor, RestoreCursor::OuterFailedDisarmVelocity(_)) {
                    machine.cursor = RestoreCursor::OuterFailedStagedTiming;
                }
                machine
            }
        }
    }

    /// Begin a synthetic terminal restore for a channel whose engine is still stepping.
    ///
    /// Builds a placeholder payload for the engine kind and jumps to `ForceSafeDisarm`.
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
            #[cfg(test)]
            entered_single_op_arm_this_poll: false,
        }
    }

    pub(crate) fn set_status_error(&mut self, status_error: CommissionError) {
        self.terminal.status_error = status_error;
    }

    #[cfg(test)]
    pub(crate) fn status_error(&self) -> CommissionError {
        self.terminal.status_error
    }

    /// Return this machine's board-local TMC channel.
    pub const fn channel(&self) -> u8 {
        self.channel
    }

    /// Return whether the restore still has work left.
    pub const fn is_pending(&self) -> bool {
        !matches!(self.cursor, RestoreCursor::Finished)
    }

    #[cfg(test)]
    pub(crate) fn entered_single_op_arm_this_poll(&self) -> bool {
        self.entered_single_op_arm_this_poll
    }

    /// Advance the restore by at most one awaited backend operation.
    ///
    /// Returns [`RestoreStep::Done`] once the terminal outcome is ready.
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
        #[cfg(test)]
        {
            self.entered_single_op_arm_this_poll = false;
        }
        let mut retention = retention;
        loop {
            match self.cursor {
                RestoreCursor::Finished => {
                    unreachable!("terminal restore polled after it reported its terminal")
                }

                RestoreCursor::SelftestRestoreGains(step) => {
                    let mut step = step.unwrap_or_else(|| {
                        state.tmc.commission_in_flight.store(false, Relaxed);
                        restore::CalibrateGainsRestoreStep::start(*state.stored_gains)
                    });
                    if let Some(result) = step.advance(backend, state.tmc).await {
                        if result.is_err() {
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
                    self.cursor = RestoreCursor::SelftestRestoreGains(Some(step));
                    return RestoreStep::Pending;
                }

                RestoreCursor::InnerFailedSafeState(mut step) => {
                    if step.is_start() {
                        state.tmc.commission_in_flight.store(false, Relaxed);
                    }
                    if step.advance(backend).await {
                        self.cursor = RestoreCursor::InnerFailedSafeState(step);
                    } else {
                        self.cursor = RestoreCursor::InnerFailedRestoreGains(None);
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::InnerFailedRestoreGains(step) => {
                    let kind = self.engine_kind(engine);
                    if matches!(kind, EngineKind::Selftest | EngineKind::Calibrate) {
                        let mut s = step.unwrap_or_else(|| {
                            restore::CalibrateGainsRestoreStep::start(*state.stored_gains)
                        });
                        if let Some(result) = s.advance(backend, state.tmc).await {
                            if result.is_err() {
                                defmt::error!("InnerFailed: saved gain restore SPI failed");
                            }
                        } else {
                            self.cursor = RestoreCursor::InnerFailedRestoreGains(Some(s));
                            return RestoreStep::Pending;
                        }
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

                RestoreCursor::CalibrateRestoreGains(step) => {
                    let mut step = step.unwrap_or_else(|| {
                        state.tmc.motor_armed.store(false, Relaxed);
                        restore::CalibrateGainsRestoreStep::start(*state.stored_gains)
                    });
                    match step.advance(backend, state.tmc).await {
                        None => {
                            self.cursor = RestoreCursor::CalibrateRestoreGains(Some(step));
                        }
                        Some(Ok(())) => {
                            self.cursor = RestoreCursor::CalibrateApplyFilters(None);
                        }
                        Some(Err(kind)) => {
                            defmt::error!("CalibrateDone: saved gain restore SPI failed");
                            let status = match kind {
                                restore::GainsRestoreErrorKind::PiRepresentation => {
                                    crate::calibration::CalibrationError::ChipResetDetected
                                        .status_code()
                                }
                                restore::GainsRestoreErrorKind::Spi
                                | restore::GainsRestoreErrorKind::InvalidGains => {
                                    crate::calibration::CalibrationError::SpiError.status_code()
                                }
                            };
                            self.cursor = RestoreCursor::CalibrateDisarm(
                                SafeStateRestoreStep::default(),
                                status,
                            );
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CalibrateApplyFilters(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    let mut step = step.unwrap_or_else(|| {
                        crate::tmc_control::filters::FilterCutoffsStep::from_stored(
                            state.tmc,
                            self.pwm_maxcnt,
                        )
                    });
                    match step.advance(backend).await {
                        None => {
                            self.cursor = RestoreCursor::CalibrateApplyFilters(Some(step));
                        }
                        Some(Ok(())) => {
                            self.cursor = RestoreCursor::CalibrateArm(None);
                        }
                        Some(Err(())) => {
                            defmt::error!("CalibrateDone: stored filter restore SPI failed");
                            self.cursor = RestoreCursor::CalibrateDisarm(
                                SafeStateRestoreStep::default(),
                                crate::calibration::CalibrationError::SpiError.status_code(),
                            );
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CalibrateArm(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    *state.calibrated = true;
                    let mut step = step.unwrap_or_else(ArmMotorElectricalStep::start);
                    match step.advance(backend, state).await {
                        None => {
                            self.cursor = RestoreCursor::CalibrateArm(Some(step));
                        }
                        Some(Ok(())) => {
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
                        Some(Err(error)) => {
                            defmt::error!("CalibrateDone: arm sentinel verify failed: {}", error);
                            let status = match error {
                                CommissionError::ChipResetDetected => {
                                    crate::calibration::CalibrationError::ChipResetDetected
                                        .status_code()
                                }
                                CommissionError::FaultInterlock => {
                                    crate::calibration::CalibrationError::FaultInterlock
                                        .status_code()
                                }
                                CommissionError::InvalidCurrentLimit => {
                                    crate::calibration::CalibrationError::ConfigFault.status_code()
                                }
                                CommissionError::SafeStateIncomplete => {
                                    crate::calibration::CalibrationError::SafeStateIncomplete
                                        .status_code()
                                }
                                _ => crate::calibration::CalibrationError::SpiError.status_code(),
                            };
                            self.cursor = RestoreCursor::CalibrateDisarm(
                                SafeStateRestoreStep::default(),
                                status,
                            );
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CalibrateDisarm(mut step, status) => {
                    if poll_disarm_motor(&mut step, backend, state).await {
                        self.cursor = RestoreCursor::CalibrateDisarm(step, status);
                        return RestoreStep::Pending;
                    }
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
                                SafeStateRestoreStep::default(),
                                CommissionError::CurrentValidationFailed.status_code(),
                            );
                        }
                    }
                }
                RestoreCursor::CommissionWriteCurrentLimit(gains) => {
                    if backend
                        .write_register(
                            foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                            crate::tmc_control::current_limit::operational_limit_register(
                                state.tmc,
                            ),
                        )
                        .await
                        .is_err()
                    {
                        defmt::error!("CommissionDone: current-limit restore SPI failed");
                        self.cursor = RestoreCursor::CommissionDisarm(
                            SafeStateRestoreStep::default(),
                            CommissionError::Spi.status_code(),
                        );
                    } else {
                        self.cursor = RestoreCursor::CommissionArm(gains, None);
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CommissionArm(fallback_outer_gains, step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    *state.calibrated = true;
                    let mut step = step.unwrap_or_else(ArmMotorElectricalStep::start);
                    match step.advance(backend, state).await {
                        None => {
                            self.cursor =
                                RestoreCursor::CommissionArm(fallback_outer_gains, Some(step));
                        }
                        Some(Err(error)) => {
                            defmt::error!("CommissionDone: arm sentinel verify failed: {}", error);
                            self.cursor = RestoreCursor::CommissionDisarm(
                                SafeStateRestoreStep::default(),
                                error.status_code(),
                            );
                        }
                        Some(Ok(())) => {
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
                                    stepper: StepperPostAction::ArmPreservePosition,
                                },
                            });
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::CommissionDisarm(mut step, status) => {
                    if poll_disarm_motor(&mut step, backend, state).await {
                        self.cursor = RestoreCursor::CommissionDisarm(step, status);
                        return RestoreStep::Pending;
                    }
                    let TerminalPayload::CommissionDone(commission_result) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("commission cursor without a commission payload")
                    };
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

                RestoreCursor::OuterDoneDisarm { mut step, outcome } => {
                    if poll_disarm_motor(&mut step, backend, state).await {
                        self.cursor = RestoreCursor::OuterDoneDisarm { step, outcome };
                        return RestoreStep::Pending;
                    }
                    let TerminalPayload::OuterDone(outer) = self.terminal.payload.clone() else {
                        unreachable!("outer-done cursor without an outer-done payload")
                    };
                    let (status, result) = match outcome {
                        OuterDoneOutcome::PublishTunedGains(gains) => {
                            state.tmc.store_outer_gains(gains, Relaxed);
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
                            (commission_status_code(outer.status), outer)
                        }
                        OuterDoneOutcome::RejectInvalidGains => (
                            CommissionError::CurrentValidationFailed.status_code(),
                            zero_outer_result(),
                        ),
                    };
                    state.tmc.commission_in_flight.store(false, Relaxed);
                    let consumed = self.take_engine(engine);
                    let result = OuterResult {
                        motor_disarmed: true,
                        ..result
                    };
                    return RestoreStep::Done(CommissionOutcome::Terminal {
                        reply: ReplyPayload::TuneResult {
                            oid: consumed.oid(),
                            status,
                            result,
                            safety_fault: None,
                            robustness_evidence: self.terminal.robustness_evidence,
                        },
                        actions: PostActions {
                            stop_trace: true,
                            stepper: StepperPostAction::Stop,
                        },
                    });
                }

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
                    self.cursor = RestoreCursor::OuterFailedDisarmVelocity(None);
                }
                RestoreCursor::OuterFailedDisarmVelocity(step) => {
                    if engine
                        .as_ref()
                        .and_then(CommissioningEngine::velocity_operational_torque_limit)
                        .is_some()
                    {
                        let mut step = step.unwrap_or_else(|| VelocityDisarmStep::start(engine));
                        match poll_velocity_disarm(&mut step, backend, state).await {
                            None => {
                                self.cursor = RestoreCursor::OuterFailedDisarmVelocity(Some(step));
                                return RestoreStep::Pending;
                            }
                            Some(restored) => {
                                self.terminal.velocity_workflow_restored = restored;
                                engine
                                    .as_mut()
                                    .expect("velocity workflow disappeared during restoration")
                                    .mark_velocity_limit_restored();
                                self.cursor = RestoreCursor::OuterFailedClassify;
                                return RestoreStep::Pending;
                            }
                        }
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
                        if matches!(error, CommissionError::SafetyEnvelopeViolation)
                            && self.entry_snapshot(engine).is_some()
                        {
                            self.cursor = RestoreCursor::OuterFailedSafetyRestore(None);
                        } else {
                            self.cursor = RestoreCursor::OuterFailedModeRamp;
                        }
                    } else if self.entry_snapshot(engine).is_some() {
                        self.cursor = RestoreCursor::OuterFailedRestoreEntryGains(None);
                    } else {
                        self.terminal.outer_restore_failure = Some(CommissionError::Spi);
                        self.cursor = RestoreCursor::OuterFailedModeRamp;
                    }
                }
                RestoreCursor::OuterFailedRestoreEntryGains(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    let snapshot = self
                        .entry_snapshot(engine)
                        .expect("entry snapshot disappeared during outer-failed restore");
                    let mut step =
                        step.unwrap_or_else(|| restore::RestoreEntryGainsStep::start(&snapshot));
                    match step.advance(backend, state.tmc).await {
                        None => {
                            self.cursor = RestoreCursor::OuterFailedRestoreEntryGains(Some(step));
                        }
                        Some(Ok(())) => {
                            self.cursor = RestoreCursor::OuterFailedStability(None);
                        }
                        Some(Err(kind)) => {
                            let restore_err = CommissionError::from(kind);
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
                RestoreCursor::OuterFailedSafetyRestore(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    let snapshot = self
                        .entry_snapshot(engine)
                        .expect("entry snapshot disappeared during safety-fault restore");
                    let mut step =
                        step.unwrap_or_else(|| restore::RestoreEntryGainsStep::start(&snapshot));
                    match step.advance(backend, state.tmc).await {
                        None => {
                            self.cursor = RestoreCursor::OuterFailedSafetyRestore(Some(step));
                        }
                        Some(Ok(())) => {
                            self.cursor = RestoreCursor::OuterFailedModeRamp;
                        }
                        Some(Err(kind)) => {
                            let restore_err = CommissionError::from(kind);
                            defmt::error!(
                                "OuterFailed: safety-fault gain restore failed: {}",
                                restore_err
                            );
                            self.cursor = RestoreCursor::OuterFailedModeRamp;
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedStability(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    let mut step = step.unwrap_or_else(|| {
                        restore::PositionStabilityStep::start(backend.elapsed_us())
                    });
                    match step.advance(backend).await {
                        None => {
                            self.cursor = RestoreCursor::OuterFailedStability(Some(step));
                        }
                        Some(Ok(stability)) => {
                            self.terminal.restore_drift_counts = stability.span_counts;
                            if stability.stable {
                                self.terminal.outer_restore_failure = None;
                                self.cursor = RestoreCursor::OuterFailedArm(None);
                            } else {
                                self.terminal.outer_restore_failure =
                                    Some(self.terminal.outer_error);
                                self.cursor = RestoreCursor::OuterFailedModeRamp;
                            }
                        }
                        Some(Err(
                            restore::PositionStabilityErrorKind::Spi
                            | restore::PositionStabilityErrorKind::Stale,
                        )) => {
                            defmt::error!("OuterFailed restore: stability sampling failed");
                            self.terminal.outer_restore_failure = Some(CommissionError::Spi);
                            self.cursor = RestoreCursor::OuterFailedModeRamp;
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedModeRamp => {
                    let _ = backend
                        .write_register(
                            foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                            MOTION_STOPPED,
                        )
                        .await;
                    self.cursor =
                        RestoreCursor::OuterFailedFaultDisarm(SafeStateRestoreStep::default());
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedFaultDisarm(mut step) => {
                    if poll_disarm_motor(&mut step, backend, state).await {
                        self.cursor = RestoreCursor::OuterFailedFaultDisarm(step);
                        return RestoreStep::Pending;
                    }
                    let fault = self
                        .terminal
                        .outer_restore_failure
                        .expect("outer-failed fault disarm without a restore failure");
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
                RestoreCursor::OuterFailedArm(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    *state.calibrated = true;
                    let mut step = step.unwrap_or_else(ArmMotorElectricalStep::start);
                    match step.advance(backend, state).await {
                        None => {
                            self.cursor = RestoreCursor::OuterFailedArm(Some(step));
                        }
                        Some(Err(arm_error)) => {
                            self.cursor = RestoreCursor::OuterFailedArmDisarm(
                                SafeStateRestoreStep::default(),
                                arm_error.status_code(),
                            );
                            defmt::error!(
                                "OuterFailed: arm sentinel verify failed after soft failure: {}",
                                arm_error
                            );
                        }
                        Some(Ok(())) => {
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
                    return RestoreStep::Pending;
                }
                RestoreCursor::OuterFailedArmDisarm(mut step, status) => {
                    if poll_disarm_motor(&mut step, backend, state).await {
                        self.cursor = RestoreCursor::OuterFailedArmDisarm(step, status);
                        return RestoreStep::Pending;
                    }
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

                RestoreCursor::RobustnessStagedClassify => {
                    let TerminalPayload::RobustnessReversal(terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("robustness cursor without a robustness payload")
                    };
                    let retention = retention
                        .take()
                        .expect("staged terminal restore requires Integral retention");
                    let mut terminal =
                        robustness::apply_robustness_terminal(terminal, retention.integral);
                    let deploy = (terminal.outcome == RobustnessOutcome::Complete)
                        .then(|| {
                            engine
                                .as_ref()
                                .and_then(CommissioningEngine::entry_snapshot)
                                .zip(retention.integral.acceptance_point())
                        })
                        .flatten();
                    match deploy {
                        Some((entry, point)) => {
                            let result = crate::commissioning::outer::velocity::robustness_reversal::robustness_pass_outer_result(
                                &terminal,
                                &entry,
                                &point,
                            );
                            self.cursor = outer_done_disarm_cursor(&result);
                            self.terminal.payload = TerminalPayload::OuterDone(result);
                            self.terminal.robustness_evidence = Some(terminal.evidence);
                        }
                        None => {
                            if terminal.outcome == RobustnessOutcome::Complete {
                                terminal.outcome = RobustnessOutcome::Failed;
                                terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
                            }
                            self.terminal.payload = TerminalPayload::RobustnessReversal(terminal);
                            self.cursor = RestoreCursor::RobustnessRestoreBaseline(None);
                        }
                    }
                }
                RestoreCursor::RobustnessRestoreBaseline(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    let TerminalPayload::RobustnessReversal(mut terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("robustness cursor without a robustness payload")
                    };
                    if step.is_none() && terminal.outcome == RobustnessOutcome::Complete {
                        self.cursor =
                            RestoreCursor::RobustnessDisarm(SafeStateRestoreStep::default());
                        continue;
                    }
                    let Some(snapshot) = self.entry_snapshot(engine) else {
                        defmt::error!(
                            "robustness terminal: no entry snapshot to restore baseline from"
                        );
                        terminal.outcome = RobustnessOutcome::Failed;
                        terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
                        self.terminal.payload = TerminalPayload::RobustnessReversal(terminal);
                        self.cursor =
                            RestoreCursor::RobustnessDisarm(SafeStateRestoreStep::default());
                        return RestoreStep::Pending;
                    };
                    let pwm_maxcnt = self.pwm_maxcnt;
                    let mut step = step.unwrap_or_else(|| {
                        restore::EntrySnapshotRestoreStep::start(&snapshot, pwm_maxcnt)
                    });
                    match step.advance(backend).await {
                        None => {
                            self.cursor = RestoreCursor::RobustnessRestoreBaseline(Some(step));
                        }
                        Some(Ok(())) => {
                            self.cursor =
                                RestoreCursor::RobustnessDisarm(SafeStateRestoreStep::default());
                        }
                        Some(Err(err)) => {
                            defmt::error!("robustness terminal: baseline restore failed: {}", err);
                            terminal.outcome = RobustnessOutcome::Failed;
                            terminal.cause = robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT;
                            self.terminal.payload = TerminalPayload::RobustnessReversal(terminal);
                            self.cursor =
                                RestoreCursor::RobustnessDisarm(SafeStateRestoreStep::default());
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::RobustnessDisarm(mut step) => {
                    if poll_disarm_motor(&mut step, backend, state).await {
                        self.cursor = RestoreCursor::RobustnessDisarm(step);
                        return RestoreStep::Pending;
                    }
                    let TerminalPayload::RobustnessReversal(terminal) =
                        self.terminal.payload.clone()
                    else {
                        unreachable!("robustness cursor without a robustness payload")
                    };
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
                        .expect("staged terminal restore requires Integral retention");
                    self.velocity_staged_prologue(engine, backend, retention);
                    if self.entry_snapshot(engine).is_some() {
                        self.cursor = RestoreCursor::VelocityStagedRestoreBaseline(None);
                    } else {
                        self.terminal.baseline_restored = false;
                        self.cursor = RestoreCursor::VelocityStagedDisarmVelocity(None);
                    }
                }
                RestoreCursor::VelocityStagedRestoreBaseline(step) => {
                    #[cfg(test)]
                    {
                        self.entered_single_op_arm_this_poll = true;
                    }
                    let snapshot = self
                        .entry_snapshot(engine)
                        .expect("entry snapshot disappeared during Integral restore");
                    let pwm_maxcnt = self.pwm_maxcnt;
                    let mut step = step.unwrap_or_else(|| {
                        restore::EntrySnapshotRestoreStep::start(&snapshot, pwm_maxcnt)
                    });
                    match step.advance(backend).await {
                        None => {
                            self.cursor = RestoreCursor::VelocityStagedRestoreBaseline(Some(step));
                        }
                        Some(result) => {
                            self.terminal.baseline_restored = result.is_ok();
                            self.cursor = RestoreCursor::VelocityStagedDisarmVelocity(None);
                        }
                    }
                    return RestoreStep::Pending;
                }
                RestoreCursor::VelocityStagedDisarmVelocity(step) => {
                    let retention = retention
                        .take()
                        .expect("staged terminal restore requires Integral retention");
                    let mut step = step.unwrap_or_else(|| VelocityDisarmStep::start(engine));
                    match poll_velocity_disarm(&mut step, backend, state).await {
                        None => {
                            self.cursor = RestoreCursor::VelocityStagedDisarmVelocity(Some(step));
                            return RestoreStep::Pending;
                        }
                        Some(restored) => {
                            let TerminalPayload::VelocityIntegral(mut terminal) =
                                self.terminal.payload.clone()
                            else {
                                unreachable!("velocity cursor without a velocity payload")
                            };
                            let no_limit_branch = matches!(step, VelocityDisarmStep::Disarm(_));
                            if !no_limit_branch {
                                engine
                                    .as_mut()
                                    .expect("velocity workflow disappeared during restoration")
                                    .mark_velocity_limit_restored();
                            }
                            if !restored || !self.terminal.baseline_restored {
                                retention.integral.observe_fault();
                                terminal.outcome = VelocityIntegralOutcome::Fault;
                                terminal.cause =
                                    IntegralOutcomeCause::Error(CommissionError::Spi.status_code());
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
                                .emit_integral_terminal_trace(
                                    backend,
                                    &terminal,
                                    self.terminal.reproduction,
                                );
                            self.cursor = RestoreCursor::VelocityDisarmVelocity(None);
                            return RestoreStep::Pending;
                        }
                    }
                }
                RestoreCursor::VelocityDisarmVelocity(step) => {
                    let mut step = match step {
                        Some(step) => step,
                        None => {
                            let TerminalPayload::VelocityIntegral(mut terminal) =
                                self.terminal.payload.clone()
                            else {
                                unreachable!("velocity cursor without a velocity payload")
                            };
                            if let Some(record) = engine.as_ref().and_then(|engine| {
                                engine.attach_integral_rest_boundary(&mut terminal)
                            }) {
                                #[cfg(feature = "trace")]
                                backend.record_velocity_sweep_trace_record(record);
                                #[cfg(not(feature = "trace"))]
                                let _ = record;
                            }
                            self.terminal.payload = TerminalPayload::VelocityIntegral(terminal);
                            VelocityDisarmStep::start(engine)
                        }
                    };
                    match poll_velocity_disarm(&mut step, backend, state).await {
                        None => {
                            self.cursor = RestoreCursor::VelocityDisarmVelocity(Some(step));
                            return RestoreStep::Pending;
                        }
                        Some(restored) => {
                            let no_limit_branch = matches!(step, VelocityDisarmStep::Disarm(_));
                            if !no_limit_branch {
                                engine
                                    .as_mut()
                                    .expect("velocity workflow disappeared during restoration")
                                    .mark_velocity_limit_restored();
                            }
                            let TerminalPayload::VelocityIntegral(mut terminal) =
                                self.terminal.payload.clone()
                            else {
                                unreachable!("velocity cursor without a velocity payload")
                            };
                            if !restored {
                                terminal.outcome = VelocityIntegralOutcome::Fault;
                                terminal.cause =
                                    IntegralOutcomeCause::Error(CommissionError::Spi.status_code());
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
                    }
                }

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
    /// Always de-energizes through [`crate::tmc_control::lifecycle::disarm_motor`].
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
            StepperPostAction::ArmPreservePosition | StepperPostAction::None => 0,
            StepperPostAction::Stop if self.terminal.is_safety_fault => 2,
            StepperPostAction::Stop => 1,
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

        let (oid, safety_fault, position_tune_outcome_code, position_tune_provenance) = {
            let engine_ref = engine.as_ref().expect("engine slot None on OuterFailed");
            (
                engine_ref.oid(),
                engine_ref.outer_safety_fault(),
                engine_ref.position_tune_outcome_code(),
                engine_ref.position_tune_failure_provenance(),
            )
        };
        state.tmc.commission_in_flight.store(false, Relaxed);
        self.terminal.use_handled_stop_take = self.terminal.velocity_workflow_restored;
        let _ = self.take_engine(engine);

        let mut result = zero_outer_result();
        result.position_tune_outcome_code = position_tune_outcome_code;
        result.position_tune = position_tune_provenance;
        result.motor_disarmed = matches!(actions.stepper, StepperPostAction::Stop);

        CommissionOutcome::Terminal {
            reply: ReplyPayload::TuneResult {
                oid,
                status: reply_status,
                result,
                safety_fault,
                robustness_evidence: None,
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
            .and_then(|engine| engine.attach_integral_rest_boundary(&mut terminal))
        {
            #[cfg(feature = "trace")]
            backend.record_velocity_sweep_trace_record(record);
            #[cfg(not(feature = "trace"))]
            let _ = record;
        }
        #[cfg(feature = "trace")]
        let promotion_source_diagnostic = engine.as_ref().map_or(
            (0, false),
            CommissioningEngine::integral_promotion_source_diagnostic,
        );
        let breakaway_execution_plan = engine
            .as_ref()
            .and_then(CommissioningEngine::breakaway_integral_execution_plan);
        #[cfg(feature = "trace")]
        let promotion_execution_plan_present = breakaway_execution_plan.is_some();
        let mut dispatch = integral::apply_integral_terminal(terminal, retention.integral);
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
        if dispatch.terminal.outcome == VelocityIntegralOutcome::RepeatabilityConfirmed
            && let (Some(execution_plan), Some(authority)) =
                (breakaway_execution_plan, dispatch.authority)
        {
            match integral::breakaway_acceptance_point(
                self.channel,
                &authority,
                execution_plan,
                &dispatch.terminal,
            ) {
                Ok(integral::AcceptanceInstall::Installed(point)) => {
                    defmt::info!("velocity-integral promotion: acceptance point installed");
                    dispatch.terminal.candidate_i = Some(point.selected_i);
                    retention.integral.install_acceptance_point(point);
                    #[cfg(feature = "trace")]
                    {
                        promotion_installer_result = 1;
                    }
                }
                Ok(integral::AcceptanceInstall::MissingTargetTerminus) => {
                    defmt::warn!("velocity-integral promotion: missing target terminus");
                    dispatch.terminal.cause = IntegralOutcomeCause::Dispatch(
                        integral::INTEGRAL_DISPATCH_CAUSE_NO_ACCEPTANCE_TERMINUS,
                    );
                    #[cfg(feature = "trace")]
                    {
                        promotion_installer_result = 2;
                    }
                }
                Ok(integral::AcceptanceInstall::NotAcceptanceTerminal) => {
                    defmt::info!("velocity-integral promotion: not an acceptance terminal");
                    #[cfg(feature = "trace")]
                    {
                        promotion_installer_result = 3;
                    }
                }
                Err(()) => {
                    defmt::warn!("velocity-integral promotion: evidence integrity fault");
                    retention.integral.observe_fault();
                    dispatch.terminal.outcome = VelocityIntegralOutcome::Fault;
                    dispatch.terminal.cause = IntegralOutcomeCause::Dispatch(
                        integral::INTEGRAL_DISPATCH_CAUSE_EVIDENCE_INTEGRITY,
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
                crate::commissioning::outer::velocity::integral_reporting::encode_integral_promotion_diagnostic(
                    crate::commissioning::outer::velocity::integral_reporting::IntegralPromotionDiagnostic {
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

    #[inline(never)]
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
                    result: OuterResult {
                        motor_disarmed: true,
                        ..zero_outer_result()
                    },
                    safety_fault: None,
                    robustness_evidence: None,
                }
            }
            TerminalPayload::VelocityIntegral(mut terminal) => {
                terminal.outcome = VelocityIntegralOutcome::Fault;
                terminal.cause = IntegralOutcomeCause::Error(status);
                terminal.recovery_unavailable = true;
                terminal.safety_fault = None;
                ReplyPayload::VelocityIntegralTerminal {
                    oid,
                    terminal,
                    reproduction: None,
                    authority: None,
                }
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

async fn poll_disarm_motor<B: CommissioningBackend>(
    step: &mut SafeStateRestoreStep,
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> bool {
    if step.is_start() {
        state.tmc.motor_armed.store(false, Relaxed);
        backend.record_lifecycle(LifecycleFlag::MotorArmed, false);
    }
    let more = step.advance(backend).await;
    if !more {
        let failed_writes = step.report().failed_writes;
        if failed_writes > 0 {
            defmt::error!(
                "safe-state restore had {=u8} verified-write failures after retries",
                failed_writes
            );
        }
        state.tmc.clear_velocity_clamp_latch();
        *state.calibrated = false;
        *state.motor_enabled = false;
        backend.record_lifecycle(LifecycleFlag::MotorEnabled, false);
    }
    more
}

#[derive(Clone, Copy)]
enum VelocityDisarmStep {
    Disarm(SafeStateRestoreStep),
    WriteVelocityTarget {
        operational_limit: u16,
    },
    WriteErrorSumAddr {
        operational_limit: u16,
    },
    WriteErrorSumData {
        operational_limit: u16,
    },
    LimitDisarm {
        operational_limit: u16,
        step: SafeStateRestoreStep,
    },
    ClampRestore {
        operational_limit: u16,
    },
}

impl VelocityDisarmStep {
    fn start(engine: &Option<CommissioningEngine>) -> Self {
        match engine
            .as_ref()
            .and_then(CommissioningEngine::velocity_operational_torque_limit)
        {
            Some(operational_limit) => Self::WriteVelocityTarget { operational_limit },
            None => Self::Disarm(SafeStateRestoreStep::default()),
        }
    }
}

async fn poll_velocity_disarm<B: CommissioningBackend>(
    step: &mut VelocityDisarmStep,
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> Option<bool> {
    use foci_tmc4671::registers::{PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_VELOCITY_TARGET};

    match step {
        VelocityDisarmStep::Disarm(s) => {
            if poll_disarm_motor(s, backend, state).await {
                None
            } else {
                Some(true)
            }
        }
        VelocityDisarmStep::WriteVelocityTarget { operational_limit } => {
            let operational_limit = *operational_limit;
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return Some(false);
            }
            *step = VelocityDisarmStep::WriteErrorSumAddr { operational_limit };
            None
        }
        VelocityDisarmStep::WriteErrorSumAddr { operational_limit } => {
            let operational_limit = *operational_limit;
            let sub = PID_ERROR_PID_VELOCITY_ERROR_SUM;
            if backend
                .write_register(sub.addr, u32::from(sub.sub_addr))
                .await
                .is_err()
            {
                return Some(false);
            }
            *step = VelocityDisarmStep::WriteErrorSumData { operational_limit };
            None
        }
        VelocityDisarmStep::WriteErrorSumData { operational_limit } => {
            let operational_limit = *operational_limit;
            let sub = PID_ERROR_PID_VELOCITY_ERROR_SUM;
            if backend.write_register(sub.data, 0).await.is_err() {
                return Some(false);
            }
            *step = VelocityDisarmStep::LimitDisarm {
                operational_limit,
                step: SafeStateRestoreStep::default(),
            };
            None
        }
        VelocityDisarmStep::LimitDisarm {
            operational_limit,
            step: s,
        } => {
            let operational_limit = *operational_limit;
            if poll_disarm_motor(s, backend, state).await {
                None
            } else {
                *step = VelocityDisarmStep::ClampRestore { operational_limit };
                None
            }
        }
        VelocityDisarmStep::ClampRestore { operational_limit } => {
            let mut clamp = match crate::commissioning::outer::velocity::velocity_clamp::VelocityCurrentClampSession::try_new(
                u32::from(*operational_limit),
            ) {
                Ok(clamp) => clamp,
                Err(_) => return Some(false),
            };
            Some(clamp.restore_operational_limit(backend).await.is_ok())
        }
    }
}

#[cfg(test)]
mod poll_disarm_motor_tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, dispatch_state};
    use foci_firmware_macros::fixture_test;

    #[fixture_test]
    async fn poll_disarm_motor_matches_disarm_motor_when_driven_to_completion() {
        let mut oracle_backend = MockBackend::new();
        let mut oracle_calibrated = true;
        let mut oracle_motor_enabled = true;
        let mut oracle_gains = None;
        let mut oracle_state = dispatch_state(
            &mut oracle_calibrated,
            &mut oracle_motor_enabled,
            &mut oracle_gains,
        );
        crate::tmc_control::lifecycle::disarm_motor(&mut oracle_backend, &mut oracle_state).await;

        let mut backend = MockBackend::new();
        let mut calibrated = true;
        let mut motor_enabled = true;
        let mut gains = None;
        let mut state = dispatch_state(&mut calibrated, &mut motor_enabled, &mut gains);
        let mut step = SafeStateRestoreStep::default();
        let mut polls = 0u32;
        loop {
            assert!(polls < 32, "poll_disarm_motor did not terminate");
            let more = poll_disarm_motor(&mut step, &mut backend, &mut state).await;
            polls += 1;
            if !more {
                break;
            }
        }

        assert_eq!(backend.spi_log, oracle_backend.spi_log);
        assert_eq!(calibrated, oracle_calibrated);
        assert_eq!(motor_enabled, oracle_motor_enabled);
    }
}

#[cfg(test)]
mod velocity_disarm_step_tests {
    use super::*;
    use crate::autotune::CommissionProfile;
    use crate::commission_dispatch::test_support::{
        MockBackend, SpiOp, breakaway_campaign_engine, dispatch_state,
    };
    use foci_firmware_macros::fixture_test;

    fn expected_disarm_motor_ops() -> heapless::Vec<SpiOp, 27> {
        use foci_tmc4671::registers::*;

        let mut ops: heapless::Vec<SpiOp, 27> = heapless::Vec::new();
        ops.push(SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorArmed,
            value: false,
        })
        .ok();
        ops.push(SpiOp::EnablePin(false)).ok();
        for (reg, val) in [
            (MODE_RAMP_MODE_MOTION, 0u32),
            (PID_TORQUE_FLUX_TARGET, 0),
            (PID_TORQUE_FLUX_OFFSET, 0),
            (UQ_UD_EXT, 0),
            (PHI_E_SELECTION, 0),
            (PWM_SV_CHOP, 0),
            (PID_VELOCITY_OFFSET, 0),
            (PIDOUT_UQ_UD_LIMITS, 1024),
        ] {
            ops.push(SpiOp::Write(reg.addr(), val)).ok();
            ops.push(SpiOp::Read).ok();
        }
        for sub in [
            PID_ERROR_PID_TORQUE_ERROR_SUM,
            PID_ERROR_PID_FLUX_ERROR_SUM,
            PID_ERROR_PID_VELOCITY_ERROR_SUM,
            PID_ERROR_PID_POSITION_ERROR_SUM,
        ] {
            ops.push(SpiOp::Write(sub.addr.addr(), u32::from(sub.sub_addr)))
                .ok();
            ops.push(SpiOp::Write(sub.data.addr(), 0)).ok();
        }
        ops.push(SpiOp::Lifecycle {
            flag: LifecycleFlag::MotorEnabled,
            value: false,
        })
        .ok();
        ops
    }

    async fn drive_to_completion(
        step: &mut VelocityDisarmStep,
        backend: &mut MockBackend,
        calibrated: &mut bool,
        motor_enabled: &mut bool,
    ) -> bool {
        let mut gains = None;
        let mut polls = 0u32;
        loop {
            assert!(polls < 32, "velocity disarm step did not terminate");
            let mut state = dispatch_state(calibrated, motor_enabled, &mut gains);
            if let Some(restored) = poll_velocity_disarm(step, backend, &mut state).await {
                return restored;
            }
            polls += 1;
        }
    }

    #[fixture_test]
    async fn poll_velocity_disarm_without_operational_limit_matches_safe_state_sequence() {
        let engine = Some(CommissioningEngine::new_commission(
            0,
            CommissionProfile::Balanced,
        ));
        assert!(
            engine
                .as_ref()
                .and_then(CommissioningEngine::velocity_operational_torque_limit)
                .is_none(),
            "commission engine fixture must not carry an operational torque limit for this test"
        );

        let mut backend = MockBackend::new();
        let mut calibrated = true;
        let mut motor_enabled = true;
        let mut step = VelocityDisarmStep::start(&engine);
        assert!(
            matches!(step, VelocityDisarmStep::Disarm(_)),
            "no operational limit must select the bare-disarm branch"
        );

        let restored =
            drive_to_completion(&mut step, &mut backend, &mut calibrated, &mut motor_enabled).await;

        assert!(restored, "bare disarm without faults must report restored");
        assert!(!calibrated);
        assert!(!motor_enabled);
        assert_eq!(backend.spi_log.as_slice(), expected_disarm_motor_ops());
    }

    #[fixture_test]
    async fn poll_velocity_disarm_with_operational_limit_matches_full_trace() {
        let engine = Some(breakaway_campaign_engine());
        let operational_limit = engine
            .as_ref()
            .and_then(CommissioningEngine::velocity_operational_torque_limit)
            .expect("breakaway_campaign_engine fixture must carry an operational torque limit");

        let mut backend = MockBackend::new();
        let mut calibrated = true;
        let mut motor_enabled = true;
        let mut step = VelocityDisarmStep::start(&engine);
        assert!(
            matches!(step, VelocityDisarmStep::WriteVelocityTarget { .. }),
            "an operational limit must select the velocity-limited branch"
        );

        let restored =
            drive_to_completion(&mut step, &mut backend, &mut calibrated, &mut motor_enabled).await;

        assert!(
            restored,
            "clamp restore without faults must report restored"
        );
        assert!(!calibrated);
        assert!(!motor_enabled);

        use foci_tmc4671::registers::{
            PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_TORQUE_FLUX_LIMITS, PID_VELOCITY_TARGET,
        };

        let velocity_specific_prefix = [
            SpiOp::Write(PID_VELOCITY_TARGET.addr(), 0),
            SpiOp::Write(
                PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                u32::from(PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr),
            ),
            SpiOp::Write(PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
        ];
        let clamp_restore_suffix = [
            SpiOp::Write(PID_TORQUE_FLUX_LIMITS.addr(), u32::from(operational_limit)),
            SpiOp::Read,
        ];

        let embedded_disarm = expected_disarm_motor_ops();
        let mut expected: heapless::Vec<SpiOp, 32> = heapless::Vec::new();
        for op in velocity_specific_prefix {
            expected.push(op).ok();
        }
        for op in embedded_disarm {
            expected.push(op).ok();
        }
        for op in clamp_restore_suffix {
            expected.push(op).ok();
        }

        assert_eq!(backend.spi_log.as_slice(), expected.as_slice());
    }
}
