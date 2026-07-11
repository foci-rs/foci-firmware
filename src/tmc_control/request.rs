//! Shared TMC control request runner.
//!
//! Board crates keep ownership of SPI, GPIO, RTIC task context, and reply
//! emission. This module owns board-agnostic request ordering and policy.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::autotune::{CommissionError, CommissionProfile, OperatingMode};
use crate::calibration::CalibrationError;
use crate::commission::{CommissioningEngine, EntrySnapshot, OuterTuneParams};
use crate::commission_dispatch::{ReplyPayload, StepperPostAction};
use crate::pid::{InnerLoopGains, OuterLoopGains};
use crate::selftest::{CommissionDiagnostics, SelftestResults};
use crate::state::stepper::StepperChannelState;
use crate::state::tmc::TmcChannelState;
use crate::tmc::{TmcRequest, TmcRequestKind};
use crate::tmc_control::actions::{
    BoardAction, BoardActions, ShutdownReason, StepperAction, TmcControlRequestOutcome,
    TmcControlRequestTail,
};
use crate::tmc_control::helpers::{
    apply_live_inner_gains, apply_live_outer_gains, apply_motion_scale,
};
use crate::tmc_control::state::TmcControlRuntime;

/// Board-local disposition computed before shared request handling.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum TmcPreRequestDisposition {
    /// Continue into the shared request match.
    ContinueRequest,
    /// The board already consumed this request.
    RequestHandled,
    /// Run the delegated active diagnostic request in board context.
    RunDiagnostic(TmcRequest),
}

/// A board could not install the validated mapper into its idle P3 scheduler.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MotionScaleMapperInstallError;

/// Board hooks required by the shared TMC request runner.
#[allow(async_fn_in_trait)]
pub trait TmcRequestBoard<Spi>
where
    Spi: SpiDevice<u8>,
{
    /// Return the selected board-local channel.
    fn channel(&self) -> u8;
    /// Return shared TMC state for the selected channel.
    fn tmc_state(&self) -> &'static TmcChannelState;
    /// Return shared stepper state for the selected channel.
    fn stepper_state(&self) -> &'static StepperChannelState;
    /// Return whether a board-local commissioning guard blocks this request.
    fn commissioning_blocked(&self, runtime: &TmcControlRuntime) -> bool;
    /// Return the next trace sequence id for a commissioning engine.
    fn next_commission_trace_seq(&mut self) -> u32;
    /// Return the board's normal request-tail policy.
    fn normal_tail(&self) -> TmcControlRequestTail;
    /// Return the board default current-filter cutoff for omitted host config.
    fn default_current_filter_hz(&self) -> u16;
    /// Return whether the selected channel's physical step queue is empty.
    fn motion_scale_queue_empty(&self) -> bool;
    /// Install a freshly centered pulse mapper into the idle board P3 scheduler.
    ///
    /// The shared runner calls this only while the motor is disarmed, the physical queue is
    /// empty, and the timer is inactive. Failure must leave the scheduler unconfigured.
    fn install_motion_scale_mapper(
        &mut self,
        mapper: foci_klipper::pulse_mapper::PulseMapperConfig,
    ) -> Result<(), MotionScaleMapperInstallError>;

    /// Emit a host-visible reason before rejecting motion-scale configuration.
    fn report_motion_scale_rejection(
        &mut self,
        oid: u8,
        rejection: crate::motion_scale::MotionScaleRejection,
    );

    /// Emit a host reply using board-local sender glue.
    fn emit_reply(&mut self, payload: ReplyPayload);
    /// Emit a board-local motor arm event, if supported.
    fn emit_motor_arm_event(&mut self) {}
    /// Emit a board-local motor disarm event, if supported.
    fn emit_motor_disarm_event(&mut self) {}
    /// Apply board-local stepper or terminal actions emitted by the runtime.
    fn apply_stepper_actions(&mut self, actions: BoardActions);

    /// Electrically arm the motor, then apply the board-local stepper post action.
    async fn arm_motor(
        &mut self,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<Spi>,
        post_action: StepperPostAction,
    ) -> Result<(), CommissionError>;

    /// Electrically disarm the motor in board context.
    async fn disarm_motor(&mut self, runtime: &mut TmcControlRuntime, tmc: &mut Tmc4671Spi<Spi>);

    /// Dispatch a plain TMC register/configuration request.
    async fn dispatch_register_request(
        &mut self,
        req: TmcRequest,
        tmc: &mut Tmc4671Spi<Spi>,
    ) -> Result<(), ()>;

    /// Run an active diagnostic request in board context.
    async fn run_diagnostic_request(
        &mut self,
        req: TmcRequest,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<Spi>,
    );

    /// Run dev-only open-loop initialization.
    #[cfg(feature = "dev")]
    async fn run_dev_init(&mut self, tmc: &mut Tmc4671Spi<Spi>);

    /// Run dev-only open-loop test spin.
    #[cfg(feature = "dev")]
    async fn run_dev_test_spin(&mut self, tmc: &mut Tmc4671Spi<Spi>, revolutions: u8);
}

/// Handle one board-routed TMC control request.
pub async fn handle_tmc_control_request<Spi, Board>(
    req: TmcRequest,
    pre_request: TmcPreRequestDisposition,
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<Spi>,
    board: &mut Board,
) -> TmcControlRequestOutcome
where
    Spi: SpiDevice<u8>,
    Board: TmcRequestBoard<Spi>,
{
    match pre_request {
        TmcPreRequestDisposition::ContinueRequest => {}
        TmcPreRequestDisposition::RequestHandled => {
            return normal_outcome(board.normal_tail());
        }
        TmcPreRequestDisposition::RunDiagnostic(diagnostic_req) => {
            board
                .run_diagnostic_request(diagnostic_req, runtime, tmc)
                .await;
            return normal_outcome(board.normal_tail());
        }
    }

    match req {
        TmcRequest {
            kind: TmcRequestKind::SetEnable(enabled),
            ..
        } => {
            if enabled {
                if !runtime.is_calibrated() {
                    if board.commissioning_blocked(runtime) {
                        defmt::warn!("Auto-calibrate rejected: commissioning already active");
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    if !crate::tmc::auto_calibrate_on_enable_allowed(board.channel()) {
                        defmt::warn!("Auto-calibrate rejected: host gate closed");
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    if board
                        .tmc_state()
                        .commission_in_flight
                        .swap(true, core::sync::atomic::Ordering::Relaxed)
                    {
                        defmt::warn!("Auto-calibrate skipped: commission in flight");
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    defmt::info!("Auto-calibrating on enable");
                    runtime.set_commission_engine(
                        CommissioningEngine::new_calibrate_for_tmc_with_gains_and_trace_seq(
                            0,
                            board.tmc_state(),
                            runtime.stored_gains(),
                            board.default_current_filter_hz(),
                            board.next_commission_trace_seq(),
                        ),
                    );
                    return TmcControlRequestOutcome::skip_tick_tail();
                }

                if runtime.is_motor_enabled() {
                    defmt::info!("Motor enable sync (already armed)");
                    return TmcControlRequestOutcome::skip_tick_tail();
                }

                let actions = runtime.prepare_enable_request(board.tmc_state());
                for action in actions {
                    if let BoardAction::Stepper(StepperAction::ArmCheck { post_action }) = action {
                        if !runtime.restore_phase_advance_now(tmc).await {
                            defmt::warn!("phase advance: failed to restore ABN offset before arm");
                            return TmcControlRequestOutcome::skip_tick_tail();
                        }
                        if let Err(error) = board.arm_motor(runtime, tmc, post_action).await {
                            defmt::error!("Motor enable rejected by sentinel verify: {}", error);
                            board.disarm_motor(runtime, tmc).await;
                            return TmcControlRequestOutcome::skip_tick_tail();
                        }

                        defmt::info!("Motor enabled (calibrated, step/dir armed)");
                        board.emit_motor_arm_event();
                    }
                }
            } else {
                let actions = runtime.prepare_disable_request();
                board.apply_stepper_actions(actions);

                let phase_advance_restored = runtime.restore_phase_advance_now(tmc).await;
                if !phase_advance_restored {
                    defmt::warn!("phase advance: failed to restore ABN offset before disarm");
                }

                board.disarm_motor(runtime, tmc).await;
                board.emit_motor_disarm_event();

                if !phase_advance_restored {
                    return TmcControlRequestOutcome::skip_tick_tail();
                }

                defmt::info!("TMC4671 motor disabled");
            }
        }
        TmcRequest {
            kind: TmcRequestKind::Calibrate { oid },
            ..
        } => {
            if board.commissioning_blocked(runtime) {
                defmt::warn!("foci_calibrate rejected: commission in flight");
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_calibrate_failure::<Board, Spi>(
                    board,
                    oid,
                    CalibrationError::InternalError.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            if runtime.is_motor_enabled() {
                defmt::warn!("foci_calibrate rejected: motor already enabled");
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_calibrate_failure::<Board, Spi>(
                    board,
                    oid,
                    CalibrationError::AlreadyEnabled.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            runtime.clear_calibrated();
            runtime.set_commission_engine(
                CommissioningEngine::new_calibrate_for_tmc_with_gains_and_trace_seq(
                    oid,
                    board.tmc_state(),
                    runtime.stored_gains(),
                    board.default_current_filter_hz(),
                    board.next_commission_trace_seq(),
                ),
            );
        }
        #[cfg(feature = "dev")]
        TmcRequest {
            kind: TmcRequestKind::Init,
            ..
        } => {
            defmt::info!("TMC4671 init: starting open-loop stepper config");
            board.run_dev_init(tmc).await;
        }
        #[cfg(feature = "dev")]
        TmcRequest {
            kind: TmcRequestKind::TestSpin { revolutions },
            ..
        } => {
            board.run_dev_test_spin(tmc, revolutions).await;
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPidGains {
                    flux_p,
                    flux_i,
                    torque_p,
                    torque_i,
                },
            ..
        } => {
            let gains = InnerLoopGains {
                flux_p,
                flux_i,
                torque_p,
                torque_i,
            };
            let write_status = apply_live_inner_gains(runtime, tmc, gains).await;
            if !write_status.all_ok() {
                defmt::warn!("PID gains live write failed");
            }
            defmt::info!(
                "PID gains received: flux P={} I={}, torque P={} I={}",
                flux_p,
                flux_i,
                torque_p,
                torque_i,
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPositionGains {
                    position_p,
                    position_i,
                    velocity_p,
                    velocity_i,
                },
            ..
        } => {
            let gains = OuterLoopGains {
                position_p,
                position_i,
                velocity_p,
                velocity_i,
            };
            let write_status = apply_live_outer_gains(board.tmc_state(), tmc, gains).await;
            if !write_status.all_ok() {
                defmt::warn!("Position gains live write failed");
            }
            defmt::info!(
                "Position gains received: pos P={} I={}, vel P={} I={}",
                position_p,
                position_i,
                velocity_p,
                velocity_i,
            );
        }
        TmcRequest {
            kind: TmcRequestKind::SetVelocityFeedforward { enable, multiplier },
            ..
        } => {
            defmt::info!(
                "Velocity feedforward: {} multiplier={} (via queue)",
                enable,
                multiplier
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetVelocityTransientFeedforward {
                    enable,
                    lead_time_us,
                    gain_permille,
                    max_offset,
                    rate_hz,
                },
            ..
        } => {
            defmt::info!(
                "Velocity transient feedforward: {} lead_time_us={} \
                gain_permille={} max_offset={} rate_hz={} (via queue)",
                enable,
                lead_time_us,
                gain_permille,
                max_offset,
                rate_hz
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetAccelFeedforward {
                    enable,
                    accel_gain_permille,
                    decel_gain_permille,
                },
            ..
        } => {
            defmt::info!(
                "Acceleration feedforward: {} accel_gain_permille={} \
                decel_gain_permille={} (via queue)",
                enable,
                accel_gain_permille,
                decel_gain_permille
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetDecouplingFeedforward {
                    enable,
                    r_int,
                    l_int,
                    pole_pairs,
                    position_units_per_rev,
                    f_pwm_hz,
                    max_offset,
                },
            ..
        } => {
            defmt::info!(
                "Decoupling feedforward: {} r_int={} l_int={} pole_pairs={} \
                position_units_per_rev={} f_pwm_hz={} max_offset={} (via queue)",
                enable,
                r_int,
                l_int,
                pole_pairs,
                position_units_per_rev,
                f_pwm_hz,
                max_offset
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPositionLead {
                    enable,
                    gain_permille,
                    max_counts,
                },
            ..
        } => {
            defmt::info!(
                "Position lead: {} gain_permille={} max_counts={} (via queue)",
                enable,
                gain_permille,
                max_counts
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetPhaseAdvance {
                    enable,
                    gain_ppm,
                    max_counts,
                    deadband,
                },
            ..
        } => {
            defmt::info!(
                "Phase advance: {} gain_ppm={} max_counts={} deadband={} \
                (via queue)",
                enable,
                gain_ppm,
                max_counts,
                deadband
            );
        }
        TmcRequest {
            kind: TmcRequestKind::Commission { oid, profile },
            ..
        } => {
            if board.commissioning_blocked(runtime) {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::AlreadyRunning.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            if runtime.is_motor_enabled() {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::MotorEnabled.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let current_limit = board
                .tmc_state()
                .current_limit_reg
                .load(core::sync::atomic::Ordering::Relaxed);
            if current_limit == 0 {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::NoCurrent.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            runtime.clear_calibrated();
            runtime.set_commission_engine(
                CommissioningEngine::new_commission_for_tmc_with_trace_seq(
                    oid,
                    board.tmc_state(),
                    CommissionProfile::from_u8(profile),
                    board.next_commission_trace_seq(),
                ),
            );
        }
        TmcRequest {
            kind:
                TmcRequestKind::Tune {
                    oid,
                    profile,
                    mode,
                    inner_lambda_us,
                    theta_e_us,
                    current_ringing_count,
                    current_bandwidth_hz,
                    tau_e_us,
                    inner_warning_flags,
                    max_travel_mrev,
                    max_velocity_mrev_s,
                    max_duration_ms,
                    direction_mask,
                },
            ..
        } => {
            if board.commissioning_blocked(runtime) {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::AlreadyRunning.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            if !runtime.is_motor_enabled() || !runtime.is_calibrated() {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::EncoderNotAligned.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let current_limit = board
                .tmc_state()
                .current_limit_reg
                .load(core::sync::atomic::Ordering::Relaxed);
            if current_limit == 0 {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::NoCurrent.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before tune");
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::Spi.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let tmc_state = board.tmc_state();
            let entry_snapshot = {
                use core::sync::atomic::Ordering::Relaxed;
                let default_current_filter_hz = board.default_current_filter_hz();
                EntrySnapshot {
                    velocity_p: tmc_state.velocity_p_value.load(Relaxed),
                    velocity_i: tmc_state.velocity_i_value.load(Relaxed),
                    position_p: tmc_state.position_p_value.load(Relaxed),
                    position_i: tmc_state.position_i_value.load(Relaxed),
                    velocity_limit: tmc_state.velocity_limit_cfg.load(Relaxed),
                    voltage_limit: tmc_state.voltage_limit_cfg.load(Relaxed),
                    velocity_filter_hz: tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
                    torque_filter_hz: tmc_state
                        .torque_filter_hz
                        .load(Relaxed)
                        .unwrap_or(default_current_filter_hz),
                    position_filter_hz: tmc_state.position_filter_hz.load_or_zero(Relaxed),
                    flux_filter_hz: tmc_state
                        .flux_filter_hz
                        .load(Relaxed)
                        .unwrap_or(default_current_filter_hz),
                }
            };
            let outer_params = OuterTuneParams {
                profile: CommissionProfile::from_u8(profile),
                mode: OperatingMode::from_u8(mode),
                inner_lambda_us,
                theta_e_us,
                current_ringing: current_ringing_count > 1,
                current_bandwidth_hz,
                tau_e_us,
                inner_warning_flags,
                max_travel_mrev,
                max_velocity_mrev_s,
                max_duration_ms,
                direction_mask,
            };
            runtime.set_commission_engine(CommissioningEngine::new_outer_for_tmc_with_trace_seq(
                oid,
                tmc_state,
                outer_params,
                entry_snapshot,
                board.next_commission_trace_seq(),
            ));
        }
        TmcRequest {
            kind: TmcRequestKind::SelfTest { oid },
            ..
        } => {
            if board.commissioning_blocked(runtime) {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_selftest_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::AlreadyRunning.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            if runtime.is_motor_enabled() {
                defmt::warn!("foci_selftest rejected: motor enabled");
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_selftest_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::MotorEnabled.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            runtime.set_commission_engine(
                CommissioningEngine::new_selftest_for_tmc_with_trace_seq(
                    oid,
                    board.tmc_state(),
                    board.next_commission_trace_seq(),
                ),
            );
        }
        diagnostic_req @ TmcRequest {
            kind:
                TmcRequestKind::CurrentStepTest { .. }
                | TmcRequestKind::CurrentVectorStepTest { .. }
                | TmcRequestKind::CurrentTorqueSampleTest { .. }
                | TmcRequestKind::PositionTorqueOffsetSampleTest { .. }
                | TmcRequestKind::VoltageStepTest { .. }
                | TmcRequestKind::ResistanceTest { .. },
            ..
        } => {
            board
                .run_diagnostic_request(diagnostic_req, runtime, tmc)
                .await;
        }
        TmcRequest {
            kind:
                TmcRequestKind::SetMotionScale {
                    oid,
                    encoder_ppr,
                    planner_steps_per_rev,
                },
            ..
        } => {
            let scale =
                match crate::motion_scale::MotionScale::new(encoder_ppr, planner_steps_per_rev) {
                    Ok(scale) => scale,
                    Err(error) => {
                        board.report_motion_scale_rejection(oid, error.into());
                        return TmcControlRequestOutcome::shutdown(
                            ShutdownReason::RequestValidationFailed,
                            board.normal_tail(),
                        );
                    }
                };
            let rejection = if runtime.is_motor_enabled() {
                Some(crate::motion_scale::MotionScaleRejection::RuntimeEnabled)
            } else if board
                .tmc_state()
                .motor_armed
                .load(core::sync::atomic::Ordering::Relaxed)
            {
                Some(crate::motion_scale::MotionScaleRejection::MotorArmed)
            } else if !board.motion_scale_queue_empty() {
                Some(crate::motion_scale::MotionScaleRejection::QueueNotEmpty)
            } else if board
                .stepper_state()
                .timer_active_mirror
                .load(core::sync::atomic::Ordering::Relaxed)
            {
                Some(crate::motion_scale::MotionScaleRejection::TimerActive)
            } else {
                None
            };
            if let Some(rejection) = rejection {
                board.report_motion_scale_rejection(oid, rejection);
                return TmcControlRequestOutcome::shutdown(
                    ShutdownReason::RequestValidationFailed,
                    board.normal_tail(),
                );
            }
            let tmc_state = board.tmc_state();
            let stepper_state = board.stepper_state();
            if let Err(rejection) =
                apply_motion_scale(tmc, tmc_state, stepper_state, scale, |mapper| {
                    board.install_motion_scale_mapper(mapper)
                })
                .await
            {
                board.report_motion_scale_rejection(oid, rejection);
                return TmcControlRequestOutcome::shutdown_then_refresh_velocity_scale(
                    ShutdownReason::RequestValidationFailed,
                    board.normal_tail(),
                );
            }
            return TmcControlRequestOutcome::refresh_velocity_scale(board.normal_tail());
        }
        other => {
            if board.dispatch_register_request(other, tmc).await.is_err() {
                return TmcControlRequestOutcome::shutdown(
                    ShutdownReason::RequestValidationFailed,
                    board.normal_tail(),
                );
            }
        }
    }

    normal_outcome(board.normal_tail())
}

fn normal_outcome(tail: TmcControlRequestTail) -> TmcControlRequestOutcome {
    match tail {
        TmcControlRequestTail::FallThrough => TmcControlRequestOutcome::fall_through(),
        TmcControlRequestTail::SkipTickTail => TmcControlRequestOutcome::skip_tick_tail(),
    }
}

fn emit_calibrate_failure<Board, Spi>(board: &mut Board, oid: u8, status: u8)
where
    Spi: SpiDevice<u8>,
    Board: TmcRequestBoard<Spi>,
{
    board.emit_reply(ReplyPayload::CalibrateResult {
        oid,
        status,
        result: crate::commissioning::types::AdcCalResult {
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
        },
        diagnostics: CommissionDiagnostics::default(),
    });
}

fn emit_commission_phase_failure<Board, Spi>(board: &mut Board, oid: u8, status: u8)
where
    Spi: SpiDevice<u8>,
    Board: TmcRequestBoard<Spi>,
{
    board.emit_reply(ReplyPayload::CommissionPhase {
        oid,
        phase: 0,
        status,
        diagnostics: CommissionDiagnostics::default(),
    });
}

fn emit_selftest_failure<Board, Spi>(board: &mut Board, oid: u8, status: u8)
where
    Spi: SpiDevice<u8>,
    Board: TmcRequestBoard<Spi>,
{
    board.emit_reply(ReplyPayload::Selftest {
        oid,
        results: SelftestResults::default(),
        diagnostics: CommissionDiagnostics::default(),
        done_status: status,
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering::Relaxed;

    use embedded_hal_async::spi::{ErrorKind, ErrorType, Operation};
    use foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET;

    use crate::autotune::CommissionProfile;
    use crate::calibration::CalibrationError;
    use crate::commissioning::types::{AdcCalResult, EngineKind};
    use crate::state::stepper::{reset_all_stepper_channels, stepper_channel0};
    use crate::state::tmc::{reset_all_tmc_channels, tmc_channel0};
    use crate::tmc::{TmcRequestKind, set_auto_calibrate_on_enable_allowed};
    use crate::tmc_control::actions::{BoardAction, ShutdownReason, StepperAction};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockSpiError;

    impl embedded_hal_async::spi::Error for MockSpiError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    #[derive(Default)]
    struct RuntimeMockSpi {
        writes: std::vec::Vec<(u8, u32)>,
        write_attempts: std::vec::Vec<(u8, u32)>,
        read_requests: std::vec::Vec<u8>,
        fail_writes_to: Option<u8>,
    }

    impl RuntimeMockSpi {
        fn fail_writes_to(mut self, addr: u8) -> Self {
            self.fail_writes_to = Some(addr);
            self
        }
    }

    impl ErrorType for RuntimeMockSpi {
        type Error = MockSpiError;
    }

    impl SpiDevice<u8> for RuntimeMockSpi {
        async fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    Operation::Write(buf) if buf.len() == 5 => {
                        let addr = buf[0] & 0x7f;
                        let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                        self.write_attempts.push((addr, value));
                        if self.fail_writes_to == Some(addr) {
                            return Err(MockSpiError);
                        }
                        self.writes.push((addr, value));
                    }
                    Operation::Write(buf) if buf.len() == 1 => self.read_requests.push(buf[0]),
                    Operation::Read(buf) => buf.copy_from_slice(&0_u32.to_be_bytes()),
                    Operation::Write(_) | Operation::DelayNs(_) => {}
                    Operation::Transfer(read, write) => read.copy_from_slice(write),
                    Operation::TransferInPlace(_) => {}
                }
            }
            Ok(())
        }
    }

    struct RecordingBoard {
        channel: u8,
        commissioning_blocked: bool,
        normal_tail: TmcControlRequestTail,
        default_current_filter_hz: u16,
        motion_scale_queue_empty: bool,
        mapper_install_result: Result<(), MotionScaleMapperInstallError>,
        mapper_install_calls: u8,
        installed_mapper: Option<foci_klipper::pulse_mapper::PulseMapperConfig>,
        configured_during_mapper_install: bool,
        motion_scale_rejections: std::vec::Vec<(u8, crate::motion_scale::MotionScaleRejection)>,
        next_trace_seq: u32,
        register_result: Result<(), ()>,
        arm_result: Result<(), CommissionError>,
        replies: std::vec::Vec<ReplyPayload>,
        diagnostic_calls: u8,
        arm_calls: u8,
        disarm_calls: u8,
        disable_action_count: usize,
        motor_arm_events: u8,
        motor_disarm_events: u8,
    }

    impl RecordingBoard {
        fn new(normal_tail: TmcControlRequestTail) -> Self {
            reset_globals();
            Self {
                channel: 0,
                commissioning_blocked: false,
                normal_tail,
                default_current_filter_hz: 6000,
                motion_scale_queue_empty: true,
                mapper_install_result: Ok(()),
                mapper_install_calls: 0,
                installed_mapper: None,
                configured_during_mapper_install: false,
                motion_scale_rejections: std::vec::Vec::new(),
                next_trace_seq: 1,
                register_result: Ok(()),
                arm_result: Ok(()),
                replies: std::vec::Vec::new(),
                diagnostic_calls: 0,
                arm_calls: 0,
                disarm_calls: 0,
                disable_action_count: 0,
                motor_arm_events: 0,
                motor_disarm_events: 0,
            }
        }

        fn skip_tick_tail() -> Self {
            Self::new(TmcControlRequestTail::SkipTickTail)
        }

        fn fall_through() -> Self {
            Self::new(TmcControlRequestTail::FallThrough)
        }
    }

    impl TmcRequestBoard<RuntimeMockSpi> for RecordingBoard {
        fn channel(&self) -> u8 {
            self.channel
        }

        fn tmc_state(&self) -> &'static TmcChannelState {
            tmc_channel0()
        }

        fn stepper_state(&self) -> &'static StepperChannelState {
            stepper_channel0()
        }

        fn commissioning_blocked(&self, _runtime: &TmcControlRuntime) -> bool {
            self.commissioning_blocked
        }

        fn next_commission_trace_seq(&mut self) -> u32 {
            let seq = self.next_trace_seq;
            self.next_trace_seq = self.next_trace_seq.wrapping_add(1);
            seq
        }

        fn normal_tail(&self) -> TmcControlRequestTail {
            self.normal_tail
        }

        fn default_current_filter_hz(&self) -> u16 {
            self.default_current_filter_hz
        }

        fn motion_scale_queue_empty(&self) -> bool {
            self.motion_scale_queue_empty
        }

        fn install_motion_scale_mapper(
            &mut self,
            mapper: foci_klipper::pulse_mapper::PulseMapperConfig,
        ) -> Result<(), MotionScaleMapperInstallError> {
            self.mapper_install_calls = self.mapper_install_calls.saturating_add(1);
            self.installed_mapper = Some(mapper);
            self.configured_during_mapper_install = tmc_channel0().motion_scale_configured();
            self.mapper_install_result
        }

        fn report_motion_scale_rejection(
            &mut self,
            oid: u8,
            rejection: crate::motion_scale::MotionScaleRejection,
        ) {
            self.motion_scale_rejections.push((oid, rejection));
        }

        fn emit_reply(&mut self, payload: ReplyPayload) {
            self.replies.push(payload);
        }

        fn emit_motor_arm_event(&mut self) {
            self.motor_arm_events = self.motor_arm_events.saturating_add(1);
        }

        fn emit_motor_disarm_event(&mut self) {
            self.motor_disarm_events = self.motor_disarm_events.saturating_add(1);
        }

        fn apply_stepper_actions(&mut self, actions: BoardActions) {
            for action in actions {
                if matches!(action, BoardAction::Stepper(StepperAction::StopNow)) {
                    self.disable_action_count += 1;
                }
            }
        }

        async fn arm_motor(
            &mut self,
            _runtime: &mut TmcControlRuntime,
            _tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
            _post_action: StepperPostAction,
        ) -> Result<(), CommissionError> {
            self.arm_calls = self.arm_calls.saturating_add(1);
            self.arm_result
        }

        async fn disarm_motor(
            &mut self,
            _runtime: &mut TmcControlRuntime,
            _tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
        ) {
            self.disarm_calls = self.disarm_calls.saturating_add(1);
        }

        async fn dispatch_register_request(
            &mut self,
            _req: TmcRequest,
            _tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
        ) -> Result<(), ()> {
            self.register_result
        }

        async fn run_diagnostic_request(
            &mut self,
            _req: TmcRequest,
            _runtime: &mut TmcControlRuntime,
            _tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
        ) {
            self.diagnostic_calls = self.diagnostic_calls.saturating_add(1);
        }
    }

    fn reset_globals() {
        reset_all_tmc_channels();
        reset_all_stepper_channels();
        set_auto_calibrate_on_enable_allowed(0, false);
    }

    fn run_request_test(future: impl core::future::Future<Output = ()>) {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            future.await;
        });
    }

    fn stored_gains() -> crate::pid::InnerLoopGains {
        crate::pid::InnerLoopGains {
            flux_p: 11,
            flux_i: 12,
            torque_p: 13,
            torque_i: 14,
        }
    }

    fn tune_request() -> TmcRequest {
        TmcRequest::new(
            0,
            TmcRequestKind::Tune {
                oid: 4,
                profile: CommissionProfile::Balanced as u8,
                mode: 1,
                inner_lambda_us: 100,
                theta_e_us: 20,
                current_ringing_count: 0,
                current_bandwidth_hz: 1_000,
                tau_e_us: 10,
                inner_warning_flags: 0,
                max_travel_mrev: 1000,
                max_velocity_mrev_s: 6000,
                max_duration_ms: 3000,
                direction_mask: 3,
            },
        )
    }

    #[derive(Debug, PartialEq, Eq)]
    struct MotionScaleSnapshot {
        configured: bool,
        encoder_ppr: Option<u32>,
        encoder_counts_per_rev: Option<u32>,
        planner_steps_per_rev: Option<u32>,
        tmc_grid: Option<u32>,
        physical_step_width: Option<u32>,
    }

    fn motion_scale_snapshot() -> MotionScaleSnapshot {
        let tmc_state = tmc_channel0();
        let stepper_state = stepper_channel0();
        MotionScaleSnapshot {
            configured: tmc_state.motion_scale_configured(),
            encoder_ppr: tmc_state.encoder_ppr(),
            encoder_counts_per_rev: tmc_state.encoder_counts_per_rev(),
            planner_steps_per_rev: stepper_state.planner_steps_per_rev(tmc_state),
            tmc_grid: tmc_state.tmc_grid(),
            physical_step_width: tmc_state.physical_step_width(),
        }
    }

    #[derive(Clone, Copy)]
    enum MotionScaleRevalidationRejection {
        InvalidScale(crate::motion_scale::MotionScaleRejection),
        RuntimeEnabled,
        MotorArmedMirror,
        PhysicalQueueNonempty,
        TimerActive,
    }

    async fn assert_motion_scale_revalidation_rejected(
        rejection: MotionScaleRevalidationRejection,
    ) {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut board = RecordingBoard::fall_through();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1000, 3200)
            .unwrap();
        let before = motion_scale_snapshot();
        let mut encoder_ppr = 2000;

        let expected = match rejection {
            MotionScaleRevalidationRejection::InvalidScale(expected) => {
                encoder_ppr = 0;
                expected
            }
            MotionScaleRevalidationRejection::RuntimeEnabled => {
                runtime.set_motor_enabled_for_test(true);
                crate::motion_scale::MotionScaleRejection::RuntimeEnabled
            }
            MotionScaleRevalidationRejection::MotorArmedMirror => {
                tmc_channel0().motor_armed.store(true, Relaxed);
                crate::motion_scale::MotionScaleRejection::MotorArmed
            }
            MotionScaleRevalidationRejection::PhysicalQueueNonempty => {
                board.motion_scale_queue_empty = false;
                crate::motion_scale::MotionScaleRejection::QueueNotEmpty
            }
            MotionScaleRevalidationRejection::TimerActive => {
                stepper_channel0().timer_active_mirror.store(true, Relaxed);
                crate::motion_scale::MotionScaleRejection::TimerActive
            }
        };

        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let outcome = handle_tmc_control_request(
            TmcRequest::new(
                0,
                TmcRequestKind::SetMotionScale {
                    oid: 7,
                    encoder_ppr,
                    planner_steps_per_rev: 6400,
                },
            ),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        )
        .await;
        let spi = tmc.into_inner();

        assert!(spi.write_attempts.is_empty());
        assert_eq!(motion_scale_snapshot(), before);
        assert_eq!(
            outcome.shutdown,
            Some(ShutdownReason::RequestValidationFailed)
        );
        assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
        assert!(!outcome.should_refresh_velocity_scale);
        assert_eq!(board.arm_calls, 0);
        assert_eq!(board.motion_scale_rejections, std::vec![(7, expected)]);
    }

    async fn assert_motion_scale_spi_failure(
        failed_register: u8,
        expected_attempts: &[(u8, u32)],
        expected_completed_writes: &[(u8, u32)],
    ) {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut board = RecordingBoard::fall_through();
        let before = motion_scale_snapshot();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(failed_register));

        let outcome = handle_tmc_control_request(
            TmcRequest::new(
                0,
                TmcRequestKind::SetMotionScale {
                    oid: 7,
                    encoder_ppr: 4000,
                    planner_steps_per_rev: 3200,
                },
            ),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(spi.write_attempts, expected_attempts);
        assert_eq!(spi.writes, expected_completed_writes);
        assert_eq!(motion_scale_snapshot(), before);
        assert!(!tmc_channel0().motion_scale_configured());
        assert_eq!(tmc_channel0().encoder_ppr(), None);
        assert_eq!(tmc_channel0().encoder_counts_per_rev(), None);
        assert_eq!(
            stepper_channel0().planner_steps_per_rev(tmc_channel0()),
            None
        );
        assert_eq!(tmc_channel0().tmc_grid(), None);
        assert_eq!(tmc_channel0().physical_step_width(), None);
        assert_eq!(
            outcome.shutdown,
            Some(ShutdownReason::RequestValidationFailed)
        );
        assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
        assert!(outcome.should_refresh_velocity_scale);
        let expected_rejection =
            if failed_register == foci_tmc4671::registers::ABN_DECODER_PPR.addr() {
                crate::motion_scale::MotionScaleRejection::EncoderRegisterWriteFailed
            } else {
                crate::motion_scale::MotionScaleRejection::StepWidthRegisterWriteFailed
            };
        assert_eq!(
            board.motion_scale_rejections,
            std::vec![(7, expected_rejection)]
        );
        assert_eq!(board.arm_calls, 0);
    }

    fn assert_calibrate_failure(reply: &ReplyPayload, oid: u8, status: u8) {
        match reply {
            ReplyPayload::CalibrateResult {
                oid: actual_oid,
                status: actual_status,
                result:
                    AdcCalResult {
                        adc_i0,
                        adc_i1,
                        encoder_count,
                        ..
                    },
                ..
            } => {
                assert_eq!(*actual_oid, oid);
                assert_eq!(*actual_status, status);
                assert_eq!(*adc_i0, 0);
                assert_eq!(*adc_i1, 0);
                assert_eq!(*encoder_count, 0);
            }
            _ => panic!("expected CalibrateResult reply"),
        }
    }

    fn assert_commission_phase_failure(reply: &ReplyPayload, oid: u8, status: u8) {
        match reply {
            ReplyPayload::CommissionPhase {
                oid: actual_oid,
                phase,
                status: actual_status,
                ..
            } => {
                assert_eq!(*actual_oid, oid);
                assert_eq!(*phase, 0);
                assert_eq!(*actual_status, status);
            }
            _ => panic!("expected CommissionPhase reply"),
        }
    }

    fn assert_selftest_failure(reply: &ReplyPayload, oid: u8, status: u8) {
        match reply {
            ReplyPayload::Selftest {
                oid: actual_oid,
                results,
                done_status,
                ..
            } => {
                assert_eq!(*actual_oid, oid);
                assert_eq!(*done_status, status);
                assert_eq!(results.iter().count(), 0);
            }
            _ => panic!("expected Selftest reply"),
        }
    }

    #[test]
    fn auto_calibrate_on_enable_uses_stored_gains() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_stored_gains(stored_gains());
            let mut board = RecordingBoard::skip_tick_tail();
            set_auto_calibrate_on_enable_allowed(0, true);
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::SetEnable(true)),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);
            assert_eq!(
                runtime.pending_commission_kind(),
                Some(EngineKind::Calibrate)
            );
            assert_eq!(
                runtime.pending_calibrate_inner_gains_for_test(),
                Some(stored_gains())
            );
        });
    }

    #[test]
    fn explicit_calibrate_uses_stored_gains() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_stored_gains(stored_gains());
            let mut board = RecordingBoard::fall_through();
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::Calibrate { oid: 9 }),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
            assert_eq!(
                runtime.pending_commission_kind(),
                Some(EngineKind::Calibrate)
            );
            assert_eq!(
                runtime.pending_calibrate_inner_gains_for_test(),
                Some(stored_gains())
            );
        });
    }

    #[test]
    fn auto_calibrate_without_stored_gains_still_starts_engine() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let mut board = RecordingBoard::skip_tick_tail();
            set_auto_calibrate_on_enable_allowed(0, true);
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::SetEnable(true)),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);
            assert_eq!(
                runtime.pending_commission_kind(),
                Some(EngineKind::Calibrate)
            );
            assert_eq!(
                runtime.pending_calibrate_inner_gains_for_test(),
                Some(crate::pid::DEFAULT_INNER_GAINS)
            );
        });
    }

    #[test]
    fn commissioning_blocked_emits_existing_failure_replies() {
        run_request_test(async {
            let requests = [
                TmcRequest::new(0, TmcRequestKind::Calibrate { oid: 1 }),
                TmcRequest::new(0, TmcRequestKind::Commission { oid: 2, profile: 0 }),
                tune_request(),
                TmcRequest::new(0, TmcRequestKind::SelfTest { oid: 5 }),
            ];

            for req in requests {
                let mut runtime = TmcControlRuntime::new_for_channel(0);
                let mut board = RecordingBoard::skip_tick_tail();
                board.commissioning_blocked = true;
                let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

                let _outcome = handle_tmc_control_request(
                    req,
                    TmcPreRequestDisposition::ContinueRequest,
                    &mut runtime,
                    &mut tmc,
                    &mut board,
                )
                .await;

                assert_eq!(board.replies.len(), 1);
                match board.replies.first().unwrap() {
                    reply @ ReplyPayload::CalibrateResult { .. } => assert_calibrate_failure(
                        reply,
                        1,
                        CalibrationError::InternalError.status_code(),
                    ),
                    reply @ ReplyPayload::CommissionPhase { oid: 2, .. } => {
                        assert_commission_phase_failure(
                            reply,
                            2,
                            CommissionError::AlreadyRunning.status_code(),
                        );
                    }
                    reply @ ReplyPayload::CommissionPhase { oid: 4, .. } => {
                        assert_commission_phase_failure(
                            reply,
                            4,
                            CommissionError::AlreadyRunning.status_code(),
                        );
                    }
                    reply @ ReplyPayload::Selftest { .. } => assert_selftest_failure(
                        reply,
                        5,
                        CommissionError::AlreadyRunning.status_code(),
                    ),
                    _ => panic!("unexpected reply"),
                }
            }
        });
    }

    #[test]
    fn commissioning_block_false_preserves_openffboard_overwrite_behavior() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_commission_engine(
                crate::commission::CommissioningEngine::new_selftest_for_tmc_with_trace_seq(
                    8,
                    tmc_channel0(),
                    1,
                ),
            );
            let mut board = RecordingBoard::fall_through();
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::Calibrate { oid: 9 }),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert!(board.replies.is_empty());
            assert_eq!(
                runtime.pending_commission_kind(),
                Some(EngineKind::Calibrate)
            );
            assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
        });
    }

    #[test]
    fn tune_restore_phase_advance_failure_emits_spi_failure() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_motor_enabled_for_test(true);
            runtime.set_calibrated_for_test(true);
            runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
            let mut board = RecordingBoard::skip_tick_tail();
            tmc_channel0().current_limit_reg.store(1, Relaxed);
            let mut tmc = Tmc4671Spi::new(
                RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
            );

            let outcome = handle_tmc_control_request(
                tune_request(),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(board.replies.len(), 1);
            assert_commission_phase_failure(
                board.replies.first().unwrap(),
                4,
                CommissionError::Spi.status_code(),
            );
            assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);
            assert!(runtime.phase_advance_active_for_test());
        });
    }

    #[test]
    fn set_motion_scale_success_uses_board_tail() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let mut board = RecordingBoard::fall_through();
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(
                    0,
                    TmcRequestKind::SetMotionScale {
                        oid: 7,
                        encoder_ppr: 4000,
                        planner_steps_per_rev: 3200,
                    },
                ),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
            assert!(outcome.should_refresh_velocity_scale);
            assert_eq!(outcome.shutdown, None);
            assert!(tmc_channel0().motion_scale_configured());
            assert_eq!(board.mapper_install_calls, 1);
            let mapper = board.installed_mapper.unwrap();
            assert_eq!(mapper.planner_steps_per_rev(), 3200);
            assert_eq!(mapper.tmc_grid(), 4096);
            assert_eq!(mapper.step_width(), 16);
            assert!(!board.configured_during_mapper_install);
            assert_eq!(board.arm_calls, 0);
        });
    }

    #[test]
    fn set_motion_scale_mapper_install_failure_shuts_down_before_publication() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let mut board = RecordingBoard::fall_through();
            board.mapper_install_result = Err(MotionScaleMapperInstallError);
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(
                    0,
                    TmcRequestKind::SetMotionScale {
                        oid: 7,
                        encoder_ppr: 4000,
                        planner_steps_per_rev: 3200,
                    },
                ),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;
            let spi = tmc.into_inner();

            assert_eq!(
                spi.writes,
                std::vec![
                    (foci_tmc4671::registers::ABN_DECODER_PPR.addr(), 16000),
                    (foci_tmc4671::registers::STEP_WIDTH.addr(), 16),
                ]
            );
            assert_eq!(board.mapper_install_calls, 1);
            assert!(!board.configured_during_mapper_install);
            assert!(!tmc_channel0().motion_scale_configured());
            assert_eq!(
                board.motion_scale_rejections,
                std::vec![(
                    7,
                    crate::motion_scale::MotionScaleRejection::MapperInstallFailed,
                )]
            );
            assert_eq!(
                outcome.shutdown,
                Some(ShutdownReason::RequestValidationFailed)
            );
            assert!(outcome.should_refresh_velocity_scale);
        });
    }

    #[test]
    fn set_motion_scale_rejects_invalid_scale_before_spi_or_publication() {
        run_request_test(async {
            assert_motion_scale_revalidation_rejected(
                MotionScaleRevalidationRejection::InvalidScale(
                    crate::motion_scale::MotionScaleRejection::EncoderPprZero,
                ),
            )
            .await;
        });
    }

    #[test]
    fn set_motion_scale_rejects_runtime_enabled_before_spi_or_publication() {
        run_request_test(async {
            assert_motion_scale_revalidation_rejected(
                MotionScaleRevalidationRejection::RuntimeEnabled,
            )
            .await;
        });
    }

    #[test]
    fn set_motion_scale_rejects_motor_armed_mirror_before_spi_or_publication() {
        run_request_test(async {
            assert_motion_scale_revalidation_rejected(
                MotionScaleRevalidationRejection::MotorArmedMirror,
            )
            .await;
        });
    }

    #[test]
    fn set_motion_scale_rejects_physical_queue_before_spi_or_publication() {
        run_request_test(async {
            assert_motion_scale_revalidation_rejected(
                MotionScaleRevalidationRejection::PhysicalQueueNonempty,
            )
            .await;
        });
    }

    #[test]
    fn set_motion_scale_rejects_active_timer_before_spi_or_publication() {
        run_request_test(async {
            assert_motion_scale_revalidation_rejected(
                MotionScaleRevalidationRejection::TimerActive,
            )
            .await;
        });
    }

    #[test]
    fn set_motion_scale_abn_decoder_ppr_failure_shuts_down_without_publication() {
        run_request_test(async {
            assert_motion_scale_spi_failure(
                foci_tmc4671::registers::ABN_DECODER_PPR.addr(),
                &[(foci_tmc4671::registers::ABN_DECODER_PPR.addr(), 16000)],
                &[],
            )
            .await;
        });
    }

    #[test]
    fn set_motion_scale_step_width_failure_shuts_down_without_publication() {
        run_request_test(async {
            assert_motion_scale_spi_failure(
                foci_tmc4671::registers::STEP_WIDTH.addr(),
                &[
                    (foci_tmc4671::registers::ABN_DECODER_PPR.addr(), 16000),
                    (foci_tmc4671::registers::STEP_WIDTH.addr(), 16),
                ],
                &[(foci_tmc4671::registers::ABN_DECODER_PPR.addr(), 16000)],
            )
            .await;
        });
    }

    #[test]
    fn generic_register_failure_uses_board_tail() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let mut board = RecordingBoard::fall_through();
            board.register_result = Err(());
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 1000 }),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
            assert_eq!(
                outcome.shutdown,
                Some(ShutdownReason::RequestValidationFailed)
            );
            assert!(!outcome.should_refresh_velocity_scale);
        });
    }

    #[test]
    fn pre_request_diagnostic_disposition_uses_board_tail() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let mut board = RecordingBoard::fall_through();
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 1000 }),
                TmcPreRequestDisposition::RunDiagnostic(TmcRequest::new(
                    0,
                    TmcRequestKind::CurrentStepTest {
                        oid: 1,
                        axis: crate::tmc::CurrentStepAxis::Torque,
                        target: 10,
                        duration_ms: 20,
                        voltage_limit: 12000,
                    },
                )),
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(board.diagnostic_calls, 1);
            assert!(board.replies.is_empty());
            assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
        });
    }

    #[test]
    fn resistance_test_runs_diagnostic_request() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let mut board = RecordingBoard::fall_through();
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::ResistanceTest { oid: 7, detail: 1 }),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(board.diagnostic_calls, 1);
            assert!(board.replies.is_empty());
            assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
        });
    }

    #[test]
    fn failed_arm_disarms_once_and_returns_skip_tick_tail() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_calibrated_for_test(true);
            let mut board = RecordingBoard::fall_through();
            board.arm_result = Err(CommissionError::Spi);
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::SetEnable(true)),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(board.arm_calls, 1);
            assert_eq!(board.disarm_calls, 1);
            assert_eq!(board.motor_arm_events, 0);
            assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);
        });
    }

    #[test]
    fn tune_entry_snapshot_uses_default_current_filters_when_unconfigured() {
        run_request_test(async {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_motor_enabled_for_test(true);
            runtime.set_calibrated_for_test(true);
            let mut board = RecordingBoard::skip_tick_tail();
            tmc_channel0().current_limit_reg.store(1, Relaxed);
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_tmc_control_request(
                tune_request(),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            )
            .await;

            assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);
            let engine = runtime
                .take_commission_engine()
                .expect("tune should start outer engine");
            let snapshot = engine
                .entry_snapshot()
                .expect("outer engine should carry entry snapshot");
            assert_eq!(snapshot.velocity_filter_hz, 0);
            assert_eq!(snapshot.torque_filter_hz, 6000);
            assert_eq!(snapshot.position_filter_hz, 0);
            assert_eq!(snapshot.flux_filter_hz, 6000);
        });
    }

    #[test]
    fn success_paths_preserve_both_normal_tail_policies() {
        run_request_test(async {
            for normal_tail in [
                TmcControlRequestTail::SkipTickTail,
                TmcControlRequestTail::FallThrough,
            ] {
                let mut runtime = TmcControlRuntime::new_for_channel(0);
                let mut board = RecordingBoard::new(normal_tail);
                let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

                let outcome = handle_tmc_control_request(
                    TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 1000 }),
                    TmcPreRequestDisposition::ContinueRequest,
                    &mut runtime,
                    &mut tmc,
                    &mut board,
                )
                .await;

                assert_eq!(outcome.tail, normal_tail);
                assert_eq!(outcome.shutdown, None);
            }
        });
    }
}
