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
    BoardActions, ShutdownReason, TmcControlRequestOutcome, TmcControlRequestTail,
};
use crate::tmc_control::dispatch::RegisterRequestError;
use crate::tmc_control::helpers::{
    LiveGainWriteStatus, apply_live_inner_gains, apply_live_outer_gains, apply_motion_scale,
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

// The breakaway campaign carries the whole admitted campaign by value; boxing is
// unavailable in this heapless crate, matching the campaign's own terminal enums.
#[allow(clippy::large_enum_variant)]
enum TuneRoute {
    Acceptance(crate::commission_dispatch::acceptance::AcceptanceMatrixPreflight),
    Breakaway(crate::commissioning::outer::velocity::breakaway_campaign::BreakawayCampaign),
    StageCResume(crate::commission_dispatch::stage_c::StageCPreflight),
}

/// Map a refused Stage-C resume onto the terminal the host will render.
///
/// Every rejection here names retained state that disagrees with the request,
/// so each keeps a plan digest where one exists. The empty-slot case is not a
/// rejection and is built separately.
fn stage_c_resume_rejection_terminal(
    run_sequence: u32,
    rejection: crate::commission_dispatch::stage_c::StageCPreflightRejection,
    retained_plan_digest: u64,
) -> crate::commissioning::types::VelocityIntegralTerminal {
    use crate::commission_dispatch::stage_c::{
        STAGE_C_CAUSE_PLAN_MISMATCH, StageCPreflightRejection, failed_stage_c_terminal,
        missing_authority_stage_c_terminal,
    };

    match rejection {
        StageCPreflightRejection::NoTransitionCapableOperatingPoint(failure) => {
            failed_stage_c_terminal(run_sequence, failure)
        }
        // Both remaining rejections mean retained authority exists and the
        // request disagrees with it, so both name the retained plan. Only the
        // empty slot reports no plan at all, which is what lets the host tell
        // "nothing to resume" apart from "that is not what was retained".
        StageCPreflightRejection::RejectedPlanMismatch { retained, .. } => {
            let mut terminal = missing_authority_stage_c_terminal(run_sequence);
            terminal.cause = STAGE_C_CAUSE_PLAN_MISMATCH;
            terminal.plan_digest = retained.plan_digest;
            terminal
        }
        StageCPreflightRejection::InvalidPlan => {
            let mut terminal = missing_authority_stage_c_terminal(run_sequence);
            terminal.cause = STAGE_C_CAUSE_PLAN_MISMATCH;
            terminal.plan_digest = retained_plan_digest;
            terminal
        }
    }
}

/// A board could not install the validated mapper into its idle P3 scheduler.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MotionScaleMapperInstallError;

/// Reply-only capabilities for rejecting queued work during commissioning.
pub trait CommissionBusyReplySink {
    /// Return the board's normal request-tail policy.
    fn normal_tail(&self) -> TmcControlRequestTail;
    /// Emit one established commissioning-family busy response.
    fn emit_reply(&mut self, payload: ReplyPayload);
    /// Emit one established diagnostic not-ready response.
    fn emit_busy_diagnostic(&mut self, request: TmcRequest);
}

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
    /// Return the configured PWM frequency that defines the Stage-C I ladder.
    fn pwm_freq_hz(&self) -> u32;
    /// Return the board-verified velocity-limit evidence capability.
    fn velocity_limit_evidence_capability(&self) -> u32;
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
    /// Record one canonical Stage B record in the trace stream. No-op unless a
    /// board provides a `trace`-gated implementation.
    fn record_velocity_sweep_trace_record(
        &mut self,
        record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        let _ = record;
    }
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
    ) -> Result<(), RegisterRequestError>;

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

/// Reject a queued request while commissioning exclusively owns the motor.
///
/// This synchronous path intentionally receives no runtime, SPI, or GPIO
/// capability, so it cannot disturb the active engine or its deadline.
pub fn handle_commission_wait_reject<Sink>(
    req: TmcRequest,
    sink: &mut Sink,
) -> TmcControlRequestOutcome
where
    Sink: CommissionBusyReplySink,
{
    match req {
        TmcRequest {
            kind: TmcRequestKind::SetEnable(true) | TmcRequestKind::DumpRegisters { .. },
            ..
        } => {}
        TmcRequest {
            kind: TmcRequestKind::Calibrate { oid },
            ..
        } => sink.emit_reply(calibrate_failure_payload(
            oid,
            CalibrationError::InternalError.status_code(),
        )),
        TmcRequest {
            kind: TmcRequestKind::SelfTest { oid },
            ..
        } => sink.emit_reply(selftest_failure_payload(
            oid,
            CommissionError::AlreadyRunning.status_code(),
        )),
        TmcRequest {
            kind: TmcRequestKind::Commission { oid, .. } | TmcRequestKind::Tune { oid, .. },
            ..
        } => sink.emit_reply(commission_phase_failure_payload(
            oid,
            CommissionError::AlreadyRunning.status_code(),
        )),
        diagnostic_req @ TmcRequest {
            kind:
                TmcRequestKind::CurrentStepTest { .. }
                | TmcRequestKind::CurrentVectorStepTest { .. }
                | TmcRequestKind::CurrentTorqueSampleTest { .. }
                | TmcRequestKind::PositionTorqueOffsetSampleTest { .. }
                | TmcRequestKind::VoltageStepTest { .. }
                | TmcRequestKind::ResistanceTest { .. },
            ..
        } => sink.emit_busy_diagnostic(diagnostic_req),
        #[cfg(feature = "trace-velocity-limit-latch-test")]
        diagnostic_req @ TmcRequest {
            kind: TmcRequestKind::VelocityLimitLatchTest { .. },
            ..
        } => sink.emit_busy_diagnostic(diagnostic_req),
        #[cfg(feature = "dev")]
        TmcRequest {
            kind:
                TmcRequestKind::Read { .. } | TmcRequestKind::Init | TmcRequestKind::TestSpin { .. },
            ..
        } => {}
        _ => defmt::warn!("non-reject request reached commission busy-only path"),
    }
    normal_outcome(sink.normal_tail())
}

async fn pi_representation_available<Spi: SpiDevice<u8>>(
    tmc: &mut Tmc4671Spi<Spi>,
    state: &TmcChannelState,
) -> bool {
    crate::tmc_control::pi_representation::verify_pi_representation_spi(tmc, state)
        .await
        .is_ok()
}

/// Handle one board-routed request with board-wide Stage C retention.
pub async fn handle_tmc_control_request_with_stages<Spi, Board>(
    req: TmcRequest,
    pre_request: TmcPreRequestDisposition,
    runtime: &mut TmcControlRuntime,
    stage_c_candidate: &mut crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot,
    tmc: &mut Tmc4671Spi<Spi>,
    board: &mut Board,
) -> TmcControlRequestOutcome
where
    Spi: SpiDevice<u8>,
    Board: TmcRequestBoard<Spi>,
{
    let autotune_action = req.autotune_action;
    match pre_request {
        TmcPreRequestDisposition::ContinueRequest => {}
        TmcPreRequestDisposition::RequestHandled => {
            return normal_outcome(board.normal_tail());
        }
        TmcPreRequestDisposition::RunDiagnostic(diagnostic_req) => {
            if !pi_representation_available(tmc, board.tmc_state()).await {
                defmt::error!("Diagnostic rejected: PI representation unavailable");
                return TmcControlRequestOutcome::skip_tick_tail();
            }
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
                if !pi_representation_available(tmc, board.tmc_state()).await {
                    defmt::error!("Motor enable rejected: PI representation unavailable");
                    return TmcControlRequestOutcome::skip_tick_tail();
                }
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
                    // A routine same-channel re-enable may auto-calibrate after the
                    // required disarm/cooling interval; preserve that channel's
                    // Stage-C authority across the alignment lifecycle.
                    stage_c_candidate.preserve_for_same_channel_calibration(board.channel());
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

                if let Some(post_action) = runtime.prepare_enable_request(board.tmc_state()) {
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
                    runtime.take_commission_engine();
                    return TmcControlRequestOutcome::skip_tick_tail();
                }

                runtime.take_commission_engine_after_handled_stop();
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
            if !pi_representation_available(tmc, board.tmc_state()).await {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_calibrate_failure::<Board, Spi>(
                    board,
                    oid,
                    CalibrationError::ChipResetDetected.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            // Host homing explicitly calibrates after the required disarm/cooling
            // interval. Preserve same-channel Stage-C authority across that
            // alignment lifecycle.
            stage_c_candidate.preserve_for_same_channel_calibration(board.channel());
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
            let write_status = apply_live_inner_gains(runtime, board.tmc_state(), tmc, gains).await;
            if let Some(outcome) = live_gain_failure_outcome(write_status) {
                defmt::warn!("PID gains live write failed");
                return outcome;
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
            if let Some(outcome) = live_gain_failure_outcome(write_status) {
                defmt::warn!("Position gains live write failed");
                return outcome;
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
            if !pi_representation_available(tmc, board.tmc_state()).await {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::ChipResetDetected.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let current_limit = board
                .tmc_state()
                .load_current_limit(core::sync::atomic::Ordering::Relaxed);
            if current_limit.raw() == 0 {
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
            if !pi_representation_available(tmc, board.tmc_state()).await {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_selftest_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::ChipResetDetected.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            stage_c_candidate.invalidate_for_unrelated_commissioning();
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
                    inner_warning_flags,
                    requested_velocity_mrev_s,
                    machine_velocity_ceiling_mrev_s,
                    requested_velocity_source,
                    max_stroke_travel_mrev,
                    settle_travel_reserve_mrev,
                    negative_position_headroom_mrev,
                    positive_position_headroom_mrev,
                    max_duration_ms,
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
            if !pi_representation_available(tmc, board.tmc_state()).await {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::ChipResetDetected.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            let current_limit = board
                .tmc_state()
                .load_current_limit(core::sync::atomic::Ordering::Relaxed);
            if current_limit.raw() == 0 {
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
            let run_sequence = board.next_commission_trace_seq();
            let Some(encoder_counts_per_rev) = board.tmc_state().encoder_counts_per_rev() else {
                board
                    .tmc_state()
                    .commission_in_flight
                    .store(false, core::sync::atomic::Ordering::Relaxed);
                emit_commission_phase_failure::<Board, Spi>(
                    board,
                    oid,
                    CommissionError::InvalidSchedule.status_code(),
                );
                return TmcControlRequestOutcome::skip_tick_tail();
            };
            let request = crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest {
                requested_velocity_mrev_s,
                machine_velocity_ceiling_mrev_s,
                requested_velocity_source,
                max_stroke_travel_mrev,
                settle_travel_reserve_mrev,
                negative_position_headroom_mrev,
                positive_position_headroom_mrev,
                max_duration_ms,
            };
            let tune_route = match autotune_action {
                action @ (crate::tmc::AutotuneAction::MatrixAscending
                | crate::tmc::AutotuneAction::MatrixDescending) => {
                    let matrix_config =
                        crate::commission_dispatch::stage_c::StageCPreflightConfig {
                            encoder_counts_per_rev,
                            torque_limit: current_limit.raw(),
                            limit_evidence_capability: board.velocity_limit_evidence_capability(),
                            channel: board.channel(),
                            pwm_hz: board.pwm_freq_hz(),
                        };
                    let result =
                        crate::commission_dispatch::acceptance::preflight_acceptance_matrix(
                            action,
                            request,
                            &matrix_config,
                            run_sequence,
                            stage_c_candidate,
                        );
                    let preflight = match result {
                        Ok(preflight) => preflight,
                        Err(rejection) => {
                            let terminal = crate::commission_dispatch::acceptance::
                                failed_acceptance_matrix_terminal(
                                    run_sequence,
                                    rejection,
                                    stage_c_candidate,
                                );
                            board.record_velocity_sweep_trace_record(
                                crate::commissioning::outer::velocity::acceptance_reporting::
                                    encode_acceptance_terminal(&terminal),
                            );
                            board.emit_reply(ReplyPayload::AcceptanceMatrixTerminal {
                                oid,
                                terminal,
                            });
                            board
                                .tmc_state()
                                .commission_in_flight
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            return TmcControlRequestOutcome::skip_tick_tail();
                        }
                    };
                    let workflow = crate::commissioning::outer::velocity::integral_plan::
                        CommissioningWorkflowPlan::new_acceptance_matrix(
                            run_sequence,
                            preflight.plan.order,
                            preflight.plan.nominal_workflow_ms,
                            preflight.plan.maximum_workflow_ms,
                        )
                        .expect("a valid matrix plan has a valid workflow envelope");
                    board.emit_reply(ReplyPayload::CommissioningWorkflowPlan {
                        oid,
                        plan: workflow,
                    });
                    for record in crate::commissioning::outer::velocity::acceptance_reporting::
                        encode_acceptance_plan(run_sequence, preflight.plan)
                    {
                        board.record_velocity_sweep_trace_record(record);
                    }
                    board.emit_reply(ReplyPayload::AcceptanceMatrixPlan {
                        oid,
                        run_sequence,
                        plan: preflight.plan,
                    });
                    TuneRoute::Acceptance(preflight)
                }
                crate::tmc::AutotuneAction::StageCResume => {
                    let config = crate::commission_dispatch::stage_c::StageCPreflightConfig {
                        encoder_counts_per_rev,
                        torque_limit: current_limit.raw(),
                        limit_evidence_capability: board.velocity_limit_evidence_capability(),
                        channel: board.channel(),
                        pwm_hz: board.pwm_freq_hz(),
                    };
                    // `Ok(None)` is the admission path's "proceed to Stage B"
                    // signal. A resume has no Stage B to fall back to, and
                    // running a campaign here would actuate the motor for
                    // minutes when the operator asked to reproduce a plan that
                    // does not exist. Translate it here rather than changing
                    // what the shared preflight means.
                    let refusal = match crate::commission_dispatch::stage_c::
                        preflight_stage_c_request(request, &config, run_sequence, stage_c_candidate)
                    {
                        Ok(Some(preflight)) => Ok(preflight),
                        Ok(None) => Err(crate::commission_dispatch::stage_c::
                            missing_authority_stage_c_terminal(run_sequence)),
                        Err(rejection) => Err(stage_c_resume_rejection_terminal(
                            run_sequence,
                            rejection,
                            stage_c_candidate
                                .authority()
                                .map_or(0, |authority| authority.identity.plan_digest),
                        )),
                    };
                    let preflight = match refusal {
                        Ok(preflight) => preflight,
                        Err(terminal) => {
                            board.record_velocity_sweep_trace_record(
                                crate::commissioning::outer::velocity::integral_reporting::
                                    encode_velocity_integral_terminal(&terminal),
                            );
                            board.emit_reply(ReplyPayload::VelocityIntegralRefusal {
                                oid,
                                terminal,
                            });
                            board
                                .tmc_state()
                                .commission_in_flight
                                .store(false, core::sync::atomic::Ordering::Relaxed);
                            return TmcControlRequestOutcome::skip_tick_tail();
                        }
                    };
                    let workflow = crate::commissioning::outer::velocity::integral_plan::
                        CommissioningWorkflowPlan::new(
                            run_sequence,
                            crate::commissioning::outer::velocity::integral_plan::
                                StageCPlanShape::StageCResume,
                            0,
                            preflight.plan.maximum_workflow_ms,
                        )
                        .expect("a preflighted resume has a valid workflow envelope");
                    board.emit_reply(ReplyPayload::CommissioningWorkflowPlan {
                        oid,
                        plan: workflow,
                    });
                    // The workflow envelope only discloses the shape and its
                    // durations. The exact plan is a separate reply, and the
                    // host refuses Stage-C evidence that arrives before it, so
                    // it has to be on the wire before the engine takes a stroke.
                    for record in crate::commissioning::outer::velocity::integral_reporting::
                        encode_velocity_integral_plan_trace(run_sequence, preflight.plan)
                    {
                        board.record_velocity_sweep_trace_record(record);
                    }
                    board.emit_reply(ReplyPayload::VelocityIntegralPlan {
                        oid,
                        run_sequence,
                        plan: preflight.plan,
                    });
                    TuneRoute::StageCResume(preflight)
                }
                crate::tmc::AutotuneAction::BreakawaySeeded => {
                    let (campaign, workflow) = match crate::commissioning::outer::velocity::
                        breakaway_campaign::plan_breakaway_campaign_command(
                            request,
                            encoder_counts_per_rev,
                            current_limit.raw(),
                            board.velocity_limit_evidence_capability(),
                            board.pwm_freq_hz(),
                            board.channel(),
                            run_sequence,
                        ) {
                            Ok(planned) => planned,
                            Err(error) => {
                                board.tmc_state().commission_in_flight.store(
                                    false,
                                    core::sync::atomic::Ordering::Relaxed,
                                );
                                emit_commission_phase_failure::<Board, Spi>(
                                    board,
                                    oid,
                                    error.status_code(),
                                );
                                return TmcControlRequestOutcome::skip_tick_tail();
                            }
                        };
                    board.emit_reply(ReplyPayload::CommissioningWorkflowPlan {
                        oid,
                        plan: workflow,
                    });
                    TuneRoute::Breakaway(campaign)
                }
            };
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
                let outer_gains = match tmc_state.load_outer_gains(Relaxed) {
                    Ok(gains) => gains,
                    Err(_) => {
                        tmc_state.commission_in_flight.store(false, Relaxed);
                        emit_commission_phase_failure::<Board, Spi>(
                            board,
                            oid,
                            CommissionError::VelocityValidationFailed.status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                };
                EntrySnapshot {
                    velocity_p: outer_gains.velocity_p,
                    velocity_i: outer_gains.velocity_i,
                    position_p: outer_gains.position_p,
                    position_i: outer_gains.position_i,
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
                slot_order: req.autotune_action.slot_order(),
                profile: CommissionProfile::from_u8(profile),
                mode: OperatingMode::from_u8(mode),
                inner_lambda_us,
                theta_e_us,
                current_ringing: current_ringing_count > 1,
                current_bandwidth_hz,
                inner_warning_flags,
                requested_velocity_mrev_s,
                machine_velocity_ceiling_mrev_s,
                requested_velocity_source,
                max_stroke_travel_mrev,
                settle_travel_reserve_mrev,
                negative_position_headroom_mrev,
                positive_position_headroom_mrev,
                max_duration_ms,
            };
            {
                match tune_route {
                    TuneRoute::Acceptance(preflight) => {
                        runtime.set_commission_engine(
                            CommissioningEngine::new_outer_for_tmc_with_acceptance_preflight(
                                oid,
                                tmc_state,
                                outer_params,
                                entry_snapshot,
                                preflight,
                            ),
                        );
                    }
                    TuneRoute::StageCResume(preflight) => {
                        runtime.set_commission_engine(
                            CommissioningEngine::new_outer_for_tmc_with_stage_c_preflight(
                                oid,
                                tmc_state,
                                outer_params,
                                entry_snapshot,
                                preflight,
                            ),
                        );
                    }
                    TuneRoute::Breakaway(campaign) => {
                        let engine =
                            match CommissioningEngine::new_outer_for_tmc_with_breakaway_campaign(
                                oid,
                                tmc_state,
                                outer_params,
                                entry_snapshot,
                                campaign,
                            ) {
                                Ok(engine) => engine,
                                Err(error) => {
                                    board
                                        .tmc_state()
                                        .commission_in_flight
                                        .store(false, core::sync::atomic::Ordering::Relaxed);
                                    emit_commission_phase_failure::<Board, Spi>(
                                        board,
                                        oid,
                                        error.status_code(),
                                    );
                                    return TmcControlRequestOutcome::skip_tick_tail();
                                }
                            };
                        runtime.set_commission_engine(engine);
                        crate::diagnostic::begin_breakaway_campaign();
                    }
                }
            }
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
            stage_c_candidate.invalidate_for_unrelated_commissioning();
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
            if !pi_representation_available(tmc, board.tmc_state()).await {
                defmt::error!("Diagnostic rejected: PI representation unavailable");
                return TmcControlRequestOutcome::skip_tick_tail();
            }
            board
                .run_diagnostic_request(diagnostic_req, runtime, tmc)
                .await;
        }
        #[cfg(feature = "trace-velocity-limit-latch-test")]
        diagnostic_req @ TmcRequest {
            kind: TmcRequestKind::VelocityLimitLatchTest { .. },
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
            if let Err(error) = board.dispatch_register_request(other, tmc).await {
                let reason = match error {
                    RegisterRequestError::CurrentLimitInvalid => {
                        ShutdownReason::CurrentLimitInvalid
                    }
                    RegisterRequestError::Other => ShutdownReason::RequestValidationFailed,
                };
                return TmcControlRequestOutcome::shutdown(reason, board.normal_tail());
            }
        }
    }

    normal_outcome(board.normal_tail())
}

/// Handle one request with temporary empty Stage-C retention.
///
/// Board integration switches to [`handle_tmc_control_request_with_stages`] when
/// its persistent Stage-C task-local is added.
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
    let mut stage_c_candidate =
        crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot::new();
    handle_tmc_control_request_with_stages(
        req,
        pre_request,
        runtime,
        &mut stage_c_candidate,
        tmc,
        board,
    )
    .await
}

pub(super) fn live_gain_failure_outcome(
    status: LiveGainWriteStatus,
) -> Option<TmcControlRequestOutcome> {
    if status.all_ok() {
        None
    } else {
        Some(TmcControlRequestOutcome::shutdown(
            ShutdownReason::LiveGainApplyFailed,
            TmcControlRequestTail::SkipTickTail,
        ))
    }
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
    board.emit_reply(calibrate_failure_payload(oid, status));
}

fn calibrate_failure_payload(oid: u8, status: u8) -> ReplyPayload {
    ReplyPayload::CalibrateResult {
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
    }
}

fn emit_commission_phase_failure<Board, Spi>(board: &mut Board, oid: u8, status: u8)
where
    Spi: SpiDevice<u8>,
    Board: TmcRequestBoard<Spi>,
{
    board.emit_reply(commission_phase_failure_payload(oid, status));
}

fn commission_phase_failure_payload(oid: u8, status: u8) -> ReplyPayload {
    ReplyPayload::CommissionPhase {
        oid,
        phase: 0,
        status,
        diagnostics: CommissionDiagnostics::default(),
    }
}

fn emit_selftest_failure<Board, Spi>(board: &mut Board, oid: u8, status: u8)
where
    Spi: SpiDevice<u8>,
    Board: TmcRequestBoard<Spi>,
{
    board.emit_reply(selftest_failure_payload(oid, status));
}

fn selftest_failure_payload(oid: u8, status: u8) -> ReplyPayload {
    ReplyPayload::Selftest {
        oid,
        results: SelftestResults::default(),
        diagnostics: CommissionDiagnostics::default(),
        done_status: status,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "trace-velocity-sweep")]
    use crate::commissioning::outer::velocity::integral_plan::StageCAcquisition;
    use core::sync::atomic::Ordering::Relaxed;

    use embedded_hal_async::spi::{ErrorKind, ErrorType, Operation};
    use foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET;

    use crate::autotune::CommissionProfile;
    use crate::calibration::CalibrationError;
    #[cfg(feature = "trace-velocity-sweep")]
    use crate::commission_dispatch::test_support::breakaway_campaign_engine;
    use crate::commission_dispatch::test_support::current_limit;
    use crate::commissioning::types::{AdcCalResult, EngineKind};
    use crate::state::stepper::{reset_all_stepper_channels, stepper_channel0};
    use crate::state::tmc::{reset_all_tmc_channels, tmc_channel0};
    use crate::tmc::{TmcRequestKind, set_auto_calibrate_on_enable_allowed};
    use crate::tmc_control::actions::{BoardAction, ShutdownReason, StepperAction};
    use foci_firmware_macros::fixture_test;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockSpiError;

    impl embedded_hal_async::spi::Error for MockSpiError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    struct RuntimeMockSpi {
        writes: std::vec::Vec<(u8, u32)>,
        write_attempts: std::vec::Vec<(u8, u32)>,
        read_requests: std::vec::Vec<u8>,
        fail_writes_to: Option<u8>,
        read_value: u32,
    }

    impl Default for RuntimeMockSpi {
        fn default() -> Self {
            Self {
                writes: std::vec::Vec::new(),
                write_attempts: std::vec::Vec::new(),
                read_requests: std::vec::Vec::new(),
                fail_writes_to: None,
                read_value: foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12,
            }
        }
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
                    Operation::Read(buf) => buf.copy_from_slice(&self.read_value.to_be_bytes()),
                    Operation::Write(_) | Operation::DelayNs(_) => {}
                    Operation::Transfer(read, write) => read.copy_from_slice(write),
                    Operation::TransferInPlace(_) => {}
                }
            }
            Ok(())
        }
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    fn trace_request_future_size_stays_bounded() {
        use crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot;

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut stage_c = StageCCandidateSlot::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let mut board = RecordingBoard::skip_tick_tail();
        let request = TmcRequest::new(0, TmcRequestKind::SetEnable(false));

        let future = handle_tmc_control_request_with_stages(
            request,
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        );

        assert!(core::mem::size_of_val(&future) <= 3_232);
    }

    struct RecordingBoard {
        channel: u8,
        commissioning_blocked: bool,
        normal_tail: TmcControlRequestTail,
        default_current_filter_hz: u16,
        velocity_limit_evidence_capability: u32,
        motion_scale_queue_empty: bool,
        mapper_install_result: Result<(), MotionScaleMapperInstallError>,
        mapper_install_calls: u8,
        installed_mapper: Option<foci_klipper::pulse_mapper::PulseMapperConfig>,
        configured_during_mapper_install: bool,
        motion_scale_rejections: std::vec::Vec<(u8, crate::motion_scale::MotionScaleRejection)>,
        next_trace_seq: u32,
        register_result: Result<(), crate::tmc_control::dispatch::RegisterRequestError>,
        arm_result: Result<(), CommissionError>,
        replies: std::vec::Vec<ReplyPayload>,
        #[cfg(feature = "trace-velocity-sweep")]
        velocity_sweep_records:
            std::vec::Vec<crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord>,
        busy_diagnostic_requests: std::vec::Vec<TmcRequest>,
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
            tmc_channel0().mark_pi_representation_established();
            Self {
                channel: 0,
                commissioning_blocked: false,
                normal_tail,
                default_current_filter_hz: 6000,
                velocity_limit_evidence_capability:
                    crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
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
                #[cfg(feature = "trace-velocity-sweep")]
                velocity_sweep_records: std::vec::Vec::new(),
                busy_diagnostic_requests: std::vec::Vec::new(),
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

        fn pwm_freq_hz(&self) -> u32 {
            25_000
        }

        fn velocity_limit_evidence_capability(&self) -> u32 {
            self.velocity_limit_evidence_capability
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

        #[cfg(feature = "trace-velocity-sweep")]
        fn record_velocity_sweep_trace_record(
            &mut self,
            record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
        ) {
            self.velocity_sweep_records.push(record);
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
        ) -> Result<(), crate::tmc_control::dispatch::RegisterRequestError> {
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

    impl CommissionBusyReplySink for RecordingBoard {
        fn normal_tail(&self) -> TmcControlRequestTail {
            self.normal_tail
        }

        fn emit_reply(&mut self, payload: ReplyPayload) {
            self.replies.push(payload);
        }

        fn emit_busy_diagnostic(&mut self, request: TmcRequest) {
            self.busy_diagnostic_requests.push(request);
        }
    }

    fn reset_globals() {
        reset_all_tmc_channels();
        reset_all_stepper_channels();
        set_auto_calibrate_on_enable_allowed(0, false);
    }

    fn stored_gains() -> crate::pid::InnerLoopGains {
        crate::pid::InnerLoopGains {
            flux_p: crate::pid::p_gain(11),
            flux_i: crate::pid::i_gain(12),
            torque_p: crate::pid::p_gain(13),
            torque_i: crate::pid::i_gain(14),
        }
    }

    #[cfg(feature = "trace-velocity-sweep")]
    fn retained_stage_c_authority()
    -> crate::commissioning::outer::velocity::integral_reproduction::StageCAuthoritySummary {
        use crate::commissioning::outer::velocity::integral_reproduction::{
            StageCAuthoritySummary, StageCPlanIdentity,
        };

        StageCAuthoritySummary {
            identity: StageCPlanIdentity {
                channel: 0,
                build_features: 1,
                schema_revision: crate::commissioning::outer::velocity::integral_plan::STAGE_C_EVIDENCE_SCHEMA_REVISION,
                plan_digest: 2,
            },
            acquisition: StageCAcquisition::ReproducedStageB(3),
            final_p: 724,
            directional_memberships: [0x001c_0000; 2],
            stage_b_joint_membership: 0x001c_0000,
            joint_membership: 0x0004_0000,
            directional_validity: [
                crate::commissioning::outer::velocity::reproduction::StageBDirectionalValidity::ReproducedRegionCoversFinalP;
                2
            ],
            reduced_margin: true,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
            limit_evidence_capability:
                crate::commissioning::outer::velocity::current_guard::VelocityLimitEvidenceCapability::BestEffortPolling,
            directional_pooled_intervals: [[100, 120], [-130, -100]],
            planned_velocity_mrev_s: 2_929,
            target_velocity_rpm: 176,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            positive_rung_count: 5,
            family_size: 28,
            maximum_workflow_ms: 90_888,
            probe_constrained_test_point: true,
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
                inner_warning_flags: 0,
                requested_velocity_mrev_s: 6000,
                machine_velocity_ceiling_mrev_s: 10_000,
                requested_velocity_source: crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
                max_stroke_travel_mrev: 2500,
                settle_travel_reserve_mrev: 250,
                negative_position_headroom_mrev: 7025,
                positive_position_headroom_mrev: 7025,
                max_duration_ms: 3000,
            },
        )
    }

    fn tune_request_with_action(action: crate::tmc::AutotuneAction) -> TmcRequest {
        let mut request = tune_request();
        request.autotune_action = action;
        request
    }

    #[cfg(feature = "trace-velocity-sweep")]
    fn velocity_sweep_request()
    -> crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest {
        let TmcRequestKind::Tune {
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s,
            requested_velocity_source,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev,
            negative_position_headroom_mrev,
            positive_position_headroom_mrev,
            max_duration_ms,
            ..
        } = tune_request().kind
        else {
            unreachable!()
        };
        crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest {
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s,
            requested_velocity_source,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev,
            negative_position_headroom_mrev,
            positive_position_headroom_mrev,
            max_duration_ms,
        }
    }

    #[fixture_test]
    async fn commission_wait_reject_preserves_active_run_and_emits_busy_reply() {
        enum ExpectedReply {
            None,
            Calibrate(u8),
            Selftest(u8),
            Commission(u8),
            Diagnostic,
        }

        let cases = [
            (
                TmcRequest::new(0, TmcRequestKind::SetEnable(true)),
                ExpectedReply::None,
            ),
            (
                TmcRequest::new(0, TmcRequestKind::DumpRegisters { oid: 2 }),
                ExpectedReply::None,
            ),
            (
                TmcRequest::new(
                    0,
                    TmcRequestKind::CurrentStepTest {
                        oid: 3,
                        axis: crate::tmc::CurrentStepAxis::Torque,
                        target: 64,
                        duration_ms: 5,
                        voltage_limit: 512,
                    },
                ),
                ExpectedReply::Diagnostic,
            ),
            (
                TmcRequest::new(0, TmcRequestKind::Calibrate { oid: 4 }),
                ExpectedReply::Calibrate(4),
            ),
            (
                TmcRequest::new(0, TmcRequestKind::SelfTest { oid: 5 }),
                ExpectedReply::Selftest(5),
            ),
            (
                TmcRequest::new(0, TmcRequestKind::Commission { oid: 6, profile: 1 }),
                ExpectedReply::Commission(6),
            ),
            (tune_request(), ExpectedReply::Commission(4)),
        ];

        for (request, expected) in cases {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(
                11,
                tmc_channel0(),
            ));
            let active_kind = runtime.pending_commission_kind();
            let mut board = RecordingBoard::skip_tick_tail();
            tmc_channel0().commission_in_flight.store(true, Relaxed);
            let tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let outcome = handle_commission_wait_reject(request.clone(), &mut board);

            assert_eq!(runtime.pending_commission_kind(), active_kind);
            assert!(tmc_channel0().commission_in_flight.load(Relaxed));
            assert!(tmc.into_inner().write_attempts.is_empty());
            assert_eq!(board.arm_calls, 0);
            assert_eq!(board.disarm_calls, 0);
            assert_eq!(board.diagnostic_calls, 0);
            assert_eq!(board.disable_action_count, 0);
            assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);

            match expected {
                ExpectedReply::None => {
                    assert!(board.replies.is_empty());
                    assert!(board.busy_diagnostic_requests.is_empty());
                }
                ExpectedReply::Calibrate(oid) => {
                    assert_calibrate_failure(
                        board.replies.first().expect("calibrate busy reply"),
                        oid,
                        CalibrationError::InternalError.status_code(),
                    );
                }
                ExpectedReply::Selftest(oid) => {
                    assert_selftest_failure(
                        board.replies.first().expect("selftest busy reply"),
                        oid,
                        CommissionError::AlreadyRunning.status_code(),
                    );
                }
                ExpectedReply::Commission(oid) => {
                    assert_commission_phase_failure(
                        board.replies.first().expect("commission busy reply"),
                        oid,
                        CommissionError::AlreadyRunning.status_code(),
                    );
                }
                ExpectedReply::Diagnostic => {
                    assert!(board.replies.is_empty());
                    assert_eq!(board.busy_diagnostic_requests, std::vec![request]);
                }
            }
        }
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

    #[fixture_test]
    async fn auto_calibrate_on_enable_uses_stored_gains() {
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
    }

    #[fixture_test]
    async fn explicit_calibrate_uses_stored_gains() {
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
    }

    #[fixture_test]
    async fn auto_calibrate_without_stored_gains_still_starts_engine() {
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
    }

    #[fixture_test]
    async fn commissioning_blocked_emits_existing_failure_replies() {
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
                reply @ ReplyPayload::Selftest { .. } => {
                    assert_selftest_failure(reply, 5, CommissionError::AlreadyRunning.status_code())
                }
                _ => panic!("unexpected reply"),
            }
        }
    }

    #[fixture_test]
    async fn commissioning_admission_refuses_missing_pi_representation() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut board = RecordingBoard::skip_tick_tail();
        board
            .tmc_state()
            .store_current_limit(current_limit(2_816), Relaxed);
        board.tmc_state().clear_pi_representation_established();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let outcome = handle_tmc_control_request(
            TmcRequest::new(0, TmcRequestKind::Commission { oid: 2, profile: 0 }),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);
        assert_eq!(runtime.pending_commission_kind(), None);
        assert_eq!(board.replies.len(), 1);
        assert_commission_phase_failure(
            &board.replies[0],
            2,
            CommissionError::ChipResetDetected.status_code(),
        );
        let spi = tmc.into_inner();
        assert!(spi.write_attempts.is_empty());
        assert!(spi.read_requests.is_empty());
    }

    #[fixture_test]
    async fn commissioning_block_false_preserves_openffboard_overwrite_behavior() {
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
    }

    #[fixture_test]
    async fn tune_restore_phase_advance_failure_emits_spi_failure() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
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

        // Production routes the breakaway campaign, so the workflow envelope is
        // emitted before the phase-advance restore runs; its failure then adds
        // the commission-phase failure. Both builds emit the same two replies.
        assert_eq!(board.replies.len(), 2);
        assert!(matches!(
            board.replies[0],
            ReplyPayload::CommissioningWorkflowPlan { .. }
        ));
        assert_commission_phase_failure(
            board.replies.get(1).unwrap(),
            4,
            CommissionError::Spi.status_code(),
        );
        assert_eq!(outcome.tail, TmcControlRequestTail::SkipTickTail);
        assert!(runtime.phase_advance_active_for_test());
    }

    #[fixture_test]
    async fn production_tune_routes_breakaway_seeded_to_campaign() {
        use crate::commissioning::outer::velocity::integral_plan::StageCPlanShape;

        let mut board = RecordingBoard::skip_tick_tail();
        let tmc_state = tmc_channel0();
        tmc_state
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_state.store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request(
            tune_request_with_action(crate::tmc::AutotuneAction::BreakawaySeeded),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(board.replies.len(), 1);
        let ReplyPayload::CommissioningWorkflowPlan { plan: workflow, .. } = board.replies[0]
        else {
            panic!("breakaway admission must emit exactly the workflow envelope")
        };
        assert_eq!(workflow.shape, StageCPlanShape::BreakawaySeededPThenI);
        assert!(runtime.is_commission_active());
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn breakaway_breadcrumb_begins_when_workflow_is_installed() {
        use crate::commissioning::outer::velocity::integral_plan::StageCPlanShape;

        crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
        let mut board = RecordingBoard::skip_tick_tail();
        let tmc_state = tmc_channel0();
        tmc_state
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_state.store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request(
            tune_request_with_action(crate::tmc::AutotuneAction::BreakawaySeeded),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        )
        .await;

        // The campaign discloses only its command-level envelope before
        // construction; the probe's Planned evidence is recorded on its first
        // engine step, not at admission, so exactly one reply is emitted.
        assert_eq!(board.replies.len(), 1);
        let ReplyPayload::CommissioningWorkflowPlan { plan: workflow, .. } = board.replies[0]
        else {
            panic!("breakaway workflow envelope must be the only admission reply")
        };
        assert_eq!(workflow.shape, StageCPlanShape::BreakawaySeededPThenI);
        assert!(workflow.nominal_workflow_ms > 0);
        assert!(workflow.nominal_workflow_ms <= workflow.maximum_workflow_ms);
        assert!(runtime.is_commission_active());
        assert_eq!(
            crate::diagnostic::current_breakaway_breadcrumb_for_test(),
            Some(crate::diagnostic::BreakawayBreadcrumb {
                checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignInstalled,
                interrupted: true,
            })
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn encoder_not_ready_tune_fails_as_commission_phase_not_velocity_sweep_terminal() {
        // Reach the Stage-B preflight with the encoder domain unestablished:
        // the current limit is stored (so the NoCurrent pre-check passes) but
        // motion scale is never configured, so `encoder_counts_per_rev()` is
        // None and the preflight rejects before any workflow plan is emitted.
        let mut board = RecordingBoard::skip_tick_tail();
        let tmc_state = tmc_channel0();
        tmc_state.store_current_limit(current_limit(2_816), Relaxed);
        assert_eq!(tmc_state.encoder_counts_per_rev(), None);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request(
            tune_request_with_action(crate::tmc::AutotuneAction::BreakawaySeeded),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        )
        .await;

        // The encoder-not-ready preflight must surface as a commission-phase
        // failure the host already handles (autotune.py sets commissioning
        // error_code and raises), not a planless velocity-sweep terminal reply
        // that the host's "envelope before evidence" gate rejects with a cryptic
        // protocol error while discarding the real cause.
        assert_eq!(board.replies.len(), 1);
        assert_commission_phase_failure(
            &board.replies[0],
            4,
            CommissionError::InvalidSchedule.status_code(),
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn matrix_request_without_acceptance_point_fails_without_motion_writes() {
        use crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot;
        use crate::commissioning::types::{AcceptanceMatrixCause, AcceptanceMatrixOutcome};

        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut stage_c = StageCCandidateSlot::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request_with_stages(
            tune_request_with_action(crate::tmc::AutotuneAction::MatrixAscending),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(board.replies.len(), 1);
        let ReplyPayload::AcceptanceMatrixTerminal { terminal, .. } = board.replies[0] else {
            panic!("missing point must use the matrix terminal namespace")
        };
        assert_eq!(terminal.outcome, AcceptanceMatrixOutcome::Failed);
        assert_eq!(
            terminal.cause,
            AcceptanceMatrixCause::MissingAcceptancePoint
        );
        assert!(!runtime.is_commission_active());
        assert_eq!(board.velocity_sweep_records.len(), 1);
        let spi = tmc.into_inner();
        assert!(spi.write_attempts.iter().all(|(addr, _)| {
            *addr != foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr()
                && *addr != foci_tmc4671::registers::PID_VELOCITY_TARGET.addr()
                && *addr != foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
        }));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn admitted_matrix_discloses_exact_plan_before_engine_execution() {
        use crate::commissioning::outer::velocity::integral_reproduction::{
            AcceptanceMatrixProgress, StageCCandidateSlot,
        };

        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut stage_c = StageCCandidateSlot::new();
        stage_c.install_acceptance_point(
            crate::commission_dispatch::acceptance::test_acceptance_point(
                velocity_sweep_request(),
                AcceptanceMatrixProgress::ReadyAscending,
            ),
        );
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request_with_stages(
            tune_request_with_action(crate::tmc::AutotuneAction::MatrixAscending),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(board.replies.len(), 2);
        let ReplyPayload::CommissioningWorkflowPlan { plan: workflow, .. } = board.replies[0]
        else {
            panic!("matrix workflow envelope must be first")
        };
        assert_eq!(
            workflow.shape,
            crate::commissioning::outer::velocity::integral_plan::StageCPlanShape::
                AcceptanceMatrixAscending
        );
        assert_eq!(workflow.maximum_workflow_ms, 66_456);
        let ReplyPayload::AcceptanceMatrixPlan { plan, .. } = board.replies[1] else {
            panic!("matrix plan must be disclosed before execution")
        };
        assert_eq!(plan.target_rpm, [16, 33, 66, 132, 263]);
        assert_eq!(board.velocity_sweep_records.len(), 3);
        assert!(runtime.is_commission_active());
        assert_eq!(
            stage_c.acceptance_point().unwrap().progress,
            AcceptanceMatrixProgress::ReadyAscending
        );
    }

    /// Build the exact Stage-C plan a resume of `velocity_sweep_request` reproduces.
    #[cfg(feature = "trace-velocity-sweep")]
    fn resumable_stage_c_plan()
    -> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
        use crate::commissioning::outer::OuterMotionBudget;
        use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
        use crate::commissioning::outer::velocity::integral_plan::{
            StageCAuthority, plan_velocity_integral_response,
        };
        use crate::commissioning::outer::velocity::reproduction::StageBDirectionalValidity;
        use crate::commissioning::outer::velocity::sweep_plan::plan_velocity_sweep;

        let request = velocity_sweep_request();
        let stage_b = plan_velocity_sweep(
            request,
            4_000,
            2_816,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        )
        .expect("valid Stage-B plan");
        plan_velocity_integral_response(
            StageCAuthority {
                channel: 0,
                acquisition: StageCAcquisition::ReproducedStageB(stage_b.digest()),
                directional_memberships: [0x000e_0000; 2],
                stage_b_joint_membership: 0x000e_0000,
                joint_membership: 0x000e_0000,
                directional_validity: [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2],
                reduced_margin: false,
                directional_pooled_intervals: [
                    [15_000_000, 17_000_000],
                    [-19_000_000, -17_000_000],
                ],
            },
            stage_b.execution_plan(stage_b.digest()),
            OuterMotionBudget {
                requested_velocity_mrev_s: request.requested_velocity_mrev_s,
                machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
                requested_velocity_source: request.requested_velocity_source,
                max_stroke_travel_mrev: request.max_stroke_travel_mrev,
                settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
                negative_position_headroom_mrev: request.negative_position_headroom_mrev,
                positive_position_headroom_mrev: request.positive_position_headroom_mrev,
                max_duration_ms: request.max_duration_ms,
            },
            25_000,
            4_000,
        )
        .expect("valid Stage-C plan")
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn admitted_resume_discloses_exact_plan_before_engine_execution() {
        use crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot;

        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut stage_c = StageCCandidateSlot::new();
        let plan = resumable_stage_c_plan();
        crate::commission_dispatch::stage_c::install_authority_from_plan(0, &mut stage_c, plan);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request_with_stages(
            tune_request_with_action(crate::tmc::AutotuneAction::StageCResume),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(board.replies.len(), 2);
        let ReplyPayload::CommissioningWorkflowPlan { plan: workflow, .. } = board.replies[0]
        else {
            panic!("resume workflow envelope must be first")
        };
        assert_eq!(
            workflow.shape,
            crate::commissioning::outer::velocity::integral_plan::StageCPlanShape::StageCResume
        );
        assert_eq!(workflow.maximum_workflow_ms, plan.maximum_workflow_ms);
        // The exact plan, not just the envelope: motion evidence that reaches
        // the host before this reply has no plan to be assembled against.
        let ReplyPayload::VelocityIntegralPlan {
            plan: disclosed, ..
        } = board.replies[1]
        else {
            panic!("exact Stage-C plan must be disclosed before execution")
        };
        assert_eq!(disclosed, plan);
        assert_eq!(board.velocity_sweep_records.len(), 3);
        assert!(runtime.is_commission_active());
        assert_eq!(
            stage_c
                .authority()
                .map(|authority| authority.identity.plan_digest),
            Some(plan.digest())
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn resume_plan_mismatch_names_the_retained_plan_without_motion_writes() {
        use crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot;
        use crate::commissioning::types::VelocityIntegralOutcome;

        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut stage_c = StageCCandidateSlot::new();
        let plan = resumable_stage_c_plan();
        let retained =
            crate::commission_dispatch::stage_c::install_authority_from_plan(0, &mut stage_c, plan);
        let mut request = tune_request_with_action(crate::tmc::AutotuneAction::StageCResume);
        let TmcRequestKind::Tune {
            max_stroke_travel_mrev,
            ..
        } = &mut request.kind
        else {
            unreachable!()
        };
        *max_stroke_travel_mrev += 1;
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request_with_stages(
            request,
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(board.replies.len(), 1);
        // The refusal namespace, not the ordinary terminal: an ordinary terminal
        // expands to leading evidence fragments describing a run that never ran.
        let ReplyPayload::VelocityIntegralRefusal { terminal, .. } = board.replies[0] else {
            panic!("a refused resume must use the Stage-C refusal namespace")
        };
        assert_eq!(terminal.outcome, VelocityIntegralOutcome::Failed);
        assert_eq!(
            terminal.cause,
            crate::commission_dispatch::stage_c::STAGE_C_CAUSE_PLAN_MISMATCH
        );
        // The host tells "nothing retained" from "not what was retained" by the
        // digest, so a mismatch has to name the plan it disagreed with.
        assert_eq!(terminal.plan_digest, retained.identity.plan_digest);
        assert_ne!(terminal.plan_digest, 0);
        assert!(!runtime.is_commission_active());
        assert_eq!(stage_c.authority(), Some(retained));
        let spi = tmc.into_inner();
        assert!(spi.write_attempts.iter().all(|(addr, _)| {
            *addr != foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr()
                && *addr != foci_tmc4671::registers::PID_VELOCITY_TARGET.addr()
                && *addr != foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
        }));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn resume_of_an_unplannable_request_still_names_the_retained_plan() {
        use crate::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot;
        use crate::commissioning::types::VelocityIntegralOutcome;

        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut stage_c = StageCCandidateSlot::new();
        let retained = crate::commission_dispatch::stage_c::install_authority_from_plan(
            0,
            &mut stage_c,
            resumable_stage_c_plan(),
        );
        // No Stage-B plan reconstructs from this, so the preflight refuses
        // before it can compare digests. The refusal still has to say which
        // retained plan the operator failed to resume.
        let mut request = tune_request_with_action(crate::tmc::AutotuneAction::StageCResume);
        let TmcRequestKind::Tune {
            max_duration_ms, ..
        } = &mut request.kind
        else {
            unreachable!()
        };
        *max_duration_ms = 0;
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request_with_stages(
            request,
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(board.replies.len(), 1);
        // The refusal namespace, not the ordinary terminal: an ordinary terminal
        // expands to leading evidence fragments describing a run that never ran.
        let ReplyPayload::VelocityIntegralRefusal { terminal, .. } = board.replies[0] else {
            panic!("a refused resume must use the Stage-C refusal namespace")
        };
        assert_eq!(terminal.outcome, VelocityIntegralOutcome::Failed);
        assert_eq!(
            terminal.cause,
            crate::commission_dispatch::stage_c::STAGE_C_CAUSE_PLAN_MISMATCH
        );
        assert_eq!(terminal.plan_digest, retained.identity.plan_digest);
        assert_ne!(terminal.plan_digest, 0);
        assert!(!runtime.is_commission_active());
        assert_eq!(stage_c.authority(), Some(retained));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn matrix_order_mismatch_preserves_point_without_motion() {
        use crate::commissioning::outer::velocity::integral_reproduction::{
            AcceptanceMatrixProgress, StageCCandidateSlot,
        };
        use crate::commissioning::types::AcceptanceMatrixCause;

        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut stage_c = StageCCandidateSlot::new();
        let point = crate::commission_dispatch::acceptance::test_acceptance_point(
            velocity_sweep_request(),
            AcceptanceMatrixProgress::ReadyDescending,
        );
        stage_c.install_acceptance_point(point);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request_with_stages(
            tune_request_with_action(crate::tmc::AutotuneAction::MatrixAscending),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        )
        .await;

        let ReplyPayload::AcceptanceMatrixTerminal { terminal, .. } = board.replies[0] else {
            panic!("wrong order must use the matrix terminal namespace")
        };
        assert_eq!(terminal.cause, AcceptanceMatrixCause::MatrixOrderMismatch);
        assert_eq!(stage_c.acceptance_point(), Some(point));
        assert!(!runtime.is_commission_active());
        assert!(tmc.into_inner().write_attempts.iter().all(|(addr, _)| {
            *addr != foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr()
                && *addr != foci_tmc4671::registers::PID_VELOCITY_TARGET.addr()
                && *addr != foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
        }));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[fixture_test]
    async fn same_channel_explicit_calibration_preserves_stage_c_authority() {
        use crate::commissioning::outer::velocity::integral_reproduction::{
            StageCCandidateSlot, StageCSlotState,
        };

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut stage_c = StageCCandidateSlot::new();
        stage_c.install_authority(retained_stage_c_authority());
        let mut board = RecordingBoard::skip_tick_tail();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        handle_tmc_control_request_with_stages(
            TmcRequest::new(0, TmcRequestKind::Calibrate { oid: 7 }),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut stage_c,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(stage_c.state(), StageCSlotState::AuthorityOnly);
        assert_eq!(stage_c.authority(), Some(retained_stage_c_authority()));
        assert_eq!(
            runtime.pending_commission_kind(),
            Some(EngineKind::Calibrate)
        );
    }

    #[fixture_test]
    async fn set_motion_scale_success_uses_board_tail() {
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
    }

    #[fixture_test]
    async fn set_motion_scale_mapper_install_failure_shuts_down_before_publication() {
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
    }

    #[fixture_test]
    async fn set_motion_scale_rejects_invalid_scale_before_spi_or_publication() {
        assert_motion_scale_revalidation_rejected(MotionScaleRevalidationRejection::InvalidScale(
            crate::motion_scale::MotionScaleRejection::EncoderPprZero,
        ))
        .await;
    }

    #[fixture_test]
    async fn set_motion_scale_rejects_runtime_enabled_before_spi_or_publication() {
        assert_motion_scale_revalidation_rejected(MotionScaleRevalidationRejection::RuntimeEnabled)
            .await;
    }

    #[fixture_test]
    async fn set_motion_scale_rejects_motor_armed_mirror_before_spi_or_publication() {
        assert_motion_scale_revalidation_rejected(
            MotionScaleRevalidationRejection::MotorArmedMirror,
        )
        .await;
    }

    #[fixture_test]
    async fn set_motion_scale_rejects_physical_queue_before_spi_or_publication() {
        assert_motion_scale_revalidation_rejected(
            MotionScaleRevalidationRejection::PhysicalQueueNonempty,
        )
        .await;
    }

    #[fixture_test]
    async fn set_motion_scale_rejects_active_timer_before_spi_or_publication() {
        assert_motion_scale_revalidation_rejected(MotionScaleRevalidationRejection::TimerActive)
            .await;
    }

    #[fixture_test]
    async fn set_motion_scale_abn_decoder_ppr_failure_shuts_down_without_publication() {
        assert_motion_scale_spi_failure(
            foci_tmc4671::registers::ABN_DECODER_PPR.addr(),
            &[(foci_tmc4671::registers::ABN_DECODER_PPR.addr(), 16000)],
            &[],
        )
        .await;
    }

    #[fixture_test]
    async fn set_motion_scale_step_width_failure_shuts_down_without_publication() {
        assert_motion_scale_spi_failure(
            foci_tmc4671::registers::STEP_WIDTH.addr(),
            &[
                (foci_tmc4671::registers::ABN_DECODER_PPR.addr(), 16000),
                (foci_tmc4671::registers::STEP_WIDTH.addr(), 16),
            ],
            &[(foci_tmc4671::registers::ABN_DECODER_PPR.addr(), 16000)],
        )
        .await;
    }

    #[fixture_test]
    async fn generic_register_failure_uses_board_tail() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut board = RecordingBoard::fall_through();
        board.register_result = Err(crate::tmc_control::dispatch::RegisterRequestError::Other);
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
    }

    #[fixture_test]
    async fn current_limit_invalid_uses_dedicated_shutdown() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut board = RecordingBoard::fall_through();
        board.register_result =
            Err(crate::tmc_control::dispatch::RegisterRequestError::CurrentLimitInvalid);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let outcome = handle_tmc_control_request(
            TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: u32::MAX }),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        )
        .await;

        assert_eq!(outcome.tail, TmcControlRequestTail::FallThrough);
        assert_eq!(outcome.shutdown, Some(ShutdownReason::CurrentLimitInvalid));
        assert!(!outcome.should_refresh_velocity_scale);
    }

    #[fixture_test]
    async fn pre_request_diagnostic_disposition_uses_board_tail() {
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
    }

    #[fixture_test]
    async fn resistance_test_runs_diagnostic_request() {
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
    }

    #[fixture_test]
    async fn failed_arm_disarms_once_and_returns_skip_tick_tail() {
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
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn explicit_disable_breakaway_cancellation_completes_after_disarm() {
        let _guard =
            pollster::block_on(crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.lock());
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_commission_engine(breakaway_campaign_engine());
        let mut board = RecordingBoard::fall_through();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
        crate::diagnostic::begin_breakaway_campaign();
        crate::diagnostic::panic_after_breakaway_completion_for_test();

        let reset = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            pollster::block_on(handle_tmc_control_request(
                TmcRequest::new(0, TmcRequestKind::SetEnable(false)),
                TmcPreRequestDisposition::ContinueRequest,
                &mut runtime,
                &mut tmc,
                &mut board,
            ));
        }));

        assert!(reset.is_err(), "completion must reach the reset injection");
        assert_eq!(board.disarm_calls, 1, "disarm must precede completion");
        assert!(
            runtime.is_commission_active(),
            "engine removal must follow completion"
        );
        assert_eq!(
            crate::diagnostic::current_breakaway_breadcrumb_for_test(),
            Some(crate::diagnostic::BreakawayBreadcrumb {
                checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
                interrupted: false,
            })
        );

        pollster::block_on(handle_tmc_control_request(
            TmcRequest::new(0, TmcRequestKind::SetEnable(false)),
            TmcPreRequestDisposition::ContinueRequest,
            &mut runtime,
            &mut tmc,
            &mut board,
        ));
        assert!(!runtime.is_commission_active());
        assert_eq!(board.disarm_calls, 2);
    }

    #[fixture_test]
    async fn tune_entry_snapshot_uses_default_current_filters_when_unconfigured() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_calibrated_for_test(true);
        let mut board = RecordingBoard::skip_tick_tail();
        tmc_channel0()
            .configure_motion_scale(stepper_channel0(), 1_000, 3_200)
            .expect("valid motion scale");
        tmc_channel0().store_current_limit(current_limit(2_816), Relaxed);
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
    }

    #[fixture_test]
    async fn success_paths_preserve_both_normal_tail_policies() {
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
    }
}
