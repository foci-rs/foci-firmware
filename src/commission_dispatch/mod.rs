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
#[cfg(test)]
mod characterization_tests;
pub mod fixed_gain_amplitude;
#[cfg(test)]
mod fixed_gain_amplitude_tests;
pub mod integral;
#[cfg(test)]
mod integral_tests;
#[cfg(test)]
mod manifest_completeness_tests;
pub mod post_actions;
pub mod replies;
mod restore;
pub mod restore_machine;
pub mod robustness;
#[cfg(test)]
pub(crate) mod test_support;
#[cfg(test)]
mod tests;

pub use post_actions::{StepperTaskDirective, apply_stepper_post_action_state};
pub use replies::{CommissionReplySink, emit_phase_reply, emit_terminal_reply};
pub use restore_machine::{RestoreStep, TerminalRestoreMachine};

use crate::autotune::CommissionPhase;
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
    /// Board-local TMC channel the borrowed state belongs to.
    pub channel: u8,
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
    pub integral:
        &'a mut crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot,
}

impl<'a> CommissionRetention<'a> {
    /// Borrow the Stage-C RAM slot as one lifecycle context.
    pub fn new(
        integral: &'a mut crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot,
    ) -> Self {
        Self { integral }
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
        integral: &'a mut crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot,
        emit_evidence: E,
    ) -> Self {
        Self {
            retention: CommissionRetention::new(integral),
            emit_evidence,
        }
    }
}

impl<'a> DispatchState<'a> {
    /// Build a dispatch-state view for one board-local motor channel.
    pub fn new(
        channel: u8,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<InnerLoopGains>,
    ) -> Self {
        Self {
            channel,
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
        phase_id: CommissionPhase,
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
/// Most a sample may run late past its `OUTER_RESTORE_STABILITY_INTERVAL_US`
/// deadline before `PositionStabilityStep` gives up on the whole check as
/// stale rather than risk a verdict computed over an unbounded window.
const OUTER_RESTORE_STABILITY_MAX_SAMPLE_GAP_US: u32 = 5_000;
const OUTER_RESTORE_STABILITY_ENCODER_COUNTS: u32 = 2;

#[derive(Debug, Clone, Copy, defmt::Format)]
struct RestoreStability {
    stable: bool,
    span_counts: u32,
}

fn stability_from_span(
    min_delta: i32,
    max_delta: i32,
    encoder_counts_per_rev: u32,
) -> RestoreStability {
    let span = (max_delta as i64 - min_delta as i64).unsigned_abs();
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
        RestoreStability {
            stable: false,
            span_counts,
        }
    } else {
        RestoreStability {
            stable: true,
            span_counts,
        }
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
        plan: crate::commissioning::outer::velocity::workflow_plan::CommissioningWorkflowPlan,
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
        /// Per-direction robustness evidence when this tune deployed through
        /// a robustness pass; the pass never emits a robustness terminal, so
        /// this is the only way the cycle-evidence replies reach the host.
        robustness_evidence:
            Option<crate::commissioning::outer::velocity::robustness_evidence::RobustnessEvidence>,
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
        reproduction: Option<integral::IntegralReproductionReport>,
        authority: Option<
            crate::commissioning::outer::velocity::integral_reproduction::IntegralAuthoritySummary,
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
    /// Complete immutable fixed-I fixed-gain amplitude plan disclosed before motion.
    FixedGainAmplitudePlan {
        oid: u8,
        run_sequence: u32,
        plan: crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::FixedGainAmplitudePlan,
    },
    /// Compact firmware-authored fixed-I fixed-gain amplitude terminal.
    FixedGainAmplitudeTerminal {
        oid: u8,
        terminal: crate::commissioning::types::FixedGainAmplitudeTerminal,
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

/// Helper: zero-filled CommissionResult for the mid-phase cancel synthesis
/// path (`TerminalRestoreMachine::start_cancelled`), reporting a `Cancelled`
/// inner status rather than `zero_outer_result()`'s `Spi` convention.
fn zero_commission_result() -> CommissionResult {
    use crate::commissioning::types::{
        AdcResidualEvidence, CommissionTimingSummary, CurrentValidationAxisEvidence,
        CurrentValidationSettledSampleEvidence, InductanceEvidence, InnerResult,
    };

    let inner = InnerResult {
        status: crate::autotune::CommissionStatus::Failed(
            crate::autotune::CommissionError::Cancelled,
        ),
        r_count_milli: 0,
        l_count_micro: 0,
        flux_p: 0,
        flux_i: 0,
        torque_p: 0,
        torque_i: 0,
        candidate_gains_source: 0,
        candidate_axis_split_source: 0,
        candidate_gains_tier: 0,
        candidate_measured_axis_split_permille: 0,
        candidate_applied_axis_split_permille: 0,
        candidate_axis_split_clamped: 0,
        candidate_flux_p: 0,
        candidate_flux_i: 0,
        candidate_torque_p: 0,
        candidate_torque_i: 0,
        candidate_attempt: 0,
        lambda_us: 0,
        theta_e_us: 0,
        theta_source: 0,
        ringing_count: 0,
        bandwidth_hz: 0,
        tau_e_us: 0,
        inductance: InductanceEvidence::EMPTY,
        inner_warning_flags: 0,
        timing: [CommissionTimingSummary::NOT_RUN; 3],
        timing_summary: 0,
        resistance_selected_count_slope_milli: 0,
        resistance_gain_path_count_slope_milli: 0,
        resistance_axis0_count_slope_milli: 0,
        resistance_axis1_count_slope_milli: 0,
        resistance_axis0_intercept_count: 0,
        resistance_axis1_intercept_count: 0,
        resistance_axis0_rmse_permille: 0,
        resistance_axis1_rmse_permille: 0,
        resistance_selected_mask_axis0: 0,
        resistance_selected_mask_axis1: 0,
        resistance_axis0_signed_count_slope_milli: 0,
        resistance_axis1_signed_count_slope_milli: 0,
        resistance_axis0_signed_asymmetry_permille: 0,
        resistance_axis1_signed_asymmetry_permille: 0,
        resistance_axis0_drift_permille: 0,
        resistance_axis1_drift_permille: 0,
        resistance_status_flags_or: 0,
        resistance_warning_flags: 0,
        resistance_peak_abs_current_count: 0,
        resistance_max_abs_steady_mean_current_count: 0,
        resistance_current_ceiling_count: 0,
        resistance_power_stage_tripped: false,
        encoder_alignment_count: 0,
        encoder_alignment_electrical_residual_counts: 0,
        encoder_alignment_stability_counts: 0,
        encoder_alignment_movement_counts: 0,
        encoder_alignment_min_movement_counts: 0,
        encoder_alignment_counts_per_electrical_rev: 0,
        adc_residual_chop0: AdcResidualEvidence::EMPTY,
        adc_residual_chop7: AdcResidualEvidence::EMPTY,
        current_gains_source: 0,
        axis_split_source: 0,
        gains_tier: 0,
        measured_axis_split_permille: 0,
        applied_axis_split_permille: 0,
        axis_split_clamped: 0,
        current_validation_axes: 0,
        retry_budget_exhausted: 0,
        current_loop_failure_reason: 0,
        closed_loop_activation_status: 0,
        closed_loop_activation_position_1: 0,
        closed_loop_activation_position_2: 0,
        closed_loop_activation_drift_count: 0,
        closed_loop_activation_threshold_count: 0,
        closed_loop_activation_runaway: 0,
        current_hold_status: 0,
        current_hold_warning_flags: 0,
        current_hold_elapsed_us: 0,
        current_hold_requested_sample_period_us: 0,
        current_hold_sample_count: 0,
        current_hold_position_span_count: 0,
        current_hold_position_drift_count: 0,
        current_hold_torque_mean_count: 0,
        current_hold_torque_rms_count: 0,
        current_hold_torque_peak_to_peak_count: 0,
        current_hold_torque_crossing_count: 0,
        current_hold_flux_mean_count: 0,
        current_hold_flux_rms_count: 0,
        current_hold_flux_peak_to_peak_count: 0,
        current_hold_flux_crossing_count: 0,
        current_hold_status_flags_or: 0,
        current_hold_actionable_status_count: 0,
        current_validation_step_amplitude: 0,
        current_validation_flux_step_amplitude: 0,
        current_validation_torque_step_amplitude: 0,
        current_validation_pidout_limit: 0,
        current_validation_current_limited: 0,
        current_validation_voltage_limited: 0,
        current_validation_max_p: 0,
        current_settled_validation_count: 0,
        current_settled_validation: [CurrentValidationSettledSampleEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
        current_flux_validation_count: 0,
        current_flux_validation: [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
        current_torque_validation_count: 0,
        current_torque_validation: [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
    };

    CommissionResult {
        inner,
        fallback_velocity_p: 0,
        fallback_velocity_i: 0,
        fallback_position_p: 0,
        fallback_position_i: 0,
        fallback_velocity_limit: 0,
        velocity_filter_hz: 0,
        torque_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
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
        CommissionError::ClosedLoopActivationUnstable => {
            CalibrationError::ClosedLoopActivationUnstable
        }
        CommissionError::Cancelled => CalibrationError::Cancelled,
        CommissionError::AdcFault => CalibrationError::AdcFault,
        CommissionError::CoilFault => CalibrationError::CoilFault,
        CommissionError::PhaseFault => CalibrationError::PhaseFault,
        CommissionError::NoCurrent => CalibrationError::NoCurrent,
        CommissionError::CurrentValidationFailed => CalibrationError::CurrentValidationFailed,
        CommissionError::InvalidSchedule => CalibrationError::InvalidSchedule,
        CommissionError::CurrentHoldFailed => CalibrationError::CurrentHoldFailed,
        CommissionError::FaultInterlock => CalibrationError::FaultInterlock,
        CommissionError::Shutdown => CalibrationError::Shutdown,
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
        StepResult::FixedGainAmplitudeEvent(_) => CommissionOutcome::Continue,
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
        // A breakaway campaign normally reaches Integral on the scheduled-tick
        // path, where `adapt_step_result` carries the plan out as a
        // `CommissionEvent`. This arm covers the request-driven path so the
        // plan is reported either way. Trace records are not emitted here: the
        // engine's own trace hook already emits them for this result, and doing
        // it twice would duplicate the plan on the wire.
        StepResult::BreakawayIntegralPlanReady { run_sequence, plan } => {
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on BreakawayIntegralPlanReady");
            CommissionOutcome::Evidence {
                reply: ReplyPayload::VelocityIntegralPlan {
                    oid: engine_ref.oid(),
                    run_sequence,
                    plan,
                },
            }
        }
        terminal @ (StepResult::VelocityIntegralTerminal(_)
        | StepResult::FixedGainAmplitudeTerminal(_)
        | StepResult::RobustnessReversalTerminal(_)
        | StepResult::SelftestDone
        | StepResult::InnerFailed(_)
        | StepResult::CalibrateDone(_)
        | StepResult::CommissionDone(_)
        | StepResult::OuterDone(_)
        | StepResult::OuterFailed(_)) => {
            let mut machine = TerminalRestoreMachine::start(terminal, state.channel, pwm_maxcnt);
            drive_restore_machine(&mut machine, engine, backend, state, None).await
        }
    }
}

/// Drive a terminal restore to completion in one call.
///
/// The board polls [`TerminalRestoreMachine::poll_step`] once per tick instead;
/// this driver serves the callers that own the whole dispatch inline.
async fn drive_restore_machine<B: CommissioningBackend>(
    machine: &mut TerminalRestoreMachine,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    mut retention: Option<&mut CommissionRetention<'_>>,
) -> CommissionOutcome {
    loop {
        let retention = retention.as_deref_mut();
        match machine.poll_step(engine, backend, state, retention).await {
            RestoreStep::Pending => {}
            RestoreStep::Done(outcome) => return outcome,
        }
    }
}

/// Dispatch one Stage-C terminal while applying its retained-evidence lifecycle.
pub async fn drive_velocity_integral_terminal_with_stages<B: CommissioningBackend>(
    terminal: crate::commissioning::types::VelocityIntegralTerminal,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    pwm_maxcnt: u16,
    channel: u8,
    mut retention: CommissionRetention<'_>,
) -> CommissionOutcome {
    let mut machine = TerminalRestoreMachine::start_staged(
        crate::commissioning::schedule::CommissionTerminal::VelocityIntegral(terminal),
        channel,
        pwm_maxcnt,
    );
    drive_restore_machine(&mut machine, engine, backend, state, Some(&mut retention)).await
}

/// Dispatch one Stage-B result while applying its retained-evidence lifecycle.
pub async fn drive_commission_result_with_stages<B: CommissioningBackend>(
    result: StepResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    pwm_maxcnt: u16,
    channel: u8,
    mut retention: CommissionRetention<'_>,
) -> CommissionOutcome {
    let staged = match result {
        StepResult::FixedGainAmplitudeTerminal(terminal) => {
            crate::commissioning::schedule::CommissionTerminal::FixedGainAmplitude(terminal)
        }
        StepResult::RobustnessReversalTerminal(terminal) => {
            crate::commissioning::schedule::CommissionTerminal::RobustnessReversal(terminal)
        }
        StepResult::BreakawayIntegralPlanReady { run_sequence, plan } => {
            integral::install_authority_from_plan(channel, retention.integral, plan);
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on BreakawayIntegralPlanReady");
            return CommissionOutcome::Evidence {
                reply: ReplyPayload::VelocityIntegralPlan {
                    oid: engine_ref.oid(),
                    run_sequence,
                    plan,
                },
            };
        }
        StepResult::VelocityIntegralTerminal(_) => {
            unreachable!("Stage-C terminals use their stage-owned dispatcher")
        }
        _ => unreachable!("non-velocity terminal reached retained velocity dispatch"),
    };
    let mut machine = TerminalRestoreMachine::start_staged(staged, channel, pwm_maxcnt);
    drive_restore_machine(&mut machine, engine, backend, state, Some(&mut retention)).await
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn calibration_status_from_commission_error_maps_cancelled() {
        assert_eq!(
            calibration_status_from_commission_error(crate::autotune::CommissionError::Cancelled),
            crate::calibration::CalibrationError::Cancelled.status_code(),
        );
    }
}
