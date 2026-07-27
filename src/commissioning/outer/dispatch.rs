//! Outer commissioning phase dispatcher.

use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, TransientBuffer};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::types::{OuterSafetyFault, StepResult};

use super::safety::outer_safety_mut;
#[cfg(feature = "trace-velocity-sweep")]
use super::state::VelocitySweepEvidenceState;
use super::{
    OuterPhase, TraceSummarySeqs, check_outer_safety, commit, filter, mechanical, position,
    safety_error_for_phase, velocity,
};
#[cfg(feature = "trace-velocity-sweep")]
use super::{
    OuterSafetyTimingContext, OuterSafetyTimingPhase, OuterSafetyTimingStage,
    VelocityIntegralState, VelocitySweepState, check_outer_safety_with_timing,
};

/// Transition the outer phase to `Failed` and emit an `OuterFailed` result.
pub(crate) fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
    *phase = OuterPhase::Failed(err);
    StepResult::OuterFailed(err)
}

/// Transition the outer phase to `FailedWithSafety` and emit an `OuterFailed` result.
pub(crate) fn fail_outer_with_safety(
    phase: &mut OuterPhase,
    err: CommissionError,
    safety_fault: OuterSafetyFault,
) -> StepResult {
    *phase = OuterPhase::FailedWithSafety {
        error: err,
        safety_fault,
    };
    StepResult::OuterFailed(err)
}

fn clamp_outer_safety_deadline(
    result: StepResult,
    next_safety_sample_us: Option<u64>,
) -> StepResult {
    match (result, next_safety_sample_us) {
        (StepResult::At(state_deadline_us), Some(safety_deadline_us)) => {
            StepResult::At(state_deadline_us.min(safety_deadline_us))
        }
        (result, _) => result,
    }
}

#[cfg(feature = "trace-velocity-sweep")]
fn velocity_fault_terminal(
    phase: &OuterPhase,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    match phase {
        OuterPhase::VelocitySweep { .. } => {
            velocity::sweep_fault_terminal(phase, error, safety_fault)
        }
        OuterPhase::VelocityIntegral { .. } => {
            velocity::integral_fault_terminal(phase, error, safety_fault)
        }
        _ => StepResult::OuterFailed(error),
    }
}

#[cfg(feature = "trace-velocity-sweep")]
fn velocity_integral_timing_phase(state: &VelocityIntegralState) -> (OuterSafetyTimingPhase, bool) {
    match state {
        VelocityIntegralState::Setup
        | VelocityIntegralState::WriteGains
        | VelocityIntegralState::ClearSignedIntegrator
        | VelocityIntegralState::CommandTarget => (OuterSafetyTimingPhase::Setup, true),
        VelocityIntegralState::Capture { .. } | VelocityIntegralState::CommandZero => {
            (OuterSafetyTimingPhase::MovingCapture, false)
        }
        VelocityIntegralState::ClearSettleIntegrator { .. }
        | VelocityIntegralState::BeginSettleCapture { .. }
        | VelocityIntegralState::SettleCapture { .. }
        | VelocityIntegralState::BeginPositiveIRestCleanup
        | VelocityIntegralState::PositiveIRestCleanupCapture { .. } => {
            (OuterSafetyTimingPhase::ZeroSettle, true)
        }
        VelocityIntegralState::AnalyzeMoving { .. }
        | VelocityIntegralState::AnalyzeSettle { .. }
        | VelocityIntegralState::AnalyzePositiveIRestCleanup { .. }
        | VelocityIntegralState::Advance
        | VelocityIntegralState::AnalyzeRung => (OuterSafetyTimingPhase::Analysis, true),
        VelocityIntegralState::RecoverRung(runtime) => runtime.outer_safety_timing_phase(),
        VelocityIntegralState::FinalizeRecovery(_) => {
            (OuterSafetyTimingPhase::EvidenceEmission, true)
        }
        VelocityIntegralState::FaultTerminal { .. } | VelocityIntegralState::Terminal => {
            (OuterSafetyTimingPhase::Terminalization, true)
        }
    }
}

#[cfg(feature = "trace-velocity-sweep")]
fn velocity_sweep_timing_phase(
    state: &VelocitySweepState,
    evidence: &VelocitySweepEvidenceState,
) -> Result<(OuterSafetyTimingPhase, bool), CommissionError> {
    let phase = match state {
        VelocitySweepState::Setup
        | VelocitySweepState::WriteGain
        | VelocitySweepState::ClearSignedIntegrator
        | VelocitySweepState::CommandTarget => (OuterSafetyTimingPhase::Setup, true),
        VelocitySweepState::Capture { .. } | VelocitySweepState::CommandZero => {
            (OuterSafetyTimingPhase::MovingCapture, false)
        }
        VelocitySweepState::ClearSettleIntegrator { .. }
        | VelocitySweepState::BeginSettleCapture { .. }
        | VelocitySweepState::SettleCapture { .. } => (OuterSafetyTimingPhase::ZeroSettle, true),
        VelocitySweepState::AnalyzeMoving { .. }
        | VelocitySweepState::AnalyzeSettle { .. }
        | VelocitySweepState::Advance
        | VelocitySweepState::AnalyzeForwardRung { .. }
        | VelocitySweepState::FoldForwardRung { .. }
        | VelocitySweepState::AnalyzeReverseRung { .. }
        | VelocitySweepState::FoldReverseRung { .. }
        | VelocitySweepState::FoldJointRung { .. } => (OuterSafetyTimingPhase::Analysis, true),
        VelocitySweepState::EmitRung { .. }
        | VelocitySweepState::FinalizeRecovery { .. }
        | VelocitySweepState::FinishForward { .. }
        | VelocitySweepState::FinishReverse { .. }
        | VelocitySweepState::FinishJoint { .. }
        | VelocitySweepState::EmitHandoff { .. } => {
            (OuterSafetyTimingPhase::EvidenceEmission, true)
        }
        VelocitySweepState::RecoverRung { .. } => {
            let VelocitySweepEvidenceState::Recovering { runtime, .. } = evidence else {
                return Err(CommissionError::InvalidSchedule);
            };
            runtime.outer_safety_timing_phase()
        }
        VelocitySweepState::FaultTerminal { .. } | VelocitySweepState::Terminal { .. } => {
            (OuterSafetyTimingPhase::Terminalization, true)
        }
    };
    Ok(phase)
}

#[cfg(feature = "trace-velocity-sweep")]
fn velocity_timing_context(
    phase: &OuterPhase,
) -> Result<Option<OuterSafetyTimingContext>, CommissionError> {
    match phase {
        OuterPhase::VelocitySweep { state, data, .. } => {
            let (timing_phase, target_zero) = velocity_sweep_timing_phase(state, &data.evidence)?;
            OuterSafetyTimingContext::new(
                OuterSafetyTimingStage::Proportional,
                timing_phase,
                data.rung_index,
                data.slot,
                target_zero,
            )
            .map(Some)
        }
        OuterPhase::VelocityIntegral { state, data, .. } => {
            let (timing_phase, target_zero) = velocity_integral_timing_phase(state);
            OuterSafetyTimingContext::new(
                OuterSafetyTimingStage::Integral,
                timing_phase,
                data.rung_index,
                data.slot,
                target_zero,
            )
            .map(Some)
        }
        OuterPhase::MechanicalId { .. }
        | OuterPhase::VelocityTune { .. }
        | OuterPhase::VelocityValidate { .. }
        | OuterPhase::PositionTune { .. }
        | OuterPhase::FilterTune { .. }
        | OuterPhase::Commit { .. }
        | OuterPhase::Done(_)
        | OuterPhase::Failed(_)
        | OuterPhase::FailedWithSafety { .. } => Ok(None),
    }
}

/// Step outer phase state machine by one cooperative tick.
///
/// Dispatches to the per-phase handler in the matching submodule. Every
/// tick runs the safety envelope check before the phase-specific logic;
/// a violation aborts to `Failed(SafetyEnvelopeViolation)` after zeroing
/// excitation.
#[allow(clippy::too_many_arguments)]
#[allow(dead_code)]
pub(crate) async fn step_outer<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    inner_warning_flags: u8,
    backend: &mut B,
) -> StepResult {
    let mut trace_summary_seqs = TraceSummarySeqs::new();
    step_outer_with_trace(
        phase,
        buf,
        profile,
        mode,
        inner_lambda_us,
        theta_e_us,
        current_ringing,
        current_bandwidth_hz,
        inner_warning_flags,
        &mut trace_summary_seqs,
        backend,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_outer_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    inner_warning_flags: u8,
    trace_summary_seqs: &mut TraceSummarySeqs,
    backend: &mut B,
) -> StepResult {
    #[cfg(feature = "trace-velocity-sweep")]
    let timing_context = match velocity_timing_context(phase) {
        Ok(context) => context,
        Err(error) => return velocity_fault_terminal(phase, error, None),
    };
    if let Some(safety) = outer_safety_mut(phase) {
        if !safety.budget_request_is_valid() {
            defmt::warn!(
                "outer safety: invalid motion budget travel={} velocity={} duration={} dir={}",
                safety.budget.max_stroke_travel_mrev,
                safety.budget.requested_velocity_mrev_s,
                safety.budget.max_duration_ms,
                0x03_u8,
            );
            let fault = safety.invalid_budget_fault();
            let err = safety_error_for_phase(phase);
            #[cfg(feature = "trace-velocity-sweep")]
            if matches!(
                phase,
                OuterPhase::VelocitySweep { .. } | OuterPhase::VelocityIntegral { .. }
            ) {
                return velocity_fault_terminal(phase, err, Some(fault));
            }
            return fail_outer_with_safety(phase, err, fault);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        let safety_result = if let Some(context) = timing_context {
            check_outer_safety_with_timing(
                safety,
                trace_summary_seqs.outer_safety_timing_mut(),
                context,
                backend,
            )
            .await
        } else {
            check_outer_safety(safety, backend).await
        };
        #[cfg(not(feature = "trace-velocity-sweep"))]
        let safety_result = check_outer_safety(safety, backend).await;
        match safety_result {
            Ok(Some(fault)) => {
                // Zero excitation immediately on any safety violation.
                let _ = backend
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                    .await;
                #[cfg(feature = "trace-velocity-sweep")]
                let _ = backend
                    .write_register(foci_tmc4671::registers::PID_VELOCITY_TARGET, 0)
                    .await;
                let _ = backend
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let err = safety_error_for_phase(phase);
                #[cfg(feature = "trace-velocity-sweep")]
                if matches!(
                    phase,
                    OuterPhase::VelocitySweep { .. } | OuterPhase::VelocityIntegral { .. }
                ) {
                    return velocity_fault_terminal(phase, err, Some(fault));
                }
                return fail_outer_with_safety(phase, err, fault);
            }
            Ok(None) => {}
            Err(error) => {
                defmt::warn!("outer safety check failed: {}", error);
                #[cfg(feature = "trace-velocity-sweep")]
                if matches!(
                    phase,
                    OuterPhase::VelocitySweep { .. } | OuterPhase::VelocityIntegral { .. }
                ) {
                    return velocity_fault_terminal(phase, error, None);
                }
                return fail_outer(phase, error);
            }
        }
    }

    let result = match phase {
        OuterPhase::MechanicalId { .. } => {
            mechanical::step_mechanical_id_with_trace(phase, buf, trace_summary_seqs, backend).await
        }
        #[cfg(feature = "trace-velocity-sweep")]
        OuterPhase::VelocitySweep { .. } => {
            velocity::step_velocity_sweep_with_trace(phase, backend).await
        }
        #[cfg(feature = "trace-velocity-sweep")]
        OuterPhase::VelocityIntegral { .. } => {
            velocity::step_velocity_integral_with_trace(phase, buf, backend).await
        }
        OuterPhase::VelocityTune { .. } => {
            velocity::step_velocity_tune(
                phase,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                inner_warning_flags,
                current_bandwidth_hz,
                backend,
            )
            .await
        }
        OuterPhase::VelocityValidate { .. } => {
            velocity::step_velocity_validate_with_trace(
                phase,
                buf,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_ringing,
                current_bandwidth_hz,
                trace_summary_seqs,
                backend,
            )
            .await
        }
        OuterPhase::PositionTune { .. } => {
            position::step_position_tune_with_trace(phase, buf, trace_summary_seqs, backend).await
        }
        OuterPhase::FilterTune { .. } => filter::step_filter_tune(phase, backend).await,
        OuterPhase::Commit { .. } => commit::step_commit(phase),
        OuterPhase::Done(result) => StepResult::OuterDone(result.clone()),
        OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
        OuterPhase::FailedWithSafety { error, .. } => StepResult::OuterFailed(*error),
    };
    let next_safety_sample_us =
        outer_safety_mut(phase).and_then(|safety| safety.next_safety_deadline());
    clamp_outer_safety_deadline(result, next_safety_sample_us)
}

#[cfg(test)]
mod tests {
    use super::clamp_outer_safety_deadline;
    #[cfg(feature = "trace-velocity-sweep")]
    use super::velocity_integral_timing_phase;
    #[cfg(feature = "trace-velocity-sweep")]
    use crate::commissioning::outer::{OuterSafetyTimingPhase, VelocityIntegralState};
    use crate::commissioning::types::StepResult;

    #[test]
    fn long_state_deadline_is_clamped_to_earlier_safety_sample() {
        let result = clamp_outer_safety_deadline(StepResult::At(300_000), Some(2_000));

        assert!(matches!(result, StepResult::At(2_000)));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn outer_safety_timing_context_distinguishes_motion_from_zero_target_work() {
        assert_eq!(
            velocity_integral_timing_phase(&VelocityIntegralState::Capture {
                next_sample_us: 1,
                deadline_us: 2,
            }),
            (OuterSafetyTimingPhase::MovingCapture, false)
        );
        assert_eq!(
            velocity_integral_timing_phase(&VelocityIntegralState::AnalyzeMoving {
                settle_deadline_us: 1,
                analysis_deadline_us: 2,
            }),
            (OuterSafetyTimingPhase::Analysis, true)
        );
        assert_eq!(
            velocity_integral_timing_phase(&VelocityIntegralState::PositiveIRestCleanupCapture {
                next_sample_us: 1,
                deadline_us: 2,
            }),
            (OuterSafetyTimingPhase::ZeroSettle, true)
        );
        assert_eq!(
            velocity_integral_timing_phase(&VelocityIntegralState::AnalyzePositiveIRestCleanup {
                analysis_deadline_us: 2,
            }),
            (OuterSafetyTimingPhase::Analysis, true)
        );
        assert_eq!(
            velocity_integral_timing_phase(&VelocityIntegralState::Terminal),
            (OuterSafetyTimingPhase::Terminalization, true)
        );
    }
}
