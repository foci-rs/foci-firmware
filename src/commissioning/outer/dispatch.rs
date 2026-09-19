//! Outer commissioning phase dispatcher.

use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, TransientBuffer};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::types::{OuterSafetyFault, StepResult};

use super::safety::outer_safety_mut;
use super::{
    OuterPhase, OuterSafetyTimingContext, OuterSafetyTimingPhase, OuterSafetyTimingStage,
    TraceSummarySeqs, VelocityIntegralState, check_outer_safety, check_outer_safety_with_timing,
    safety_error_for_phase, velocity,
};

/// Transition the outer phase to `Failed` and emit an `OuterFailed` result.
pub(crate) fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
    *phase = OuterPhase::Failed(err);
    StepResult::OuterFailed(err)
}

/// Transition the position-P sweep to `PositionTuneFailed` and emit an
/// `OuterFailed` result.
///
/// Identical to [`fail_outer`] from the restore machine's point of view -- it
/// drives the same generic entry-gain restore every outer phase's failure
/// uses -- except that the terminal phase retains `outcome`, so the reply can
/// report which sweep failure occurred rather than only that one did.
pub(crate) fn fail_outer_position_tune(
    phase: &mut OuterPhase,
    err: CommissionError,
    outcome: crate::autotune::PositionTuneOutcome,
) -> StepResult {
    *phase = OuterPhase::PositionTuneFailed {
        error: err,
        outcome,
    };
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

fn velocity_fault_terminal(
    phase: &mut OuterPhase,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    match &*phase {
        OuterPhase::VelocityIntegral { .. } => {
            velocity::integral_fault_terminal(phase, error, safety_fault)
        }
        OuterPhase::BreakawayCampaign { .. } => {
            velocity::breakaway_campaign_fault_terminal(phase, error, safety_fault)
        }
        OuterPhase::RobustnessReversal { .. } => {
            velocity::robustness_reversal::robustness_reversal_fault_terminal(
                phase,
                error,
                safety_fault,
            )
        }
        _ => StepResult::OuterFailed(error),
    }
}

fn velocity_integral_timing_phase(state: &VelocityIntegralState) -> (OuterSafetyTimingPhase, bool) {
    match state {
        VelocityIntegralState::Setup
        | VelocityIntegralState::PrepareMovingSlot { .. }
        | VelocityIntegralState::CommandTarget { .. } => (OuterSafetyTimingPhase::Setup, true),
        VelocityIntegralState::Capture { .. } | VelocityIntegralState::PrepareZeroSettle { .. } => {
            (OuterSafetyTimingPhase::MovingCapture, false)
        }
        VelocityIntegralState::BeginSettleCapture { .. }
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

fn velocity_timing_context(
    phase: &OuterPhase,
) -> Result<Option<OuterSafetyTimingContext>, CommissionError> {
    match phase {
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
        // The breakaway campaign uses the plain outer-safety envelope check (no
        // per-phase timing accumulator); its own drivers manage primitive-level
        // safety timing internally, exactly as they do in their standalone tests.
        OuterPhase::BreakawayCampaign { .. } => Ok(None),
        // The robustness gate likewise runs the plain envelope check; its stepper
        // opens and ends the bounded safety primitive itself.
        OuterPhase::RobustnessReversal { .. } => Ok(None),
        // PositionTune runs the plain envelope check; its stepper opens and
        // ends each bounded safety primitive itself.
        OuterPhase::PositionTune { .. } => Ok(None),
        OuterPhase::Failed(_)
        | OuterPhase::PositionTuneFailed { .. }
        | OuterPhase::FailedWithSafety { .. } => Ok(None),
    }
}

// `profile`/`mode`/`inner_lambda_us`/`theta_e_us`/`current_ringing`/
// `current_bandwidth_hz`/`inner_warning_flags` were consumed by the dead
// VelocityTune/VelocityValidate arms this task removed; the parameters stay
// (unread) so the caller's `EngineMode::Outer` threading is untouched.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_outer_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    _profile: CommissionProfile,
    _mode: OperatingMode,
    _inner_lambda_us: u32,
    _theta_e_us: u32,
    _current_ringing: bool,
    _current_bandwidth_hz: u32,
    _inner_warning_flags: u8,
    trace_summary_seqs: &mut TraceSummarySeqs,
    backend: &mut B,
) -> StepResult {
    // A breakaway campaign that has already deferred a terminal record must
    // drain it before any further outer-safety evaluation. A persistent fault
    // (invalid motion budget, elapsed-duration violation) would otherwise
    // re-enter the fault-terminal builder every tick and overwrite the pending
    // closure, so it would never reach the wire and the phase would never fail
    // out. Skipping the safety block on a drain tick lets the campaign driver
    // emit the closure and then the OuterFailed/FailedWithSafety transition.
    let draining_campaign_terminal = velocity::breakaway_campaign_pending_terminal(phase);

    let timing_context = if draining_campaign_terminal {
        None
    } else {
        match velocity_timing_context(phase) {
            Ok(context) => context,
            Err(error) => return velocity_fault_terminal(phase, error, None),
        }
    };
    let is_position_tune = matches!(phase, OuterPhase::PositionTune { .. });
    let active_safety = if draining_campaign_terminal {
        None
    } else {
        outer_safety_mut(phase)
    };
    if let Some(safety) = active_safety {
        if !safety.budget_request_is_valid() {
            if is_position_tune {
                super::position_tune::abort_stimulus(backend);
            }
            defmt::warn!(
                "outer safety: invalid motion budget travel={} velocity={} duration={} dir={}",
                safety.budget.max_stroke_travel_mrev,
                safety.budget.requested_velocity_mrev_s,
                safety.budget.max_duration_ms,
                0x03_u8,
            );
            let fault = safety.invalid_budget_fault();
            let err = safety_error_for_phase(phase);
            if matches!(
                phase,
                OuterPhase::VelocityIntegral { .. }
                    | OuterPhase::BreakawayCampaign { .. }
                    | OuterPhase::RobustnessReversal { .. }
            ) {
                return velocity_fault_terminal(phase, err, Some(fault));
            }
            return fail_outer_with_safety(phase, err, fault);
        }
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
        match safety_result {
            Ok(Some(fault)) => {
                if is_position_tune {
                    super::position_tune::abort_stimulus(backend);
                }
                // Zero excitation immediately on any safety violation.
                let _ = backend
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                    .await;
                let _ = backend
                    .write_register(foci_tmc4671::registers::PID_VELOCITY_TARGET, 0)
                    .await;
                let _ = backend
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let err = safety_error_for_phase(phase);
                if matches!(
                    phase,
                    OuterPhase::VelocityIntegral { .. }
                        | OuterPhase::BreakawayCampaign { .. }
                        | OuterPhase::RobustnessReversal { .. }
                ) {
                    return velocity_fault_terminal(phase, err, Some(fault));
                }
                return fail_outer_with_safety(phase, err, fault);
            }
            Ok(None) => {}
            Err(error) => {
                if is_position_tune {
                    super::position_tune::abort_stimulus(backend);
                }
                defmt::warn!("outer safety check failed: {}", error);
                if matches!(
                    phase,
                    OuterPhase::VelocityIntegral { .. }
                        | OuterPhase::BreakawayCampaign { .. }
                        | OuterPhase::RobustnessReversal { .. }
                ) {
                    return velocity_fault_terminal(phase, error, None);
                }
                return fail_outer(phase, error);
            }
        }
    }

    let result = match phase {
        OuterPhase::VelocityIntegral { .. } => {
            velocity::step_velocity_integral_with_trace(phase, buf, backend).await
        }
        OuterPhase::BreakawayCampaign { .. } => {
            velocity::step_breakaway_campaign_with_trace(phase, backend).await
        }
        OuterPhase::RobustnessReversal {
            state,
            workspace,
            session,
            safety,
        } => {
            velocity::robustness_reversal::step_robustness_reversal(
                state, workspace, session, safety, backend,
            )
            .await
        }
        OuterPhase::PositionTune { .. } => {
            super::position_tune::step_position_tune(phase, buf, backend).await
        }
        OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
        OuterPhase::PositionTuneFailed { error, .. } => StepResult::OuterFailed(*error),
        OuterPhase::FailedWithSafety { error, .. } => StepResult::OuterFailed(*error),
    };
    let next_safety_sample_us =
        outer_safety_mut(phase).and_then(|safety| safety.next_safety_deadline());
    clamp_outer_safety_deadline(result, next_safety_sample_us)
}

#[cfg(test)]
mod tests {
    use super::clamp_outer_safety_deadline;
    use super::velocity_integral_timing_phase;
    use crate::commissioning::outer::{OuterSafetyTimingPhase, VelocityIntegralState};
    use crate::commissioning::types::StepResult;

    #[test]
    fn long_state_deadline_is_clamped_to_earlier_safety_sample() {
        let result = clamp_outer_safety_deadline(StepResult::At(300_000), Some(2_000));

        assert!(matches!(result, StepResult::At(2_000)));
    }

    #[test]
    fn outer_safety_timing_context_distinguishes_motion_from_zero_target_work() {
        assert_eq!(
            velocity_integral_timing_phase(&VelocityIntegralState::Capture {
                next_sample_us: 1,
                deadline_us: 2,
                entry_transition_us: 3,
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
