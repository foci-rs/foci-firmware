//! Outer commissioning phase dispatcher.

use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, TransientBuffer};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::types::StepResult;

use super::safety::outer_safety_mut;
use super::{
    OuterPhase, TraceSummarySeqs, check_outer_safety, commit, filter, mechanical, position,
    safety_error_for_phase, velocity,
};

/// Transition the outer phase to `Failed` and emit an `OuterFailed` result.
pub(crate) fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
    *phase = OuterPhase::Failed(err);
    StepResult::OuterFailed(err)
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
    tau_e_us: u32,
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
        tau_e_us,
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
    tau_e_us: u32,
    inner_warning_flags: u8,
    trace_summary_seqs: &mut TraceSummarySeqs,
    backend: &mut B,
) -> StepResult {
    // `tau_e_us` is persisted in the outer engine for a future Phase 2
    // adaptive-measurement path; Phase 1 outer logic uses only
    // `inner_warning_flags`.
    let _ = tau_e_us;
    if let Some(safety) = outer_safety_mut(phase) {
        match check_outer_safety(safety, backend).await {
            Ok(true) => {
                // Zero excitation immediately on any safety violation.
                let _ = backend
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                    .await;
                let _ = backend
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let err = safety_error_for_phase(phase);
                return fail_outer(phase, err);
            }
            Ok(false) => {}
            Err(_) => {
                defmt::warn!("outer safety: encoder read failed");
                return fail_outer(phase, CommissionError::Spi);
            }
        }
    }

    match phase {
        OuterPhase::MechanicalId { .. } => {
            mechanical::step_mechanical_id_with_trace(phase, buf, trace_summary_seqs, backend).await
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
                profile,
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
            position::step_position_tune_with_trace(phase, buf, mode, trace_summary_seqs, backend)
                .await
        }
        OuterPhase::FilterTune { .. } => filter::step_filter_tune(phase, backend).await,
        OuterPhase::Commit { .. } => commit::step_commit(phase),
        OuterPhase::Done(result) => StepResult::OuterDone(result.clone()),
        OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
    }
}
