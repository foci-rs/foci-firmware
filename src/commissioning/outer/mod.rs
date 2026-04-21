//! Stage 2 (mechanical / velocity / position / filter) commissioning.
//!
//! Submodules implement individual outer-loop phase step routines; the
//! `commission` module's `step_outer` dispatcher will eventually move
//! here too. For now this module hosts the helpers shared across phases
//! plus the safety-envelope guard used by every outer step.

pub mod response;

use crate::autotune::CommissionError;
use crate::commission::{OuterPhase, OuterSafetyLimits};
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Transition the outer phase to `Failed` and emit an `OuterFailed` result.
pub(crate) fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
    *phase = OuterPhase::Failed(err);
    StepResult::OuterFailed(err)
}

/// Map a safety-envelope violation to the dedicated error variant.
///
/// Always returns `SafetyEnvelopeViolation` so the OuterFailed handler
/// can classify it as a safety fault requiring hard disable, distinct
/// from normal phase failures that are eligible for soft restore.
pub(crate) fn safety_error_for_phase(_phase: &OuterPhase) -> CommissionError {
    CommissionError::SafetyEnvelopeViolation
}

/// Per-tick safety envelope check for all outer phases.
///
/// Reads the current encoder position, computes a wrap-aware velocity
/// delta, and accumulates an unwrapped signed position. Checks:
/// - Velocity within cap.
/// - Unwrapped position within window from start.
///
/// Returns `true` if a safety limit is violated.
pub(crate) async fn check_outer_safety<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> bool {
    // Safety limits are initialized by MechanicalId::Setup after it reads
    // the real encoder baseline. Skip the pre-setup placeholder values.
    if safety.torque_cap == 0 {
        return false;
    }

    let encoder_raw = backend.read_encoder_count().await;
    let encoder_count = encoder_raw as i32;

    // Wrap-aware velocity: delta encoder counts per 1ms tick.
    let velocity = response::encoder_delta(
        encoder_count,
        safety.prev_encoder_count,
        safety.counts_per_rev,
    );
    safety.prev_encoder_count = encoder_count;
    safety.unwrapped_position += velocity;

    // Check velocity cap (absolute value).
    let abs_velocity = velocity.unsigned_abs() as i32;
    if abs_velocity > safety.velocity_cap {
        defmt::warn!(
            "outer safety: velocity {} exceeds cap {}",
            abs_velocity,
            safety.velocity_cap
        );
        return true;
    }

    // Check position window against unwrapped accumulator.
    let abs_position = safety.unwrapped_position.unsigned_abs() as i32;
    if abs_position > safety.position_window {
        defmt::warn!(
            "outer safety: position {} exceeds window {}",
            abs_position,
            safety.position_window
        );
        return true;
    }

    false
}
