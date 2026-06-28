//! Shared types and analysis helpers for the FOCI commissioning workflow.
//!
//! This module provides:
//! - [`TransientBuffer`]: Fixed-size ring buffer for step-response capture
//! - [`CommissionProfile`]: Tuning aggressiveness selection
//! - [`OperatingMode`]: Load condition selection
//! - [`CommissionError`]: Error variants for commissioning phase failures
//! - [`CommissionStatus`]: Overall commissioning outcome
//! - [`PhaseId`]: Identifies each commissioning phase for progress reporting
//!
//! All types are `no_std` compatible. No heap allocation is used.

mod buffer;
mod current;
mod filters;
mod models;
mod position;
#[cfg(test)]
mod test_support;
mod types;
mod velocity;

pub use buffer::{CaptureDirection, TRANSIENT_LEN, TransientBuffer, normalize_capture_sample};
pub use current::{
    CURRENT_AXIS_VALIDATION_CROSS_AXIS, CURRENT_AXIS_VALIDATION_HIGH_RESPONSE,
    CURRENT_AXIS_VALIDATION_LOW_RESPONSE, CURRENT_AXIS_VALIDATION_MOTION,
    CURRENT_AXIS_VALIDATION_PASS, CURRENT_AXIS_VALIDATION_SATURATION,
    CURRENT_AXIS_VALIDATION_STATUS_FLAGS, CURRENT_AXIS_VALIDATION_WRONG_SIGN,
    CURRENT_AXIS_VALIDATION_ZERO_TARGET, CurrentAxisGate, CurrentAxisGateResult,
    CurrentAxisRetryDecision, CurrentAxisSample, CurrentValidation, classify_current_axis_sample,
    classify_current_validation, compute_tail_symmetry_pct, compute_target_error_pct,
    current_axis_retry_decision, current_validation_retry_gains, extract_step_metrics,
    retry_current_axis_gains, should_accept_current_validation_without_retry,
    should_retry_current_validation, simc_current_gains, simc_current_gains_checked,
    simc_current_gains_with_lambda, simc_current_gains_with_lambda_checked,
    synthesize_current_gains,
};
pub use filters::recommend_filters;
pub(crate) use models::VELOCITY_SIMC_MAX_P_REG;
pub use models::{
    AXIS_SPLIT_SOURCE_IMPEDANCE, AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE,
    CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, CURRENT_GAINS_SOURCE_DEFAULT,
    CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED, CURRENT_GAINS_TIER_DEFAULT,
    CURRENT_GAINS_TIER_MEASURED_SPLIT, CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
    CURRENT_GAINS_TIER_NONE, CURRENT_LOOP_FAILURE_FLUX_VALIDATION,
    CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS, CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID,
    CURRENT_LOOP_FAILURE_MOTION, CURRENT_LOOP_FAILURE_NONE,
    CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED,
    CURRENT_LOOP_FAILURE_SATURATION, CURRENT_LOOP_FAILURE_SPI, CURRENT_LOOP_FAILURE_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT,
    CURRENT_VALIDATION_AXIS_FLUX, CURRENT_VALIDATION_AXIS_TORQUE,
    CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS, CurrentGainSynthesis, CurrentGainSynthesisInput,
    CurrentGains, CurrentLoopEvidence, ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel,
    FilterConfig, MechanicalModel, PositionGains, VelocityGains,
};
pub use position::{
    PositionOscillationDiagnostics, analyze_position_oscillation, detect_position_oscillation,
};
pub use types::{
    ClosedLoopEntryState, CommissionError, CommissionProfile, CommissionStatus, InnerSequence,
    OperatingMode, PhaseId, SavedGains, ValidationResult,
};
pub(crate) use velocity::simc_velocity_gains_with_lambda_multiplier;
pub use velocity::{
    VelocityValidation, classify_velocity_validation, extract_velocity_metrics, simc_velocity_gains,
};

/// Compute symmetry metric from forward and reverse rise times.
///
/// Returns `|a - b| / avg(a, b)` in 0.1% units (200 = 20.0%).
/// Returns 0 if both values are zero.
pub fn compute_symmetry_pct(rise_fwd: u32, rise_rev: u32) -> u16 {
    let avg = (rise_fwd as u64 + rise_rev as u64) / 2;
    if avg == 0 {
        return 0;
    }
    let diff = rise_fwd.abs_diff(rise_rev);
    (diff as u64 * 1000 / avg) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Symmetry ---

    #[test]
    fn compute_symmetry_identical() {
        assert_eq!(compute_symmetry_pct(500, 500), 0);
    }

    #[test]
    fn compute_symmetry_20_percent() {
        // |600 - 500| / 550 ≈ 18.2% → 181 in 0.1% units
        let result = compute_symmetry_pct(600, 500);
        assert!(
            (170..=190).contains(&result),
            "symmetry {result} expected ~181"
        );
    }

    #[test]
    fn compute_symmetry_zero_avg() {
        assert_eq!(compute_symmetry_pct(0, 0), 0);
    }
}
