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
mod equivalent_disturbance;
mod filters;
mod hold;
mod metrics;
mod models;
mod position;
#[cfg(test)]
mod test_support;
mod types;
mod velocity;
pub mod velocity_measurement;

pub use buffer::{
    CaptureDirection, DelayTimestampOffsets, TRANSIENT_LEN, TransientBuffer,
    normalize_capture_sample,
};
pub use current::current_loop_actionable_status_mask;
pub use current::estimate_electrical_delay;
pub use current::{
    CURRENT_AXIS_VALIDATION_CROSS_AXIS, CURRENT_AXIS_VALIDATION_HIGH_RESPONSE,
    CURRENT_AXIS_VALIDATION_LOW_RESPONSE, CURRENT_AXIS_VALIDATION_MOTION,
    CURRENT_AXIS_VALIDATION_PASS, CURRENT_AXIS_VALIDATION_SATURATION,
    CURRENT_AXIS_VALIDATION_STATUS_FLAGS, CURRENT_AXIS_VALIDATION_WRONG_SIGN,
    CURRENT_AXIS_VALIDATION_ZERO_TARGET, CURRENT_VALIDATION_SETTLED_MAX_SAMPLES, CurrentAxisGate,
    CurrentAxisGateResult, CurrentAxisRetryDecision, CurrentAxisSample,
    classify_current_axis_sample, current_axis_retry_decision, retry_current_loop_gains,
    synthesize_current_gains,
};
pub use equivalent_disturbance::{
    EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16, equivalent_disturbance_from_rate,
    equivalent_disturbance_variance_from_rate,
};
pub use filters::recommend_filters;
pub use hold::{
    HOLD_GATE_DWELL_US, HOLD_GATE_FINE_DWELL_US, HOLD_GATE_FINE_SAMPLE_PERIOD_US,
    HOLD_GATE_MAX_SAMPLES, HOLD_GATE_POSITION_SPAN_LIMIT_COUNTS, HOLD_GATE_SAMPLE_PERIOD_US,
    HOLD_GATE_STATUS_FAIL_POSITION_SPAN, HOLD_GATE_STATUS_FAIL_SAMPLE_ERROR,
    HOLD_GATE_STATUS_FAIL_STATUS_FLAGS, HOLD_GATE_STATUS_NOT_RUN, HOLD_GATE_STATUS_PASS,
    HOLD_GATE_WARNING_FLUX_CROSSINGS, HOLD_GATE_WARNING_FLUX_RMS, HOLD_GATE_WARNING_FLUX_SPAN,
    HOLD_GATE_WARNING_NON_ACTIONABLE_STATUS, HOLD_GATE_WARNING_POSITION_SPAN,
    HOLD_GATE_WARNING_SAMPLE_OVERRUN, HOLD_GATE_WARNING_TORQUE_CROSSINGS,
    HOLD_GATE_WARNING_TORQUE_RMS, HOLD_GATE_WARNING_TORQUE_SPAN, HoldGateAccumulator,
    HoldGateMetrics, HoldGateSample, hold_status_failure_reason,
};
pub use metrics::Permille;
pub(crate) use metrics::saturation_permille;
pub(crate) use models::VELOCITY_SIMC_MAX_P_REG;
pub use models::{
    AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE,
    CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, CURRENT_GAINS_SOURCE_DEFAULT,
    CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED, CURRENT_GAINS_TIER_DEFAULT,
    CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, CURRENT_GAINS_TIER_NONE,
    CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC, CURRENT_LOOP_FAILURE_CLOSED_LOOP_ENTRY,
    CURRENT_LOOP_FAILURE_CROSS_AXIS_COUPLING, CURRENT_LOOP_FAILURE_FLUX_VALIDATION,
    CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS, CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN,
    CURRENT_LOOP_FAILURE_HOLD_SAMPLE, CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID, CURRENT_LOOP_FAILURE_MOTION, CURRENT_LOOP_FAILURE_NONE,
    CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, CURRENT_LOOP_FAILURE_RESPONSE_MAGNITUDE,
    CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED, CURRENT_LOOP_FAILURE_SATURATION,
    CURRENT_LOOP_FAILURE_SPI, CURRENT_LOOP_FAILURE_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, CURRENT_LOOP_FAILURE_WRONG_SIGN,
    CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT, CURRENT_VALIDATION_AXIS_FLUX,
    CURRENT_VALIDATION_AXIS_TORQUE, CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS, CurrentGainSynthesis,
    CurrentGainSynthesisError, CurrentGainSynthesisInput, CurrentGains, CurrentLoopEvidence,
    ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel, FilterConfig, MechanicalModel,
    PositionGains, VelocityGains,
};
pub use position::{
    PositionOscillationDiagnostics, analyze_position_oscillation, detect_position_oscillation,
};
pub use types::{
    CLOSED_LOOP_ENTRY_STATUS_FAIL_DRIFT, CLOSED_LOOP_ENTRY_STATUS_FAIL_RUNAWAY,
    CLOSED_LOOP_ENTRY_STATUS_NOT_RUN, CLOSED_LOOP_ENTRY_STATUS_PASS,
    CLOSED_LOOP_ENTRY_STATUS_WARN_DRIFT, ClosedLoopEntryState, CommissionError, CommissionProfile,
    CommissionStatus, InnerSequence, OperatingMode, PhaseId, SavedGains, ValidationResult,
};
pub(crate) use velocity::simc_velocity_gains_with_lambda_multiplier;
pub use velocity::{
    VelocityStiffnessVerdict, VelocityValidation, classify_velocity_stiffness,
    classify_velocity_validation, extract_velocity_metrics, simc_velocity_gains,
};

/// Compute symmetry metric from forward and reverse rise times.
///
/// Returns `|a - b| / avg(a, b)` in 0.1% units (200 = 20.0%).
/// Returns 0 if both values are zero.
pub fn compute_symmetry_permille(rise_fwd: u32, rise_rev: u32) -> Permille {
    metrics::normalized_difference(rise_fwd, rise_rev)
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Symmetry ---

    #[test]
    fn compute_symmetry_identical() {
        assert_eq!(compute_symmetry_permille(500, 500), Permille::ZERO);
    }

    #[test]
    fn compute_symmetry_20_percent() {
        // |600 - 500| / 550 ≈ 18.2% → 181 in 0.1% units
        let result = compute_symmetry_permille(600, 500);
        assert!(
            (170..=190).contains(&result.raw()),
            "symmetry {} expected ~181",
            result.raw()
        );
    }

    #[test]
    fn compute_symmetry_zero_avg() {
        assert_eq!(compute_symmetry_permille(0, 0), Permille::ZERO);
    }
}
