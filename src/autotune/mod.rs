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
mod models;
#[cfg(test)]
mod test_support;
mod types;
mod velocity;

pub use buffer::{CaptureDirection, TRANSIENT_LEN, TransientBuffer, normalize_capture_sample};
pub use current::{
    CurrentValidation, classify_current_validation, compute_tail_symmetry_pct,
    compute_target_error_pct, current_settling_reference_us, current_validation_retry_gains,
    extract_step_metrics, should_accept_current_validation_without_retry,
    should_retry_current_validation, simc_current_gains, simc_current_gains_checked,
    simc_current_gains_with_lambda, simc_current_gains_with_lambda_checked,
};
pub(crate) use models::VELOCITY_SIMC_MAX_P_REG;
pub use models::{
    CurrentGains, ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel, FilterConfig,
    MechanicalModel, PositionGains, VelocityGains,
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

// ---------------------------------------------------------------------------
// Position oscillation detection (Phase 11)
// ---------------------------------------------------------------------------

/// Diagnostic breakdown of `analyze_position_oscillation`. Exposed so the
/// caller can log the inputs that drove the bool decision when an autotune
/// rejects a gain choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PositionOscillationDiagnostics {
    pub oscillating: bool,
    pub peak: u32,
    pub overshoot: u32,
    pub crossings: u8,
}

/// Compute oscillation diagnostics from a position capture.
///
/// Returns the same boolean decision as `detect_position_oscillation`, plus
/// the peak sample, the overshoot above target, and the post-peak zero-crossing
/// count. All three feed the bool so logging them tells you exactly which
/// rule fired (or which one almost fired).
pub fn analyze_position_oscillation(
    buf: &TransientBuffer,
    target_position: u32,
) -> PositionOscillationDiagnostics {
    let samples = buf.samples();
    if samples.is_empty() || target_position == 0 {
        return PositionOscillationDiagnostics {
            oscillating: false,
            peak: 0,
            overshoot: 0,
            crossings: 0,
        };
    }

    // Overshoot.
    let overshoot_threshold = target_position / 10;
    let peak = samples.iter().copied().max().unwrap_or(0);
    let overshoot = peak.saturating_sub(target_position);

    // Ringing: zero-crossings of (actual - target) after peak.
    let mut crossings: u8 = 0;
    let peak_idx = samples.iter().position(|&s| s == peak).unwrap_or(0);
    if peak_idx + 1 < samples.len() {
        let mut above = samples[peak_idx] > target_position;
        for &sample in &samples[peak_idx + 1..] {
            let now_above = sample > target_position;
            if now_above != above {
                crossings = crossings.saturating_add(1);
                above = now_above;
            }
        }
    }

    let oscillating = (peak > target_position && overshoot > overshoot_threshold) || crossings > 2;

    PositionOscillationDiagnostics {
        oscillating,
        peak,
        overshoot,
        crossings,
    }
}

/// Returns true when the position capture overshoots target by more than 10%
/// or shows more than one full ringing cycle (>2 zero-crossings) after peak.
///
/// Thin wrapper around `analyze_position_oscillation`; preserves the original
/// boolean API for callers that don't need the diagnostic breakdown.
pub fn detect_position_oscillation(buf: &TransientBuffer, target_position: u32) -> bool {
    analyze_position_oscillation(buf, target_position).oscillating
}

// ---------------------------------------------------------------------------
// Filter recommendation (Phase 12)
// ---------------------------------------------------------------------------

/// Recommend filter cutoff frequencies based on prior validation metrics.
///
/// Follows the spec policy:
/// - Velocity chatter detected: enable velocity biquad, cutoff = velocity_bw * 2
/// - Position ramp-up hit oscillation early: enable position filter, cutoff = position_bw * 3
/// - Current ringing detected: enable flux/torque biquad, cutoff = current_bw * 2
///
/// Policy ordering: prefer velocity filtering before position filtering.
/// Leave flux/torque filters off unless current noise demands it.
///
/// # Parameters
/// - `velocity_chatter`: true if velocity validation detected zero-speed chatter
/// - `velocity_bandwidth_hz`: velocity-loop effective bandwidth
/// - `position_oscillation_early`: true if position P ramp-up hit oscillation
///   before reaching a useful gain value
/// - `position_bandwidth_hz`: estimated position-loop bandwidth (from velocity BW / 4)
/// - `current_ringing`: true if current validation detected ringing (count > 1)
/// - `current_bandwidth_hz`: current-loop effective bandwidth
pub fn recommend_filters(
    velocity_chatter: bool,
    velocity_bandwidth_hz: u32,
    position_oscillation_early: bool,
    position_bandwidth_hz: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
) -> FilterConfig {
    let velocity_filter_hz = if velocity_chatter {
        clamp_filter_cutoff(velocity_bandwidth_hz * 2)
    } else {
        None
    };

    let position_filter_hz = if position_oscillation_early {
        clamp_filter_cutoff(position_bandwidth_hz * 3)
    } else {
        None
    };

    let (flux_filter_hz, torque_filter_hz) = if current_ringing {
        let cutoff = clamp_filter_cutoff(current_bandwidth_hz * 2);
        (cutoff, cutoff)
    } else {
        (None, None)
    };

    FilterConfig {
        velocity_filter_hz,
        position_filter_hz,
        flux_filter_hz,
        torque_filter_hz,
    }
}

/// Clamp a filter cutoff to the valid biquad range (10-1000 Hz).
///
/// Returns `None` if the cutoff is below the minimum (filter too low
/// to be useful). Clamps to 1000 Hz if above the maximum.
fn clamp_filter_cutoff(cutoff_hz: u32) -> Option<u32> {
    const MIN_HZ: u32 = 10;
    const MAX_HZ: u32 = 1000;
    if cutoff_hz < MIN_HZ {
        None
    } else if cutoff_hz > MAX_HZ {
        Some(MAX_HZ)
    } else {
        Some(cutoff_hz)
    }
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

    // --- Position oscillation detection ---

    #[test]
    fn position_no_oscillation_clean_settle() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        // Smooth approach to target 1000.
        for i in 0..256u32 {
            let value = (1000.0 * (1.0 - (-((i as f32) / 20.0)).exp())) as u32;
            buf.push(value);
        }
        assert!(
            !detect_position_oscillation(&buf, 1000),
            "clean settle should not detect oscillation"
        );
    }

    #[test]
    fn position_oscillation_detected_overshoot() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        // Overshoot > 10%: target 1000, peak 1200.
        for i in 0..256u32 {
            let base = 1000.0 * (1.0 - (-((i as f32) / 20.0)).exp());
            let overshoot = 250.0 * (-((i as f32) / 30.0)).exp() * ((i as f32) / 8.0).sin();
            buf.push((base + overshoot).max(0.0) as u32);
        }
        let peak = buf.samples().iter().copied().max().unwrap_or(0);
        if peak > 1100 {
            assert!(
                detect_position_oscillation(&buf, 1000),
                "should detect oscillation with peak={}",
                peak
            );
        }
    }

    #[test]
    fn position_oscillation_empty_buffer() {
        let buf = TransientBuffer::new();
        assert!(!detect_position_oscillation(&buf, 1000));
    }

    // --- Filter recommendations ---

    #[test]
    fn filter_no_issues_no_filters() {
        let config = recommend_filters(false, 100, false, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
        assert_eq!(config.torque_filter_hz, None);
    }

    #[test]
    fn filter_velocity_chatter_enables_velocity_filter() {
        let config = recommend_filters(true, 100, false, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, Some(200)); // 100 * 2
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
    }

    #[test]
    fn filter_position_oscillation_enables_position_filter() {
        let config = recommend_filters(false, 100, true, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, Some(75)); // 25 * 3
        assert_eq!(config.flux_filter_hz, None);
    }

    #[test]
    fn filter_current_ringing_enables_flux_torque() {
        let config = recommend_filters(false, 100, false, 25, true, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, Some(1000)); // 500 * 2
        assert_eq!(config.torque_filter_hz, Some(1000));
    }

    #[test]
    fn filter_all_issues_all_filters() {
        let config = recommend_filters(true, 100, true, 25, true, 500);
        assert_eq!(config.velocity_filter_hz, Some(200));
        assert_eq!(config.position_filter_hz, Some(75));
        assert_eq!(config.flux_filter_hz, Some(1000));
        assert_eq!(config.torque_filter_hz, Some(1000));
    }
}
