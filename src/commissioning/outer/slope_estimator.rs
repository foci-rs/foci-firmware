//! Confidence-aware early-slope estimator for MechanicalId captures.
//!
//! The previous LSQ-on-velocity fit lost precision on low-inertia rigs where
//! the rotor reaches terminal velocity before the fit window starts. This
//! estimator fits a quadratic position model over a window that *straddles*
//! onset (`PRE_ONSET_US` of samples before, `POST_ONSET_US` after) so the
//! rest → steady-state transition is always included in the fit, regardless
//! of where onset is detected.
//!
//! See `docs/specs/2026-04-24-mechanical-id-slope-estimator-design.md`.

#![allow(dead_code)] // Scaffolding — items are used across tasks 2-8 as the algorithm lands.

use crate::autotune::TransientBuffer;
use crate::commissioning::outer::response::mechanical_id_velocity_onset_threshold_mcpms;

/// Fit window placement: straddle onset. The LSQ window includes
/// `PRE_ONSET_US` before onset and `POST_ONSET_US` after, so the fit sees
/// the rest → steady-state transition regardless of whether the rotor is
/// already near terminal by the time onset fires. Clamped to capture bounds.
pub(crate) const PRE_ONSET_US: u64 = 2_000;

pub(crate) const POST_ONSET_US: u64 = 4_000;

/// Total fit window (sum of pre + post). Used for overflow bounds and tests.
/// Keep in sync with `PRE_ONSET_US + POST_ONSET_US`.
pub(crate) const FIT_WINDOW_US: u64 = PRE_ONSET_US + POST_ONSET_US;

/// Minimum samples within the fit window for `is_high_confidence`.
pub(crate) const FIT_MIN_SAMPLES: u8 = 4;

/// Minimum velocity rise across the fit window for high confidence.
/// Also used as the deadband below which sign agreement carries no
/// information — keeping one threshold for both roles prevents the
/// dead-code configuration that arises when the deadband is separately
/// configured below the rise gate.
pub(crate) const MIN_RISE_MCPMS: u32 = 4_000;

/// Maximum RMS position-fit residual (counts) for high confidence.
pub(crate) const MAX_RESIDUAL_COUNTS: u32 = 3;

/// Conservative J_eff substituted in the response-ramp fallback when the
/// slope estimator reports low confidence. Chosen against the current
/// 15/25/40% torque ladder and SIMC scaling so the fallback velocity gains
/// land well below the outer_max_velocity_p cap.
pub(crate) const J_EFF_CONSERVATIVE_FALLBACK: u32 = 50_000;

/// Confidence-attributed early-slope estimate for one direction of a
/// MechanicalId capture. All raw inputs are exposed for logging and test
/// introspection; `is_high_confidence()` is the single policy gate.
///
/// `defmt::Format` is required because `MechanicalIdData` (which carries
/// two of these) derives `defmt::Format` and is logged via RTT.
///
/// `Default` produces a low-confidence zero estimate: `accel: 0`,
/// `residual_counts: 0`, `velocity_rise_mcpms: 0`, `fit_sample_count: 0`,
/// `sign_match: false`. Used by `MechanicalIdData::new()` and
/// `reset_captures()` as the pre-capture placeholder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub(crate) struct SlopeEstimate {
    /// Initial acceleration in counts per ms squared (cpms²).
    /// Signed; matches the direction of the applied torque step.
    pub(crate) accel: i32,
    /// RMS residual of the fitted quadratic position model, in counts.
    pub(crate) residual_counts: u32,
    /// Velocity change across the fit window (`v_last − v_first`), signed.
    /// Independent cross-check of the fitted slope's sign.
    pub(crate) velocity_rise_mcpms: i32,
    /// Number of samples that contributed to the fit.
    pub(crate) fit_sample_count: u8,
    /// `sign(accel) == sign(velocity_rise_mcpms)`, or deadband bypass when
    /// `|velocity_rise_mcpms| < MIN_RISE_MCPMS`.
    pub(crate) sign_match: bool,
}

impl SlopeEstimate {
    /// True iff all four gates pass. Single place for the confidence policy.
    pub(crate) fn is_high_confidence(&self) -> bool {
        self.fit_sample_count >= FIT_MIN_SAMPLES
            && self.sign_match
            && self.velocity_rise_mcpms.unsigned_abs() >= MIN_RISE_MCPMS
            && self.residual_counts <= MAX_RESIDUAL_COUNTS
    }
}

/// Estimate early-stage acceleration from a MechanicalId transient capture.
///
/// Returns a low-confidence `SlopeEstimate::default()` if the capture does
/// not contain a detectable onset or an onset-straddling fit window with at
/// least `FIT_MIN_SAMPLES` samples. Stages 2–3 are implemented in
/// subsequent tasks; this scaffold always returns the low-confidence default.
pub(crate) fn estimate_early_slope(
    buf: &TransientBuffer,
    timestamps_us: &[u64],
    capture_start_us: u64,
    reference_position: i32,
    counts_per_rev: i32,
) -> SlopeEstimate {
    let samples = buf.samples();
    debug_assert_eq!(timestamps_us.len(), samples.len());
    if samples.len() < 2 || timestamps_us.len() != samples.len() {
        defmt::error!(
            "slope_estimator: ts/sample length desync: {}/{}",
            timestamps_us.len(),
            samples.len(),
        );
        return SlopeEstimate::default();
    }

    // Stage 1: onset detection — first sample with per-sample velocity
    // above the onset threshold (in milli-counts per millisecond).
    let threshold_mcpms = mechanical_id_velocity_onset_threshold_mcpms() as i64;
    let onset_idx = {
        let mut found: Option<usize> = None;
        for i in 0..samples.len() {
            let (prev_count, prev_ts) = if i == 0 {
                (reference_position as i64, capture_start_us)
            } else {
                (samples[i - 1] as i64, timestamps_us[i - 1])
            };
            let delta = wrap_aware_delta(samples[i] as i64, prev_count, counts_per_rev as i64);
            let dt_us = timestamps_us[i].saturating_sub(prev_ts) as i64;
            if dt_us == 0 {
                continue;
            }
            let v_mcpms = delta.saturating_mul(1_000_000) / dt_us;
            if v_mcpms.abs() >= threshold_mcpms {
                found = Some(i);
                break;
            }
        }
        match found {
            Some(idx) => idx,
            None => return SlopeEstimate::default(),
        }
    };

    // Stages 2 & 3 are placeholders for now; see subsequent tasks.
    let _ = onset_idx;
    SlopeEstimate::default()
}

/// Wrap-aware encoder delta in the range `(-counts_per_rev/2, +counts_per_rev/2]`.
fn wrap_aware_delta(current: i64, previous: i64, counts_per_rev: i64) -> i64 {
    let raw = current - previous;
    let half = counts_per_rev / 2;
    if raw > half {
        raw - counts_per_rev
    } else if raw < -half {
        raw + counts_per_rev
    } else {
        raw
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::TransientBuffer;

    /// Build a buffer populated with the given encoder samples.
    fn buf_from(samples: &[u32]) -> TransientBuffer {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for &s in samples {
            assert!(buf.push(s));
        }
        buf
    }

    #[test]
    fn no_motion_returns_low_confidence_zero_accel() {
        let samples = [1000_u32; 8];
        let timestamps: [u64; 8] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 1000, 4000);

        assert_eq!(est.accel, 0);
        assert_eq!(est.fit_sample_count, 0);
        assert!(!est.is_high_confidence());
    }

    #[test]
    fn below_onset_threshold_returns_low_confidence() {
        // Positions creep by 1 count every 1 ms → 1 cpms = 1000 mcpms,
        // which is below the 2 cpms = 2000 mcpms onset threshold.
        let samples = [1000_u32, 1001, 1002, 1003, 1004, 1005, 1006, 1007];
        let timestamps: [u64; 8] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 1000, 4000);

        assert_eq!(est.accel, 0);
        assert_eq!(est.fit_sample_count, 0);
        assert!(!est.is_high_confidence());
    }
}
