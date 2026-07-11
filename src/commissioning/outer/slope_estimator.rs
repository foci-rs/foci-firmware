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

use crate::autotune::TransientBuffer;
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain};
use crate::commissioning::outer::response::mechanical_id_velocity_onset_threshold_mcpms;

/// Fit window placement: straddle onset. The LSQ window includes
/// `PRE_ONSET_US` before onset and `POST_ONSET_US` after, so the fit sees
/// the rest → steady-state transition regardless of whether the rotor is
/// already near terminal by the time onset fires. Clamped to capture bounds.
pub(crate) const PRE_ONSET_US: u64 = 2_000;

pub(crate) const POST_ONSET_US: u64 = 4_000;

/// Total fit window (sum of pre + post). Used for overflow bounds and tests.
/// Keep in sync with `PRE_ONSET_US + POST_ONSET_US`.
#[allow(dead_code)] // Overflow-bounds constant; value is documented in the spec.
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
    reference_position: EncoderCount,
    encoder_domain: EncoderDomain,
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

    // Validate and materialize the complete displacement series before
    // deriving any metric. An ambiguous sample invalidates the whole capture,
    // including evidence before and after it.
    let mut deltas = [0_i32; crate::autotune::TRANSIENT_LEN];
    let mut previous_count = reference_position;
    for (i, &sample) in samples.iter().enumerate() {
        let current_count = encoder_domain.count(sample);
        deltas[i] = match encoder_domain.delta(previous_count, current_count) {
            Ok(delta) => delta.signed_counts(),
            Err(_) => return SlopeEstimate::default(),
        };
        previous_count = current_count;
    }

    // Stage 1: onset detection — first sample with per-sample velocity
    // above the onset threshold (in milli-counts per millisecond).
    let threshold_mcpms = i64::from(mechanical_id_velocity_onset_threshold_mcpms(
        encoder_domain.counts_per_rev(),
    ));
    let onset_idx = {
        let mut found: Option<usize> = None;
        for i in 0..samples.len() {
            let prev_ts = if i == 0 {
                capture_start_us
            } else {
                timestamps_us[i - 1]
            };
            let dt_us = timestamps_us[i].saturating_sub(prev_ts) as i64;
            if dt_us == 0 {
                continue;
            }
            let v_mcpms = i64::from(deltas[i]).saturating_mul(1_000_000) / dt_us;
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

    // Stage 2: fit window placement (onset-straddling).
    let onset_us = timestamps_us[onset_idx];
    let fit_start_us = onset_us.saturating_sub(PRE_ONSET_US).max(capture_start_us);
    let fit_end_us = onset_us.saturating_add(POST_ONSET_US);

    let fit_start_idx = match timestamps_us.iter().position(|&ts| ts >= fit_start_us) {
        Some(idx) => idx,
        None => return SlopeEstimate::default(),
    };
    let fit_end_idx = {
        let mut last: Option<usize> = None;
        for (offset, &ts) in timestamps_us[fit_start_idx..].iter().enumerate() {
            if ts <= fit_end_us {
                last = Some(fit_start_idx + offset);
            } else {
                break;
            }
        }
        match last {
            Some(idx) => idx,
            None => return SlopeEstimate::default(),
        }
    };

    let n_window = fit_end_idx - fit_start_idx + 1;
    if n_window < FIT_MIN_SAMPLES as usize {
        return SlopeEstimate::default();
    }

    // Stage 3: quadratic LSQ on unwrapped positions.
    //
    // Build an unwrapped monotonic position sequence across the fit
    // window by integrating wrap-aware deltas from the reference point.
    let mut unwrapped = [0_i128; crate::autotune::TRANSIENT_LEN];
    let mut running: i128 = 0;
    for (i, &delta) in deltas[..samples.len()].iter().enumerate() {
        running += i128::from(delta);
        unwrapped[i] = running;
    }

    // Normal-equation accumulators. Use the fit-start timestamp as the
    // relative origin so t values stay small and the matrix is well-
    // conditioned.
    let t_origin_us = timestamps_us[fit_start_idx];
    let n: i128 = (fit_end_idx - fit_start_idx + 1) as i128;
    let mut s1: i128 = 0;
    let mut s2: i128 = 0;
    let mut s3: i128 = 0;
    let mut s4: i128 = 0;
    let mut sp0: i128 = 0;
    let mut sp1: i128 = 0;
    let mut sp2: i128 = 0;
    for i in fit_start_idx..=fit_end_idx {
        let t = timestamps_us[i].saturating_sub(t_origin_us) as i128;
        let p = unwrapped[i];
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        s1 += t;
        s2 += t2;
        s3 += t3;
        s4 += t4;
        sp0 += p;
        sp1 += t * p;
        sp2 += t2 * p;
    }

    // Cramer's rule for the c₂ coefficient of p(t) = c₀ + c₁·t + c₂·t².
    //
    //     | n  s1 s2 | | c₀ |   | sp0 |
    //     | s1 s2 s3 | | c₁ | = | sp1 |
    //     | s2 s3 s4 | | c₂ |   | sp2 |
    //
    let det = n * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s2 * s3) + s2 * (s1 * s3 - s2 * s2);
    let det_c2 = n * (s2 * sp2 - s3 * sp1) - s1 * (s1 * sp2 - s2 * sp1) + sp0 * (s1 * s3 - s2 * s2);
    if det == 0 {
        return SlopeEstimate::default();
    }

    // accel (cpms²) = 2 * c₂ * 10⁶ where c₂ is in counts/µs².
    //
    //   Units: c₂ is counts/µs². 1 ms² = (10³ µs)² = 10⁶ µs².
    //   Therefore counts/µs² × 10⁶ = counts/ms² = cpms². The factor of 2
    //   comes from d²p/dt² = 2·c₂ for p(t) = c₀ + c₁·t + c₂·t².
    //
    //   Sanity check: clean-ramp test has positions [0, 5, 20, 45, 80, 125,
    //   180, 245] at 1 ms spacing. In natural units p = 5·t_ms², so
    //   c₂ = 5 counts/ms² = 5·10⁻⁶ counts/µs². accel = 2·c₂ = 10·10⁻⁶
    //   counts/µs² = 10 cpms². ✓
    let accel_num: i128 = 2_i128 * 1_000_000 * det_c2;
    let accel_cpms2 = round_div_signed_i128(accel_num, det) as i32;

    // velocity_rise_mcpms: per-sample velocity at window end minus start,
    // independent of the fitted model.
    let v_at = |i: usize| -> i64 {
        let prev_ts = if i == 0 {
            capture_start_us
        } else {
            timestamps_us[i - 1]
        };
        let dt_us = timestamps_us[i].saturating_sub(prev_ts) as i64;
        if dt_us == 0 {
            0
        } else {
            i64::from(deltas[i]).saturating_mul(1_000_000) / dt_us
        }
    };
    let v_start = v_at(fit_start_idx);
    let v_end = v_at(fit_end_idx);
    let velocity_rise_mcpms = (v_end - v_start) as i32;

    // Residual: compute predicted position at each fit sample, then RMS.
    // det is known ≠ 0; scale c₀, c₁, c₂ into explicit i128 (num, denom)
    // rationals and evaluate predicted_p in i128.
    let det_c0 = sp0 * (s2 * s4 - s3 * s3) - sp1 * (s1 * s4 - s2 * s3) + sp2 * (s1 * s3 - s2 * s2);
    let det_c1 =
        n * (sp1 * s4 - sp2 * s3) - s1 * (sp0 * s4 - sp2 * s2) + s2 * (sp0 * s3 - sp1 * s2);
    let mut residual_sq_sum: i128 = 0;
    for (offset, &ts) in timestamps_us[fit_start_idx..=fit_end_idx]
        .iter()
        .enumerate()
    {
        let i = fit_start_idx + offset;
        let t = ts.saturating_sub(t_origin_us) as i128;
        // predicted_p = (det_c0 + det_c1·t + det_c2·t²) / det
        let predicted_scaled = det_c0 + det_c1 * t + det_c2 * t * t;
        let observed_scaled = unwrapped[i] * det;
        let err = predicted_scaled - observed_scaled;
        // err² can be large — divide before squaring isn't accurate, so
        // compute (err / det)² and accumulate.
        let err_counts = round_div_signed_i128(err, det);
        residual_sq_sum = residual_sq_sum.saturating_add(err_counts * err_counts);
    }
    let mean_sq = residual_sq_sum / n;
    let residual_counts = isqrt_i128(mean_sq) as u32;

    // sign_match: fitted slope agrees with observed rise direction, or
    // rise is below the rise-magnitude gate in which case sign carries
    // no information (deadband bypass).
    let sign_match = if velocity_rise_mcpms.unsigned_abs() < MIN_RISE_MCPMS {
        true
    } else {
        accel_cpms2.signum() == velocity_rise_mcpms.signum()
    };

    SlopeEstimate {
        accel: accel_cpms2,
        residual_counts,
        velocity_rise_mcpms,
        fit_sample_count: n as u8,
        sign_match,
    }
}

/// Symmetric signed integer division with banker's rounding.
fn round_div_signed_i128(numerator: i128, denominator: i128) -> i128 {
    if denominator == 0 {
        return 0;
    }
    let half = denominator.abs() / 2;
    if numerator >= 0 {
        (numerator + half) / denominator
    } else {
        (numerator - half) / denominator
    }
}

/// Integer square root of a non-negative i128 via Newton iteration.
/// Returns 0 for negative inputs.
fn isqrt_i128(value: i128) -> i128 {
    if value < 2 {
        return value.max(0);
    }
    let mut x = value;
    let mut y = (x + 1) / 2;
    while y < x {
        x = y;
        y = (x + value / x) / 2;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::TransientBuffer;

    fn estimate_early_slope(
        buf: &TransientBuffer,
        timestamps_us: &[u64],
        capture_start_us: u64,
        reference_position: u32,
        counts_per_rev: u32,
    ) -> SlopeEstimate {
        let domain = EncoderDomain::new(counts_per_rev).unwrap();
        super::estimate_early_slope(
            buf,
            timestamps_us,
            capture_start_us,
            domain.count(reference_position),
            domain,
        )
    }

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

    #[test]
    fn fit_window_starves_returns_low_confidence() {
        // Onset fires at sample 0 (jumps by 5 counts in 1 ms → 5 cpms).
        // Capture ends immediately — no room for a post-onset window.
        let samples = [0_u32, 5];
        let timestamps: [u64; 2] = [0, 1000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 0, 4000);

        assert_eq!(est.fit_sample_count, 0);
        assert!(!est.is_high_confidence());
    }

    #[test]
    fn clean_linear_velocity_ramp_extracts_positive_slope() {
        // Quadratic positions p(t) = 5*t² (t in ms), sampled at 1 ms.
        // Velocity = 10t cpms at time t, so velocity rises from 0 to 70 cpms.
        // Expected d²p/dt² = 10 counts/ms² (the coefficient is 5 but accel
        // is 2 * c₂ = 10 because p = c₀ + c₁·t + c₂·t² with c₂=5).
        let samples: [u32; 8] = [0, 5, 20, 45, 80, 125, 180, 245];
        let timestamps: [u64; 8] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 0, 4000);

        assert!(
            (9..=11).contains(&est.accel),
            "expected accel ~10 cpms², got {}",
            est.accel
        );
        assert!(est.fit_sample_count >= FIT_MIN_SAMPLES);
        assert!(
            est.velocity_rise_mcpms > 0,
            "rise should be positive, got {}",
            est.velocity_rise_mcpms
        );
        assert!(
            est.is_high_confidence(),
            "clean ramp should be high-confidence: {:?}",
            est,
        );
    }

    #[test]
    fn is_high_confidence_requires_sign_match() {
        // Everything passes EXCEPT sign_match → false.
        let est = SlopeEstimate {
            accel: 10,
            residual_counts: 1,
            velocity_rise_mcpms: 10_000,
            fit_sample_count: FIT_MIN_SAMPLES,
            sign_match: false,
        };
        assert!(
            !est.is_high_confidence(),
            "sign mismatch alone should block high-confidence"
        );
    }

    #[test]
    fn is_high_confidence_requires_minimum_samples() {
        // Everything passes EXCEPT fit_sample_count below minimum.
        let est = SlopeEstimate {
            accel: 10,
            residual_counts: 1,
            velocity_rise_mcpms: 10_000,
            fit_sample_count: FIT_MIN_SAMPLES - 1,
            sign_match: true,
        };
        assert!(
            !est.is_high_confidence(),
            "insufficient fit samples should block high-confidence"
        );
    }

    #[test]
    fn is_high_confidence_requires_rise_above_min() {
        // Everything passes EXCEPT rise magnitude too small.
        let est = SlopeEstimate {
            accel: 10,
            residual_counts: 1,
            velocity_rise_mcpms: MIN_RISE_MCPMS as i32 - 1,
            fit_sample_count: FIT_MIN_SAMPLES,
            sign_match: true,
        };
        assert!(
            !est.is_high_confidence(),
            "rise below MIN_RISE_MCPMS should block high-confidence"
        );
    }

    #[test]
    fn is_high_confidence_requires_residual_below_max() {
        // Everything passes EXCEPT residual too high.
        let est = SlopeEstimate {
            accel: 10,
            residual_counts: MAX_RESIDUAL_COUNTS + 1,
            velocity_rise_mcpms: 10_000,
            fit_sample_count: FIT_MIN_SAMPLES,
            sign_match: true,
        };
        assert!(
            !est.is_high_confidence(),
            "residual above MAX_RESIDUAL_COUNTS should block high-confidence"
        );
    }

    #[test]
    fn is_high_confidence_accepts_when_all_gates_pass() {
        let est = SlopeEstimate {
            accel: 10,
            residual_counts: MAX_RESIDUAL_COUNTS,
            velocity_rise_mcpms: MIN_RISE_MCPMS as i32,
            fit_sample_count: FIT_MIN_SAMPLES,
            sign_match: true,
        };
        assert!(est.is_high_confidence());
    }

    #[test]
    fn sign_match_deadband_bypass_for_tiny_rise() {
        // When velocity_rise is below MIN_RISE_MCPMS, sign agreement is
        // meaningless; the estimator sets sign_match = true. But the
        // rise-magnitude gate still fails, so is_high_confidence stays
        // false. This test pins the layered semantics.
        let est = SlopeEstimate {
            accel: -10, // deliberately opposite sign to any positive rise
            residual_counts: 1,
            velocity_rise_mcpms: 100, // well below MIN_RISE_MCPMS=4000
            fit_sample_count: FIT_MIN_SAMPLES,
            sign_match: true, // deadband bypass in estimator body
        };
        assert!(
            !est.is_high_confidence(),
            "tiny rise should fail rise-magnitude gate even with deadband bypass"
        );
    }

    #[test]
    fn estimator_reports_sign_match_false_on_decelerating_capture() {
        // Strictly increasing positions with rapidly decelerating velocity.
        // Velocity deltas per 1 ms: 200, 100, 50, 30, 15, 10, 5.
        // LSQ fits a concave-down parabola → c₂ < 0 → accel < 0.
        // But v_start = 0 (first sample: delta from reference is 0 with
        // dt_us = 0) and v_end = 15 cpms × 1000 = 15000 mcpms → rise > 0
        // and |rise| >> MIN_RISE_MCPMS. Signs disagree → sign_match = false.
        let samples: [u32; 8] = [0, 200, 300, 350, 380, 395, 405, 410];
        let timestamps: [u64; 8] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 0, 4000);

        assert!(
            est.accel < 0,
            "decelerating pattern should fit a negative c₂: accel={}",
            est.accel,
        );
        assert!(
            est.velocity_rise_mcpms > 0,
            "boundary-velocity rise should be positive: rise={}",
            est.velocity_rise_mcpms,
        );
        assert!(
            est.velocity_rise_mcpms.unsigned_abs() >= MIN_RISE_MCPMS,
            "rise magnitude must clear the deadband for sign_match to apply",
        );
        assert!(
            !est.sign_match,
            "sign(accel) ≠ sign(rise) with rise above deadband → sign_match=false"
        );
        assert!(
            !est.is_high_confidence(),
            "sign mismatch must propagate through is_high_confidence()"
        );
    }

    #[test]
    fn fast_ramp_then_terminal_yields_high_confidence_positive_accel() {
        // RTT failure pattern reproduction: rotor at rest for ~2 ms,
        // then ramps to terminal velocity (~18 cpms) in 1-2 ms, then
        // stays flat. Onset (|v| >= 2 cpms) fires on sample 2 or 3.
        //
        // Position deltas: 0, 0, 5, 13, 18, 18, 18, 18, 18.
        // Cumulative:     0, 0, 5, 18, 36, 54, 72, 90, 108.
        //
        // With capture_start_us = 0 and 1 ms cadence, onset_us ≈ 2000 µs,
        // fit window = [onset - 2 ms, onset + 4 ms] = [0, 6000], which
        // includes samples 0..6 — the full ramp plus some plateau.
        let samples: [u32; 9] = [0, 0, 5, 18, 36, 54, 72, 90, 108];
        let timestamps: [u64; 9] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 0, 4000);

        assert!(
            est.accel > 0,
            "rig regression: accel should be positive, got {} ({:?})",
            est.accel,
            est,
        );
        assert!(
            est.is_high_confidence(),
            "expected high confidence: {:?}",
            est
        );
        assert!(est.velocity_rise_mcpms > 0);
        assert!(est.fit_sample_count >= FIT_MIN_SAMPLES);
    }

    #[test]
    fn one_sample_outlier_does_not_corrupt_slope() {
        // Same clean ramp as before, but with one sample shifted by one
        // count (a realistic quantization perturbation).
        let samples: [u32; 8] = [0, 5, 20, 46, 80, 125, 180, 245];
        //                                 ^^ was 45
        let timestamps: [u64; 8] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 0, 4000);

        // Expected accel from the clean ramp was 10; with one quantized
        // sample we allow ±2 cpms² drift.
        assert!(
            (8..=12).contains(&est.accel),
            "outlier should not pull slope far from 10 cpms²: {}",
            est.accel
        );
    }

    #[test]
    fn jittered_timestamps_still_produce_coherent_slope() {
        // Same positions as the clean ramp, but timestamps jittered by
        // ±200 µs. The time-aware estimator should still land near 10
        // cpms² because it divides delta_counts by actual dt_us.
        let samples: [u32; 8] = [0, 5, 20, 45, 80, 125, 180, 245];
        let timestamps: [u64; 8] = [0, 1200, 1900, 3100, 3950, 5200, 5850, 7100];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 0, 4000);

        assert!(
            (8..=12).contains(&est.accel),
            "jittered timestamps should keep accel close to 10: got {}",
            est.accel
        );
    }

    #[test]
    fn encoder_wrap_within_window_is_handled() {
        // Positions cross the encoder wrap (counts_per_rev = 4000).
        // Unwrapped progression: 0, 3, 8, 15, 24, 35, 48, 63 (truly
        // quadratic). Wrapped samples: 3995, 3998, 3, 10, 19, 30, 43, 58.
        // Estimator should unwrap via encoder_delta integration and
        // recover the same accel as the un-wrapped case.
        let samples: [u32; 8] = [3995, 3998, 3, 10, 19, 30, 43, 58];
        let timestamps: [u64; 8] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 3995, 4000);
        // Note reference_position = 3995 (last known rest position before capture).

        assert!(
            est.accel > 0,
            "wrap capture should still show positive accel: {}",
            est.accel
        );
    }

    #[test]
    fn half_turn_discards_slope_before_later_samples() {
        let samples: [u32; 8] = [4, 8, 2008, 2100, 2300, 2600, 3000, 3500];
        let timestamps: [u64; 8] = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000];
        let buf = buf_from(&samples);

        let est = estimate_early_slope(&buf, &timestamps, 0, 0, 4000);

        assert_eq!(est, SlopeEstimate::default());
    }
}
