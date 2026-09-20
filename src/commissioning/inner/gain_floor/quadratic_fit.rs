//! Position-domain quadratic least-squares fit for the commissioning
//! fallback gain-floor measurement.
//!
//! Fits `centered_position(t) = p0 + v0*t + 0.5*accel_raw*t^2` over the
//! captured window via normal equations in i64 fixed-point arithmetic —
//! deliberately not a per-sample velocity finite difference, which
//! truncates to zero at this design's 30 us sample period (the failure mode
//! an earlier draft of this design hit).

const MIN_SAMPLES: usize = 20;
/// About 2.8 degrees: twice the largest travel a torque step produced on
/// hardware while only deflecting the belt, so a deflection-only capture is
/// refused rather than fitted as a spring.
const MIN_PEAK_TO_PEAK_POSITION_UNITS: i32 = 512;
const RESIDUAL_FAIL_PERMILLE: i64 = 75;
/// Under one encoder count at 4000 counts/rev: any registered motion ends
/// the flat stiction/plateau stretches the fit must not see.
const FLAT_END_THRESHOLD_UNITS: i32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum GainFloorFitError {
    TooFewSamples,
    InsufficientExcitation,
    HighResidual,
    WrongSign,
}

/// Fit `accel_raw` (raw position counts per microsecond^2) from a captured
/// position trace, requiring the fitted sign to match `commanded_sign`
/// (`1` or `-1`).
pub(crate) fn fit_accel_raw(
    samples: &[(u32, i32)],
    commanded_sign: i8,
) -> Result<i64, GainFloorFitError> {
    if samples.len() < MIN_SAMPLES {
        return Err(GainFloorFitError::TooFewSamples);
    }

    // Excitation is judged on the whole window so a capture that never
    // moved reads as no excitation, not as too few samples after trimming.
    let min_pos = samples.iter().map(|(_, p)| *p).min().unwrap_or(0);
    let max_pos = samples.iter().map(|(_, p)| *p).max().unwrap_or(0);
    if max_pos.saturating_sub(min_pos) < MIN_PEAK_TO_PEAK_POSITION_UNITS {
        return Err(GainFloorFitError::InsufficientExcitation);
    }

    let samples = trim_flat_ends(samples);
    if samples.len() < MIN_SAMPLES {
        return Err(GainFloorFitError::TooFewSamples);
    }

    // Least-squares quadratic fit y = a0 + a1*t + a2*t^2 via normal
    // equations, in f64 — not integer arithmetic. `accel_raw` for a real
    // capture is typically well below 1 raw count per us^2 (e.g. a
    // ~3,000-count displacement over a ~6,000 us window implies
    // accel_raw ~= 2 * 3000 / 6000^2 ~= 1.7e-4), so an integer division
    // here truncates every realistic fit to zero. f64 also avoids the i64
    // overflow an i128-fixed-point sum-of-t^4-times-y term risks at the
    // domain's extremes (200 samples, t up to ~6_000, |y| up to i32::MAX).
    // This mirrors the current-response fit's own choice of floating point
    // (Task 4) for the same class of one-shot, non-hot-path commissioning
    // math — this codebase already accepts f32 in `autotune/current.rs` for
    // exactly this reason; f64 here is a deliberate step up in precision
    // for the wider dynamic range a t^4 term spans, not a new convention.
    let n = samples.len() as f64;
    let (mut sx, mut sx2, mut sx3, mut sx4) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let (mut sy, mut sxy, mut sx2y) = (0.0f64, 0.0f64, 0.0f64);
    for &(t, y) in samples {
        let t = f64::from(t);
        let y = f64::from(y);
        let t2 = t * t;
        sx += t;
        sx2 += t2;
        sx3 += t2 * t;
        sx4 += t2 * t2;
        sy += y;
        sxy += t * y;
        sx2y += t2 * y;
    }

    // Solve the 3x3 normal-equations system for [a0, a1, a2] via Cramer's
    // rule.
    let det =
        n * (sx2 * sx4 - sx3 * sx3) - sx * (sx * sx4 - sx2 * sx3) + sx2 * (sx * sx3 - sx2 * sx2);
    if det.abs() < f64::EPSILON {
        // Degenerate design matrix. Guarded against by construction per the
        // spec (a gap-free, evenly-spaced t grid with >= 20 samples cannot
        // produce a singular system), but checked explicitly rather than
        // dividing by (near-)zero.
        return Err(GainFloorFitError::HighResidual);
    }
    let det_a2 =
        n * (sx2 * sx2y - sx3 * sxy) - sx * (sx * sx2y - sx2 * sxy) + sy * (sx * sx3 - sx2 * sx2);
    let a2 = det_a2 / det;
    // a2 = 0.5 * accel_raw, scaled by 2 to recover accel_raw directly.
    let accel_raw_f64 = 2.0 * a2;

    let det_a1 =
        n * (sxy * sx4 - sx2y * sx3) - sy * (sx * sx4 - sx2 * sx3) + sx2 * (sx * sx2y - sx2 * sxy);
    let a1 = det_a1 / det;
    let det_a0 = sy * (sx2 * sx4 - sx3 * sx3) - sx * (sxy * sx4 - sx2y * sx3)
        + sx2 * (sxy * sx3 - sx2y * sx2);
    let a0 = det_a0 / det;

    // Residual check: RMSE of the fit against the mean absolute response,
    // in permille, reusing resistance identification's Balanced-profile
    // high_residual_fail_permille numeric convention.
    let mut sum_sq_residual = 0.0f64;
    let mut sum_abs_y = 0.0f64;
    for &(t, y) in samples {
        let t = f64::from(t);
        let y = f64::from(y);
        let predicted = a0 + a1 * t + a2 * t * t;
        let residual = y - predicted;
        sum_sq_residual += residual * residual;
        sum_abs_y += y.abs();
    }
    let mean_abs_y = (sum_abs_y / n).max(1.0);
    let rmse = libm::sqrt(sum_sq_residual / n);
    let residual_permille = (rmse / mean_abs_y) * 1000.0;
    if residual_permille > RESIDUAL_FAIL_PERMILLE as f64 {
        return Err(GainFloorFitError::HighResidual);
    }

    if accel_raw_f64 == 0.0 || !accel_raw_f64.is_finite() {
        return Err(GainFloorFitError::WrongSign);
    }
    let fitted_sign: i8 = if accel_raw_f64 > 0.0 { 1 } else { -1 };
    if fitted_sign != commanded_sign {
        return Err(GainFloorFitError::WrongSign);
    }

    // Return accel_raw scaled by 2^32 (Q32 fixed point) so the caller
    // (Task 7's accel_per_count conversion) carries the sub-unit precision
    // this value needs through to the final PidPGain — the f64 value
    // itself must not cross the fit/consumer boundary, since downstream
    // register-domain math in this codebase is integer, not float.
    const ACCEL_RAW_Q32_SHIFT: f64 = 4_294_967_296.0; // 2^32
    let accel_raw_q32 = accel_raw_f64 * ACCEL_RAW_Q32_SHIFT;
    if !accel_raw_q32.is_finite() || accel_raw_q32.abs() >= i64::MAX as f64 {
        return Err(GainFloorFitError::HighResidual);
    }
    Ok(accel_raw_q32 as i64)
}

/// Drop the leading stiction dead zone and the trailing plateau. The
/// quadratic coefficient is invariant under a time shift, so the trimmed
/// slice fits on its original timestamps.
fn trim_flat_ends(samples: &[(u32, i32)]) -> &[(u32, i32)] {
    let moved = |from: i32, p: i32| (p - from).unsigned_abs() >= FLAT_END_THRESHOLD_UNITS as u32;
    let Some(&(_, head)) = samples.first() else {
        return samples;
    };
    let start = samples
        .iter()
        .position(|&(_, p)| moved(head, p))
        .unwrap_or(0);
    let samples = &samples[start..];
    let Some(&(_, tail)) = samples.last() else {
        return samples;
    };
    let end = samples
        .iter()
        .rposition(|&(_, p)| moved(tail, p))
        .map_or(1, |idx| idx + 1);
    &samples[..end]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_samples(
        accel_raw_per_us2: f64,
        n: usize,
        period_us: u32,
    ) -> heapless::Vec<(u32, i32), 200> {
        let mut v = heapless::Vec::new();
        for i in 0..n {
            let t = i as u32 * period_us;
            let pos = (0.5 * accel_raw_per_us2 * f64::from(t) * f64::from(t)) as i32;
            v.push((t, pos)).unwrap();
        }
        v
    }

    #[test]
    fn valid_curvature_fits_positive_accel() {
        let samples = synthetic_samples(0.002, 40, 30);
        let accel = fit_accel_raw(&samples, 1).expect("valid fit");
        assert!(accel > 0);
    }

    #[test]
    fn too_few_samples_rejected() {
        let samples = synthetic_samples(0.002, 10, 30); // < 20
        assert_eq!(
            fit_accel_raw(&samples, 1),
            Err(GainFloorFitError::TooFewSamples)
        );
    }

    #[test]
    fn stiction_dead_zone_before_breakaway_is_trimmed_before_fitting() {
        let period_us = 60u32;
        let dead_zone_slots = 10;
        let mut v: heapless::Vec<(u32, i32), 200> = heapless::Vec::new();
        for i in 0..dead_zone_slots {
            v.push((i as u32 * period_us, 0)).unwrap();
        }
        let accel_raw_per_us2 = 0.002;
        for i in 0..74 {
            let t = (dead_zone_slots + i) as u32 * period_us;
            let t_since_breakaway = f64::from(i as u32 * period_us);
            let pos = (0.5 * accel_raw_per_us2 * t_since_breakaway * t_since_breakaway) as i32;
            v.push((t, pos)).unwrap();
        }
        let accel = fit_accel_raw(&v, 1).expect("dead zone must be trimmed, not fitted as-is");
        assert!(accel > 0);
    }

    #[test]
    fn deflection_only_hardware_trace_is_rejected_as_insufficient_excitation() {
        let points: [(u32, i32); 17] = [
            (0, 0),
            (300, 0),
            (600, 0),
            (900, 17),
            (1200, 33),
            (1500, 50),
            (1800, 82),
            (2100, 99),
            (2400, 131),
            (2700, 164),
            (3000, 181),
            (3300, 197),
            (3600, 213),
            (3900, 230),
            (4200, 246),
            (4500, 263),
            (4800, 263),
        ];
        let mut v: heapless::Vec<(u32, i32), 200> = heapless::Vec::new();
        for window in points.windows(2) {
            let (t0, p0) = window[0];
            let (t1, p1) = window[1];
            for step in 0..5 {
                let frac = f64::from(step) / 5.0;
                let t = t0 + (t1 - t0) * step / 5;
                let p = (f64::from(p0) + (f64::from(p1) - f64::from(p0)) * frac) as i32;
                v.push((t, p)).unwrap();
            }
        }
        v.push(*points.last().unwrap()).unwrap();
        assert_eq!(
            fit_accel_raw(&v, 1),
            Err(GainFloorFitError::InsufficientExcitation)
        );
    }

    #[test]
    fn stationary_capture_rejected_for_insufficient_excitation() {
        let mut v: heapless::Vec<(u32, i32), 200> = heapless::Vec::new();
        for i in 0..30u32 {
            v.push((i * 30, 0)).unwrap();
        }
        assert_eq!(
            fit_accel_raw(&v, 1),
            Err(GainFloorFitError::InsufficientExcitation)
        );
    }

    #[test]
    fn wrong_sign_rejected() {
        // Commanded positive torque, but position curves negative.
        let samples = synthetic_samples(-0.002, 40, 30);
        assert_eq!(
            fit_accel_raw(&samples, 1),
            Err(GainFloorFitError::WrongSign)
        );
    }

    #[test]
    fn valid_curvature_still_fits_across_a_position_register_wrap() {
        // The caller (Task 7) centers samples via wrapping_sub against the
        // first sample before calling this function, so a signed
        // register-domain wrap must not appear as a large discontinuity here —
        // simulate that by generating already-centered samples whose magnitude
        // exceeds a plausible i16 wrap boundary, confirming the i32 domain this
        // function operates on does not itself need any additional unwrapping.
        let mut samples = synthetic_samples(0.002, 40, 30);
        for (_, pos) in samples.iter_mut() {
            *pos = pos.wrapping_add(40_000); // shift well past an i16 boundary
        }
        let accel = fit_accel_raw(&samples, 1).expect("valid fit unaffected by a large offset");
        assert!(accel > 0);
    }

    #[test]
    fn high_residual_non_quadratic_trace_rejected() {
        // A trace that doesn't follow a quadratic (a square wave) should
        // fail the residual check even with enough points and excitation.
        let mut v: heapless::Vec<(u32, i32), 200> = heapless::Vec::new();
        for i in 0..40u32 {
            let t = i * 30;
            let pos = if (i / 5) % 2 == 0 { 0 } else { 10_000 };
            v.push((t, pos)).unwrap();
        }
        assert_eq!(fit_accel_raw(&v, 1), Err(GainFloorFitError::HighResidual));
    }
}
