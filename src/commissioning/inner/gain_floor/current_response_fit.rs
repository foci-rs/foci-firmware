//! Exponential current-response fit for `bw_current_floor_hz`.
//! Fits `i(t) = i_target * (1 - exp(-t/tau_current))` via a
//! linearized least-squares regression on `ln(1 - i(t)/i_target)` against
//! `t`, which is linear in `-1/tau_current` — avoiding a nonlinear solver
//! in `no_std`.

const MIN_SAMPLES: usize = 20;
const RESIDUAL_FAIL_PERMILLE: i64 = 75;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum CurrentFitError {
    TooFewSamples,
    HighResidual,
    InsufficientCoverage,
}

pub(crate) fn fit_current_response(
    samples: &[(u32, i16)],
    i_target: i16,
) -> Result<u32, CurrentFitError> {
    if samples.len() < MIN_SAMPLES || i_target == 0 {
        return Err(CurrentFitError::TooFewSamples);
    }

    // Saturation check (before linearization): the spec groups a saturated
    // trace under the same distinct outcome as a high residual, not under
    // "too few points" — a trace stuck at or above i_target from the start
    // is not an insufficient-data problem, it is a bad-fit problem.
    let saturated_count = samples
        .iter()
        .filter(|&&(_, i_actual)| f32::from(i_actual) / f32::from(i_target) >= 1.0)
        .count();
    if saturated_count > samples.len() / 2 {
        return Err(CurrentFitError::HighResidual);
    }

    // Monotonicity check (before linearization): a first-order step
    // response is non-decreasing (for a positive step) by construction; any
    // sample that decreases by more than a small noise tolerance below the
    // running maximum means the data does not support the first-order
    // model at all, regardless of what a partial linear fit's residual
    // over the onset alone might report.
    let noise_tolerance = (f32::from(i_target) * 0.02).max(1.0); // 2% of target
    let mut running_max = f32::MIN;
    for &(_, i_actual) in samples {
        let value = f32::from(i_actual);
        if value < running_max - noise_tolerance {
            return Err(CurrentFitError::HighResidual);
        }
        running_max = running_max.max(value);
    }

    // Linearize: z(t) = ln(1 - i(t)/i_target) = -t/tau_current.
    // Points where i(t) >= i_target (saturated/noisy) are excluded from the
    // regression but counted toward the residual check below.
    let mut sx = 0.0f32;
    let mut sz = 0.0f32;
    let mut sxz = 0.0f32;
    let mut sxx = 0.0f32;
    let mut usable = 0usize;
    for &(t, i_actual) in samples {
        let ratio = f32::from(i_actual) / f32::from(i_target);
        if !(0.0..1.0).contains(&ratio) {
            continue;
        }
        let z = libm::logf(1.0 - ratio);
        let t = t as f32;
        sx += t;
        sz += z;
        sxz += t * z;
        sxx += t * t;
        usable += 1;
    }
    if usable < MIN_SAMPLES {
        return Err(CurrentFitError::TooFewSamples);
    }
    let n = usable as f32;
    let denom = n * sxx - sx * sx;
    if denom.abs() < f32::EPSILON {
        return Err(CurrentFitError::HighResidual);
    }
    let slope = (n * sxz - sx * sz) / denom; // slope = -1/tau_current
    if slope >= 0.0 {
        return Err(CurrentFitError::HighResidual);
    }
    let tau_current_us = -1.0 / slope;

    // Coverage: captured window must extend to at least 3*tau_current.
    let last_t = samples.last().map(|(t, _)| *t as f32).unwrap_or(0.0);
    if last_t < 3.0 * tau_current_us {
        return Err(CurrentFitError::InsufficientCoverage);
    }

    // Residual: RMSE of the fitted exponential against the full captured
    // rise (not just the linearized onset window used for the slope).
    let mut sum_sq_residual = 0.0f32;
    let mut sum_abs_actual = 0.0f32;
    for &(t, i_actual) in samples {
        let predicted = f32::from(i_target) * (1.0 - libm::expf(-(t as f32) / tau_current_us));
        let residual = f32::from(i_actual) - predicted;
        sum_sq_residual += residual * residual;
        sum_abs_actual += f32::from(i_actual).abs();
    }
    let n_all = samples.len() as f32;
    let rmse = libm::sqrtf(sum_sq_residual / n_all);
    let mean_abs = (sum_abs_actual / n_all).max(1.0);
    let residual_permille = (rmse / mean_abs) * 1000.0;
    if residual_permille > RESIDUAL_FAIL_PERMILLE as f32 {
        return Err(CurrentFitError::HighResidual);
    }

    let bw_hz = 1.0 / (2.0 * core::f32::consts::PI * tau_current_us / 1_000_000.0);
    Ok(bw_hz.round() as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_exponential(
        i_target: i16,
        tau_us: f64,
        n: usize,
        period_us: u32,
    ) -> heapless::Vec<(u32, i16), 200> {
        let mut v = heapless::Vec::new();
        for i in 0..n {
            let t = i as u32 * period_us;
            let value = f64::from(i_target) * (1.0 - (-f64::from(t) / tau_us).exp());
            v.push((t, value as i16)).unwrap();
        }
        v
    }

    #[test]
    fn valid_response_at_ouroboros_tau_fits() {
        // tau_current ~= 265 us (Ouroboros, 600 Hz target) -> 3*tau ~= 796 us,
        // well within the 5 ms duration bound and the 30 us cadence's 600 us
        // sample-count floor.
        let samples = synthetic_exponential(2000, 265.0, 40, 30);
        let bw = fit_current_response(&samples, 2000).expect("valid fit");
        // bw_current_floor_hz = 1 / (2*pi*tau_current) ~= 601 Hz for tau=265us.
        assert!((550..=650).contains(&bw));
    }

    #[test]
    fn too_few_samples_rejected() {
        let samples = synthetic_exponential(2000, 265.0, 10, 30); // < 20
        assert_eq!(
            fit_current_response(&samples, 2000),
            Err(CurrentFitError::TooFewSamples)
        );
    }

    #[test]
    fn insufficient_coverage_rejected_when_capture_ends_before_3_tau() {
        // tau_current so large that 3*tau exceeds the captured window even
        // though 20+ samples exist.
        let samples = synthetic_exponential(2000, 3000.0, 20, 30); // 3*tau=9ms > 600us captured
        assert_eq!(
            fit_current_response(&samples, 2000),
            Err(CurrentFitError::InsufficientCoverage)
        );
    }

    #[test]
    fn saturated_trace_rejected_as_high_residual_not_too_few_samples() {
        let mut v: heapless::Vec<(u32, i16), 200> = heapless::Vec::new();
        for i in 0..40u32 {
            v.push((i * 30, 2000)).unwrap(); // instantly saturated (ratio >= 1.0 every sample)
        }
        // The spec groups saturation under the same distinct outcome as a high
        // residual ("A non-monotonic, saturated, high-residual, or otherwise
        // unfit-able current trace fails... reusing the same rejection-vocabulary
        // family"), not under TooFewSamples — this must be checked explicitly
        // before the linearization loop, not left to fall out of "not enough
        // unsaturated points remained."
        assert_eq!(
            fit_current_response(&v, 2000),
            Err(CurrentFitError::HighResidual)
        );
    }

    #[test]
    fn non_monotonic_trace_rejected() {
        let mut v: heapless::Vec<(u32, i16), 200> = heapless::Vec::new();
        for i in 0..40u32 {
            let t = i * 30;
            // Rises normally for the first half, then decreases — not a valid
            // first-order step response.
            let value = if i < 20 {
                (f64::from(2000i16) * (1.0 - (-f64::from(t) / 265.0).exp())) as i16
            } else {
                1200 - (i as i16 - 20) * 10
            };
            v.push((t, value)).unwrap();
        }
        assert_eq!(
            fit_current_response(&v, 2000),
            Err(CurrentFitError::HighResidual)
        );
    }
}
