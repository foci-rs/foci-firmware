const MIN_SAMPLES: usize = 20;
const MIN_PEAK_TO_PEAK_POSITION_UNITS: i32 = 512;
const RESIDUAL_FAIL_PERMILLE: i64 = 75;
pub(crate) const CONSTANT_ACCELERATION_SPAN_US: u32 = 3_000;
pub(crate) const BREAKAWAY_THRESHOLD_UNITS: i32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum GainFloorFitError {
    TooFewSamples,
    InsufficientExcitation,
    HighResidual,
    WrongSign,
}

pub(crate) fn fit_accel_raw(
    samples: &[(u32, i32)],
    commanded_sign: i8,
) -> Result<i64, GainFloorFitError> {
    if samples.len() < MIN_SAMPLES {
        return Err(GainFloorFitError::TooFewSamples);
    }

    let min_pos = samples.iter().map(|(_, p)| *p).min().unwrap_or(0);
    let max_pos = samples.iter().map(|(_, p)| *p).max().unwrap_or(0);
    if max_pos.saturating_sub(min_pos) < MIN_PEAK_TO_PEAK_POSITION_UNITS {
        return Err(GainFloorFitError::InsufficientExcitation);
    }

    let samples = &samples[fit_window(samples)];
    if samples.len() < MIN_SAMPLES {
        return Err(GainFloorFitError::TooFewSamples);
    }

    let Some([a0, a1, a2]) = quadratic_coefficients(samples) else {
        return Err(GainFloorFitError::HighResidual);
    };
    let accel_raw_f64 = 2.0 * a2;

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
    let n = samples.len() as f64;
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

    const ACCEL_RAW_Q32_SHIFT: f64 = 4_294_967_296.0;
    let accel_raw_q32 = accel_raw_f64 * ACCEL_RAW_Q32_SHIFT;
    if !accel_raw_q32.is_finite() || accel_raw_q32.abs() >= i64::MAX as f64 {
        return Err(GainFloorFitError::HighResidual);
    }
    Ok(accel_raw_q32 as i64)
}

fn quadratic_coefficients(samples: &[(u32, i32)]) -> Option<[f64; 3]> {
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

    let det =
        n * (sx2 * sx4 - sx3 * sx3) - sx * (sx * sx4 - sx2 * sx3) + sx2 * (sx * sx3 - sx2 * sx2);
    if det.abs() < f64::EPSILON {
        return None;
    }
    let det_a2 =
        n * (sx2 * sx2y - sx3 * sxy) - sx * (sx * sx2y - sx2 * sxy) + sy * (sx * sx3 - sx2 * sx2);
    let a2 = det_a2 / det;

    let det_a1 =
        n * (sxy * sx4 - sx2y * sx3) - sy * (sx * sx4 - sx2 * sx3) + sx2 * (sx * sx2y - sx2 * sxy);
    let a1 = det_a1 / det;
    let det_a0 = sy * (sx2 * sx4 - sx3 * sx3) - sx * (sxy * sx4 - sx2y * sx3)
        + sx2 * (sxy * sx3 - sx2y * sx2);
    let a0 = det_a0 / det;
    Some([a0, a1, a2])
}

pub(crate) fn fit_window(samples: &[(u32, i32)]) -> core::ops::Range<usize> {
    let Some(&(_, head)) = samples.first() else {
        return 0..0;
    };
    let start = samples
        .iter()
        .position(|&(_, p)| (p - head).unsigned_abs() >= BREAKAWAY_THRESHOLD_UNITS as u32)
        .unwrap_or(0);
    let t_start = samples[start].0;
    let end = samples[start..]
        .iter()
        .position(|&(t, _)| t.wrapping_sub(t_start) > CONSTANT_ACCELERATION_SPAN_US)
        .map_or(samples.len(), |offset| start + offset);
    start..end
}

pub(crate) fn integration_window(samples: &[(u32, i32)]) -> core::ops::Range<usize> {
    let fit = fit_window(samples);
    fit.start.saturating_sub(1)..fit.end
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integration_window_opens_on_the_last_sample_at_rest() {
        let samples: heapless::Vec<(u32, i32), 8> =
            [(0, 0), (60, 0), (120, 2), (180, 9), (240, 20), (300, 35)]
                .into_iter()
                .collect();
        assert_eq!(fit_window(&samples).start, 3);
        assert_eq!(integration_window(&samples), 2..6);
    }

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
        let samples = synthetic_samples(0.002, 10, 30);
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
    fn rise_then_plateau_under_torque_is_rejected_not_trimmed() {
        let mut v: heapless::Vec<(u32, i32), 200> = heapless::Vec::new();
        for i in 0..84u32 {
            let t = i * 60;
            let pos = if i < 30 { (i * i) as i32 } else { 29 * 29 };
            v.push((t, pos)).unwrap();
        }
        assert!(matches!(
            fit_accel_raw(&v, 1),
            Err(GainFloorFitError::HighResidual | GainFloorFitError::WrongSign)
        ));
    }

    #[test]
    fn fit_uses_only_the_constant_acceleration_span_after_breakaway() {
        let accel = 1.0e-4f64;
        let period = 60u32;
        let t_breakaway = (0..84u32)
            .map(|i| i * period)
            .find(|&t| {
                (0.5 * accel * f64::from(t) * f64::from(t)) as i32 >= BREAKAWAY_THRESHOLD_UNITS
            })
            .expect("trace breaks away");
        let t_split = f64::from(t_breakaway + CONSTANT_ACCELERATION_SPAN_US);
        let v_split = accel * t_split;
        let p_split = 0.5 * accel * t_split * t_split;
        let mut v: heapless::Vec<(u32, i32), 200> = heapless::Vec::new();
        for i in 0..84u32 {
            let t = f64::from(i * period);
            let pos = if t <= t_split {
                0.5 * accel * t * t
            } else {
                let dt = t - t_split;
                p_split + v_split * dt - 0.5 * accel * dt * dt
            };
            v.push((i * period, pos as i32)).unwrap();
        }
        let window = fit_window(&v);
        let breakaway_index = (t_breakaway / period) as usize;
        assert_eq!(
            window,
            breakaway_index
                ..breakaway_index + 1 + (CONSTANT_ACCELERATION_SPAN_US / period) as usize
        );
        let fitted = fit_accel_raw(&v, 1).expect("fits") as f64 / 4_294_967_296.0;
        assert!(
            (fitted - accel).abs() < 0.01 * accel,
            "fitted {fitted} should track the early regime {accel}"
        );
        let full_window_fit = 2.0 * quadratic_coefficients(&v).expect("fits")[2];
        assert!(
            full_window_fit < 0.6 * accel,
            "the whole-window fit {full_window_fit} must underestimate; otherwise this test proves nothing"
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
        let samples = synthetic_samples(-0.002, 40, 30);
        assert_eq!(
            fit_accel_raw(&samples, 1),
            Err(GainFloorFitError::WrongSign)
        );
    }

    #[test]
    fn valid_curvature_still_fits_across_a_position_register_wrap() {
        let mut samples = synthetic_samples(0.002, 40, 30);
        for (_, pos) in samples.iter_mut() {
            *pos = pos.wrapping_add(40_000);
        }
        let accel = fit_accel_raw(&samples, 1).expect("valid fit unaffected by a large offset");
        assert!(accel > 0);
    }

    #[test]
    fn high_residual_non_quadratic_trace_rejected() {
        let mut v: heapless::Vec<(u32, i32), 200> = heapless::Vec::new();
        for i in 0..40u32 {
            let t = i * 30;
            let pos = if (i / 5) % 2 == 0 { 0 } else { 10_000 };
            v.push((t, pos)).unwrap();
        }
        assert_eq!(fit_accel_raw(&v, 1), Err(GainFloorFitError::HighResidual));
    }
}
