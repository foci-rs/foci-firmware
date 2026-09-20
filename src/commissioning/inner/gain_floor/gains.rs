//! velocity_p / position_p derivation from measured accel_per_count and the
//! current-loop bandwidth CurrentTune already measured.

/// Velocity-loop target bandwidth as a fraction of the current-loop
/// bandwidth. Commissioning gains only have to hold the axis quietly through
/// homing; on the live CoreXY a ratio of 10 landed velocity_p an order of
/// magnitude above the autotuned range, 60 lands inside it.
const CURRENT_TO_VELOCITY_BANDWIDTH_RATIO: f32 = 60.0;
/// Position-loop target as a fraction of the current-loop bandwidth. The
/// register-scaling formula below labels this a bandwidth, but on the live
/// CoreXY a ratio of 100 (position_p 9) let the axis lag 2.2 mm at homing
/// speed and trip the position-error stall detector; 9 reproduces the hand
/// tune (99 against 100), which holds it under 0.1 mm.
const CURRENT_TO_POSITION_BANDWIDTH_RATIO: f32 = 9.0;

pub(crate) fn compute_velocity_p(
    accel_per_count: f32,
    bw_current_hz: u32,
) -> Result<foci_tmc4671::pid::PidPGain, crate::autotune::CommissionError> {
    let omega_target_rad_s =
        2.0 * core::f32::consts::PI * bw_current_hz as f32 / CURRENT_TO_VELOCITY_BANDWIDTH_RATIO;
    let velocity_p_raw = libm::ceilf(
        256.0 * omega_target_rad_s * 2.0 * core::f32::consts::PI / (60.0 * accel_per_count),
    );
    if velocity_p_raw <= 0.0 {
        return Err(crate::autotune::CommissionError::GainFloorVelocityGainUnrepresentable);
    }
    let raw = velocity_p_raw as u32;
    let gain = foci_tmc4671::pid::PidPGain::new(raw.min(u16::MAX as u32) as u16)
        .map_err(|_| crate::autotune::CommissionError::GainFloorVelocityGainUnrepresentable)?;
    if gain == foci_tmc4671::pid::PidPGain::ZERO
        || raw > u32::from(foci_tmc4671::pid::PidPGain::MAX.raw())
    {
        return Err(crate::autotune::CommissionError::GainFloorVelocityGainUnrepresentable);
    }
    Ok(gain)
}

pub(crate) fn compute_position_p(
    bw_current_hz: u32,
) -> Result<foci_tmc4671::pid::PidPGain, crate::autotune::CommissionError> {
    let bw_position_target_rad_s =
        2.0 * core::f32::consts::PI * bw_current_hz as f32 / CURRENT_TO_POSITION_BANDWIDTH_RATIO;
    let position_p_raw = libm::ceilf(bw_position_target_rad_s * 60.0 * 256.0 / 65536.0);
    if position_p_raw <= 0.0 {
        return Err(crate::autotune::CommissionError::GainFloorPositionGainUnrepresentable);
    }
    let raw = position_p_raw as u32;
    let gain = foci_tmc4671::pid::PidPGain::new(raw.min(u16::MAX as u32) as u16)
        .map_err(|_| crate::autotune::CommissionError::GainFloorPositionGainUnrepresentable)?;
    if gain == foci_tmc4671::pid::PidPGain::ZERO
        || raw > u32::from(foci_tmc4671::pid::PidPGain::MAX.raw())
    {
        return Err(crate::autotune::CommissionError::GainFloorPositionGainUnrepresentable);
    }
    Ok(gain)
}

/// Convert Task 3's Q32-scaled `accel_raw` (raw position counts per
/// microsecond^2, fixed-point scaled by 2^32) to `accel_per_count` (rad/s^2
/// per raw torque-register count). Before calling this from Task 7, check
/// `autotune/velocity_units.rs` and `autotune/models.rs` for an existing
/// counts-to-physical-units conversion this measurement could reuse
/// instead — this function is the explicit, dimensionally-checked fallback
/// if none exists, not a placeholder to leave in place regardless.
pub(crate) fn accel_raw_q32_to_accel_per_count(
    accel_raw_q32: i64,
    position_units_per_rev: f64,
    i_target: i16,
) -> Option<f32> {
    const ACCEL_RAW_Q32_SHIFT: f64 = 4_294_967_296.0; // 2^32, must match Task 3
    const US_TO_S_SQUARED: f64 = 1_000_000.0 * 1_000_000.0; // (1e6)^2: counts/us^2 -> counts/s^2
    if position_units_per_rev <= 0.0 || i_target == 0 {
        return None;
    }
    let accel_raw_true = accel_raw_q32 as f64 / ACCEL_RAW_Q32_SHIFT;
    let accel_per_count =
        accel_raw_true * US_TO_S_SQUARED * (2.0 * core::f64::consts::PI / position_units_per_rev)
            / f64::from(i_target);
    if !accel_per_count.is_finite() {
        return None;
    }
    Some(accel_per_count as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn live_rig_plant_lands_inside_the_autotuned_velocity_p_range() {
        // Measured on the CoreXY stepper_x: 1.95 rad/s^2 per count at a 600 Hz
        // current loop. Autotune produces velocity_p around 800 there and the
        // hand tune is 350; the commissioning floor must land in that band.
        let velocity_p = compute_velocity_p(1.95, 600).expect("representable");
        assert!(
            (350..=900).contains(&velocity_p.raw()),
            "{}",
            velocity_p.raw()
        );
    }

    #[test]
    fn velocity_p_round_up_never_down() {
        // 256 * (2pi*600/60) * 2pi / (60 * 28.9) = 58.28... -> 59.
        let velocity_p = compute_velocity_p(28.9, 600).expect("representable");
        assert_eq!(velocity_p.raw(), 59);
    }

    #[test]
    fn position_p_reproduces_the_hand_tune_at_the_live_rig_bandwidth() {
        // 2pi*600/9 rad/s * 60 * 256 / 65536 = 98.2 -> 99, against a hand
        // tune of 100 that homes with under 0.1 mm of following error.
        assert_eq!(compute_position_p(600).expect("representable").raw(), 99);
    }

    #[test]
    fn velocity_p_max_overflow_rejected() {
        let result = compute_velocity_p(1e-6, 600);
        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::GainFloorVelocityGainUnrepresentable)
        );
    }

    #[test]
    fn position_p_max_overflow_rejected() {
        let result = compute_position_p(1_000_000_000);
        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::GainFloorPositionGainUnrepresentable)
        );
    }

    #[test]
    fn accel_raw_q32_conversion_is_dimensionally_correct() {
        // A synthetic Q32 value representing exactly 0.0001 counts/us^2, a
        // 4000-count-per-rev encoder, and i_target=2000. Worked by hand:
        // accel_raw_true = 0.0001; * 1e12 (the (1e6)^2 time-scale term this
        // test exists specifically to catch if it's missing or duplicated)
        // = 1e8; * 2*pi/4000 = 157,079.6; / 2000 = 78.54 rad/s^2 per count.
        // A tight tolerance here fails if the 1e12 term is dropped (result
        // would be ~7.85e-5, off by 12 orders of magnitude) or duplicated
        // (result would be ~7.85e13).
        let accel_raw_q32 = (0.0001 * 4_294_967_296.0) as i64;
        let result = accel_raw_q32_to_accel_per_count(accel_raw_q32, 4000.0, 2000)
            .expect("valid conversion");
        assert!(
            (result - 78.54).abs() < 0.01,
            "expected ~78.54, got {result}"
        );
    }

    #[test]
    fn accel_raw_q32_conversion_rejects_zero_ppr_or_target() {
        assert_eq!(accel_raw_q32_to_accel_per_count(1000, 0.0, 2000), None);
        assert_eq!(accel_raw_q32_to_accel_per_count(1000, 4000.0, 0), None);
    }
}
