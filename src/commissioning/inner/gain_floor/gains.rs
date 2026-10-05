pub(crate) const OUTER_FLOOR_REFERENCE_BANDWIDTH_HZ: u32 = 1000;

const REFERENCE_TO_VELOCITY_BANDWIDTH_RATIO: f32 = 17.0;
const VELOCITY_I_TO_P_NUMERATOR: u32 = 1;
const VELOCITY_I_TO_P_DENOMINATOR: u32 = 2;

const REFERENCE_TO_POSITION_BANDWIDTH_RATIO: f32 = 9.0;

pub(crate) fn compute_velocity_p(
    accel_per_count: f32,
    reference_bw_hz: u32,
) -> Result<foci_tmc4671::pid::PidPGain, crate::autotune::CommissionError> {
    let omega_target_rad_s = 2.0 * core::f32::consts::PI * reference_bw_hz as f32
        / REFERENCE_TO_VELOCITY_BANDWIDTH_RATIO;
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

pub(crate) fn compute_velocity_i(
    velocity_p: foci_tmc4671::pid::PidPGain,
) -> Result<foci_tmc4671::pid::PidIGain, crate::autotune::CommissionError> {
    let raw = (u32::from(velocity_p.raw()) * VELOCITY_I_TO_P_NUMERATOR)
        .div_ceil(VELOCITY_I_TO_P_DENOMINATOR);
    foci_tmc4671::pid::PidIGain::new(raw.min(u16::MAX as u32) as u16)
        .map_err(|_| crate::autotune::CommissionError::GainFloorVelocityGainUnrepresentable)
}

pub(crate) fn compute_position_p(
    reference_bw_hz: u32,
) -> Result<foci_tmc4671::pid::PidPGain, crate::autotune::CommissionError> {
    let bw_position_target_rad_s = 2.0 * core::f32::consts::PI * reference_bw_hz as f32
        / REFERENCE_TO_POSITION_BANDWIDTH_RATIO;
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

pub(crate) fn accel_raw_per_count_to_accel_per_count(
    accel_raw_per_us2_per_count: f64,
    position_units_per_rev: f64,
) -> Option<f32> {
    const US_TO_S_SQUARED: f64 = 1_000_000.0 * 1_000_000.0;
    if position_units_per_rev <= 0.0 {
        return None;
    }
    let accel_per_count = accel_raw_per_us2_per_count
        * US_TO_S_SQUARED
        * (2.0 * core::f64::consts::PI / position_units_per_rev);
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
        let velocity_p =
            compute_velocity_p(16.7, OUTER_FLOOR_REFERENCE_BANDWIDTH_HZ).expect("representable");
        assert!(
            (350..=900).contains(&velocity_p.raw()),
            "{}",
            velocity_p.raw()
        );
    }

    #[test]
    fn velocity_i_is_half_of_velocity_p_rounded_up() {
        let velocity_p = foci_tmc4671::pid::PidPGain::new(441).expect("valid");
        assert_eq!(
            compute_velocity_i(velocity_p).expect("representable").raw(),
            221
        );
    }

    #[test]
    fn velocity_p_round_up_never_down() {
        let velocity_p = compute_velocity_p(28.9, 600).expect("representable");
        assert_eq!(velocity_p.raw(), 206);
    }

    #[test]
    fn position_p_reproduces_the_hand_tune_at_the_live_rig_bandwidth() {
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
        let result = accel_raw_per_count_to_accel_per_count(0.0001 / 2000.0, 4000.0)
            .expect("valid conversion");
        assert!(
            (result - 78.54).abs() < 0.01,
            "expected ~78.54, got {result}"
        );
    }

    #[test]
    fn accel_raw_conversion_rejects_zero_ppr_or_non_finite_gain() {
        assert_eq!(accel_raw_per_count_to_accel_per_count(1e-6, 0.0), None);
        assert_eq!(
            accel_raw_per_count_to_accel_per_count(f64::NAN, 4000.0),
            None
        );
    }
}
