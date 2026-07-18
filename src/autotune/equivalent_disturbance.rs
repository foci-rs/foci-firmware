use super::velocity_measurement::{RateBlock, VELOCITY_RATE_Q_SCALE, VelocityMeasurementError};

const TMC_VELOCITY_SCALE: i128 = 65_536;
const MICROS_PER_SECOND: i128 = 1_000_000;
const VELOCITY_P_DIVISOR: i128 = 256;

/// Signed Q16.16 equivalent TMC torque-target counts.
///
/// This controller-domain value is not physical torque, current, or force.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct EquivalentTorqueTargetQ16(i32);

impl EquivalentTorqueTargetQ16 {
    /// Construct a value from its exact signed wire representation.
    pub const fn from_raw(raw: i32) -> Self {
        Self(raw)
    }

    /// Return the exact signed wire representation.
    pub const fn raw(self) -> i32 {
        self.0
    }

    fn checked_from_i128(raw: i128) -> Result<Self, VelocityMeasurementError> {
        i32::try_from(raw)
            .map(Self)
            .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)
    }
}

/// Outward-rounded interval in equivalent TMC torque-target Q16.16 counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct EquivalentTorqueTargetInterval {
    low: EquivalentTorqueTargetQ16,
    high: EquivalentTorqueTargetQ16,
}

impl EquivalentTorqueTargetInterval {
    /// Construct an ordered interval.
    pub const fn new(
        low: EquivalentTorqueTargetQ16,
        high: EquivalentTorqueTargetQ16,
    ) -> Result<Self, VelocityMeasurementError> {
        if low.raw() > high.raw() {
            Err(VelocityMeasurementError::ArithmeticOverflow)
        } else {
            Ok(Self { low, high })
        }
    }

    /// Construct a zero-width interval around one exact value.
    pub const fn point(value: EquivalentTorqueTargetQ16) -> Self {
        Self {
            low: value,
            high: value,
        }
    }

    /// Return the inclusive lower interval edge.
    pub const fn low(self) -> EquivalentTorqueTargetQ16 {
        self.low
    }

    /// Return the inclusive upper interval edge.
    pub const fn high(self) -> EquivalentTorqueTargetQ16 {
        self.high
    }
}

/// Transform exact encoder-rate evidence into equivalent controller disturbance.
///
/// The calculation retains raw displacement and elapsed-time totals through one
/// checked `i128` rational. Each returned Q16.16 value is narrowed exactly once.
pub fn equivalent_disturbance_from_rate(
    p_raw: u16,
    target_velocity_register: i32,
    totals: RateBlock,
    half_width_rate_q: u64,
    encoder_counts_per_rev: u32,
    f_pwm_hz: u32,
) -> Result<(EquivalentTorqueTargetQ16, EquivalentTorqueTargetInterval), VelocityMeasurementError> {
    if totals.elapsed_us == 0 {
        return Err(VelocityMeasurementError::ZeroElapsed);
    }
    if encoder_counts_per_rev == 0 || f_pwm_hz == 0 {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }

    let scale_denominator = i128::from(encoder_counts_per_rev)
        .checked_mul(i128::from(f_pwm_hz))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let elapsed_us = i128::from(totals.elapsed_us);
    let target_q16 = i128::from(target_velocity_register)
        .checked_mul(TMC_VELOCITY_SCALE)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let target_numerator = target_q16
        .checked_mul(scale_denominator)
        .and_then(|value| value.checked_mul(elapsed_us))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let encoder_numerator = i128::from(totals.delta_counts)
        .checked_mul(TMC_VELOCITY_SCALE)
        .and_then(|value| value.checked_mul(TMC_VELOCITY_SCALE))
        .and_then(|value| value.checked_mul(MICROS_PER_SECOND))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let error_numerator = target_numerator
        .checked_sub(encoder_numerator)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let disturbance_numerator = i128::from(p_raw)
        .checked_mul(error_numerator)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let disturbance_denominator = VELOCITY_P_DIVISOR
        .checked_mul(scale_denominator)
        .and_then(|value| value.checked_mul(elapsed_us))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let estimate = EquivalentTorqueTargetQ16::checked_from_i128(
        disturbance_numerator / disturbance_denominator,
    )?;

    let common_denominator = disturbance_denominator
        .checked_mul(i128::from(VELOCITY_RATE_Q_SCALE))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let common_mean_numerator = disturbance_numerator
        .checked_mul(i128::from(VELOCITY_RATE_Q_SCALE))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let common_half_width_numerator = i128::from(p_raw)
        .checked_mul(i128::from(half_width_rate_q))
        .and_then(|value| value.checked_mul(TMC_VELOCITY_SCALE))
        .and_then(|value| value.checked_mul(TMC_VELOCITY_SCALE))
        .and_then(|value| value.checked_mul(elapsed_us))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let low_numerator = common_mean_numerator
        .checked_sub(common_half_width_numerator)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let high_numerator = common_mean_numerator
        .checked_add(common_half_width_numerator)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let low = EquivalentTorqueTargetQ16::checked_from_i128(floor_div(
        low_numerator,
        common_denominator,
    )?)?;
    let high = EquivalentTorqueTargetQ16::checked_from_i128(ceil_div(
        high_numerator,
        common_denominator,
    )?)?;
    let interval = EquivalentTorqueTargetInterval::new(low, high)?;

    Ok((estimate, interval))
}

/// Transform encoder-rate variance into controller-domain Q16.16 variance.
pub fn equivalent_disturbance_variance_from_rate(
    p_raw: u16,
    variance_of_mean_rate_q2: u128,
    encoder_counts_per_rev: u32,
    f_pwm_hz: u32,
) -> Result<u64, VelocityMeasurementError> {
    if encoder_counts_per_rev == 0 || f_pwm_hz == 0 {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }
    let scale_numerator = i128::from(p_raw)
        .checked_mul(TMC_VELOCITY_SCALE)
        .and_then(|value| value.checked_mul(TMC_VELOCITY_SCALE))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let scale_denominator = VELOCITY_P_DIVISOR
        .checked_mul(i128::from(encoder_counts_per_rev))
        .and_then(|value| value.checked_mul(i128::from(f_pwm_hz)))
        .and_then(|value| value.checked_mul(i128::from(VELOCITY_RATE_Q_SCALE)))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let numerator = variance_of_mean_rate_q2
        .checked_mul(scale_numerator.unsigned_abs())
        .and_then(|value| value.checked_mul(scale_numerator.unsigned_abs()))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let denominator = scale_denominator
        .unsigned_abs()
        .checked_mul(scale_denominator.unsigned_abs())
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let rounded_up = numerator
        .checked_add(denominator - 1)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?
        / denominator;
    u64::try_from(rounded_up).map_err(|_| VelocityMeasurementError::ArithmeticOverflow)
}

fn floor_div(numerator: i128, denominator: i128) -> Result<i128, VelocityMeasurementError> {
    if denominator <= 0 {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    if remainder != 0 && numerator < 0 {
        quotient
            .checked_sub(1)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)
    } else {
        Ok(quotient)
    }
}

fn ceil_div(numerator: i128, denominator: i128) -> Result<i128, VelocityMeasurementError> {
    if denominator <= 0 {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    if remainder != 0 && numerator > 0 {
        quotient
            .checked_add(1)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)
    } else {
        Ok(quotient)
    }
}

#[cfg(test)]
mod tests {
    use super::{
        EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16,
        equivalent_disturbance_from_rate, equivalent_disturbance_variance_from_rate,
    };
    use crate::autotune::velocity_measurement::{
        RateBlock, VELOCITY_RATE_Q_SCALE, VelocityMeasurementError,
    };

    fn block(delta_counts: i64, elapsed_us: u64) -> RateBlock {
        RateBlock {
            delta_counts,
            elapsed_us,
            observation_count: 2,
        }
    }

    #[test]
    fn zero_rate_proves_explicit_target_q16_promotion() {
        let (estimate, interval) =
            equivalent_disturbance_from_rate(256, 1, block(0, 2_000), 0, 4_000, 25_000).unwrap();

        assert_eq!(estimate.raw(), 1 << 16);
        assert_eq!(interval.low().raw(), 1 << 16);
        assert_eq!(interval.high().raw(), 1 << 16);
    }

    #[test]
    fn forward_and_reverse_results_preserve_sign() {
        for (target, expected) in [(2, 131_072), (-2, -131_072)] {
            let (estimate, _) =
                equivalent_disturbance_from_rate(256, target, block(0, 1_000), 0, 4_000, 25_000)
                    .unwrap();
            assert_eq!(estimate.raw(), expected);
        }
    }

    #[test]
    fn encoder_rate_equal_to_target_produces_zero_disturbance() {
        let (estimate, interval) =
            equivalent_disturbance_from_rate(256, 8, block(800, 65_536), 0, 4_000, 25_000).unwrap();

        assert_eq!(estimate.raw(), 0);
        assert_eq!(interval, EquivalentTorqueTargetInterval::point(estimate));
    }

    #[test]
    fn exact_totals_bypass_the_rounded_counts_per_ms_helper() {
        let (estimate, _) =
            equivalent_disturbance_from_rate(256, 0, block(1, 1_500), 0, 4_000, 25_000).unwrap();

        assert_eq!(estimate.raw(), -28_633);
        assert_ne!(estimate.raw(), 0);
    }

    #[test]
    fn unequal_elapsed_intervals_use_the_merged_exact_rational() {
        let first = block(1, 1_000);
        let second = block(2, 2_000);
        let totals = first.checked_merge(second).unwrap();

        let (estimate, _) =
            equivalent_disturbance_from_rate(128, 0, totals, 0, 4_000, 25_000).unwrap();

        assert_eq!(estimate.raw(), -21_474);
    }

    #[test]
    fn confidence_conversion_rounds_both_interval_edges_outward() {
        let (estimate, interval) =
            equivalent_disturbance_from_rate(256, 0, block(0, 1_000), 1, 1, 1).unwrap();

        assert_eq!(VELOCITY_RATE_Q_SCALE, 1_000);
        assert_eq!(estimate.raw(), 0);
        assert_eq!(interval.low().raw(), -4_294_968);
        assert_eq!(interval.high().raw(), 4_294_968);
    }

    #[test]
    fn variance_conversion_preserves_scale_and_rounds_nonzero_outward() {
        assert_eq!(
            equivalent_disturbance_variance_from_rate(256, 1_000_000, 1, 65_536).unwrap(),
            4_294_967_296
        );
        assert_eq!(
            equivalent_disturbance_variance_from_rate(256, 1, 1, 65_536).unwrap(),
            4_295
        );
        assert_eq!(
            equivalent_disturbance_variance_from_rate(0, 1_000_000, 1, 65_536).unwrap(),
            0
        );
    }

    #[test]
    fn maximum_positive_controller_target_fits_q16() {
        let (estimate, interval) =
            equivalent_disturbance_from_rate(256, 32_767, block(0, 1_000), 0, 4_000, 25_000)
                .unwrap();

        assert_eq!(estimate.raw(), 2_147_418_112);
        assert_eq!(interval, EquivalentTorqueTargetInterval::point(estimate));
    }

    #[test]
    fn checked_final_narrow_rejects_out_of_domain_result() {
        assert_eq!(
            equivalent_disturbance_from_rate(32_767, 32_767, block(0, 1_000), 0, 4_000, 25_000,),
            Err(VelocityMeasurementError::ArithmeticOverflow)
        );
    }

    #[test]
    fn zero_elapsed_and_wide_intermediate_overflow_fail_closed() {
        assert_eq!(
            equivalent_disturbance_from_rate(1, 1, block(0, 0), 0, 4_000, 25_000,),
            Err(VelocityMeasurementError::ZeroElapsed)
        );
        assert_eq!(
            equivalent_disturbance_from_rate(
                u16::MAX,
                i32::MAX,
                block(i64::MAX, u64::MAX),
                u64::MAX,
                u32::MAX,
                u32::MAX,
            ),
            Err(VelocityMeasurementError::ArithmeticOverflow)
        );
    }

    #[test]
    fn raw_wire_access_is_explicit_and_checked() {
        let minimum = EquivalentTorqueTargetQ16::from_raw(i32::MIN);
        let maximum = EquivalentTorqueTargetQ16::from_raw(i32::MAX);

        assert_eq!(minimum.raw(), i32::MIN);
        assert_eq!(maximum.raw(), i32::MAX);
        assert_eq!(
            EquivalentTorqueTargetInterval::new(maximum, minimum),
            Err(VelocityMeasurementError::ArithmeticOverflow)
        );
    }
}
