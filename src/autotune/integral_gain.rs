//! Exact integral-gain synthesis for the fixed TMC4671 Q4.12 domain.

use foci_tmc4671::pid::PidIGain;

const MICROS_PER_SECOND: u128 = 1_000_000;

/// Q4.12 normalization numerator for undetuned TMC4671 integral gains.
pub const ADVANCED_PI_I_Q4_12_NUMERATOR: u64 = 1_048_576;
/// Q4.12 normalization numerator for the eight-times-detuned current loop.
pub const CURRENT_I_Q4_12_DETUNED_NUMERATOR: u64 = 131_072;

/// Failure to synthesize one exact Q4.12 integral gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralGainError {
    /// A required denominator or physical input was zero.
    InvalidInput,
    /// Checked intermediate arithmetic overflowed.
    Arithmetic,
    /// The rounded result exceeded the non-negative signed register domain.
    GainOutOfRange {
        /// Exact rounded raw Q4.12 value that could not be represented.
        attempted_raw: u128,
    },
}

/// Round one positive rational to nearest, with exact halves rounded upward.
///
/// # Errors
///
/// Returns [`IntegralGainError::InvalidInput`] for a zero denominator and
/// [`IntegralGainError::Arithmetic`] if checked rounding arithmetic overflows.
pub const fn round_positive_ratio(
    numerator: u128,
    denominator: u128,
) -> Result<u128, IntegralGainError> {
    if denominator == 0 {
        return Err(IntegralGainError::InvalidInput);
    }
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    let doubled = match remainder.checked_mul(2) {
        Some(value) => value,
        None => return Err(IntegralGainError::Arithmetic),
    };
    match quotient.checked_add((doubled >= denominator) as u128) {
        Some(value) => Ok(value),
        None => Err(IntegralGainError::Arithmetic),
    }
}

/// Synthesize a Q4.12 integral gain from an exact whole-cycle time constant.
///
/// # Errors
///
/// Returns an error when the cycle count is zero, arithmetic overflows, or the
/// rounded gain exceeds the TMC4671 non-negative signed field.
pub fn synthesize_i_gain_from_cycles(
    numerator: u64,
    cycles: u64,
) -> Result<PidIGain, IntegralGainError> {
    if numerator == 0 || cycles == 0 {
        return Err(IntegralGainError::InvalidInput);
    }
    let raw = round_positive_ratio(u128::from(numerator), u128::from(cycles))?;
    if raw > u128::from(PidIGain::MAX.raw()) {
        return Err(IntegralGainError::GainOutOfRange { attempted_raw: raw });
    }
    PidIGain::new(raw as u16).map_err(|_| IntegralGainError::GainOutOfRange { attempted_raw: raw })
}

/// Synthesize a Q4.12 integral gain from a time constant and PWM frequency.
///
/// # Errors
///
/// Returns an error for zero inputs, checked arithmetic overflow, or a rounded
/// gain outside the TMC4671 non-negative signed field.
pub fn synthesize_i_gain(
    numerator: u64,
    ti_us: u64,
    pwm_hz: u32,
) -> Result<PidIGain, IntegralGainError> {
    if numerator == 0 || ti_us == 0 || pwm_hz == 0 {
        return Err(IntegralGainError::InvalidInput);
    }
    let scaled_numerator = u128::from(numerator)
        .checked_mul(MICROS_PER_SECOND)
        .ok_or(IntegralGainError::Arithmetic)?;
    let denominator = u128::from(ti_us)
        .checked_mul(u128::from(pwm_hz))
        .ok_or(IntegralGainError::Arithmetic)?;
    let raw = round_positive_ratio(scaled_numerator, denominator)?;
    if raw > u128::from(PidIGain::MAX.raw()) {
        return Err(IntegralGainError::GainOutOfRange { attempted_raw: raw });
    }
    PidIGain::new(raw as u16).map_err(|_| IntegralGainError::GainOutOfRange { attempted_raw: raw })
}

/// Evaluate the undetuned Q4.12 integral time constant with outward rounding.
pub const fn integral_tau_us(gain: PidIGain, pwm_hz: u32) -> Option<u64> {
    if gain.raw() == 0 || pwm_hz == 0 {
        return None;
    }
    let numerator = match ADVANCED_PI_I_Q4_12_NUMERATOR.checked_mul(MICROS_PER_SECOND as u64) {
        Some(value) => value,
        None => return None,
    };
    let denominator = match (gain.raw() as u64).checked_mul(pwm_hz as u64) {
        Some(value) if value != 0 => value,
        _ => return None,
    };
    Some(numerator.div_ceil(denominator))
}

#[cfg(test)]
fn migrate_q8_8_i_for_test(raw: u16) -> Result<PidIGain, IntegralGainError> {
    let attempted_raw = u128::from(raw) * 16;
    if attempted_raw > u128::from(PidIGain::MAX.raw()) {
        return Err(IntegralGainError::GainOutOfRange { attempted_raw });
    }
    PidIGain::new(attempted_raw as u16)
        .map_err(|_| IntegralGainError::GainOutOfRange { attempted_raw })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q4_12_examples_are_exact() {
        assert_eq!(
            synthesize_i_gain_from_cycles(1_048_576, 64).unwrap().raw(),
            16_384
        );
        assert_eq!(
            synthesize_i_gain_from_cycles(131_072, 64).unwrap().raw(),
            2_048
        );
        assert_eq!(
            integral_tau_us(PidIGain::new(16_384).unwrap(), 25_000),
            Some(2_560)
        );
    }

    #[test]
    fn half_ties_round_away_from_zero() {
        assert_eq!(round_positive_ratio(5, 2).unwrap(), 3);
        assert_eq!(round_positive_ratio(7, 2).unwrap(), 4);
    }

    #[test]
    fn zero_inputs_fail_and_positive_results_are_never_clamped_to_one() {
        assert_eq!(
            synthesize_i_gain_from_cycles(0, 1),
            Err(IntegralGainError::InvalidInput)
        );
        assert_eq!(
            synthesize_i_gain_from_cycles(1, 0),
            Err(IntegralGainError::InvalidInput)
        );
        assert_eq!(
            synthesize_i_gain(1, 0, 25_000),
            Err(IntegralGainError::InvalidInput)
        );
        assert_eq!(synthesize_i_gain_from_cycles(1, 3).unwrap(), PidIGain::ZERO);
    }

    #[test]
    fn whole_cycle_range_boundary_is_exact() {
        assert_eq!(
            synthesize_i_gain_from_cycles(1_048_576, 33).unwrap().raw(),
            31_775
        );
        assert!(matches!(
            synthesize_i_gain_from_cycles(1_048_576, 32),
            Err(IntegralGainError::GainOutOfRange {
                attempted_raw: 32_768
            })
        ));
        assert_eq!(
            synthesize_i_gain_from_cycles(131_072, 5).unwrap().raw(),
            26_214
        );
        assert!(matches!(
            synthesize_i_gain_from_cycles(131_072, 4),
            Err(IntegralGainError::GainOutOfRange {
                attempted_raw: 32_768
            })
        ));
    }

    #[test]
    fn migration_boundary_is_checked() {
        assert_eq!(migrate_q8_8_i_for_test(2_047).unwrap().raw(), 32_752);
        assert!(migrate_q8_8_i_for_test(2_048).is_err());
    }
}
