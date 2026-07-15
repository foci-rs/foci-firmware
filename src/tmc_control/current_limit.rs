//! Checked conversion into the TMC4671 torque/flux current-limit domain.

use foci_tmc4671::current_limit::PidTorqueFluxLimit;

/// Failure to convert an RMS current request into the chip register domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CurrentLimitConversionError {
    /// The board current scale is zero, negative, or non-finite.
    InvalidScale,
    /// The floating-point conversion produced a non-finite result.
    NonFiniteResult,
    /// The truncated count lies outside `0..=0x7fff`.
    OutOfRange,
}

/// Convert RMS milliamps into a validated peak torque/flux current limit.
///
/// The conversion preserves the established truncation semantics: a finite
/// result in `[32767.0, 32768.0)` becomes `0x7fff` before domain validation.
///
/// # Errors
///
/// Returns a cause-specific rejection for invalid scale metadata, non-finite
/// arithmetic, or a count outside the TMC4671 register domain.
pub fn current_limit_from_run_ma(
    run_ma: u32,
    current_scale_ma_per_lsb: f32,
) -> Result<PidTorqueFluxLimit, CurrentLimitConversionError> {
    if !current_scale_ma_per_lsb.is_finite() || current_scale_ma_per_lsb <= 0.0 {
        return Err(CurrentLimitConversionError::InvalidScale);
    }

    let counts = run_ma as f32 * core::f32::consts::SQRT_2 / current_scale_ma_per_lsb;
    if !counts.is_finite() {
        return Err(CurrentLimitConversionError::NonFiniteResult);
    }
    // A negative finite result is unreachable after validating an unsigned
    // current and positive scale, but remains a domain error if those input
    // contracts change.
    if counts < 0.0 || counts >= PidTorqueFluxLimit::MAX.register_value() as f32 + 1.0 {
        return Err(CurrentLimitConversionError::OutOfRange);
    }

    PidTorqueFluxLimit::new(counts as u32).map_err(|_| CurrentLimitConversionError::OutOfRange)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversion_truncates_before_domain_validation() {
        assert_eq!(
            current_limit_from_run_ma(29_472, 1.272).unwrap(),
            PidTorqueFluxLimit::MAX,
        );
        assert_eq!(
            current_limit_from_run_ma(29_473, 1.272),
            Err(CurrentLimitConversionError::OutOfRange),
        );
    }

    #[test]
    fn conversion_rejects_invalid_scale_metadata() {
        for scale in [0.0, -1.0, f32::INFINITY, f32::NEG_INFINITY, f32::NAN] {
            assert_eq!(
                current_limit_from_run_ma(1_000, scale),
                Err(CurrentLimitConversionError::InvalidScale),
            );
        }
    }

    #[test]
    fn conversion_rejects_nonfinite_and_very_large_results() {
        assert_eq!(
            current_limit_from_run_ma(u32::MAX, f32::MIN_POSITIVE),
            Err(CurrentLimitConversionError::NonFiniteResult),
        );
        assert_eq!(
            current_limit_from_run_ma(u32::MAX, 1.155),
            Err(CurrentLimitConversionError::OutOfRange),
        );
    }
}
