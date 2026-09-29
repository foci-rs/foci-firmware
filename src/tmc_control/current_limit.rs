//! Checked conversion into the TMC4671 torque/flux current-limit domain.

use crate::autotune::Permille;
use crate::state::tmc::TmcChannelState;
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
    if counts < 0.0 || counts >= PidTorqueFluxLimit::MAX.register_value() as f32 + 1.0 {
        return Err(CurrentLimitConversionError::OutOfRange);
    }

    PidTorqueFluxLimit::new(counts as u32).map_err(|_| CurrentLimitConversionError::OutOfRange)
}

/// Diagnostic-current envelope in ADC counts.
///
/// `fraction` is a permille ratio of the configured torque/flux current limit.
/// The product truncates toward zero.
///
/// A zero `current_limit` yields a zero envelope.
#[must_use]
pub fn diagnostic_current_envelope(current_limit: PidTorqueFluxLimit, fraction: Permille) -> u16 {
    let product = u32::from(current_limit) * u32::from(fraction.raw());
    (product / 1000) as u16
}

/// Torque-offset headroom for a committed limit: `floor(limit / 2)`.
///
/// The applied torque/flux limit plus the magnitude of any applied torque offset never exceeds
/// the committed limit while the offset stays within this headroom.
#[must_use]
pub fn offset_headroom(limit: PidTorqueFluxLimit) -> u16 {
    limit.raw() / 2
}

/// Torque/flux limit left after reserving [`offset_headroom`] from `limit`.
#[must_use]
pub fn reserved_limit(limit: PidTorqueFluxLimit) -> PidTorqueFluxLimit {
    PidTorqueFluxLimit::new(u32::from(limit.raw() - offset_headroom(limit)))
        .expect("a reduced validated limit stays in the limit domain")
}

/// Register value for the torque/flux limit the chip should run with on this channel.
///
/// Returns the reserved limit while the offset-headroom reservation is set, and the full
/// committed limit otherwise.
#[must_use]
pub fn operational_limit_register(state: &TmcChannelState) -> u32 {
    use core::sync::atomic::Ordering::Relaxed;
    let limit = state.load_current_limit(Relaxed);
    if state.offset_headroom_reserved(Relaxed) {
        reserved_limit(limit).register_value()
    } else {
        limit.register_value()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::Permille;
    use crate::state::tmc::TmcChannelState;

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

    #[test]
    fn diagnostic_envelope_truncates_toward_zero() {
        let limit = PidTorqueFluxLimit::new(2816).unwrap();
        assert_eq!(diagnostic_current_envelope(limit, Permille::new(600)), 1689);
        assert_eq!(diagnostic_current_envelope(limit, Permille::new(200)), 563);
        assert_eq!(diagnostic_current_envelope(limit, Permille::new(568)), 1599);
        assert_eq!(diagnostic_current_envelope(limit, Permille::new(569)), 1602);
    }

    #[test]
    fn diagnostic_envelope_zero_limit_fails_closed() {
        let envelope = diagnostic_current_envelope(PidTorqueFluxLimit::ZERO, Permille::new(600));
        assert_eq!(envelope, 0);
    }

    #[test]
    fn diagnostic_envelope_saturates_within_u16() {
        let envelope = diagnostic_current_envelope(PidTorqueFluxLimit::MAX, Permille::new(1000));
        assert_eq!(envelope, 0x7fff);
    }

    #[test]
    fn reserved_limit_and_headroom_partition_the_limit() {
        for raw in [0u32, 1, 2, 3, 2557, 0x7fff] {
            let limit = PidTorqueFluxLimit::new(raw).unwrap();

            assert_eq!(u32::from(offset_headroom(limit)), raw / 2);
            assert_eq!(
                u32::from(reserved_limit(limit).raw()) + u32::from(offset_headroom(limit)),
                raw
            );
        }
    }

    #[test]
    fn operational_limit_register_follows_the_reservation_flag() {
        use core::sync::atomic::Ordering::Relaxed;
        let state = TmcChannelState::new();
        state.store_current_limit(PidTorqueFluxLimit::new(2557).unwrap(), Relaxed);

        assert_eq!(operational_limit_register(&state), 2557);
        state.set_offset_headroom_reserved(true, Relaxed);
        assert_eq!(operational_limit_register(&state), 2557 - 1278);
        state.set_offset_headroom_reserved(false, Relaxed);
        assert_eq!(operational_limit_register(&state), 2557);
    }
}
