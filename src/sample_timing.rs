//! Canonical sample-timing helpers for the inner commissioning sample
//! loops.
//!
//! [`sample_offset_us`] and [`sample_elapsed_us`] formalize the two timing
//! conventions used by electrical-delay analysis and current-validation
//! metric extraction. Phase 1 keeps the existing first-sample-at-`now +
//! period` capture behavior — these helpers only document and centralize
//! the arithmetic.

/// Sample offset from the capture origin (zero-based index times period).
///
/// Use when comparing two captured samples; the capture origin cancels
/// out in the difference.
#[inline]
pub const fn sample_offset_us(index: usize, period_us: u32) -> u32 {
    (index as u32) * period_us
}

/// Elapsed duration from the command step through the sample period
/// containing the indexed sample.
///
/// Use when reporting an "elapsed since command" duration, e.g. settling
/// time. Equal to `(index + 1) * period`, matching the firmware capture
/// convention where the first sample is taken one period after the
/// command step.
#[inline]
pub const fn sample_elapsed_us(index: usize, period_us: u32) -> u32 {
    ((index as u32) + 1) * period_us
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_offset_us_zero_at_first_sample() {
        assert_eq!(sample_offset_us(0, 160), 0);
    }

    #[test]
    fn sample_offset_us_scales_linearly() {
        assert_eq!(sample_offset_us(7, 160), 7 * 160);
    }

    #[test]
    fn sample_elapsed_us_equals_one_period_at_first_sample() {
        assert_eq!(sample_elapsed_us(0, 160), 160);
    }

    #[test]
    fn sample_elapsed_us_scales_with_index_plus_one() {
        assert_eq!(sample_elapsed_us(7, 160), 8 * 160);
    }
}
