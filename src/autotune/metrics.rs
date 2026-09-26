//! Checked fixed-point ratios used by autotune acceptance decisions.

/// A non-negative ratio in thousandths.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, defmt::Format)]
pub struct Permille(u16);

impl Permille {
    /// Zero thousandths.
    pub const ZERO: Self = Self(0);
    /// Largest representable saturated ratio.
    pub const MAX: Self = Self(u16::MAX);

    /// Construct a ratio from its raw thousandths representation.
    pub const fn new(raw: u16) -> Self {
        Self(raw)
    }

    /// Return the raw thousandths representation.
    pub const fn raw(self) -> u16 {
        self.0
    }

    /// Compute `floor(numerator * 1000 / denominator)` without overflow.
    ///
    /// Returns `None` when `denominator` is zero and saturates valid results
    /// that exceed the `u16` representation.
    pub fn checked_ratio(numerator: u64, denominator: u64) -> Option<Self> {
        if denominator == 0 {
            return None;
        }
        let scaled = u128::from(numerator) * 1000 / u128::from(denominator);
        Some(Self(scaled.min(u128::from(u16::MAX)) as u16))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_ratio_rejects_zero_denominator() {
        assert_eq!(Permille::checked_ratio(1, 0), None);
    }

    #[test]
    fn checked_ratio_handles_exact_values_and_full_u64_domain() {
        assert_eq!(Permille::checked_ratio(0, 7), Some(Permille::ZERO));
        assert_eq!(Permille::checked_ratio(1, 1), Some(Permille::new(1000)));
        assert_eq!(Permille::checked_ratio(2, 1), Some(Permille::new(2000)));
        assert_eq!(
            Permille::checked_ratio(u64::MAX, u64::MAX),
            Some(Permille::new(1000)),
        );
    }

    #[test]
    fn checked_ratio_saturates_instead_of_wrapping() {
        assert_eq!(Permille::checked_ratio(65_535, 1000), Some(Permille::MAX));
        assert_eq!(Permille::checked_ratio(u64::MAX, 1), Some(Permille::MAX));
        assert_eq!(Permille::checked_ratio(65_536, 1), Some(Permille::MAX));
    }

    #[test]
    fn checked_ratio_is_monotonic_before_saturation() {
        let mut previous = Permille::ZERO;
        for numerator in 0..=10_000 {
            let current = Permille::checked_ratio(numerator, 10_000).unwrap();
            assert!(current >= previous);
            previous = current;
        }
    }
}
