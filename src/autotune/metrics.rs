//! Checked fixed-point ratios used by autotune acceptance decisions.

/// A non-negative ratio in thousandths.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, defmt::Format)]
pub struct Permille(u16);

impl Permille {
    /// Zero thousandths.
    pub const ZERO: Self = Self(0);
    /// One hundred percent, represented as 1000 thousandths.
    pub const HUNDRED_PERCENT: Self = Self(1000);
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

// Task 2 consumes this helper when acceptance metrics migrate to `Permille`.
#[allow(dead_code)]
pub(crate) fn normalized_difference(a: u32, b: u32) -> Permille {
    if a == 0 && b == 0 {
        return Permille::ZERO;
    }
    let numerator = u64::from(a.abs_diff(b)) * 2;
    let denominator = u64::from(a) + u64::from(b);
    Permille::checked_ratio(numerator, denominator).expect("nonzero normalized-difference sum")
}

// Task 2 consumes this helper when acceptance metrics migrate to `Permille`.
#[allow(dead_code)]
pub(crate) fn saturation_permille(saturated_samples: u16, total_samples: u16) -> Option<Permille> {
    Permille::checked_ratio(u64::from(saturated_samples), u64::from(total_samples))
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
        assert_eq!(
            Permille::checked_ratio(1, 1),
            Some(Permille::HUNDRED_PERCENT)
        );
        assert_eq!(Permille::checked_ratio(2, 1), Some(Permille::new(2000)));
        assert_eq!(
            Permille::checked_ratio(u64::MAX, u64::MAX),
            Some(Permille::HUNDRED_PERCENT),
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

    #[test]
    fn normalized_difference_handles_domain_boundaries() {
        assert_eq!(normalized_difference(0, 0), Permille::ZERO);
        assert_eq!(normalized_difference(500, 500), Permille::ZERO);
        assert_eq!(normalized_difference(0, 500), Permille::new(2000));
        assert_eq!(normalized_difference(600, 500), Permille::new(181));
    }

    #[test]
    fn saturation_ratio_preserves_fractional_thresholds() {
        assert_eq!(saturation_permille(100, 1000), Some(Permille::new(100)));
        assert_eq!(saturation_permille(101, 1000), Some(Permille::new(101)));
        assert_eq!(saturation_permille(0, 0), None);
    }
}
