//! Typed normalization and shortest-path arithmetic for modulo encoders.

/// Error returned when encoder-domain arithmetic cannot produce valid evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum EncoderMathError {
    /// The configured counts per revolution is outside the backend contract.
    InvalidCountsPerRev,
    /// An exact half-turn has no unique shortest direction.
    AmbiguousHalfTurn,
}

/// A validated modulo encoder-count domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct EncoderDomain {
    counts_per_rev: u32,
}

impl EncoderDomain {
    /// Creates a domain for the backend's signed quadrature-count contract.
    ///
    /// # Errors
    ///
    /// Returns [`EncoderMathError::InvalidCountsPerRev`] for zero or values
    /// above `i32::MAX`.
    pub(crate) fn new(counts_per_rev: u32) -> Result<Self, EncoderMathError> {
        if counts_per_rev == 0 || counts_per_rev > i32::MAX as u32 {
            return Err(EncoderMathError::InvalidCountsPerRev);
        }

        Ok(Self { counts_per_rev })
    }

    /// Returns the validated encoder counts per mechanical revolution.
    pub(crate) const fn counts_per_rev(self) -> u32 {
        self.counts_per_rev
    }

    /// Normalizes a raw encoder reading into this domain.
    pub(crate) fn count(self, raw_counts: u32) -> EncoderCount {
        EncoderCount(raw_counts % self.counts_per_rev)
    }

    /// Returns the unique shortest displacement from `previous` to `current`.
    ///
    /// # Errors
    ///
    /// Returns [`EncoderMathError::AmbiguousHalfTurn`] when an even domain's
    /// displacement is exactly half a revolution.
    pub(crate) fn delta(
        self,
        previous: EncoderCount,
        current: EncoderCount,
    ) -> Result<EncoderDelta, EncoderMathError> {
        let counts_per_rev = i64::from(self.counts_per_rev);
        let forward = (i64::from(current.0) - i64::from(previous.0)).rem_euclid(counts_per_rev);
        let half_turn = counts_per_rev / 2;

        if counts_per_rev % 2 == 0 && forward == half_turn {
            return Err(EncoderMathError::AmbiguousHalfTurn);
        }

        let shortest = if forward > half_turn {
            forward - counts_per_rev
        } else {
            forward
        };
        // The domain is at most i32::MAX and the shortest displacement is at
        // most half a revolution, so this conversion cannot fail.
        let signed_counts = i32::try_from(shortest).expect("encoder delta fits in i32");

        Ok(EncoderDelta(signed_counts))
    }
}

/// A normalized position within an [`EncoderDomain`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct EncoderCount(u32);

impl EncoderCount {
    /// Returns the normalized raw count value.
    pub(crate) const fn raw_counts(self) -> u32 {
        self.0
    }
}

/// A signed shortest displacement within an [`EncoderDomain`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct EncoderDelta(i32);

impl EncoderDelta {
    /// Returns the signed displacement in encoder counts.
    pub(crate) const fn signed_counts(self) -> i32 {
        self.0
    }

    /// Returns the displacement magnitude in encoder counts.
    pub(crate) const fn absolute_counts(self) -> u32 {
        self.0.unsigned_abs()
    }
}

#[cfg(test)]
mod tests {
    use super::{EncoderDomain, EncoderMathError};

    #[test]
    fn zero_counts_per_rev_is_invalid() {
        assert_eq!(
            EncoderDomain::new(0),
            Err(EncoderMathError::InvalidCountsPerRev)
        );
    }

    #[test]
    fn largest_supported_domain_is_accepted() {
        let domain = EncoderDomain::new(i32::MAX as u32).unwrap();

        assert_eq!(domain.counts_per_rev(), i32::MAX as u32);
    }

    #[test]
    fn first_domain_above_backend_contract_is_rejected() {
        assert_eq!(
            EncoderDomain::new(i32::MAX as u32 + 1),
            Err(EncoderMathError::InvalidCountsPerRev)
        );
    }

    #[test]
    fn raw_counts_are_normalized_into_the_domain() {
        let domain = EncoderDomain::new(4_000).unwrap();

        assert_eq!(domain.count(12_003).raw_counts(), 3);
    }

    #[test]
    fn repeated_count_has_zero_delta() {
        let domain = EncoderDomain::new(4_000).unwrap();
        let count = domain.count(8_321);

        let delta = domain.delta(count, count).unwrap();

        assert_eq!(delta.signed_counts(), 0);
        assert_eq!(delta.absolute_counts(), 0);
    }

    #[test]
    fn forward_origin_crossing_keeps_positive_direction() {
        let domain = EncoderDomain::new(4_000).unwrap();

        let delta = domain.delta(domain.count(3_998), domain.count(2)).unwrap();

        assert_eq!(delta.signed_counts(), 4);
        assert_eq!(delta.absolute_counts(), 4);
    }

    #[test]
    fn reverse_origin_crossing_keeps_negative_direction() {
        let domain = EncoderDomain::new(4_000).unwrap();

        let delta = domain.delta(domain.count(2), domain.count(3_998)).unwrap();

        assert_eq!(delta.signed_counts(), -4);
        assert_eq!(delta.absolute_counts(), 4);
    }

    #[test]
    fn even_domain_rejects_exact_half_turn() {
        let domain = EncoderDomain::new(4_000).unwrap();

        assert_eq!(
            domain.delta(domain.count(3_000), domain.count(1_000)),
            Err(EncoderMathError::AmbiguousHalfTurn)
        );
    }

    #[test]
    fn odd_domain_has_no_half_turn_tie() {
        let domain = EncoderDomain::new(5).unwrap();

        assert_eq!(
            domain
                .delta(domain.count(0), domain.count(2))
                .unwrap()
                .signed_counts(),
            2
        );
        assert_eq!(
            domain
                .delta(domain.count(0), domain.count(3))
                .unwrap()
                .signed_counts(),
            -2
        );
    }

    #[test]
    fn non_ambiguous_deltas_are_antisymmetric() {
        for counts_per_rev in [3, 4, 5, 16, 4_000] {
            let domain = EncoderDomain::new(counts_per_rev).unwrap();
            let samples = [0, 1, counts_per_rev / 2, counts_per_rev - 1];

            for previous in samples {
                for current in samples {
                    let forward = domain.delta(domain.count(previous), domain.count(current));
                    let reverse = domain.delta(domain.count(current), domain.count(previous));

                    match (forward, reverse) {
                        (Ok(forward), Ok(reverse)) => {
                            assert_eq!(forward.signed_counts(), -reverse.signed_counts());
                        }
                        (
                            Err(EncoderMathError::AmbiguousHalfTurn),
                            Err(EncoderMathError::AmbiguousHalfTurn),
                        ) => {}
                        pair => panic!("delta results were not symmetric: {pair:?}"),
                    }
                }
            }
        }
    }

    #[test]
    fn absolute_delta_never_exceeds_half_a_revolution() {
        for counts_per_rev in 1..=32 {
            let domain = EncoderDomain::new(counts_per_rev).unwrap();

            for previous in 0..counts_per_rev {
                for current in 0..counts_per_rev {
                    if let Ok(delta) = domain.delta(domain.count(previous), domain.count(current)) {
                        assert!(delta.absolute_counts() <= counts_per_rev / 2);
                    }
                }
            }
        }
    }
}
