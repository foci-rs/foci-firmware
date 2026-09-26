/// Fixed-point scale applied to velocity rates expressed in encoder counts per second.
pub const VELOCITY_RATE_Q_SCALE: i64 = 1_000;

const MICROS_PER_SECOND: i128 = 1_000_000;

/// Failure while constructing or combining fixed-memory velocity evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityMeasurementError {
    /// An observation or block has no elapsed time.
    ZeroElapsed,
    /// A confidence family is empty or exceeds the audited table.
    InvalidConfidenceFamilySize,
    /// A checked integer operation exceeded its representation.
    ArithmeticOverflow,
}

/// One signed encoder displacement observed over an actual elapsed interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RawVelocityObservation {
    /// Wrap-aware signed mechanical encoder displacement.
    pub delta_counts: i32,
    /// Actual elapsed time between the encoder reads.
    pub elapsed_us: u32,
}

impl RawVelocityObservation {
    /// Construct an observation with a non-zero elapsed interval.
    pub const fn new(delta_counts: i32, elapsed_us: u32) -> Result<Self, VelocityMeasurementError> {
        if elapsed_us == 0 {
            Err(VelocityMeasurementError::ZeroElapsed)
        } else {
            Ok(Self {
                delta_counts,
                elapsed_us,
            })
        }
    }
}

/// Exact raw totals for one completed dyadic velocity block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RateBlock {
    /// Sum of signed encoder displacements in the block.
    pub delta_counts: i64,
    /// Sum of actual elapsed intervals in the block.
    pub elapsed_us: u64,
    /// Number of raw observations represented by the block.
    pub observation_count: u32,
}

impl RateBlock {
    /// Promote one validated raw observation into a level-zero block.
    pub fn from_observation(
        observation: RawVelocityObservation,
    ) -> Result<Self, VelocityMeasurementError> {
        if observation.elapsed_us == 0 {
            return Err(VelocityMeasurementError::ZeroElapsed);
        }
        Ok(Self {
            delta_counts: i64::from(observation.delta_counts),
            elapsed_us: u64::from(observation.elapsed_us),
            observation_count: 1,
        })
    }

    /// Merge adjacent raw blocks without converting either block to a rate first.
    pub fn checked_merge(self, other: Self) -> Result<Self, VelocityMeasurementError> {
        let delta_counts = self
            .delta_counts
            .checked_add(other.delta_counts)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let elapsed_us = self
            .elapsed_us
            .checked_add(other.elapsed_us)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        if elapsed_us == 0 {
            return Err(VelocityMeasurementError::ZeroElapsed);
        }
        let observation_count = self
            .observation_count
            .checked_add(other.observation_count)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        Ok(Self {
            delta_counts,
            elapsed_us,
            observation_count,
        })
    }

    /// Return the block's time-weighted mean rate in fixed-point counts per second.
    pub fn rate_q(self) -> Result<i64, VelocityMeasurementError> {
        if self.elapsed_us == 0 {
            return Err(VelocityMeasurementError::ZeroElapsed);
        }
        let numerator = i128::from(self.delta_counts)
            .checked_mul(MICROS_PER_SECOND)
            .and_then(|value| value.checked_mul(i128::from(VELOCITY_RATE_Q_SCALE)))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let rate = numerator / i128::from(self.elapsed_us);
        i64::try_from(rate).map_err(|_| VelocityMeasurementError::ArithmeticOverflow)
    }
}

/// Mergeable Welford moments over completed fixed-point block rates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BlockMoments {
    /// Number of completed block rates observed.
    pub count: u32,
    /// Running fixed-point mean block rate.
    pub mean_rate_q: i64,
    /// Sum of squared deviations from the running mean.
    pub m2_rate_q2: i128,
}

impl BlockMoments {
    /// Construct empty block-rate moments.
    pub const fn new() -> Self {
        Self {
            count: 0,
            mean_rate_q: 0,
            m2_rate_q2: 0,
        }
    }

    /// Add one completed block rate with checked integer Welford arithmetic.
    pub fn observe_rate_q(&mut self, rate_q: i64) -> Result<(), VelocityMeasurementError> {
        if self.count == 0 {
            self.count = 1;
            self.mean_rate_q = rate_q;
            return Ok(());
        }

        let next_count = self
            .count
            .checked_add(1)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let delta = i128::from(rate_q) - i128::from(self.mean_rate_q);
        let next_mean = i128::from(self.mean_rate_q) + delta / i128::from(next_count);
        let next_mean_i64 =
            i64::try_from(next_mean).map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
        let delta_after = i128::from(rate_q) - next_mean;
        let m2_increment = delta
            .checked_mul(delta_after)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let next_m2 = self
            .m2_rate_q2
            .checked_add(m2_increment)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;

        self.count = next_count;
        self.mean_rate_q = next_mean_i64;
        self.m2_rate_q2 = next_m2;
        Ok(())
    }

    /// Return the completed-block sample variance in squared fixed-point rate units.
    pub fn sample_variance_rate_q2(self) -> Option<u128> {
        if self.count < 2 {
            return None;
        }
        let m2 = u128::try_from(self.m2_rate_q2).ok()?;
        Some(m2 / u128::from(self.count - 1))
    }
}

impl Default for BlockMoments {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        BlockMoments, RateBlock, RawVelocityObservation, VELOCITY_RATE_Q_SCALE,
        VelocityMeasurementError,
    };

    #[test]
    fn raw_observation_preserves_signed_displacement_and_elapsed_time() {
        let observation = RawVelocityObservation::new(-7, 1_037).unwrap();

        assert_eq!(observation.delta_counts, -7);
        assert_eq!(observation.elapsed_us, 1_037);
    }

    #[test]
    fn raw_observation_rejects_zero_elapsed_time() {
        assert_eq!(
            RawVelocityObservation::new(1, 0),
            Err(VelocityMeasurementError::ZeroElapsed)
        );
    }

    #[test]
    fn merged_block_rate_uses_total_displacement_over_total_time() {
        let fast =
            RateBlock::from_observation(RawVelocityObservation::new(1, 1_000).unwrap()).unwrap();
        let slow =
            RateBlock::from_observation(RawVelocityObservation::new(1, 2_000).unwrap()).unwrap();

        let merged = fast.checked_merge(slow).unwrap();

        assert_eq!(merged.delta_counts, 2);
        assert_eq!(merged.elapsed_us, 3_000);
        assert_eq!(merged.observation_count, 2);
        assert_eq!(merged.rate_q().unwrap(), 666_666);
        assert_ne!(merged.rate_q().unwrap(), 750_000);
        assert_eq!(VELOCITY_RATE_Q_SCALE, 1_000);
    }

    #[test]
    fn block_merge_rejects_displacement_overflow() {
        let left = RateBlock {
            delta_counts: i64::MAX,
            elapsed_us: 1,
            observation_count: 1,
        };
        let right = RateBlock {
            delta_counts: 1,
            elapsed_us: 1,
            observation_count: 1,
        };

        assert_eq!(
            left.checked_merge(right),
            Err(VelocityMeasurementError::ArithmeticOverflow)
        );
    }

    #[test]
    fn block_merge_rejects_elapsed_time_overflow() {
        let left = RateBlock {
            delta_counts: 0,
            elapsed_us: u64::MAX,
            observation_count: 1,
        };
        let right = RateBlock {
            delta_counts: 0,
            elapsed_us: 1,
            observation_count: 1,
        };

        assert_eq!(
            left.checked_merge(right),
            Err(VelocityMeasurementError::ArithmeticOverflow)
        );
    }

    #[test]
    fn block_moments_track_integer_welford_state() {
        let mut moments = BlockMoments::new();

        moments.observe_rate_q(1_000).unwrap();
        moments.observe_rate_q(3_000).unwrap();

        assert_eq!(moments.count, 2);
        assert_eq!(moments.mean_rate_q, 2_000);
        assert_eq!(moments.m2_rate_q2, 2_000_000);
        assert_eq!(moments.sample_variance_rate_q2(), Some(2_000_000));
    }

    #[test]
    fn moment_update_rejects_accumulation_overflow() {
        let mut moments = BlockMoments {
            count: 1,
            mean_rate_q: 0,
            m2_rate_q2: i128::MAX,
        };

        assert_eq!(
            moments.observe_rate_q(1),
            Err(VelocityMeasurementError::ArithmeticOverflow)
        );
    }
}
