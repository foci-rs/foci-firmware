use super::{BlockMoments, RateBlock, RawVelocityObservation, VelocityMeasurementError};

/// Minimum completed-block population accepted at the selected blocking level.
pub const VELOCITY_MIN_SELECTED_BLOCKS: u32 = 16;

/// One fixed-memory level of the online dyadic blocker.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BlockLevel {
    /// Unpaired exact child block at this level.
    pub pending_child: Option<RateBlock>,
    /// Moments over every completed block rate observed at this level.
    pub completed: BlockMoments,
}

impl BlockLevel {
    const fn new() -> Self {
        Self {
            pending_child: None,
            completed: BlockMoments::new(),
        }
    }
}

/// Bounded online blocker that forms successively doubled observation blocks.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DyadicBlocker<const LEVELS: usize> {
    levels: [BlockLevel; LEVELS],
}

/// Qualified variance-of-the-mean plateau selected from the blocker.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PlateauSelection {
    /// Zero-based dyadic blocking level.
    pub level: u8,
    /// Completed block population at the selected level.
    pub block_count: u32,
    /// Estimated variance of the overall mean in squared fixed-point rate units.
    pub variance_of_mean_q2: u128,
}

impl<const LEVELS: usize> DyadicBlocker<LEVELS> {
    /// Construct an empty fixed-memory blocker.
    pub const fn new() -> Self {
        Self {
            levels: [BlockLevel::new(); LEVELS],
        }
    }

    /// Add one raw observation and propagate every completed dyadic pair.
    pub fn observe(
        &mut self,
        observation: RawVelocityObservation,
    ) -> Result<(), VelocityMeasurementError> {
        let mut block = Some(RateBlock::from_observation(observation)?);
        let mut level_index = 0;

        while let Some(current) = block {
            if level_index >= LEVELS {
                break;
            }

            self.levels[level_index]
                .completed
                .observe_rate_q(current.rate_q()?)?;
            if let Some(pending) = self.levels[level_index].pending_child.take() {
                block = Some(pending.checked_merge(current)?);
                level_index += 1;
            } else {
                self.levels[level_index].pending_child = Some(current);
                block = None;
            }
        }

        Ok(())
    }

    /// Discard every incomplete block before a signed-step boundary.
    pub fn discard_partial_blocks(&mut self) {
        for level in &mut self.levels {
            level.pending_child = None;
        }
    }

    /// Merge completed block populations without joining partial blocks across strokes.
    pub fn merge_completed(&mut self, other: &Self) -> Result<(), VelocityMeasurementError> {
        for (target, source) in self.levels.iter_mut().zip(other.levels.iter()) {
            target.completed.merge(source.completed)?;
        }
        Ok(())
    }

    /// Select the coarsest supported level in a two-doubling plateau run.
    pub fn select_plateau(&self) -> Option<PlateauSelection> {
        if LEVELS < 3 {
            return None;
        }

        let mut consecutive_pairs = 0usize;
        let mut selected = None;

        for lower_index in 0..LEVELS - 1 {
            let lower = &self.levels[lower_index].completed;
            let upper = &self.levels[lower_index + 1].completed;
            if plateau_like(lower, upper) {
                consecutive_pairs += 1;
                if consecutive_pairs >= 2 {
                    let first_level = lower_index + 1 - consecutive_pairs;
                    let last_level = lower_index + 1;
                    for candidate_index in (first_level..=last_level).rev() {
                        let candidate = &self.levels[candidate_index].completed;
                        if candidate.count < VELOCITY_MIN_SELECTED_BLOCKS {
                            continue;
                        }
                        let variance_of_mean_q2 = variance_of_mean_q2(candidate)?;
                        selected = Some(PlateauSelection {
                            level: u8::try_from(candidate_index).ok()?,
                            block_count: candidate.count,
                            variance_of_mean_q2,
                        });
                        break;
                    }
                }
            } else {
                consecutive_pairs = 0;
            }
        }

        selected
    }
}

impl<const LEVELS: usize> Default for DyadicBlocker<LEVELS> {
    fn default() -> Self {
        Self::new()
    }
}

fn variance_of_mean_q2(moments: &BlockMoments) -> Option<u128> {
    moments
        .sample_variance_rate_q2()
        .map(|variance| variance / u128::from(moments.count))
}

fn plateau_like(lower: &BlockMoments, upper: &BlockMoments) -> bool {
    let Some(lower_q) = variance_of_mean_q2(lower) else {
        return false;
    };
    let Some(upper_q) = variance_of_mean_q2(upper) else {
        return false;
    };
    let lower_q_f = lower_q as f32;
    let upper_q_f = upper_q as f32;
    let lower_se = lower_q_f * libm::sqrtf(2.0 / (lower.count - 1) as f32);
    let upper_se = upper_q_f * libm::sqrtf(2.0 / (upper.count - 1) as f32);
    let difference_threshold = 1.96 * libm::sqrtf(lower_se * lower_se + upper_se * upper_se);

    upper_q_f - lower_q_f <= difference_threshold
}

#[cfg(test)]
mod tests {
    use core::mem::size_of;

    use super::{
        BlockMoments, DyadicBlocker, PlateauSelection, RawVelocityObservation,
        VELOCITY_MIN_SELECTED_BLOCKS, plateau_like,
    };

    const TEST_LEVELS: usize = 8;
    const MAX_BLOCKER_BYTES: usize = 768;

    fn observation(delta_counts: i32) -> RawVelocityObservation {
        RawVelocityObservation::new(delta_counts, 1_000).unwrap()
    }

    fn moments_with_q(block_count: u32, variance_of_mean_q2: u128) -> BlockMoments {
        BlockMoments {
            count: block_count,
            mean_rate_q: 0,
            m2_rate_q2: i128::try_from(
                variance_of_mean_q2 * u128::from(block_count) * u128::from(block_count - 1),
            )
            .unwrap(),
        }
    }

    fn set_level_q<const LEVELS: usize>(
        blocker: &mut DyadicBlocker<LEVELS>,
        level: usize,
        block_count: u32,
        variance_of_mean_q2: u128,
    ) {
        blocker.levels[level].completed = moments_with_q(block_count, variance_of_mean_q2);
    }

    #[test]
    fn observations_propagate_to_each_completed_dyadic_level() {
        let mut blocker = DyadicBlocker::<TEST_LEVELS>::new();

        for delta_counts in [1, 0, 1, 1] {
            blocker.observe(observation(delta_counts)).unwrap();
        }

        assert_eq!(blocker.levels[0].completed.count, 4);
        assert_eq!(blocker.levels[1].completed.count, 2);
        assert_eq!(blocker.levels[2].completed.count, 1);
        assert_eq!(blocker.levels[2].completed.mean_rate_q, 750_000);
        assert!(blocker.levels[0].pending_child.is_none());
        assert!(blocker.levels[1].pending_child.is_none());
        assert!(blocker.levels[2].pending_child.is_some());
    }

    #[test]
    fn discarding_partial_blocks_preserves_completed_moments() {
        let mut blocker = DyadicBlocker::<TEST_LEVELS>::new();
        for delta_counts in [1, 0, 1] {
            blocker.observe(observation(delta_counts)).unwrap();
        }
        let completed_counts = core::array::from_fn::<_, TEST_LEVELS, _>(|level| {
            blocker.levels[level].completed.count
        });

        blocker.discard_partial_blocks();

        for (level, expected_count) in completed_counts.into_iter().enumerate() {
            assert_eq!(blocker.levels[level].completed.count, expected_count);
            assert!(blocker.levels[level].pending_child.is_none());
        }
    }

    #[test]
    fn completed_moments_merge_without_crossing_stroke_boundaries() {
        let mut first = DyadicBlocker::<TEST_LEVELS>::new();
        let mut second = DyadicBlocker::<TEST_LEVELS>::new();
        for delta_counts in [1, 0, 1] {
            first.observe(observation(delta_counts)).unwrap();
        }
        for delta_counts in [0, 1, 0] {
            second.observe(observation(delta_counts)).unwrap();
        }

        first.merge_completed(&second).unwrap();

        assert_eq!(first.levels[0].completed.count, 6);
        assert_eq!(first.levels[1].completed.count, 2);
        assert!(first.levels[2].completed.count == 0);
        assert!(first.levels[0].pending_child.is_some());
    }

    #[test]
    fn plateau_requires_two_consecutive_qualifying_doublings() {
        let mut blocker = DyadicBlocker::<4>::new();
        set_level_q(&mut blocker, 0, 64, 100);
        set_level_q(&mut blocker, 1, 32, 105);
        set_level_q(&mut blocker, 2, 16, 10_000);

        assert_eq!(blocker.select_plateau(), None);
    }

    #[test]
    fn correlated_false_plateau_at_one_doubling_remains_inconclusive() {
        let mut blocker = DyadicBlocker::<4>::new();
        for sample_index in 0..256 {
            let alternating = if sample_index % 2 == 0 { 10 } else { -10 };
            let slow_square = if (sample_index / 8) % 2 == 0 { 10 } else { -10 };
            blocker
                .observe(observation(alternating + slow_square))
                .unwrap();
        }

        assert!(plateau_like(
            &blocker.levels[0].completed,
            &blocker.levels[1].completed
        ));
        assert!(!plateau_like(
            &blocker.levels[1].completed,
            &blocker.levels[2].completed
        ));
        assert_eq!(blocker.select_plateau(), None);
    }

    #[test]
    fn plateau_selects_coarsest_qualified_level_with_enough_blocks() {
        let mut blocker = DyadicBlocker::<4>::new();
        set_level_q(&mut blocker, 0, 128, 100);
        set_level_q(&mut blocker, 1, 64, 105);
        set_level_q(&mut blocker, 2, 32, 103);
        set_level_q(&mut blocker, 3, 16, 101);

        assert_eq!(
            blocker.select_plateau(),
            Some(PlateauSelection {
                level: 3,
                block_count: 16,
                variance_of_mean_q2: 101,
            })
        );
    }

    #[test]
    fn plateau_never_selects_a_level_below_the_block_floor() {
        let mut blocker = DyadicBlocker::<4>::new();
        set_level_q(&mut blocker, 0, 32, 100);
        set_level_q(&mut blocker, 1, VELOCITY_MIN_SELECTED_BLOCKS, 100);
        set_level_q(&mut blocker, 2, VELOCITY_MIN_SELECTED_BLOCKS - 1, 100);

        let selection = blocker.select_plateau().unwrap();

        assert_eq!(selection.level, 1);
        assert_eq!(selection.block_count, VELOCITY_MIN_SELECTED_BLOCKS);
    }

    #[test]
    fn maximum_level_without_two_doublings_is_inconclusive() {
        let mut blocker = DyadicBlocker::<2>::new();
        set_level_q(&mut blocker, 0, 32, 100);
        set_level_q(&mut blocker, 1, 16, 100);

        assert_eq!(blocker.select_plateau(), None);
    }

    #[test]
    fn quantized_encoder_sequence_reaches_a_supported_plateau() {
        let mut blocker = DyadicBlocker::<TEST_LEVELS>::new();
        for _ in 0..128 {
            for delta_counts in [1, 0, 1, 1, 0] {
                blocker.observe(observation(delta_counts)).unwrap();
            }
        }

        let selection = blocker.select_plateau().unwrap();

        assert!(selection.block_count >= VELOCITY_MIN_SELECTED_BLOCKS);
        assert!(selection.level >= 2);
    }

    #[test]
    fn positively_correlated_sequence_uses_a_coarser_plateau() {
        let mut blocker = DyadicBlocker::<TEST_LEVELS>::new();
        for _ in 0..32 {
            for delta_counts in [1; 8] {
                blocker.observe(observation(delta_counts)).unwrap();
            }
            for delta_counts in [0; 8] {
                blocker.observe(observation(delta_counts)).unwrap();
            }
        }

        let selection = blocker.select_plateau().unwrap();

        assert!(selection.block_count >= VELOCITY_MIN_SELECTED_BLOCKS);
        assert!(selection.level >= 3);
    }

    #[test]
    fn blocker_has_a_bounded_static_memory_footprint() {
        assert!(size_of::<DyadicBlocker<TEST_LEVELS>>() <= MAX_BLOCKER_BYTES);
    }
}
