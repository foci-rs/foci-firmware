use super::{
    DyadicBlocker, PlateauSelection, RateBlock, RawVelocityObservation,
    VELOCITY_MIN_SELECTED_BLOCKS, VelocityMeasurementError, confidence_multiplier_95_df15,
    estimate_confidence,
};

/// Nested stationary-tail candidates, anchored at the end of a capture.
pub const VELOCITY_SUFFIX_LENGTHS: [u16; 3] = [64, 128, 256];

const MICROS_PER_SECOND: f32 = 1_000_000.0;
const LAG_ONE_Q_SCALE: f32 = 1_000_000.0;

/// Reason a captured tail cannot support a stationary settled estimate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StationarityExclusion {
    /// No two-consecutive-doubling variance plateau was found.
    NoBlockingPlateau,
    /// The selected scale has fewer than the required decorrelated blocks.
    TooFewDecorrelatedBlocks,
    /// The fitted rate slope remains distinguishable from zero.
    ResolvedSlope,
    /// Detrended residuals retain significant lag-one structure.
    StructuredResidual,
    /// Checked arithmetic or a numeric conversion failed.
    Arithmetic,
}

/// Result of selecting one target-independent stationary tail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StationarityResult {
    /// A contiguous suffix passed every stationarity gate.
    Usable(StationarySuffix),
    /// No declared suffix passed every stationarity gate.
    Excluded(StationarityExclusion),
}

/// Statistics retained for the longest usable stationary suffix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StationarySuffix {
    /// Blocking plateau selected for the suffix.
    pub selection: PlateauSelection,
    /// Number of raw observations in the selected suffix.
    pub suffix_len: u16,
    /// Fitted rate slope in fixed-point rate units per second.
    pub slope_q: i64,
    /// Outward-rounded family-wise slope half-width in the same units.
    pub slope_half_width_q: u64,
    /// Lag-one residual correlation scaled by one million.
    pub lag_one_q: i32,
    /// Outward-rounded lag-one half-width scaled by one million.
    pub lag_one_half_width_q: u32,
    /// Mean squared detrended residual in squared fixed-point rate units.
    pub residual_magnitude_q2: u128,
    /// Number of nested suffix candidates examined.
    pub tested_suffixes: u8,
}

/// Non-binding evidence for one declared suffix length.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct SuffixDiagnostic {
    /// Number of raw observations in this suffix.
    pub suffix_len: u16,
    /// Whether the blocker found a usable two-doubling plateau.
    pub usable_plateau: bool,
    /// Whether the fitted slope confidence interval contains zero.
    pub slope_passed: bool,
    /// Whether the lag-one residual confidence interval contains its null.
    pub lag_one_passed: bool,
    /// Outward-rounded settled-rate confidence interval.
    pub settled_rate_interval_q: [i64; 2],
    /// Positive clearance or negative rejection beyond the slope interval.
    pub slope_rejection_margin_q: i64,
    /// Positive clearance or negative rejection beyond the lag-one interval.
    pub lag_one_rejection_margin_q: i32,
}

/// Binding stationarity result plus all declared suffix diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StationarityDiagnostics {
    /// Unchanged shortest-first, no-stitch stationarity decision.
    pub binding: StationarityResult,
    /// Available suffix diagnostics in [`VELOCITY_SUFFIX_LENGTHS`] order.
    pub suffixes: [Option<SuffixDiagnostic>; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StationarityMetrics {
    slope_q: i64,
    slope_half_width_q: u64,
    lag_one_q: i32,
    lag_one_half_width_q: u32,
    residual_magnitude_q2: u128,
    slope_passed: bool,
    lag_one_passed: bool,
    slope_rejection_margin_q: i64,
    lag_one_rejection_margin_q: i32,
}

/// Maximum raw observations consumed by one cooperative stationarity step.
///
/// Eight keeps one F407 work quantum below the outer-safety observation-gap
/// guard while leaving complete analysis inside the settle-derived budget.
pub const STATIONARITY_WORK_OBSERVATIONS_PER_STEP: usize = 8;

/// Result of one bounded cooperative stationarity-analysis step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StationarityWorkPoll {
    /// More bounded work remains.
    Pending,
    /// The declared suffix family has been classified.
    Complete(StationarityResult),
}

#[derive(Debug, Clone, Copy)]
enum StationarityWorkStage {
    Idle,
    Blocking {
        suffix_index: u8,
        suffix_start: u16,
        suffix_len: u16,
        next_index: u16,
    },
    Regression {
        suffix_index: u8,
        suffix_start: u16,
        suffix_len: u16,
        block_observations: u16,
        block_count: u16,
        next_block: u16,
        next_observation: u16,
        partial_block: Option<RateBlock>,
        cumulative_elapsed_us: u64,
        sum_x: i128,
        sum_y: i128,
        sum_x2: i128,
        sum_xy: i128,
    },
    Residual {
        suffix_index: u8,
        suffix_start: u16,
        suffix_len: u16,
        block_observations: u16,
        block_count: u16,
        next_block: u16,
        next_observation: u16,
        partial_block: Option<RateBlock>,
        cumulative_elapsed_us: u64,
        slope_per_us: f32,
        intercept: f32,
        denominator: i128,
        residual_sum_q2: f32,
        lag_product_sum_q2: f32,
        previous_residual: Option<f32>,
    },
    Done,
}

/// Resumable nested-suffix stationarity analysis for a fixed capture.
#[derive(Debug, Clone, Copy)]
pub struct StationarityWork {
    stage: StationarityWorkStage,
    selected: Option<StationarySuffix>,
    tested_suffixes: u8,
}

impl StationarityWork {
    /// Construct an idle work item.
    pub const fn new() -> Self {
        Self {
            stage: StationarityWorkStage::Idle,
            selected: None,
            tested_suffixes: 0,
        }
    }

    /// Reset all progress before analyzing another fixed capture.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Advance analysis by at most [`STATIONARITY_WORK_OBSERVATIONS_PER_STEP`]
    /// raw-observation visits.
    pub fn poll<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        if matches!(self.stage, StationarityWorkStage::Idle) {
            let Some((suffix_index, suffix_len)) = first_available_suffix(observations) else {
                self.stage = StationarityWorkStage::Done;
                return Ok(StationarityWorkPoll::Complete(
                    StationarityResult::Excluded(StationarityExclusion::TooFewDecorrelatedBlocks),
                ));
            };
            self.start_suffix(observations, blocker, suffix_index, suffix_len)?;
        }

        match self.stage {
            StationarityWorkStage::Blocking { .. } => self.poll_blocking(observations, blocker),
            StationarityWorkStage::Regression { .. } => self.poll_regression(observations),
            StationarityWorkStage::Residual { .. } => self.poll_residual(observations, blocker),
            StationarityWorkStage::Done => Err(VelocityMeasurementError::ArithmeticOverflow),
            StationarityWorkStage::Idle => unreachable!("idle work starts before dispatch"),
        }
    }

    fn start_suffix<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
        suffix_index: u8,
        suffix_len: u16,
    ) -> Result<(), VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        self.tested_suffixes = self
            .tested_suffixes
            .checked_add(1)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        *blocker = DyadicBlocker::new();
        let suffix_start = observations
            .observation_len()
            .checked_sub(usize::from(suffix_len))
            .and_then(|value| u16::try_from(value).ok())
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        self.stage = StationarityWorkStage::Blocking {
            suffix_index,
            suffix_start,
            suffix_len,
            next_index: suffix_start,
        };
        Ok(())
    }

    fn poll_blocking<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkStage::Blocking {
            suffix_index,
            suffix_start,
            suffix_len,
            mut next_index,
        } = self.stage
        else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let end = suffix_start
            .checked_add(suffix_len)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let step_end = end.min(
            next_index
                .checked_add(STATIONARITY_WORK_OBSERVATIONS_PER_STEP as u16)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
        );
        while next_index < step_end {
            blocker.observe(
                observations
                    .observation(usize::from(next_index))
                    .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
            )?;
            next_index += 1;
        }
        if next_index < end {
            self.stage = StationarityWorkStage::Blocking {
                suffix_index,
                suffix_start,
                suffix_len,
                next_index,
            };
            return Ok(StationarityWorkPoll::Pending);
        }

        let Some(selection) = blocker.select_plateau() else {
            let exclusion = if blocker.has_two_doubling_plateau() {
                StationarityExclusion::TooFewDecorrelatedBlocks
            } else {
                StationarityExclusion::NoBlockingPlateau
            };
            return Ok(self.finish_on_exclusion(exclusion));
        };
        let block_observations = 1u16
            .checked_shl(u32::from(selection.level))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let block_count = suffix_len / block_observations;
        if usize::from(block_count) < VELOCITY_MIN_SELECTED_BLOCKS as usize {
            return Ok(self.finish_on_exclusion(StationarityExclusion::TooFewDecorrelatedBlocks));
        }
        self.stage = StationarityWorkStage::Regression {
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            next_block: 0,
            next_observation: 0,
            partial_block: None,
            cumulative_elapsed_us: 0,
            sum_x: 0,
            sum_y: 0,
            sum_x2: 0,
            sum_xy: 0,
        };
        Ok(StationarityWorkPoll::Pending)
    }

    fn poll_regression<S>(
        &mut self,
        observations: &S,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkStage::Regression {
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            mut next_block,
            mut next_observation,
            mut partial_block,
            mut cumulative_elapsed_us,
            mut sum_x,
            mut sum_y,
            mut sum_x2,
            mut sum_xy,
        } = self.stage
        else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let mut remaining_visits = STATIONARITY_WORK_OBSERVATIONS_PER_STEP;
        while next_block < block_count && remaining_visits > 0 {
            let index = usize::from(suffix_start)
                .checked_add(
                    usize::from(next_block)
                        .checked_mul(usize::from(block_observations))
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .and_then(|value| value.checked_add(usize::from(next_observation)))
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let observation = observations
                .observation(index)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let next = RateBlock::from_observation(observation)?;
            partial_block = Some(match partial_block {
                Some(current) => current.checked_merge(next)?,
                None => next,
            });
            next_observation = next_observation
                .checked_add(1)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            remaining_visits -= 1;
            if next_observation < block_observations {
                continue;
            }
            let block = partial_block
                .take()
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let midpoint_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us / 2)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            cumulative_elapsed_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let x = i128::from(midpoint_us);
            let y = i128::from(block.rate_q()?);
            sum_x = sum_x
                .checked_add(x)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            sum_y = sum_y
                .checked_add(y)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            sum_x2 = sum_x2
                .checked_add(
                    x.checked_mul(x)
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            sum_xy = sum_xy
                .checked_add(
                    x.checked_mul(y)
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            next_block += 1;
            next_observation = 0;
        }
        if next_block < block_count {
            self.stage = StationarityWorkStage::Regression {
                suffix_index,
                suffix_start,
                suffix_len,
                block_observations,
                block_count,
                next_block,
                next_observation,
                partial_block,
                cumulative_elapsed_us,
                sum_x,
                sum_y,
                sum_x2,
                sum_xy,
            };
            return Ok(StationarityWorkPoll::Pending);
        }

        let n = i128::from(block_count);
        let denominator = n
            .checked_mul(sum_x2)
            .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_x)?))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        if denominator <= 0 {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        }
        let numerator = n
            .checked_mul(sum_xy)
            .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_y)?))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let slope_per_us = numerator as f32 / denominator as f32;
        let intercept = (sum_y as f32 - slope_per_us * sum_x as f32) / block_count as f32;
        if !slope_per_us.is_finite() || !intercept.is_finite() {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        }
        self.stage = StationarityWorkStage::Residual {
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            next_block: 0,
            next_observation: 0,
            partial_block: None,
            cumulative_elapsed_us: 0,
            slope_per_us,
            intercept,
            denominator,
            residual_sum_q2: 0.0,
            lag_product_sum_q2: 0.0,
            previous_residual: None,
        };
        Ok(StationarityWorkPoll::Pending)
    }

    fn poll_residual<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkStage::Residual {
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            mut next_block,
            mut next_observation,
            mut partial_block,
            mut cumulative_elapsed_us,
            slope_per_us,
            intercept,
            denominator,
            mut residual_sum_q2,
            mut lag_product_sum_q2,
            mut previous_residual,
        } = self.stage
        else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let mut remaining_visits = STATIONARITY_WORK_OBSERVATIONS_PER_STEP;
        while next_block < block_count && remaining_visits > 0 {
            let index = usize::from(suffix_start)
                .checked_add(
                    usize::from(next_block)
                        .checked_mul(usize::from(block_observations))
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .and_then(|value| value.checked_add(usize::from(next_observation)))
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let observation = observations
                .observation(index)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let next = RateBlock::from_observation(observation)?;
            partial_block = Some(match partial_block {
                Some(current) => current.checked_merge(next)?,
                None => next,
            });
            next_observation = next_observation
                .checked_add(1)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            remaining_visits -= 1;
            if next_observation < block_observations {
                continue;
            }
            let block = partial_block
                .take()
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let midpoint_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us / 2)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            cumulative_elapsed_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let residual = block.rate_q()? as f32 - (intercept + slope_per_us * midpoint_us as f32);
            residual_sum_q2 += residual * residual;
            if let Some(previous) = previous_residual {
                lag_product_sum_q2 += residual * previous;
            }
            previous_residual = Some(residual);
            next_block += 1;
            next_observation = 0;
        }
        if next_block < block_count {
            self.stage = StationarityWorkStage::Residual {
                suffix_index,
                suffix_start,
                suffix_len,
                block_observations,
                block_count,
                next_block,
                next_observation,
                partial_block,
                cumulative_elapsed_us,
                slope_per_us,
                intercept,
                denominator,
                residual_sum_q2,
                lag_product_sum_q2,
                previous_residual,
            };
            return Ok(StationarityWorkPoll::Pending);
        }
        let selection = blocker
            .select_plateau()
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let metrics = finish_stationarity_metrics(
            block_count as usize,
            denominator,
            slope_per_us,
            residual_sum_q2,
            lag_product_sum_q2,
            3,
            selection.variance_of_mean_q2,
        );
        match metrics {
            Ok(metrics) => {
                self.selected = Some(StationarySuffix {
                    selection,
                    suffix_len,
                    slope_q: metrics.slope_q,
                    slope_half_width_q: metrics.slope_half_width_q,
                    lag_one_q: metrics.lag_one_q,
                    lag_one_half_width_q: metrics.lag_one_half_width_q,
                    residual_magnitude_q2: metrics.residual_magnitude_q2,
                    tested_suffixes: self.tested_suffixes,
                });
                self.advance_or_complete(observations, blocker, suffix_index)
            }
            Err(exclusion) => Ok(self.finish_on_exclusion(exclusion)),
        }
    }

    fn finish_on_exclusion(&mut self, exclusion: StationarityExclusion) -> StationarityWorkPoll {
        if let Some(mut usable) = self.selected {
            usable.tested_suffixes = self.tested_suffixes;
            self.stage = StationarityWorkStage::Done;
            return StationarityWorkPoll::Complete(StationarityResult::Usable(usable));
        }
        self.stage = StationarityWorkStage::Done;
        StationarityWorkPoll::Complete(StationarityResult::Excluded(exclusion))
    }

    fn advance_or_complete<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
        suffix_index: u8,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let next_index = suffix_index.saturating_add(1);
        if let Some(&suffix_len) = VELOCITY_SUFFIX_LENGTHS.get(usize::from(next_index))
            && usize::from(suffix_len) <= observations.observation_len()
        {
            self.start_suffix(observations, blocker, next_index, suffix_len)?;
            return Ok(StationarityWorkPoll::Pending);
        }
        let mut result = self
            .selected
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        result.tested_suffixes = self.tested_suffixes;
        self.stage = StationarityWorkStage::Done;
        Ok(StationarityWorkPoll::Complete(StationarityResult::Usable(
            result,
        )))
    }
}

impl Default for StationarityWork {
    fn default() -> Self {
        Self::new()
    }
}

fn first_available_suffix(
    observations: &(impl VelocityObservationSource + ?Sized),
) -> Option<(u8, u16)> {
    VELOCITY_SUFFIX_LENGTHS
        .iter()
        .copied()
        .enumerate()
        .find(|(_, suffix_len)| usize::from(*suffix_len) <= observations.observation_len())
        .and_then(|(index, suffix_len)| u8::try_from(index).ok().map(|index| (index, suffix_len)))
}

/// Random-access source for a fixed chronological velocity capture.
pub trait VelocityObservationSource {
    /// Number of chronological observations available.
    fn observation_len(&self) -> usize;

    /// Return one chronological observation.
    fn observation(&self, index: usize) -> Option<RawVelocityObservation>;
}

impl VelocityObservationSource for [RawVelocityObservation] {
    fn observation_len(&self) -> usize {
        self.len()
    }

    fn observation(&self, index: usize) -> Option<RawVelocityObservation> {
        self.get(index).copied()
    }
}

/// Select the longest contiguous stationary suffix using one reusable blocker.
pub fn select_stationary_suffix<const LEVELS: usize>(
    observations: &[RawVelocityObservation],
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<StationarityResult, VelocityMeasurementError> {
    select_stationary_suffix_from_source(observations, blocker)
}

/// Analyze every available declared suffix without changing the binding verdict.
pub fn analyze_stationarity_diagnostics<const LEVELS: usize, S>(
    observations: &S,
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<StationarityDiagnostics, VelocityMeasurementError>
where
    S: VelocityObservationSource + ?Sized,
{
    let binding = select_stationary_suffix_from_source(observations, blocker)?;
    let mut suffixes = [None; VELOCITY_SUFFIX_LENGTHS.len()];
    for (suffix_index, suffix_len) in VELOCITY_SUFFIX_LENGTHS.iter().copied().enumerate() {
        if usize::from(suffix_len) > observations.observation_len() {
            continue;
        }
        suffixes[suffix_index] = Some(analyze_suffix_diagnostic(
            observations,
            blocker,
            suffix_len,
        )?);
    }
    Ok(StationarityDiagnostics { binding, suffixes })
}

fn analyze_suffix_diagnostic<const LEVELS: usize, S>(
    observations: &S,
    blocker: &mut DyadicBlocker<LEVELS>,
    suffix_len: u16,
) -> Result<SuffixDiagnostic, VelocityMeasurementError>
where
    S: VelocityObservationSource + ?Sized,
{
    *blocker = DyadicBlocker::new();
    let suffix_start = observations
        .observation_len()
        .checked_sub(usize::from(suffix_len))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    for index in suffix_start..observations.observation_len() {
        blocker.observe(
            observations
                .observation(index)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
        )?;
    }
    let Some(selection) = blocker.select_plateau() else {
        return Ok(SuffixDiagnostic {
            suffix_len,
            usable_plateau: false,
            slope_passed: false,
            lag_one_passed: false,
            settled_rate_interval_q: [0, 0],
            slope_rejection_margin_q: 0,
            lag_one_rejection_margin_q: 0,
        });
    };
    let block_observations = 1usize
        .checked_shl(u32::from(selection.level))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let metrics = calculate_stationarity_metrics_from_source(
        observations,
        suffix_start,
        usize::from(suffix_len),
        block_observations,
        3,
        selection.variance_of_mean_q2,
    )
    .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
    let confidence = estimate_confidence(selection, 3)?;
    let mean_rate_q = selection.exact_totals.rate_q()?;
    let half_width = i64::try_from(confidence.half_width_rate_q)
        .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
    let low = mean_rate_q
        .checked_sub(half_width)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let high = mean_rate_q
        .checked_add(half_width)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    Ok(SuffixDiagnostic {
        suffix_len,
        usable_plateau: true,
        slope_passed: metrics.slope_passed,
        lag_one_passed: metrics.lag_one_passed,
        settled_rate_interval_q: [low, high],
        slope_rejection_margin_q: metrics.slope_rejection_margin_q,
        lag_one_rejection_margin_q: metrics.lag_one_rejection_margin_q,
    })
}

/// Select the longest stationary suffix from a fixed random-access source.
pub fn select_stationary_suffix_from_source<const LEVELS: usize, S>(
    observations: &S,
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<StationarityResult, VelocityMeasurementError>
where
    S: VelocityObservationSource + ?Sized,
{
    let mut selected: Option<StationarySuffix> = None;
    let mut tested_suffixes = 0u8;

    for suffix_len in VELOCITY_SUFFIX_LENGTHS {
        let suffix_len = usize::from(suffix_len);
        if suffix_len > observations.observation_len() {
            continue;
        }
        tested_suffixes = tested_suffixes
            .checked_add(1)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        *blocker = DyadicBlocker::new();
        let suffix_start = observations.observation_len() - suffix_len;
        for index in suffix_start..observations.observation_len() {
            blocker.observe(
                observations
                    .observation(index)
                    .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
            )?;
        }

        let Some(selection) = blocker.select_plateau() else {
            if let Some(mut usable) = selected {
                usable.tested_suffixes = tested_suffixes;
                return Ok(StationarityResult::Usable(usable));
            }
            let exclusion = if blocker.has_two_doubling_plateau() {
                StationarityExclusion::TooFewDecorrelatedBlocks
            } else {
                StationarityExclusion::NoBlockingPlateau
            };
            return Ok(StationarityResult::Excluded(exclusion));
        };
        let Some(block_observations) = 1usize.checked_shl(u32::from(selection.level)) else {
            return Ok(StationarityResult::Excluded(
                StationarityExclusion::Arithmetic,
            ));
        };
        let metrics = match analyze_stationarity_from_source(
            observations,
            suffix_start,
            suffix_len,
            block_observations,
            3,
            selection.variance_of_mean_q2,
        ) {
            Ok(metrics) => metrics,
            Err(exclusion) => {
                if let Some(mut usable) = selected {
                    usable.tested_suffixes = tested_suffixes;
                    return Ok(StationarityResult::Usable(usable));
                }
                return Ok(StationarityResult::Excluded(exclusion));
            }
        };
        selected = Some(StationarySuffix {
            selection,
            suffix_len: u16::try_from(suffix_len)
                .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?,
            slope_q: metrics.slope_q,
            slope_half_width_q: metrics.slope_half_width_q,
            lag_one_q: metrics.lag_one_q,
            lag_one_half_width_q: metrics.lag_one_half_width_q,
            residual_magnitude_q2: metrics.residual_magnitude_q2,
            tested_suffixes,
        });
    }

    Ok(selected.map_or(
        StationarityResult::Excluded(StationarityExclusion::TooFewDecorrelatedBlocks),
        StationarityResult::Usable,
    ))
}

fn analyze_stationarity_from_source(
    observations: &(impl VelocityObservationSource + ?Sized),
    suffix_start: usize,
    suffix_len: usize,
    block_observations: usize,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
) -> Result<StationarityMetrics, StationarityExclusion> {
    let metrics = calculate_stationarity_metrics_from_source(
        observations,
        suffix_start,
        suffix_len,
        block_observations,
        family_size,
        blocking_variance_of_mean_q2,
    )?;
    classify_stationarity_metrics(metrics)
}

fn calculate_stationarity_metrics_from_source(
    observations: &(impl VelocityObservationSource + ?Sized),
    suffix_start: usize,
    suffix_len: usize,
    block_observations: usize,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
) -> Result<StationarityMetrics, StationarityExclusion> {
    if block_observations == 0 {
        return Err(StationarityExclusion::Arithmetic);
    }
    let block_count = suffix_len / block_observations;
    if block_count < VELOCITY_MIN_SELECTED_BLOCKS as usize {
        return Err(StationarityExclusion::TooFewDecorrelatedBlocks);
    }

    let mut cumulative_elapsed_us = 0u64;
    let mut sum_x = 0i128;
    let mut sum_y = 0i128;
    let mut sum_x2 = 0i128;
    let mut sum_xy = 0i128;
    for block_index in 0..block_count {
        let block = rate_block_at(observations, suffix_start, block_index, block_observations)?;
        let midpoint_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us / 2)
            .ok_or(StationarityExclusion::Arithmetic)?;
        cumulative_elapsed_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us)
            .ok_or(StationarityExclusion::Arithmetic)?;
        let x = i128::from(midpoint_us);
        let y = i128::from(
            block
                .rate_q()
                .map_err(|_| StationarityExclusion::Arithmetic)?,
        );
        sum_x = sum_x
            .checked_add(x)
            .ok_or(StationarityExclusion::Arithmetic)?;
        sum_y = sum_y
            .checked_add(y)
            .ok_or(StationarityExclusion::Arithmetic)?;
        sum_x2 = sum_x2
            .checked_add(x.checked_mul(x).ok_or(StationarityExclusion::Arithmetic)?)
            .ok_or(StationarityExclusion::Arithmetic)?;
        sum_xy = sum_xy
            .checked_add(x.checked_mul(y).ok_or(StationarityExclusion::Arithmetic)?)
            .ok_or(StationarityExclusion::Arithmetic)?;
    }

    let n = i128::try_from(block_count).map_err(|_| StationarityExclusion::Arithmetic)?;
    let denominator = n
        .checked_mul(sum_x2)
        .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_x)?))
        .ok_or(StationarityExclusion::Arithmetic)?;
    if denominator <= 0 {
        return Err(StationarityExclusion::Arithmetic);
    }
    let numerator = n
        .checked_mul(sum_xy)
        .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_y)?))
        .ok_or(StationarityExclusion::Arithmetic)?;
    let slope_per_us = numerator as f32 / denominator as f32;
    let intercept = (sum_y as f32 - slope_per_us * sum_x as f32) / block_count as f32;
    if !slope_per_us.is_finite() || !intercept.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }

    let mut residual_sum_q2 = 0.0f32;
    let mut lag_product_sum_q2 = 0.0f32;
    let mut previous_residual = None;
    cumulative_elapsed_us = 0;
    for block_index in 0..block_count {
        let block = rate_block_at(observations, suffix_start, block_index, block_observations)?;
        let midpoint_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us / 2)
            .ok_or(StationarityExclusion::Arithmetic)?;
        cumulative_elapsed_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us)
            .ok_or(StationarityExclusion::Arithmetic)?;
        let rate_q = block
            .rate_q()
            .map_err(|_| StationarityExclusion::Arithmetic)? as f32;
        let residual = rate_q - (intercept + slope_per_us * midpoint_us as f32);
        residual_sum_q2 += residual * residual;
        if let Some(previous) = previous_residual {
            lag_product_sum_q2 += residual * previous;
        }
        previous_residual = Some(residual);
    }
    calculate_stationarity_metrics(
        block_count,
        denominator,
        slope_per_us,
        residual_sum_q2,
        lag_product_sum_q2,
        family_size,
        blocking_variance_of_mean_q2,
    )
}

#[allow(clippy::too_many_arguments)]
fn finish_stationarity_metrics(
    block_count: usize,
    denominator: i128,
    slope_per_us: f32,
    residual_sum_q2: f32,
    lag_product_sum_q2: f32,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
) -> Result<StationarityMetrics, StationarityExclusion> {
    let metrics = calculate_stationarity_metrics(
        block_count,
        denominator,
        slope_per_us,
        residual_sum_q2,
        lag_product_sum_q2,
        family_size,
        blocking_variance_of_mean_q2,
    )?;
    classify_stationarity_metrics(metrics)
}

#[allow(clippy::too_many_arguments)]
fn calculate_stationarity_metrics(
    block_count: usize,
    denominator: i128,
    slope_per_us: f32,
    residual_sum_q2: f32,
    lag_product_sum_q2: f32,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
) -> Result<StationarityMetrics, StationarityExclusion> {
    if block_count < 3 || !residual_sum_q2.is_finite() || !lag_product_sum_q2.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let multiplier = confidence_multiplier_95_df15(family_size)
        .map_err(|_| StationarityExclusion::Arithmetic)?;
    let degrees_of_freedom = (block_count - 2) as f32;
    let residual_variance_q2 = residual_sum_q2 / degrees_of_freedom;
    let blocker_variance_q2 = blocking_variance_of_mean_q2 as f32 * block_count as f32;
    if !blocker_variance_q2.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let slope_variance_q2 = libm::fmaxf(residual_variance_q2, blocker_variance_q2);
    let sxx_us2 = denominator as f32 / block_count as f32;
    let slope_per_second = slope_per_us * MICROS_PER_SECOND;
    let slope_half_width =
        multiplier * libm::sqrtf(slope_variance_q2 / sxx_us2) * MICROS_PER_SECOND;
    if !slope_per_second.is_finite() || !slope_half_width.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let slope_rejection_margin = slope_half_width - libm::fabsf(slope_per_second);

    let lag_one = if residual_sum_q2 == 0.0 {
        0.0
    } else {
        lag_product_sum_q2 / residual_sum_q2
    };
    let lag_null = -1.0 / block_count as f32;
    let lag_half_width = multiplier / libm::sqrtf(block_count as f32);
    if !lag_one.is_finite() || !lag_half_width.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let lag_rejection_margin = lag_half_width - libm::fabsf(lag_one - lag_null);

    Ok(StationarityMetrics {
        slope_q: rounded_i64(slope_per_second)?,
        slope_half_width_q: ceiled_u64(slope_half_width)?,
        lag_one_q: rounded_i32(lag_one * LAG_ONE_Q_SCALE)?,
        lag_one_half_width_q: ceiled_u32(lag_half_width * LAG_ONE_Q_SCALE)?,
        residual_magnitude_q2: libm::ceilf(residual_sum_q2 / block_count as f32) as u128,
        slope_passed: slope_rejection_margin >= 0.0,
        lag_one_passed: lag_rejection_margin >= 0.0,
        slope_rejection_margin_q: rounded_i64(slope_rejection_margin)?,
        lag_one_rejection_margin_q: rounded_i32(lag_rejection_margin * LAG_ONE_Q_SCALE)?,
    })
}

fn classify_stationarity_metrics(
    metrics: StationarityMetrics,
) -> Result<StationarityMetrics, StationarityExclusion> {
    if !metrics.slope_passed {
        return Err(StationarityExclusion::ResolvedSlope);
    }
    if !metrics.lag_one_passed {
        return Err(StationarityExclusion::StructuredResidual);
    }
    Ok(metrics)
}

#[cfg(test)]
fn analyze_stationarity_with_blocking_variance(
    observations: &[RawVelocityObservation],
    block_observations: usize,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
) -> Result<StationarityMetrics, StationarityExclusion> {
    analyze_stationarity_from_source(
        observations,
        0,
        observations.len(),
        block_observations,
        family_size,
        blocking_variance_of_mean_q2,
    )
}

fn rate_block_at(
    observations: &(impl VelocityObservationSource + ?Sized),
    suffix_start: usize,
    block_index: usize,
    block_observations: usize,
) -> Result<RateBlock, StationarityExclusion> {
    let start = suffix_start
        .checked_add(
            block_index
                .checked_mul(block_observations)
                .ok_or(StationarityExclusion::Arithmetic)?,
        )
        .ok_or(StationarityExclusion::Arithmetic)?;
    let end = start
        .checked_add(block_observations)
        .ok_or(StationarityExclusion::Arithmetic)?;
    let mut block: Option<RateBlock> = None;
    for index in start..end {
        let observation = observations
            .observation(index)
            .ok_or(StationarityExclusion::Arithmetic)?;
        let next = RateBlock::from_observation(observation)
            .map_err(|_| StationarityExclusion::Arithmetic)?;
        block = Some(match block {
            Some(current) => current
                .checked_merge(next)
                .map_err(|_| StationarityExclusion::Arithmetic)?,
            None => next,
        });
    }
    block.ok_or(StationarityExclusion::Arithmetic)
}

fn rounded_i64(value: f32) -> Result<i64, StationarityExclusion> {
    if !value.is_finite() || value <= i64::MIN as f32 || value >= i64::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::roundf(value) as i64)
}

fn rounded_i32(value: f32) -> Result<i32, StationarityExclusion> {
    if !value.is_finite() || value <= i32::MIN as f32 || value >= i32::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::roundf(value) as i32)
}

fn ceiled_u64(value: f32) -> Result<u64, StationarityExclusion> {
    if !value.is_finite() || value < 0.0 || value >= u64::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::ceilf(value) as u64)
}

fn ceiled_u32(value: f32) -> Result<u32, StationarityExclusion> {
    if !value.is_finite() || value < 0.0 || value >= u32::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::ceilf(value) as u32)
}

#[cfg(test)]
#[path = "stationarity_tests.rs"]
mod tests;
