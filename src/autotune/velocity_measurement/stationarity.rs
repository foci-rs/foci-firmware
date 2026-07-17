use super::{
    DyadicBlocker, PlateauSelection, RateBlock, RawVelocityObservation,
    VELOCITY_MIN_SELECTED_BLOCKS, VelocityMeasurementError, confidence_multiplier_95_df15,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StationarityMetrics {
    slope_q: i64,
    slope_half_width_q: u64,
    lag_one_q: i32,
    lag_one_half_width_q: u32,
    residual_magnitude_q2: u128,
}

/// Select the longest contiguous stationary suffix using one reusable blocker.
pub fn select_stationary_suffix<const LEVELS: usize>(
    observations: &[RawVelocityObservation],
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<StationarityResult, VelocityMeasurementError> {
    let mut selected: Option<StationarySuffix> = None;
    let mut tested_suffixes = 0u8;

    for suffix_len in VELOCITY_SUFFIX_LENGTHS {
        let suffix_len = usize::from(suffix_len);
        if suffix_len > observations.len() {
            continue;
        }
        tested_suffixes = tested_suffixes
            .checked_add(1)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        *blocker = DyadicBlocker::new();
        let suffix = &observations[observations.len() - suffix_len..];
        for observation in suffix {
            blocker.observe(*observation)?;
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
        let metrics = match analyze_stationarity_with_blocking_variance(
            suffix,
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

fn analyze_stationarity_with_blocking_variance(
    observations: &[RawVelocityObservation],
    block_observations: usize,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
) -> Result<StationarityMetrics, StationarityExclusion> {
    if block_observations == 0 {
        return Err(StationarityExclusion::Arithmetic);
    }
    let block_count = observations.len() / block_observations;
    if block_count < VELOCITY_MIN_SELECTED_BLOCKS as usize {
        return Err(StationarityExclusion::TooFewDecorrelatedBlocks);
    }

    let mut cumulative_elapsed_us = 0u64;
    let mut sum_x = 0i128;
    let mut sum_y = 0i128;
    let mut sum_x2 = 0i128;
    let mut sum_xy = 0i128;
    for block_index in 0..block_count {
        let block = rate_block_at(observations, block_index, block_observations)?;
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
        let block = rate_block_at(observations, block_index, block_observations)?;
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
    if !residual_sum_q2.is_finite() || !lag_product_sum_q2.is_finite() {
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
    if libm::fabsf(slope_per_second) > slope_half_width {
        return Err(StationarityExclusion::ResolvedSlope);
    }

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
    if libm::fabsf(lag_one - lag_null) > lag_half_width {
        return Err(StationarityExclusion::StructuredResidual);
    }

    Ok(StationarityMetrics {
        slope_q: rounded_i64(slope_per_second)?,
        slope_half_width_q: ceiled_u64(slope_half_width)?,
        lag_one_q: rounded_i32(lag_one * LAG_ONE_Q_SCALE)?,
        lag_one_half_width_q: ceiled_u32(lag_half_width * LAG_ONE_Q_SCALE)?,
        residual_magnitude_q2: libm::ceilf(residual_sum_q2 / block_count as f32) as u128,
    })
}

fn rate_block_at(
    observations: &[RawVelocityObservation],
    block_index: usize,
    block_observations: usize,
) -> Result<RateBlock, StationarityExclusion> {
    let start = block_index
        .checked_mul(block_observations)
        .ok_or(StationarityExclusion::Arithmetic)?;
    let end = start
        .checked_add(block_observations)
        .ok_or(StationarityExclusion::Arithmetic)?;
    let mut block: Option<RateBlock> = None;
    for observation in observations
        .get(start..end)
        .ok_or(StationarityExclusion::Arithmetic)?
    {
        let next = RateBlock::from_observation(*observation)
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
