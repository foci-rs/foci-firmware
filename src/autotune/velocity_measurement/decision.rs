use super::blocker::PlateauSelection;
use super::observation::{
    RateBlock, RawVelocityObservation, VELOCITY_RATE_Q_SCALE, VelocityMeasurementError,
};

const CONFIDENCE_MULTIPLIER_95: f32 = 2.131;
const MICROS_PER_SECOND: i128 = 1_000_000;
const MILLISECONDS_PER_SECOND: i128 = 1_000;
const MILLIREVOLUTIONS_PER_REVOLUTION: i128 = 1_000;

/// Confidence-gated physical-accuracy classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AccuracyDecision {
    /// The complete confidence interval is inside the physical error limit.
    Pass,
    /// The complete confidence interval is outside the physical error limit.
    ConfidentFail,
    /// The confidence interval still straddles the physical error limit.
    CollectMore,
}

/// Foundation-level velocity measurement outcome before terminal-status mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityMeasurementOutcome {
    /// The complete confidence interval is inside the physical error limit.
    Pass,
    /// The complete confidence interval is outside the physical error limit.
    ConfidentFail,
    /// Available evidence cannot yet support either conclusion.
    Inconclusive,
}

/// Confidence interval derived from one selected dyadic blocking level.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ConfidenceEstimate {
    /// Selected zero-based blocking level.
    pub selected_level: u8,
    /// Completed block population used by the estimate.
    pub selected_blocks: u32,
    /// Outward-rounded 95% half-width in fixed-point rate units.
    pub half_width_rate_q: u64,
    /// Variance of the mean in squared fixed-point rate units.
    pub variance_of_mean_q2: u128,
}

/// Sliding two-observation rate window with exact time weighting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TwoIntervalRateWindow {
    previous: Option<RawVelocityObservation>,
}

impl TwoIntervalRateWindow {
    /// Construct an empty window that requires one priming observation.
    pub const fn new() -> Self {
        Self { previous: None }
    }

    /// Add an observation and return the exact two-interval block once primed.
    pub fn observe(
        &mut self,
        observation: RawVelocityObservation,
    ) -> Result<Option<RateBlock>, VelocityMeasurementError> {
        let current = RateBlock::from_observation(observation)?;
        let previous = self.previous.replace(observation);
        previous
            .map(|previous| RateBlock::from_observation(previous)?.checked_merge(current))
            .transpose()
    }

    /// Clear the priming observation at a motion or measurement boundary.
    pub fn reset(&mut self) {
        self.previous = None;
    }
}

impl Default for TwoIntervalRateWindow {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert a selected variance-of-the-mean plateau into a 95% confidence interval.
pub fn estimate_confidence(
    selection: PlateauSelection,
) -> Result<ConfidenceEstimate, VelocityMeasurementError> {
    let variance = selection.variance_of_mean_q2 as f32;
    if !variance.is_finite() {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }
    let half_width = CONFIDENCE_MULTIPLIER_95 * libm::sqrtf(variance);
    if !half_width.is_finite() || half_width >= u64::MAX as f32 {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }

    Ok(ConfidenceEstimate {
        selected_level: selection.level,
        selected_blocks: selection.block_count,
        half_width_rate_q: libm::ceilf(half_width) as u64,
        variance_of_mean_q2: selection.variance_of_mean_q2,
    })
}

/// Classify an exact settled rate against an encoder-normalized physical error limit.
pub fn classify_absolute_rate(
    settled_delta_counts: i64,
    settled_elapsed_us: u64,
    target_counts_per_ms: i32,
    half_width_rate_q: u64,
    counts_per_rev: u32,
    limit_mrev_per_s: u32,
) -> Result<AccuracyDecision, VelocityMeasurementError> {
    if settled_elapsed_us == 0 {
        return Err(VelocityMeasurementError::ZeroElapsed);
    }

    let elapsed = i128::from(settled_elapsed_us);
    let settled_rate_numerator = i128::from(settled_delta_counts)
        .checked_mul(MICROS_PER_SECOND)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let target_rate_numerator = i128::from(target_counts_per_ms)
        .checked_mul(MILLISECONDS_PER_SECOND)
        .and_then(|value| value.checked_mul(elapsed))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let error_numerator = settled_rate_numerator
        .checked_sub(target_rate_numerator)
        .and_then(i128::checked_abs)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let scaled_error = error_numerator
        .checked_mul(i128::from(VELOCITY_RATE_Q_SCALE))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let scaled_half_width = i128::from(half_width_rate_q)
        .checked_mul(elapsed)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let upper_left = scaled_error
        .checked_add(scaled_half_width)
        .and_then(|value| value.checked_mul(MILLIREVOLUTIONS_PER_REVOLUTION))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let right = i128::from(limit_mrev_per_s)
        .checked_mul(i128::from(counts_per_rev))
        .and_then(|value| value.checked_mul(i128::from(VELOCITY_RATE_Q_SCALE)))
        .and_then(|value| value.checked_mul(elapsed))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;

    if upper_left <= right {
        return Ok(AccuracyDecision::Pass);
    }

    let lower_error = if scaled_error > scaled_half_width {
        scaled_error - scaled_half_width
    } else {
        0
    };
    let lower_left = lower_error
        .checked_mul(MILLIREVOLUTIONS_PER_REVOLUTION)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    if lower_left > right {
        Ok(AccuracyDecision::ConfidentFail)
    } else {
        Ok(AccuracyDecision::CollectMore)
    }
}

/// Classify one selected plateau, preserving missing or straddling evidence as inconclusive.
pub fn classify_plateau(
    selection: Option<PlateauSelection>,
    target_counts_per_ms: i32,
    counts_per_rev: u32,
    limit_mrev_per_s: u32,
) -> Result<VelocityMeasurementOutcome, VelocityMeasurementError> {
    let Some(selection) = selection else {
        return Ok(VelocityMeasurementOutcome::Inconclusive);
    };
    let confidence = estimate_confidence(selection)?;
    let accuracy = classify_absolute_rate(
        selection.exact_totals.delta_counts,
        selection.exact_totals.elapsed_us,
        target_counts_per_ms,
        confidence.half_width_rate_q,
        counts_per_rev,
        limit_mrev_per_s,
    )?;
    Ok(match accuracy {
        AccuracyDecision::Pass => VelocityMeasurementOutcome::Pass,
        AccuracyDecision::ConfidentFail => VelocityMeasurementOutcome::ConfidentFail,
        AccuracyDecision::CollectMore => VelocityMeasurementOutcome::Inconclusive,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        AccuracyDecision, PlateauSelection, RawVelocityObservation, TwoIntervalRateWindow,
        VelocityMeasurementError, VelocityMeasurementOutcome, classify_absolute_rate,
        classify_plateau, estimate_confidence,
    };
    use crate::autotune::velocity_measurement::RateBlock;

    #[test]
    fn confidence_half_width_rounds_outward() {
        let estimate = estimate_confidence(PlateauSelection {
            level: 3,
            block_count: 16,
            variance_of_mean_q2: 1,
            exact_totals: RateBlock {
                delta_counts: 16,
                elapsed_us: 16_000,
                observation_count: 16,
            },
        })
        .unwrap();

        assert_eq!(estimate.selected_level, 3);
        assert_eq!(estimate.selected_blocks, 16);
        assert_eq!(estimate.variance_of_mean_q2, 1);
        assert_eq!(estimate.half_width_rate_q, 3);
    }

    #[test]
    fn missing_plateau_is_a_first_class_inconclusive_outcome() {
        assert_eq!(
            classify_plateau(None, 0, 1_000, 1_000),
            Ok(VelocityMeasurementOutcome::Inconclusive)
        );
    }

    #[test]
    fn plateau_classification_uses_exact_time_weighted_totals() {
        let selection = PlateauSelection {
            level: 2,
            block_count: 16,
            variance_of_mean_q2: 0,
            exact_totals: RateBlock {
                delta_counts: 2,
                elapsed_us: 3_000,
                observation_count: 16,
            },
        };

        assert_eq!(
            classify_plateau(Some(selection), 0, 1_000, 667),
            Ok(VelocityMeasurementOutcome::Pass)
        );
        assert_eq!(
            classify_plateau(Some(selection), 0, 1_000, 666),
            Ok(VelocityMeasurementOutcome::ConfidentFail)
        );
    }

    #[test]
    fn exact_upper_boundary_passes() {
        assert_eq!(
            classify_absolute_rate(1, 1_000, 0, 0, 1_000, 1_000),
            Ok(AccuracyDecision::Pass)
        );
    }

    #[test]
    fn confidence_interval_straddling_boundary_collects_more() {
        assert_eq!(
            classify_absolute_rate(1, 1_000, 0, 1, 1_000, 1_000),
            Ok(AccuracyDecision::CollectMore)
        );
    }

    #[test]
    fn lower_confidence_edge_outside_boundary_fails() {
        assert_eq!(
            classify_absolute_rate(2, 1_000, 0, 0, 1_000, 1_000),
            Ok(AccuracyDecision::ConfidentFail)
        );
    }

    #[test]
    fn reverse_target_uses_signed_mean_error() {
        assert_eq!(
            classify_absolute_rate(-10, 1_000, -10, 0, 4_000, 1),
            Ok(AccuracyDecision::Pass)
        );
    }

    #[test]
    fn physical_limit_is_invariant_across_encoder_resolutions() {
        let low_resolution = classify_absolute_rate(1, 1_000, 0, 0, 1_000, 1_000);
        let high_resolution = classify_absolute_rate(4, 1_000, 0, 0, 4_000, 1_000);

        assert_eq!(low_resolution, Ok(AccuracyDecision::Pass));
        assert_eq!(high_resolution, low_resolution);
    }

    #[test]
    fn cross_product_overflow_is_rejected() {
        assert_eq!(
            classify_absolute_rate(0, u64::MAX, 0, u64::MAX, 1, 1),
            Err(VelocityMeasurementError::ArithmeticOverflow)
        );
    }

    #[test]
    fn two_interval_window_uses_total_displacement_over_jittered_time() {
        let mut window = TwoIntervalRateWindow::new();
        let first = RawVelocityObservation::new(1, 900).unwrap();
        let second = RawVelocityObservation::new(1, 1_100).unwrap();

        assert_eq!(window.observe(first).unwrap(), None);
        let combined = window.observe(second).unwrap().unwrap();

        assert_eq!(combined.delta_counts, 2);
        assert_eq!(combined.elapsed_us, 2_000);
        assert_eq!(combined.rate_q().unwrap(), 1_000_000);
    }

    #[test]
    fn isolated_one_interval_spike_is_reduced_by_robust_window() {
        let mut window = TwoIntervalRateWindow::new();
        let spike = RawVelocityObservation::new(2, 1_000).unwrap();
        let ordinary = RawVelocityObservation::new(0, 1_000).unwrap();
        let raw_spike_rate = super::RateBlock::from_observation(spike)
            .unwrap()
            .rate_q()
            .unwrap();

        assert_eq!(window.observe(spike).unwrap(), None);
        let robust_rate = window.observe(ordinary).unwrap().unwrap().rate_q().unwrap();

        assert_eq!(raw_spike_rate, 2_000_000);
        assert_eq!(robust_rate, 1_000_000);
    }

    #[test]
    fn reset_requires_two_fresh_intervals() {
        let mut window = TwoIntervalRateWindow::new();
        assert_eq!(
            window
                .observe(RawVelocityObservation::new(1, 1_000).unwrap())
                .unwrap(),
            None
        );
        window.reset();

        assert_eq!(
            window
                .observe(RawVelocityObservation::new(1, 1_000).unwrap())
                .unwrap(),
            None
        );
    }
}
