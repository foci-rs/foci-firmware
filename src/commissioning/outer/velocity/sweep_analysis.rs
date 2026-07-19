//! Bounded post-capture analysis for the velocity sweep.

use crate::autotune::velocity_measurement::{
    StationarityResult, StationarityWorkPoll, VELOCITY_RATE_Q_SCALE, VelocityDirection,
    estimate_confidence,
};
use crate::autotune::{
    CommissionError, equivalent_disturbance_from_rate, equivalent_disturbance_variance_from_rate,
};
use crate::commissioning::outer::state::{VelocitySweepData, VelocitySweepEvidenceState};

use super::sweep_evidence::{
    VelocityEvidenceError, VelocityObservationInput, VelocityObservationSummary,
    classify_velocity_observation, reproduce_velocity_rung,
};

pub(super) struct MovingObservationAnalysis {
    pub(super) summary: VelocityObservationSummary,
    pub(super) stationarity: StationarityResult,
    pub(super) selected_delta_counts: i64,
    pub(super) selected_elapsed_us: u64,
    pub(super) selected_level: u8,
    pub(super) selected_blocks: u32,
    pub(super) variance_of_mean_q2: u128,
    pub(super) settled_rate_mean_q: i64,
}

fn slot_direction(slot: u8) -> i32 {
    if slot.is_multiple_of(2) { 1 } else { -1 }
}

fn slot_velocity_direction(slot: u8) -> VelocityDirection {
    if slot_direction(slot) > 0 {
        VelocityDirection::Forward
    } else {
        VelocityDirection::Reverse
    }
}

pub(super) fn poll_moving_observation_analysis(
    data: &mut VelocitySweepData,
    counts_per_rev: u32,
) -> Result<Option<MovingObservationAnalysis>, CommissionError> {
    let stationarity = match data
        .analysis
        .poll(&data.observations, &mut data.blocker)
        .map_err(|_| CommissionError::VelocityValidationFailed)?
    {
        StationarityWorkPoll::Pending => return Ok(None),
        StationarityWorkPoll::Complete(stationarity) => stationarity,
    };
    finish_moving_observation_analysis(data, counts_per_rev, stationarity).map(Some)
}

fn finish_moving_observation_analysis(
    data: &mut VelocitySweepData,
    counts_per_rev: u32,
    stationarity: StationarityResult,
) -> Result<MovingObservationAnalysis, CommissionError> {
    let plan = data.plan;
    if !plan.is_present() {
        return Err(CommissionError::InvalidSchedule);
    }
    let p = plan
        .gain_at(data.rung_index)
        .ok_or(CommissionError::InvalidSchedule)?;
    let direction = slot_velocity_direction(data.slot);
    let predicted = u32::from(p.raw())
        .checked_mul(plan.target_velocity_rpm.raw().unsigned_abs())
        .and_then(|value| value.checked_add(255))
        .map(|value| value / 256)
        .ok_or(CommissionError::InvalidSchedule)?
        .max(u32::from(data.peak_torque_target))
        .min(u32::from(u16::MAX)) as u16;

    let mut input = VelocityObservationInput {
        direction,
        p,
        stationarity,
        settled_rate_low_q: 0,
        settled_rate_high_q: 0,
        settled_rate_mean_q: 0,
        deficit_low_q: 0,
        deficit_high_q: 0,
        disturbance: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: predicted,
        usable_torque_limit: plan.usable_torque_limit,
        hard_torque_limit: plan.hard_torque_limit,
        initial_lead_in: data.rung_index == 0 && data.slot == 0,
    };
    let mut settled_rate_mean_q = 0;
    if let StationarityResult::Usable(suffix) = stationarity {
        let family_size = u16::from(plan.rung_count)
            .checked_mul(plan.observations_per_direction)
            .ok_or(CommissionError::InvalidSchedule)?;
        let confidence = estimate_confidence(suffix.selection, family_size)
            .map_err(|_| CommissionError::VelocityValidationFailed)?;
        let mean_rate_q = suffix
            .selection
            .exact_totals
            .rate_q()
            .map_err(|_| CommissionError::VelocityValidationFailed)?;
        settled_rate_mean_q = mean_rate_q;
        input.settled_rate_mean_q = mean_rate_q;
        let half_width = i64::try_from(confidence.half_width_rate_q)
            .map_err(|_| CommissionError::VelocityValidationFailed)?;
        input.settled_rate_low_q = mean_rate_q.saturating_sub(half_width);
        input.settled_rate_high_q = mean_rate_q.saturating_add(half_width);
        let signed_target = plan
            .target_velocity_rpm
            .with_direction(slot_direction(data.slot))
            .map_err(|_| CommissionError::InvalidSchedule)?;
        (input.deficit_low_q, input.deficit_high_q) = deficit_interval_q(
            signed_target.raw(),
            counts_per_rev,
            input.settled_rate_low_q,
            input.settled_rate_high_q,
        )?;
        input.disturbance = Some(
            equivalent_disturbance_from_rate(
                p.raw(),
                signed_target,
                suffix.selection.exact_totals,
                confidence.half_width_rate_q,
                counts_per_rev,
            )
            .map_err(|_| CommissionError::VelocityValidationFailed)?,
        );
        input.disturbance_variance_q2 = equivalent_disturbance_variance_from_rate(
            p.raw(),
            confidence.variance_of_mean_q2,
            counts_per_rev,
        )
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    }
    let summary = classify_velocity_observation(input);
    let (
        selected_delta_counts,
        selected_elapsed_us,
        selected_level,
        selected_blocks,
        variance_of_mean_q2,
    ) = match stationarity {
        StationarityResult::Usable(suffix) => (
            suffix.selection.exact_totals.delta_counts,
            suffix.selection.exact_totals.elapsed_us,
            suffix.selection.level,
            suffix.selection.block_count,
            suffix.selection.variance_of_mean_q2,
        ),
        StationarityResult::Excluded(_) => (0, 0, 0, 0, 0),
    };
    Ok(MovingObservationAnalysis {
        summary,
        stationarity,
        selected_delta_counts,
        selected_elapsed_us,
        selected_level,
        selected_blocks,
        variance_of_mean_q2,
        settled_rate_mean_q,
    })
}

fn deficit_interval_q(
    target_velocity_rpm: i32,
    counts_per_rev: u32,
    settled_rate_low_q: i64,
    settled_rate_high_q: i64,
) -> Result<(i64, i64), CommissionError> {
    if counts_per_rev == 0 {
        return Err(CommissionError::InvalidSchedule);
    }
    let target_numerator = i128::from(target_velocity_rpm)
        .checked_mul(i128::from(counts_per_rev))
        .and_then(|value| value.checked_mul(i128::from(VELOCITY_RATE_Q_SCALE)))
        .ok_or(CommissionError::InvalidSchedule)?;
    let low_numerator = target_numerator
        .checked_sub(i128::from(settled_rate_high_q) * 60)
        .ok_or(CommissionError::InvalidSchedule)?;
    let high_numerator = target_numerator
        .checked_sub(i128::from(settled_rate_low_q) * 60)
        .ok_or(CommissionError::InvalidSchedule)?;
    let low = floor_div_60(low_numerator)?;
    let high = ceil_div_60(high_numerator)?;
    Ok((
        i64::try_from(low).map_err(|_| CommissionError::InvalidSchedule)?,
        i64::try_from(high).map_err(|_| CommissionError::InvalidSchedule)?,
    ))
}

fn floor_div_60(numerator: i128) -> Result<i128, CommissionError> {
    let quotient = numerator / 60;
    if numerator < 0 && numerator % 60 != 0 {
        quotient
            .checked_sub(1)
            .ok_or(CommissionError::InvalidSchedule)
    } else {
        Ok(quotient)
    }
}

fn ceil_div_60(numerator: i128) -> Result<i128, CommissionError> {
    let quotient = numerator / 60;
    if numerator > 0 && numerator % 60 != 0 {
        quotient
            .checked_add(1)
            .ok_or(CommissionError::InvalidSchedule)
    } else {
        Ok(quotient)
    }
}

pub(super) fn retain_observation(
    data: &mut VelocitySweepData,
    summary: VelocityObservationSummary,
) -> Result<(), VelocityEvidenceError> {
    data.evidence = match (
        data.slot,
        core::mem::replace(&mut data.evidence, VelocitySweepEvidenceState::Empty),
    ) {
        (0, VelocitySweepEvidenceState::Empty) => VelocitySweepEvidenceState::ForwardFirst(summary),
        (1, VelocitySweepEvidenceState::ForwardFirst(forward)) => {
            VelocitySweepEvidenceState::FirstPair {
                forward,
                reverse: summary,
            }
        }
        (2, VelocitySweepEvidenceState::FirstPair { forward, reverse }) => {
            VelocitySweepEvidenceState::ForwardComplete {
                forward: reproduce_velocity_rung(forward, summary)?,
                reverse_first: reverse,
            }
        }
        (
            3,
            VelocitySweepEvidenceState::ForwardComplete {
                forward,
                reverse_first,
            },
        ) => VelocitySweepEvidenceState::CompletePair {
            forward,
            reverse: reproduce_velocity_rung(reverse_first, summary)?,
        },
        _ => return Err(VelocityEvidenceError::MismatchedPair),
    };
    Ok(())
}

pub(super) fn poll_settle_confirmation(
    data: &mut VelocitySweepData,
) -> Result<Option<bool>, CommissionError> {
    let stationarity = match data
        .analysis
        .poll(&data.observations, &mut data.blocker)
        .map_err(|_| CommissionError::VelocityValidationFailed)?
    {
        StationarityWorkPoll::Pending => return Ok(None),
        StationarityWorkPoll::Complete(stationarity) => stationarity,
    };
    let StationarityResult::Usable(suffix) = stationarity else {
        return Ok(Some(bounded_rest_displacement(data)));
    };
    let confidence = estimate_confidence(suffix.selection, 3)
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    let mean = suffix
        .selection
        .exact_totals
        .rate_q()
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    let half_width = i64::try_from(confidence.half_width_rate_q)
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    let stationary = mean.saturating_sub(half_width) <= 0 && mean.saturating_add(half_width) >= 0;
    Ok(Some(stationary || bounded_rest_displacement(data)))
}

/// Accept a zero-target settle that never leaves one encoder-count band.
///
/// This covers the observed boundary-toggle dither while rejecting the
/// multi-count rebound that the statistical stationarity gate is intended to
/// catch. The bounds include the settle origin at zero.
fn bounded_rest_displacement(data: &VelocitySweepData) -> bool {
    data.settle_max_displacement_counts - data.settle_min_displacement_counts <= 1
}

#[cfg(test)]
mod tests {
    use super::{bounded_rest_displacement, deficit_interval_q};
    use crate::commissioning::outer::VelocitySweepData;

    #[test]
    fn bounded_boundary_dither_is_accepted() {
        let mut data = VelocitySweepData::new(0);
        data.settle_min_displacement_counts = 0;
        data.settle_max_displacement_counts = 1;
        data.settle_displacement_counts = -1;

        assert!(bounded_rest_displacement(&data));
    }

    #[test]
    fn multi_count_creep_is_not_accepted_as_bounded_rest() {
        let mut data = VelocitySweepData::new(0);
        data.settle_min_displacement_counts = -1;
        data.settle_max_displacement_counts = 2;
        data.settle_displacement_counts = 2;

        assert!(!bounded_rest_displacement(&data));
    }

    #[test]
    fn deficit_interval_uses_exact_mechanical_rpm_target() {
        assert_eq!(
            deficit_interval_q(176, 4_000, 11_733_333, 11_733_334),
            Ok((-1, 1))
        );
        assert_ne!(
            deficit_interval_q(8, 4_000, 11_733_333, 11_733_334),
            Ok((-1, 1))
        );
    }
}
