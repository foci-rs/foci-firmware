//! Bounded post-capture analysis for the velocity sweep.

use crate::autotune::velocity_measurement::{
    DyadicBlocker, StationarityResult, StationarityWork, StationarityWorkPoll,
    StructuredStationarityEvidence, VELOCITY_RATE_Q_SCALE, VelocityDirection,
    VelocityObservationSource, estimate_confidence,
};
use crate::autotune::{
    CommissionError, equivalent_disturbance_from_rate, equivalent_disturbance_variance_from_rate,
};
use crate::commissioning::outer::state::{VelocitySweepData, VelocitySweepEvidenceState};

use super::sweep_evidence::{
    VelocityEvidenceError, VelocityObservationInput, VelocityObservationSummary,
    classify_velocity_observation,
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
    pub(super) structured_boundary: Option<StructuredStationarityEvidence>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct SettleConfirmation {
    pub(super) stationarity: StationarityResult,
    pub(super) confirmed_rest: bool,
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
    let Some(stationarity) =
        poll_stationarity(&mut data.analysis, &data.observations, &mut data.blocker)?
    else {
        return Ok(None);
    };
    let structured_boundary = data.analysis.structured_evidence();
    finish_moving_observation_analysis(data, counts_per_rev, stationarity, structured_boundary)
        .map(Some)
}

fn finish_moving_observation_analysis(
    data: &mut VelocitySweepData,
    counts_per_rev: u32,
    stationarity: StationarityResult,
    structured_boundary: Option<StructuredStationarityEvidence>,
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
        .max(u32::from(data.current_observer.moving_pid_output_peak()))
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
        structured_boundary,
    })
}

pub(super) fn deficit_interval_q(
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

pub(super) fn floor_div_60(numerator: i128) -> Result<i128, CommissionError> {
    let quotient = numerator / 60;
    if numerator < 0 && numerator % 60 != 0 {
        quotient
            .checked_sub(1)
            .ok_or(CommissionError::InvalidSchedule)
    } else {
        Ok(quotient)
    }
}

pub(super) fn ceil_div_60(numerator: i128) -> Result<i128, CommissionError> {
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
    structured: Option<StructuredStationarityEvidence>,
) -> Result<(), VelocityEvidenceError> {
    let VelocitySweepEvidenceState::Collecting(collected) = &mut data.evidence else {
        return Err(VelocityEvidenceError::MismatchedObservation);
    };
    collected.retain_with_structured(data.slot, summary, structured)
}

pub(super) fn poll_settle_confirmation(
    data: &mut VelocitySweepData,
) -> Result<Option<bool>, CommissionError> {
    poll_settle_confirmation_parts(
        &mut data.analysis,
        &data.observations,
        &mut data.blocker,
        data.settle_min_displacement_counts,
        data.settle_max_displacement_counts,
    )
    .map(|result| result.map(|result| result.confirmed_rest))
}

pub(super) fn poll_stationarity<const LEVELS: usize>(
    analysis: &mut StationarityWork,
    observations: &impl VelocityObservationSource,
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<Option<StationarityResult>, CommissionError> {
    match analysis
        .poll(observations, blocker)
        .map_err(|_| CommissionError::VelocityValidationFailed)?
    {
        StationarityWorkPoll::Pending => Ok(None),
        StationarityWorkPoll::Complete(stationarity) => Ok(Some(stationarity)),
    }
}

pub(super) fn poll_settle_confirmation_parts<const LEVELS: usize>(
    analysis: &mut StationarityWork,
    observations: &impl VelocityObservationSource,
    blocker: &mut DyadicBlocker<LEVELS>,
    settle_min_displacement_counts: i8,
    settle_max_displacement_counts: i8,
) -> Result<Option<SettleConfirmation>, CommissionError> {
    let Some(stationarity) = poll_stationarity(analysis, observations, blocker)? else {
        return Ok(None);
    };
    let confirmed_rest = match stationarity {
        StationarityResult::Usable(suffix) => {
            let confidence = estimate_confidence(suffix.selection, 3)
                .map_err(|_| CommissionError::VelocityValidationFailed)?;
            let mean = suffix
                .selection
                .exact_totals
                .rate_q()
                .map_err(|_| CommissionError::VelocityValidationFailed)?;
            let half_width = i64::try_from(confidence.half_width_rate_q)
                .map_err(|_| CommissionError::VelocityValidationFailed)?;
            let stationary =
                mean.saturating_sub(half_width) <= 0 && mean.saturating_add(half_width) >= 0;
            stationary
                || bounded_rest_displacement(
                    settle_min_displacement_counts,
                    settle_max_displacement_counts,
                )
        }
        StationarityResult::Excluded(_) => bounded_rest_displacement(
            settle_min_displacement_counts,
            settle_max_displacement_counts,
        ),
    };
    Ok(Some(SettleConfirmation {
        stationarity,
        confirmed_rest,
    }))
}

/// Accept a zero-target settle that never leaves one encoder-count band.
///
/// This covers the observed boundary-toggle dither while rejecting the
/// multi-count rebound that the statistical stationarity gate is intended to
/// catch. The bounds include the settle origin at zero.
fn bounded_rest_displacement(minimum: i8, maximum: i8) -> bool {
    maximum - minimum <= 1
}

#[cfg(test)]
mod tests {
    use crate::autotune::velocity_measurement::{
        DyadicBlocker, RawVelocityObservation, StationarityWork, select_stationary_suffix,
    };
    use crate::commissioning::outer::state::VelocityObservationRing;

    use super::{bounded_rest_displacement, deficit_interval_q, poll_settle_confirmation_parts};

    #[test]
    fn settle_analysis_retains_the_exact_stationarity_result() {
        let observations: [RawVelocityObservation; 256] = core::array::from_fn(|index| {
            RawVelocityObservation::new(if index.is_multiple_of(5) { 0 } else { 1 }, 1_000).unwrap()
        });
        let mut expected_blocker = DyadicBlocker::<5>::new();
        let expected = select_stationary_suffix(&observations, &mut expected_blocker).unwrap();
        let mut ring = VelocityObservationRing::new();
        for observation in observations {
            ring.push(observation.delta_counts, observation.elapsed_us as u16)
                .unwrap();
        }
        let mut analysis = StationarityWork::new();
        let mut blocker = DyadicBlocker::<5>::new();

        let result = loop {
            match poll_settle_confirmation_parts(&mut analysis, &ring, &mut blocker, 0, 1).unwrap()
            {
                Some(result) => break result,
                None => {}
            }
        };

        assert_eq!(result.stationarity, expected);
        assert!(result.confirmed_rest);
    }

    #[test]
    fn bounded_boundary_dither_is_accepted() {
        assert!(bounded_rest_displacement(0, 1));
    }

    #[test]
    fn multi_count_creep_is_not_accepted_as_bounded_rest() {
        assert!(!bounded_rest_displacement(-1, 2));
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
