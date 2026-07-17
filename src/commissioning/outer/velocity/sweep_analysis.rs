//! Bounded post-capture analysis for the velocity sweep.

#![allow(dead_code)] // Activated by the execution-state commit that follows.

use crate::autotune::velocity_measurement::{
    StationarityResult, VelocityDirection, estimate_confidence,
    select_stationary_suffix_from_source,
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

pub(super) fn analyze_moving_observation(
    data: &mut VelocitySweepData,
    counts_per_rev: u32,
    f_pwm_hz: u32,
) -> Result<MovingObservationAnalysis, CommissionError> {
    let plan = data.plan.ok_or(CommissionError::InvalidSchedule)?;
    let p = *plan
        .gains
        .get(usize::from(data.rung_index))
        .ok_or(CommissionError::InvalidSchedule)?;
    let stationarity = select_stationary_suffix_from_source(&data.observations, &mut data.blocker)
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    let direction = slot_velocity_direction(data.slot);
    let predicted = u32::from(p.raw())
        .checked_mul(plan.target_velocity_register.unsigned_abs())
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
        let half_width = i64::try_from(confidence.half_width_rate_q)
            .map_err(|_| CommissionError::VelocityValidationFailed)?;
        input.settled_rate_low_q = mean_rate_q.saturating_sub(half_width);
        input.settled_rate_high_q = mean_rate_q.saturating_add(half_width);
        let target_rate_q = i64::from(slot_direction(data.slot))
            .checked_mul(i64::from(plan.used_velocity_mrev_s))
            .and_then(|value| value.checked_mul(i64::from(counts_per_rev)))
            .ok_or(CommissionError::InvalidSchedule)?;
        input.deficit_low_q = target_rate_q.saturating_sub(input.settled_rate_high_q);
        input.deficit_high_q = target_rate_q.saturating_sub(input.settled_rate_low_q);
        let signed_target = plan
            .target_velocity_register
            .saturating_mul(slot_direction(data.slot));
        input.disturbance = Some(
            equivalent_disturbance_from_rate(
                p.raw(),
                signed_target,
                suffix.selection.exact_totals,
                confidence.half_width_rate_q,
                counts_per_rev,
                f_pwm_hz,
            )
            .map_err(|_| CommissionError::VelocityValidationFailed)?,
        );
        input.disturbance_variance_q2 = equivalent_disturbance_variance_from_rate(
            p.raw(),
            confidence.variance_of_mean_q2,
            counts_per_rev,
            f_pwm_hz,
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

pub(super) fn settle_is_confirmed(data: &mut VelocitySweepData) -> Result<bool, CommissionError> {
    let stationarity = select_stationary_suffix_from_source(&data.observations, &mut data.blocker)
        .map_err(|_| CommissionError::VelocityValidationFailed)?;
    let StationarityResult::Usable(suffix) = stationarity else {
        return Ok(false);
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
    Ok(mean.saturating_sub(half_width) <= 0 && mean.saturating_add(half_width) >= 0)
}
