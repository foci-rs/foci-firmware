//! Pure planning and response-selection rules for combined velocity tuning.

use crate::autotune::velocity_measurement::VELOCITY_RATE_Q_SCALE;

use super::integral_plan::{
    INTEGRAL_OBSERVATIONS_PER_RUNG, INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US, VelocityIntegralLadder,
    VelocityIntegralPlanError, build_integral_ladder, confidence_family_size,
};
use super::sweep_plan::{SETTLE_ANALYSIS_BUDGET_US, VelocitySweepPlan, ZERO_SETTLE_US};

/// Live Proportional schema for the shared Proportional velocity sweep planner.
pub const PROPORTIONAL_PLAN_SCHEMA_REVISION: u16 = 14;
/// First Proportional schema with cap-safe target quantization.
pub const FIRST_CAP_SAFE_PROPORTIONAL_SCHEMA_REVISION: u16 = 10;
/// Live Integral schema for the shared Integral continuation planner.
pub const INTEGRAL_PLAN_SCHEMA_REVISION: u16 = 13;
/// Live Proportional schema for the breakaway-seeded campaign.
///
/// The breakaway probe, additive discovery, and held-out confirmation plans
/// replace the combined proportional sweep entirely; there is no Proportional
/// sweep plan under this schema. Historical
/// schemas 8 through [`PROPORTIONAL_PLAN_SCHEMA_REVISION`] keep their exact
/// digest and behavior; this schema is additive.
pub const BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION: u16 = 17;
/// Live Integral schema for the breakaway-seeded campaign.
///
/// An Integral plan built under this schema binds the accepted
/// [`super::band_confirmation::BandConfirmationPlan`] digest instead of a
/// Proportional sweep digest -- see
/// `super::integral_plan::bind_breakaway_integral_authority`. Historical
/// schemas up to and including [`INTEGRAL_PLAN_SCHEMA_REVISION`] keep
/// their exact digest and behavior; this schema is additive.
///
/// 16 -> 18 (17 belongs to the half-octave standalone range): rung-origin
/// recovery was unified onto the stabilized rest schedule, so every recovery
/// that completes a scored rest now emits a rest record where schema-16
/// captures emitted one only for fixed-gain-amplitude runs. That is an evidence
/// grammar change, not a measurement rule change, so offline decoding has to
/// be able to tell the two apart rather than widen the existing range.
pub const BREAKAWAY_INTEGRAL_SCHEMA_REVISION: u16 = 18;

const I_RESPONSE_TARGET_PERCENT: i128 = 98;
const PERCENT_SCALE: i128 = 100;
const SECONDS_PER_MINUTE: i128 = 60;
const MICROS_PER_MILLISECOND: u64 = 1_000;

/// Reportable outcome of the positive-I acquisition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum CombinedTargetStatus {
    /// The 98-percent target was reached with a sufficient response curve.
    TargetReached = 0,
    /// The target was reached before three positive rungs were eligible.
    TargetReachedSparse = 1,
    /// The current-headroom boundary ended the positive-I ladder.
    CurrentHeadroom = 2,
    /// The exact cap rung remained below the target.
    TargetNotReachedAtCap = 3,
}

/// Exact acquisition counts and duration envelope for one combined command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CombinedWorkflowGeometry {
    /// Number of positive-I rungs, including the exact cap.
    pub positive_i_rungs: u8,
    /// Opening anchor, positive-I rungs, and closing bookend.
    pub total_i_rungs: u8,
    /// Forward and reverse moving observations in the I phase.
    pub i_observations: u16,
    /// Per-direction simultaneous confidence family.
    pub i_family_size: u16,
    /// Recovery slots reserved by the I phase.
    pub i_recovery_slots: u8,
    /// Exact I-phase nominal duration rounded outward to milliseconds.
    pub i_nominal_ms: u32,
    /// Exact I-phase maximum duration rounded outward to milliseconds.
    pub i_maximum_ms: u32,
    /// Exact combined nominal duration rounded only after summing phases.
    pub combined_nominal_ms: u32,
    /// Exact combined maximum duration rounded only after summing phases.
    pub combined_maximum_ms: u32,
}

/// Exact inclusive settled-rate-Q interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CombinedRateInterval {
    /// Inclusive lower endpoint.
    pub low_q: i64,
    /// Inclusive upper endpoint.
    pub high_q: i64,
}

impl CombinedRateInterval {
    /// Construct one ordered closed interval.
    pub const fn new(low_q: i64, high_q: i64) -> Result<Self, CombinedPlanError> {
        if low_q > high_q {
            Err(CombinedPlanError::InvalidInterval)
        } else {
            Ok(Self { low_q, high_q })
        }
    }
}

/// Invalid combined-plan geometry or response evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CombinedPlanError {
    /// A closed interval was malformed or carried the wrong directional sign.
    InvalidInterval,
    /// The target geometry was zero or signed incorrectly.
    InvalidTarget,
    /// Checked response arithmetic exceeded its representation.
    Arithmetic,
}

/// Build the combined ladder, appending the exact cap after the octave schedule.
pub fn build_combined_integral_ladder(
    pwm_hz: u32,
    moving_stroke_us: u32,
    shortest_suffix_us: u32,
    maximum_i: u16,
) -> Result<VelocityIntegralLadder, VelocityIntegralPlanError> {
    let mut ladder =
        build_integral_ladder(pwm_hz, moving_stroke_us, shortest_suffix_us, maximum_i)?;
    if ladder.rungs.last().copied() != Some(ladder.i_top_cap) {
        ladder
            .rungs
            .push(ladder.i_top_cap)
            .map_err(|_| VelocityIntegralPlanError::ConfidenceFamilyTooLarge)?;
    }
    confidence_family_size(ladder.rungs.len())?;
    Ok(ladder)
}

/// Derive the exact Integral continuation duration without summing rounded
/// phase reports.
pub fn integral_continuation_geometry(
    proportional: VelocitySweepPlan,
    pwm_hz: u32,
) -> Result<CombinedWorkflowGeometry, VelocityIntegralPlanError> {
    if proportional.rung_count == 0
        || proportional.observations_per_direction == 0
        || proportional.recovery_slot_count != proportional.rung_count
    {
        return Err(VelocityIntegralPlanError::InvalidRequest);
    }
    let ladder = build_combined_integral_ladder(
        pwm_hz,
        proportional.moving_stroke_us,
        super::sweep_plan::MIN_SETTLE_CAPTURE_US,
        crate::commissioning::outer::OUTER_MAX_VELOCITY_I,
    )?;
    let positive_i_rungs =
        u8::try_from(ladder.rungs.len()).map_err(|_| VelocityIntegralPlanError::Arithmetic)?;
    let total_i_rungs = positive_i_rungs
        .checked_add(2)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let i_observations = u16::from(total_i_rungs)
        .checked_mul(INTEGRAL_OBSERVATIONS_PER_RUNG as u16)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let i_family_size = confidence_family_size(ladder.rungs.len())?;
    let primitive_us = u64::from(proportional.moving_stroke_us)
        .checked_add(u64::from(proportional.zero_settle_us))
        .and_then(|value| value.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let p_observations = u64::from(proportional.rung_count)
        .checked_mul(u64::from(proportional.observations_per_direction))
        .and_then(|value| value.checked_mul(2))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let p_base_us = p_observations
        .checked_mul(primitive_us)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let p_nominal_us = p_base_us
        .checked_add(
            u64::from(proportional.recovery_slot_count)
                .checked_mul(u64::from(proportional.recovery_nominal_slot_us))
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let p_maximum_us = p_base_us
        .checked_add(
            u64::from(proportional.recovery_slot_count)
                .checked_mul(u64::from(proportional.recovery_maximum_slot_us))
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let i_base_us = u64::from(i_observations)
        .checked_mul(primitive_us)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let slot_transition_us = u64::from(i_observations)
        .checked_mul(u64::from(INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let cleanup_us = u64::from(ZERO_SETTLE_US)
        .checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let i_nominal_us = i_base_us
        .checked_add(
            u64::from(total_i_rungs)
                .checked_mul(u64::from(proportional.recovery_nominal_slot_us))
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .and_then(|value| value.checked_add(slot_transition_us))
        .and_then(|value| value.checked_add(cleanup_us))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let i_maximum_us = i_base_us
        .checked_add(
            u64::from(total_i_rungs)
                .checked_mul(u64::from(proportional.recovery_maximum_slot_us))
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .and_then(|value| value.checked_add(slot_transition_us))
        .and_then(|value| value.checked_add(cleanup_us))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;

    Ok(CombinedWorkflowGeometry {
        positive_i_rungs,
        total_i_rungs,
        i_observations,
        i_family_size,
        i_recovery_slots: total_i_rungs,
        i_nominal_ms: micros_to_millis(i_nominal_us)?,
        i_maximum_ms: micros_to_millis(i_maximum_us)?,
        combined_nominal_ms: micros_to_millis(
            p_nominal_us
                .checked_add(i_nominal_us)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )?,
        combined_maximum_ms: micros_to_millis(
            p_maximum_us
                .checked_add(i_maximum_us)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )?,
    })
}

fn micros_to_millis(value: u64) -> Result<u32, VelocityIntegralPlanError> {
    u32::try_from(value.div_ceil(MICROS_PER_MILLISECOND))
        .map_err(|_| VelocityIntegralPlanError::Arithmetic)
}

/// Test whether signed deficit intervals prove at least the integral target response.
pub fn integral_deficit_reaches_target(
    target_rpm: i32,
    encoder_counts_per_rev: u32,
    deficits: [CombinedRateInterval; 2],
) -> Result<bool, CombinedPlanError> {
    if target_rpm <= 0
        || encoder_counts_per_rev == 0
        || deficits
            .iter()
            .any(|interval| interval.low_q > interval.high_q)
    {
        return Err(CombinedPlanError::InvalidTarget);
    }
    let target_numerator = i128::from(target_rpm)
        .checked_mul(i128::from(encoder_counts_per_rev))
        .and_then(|value| value.checked_mul(i128::from(VELOCITY_RATE_Q_SCALE)))
        .ok_or(CombinedPlanError::Arithmetic)?;
    let target_floor = target_numerator
        .checked_mul(I_RESPONSE_TARGET_PERCENT)
        .ok_or(CombinedPlanError::Arithmetic)?;
    let forward_response = target_numerator
        .checked_sub(i128::from(deficits[0].high_q) * SECONDS_PER_MINUTE)
        .and_then(|value| value.checked_mul(PERCENT_SCALE))
        .ok_or(CombinedPlanError::Arithmetic)?;
    let reverse_response = target_numerator
        .checked_add(i128::from(deficits[1].low_q) * SECONDS_PER_MINUTE)
        .and_then(|value| value.checked_mul(PERCENT_SCALE))
        .ok_or(CombinedPlanError::Arithmetic)?;
    Ok(forward_response >= target_floor && reverse_response >= target_floor)
}
