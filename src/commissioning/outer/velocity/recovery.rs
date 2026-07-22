//! Shared rung-origin recovery geometry and checked deadline arithmetic.

use crate::autotune::velocity_measurement::{VELOCITY_RATE_Q_SCALE, VelocityDirection};
use crate::commissioning::outer::OuterMotionBudget;

use super::sweep_plan::{MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US, ZERO_SETTLE_US};

const MREV_PER_REV: u128 = 1_000;
const MICROS_PER_SECOND: u128 = 1_000_000;

/// Immutable recovery geometry derived before any commissioning motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RungOriginRecoveryPlan {
    /// Inclusive distance from the immutable campaign origin accepted as centered.
    pub(crate) origin_band_counts: u32,
    /// Conservative fixed slot used by the nominal workflow declaration.
    pub(crate) nominal_slot_us: u32,
    /// Host-declared per-primitive cap reserved by the maximum workflow.
    pub(crate) maximum_slot_us: u32,
}

/// Failure to derive representable, bounded recovery geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RungOriginRecoveryPlanError {
    /// A required scale or motion bound was zero or internally inconsistent.
    InvalidRequest,
    /// Checked arithmetic or final narrowing exceeded its representation.
    Arithmetic,
    /// The derived recovery duration exceeds the declared primitive cap.
    ExceedsPrimitiveCap,
}

/// Derive the fixed origin band and nominal/maximum recovery slots.
pub(crate) fn plan_rung_origin_recovery(
    budget: OuterMotionBudget,
    counts_per_rev: u32,
    planned_velocity_mrev_s: u32,
) -> Result<RungOriginRecoveryPlan, RungOriginRecoveryPlanError> {
    if counts_per_rev == 0
        || planned_velocity_mrev_s == 0
        || budget.max_stroke_travel_mrev == 0
        || budget.settle_travel_reserve_mrev == 0
        || budget.settle_travel_reserve_mrev >= budget.max_stroke_travel_mrev
        || budget.max_duration_ms == 0
    {
        return Err(RungOriginRecoveryPlanError::InvalidRequest);
    }

    let origin_band_counts = u128::from(budget.settle_travel_reserve_mrev)
        .checked_mul(u128::from(counts_per_rev))
        .ok_or(RungOriginRecoveryPlanError::Arithmetic)?
        .div_ceil(MREV_PER_REV);
    let travel_us = u128::from(budget.max_stroke_travel_mrev)
        .checked_mul(MICROS_PER_SECOND)
        .ok_or(RungOriginRecoveryPlanError::Arithmetic)?
        .div_ceil(u128::from(planned_velocity_mrev_s));
    let nominal_slot_us = recovery_allowance_us(travel_us)?;
    let maximum_slot_us = u128::from(budget.max_duration_ms)
        .checked_mul(1_000)
        .ok_or(RungOriginRecoveryPlanError::Arithmetic)?;
    if nominal_slot_us > maximum_slot_us {
        return Err(RungOriginRecoveryPlanError::ExceedsPrimitiveCap);
    }

    Ok(RungOriginRecoveryPlan {
        origin_band_counts: checked_u32(origin_band_counts)?,
        nominal_slot_us: checked_u32(nominal_slot_us)?,
        maximum_slot_us: checked_u32(maximum_slot_us)?,
    })
}

/// Return the conservative commanded-side magnitude of one settled-rate interval.
#[allow(dead_code)]
pub(crate) const fn commanded_side_lower_rate_q(
    direction: VelocityDirection,
    low_q: i64,
    high_q: i64,
) -> Option<u64> {
    if low_q > high_q {
        return None;
    }
    match direction {
        VelocityDirection::Forward if low_q > 0 => Some(low_q as u64),
        VelocityDirection::Reverse if high_q < 0 => Some(high_q.unsigned_abs()),
        VelocityDirection::Forward | VelocityDirection::Reverse => None,
    }
}

/// Return the distance that must be traversed before entering the inclusive band.
#[allow(dead_code)]
pub(crate) const fn recovery_return_distance_counts(
    start_offset_counts: i32,
    origin_band_counts: u32,
) -> Option<u32> {
    if start_offset_counts == i32::MIN || origin_band_counts == 0 {
        return None;
    }
    Some(
        start_offset_counts
            .unsigned_abs()
            .saturating_sub(origin_band_counts),
    )
}

/// Derive the runtime deadline from a measured settled-rate lower bound.
#[allow(dead_code)]
pub(crate) fn recovery_runtime_us(
    return_distance_counts: u32,
    lower_rate_q: u64,
    primitive_cap_us: u32,
) -> Result<u32, RungOriginRecoveryPlanError> {
    if lower_rate_q == 0 || primitive_cap_us == 0 {
        return Err(RungOriginRecoveryPlanError::InvalidRequest);
    }
    let rate_q_scale = u128::try_from(VELOCITY_RATE_Q_SCALE)
        .map_err(|_| RungOriginRecoveryPlanError::Arithmetic)?;
    let measured_return_us = u128::from(return_distance_counts)
        .checked_mul(MICROS_PER_SECOND)
        .and_then(|value| value.checked_mul(rate_q_scale))
        .ok_or(RungOriginRecoveryPlanError::Arithmetic)?
        .div_ceil(u128::from(lower_rate_q));
    let runtime_us = recovery_allowance_us(measured_return_us)?;
    if runtime_us > u128::from(primitive_cap_us) {
        return Err(RungOriginRecoveryPlanError::ExceedsPrimitiveCap);
    }
    checked_u32(runtime_us)
}

fn recovery_allowance_us(moving_us: u128) -> Result<u128, RungOriginRecoveryPlanError> {
    moving_us
        .checked_add(u128::from(MOVING_STROKE_US))
        .and_then(|value| value.checked_add(u128::from(ZERO_SETTLE_US)))
        .and_then(|value| value.checked_add(u128::from(SETTLE_ANALYSIS_BUDGET_US)))
        .ok_or(RungOriginRecoveryPlanError::Arithmetic)
}

fn checked_u32(value: u128) -> Result<u32, RungOriginRecoveryPlanError> {
    u32::try_from(value).map_err(|_| RungOriginRecoveryPlanError::Arithmetic)
}
