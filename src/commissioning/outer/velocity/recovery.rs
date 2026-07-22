//! Shared rung-origin recovery geometry and checked deadline arithmetic.

#![allow(dead_code)] // Stage-neutral worker is wired into Stage B/C in the next two tasks.

use crate::autotune::velocity_measurement::{
    DyadicBlocker, StationarityWork, VELOCITY_RATE_Q_SCALE, VelocityDirection,
};
use crate::autotune::{CommissionError, TmcMechanicalRpm};
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocityObservationRing};
use crate::commissioning::outer::{OuterMotionBudget, OuterSafetyLimits};
use crate::commissioning::types::OuterSafetyFault;
use foci_tmc4671::pid::PidGain;

use super::sweep_plan::{MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US, ZERO_SETTLE_US};

const MREV_PER_REV: u128 = 1_000;
const MICROS_PER_SECOND: u128 = 1_000_000;

/// Commissioning stage that owns one recovery record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
#[allow(dead_code)]
pub(crate) enum RecoveryStage {
    /// Proportional-gain identification.
    Proportional = 0,
    /// Integral-response characterization.
    Integral = 1,
}

/// Terminal classification for one bounded recovery attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum RungOriginRecoveryOutcome {
    /// The rung ended inside the reserve-derived band, so no motion was needed.
    AlreadyCentered = 0,
    /// Constant-amplitude recovery and fixed settle both completed.
    Recovered = 1,
    /// The needed direction had no commanded-side settled-rate authority.
    ContainedNoRate = 2,
    /// The return could not fit the declared geometry or deadline.
    ContainedDeadline = 3,
}

/// Exact reason that bounded or authorized the recovery attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum RecoveryBindingSource {
    AlreadyCentered = 0,
    SettledRate = 1,
    MissingRate = 2,
    Deadline = 3,
    LocalTravel = 4,
}

/// Complete stage-neutral summary of one recovery slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RungOriginRecoveryReport {
    pub(crate) stage: RecoveryStage,
    pub(crate) rung_index: u8,
    pub(crate) p_raw: u16,
    pub(crate) planned_velocity_mrev_s: u32,
    pub(crate) start_offset_counts: i32,
    pub(crate) closest_offset_counts: i32,
    pub(crate) final_offset_counts: i32,
    pub(crate) origin_band_counts: u32,
    pub(crate) lower_rate_q: u64,
    pub(crate) moving_duration_us: u32,
    pub(crate) settle_duration_us: u32,
    pub(crate) total_duration_us: u32,
    pub(crate) peak_torque_target_abs: u16,
    pub(crate) binding_source: RecoveryBindingSource,
    pub(crate) outcome: RungOriginRecoveryOutcome,
}

/// One cooperative result from the shared recovery worker.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RungOriginRecoveryStep {
    ReadyNow,
    At(u64),
    Finished(RungOriginRecoveryReport),
    Contained(RungOriginRecoveryReport),
    Fault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum RecoveryState {
    WriteGains,
    ClearMovingIntegrator,
    CommandTowardOrigin,
    Observe {
        moving_deadline_us: u64,
    },
    ClearSettleIntegrator {
        settle_started_us: u64,
        contained: bool,
    },
    CaptureSettle {
        next_sample_us: u64,
        deadline_us: u64,
        contained: bool,
    },
    AnalyzeSettle {
        analysis_deadline_us: u64,
        contained: bool,
    },
    Terminal {
        outcome: RungOriginRecoveryOutcome,
        binding_source: RecoveryBindingSource,
    },
}

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

/// Compact state retained by the owning phase while recovery borrows analysis storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RungOriginRecoveryRuntime {
    stage: RecoveryStage,
    rung_index: u8,
    p: PidGain,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    start_offset_counts: i32,
    closest_offset_counts: i32,
    lower_rate_q: u64,
    plan: RungOriginRecoveryPlan,
    runtime_us: u32,
    motion_started: bool,
    recovery_started_us: u64,
    zero_commanded_us: u64,
    peak_torque_target_abs: u16,
    state: RecoveryState,
}

impl RungOriginRecoveryRuntime {
    /// Construct one checked recovery attempt without touching hardware.
    pub(crate) fn new(
        stage: RecoveryStage,
        rung_index: u8,
        p: PidGain,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
        start_offset_counts: i32,
        lower_rate_q: Option<u64>,
        plan: RungOriginRecoveryPlan,
    ) -> Result<Self, CommissionError> {
        if p == PidGain::ZERO
            || target_velocity_rpm.raw() <= 0
            || planned_velocity_mrev_s == 0
            || plan.origin_band_counts == 0
            || start_offset_counts == i32::MIN
        {
            return Err(CommissionError::InvalidSchedule);
        }
        let return_distance =
            recovery_return_distance_counts(start_offset_counts, plan.origin_band_counts)
                .ok_or(CommissionError::InvalidSchedule)?;
        let (state, runtime_us, lower_rate_q) = if return_distance == 0 {
            (
                RecoveryState::Terminal {
                    outcome: RungOriginRecoveryOutcome::AlreadyCentered,
                    binding_source: RecoveryBindingSource::AlreadyCentered,
                },
                0,
                0,
            )
        } else if let Some(rate) = lower_rate_q.filter(|rate| *rate != 0) {
            match recovery_runtime_us(return_distance, rate, plan.maximum_slot_us) {
                Ok(runtime_us) => (RecoveryState::WriteGains, runtime_us, rate),
                Err(RungOriginRecoveryPlanError::ExceedsPrimitiveCap) => (
                    RecoveryState::Terminal {
                        outcome: RungOriginRecoveryOutcome::ContainedDeadline,
                        binding_source: RecoveryBindingSource::Deadline,
                    },
                    0,
                    rate,
                ),
                Err(_) => return Err(CommissionError::InvalidSchedule),
            }
        } else {
            (
                RecoveryState::Terminal {
                    outcome: RungOriginRecoveryOutcome::ContainedNoRate,
                    binding_source: RecoveryBindingSource::MissingRate,
                },
                0,
                0,
            )
        };
        Ok(Self {
            stage,
            rung_index,
            p,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            start_offset_counts,
            closest_offset_counts: start_offset_counts,
            lower_rate_q,
            plan,
            runtime_us,
            motion_started: false,
            recovery_started_us: 0,
            zero_commanded_us: 0,
            peak_torque_target_abs: 0,
            state,
        })
    }

    fn report(
        &self,
        outcome: RungOriginRecoveryOutcome,
        binding_source: RecoveryBindingSource,
        final_offset_counts: i32,
        now_us: u64,
    ) -> RungOriginRecoveryReport {
        let (moving_duration_us, settle_duration_us, total_duration_us) = if self.motion_started {
            (
                self.zero_commanded_us
                    .saturating_sub(self.recovery_started_us)
                    .min(u64::from(u32::MAX)) as u32,
                now_us
                    .saturating_sub(self.zero_commanded_us)
                    .min(u64::from(u32::MAX)) as u32,
                now_us
                    .saturating_sub(self.recovery_started_us)
                    .min(u64::from(u32::MAX)) as u32,
            )
        } else {
            (0, 0, 0)
        };
        RungOriginRecoveryReport {
            stage: self.stage,
            rung_index: self.rung_index,
            p_raw: self.p.raw(),
            planned_velocity_mrev_s: self.planned_velocity_mrev_s,
            start_offset_counts: self.start_offset_counts,
            closest_offset_counts: self.closest_offset_counts,
            final_offset_counts,
            origin_band_counts: self.plan.origin_band_counts,
            lower_rate_q: self.lower_rate_q,
            moving_duration_us,
            settle_duration_us,
            total_duration_us,
            peak_torque_target_abs: self.peak_torque_target_abs,
            binding_source,
            outcome,
        }
    }
}

/// Borrowed large state shared with the owning Stage-B or Stage-C phase.
pub(crate) struct RungOriginRecoveryResources<'a> {
    pub(crate) safety: &'a mut OuterSafetyLimits,
    pub(crate) observations: &'a mut VelocityObservationRing,
    pub(crate) blocker: &'a mut DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    pub(crate) analysis: &'a mut StationarityWork,
    pub(crate) settle_displacement_counts: &'a mut i8,
    pub(crate) settle_min_displacement_counts: &'a mut i8,
    pub(crate) settle_max_displacement_counts: &'a mut i8,
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
