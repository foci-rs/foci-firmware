//! Shared rung-origin recovery geometry and checked deadline arithmetic.

#[cfg(feature = "trace-velocity-sweep")]
mod controller;

#[cfg(feature = "trace-velocity-sweep")]
pub(crate) use controller::step_rung_origin_recovery;

#[cfg(feature = "trace-velocity-sweep")]
use crate::autotune::velocity_measurement::{
    DyadicBlocker, StationarityWork, VELOCITY_RATE_Q_SCALE, VelocityDirection,
};
#[cfg(not(feature = "trace-velocity-sweep"))]
use crate::autotune::velocity_measurement::{VELOCITY_RATE_Q_SCALE, VelocityDirection};
#[cfg(feature = "trace-velocity-sweep")]
use crate::autotune::{CommissionError, TmcMechanicalRpm};
use crate::commissioning::outer::OuterMotionBudget;
#[cfg(feature = "trace-velocity-sweep")]
use crate::commissioning::outer::OuterSafetyLimits;
#[cfg(feature = "trace-velocity-sweep")]
use crate::commissioning::outer::OuterSafetyTimingPhase;
#[cfg(feature = "trace-velocity-sweep")]
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocityObservationRing};
#[cfg(feature = "trace-velocity-sweep")]
use crate::commissioning::types::OuterSafetyFault;
#[cfg(feature = "trace-velocity-sweep")]
use foci_tmc4671::pid::PidPGain;

#[cfg(feature = "trace-velocity-sweep")]
use super::current_guard::VelocityCurrentObserver;
#[cfg(feature = "trace-velocity-sweep")]
use super::operating_region::CompactSignedRate;
use super::sweep_plan::{MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US, ZERO_SETTLE_US};

const MREV_PER_REV: u128 = 1_000;
const MICROS_PER_SECOND: u128 = 1_000_000;
pub(crate) const RECOVERY_STABILIZATION_US: u32 = 500_000;
/// Plan-digest code for the stabilization-before-score recovery schedule.
pub(crate) const STABILIZED_RECOVERY_SCHEDULE_CODE: u8 = 1;

/// Commissioning stage that owns one recovery record.
#[cfg(feature = "trace-velocity-sweep")]
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
#[cfg(feature = "trace-velocity-sweep")]
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
    /// The temporary velocity-output clamp engaged during recovery.
    RecoveryCurrentHeadroom = 4,
    /// Clamp contact was followed by confirmed passive in-band rest.
    RecoveredWithCurrentHeadroom = 5,
}

/// Response to a probe-clamp contact during one origin-recovery primitive.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RecoveryCurrentContactPolicy {
    /// Stop immediately and report the existing contained outcome.
    StopImmediately,
    /// Hold zero target and prove passive in-band rest without restarting settle.
    ProvePassiveSettle,
}

/// Zero-target rest schedule following one origin return.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RecoveryRestSchedule {
    /// Score the first fixed settle window.
    ImmediateScored,
    /// Discard one fixed lead-in before scoring a separate settle window.
    #[cfg(feature = "trace-velocity-sweep")]
    Stabilized,
}

/// Exact reason that bounded or authorized the recovery attempt.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum RecoveryBindingSource {
    AlreadyCentered = 0,
    SettledRate = 1,
    MissingRate = 2,
    Deadline = 3,
    LocalTravel = 4,
    StructuredSuffix = 5,
    CurrentHeadroom = 6,
}

/// Compact rate bound and the evidence source that authorized its use.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RecoveryRateAuthority {
    mantissa: i16,
    shift: u8,
    pub(crate) source: RecoveryBindingSource,
}

#[cfg(feature = "trace-velocity-sweep")]
impl RecoveryRateAuthority {
    pub(crate) const NONE: Self = Self {
        mantissa: 0,
        shift: 0,
        source: RecoveryBindingSource::MissingRate,
    };

    pub(crate) const fn settled(lower_rate: CompactSignedRate) -> Self {
        Self {
            mantissa: lower_rate.mantissa,
            shift: lower_rate.shift,
            source: RecoveryBindingSource::SettledRate,
        }
    }

    pub(crate) const fn structured(lower_rate: CompactSignedRate) -> Self {
        Self {
            mantissa: lower_rate.mantissa,
            shift: lower_rate.shift,
            source: RecoveryBindingSource::StructuredSuffix,
        }
    }

    pub(crate) fn lower_rate_q(self) -> Option<u64> {
        u64::try_from(self.decoded_rate())
            .ok()
            .filter(|rate| *rate != 0)
    }

    pub(crate) fn select(ordinary: Option<Self>, structured: Option<Self>) -> Self {
        match (ordinary, structured) {
            (Some(left), Some(right)) if right.decoded_rate() < left.decoded_rate() => right,
            (Some(left), Some(_)) => left,
            (Some(left), None) => left,
            (None, Some(right)) => right,
            (None, None) => Self::NONE,
        }
    }

    const fn decoded_rate(self) -> i64 {
        (self.mantissa as i64) << self.shift
    }
}

#[cfg(feature = "trace-velocity-sweep")]
const _: () = assert!(core::mem::size_of::<RecoveryRateAuthority>() <= 4);

/// Complete stage-neutral summary of one recovery slot.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RungOriginRecoveryReport {
    pub(crate) stage: RecoveryStage,
    pub(crate) rung_index: u8,
    pub(crate) p_raw: u16,
    pub(crate) planned_velocity_mrev_s: u32,
    pub(crate) start_offset_counts: i32,
    pub(crate) closest_offset_counts: i32,
    pub(crate) final_offset_counts: i32,
    pub(crate) origin_band_counts: u32,
    pub(crate) lower_rate_q: ExactU64Parts,
    pub(crate) moving_duration_us: u32,
    pub(crate) settle_duration_us: u32,
    pub(crate) total_duration_us: u32,
    pub(crate) peak_torque_target_abs: u16,
    pub(crate) binding_source: RecoveryBindingSource,
    pub(crate) outcome: RungOriginRecoveryOutcome,
}

/// Compact displacement summary for the unscored recovery stabilization window.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RecoveryStabilizationSummary {
    pub(crate) net_displacement_counts: i32,
    pub(crate) displacement_span_counts: u32,
}

/// Completed recovery rest analysis before run identity is attached.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RecoveryRestAnalysis {
    pub(crate) exact_velocity_interval_q: [i64; 2],
    pub(crate) start_offset_counts: i32,
    pub(crate) closest_offset_counts: i32,
    pub(crate) final_offset_counts: i32,
    pub(crate) planned_return_distance_counts: u32,
    pub(crate) first_in_band_offset_counts: i32,
    pub(crate) displacement_to_band_counts: i32,
    pub(crate) entry_to_band_duration_us: u32,
    pub(crate) lead_in: RecoveryStabilizationSummary,
    pub(crate) scored_net_displacement_counts: i32,
    pub(crate) scored_min_displacement_counts: i32,
    pub(crate) scored_max_displacement_counts: i32,
    pub(crate) p_raw: u16,
    pub(crate) peak_torque_target_abs: u16,
    pub(crate) stage: RecoveryStage,
    pub(crate) rung_index: u8,
    pub(crate) stationarity_code: u8,
    pub(crate) interval_present: u8,
    pub(crate) current_contact_code: u8,
    pub(crate) disposition: u8,
}

/// One cooperative result from the shared recovery worker.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RungOriginRecoveryStep {
    ReadyNow,
    At(u64),
    RestSelection {
        decision: crate::autotune::velocity_measurement::RestSelectionDecision,
        stage: RecoveryStage,
        rung_index: u8,
    },
    Finished(RungOriginRecoveryReport),
    Contained(RungOriginRecoveryReport),
    RestAnalysis(RecoveryRestAnalysis),
    RestUnconfirmed,
    Fault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum RecoveryState {
    WriteGains,
    ClearMovingIntegrator,
    CommandTowardOrigin,
    Observe {
        moving_deadline_us: ExactU64Parts,
    },
    ClearSettleIntegrator {
        settle_started_us: ExactU64Parts,
        contained: bool,
        lead_in: bool,
    },
    CaptureSettle {
        next_sample_us: ExactU64Parts,
        deadline_us: ExactU64Parts,
        contained: bool,
        lead_in: bool,
    },
    AnalyzeSettle {
        analysis_deadline_us: ExactU64Parts,
        contained: bool,
    },
    RestTerminal {
        confirmed: bool,
    },
    Terminal {
        outcome: RungOriginRecoveryOutcome,
        binding_source: RecoveryBindingSource,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(C)]
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) struct ExactU64Parts {
    low: u32,
    high: u32,
}

#[cfg(feature = "trace-velocity-sweep")]
impl ExactU64Parts {
    const ZERO: Self = Self { low: 0, high: 0 };

    pub(crate) const fn from_u64(value: u64) -> Self {
        Self {
            low: value as u32,
            high: (value >> 32) as u32,
        }
    }

    pub(crate) const fn to_u64(self) -> u64 {
        self.low as u64 | ((self.high as u64) << 32)
    }
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
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RungOriginRecoveryRuntime {
    stage: RecoveryStage,
    rung_index: u8,
    p: PidPGain,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    start_offset_counts: i32,
    closest_offset_counts: i32,
    lower_rate_q: ExactU64Parts,
    authority_source: RecoveryBindingSource,
    plan: RungOriginRecoveryPlan,
    runtime_us: u32,
    flags: u8,
    recovery_started_us: ExactU64Parts,
    zero_commanded_us: ExactU64Parts,
    first_in_band_offset_counts: i32,
    entry_to_band_duration_us: u32,
    lead_in_first_or_net_counts: i32,
    lead_in_min_or_span_counts: i32,
    state: RecoveryState,
}

/// Immutable inputs for one checked rung-origin recovery attempt.
#[cfg(feature = "trace-velocity-sweep")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RungOriginRecoveryConfig {
    pub(crate) stage: RecoveryStage,
    pub(crate) rung_index: u8,
    pub(crate) p: PidPGain,
    pub(crate) target_velocity_rpm: TmcMechanicalRpm,
    pub(crate) planned_velocity_mrev_s: u32,
    pub(crate) start_offset_counts: i32,
    pub(crate) lower_rate_q: Option<u64>,
    pub(crate) authority_source: RecoveryBindingSource,
    pub(crate) current_contact_policy: RecoveryCurrentContactPolicy,
    pub(crate) rest_schedule: RecoveryRestSchedule,
}

#[cfg(feature = "trace-velocity-sweep")]
impl RungOriginRecoveryRuntime {
    const MOTION_STARTED: u8 = 1 << 0;
    const PROVE_PASSIVE_SETTLE: u8 = 1 << 1;
    const CURRENT_CONTACT_DETECTED: u8 = 1 << 2;
    const SETTLE_INTEGRATOR_CLEARED: u8 = 1 << 3;
    const STABILIZED: u8 = 1 << 4;

    /// Construct one checked recovery attempt without touching hardware.
    pub(crate) fn new(
        config: RungOriginRecoveryConfig,
        plan: RungOriginRecoveryPlan,
    ) -> Result<Self, CommissionError> {
        if config.p == PidPGain::ZERO
            || config.target_velocity_rpm.raw() <= 0
            || config.planned_velocity_mrev_s == 0
            || plan.origin_band_counts == 0
            || config.start_offset_counts == i32::MIN
        {
            return Err(CommissionError::InvalidSchedule);
        }
        let lower_rate_q = match (config.lower_rate_q, config.authority_source) {
            (
                Some(rate),
                RecoveryBindingSource::SettledRate | RecoveryBindingSource::StructuredSuffix,
            ) if rate != 0 => Some(rate),
            (None, RecoveryBindingSource::MissingRate) => None,
            _ => return Err(CommissionError::InvalidSchedule),
        };
        let return_distance =
            recovery_return_distance_counts(config.start_offset_counts, plan.origin_band_counts)
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
        } else if let Some(rate) = lower_rate_q {
            match recovery_runtime_us_with_schedule(
                return_distance,
                rate,
                plan.maximum_slot_us,
                config.rest_schedule,
            ) {
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
            stage: config.stage,
            rung_index: config.rung_index,
            p: config.p,
            target_velocity_rpm: config.target_velocity_rpm,
            planned_velocity_mrev_s: config.planned_velocity_mrev_s,
            start_offset_counts: config.start_offset_counts,
            closest_offset_counts: config.start_offset_counts,
            lower_rate_q: ExactU64Parts::from_u64(lower_rate_q),
            authority_source: config.authority_source,
            plan,
            runtime_us,
            flags: (u8::from(
                config.current_contact_policy == RecoveryCurrentContactPolicy::ProvePassiveSettle,
            ) * Self::PROVE_PASSIVE_SETTLE)
                | (u8::from(config.rest_schedule == RecoveryRestSchedule::Stabilized)
                    * Self::STABILIZED),
            recovery_started_us: ExactU64Parts::ZERO,
            zero_commanded_us: ExactU64Parts::ZERO,
            first_in_band_offset_counts: 0,
            entry_to_band_duration_us: 0,
            lead_in_first_or_net_counts: 0,
            lead_in_min_or_span_counts: 0,
            state,
        })
    }

    pub(crate) const fn outer_safety_timing_phase(&self) -> (OuterSafetyTimingPhase, bool) {
        match self.state {
            RecoveryState::WriteGains
            | RecoveryState::ClearMovingIntegrator
            | RecoveryState::CommandTowardOrigin => (OuterSafetyTimingPhase::RecoverySetup, true),
            RecoveryState::Observe { .. } => (OuterSafetyTimingPhase::RecoveryMove, false),
            RecoveryState::ClearSettleIntegrator { .. }
            | RecoveryState::CaptureSettle { .. }
            | RecoveryState::AnalyzeSettle { .. } => (OuterSafetyTimingPhase::RecoverySettle, true),
            RecoveryState::RestTerminal { .. } => (OuterSafetyTimingPhase::Terminalization, true),
            RecoveryState::Terminal { .. } => (OuterSafetyTimingPhase::Terminalization, true),
        }
    }

    fn motion_started(&self) -> bool {
        self.flags & Self::MOTION_STARTED != 0
    }

    fn mark_motion_started(&mut self) {
        self.flags |= Self::MOTION_STARTED;
    }

    fn proves_passive_settle(&self) -> bool {
        self.flags & Self::PROVE_PASSIVE_SETTLE != 0
    }

    fn current_contact_detected(&self) -> bool {
        self.flags & Self::CURRENT_CONTACT_DETECTED != 0
    }

    fn mark_current_contact_detected(&mut self) {
        self.flags |= Self::CURRENT_CONTACT_DETECTED | Self::SETTLE_INTEGRATOR_CLEARED;
    }

    fn settle_integrator_cleared(&self) -> bool {
        self.flags & Self::SETTLE_INTEGRATOR_CLEARED != 0
    }

    fn uses_stabilization(&self) -> bool {
        self.flags & Self::STABILIZED != 0
    }

    fn rest_allowance_us(&self) -> u32 {
        ZERO_SETTLE_US
            + SETTLE_ANALYSIS_BUDGET_US
            + if self.uses_stabilization() {
                RECOVERY_STABILIZATION_US
            } else {
                0
            }
    }

    fn report(
        &self,
        outcome: RungOriginRecoveryOutcome,
        binding_source: RecoveryBindingSource,
        final_offset_counts: i32,
        now_us: u64,
        peak_torque_target_abs: u16,
    ) -> RungOriginRecoveryReport {
        let (moving_duration_us, settle_duration_us, total_duration_us) = if self.motion_started() {
            (
                self.zero_commanded_us
                    .to_u64()
                    .saturating_sub(self.recovery_started_us.to_u64())
                    .min(u64::from(u32::MAX)) as u32,
                now_us
                    .saturating_sub(self.zero_commanded_us.to_u64())
                    .min(u64::from(u32::MAX)) as u32,
                now_us
                    .saturating_sub(self.recovery_started_us.to_u64())
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
            peak_torque_target_abs,
            binding_source,
            outcome,
        }
    }
}

/// Borrowed large state shared with the owning Stage-B or Stage-C phase.
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) struct RungOriginRecoveryResources<'a> {
    pub(crate) safety: &'a mut OuterSafetyLimits,
    pub(crate) observations: &'a mut VelocityObservationRing,
    pub(crate) blocker: &'a mut DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    pub(crate) analysis: &'a mut StationarityWork,
    pub(crate) settle_displacement_counts: &'a mut i8,
    pub(crate) settle_min_displacement_counts: &'a mut i8,
    pub(crate) settle_max_displacement_counts: &'a mut i8,
    pub(crate) current_observer: &'a mut VelocityCurrentObserver,
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
    recovery_runtime_us_with_schedule(
        return_distance_counts,
        lower_rate_q,
        primitive_cap_us,
        RecoveryRestSchedule::ImmediateScored,
    )
}

/// Derive a schedule-specific runtime deadline from a settled-rate lower bound.
#[allow(dead_code)]
pub(crate) fn recovery_runtime_us_with_schedule(
    return_distance_counts: u32,
    lower_rate_q: u64,
    primitive_cap_us: u32,
    rest_schedule: RecoveryRestSchedule,
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
    let runtime_us = recovery_allowance_us(measured_return_us)?
        .checked_add(match rest_schedule {
            RecoveryRestSchedule::ImmediateScored => 0,
            #[cfg(feature = "trace-velocity-sweep")]
            RecoveryRestSchedule::Stabilized => u128::from(RECOVERY_STABILIZATION_US),
        })
        .ok_or(RungOriginRecoveryPlanError::Arithmetic)?;
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
