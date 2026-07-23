use crate::autotune::TmcMechanicalRpm;
use foci_tmc4671::pid::PidGain;
use heapless::Vec;

use super::recovery::{RungOriginRecoveryPlanError, plan_rung_origin_recovery};

/// Numerator of the runtime velocity-envelope proportional margin.
pub const ENVELOPE_PROPORTIONAL_NUM: u32 = 3;
/// Denominator of the runtime velocity-envelope proportional margin.
pub const ENVELOPE_PROPORTIONAL_DEN: u32 = 2;
/// Absolute runtime velocity-envelope margin in mechanical millirevolutions per second.
pub const ENVELOPE_ABSOLUTE_MARGIN_MREV_S: u32 = 2_000;
/// Fixed duration of one moving observation.
pub const MOVING_STROKE_US: u32 = 512_000;
/// Fixed duration of one zero-target settle.
pub const ZERO_SETTLE_US: u32 = 500_000;
/// Sampling period used by moving and zero-target velocity observations.
pub const VELOCITY_SWEEP_SAMPLE_PERIOD_US: u32 = 1_000;
/// Minimum zero-target capture retained for the shortest stationarity suffix.
pub const MIN_SETTLE_CAPTURE_US: u32 =
    crate::autotune::velocity_measurement::VELOCITY_SUFFIX_LENGTHS[0] as u32
        * VELOCITY_SWEEP_SAMPLE_PERIOD_US;
/// Maximum cooperative analysis budget validated independently of the rest window.
pub const SETTLE_ANALYSIS_BUDGET_US: u32 = 236_000;
/// Minimum number of distinct gains that can support the required evidence geometry.
pub const MIN_STRUCTURAL_RUNGS: usize = 5;
/// Maximum audited gain count in one deterministic sweep.
pub const MAX_SWEEP_RUNGS: usize = 32;
/// Independent observations collected in each direction at every rung.
pub const OBSERVATIONS_PER_DIRECTION_PER_RUNG: u16 = 4;
/// Signed directions collected at every gain rung.
pub const SWEEP_DIRECTION_COUNT: u16 = 2;
/// Total moving observations collected at every gain rung.
pub const SWEEP_SLOT_COUNT: u8 =
    (OBSERVATIONS_PER_DIRECTION_PER_RUNG * SWEEP_DIRECTION_COUNT) as u8;
/// Numerator of the usable current-headroom boundary.
pub const CURRENT_HEADROOM_NUM: u32 = 9;
/// Denominator of the usable current-headroom boundary.
pub const CURRENT_HEADROOM_DEN: u32 = 10;

const MICROS_PER_SECOND: u64 = 1_000_000;
/// Maximum measured time between encoder reads before wrap-safe interpretation is refused.
///
/// The recovery-enabled trace campaign observed a valid 14,329 us gap after the
/// highest-P rung record. Twenty milliseconds is the largest 5 ms-aligned bound
/// that keeps the live 10,000 mrev/s host ceiling below encoder observability.
pub const MAX_SAFETY_OBSERVATION_GAP_US: u64 = 20_000;
const VELOCITY_P_DIVISOR: u32 = 256;
const HALF_OCTAVE_NUM: u32 = 181;
const HALF_OCTAVE_DEN_SHIFT: u32 = 7;

/// The effective ceiling is the host-declared machine ceiling.
pub const EFFECTIVE_CEILING_MACHINE: u8 = 0;
/// The effective ceiling is limited by wrap-safe encoder observability.
pub const EFFECTIVE_CEILING_ENCODER: u8 = 1;
/// The requested amplitude exceeded the normal envelope-compatible amplitude.
pub const CLAMP_ENVELOPE: u16 = 1 << 0;
/// The requested amplitude exceeded the declared moving-travel budget.
pub const CLAMP_TRAVEL: u16 = 1 << 1;
/// A firmware-owned observability or representation ceiling reduced the machine ceiling.
pub const CLAMP_EFFECTIVE_CEILING: u16 = 1 << 2;

/// Origin of the host's requested velocity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RequestedVelocitySource {
    /// The operator supplied `TUNE_VELOCITY`.
    Explicit,
    /// The host derived the envelope-compatible default.
    Defaulted,
}

/// Host-planned motion and geometry inputs for one velocity sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepRequest {
    /// Requested mechanical velocity amplitude.
    pub requested_velocity_mrev_s: u32,
    /// Kinematics-derived absolute machine velocity ceiling.
    pub machine_velocity_ceiling_mrev_s: u32,
    /// Whether the requested amplitude was explicit or defaulted.
    pub requested_velocity_source: RequestedVelocitySource,
    /// Complete displacement window for one moving stroke and its settle.
    pub max_stroke_travel_mrev: u32,
    /// Portion of the displacement window reserved for braking and settle drift.
    pub settle_travel_reserve_mrev: u32,
    /// Negative unwrapped position headroom from the Stage-2 anchor.
    pub negative_position_headroom_mrev: u32,
    /// Positive unwrapped position headroom from the Stage-2 anchor.
    pub positive_position_headroom_mrev: u32,
    /// Host-declared upper workflow budget.
    pub max_duration_ms: u16,
}

/// Complete immutable motion plan produced before the first sweep stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepPlan {
    /// Host-requested velocity amplitude.
    pub requested_velocity_mrev_s: u32,
    /// Frozen velocity amplitude used for every rung.
    pub planned_velocity_mrev_s: u32,
    /// Origin of the requested amplitude.
    pub requested_velocity_source: RequestedVelocitySource,
    /// Bitset describing every amplitude reduction.
    pub clamp_flags: u16,
    /// Lowest machine, observability, or representation ceiling.
    pub effective_ceiling_mrev_s: u32,
    /// Source code for the binding effective ceiling.
    pub effective_ceiling_source: u8,
    /// Frozen positive TMC velocity-target register magnitude.
    pub target_velocity_rpm: TmcMechanicalRpm,
    /// Smallest gain producing one nonzero controller-output quantum.
    pub p_start: PidGain,
    /// Largest gain at the inclusive usable-current boundary.
    pub p_top: PidGain,
    /// Post-dedupe anchored half-octave gains; unused entries are zero.
    pub gains: [PidGain; MAX_SWEEP_RUNGS],
    /// Number of valid entries in `gains`.
    pub rung_count: u8,
    /// Independent observations planned in each direction per rung.
    pub observations_per_direction: u16,
    /// Fixed moving-stroke duration.
    pub moving_stroke_us: u32,
    /// Fixed zero-target settle duration.
    pub zero_settle_us: u32,
    /// Exact nominal workflow duration for the fixed schedule.
    pub nominal_workflow_ms: u32,
    /// Maximum planned duration; equal to nominal while the schedule has no extension.
    pub maximum_workflow_ms: u32,
    /// Inclusive reserve-derived origin band used after fully acquired rungs.
    pub recovery_origin_band_counts: u32,
    /// Conservative nominal duration reserved for one recovery slot.
    pub recovery_nominal_slot_us: u32,
    /// Host-declared maximum duration reserved for one recovery slot.
    pub recovery_maximum_slot_us: u32,
    /// Maximum number of recovery slots reserved by this plan.
    pub recovery_slot_count: u8,
    /// Complete per-primitive displacement window.
    pub max_stroke_travel_mrev: u32,
    /// Reserved braking and settle displacement.
    pub settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom.
    pub negative_position_headroom_mrev: u32,
    /// Positive absolute-position headroom.
    pub positive_position_headroom_mrev: u32,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive nine-tenths torque-target boundary.
    pub usable_torque_limit: u16,
}

/// Compact immutable plan retained after the complete pre-motion report is emitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepExecutionPlan {
    /// Digest of every frozen request, bound, gain, duration, and geometry field.
    pub plan_digest: u64,
    /// Frozen encoder-domain velocity amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen positive TMC mechanical-RPM target.
    pub target_velocity_rpm: TmcMechanicalRpm,
    /// Structural ladder anchor.
    pub p_start: PidGain,
    /// Explicit structural ladder top.
    pub p_top: PidGain,
    /// Number of post-dedupe rungs.
    pub rung_count: u8,
    /// Independent observations per direction and rung.
    pub observations_per_direction: u16,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
}

impl VelocitySweepExecutionPlan {
    /// Sentinel used before a trace-gated sweep has an admitted immutable plan.
    pub const EMPTY: Self = Self {
        plan_digest: 0,
        planned_velocity_mrev_s: 0,
        target_velocity_rpm: TmcMechanicalRpm::from_register(0),
        p_start: PidGain::ZERO,
        p_top: PidGain::ZERO,
        rung_count: 0,
        observations_per_direction: 0,
        hard_torque_limit: 0,
        usable_torque_limit: 0,
    };

    /// Whether command admission has installed a usable plan.
    pub const fn is_present(self) -> bool {
        self.rung_count != 0
    }

    /// Regenerate one exact post-dedupe gain without retaining the ladder array.
    pub fn gain_at(self, rung_index: u8) -> Option<PidGain> {
        gain_at(
            self.p_start.raw(),
            self.p_top.raw(),
            self.rung_count,
            rung_index,
        )
    }

    /// Whether the final predeclared rung is bounded by usable current headroom.
    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn structural_current_bound(
        self,
        rung_index: u8,
    ) -> Result<bool, VelocityPlanError> {
        if rung_index
            .checked_add(1)
            .is_none_or(|next| next != self.rung_count)
        {
            return Ok(false);
        }
        let target = self.target_velocity_rpm.raw().unsigned_abs();
        if target == 0 || self.usable_torque_limit == 0 {
            return Err(VelocityPlanError::InvalidRequest);
        }
        let unbounded_current_p_top = VELOCITY_P_DIVISOR
            .checked_mul(u32::from(self.usable_torque_limit))
            .ok_or(VelocityPlanError::Arithmetic)?
            / target;
        Ok(unbounded_current_p_top <= u32::from(PidGain::MAX.raw()))
    }
}

const _: () = assert!(core::mem::size_of::<VelocitySweepExecutionPlan>() == 32);

impl VelocitySweepPlan {
    /// Hash every frozen plan field and emitted post-dedupe gain exactly once.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        macro_rules! hash_value {
            ($value:expr) => {
                for byte in $value.to_le_bytes() {
                    hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
                }
            };
        }
        hash_value!(self.requested_velocity_mrev_s);
        hash_value!(self.planned_velocity_mrev_s);
        hash_value!(self.requested_velocity_source as u8);
        hash_value!(self.clamp_flags);
        hash_value!(self.effective_ceiling_mrev_s);
        hash_value!(self.effective_ceiling_source);
        hash_value!(self.target_velocity_rpm.raw());
        hash_value!(self.p_start.raw());
        hash_value!(self.p_top.raw());
        hash_value!(self.rung_count);
        for gain in self.gains.iter().take(usize::from(self.rung_count)) {
            hash_value!(gain.raw());
        }
        hash_value!(self.observations_per_direction);
        hash_value!(self.moving_stroke_us);
        hash_value!(self.zero_settle_us);
        hash_value!(self.nominal_workflow_ms);
        hash_value!(self.maximum_workflow_ms);
        hash_value!(self.recovery_origin_band_counts);
        hash_value!(self.recovery_nominal_slot_us);
        hash_value!(self.recovery_maximum_slot_us);
        hash_value!(self.recovery_slot_count);
        hash_value!(self.max_stroke_travel_mrev);
        hash_value!(self.settle_travel_reserve_mrev);
        hash_value!(self.negative_position_headroom_mrev);
        hash_value!(self.positive_position_headroom_mrev);
        hash_value!(self.hard_torque_limit);
        hash_value!(self.usable_torque_limit);
        hash
    }

    /// Compact this complete report plan into the state needed after emission.
    pub const fn execution_plan(self, plan_digest: u64) -> VelocitySweepExecutionPlan {
        VelocitySweepExecutionPlan {
            plan_digest,
            planned_velocity_mrev_s: self.planned_velocity_mrev_s,
            target_velocity_rpm: self.target_velocity_rpm,
            p_start: self.p_start,
            p_top: self.p_top,
            rung_count: self.rung_count,
            observations_per_direction: self.observations_per_direction,
            hard_torque_limit: self.hard_torque_limit,
            usable_torque_limit: self.usable_torque_limit,
        }
    }

    /// Build the exact pre-motion evidence record for this immutable plan.
    pub fn report(
        self,
        run_sequence: u32,
        evidence_sequence: u16,
    ) -> crate::commissioning::types::VelocitySweepPlanned {
        crate::commissioning::types::VelocitySweepPlanned {
            run_sequence,
            evidence_sequence,
            plan_digest: self.digest(),
            requested_velocity_mrev_s: self.requested_velocity_mrev_s,
            requested_velocity_source: match self.requested_velocity_source {
                RequestedVelocitySource::Explicit => 0,
                RequestedVelocitySource::Defaulted => 1,
            },
            planned_velocity_mrev_s: self.planned_velocity_mrev_s,
            effective_ceiling_mrev_s: self.effective_ceiling_mrev_s,
            clamp_flags: self.clamp_flags,
            effective_ceiling_source: self.effective_ceiling_source,
            target_velocity_rpm: self.target_velocity_rpm.raw(),
            p_start: self.p_start.raw(),
            p_top: self.p_top.raw(),
            rung_count: self.rung_count,
            observations_per_direction: self.observations_per_direction,
            moving_stroke_us: self.moving_stroke_us,
            zero_settle_us: self.zero_settle_us,
            nominal_workflow_ms: self.nominal_workflow_ms,
            maximum_workflow_ms: self.maximum_workflow_ms,
            recovery_origin_band_counts: self.recovery_origin_band_counts,
            recovery_nominal_slot_us: self.recovery_nominal_slot_us,
            recovery_maximum_slot_us: self.recovery_maximum_slot_us,
            recovery_slot_count: self.recovery_slot_count,
            max_stroke_travel_mrev: self.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: self.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: self.negative_position_headroom_mrev,
            positive_position_headroom_mrev: self.positive_position_headroom_mrev,
            hard_torque_limit: self.hard_torque_limit,
            usable_torque_limit: self.usable_torque_limit,
        }
    }
}

/// Failure to construct a safe deterministic sweep before motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityPlanError {
    /// A required request or hardware scale was zero or inconsistent.
    InvalidRequest,
    /// Positive travel remained after the reserve, but its rate floored to zero.
    ZeroMovingTravelAmplitude,
    /// The current-limited gain range cannot hold five distinct rungs.
    InsufficientStructuralSpan,
    /// The planned observation family exceeds the audited confidence table.
    ConfidenceFamilyTooLarge,
    /// Checked planner arithmetic exceeded its representation.
    Arithmetic,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RepresentabilityCeilings {
    encoder_observable_mrev_s: u32,
}

/// Construct one immutable velocity sweep from host geometry and firmware limits.
pub fn plan_velocity_sweep(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_flux_limit: u16,
) -> Result<VelocitySweepPlan, VelocityPlanError> {
    if request.requested_velocity_mrev_s == 0
        || request.machine_velocity_ceiling_mrev_s == 0
        || request.negative_position_headroom_mrev == 0
        || request.positive_position_headroom_mrev == 0
        || request.max_duration_ms == 0
        || torque_flux_limit == 0
    {
        return Err(VelocityPlanError::InvalidRequest);
    }

    let ceilings = representability_ceilings(encoder_counts_per_rev)?;
    let candidates = [
        (
            request.machine_velocity_ceiling_mrev_s,
            EFFECTIVE_CEILING_MACHINE,
        ),
        (
            ceilings.encoder_observable_mrev_s,
            EFFECTIVE_CEILING_ENCODER,
        ),
    ];
    let (effective_ceiling_mrev_s, effective_ceiling_source) = candidates
        .into_iter()
        .min_by_key(|candidate| candidate.0)
        .ok_or(VelocityPlanError::InvalidRequest)?;
    let envelope_velocity_cap = envelope_cap(effective_ceiling_mrev_s)?;
    let travel_velocity_cap = travel_cap(
        request.max_stroke_travel_mrev,
        request.settle_travel_reserve_mrev,
        MOVING_STROKE_US,
    )?;
    let planned_velocity_mrev_s = request
        .requested_velocity_mrev_s
        .min(envelope_velocity_cap)
        .min(travel_velocity_cap);
    if planned_velocity_mrev_s == 0 {
        return Err(VelocityPlanError::ZeroMovingTravelAmplitude);
    }

    let mut clamp_flags = 0;
    if request.requested_velocity_mrev_s > envelope_velocity_cap {
        clamp_flags |= CLAMP_ENVELOPE;
    }
    if request.requested_velocity_mrev_s > travel_velocity_cap {
        clamp_flags |= CLAMP_TRAVEL;
    }
    if effective_ceiling_mrev_s < request.machine_velocity_ceiling_mrev_s {
        clamp_flags |= CLAMP_EFFECTIVE_CEILING;
    }

    let target_velocity_rpm = TmcMechanicalRpm::from_mrev_s_nearest(planned_velocity_mrev_s)
        .map_err(|_| VelocityPlanError::ZeroMovingTravelAmplitude)?;
    let recovery = plan_rung_origin_recovery(
        crate::commissioning::outer::OuterMotionBudget {
            requested_velocity_mrev_s: request.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: request.requested_velocity_source,
            max_stroke_travel_mrev: request.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: request.negative_position_headroom_mrev,
            positive_position_headroom_mrev: request.positive_position_headroom_mrev,
            max_duration_ms: request.max_duration_ms,
        },
        encoder_counts_per_rev,
        planned_velocity_mrev_s,
    )
    .map_err(map_recovery_plan_error)?;
    let (p_start, p_top, usable_torque_limit) =
        current_gain_span(target_velocity_rpm, torque_flux_limit)?;
    let ladder = build_gain_ladder(p_start.raw(), p_top.raw())?;
    let rung_count = u8::try_from(ladder.len()).map_err(|_| VelocityPlanError::Arithmetic)?;
    let family_size = usize::from(rung_count)
        .checked_mul(usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG))
        .ok_or(VelocityPlanError::Arithmetic)?;
    if family_size > usize::from(crate::autotune::velocity_measurement::MAX_CONFIDENCE_FAMILY_SIZE)
    {
        return Err(VelocityPlanError::ConfidenceFamilyTooLarge);
    }
    let primitive_us = MOVING_STROKE_US
        .checked_add(ZERO_SETTLE_US)
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityPlanError::Arithmetic)?;
    if primitive_us.div_ceil(1_000) > u32::from(request.max_duration_ms) {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let workflow_us = u32::from(rung_count)
        .checked_mul(u32::from(SWEEP_SLOT_COUNT))
        .and_then(|value| value.checked_mul(primitive_us))
        .ok_or(VelocityPlanError::Arithmetic)?;
    let nominal_workflow_us = workflow_us
        .checked_add(
            u32::from(rung_count)
                .checked_mul(recovery.nominal_slot_us)
                .ok_or(VelocityPlanError::Arithmetic)?,
        )
        .ok_or(VelocityPlanError::Arithmetic)?;
    let maximum_workflow_us = workflow_us
        .checked_add(
            u32::from(rung_count)
                .checked_mul(recovery.maximum_slot_us)
                .ok_or(VelocityPlanError::Arithmetic)?,
        )
        .ok_or(VelocityPlanError::Arithmetic)?;
    let nominal_workflow_ms = nominal_workflow_us.div_ceil(1_000);
    let maximum_workflow_ms = maximum_workflow_us.div_ceil(1_000);
    let mut gains = [PidGain::ZERO; MAX_SWEEP_RUNGS];
    gains[..ladder.len()].copy_from_slice(ladder.as_slice());

    Ok(VelocitySweepPlan {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        planned_velocity_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        clamp_flags,
        effective_ceiling_mrev_s,
        effective_ceiling_source,
        target_velocity_rpm,
        p_start,
        p_top,
        gains,
        rung_count,
        observations_per_direction: OBSERVATIONS_PER_DIRECTION_PER_RUNG,
        moving_stroke_us: MOVING_STROKE_US,
        zero_settle_us: ZERO_SETTLE_US,
        nominal_workflow_ms,
        maximum_workflow_ms,
        recovery_origin_band_counts: recovery.origin_band_counts,
        recovery_nominal_slot_us: recovery.nominal_slot_us,
        recovery_maximum_slot_us: recovery.maximum_slot_us,
        recovery_slot_count: rung_count,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        hard_torque_limit: torque_flux_limit,
        usable_torque_limit,
    })
}

fn map_recovery_plan_error(error: RungOriginRecoveryPlanError) -> VelocityPlanError {
    match error {
        RungOriginRecoveryPlanError::InvalidRequest
        | RungOriginRecoveryPlanError::ExceedsPrimitiveCap => VelocityPlanError::InvalidRequest,
        RungOriginRecoveryPlanError::Arithmetic => VelocityPlanError::Arithmetic,
    }
}

fn envelope_cap(effective_ceiling_mrev_s: u32) -> Result<u32, VelocityPlanError> {
    if effective_ceiling_mrev_s <= ENVELOPE_ABSOLUTE_MARGIN_MREV_S {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let proportional = u64::from(effective_ceiling_mrev_s)
        .checked_mul(u64::from(ENVELOPE_PROPORTIONAL_DEN))
        .ok_or(VelocityPlanError::Arithmetic)?
        / u64::from(ENVELOPE_PROPORTIONAL_NUM);
    let absolute = effective_ceiling_mrev_s - ENVELOPE_ABSOLUTE_MARGIN_MREV_S;
    Ok(u32::try_from(proportional)
        .map_err(|_| VelocityPlanError::Arithmetic)?
        .min(absolute))
}

fn travel_cap(
    full_window_mrev: u32,
    settle_reserve_mrev: u32,
    moving_stroke_us: u32,
) -> Result<u32, VelocityPlanError> {
    if moving_stroke_us == 0 || settle_reserve_mrev >= full_window_mrev {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let moving_window = full_window_mrev - settle_reserve_mrev;
    let cap = u64::from(moving_window)
        .checked_mul(MICROS_PER_SECOND)
        .ok_or(VelocityPlanError::Arithmetic)?
        / u64::from(moving_stroke_us);
    if cap == 0 {
        return Err(VelocityPlanError::ZeroMovingTravelAmplitude);
    }
    u32::try_from(cap).map_err(|_| VelocityPlanError::Arithmetic)
}

fn representability_ceilings(
    encoder_counts_per_rev: u32,
) -> Result<RepresentabilityCeilings, VelocityPlanError> {
    if encoder_counts_per_rev == 0 {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let quarter_turn_fault_counts = u64::from(encoder_counts_per_rev).div_ceil(4);
    let max_nonfaulting_delta_counts = quarter_turn_fault_counts
        .checked_sub(1)
        .ok_or(VelocityPlanError::InvalidRequest)?;
    let encoder_denominator = u64::from(encoder_counts_per_rev)
        .checked_mul(MAX_SAFETY_OBSERVATION_GAP_US)
        .ok_or(VelocityPlanError::Arithmetic)?;
    let encoder_observable_mrev_s = max_nonfaulting_delta_counts
        .checked_mul(1_000_000_000)
        .ok_or(VelocityPlanError::Arithmetic)?
        / encoder_denominator;
    Ok(RepresentabilityCeilings {
        encoder_observable_mrev_s: u32::try_from(encoder_observable_mrev_s)
            .map_err(|_| VelocityPlanError::Arithmetic)?,
    })
}

fn current_gain_span(
    target_velocity_rpm: TmcMechanicalRpm,
    hard_torque_limit: u16,
) -> Result<(PidGain, PidGain, u16), VelocityPlanError> {
    let target = target_velocity_rpm.raw().unsigned_abs();
    if target == 0 || hard_torque_limit == 0 {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let usable = u32::from(hard_torque_limit)
        .checked_mul(CURRENT_HEADROOM_NUM)
        .ok_or(VelocityPlanError::Arithmetic)?
        / CURRENT_HEADROOM_DEN;
    if usable == 0 {
        return Err(VelocityPlanError::InsufficientStructuralSpan);
    }
    let p_start_raw = VELOCITY_P_DIVISOR.div_ceil(target);
    let p_top_raw = VELOCITY_P_DIVISOR
        .checked_mul(usable)
        .ok_or(VelocityPlanError::Arithmetic)?
        / target;
    let p_top_raw = p_top_raw.min(u32::from(PidGain::MAX.raw()));
    if p_start_raw > p_top_raw || p_start_raw > u32::from(PidGain::MAX.raw()) {
        return Err(VelocityPlanError::InsufficientStructuralSpan);
    }
    let p_start = PidGain::new(p_start_raw as u16)
        .map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
    let p_top = PidGain::new(p_top_raw as u16)
        .map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
    Ok((p_start, p_top, usable as u16))
}

fn build_gain_ladder(
    p_start_raw: u16,
    p_top_raw: u16,
) -> Result<Vec<PidGain, MAX_SWEEP_RUNGS>, VelocityPlanError> {
    if p_start_raw == 0 || p_start_raw > p_top_raw || p_top_raw > PidGain::MAX.raw() {
        return Err(VelocityPlanError::InsufficientStructuralSpan);
    }
    let start = u64::from(p_start_raw);
    let top = u64::from(p_top_raw);
    let odd_numerator = start
        .checked_mul(u64::from(HALF_OCTAVE_NUM))
        .ok_or(VelocityPlanError::Arithmetic)?;
    let odd_rounding = (1_u64 << HALF_OCTAVE_DEN_SHIFT) - 1;
    let mut gains = Vec::new();
    let mut previous = 0_u64;

    for octave in 0..16 {
        let Some(even_candidate) = start.checked_shl(octave) else {
            continue;
        };
        let Some(odd_candidate) = odd_numerator
            .checked_shl(octave)
            .and_then(|value| value.checked_add(odd_rounding))
            .map(|value| value >> HALF_OCTAVE_DEN_SHIFT)
        else {
            continue;
        };
        for candidate in [even_candidate, odd_candidate] {
            if candidate > top || candidate <= previous {
                continue;
            }
            let gain = PidGain::new(candidate as u16)
                .map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
            gains
                .push(gain)
                .map_err(|_| VelocityPlanError::ConfidenceFamilyTooLarge)?;
            previous = candidate;
        }
    }
    if previous != top {
        let top_gain =
            PidGain::new(p_top_raw).map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
        gains
            .push(top_gain)
            .map_err(|_| VelocityPlanError::ConfidenceFamilyTooLarge)?;
    }
    if gains.len() < MIN_STRUCTURAL_RUNGS {
        return Err(VelocityPlanError::InsufficientStructuralSpan);
    }
    if gains.len() * usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG)
        > usize::from(crate::autotune::velocity_measurement::MAX_CONFIDENCE_FAMILY_SIZE)
    {
        return Err(VelocityPlanError::ConfidenceFamilyTooLarge);
    }
    Ok(gains)
}

/// Regenerate one anchored post-dedupe ladder gain by index.
pub fn gain_at(
    p_start_raw: u16,
    p_top_raw: u16,
    rung_count: u8,
    rung_index: u8,
) -> Option<PidGain> {
    if rung_index >= rung_count
        || p_start_raw == 0
        || p_start_raw > p_top_raw
        || p_top_raw > PidGain::MAX.raw()
    {
        return None;
    }
    let start = u64::from(p_start_raw);
    let top = u64::from(p_top_raw);
    let odd_numerator = start.checked_mul(u64::from(HALF_OCTAVE_NUM))?;
    let odd_rounding = (1_u64 << HALF_OCTAVE_DEN_SHIFT) - 1;
    let mut previous = 0_u64;
    let mut emitted = 0_u8;
    for octave in 0..16 {
        let Some(even_candidate) = start.checked_shl(octave) else {
            continue;
        };
        let Some(odd_candidate) = odd_numerator
            .checked_shl(octave)
            .and_then(|value| value.checked_add(odd_rounding))
            .map(|value| value >> HALF_OCTAVE_DEN_SHIFT)
        else {
            continue;
        };
        for candidate in [even_candidate, odd_candidate] {
            if candidate > top || candidate <= previous {
                continue;
            }
            if emitted == rung_index {
                return PidGain::new(candidate as u16).ok();
            }
            previous = candidate;
            emitted = emitted.checked_add(1)?;
        }
    }
    if previous != top && emitted == rung_index {
        PidGain::new(p_top_raw).ok()
    } else {
        None
    }
}

#[cfg(test)]
#[path = "sweep_plan_tests.rs"]
mod tests;
