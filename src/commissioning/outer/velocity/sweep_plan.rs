use foci_tmc4671::pid::PidGain;
use heapless::Vec;

/// Numerator of the runtime velocity-envelope proportional margin.
pub const ENVELOPE_PROPORTIONAL_NUM: u32 = 3;
/// Denominator of the runtime velocity-envelope proportional margin.
pub const ENVELOPE_PROPORTIONAL_DEN: u32 = 2;
/// Absolute runtime velocity-envelope margin in mechanical millirevolutions per second.
pub const ENVELOPE_ABSOLUTE_MARGIN_MREV_S: u32 = 2_000;
/// Fixed duration of one moving observation.
pub const MOVING_STROKE_US: u32 = 256_000;
/// Fixed duration of one zero-target settle.
pub const ZERO_SETTLE_US: u32 = 200_000;
/// Sampling period used by moving and zero-target velocity observations.
pub const VELOCITY_SWEEP_SAMPLE_PERIOD_US: u32 = 1_000;
/// Minimum zero-target capture retained for the shortest stationarity suffix.
pub const MIN_SETTLE_CAPTURE_US: u32 =
    crate::autotune::velocity_measurement::VELOCITY_SUFFIX_LENGTHS[0] as u32
        * VELOCITY_SWEEP_SAMPLE_PERIOD_US;
/// Maximum cooperative analysis budget that preserves the minimum rest suffix.
pub const SETTLE_ANALYSIS_BUDGET_US: u32 = ZERO_SETTLE_US - MIN_SETTLE_CAPTURE_US;
/// Minimum number of distinct gains that can support the required evidence geometry.
pub const MIN_STRUCTURAL_RUNGS: usize = 5;
/// Maximum audited gain count in one deterministic sweep.
pub const MAX_SWEEP_RUNGS: usize = 32;
/// Independent observations collected in each direction at every rung.
pub const OBSERVATIONS_PER_DIRECTION_PER_RUNG: u16 = 2;
/// Numerator of the usable current-headroom boundary.
pub const CURRENT_HEADROOM_NUM: u32 = 9;
/// Denominator of the usable current-headroom boundary.
pub const CURRENT_HEADROOM_DEN: u32 = 10;

const TMC_VELOCITY_SCALE: u64 = 65_536;
const MREV_PER_REV: u64 = 1_000;
const MICROS_PER_SECOND: u64 = 1_000_000;
/// Maximum actual time between encoder reads before wrap-safe interpretation is refused.
pub const MAX_SAFETY_OBSERVATION_GAP_US: u64 = 2_000;
const VELOCITY_P_DIVISOR: u32 = 256;
const HALF_OCTAVE_NUM: u32 = 181;
const HALF_OCTAVE_DEN_SHIFT: u32 = 7;

/// The effective ceiling is the host-declared machine ceiling.
pub const EFFECTIVE_CEILING_MACHINE: u8 = 0;
/// The effective ceiling is limited by wrap-safe encoder observability.
pub const EFFECTIVE_CEILING_ENCODER: u8 = 1;
/// The effective ceiling is limited by the TMC target register.
pub const EFFECTIVE_CEILING_TMC_TARGET: u8 = 2;
/// The effective ceiling is limited by the TMC feedback register.
pub const EFFECTIVE_CEILING_TMC_FEEDBACK: u8 = 3;
/// The effective ceiling is limited by fixed-point intermediates.
pub const EFFECTIVE_CEILING_FIXED_POINT: u8 = 4;

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
    pub used_velocity_mrev_s: u32,
    /// Origin of the requested amplitude.
    pub requested_velocity_source: RequestedVelocitySource,
    /// Bitset describing every amplitude reduction.
    pub clamp_flags: u16,
    /// Lowest machine, observability, or representation ceiling.
    pub effective_ceiling_mrev_s: u32,
    /// Source code for the binding effective ceiling.
    pub effective_ceiling_source: u8,
    /// Frozen positive TMC velocity-target register magnitude.
    pub target_velocity_register: i32,
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
    tmc_target_mrev_s: u32,
    tmc_feedback_mrev_s: u32,
    fixed_point_mrev_s: u32,
}

/// Construct one immutable velocity sweep from host geometry and firmware limits.
pub fn plan_velocity_sweep(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    f_pwm_hz: u32,
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

    let ceilings = representability_ceilings(encoder_counts_per_rev, f_pwm_hz)?;
    let candidates = [
        (
            request.machine_velocity_ceiling_mrev_s,
            EFFECTIVE_CEILING_MACHINE,
        ),
        (
            ceilings.encoder_observable_mrev_s,
            EFFECTIVE_CEILING_ENCODER,
        ),
        (ceilings.tmc_target_mrev_s, EFFECTIVE_CEILING_TMC_TARGET),
        (ceilings.tmc_feedback_mrev_s, EFFECTIVE_CEILING_TMC_FEEDBACK),
        (ceilings.fixed_point_mrev_s, EFFECTIVE_CEILING_FIXED_POINT),
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
    let used_velocity_mrev_s = request
        .requested_velocity_mrev_s
        .min(envelope_velocity_cap)
        .min(travel_velocity_cap);
    if used_velocity_mrev_s == 0 {
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

    let target_velocity_register = velocity_target_register(used_velocity_mrev_s, f_pwm_hz)?;
    let (p_start, p_top, usable_torque_limit) =
        current_gain_span(target_velocity_register, torque_flux_limit)?;
    let ladder = build_gain_ladder(p_start.raw(), p_top.raw())?;
    let rung_count = u8::try_from(ladder.len()).map_err(|_| VelocityPlanError::Arithmetic)?;
    let family_size = usize::from(rung_count)
        .checked_mul(usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG))
        .ok_or(VelocityPlanError::Arithmetic)?;
    if family_size > 64 {
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
        .checked_mul(4)
        .and_then(|value| value.checked_mul(primitive_us))
        .ok_or(VelocityPlanError::Arithmetic)?;
    let workflow_ms = workflow_us / 1_000;
    let mut gains = [PidGain::ZERO; MAX_SWEEP_RUNGS];
    gains[..ladder.len()].copy_from_slice(ladder.as_slice());

    Ok(VelocitySweepPlan {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        used_velocity_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        clamp_flags,
        effective_ceiling_mrev_s,
        effective_ceiling_source,
        target_velocity_register,
        p_start,
        p_top,
        gains,
        rung_count,
        observations_per_direction: OBSERVATIONS_PER_DIRECTION_PER_RUNG,
        moving_stroke_us: MOVING_STROKE_US,
        zero_settle_us: ZERO_SETTLE_US,
        nominal_workflow_ms: workflow_ms,
        maximum_workflow_ms: workflow_ms,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        hard_torque_limit: torque_flux_limit,
        usable_torque_limit,
    })
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
    f_pwm_hz: u32,
) -> Result<RepresentabilityCeilings, VelocityPlanError> {
    if encoder_counts_per_rev == 0 || f_pwm_hz == 0 {
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
    let register_mrev_s = |register_limit: u64| -> Result<u32, VelocityPlanError> {
        let value = u128::from(register_limit)
            .checked_mul(u128::from(f_pwm_hz))
            .and_then(|product| product.checked_mul(u128::from(MREV_PER_REV)))
            .ok_or(VelocityPlanError::Arithmetic)?
            / u128::from(TMC_VELOCITY_SCALE);
        Ok(value.min(u128::from(u32::MAX)) as u32)
    };
    let signed_register_limit = i32::MAX as u64;
    let fixed_point_register_limit = (i64::MAX as u64) / TMC_VELOCITY_SCALE;

    Ok(RepresentabilityCeilings {
        encoder_observable_mrev_s: u32::try_from(encoder_observable_mrev_s)
            .map_err(|_| VelocityPlanError::Arithmetic)?,
        tmc_target_mrev_s: register_mrev_s(signed_register_limit)?,
        tmc_feedback_mrev_s: register_mrev_s(signed_register_limit)?,
        fixed_point_mrev_s: register_mrev_s(fixed_point_register_limit)?,
    })
}

fn velocity_target_register(velocity_mrev_s: u32, f_pwm_hz: u32) -> Result<i32, VelocityPlanError> {
    if velocity_mrev_s == 0 || f_pwm_hz == 0 {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let numerator = u64::from(velocity_mrev_s)
        .checked_mul(TMC_VELOCITY_SCALE)
        .ok_or(VelocityPlanError::Arithmetic)?;
    let denominator = u64::from(f_pwm_hz)
        .checked_mul(MREV_PER_REV)
        .ok_or(VelocityPlanError::Arithmetic)?;
    let rounded = numerator
        .checked_add(denominator / 2)
        .ok_or(VelocityPlanError::Arithmetic)?
        / denominator;
    if rounded == 0 {
        return Err(VelocityPlanError::ZeroMovingTravelAmplitude);
    }
    i32::try_from(rounded).map_err(|_| VelocityPlanError::Arithmetic)
}

fn current_gain_span(
    target_velocity_register: i32,
    hard_torque_limit: u16,
) -> Result<(PidGain, PidGain, u16), VelocityPlanError> {
    let target = target_velocity_register.unsigned_abs();
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
    let start = u32::from(p_start_raw);
    let top = u32::from(p_top_raw);
    let odd_anchor = start
        .checked_mul(HALF_OCTAVE_NUM)
        .and_then(|value| value.checked_add((1 << HALF_OCTAVE_DEN_SHIFT) - 1))
        .ok_or(VelocityPlanError::Arithmetic)?
        >> HALF_OCTAVE_DEN_SHIFT;
    let mut gains = Vec::new();
    let mut previous = 0_u32;

    for octave in 0..16 {
        for anchor in [start, odd_anchor] {
            let Some(candidate) = anchor.checked_shl(octave) else {
                continue;
            };
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
    if gains.len() * usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG) > 64 {
        return Err(VelocityPlanError::ConfidenceFamilyTooLarge);
    }
    Ok(gains)
}

#[cfg(test)]
#[path = "sweep_plan_tests.rs"]
mod tests;
