//! Immutable Stage-C velocity-integral acquisition planning.

use heapless::Vec;

use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};

use super::current_guard::VelocityLimitEvidenceCapability;
use super::recovery::{RungOriginRecoveryPlanError, plan_rung_origin_recovery};
use super::reproduction::{STAGE_B_BUILD_REVISION, StageBDirectionalValidity};
use super::sweep_plan::{
    MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US, VelocitySweepExecutionPlan,
    VelocitySweepRequest, ZERO_SETTLE_US,
};

/// Evidence schema stored in the RAM-only Stage-C candidate slot.
pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u16 = 6;
/// Largest predeclared positive-I ladder accepted by the family-128 table.
pub const STAGE_C_MAX_POSITIVE_RUNGS: usize = 30;
/// Minimum representable positive Q4.12 velocity-I gain.
pub const STAGE_C_MIN_POSITIVE_I: u16 = 1;
/// Independent observations collected in one direction at every Stage-C rung.
pub const STAGE_C_OBSERVATIONS_PER_DIRECTION: u16 = 4;
/// Forward and reverse observations collected at every Stage-C rung.
pub const STAGE_C_OBSERVATIONS_PER_RUNG: u32 = 8;

const MICROS_PER_SECOND: u64 = 1_000_000;
const CONFIDENCE_FAMILY_MAX: u16 = 128;

/// Command-level acquisition shape disclosed before any motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum StageCPlanShape {
    /// Run one Stage-B candidate without a possible continuation.
    StageB = 0,
    /// Run Stage-B reproduction and continue into Stage C on success.
    StageBThenStageC = 1,
    /// Resume Stage C from retained firmware authority.
    StageCResume = 2,
}

/// Conservative command-level plan used only for disclosure and host timeout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CommissioningWorkflowPlan {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Motion shape selected from retained firmware state.
    pub shape: StageCPlanShape,
    /// Expected duration under the selected fixed schedule.
    pub nominal_workflow_ms: u32,
    /// Conservative host-timeout authority.
    pub maximum_workflow_ms: u32,
}

impl CommissioningWorkflowPlan {
    /// Construct one checked command-level workflow envelope.
    pub fn new(
        run_sequence: u32,
        shape: StageCPlanShape,
        stage_b_ms: u32,
        stage_c_ms: u32,
    ) -> Result<Self, VelocityIntegralPlanError> {
        let valid = match shape {
            StageCPlanShape::StageB => stage_b_ms != 0 && stage_c_ms == 0,
            StageCPlanShape::StageBThenStageC => stage_b_ms != 0 && stage_c_ms != 0,
            StageCPlanShape::StageCResume => stage_b_ms == 0 && stage_c_ms != 0,
        };
        if !valid {
            return Err(VelocityIntegralPlanError::InvalidRequest);
        }
        let maximum_workflow_ms = stage_b_ms
            .checked_add(stage_c_ms)
            .ok_or(VelocityIntegralPlanError::Arithmetic)?;
        Ok(Self {
            run_sequence,
            shape,
            nominal_workflow_ms: maximum_workflow_ms,
            maximum_workflow_ms,
        })
    }

    /// Hash the command-level envelope independently of either stage plan.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        for byte in self.run_sequence.to_le_bytes() {
            hash = fnv1a64_byte(hash, byte);
        }
        hash = fnv1a64_byte(hash, self.shape as u8);
        for byte in self.nominal_workflow_ms.to_le_bytes() {
            hash = fnv1a64_byte(hash, byte);
        }
        for byte in self.maximum_workflow_ms.to_le_bytes() {
            hash = fnv1a64_byte(hash, byte);
        }
        hash
    }
}

/// Reproduced Stage-B evidence that authorizes one fixed-P Stage-C plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCAuthority {
    /// Physical firmware channel.
    pub channel: u8,
    /// Exact Stage-B stage-plan digest.
    pub stage_b_plan_digest: u64,
    /// Firmware-authoritative reproduced proportional gain.
    pub final_p: u16,
    /// Cross-run forward and reverse model-or-fragment intersections.
    pub directional_memberships: [u32; 2],
    /// Selected reproduced joint-operability component.
    pub joint_membership: u32,
    /// Cross-run validity of the forward and reverse Stage-B models.
    pub directional_validity: [StageBDirectionalValidity; 2],
    /// Whether the selected joint component contains fewer than three rungs.
    pub reduced_margin: bool,
    /// Reproduced pooled intervals, or `[0, 0]` where model comparison is unavailable.
    pub directional_pooled_intervals: [[i32; 2]; 2],
}

impl StageCAuthority {
    /// Whether one direction carries a reproduced cross-P model interval.
    pub const fn model_interval_available(&self, direction: usize) -> bool {
        if direction >= self.directional_validity.len() {
            return false;
        }
        matches!(
            self.directional_validity[direction],
            StageBDirectionalValidity::ReproducedRegionCoversFinalP
                | StageBDirectionalValidity::ReproducedRegionTransferred
        )
    }
}

/// Exact positive-I acquisition geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VelocityIntegralLadder {
    /// Minimum representable positive raw velocity-I gain.
    pub i_start: u16,
    /// Largest I whose time constant spans half the shortest suffix.
    pub i_top_cap: u16,
    /// Fixed low-I prefix followed by whole-octave rungs no greater than `i_top_cap`.
    pub rungs: Vec<u16, STAGE_C_MAX_POSITIVE_RUNGS>,
}

/// Complete immutable Stage-C plan emitted before its first stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralPlan {
    /// Shared-firmware build identity.
    pub build_revision: u32,
    /// Stage-C evidence schema.
    pub schema_revision: u16,
    /// Reproduced Stage-B authority.
    pub authority: StageCAuthority,
    /// Fixed proportional gain.
    pub final_p: u16,
    /// Frozen encoder-domain amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen signed-target magnitude in mechanical RPM register units.
    pub target_velocity_rpm: i32,
    /// Frozen PWM frequency that defines every I time constant.
    pub pwm_hz: u32,
    /// Mechanical encoder quadrature counts per revolution used by rate inversion.
    pub encoder_counts_per_rev: u32,
    /// Whole-octave positive-I rungs; unused entries are zero.
    pub positive_i: [u16; STAGE_C_MAX_POSITIVE_RUNGS],
    /// Number of valid positive-I entries.
    pub positive_rung_count: u8,
    /// Family-wise observation count per direction.
    pub family_size: u16,
    /// Opening anchor, positive rungs, and closing bookend.
    pub total_rung_count: u8,
    /// Total forward and reverse observations.
    pub expected_observations: u16,
    /// Fixed moving-capture duration.
    pub moving_stroke_us: u32,
    /// Fixed zero-target settle duration.
    pub zero_settle_us: u32,
    /// Cooperative analysis allowance per observation.
    pub analysis_budget_us: u32,
    /// Conservative fixed-schedule duration.
    pub nominal_workflow_ms: u32,
    /// Maximum duration, equal to nominal for a nonextending schedule.
    pub maximum_workflow_ms: u32,
    /// Inclusive reserve-derived origin band used after fully acquired rungs.
    pub recovery_origin_band_counts: u32,
    /// Conservative nominal duration reserved for one recovery slot.
    pub recovery_nominal_slot_us: u32,
    /// Host-declared maximum duration reserved for one recovery slot.
    pub recovery_maximum_slot_us: u32,
    /// Maximum number of recovery slots reserved by this plan.
    pub recovery_slot_count: u8,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    /// One opposite-sign meter quantum can contact the probe clamp during recovery.
    pub recovery_quantization_exposed: bool,
    /// Complete motion/travel request retained for plan identity.
    pub motion_request: VelocitySweepRequest,
}

impl VelocityIntegralPlan {
    /// Return only the emitted positive-I rungs.
    pub fn positive_rungs(&self) -> &[u16] {
        &self.positive_i[..usize::from(self.positive_rung_count)]
    }

    /// Hash every exact stage-local authority, geometry, and safety field.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        macro_rules! hash_value {
            ($value:expr) => {
                for byte in $value.to_le_bytes() {
                    hash = fnv1a64_byte(hash, byte);
                }
            };
        }
        hash_value!(self.build_revision);
        hash_value!(self.schema_revision);
        hash_value!(self.authority.channel);
        hash_value!(self.authority.stage_b_plan_digest);
        hash_value!(self.final_p);
        for membership in self.authority.directional_memberships {
            hash_value!(membership);
        }
        hash_value!(self.authority.joint_membership);
        for validity in self.authority.directional_validity {
            hash_value!(validity as u8);
        }
        hash_value!(u8::from(self.authority.reduced_margin));
        for interval in self.authority.directional_pooled_intervals {
            hash_value!(interval[0]);
            hash_value!(interval[1]);
        }
        hash_value!(self.planned_velocity_mrev_s);
        hash_value!(self.target_velocity_rpm);
        hash_value!(self.pwm_hz);
        hash_value!(self.encoder_counts_per_rev);
        hash_value!(self.positive_rung_count);
        for gain in self.positive_rungs() {
            hash_value!(*gain);
        }
        hash_value!(self.family_size);
        hash_value!(self.total_rung_count);
        hash_value!(self.expected_observations);
        hash_value!(self.moving_stroke_us);
        hash_value!(self.zero_settle_us);
        hash_value!(self.analysis_budget_us);
        hash_value!(self.nominal_workflow_ms);
        hash_value!(self.maximum_workflow_ms);
        hash_value!(self.recovery_origin_band_counts);
        hash_value!(self.recovery_nominal_slot_us);
        hash_value!(self.recovery_maximum_slot_us);
        hash_value!(self.recovery_slot_count);
        hash_value!(self.hard_torque_limit);
        hash_value!(self.usable_torque_limit);
        hash_value!(self.limit_evidence_capability as u8);
        hash_value!(u8::from(self.recovery_quantization_exposed));
        hash_value!(self.motion_request.requested_velocity_mrev_s);
        hash_value!(self.motion_request.machine_velocity_ceiling_mrev_s);
        hash_value!(self.motion_request.requested_velocity_source as u8);
        hash_value!(self.motion_request.max_stroke_travel_mrev);
        hash_value!(self.motion_request.settle_travel_reserve_mrev);
        hash_value!(self.motion_request.negative_position_headroom_mrev);
        hash_value!(self.motion_request.positive_position_headroom_mrev);
        hash_value!(self.motion_request.max_duration_ms);
        hash
    }

    /// Compact the report plan into phase-local execution state.
    pub fn execution_plan(self) -> VelocityIntegralExecutionPlan {
        VelocityIntegralExecutionPlan {
            plan_digest: self.digest(),
            final_p: self.final_p,
            planned_velocity_mrev_s: self.planned_velocity_mrev_s,
            target_velocity_rpm: self.target_velocity_rpm,
            pwm_hz: self.pwm_hz,
            encoder_counts_per_rev: self.encoder_counts_per_rev,
            i_start: self.positive_i[0],
            positive_rung_count: self.positive_rung_count,
            family_size: self.family_size,
            moving_stroke_us: self.moving_stroke_us,
            zero_settle_us: self.zero_settle_us,
            analysis_budget_us: self.analysis_budget_us,
            maximum_workflow_ms: self.maximum_workflow_ms,
            recovery_origin_band_counts: self.recovery_origin_band_counts,
            recovery_nominal_slot_us: self.recovery_nominal_slot_us,
            recovery_maximum_slot_us: self.recovery_maximum_slot_us,
            recovery_slot_count: self.recovery_slot_count,
            hard_torque_limit: self.hard_torque_limit,
            usable_torque_limit: self.usable_torque_limit,
            limit_evidence_capability: self.limit_evidence_capability,
            recovery_quantization_exposed: self.recovery_quantization_exposed,
        }
    }
}

/// Compact immutable Stage-C state retained after plan emission.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralExecutionPlan {
    /// Exact complete-plan digest.
    pub plan_digest: u64,
    /// Fixed proportional gain.
    pub final_p: u16,
    /// Frozen encoder-domain amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen signed-target magnitude.
    pub target_velocity_rpm: i32,
    /// Frozen PWM frequency retained in candidate identity.
    pub pwm_hz: u32,
    /// Mechanical encoder quadrature counts per revolution.
    pub encoder_counts_per_rev: u32,
    /// First positive-I rung; the full fixed schedule comes from the canonical accessor.
    pub i_start: u16,
    /// Number of valid positive-I entries.
    pub positive_rung_count: u8,
    /// Family-wise observation count per direction.
    pub family_size: u16,
    /// Fixed moving-capture duration.
    pub moving_stroke_us: u32,
    /// Fixed zero-target settle duration.
    pub zero_settle_us: u32,
    /// Cooperative analysis allowance per observation.
    pub analysis_budget_us: u32,
    /// Conservative complete Stage-C duration retained in candidate identity.
    pub maximum_workflow_ms: u32,
    /// Inclusive immutable-origin recovery band.
    pub recovery_origin_band_counts: u32,
    /// Conservative nominal duration reserved for one recovery slot.
    pub recovery_nominal_slot_us: u32,
    /// Host-declared maximum duration reserved for one recovery slot.
    pub recovery_maximum_slot_us: u32,
    /// Maximum recovery-record count reserved by the plan.
    pub recovery_slot_count: u8,
    /// Hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    /// One opposite-sign meter quantum can contact the probe clamp during recovery.
    pub recovery_quantization_exposed: bool,
}

const _: () = assert!(core::mem::size_of::<VelocityIntegralExecutionPlan>() == 72);

/// Failure to construct a deterministic Stage-C plan before motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityIntegralPlanError {
    /// A required duration, scale, or workflow component was invalid.
    InvalidRequest,
    /// Stage-B Complete authority was absent.
    MissingAuthority,
    /// The compact Stage-B motion plan was absent.
    MissingStageBPlan,
    /// Stage-B authority does not belong to the supplied stage plan.
    MismatchedAuthority,
    /// Fewer than three positive-I rungs fit the acquisition geometry.
    InsufficientStructuralSpan,
    /// The simultaneous observation family exceeds 128.
    ConfidenceFamilyTooLarge,
    /// Checked planner arithmetic exceeded its representation.
    Arithmetic,
}

/// Evaluate the advanced-PI time constant with outward ceiling.
pub const fn velocity_i_tau_us(i_raw: u16, pwm_hz: u32) -> Option<u64> {
    if i_raw == 0 || pwm_hz == 0 {
        return None;
    }
    let numerator =
        match crate::autotune::ADVANCED_PI_I_Q4_12_NUMERATOR.checked_mul(MICROS_PER_SECOND) {
            Some(value) => value,
            None => return None,
        };
    let denominator = match (i_raw as u64).checked_mul(pwm_hz as u64) {
        Some(value) if value != 0 => value,
        _ => return None,
    };
    Some(numerator.div_ceil(denominator))
}

/// Return one positive rung from the fixed native-Q4.12 schedule.
pub(crate) const fn native_velocity_i_at(index: u8) -> Option<u16> {
    match index {
        0 => Some(1),
        1 => Some(2),
        2 => Some(3),
        value => 1_u16.checked_shl((value - 1) as u32),
    }
}

/// Build the fixed-prefix and whole-octave I geometry from declared acquisition times.
pub fn build_integral_ladder(
    pwm_hz: u32,
    moving_stroke_us: u32,
    shortest_suffix_us: u32,
    maximum_i: u16,
) -> Result<VelocityIntegralLadder, VelocityIntegralPlanError> {
    if pwm_hz == 0 || moving_stroke_us == 0 || shortest_suffix_us < 2 || maximum_i == 0 {
        return Err(VelocityIntegralPlanError::InvalidRequest);
    }
    let i_start = STAGE_C_MIN_POSITIVE_I;
    if maximum_i < i_start {
        return Err(VelocityIntegralPlanError::InsufficientStructuralSpan);
    }
    let i_top_cap = largest_i_for_tau(pwm_hz, shortest_suffix_us / 2, maximum_i)?;
    if i_start > i_top_cap {
        return Err(VelocityIntegralPlanError::InsufficientStructuralSpan);
    }
    let mut rungs = Vec::new();
    let mut index = 0_u8;
    while let Some(candidate) = native_velocity_i_at(index) {
        if candidate > i_top_cap {
            break;
        }
        rungs
            .push(candidate)
            .map_err(|_| VelocityIntegralPlanError::ConfidenceFamilyTooLarge)?;
        index = index
            .checked_add(1)
            .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    }
    if rungs.len() < 3 {
        return Err(VelocityIntegralPlanError::InsufficientStructuralSpan);
    }
    confidence_family_size(rungs.len())?;
    Ok(VelocityIntegralLadder {
        i_start,
        i_top_cap,
        rungs,
    })
}

/// Return the per-direction simultaneous family for positive rungs plus anchors.
pub fn confidence_family_size(positive_rungs: usize) -> Result<u16, VelocityIntegralPlanError> {
    let rungs = positive_rungs
        .checked_add(2)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let family = rungs
        .checked_mul(usize::from(STAGE_C_OBSERVATIONS_PER_DIRECTION))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    if positive_rungs > STAGE_C_MAX_POSITIVE_RUNGS || family > usize::from(CONFIDENCE_FAMILY_MAX) {
        return Err(VelocityIntegralPlanError::ConfidenceFamilyTooLarge);
    }
    u16::try_from(family).map_err(|_| VelocityIntegralPlanError::Arithmetic)
}

/// Report whether one opposite-sign meter quantum can contact the probe clamp.
pub(crate) fn recovery_quantization_exposed(
    p_raw: u16,
    target_rpm: i32,
    counts_per_rev: u32,
    pwm_hz: u32,
    probe_limit: u16,
) -> Result<bool, VelocityIntegralPlanError> {
    let target_rpm = target_rpm
        .checked_abs()
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let velocity_term = u128::from(target_rpm as u32)
        .checked_mul(u128::from(counts_per_rev))
        .and_then(|value| value.checked_add(60_u128 * u128::from(pwm_hz)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let demand = u128::from(p_raw)
        .checked_mul(velocity_term)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let clamp = 256_u128
        .checked_mul(u128::from(probe_limit))
        .and_then(|value| value.checked_mul(u128::from(counts_per_rev)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    Ok(demand > clamp)
}

/// Derive the conservative fixed Stage-C duration for one frozen PWM rate.
pub fn integral_workflow_maximum_ms(
    pwm_hz: u32,
    maximum_recovery_slot_ms: u16,
) -> Result<u32, VelocityIntegralPlanError> {
    if maximum_recovery_slot_ms == 0 {
        return Err(VelocityIntegralPlanError::InvalidRequest);
    }
    let ladder = build_integral_ladder(
        pwm_hz,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )?;
    let total_rungs = ladder
        .rungs
        .len()
        .checked_add(2)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let observations = total_rungs
        .checked_mul(STAGE_C_OBSERVATIONS_PER_RUNG as usize)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let primitive_us = MOVING_STROKE_US
        .checked_add(ZERO_SETTLE_US)
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let workflow_us = u32::try_from(observations)
        .map_err(|_| VelocityIntegralPlanError::Arithmetic)?
        .checked_mul(primitive_us)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let recovery_us = u32::try_from(total_rungs)
        .map_err(|_| VelocityIntegralPlanError::Arithmetic)?
        .checked_mul(u32::from(maximum_recovery_slot_ms))
        .and_then(|value| value.checked_mul(1_000))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    workflow_us
        .checked_add(recovery_us)
        .and_then(|value| value.checked_add(ZERO_SETTLE_US))
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .map(|value| value.div_ceil(1_000))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Construct one exact Stage-C stage plan from reproduced Stage-B authority.
#[allow(dead_code)]
pub(crate) fn plan_velocity_integral_response(
    authority: StageCAuthority,
    stage_b_plan: VelocitySweepExecutionPlan,
    motion_budget: OuterMotionBudget,
    pwm_hz: u32,
    encoder_counts_per_rev: u32,
) -> Result<VelocityIntegralPlan, VelocityIntegralPlanError> {
    if authority.final_p == 0 {
        return Err(VelocityIntegralPlanError::MissingAuthority);
    }
    if !stage_b_plan.is_present() {
        return Err(VelocityIntegralPlanError::MissingStageBPlan);
    }
    if authority.stage_b_plan_digest != stage_b_plan.plan_digest {
        return Err(VelocityIntegralPlanError::MismatchedAuthority);
    }
    let motion_request = VelocitySweepRequest {
        requested_velocity_mrev_s: motion_budget.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: motion_budget.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: motion_budget.requested_velocity_source,
        max_stroke_travel_mrev: motion_budget.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: motion_budget.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: motion_budget.negative_position_headroom_mrev,
        positive_position_headroom_mrev: motion_budget.positive_position_headroom_mrev,
        max_duration_ms: motion_budget.max_duration_ms,
    };
    if authority.joint_membership == 0
        || authority
            .directional_pooled_intervals
            .iter()
            .enumerate()
            .any(|(direction, interval)| {
                let membership = authority.directional_memberships[direction];
                match authority.directional_validity[direction] {
                    StageBDirectionalValidity::NoRegion => membership != 0 || *interval != [0, 0],
                    StageBDirectionalValidity::Fragment => {
                        membership.count_ones() > 2 || *interval != [0, 0]
                    }
                    StageBDirectionalValidity::RegionNotReproduced => *interval != [0, 0],
                    StageBDirectionalValidity::ReproducedRegionCoversFinalP
                    | StageBDirectionalValidity::ReproducedRegionTransferred => {
                        membership.count_ones() < 3 || interval[0] > interval[1]
                    }
                }
            })
        || encoder_counts_per_rev == 0
        || motion_request.requested_velocity_mrev_s == 0
        || motion_request.machine_velocity_ceiling_mrev_s == 0
        || motion_request.max_stroke_travel_mrev == 0
        || motion_request.settle_travel_reserve_mrev >= motion_request.max_stroke_travel_mrev
        || motion_request.negative_position_headroom_mrev == 0
        || motion_request.positive_position_headroom_mrev == 0
        || motion_request.max_duration_ms == 0
    {
        return Err(VelocityIntegralPlanError::InvalidRequest);
    }
    let ladder = build_integral_ladder(
        pwm_hz,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )?;
    let positive_rung_count =
        u8::try_from(ladder.rungs.len()).map_err(|_| VelocityIntegralPlanError::Arithmetic)?;
    let total_rung_count = positive_rung_count
        .checked_add(2)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let expected_observations = u16::from(total_rung_count)
        .checked_mul(STAGE_C_OBSERVATIONS_PER_RUNG as u16)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let recovery = plan_rung_origin_recovery(
        motion_budget,
        encoder_counts_per_rev,
        stage_b_plan.planned_velocity_mrev_s,
    )
    .map_err(map_recovery_plan_error)?;
    let primitive_us = MOVING_STROKE_US
        .checked_add(ZERO_SETTLE_US)
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    if primitive_us.div_ceil(1_000) > u32::from(motion_budget.max_duration_ms) {
        return Err(VelocityIntegralPlanError::InvalidRequest);
    }
    let workflow_us = u32::from(expected_observations)
        .checked_mul(primitive_us)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let nominal_workflow_us = workflow_us
        .checked_add(
            u32::from(total_rung_count)
                .checked_mul(recovery.nominal_slot_us)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .and_then(|value| value.checked_add(ZERO_SETTLE_US))
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let maximum_workflow_us = workflow_us
        .checked_add(
            u32::from(total_rung_count)
                .checked_mul(recovery.maximum_slot_us)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .and_then(|value| value.checked_add(ZERO_SETTLE_US))
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let mut positive_i = [0; STAGE_C_MAX_POSITIVE_RUNGS];
    positive_i[..ladder.rungs.len()].copy_from_slice(ladder.rungs.as_slice());
    let recovery_quantization_exposed = recovery_quantization_exposed(
        authority.final_p,
        stage_b_plan.target_velocity_rpm.raw(),
        encoder_counts_per_rev,
        pwm_hz,
        stage_b_plan.usable_torque_limit,
    )?;

    Ok(VelocityIntegralPlan {
        build_revision: STAGE_B_BUILD_REVISION,
        schema_revision: STAGE_C_EVIDENCE_SCHEMA_REVISION,
        authority,
        final_p: authority.final_p,
        planned_velocity_mrev_s: stage_b_plan.planned_velocity_mrev_s,
        target_velocity_rpm: stage_b_plan.target_velocity_rpm.raw(),
        pwm_hz,
        encoder_counts_per_rev,
        positive_i,
        positive_rung_count,
        family_size: confidence_family_size(ladder.rungs.len())?,
        total_rung_count,
        expected_observations,
        moving_stroke_us: MOVING_STROKE_US,
        zero_settle_us: ZERO_SETTLE_US,
        analysis_budget_us: SETTLE_ANALYSIS_BUDGET_US,
        nominal_workflow_ms: nominal_workflow_us.div_ceil(1_000),
        maximum_workflow_ms: maximum_workflow_us.div_ceil(1_000),
        recovery_origin_band_counts: recovery.origin_band_counts,
        recovery_nominal_slot_us: recovery.nominal_slot_us,
        recovery_maximum_slot_us: recovery.maximum_slot_us,
        recovery_slot_count: total_rung_count,
        hard_torque_limit: stage_b_plan.hard_torque_limit,
        usable_torque_limit: stage_b_plan.usable_torque_limit,
        limit_evidence_capability: stage_b_plan.limit_evidence_capability,
        recovery_quantization_exposed,
        motion_request,
    })
}

fn map_recovery_plan_error(error: RungOriginRecoveryPlanError) -> VelocityIntegralPlanError {
    match error {
        RungOriginRecoveryPlanError::InvalidRequest
        | RungOriginRecoveryPlanError::ExceedsPrimitiveCap => {
            VelocityIntegralPlanError::InvalidRequest
        }
        RungOriginRecoveryPlanError::Arithmetic => VelocityIntegralPlanError::Arithmetic,
    }
}

fn largest_i_for_tau(
    pwm_hz: u32,
    minimum_tau_us: u32,
    maximum_i: u16,
) -> Result<u16, VelocityIntegralPlanError> {
    let mut largest = 0;
    let mut candidate = 1;
    while candidate <= maximum_i {
        let tau =
            velocity_i_tau_us(candidate, pwm_hz).ok_or(VelocityIntegralPlanError::Arithmetic)?;
        if tau >= u64::from(minimum_tau_us) {
            largest = candidate;
        } else {
            break;
        }
        candidate = candidate
            .checked_add(1)
            .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    }
    if largest == 0 {
        Err(VelocityIntegralPlanError::InsufficientStructuralSpan)
    } else {
        Ok(largest)
    }
}

const fn fnv1a64_byte(hash: u64, byte: u8) -> u64 {
    (hash ^ byte as u64).wrapping_mul(0x0000_0100_0000_01b3)
}
