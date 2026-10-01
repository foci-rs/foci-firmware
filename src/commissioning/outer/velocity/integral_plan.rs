//! Immutable Integral velocity-integral acquisition planning.

use heapless::Vec;

use crate::commissioning::inner::closed_loop_activation::STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE;
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};

use super::band_confirmation::{
    BandConfirmationAcceptance, BandConfirmationPlan, CONFIRMATION_FAMILY_SIZE,
    TargetBandConfirmationInconclusive, validate_fixed_contract,
};
use super::breakaway_plan::PROBE_MAX_OBSERVATIONS;
use super::recovery::{RungOriginRecoveryPlanError, plan_rung_origin_recovery};
use super::reproduction::{BUILD_FEATURES, ProportionalDirectionalValidity};
use super::sweep_plan::{
    MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US,
    VelocitySweepExecutionPlan, VelocitySweepRequest, ZERO_SETTLE_US,
};
use super::velocity_clamp::VelocityLimitEvidenceCapability;

/// Evidence schema stored in the RAM-only Integral candidate slot.
pub const INTEGRAL_EVIDENCE_SCHEMA_REVISION: u16 = 17;
/// Largest predeclared positive-I ladder accepted by the family-128 table.
pub const INTEGRAL_MAX_POSITIVE_RUNGS: usize = 30;
/// Minimum representable positive Q4.12 velocity-I gain.
pub const INTEGRAL_MIN_POSITIVE_I: u16 = 1;
/// Independent observations collected in one direction at every Integral rung.
pub const INTEGRAL_OBSERVATIONS_PER_DIRECTION: u16 = 4;
/// Forward and reverse observations collected at every Integral rung.
pub const INTEGRAL_OBSERVATIONS_PER_RUNG: u32 = 8;
/// Maximum combined entry-plus-exit SPI time reserved for one moving slot.
pub const INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US: u32 = 20_000;

const MICROS_PER_SECOND: u64 = 1_000_000;
const CONFIDENCE_FAMILY_MAX: u16 = 128;
const RETIRED_COMBINED_RESPONSE_PRESENCE: u8 = 0;

/// Reproduced Proportional evidence that authorizes one fixed-P Integral plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralAcquisitionAuthority {
    /// Physical firmware channel.
    pub channel: u8,
    /// Token of the accepted breakaway confirmation that authorized this Integral plan.
    pub acquisition: ConfirmedAcquisitionDigest,
    /// Cross-run forward and reverse model-or-fragment intersections.
    pub directional_memberships: [u32; 2],
    /// Reproduced Proportional joint-operability component before Integral filtering.
    pub proportional_joint_membership: u32,
    /// Integral transition-capable subset of the reproduced joint component.
    pub joint_membership: u32,
    /// Cross-run validity of the forward and reverse Proportional models.
    pub directional_validity: [ProportionalDirectionalValidity; 2],
    /// Whether the selected joint component contains fewer than three rungs.
    pub reduced_margin: bool,
    /// Reproduced pooled intervals, or `[0, 0]` where model comparison is unavailable.
    pub directional_pooled_intervals: [[i32; 2]; 2],
    /// Held-out confirmation confidence in Q0.16, or `0` off the breakaway path.
    /// See `super::band_confirmation::BandConfirmationAcceptance::confidence_q`.
    pub confidence_q: u16,
    /// Held-out confirmation band position in Q0.16, or `0` off the breakaway
    /// path. See
    /// `super::band_confirmation::BandConfirmationAcceptance::band_position_q`.
    pub band_position_q: u16,
}

impl IntegralAcquisitionAuthority {
    /// Whether one direction carries a reproduced cross-P model interval.
    pub const fn model_interval_available(&self, direction: usize) -> bool {
        if direction >= self.directional_validity.len() {
            return false;
        }
        matches!(
            self.directional_validity[direction],
            ProportionalDirectionalValidity::ReproducedRegionCoversFinalP
                | ProportionalDirectionalValidity::ReproducedRegionTransferred
        )
    }

    pub(crate) const fn new_breakaway(
        channel: u8,
        confirmed_digest: ConfirmedAcquisitionDigest,
        held_out_response_intervals_q: [[i32; 2]; 2],
        confidence_q: u16,
        band_position_q: u16,
    ) -> Self {
        Self {
            channel,
            acquisition: confirmed_digest,
            directional_memberships: [0; 2],
            proportional_joint_membership: 0,
            joint_membership: 0,
            directional_validity: [ProportionalDirectionalValidity::NoRegion; 2],
            reduced_margin: false,
            directional_pooled_intervals: held_out_response_intervals_q,
            confidence_q,
            band_position_q,
        }
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
    pub rungs: Vec<u16, INTEGRAL_MAX_POSITIVE_RUNGS>,
}

/// Complete immutable Integral plan emitted before its first stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralPlan {
    /// Analysis-rule contract this plan was built under.
    pub build_features: u16,
    /// Integral evidence schema.
    pub schema_revision: u16,
    /// Reproduced Proportional authority.
    pub authority: IntegralAcquisitionAuthority,
    /// Fixed proportional gain.
    pub fixed_p: u16,
    /// Frozen encoder-domain amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen signed-target magnitude in mechanical RPM register units.
    pub target_velocity_rpm: i32,
    /// Frozen PWM frequency that defines every I time constant.
    pub pwm_hz: u32,
    /// Mechanical encoder quadrature counts per revolution used by rate inversion.
    pub encoder_counts_per_rev: u32,
    /// Whole-octave positive-I rungs; unused entries are zero.
    pub positive_i: [u16; INTEGRAL_MAX_POSITIVE_RUNGS],
    /// Number of valid positive-I entries.
    pub positive_rung_count: u8,
    /// Largest native positive-I ladder cap the half-octave rung was derived
    /// from, for a `Breakaway`-sourced plan; `0` when no ladder cap applies.
    pub i_top_cap: u16,
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
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    /// One opposite-sign meter quantum can contact the probe clamp during recovery.
    pub recovery_quantization_exposed: bool,
    /// The commissioning probe clamp removed at least one Proportional-operable rung.
    pub probe_constrained_test_point: bool,
    /// Complete motion/travel request retained for plan identity.
    pub motion_request: VelocitySweepRequest,
    /// Execution order of the forward and reverse observation slots.
    pub slot_order: IntegralSlotOrder,
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
        hash_value!(self.build_features);
        hash_value!(self.schema_revision);
        hash_value!(self.authority.channel);
        hash_value!(self.authority.acquisition.get());
        hash_value!(self.fixed_p);
        for membership in self.authority.directional_memberships {
            hash_value!(membership);
        }
        hash_value!(self.authority.proportional_joint_membership);
        hash_value!(self.authority.joint_membership);
        for validity in self.authority.directional_validity {
            hash_value!(validity as u8);
        }
        hash_value!(u8::from(self.authority.reduced_margin));
        for interval in self.authority.directional_pooled_intervals {
            hash_value!(interval[0]);
            hash_value!(interval[1]);
        }
        hash_value!(RETIRED_COMBINED_RESPONSE_PRESENCE);
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
        hash_value!(u8::from(self.probe_constrained_test_point));
        hash_value!(self.motion_request.requested_velocity_mrev_s);
        hash_value!(self.motion_request.machine_velocity_ceiling_mrev_s);
        hash_value!(self.motion_request.requested_velocity_source as u8);
        hash_value!(self.motion_request.max_stroke_travel_mrev);
        hash_value!(self.motion_request.settle_travel_reserve_mrev);
        hash_value!(self.motion_request.negative_position_headroom_mrev);
        hash_value!(self.motion_request.positive_position_headroom_mrev);
        hash_value!(self.motion_request.max_duration_ms);
        if !matches!(self.slot_order, IntegralSlotOrder::ForwardFirst) {
            hash_value!(self.slot_order.report_code());
        }
        if matches!(
            self.schema_revision,
            INTEGRAL_EVIDENCE_SCHEMA_REVISION
                | super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION
        ) {
            hash_value!(self.i_top_cap);
        }
        hash
    }

    /// Compact the report plan into phase-local execution state.
    pub fn execution_plan(self) -> VelocityIntegralExecutionPlan {
        VelocityIntegralExecutionPlan {
            slot_order: self.slot_order,
            plan_digest: self.digest(),
            fixed_p: self.fixed_p,
            planned_velocity_mrev_s: self.planned_velocity_mrev_s,
            target_velocity_rpm: self.target_velocity_rpm,
            pwm_hz: self.pwm_hz,
            encoder_counts_per_rev: self.encoder_counts_per_rev,
            i_start: self.positive_i[0],
            positive_rung_count: self.positive_rung_count,
            i_top_cap: self.i_top_cap,
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
            probe_constrained_test_point: self.probe_constrained_test_point,
        }
    }
}

/// Order in which the fixed forward and reverse observation slots execute, recorded explicitly
/// because travel direction and settled-tail position are confounded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum IntegralSlotOrder {
    /// Even slots travel forward.
    ForwardFirst = 0,
}

impl IntegralSlotOrder {
    /// Every declared order, so a new variant cannot be added untested.
    #[cfg(test)]
    pub const ALL: [Self; 1] = [Self::ForwardFirst];

    const fn parity_twist(self, ordinal: usize) -> usize {
        let _ = ordinal;
        match self {
            Self::ForwardFirst => 0,
        }
    }

    /// Direction index for one slot: 0 is forward, 1 is reverse.
    pub const fn direction_index(self, slot: u8) -> usize {
        (slot as usize & 1) ^ self.parity_twist(slot as usize >> 1)
    }

    /// Signed mechanical travel direction for one slot.
    pub const fn signed_direction(self, slot: u8) -> i32 {
        if self.direction_index(slot) == 0 {
            1
        } else {
            -1
        }
    }

    /// Zero-based same-direction observation ordinal for one slot.
    #[cfg(test)]
    pub const fn observation_index(self, slot: u8) -> usize {
        (slot / 2) as usize
    }

    /// Slot carrying one same-direction observation ordinal; inverse of [`Self::direction_index`]
    /// and `Self::observation_index`.
    pub const fn slot_for(self, observation: usize, direction: usize) -> u8 {
        (observation * 2 + (direction ^ self.parity_twist(observation))) as u8
    }

    /// Stable wire code.
    pub const fn report_code(self) -> u8 {
        self as u8
    }
}

/// Compact immutable Integral state retained after plan emission.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralExecutionPlan {
    /// Exact complete-plan digest.
    pub plan_digest: u64,
    /// Fixed proportional gain.
    pub fixed_p: u16,
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
    /// Largest native positive-I ladder cap the half-octave rung was derived
    /// from, for a `Breakaway`-sourced plan; `0` when no ladder cap applies.
    pub i_top_cap: u16,
    /// Family-wise observation count per direction.
    pub family_size: u16,
    /// Fixed moving-capture duration.
    pub moving_stroke_us: u32,
    /// Fixed zero-target settle duration.
    pub zero_settle_us: u32,
    /// Cooperative analysis allowance per observation.
    pub analysis_budget_us: u32,
    /// Conservative complete Integral duration retained in candidate identity.
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
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    /// One opposite-sign meter quantum can contact the probe clamp during recovery.
    pub recovery_quantization_exposed: bool,
    /// The commissioning probe clamp removed at least one Proportional-operable rung.
    pub probe_constrained_test_point: bool,
    /// Execution order of the fixed forward and reverse observation slots.
    pub slot_order: IntegralSlotOrder,
}

const _: () = assert!(core::mem::size_of::<VelocityIntegralExecutionPlan>() == 72);
const _: () = assert!(core::mem::size_of::<VelocityIntegralExecutionPlan>() <= 72);

/// Exact identity retained when Integral has no transition-capable test point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralFailedAdmission {
    /// Digest of the rejected authority and controller geometry, tagged with
    /// [`INTEGRAL_EVIDENCE_SCHEMA_REVISION`].
    pub plan_digest: u64,
    /// The commissioning probe clamp removed every Proportional-operable rung.
    pub probe_constrained_test_point: bool,
}

/// Failure to construct a deterministic Integral plan before motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityIntegralPlanError {
    /// A required duration, scale, or workflow component was invalid.
    InvalidRequest,
    /// Proportional authority does not belong to the supplied stage plan.
    MismatchedAuthority,
    /// Fewer than three positive-I rungs fit the acquisition geometry.
    InsufficientStructuralSpan,
    /// The simultaneous observation family exceeds 128.
    ConfidenceFamilyTooLarge,
    /// Checked planner arithmetic exceeded its representation.
    Arithmetic,
    /// The reproduced Proportional component contains no transition-capable Integral rung.
    NoTransitionCapableOperatingPoint(IntegralFailedAdmission),
    /// A breakaway confirmation plan's recomputed digest, fixed family size, capture profile, or
    /// acceptance rule did not match.
    InvalidConfirmationPlan,
    /// The confirmed proportional gain failed `transition_capable_integral_gain`; carries the digest
    /// of the rejected [`BandConfirmationPlan`].
    ConfirmedGainNotTransitionCapable(u64),
    /// The confirmation evidence's PWM carrier frequency or encoder counts per revolution does not
    /// match the live values presented at Integral construction.
    PhysicalContextMismatch,
}

/// Rejected attempt to bind Integral authority to a breakaway confirmation; fail-closed.
///
/// Returned only by `bind_breakaway_integral_authority`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BreakawayAuthorityBindingError {
    /// The held-out confirmation block did not accept.
    ConfirmationNotAccepted,
    /// The accepted candidate gain does not match the confirmation plan's
    /// declared candidate: the acceptance does not belong to this plan.
    AcceptanceCandidateMismatch,
    /// The candidate authority digest does not name this confirmation plan --
    /// for example, a discovery or probe digest offered in its place.
    AcquisitionDigestMismatch,
}

/// Digest of an accepted breakaway-campaign confirmation.
///
/// The inner field is private; the only way to obtain one is `bind_breakaway_integral_authority`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ConfirmedAcquisitionDigest(u64);

impl ConfirmedAcquisitionDigest {
    /// The validated digest, exactly as carried by
    /// [`IntegralAcquisitionAuthority::acquisition`].
    pub const fn get(self) -> u64 {
        self.0
    }
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

pub(crate) const fn native_velocity_i_at(index: u8) -> Option<u16> {
    match index {
        0 => Some(1),
        1 => Some(2),
        2 => Some(3),
        value => 1_u16.checked_shl((value - 1) as u32),
    }
}

pub(crate) fn native_rung_count(i_top_cap: u16) -> u8 {
    let mut index = 0_u8;
    while let Some(candidate) = native_velocity_i_at(index) {
        if candidate > i_top_cap {
            break;
        }
        index = match index.checked_add(1) {
            Some(next) => next,
            None => break,
        };
    }
    index
}

pub(crate) fn integral_top_gap_candidate(i_top_cap: u16) -> Option<u16> {
    let count = native_rung_count(i_top_cap);
    if count == 0 {
        return None;
    }
    let upper_octave = native_velocity_i_at(count - 1)?;
    if upper_octave < 8 {
        return None;
    }
    let lower_octave = upper_octave / 2;
    if lower_octave < 4 {
        return None;
    }
    let candidate = super::sweep_plan::half_octave_candidate(lower_octave)?;
    if candidate > lower_octave && candidate < upper_octave {
        Some(candidate)
    } else {
        None
    }
}

pub(crate) fn integral_gain_at(i_top_cap: u16, positive_index: u8) -> Option<u16> {
    if i_top_cap == 0 {
        return native_velocity_i_at(positive_index);
    }
    let native_len = native_rung_count(i_top_cap);
    match integral_top_gap_candidate(i_top_cap) {
        None => {
            if positive_index < native_len {
                native_velocity_i_at(positive_index)
            } else {
                None
            }
        }
        Some(candidate) => {
            let insert_index = native_len.checked_sub(1)?;
            if positive_index < insert_index {
                native_velocity_i_at(positive_index)
            } else if positive_index == insert_index {
                Some(candidate)
            } else if positive_index == insert_index.checked_add(1)? {
                native_velocity_i_at(insert_index)
            } else {
                None
            }
        }
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
    let i_start = INTEGRAL_MIN_POSITIVE_I;
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
        .checked_mul(usize::from(INTEGRAL_OBSERVATIONS_PER_DIRECTION))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    if positive_rungs > INTEGRAL_MAX_POSITIVE_RUNGS || family > usize::from(CONFIDENCE_FAMILY_MAX) {
        return Err(VelocityIntegralPlanError::ConfidenceFamilyTooLarge);
    }
    u16::try_from(family).map_err(|_| VelocityIntegralPlanError::Arithmetic)
}

pub(crate) fn recovery_quantization_exposed(
    p_raw: u16,
    target_rpm: i32,
    counts_per_rev: u32,
    probe_limit: u16,
) -> Result<bool, VelocityIntegralPlanError> {
    let target_rpm = target_rpm
        .checked_abs()
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let velocity_term = u128::from(target_rpm as u32)
        .checked_mul(u128::from(counts_per_rev))
        .and_then(|value| value.checked_add(u128::from(STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE)))
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

pub(crate) fn transition_capable_integral_gain(
    p_raw: u16,
    target_rpm: i32,
    counts_per_rev: u32,
    pwm_hz: u32,
    moving_stroke_us: u32,
    probe_limit: u16,
) -> Result<bool, VelocityIntegralPlanError> {
    if p_raw == 0 || counts_per_rev == 0 || pwm_hz == 0 || moving_stroke_us == 0 || probe_limit == 0
    {
        return Err(VelocityIntegralPlanError::InvalidRequest);
    }
    let target_rpm = target_rpm
        .checked_abs()
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let stroke_cycles = u128::from(pwm_hz)
        .checked_mul(u128::from(moving_stroke_us))
        .and_then(|value| value.checked_add(u128::from(MICROS_PER_SECOND - 1)))
        .map(|value| value / u128::from(MICROS_PER_SECOND))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let error_num = u128::from(target_rpm as u32)
        .checked_mul(u128::from(counts_per_rev))
        .and_then(|value| value.checked_add(u128::from(STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let prefix_scale = u128::from(crate::autotune::ADVANCED_PI_I_Q4_12_NUMERATOR)
        .checked_add(
            stroke_cycles
                .checked_mul(3)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let lhs = u128::from(p_raw)
        .checked_mul(error_num)
        .and_then(|value| value.checked_mul(prefix_scale))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let rhs = 256_u128
        .checked_mul(u128::from(counts_per_rev))
        .and_then(|value| {
            value.checked_mul(u128::from(crate::autotune::ADVANCED_PI_I_Q4_12_NUMERATOR))
        })
        .and_then(|value| value.checked_mul(u128::from(probe_limit)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    Ok(lhs < rhs)
}

pub(crate) fn max_transition_capable_p(
    target_rpm: i32,
    counts_per_rev: u32,
    pwm_hz: u32,
    moving_stroke_us: u32,
    probe_limit: u16,
) -> Result<u16, VelocityIntegralPlanError> {
    if counts_per_rev == 0 || pwm_hz == 0 || moving_stroke_us == 0 || probe_limit == 0 {
        return Err(VelocityIntegralPlanError::InvalidRequest);
    }
    let target_rpm = target_rpm
        .checked_abs()
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let stroke_cycles = u128::from(pwm_hz)
        .checked_mul(u128::from(moving_stroke_us))
        .and_then(|value| value.checked_add(u128::from(MICROS_PER_SECOND - 1)))
        .map(|value| value / u128::from(MICROS_PER_SECOND))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let error_num = u128::from(target_rpm as u32)
        .checked_mul(u128::from(counts_per_rev))
        .and_then(|value| value.checked_add(u128::from(STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let prefix_scale = u128::from(crate::autotune::ADVANCED_PI_I_Q4_12_NUMERATOR)
        .checked_add(
            stroke_cycles
                .checked_mul(3)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let denominator = error_num
        .checked_mul(prefix_scale)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let rhs = 256_u128
        .checked_mul(u128::from(counts_per_rev))
        .and_then(|value| {
            value.checked_mul(u128::from(crate::autotune::ADVANCED_PI_I_Q4_12_NUMERATOR))
        })
        .and_then(|value| value.checked_mul(u128::from(probe_limit)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let max_p = (rhs - 1) / denominator;
    Ok(u16::try_from(max_p.min(u128::from(u16::MAX))).unwrap_or(u16::MAX))
}

const BREAKAWAY_PRIMITIVE_US: u32 = MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US;
const BREAKAWAY_DISCOVERY_PRIMITIVE_US: u32 =
    BREAKAWAY_PRIMITIVE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US;

/// Worst-case upward-probe duration reserved before motion: [`PROBE_MAX_OBSERVATIONS`] primitives.
pub fn breakaway_probe_slots_us() -> Result<u64, VelocityIntegralPlanError> {
    u64::from(PROBE_MAX_OBSERVATIONS)
        .checked_mul(u64::from(BREAKAWAY_PRIMITIVE_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Exact discovery-phase duration for one resolved rung count, including one fresh zero-target
/// rest window after an initial rest rejection.
///
/// Increases with rung count; the worst case, [`MAX_SWEEP_RUNGS`], is reserved by
/// [`breakaway_workflow_envelope_us`].
pub fn breakaway_discovery_slots_us(rung_count: u8) -> Result<u64, VelocityIntegralPlanError> {
    u64::from(rung_count)
        .checked_mul(u64::from(INTEGRAL_OBSERVATIONS_PER_RUNG))
        .and_then(|slots| slots.checked_mul(u64::from(BREAKAWAY_DISCOVERY_PRIMITIVE_US)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Fixed family-8 confirmation-phase primitive duration: 8 strokes.
pub fn breakaway_confirmation_slots_us() -> Result<u64, VelocityIntegralPlanError> {
    u64::from(CONFIRMATION_FAMILY_SIZE)
        .checked_mul(u64::from(BREAKAWAY_PRIMITIVE_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Reserve the complete pre-motion breakaway-campaign workflow envelope, in microseconds.
///
/// Sums the worst-case probe, [`MAX_SWEEP_RUNGS`]-rung discovery, family-8 confirmation, the
/// caller's recovery reservation (`recovery_slot_count * recovery_maximum_slot_us`), and the
/// caller's maximum Integral continuation. Every later phase's exact duration must fit inside it.
pub fn breakaway_workflow_envelope_us(
    recovery_slot_count: u32,
    recovery_maximum_slot_us: u32,
    integral_maximum_us: u64,
) -> Result<u64, VelocityIntegralPlanError> {
    let probe_us = breakaway_probe_slots_us()?;
    let discovery_us = breakaway_discovery_slots_us(MAX_SWEEP_RUNGS as u8)?;
    let confirmation_us = breakaway_confirmation_slots_us()?;
    let recovery_us = u64::from(recovery_slot_count)
        .checked_mul(u64::from(recovery_maximum_slot_us))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    probe_us
        .checked_add(recovery_us)
        .and_then(|value| value.checked_add(discovery_us))
        .and_then(|value| value.checked_add(confirmation_us))
        .and_then(|value| value.checked_add(integral_maximum_us))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Round the breakaway-campaign workflow envelope outward to milliseconds, once, after summing
/// every microsecond term of [`breakaway_workflow_envelope_us`].
pub fn breakaway_workflow_envelope_ms(
    recovery_slot_count: u32,
    recovery_maximum_slot_us: u32,
    integral_maximum_us: u64,
) -> Result<u32, VelocityIntegralPlanError> {
    let total_us = breakaway_workflow_envelope_us(
        recovery_slot_count,
        recovery_maximum_slot_us,
        integral_maximum_us,
    )?;
    u32::try_from(total_us.div_ceil(1_000)).map_err(|_| VelocityIntegralPlanError::Arithmetic)
}

pub(crate) fn bind_breakaway_integral_authority(
    candidate_acquisition_digest: u64,
    confirmation_plan: BandConfirmationPlan,
    confirmation_outcome: Result<BandConfirmationAcceptance, TargetBandConfirmationInconclusive>,
) -> Result<ConfirmedAcquisitionDigest, BreakawayAuthorityBindingError> {
    let acceptance = confirmation_outcome
        .map_err(|_| BreakawayAuthorityBindingError::ConfirmationNotAccepted)?;
    if acceptance.nominated_p() != confirmation_plan.nominated_p {
        return Err(BreakawayAuthorityBindingError::AcceptanceCandidateMismatch);
    }
    if candidate_acquisition_digest != confirmation_plan.plan_digest {
        return Err(BreakawayAuthorityBindingError::AcquisitionDigestMismatch);
    }
    Ok(ConfirmedAcquisitionDigest(confirmation_plan.plan_digest))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct AcquisitionContext {
    pub(crate) planned_velocity_mrev_s: u32,
    pub(crate) target_velocity_rpm: i32,
    pub(crate) hard_torque_limit: u16,
    pub(crate) usable_torque_limit: u16,
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    pub(crate) channel: u8,
    pub(crate) plan_time_pwm_hz: u32,
    pub(crate) plan_time_encoder_counts_per_rev: u32,
}

impl From<VelocitySweepExecutionPlan> for AcquisitionContext {
    fn from(plan: VelocitySweepExecutionPlan) -> Self {
        Self {
            planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
            target_velocity_rpm: plan.target_velocity_rpm.raw(),
            hard_torque_limit: plan.hard_torque_limit,
            usable_torque_limit: plan.usable_torque_limit,
            limit_evidence_capability: plan.limit_evidence_capability,
            channel: 0,
            plan_time_pwm_hz: 0,
            plan_time_encoder_counts_per_rev: 0,
        }
    }
}

/// Held-out breakaway acceptance evidence behind an integral plan.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BreakawayIntegralEvidence {
    pub(crate) held_out_response_intervals_q: [[i32; 2]; 2],
    pub(crate) confidence_q: u16,
    pub(crate) band_position_q: u16,
}

/// Motion budget and plan-time physical context for an integral plan.
#[derive(Debug, Clone, Copy)]
pub(crate) struct IntegralPlanPhysics {
    pub(crate) motion_budget: OuterMotionBudget,
    pub(crate) pwm_hz: u32,
    pub(crate) encoder_counts_per_rev: u32,
}

pub(crate) fn plan_breakaway_velocity_integral_response(
    token: ConfirmedAcquisitionDigest,
    confirmation_plan: BandConfirmationPlan,
    context: AcquisitionContext,
    evidence: BreakawayIntegralEvidence,
    physics: IntegralPlanPhysics,
) -> Result<VelocityIntegralPlan, VelocityIntegralPlanError> {
    let BreakawayIntegralEvidence {
        held_out_response_intervals_q,
        confidence_q,
        band_position_q,
    } = evidence;
    let IntegralPlanPhysics {
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    } = physics;
    validate_fixed_contract(&confirmation_plan)
        .map_err(|_| VelocityIntegralPlanError::InvalidConfirmationPlan)?;
    if confirmation_plan.plan_digest != token.get() {
        return Err(VelocityIntegralPlanError::MismatchedAuthority);
    }
    if context.plan_time_pwm_hz != pwm_hz
        || context.plan_time_encoder_counts_per_rev != encoder_counts_per_rev
    {
        return Err(VelocityIntegralPlanError::PhysicalContextMismatch);
    }
    let authority = IntegralAcquisitionAuthority::new_breakaway(
        context.channel,
        token,
        held_out_response_intervals_q,
        confidence_q,
        band_position_q,
    );
    let fixed_p = confirmation_plan.nominated_p.raw();
    if !transition_capable_integral_gain(
        fixed_p,
        context.target_velocity_rpm,
        encoder_counts_per_rev,
        pwm_hz,
        MOVING_STROKE_US,
        context.usable_torque_limit,
    )? {
        return Err(
            VelocityIntegralPlanError::ConfirmedGainNotTransitionCapable(
                confirmation_plan.plan_digest,
            ),
        );
    }
    plan_velocity_integral_response_from_source(
        IntegralPlanSource {
            authority,
            fixed_p,
            schema_revision: super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION,
            probe_constrained_test_point: false,
        },
        context,
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    )
}

pub(crate) fn plan_retained_breakaway_velocity_integral_response(
    token: ConfirmedAcquisitionDigest,
    fixed_p: u16,
    context: AcquisitionContext,
    evidence: BreakawayIntegralEvidence,
    physics: IntegralPlanPhysics,
) -> Result<VelocityIntegralPlan, VelocityIntegralPlanError> {
    let BreakawayIntegralEvidence {
        held_out_response_intervals_q,
        confidence_q,
        band_position_q,
    } = evidence;
    let IntegralPlanPhysics {
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    } = physics;
    if context.plan_time_pwm_hz != pwm_hz
        || context.plan_time_encoder_counts_per_rev != encoder_counts_per_rev
    {
        return Err(VelocityIntegralPlanError::PhysicalContextMismatch);
    }
    let authority = IntegralAcquisitionAuthority::new_breakaway(
        context.channel,
        token,
        held_out_response_intervals_q,
        confidence_q,
        band_position_q,
    );
    if !transition_capable_integral_gain(
        fixed_p,
        context.target_velocity_rpm,
        encoder_counts_per_rev,
        pwm_hz,
        MOVING_STROKE_US,
        context.usable_torque_limit,
    )? {
        return Err(VelocityIntegralPlanError::ConfirmedGainNotTransitionCapable(token.get()));
    }
    plan_velocity_integral_response_from_source(
        IntegralPlanSource {
            authority,
            fixed_p,
            schema_revision: super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION,
            probe_constrained_test_point: false,
        },
        context,
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    )
}

#[derive(Debug, Clone, Copy)]
struct IntegralPlanSource {
    authority: IntegralAcquisitionAuthority,
    fixed_p: u16,
    schema_revision: u16,
    probe_constrained_test_point: bool,
}

fn plan_velocity_integral_response_from_source(
    source: IntegralPlanSource,
    context: AcquisitionContext,
    motion_budget: OuterMotionBudget,
    pwm_hz: u32,
    encoder_counts_per_rev: u32,
) -> Result<VelocityIntegralPlan, VelocityIntegralPlanError> {
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
    if encoder_counts_per_rev == 0
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
    let mut ladder = build_integral_ladder(
        pwm_hz,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )?;
    let i_top_cap = {
        if let Some(candidate) = integral_top_gap_candidate(ladder.i_top_cap) {
            let insert_index = ladder
                .rungs
                .len()
                .checked_sub(1)
                .ok_or(VelocityIntegralPlanError::InsufficientStructuralSpan)?;
            ladder
                .rungs
                .insert(insert_index, candidate)
                .map_err(|_| VelocityIntegralPlanError::ConfidenceFamilyTooLarge)?;
        }
        ladder.i_top_cap
    };
    let positive_rung_count =
        u8::try_from(ladder.rungs.len()).map_err(|_| VelocityIntegralPlanError::Arithmetic)?;
    let total_rung_count = positive_rung_count
        .checked_add(2)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let expected_observations = u16::from(total_rung_count)
        .checked_mul(INTEGRAL_OBSERVATIONS_PER_RUNG as u16)
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let recovery = plan_rung_origin_recovery(
        motion_budget,
        encoder_counts_per_rev,
        context.planned_velocity_mrev_s,
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
    let slot_transition_us = 0;
    let recovery_nominal_slot_us = recovery.nominal_slot_us;
    let nominal_workflow_us = workflow_us
        .checked_add(
            u32::from(total_rung_count)
                .checked_mul(recovery_nominal_slot_us)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .and_then(|value| value.checked_add(slot_transition_us))
        .and_then(|value| value.checked_add(ZERO_SETTLE_US))
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let maximum_workflow_us = workflow_us
        .checked_add(
            u32::from(total_rung_count)
                .checked_mul(recovery.maximum_slot_us)
                .ok_or(VelocityIntegralPlanError::Arithmetic)?,
        )
        .and_then(|value| value.checked_add(slot_transition_us))
        .and_then(|value| value.checked_add(ZERO_SETTLE_US))
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)?;
    let mut positive_i = [0; INTEGRAL_MAX_POSITIVE_RUNGS];
    positive_i[..ladder.rungs.len()].copy_from_slice(ladder.rungs.as_slice());
    let recovery_quantization_exposed = recovery_quantization_exposed(
        source.fixed_p,
        context.target_velocity_rpm,
        encoder_counts_per_rev,
        context.usable_torque_limit,
    )?;

    Ok(VelocityIntegralPlan {
        slot_order: IntegralSlotOrder::ForwardFirst,
        build_features: BUILD_FEATURES,
        schema_revision: source.schema_revision,
        authority: source.authority,
        fixed_p: source.fixed_p,
        planned_velocity_mrev_s: context.planned_velocity_mrev_s,
        target_velocity_rpm: context.target_velocity_rpm,
        pwm_hz,
        encoder_counts_per_rev,
        positive_i,
        positive_rung_count,
        i_top_cap,
        family_size: confidence_family_size(ladder.rungs.len())?,
        total_rung_count,
        expected_observations,
        moving_stroke_us: MOVING_STROKE_US,
        zero_settle_us: ZERO_SETTLE_US,
        analysis_budget_us: SETTLE_ANALYSIS_BUDGET_US,
        nominal_workflow_ms: nominal_workflow_us.div_ceil(1_000),
        maximum_workflow_ms: maximum_workflow_us.div_ceil(1_000),
        recovery_origin_band_counts: recovery.origin_band_counts,
        recovery_nominal_slot_us,
        recovery_maximum_slot_us: recovery.maximum_slot_us,
        recovery_slot_count: total_rung_count,
        hard_torque_limit: context.hard_torque_limit,
        usable_torque_limit: context.usable_torque_limit,
        limit_evidence_capability: context.limit_evidence_capability,
        recovery_quantization_exposed,
        probe_constrained_test_point: source.probe_constrained_test_point,
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

pub(super) fn largest_i_for_tau(
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

pub(super) const fn fnv1a64_byte(hash: u64, byte: u8) -> u64 {
    (hash ^ byte as u64).wrapping_mul(0x0000_0100_0000_01b3)
}
