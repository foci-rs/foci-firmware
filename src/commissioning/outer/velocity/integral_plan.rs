//! Immutable Stage-C velocity-integral acquisition planning.

use heapless::Vec;

use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};

use super::band_confirmation::{
    BandConfirmationAcceptance, BandConfirmationPlan, CONFIRMATION_FAMILY_SIZE,
    TargetBandConfirmationInconclusive, validate_fixed_contract,
};
use super::breakaway_plan::PROBE_MAX_OBSERVATIONS;
use super::current_guard::VelocityLimitEvidenceCapability;
use super::recovery::{RungOriginRecoveryPlanError, plan_rung_origin_recovery};
use super::reproduction::{BUILD_FEATURES, StageBDirectionalValidity};
use super::sweep_plan::{
    MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US,
    VelocitySweepExecutionPlan, VelocitySweepRequest, ZERO_SETTLE_US,
};

/// Evidence schema stored in the RAM-only Stage-C candidate slot.
pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u16 = 17;
/// Largest predeclared positive-I ladder accepted by the family-128 table.
pub const STAGE_C_MAX_POSITIVE_RUNGS: usize = 30;
/// Minimum representable positive Q4.12 velocity-I gain.
pub const STAGE_C_MIN_POSITIVE_I: u16 = 1;
/// Independent observations collected in one direction at every Stage-C rung.
pub const STAGE_C_OBSERVATIONS_PER_DIRECTION: u16 = 4;
/// Forward and reverse observations collected at every Stage-C rung.
pub const STAGE_C_OBSERVATIONS_PER_RUNG: u32 = 8;
/// Maximum combined entry-plus-exit SPI time reserved for one moving slot.
pub const INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US: u32 = 20_000;

const MICROS_PER_SECOND: u64 = 1_000_000;
const CONFIDENCE_FAMILY_MAX: u16 = 128;

/// Reproduced Stage-B evidence that authorizes one fixed-P Stage-C plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCAuthority {
    /// Physical firmware channel.
    pub channel: u8,
    /// The unforgeable token of the accepted breakaway confirmation that
    /// authorized this Stage-C plan. There is no Stage-B sweep plan behind
    /// it: the breakaway campaign has no Stage-B phase, so nothing
    /// recomputes from the request.
    pub acquisition: ConfirmedAcquisitionDigest,
    /// Cross-run forward and reverse model-or-fragment intersections.
    pub directional_memberships: [u32; 2],
    /// Reproduced Stage-B joint-operability component before Stage-C filtering.
    pub stage_b_joint_membership: u32,
    /// Stage-C transition-capable subset of the reproduced joint component.
    pub joint_membership: u32,
    /// Cross-run validity of the forward and reverse Stage-B models.
    pub directional_validity: [StageBDirectionalValidity; 2],
    /// Whether the selected joint component contains fewer than three rungs.
    pub reduced_margin: bool,
    /// Reproduced pooled intervals, or `[0, 0]` where model comparison is unavailable.
    pub directional_pooled_intervals: [[i32; 2]; 2],
    /// Held-out confirmation confidence in Q0.16, or `0` off the breakaway path.
    /// See [`super::band_confirmation::BandConfirmationAcceptance::confidence_q`].
    pub confidence_q: u16,
    /// Held-out confirmation band position in Q0.16, or `0` off the breakaway
    /// path. See
    /// [`super::band_confirmation::BandConfirmationAcceptance::band_position_q`].
    pub band_position_q: u16,
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

    /// Construct breakaway-campaign Stage-C authority bound to a confirmed
    /// acquisition digest.
    ///
    /// The breakaway campaign has no reproduced Stage-B directional evidence,
    /// so every membership/validity field is the same empty/zero value a
    /// nonretained, same-command authority uses.
    /// `directional_pooled_intervals` is
    /// the one exception: it carries `held_out_response_intervals_q`, the
    /// held-out confirmation block's own pooled response interval, already in
    /// the same signed settled-rate-Q domain this field carries on the other
    /// paths (see [`super::band_confirmation::BandConfirmationAcceptance`]'s
    /// field doc), not a reproduced Stage-B value.
    /// The only way to obtain a [`ConfirmedAcquisitionDigest`] is a successful
    /// call to [`bind_breakaway_stage_c_authority`], so this constructor makes
    /// "Stage C authorized by anything other than an accepted confirmation"
    /// unrepresentable for every breakaway authority built this way.
    #[allow(dead_code)]
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
            stage_b_joint_membership: 0,
            joint_membership: 0,
            directional_validity: [StageBDirectionalValidity::NoRegion; 2],
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
    pub rungs: Vec<u16, STAGE_C_MAX_POSITIVE_RUNGS>,
}

/// Complete immutable Stage-C plan emitted before its first stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralPlan {
    /// Analysis-rule contract this plan was built under.
    pub build_features: u16,
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
    /// Largest native positive-I ladder cap the half-octave rung was derived
    /// from, for a `Breakaway`-sourced plan; `0` (not applicable) for
    /// amplitude-sourced plans, which never regenerate one.
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
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    /// One opposite-sign meter quantum can contact the probe clamp during recovery.
    pub recovery_quantization_exposed: bool,
    /// The commissioning probe clamp removed at least one Stage-B-operable rung.
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
        hash_value!(self.final_p);
        for membership in self.authority.directional_memberships {
            hash_value!(membership);
        }
        hash_value!(self.authority.stage_b_joint_membership);
        hash_value!(self.authority.joint_membership);
        for validity in self.authority.directional_validity {
            hash_value!(validity as u8);
        }
        hash_value!(u8::from(self.authority.reduced_margin));
        for interval in self.authority.directional_pooled_intervals {
            hash_value!(interval[0]);
            hash_value!(interval[1]);
        }
        // Preserves the wire/offline plan digest across the
        // removal of `combined_selected_response`. Every surviving plan
        // (breakaway, amplitude, resume) always had that field `None` -- only
        // the deleted live combined P-then-I workflow ever set it `Some` --
        // so `u8::from(combined_selected_response.is_some())` was always the
        // constant byte 0 here. Hashing that same constant keeps `digest()`
        // byte-identical to pre-removal for every plan that ever existed in
        // production, which foci-trace's independent offline recomputation
        // (`velocity_integral.py::integral_plan_digest`) still assumes: it
        // hashes `int(plan.get("combined_selected_response", False))`, which
        // is the same `0` for a capture that never carried the key. Do not
        // turn this back into a real `combined_selected_response` read --
        // the field is gone; this is a frozen digest-compatibility constant.
        hash_value!(0u8);
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
        // Hashed only when mirrored. Forward-first is the historical order, so
        // omitting it keeps every archived plan digest and retained
        // acceptance-point binding valid without a Stage-C schema bump.
        if !matches!(self.slot_order, IntegralSlotOrder::ForwardFirst) {
            hash_value!(self.slot_order.report_code());
        }
        if matches!(
            self.schema_revision,
            STAGE_C_EVIDENCE_SCHEMA_REVISION
                | super::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION
        ) {
            hash_value!(self.i_top_cap);
        }
        hash
    }

    /// Compact the report plan into phase-local execution state.
    pub fn execution_plan(self) -> VelocityIntegralExecutionPlan {
        let selected_response_intervals_q = [[0; 2]; 2];
        VelocityIntegralExecutionPlan {
            slot_order: self.slot_order,
            plan_digest: self.digest(),
            acceptance_plan_digest: 0,
            qualifier_bits: 0,
            final_p: self.final_p,
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
            acquisition_kind: VelocityIntegralAcquisitionKind::ResponseLadder,
            acceptance_source: AcceptanceSource::Breakaway,
            amplitude_targets_rpm: [0; 5],
            selected_response_intervals_q,
            recovery_lower_rate_q: [0; 2],
            recovery_quantization_exposed: self.recovery_quantization_exposed,
            probe_constrained_test_point: self.probe_constrained_test_point,
        }
    }
}

/// Order in which the fixed forward and reverse observation slots execute.
///
/// The eight slots of a rung alternate travel direction about a common start
/// point, and origin recovery runs only between rungs. One direction therefore
/// always begins its stroke at the low end of the span and the other at the high
/// end, so every settled tail for a given direction is measured at the same end.
/// Direction and tail position are confounded, and this type makes which way
/// round explicit and recordable instead of implicit in slot parity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum IntegralSlotOrder {
    /// Even slots travel forward. The historical order.
    ///
    /// `ReverseFirst` and `PairedOutAndBack`, the two mirrored orders this type
    /// once offered, were producible only by the live combined P-then-I
    /// workflow and the mirrored-amplitude actions. Both were removed, and
    /// with them every surviving live producer of this type
    /// -- `AutotuneAction::slot_order`, `AutotuneAction::amplitude_slot_order`,
    /// and the sole `VelocityIntegralPlan` constructor -- became unconditional
    /// `ForwardFirst`. The variants were removed in T9. The type stays an enum
    /// rather than collapsing to a unit so the wire contract (`report_code`)
    /// and foci-trace's independent decode copies keep the same shape.
    ForwardFirst = 0,
}

impl IntegralSlotOrder {
    /// Every declared order, so a new variant cannot be added untested.
    pub const ALL: [Self; 1] = [Self::ForwardFirst];

    /// Parity offset applied to one same-direction observation ordinal.
    ///
    /// Forward-first is the only surviving order and applies no offset.
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
    ///
    /// Independent of the order: mirroring changes a slot's direction, not which
    /// repeat within that direction it is.
    pub const fn observation_index(self, slot: u8) -> usize {
        (slot / 2) as usize
    }

    /// Slot carrying one same-direction observation ordinal.
    ///
    /// Inverse of [`Self::direction_index`] and [`Self::observation_index`]. The
    /// two are pinned together by test, because evidence is written by raw slot
    /// and read back by observation and direction.
    pub const fn slot_for(self, observation: usize, direction: usize) -> u8 {
        (observation * 2 + (direction ^ self.parity_twist(observation))) as u8
    }

    /// Stable wire code.
    pub const fn report_code(self) -> u8 {
        self as u8
    }
}

/// Compact acquisition shape interpreted by the shared velocity-integral engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum VelocityIntegralAcquisitionKind {
    /// Historical opening-anchor, positive-I response ladder, and bookend.
    ResponseLadder = 0,
    /// Fixed-I fixed-gain amplitudes in ascending order.
    FixedGainAmplitudeAscending = 1,
    /// Fixed-I fixed-gain amplitudes in descending order.
    FixedGainAmplitudeDescending = 2,
}

/// Which acquisition authorized one Stage-C execution plan.
///
/// `acquisition_kind == ResponseLadder` alone cannot distinguish these: an
/// accepted breakaway confirmation and a fixed-I fixed-gain-amplitude plan can
/// both report that same acquisition kind. This discriminator isolates
/// breakaway so [`VelocityIntegralExecutionPlan::records_target_terminus`]
/// can admit it without over-admitting a plan that carries no target evidence
/// at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum AcceptanceSource {
    // Discriminant 0 belonged to a since-retired live combined P-then-I
    // workflow; it is left unassigned rather than renumbering the variants
    // below it.
    /// Accepted breakaway-campaign confirmation.
    Breakaway = 1,
    /// Any acceptance source that is not a breakaway confirmation. Carries no
    /// target-reaching evidence.
    NonBreakaway = 2,
}

/// Compact immutable Stage-C state retained after plan emission.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralExecutionPlan {
    /// Exact complete-plan digest.
    pub plan_digest: u64,
    /// Retained acceptance-plan identity digest for an amplitude-sourced plan,
    /// matched against dispatches that resume it; `0` off the amplitude path
    /// (`acceptance_source != AcceptanceSource::NonBreakaway` implies this
    /// acquisition never carries one). Copied verbatim from
    /// `FixedGainAmplitudePlan.acceptance_identity.plan_digest`, never
    /// recomputed.
    pub acceptance_plan_digest: u64,
    /// Packed operating-point, model-validity, and evidence-quality
    /// qualifiers for an amplitude-sourced plan, frozen at the original
    /// breakaway-acceptance install; `0` off the amplitude path. Copied
    /// verbatim from `FixedGainAmplitudePlan.qualifier_bits` — this is a
    /// distinct, earlier snapshot from this struct's own
    /// `recovery_quantization_exposed`/`probe_constrained_test_point`
    /// fields below, which reflect the *current* run instead.
    pub qualifier_bits: u16,
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
    /// Largest native positive-I ladder cap the half-octave rung was derived
    /// from, for a `Breakaway`-sourced plan; `0` (not applicable) for
    /// amplitude-sourced plans, which never regenerate one.
    pub i_top_cap: u16,
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
    /// Response-ladder or fixed-I fixed-gain amplitude acquisition.
    pub acquisition_kind: VelocityIntegralAcquisitionKind,
    /// Which acquisition category authorized this plan.
    ///
    /// Isolates breakaway from reproduced Stage-B within `acquisition_kind ==
    /// ResponseLadder`; see [`Self::records_target_terminus`].
    pub acceptance_source: AcceptanceSource,
    /// Exact fixed targets for an amplitude acquisition; zero for response ladders.
    pub amplitude_targets_rpm: [i16; 5],
    /// Selected Stage-B response intervals; zero for an amplitude acquisition,
    /// which carries its identity and qualifiers in `acceptance_plan_digest`
    /// and `qualifier_bits` instead.
    pub selected_response_intervals_q: [[i32; 2]; 2],
    /// Conservative top-point settled-rate bounds used only for amplitude recovery deadlines.
    pub recovery_lower_rate_q: [u64; 2],
    /// One opposite-sign meter quantum can contact the probe clamp during recovery.
    pub recovery_quantization_exposed: bool,
    /// The commissioning probe clamp removed at least one Stage-B-operable rung.
    pub probe_constrained_test_point: bool,
    /// Execution order of the fixed forward and reverse observation slots.
    pub slot_order: IntegralSlotOrder,
}

// 112 -> 120: `i_top_cap: u16` is needed so response-ladder plans can
// regenerate their half-octave rung during execution; unlike
// `VelocityIntegralPlan`, this struct has no unused padding to absorb it
// (its leading `u64` field forces 8-byte alignment, so the two extra bytes
// push the total past the previous 112-byte boundary to the next multiple).
// 120 -> 128: `acceptance_plan_digest: u64` and `qualifier_bits: u16`
// replaced a tunnel through `selected_response_intervals_q`, adding real
// storage instead of reusing the existing array.
const _: () = assert!(core::mem::size_of::<VelocityIntegralExecutionPlan>() == 128);
const _: () = assert!(core::mem::size_of::<VelocityIntegralExecutionPlan>() <= 128);

impl VelocityIntegralExecutionPlan {
    /// Whether a target-reaching run on this plan records a target terminus.
    ///
    /// True only for an accepted breakaway confirmation. A reproduced Stage-B
    /// response ladder must not record a terminus even though it shares
    /// `acquisition_kind == VelocityIntegralAcquisitionKind::ResponseLadder`.
    pub const fn records_target_terminus(&self) -> bool {
        matches!(self.acceptance_source, AcceptanceSource::Breakaway)
    }
}

/// Exact identity retained when Stage C has no transition-capable test point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCFailedAdmission {
    /// Digest of the rejected authority and controller geometry, tagged with
    /// [`STAGE_C_EVIDENCE_SCHEMA_REVISION`].
    pub plan_digest: u64,
    /// The commissioning probe clamp removed every Stage-B-operable rung.
    pub probe_constrained_test_point: bool,
}

/// Failure to construct a deterministic Stage-C plan before motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityIntegralPlanError {
    /// A required duration, scale, or workflow component was invalid.
    InvalidRequest,
    /// Stage-B authority does not belong to the supplied stage plan.
    MismatchedAuthority,
    /// Fewer than three positive-I rungs fit the acquisition geometry.
    InsufficientStructuralSpan,
    /// The simultaneous observation family exceeds 128.
    ConfidenceFamilyTooLarge,
    /// Checked planner arithmetic exceeded its representation.
    Arithmetic,
    /// The reproduced Stage-B component contains no transition-capable Stage-C rung.
    NoTransitionCapableOperatingPoint(StageCFailedAdmission),
    /// A breakaway confirmation plan's recomputed digest, or its fixed family
    /// size, capture profile, or acceptance rule, did not match what
    /// [`plan_breakaway_velocity_integral_response`] requires before it may
    /// authorize Stage C.
    InvalidConfirmationPlan,
    /// The breakaway campaign's confirmed proportional gain -- the exact
    /// candidate a held-out confirmation accepted, with no reselection --
    /// failed [`transition_capable_stage_c_gain`]. Carries the digest of the
    /// rejected [`BandConfirmationPlan`].
    ConfirmedGainNotTransitionCapable(u64),
    /// The plan-time physical acquisition context frozen into the confirmation
    /// evidence (PWM carrier frequency or encoder counts per revolution) does
    /// not match the live values presented at Stage-C construction. Confirmation
    /// evidence gathered under one physical timebase cannot seed a Stage-C plan
    /// admitted under another; the gate refuses without motion.
    PhysicalContextMismatch,
}

/// Rejected attempt to bind Stage-C authority to a breakaway confirmation.
///
/// Returned only by [`bind_breakaway_stage_c_authority`], the single gate
/// that may produce the digest a breakaway-schema [`StageCAuthority::
/// stage_b_plan_digest`] carries. Every rejection is fail-closed: no digest,
/// discovery or otherwise, ever authorizes Stage C unless it is exactly the
/// digest of a [`BandConfirmationPlan`] whose held-out block accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BreakawayAuthorityBindingError {
    /// The held-out confirmation block did not accept; no digest may authorize
    /// Stage C for this campaign, even though the confirmation plan itself is
    /// otherwise well-formed.
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
/// The inner field is private, and this module exposes no way to construct
/// one from a raw `u64`: the only path to a value of this type is a
/// successful call to [`bind_breakaway_stage_c_authority`], which requires an
/// `Ok` [`BandConfirmationAcceptance`] whose candidate matches the bound
/// [`BandConfirmationPlan`] and whose digest matches the candidate digest
/// under review. A sibling module -- including this module's own test
/// module, which lives in a separate file and is therefore not a descendant
/// of this one -- cannot write `ConfirmedAcquisitionDigest(raw_digest)`; only
/// code inside `integral_plan.rs` can. This makes "Stage C authorized by
/// anything other than an accepted confirmation" a compile-time
/// impossibility for every breakaway authority built through
/// [`StageCAuthority::new_breakaway`], not merely an untested code path.
///
/// The type is `pub` only so [`StageCAuthority::acquisition`] can name it.
/// Its field is private and no public constructor exists, so the
/// unforgeability above holds outside the crate as well.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ConfirmedAcquisitionDigest(u64);

impl ConfirmedAcquisitionDigest {
    /// The validated digest, exactly as carried by
    /// [`StageCAuthority::acquisition`].
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

/// Classify one declared integral time constant against the moving horizon.
pub const fn response_horizon(
    i_raw: u16,
    pwm_hz: u32,
    moving_horizon_us: u32,
) -> Option<(u64, crate::commissioning::types::ResponseHorizonClass)> {
    use crate::commissioning::types::ResponseHorizonClass;

    if i_raw == 0 {
        return Some((0, ResponseHorizonClass::NotApplicable));
    }
    let tau_us = match velocity_i_tau_us(i_raw, pwm_hz) {
        Some(value) => value,
        None => return None,
    };
    let class = if tau_us > moving_horizon_us as u64 {
        ResponseHorizonClass::SubHorizon
    } else {
        ResponseHorizonClass::HorizonResolved
    };
    Some((tau_us, class))
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

/// Count of [`native_velocity_i_at`] entries at or below `i_top_cap`.
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

/// The one possible half-octave rung for the gap immediately below
/// `i_top_cap`, or `None` if no insertion applies.
///
/// Applies only when the gap between the last two native octave rungs
/// (`upper_octave`, `lower_octave = upper_octave / 2`) is entirely within
/// the pure-octave region (`lower_octave >= 4`), never inside the fixed
/// `1, 2, 3` prefix. `i_top_cap = 0` (the not-applicable sentinel used by
/// amplitude-sourced plans) always yields `None`.
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

/// Regenerate one positive-I response-ladder rung by index, matching the
/// exact array [`plan_velocity_integral_response_from_source`] plans:
/// the native fixed-prefix/octave sequence, with the half-octave candidate
/// (if any) inserted immediately before the final native rung.
///
/// `i_top_cap == 0` is the not-applicable sentinel (amplitude-sourced plans,
/// per `VelocityIntegralPlan`/`VelocityIntegralExecutionPlan` doc comments)
/// and always delegates straight to [`native_velocity_i_at`] with no
/// augmentation and no bound -- this is the exact pre-existing behavior
/// those plans had before this function existed. It must be checked before
/// `native_rung_count`, which treats `i_top_cap` as a real ceiling and
/// would otherwise compute zero native rungs at cap zero.
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

/// Whether one proportional gain can exercise the required Q4.12 I prefix unclipped.
pub(crate) fn transition_capable_stage_c_gain(
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
        .and_then(|value| value.checked_add(60_u128 * u128::from(pwm_hz)))
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

/// Largest proportional gain that still satisfies [`transition_capable_stage_c_gain`]
/// for one physical acquisition context, or `0` when not even `p = 1` is capable.
///
/// [`transition_capable_stage_c_gain`] is strictly monotone increasing in
/// `p_raw` (the gain appears only on the left of a `lhs < rhs` comparison whose
/// right side is P-independent), so a single ceiling is exactly equivalent to
/// testing every candidate individually: `p <= max_transition_capable_p(..)`
/// iff the gate admits `p`. Nomination uses this to refuse a gain the Stage-C
/// admission would later reject, before any stroke is planned.
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
        .and_then(|value| value.checked_add(60_u128 * u128::from(pwm_hz)))
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
    // Largest integer P with `P * denominator < rhs`, mirroring the strict
    // `lhs < rhs` in `transition_capable_stage_c_gain`. `rhs >= 1` and
    // `denominator >= 1` always hold, so `(rhs - 1) / denominator` is exact.
    let max_p = (rhs - 1) / denominator;
    Ok(u16::try_from(max_p.min(u128::from(u16::MAX))).unwrap_or(u16::MAX))
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

/// Fixed per-observation primitive cost shared by every probe, discovery, and
/// confirmation stroke: moving capture, zero-target settle, and the
/// cooperative analysis allowance. `512_000 + 500_000 + 236_000 = 1_248_000`.
const BREAKAWAY_PRIMITIVE_US: u32 = MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US;
const BREAKAWAY_DISCOVERY_PRIMITIVE_US: u32 =
    BREAKAWAY_PRIMITIVE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US;

/// Worst-case upward-probe observation count reserved before motion:
/// [`PROBE_MAX_OBSERVATIONS`] primitives, 159.744 s.
pub fn breakaway_probe_slots_us() -> Result<u64, VelocityIntegralPlanError> {
    u64::from(PROBE_MAX_OBSERVATIONS)
        .checked_mul(u64::from(BREAKAWAY_PRIMITIVE_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Exact discovery-phase duration for one resolved rung count, including one
/// fresh zero-target rest window after an initial rest rejection.
///
/// Every realizable additive ladder holds between
/// [`super::sweep_plan::MIN_STRUCTURAL_RUNGS`] and [`MAX_SWEEP_RUNGS`] rungs;
/// the worst case, [`MAX_SWEEP_RUNGS`] (`32 * 8 = 256` slots), is the one
/// reserved by [`breakaway_workflow_envelope_us`]. Every smaller resolved
/// rung count reports a strictly smaller duration here, so its plan always
/// fits inside the pre-motion envelope.
pub fn breakaway_discovery_slots_us(rung_count: u8) -> Result<u64, VelocityIntegralPlanError> {
    u64::from(rung_count)
        .checked_mul(u64::from(STAGE_C_OBSERVATIONS_PER_RUNG))
        .and_then(|slots| slots.checked_mul(u64::from(BREAKAWAY_DISCOVERY_PRIMITIVE_US)))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Fixed family-8 confirmation-phase primitive duration: 8 strokes, ~9.984 s.
pub fn breakaway_confirmation_slots_us() -> Result<u64, VelocityIntegralPlanError> {
    u64::from(CONFIRMATION_FAMILY_SIZE)
        .checked_mul(u64::from(BREAKAWAY_PRIMITIVE_US))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Reserve the complete pre-motion breakaway-campaign workflow envelope, in
/// microseconds.
///
/// Sums the worst-case joint probe, the worst-case [`MAX_SWEEP_RUNGS`]-rung
/// discovery ladder, the fixed family-8 confirmation block, the caller's
/// worst-case recovery reservation (`recovery_slot_count *
/// recovery_maximum_slot_us`), and the caller's maximum possible Stage-C
/// continuation duration. Every term is added in microseconds; callers round
/// to milliseconds exactly once, in [`breakaway_workflow_envelope_ms`], never
/// before. Every later phase's exact plan duration must fit inside the
/// returned reservation; a phase plan whose exact duration would exceed it is
/// a zero-motion planning failure -- see, for example,
/// [`super::breakaway_plan::AdditiveLadderPlanError::WorkflowEnvelopeExceeded`].
pub fn breakaway_workflow_envelope_us(
    recovery_slot_count: u32,
    recovery_maximum_slot_us: u32,
    stage_c_maximum_us: u64,
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
        .and_then(|value| value.checked_add(stage_c_maximum_us))
        .ok_or(VelocityIntegralPlanError::Arithmetic)
}

/// Round the breakaway-campaign workflow envelope outward to milliseconds.
///
/// Every microsecond term in [`breakaway_workflow_envelope_us`] is summed
/// first; this is the single point where the sum is rounded, via
/// `div_ceil(1_000)`, matching every other workflow-duration reservation in
/// this module. The result feeds
/// [`super::workflow_plan::CommissioningWorkflowPlan::new_breakaway`]
/// directly.
pub fn breakaway_workflow_envelope_ms(
    recovery_slot_count: u32,
    recovery_maximum_slot_us: u32,
    stage_c_maximum_us: u64,
) -> Result<u32, VelocityIntegralPlanError> {
    let total_us = breakaway_workflow_envelope_us(
        recovery_slot_count,
        recovery_maximum_slot_us,
        stage_c_maximum_us,
    )?;
    u32::try_from(total_us.div_ceil(1_000)).map_err(|_| VelocityIntegralPlanError::Arithmetic)
}

/// Validate and return the one token that may authorize Stage C for a
/// breakaway campaign: proof that `confirmation_plan`'s held-out block
/// accepted, bound to its digest.
///
/// The breakaway campaign has no Stage-B sweep plan, so a breakaway-schema
/// [`StageCAuthority::stage_b_plan_digest`] instead carries the accepted
/// [`BandConfirmationPlan`]'s digest, obtained only through this function and
/// only via [`StageCAuthority::new_breakaway`]. The returned
/// [`ConfirmedAcquisitionDigest`] cannot be constructed any other way: this
/// is the single gate, and it is fail-closed in both directions the safety
/// invariant forbids:
///
/// - `candidate_acquisition_digest` must be exactly `confirmation_plan.
///   plan_digest`. A discovery or probe digest offered in its place --
///   whether by a call-site copy-paste error or a corrupted evidence record
///   -- is rejected with [`BreakawayAuthorityBindingError::
///   AcquisitionDigestMismatch`].
/// - `confirmation_outcome` must be `Ok`. A confirmation plan whose held-out
///   block did not accept is rejected with [`BreakawayAuthorityBindingError::
///   ConfirmationNotAccepted`] even when its `plan_digest` is otherwise
///   well-formed and matches `candidate_acquisition_digest` exactly: a
///   syntactically valid digest is never enough on its own.
///
/// Callers must supply the actual `Result` returned by
/// [`super::band_confirmation::evaluate_band_confirmation`] as
/// `confirmation_outcome`; there is no way to synthesize acceptance from a
/// bare digest.
#[allow(dead_code)]
pub(crate) fn bind_breakaway_stage_c_authority(
    candidate_acquisition_digest: u64,
    confirmation_plan: BandConfirmationPlan,
    confirmation_outcome: Result<BandConfirmationAcceptance, TargetBandConfirmationInconclusive>,
) -> Result<ConfirmedAcquisitionDigest, BreakawayAuthorityBindingError> {
    let acceptance = confirmation_outcome
        .map_err(|_| BreakawayAuthorityBindingError::ConfirmationNotAccepted)?;
    if acceptance.candidate_p() != confirmation_plan.candidate_p {
        return Err(BreakawayAuthorityBindingError::AcceptanceCandidateMismatch);
    }
    if candidate_acquisition_digest != confirmation_plan.plan_digest {
        return Err(BreakawayAuthorityBindingError::AcquisitionDigestMismatch);
    }
    Ok(ConfirmedAcquisitionDigest(confirmation_plan.plan_digest))
}

/// Explicit physical acquisition inputs the shared Stage-C tail needs.
///
/// The breakaway campaign ([`plan_breakaway_velocity_integral_response`]) has
/// no Stage-B sweep plan at all and builds one directly from its own campaign
/// parameters instead of fabricating a `VelocitySweepExecutionPlan`. The
/// fixed-gain-amplitude preflight (`commission_dispatch::fixed_gain_amplitude`) still
/// reconstructs a Stage-B sweep plan to source these physical fields, via
/// [`From`], then overwrites the breakaway-only fields below directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct AcquisitionContext {
    /// Frozen encoder-domain velocity amplitude.
    pub(crate) planned_velocity_mrev_s: u32,
    /// Frozen signed-target magnitude in mechanical RPM register units.
    pub(crate) target_velocity_rpm: i32,
    /// Configured hard torque-target limit.
    pub(crate) hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub(crate) usable_torque_limit: u16,
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    /// Physical firmware channel the breakaway Stage-C authority is minted for.
    ///
    /// Consumed only by [`plan_breakaway_velocity_integral_response`], which
    /// builds its [`StageCAuthority`] internally from the confirmation token so
    /// no forged authority can be passed in. [`From`] leaves this zero; the
    /// fixed-gain-amplitude preflight overwrites it with the real channel
    /// immediately after constructing from it.
    pub(crate) channel: u8,
    /// Plan-time PWM carrier frequency the confirmation evidence was gathered
    /// under, in Hz.
    ///
    /// The breakaway gate refuses if this does not match the live PWM frequency
    /// handed to it at Stage-C construction, so confirmation evidence measured
    /// under one physical timebase can never seed a Stage-C plan admitted under
    /// another. [`From`] leaves this zero (fail-closed: zero never matches a
    /// live frequency); the fixed-gain-amplitude preflight overwrites it with the
    /// live value immediately after constructing from it.
    pub(crate) plan_time_pwm_hz: u32,
    /// Plan-time encoder counts per mechanical revolution the confirmation
    /// evidence was gathered under.
    ///
    /// Cross-checked against the live encoder resolution by the breakaway gate,
    /// exactly like [`Self::plan_time_pwm_hz`]. [`From`] leaves this zero; the
    /// fixed-gain-amplitude preflight overwrites it the same way.
    pub(crate) plan_time_encoder_counts_per_rev: u32,
}

impl From<VelocitySweepExecutionPlan> for AcquisitionContext {
    /// Extract exactly the physical fields the shared Stage-C tail reads.
    ///
    /// The breakaway-only `channel` and `plan_time_*` fields are left zero:
    /// every caller of this conversion overwrites them explicitly with its
    /// own live values immediately afterward, rather than relying on this
    /// impl to know them.
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

/// Construct one exact breakaway-campaign Stage-C plan from a confirmed
/// candidate proportional gain, with no reselection.
///
/// The normative decision (spec section 7):
/// confirmation selects the exact Stage-C proportional gain but does not
/// waive Stage-C safety admission. `fixed_p = confirmation_plan.candidate_p`
/// unconditionally -- there is no joint-membership test-point selection, no
/// lower-middle or highest-in-band reselection, and no fallback to another
/// discovery candidate. Every existing plan-time safety check still gates
/// Stage C, and any failure refuses without motion.
///
/// The only authority input is `token`, a [`ConfirmedAcquisitionDigest`]. That
/// token can be minted solely by [`bind_breakaway_stage_c_authority`] from a
/// genuine `Ok(`[`BandConfirmationAcceptance`]`)` -- itself unforgeable (its
/// `candidate_p` field is private) -- so this gate cannot be reached for an
/// arbitrary gain without a real all-eight held-out pass. The gate builds the
/// [`StageCAuthority`] INTERNALLY via [`StageCAuthority::new_breakaway`]; it
/// never accepts a pre-built `StageCAuthority`, closing the bypass where a
/// caller forged one by struct literal (its fields are `pub`) and paired it
/// with a self-consistent but fabricated `BandConfirmationPlan`.
/// `held_out_response_intervals_q` is not an authority input -- it gates
/// nothing here and carries only the same accepted block's pooled response
/// interval through to `StageCAuthority::directional_pooled_intervals`, for
/// the acceptance-point recovery authority downstream planning reads from the
/// resulting plan.
///
/// It additionally: (a) re-validates the fixed confirmation contract through
/// the single canonical [`super::band_confirmation::validate_fixed_contract`]
/// -- every frozen field (family size, observations per direction, capture
/// profile, all three timing fields, both band edges, the acceptance rule, the
/// fixed slot order, and digest self-consistency) -- the same function the
/// confirmation phase runs before its first stroke, so the contract cannot
/// drift between the two sites; (b) ties `confirmation_plan.plan_digest` to
/// `token.get()`, so a forged plan whose candidate (or any hashed field)
/// differs is refused; and (c) refuses if the plan-time PWM/encoder frozen in
/// `context` does not match the live `pwm_hz`/`encoder_counts_per_rev`, so
/// confirmation evidence gathered under one physical timebase cannot seed a
/// plan admitted under another. `context` carries the breakaway campaign's own
/// physical acquisition parameters -- there is no reproduced Stage-B sweep plan
/// to read them from.
///
/// Order of checks, all before any plan is constructed: confirmation-plan
/// contract validation; plan-to-token digest binding; plan-time-vs-live
/// physical-context match; authority built from the token;
/// [`transition_capable_stage_c_gain`] for the exact confirmed gain; then the
/// shared tail's own recovery-geometry and duration admission. Emits schema
/// [`super::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION`], using the
/// existing non-combined integral ladder unchanged.
///
/// `confidence_q`/`band_position_q` are carried-through diagnostics only:
/// unlike `held_out_response_intervals_q`, they are excluded from
/// `plan.digest()` and so never affect resume matching.
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn plan_breakaway_velocity_integral_response(
    token: ConfirmedAcquisitionDigest,
    confirmation_plan: BandConfirmationPlan,
    context: AcquisitionContext,
    held_out_response_intervals_q: [[i32; 2]; 2],
    confidence_q: u16,
    band_position_q: u16,
    motion_budget: OuterMotionBudget,
    pwm_hz: u32,
    encoder_counts_per_rev: u32,
) -> Result<VelocityIntegralPlan, VelocityIntegralPlanError> {
    validate_fixed_contract(&confirmation_plan)
        .map_err(|_| VelocityIntegralPlanError::InvalidConfirmationPlan)?;
    // The token is the confirmed plan's digest and can be minted only by
    // `bind_breakaway_stage_c_authority` from a genuine, unforgeable
    // `Ok(BandConfirmationAcceptance)`. Tie the supplied plan to it: a plan
    // whose candidate (or any hashed field) differs has a different digest and
    // is refused, so no forged `BandConfirmationPlan` can be admitted here.
    if confirmation_plan.plan_digest != token.get() {
        return Err(VelocityIntegralPlanError::MismatchedAuthority);
    }
    // Fail-closed physical-context guard: confirmation evidence frozen under one
    // PWM timebase / encoder resolution must not seed a Stage-C plan admitted
    // under different live values.
    if context.plan_time_pwm_hz != pwm_hz
        || context.plan_time_encoder_counts_per_rev != encoder_counts_per_rev
    {
        return Err(VelocityIntegralPlanError::PhysicalContextMismatch);
    }
    // Build the authority INTERNALLY from the token. The gate never accepts a
    // pre-built `StageCAuthority`, so a caller cannot forge one (its fields are
    // `pub`) and pass it in to bypass confirmation.
    let authority = StageCAuthority::new_breakaway(
        context.channel,
        token,
        held_out_response_intervals_q,
        confidence_q,
        band_position_q,
    );
    let fixed_p = confirmation_plan.candidate_p.raw();
    if !transition_capable_stage_c_gain(
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
            final_p: fixed_p,
            schema_revision: super::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION,
            probe_constrained_test_point: false,
        },
        context,
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    )
}

/// Rebuild one breakaway Stage-C plan from authority firmware already retained.
///
/// The resume counterpart to [`plan_breakaway_velocity_integral_response`]. It
/// takes the same unforgeable [`ConfirmedAcquisitionDigest`], so a gain still
/// cannot reach Stage C without a genuine accepted confirmation; the token here
/// comes from the retention slot, which only a plan that already passed that
/// gate can write.
///
/// It omits the confirmation-plan contract validation and the plan-to-token
/// digest binding, because both admitted this gain when the campaign ran and
/// neither has an input at resume time -- the confirmation plan is not retained.
/// Everything that bounds *this* run is re-checked live: the physical-context
/// guard, the transition-capable test for the confirmed gain, and the shared
/// tail's recovery-geometry and duration admission.
///
/// The caller must still compare the resulting plan digest against the retained
/// identity. That comparison is what proves the request reproduces the retained
/// plan, and it replaces the Stage-B digest check the reproduced-Stage-B path
/// uses, which has no meaning here.
///
/// `held_out_response_intervals_q` must be the exact value
/// [`StageCAuthority::directional_pooled_intervals`] carried on the plan being
/// resumed -- callers read it back from the retained
/// [`super::integral_reproduction::StageCAuthoritySummary`], which
/// `StageCAuthoritySummary::from_plan` captured from the original accepted
/// plan. `plan.digest()` folds this field in, so any other value -- including
/// the unavailable-value zero this parameter's predecessor hardcoded --
/// rebuilds a plan whose digest can never match the retained identity, and
/// the resume is refused as a mismatch even though nothing about the request
/// actually changed.
///
/// `confidence_q`/`band_position_q` are carried-through diagnostics only:
/// unlike `held_out_response_intervals_q` above, they are excluded from
/// `plan.digest()` and so never affect resume matching.
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn plan_retained_breakaway_velocity_integral_response(
    token: ConfirmedAcquisitionDigest,
    final_p: u16,
    context: AcquisitionContext,
    held_out_response_intervals_q: [[i32; 2]; 2],
    confidence_q: u16,
    band_position_q: u16,
    motion_budget: OuterMotionBudget,
    pwm_hz: u32,
    encoder_counts_per_rev: u32,
) -> Result<VelocityIntegralPlan, VelocityIntegralPlanError> {
    if context.plan_time_pwm_hz != pwm_hz
        || context.plan_time_encoder_counts_per_rev != encoder_counts_per_rev
    {
        return Err(VelocityIntegralPlanError::PhysicalContextMismatch);
    }
    let authority = StageCAuthority::new_breakaway(
        context.channel,
        token,
        held_out_response_intervals_q,
        confidence_q,
        band_position_q,
    );
    if !transition_capable_stage_c_gain(
        final_p,
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
            final_p,
            schema_revision: super::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION,
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
    authority: StageCAuthority,
    final_p: u16,
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
        .checked_mul(STAGE_C_OBSERVATIONS_PER_RUNG as u16)
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
    let mut positive_i = [0; STAGE_C_MAX_POSITIVE_RUNGS];
    positive_i[..ladder.rungs.len()].copy_from_slice(ladder.rungs.as_slice());
    let recovery_quantization_exposed = recovery_quantization_exposed(
        source.final_p,
        context.target_velocity_rpm,
        encoder_counts_per_rev,
        pwm_hz,
        context.usable_torque_limit,
    )?;

    Ok(VelocityIntegralPlan {
        slot_order: IntegralSlotOrder::ForwardFirst,
        build_features: BUILD_FEATURES,
        schema_revision: source.schema_revision,
        authority: source.authority,
        final_p: source.final_p,
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
