//! RAM-only Integral authority, candidate retention, and curve reproduction.

use super::integral_evidence::{ExactRateInterval, IntegralEvidenceError, IntegralRunEvidence};
use super::integral_plan::{INTEGRAL_MAX_POSITIVE_RUNGS, VelocityIntegralExecutionPlan};
use super::reproduction::ProportionalDirectionalValidity;

const DIRECTION_COUNT: usize = 2;

/// Compact report-only reference to one positive-I rest boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralRestBoundaryReference {
    /// Zero-based positive-I rung index, excluding the opening anchor.
    pub positive_rung_index: u8,
    /// Fixed F/R observation slot that reached the boundary.
    pub slot: u8,
}

/// Pre-motion identity for one exact Integral plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralPlanIdentity {
    /// Physical firmware channel.
    pub channel: u8,
    /// Analysis-rule contract this plan was built under.
    pub build_features: u16,
    /// Integral evidence schema revision.
    pub schema_revision: u16,
    /// Exact digest of Integral authority, geometry, durations, and safety identity.
    pub plan_digest: u64,
}

/// Compact Proportional-derived authority retained across Integral commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralAuthoritySummary {
    /// Exact pre-motion identity.
    pub identity: IntegralPlanIdentity,
    /// Which acquisition authorized this Integral plan, and its exact digest.
    pub acquisition: super::integral_plan::ConfirmedAcquisitionDigest,
    /// Firmware-authoritative fixed proportional gain.
    pub fixed_p: u16,
    /// Cross-run forward and reverse model-or-fragment intersections.
    pub directional_memberships: [u32; DIRECTION_COUNT],
    /// Reproduced Proportional joint-operability component before Integral filtering.
    pub proportional_joint_membership: u32,
    /// Integral transition-capable subset of the reproduced joint component.
    pub joint_membership: u32,
    /// Cross-run validity of the forward and reverse Proportional models.
    pub directional_validity: [ProportionalDirectionalValidity; DIRECTION_COUNT],
    /// Whether the selected joint component contains fewer than three rungs.
    pub reduced_margin: bool,
    /// Reproduced pooled intervals, or selected response intervals for a combined plan.
    pub directional_pooled_intervals: [[i32; 2]; DIRECTION_COUNT],
    /// Held-out confirmation confidence in Q0.16, or `0` off the breakaway path.
    pub confidence_q: u16,
    /// Held-out confirmation band position in Q0.16, or `0` off the breakaway path.
    pub band_position_q: u16,
    /// Frozen encoder-domain amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen signed-target magnitude.
    pub target_velocity_rpm: i32,
    /// Frozen PWM frequency defining every I time constant.
    pub pwm_hz: u32,
    /// Mechanical encoder quadrature counts per revolution.
    pub encoder_counts_per_rev: u32,
    /// Hard torque-target limit the plan was built under.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary the plan was built under.
    ///
    /// Retained rather than recomputed because the breakaway campaign measures
    /// it from the live current observer's clamp limit, so nothing derives it
    /// from configuration at resume time.
    pub usable_torque_limit: u16,
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: super::velocity_clamp::VelocityLimitEvidenceCapability,
    /// Number of positive-I rungs.
    pub positive_rung_count: u8,
    /// Simultaneous interval family size per direction.
    pub family_size: u16,
    /// Conservative Integral schedule maximum.
    pub maximum_workflow_ms: u32,
    /// The commissioning probe clamp removed at least one Proportional-operable rung.
    pub probe_constrained_test_point: bool,
}

impl IntegralAuthoritySummary {
    /// Compact one emitted Integral execution plan into retained authority.
    pub fn from_plan(
        channel: u8,
        build_features: u16,
        schema_revision: u16,
        acquisition: super::integral_plan::ConfirmedAcquisitionDigest,
        plan: VelocityIntegralExecutionPlan,
        authority: super::integral_plan::IntegralAcquisitionAuthority,
    ) -> Self {
        Self {
            identity: IntegralPlanIdentity {
                channel,
                build_features,
                schema_revision,
                plan_digest: plan.plan_digest,
            },
            acquisition,
            fixed_p: plan.fixed_p,
            directional_memberships: authority.directional_memberships,
            proportional_joint_membership: authority.proportional_joint_membership,
            joint_membership: authority.joint_membership,
            directional_validity: authority.directional_validity,
            reduced_margin: authority.reduced_margin,
            directional_pooled_intervals: authority.directional_pooled_intervals,
            confidence_q: authority.confidence_q,
            band_position_q: authority.band_position_q,
            planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
            target_velocity_rpm: plan.target_velocity_rpm,
            pwm_hz: plan.pwm_hz,
            encoder_counts_per_rev: plan.encoder_counts_per_rev,
            hard_torque_limit: plan.hard_torque_limit,
            usable_torque_limit: plan.usable_torque_limit,
            limit_evidence_capability: plan.limit_evidence_capability,
            positive_rung_count: plan.positive_rung_count,
            family_size: plan.family_size,
            maximum_workflow_ms: plan.maximum_workflow_ms,
            probe_constrained_test_point: plan.probe_constrained_test_point,
        }
    }

    /// Rebuild the exact Proportional handoff used by Integral planning.
    pub const fn integral_authority(self) -> super::integral_plan::IntegralAcquisitionAuthority {
        super::integral_plan::IntegralAcquisitionAuthority {
            channel: self.identity.channel,
            acquisition: self.acquisition,
            directional_memberships: self.directional_memberships,
            proportional_joint_membership: self.proportional_joint_membership,
            joint_membership: self.joint_membership,
            directional_validity: self.directional_validity,
            reduced_margin: self.reduced_margin,
            directional_pooled_intervals: self.directional_pooled_intervals,
            confidence_q: self.confidence_q,
            band_position_q: self.band_position_q,
        }
    }

    /// Whether one direction carries a reproduced cross-P model interval.
    pub const fn model_interval_available(&self, direction: usize) -> bool {
        self.integral_authority()
            .model_interval_available(direction)
    }
}

/// One sufficient raw curve retained for a separately commanded repeat.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RetainedIntegralEvidence {
    /// Authority shared by both runs in a valid pair.
    pub authority: IntegralAuthoritySummary,
    /// Exact unmodified anchor and positive-rung intervals.
    pub evidence: IntegralRunEvidence,
    /// Optional response-boundary location; never part of reproduction authority.
    pub rest_boundary: Option<IntegralRestBoundaryReference>,
    /// One-based breakaway target-terminus rung, or `0` when this run's own
    /// sweep did not record one.
    ///
    /// Distinct from `rest_boundary`: a breakaway target terminus is encoded
    /// on the wire as `rest_boundary_rung_plus_one` alone, with
    /// `rest_boundary_slot_plus_one == 0` (see `terminal_target_terminus`),
    /// which `rest_boundary`'s pairwise decode cannot represent. Retained so a
    /// reproduced `Complete` promotion can fall back to the terminus already
    /// proven by the first sufficient run when the confirmation run's own
    /// terminus check did not re-trip.
    pub target_terminus_rung_plus_one: u8,
    /// Run-completion time retained as non-gating context.
    pub run_completed_us: u64,
    /// Exact digest of the emitted run evidence.
    pub run_digest: u64,
}

/// Next amplitude acquisition allowed by one retained acceptance point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FixedGainAmplitudeProgress {
    /// The ascending amplitude is the next allowed acquisition.
    ReadyAscending = 0,
    /// The ascending amplitude completed and the descending amplitude is next.
    ReadyDescending = 1,
    /// The fixed two-run amplitude sequence is closed.
    Closed = 2,
}

/// Compact RAM-only handoff from one target-reaching breakaway run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AcceptancePoint {
    /// Exact breakaway Integral identity that established the point.
    pub identity: IntegralPlanIdentity,
    /// Firmware-selected proportional gain.
    pub selected_p: u16,
    /// First raw Q4.12 integral gain whose response reached the target.
    pub selected_i: u16,
    /// Exact signed TMC mechanical-RPM target used by the breakaway run.
    pub target_velocity_rpm: i32,
    /// Exact compact response intervals needed to reconstruct the accepted plan identity.
    pub selected_response_intervals_q: [[i32; 2]; 2],
    /// Held-out confirmation confidence in Q0.16, or `0` off the breakaway path.
    pub confidence_q: u16,
    /// Held-out confirmation band position in Q0.16, or `0` off the breakaway path.
    pub band_position_q: u16,
    /// Packed operating-point, model-validity, and evidence-quality qualifiers.
    pub qualifier_bits: u16,
    /// Unforgeable breakaway confirmation token that authorized this point.
    /// A breakaway plan cannot be reconstructed from the request alone, so the
    /// amplitude preflight rebuilds it deterministically from this token.
    pub token: Option<super::integral_plan::ConfirmedAcquisitionDigest>,
    /// Next allowed amplitude acquisition.
    pub progress: FixedGainAmplitudeProgress,
}

/// Externally visible lifecycle state of the Integral RAM slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralRetentionSlotState {
    /// No Proportional handoff authority is retained.
    Empty,
    /// Proportional authority exists but no sufficient Integral curve is retained.
    AuthorityOnly,
    /// Authority and one sufficient Integral curve await reproduction.
    Candidate,
    /// One target-reaching breakaway point awaits its fixed amplitude sequence.
    AcceptancePoint,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
// Fixed inline storage is required because firmware has no heap. The exact
// slot remains below its explicit 648-byte ceiling.
#[allow(clippy::large_enum_variant)]
enum RetainedIntegral {
    Empty,
    AuthorityOnly(IntegralAuthoritySummary),
    Candidate(RetainedIntegralEvidence),
    AcceptancePoint(AcceptancePoint),
}

/// Exact per-direction masks and interval comparisons for one run pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralReproductionDelta {
    /// Exact retained-run curve used to derive every comparison interval.
    pub previous_evidence: IntegralRunEvidence,
    /// Retained-run response boundary, when present.
    pub previous_rest_boundary: Option<IntegralRestBoundaryReference>,
    /// Current-run response boundary, when present.
    pub current_rest_boundary: Option<IntegralRestBoundaryReference>,
    /// Retained-run positive-I membership.
    pub previous_masks: [u32; DIRECTION_COUNT],
    /// Current-run positive-I membership.
    pub current_masks: [u32; DIRECTION_COUNT],
    /// Membership present in both runs.
    pub shared_masks: [u32; DIRECTION_COUNT],
    /// Shared rungs whose anchor-relative intervals overlap.
    pub reproduced_masks: [u32; DIRECTION_COUNT],
    /// Shared rungs whose anchor-relative intervals are disjoint.
    pub divergent_masks: [u32; DIRECTION_COUNT],
    /// Rungs present only in the retained run.
    pub previous_only_masks: [u32; DIRECTION_COUNT],
    /// Rungs present only in the current run.
    pub current_only_masks: [u32; DIRECTION_COUNT],
}

impl IntegralReproductionDelta {
    /// Whether both directions contain at least three reproduced response points.
    pub fn reproduces(&self) -> bool {
        self.reproduced_masks
            .iter()
            .all(|mask| mask.count_ones() >= 3)
    }
}

// Keep comparison evidence compact: this value is carried through board-facing
// commissioning outcomes, so storing derived per-rung arrays here can exhaust
// the F407 RTIC task stack before any motion begins.
const _: () = assert!(core::mem::size_of::<IntegralReproductionDelta>() <= 600);

/// Firmware-owned outcome of one Integral retained-state transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralReproductionDecision {
    /// First sufficient curve retained for a separately commanded repeat.
    FirstRunRetained(RetainedIntegralEvidence),
    /// Second sufficient curve reproduced at least three points per direction.
    RepeatabilityConfirmed(IntegralReproductionDelta),
    /// Requested plan differs; no motion begins and retention is unchanged.
    RejectedPlanMismatch {
        /// Existing retained identity.
        retained: IntegralPlanIdentity,
        /// Newly requested identity.
        requested: IntegralPlanIdentity,
    },
    /// Sufficient evidence did not reproduce and replaced the retained curve.
    RepeatabilityMismatch(IntegralReproductionDelta),
    /// Retained or live evidence violated an exact structural invariant.
    EvidenceIntegrityFault,
}

/// One RAM-only Integral authority/candidate slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralRetentionSlot {
    retained: RetainedIntegral,
}

impl IntegralRetentionSlot {
    /// Construct an empty slot. MCU restart therefore clears both authority and evidence.
    pub const fn new() -> Self {
        Self {
            retained: RetainedIntegral::Empty,
        }
    }

    /// Return the current lifecycle state without exposing enum payload layout.
    pub const fn state(&self) -> IntegralRetentionSlotState {
        match self.retained {
            RetainedIntegral::Empty => IntegralRetentionSlotState::Empty,
            RetainedIntegral::AuthorityOnly(_) => IntegralRetentionSlotState::AuthorityOnly,
            RetainedIntegral::Candidate(_) => IntegralRetentionSlotState::Candidate,
            RetainedIntegral::AcceptancePoint(_) => IntegralRetentionSlotState::AcceptancePoint,
        }
    }

    /// Return retained authority from either nonempty state.
    pub const fn authority(&self) -> Option<IntegralAuthoritySummary> {
        match self.retained {
            RetainedIntegral::Empty => None,
            RetainedIntegral::AuthorityOnly(authority) => Some(authority),
            RetainedIntegral::Candidate(candidate) => Some(candidate.authority),
            RetainedIntegral::AcceptancePoint(_) => None,
        }
    }

    /// Return the retained sufficient curve, if present.
    pub const fn candidate(&self) -> Option<RetainedIntegralEvidence> {
        match self.retained {
            RetainedIntegral::Candidate(candidate) => Some(candidate),
            RetainedIntegral::Empty
            | RetainedIntegral::AuthorityOnly(_)
            | RetainedIntegral::AcceptancePoint(_) => None,
        }
    }

    /// Return the accepted breakaway operating point, if present.
    pub const fn acceptance_point(&self) -> Option<AcceptancePoint> {
        match self.retained {
            RetainedIntegral::AcceptancePoint(point) => Some(point),
            RetainedIntegral::Empty
            | RetainedIntegral::AuthorityOnly(_)
            | RetainedIntegral::Candidate(_) => None,
        }
    }

    /// Install newly reproduced Proportional handoff authority.
    pub fn install_authority(&mut self, authority: IntegralAuthoritySummary) {
        self.retained = RetainedIntegral::AuthorityOnly(authority);
    }

    /// Install one target-reaching breakaway operating point.
    pub fn install_acceptance_point(&mut self, point: AcceptancePoint) {
        self.retained = RetainedIntegral::AcceptancePoint(point);
    }

    /// Advance a completed ascending amplitude to the descending acquisition.
    pub fn advance_matrix_progress(&mut self) -> bool {
        let RetainedIntegral::AcceptancePoint(mut point) = self.retained else {
            return false;
        };
        if point.progress != FixedGainAmplitudeProgress::ReadyAscending {
            return false;
        }
        point.progress = FixedGainAmplitudeProgress::ReadyDescending;
        self.retained = RetainedIntegral::AcceptancePoint(point);
        true
    }

    /// Close a nonclosed amplitude sequence after its final safe terminal.
    pub fn close_matrix_progress(&mut self) -> bool {
        let RetainedIntegral::AcceptancePoint(mut point) = self.retained else {
            return false;
        };
        if point.progress == FixedGainAmplitudeProgress::Closed {
            return false;
        }
        point.progress = FixedGainAmplitudeProgress::Closed;
        self.retained = RetainedIntegral::AcceptancePoint(point);
        true
    }

    /// Reject a changed plan before actuation without modifying retention.
    pub fn preflight(
        &self,
        requested: IntegralPlanIdentity,
    ) -> Option<IntegralReproductionDecision> {
        let retained = self.authority()?.identity;
        (retained != requested).then_some(IntegralReproductionDecision::RejectedPlanMismatch {
            retained,
            requested,
        })
    }

    /// Retain or compare one independently sufficient Integral run.
    pub fn observe_sufficient(
        &mut self,
        current: RetainedIntegralEvidence,
    ) -> IntegralReproductionDecision {
        let Some(authority) = self.authority() else {
            return IntegralReproductionDecision::EvidenceIntegrityFault;
        };
        if authority != current.authority {
            return IntegralReproductionDecision::RejectedPlanMismatch {
                retained: authority.identity,
                requested: current.authority.identity,
            };
        }
        let Some(previous) = self.candidate() else {
            self.retained = RetainedIntegral::Candidate(current);
            return IntegralReproductionDecision::FirstRunRetained(current);
        };
        let delta = match compare_integral_runs_with_boundaries(
            previous.evidence,
            current.evidence,
            usize::from(authority.positive_rung_count),
            previous.rest_boundary,
            current.rest_boundary,
        ) {
            Ok(delta) => delta,
            Err(_) => {
                self.observe_fault();
                return IntegralReproductionDecision::EvidenceIntegrityFault;
            }
        };
        if delta.reproduces() {
            self.observe_fault();
            IntegralReproductionDecision::RepeatabilityConfirmed(delta)
        } else {
            self.retained = RetainedIntegral::Candidate(current);
            IntegralReproductionDecision::RepeatabilityMismatch(delta)
        }
    }

    /// Discard candidate evidence after a safe Inconclusive while preserving authority.
    pub fn observe_inconclusive(&mut self) {
        if let RetainedIntegral::Candidate(candidate) = self.retained {
            self.retained = RetainedIntegral::AuthorityOnly(candidate.authority);
        }
    }

    /// Clear authority and evidence after a firmware safety or integrity Fault.
    pub fn observe_fault(&mut self) {
        self.retained = RetainedIntegral::Empty;
    }

    /// Same-channel explicit homing calibration preserves retained Integral state.
    pub fn preserve_for_same_channel_calibration(&mut self, channel: u8) {
        self.invalidate_for_channel(channel);
    }

    /// Invalidate authority and evidence when the active channel changes.
    pub fn invalidate_for_channel(&mut self, channel: u8) {
        let retained_channel = match self.retained {
            RetainedIntegral::AuthorityOnly(authority) => Some(authority.identity.channel),
            RetainedIntegral::Candidate(candidate) => Some(candidate.authority.identity.channel),
            RetainedIntegral::AcceptancePoint(point) => Some(point.identity.channel),
            RetainedIntegral::Empty => None,
        };
        if retained_channel.is_some_and(|retained| retained != channel) {
            self.observe_fault();
        }
    }

    /// Invalidate authority and evidence before unrelated commissioning begins.
    pub fn invalidate_for_unrelated_commissioning(&mut self) {
        self.observe_fault();
    }
}

impl Default for IntegralRetentionSlot {
    fn default() -> Self {
        Self::new()
    }
}

/// Channel-indexed RAM-only integral retention slots.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralRetentionSlots<const CHANNELS: usize> {
    slots: [IntegralRetentionSlot; CHANNELS],
}

impl<const CHANNELS: usize> IntegralRetentionSlots<CHANNELS> {
    /// Construct an empty slot for every board-local TMC4671 channel.
    pub const fn new() -> Self {
        Self {
            slots: [IntegralRetentionSlot::new(); CHANNELS],
        }
    }

    /// Return the retention slot for `channel`, or `None` when it is invalid.
    pub fn slot(&self, channel: u8) -> Option<&IntegralRetentionSlot> {
        self.slots.get(usize::from(channel))
    }

    /// Return the mutable retention slot for `channel`, or `None` when it is invalid.
    pub fn slot_mut(&mut self, channel: u8) -> Option<&mut IntegralRetentionSlot> {
        self.slots.get_mut(usize::from(channel))
    }

    /// Clear retention for every channel after a board-wide fault or reset.
    pub fn invalidate_all(&mut self) {
        self.slots
            .iter_mut()
            .for_each(IntegralRetentionSlot::observe_fault);
    }
}

impl<const CHANNELS: usize> Default for IntegralRetentionSlots<CHANNELS> {
    fn default() -> Self {
        Self::new()
    }
}

// The retained slot is fixed inline storage on a target with no heap, so its
// ceiling is enforced rather than assumed.
const _: () = assert!(core::mem::size_of::<IntegralRetentionSlot>() <= 656);

/// Derive one anchor-relative response interval with checked outward subtraction.
pub fn delta_e(
    rung: ExactRateInterval,
    anchor: ExactRateInterval,
) -> Result<ExactRateInterval, IntegralEvidenceError> {
    let low = i64::from(rung.low_q) - i64::from(anchor.high_q);
    let high = i64::from(rung.high_q) - i64::from(anchor.low_q);
    ExactRateInterval::new(
        i32::try_from(low).map_err(|_| IntegralEvidenceError::Conversion)?,
        i32::try_from(high).map_err(|_| IntegralEvidenceError::Conversion)?,
    )
}

/// Return the exact closed overlap, including endpoint touch.
pub const fn closed_intersection(
    left: ExactRateInterval,
    right: ExactRateInterval,
) -> Option<ExactRateInterval> {
    let low_q = if left.low_q > right.low_q {
        left.low_q
    } else {
        right.low_q
    };
    let high_q = if left.high_q < right.high_q {
        left.high_q
    } else {
        right.high_q
    };
    if low_q <= high_q {
        Some(ExactRateInterval { low_q, high_q })
    } else {
        None
    }
}

/// Return positive current-above-retained, negative current-below-retained, or zero overlap.
pub fn signed_divergence_gap(
    previous: ExactRateInterval,
    current: ExactRateInterval,
) -> Result<i32, IntegralEvidenceError> {
    let gap = if current.low_q > previous.high_q {
        i64::from(current.low_q) - i64::from(previous.high_q)
    } else if current.high_q < previous.low_q {
        i64::from(current.high_q) - i64::from(previous.low_q)
    } else {
        0
    };
    i32::try_from(gap).map_err(|_| IntegralEvidenceError::Conversion)
}

/// Compare two sufficient raw curves without contiguity or cross-direction gates.
pub(super) fn compare_integral_runs_with_boundaries(
    previous: IntegralRunEvidence,
    current: IntegralRunEvidence,
    positive_rung_count: usize,
    previous_rest_boundary: Option<IntegralRestBoundaryReference>,
    current_rest_boundary: Option<IntegralRestBoundaryReference>,
) -> Result<IntegralReproductionDelta, IntegralEvidenceError> {
    if positive_rung_count == 0 || positive_rung_count > INTEGRAL_MAX_POSITIVE_RUNGS {
        return Err(IntegralEvidenceError::MaskOutOfRange);
    }
    let valid_mask = (1_u32 << positive_rung_count) - 1;
    if previous
        .eligible_masks
        .iter()
        .chain(current.eligible_masks.iter())
        .any(|mask| mask & !valid_mask != 0)
    {
        return Err(IntegralEvidenceError::MaskOutOfRange);
    }

    let mut result = IntegralReproductionDelta {
        previous_evidence: previous,
        previous_rest_boundary,
        current_rest_boundary,
        previous_masks: previous.eligible_masks,
        current_masks: current.eligible_masks,
        shared_masks: [0; DIRECTION_COUNT],
        reproduced_masks: [0; DIRECTION_COUNT],
        divergent_masks: [0; DIRECTION_COUNT],
        previous_only_masks: [0; DIRECTION_COUNT],
        current_only_masks: [0; DIRECTION_COUNT],
    };
    for direction in 0..DIRECTION_COUNT {
        result.shared_masks[direction] =
            previous.eligible_masks[direction] & current.eligible_masks[direction];
        result.previous_only_masks[direction] =
            previous.eligible_masks[direction] & !current.eligible_masks[direction] & valid_mask;
        result.current_only_masks[direction] =
            current.eligible_masks[direction] & !previous.eligible_masks[direction] & valid_mask;
        for rung in 0..positive_rung_count {
            let bit = 1_u32 << rung;
            if result.shared_masks[direction] & bit == 0 {
                continue;
            }
            let previous_delta = delta_e(
                previous.positive[direction][rung],
                previous.opening[direction],
            )?;
            let current_delta = delta_e(
                current.positive[direction][rung],
                current.opening[direction],
            )?;
            if closed_intersection(previous_delta, current_delta).is_some() {
                result.reproduced_masks[direction] |= bit;
            } else {
                result.divergent_masks[direction] |= bit;
                signed_divergence_gap(previous_delta, current_delta)?;
            }
        }
    }
    Ok(result)
}
