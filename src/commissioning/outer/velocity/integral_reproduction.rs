//! RAM-only Stage-C authority, candidate retention, and curve reproduction.

use super::integral_evidence::{ExactRateInterval, IntegralRunEvidence, StageCEvidenceError};
use super::integral_plan::{STAGE_C_MAX_POSITIVE_RUNGS, VelocityIntegralExecutionPlan};
use super::reproduction::StageBDirectionalValidity;

const DIRECTION_COUNT: usize = 2;

/// Compact report-only reference to one positive-I rest boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCRestBoundaryReference {
    /// Zero-based positive-I rung index, excluding the opening anchor.
    pub positive_rung_index: u8,
    /// Fixed F/R observation slot that reached the boundary.
    pub slot: u8,
}

/// Pre-motion identity for one exact Stage-C plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCPlanIdentity {
    /// Physical firmware channel.
    pub channel: u8,
    /// Shared-firmware package revision.
    pub build_revision: u32,
    /// Stage-C evidence schema revision.
    pub schema_revision: u16,
    /// Exact digest of Stage-C authority, geometry, durations, and safety identity.
    pub plan_digest: u64,
}

/// Compact Stage-B-derived authority retained across Stage-C commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCAuthoritySummary {
    /// Exact pre-motion identity.
    pub identity: StageCPlanIdentity,
    /// Stage-B plan that produced the reproduced handoff.
    pub stage_b_plan_digest: u64,
    /// Firmware-authoritative fixed proportional gain.
    pub final_p: u16,
    /// Cross-run forward and reverse model-or-fragment intersections.
    pub directional_memberships: [u32; DIRECTION_COUNT],
    /// Selected reproduced Stage-B joint-operability component.
    pub joint_membership: u32,
    /// Cross-run validity of the forward and reverse Stage-B models.
    pub directional_validity: [StageBDirectionalValidity; DIRECTION_COUNT],
    /// Whether the selected joint component contains fewer than three rungs.
    pub reduced_margin: bool,
    /// Reproduced pooled intervals, or `[0, 0]` where model comparison is unavailable.
    pub directional_pooled_intervals: [[i32; 2]; DIRECTION_COUNT],
    /// Frozen encoder-domain amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen signed-target magnitude.
    pub target_velocity_rpm: i32,
    /// Frozen PWM frequency defining every I time constant.
    pub pwm_hz: u32,
    /// Mechanical encoder quadrature counts per revolution.
    pub encoder_counts_per_rev: u32,
    /// Number of positive-I rungs.
    pub positive_rung_count: u8,
    /// Simultaneous interval family size per direction.
    pub family_size: u16,
    /// Conservative Stage-C schedule maximum.
    pub maximum_workflow_ms: u32,
}

impl StageCAuthoritySummary {
    /// Compact one emitted Stage-C execution plan into retained authority.
    pub fn from_plan(
        channel: u8,
        build_revision: u32,
        schema_revision: u16,
        stage_b_plan_digest: u64,
        plan: VelocityIntegralExecutionPlan,
        authority: super::integral_plan::StageCAuthority,
    ) -> Self {
        Self {
            identity: StageCPlanIdentity {
                channel,
                build_revision,
                schema_revision,
                plan_digest: plan.plan_digest,
            },
            stage_b_plan_digest,
            final_p: plan.final_p,
            directional_memberships: authority.directional_memberships,
            joint_membership: authority.joint_membership,
            directional_validity: authority.directional_validity,
            reduced_margin: authority.reduced_margin,
            directional_pooled_intervals: authority.directional_pooled_intervals,
            planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
            target_velocity_rpm: plan.target_velocity_rpm,
            pwm_hz: plan.pwm_hz,
            encoder_counts_per_rev: plan.encoder_counts_per_rev,
            positive_rung_count: plan.positive_rung_count,
            family_size: plan.family_size,
            maximum_workflow_ms: plan.maximum_workflow_ms,
        }
    }

    /// Rebuild the exact Stage-B handoff used by Stage-C planning.
    pub const fn stage_c_authority(self) -> super::integral_plan::StageCAuthority {
        super::integral_plan::StageCAuthority {
            channel: self.identity.channel,
            stage_b_plan_digest: self.stage_b_plan_digest,
            final_p: self.final_p,
            directional_memberships: self.directional_memberships,
            joint_membership: self.joint_membership,
            directional_validity: self.directional_validity,
            reduced_margin: self.reduced_margin,
            directional_pooled_intervals: self.directional_pooled_intervals,
        }
    }

    /// Whether one direction carries a reproduced cross-P model interval.
    pub const fn model_interval_available(&self, direction: usize) -> bool {
        self.stage_c_authority().model_interval_available(direction)
    }
}

/// One sufficient raw curve retained for a separately commanded repeat.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCCandidateSummary {
    /// Authority shared by both runs in a valid pair.
    pub authority: StageCAuthoritySummary,
    /// Exact unmodified anchor and positive-rung intervals.
    pub evidence: IntegralRunEvidence,
    /// Optional response-boundary location; never part of reproduction authority.
    pub rest_boundary: Option<StageCRestBoundaryReference>,
    /// Run-completion time retained as non-gating context.
    pub run_completed_us: u64,
    /// Exact digest of the emitted run evidence.
    pub run_digest: u64,
}

/// Externally visible lifecycle state of the Stage-C RAM slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StageCSlotState {
    /// No Stage-B handoff authority is retained.
    Empty,
    /// Stage-B authority exists but no sufficient Stage-C curve is retained.
    AuthorityOnly,
    /// Authority and one sufficient Stage-C curve await reproduction.
    Candidate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
// Fixed inline storage is required because firmware has no heap. The exact
// slot remains below its explicit 640-byte ceiling.
#[allow(clippy::large_enum_variant)]
enum RetainedStageC {
    Empty,
    AuthorityOnly(StageCAuthoritySummary),
    Candidate(StageCCandidateSummary),
}

/// Exact per-direction masks and interval comparisons for one run pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCReproductionDelta {
    /// Exact retained-run curve used to derive every comparison interval.
    pub previous_evidence: IntegralRunEvidence,
    /// Retained-run response boundary, when present.
    pub previous_rest_boundary: Option<StageCRestBoundaryReference>,
    /// Current-run response boundary, when present.
    pub current_rest_boundary: Option<StageCRestBoundaryReference>,
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

impl StageCReproductionDelta {
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
const _: () = assert!(core::mem::size_of::<StageCReproductionDelta>() <= 600);

/// Firmware-owned outcome of one Stage-C retained-state transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StageCReproductionDecision {
    /// First sufficient curve retained for a separately commanded repeat.
    CompleteCandidate(StageCCandidateSummary),
    /// Second sufficient curve reproduced at least three points per direction.
    Complete(StageCReproductionDelta),
    /// Requested plan differs; no motion begins and retention is unchanged.
    RejectedPlanMismatch {
        /// Existing retained identity.
        retained: StageCPlanIdentity,
        /// Newly requested identity.
        requested: StageCPlanIdentity,
    },
    /// Sufficient evidence did not reproduce and replaced the retained curve.
    InconclusiveReproductionMismatch(StageCReproductionDelta),
    /// Retained or live evidence violated an exact structural invariant.
    EvidenceIntegrityFault,
}

/// One RAM-only Stage-C authority/candidate slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCCandidateSlot {
    retained: RetainedStageC,
}

impl StageCCandidateSlot {
    /// Construct an empty slot. MCU restart therefore clears both authority and evidence.
    pub const fn new() -> Self {
        Self {
            retained: RetainedStageC::Empty,
        }
    }

    /// Return the current lifecycle state without exposing enum payload layout.
    pub const fn state(&self) -> StageCSlotState {
        match self.retained {
            RetainedStageC::Empty => StageCSlotState::Empty,
            RetainedStageC::AuthorityOnly(_) => StageCSlotState::AuthorityOnly,
            RetainedStageC::Candidate(_) => StageCSlotState::Candidate,
        }
    }

    /// Return retained authority from either nonempty state.
    pub const fn authority(&self) -> Option<StageCAuthoritySummary> {
        match self.retained {
            RetainedStageC::Empty => None,
            RetainedStageC::AuthorityOnly(authority) => Some(authority),
            RetainedStageC::Candidate(candidate) => Some(candidate.authority),
        }
    }

    /// Return the retained sufficient curve, if present.
    pub const fn candidate(&self) -> Option<StageCCandidateSummary> {
        match self.retained {
            RetainedStageC::Candidate(candidate) => Some(candidate),
            RetainedStageC::Empty | RetainedStageC::AuthorityOnly(_) => None,
        }
    }

    /// Install newly reproduced Stage-B handoff authority.
    pub fn install_authority(&mut self, authority: StageCAuthoritySummary) {
        self.retained = RetainedStageC::AuthorityOnly(authority);
    }

    /// Reject a changed plan before actuation without modifying retention.
    pub fn preflight(&self, requested: StageCPlanIdentity) -> Option<StageCReproductionDecision> {
        let retained = self.authority()?.identity;
        (retained != requested).then_some(StageCReproductionDecision::RejectedPlanMismatch {
            retained,
            requested,
        })
    }

    /// Retain or compare one independently sufficient Stage-C run.
    pub fn observe_sufficient(
        &mut self,
        current: StageCCandidateSummary,
    ) -> StageCReproductionDecision {
        let Some(authority) = self.authority() else {
            return StageCReproductionDecision::EvidenceIntegrityFault;
        };
        if authority != current.authority {
            return StageCReproductionDecision::RejectedPlanMismatch {
                retained: authority.identity,
                requested: current.authority.identity,
            };
        }
        let Some(previous) = self.candidate() else {
            self.retained = RetainedStageC::Candidate(current);
            return StageCReproductionDecision::CompleteCandidate(current);
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
                return StageCReproductionDecision::EvidenceIntegrityFault;
            }
        };
        if delta.reproduces() {
            self.observe_fault();
            StageCReproductionDecision::Complete(delta)
        } else {
            self.retained = RetainedStageC::Candidate(current);
            StageCReproductionDecision::InconclusiveReproductionMismatch(delta)
        }
    }

    /// Discard candidate evidence after a safe Inconclusive while preserving authority.
    pub fn observe_inconclusive(&mut self) {
        if let RetainedStageC::Candidate(candidate) = self.retained {
            self.retained = RetainedStageC::AuthorityOnly(candidate.authority);
        }
    }

    /// Clear authority and evidence after a firmware safety or integrity Fault.
    pub fn observe_fault(&mut self) {
        self.retained = RetainedStageC::Empty;
    }

    /// A host-side error does not rewrite a completed firmware terminal or slot state.
    pub const fn observe_host_error(&mut self) {}

    /// Same-channel explicit homing calibration preserves retained Stage-C state.
    pub fn preserve_for_same_channel_calibration(&mut self, channel: u8) {
        self.invalidate_for_channel(channel);
    }

    /// Invalidate authority and evidence when the active channel changes.
    pub fn invalidate_for_channel(&mut self, channel: u8) {
        if self
            .authority()
            .is_some_and(|authority| authority.identity.channel != channel)
        {
            self.observe_fault();
        }
    }

    /// Invalidate authority and evidence before unrelated commissioning begins.
    pub fn invalidate_for_unrelated_commissioning(&mut self) {
        self.observe_fault();
    }
}

impl Default for StageCCandidateSlot {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<StageCCandidateSlot>() <= 640);

/// Derive one anchor-relative response interval with checked outward subtraction.
pub fn delta_e(
    rung: ExactRateInterval,
    anchor: ExactRateInterval,
) -> Result<ExactRateInterval, StageCEvidenceError> {
    let low = i64::from(rung.low_q) - i64::from(anchor.high_q);
    let high = i64::from(rung.high_q) - i64::from(anchor.low_q);
    ExactRateInterval::new(
        i32::try_from(low).map_err(|_| StageCEvidenceError::Conversion)?,
        i32::try_from(high).map_err(|_| StageCEvidenceError::Conversion)?,
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
) -> Result<i32, StageCEvidenceError> {
    let gap = if current.low_q > previous.high_q {
        i64::from(current.low_q) - i64::from(previous.high_q)
    } else if current.high_q < previous.low_q {
        i64::from(current.high_q) - i64::from(previous.low_q)
    } else {
        0
    };
    i32::try_from(gap).map_err(|_| StageCEvidenceError::Conversion)
}

/// Compare two sufficient raw curves without contiguity or cross-direction gates.
pub fn compare_integral_runs(
    previous: IntegralRunEvidence,
    current: IntegralRunEvidence,
    positive_rung_count: usize,
) -> Result<StageCReproductionDelta, StageCEvidenceError> {
    compare_integral_runs_with_boundaries(previous, current, positive_rung_count, None, None)
}

fn compare_integral_runs_with_boundaries(
    previous: IntegralRunEvidence,
    current: IntegralRunEvidence,
    positive_rung_count: usize,
    previous_rest_boundary: Option<StageCRestBoundaryReference>,
    current_rest_boundary: Option<StageCRestBoundaryReference>,
) -> Result<StageCReproductionDelta, StageCEvidenceError> {
    if positive_rung_count == 0 || positive_rung_count > STAGE_C_MAX_POSITIVE_RUNGS {
        return Err(StageCEvidenceError::MaskOutOfRange);
    }
    let valid_mask = (1_u32 << positive_rung_count) - 1;
    if previous
        .eligible_masks
        .iter()
        .chain(current.eligible_masks.iter())
        .any(|mask| mask & !valid_mask != 0)
    {
        return Err(StageCEvidenceError::MaskOutOfRange);
    }

    let mut result = StageCReproductionDelta {
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
