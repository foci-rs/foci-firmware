use crate::commissioning::outer::velocity::operating_region::CoverageKind;
use crate::commissioning::outer::velocity::sweep_plan::{MAX_SWEEP_RUNGS, gain_at};

/// Evidence schema stored in the RAM-only Stage B reproduction slot.
pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u16 = 6;

/// Forward shared model core contains fewer than three rungs.
pub const STAGE_B_REPRO_REASON_FORWARD_CORE_TOO_SMALL: u8 = 1 << 0;
/// Reverse shared model core contains fewer than three rungs.
pub const STAGE_B_REPRO_REASON_REVERSE_CORE_TOO_SMALL: u8 = 1 << 1;
/// The retained and current joint-operability regions do not overlap.
pub const STAGE_B_REPRO_REASON_JOINT_CORE_EMPTY: u8 = 1 << 2;
/// Forward per-run pooled intervals do not overlap.
pub const STAGE_B_REPRO_REASON_FORWARD_POOLED_DISJOINT: u8 = 1 << 3;
/// Reverse per-run pooled intervals do not overlap.
pub const STAGE_B_REPRO_REASON_REVERSE_POOLED_DISJOINT: u8 = 1 << 4;

/// Shared-firmware package revision stored in the RAM-only Stage B slot.
pub const STAGE_B_BUILD_REVISION: u32 = const_fnv1a32(env!("CARGO_PKG_VERSION").as_bytes());

const fn const_fnv1a32(bytes: &[u8]) -> u32 {
    let mut hash = 0x811c_9dc5_u32;
    let mut index = 0;
    while index < bytes.len() {
        hash ^= bytes[index] as u32;
        hash = hash.wrapping_mul(0x0100_0193);
        index += 1;
    }
    hash
}

/// Pre-motion identity for one frozen Stage B plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBPlanIdentity {
    /// Physical firmware channel.
    pub channel: u8,
    /// Shared-firmware package build revision.
    pub build_revision: u32,
    /// Stage B evidence schema revision.
    pub schema_revision: u16,
    /// Exact digest of the frozen motion and analysis plan.
    pub plan_digest: u64,
}

/// Lossless sufficient-run summary retained across separately commanded runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBCandidateSummary {
    /// Frozen-plan identity checked before motion.
    pub identity: StageBPlanIdentity,
    /// Selected forward directional-region membership.
    pub forward_membership: u32,
    /// Selected reverse directional-region membership.
    pub reverse_membership: u32,
    /// Selected joint-operability membership.
    pub joint_membership: u32,
    /// Firmware-nominated provisional proportional gain.
    pub nominated_p: u16,
    /// Structural ladder anchor.
    pub p_start: u16,
    /// Inclusive structural ladder top.
    pub p_top: u16,
    /// Number of post-dedupe ladder rungs.
    pub rung_count: u8,
    /// Forward pooled equivalent-disturbance interval.
    pub forward_interval: [i32; 2],
    /// Reverse pooled equivalent-disturbance interval.
    pub reverse_interval: [i32; 2],
    /// Forward full-region common disturbance interval.
    pub forward_common_interval: [i32; 2],
    /// Reverse full-region common disturbance interval.
    pub reverse_common_interval: [i32; 2],
    /// Run-completion timestamp retained as non-gating context.
    pub run_completed_us: u64,
    /// Exact digest of the emitted run evidence.
    pub run_digest: u64,
}

/// Exact membership intersection and boundary movement for one reproduced object.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBMembershipComparison {
    /// Membership from the retained run.
    pub previous: u32,
    /// Membership from the current run.
    pub current: u32,
    /// Exact cross-run membership intersection.
    pub core: u32,
    /// Rungs present only in the retained run.
    pub previous_only: u32,
    /// Rungs present only in the current run.
    pub current_only: u32,
    /// Current low edge minus retained low edge.
    pub low_delta: i8,
    /// Current high edge minus retained high edge.
    pub high_delta: i8,
}

/// Exact pooled and common interval evidence for one direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBIntervalComparison {
    /// Pooled interval from the retained run.
    pub previous_pooled: [i32; 2],
    /// Pooled interval from the current run.
    pub current_pooled: [i32; 2],
    /// Exact closed pooled-interval overlap.
    pub pooled_overlap: [i32; 2],
    /// Full-region common interval from the retained run.
    pub previous_common: [i32; 2],
    /// Full-region common interval from the current run.
    pub current_common: [i32; 2],
    /// Conservative intersection of both full-region common intervals.
    pub conservative_common: [i32; 2],
    /// Whether the conservative common intersection is nonempty.
    pub conservative_common_nonempty: bool,
}

/// Recomputed relationship between one directional core and the final P.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBCoreCoverage {
    /// Direct or transferred coverage.
    pub kind: CoverageKind,
    /// Signed nearest-core-edge distance from the final rung.
    pub signed_rung_distance: i8,
    /// Larger gain in the source/final ratio.
    pub gain_ratio_num: u16,
    /// Smaller gain in the source/final ratio.
    pub gain_ratio_den: u16,
}

impl StageBCoreCoverage {
    const EMPTY: Self = Self {
        kind: CoverageKind::TransferredCoverage,
        signed_rung_distance: 0,
        gain_ratio_num: 0,
        gain_ratio_den: 0,
    };
}

/// Exact evidence delta reported for a completed or mismatched pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBReproductionDelta {
    /// Forward, reverse, and joint membership comparisons.
    pub memberships: [StageBMembershipComparison; 3],
    /// Forward and reverse interval comparisons.
    pub intervals: [StageBIntervalComparison; 2],
    /// Retained provisional P.
    pub previous_provisional_p: u16,
    /// Current provisional P.
    pub current_provisional_p: u16,
    /// Final P derived from the shared joint core.
    pub final_p: u16,
    /// Forward and reverse coverage recomputed against the final P.
    pub coverage: [StageBCoreCoverage; 2],
    /// A one- or two-rung joint core carries reduced I-ladder margin.
    pub reduced_margin: bool,
    /// Independent reproduction mismatch reasons.
    pub reason_mask: u8,
    /// Retained run digest.
    pub previous_run_digest: u64,
    /// Current run digest.
    pub current_run_digest: u64,
}

impl StageBReproductionDelta {
    /// Encode direct directional coverage as the terminal's established bit mask.
    pub fn coverage_mask(self) -> u8 {
        u8::from(matches!(self.coverage[0].kind, CoverageKind::FullCoverage))
            | (u8::from(matches!(self.coverage[1].kind, CoverageKind::FullCoverage)) << 1)
    }
}

/// Board-owned outcome of one Stage B reproduction transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StageBReproductionDecision {
    /// First sufficient run retained for a separately commanded repeat.
    CompleteCandidate(StageBCandidateSummary),
    /// Second sufficient run reproduced all required evidence.
    Complete(StageBReproductionDelta),
    /// Frozen plan differs, so motion must not begin and retention is unchanged.
    RejectedPlanMismatch {
        /// Existing retained identity.
        retained: StageBPlanIdentity,
        /// Newly requested identity.
        requested: StageBPlanIdentity,
    },
    /// Sufficient evidence did not reproduce and replaced the retained run.
    InconclusiveReproductionMismatch(StageBReproductionDelta),
    /// Selected or core membership evidence violated structural invariants.
    EvidenceIntegrityFault,
    /// Retained evidence was invalidated.
    Cleared,
}

/// One RAM-only, identity-checked Stage B candidate slot for the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBCandidateSlot {
    candidate: Option<StageBCandidateSummary>,
}

impl StageBCandidateSlot {
    /// Construct an empty slot. MCU restart therefore clears reproduction state.
    pub const fn new() -> Self {
        Self { candidate: None }
    }

    /// Return the currently retained sufficient run.
    pub const fn candidate(&self) -> Option<StageBCandidateSummary> {
        self.candidate
    }

    /// Reject a changed plan before actuation without modifying retained evidence.
    pub fn preflight(&self, requested: StageBPlanIdentity) -> Option<StageBReproductionDecision> {
        let retained = self.candidate?.identity;
        (retained != requested).then_some(StageBReproductionDecision::RejectedPlanMismatch {
            retained,
            requested,
        })
    }

    /// Compare one sufficient run with the retained candidate.
    pub fn observe_sufficient(
        &mut self,
        current: StageBCandidateSummary,
    ) -> StageBReproductionDecision {
        let Some(previous) = self.candidate else {
            self.candidate = Some(current);
            return StageBReproductionDecision::CompleteCandidate(current);
        };
        if previous.identity != current.identity {
            return StageBReproductionDecision::RejectedPlanMismatch {
                retained: previous.identity,
                requested: current.identity,
            };
        }
        let Some(delta) = reproduction_delta(previous, current) else {
            self.clear();
            return StageBReproductionDecision::EvidenceIntegrityFault;
        };
        if delta.reason_mask == 0 {
            self.clear();
            StageBReproductionDecision::Complete(delta)
        } else {
            self.candidate = Some(current);
            StageBReproductionDecision::InconclusiveReproductionMismatch(delta)
        }
    }

    /// Clear the RAM-only candidate after insufficient or faulted evidence.
    pub fn clear(&mut self) {
        self.candidate = None;
    }

    /// Replace the retained run digest after post-engine reproduction evidence is hashed.
    pub fn replace_retained_digest(&mut self, digest: u64) {
        if let Some(candidate) = &mut self.candidate {
            candidate.run_digest = digest;
        }
    }

    /// Invalidate when commissioning starts on another channel.
    pub fn invalidate_for_channel(&mut self, channel: u8) -> StageBReproductionDecision {
        if self
            .candidate
            .is_some_and(|candidate| candidate.identity.channel != channel)
        {
            self.clear();
        }
        StageBReproductionDecision::Cleared
    }

    /// Invalidate when any unrelated commissioning activity begins.
    pub fn invalidate_for_unrelated_commissioning(&mut self) -> StageBReproductionDecision {
        self.clear();
        StageBReproductionDecision::Cleared
    }
}

impl Default for StageBCandidateSlot {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<StageBCandidateSlot>() <= 96);

fn reproduction_delta(
    previous: StageBCandidateSummary,
    current: StageBCandidateSummary,
) -> Option<StageBReproductionDelta> {
    if previous.p_start != current.p_start
        || previous.p_top != current.p_top
        || previous.rung_count != current.rung_count
        || !summary_geometry_valid(previous)
        || !summary_geometry_valid(current)
    {
        return None;
    }
    let previous_memberships = [
        previous.forward_membership,
        previous.reverse_membership,
        previous.joint_membership,
    ];
    let current_memberships = [
        current.forward_membership,
        current.reverse_membership,
        current.joint_membership,
    ];
    let memberships = [
        membership_comparison(previous_memberships[0], current_memberships[0])?,
        membership_comparison(previous_memberships[1], current_memberships[1])?,
        membership_comparison(previous_memberships[2], current_memberships[2])?,
    ];
    if memberships
        .iter()
        .any(|membership| membership.core != 0 && !contiguous_nonempty(membership.core))
    {
        return None;
    }

    let intervals = [
        interval_comparison(
            previous.forward_interval,
            current.forward_interval,
            previous.forward_common_interval,
            current.forward_common_interval,
        ),
        interval_comparison(
            previous.reverse_interval,
            current.reverse_interval,
            previous.reverse_common_interval,
            current.reverse_common_interval,
        ),
    ];
    let mut reason_mask = 0;
    if memberships[0].core.count_ones() < 3 {
        reason_mask |= STAGE_B_REPRO_REASON_FORWARD_CORE_TOO_SMALL;
    }
    if memberships[1].core.count_ones() < 3 {
        reason_mask |= STAGE_B_REPRO_REASON_REVERSE_CORE_TOO_SMALL;
    }
    if memberships[2].core == 0 {
        reason_mask |= STAGE_B_REPRO_REASON_JOINT_CORE_EMPTY;
    }
    if !closed_interval_nonempty(intervals[0].pooled_overlap) {
        reason_mask |= STAGE_B_REPRO_REASON_FORWARD_POOLED_DISJOINT;
    }
    if !closed_interval_nonempty(intervals[1].pooled_overlap) {
        reason_mask |= STAGE_B_REPRO_REASON_REVERSE_POOLED_DISJOINT;
    }

    let (final_p, coverage, reduced_margin) = if memberships[2].core == 0 {
        (0, [StageBCoreCoverage::EMPTY; 2], false)
    } else {
        let joint_count = memberships[2].core.count_ones() as u8;
        let joint_first = memberships[2].core.trailing_zeros() as u8;
        let final_rung = joint_first + (joint_count - 1) / 2;
        let final_p = gain_at(
            previous.p_start,
            previous.p_top,
            previous.rung_count,
            final_rung,
        )?
        .raw();
        (
            final_p,
            [
                core_coverage(
                    memberships[0].core,
                    final_rung,
                    previous.p_start,
                    previous.p_top,
                    previous.rung_count,
                )?,
                core_coverage(
                    memberships[1].core,
                    final_rung,
                    previous.p_start,
                    previous.p_top,
                    previous.rung_count,
                )?,
            ],
            joint_count <= 2,
        )
    };

    Some(StageBReproductionDelta {
        memberships,
        intervals,
        previous_provisional_p: previous.nominated_p,
        current_provisional_p: current.nominated_p,
        final_p,
        coverage,
        reduced_margin,
        reason_mask,
        previous_run_digest: previous.run_digest,
        current_run_digest: current.run_digest,
    })
}

fn summary_geometry_valid(summary: StageBCandidateSummary) -> bool {
    let rung_count = u32::from(summary.rung_count);
    summary.p_start != 0
        && summary.p_start <= summary.p_top
        && summary.rung_count != 0
        && usize::from(summary.rung_count) <= MAX_SWEEP_RUNGS
        && [
            summary.forward_membership,
            summary.reverse_membership,
            summary.joint_membership,
        ]
        .into_iter()
        .all(|mask| {
            let valid_bits = if rung_count == 32 {
                u32::MAX
            } else {
                (1_u32 << rung_count) - 1
            };
            contiguous_nonempty(mask) && mask & !valid_bits == 0
        })
}

fn contiguous_nonempty(mask: u32) -> bool {
    if mask == 0 {
        return false;
    }
    let shifted = mask >> mask.trailing_zeros();
    shifted == u32::MAX || shifted.wrapping_add(1).is_power_of_two()
}

fn membership_comparison(previous: u32, current: u32) -> Option<StageBMembershipComparison> {
    let low_delta = i16::try_from(current.trailing_zeros()).ok()?
        - i16::try_from(previous.trailing_zeros()).ok()?;
    let current_high = 31_u32.checked_sub(current.leading_zeros())?;
    let previous_high = 31_u32.checked_sub(previous.leading_zeros())?;
    let high_delta = i16::try_from(current_high).ok()? - i16::try_from(previous_high).ok()?;
    Some(StageBMembershipComparison {
        previous,
        current,
        core: previous & current,
        previous_only: previous & !current,
        current_only: current & !previous,
        low_delta: i8::try_from(low_delta).ok()?,
        high_delta: i8::try_from(high_delta).ok()?,
    })
}

fn interval_comparison(
    previous_pooled: [i32; 2],
    current_pooled: [i32; 2],
    previous_common: [i32; 2],
    current_common: [i32; 2],
) -> StageBIntervalComparison {
    let pooled_overlap = closed_intersection(previous_pooled, current_pooled);
    let conservative_common = closed_intersection(previous_common, current_common);
    StageBIntervalComparison {
        previous_pooled,
        current_pooled,
        pooled_overlap,
        previous_common,
        current_common,
        conservative_common,
        conservative_common_nonempty: closed_interval_nonempty(conservative_common),
    }
}

fn closed_intersection(first: [i32; 2], second: [i32; 2]) -> [i32; 2] {
    [first[0].max(second[0]), first[1].min(second[1])]
}

fn closed_interval_nonempty(interval: [i32; 2]) -> bool {
    interval[0] <= interval[1]
}

fn core_coverage(
    core: u32,
    final_rung: u8,
    p_start: u16,
    p_top: u16,
    rung_count: u8,
) -> Option<StageBCoreCoverage> {
    if core == 0 {
        return Some(StageBCoreCoverage::EMPTY);
    }
    let first = core.trailing_zeros() as u8;
    let last = (31 - core.leading_zeros()) as u8;
    let (kind, distance, source_rung) = if final_rung < first {
        (
            CoverageKind::TransferredCoverage,
            i16::from(first) - i16::from(final_rung),
            first,
        )
    } else if final_rung > last {
        (
            CoverageKind::TransferredCoverage,
            i16::from(last) - i16::from(final_rung),
            last,
        )
    } else {
        (CoverageKind::FullCoverage, 0, final_rung)
    };
    let source_p = gain_at(p_start, p_top, rung_count, source_rung)?.raw();
    let final_p = gain_at(p_start, p_top, rung_count, final_rung)?.raw();
    let (gain_ratio_num, gain_ratio_den) = if kind == CoverageKind::FullCoverage {
        (1, 1)
    } else {
        (source_p.max(final_p), source_p.min(final_p).max(1))
    };
    Some(StageBCoreCoverage {
        kind,
        signed_rung_distance: i8::try_from(distance).ok()?,
        gain_ratio_num,
        gain_ratio_den,
    })
}

#[cfg(test)]
#[path = "reproduction_tests.rs"]
mod tests;
