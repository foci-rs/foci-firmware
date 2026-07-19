/// Evidence schema stored in the RAM-only Stage B reproduction slot.
pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u16 = 1;

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
    /// Firmware-nominated proportional gain.
    pub nominated_p: u16,
    /// Forward pooled equivalent-disturbance interval.
    pub forward_interval: [i32; 2],
    /// Reverse pooled equivalent-disturbance interval.
    pub reverse_interval: [i32; 2],
    /// Run-start timestamp retained as non-gating context.
    pub run_started_us: u64,
    /// Run-completion timestamp retained as non-gating context.
    pub run_completed_us: u64,
    /// Exact digest of the emitted run evidence.
    pub run_digest: u64,
}

/// Exact evidence delta reported for a completed or mismatched pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBReproductionDelta {
    /// Forward, reverse, and joint memberships from the retained run.
    pub previous_memberships: [u32; 3],
    /// Forward, reverse, and joint memberships from the current run.
    pub current_memberships: [u32; 3],
    /// Retained provisional P.
    pub previous_nominated_p: u16,
    /// Current provisional P.
    pub current_nominated_p: u16,
    /// Retained forward and reverse pooled intervals.
    pub previous_intervals: [[i32; 2]; 2],
    /// Current forward and reverse pooled intervals.
    pub current_intervals: [[i32; 2]; 2],
    /// Retained run digest.
    pub previous_run_digest: u64,
    /// Current run digest.
    pub current_run_digest: u64,
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
        let delta = reproduction_delta(previous, current);
        if summaries_reproduce(previous, current) {
            self.candidate = None;
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

fn summaries_reproduce(previous: StageBCandidateSummary, current: StageBCandidateSummary) -> bool {
    previous.forward_membership == current.forward_membership
        && previous.reverse_membership == current.reverse_membership
        && previous.joint_membership == current.joint_membership
        && previous.nominated_p == current.nominated_p
        && intervals_overlap(previous.forward_interval, current.forward_interval)
        && intervals_overlap(previous.reverse_interval, current.reverse_interval)
}

fn intervals_overlap(first: [i32; 2], second: [i32; 2]) -> bool {
    first[0] <= first[1]
        && second[0] <= second[1]
        && first[0].max(second[0]) <= first[1].min(second[1])
}

fn reproduction_delta(
    previous: StageBCandidateSummary,
    current: StageBCandidateSummary,
) -> StageBReproductionDelta {
    StageBReproductionDelta {
        previous_memberships: [
            previous.forward_membership,
            previous.reverse_membership,
            previous.joint_membership,
        ],
        current_memberships: [
            current.forward_membership,
            current.reverse_membership,
            current.joint_membership,
        ],
        previous_nominated_p: previous.nominated_p,
        current_nominated_p: current.nominated_p,
        previous_intervals: [previous.forward_interval, previous.reverse_interval],
        current_intervals: [current.forward_interval, current.reverse_interval],
        previous_run_digest: previous.run_digest,
        current_run_digest: current.run_digest,
    }
}

#[cfg(test)]
#[path = "reproduction_tests.rs"]
mod tests;
