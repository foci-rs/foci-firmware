//! Board-owned Stage B preflight and cross-run reproduction policy.

use crate::commissioning::outer::velocity::reproduction::{
    STAGE_B_BUILD_REVISION, STAGE_B_EVIDENCE_SCHEMA_REVISION, StageBCandidateSlot,
    StageBCandidateSummary, StageBPlanIdentity, StageBReproductionDecision,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepPlan, VelocitySweepRequest, plan_velocity_sweep,
};
use crate::commissioning::types::{VelocitySweepOutcome, VelocitySweepTerminal};

/// Dedicated terminal cause for a sufficient run that did not reproduce.
pub const STAGE_B_CAUSE_REPRODUCTION_MISMATCH: u8 = 5;
/// Dedicated terminal cause for a frozen-plan identity mismatch.
pub const STAGE_B_CAUSE_PLAN_MISMATCH: u8 = 6;

/// Immutable plan and identity accepted before any Stage B actuation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBPreflight {
    /// The sole plan instance consumed by engine construction and reporting.
    pub plan: VelocitySweepPlan,
    /// Exact cross-run identity derived from the frozen plan.
    pub identity: StageBPlanIdentity,
    /// Caller-assigned sequence retained for engine evidence.
    pub run_sequence: u32,
}

impl StageBPreflight {
    /// Return the plan evidence that must be emitted before engine construction.
    pub fn planned_event(self) -> crate::commissioning::types::VelocitySweepEvent {
        crate::commissioning::types::VelocitySweepEvent::Planned(
            self.plan.report(self.run_sequence, 0),
        )
    }
}

/// Fail-closed reason a Stage B command was rejected before actuation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StageBPreflightRejection {
    /// The request could not produce a safe deterministic sweep plan.
    InvalidPlan,
    /// A retained candidate belongs to a different frozen plan.
    RejectedPlanMismatch {
        /// Identity retained from the previous sufficient run.
        retained: StageBPlanIdentity,
        /// Identity computed for the newly requested run.
        requested: StageBPlanIdentity,
    },
}

/// Exact cross-run comparison emitted immediately before terminal fragments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBReproductionReport {
    /// Current caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence allocated to this comparison.
    pub evidence_sequence: u16,
    /// Matching complete or nonmatching outcome code.
    pub outcome: u8,
    /// Exact retained-versus-current evidence delta.
    pub delta: crate::commissioning::outer::velocity::reproduction::StageBReproductionDelta,
}

/// Terminal plus the optional comparison record inserted before it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBTerminalDispatch {
    /// Authoritative terminal after the cross-run decision and digest extension.
    pub terminal: VelocitySweepTerminal,
    /// Comparison evidence present only when a retained sufficient run existed.
    pub reproduction: Option<StageBReproductionReport>,
}

/// Construct and identity-check the only Stage B plan before motor initialization.
///
/// The retained candidate is deliberately not modified on either rejection path.
pub fn preflight_stage_b_request(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_limit: u16,
    channel: u8,
    run_sequence: u32,
    stage_b_candidate: &mut StageBCandidateSlot,
) -> Result<StageBPreflight, StageBPreflightRejection> {
    let plan = plan_velocity_sweep(request, encoder_counts_per_rev, torque_limit)
        .map_err(|_| StageBPreflightRejection::InvalidPlan)?;
    let identity = StageBPlanIdentity {
        channel,
        build_revision: STAGE_B_BUILD_REVISION,
        schema_revision: STAGE_B_EVIDENCE_SCHEMA_REVISION,
        plan_digest: plan.digest(),
    };
    if let Some(
        crate::commissioning::outer::velocity::reproduction::StageBReproductionDecision::RejectedPlanMismatch {
            retained,
            requested,
        },
    ) = stage_b_candidate.preflight(identity)
    {
        return Err(StageBPreflightRejection::RejectedPlanMismatch {
            retained,
            requested,
        });
    }
    Ok(StageBPreflight {
        plan,
        identity,
        run_sequence,
    })
}

/// Apply one board-owned candidate transition to an engine-provided terminal.
pub fn apply_stage_b_terminal(
    mut terminal: VelocitySweepTerminal,
    channel: u8,
    stage_b_candidate: &mut StageBCandidateSlot,
) -> StageBTerminalDispatch {
    if terminal.outcome != VelocitySweepOutcome::CompleteCandidate {
        stage_b_candidate.clear();
        return StageBTerminalDispatch {
            terminal,
            reproduction: None,
        };
    }

    let current = StageBCandidateSummary {
        identity: StageBPlanIdentity {
            channel,
            build_revision: STAGE_B_BUILD_REVISION,
            schema_revision: STAGE_B_EVIDENCE_SCHEMA_REVISION,
            plan_digest: terminal.plan_digest,
        },
        forward_membership: terminal.selected_memberships[0],
        reverse_membership: terminal.selected_memberships[1],
        joint_membership: terminal.selected_memberships[2],
        nominated_p: terminal.nominated_p,
        forward_interval: terminal.selected_intervals[0],
        reverse_interval: terminal.selected_intervals[1],
        run_started_us: terminal.run_started_us,
        run_completed_us: terminal.run_completed_us,
        run_digest: terminal.digest,
    };

    let reproduction = match stage_b_candidate.observe_sufficient(current) {
        StageBReproductionDecision::CompleteCandidate(_) => None,
        StageBReproductionDecision::Complete(delta) => {
            terminal.outcome = VelocitySweepOutcome::Complete;
            terminal.cause = 0;
            Some(StageBReproductionReport {
                run_sequence: terminal.run_sequence,
                evidence_sequence: terminal.evidence_sequence,
                outcome: 1,
                delta,
            })
        }
        StageBReproductionDecision::InconclusiveReproductionMismatch(delta) => {
            terminal.outcome = VelocitySweepOutcome::Inconclusive;
            terminal.cause = STAGE_B_CAUSE_REPRODUCTION_MISMATCH;
            Some(StageBReproductionReport {
                run_sequence: terminal.run_sequence,
                evidence_sequence: terminal.evidence_sequence,
                outcome: 2,
                delta,
            })
        }
        StageBReproductionDecision::RejectedPlanMismatch { .. } => {
            terminal.outcome = VelocitySweepOutcome::Inconclusive;
            terminal.cause = STAGE_B_CAUSE_PLAN_MISMATCH;
            None
        }
        StageBReproductionDecision::Cleared => None,
    };

    if let Some(report) = reproduction {
        let record = encode_stage_b_reproduction(report);
        terminal.digest = crate::ankyra::velocity_sweep_encoding::extend_velocity_sweep_digest(
            terminal.digest,
            &record,
        );
        terminal.evidence_sequence = terminal.evidence_sequence.saturating_add(1);
        stage_b_candidate.replace_retained_digest(terminal.digest);
    }
    StageBTerminalDispatch {
        terminal,
        reproduction,
    }
}

fn encode_stage_b_reproduction(
    report: StageBReproductionReport,
) -> crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord {
    let mut record = crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord::new();
    record.push_u8(9);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u8(report.outcome);
    record.push_u16(report.delta.previous_nominated_p);
    record.push_u16(report.delta.current_nominated_p);
    for membership in report.delta.previous_memberships {
        record.push_u32(membership);
    }
    for membership in report.delta.current_memberships {
        record.push_u32(membership);
    }
    for interval in report.delta.previous_intervals {
        record.push_i32(interval[0]);
        record.push_i32(interval[1]);
    }
    for interval in report.delta.current_intervals {
        record.push_i32(interval[0]);
        record.push_i32(interval[1]);
    }
    record.push_u32(report.delta.previous_run_digest as u32);
    record.push_u32((report.delta.previous_run_digest >> 32) as u32);
    record.push_u32(report.delta.current_run_digest as u32);
    record.push_u32((report.delta.current_run_digest >> 32) as u32);
    record
}
