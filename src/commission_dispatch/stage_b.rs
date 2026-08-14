//! Board-owned Stage B preflight and cross-run reproduction policy.

use crate::commissioning::outer::velocity::reproduction::{
    BUILD_FEATURES, STAGE_B_EVIDENCE_SCHEMA_REVISION, StageBCandidateSlot, StageBPlanIdentity,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepPlan, VelocitySweepRequest, plan_velocity_sweep,
};

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
    /// A RAM-only candidate from the retired paired workflow is still present.
    RejectedLegacyState {
        /// Identity of the combined plan that could run after an MCU restart.
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

/// Construct and identity-check the only Stage B plan before motor initialization.
///
/// The retained candidate is deliberately not modified on either rejection path.
pub fn preflight_stage_b_request(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_limit: u16,
    limit_evidence_capability: u32,
    channel: u8,
    run_sequence: u32,
    stage_b_candidate: &mut StageBCandidateSlot,
) -> Result<StageBPreflight, StageBPreflightRejection> {
    let plan = plan_velocity_sweep(
        request,
        encoder_counts_per_rev,
        torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| StageBPreflightRejection::InvalidPlan)?;
    let identity = StageBPlanIdentity {
        channel,
        build_features: BUILD_FEATURES,
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
