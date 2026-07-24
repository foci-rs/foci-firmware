//! Board-owned Stage-C admission and retained-evidence policy.

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::integral_plan::{
    STAGE_C_EVIDENCE_SCHEMA_REVISION, VelocityIntegralPlan, plan_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    StageCAuthoritySummary, StageCCandidateSlot, StageCCandidateSummary, StageCPlanIdentity,
    StageCReproductionDecision, StageCReproductionDelta,
};
use crate::commissioning::outer::velocity::reproduction::STAGE_B_BUILD_REVISION;
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepRequest, plan_velocity_sweep,
};
use crate::commissioning::types::{
    VelocityIntegralEvent, VelocityIntegralOutcome, VelocityIntegralPlanned,
    VelocityIntegralTerminal,
};

/// Dedicated cause for retained or live Stage-C evidence violating its contract.
pub const STAGE_C_CAUSE_EVIDENCE_INTEGRITY: u8 = 4;
/// Dedicated cause for a sufficient curve that did not reproduce.
pub const STAGE_C_CAUSE_REPRODUCTION_MISMATCH: u8 = 6;
/// Dedicated cause for a frozen Stage-C plan mismatch.
pub const STAGE_C_CAUSE_PLAN_MISMATCH: u8 = 7;

/// Immutable plan and identity accepted before Stage-C actuation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StageCPreflight {
    /// The sole exact Stage-C plan consumed by construction and reporting.
    pub plan: VelocityIntegralPlan,
    /// Exact cross-run identity derived from the plan.
    pub identity: StageCPlanIdentity,
    /// Caller-assigned trace sequence.
    pub run_sequence: u32,
}

impl StageCPreflight {
    /// Return the exact plan evidence emitted before the first Stage-C stroke.
    pub const fn planned_event(self) -> VelocityIntegralEvent {
        VelocityIntegralEvent::Planned(VelocityIntegralPlanned {
            run_sequence: self.run_sequence,
            evidence_sequence: 0,
            plan_digest: self.identity.plan_digest,
            final_p: self.plan.final_p,
            positive_rung_count: self.plan.positive_rung_count,
            maximum_workflow_ms: self.plan.maximum_workflow_ms,
        })
    }
}

/// Fail-closed reason a retained Stage-C request was rejected before actuation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StageCPreflightRejection {
    /// The current request could not reconstruct a safe exact Stage-C plan.
    InvalidPlan,
    /// The current request differs from retained Stage-C authority.
    RejectedPlanMismatch {
        /// Exact retained Stage-C identity.
        retained: StageCPlanIdentity,
        /// Current Stage-B plan digest, which prevented exact reconstruction.
        requested_stage_b_plan_digest: u64,
    },
}

/// Cross-run comparison attached to the Stage-C terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCReproductionReport {
    /// One exact retained-versus-current comparison.
    pub delta: StageCReproductionDelta,
    /// One for Complete, two for mismatch.
    pub outcome: u8,
    /// Exact retained-run evidence digest.
    pub previous_run_digest: u64,
    /// Exact current-run evidence digest.
    pub current_run_digest: u64,
}

/// Stage-C terminal after firmware-owned retention policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageCTerminalDispatch {
    /// Authoritative terminal class after comparison.
    pub terminal: VelocityIntegralTerminal,
    /// Comparison evidence present only for a second sufficient run.
    pub reproduction: Option<StageCReproductionReport>,
    /// Retained Stage-B authority used for anchor comparison and context.
    pub authority: Option<StageCAuthoritySummary>,
}

/// Reconstruct and identity-check retained Stage-C authority before any SPI work.
///
/// `Ok(None)` means no Stage-C authority exists and admission must proceed to Stage B.
pub fn preflight_stage_c_request(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_limit: u16,
    limit_evidence_capability: u32,
    channel: u8,
    pwm_hz: u32,
    run_sequence: u32,
    stage_c_candidate: &StageCCandidateSlot,
) -> Result<Option<StageCPreflight>, StageCPreflightRejection> {
    let Some(authority) = stage_c_candidate.authority() else {
        return Ok(None);
    };
    if authority.identity.channel != channel {
        return Err(StageCPreflightRejection::RejectedPlanMismatch {
            retained: authority.identity,
            requested_stage_b_plan_digest: 0,
        });
    }
    let stage_b_plan = plan_velocity_sweep(
        request,
        encoder_counts_per_rev,
        torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| StageCPreflightRejection::InvalidPlan)?;
    let stage_b_plan_digest = stage_b_plan.digest();
    if authority.stage_b_plan_digest != stage_b_plan_digest {
        return Err(StageCPreflightRejection::RejectedPlanMismatch {
            retained: authority.identity,
            requested_stage_b_plan_digest: stage_b_plan_digest,
        });
    }
    let motion_budget = OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
    };
    let plan = plan_velocity_integral_response(
        authority.stage_c_authority(),
        stage_b_plan.execution_plan(stage_b_plan_digest),
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
    )
    .map_err(|_| StageCPreflightRejection::InvalidPlan)?;
    let identity = StageCPlanIdentity {
        channel,
        build_revision: STAGE_B_BUILD_REVISION,
        schema_revision: STAGE_C_EVIDENCE_SCHEMA_REVISION,
        plan_digest: plan.digest(),
    };
    if stage_c_candidate.preflight(identity).is_some() {
        return Err(StageCPreflightRejection::RejectedPlanMismatch {
            retained: authority.identity,
            requested_stage_b_plan_digest: stage_b_plan_digest,
        });
    }
    Ok(Some(StageCPreflight {
        plan,
        identity,
        run_sequence,
    }))
}

/// Apply one Stage-C terminal to the authoritative RAM-only lifecycle.
pub fn apply_stage_c_terminal(
    mut terminal: VelocityIntegralTerminal,
    stage_c_candidate: &mut StageCCandidateSlot,
) -> StageCTerminalDispatch {
    let Some(authority) = stage_c_candidate.authority() else {
        terminal.outcome = VelocityIntegralOutcome::Fault;
        terminal.cause = STAGE_C_CAUSE_EVIDENCE_INTEGRITY;
        return StageCTerminalDispatch {
            terminal,
            reproduction: None,
            authority: None,
        };
    };
    if terminal.plan_digest != authority.identity.plan_digest {
        stage_c_candidate.observe_fault();
        terminal.outcome = VelocityIntegralOutcome::Fault;
        terminal.cause = STAGE_C_CAUSE_EVIDENCE_INTEGRITY;
        return StageCTerminalDispatch {
            terminal,
            reproduction: None,
            authority: Some(authority),
        };
    }

    let reproduction = match terminal.outcome {
        VelocityIntegralOutcome::CompleteCandidate => {
            let previous_run_digest = stage_c_candidate
                .candidate()
                .map_or(0, |candidate| candidate.run_digest);
            let current = StageCCandidateSummary {
                authority,
                evidence: terminal.evidence,
                run_completed_us: terminal.run_completed_us,
                run_digest: terminal.digest,
            };
            match stage_c_candidate.observe_sufficient(current) {
                StageCReproductionDecision::CompleteCandidate(_) => None,
                StageCReproductionDecision::Complete(delta) => {
                    terminal.outcome = VelocityIntegralOutcome::Complete;
                    terminal.cause = 0;
                    Some(StageCReproductionReport {
                        delta,
                        outcome: 1,
                        previous_run_digest,
                        current_run_digest: terminal.digest,
                    })
                }
                StageCReproductionDecision::InconclusiveReproductionMismatch(delta) => {
                    terminal.outcome = VelocityIntegralOutcome::Inconclusive;
                    terminal.cause = STAGE_C_CAUSE_REPRODUCTION_MISMATCH;
                    Some(StageCReproductionReport {
                        delta,
                        outcome: 2,
                        previous_run_digest,
                        current_run_digest: terminal.digest,
                    })
                }
                StageCReproductionDecision::RejectedPlanMismatch { .. } => {
                    terminal.outcome = VelocityIntegralOutcome::RejectedPlanMismatch;
                    terminal.cause = STAGE_C_CAUSE_PLAN_MISMATCH;
                    None
                }
                StageCReproductionDecision::EvidenceIntegrityFault => {
                    terminal.outcome = VelocityIntegralOutcome::Fault;
                    terminal.cause = STAGE_C_CAUSE_EVIDENCE_INTEGRITY;
                    None
                }
            }
        }
        VelocityIntegralOutcome::Inconclusive => {
            stage_c_candidate.observe_inconclusive();
            None
        }
        VelocityIntegralOutcome::Fault => {
            stage_c_candidate.observe_fault();
            None
        }
        VelocityIntegralOutcome::Complete | VelocityIntegralOutcome::RejectedPlanMismatch => {
            stage_c_candidate.observe_fault();
            terminal.outcome = VelocityIntegralOutcome::Fault;
            terminal.cause = STAGE_C_CAUSE_EVIDENCE_INTEGRITY;
            None
        }
    };
    StageCTerminalDispatch {
        terminal,
        reproduction,
        authority: Some(authority),
    }
}
