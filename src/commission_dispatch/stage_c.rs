//! Board-owned Stage-C admission and retained-evidence policy.

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::integral_plan::{
    StageCFailedAdmission, VelocityIntegralPlan, VelocityIntegralPlanError,
    plan_combined_velocity_integral_response, plan_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, AcceptancePoint, StageCAuthoritySummary, StageCCandidateSlot,
    StageCCandidateSummary, StageCPlanIdentity, StageCReproductionDecision,
    StageCReproductionDelta, StageCRestBoundaryReference,
};
use crate::commissioning::outer::velocity::reproduction::STAGE_B_BUILD_REVISION;
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepRequest, plan_combined_velocity_sweep, plan_velocity_sweep,
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
/// No reproduced operating rung can run under the Stage-C probe clamp.
pub const STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT: u8 = 11;

const ACCEPTANCE_QUALIFIER_RECOVERY_QUANTIZATION_EXPOSED: u16 = 1 << 7;
const ACCEPTANCE_QUALIFIER_PROBE_CONSTRAINED_TEST_POINT: u16 = 1 << 8;
const ACCEPTANCE_QUALIFIER_LATCHED_LIMIT_EVIDENCE: u16 = 1 << 9;

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
    /// The reproduced operating component contains no Stage-C-safe test point.
    NoTransitionCapableOperatingPoint(StageCFailedAdmission),
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

/// Derive one executable matrix handoff from a safe combined terminal.
pub(crate) fn combined_acceptance_point(
    channel: u8,
    plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralExecutionPlan,
    terminal: &VelocityIntegralTerminal,
) -> Result<Option<AcceptancePoint>, ()> {
    use crate::commissioning::outer::velocity::combined_plan::{
        COMBINED_STAGE_C_SCHEMA_REVISION, CombinedTargetStatus,
    };
    use crate::commissioning::outer::velocity::integral_reporting::{
        terminal_target_status, terminal_target_terminus,
    };
    use crate::commissioning::outer::velocity::{
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG, integral_i_at,
    };

    if terminal.outcome != VelocityIntegralOutcome::Complete
        || terminal_target_status(terminal) != Some(CombinedTargetStatus::TargetReached)
    {
        return Ok(None);
    }
    if terminal.plan_digest != plan.plan_digest || terminal.final_p != plan.final_p {
        return Err(());
    }
    let positive_index = terminal_target_terminus(terminal).ok_or(())?;
    let selected_i = integral_i_at(plan, positive_index.checked_add(1).ok_or(())?).ok_or(())?;
    let mut qualifier_bits = 0;
    if terminal.recovery_quantization_exposed {
        qualifier_bits |= ACCEPTANCE_QUALIFIER_RECOVERY_QUANTIZATION_EXPOSED;
    }
    if terminal.probe_constrained_test_point {
        qualifier_bits |= ACCEPTANCE_QUALIFIER_PROBE_CONSTRAINED_TEST_POINT;
    }
    if plan.limit_evidence_capability as u32 == VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG {
        qualifier_bits |= ACCEPTANCE_QUALIFIER_LATCHED_LIMIT_EVIDENCE;
    }
    Ok(Some(AcceptancePoint {
        identity: StageCPlanIdentity {
            channel,
            build_revision: STAGE_B_BUILD_REVISION,
            schema_revision: COMBINED_STAGE_C_SCHEMA_REVISION,
            plan_digest: plan.plan_digest,
        },
        selected_p: plan.final_p,
        selected_i,
        target_velocity_rpm: plan.target_velocity_rpm,
        selected_response_intervals_q: plan.selected_response_intervals_q,
        qualifier_bits,
        progress: AcceptanceMatrixProgress::ReadyAscending,
    }))
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
    let combined = authority.identity.schema_revision
        == crate::commissioning::outer::velocity::combined_plan::COMBINED_STAGE_C_SCHEMA_REVISION;
    let stage_b_plan = if combined {
        plan_combined_velocity_sweep(
            request,
            encoder_counts_per_rev,
            torque_limit,
            limit_evidence_capability,
        )
    } else {
        plan_velocity_sweep(
            request,
            encoder_counts_per_rev,
            torque_limit,
            limit_evidence_capability,
        )
    }
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
    let stage_b_execution = stage_b_plan.execution_plan(stage_b_plan_digest);
    let plan_result = if combined {
        let p = foci_tmc4671::pid::PidPGain::new(authority.final_p)
            .map_err(|_| StageCPreflightRejection::InvalidPlan)?;
        let selected =
            crate::commissioning::outer::velocity::combined_plan::CombinedSelectedResponse::from_retained_intervals(
                p,
                authority.directional_pooled_intervals,
            )
            .map_err(|_| StageCPreflightRejection::InvalidPlan)?;
        plan_combined_velocity_integral_response(
            selected,
            stage_b_execution,
            motion_budget,
            pwm_hz,
            encoder_counts_per_rev,
        )
    } else {
        plan_velocity_integral_response(
            authority.stage_c_authority(),
            stage_b_execution,
            motion_budget,
            pwm_hz,
            encoder_counts_per_rev,
        )
    };
    let plan = match plan_result {
        Ok(plan) => plan,
        Err(VelocityIntegralPlanError::NoTransitionCapableOperatingPoint(failure)) => {
            return Err(StageCPreflightRejection::NoTransitionCapableOperatingPoint(
                failure,
            ));
        }
        Err(_) => return Err(StageCPreflightRejection::InvalidPlan),
    };
    let identity = StageCPlanIdentity {
        channel,
        build_revision: STAGE_B_BUILD_REVISION,
        schema_revision: plan.schema_revision,
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
                rest_boundary: terminal
                    .rest_boundary_rung_plus_one
                    .checked_sub(1)
                    .zip(terminal.rest_boundary_slot_plus_one.checked_sub(1))
                    .map(|(positive_rung_index, slot)| StageCRestBoundaryReference {
                        positive_rung_index,
                        slot,
                    }),
                run_completed_us: terminal.run_completed_us,
                run_digest: terminal.digest,
            };
            match stage_c_candidate.observe_sufficient(current) {
                StageCReproductionDecision::CompleteCandidate(_) => None,
                StageCReproductionDecision::Complete(delta) => {
                    terminal.outcome = VelocityIntegralOutcome::Complete;
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
        VelocityIntegralOutcome::Failed => {
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

/// Build the authoritative zero-motion terminal for a failed Stage-C admission.
pub(crate) fn failed_stage_c_terminal(
    run_sequence: u32,
    failure: StageCFailedAdmission,
) -> VelocityIntegralTerminal {
    use crate::commissioning::outer::velocity::integral_evidence::{
        ExactRateInterval, IntegralRunEvidence,
    };

    VelocityIntegralTerminal {
        run_sequence,
        evidence_sequence: 0,
        plan_digest: failure.plan_digest,
        digest: 0,
        outcome: VelocityIntegralOutcome::Failed,
        cause: STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT,
        final_p: 0,
        recovery_unavailable: false,
        recovered_with_current_headroom: false,
        recovery_quantization_exposed: false,
        probe_constrained_test_point: failure.probe_constrained_test_point,
        rest_boundary_rung_plus_one: 0,
        rest_boundary_slot_plus_one: 0,
        evidence: IntegralRunEvidence::new([ExactRateInterval::EMPTY; 2]),
        expected_observations: 0,
        emitted_observations: 0,
        expected_rungs: 0,
        emitted_rungs: 0,
        run_started_us: 0,
        run_completed_us: 0,
        safety_fault: None,
    }
}
