//! Board-owned Stage-C admission and retained-evidence policy.

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::integral_plan::{
    StageCFailedAdmission, VelocityIntegralPlan, VelocityIntegralPlanError,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, AcceptancePoint, StageCAuthoritySummary, StageCCandidateSlot,
    StageCCandidateSummary, StageCPlanIdentity, StageCReproductionDecision,
    StageCReproductionDelta, StageCRestBoundaryReference,
};
use crate::commissioning::outer::velocity::reproduction::BUILD_FEATURES;
use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;
use crate::commissioning::types::{
    StageCCause, VelocityIntegralEvent, VelocityIntegralOutcome, VelocityIntegralPlanned,
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
/// A Stage-C resume was requested with nothing retained to resume from.
///
/// Distinct from every rejection cause: the request was well formed and the
/// slot is simply empty, which is the ordinary state after any reset because
/// retention is RAM-only. Sharing a cause with a rejection would tell the
/// operator to inspect a mismatch that does not exist.
pub const STAGE_C_CAUSE_NO_RETAINED_AUTHORITY: u8 = 12;
/// A breakaway acceptance `Complete` recorded no target terminus.
///
/// Distinct from an evidence-integrity fault: the run was well formed and simply
/// did not reach a target terminus, so no acceptance point is installed. Sharing
/// a cause with a fault would tell the operator to inspect an integrity breach
/// that did not occur.
pub const STAGE_C_CAUSE_NO_ACCEPTANCE_TERMINUS: u8 = 13;

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

/// Install retained Stage-C authority from one exact execution plan.
///
/// Public so a board can retain authority directly from the scheduled plan
/// event. Boards whose budget keeps nonterminal classification outside the
/// terminal async dispatch future cannot reach the retention inside
/// [`crate::tmc_control::TmcControlRuntime::drive_commissioning_poll_with_stages`],
/// and a Stage-C terminal arriving with an empty slot fails closed.
pub fn install_authority_from_plan(
    channel: u8,
    stage_c_candidate: &mut StageCCandidateSlot,
    plan: VelocityIntegralPlan,
) -> StageCAuthoritySummary {
    let authority = StageCAuthoritySummary::from_plan(
        channel,
        BUILD_FEATURES,
        plan.schema_revision,
        plan.authority.acquisition,
        plan.execution_plan(),
        plan.authority,
    );
    stage_c_candidate.install_authority(authority);
    authority
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

/// Typed outcome of the breakaway acceptance-point installer.
///
/// The no-terminus refusal is a distinct variant, not folded into an ordinary
/// non-install, so the caller can attach a dedicated cause instead of silently
/// dropping a breakaway `Complete` that carried no target terminus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AcceptanceInstall {
    /// A reproduced breakaway target run yielded one installable point.
    Installed(AcceptancePoint),
    /// The terminal is not a breakaway acceptance terminal (no install).
    NotAcceptanceTerminal,
    /// A breakaway acceptance `Complete` that recorded no target terminus.
    MissingTargetTerminus,
}

/// Derive one token-carrying matrix handoff from a reproduced breakaway `Complete`.
///
/// A breakaway point sources the token and signed recovery intervals from the
/// run's breakaway authority; it cannot be reconstructed from the request
/// alone. The token is extracted from `authority.acquisition` via
/// [`crate::commissioning::outer::velocity::integral_plan::StageCAcquisition::confirmed_breakaway`],
/// never constructed, so the confirmation's unforgeability is preserved.
///
/// `Err(())` is an evidence-integrity fault: a breakaway acceptance `Complete`
/// whose recorded target status, plan digest, gain, or ladder index does not
/// hold together.
pub(crate) fn breakaway_acceptance_point(
    channel: u8,
    authority: &StageCAuthoritySummary,
    plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralExecutionPlan,
    terminal: &VelocityIntegralTerminal,
) -> Result<AcceptanceInstall, ()> {
    use crate::commissioning::outer::velocity::combined_plan::{
        BREAKAWAY_STAGE_C_SCHEMA_REVISION, CombinedTargetStatus,
    };
    use crate::commissioning::outer::velocity::integral_reporting::{
        terminal_target_status, terminal_target_terminus,
    };
    use crate::commissioning::outer::velocity::{
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG, integral_i_at,
    };

    // (1) Not a breakaway acceptance terminal: nothing to install. The
    // terminus-independent `breakaway_acceptance` flag is what keeps a
    // no-terminus breakaway `Complete` from being dismissed here, so it can
    // reach the distinct missing-terminus branch below.
    if !terminal.breakaway_acceptance {
        return Ok(AcceptanceInstall::NotAcceptanceTerminal);
    }
    // (2) Defensive: the caller invokes only on a promoted `Complete`.
    if terminal.outcome != VelocityIntegralOutcome::Complete {
        return Ok(AcceptanceInstall::NotAcceptanceTerminal);
    }
    // (3) A breakaway `Complete` that recorded no target terminus is a distinct
    // refusal, not a fabricated point.
    let Some(positive_index) = terminal_target_terminus(terminal) else {
        return Ok(AcceptanceInstall::MissingTargetTerminus);
    };
    // (4) A recorded terminus must be a genuinely target-reaching run whose
    // identity matches the authority the promotion held.
    if terminal_target_status(terminal) != Some(CombinedTargetStatus::TargetReached) {
        return Err(());
    }
    if terminal.plan_digest != plan.plan_digest || terminal.final_p != authority.final_p {
        return Err(());
    }
    let selected_i = integral_i_at(plan, positive_index.checked_add(1).ok_or(())?).ok_or(())?;
    // Extracted, never constructed: preserves confirmation unforgeability.
    let token = authority.acquisition.confirmed_breakaway();
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
    Ok(AcceptanceInstall::Installed(AcceptancePoint {
        identity: StageCPlanIdentity {
            channel,
            build_features: BUILD_FEATURES,
            schema_revision: BREAKAWAY_STAGE_C_SCHEMA_REVISION,
            plan_digest: terminal.plan_digest,
        },
        selected_p: authority.final_p,
        selected_i,
        target_velocity_rpm: authority.target_velocity_rpm,
        selected_response_intervals_q: authority.directional_pooled_intervals,
        confidence_q: authority.confidence_q,
        band_position_q: authority.band_position_q,
        qualifier_bits,
        token,
        progress: AcceptanceMatrixProgress::ReadyAscending,
    }))
}

/// Device parameters needed to reproduce a retained Stage-C authority's plan.
///
/// Grouped out of `preflight_stage_c_request`'s argument list to bring it under the
/// project's <=5-parameter rule: these five are the axis/hardware description the
/// preflight reproduces against, distinct from the per-call request and sequence.
pub struct StageCPreflightConfig {
    pub encoder_counts_per_rev: u32,
    pub torque_limit: u16,
    pub limit_evidence_capability: u32,
    pub channel: u8,
    pub pwm_hz: u32,
}

/// Admit one resume against retained breakaway authority.
///
/// The torque limits and evidence capability come from the retained authority
/// because the campaign measured its usable limit from the current observer, so
/// recomputing them here could plan against a different boundary than the one
/// the confirmation evidence was gathered under.
///
/// PWM and encoder resolution are the opposite case: they must be compared
/// against live configuration, which is why both are passed in separately from
/// the retained values the context freezes. The plan digest cannot police this
/// on its own, because a plan rebuilt from retained context hashes those same
/// retained values and still matches the retained identity while execution does
/// its target-rate math against the stale resolution.
fn preflight_retained_breakaway(
    token: crate::commissioning::outer::velocity::integral_plan::ConfirmedAcquisitionDigest,
    authority: StageCAuthoritySummary,
    motion_budget: OuterMotionBudget,
    config: &StageCPreflightConfig,
    run_sequence: u32,
    stage_c_candidate: &StageCCandidateSlot,
) -> Result<Option<StageCPreflight>, StageCPreflightRejection> {
    use crate::commissioning::outer::velocity::integral_plan::{
        AcquisitionContext, plan_retained_breakaway_velocity_integral_response,
    };

    let context = AcquisitionContext {
        planned_velocity_mrev_s: authority.planned_velocity_mrev_s,
        target_velocity_rpm: authority.target_velocity_rpm,
        hard_torque_limit: authority.hard_torque_limit,
        usable_torque_limit: authority.usable_torque_limit,
        limit_evidence_capability: authority.limit_evidence_capability,
        channel: authority.identity.channel,
        plan_time_pwm_hz: authority.pwm_hz,
        plan_time_encoder_counts_per_rev: authority.encoder_counts_per_rev,
    };
    let plan = match plan_retained_breakaway_velocity_integral_response(
        token,
        authority.final_p,
        context,
        authority.directional_pooled_intervals,
        authority.confidence_q,
        authority.band_position_q,
        motion_budget,
        config.pwm_hz,
        config.encoder_counts_per_rev,
    ) {
        Ok(plan) => plan,
        Err(VelocityIntegralPlanError::NoTransitionCapableOperatingPoint(failure)) => {
            return Err(StageCPreflightRejection::NoTransitionCapableOperatingPoint(
                failure,
            ));
        }
        Err(_) => return Err(StageCPreflightRejection::InvalidPlan),
    };
    let identity = StageCPlanIdentity {
        channel: authority.identity.channel,
        build_features: BUILD_FEATURES,
        schema_revision: plan.schema_revision,
        plan_digest: plan.digest(),
    };
    // The sole identity check on this path: the rebuilt plan must be the exact
    // plan that was retained. A request that changed anything material produces
    // a different digest and is refused before any stroke.
    if stage_c_candidate.preflight(identity).is_some() {
        return Err(StageCPreflightRejection::RejectedPlanMismatch {
            retained: authority.identity,
        });
    }
    Ok(Some(StageCPreflight {
        plan,
        identity,
        run_sequence,
    }))
}

/// Reconstruct and identity-check retained Stage-C authority before any SPI work.
///
/// `Ok(None)` means no Stage-C authority exists and admission must proceed to Stage B.
pub fn preflight_stage_c_request(
    request: VelocitySweepRequest,
    config: &StageCPreflightConfig,
    run_sequence: u32,
    stage_c_candidate: &StageCCandidateSlot,
) -> Result<Option<StageCPreflight>, StageCPreflightRejection> {
    let Some(authority) = stage_c_candidate.authority() else {
        return Ok(None);
    };
    if authority.identity.channel != config.channel {
        return Err(StageCPreflightRejection::RejectedPlanMismatch {
            retained: authority.identity,
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
    let Some(token) = authority.acquisition.confirmed_breakaway() else {
        return Err(StageCPreflightRejection::InvalidPlan);
    };
    preflight_retained_breakaway(
        token,
        authority,
        motion_budget,
        config,
        run_sequence,
        stage_c_candidate,
    )
}

/// Apply one Stage-C terminal to the authoritative RAM-only lifecycle.
pub fn apply_stage_c_terminal(
    mut terminal: VelocityIntegralTerminal,
    stage_c_candidate: &mut StageCCandidateSlot,
) -> StageCTerminalDispatch {
    let Some(authority) = stage_c_candidate.authority() else {
        terminal.outcome = VelocityIntegralOutcome::Fault;
        terminal.cause = StageCCause::Dispatch(STAGE_C_CAUSE_EVIDENCE_INTEGRITY);
        return StageCTerminalDispatch {
            terminal,
            reproduction: None,
            authority: None,
        };
    };
    if terminal.plan_digest != authority.identity.plan_digest {
        stage_c_candidate.observe_fault();
        terminal.outcome = VelocityIntegralOutcome::Fault;
        terminal.cause = StageCCause::Dispatch(STAGE_C_CAUSE_EVIDENCE_INTEGRITY);
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
                    terminal.cause = StageCCause::Dispatch(STAGE_C_CAUSE_REPRODUCTION_MISMATCH);
                    Some(StageCReproductionReport {
                        delta,
                        outcome: 2,
                        previous_run_digest,
                        current_run_digest: terminal.digest,
                    })
                }
                StageCReproductionDecision::RejectedPlanMismatch { .. } => {
                    terminal.outcome = VelocityIntegralOutcome::RejectedPlanMismatch;
                    terminal.cause = StageCCause::Dispatch(STAGE_C_CAUSE_PLAN_MISMATCH);
                    None
                }
                StageCReproductionDecision::EvidenceIntegrityFault => {
                    terminal.outcome = VelocityIntegralOutcome::Fault;
                    terminal.cause = StageCCause::Dispatch(STAGE_C_CAUSE_EVIDENCE_INTEGRITY);
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
            terminal.cause = StageCCause::Dispatch(STAGE_C_CAUSE_EVIDENCE_INTEGRITY);
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
/// Terminal for a Stage-C resume with nothing retained to resume from.
///
/// Reports a zero plan digest because there is genuinely no exact plan: the
/// slot held no authority to reconstruct one from. Every other failed-admission
/// terminal names the plan it rejected, so consumers must branch on the cause
/// rather than assume a digest is present.
pub fn missing_authority_stage_c_terminal(run_sequence: u32) -> VelocityIntegralTerminal {
    use crate::commissioning::outer::velocity::integral_evidence::{
        ExactRateInterval, IntegralRunEvidence,
    };

    VelocityIntegralTerminal {
        run_sequence,
        evidence_sequence: 0,
        plan_digest: 0,
        digest: 0,
        outcome: VelocityIntegralOutcome::Failed,
        cause: StageCCause::Dispatch(STAGE_C_CAUSE_NO_RETAINED_AUTHORITY),
        breakaway_acceptance: false,
        final_p: 0,
        recovery_unavailable: false,
        recovered_with_current_headroom: false,
        recovery_quantization_exposed: false,
        probe_constrained_test_point: false,
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
        rest_rejection_after_sufficiency: false,
        rest_rejection_owner: None,
    }
}

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
        cause: StageCCause::Dispatch(STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT),
        breakaway_acceptance: false,
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
        rest_rejection_after_sufficiency: false,
        rest_rejection_owner: None,
    }
}
