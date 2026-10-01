//! Board-owned Integral admission and retained-evidence policy.

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::integral_plan::{
    IntegralFailedAdmission, VelocityIntegralPlan, VelocityIntegralPlanError,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptancePoint, IntegralAuthoritySummary, IntegralPlanIdentity, IntegralReproductionDecision,
    IntegralReproductionDelta, IntegralRestBoundaryReference, IntegralRetentionSlot,
    RetainedIntegralEvidence,
};
use crate::commissioning::outer::velocity::reproduction::BUILD_FEATURES;
use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;
use crate::commissioning::types::{
    IntegralOutcomeCause, VelocityIntegralOutcome, VelocityIntegralTerminal,
};

/// Dedicated cause for retained or live Integral evidence violating its contract.
pub const INTEGRAL_DISPATCH_CAUSE_EVIDENCE_INTEGRITY: u8 = 4;
/// Dedicated cause for a sufficient curve that did not reproduce.
pub const INTEGRAL_DISPATCH_CAUSE_REPRODUCTION_MISMATCH: u8 = 6;
/// Dedicated cause for a frozen Integral plan mismatch.
pub const INTEGRAL_DISPATCH_CAUSE_PLAN_MISMATCH: u8 = 7;
/// No reproduced operating rung can run under the Integral probe clamp.
pub const INTEGRAL_DISPATCH_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT: u8 = 11;
/// An Integral resume was requested with nothing retained to resume from.
pub const INTEGRAL_DISPATCH_CAUSE_NO_RETAINED_AUTHORITY: u8 = 12;
/// A breakaway acceptance `RepeatabilityConfirmed` recorded no target terminus.
pub const INTEGRAL_DISPATCH_CAUSE_NO_ACCEPTANCE_TERMINUS: u8 = 13;
/// An Integral terminal arrived with no retained authority to dispatch against.
pub const INTEGRAL_DISPATCH_CAUSE_AUTHORITY_LOST_MID_RUN: u8 = 14;
/// An Integral terminal arrived already carrying an outcome only this dispatch assigns.
pub const INTEGRAL_DISPATCH_CAUSE_UNEXPECTED_TERMINAL_OUTCOME: u8 = 15;

const ACCEPTANCE_QUALIFIER_RECOVERY_QUANTIZATION_EXPOSED: u16 = 1 << 7;
const ACCEPTANCE_QUALIFIER_PROBE_CONSTRAINED_TEST_POINT: u16 = 1 << 8;
const ACCEPTANCE_QUALIFIER_LATCHED_LIMIT_EVIDENCE: u16 = 1 << 9;

/// Immutable plan and identity accepted before Integral actuation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntegralPreflight {
    pub plan: VelocityIntegralPlan,
    pub identity: IntegralPlanIdentity,
    pub run_sequence: u32,
}

/// Install retained Integral authority from one exact execution plan.
pub fn install_authority_from_plan(
    channel: u8,
    integral_candidate: &mut IntegralRetentionSlot,
    plan: VelocityIntegralPlan,
) -> IntegralAuthoritySummary {
    let authority = IntegralAuthoritySummary::from_plan(
        channel,
        BUILD_FEATURES,
        plan.schema_revision,
        plan.authority.acquisition,
        plan.execution_plan(),
        plan.authority,
    );
    integral_candidate.install_authority(authority);
    authority
}

/// Fail-closed reason a retained Integral request was rejected before actuation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralPreflightRejection {
    InvalidPlan,
    RejectedPlanMismatch { retained: IntegralPlanIdentity },
    NoTransitionCapableOperatingPoint(IntegralFailedAdmission),
}

/// Cross-run comparison attached to the Integral terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralReproductionReport {
    pub delta: IntegralReproductionDelta,
    /// One for RepeatabilityConfirmed, two for RepeatabilityMismatch.
    pub outcome: u8,
    pub previous_run_digest: u64,
    pub current_run_digest: u64,
}

/// Integral terminal after firmware-owned retention policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralTerminalDispatch {
    pub terminal: VelocityIntegralTerminal,
    /// Comparison evidence present only for a second sufficient run.
    pub reproduction: Option<IntegralReproductionReport>,
    pub authority: Option<IntegralAuthoritySummary>,
}

/// Typed outcome of the breakaway acceptance-point installer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AcceptanceInstall {
    Installed(AcceptancePoint),
    NotAcceptanceTerminal,
    MissingTargetTerminus,
}

pub(crate) fn breakaway_acceptance_point(
    channel: u8,
    authority: &IntegralAuthoritySummary,
    plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralExecutionPlan,
    terminal: &VelocityIntegralTerminal,
) -> Result<AcceptanceInstall, ()> {
    use crate::commissioning::outer::velocity::combined_plan::{
        BREAKAWAY_INTEGRAL_SCHEMA_REVISION, CombinedTargetStatus,
    };
    use crate::commissioning::outer::velocity::integral_reporting::{
        terminal_target_status, terminal_target_terminus,
    };
    use crate::commissioning::outer::velocity::{
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG, integral_i_at,
    };

    if !terminal.breakaway_acceptance {
        return Ok(AcceptanceInstall::NotAcceptanceTerminal);
    }
    if terminal.outcome != VelocityIntegralOutcome::RepeatabilityConfirmed {
        return Ok(AcceptanceInstall::NotAcceptanceTerminal);
    }
    let Some(positive_index) = terminal_target_terminus(terminal) else {
        return Ok(AcceptanceInstall::MissingTargetTerminus);
    };
    if terminal_target_status(terminal) != Some(CombinedTargetStatus::TargetReached) {
        return Err(());
    }
    if terminal.plan_digest != plan.plan_digest || terminal.fixed_p != authority.fixed_p {
        return Err(());
    }
    let selected_i = integral_i_at(plan, positive_index.checked_add(1).ok_or(())?).ok_or(())?;
    let token = Some(authority.acquisition);
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
        identity: IntegralPlanIdentity {
            channel,
            build_features: BUILD_FEATURES,
            schema_revision: BREAKAWAY_INTEGRAL_SCHEMA_REVISION,
            plan_digest: terminal.plan_digest,
        },
        selected_p: authority.fixed_p,
        selected_i,
        target_velocity_rpm: authority.target_velocity_rpm,
        selected_response_intervals_q: authority.directional_pooled_intervals,
        confidence_q: authority.confidence_q,
        band_position_q: authority.band_position_q,
        qualifier_bits,
        token,
    }))
}

/// Device parameters needed to reproduce a retained Integral authority's plan.
pub struct IntegralPreflightConfig {
    pub encoder_counts_per_rev: u32,
    pub torque_limit: u16,
    pub limit_evidence_capability: u32,
    pub channel: u8,
    pub pwm_hz: u32,
}

fn preflight_retained_breakaway(
    token: crate::commissioning::outer::velocity::integral_plan::ConfirmedAcquisitionDigest,
    authority: IntegralAuthoritySummary,
    motion_budget: OuterMotionBudget,
    config: &IntegralPreflightConfig,
    run_sequence: u32,
    integral_candidate: &IntegralRetentionSlot,
) -> Result<Option<IntegralPreflight>, IntegralPreflightRejection> {
    use crate::commissioning::outer::velocity::integral_plan::{
        AcquisitionContext, BreakawayIntegralEvidence, IntegralPlanPhysics,
        plan_retained_breakaway_velocity_integral_response,
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
        authority.fixed_p,
        context,
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: authority.directional_pooled_intervals,
            confidence_q: authority.confidence_q,
            band_position_q: authority.band_position_q,
        },
        IntegralPlanPhysics {
            motion_budget,
            pwm_hz: config.pwm_hz,
            encoder_counts_per_rev: config.encoder_counts_per_rev,
        },
    ) {
        Ok(plan) => plan,
        Err(VelocityIntegralPlanError::NoTransitionCapableOperatingPoint(failure)) => {
            return Err(IntegralPreflightRejection::NoTransitionCapableOperatingPoint(failure));
        }
        Err(_) => return Err(IntegralPreflightRejection::InvalidPlan),
    };
    let identity = IntegralPlanIdentity {
        channel: authority.identity.channel,
        build_features: BUILD_FEATURES,
        schema_revision: plan.schema_revision,
        plan_digest: plan.digest(),
    };
    if integral_candidate.preflight(identity).is_some() {
        return Err(IntegralPreflightRejection::RejectedPlanMismatch {
            retained: authority.identity,
        });
    }
    Ok(Some(IntegralPreflight {
        plan,
        identity,
        run_sequence,
    }))
}

/// Reconstruct and identity-check retained Integral authority before any SPI work.
///
/// `Ok(None)` means no Integral authority exists and admission must proceed to Proportional.
pub fn preflight_integral_request(
    request: VelocitySweepRequest,
    config: &IntegralPreflightConfig,
    run_sequence: u32,
    integral_candidate: &IntegralRetentionSlot,
) -> Result<Option<IntegralPreflight>, IntegralPreflightRejection> {
    let Some(authority) = integral_candidate.authority() else {
        return Ok(None);
    };
    if authority.identity.channel != config.channel {
        return Err(IntegralPreflightRejection::RejectedPlanMismatch {
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
        homing_speed_mrev_s: 0,
        max_accel_mrev_s2: 0,
    };
    preflight_retained_breakaway(
        authority.acquisition,
        authority,
        motion_budget,
        config,
        run_sequence,
        integral_candidate,
    )
}

/// Apply one Integral terminal to the authoritative RAM-only lifecycle.
pub fn apply_integral_terminal(
    mut terminal: VelocityIntegralTerminal,
    integral_candidate: &mut IntegralRetentionSlot,
) -> IntegralTerminalDispatch {
    let Some(authority) = integral_candidate.authority() else {
        terminal.outcome = VelocityIntegralOutcome::Fault;
        terminal.cause =
            IntegralOutcomeCause::Dispatch(INTEGRAL_DISPATCH_CAUSE_AUTHORITY_LOST_MID_RUN);
        return IntegralTerminalDispatch {
            terminal,
            reproduction: None,
            authority: None,
        };
    };
    if terminal.plan_digest != authority.identity.plan_digest {
        integral_candidate.observe_fault();
        terminal.outcome = VelocityIntegralOutcome::Fault;
        terminal.cause = IntegralOutcomeCause::Dispatch(INTEGRAL_DISPATCH_CAUSE_PLAN_MISMATCH);
        return IntegralTerminalDispatch {
            terminal,
            reproduction: None,
            authority: Some(authority),
        };
    }

    let reproduction = match terminal.outcome {
        VelocityIntegralOutcome::FirstRunRetained => {
            let previous_run_digest = integral_candidate
                .candidate()
                .map_or(0, |candidate| candidate.run_digest);
            let current = RetainedIntegralEvidence {
                authority,
                evidence: terminal.evidence,
                rest_boundary: terminal
                    .rest_boundary_rung_plus_one
                    .checked_sub(1)
                    .zip(terminal.rest_boundary_slot_plus_one.checked_sub(1))
                    .map(
                        |(positive_rung_index, slot)| IntegralRestBoundaryReference {
                            positive_rung_index,
                            slot,
                        },
                    ),
                target_terminus_rung_plus_one: if terminal.rest_boundary_slot_plus_one == 0 {
                    terminal.rest_boundary_rung_plus_one
                } else {
                    0
                },
                run_completed_us: terminal.run_completed_us,
                run_digest: terminal.digest,
            };
            let previous_target_terminus_rung_plus_one = integral_candidate
                .candidate()
                .map_or(0, |candidate| candidate.target_terminus_rung_plus_one);
            match integral_candidate.observe_sufficient(current) {
                IntegralReproductionDecision::FirstRunRetained(_) => None,
                IntegralReproductionDecision::RepeatabilityConfirmed(delta) => {
                    terminal.outcome = VelocityIntegralOutcome::RepeatabilityConfirmed;
                    if terminal.rest_boundary_rung_plus_one == 0
                        && terminal.rest_boundary_slot_plus_one == 0
                        && previous_target_terminus_rung_plus_one != 0
                    {
                        terminal.rest_boundary_rung_plus_one =
                            previous_target_terminus_rung_plus_one;
                    }
                    Some(IntegralReproductionReport {
                        delta,
                        outcome: 1,
                        previous_run_digest,
                        current_run_digest: terminal.digest,
                    })
                }
                IntegralReproductionDecision::RepeatabilityMismatch(delta) => {
                    terminal.outcome = VelocityIntegralOutcome::Inconclusive;
                    terminal.cause = IntegralOutcomeCause::Dispatch(
                        INTEGRAL_DISPATCH_CAUSE_REPRODUCTION_MISMATCH,
                    );
                    defmt::warn!(
                        "integral reproduction mismatch: reproduced=[{=u32:#x}, {=u32:#x}] divergent=[{=u32:#x}, {=u32:#x}]",
                        delta.reproduced_masks[0],
                        delta.reproduced_masks[1],
                        delta.divergent_masks[0],
                        delta.divergent_masks[1],
                    );
                    Some(IntegralReproductionReport {
                        delta,
                        outcome: 2,
                        previous_run_digest,
                        current_run_digest: terminal.digest,
                    })
                }
                IntegralReproductionDecision::RejectedPlanMismatch { .. } => {
                    terminal.outcome = VelocityIntegralOutcome::RejectedPlanMismatch;
                    terminal.cause =
                        IntegralOutcomeCause::Dispatch(INTEGRAL_DISPATCH_CAUSE_PLAN_MISMATCH);
                    None
                }
                IntegralReproductionDecision::EvidenceIntegrityFault => {
                    terminal.outcome = VelocityIntegralOutcome::Fault;
                    terminal.cause =
                        IntegralOutcomeCause::Dispatch(INTEGRAL_DISPATCH_CAUSE_EVIDENCE_INTEGRITY);
                    None
                }
            }
        }
        VelocityIntegralOutcome::Inconclusive => {
            integral_candidate.observe_inconclusive();
            None
        }
        VelocityIntegralOutcome::Fault => {
            integral_candidate.observe_fault();
            None
        }
        VelocityIntegralOutcome::Failed => {
            integral_candidate.observe_fault();
            None
        }
        VelocityIntegralOutcome::RepeatabilityConfirmed
        | VelocityIntegralOutcome::RejectedPlanMismatch => {
            integral_candidate.observe_fault();
            terminal.outcome = VelocityIntegralOutcome::Fault;
            terminal.cause =
                IntegralOutcomeCause::Dispatch(INTEGRAL_DISPATCH_CAUSE_UNEXPECTED_TERMINAL_OUTCOME);
            None
        }
    };
    IntegralTerminalDispatch {
        terminal,
        reproduction,
        authority: Some(authority),
    }
}

/// Terminal for an Integral resume with nothing retained to resume from.
///
/// Reports a zero plan digest: no authority existed to reconstruct a plan from.
pub fn missing_authority_integral_terminal(run_sequence: u32) -> VelocityIntegralTerminal {
    use crate::commissioning::outer::velocity::integral_evidence::{
        ExactRateInterval, IntegralRunEvidence,
    };

    VelocityIntegralTerminal {
        run_sequence,
        evidence_sequence: 0,
        plan_digest: 0,
        digest: 0,
        outcome: VelocityIntegralOutcome::Failed,
        cause: IntegralOutcomeCause::Dispatch(INTEGRAL_DISPATCH_CAUSE_NO_RETAINED_AUTHORITY),
        breakaway_acceptance: false,
        fixed_p: 0,
        candidate_i: None,
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

pub(crate) fn failed_integral_terminal(
    run_sequence: u32,
    failure: IntegralFailedAdmission,
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
        cause: IntegralOutcomeCause::Dispatch(
            INTEGRAL_DISPATCH_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT,
        ),
        breakaway_acceptance: false,
        fixed_p: 0,
        candidate_i: None,
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
