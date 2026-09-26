//! Board-owned Integral admission and retained-evidence policy.

use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::integral_plan::{
    IntegralFailedAdmission, VelocityIntegralPlan, VelocityIntegralPlanError,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptancePoint, FixedGainAmplitudeProgress, IntegralAuthoritySummary, IntegralPlanIdentity,
    IntegralReproductionDecision, IntegralReproductionDelta, IntegralRestBoundaryReference,
    IntegralRetentionSlot, RetainedIntegralEvidence,
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
///
/// Distinct from every rejection cause: the request was well formed and the
/// slot is simply empty, which is the ordinary state after any reset because
/// retention is RAM-only. Sharing a cause with a rejection would tell the
/// operator to inspect a mismatch that does not exist.
pub const INTEGRAL_DISPATCH_CAUSE_NO_RETAINED_AUTHORITY: u8 = 12;
/// A breakaway acceptance `RepeatabilityConfirmed` recorded no target terminus.
///
/// Distinct from an evidence-integrity fault: the run was well formed and simply
/// did not reach a target terminus, so no acceptance point is installed. Sharing
/// a cause with a fault would tell the operator to inspect an integrity breach
/// that did not occur.
pub const INTEGRAL_DISPATCH_CAUSE_NO_ACCEPTANCE_TERMINUS: u8 = 13;
/// An Integral terminal arrived with no retained authority to dispatch against.
///
/// Distinct from [`INTEGRAL_DISPATCH_CAUSE_NO_RETAINED_AUTHORITY`]: a
/// terminal only exists because a run was dispatched, which requires
/// authority to have been present at start, so its absence here means
/// authority was lost mid-run, not the ordinary empty-slot state.
pub const INTEGRAL_DISPATCH_CAUSE_AUTHORITY_LOST_MID_RUN: u8 = 14;
/// An Integral terminal arrived already carrying an outcome only this
/// dispatch is meant to assign. Code 15.
///
/// Not reachable via `apply_integral_terminal`, its only production caller
/// as of this writing: no production path constructs a
/// `VelocityIntegralTerminal` with `RepeatabilityConfirmed` or
/// `RejectedPlanMismatch` before it reaches this function -- allocated
/// because the underlying check is a real code path, not because current
/// production input can reach it.
pub const INTEGRAL_DISPATCH_CAUSE_UNEXPECTED_TERMINAL_OUTCOME: u8 = 15;

const ACCEPTANCE_QUALIFIER_RECOVERY_QUANTIZATION_EXPOSED: u16 = 1 << 7;
const ACCEPTANCE_QUALIFIER_PROBE_CONSTRAINED_TEST_POINT: u16 = 1 << 8;
const ACCEPTANCE_QUALIFIER_LATCHED_LIMIT_EVIDENCE: u16 = 1 << 9;

/// Immutable plan and identity accepted before Integral actuation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntegralPreflight {
    /// The sole exact Integral plan consumed by construction and reporting.
    pub plan: VelocityIntegralPlan,
    /// Exact cross-run identity derived from the plan.
    pub identity: IntegralPlanIdentity,
    /// Caller-assigned trace sequence.
    pub run_sequence: u32,
}

/// Install retained Integral authority from one exact execution plan.
///
/// Public so a board can retain authority directly from the scheduled plan
/// event; an Integral terminal arriving with an empty slot fails closed.
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
    /// The current request could not reconstruct a safe exact Integral plan.
    InvalidPlan,
    /// The current request differs from retained Integral authority.
    RejectedPlanMismatch {
        /// Exact retained Integral identity.
        retained: IntegralPlanIdentity,
    },
    /// The reproduced operating component contains no Integral-safe test point.
    NoTransitionCapableOperatingPoint(IntegralFailedAdmission),
}

/// Cross-run comparison attached to the Integral terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralReproductionReport {
    /// One exact retained-versus-current comparison.
    pub delta: IntegralReproductionDelta,
    /// One for RepeatabilityConfirmed, two for RepeatabilityMismatch.
    pub outcome: u8,
    /// Exact retained-run evidence digest.
    pub previous_run_digest: u64,
    /// Exact current-run evidence digest.
    pub current_run_digest: u64,
}

/// Integral terminal after firmware-owned retention policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralTerminalDispatch {
    /// Authoritative terminal class after comparison.
    pub terminal: VelocityIntegralTerminal,
    /// Comparison evidence present only for a second sufficient run.
    pub reproduction: Option<IntegralReproductionReport>,
    /// Retained Proportional authority used for anchor comparison and context.
    pub authority: Option<IntegralAuthoritySummary>,
}

/// Typed outcome of the breakaway acceptance-point installer.
///
/// The no-terminus refusal is a distinct variant, not folded into an ordinary
/// non-install, so the caller can attach a dedicated cause instead of silently
/// dropping a breakaway `RepeatabilityConfirmed` that carried no target terminus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AcceptanceInstall {
    /// A reproduced breakaway target run yielded one installable point.
    Installed(AcceptancePoint),
    /// The terminal is not a breakaway acceptance terminal (no install).
    NotAcceptanceTerminal,
    /// A breakaway acceptance `RepeatabilityConfirmed` that recorded no target terminus.
    MissingTargetTerminus,
}

/// Derive one token-carrying amplitude handoff from a breakaway `RepeatabilityConfirmed`.
///
/// A breakaway point sources the token and signed recovery intervals from the
/// run's breakaway authority; it cannot be reconstructed from the request
/// alone. The token is copied from `authority.acquisition`, never
/// constructed, so the confirmation's unforgeability is preserved.
///
/// `Err(())` is an evidence-integrity fault: a breakaway acceptance
/// `RepeatabilityConfirmed` whose recorded target status, plan digest, gain,
/// or ladder index does not hold together.
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

    // (1) Not a breakaway acceptance terminal: nothing to install. The
    // terminus-independent `breakaway_acceptance` flag is what keeps a
    // no-terminus breakaway `RepeatabilityConfirmed` from being dismissed here, so it can
    // reach the distinct missing-terminus branch below.
    if !terminal.breakaway_acceptance {
        return Ok(AcceptanceInstall::NotAcceptanceTerminal);
    }
    // (2) Defensive: the caller invokes only on a promoted `RepeatabilityConfirmed`.
    if terminal.outcome != VelocityIntegralOutcome::RepeatabilityConfirmed {
        return Ok(AcceptanceInstall::NotAcceptanceTerminal);
    }
    // (3) A breakaway `RepeatabilityConfirmed` that recorded no target terminus is a distinct
    // refusal, not a fabricated point.
    let Some(positive_index) = terminal_target_terminus(terminal) else {
        return Ok(AcceptanceInstall::MissingTargetTerminus);
    };
    // (4) A recorded terminus must be a genuinely target-reaching run whose
    // identity matches the authority the promotion held.
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
        progress: FixedGainAmplitudeProgress::ReadyAscending,
    }))
}

/// Device parameters needed to reproduce a retained Integral authority's plan.
///
/// Grouped out of `preflight_integral_request`'s argument list to bring it under the
/// project's <=5-parameter rule: these five are the axis/hardware description the
/// preflight reproduces against, distinct from the per-call request and sequence.
pub struct IntegralPreflightConfig {
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
    authority: IntegralAuthoritySummary,
    motion_budget: OuterMotionBudget,
    config: &IntegralPreflightConfig,
    run_sequence: u32,
    integral_candidate: &IntegralRetentionSlot,
) -> Result<Option<IntegralPreflight>, IntegralPreflightRejection> {
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
        authority.fixed_p,
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
    // The sole identity check on this path: the rebuilt plan must be the exact
    // plan that was retained. A request that changed anything material produces
    // a different digest and is refused before any stroke.
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
                // The breakaway target-terminus wire encoding
                // (`terminal_target_terminus`) is `rest_boundary_slot_plus_one
                // == 0`, disjoint from the current-headroom encoding `rest_boundary`
                // above requires (both fields nonzero), so it is retained here
                // in its own raw field rather than lost to that field's `.zip()`.
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
                    // The confirmation run's own terminus check need not
                    // re-trip for reproduction to succeed (a breakaway target
                    // terminus is not part of reproduction authority), so fall
                    // back to the terminus already proven by the first
                    // sufficient run rather than losing it on promotion.
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
                    // The sole defmt signal for why a resume came back Inconclusive:
                    // which shared rungs reproduced vs diverged, per direction.
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

/// Build the authoritative zero-motion terminal for a failed Integral admission.
/// Terminal for an Integral resume with nothing retained to resume from.
///
/// Reports a zero plan digest because there is genuinely no exact plan: the
/// slot held no authority to reconstruct one from. Every other failed-admission
/// terminal names the plan it rejected, so consumers must branch on the cause
/// rather than assume a digest is present.
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
