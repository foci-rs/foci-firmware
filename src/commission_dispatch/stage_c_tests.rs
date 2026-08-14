use super::stage_c::{
    STAGE_C_CAUSE_EVIDENCE_INTEGRITY, STAGE_C_CAUSE_NO_RETAINED_AUTHORITY,
    STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT, STAGE_C_CAUSE_PLAN_MISMATCH,
    STAGE_C_CAUSE_REPRODUCTION_MISMATCH, StageCPreflightConfig, StageCPreflightRejection,
    apply_stage_c_terminal, missing_authority_stage_c_terminal, preflight_stage_c_request,
};
use super::{CommissionOutcome, ReplyPayload};
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use crate::commissioning::outer::velocity::integral_evidence::{
    ExactRateInterval, IntegralRunEvidence,
};
use crate::commissioning::outer::velocity::integral_plan::StageCAcquisition;
use crate::commissioning::outer::velocity::integral_plan::{
    STAGE_C_EVIDENCE_SCHEMA_REVISION, StageCAuthority, plan_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    StageCAuthoritySummary, StageCCandidateSlot, StageCSlotState,
};
use crate::commissioning::outer::velocity::reproduction::{
    BUILD_FEATURES, StageBDirectionalValidity,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    RequestedVelocitySource, VelocitySweepRequest, plan_velocity_sweep,
};
use crate::commissioning::types::{VelocityIntegralOutcome, VelocityIntegralTerminal};
use foci_firmware_macros::fixture_test;

const CHANNEL: u8 = 0;
const PWM_HZ: u32 = 25_000;
const TORQUE_LIMIT: u16 = 2_816;
const COUNTS_PER_REV: u32 = 4_000;

fn preflight_config() -> StageCPreflightConfig {
    StageCPreflightConfig {
        encoder_counts_per_rev: COUNTS_PER_REV,
        torque_limit: TORQUE_LIMIT,
        limit_evidence_capability: VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        channel: CHANNEL,
        pwm_hz: PWM_HZ,
    }
}

fn request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 2_929,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    }
}

fn authority() -> StageCAuthority {
    StageCAuthority {
        channel: CHANNEL,
        acquisition: StageCAcquisition::ReproducedStageB(stage_b_plan().digest()),
        directional_memberships: [0x000e_0000; 2],
        stage_b_joint_membership: 0x000e_0000,
        joint_membership: 0x000e_0000,
        directional_validity: [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2],
        reduced_margin: false,
        directional_pooled_intervals: [[15_000_000, 17_000_000], [-19_000_000, -17_000_000]],
    }
}

fn stage_b_plan() -> crate::commissioning::outer::velocity::sweep_plan::VelocitySweepPlan {
    plan_velocity_sweep(
        request(),
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .expect("valid Stage-B plan")
}

fn stage_c_plan() -> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
    let stage_b = stage_b_plan();
    let request = request();
    plan_velocity_integral_response(
        authority(),
        stage_b.execution_plan(stage_b.digest()),
        OuterMotionBudget {
            requested_velocity_mrev_s: request.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: request.requested_velocity_source,
            max_stroke_travel_mrev: request.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: request.negative_position_headroom_mrev,
            positive_position_headroom_mrev: request.positive_position_headroom_mrev,
            max_duration_ms: request.max_duration_ms,
        },
        PWM_HZ,
        COUNTS_PER_REV,
    )
    .expect("valid Stage-C plan")
}

/// Mint one genuine breakaway Stage-C plan through the real confirmation gate.
///
/// The `ConfirmedAcquisitionDigest` token cannot be fabricated, so this walks
/// the same `plan_band_confirmation` -> `evaluate_band_confirmation` ->
/// `bind_breakaway_stage_c_authority` chain a live campaign uses (mirrors the
/// `integral_plan_tests` fixtures), then plans the campaign's own Stage-C plan.
fn breakaway_stage_c_plan()
-> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
    use crate::autotune::velocity_measurement::VelocityDirection;
    use crate::commissioning::outer::velocity::band_confirmation::{
        ConfirmationDirectionalInput, DiscoveryObservationAtom, evaluate_band_confirmation,
        plan_band_confirmation,
    };
    use crate::commissioning::outer::velocity::integral_plan::{
        AcquisitionContext, bind_breakaway_stage_c_authority,
        plan_breakaway_velocity_integral_response,
    };
    use crate::commissioning::outer::velocity::sweep_consensus::{
        CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    };
    use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
    use foci_tmc4671::pid::PidPGain;

    let candidate_p = PidPGain::new(512).expect("raw gain within PidPGain range");
    let confirmation_plan = plan_band_confirmation(0x1111_2222_3333_4444, candidate_p)
        .expect("valid confirmation plan");

    let consensus = || DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    };
    let mean_rate_q: i64 = 750_000;
    let variance_of_mean_q2: u128 = 58_752_225;
    let target_rate_q: i64 = 1_000_000;
    let forward = ConfirmationDirectionalInput {
        consensus: consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Forward,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let reverse = ConfirmationDirectionalInput {
        consensus: consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q: -mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Reverse,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let acceptance =
        evaluate_band_confirmation(candidate_p, forward, reverse).expect("accepted confirmation");
    let intervals = acceptance.held_out_response_intervals_q;
    let token = bind_breakaway_stage_c_authority(
        confirmation_plan.plan_digest,
        confirmation_plan,
        Ok(acceptance),
    )
    .expect("bound breakaway authority");

    let stage_b = stage_b_plan();
    let mut context = AcquisitionContext::from(stage_b.execution_plan(stage_b.digest()));
    context.plan_time_pwm_hz = PWM_HZ;
    context.plan_time_encoder_counts_per_rev = COUNTS_PER_REV;

    let request = request();
    plan_breakaway_velocity_integral_response(
        token,
        confirmation_plan,
        context,
        intervals,
        OuterMotionBudget {
            requested_velocity_mrev_s: request.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: request.requested_velocity_source,
            max_stroke_travel_mrev: request.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: request.negative_position_headroom_mrev,
            positive_position_headroom_mrev: request.positive_position_headroom_mrev,
            max_duration_ms: request.max_duration_ms,
        },
        PWM_HZ,
        COUNTS_PER_REV,
    )
    .expect("valid breakaway Stage-C plan")
}

/// The retained breakaway authority summary, as `apply_stage_c_terminal` holds
/// it at the reproduced-`Complete` promotion (`acquisition ==
/// ConfirmedBreakaway`, held-out intervals, campaign `final_p`/target).
fn breakaway_authority_summary(
    plan: &crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
) -> StageCAuthoritySummary {
    StageCAuthoritySummary::from_plan(
        CHANNEL,
        BUILD_FEATURES,
        plan.schema_revision,
        plan.authority.acquisition,
        plan.execution_plan(),
        plan.authority,
    )
}

/// One breakaway Stage-C terminal (`breakaway_acceptance` set, no combined bit).
fn breakaway_terminal(
    plan: &crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
    authority: &StageCAuthoritySummary,
    outcome: VelocityIntegralOutcome,
    evidence: IntegralRunEvidence,
) -> VelocityIntegralTerminal {
    VelocityIntegralTerminal {
        run_sequence: 9,
        evidence_sequence: 57,
        plan_digest: plan.digest(),
        digest: 0x1234_5678_9abc_def0,
        outcome,
        cause: 0,
        breakaway_acceptance: true,
        final_p: authority.final_p,
        recovery_unavailable: false,
        recovered_with_current_headroom: false,
        recovery_quantization_exposed: plan.recovery_quantization_exposed,
        probe_constrained_test_point: plan.probe_constrained_test_point,
        rest_boundary_rung_plus_one: 0,
        rest_boundary_slot_plus_one: 0,
        evidence,
        expected_observations: plan.expected_observations,
        emitted_observations: plan.expected_observations,
        expected_rungs: u16::from(plan.total_rung_count),
        emitted_rungs: u16::from(plan.total_rung_count),
        run_started_us: 1_000,
        run_completed_us: 2_000,
        safety_fault: None,
    }
}

#[test]
fn breakaway_reproduced_complete_installs_token_carrying_point() {
    use crate::commissioning::outer::velocity::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION;
    use crate::commissioning::outer::velocity::integral_i_at;

    let plan = breakaway_stage_c_plan();
    let authority = breakaway_authority_summary(&plan);
    let execution = plan.execution_plan();
    let mut terminal = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::Complete,
        sufficient_evidence(0),
    );
    // terminus == 2 (rest boundary at rung index 2, slot cleared).
    terminal.rest_boundary_rung_plus_one = 3;

    let installed =
        match super::stage_c::breakaway_acceptance_point(CHANNEL, &authority, execution, &terminal)
            .expect("no fault")
        {
            super::stage_c::AcceptanceInstall::Installed(point) => point,
            other => panic!("expected install, got {other:?}"),
        };
    assert_eq!(
        installed.identity.schema_revision,
        BREAKAWAY_STAGE_C_SCHEMA_REVISION
    );
    assert_eq!(installed.identity.plan_digest, terminal.plan_digest);
    assert_eq!(installed.selected_p, authority.final_p);
    assert_eq!(
        installed.selected_response_intervals_q,
        authority.directional_pooled_intervals
    );
    // Exact, not tautological: selected_i is the ladder value at terminus + 1.
    assert_eq!(
        installed.selected_i,
        integral_i_at(execution, 3).expect("in range")
    );
    // The token is carried so the matrix preflight can reconstruct the plan.
    assert_eq!(installed.token, authority.acquisition.confirmed_breakaway());
    assert!(installed.token.is_some());
}

#[test]
fn breakaway_complete_without_terminus_reports_missing_terminus() {
    let plan = breakaway_stage_c_plan();
    let authority = breakaway_authority_summary(&plan);
    let execution = plan.execution_plan();
    // Sufficiency only: no rest boundary, so no target terminus.
    let terminal = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::Complete,
        sufficient_evidence(0),
    );
    assert!(matches!(
        super::stage_c::breakaway_acceptance_point(CHANNEL, &authority, execution, &terminal)
            .expect("no fault"),
        super::stage_c::AcceptanceInstall::MissingTargetTerminus
    ));
}

#[test]
fn breakaway_non_complete_terminal_is_not_an_acceptance_terminal() {
    let plan = breakaway_stage_c_plan();
    let authority = breakaway_authority_summary(&plan);
    let execution = plan.execution_plan();
    let mut terminal = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::Inconclusive,
        sufficient_evidence(0),
    );
    terminal.rest_boundary_rung_plus_one = 3;
    assert!(matches!(
        super::stage_c::breakaway_acceptance_point(CHANNEL, &authority, execution, &terminal)
            .expect("no fault"),
        super::stage_c::AcceptanceInstall::NotAcceptanceTerminal
    ));
}

fn retained_authority() -> StageCAuthoritySummary {
    let plan = stage_c_plan();
    StageCAuthoritySummary::from_plan(
        CHANNEL,
        BUILD_FEATURES,
        STAGE_C_EVIDENCE_SCHEMA_REVISION,
        authority().acquisition,
        plan.execution_plan(),
        plan.authority,
    )
}

fn sufficient_evidence(offset: i32) -> IntegralRunEvidence {
    let mut evidence = IntegralRunEvidence::new([
        ExactRateInterval::new(100 + offset, 110 + offset).unwrap(),
        ExactRateInterval::new(-110 + offset, -100 + offset).unwrap(),
    ]);
    for direction in 0..2 {
        for rung in 0..3 {
            let base = if direction == 0 { 70 } else { -90 } + rung as i32;
            evidence
                .retain_positive(
                    direction,
                    rung,
                    ExactRateInterval::new(base + offset, base + offset + 5).unwrap(),
                )
                .unwrap();
        }
    }
    evidence
}

fn divergent_evidence() -> IntegralRunEvidence {
    let mut evidence = sufficient_evidence(0);
    for direction in 0..2 {
        for rung in 0..3 {
            let current = evidence.positive[direction][rung];
            evidence.positive[direction][rung] =
                ExactRateInterval::new(current.low_q + 1_000, current.high_q + 1_000).unwrap();
        }
    }
    evidence
}

fn terminal(
    outcome: VelocityIntegralOutcome,
    evidence: IntegralRunEvidence,
) -> VelocityIntegralTerminal {
    let plan = stage_c_plan();
    VelocityIntegralTerminal {
        run_sequence: 9,
        evidence_sequence: 57,
        plan_digest: plan.digest(),
        digest: 0x1234_5678_9abc_def0,
        outcome,
        cause: 0,
        breakaway_acceptance: false,
        final_p: 0,
        recovery_unavailable: false,
        recovered_with_current_headroom: false,
        recovery_quantization_exposed: plan.recovery_quantization_exposed,
        probe_constrained_test_point: plan.probe_constrained_test_point,
        rest_boundary_rung_plus_one: 0,
        rest_boundary_slot_plus_one: 0,
        evidence,
        expected_observations: plan.expected_observations,
        emitted_observations: plan.expected_observations,
        expected_rungs: u16::from(plan.total_rung_count),
        emitted_rungs: u16::from(plan.total_rung_count),
        run_started_us: 1_000,
        run_completed_us: 2_000,
        safety_fault: None,
    }
}

fn empty_evidence() -> IntegralRunEvidence {
    IntegralRunEvidence::new([ExactRateInterval::EMPTY; 2])
}

#[test]
fn a_resume_without_retained_authority_fails_closed_and_carries_no_plan() {
    // A resume asked for against an empty slot has no exact plan to name, so
    // the terminal reports a zero plan digest rather than inventing one. It is
    // its own cause: the request was well formed and the machine simply has
    // nothing retained, which is the ordinary state after any reset.
    let slot = StageCCandidateSlot::new();
    assert!(
        preflight_stage_c_request(request(), &preflight_config(), 4, &slot)
            .expect("an empty slot is not a rejection")
            .is_none()
    );

    let terminal = missing_authority_stage_c_terminal(4);

    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Failed);
    assert_eq!(terminal.cause, STAGE_C_CAUSE_NO_RETAINED_AUTHORITY);
    assert_ne!(
        terminal.cause,
        STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT
    );
    assert_eq!(terminal.run_sequence, 4);
    assert_eq!(terminal.plan_digest, 0);
    assert_eq!(terminal.digest, 0);
    assert_eq!(terminal.evidence_sequence, 0);
    assert_eq!(terminal.expected_observations, 0);
    assert_eq!(terminal.emitted_observations, 0);
    assert_eq!(terminal.expected_rungs, 0);
    assert_eq!(terminal.emitted_rungs, 0);
    assert!(terminal.safety_fault.is_none());
}

#[test]
fn retained_authority_regenerates_the_exact_stage_c_plan_before_motion() {
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained_authority());

    let preflight = preflight_stage_c_request(request(), &preflight_config(), 7, &slot)
        .expect("matching retained plan")
        .expect("nonempty Stage-C slot");

    assert_eq!(preflight.plan, stage_c_plan());
    assert_eq!(preflight.identity, retained_authority().identity);
    assert_eq!(preflight.run_sequence, 7);
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);
}

#[test]
fn changed_resume_plan_is_rejected_without_mutating_retention() {
    let mut slot = StageCCandidateSlot::new();
    let retained = retained_authority();
    slot.install_authority(retained);
    let mut changed = request();
    changed.max_stroke_travel_mrev += 1;

    let rejection = preflight_stage_c_request(changed, &preflight_config(), 8, &slot)
        .expect_err("changed frozen plan must reject before motion");

    assert!(matches!(
        rejection,
        StageCPreflightRejection::RejectedPlanMismatch { .. }
    ));
    assert_eq!(slot.authority(), Some(retained));
}

#[test]
fn velocity_integral_outcome_codes_and_stage_c_namespace_are_stable() {
    use crate::commissioning::outer::velocity::integral_reporting::outcome_code;

    assert_eq!(outcome_code(VelocityIntegralOutcome::CompleteCandidate), 0);
    assert_eq!(outcome_code(VelocityIntegralOutcome::Complete), 1);
    assert_eq!(outcome_code(VelocityIntegralOutcome::Inconclusive), 2);
    assert_eq!(outcome_code(VelocityIntegralOutcome::Fault), 3);
    assert_eq!(
        outcome_code(VelocityIntegralOutcome::RejectedPlanMismatch),
        4
    );
    assert_eq!(outcome_code(VelocityIntegralOutcome::Failed), 5);
    assert_eq!(STAGE_C_CAUSE_PLAN_MISMATCH, 7);
    assert_eq!(STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT, 11);
}

#[test]
fn sufficient_runs_advance_candidate_compare_and_complete_lifecycle() {
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained_authority());

    let mut first_terminal = terminal(
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(0),
    );
    first_terminal.rest_boundary_rung_plus_one = 4;
    first_terminal.rest_boundary_slot_plus_one = 6;
    let first = apply_stage_c_terminal(first_terminal, &mut slot);
    assert_eq!(
        first.terminal.outcome,
        VelocityIntegralOutcome::CompleteCandidate
    );
    assert!(first.reproduction.is_none());
    assert_eq!(slot.state(), StageCSlotState::Candidate);
    assert_eq!(
        slot.candidate().unwrap().rest_boundary.unwrap(),
        crate::commissioning::outer::velocity::integral_reproduction::StageCRestBoundaryReference {
            positive_rung_index: 3,
            slot: 5,
        }
    );

    let mut second_terminal = terminal(
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(2),
    );
    second_terminal.rest_boundary_rung_plus_one = 5;
    second_terminal.rest_boundary_slot_plus_one = 2;
    second_terminal.cause = 10;
    let second = apply_stage_c_terminal(second_terminal, &mut slot);
    assert_eq!(second.terminal.outcome, VelocityIntegralOutcome::Complete);
    assert_eq!(second.terminal.cause, 10);
    let reproduction = second.reproduction.unwrap();
    assert_eq!(
        reproduction
            .delta
            .previous_rest_boundary
            .unwrap()
            .positive_rung_index,
        3
    );
    assert_eq!(
        reproduction
            .delta
            .current_rest_boundary
            .unwrap()
            .positive_rung_index,
        4
    );
    assert_eq!(slot.state(), StageCSlotState::Empty);
}

#[test]
fn sufficient_mismatch_replaces_candidate_and_keeps_authority() {
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained_authority());
    apply_stage_c_terminal(
        terminal(
            VelocityIntegralOutcome::CompleteCandidate,
            sufficient_evidence(0),
        ),
        &mut slot,
    );

    let dispatch = apply_stage_c_terminal(
        terminal(
            VelocityIntegralOutcome::CompleteCandidate,
            divergent_evidence(),
        ),
        &mut slot,
    );

    assert_eq!(
        dispatch.terminal.outcome,
        VelocityIntegralOutcome::Inconclusive
    );
    assert_eq!(dispatch.terminal.cause, STAGE_C_CAUSE_REPRODUCTION_MISMATCH);
    assert!(dispatch.reproduction.is_some());
    assert_eq!(slot.state(), StageCSlotState::Candidate);
}

#[test]
fn inconclusive_preserves_authority_while_fault_clears_it() {
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained_authority());
    apply_stage_c_terminal(
        terminal(
            VelocityIntegralOutcome::CompleteCandidate,
            sufficient_evidence(0),
        ),
        &mut slot,
    );

    let inconclusive = apply_stage_c_terminal(
        terminal(VelocityIntegralOutcome::Inconclusive, empty_evidence()),
        &mut slot,
    );
    assert_eq!(
        inconclusive.terminal.outcome,
        VelocityIntegralOutcome::Inconclusive
    );
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);

    let fault = apply_stage_c_terminal(
        terminal(VelocityIntegralOutcome::Fault, empty_evidence()),
        &mut slot,
    );
    assert_eq!(fault.terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(slot.state(), StageCSlotState::Empty);
}

#[test]
fn unexpected_complete_terminal_fails_closed_in_retained_stage_c_dispatch() {
    // Combined workflow terminals are handled before this retained dispatcher.
    // Non-combined retained Stage C reports sufficiency as `CompleteCandidate`.
    // A bare `Complete` reaching this function is therefore a contract
    // violation, so it must not be accepted as success.
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained_authority());

    let dispatch = apply_stage_c_terminal(
        terminal(VelocityIntegralOutcome::Complete, sufficient_evidence(0)),
        &mut slot,
    );

    assert_eq!(dispatch.terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(dispatch.terminal.cause, STAGE_C_CAUSE_EVIDENCE_INTEGRITY);
    assert_eq!(slot.state(), StageCSlotState::Empty);
}

#[test]
fn every_engine_outcome_has_a_declared_dispatch_verdict() {
    // The engine has six outcomes and `apply_stage_c_terminal` has five arms;
    // nothing tested the cross-product, which is why three instances of
    // dispatch-level fault promotion reached hardware before a test. Each row
    // states the intended verdict, so grouping a success with a rejection
    // cannot pass unnoticed again.
    let expectations = [
        (
            VelocityIntegralOutcome::Complete,
            VelocityIntegralOutcome::Fault,
            STAGE_C_CAUSE_EVIDENCE_INTEGRITY,
        ),
        (
            VelocityIntegralOutcome::CompleteCandidate,
            VelocityIntegralOutcome::CompleteCandidate,
            0,
        ),
        (
            VelocityIntegralOutcome::Inconclusive,
            VelocityIntegralOutcome::Inconclusive,
            0,
        ),
        (
            VelocityIntegralOutcome::Fault,
            VelocityIntegralOutcome::Fault,
            0,
        ),
        (
            VelocityIntegralOutcome::Failed,
            VelocityIntegralOutcome::Failed,
            0,
        ),
        (
            VelocityIntegralOutcome::RejectedPlanMismatch,
            VelocityIntegralOutcome::Fault,
            STAGE_C_CAUSE_EVIDENCE_INTEGRITY,
        ),
    ];

    for (engine_outcome, expected_outcome, expected_cause) in expectations {
        let mut slot = StageCCandidateSlot::new();
        slot.install_authority(retained_authority());

        let dispatch =
            apply_stage_c_terminal(terminal(engine_outcome, sufficient_evidence(0)), &mut slot);

        assert_eq!(
            dispatch.terminal.outcome, expected_outcome,
            "engine outcome {engine_outcome:?} produced the wrong dispatch outcome"
        );
        assert_eq!(
            dispatch.terminal.cause, expected_cause,
            "engine outcome {engine_outcome:?} produced the wrong dispatch cause"
        );
    }
}

#[test]
fn inconclusive_rest_discards_candidate_but_retains_stage_b_authority() {
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained_authority());
    apply_stage_c_terminal(
        terminal(
            VelocityIntegralOutcome::CompleteCandidate,
            sufficient_evidence(0),
        ),
        &mut slot,
    );

    let mut rest_terminal = terminal(VelocityIntegralOutcome::Inconclusive, empty_evidence());
    rest_terminal.cause = crate::autotune::CommissionError::VelocityRestNotConfirmed.status_code();
    let dispatch = apply_stage_c_terminal(rest_terminal, &mut slot);

    assert_eq!(
        dispatch.terminal.outcome,
        VelocityIntegralOutcome::Inconclusive
    );
    assert_eq!(
        dispatch.terminal.cause,
        crate::autotune::CommissionError::VelocityRestNotConfirmed.status_code()
    );
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);
}

#[test]
fn same_channel_calibration_preserves_stage_c_and_other_channel_clears_it() {
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained_authority());

    slot.preserve_for_same_channel_calibration(CHANNEL);
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);
    slot.preserve_for_same_channel_calibration(CHANNEL + 1);
    assert_eq!(slot.state(), StageCSlotState::Empty);
}

fn outer_params() -> crate::commissioning::OuterTuneParams {
    crate::commissioning::OuterTuneParams {
        slot_order:
            crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder::ForwardFirst,
        profile: crate::autotune::CommissionProfile::Balanced,
        mode: crate::autotune::OperatingMode::Nominal,
        inner_lambda_us: 100,
        theta_e_us: 50,
        current_ringing: false,
        current_bandwidth_hz: 1_000,
        inner_warning_flags: 0,
        requested_velocity_mrev_s: request().requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request().machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request().requested_velocity_source,
        max_stroke_travel_mrev: request().max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request().settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request().negative_position_headroom_mrev,
        positive_position_headroom_mrev: request().positive_position_headroom_mrev,
        max_duration_ms: request().max_duration_ms,
    }
}

fn entry_snapshot() -> crate::commissioning::EntrySnapshot {
    crate::commissioning::EntrySnapshot {
        velocity_p: crate::pid::p_gain(1_152),
        velocity_i: crate::pid::i_gain(0),
        position_p: crate::pid::p_gain(640),
        position_i: crate::pid::i_gain(0),
        velocity_limit: 500_000,
        voltage_limit: 16_000,
        velocity_filter_hz: 0,
        torque_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
    }
}

fn stage_b_terminal() -> crate::commissioning::types::VelocitySweepTerminal {
    use crate::commissioning::types::{VelocitySweepOutcome, VelocitySweepTerminal};

    VelocitySweepTerminal {
        run_sequence: 8,
        evidence_sequence: 0,
        plan_digest: stage_b_plan().digest(),
        outcome: VelocitySweepOutcome::CompleteCandidate,
        cause: 0,
        recovery_unavailable: false,
        model_direction_mask: 3,
        selected_memberships: [0x000e_0000; 3],
        nominated_p: 1_024,
        selected_intervals: authority().directional_pooled_intervals,
        selected_common_intervals: [[15_500_000, 16_500_000], [-18_500_000, -17_500_000]],
        p_start: stage_b_plan().p_start.raw(),
        p_top: stage_b_plan().p_top.raw(),
        rung_count: stage_b_plan().rung_count,
        coverage_mask: 3,
        directional_region_counts: [1; 2],
        directional_fragment_counts: [0; 2],
        expected_observations: u16::from(stage_b_plan().rung_count) * 8,
        emitted_observations: u16::from(stage_b_plan().rung_count) * 8,
        expected_rungs: u16::from(stage_b_plan().rung_count),
        emitted_rungs: u16::from(stage_b_plan().rung_count),
        digest: 0x1234,
        run_started_us: 1_000,
        run_completed_us: 2_000,
        safety_fault: None,
    }
}

#[test]
fn stage_b_continuation_requires_a_successful_final_recovery() {
    let mut terminal = stage_b_terminal();
    terminal.outcome = crate::commissioning::types::VelocitySweepOutcome::Complete;
    assert!(super::stage_b_continuation_allowed(terminal));

    terminal.recovery_unavailable = true;
    assert!(!super::stage_b_continuation_allowed(terminal));
}

#[test]
fn stage_c_terminal_causes_reserve_recovery_and_lifecycle_values() {
    assert_eq!(super::stage_c::STAGE_C_CAUSE_EVIDENCE_INTEGRITY, 4);
    assert_eq!(super::stage_c::STAGE_C_CAUSE_REPRODUCTION_MISMATCH, 6);
    assert_eq!(super::stage_c::STAGE_C_CAUSE_PLAN_MISMATCH, 7);
}

#[cfg(feature = "trace-velocity-sweep")]
#[fixture_test]
async fn breakaway_stage_c_plan_ready_retains_terminal_authority() {
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();

    use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};
    use crate::commissioning::CommissioningEngine;
    use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
    use crate::commissioning::types::StepResult;

    let plan = stage_c_plan();
    let expected_authority = StageCAuthoritySummary::from_plan(
        CHANNEL,
        BUILD_FEATURES,
        plan.schema_revision,
        plan.authority.acquisition,
        plan.execution_plan(),
        plan.authority,
    );
    let mut stage_b_slot = StageBCandidateSlot::new();
    let mut stage_c_slot = StageCCandidateSlot::new();
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut calibrated = true;
    let mut enabled = true;
    let mut state = super::DispatchState::new(
        crate::state::tmc::tmc_channel0(),
        crate::state::stepper::stepper_channel0(),
        &mut calibrated,
        &mut enabled,
        &mut gains,
    );
    let mut engine = Some(CommissioningEngine::new_outer(
        1,
        outer_params(),
        entry_snapshot(),
    ));

    let outcome = super::drive_commission_result_with_stages(
        StepResult::BreakawayStageCPlanReady {
            run_sequence: 9,
            plan,
        },
        &mut engine,
        &mut backend,
        &mut state,
        4_095,
        CHANNEL,
        super::CommissionRetention::new(&mut stage_b_slot, &mut stage_c_slot),
    )
    .await;

    assert_eq!(stage_c_slot.state(), StageCSlotState::AuthorityOnly);
    assert_eq!(stage_c_slot.authority(), Some(expected_authority));
    assert!(matches!(
        outcome,
        CommissionOutcome::Evidence {
            reply: ReplyPayload::VelocityIntegralPlan {
                oid: 1,
                run_sequence: 9,
                plan: reported,
            },
        } if reported == plan
    ));
}

#[cfg(feature = "trace-velocity-sweep")]
#[fixture_test]
async fn scheduled_stage_c_plan_event_retains_terminal_authority() {
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();

    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::CommissioningEngine;
    use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
    use crate::commissioning::schedule::{CommissionEvent, CommissionPoll};
    use crate::config::TmcConfig;
    use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig};

    let plan = stage_c_plan();
    let expected_authority = retained_authority();
    let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(CHANNEL);
    runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(
        1,
        crate::state::tmc::tmc_channel0(),
    ));
    let mut stage_b_slot = StageBCandidateSlot::new();
    let mut stage_c_slot = StageCCandidateSlot::new();
    let mut backend = MockBackend::new();
    let config = TmcControlConfig::new(84_000_000, PWM_HZ, 0.03125, TmcConfig::default_for_test());
    let poll = CommissionPoll::ReadyNow {
        event: Some(CommissionEvent::VelocityIntegralPlan {
            oid: 1,
            run_sequence: 9,
            plan,
        }),
    };
    let mut emitted_reply = None;

    let action = runtime
        .drive_commissioning_poll_with_stages(
            poll,
            &mut backend,
            super::CommissionTerminalContext::new(&mut stage_b_slot, &mut stage_c_slot, |reply| {
                emitted_reply = Some(reply)
            }),
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            &config,
        )
        .await
        .expect("active engine should classify the scheduled plan event");

    assert_eq!(stage_c_slot.state(), StageCSlotState::AuthorityOnly);
    assert_eq!(stage_c_slot.authority(), Some(expected_authority));
    assert!(emitted_reply.is_none());
    assert!(matches!(
        action,
        ScheduledCommissionTickAction::ReadyNow {
            event: Some(CommissionEvent::VelocityIntegralPlan {
                oid: 1,
                run_sequence: 9,
                plan: reported,
            }),
        } if reported == plan
    ));
}

#[test]
fn outer_safety_timing_terminal_order_stage_c_resume() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        use crate::autotune::velocity_measurement::{StationarityExclusion, StationarityResult};
        use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};
        use crate::commissioning::CommissioningEngine;
        use crate::commissioning::outer::velocity::current_guard::{
            VelocityCurrentContact, VelocityCurrentPhaseEvidence, VelocityLimitEvidenceCapability,
        };
        use crate::commissioning::outer::{PositiveIRestBoundary, VelocitySettleMetrics};

        let mut stage_b_slot =
            crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot::new();
        let mut stage_c_slot = StageCCandidateSlot::new();
        stage_c_slot.install_authority(retained_authority());
        let preflight = preflight_stage_c_request(request(), &preflight_config(), 9, &stage_c_slot)
            .unwrap()
            .unwrap();
        let mut engine = Some(
            CommissioningEngine::new_outer_for_tmc_with_stage_c_preflight(
                1,
                crate::state::tmc::tmc_channel0(),
                outer_params(),
                entry_snapshot(),
                preflight,
            ),
        );
        engine
            .as_mut()
            .unwrap()
            .set_stage_c_rest_boundary_for_test(PositiveIRestBoundary {
                rung_index: 4,
                slot: 5,
                i_raw: 4,
                direction: -1,
                active_stationarity: StationarityResult::Excluded(
                    StationarityExclusion::StructuredResidual,
                ),
                active_settle: VelocitySettleMetrics::EMPTY,
                zero_phase_current: VelocityCurrentPhaseEvidence {
                    clamp_limit: 2_534,
                    moving_pid_output_peak: 2_000,
                    zero_pid_output_peak: 2_100,
                    moving_status_flags: 0,
                    zero_status_flags: 0,
                    capability: VelocityLimitEvidenceCapability::BestEffortPolling,
                    contact: VelocityCurrentContact::None,
                },
                cleanup_confirmed: true,
            });
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        let (_, _, mut gains) = fresh_state_locals();
        let mut calibrated = true;
        let mut enabled = true;
        let mut state = super::DispatchState::new(
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            &mut calibrated,
            &mut enabled,
            &mut gains,
        );

        let outcome = super::drive_velocity_integral_terminal_with_stages(
            terminal(
                VelocityIntegralOutcome::CompleteCandidate,
                sufficient_evidence(0),
            ),
            &mut engine,
            &mut backend,
            &mut state,
            4_095,
            CHANNEL,
            super::CommissionRetention::new(&mut stage_b_slot, &mut stage_c_slot),
        )
        .await;

        assert!(engine.is_none());
        assert!(!enabled);
        assert!(!calibrated);
        assert!(!backend.enable_pin);
        assert_eq!(stage_c_slot.state(), StageCSlotState::Candidate);
        let CommissionOutcome::Terminal {
            reply: ReplyPayload::VelocityIntegralTerminal { terminal, .. },
            ..
        } = outcome
        else {
            panic!("Stage-C terminal must use the dedicated reply");
        };
        assert_eq!(terminal.rest_boundary_rung_plus_one, 4);
        assert_eq!(terminal.rest_boundary_slot_plus_one, 6);
        assert_eq!(
            backend
                .velocity_sweep_trace_records
                .iter()
                .filter(|record| record.as_bytes()[0] == 5)
                .count(),
            1
        );
        let terminal_record = backend
            .velocity_sweep_trace_records
            .iter()
            .find(|record| record.as_bytes()[0] == 4)
            .unwrap();
        assert_eq!(
            &terminal_record.as_bytes()[15..23],
            terminal.digest.to_le_bytes()
        );
        assert!(
            backend
                .trace_emission_order
                .windows(3)
                .any(|window| window == [30, 144, 4])
        );
    });
}
