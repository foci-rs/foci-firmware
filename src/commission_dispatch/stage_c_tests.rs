use super::stage_c::{
    STAGE_C_CAUSE_EVIDENCE_INTEGRITY, STAGE_C_CAUSE_NO_RETAINED_AUTHORITY,
    STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT, STAGE_C_CAUSE_PLAN_MISMATCH,
    STAGE_C_CAUSE_REPRODUCTION_MISMATCH, StageCPreflight, StageCPreflightConfig,
    StageCPreflightRejection, apply_stage_c_terminal, missing_authority_stage_c_terminal,
    preflight_stage_c_request,
};
use super::{CommissionOutcome, ReplyPayload};
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use crate::commissioning::outer::velocity::integral_evidence::{
    ExactRateInterval, IntegralRunEvidence,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    IntegralAuthoritySummary, IntegralRetentionSlot, IntegralRetentionSlotState,
};
use crate::commissioning::outer::velocity::reproduction::BUILD_FEATURES;
use crate::commissioning::outer::velocity::sweep_plan::{
    RequestedVelocitySource, VelocitySweepRequest, plan_velocity_sweep,
};
use crate::commissioning::types::{StageCCause, VelocityIntegralOutcome, VelocityIntegralTerminal};
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

fn stage_b_plan() -> crate::commissioning::outer::velocity::sweep_plan::VelocitySweepPlan {
    plan_velocity_sweep(
        request(),
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .expect("valid Stage-B plan")
}

fn breakaway_stage_c_plan()
-> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
    use crate::commission_dispatch::test_support::breakaway_confirmation_for_test;
    use crate::commissioning::outer::velocity::integral_plan::{
        AcquisitionContext, plan_breakaway_velocity_integral_response,
    };

    let confirmation = breakaway_confirmation_for_test();

    let stage_b = stage_b_plan();
    let mut context = AcquisitionContext::from(stage_b.execution_plan(stage_b.digest()));
    context.plan_time_pwm_hz = PWM_HZ;
    context.plan_time_encoder_counts_per_rev = COUNTS_PER_REV;

    let request = request();
    plan_breakaway_velocity_integral_response(
        confirmation.token,
        confirmation.confirmation_plan,
        context,
        confirmation.held_out_response_intervals_q,
        confirmation.confidence_q,
        confirmation.band_position_q,
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
) -> IntegralAuthoritySummary {
    IntegralAuthoritySummary::from_plan(
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
    authority: &IntegralAuthoritySummary,
    outcome: VelocityIntegralOutcome,
    evidence: IntegralRunEvidence,
) -> VelocityIntegralTerminal {
    VelocityIntegralTerminal {
        run_sequence: 9,
        evidence_sequence: 57,
        plan_digest: plan.digest(),
        digest: 0x1234_5678_9abc_def0,
        outcome,
        cause: StageCCause::Engine(0),
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
        rest_rejection_after_sufficiency: false,
        rest_rejection_owner: None,
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
    // The token is carried so the amplitude preflight can reconstruct the plan.
    assert_eq!(installed.token, Some(authority.acquisition));
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

fn retained_authority() -> IntegralAuthoritySummary {
    breakaway_authority_summary(&breakaway_stage_c_plan())
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
    let plan = breakaway_stage_c_plan();
    VelocityIntegralTerminal {
        run_sequence: 9,
        evidence_sequence: 57,
        plan_digest: plan.digest(),
        digest: 0x1234_5678_9abc_def0,
        outcome,
        cause: StageCCause::Engine(0),
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
        rest_rejection_after_sufficiency: false,
        rest_rejection_owner: None,
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
    let slot = IntegralRetentionSlot::new();
    assert!(
        preflight_stage_c_request(request(), &preflight_config(), 4, &slot)
            .expect("an empty slot is not a rejection")
            .is_none()
    );

    let terminal = missing_authority_stage_c_terminal(4);

    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Failed);
    assert_eq!(
        terminal.cause,
        StageCCause::Dispatch(STAGE_C_CAUSE_NO_RETAINED_AUTHORITY)
    );
    assert_ne!(
        terminal.cause,
        StageCCause::Dispatch(STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT)
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
    let mut slot = IntegralRetentionSlot::new();
    slot.install_authority(retained_authority());

    let preflight = preflight_stage_c_request(request(), &preflight_config(), 7, &slot)
        .expect("matching retained plan")
        .expect("nonempty Stage-C slot");

    assert_eq!(preflight.plan, breakaway_stage_c_plan());
    assert_eq!(preflight.identity, retained_authority().identity);
    assert_eq!(preflight.run_sequence, 7);
    assert_eq!(slot.state(), IntegralRetentionSlotState::AuthorityOnly);
}

#[test]
fn changed_resume_plan_is_rejected_without_mutating_retention() {
    let mut slot = IntegralRetentionSlot::new();
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
    let mut slot = IntegralRetentionSlot::new();
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
    assert_eq!(slot.state(), IntegralRetentionSlotState::Candidate);
    assert_eq!(
        slot.candidate().unwrap().rest_boundary.unwrap(),
        crate::commissioning::outer::velocity::integral_reproduction::IntegralRestBoundaryReference {
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
    second_terminal.cause = StageCCause::Engine(10);
    let second = apply_stage_c_terminal(second_terminal, &mut slot);
    assert_eq!(second.terminal.outcome, VelocityIntegralOutcome::Complete);
    assert_eq!(second.terminal.cause, StageCCause::Engine(10));
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
    assert_eq!(slot.state(), IntegralRetentionSlotState::Empty);
}

/// A reproduced confirmation run that does not itself re-cross the
/// target-reaching threshold must still install, using the terminus already
/// proven by the first sufficient run: a breakaway target terminus is not
/// part of reproduction authority, so a `Complete` promotion cannot require
/// the second run to redetect it.
#[test]
fn reproduced_complete_without_current_terminus_still_installs() {
    let plan = breakaway_stage_c_plan();
    let authority = breakaway_authority_summary(&plan);
    let execution = plan.execution_plan();

    let mut slot = IntegralRetentionSlot::new();
    slot.install_authority(authority);

    let mut first_terminal = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(0),
    );
    first_terminal.rest_boundary_rung_plus_one = 3;
    apply_stage_c_terminal(first_terminal, &mut slot);

    let mut second_terminal = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(2),
    );
    second_terminal.rest_boundary_rung_plus_one = 0;
    second_terminal.rest_boundary_slot_plus_one = 0;
    let dispatch = apply_stage_c_terminal(second_terminal, &mut slot);
    assert_eq!(dispatch.terminal.outcome, VelocityIntegralOutcome::Complete);

    let installed = match super::stage_c::breakaway_acceptance_point(
        CHANNEL,
        &authority,
        execution,
        &dispatch.terminal,
    )
    .expect("no fault")
    {
        super::stage_c::AcceptanceInstall::Installed(point) => point,
        other => panic!("expected install, got {other:?}"),
    };
    assert_eq!(installed.selected_p, authority.final_p);
}

#[test]
fn sufficient_mismatch_replaces_candidate_and_keeps_authority() {
    let mut slot = IntegralRetentionSlot::new();
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
    assert_eq!(
        dispatch.terminal.cause,
        StageCCause::Dispatch(STAGE_C_CAUSE_REPRODUCTION_MISMATCH)
    );
    assert!(dispatch.reproduction.is_some());
    assert_eq!(slot.state(), IntegralRetentionSlotState::Candidate);
}

#[test]
fn inconclusive_preserves_authority_while_fault_clears_it() {
    let mut slot = IntegralRetentionSlot::new();
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
    assert_eq!(slot.state(), IntegralRetentionSlotState::AuthorityOnly);

    let fault = apply_stage_c_terminal(
        terminal(VelocityIntegralOutcome::Fault, empty_evidence()),
        &mut slot,
    );
    assert_eq!(fault.terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(slot.state(), IntegralRetentionSlotState::Empty);
}

#[test]
fn unexpected_complete_terminal_fails_closed_in_retained_stage_c_dispatch() {
    // Combined workflow terminals are handled before this retained dispatcher.
    // Non-combined retained Stage C reports sufficiency as `CompleteCandidate`.
    // A bare `Complete` reaching this function is therefore a contract
    // violation, so it must not be accepted as success.
    let mut slot = IntegralRetentionSlot::new();
    slot.install_authority(retained_authority());

    let dispatch = apply_stage_c_terminal(
        terminal(VelocityIntegralOutcome::Complete, sufficient_evidence(0)),
        &mut slot,
    );

    assert_eq!(dispatch.terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        dispatch.terminal.cause,
        StageCCause::Dispatch(STAGE_C_CAUSE_EVIDENCE_INTEGRITY)
    );
    assert_eq!(slot.state(), IntegralRetentionSlotState::Empty);
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
            StageCCause::Dispatch(STAGE_C_CAUSE_EVIDENCE_INTEGRITY),
        ),
        (
            VelocityIntegralOutcome::CompleteCandidate,
            VelocityIntegralOutcome::CompleteCandidate,
            StageCCause::Engine(0),
        ),
        (
            VelocityIntegralOutcome::Inconclusive,
            VelocityIntegralOutcome::Inconclusive,
            StageCCause::Engine(0),
        ),
        (
            VelocityIntegralOutcome::Fault,
            VelocityIntegralOutcome::Fault,
            StageCCause::Engine(0),
        ),
        (
            VelocityIntegralOutcome::Failed,
            VelocityIntegralOutcome::Failed,
            StageCCause::Engine(0),
        ),
        (
            VelocityIntegralOutcome::RejectedPlanMismatch,
            VelocityIntegralOutcome::Fault,
            StageCCause::Dispatch(STAGE_C_CAUSE_EVIDENCE_INTEGRITY),
        ),
    ];

    for (engine_outcome, expected_outcome, expected_cause) in expectations {
        let mut slot = IntegralRetentionSlot::new();
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
    let mut slot = IntegralRetentionSlot::new();
    slot.install_authority(retained_authority());
    apply_stage_c_terminal(
        terminal(
            VelocityIntegralOutcome::CompleteCandidate,
            sufficient_evidence(0),
        ),
        &mut slot,
    );

    let mut rest_terminal = terminal(VelocityIntegralOutcome::Inconclusive, empty_evidence());
    rest_terminal.cause = StageCCause::Error(
        crate::autotune::CommissionError::VelocityRestNotConfirmed.status_code(),
    );
    let dispatch = apply_stage_c_terminal(rest_terminal, &mut slot);

    assert_eq!(
        dispatch.terminal.outcome,
        VelocityIntegralOutcome::Inconclusive
    );
    assert_eq!(
        dispatch.terminal.cause,
        StageCCause::Error(
            crate::autotune::CommissionError::VelocityRestNotConfirmed.status_code()
        )
    );
    assert_eq!(slot.state(), IntegralRetentionSlotState::AuthorityOnly);
}

#[test]
fn same_channel_calibration_preserves_stage_c_and_other_channel_clears_it() {
    let mut slot = IntegralRetentionSlot::new();
    slot.install_authority(retained_authority());

    slot.preserve_for_same_channel_calibration(CHANNEL);
    assert_eq!(slot.state(), IntegralRetentionSlotState::AuthorityOnly);
    slot.preserve_for_same_channel_calibration(CHANNEL + 1);
    assert_eq!(slot.state(), IntegralRetentionSlotState::Empty);
}

fn outer_params() -> crate::commissioning::OuterAutotuneInputs {
    crate::commissioning::OuterAutotuneInputs {
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

fn entry_snapshot() -> crate::commissioning::OuterGainRollbackSnapshot {
    crate::commissioning::OuterGainRollbackSnapshot {
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

#[test]
fn stage_c_terminal_causes_reserve_recovery_and_lifecycle_values() {
    assert_eq!(super::stage_c::STAGE_C_CAUSE_EVIDENCE_INTEGRITY, 4);
    assert_eq!(super::stage_c::STAGE_C_CAUSE_REPRODUCTION_MISMATCH, 6);
    assert_eq!(super::stage_c::STAGE_C_CAUSE_PLAN_MISMATCH, 7);
}

#[fixture_test]
async fn breakaway_stage_c_plan_ready_retains_terminal_authority() {
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();

    use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};
    use crate::commissioning::types::StepResult;

    let plan = breakaway_stage_c_plan();
    let expected_authority = IntegralAuthoritySummary::from_plan(
        CHANNEL,
        BUILD_FEATURES,
        plan.schema_revision,
        plan.authority.acquisition,
        plan.execution_plan(),
        plan.authority,
    );
    let mut stage_c_slot = IntegralRetentionSlot::new();
    let mut backend = MockBackend::new();
    let (_, _, mut gains) = fresh_state_locals();
    let mut calibrated = true;
    let mut enabled = true;
    let mut state = super::DispatchState::new(
        0,
        crate::state::tmc::tmc_channel0(),
        crate::state::stepper::stepper_channel0(),
        &mut calibrated,
        &mut enabled,
        &mut gains,
    );
    let mut engine = Some(breakaway_engine(plan, &expected_authority));

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
        super::CommissionRetention::new(&mut stage_c_slot),
    )
    .await;

    assert_eq!(
        stage_c_slot.state(),
        IntegralRetentionSlotState::AuthorityOnly
    );
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

#[fixture_test]
async fn scheduled_stage_c_plan_event_retains_terminal_authority() {
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();

    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::CommissioningEngine;
    use crate::commissioning::schedule::{CommissionEvent, CommissionPoll};
    use crate::config::TmcConfig;
    use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig};

    let plan = breakaway_stage_c_plan();
    let expected_authority = retained_authority();
    let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(CHANNEL);
    runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(
        1,
        crate::state::tmc::tmc_channel0(),
    ));
    let mut stage_c_slot = IntegralRetentionSlot::new();
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
            super::CommissionTerminalContext::new(&mut stage_c_slot, |reply| {
                emitted_reply = Some(reply)
            }),
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            &config,
        )
        .await
        .expect("active engine should classify the scheduled plan event");

    assert_eq!(
        stage_c_slot.state(),
        IntegralRetentionSlotState::AuthorityOnly
    );
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
        use crate::commissioning::outer::velocity::velocity_clamp::{
            VelocityCurrentContact, VelocityCurrentPhaseEvidence, VelocityLimitEvidenceCapability,
        };
        use crate::commissioning::outer::{PositiveIRestBoundary, VelocitySettleMetrics};

        let mut stage_c_slot = IntegralRetentionSlot::new();
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
            0,
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
            super::CommissionRetention::new(&mut stage_c_slot),
        )
        .await;

        assert!(engine.is_none());
        assert!(!enabled);
        assert!(!calibrated);
        assert!(!backend.enable_pin);
        assert_eq!(stage_c_slot.state(), IntegralRetentionSlotState::Candidate);
        let CommissionOutcome::Terminal {
            reply: ReplyPayload::VelocityIntegralTerminal { terminal, .. },
            ..
        } = outcome
        else {
            panic!("Stage-C terminal must use the dedicated reply");
        };
        assert_eq!(terminal.rest_boundary_rung_plus_one, 4);
        assert_eq!(terminal.rest_boundary_slot_plus_one, 6);
        // Raw FTRC canonical-record content and emission order are a `trace`-only
        // diagnostic concern; the dispatch/terminal assertions above already cover
        // this path in the default build.
        #[cfg(feature = "trace")]
        {
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
        }
    });
}

#[test]
fn candidate_terminal_restores_entry_snapshot_to_registers() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        use crate::autotune::velocity_measurement::{StationarityExclusion, StationarityResult};
        use crate::commission_dispatch::test_support::{MockBackend, SpiOp, fresh_state_locals};
        use crate::commissioning::CommissioningEngine;
        use crate::commissioning::outer::velocity::velocity_clamp::{
            VelocityCurrentContact, VelocityCurrentPhaseEvidence, VelocityLimitEvidenceCapability,
        };
        use crate::commissioning::outer::{PositiveIRestBoundary, VelocitySettleMetrics};
        use foci_tmc4671::registers::{
            PID_POSITION_P_POSITION_I, PID_VELOCITY_LIMIT, PID_VELOCITY_P_VELOCITY_I,
            PIDOUT_UQ_UD_LIMITS,
        };

        let mut stage_c_slot = IntegralRetentionSlot::new();
        stage_c_slot.install_authority(retained_authority());
        let preflight = preflight_stage_c_request(request(), &preflight_config(), 9, &stage_c_slot)
            .unwrap()
            .unwrap();
        let baseline = entry_snapshot();
        let mut engine = Some(
            CommissioningEngine::new_outer_for_tmc_with_stage_c_preflight(
                1,
                crate::state::tmc::tmc_channel0(),
                outer_params(),
                baseline,
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

        // Live "candidate" registers diverge from the baseline snapshot -- the
        // restore must overwrite them, not merely no-op because a value is
        // already present.
        let _ = backend
            .write_register(PID_VELOCITY_P_VELOCITY_I, 0xdead_beef)
            .await;
        let _ = backend
            .write_register(PID_POSITION_P_POSITION_I, 0xdead_beef)
            .await;
        let _ = backend.write_register(PID_VELOCITY_LIMIT, 999_999).await;
        let _ = backend.write_register(PIDOUT_UQ_UD_LIMITS, 999).await;
        backend.spi_log.clear();

        let (_, _, mut gains) = fresh_state_locals();
        let mut calibrated = true;
        let mut enabled = true;
        let mut state = super::DispatchState::new(
            0,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            &mut calibrated,
            &mut enabled,
            &mut gains,
        );

        let _outcome = super::drive_velocity_integral_terminal_with_stages(
            terminal(
                VelocityIntegralOutcome::CompleteCandidate,
                sufficient_evidence(0),
            ),
            &mut engine,
            &mut backend,
            &mut state,
            4_095,
            CHANNEL,
            super::CommissionRetention::new(&mut stage_c_slot),
        )
        .await;

        let (velocity_reg, position_reg) = crate::pid::OuterLoopGains {
            position_p: baseline.position_p,
            position_i: baseline.position_i,
            velocity_p: baseline.velocity_p,
            velocity_i: baseline.velocity_i,
        }
        .to_registers();

        assert!(backend.spi_log.iter().any(|op| matches!(
            op,
            SpiOp::Write(addr, val)
                if *addr == PID_VELOCITY_P_VELOCITY_I.addr() && *val == velocity_reg
        )));
        assert!(backend.spi_log.iter().any(|op| matches!(
            op,
            SpiOp::Write(addr, val)
                if *addr == PID_POSITION_P_POSITION_I.addr() && *val == position_reg
        )));
        assert!(backend.spi_log.iter().any(|op| matches!(
            op,
            SpiOp::Write(addr, val)
                if *addr == PID_VELOCITY_LIMIT.addr() && *val == baseline.velocity_limit
        )));
        assert!(backend.spi_log.iter().any(|op| matches!(
            op,
            SpiOp::Write(addr, val)
                if *addr == PIDOUT_UQ_UD_LIMITS.addr() && *val == baseline.voltage_limit
        )));

        let restore_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(op, SpiOp::Write(addr, val)
                    if *addr == PID_VELOCITY_P_VELOCITY_I.addr() && *val == velocity_reg)
            })
            .expect("restore write present");
        let disarm_index = backend
            .spi_log
            .iter()
            .position(|op| matches!(op, SpiOp::EnablePin(false)))
            .expect("disarm present");
        assert!(
            restore_index < disarm_index,
            "restore must run before disarm"
        );
    });
}

/// Build the breakaway Stage-C engine parked in `OuterPhase::VelocityIntegral`
/// whose live execution plan the dispatch seam reads for the promotion.
fn breakaway_engine(
    plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
    authority: &IntegralAuthoritySummary,
) -> crate::commissioning::CommissioningEngine {
    crate::commissioning::CommissioningEngine::new_outer_for_tmc_with_stage_c_preflight(
        1,
        crate::state::tmc::tmc_channel0(),
        outer_params(),
        entry_snapshot(),
        StageCPreflight {
            plan,
            identity: authority.identity,
            run_sequence: 9,
        },
    )
}

#[fixture_test]
async fn promoted_complete_disarms_restores_baseline_and_installs_acceptance_point() {
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, fresh_state_locals};
    use foci_tmc4671::registers::{
        PID_POSITION_P_POSITION_I, PID_VELOCITY_LIMIT, PID_VELOCITY_P_VELOCITY_I,
        PIDOUT_UQ_UD_LIMITS,
    };

    crate::state::tmc::tmc_channel0()
        .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
        .expect("valid motion scale");

    let plan = breakaway_stage_c_plan();
    let authority = breakaway_authority_summary(&plan);

    // First sufficient run retains the candidate the reproduced run confirms.
    let mut stage_c_slot = IntegralRetentionSlot::new();
    stage_c_slot.install_authority(authority);
    let mut first = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(0),
    );
    first.rest_boundary_rung_plus_one = 3;
    apply_stage_c_terminal(first, &mut stage_c_slot);
    assert_eq!(stage_c_slot.state(), IntegralRetentionSlotState::Candidate);

    let baseline = entry_snapshot();
    let mut engine = Some(breakaway_engine(plan, &authority));
    let mut backend = MockBackend::new();
    backend.enable_pin = true;

    // Live "candidate" registers diverge from the baseline snapshot -- the
    // restore must overwrite them, not merely no-op because a value is
    // already present.
    let _ = backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, 0xdead_beef)
        .await;
    let _ = backend
        .write_register(PID_POSITION_P_POSITION_I, 0xdead_beef)
        .await;
    let _ = backend.write_register(PID_VELOCITY_LIMIT, 999_999).await;
    let _ = backend.write_register(PIDOUT_UQ_UD_LIMITS, 999).await;
    backend.spi_log.clear();

    let (_, _, mut gains) = fresh_state_locals();
    let mut calibrated = true;
    let mut enabled = true;
    let mut state = super::DispatchState::new(
        0,
        crate::state::tmc::tmc_channel0(),
        crate::state::stepper::stepper_channel0(),
        &mut calibrated,
        &mut enabled,
        &mut gains,
    );

    // Reproduced second run: same evidence, matching terminus.
    let mut second = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(0),
    );
    second.rest_boundary_rung_plus_one = 3;

    let outcome = super::drive_velocity_integral_terminal_with_stages(
        second,
        &mut engine,
        &mut backend,
        &mut state,
        4_095,
        CHANNEL,
        super::CommissionRetention::new(&mut stage_c_slot),
    )
    .await;

    let CommissionOutcome::Terminal {
        reply: ReplyPayload::VelocityIntegralTerminal { oid, terminal, .. },
        ..
    } = outcome
    else {
        panic!("a promoted Complete must emit a normal velocity-integral terminal");
    };
    assert_eq!(oid, 1);
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Complete);

    // The acceptance point install (unchanged from before F3) still happens.
    assert_eq!(
        stage_c_slot.state(),
        IntegralRetentionSlotState::AcceptancePoint
    );

    // The engine is consumed and the motor fully disarmed -- no inline
    // continuation into robustness.
    assert!(engine.is_none());
    assert!(!enabled);
    assert!(!calibrated);
    assert!(!backend.enable_pin);

    // The operator's Stage 2 entry baseline is restored to the registers
    // before the disarm safe-state overwrites the torque-limit register.
    let (velocity_reg, position_reg) = crate::pid::OuterLoopGains {
        position_p: baseline.position_p,
        position_i: baseline.position_i,
        velocity_p: baseline.velocity_p,
        velocity_i: baseline.velocity_i,
    }
    .to_registers();
    assert!(backend.spi_log.iter().any(|op| matches!(
        op,
        SpiOp::Write(addr, val)
            if *addr == PID_VELOCITY_P_VELOCITY_I.addr() && *val == velocity_reg
    )));
    assert!(backend.spi_log.iter().any(|op| matches!(
        op,
        SpiOp::Write(addr, val)
            if *addr == PID_POSITION_P_POSITION_I.addr() && *val == position_reg
    )));
    assert!(backend.spi_log.iter().any(|op| matches!(
        op,
        SpiOp::Write(addr, val)
            if *addr == PID_VELOCITY_LIMIT.addr() && *val == baseline.velocity_limit
    )));
    assert!(backend.spi_log.iter().any(|op| matches!(
        op,
        SpiOp::Write(addr, val)
            if *addr == PIDOUT_UQ_UD_LIMITS.addr() && *val == baseline.voltage_limit
    )));

    let restore_index = backend
        .spi_log
        .iter()
        .position(|op| {
            matches!(op, SpiOp::Write(addr, val)
                if *addr == PID_VELOCITY_P_VELOCITY_I.addr() && *val == velocity_reg)
        })
        .expect("restore write present");
    let disarm_index = backend
        .spi_log
        .iter()
        .position(|op| matches!(op, SpiOp::EnablePin(false)))
        .expect("disarm present");
    assert!(
        restore_index < disarm_index,
        "restore must run before disarm"
    );
}

#[fixture_test]
async fn promoted_complete_from_terminal_poll_emits_terminal_reply() {
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::schedule::CommissionTerminal;
    use crate::config::TmcConfig;
    use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig, TmcControlRuntime};

    crate::state::tmc::tmc_channel0()
        .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
        .expect("valid motion scale");

    let plan = breakaway_stage_c_plan();
    let authority = breakaway_authority_summary(&plan);

    // First sufficient run retains the candidate the reproduced run confirms.
    let mut stage_c_slot = IntegralRetentionSlot::new();
    stage_c_slot.install_authority(authority);
    let mut first = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(0),
    );
    first.rest_boundary_rung_plus_one = 3;
    apply_stage_c_terminal(first, &mut stage_c_slot);
    assert_eq!(stage_c_slot.state(), IntegralRetentionSlotState::Candidate);

    let mut runtime = TmcControlRuntime::new_for_channel(CHANNEL);
    runtime.set_commission_engine(breakaway_engine(plan, &authority));
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    let mut backend = MockBackend::new();
    backend.enable_pin = true;
    let config = TmcControlConfig::new(84_000_000, PWM_HZ, 0.03125, TmcConfig::default_for_test());

    let mut second = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(0),
    );
    second.rest_boundary_rung_plus_one = 3;

    let mut emitted_reply = None;
    let action = runtime
        .drive_commissioning_terminal_with_stages(
            CommissionTerminal::VelocityIntegral(second),
            &mut backend,
            super::CommissionTerminalContext::new(&mut stage_c_slot, |reply| {
                emitted_reply = Some(reply)
            }),
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            &config,
        )
        .await
        .expect("active engine should drive the promoted terminal");

    // A promoted Complete from the terminal poll ends the workflow like any
    // other terminal outcome, disarming instead of continuing into
    // robustness.
    let ScheduledCommissionTickAction::Terminal { reply, .. } = action else {
        panic!("a promoted Complete from the terminal poll must emit a Terminal action");
    };
    let ReplyPayload::VelocityIntegralTerminal { oid, terminal, .. } = reply else {
        panic!("a promoted Complete from the terminal poll must report its terminal");
    };
    assert_eq!(oid, 1);
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Complete);
    // The Terminal action already carries the reply; the terminal poll must
    // not also forward it through the report-only evidence channel.
    assert!(emitted_reply.is_none());
}

#[fixture_test]
async fn complete_candidate_still_emits_report_only_terminal() {
    use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};

    crate::state::tmc::tmc_channel0()
        .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
        .expect("valid motion scale");

    let plan = breakaway_stage_c_plan();
    let authority = breakaway_authority_summary(&plan);

    // Authority only: the first sufficient run has nothing to reproduce, so it
    // stays a `CompleteCandidate` the host resumes from.
    let mut stage_c_slot = IntegralRetentionSlot::new();
    stage_c_slot.install_authority(authority);

    let mut engine = Some(breakaway_engine(plan, &authority));
    let mut backend = MockBackend::new();
    backend.enable_pin = true;
    let (_, _, mut gains) = fresh_state_locals();
    let mut calibrated = true;
    let mut enabled = true;
    let mut state = super::DispatchState::new(
        0,
        crate::state::tmc::tmc_channel0(),
        crate::state::stepper::stepper_channel0(),
        &mut calibrated,
        &mut enabled,
        &mut gains,
    );

    let mut first = breakaway_terminal(
        &plan,
        &authority,
        VelocityIntegralOutcome::CompleteCandidate,
        sufficient_evidence(0),
    );
    first.rest_boundary_rung_plus_one = 3;

    let outcome = super::drive_velocity_integral_terminal_with_stages(
        first,
        &mut engine,
        &mut backend,
        &mut state,
        4_095,
        CHANNEL,
        super::CommissionRetention::new(&mut stage_c_slot),
    )
    .await;

    assert!(
        engine.is_none(),
        "a first CompleteCandidate consumes the engine and ends the run"
    );
    let CommissionOutcome::Terminal { reply, .. } = outcome else {
        panic!("CompleteCandidate must emit a report-only terminal");
    };
    assert!(matches!(
        reply,
        ReplyPayload::VelocityIntegralTerminal { .. }
    ));
    // The report-only path disarms exactly as before.
    assert!(!enabled);
    assert!(!backend.enable_pin);
}
