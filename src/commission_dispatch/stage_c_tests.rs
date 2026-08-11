use super::stage_c::{
    STAGE_C_CAUSE_EVIDENCE_INTEGRITY, STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT,
    STAGE_C_CAUSE_PLAN_MISMATCH, STAGE_C_CAUSE_REPRODUCTION_MISMATCH, StageCPreflightConfig,
    StageCPreflightRejection, apply_stage_c_terminal, preflight_stage_c_request,
};
use super::{CommissionOutcome, ReplyPayload};
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use crate::commissioning::outer::velocity::integral_evidence::{
    ExactRateInterval, IntegralRunEvidence,
};
use crate::commissioning::outer::velocity::integral_plan::{
    STAGE_C_EVIDENCE_SCHEMA_REVISION, StageCAuthority, plan_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, StageCAuthoritySummary, StageCCandidateSlot, StageCSlotState,
};
use crate::commissioning::outer::velocity::reproduction::{
    STAGE_B_BUILD_REVISION, StageBDirectionalValidity,
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

fn selected_combined_response()
-> crate::commissioning::outer::velocity::combined_plan::CombinedSelectedResponse {
    use crate::commissioning::outer::velocity::combined_plan::select_proportional_response;
    use crate::commissioning::outer::velocity::operating_region::{
        BoundRounding, CompactSignedRate,
    };
    use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
    use crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport;
    use foci_tmc4671::pid::PidPGain;

    let report = |low_q, high_q| CompactDirectionalRungReport {
        p: PidPGain::new(1_024).unwrap(),
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0; 2]; 2],
        pooled: None,
        settled_rate_bounds_q: [
            CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
            CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
        ],
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
        operable_count: 4,
        flags: 0,
    };
    select_proportional_response(
        report(8_300_000, 8_900_000),
        report(-8_900_000, -8_300_000),
        175,
        COUNTS_PER_REV,
    )
    .unwrap()
    .unwrap()
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

#[fixture_test]
async fn combined_target_reached_installs_an_acceptance_point_after_restoration() {
    use crate::commission_dispatch::stage_b::preflight_combined_stage_b_request;
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::CommissioningEngine;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        StageCCandidateSlot, StageCSlotState,
    };
    use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
    use crate::commissioning::outer::velocity::sweep_plan::plan_combined_velocity_sweep;
    use crate::commissioning::types::VelocitySweepOutcome;
    use crate::config::TmcConfig;
    use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig};

    let stage_b_plan = plan_combined_velocity_sweep(
        request(),
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let preflight = preflight_combined_stage_b_request(
        request(),
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        CHANNEL,
        8,
        false,
    )
    .unwrap();
    crate::state::tmc::tmc_channel0()
        .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
        .expect("valid combined test motion scale");
    let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(CHANNEL);
    runtime.set_motor_enabled_for_test(true);
    let mut engine = CommissioningEngine::new_outer_for_tmc_with_stage_b_preflight(
        1,
        crate::state::tmc::tmc_channel0(),
        outer_params(),
        entry_snapshot(),
        preflight,
    );
    engine.set_combined_stage_b_response_for_test(selected_combined_response());
    runtime.set_commission_engine(engine);
    let mut stage_b_slot = StageBCandidateSlot::new();
    let mut stage_c_slot = StageCCandidateSlot::new();
    let mut stage_b_terminal = stage_b_terminal();
    stage_b_terminal.run_sequence = 8;
    stage_b_terminal.plan_digest = stage_b_plan.digest();
    stage_b_terminal.outcome = VelocitySweepOutcome::Complete;
    stage_b_terminal.nominated_p = 1_024;
    let mut backend = MockBackend::new();
    backend.enable_pin = true;
    let config = TmcControlConfig::new(84_000_000, PWM_HZ, 0.03125, TmcConfig::default_for_test());
    let mut emitted_reply = None;

    let action = runtime
        .drive_commissioning_terminal_with_stages(
            crate::commissioning::schedule::CommissionTerminal::VelocitySweep(stage_b_terminal),
            &mut backend,
            super::CommissionTerminalContext::new(&mut stage_b_slot, &mut stage_c_slot, |reply| {
                emitted_reply = Some(reply)
            }),
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            &config,
        )
        .await
        .expect("combined continuation must produce a scheduler action");

    assert!(matches!(
        action,
        ScheduledCommissionTickAction::WorkflowContinuation
    ));
    let Some(ReplyPayload::VelocityIntegralContinuation {
        stage_b_reproduction,
        stage_c_plan,
        ..
    }) = emitted_reply
    else {
        panic!("combined transition must reuse the integral continuation reply");
    };
    assert_eq!(stage_b_reproduction, None);
    assert_eq!(
        stage_c_plan.schema_revision,
        crate::commissioning::outer::velocity::combined_plan::COMBINED_STAGE_C_SCHEMA_REVISION
    );
    assert_eq!(stage_c_plan.final_p, 1_024);
    assert_eq!(
        stage_c_plan.combined_selected_response,
        Some(selected_combined_response())
    );
    assert_eq!(stage_b_slot.candidate(), None);
    assert_eq!(stage_c_slot.state(), StageCSlotState::Empty);
    assert!(runtime.is_commission_active());
    assert!(runtime.is_motor_enabled());
    assert!(backend.enable_pin);

    emitted_reply = None;
    let mut stage_c_terminal = terminal(VelocityIntegralOutcome::Complete, sufficient_evidence(0));
    stage_c_terminal.run_sequence = 8;
    stage_c_terminal.plan_digest = stage_c_plan.digest();
    stage_c_terminal.final_p = 1_024;
    stage_c_terminal.rest_boundary_rung_plus_one = 12;
    let action = runtime
        .drive_commissioning_terminal_with_stages(
            crate::commissioning::schedule::CommissionTerminal::VelocityIntegral(stage_c_terminal),
            &mut backend,
            super::CommissionTerminalContext::new(&mut stage_b_slot, &mut stage_c_slot, |reply| {
                emitted_reply = Some(reply)
            }),
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            &config,
        )
        .await
        .expect("combined Stage-C terminal must produce a scheduler action");
    let ScheduledCommissionTickAction::Terminal {
        reply:
            ReplyPayload::VelocityIntegralTerminal {
                reproduction,
                authority,
                ..
            },
        ..
    } = action
    else {
        panic!("combined Stage C must use the existing terminal reply")
    };
    assert_eq!(reproduction, None);
    assert_eq!(authority, None);
    assert!(emitted_reply.is_none());
    assert_eq!(stage_b_slot.candidate(), None);
    assert_eq!(stage_c_slot.state(), StageCSlotState::AcceptancePoint);
    let point = stage_c_slot
        .acceptance_point()
        .expect("TargetReached must retain the exact operating point");
    assert_eq!(point.identity.channel, CHANNEL);
    assert_eq!(
        point.identity.schema_revision,
        crate::commissioning::outer::velocity::combined_plan::COMBINED_STAGE_C_SCHEMA_REVISION
    );
    assert_eq!(point.identity.plan_digest, stage_c_plan.digest());
    assert_eq!(point.selected_p, 1_024);
    assert_eq!(point.selected_i, 1_024);
    assert_eq!(point.target_velocity_rpm, 175);
    assert_eq!(point.progress, AcceptanceMatrixProgress::ReadyAscending);
    assert!(!runtime.is_commission_active());
}

#[test]
fn combined_selected_rest_rejection_retains_stage_c_authority_without_continuing() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        use crate::commission_dispatch::stage_b::preflight_combined_stage_b_request;
        use crate::commission_dispatch::test_support::MockBackend;
        use crate::commissioning::CommissioningEngine;
        use crate::commissioning::outer::velocity::integral_reproduction::{
            StageCCandidateSlot, StageCSlotState,
        };
        use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
        use crate::commissioning::outer::velocity::sweep_plan::plan_combined_velocity_sweep;
        use crate::commissioning::types::VelocitySweepOutcome;
        use crate::config::TmcConfig;
        use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig};

        let stage_b_plan = plan_combined_velocity_sweep(
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        )
        .unwrap();
        let preflight = preflight_combined_stage_b_request(
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            CHANNEL,
            8,
            false,
        )
        .unwrap();
        crate::state::tmc::tmc_channel0()
            .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
            .expect("valid combined test motion scale");
        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(CHANNEL);
        runtime.set_motor_enabled_for_test(true);
        let mut engine = CommissioningEngine::new_outer_for_tmc_with_stage_b_preflight(
            1,
            crate::state::tmc::tmc_channel0(),
            outer_params(),
            entry_snapshot(),
            preflight,
        );
        engine.set_combined_stage_b_response_for_test(selected_combined_response());
        runtime.set_commission_engine(engine);
        let mut stage_b_slot = StageBCandidateSlot::new();
        let mut stage_c_slot = StageCCandidateSlot::new();
        let mut terminal = stage_b_terminal();
        terminal.run_sequence = 8;
        terminal.plan_digest = stage_b_plan.digest();
        terminal.outcome = VelocitySweepOutcome::Inconclusive;
        terminal.cause = crate::autotune::CommissionError::VelocityRestNotConfirmed.status_code();
        terminal.nominated_p = 1_024;
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        let config =
            TmcControlConfig::new(84_000_000, PWM_HZ, 0.03125, TmcConfig::default_for_test());

        let action = runtime
            .drive_commissioning_terminal_with_stages(
                crate::commissioning::schedule::CommissionTerminal::VelocitySweep(terminal),
                &mut backend,
                super::CommissionTerminalContext::new(&mut stage_b_slot, &mut stage_c_slot, |_| {
                    panic!("rest rejection must suppress continuation evidence")
                }),
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
                &config,
            )
            .await
            .expect("rest rejection must produce a terminal action");

        assert_eq!(stage_c_slot.state(), StageCSlotState::AuthorityOnly);
        let retained = stage_c_slot
            .authority()
            .expect("selected Stage B must retain the exact Stage-C plan");
        assert_eq!(retained.final_p, 1_024);
        assert_eq!(retained.stage_b_plan_digest, stage_b_plan.digest());
        assert_eq!(
            retained.identity.schema_revision,
            crate::commissioning::outer::velocity::combined_plan::COMBINED_STAGE_C_SCHEMA_REVISION
        );
        let resume = preflight_stage_c_request(request(), &preflight_config(), 9, &stage_c_slot)
            .expect("retained combined authority must admit the matching Stage-C request")
            .expect("retained combined authority must resume directly in Stage C");
        assert_eq!(
            resume.plan.schema_revision,
            crate::commissioning::outer::velocity::combined_plan::COMBINED_STAGE_C_SCHEMA_REVISION
        );
        assert!(!runtime.is_commission_active());
        assert!(!runtime.is_motor_enabled());
        assert!(!backend.enable_pin);
        assert!(matches!(
            action,
            ScheduledCommissionTickAction::Terminal {
                reply: ReplyPayload::VelocitySweepTerminal { terminal, .. },
                ..
            } if terminal.outcome == VelocitySweepOutcome::Inconclusive
                && terminal.cause
                    == crate::autotune::CommissionError::VelocityRestNotConfirmed.status_code()
                && !terminal.recovery_unavailable
        ));
    });
}

fn authority() -> StageCAuthority {
    StageCAuthority {
        channel: CHANNEL,
        stage_b_plan_digest: stage_b_plan().digest(),
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

fn combined_stage_c_plan()
-> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
    use crate::commissioning::outer::velocity::integral_plan::plan_combined_velocity_integral_response;
    use crate::commissioning::outer::velocity::sweep_plan::plan_combined_velocity_sweep;

    let request = request();
    let stage_b = plan_combined_velocity_sweep(
        request,
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .expect("valid combined Stage-B plan");
    plan_combined_velocity_integral_response(
        selected_combined_response(),
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
    .expect("valid combined Stage-C plan")
}

#[test]
fn combined_statuses_install_only_a_sufficient_target_reaching_point() {
    let plan = combined_stage_c_plan();
    let execution = plan.execution_plan();
    let mut reached = terminal(VelocityIntegralOutcome::Complete, sufficient_evidence(0));
    reached.plan_digest = plan.digest();
    reached.final_p = plan.final_p;
    reached.rest_boundary_rung_plus_one = 12;

    let point = super::stage_c::combined_acceptance_point(CHANNEL, execution, &reached)
        .unwrap()
        .expect("TargetReached must produce a point");
    assert_eq!(point.selected_i, 1_024);
    assert_eq!(point.target_velocity_rpm, 175);

    let mut sparse = reached;
    sparse.evidence.eligible_masks = [0b11; 2];
    assert_eq!(
        super::stage_c::combined_acceptance_point(CHANNEL, execution, &sparse),
        Ok(None)
    );

    let mut current = reached;
    current.rest_boundary_rung_plus_one = 0;
    current.evidence.current_terminus = Some(11);
    assert_eq!(
        super::stage_c::combined_acceptance_point(CHANNEL, execution, &current),
        Ok(None)
    );

    let mut cap = reached;
    cap.rest_boundary_rung_plus_one = 0;
    assert_eq!(
        super::stage_c::combined_acceptance_point(CHANNEL, execution, &cap),
        Ok(None)
    );

    let mut fault = reached;
    fault.outcome = VelocityIntegralOutcome::Fault;
    assert_eq!(
        super::stage_c::combined_acceptance_point(CHANNEL, execution, &fault),
        Ok(None)
    );
}

fn retained_authority() -> StageCAuthoritySummary {
    let plan = stage_c_plan();
    StageCAuthoritySummary::from_plan(
        CHANNEL,
        STAGE_B_BUILD_REVISION,
        STAGE_C_EVIDENCE_SCHEMA_REVISION,
        authority().stage_b_plan_digest,
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

fn stage_b_terminal_without_a_transition_capable_rung()
-> crate::commissioning::types::VelocitySweepTerminal {
    let mut terminal = stage_b_terminal();
    terminal.selected_memberships = [0x001c_0000, 0x001c_0000, 0x0018_0000];
    terminal.nominated_p = 1_024;
    terminal
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

#[test]
fn outer_safety_timing_terminal_order_composite_preserves_intermediate_state() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        use crate::commission_dispatch::stage_b::{
            apply_stage_b_terminal, preflight_stage_b_request,
        };
        use crate::commission_dispatch::test_support::MockBackend;
        use crate::commissioning::CommissioningEngine;
        use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
        use crate::config::TmcConfig;
        use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig};

        let mut stage_b_slot = StageBCandidateSlot::new();
        apply_stage_b_terminal(stage_b_terminal(), CHANNEL, &mut stage_b_slot);
        let preflight = preflight_stage_b_request(
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            CHANNEL,
            8,
            &mut stage_b_slot,
        )
        .unwrap();
        crate::state::tmc::tmc_channel0()
            .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
            .expect("valid Stage-C test motion scale");
        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(CHANNEL);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_commission_engine(
            CommissioningEngine::new_outer_for_tmc_with_stage_b_preflight(
                1,
                crate::state::tmc::tmc_channel0(),
                outer_params(),
                entry_snapshot(),
                preflight,
            ),
        );
        let mut stage_c_slot = StageCCandidateSlot::new();
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        let tmc = TmcConfig::default_for_test();
        let config = TmcControlConfig::new(84_000_000, PWM_HZ, 0.03125, tmc);

        let mut emitted_reply = None;
        let action = runtime
            .drive_commissioning_terminal_with_stages(
                crate::commissioning::schedule::CommissionTerminal::VelocitySweep(
                    stage_b_terminal(),
                ),
                &mut backend,
                super::CommissionTerminalContext::new(
                    &mut stage_b_slot,
                    &mut stage_c_slot,
                    |reply| emitted_reply = Some(reply),
                ),
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
                &config,
            )
            .await
            .expect("live Stage-C continuation must produce a scheduler action");

        assert!(runtime.is_commission_active());
        assert!(runtime.is_motor_enabled());
        assert!(backend.enable_pin);
        assert_eq!(stage_c_slot.state(), StageCSlotState::AuthorityOnly);
        assert!(matches!(
            action,
            ScheduledCommissionTickAction::WorkflowContinuation
        ));
        let (run_sequence, continued_plan) = match emitted_reply {
            Some(ReplyPayload::VelocityIntegralContinuation {
                run_sequence,
                stage_c_plan,
                ..
            }) => (run_sequence, stage_c_plan),
            _ => panic!("expected velocity-integral workflow continuation"),
        };
        let expected =
            crate::commissioning::outer::velocity::integral_reporting::encode_velocity_integral_plan_trace(
                run_sequence,
                continued_plan,
            );
        let trace_records = backend.velocity_sweep_trace_records.as_slice();
        assert!(
            !trace_records
                .iter()
                .any(|record| record.as_bytes()[0] == 30)
        );
        assert!(!backend.trace_events.iter().any(|event| {
            event.event_id == crate::trace::events::EVENT_COMMISSION_OUTER_SAFETY_TIMING
        }));
        assert_eq!(
            &trace_records[trace_records.len() - expected.len()..],
            expected.as_slice()
        );
    });
}

#[test]
fn stage_c_no_transition_continuation_disarms_and_emits_both_terminals() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        use crate::commission_dispatch::stage_b::{
            apply_stage_b_terminal, preflight_stage_b_request,
        };
        use crate::commission_dispatch::test_support::MockBackend;
        use crate::commissioning::CommissioningEngine;
        use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
        use crate::config::TmcConfig;
        use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig};

        let mut stage_b_slot = StageBCandidateSlot::new();
        apply_stage_b_terminal(
            stage_b_terminal_without_a_transition_capable_rung(),
            CHANNEL,
            &mut stage_b_slot,
        );
        let preflight = preflight_stage_b_request(
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            CHANNEL,
            8,
            &mut stage_b_slot,
        )
        .unwrap();
        crate::state::tmc::tmc_channel0()
            .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
            .expect("valid Stage-C test motion scale");
        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(CHANNEL);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_commission_engine(
            CommissioningEngine::new_outer_for_tmc_with_stage_b_preflight(
                1,
                crate::state::tmc::tmc_channel0(),
                outer_params(),
                entry_snapshot(),
                preflight,
            ),
        );
        let mut stage_c_slot = StageCCandidateSlot::new();
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        let tmc = TmcConfig::default_for_test();
        let config = TmcControlConfig::new(84_000_000, PWM_HZ, 0.03125, tmc);

        let action = runtime
            .drive_commissioning_terminal_with_stages(
                crate::commissioning::schedule::CommissionTerminal::VelocitySweep(
                    stage_b_terminal_without_a_transition_capable_rung(),
                ),
                &mut backend,
                super::CommissionTerminalContext::new(&mut stage_b_slot, &mut stage_c_slot, |_| {
                    panic!("admission failure must be a terminal action")
                }),
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
                &config,
            )
            .await
            .expect("admission failure must produce a terminal action");

        assert!(!runtime.is_commission_active());
        assert!(!runtime.is_motor_enabled());
        assert!(!backend.enable_pin);
        assert_eq!(stage_c_slot.state(), StageCSlotState::Empty);
        let ScheduledCommissionTickAction::Terminal {
            reply:
                ReplyPayload::VelocityIntegralAdmissionFailure {
                    stage_b_terminal,
                    stage_c_terminal,
                    ..
                },
            ..
        } = action
        else {
            panic!("expected the ordered Stage-B/Stage-C admission terminal");
        };
        assert_eq!(
            stage_b_terminal.outcome,
            crate::commissioning::types::VelocitySweepOutcome::Complete
        );
        assert_eq!(stage_c_terminal.outcome, VelocityIntegralOutcome::Failed);
        assert_eq!(
            stage_c_terminal.cause,
            STAGE_C_CAUSE_NO_TRANSITION_CAPABLE_OPERATING_POINT
        );
        assert!(stage_c_terminal.probe_constrained_test_point);
        assert_eq!(stage_c_terminal.expected_observations, 0);
        assert_eq!(stage_c_terminal.emitted_observations, 0);
        assert_eq!(stage_c_terminal.expected_rungs, 0);
        assert_eq!(stage_c_terminal.emitted_rungs, 0);
    });
}

#[test]
fn reproduced_joint_authority_with_fragment_survives_suppressed_continuation() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        use crate::commission_dispatch::stage_b::{
            apply_stage_b_terminal, preflight_stage_b_request,
        };
        use crate::commission_dispatch::test_support::MockBackend;
        use crate::commissioning::CommissioningEngine;
        use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
        use crate::config::TmcConfig;
        use crate::tmc_control::{ScheduledCommissionTickAction, TmcControlConfig};

        let mut stage_b_slot = StageBCandidateSlot::new();
        apply_stage_b_terminal(stage_b_terminal(), CHANNEL, &mut stage_b_slot);
        let preflight = preflight_stage_b_request(
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            CHANNEL,
            8,
            &mut stage_b_slot,
        )
        .unwrap();
        crate::state::tmc::tmc_channel0()
            .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
            .expect("valid Stage-C test motion scale");
        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(CHANNEL);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_commission_engine(
            CommissioningEngine::new_outer_for_tmc_with_stage_b_preflight(
                1,
                crate::state::tmc::tmc_channel0(),
                outer_params(),
                entry_snapshot(),
                preflight,
            ),
        );
        let mut stage_c_slot = StageCCandidateSlot::new();
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        let tmc = TmcConfig::default_for_test();
        let config = TmcControlConfig::new(84_000_000, PWM_HZ, 0.03125, tmc);
        let mut terminal = stage_b_terminal();
        terminal.model_direction_mask = 1;
        terminal.selected_memberships[1] = 0x0006_0000;
        terminal.selected_memberships[2] = 0x0006_0000;
        terminal.recovery_unavailable = true;

        let action = runtime
            .drive_commissioning_terminal_with_stages(
                crate::commissioning::schedule::CommissionTerminal::VelocitySweep(terminal),
                &mut backend,
                super::CommissionTerminalContext::new(&mut stage_b_slot, &mut stage_c_slot, |_| {
                    panic!("suppressed continuation must not emit continuation evidence")
                }),
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
                &config,
            )
            .await
            .expect("suppressed continuation must emit a terminal action");

        assert_eq!(stage_c_slot.state(), StageCSlotState::AuthorityOnly);
        let retained = stage_c_slot
            .authority()
            .expect("reproduced Stage B must retain Stage-C authority");
        let stage_b = stage_b_plan();
        let expected_final_p = stage_b
            .execution_plan(stage_b.digest())
            .gain_at(17)
            .unwrap()
            .raw();
        assert_eq!(retained.directional_memberships[1], 0x0006_0000);
        assert_eq!(retained.stage_b_joint_membership, 0x0006_0000);
        assert_eq!(retained.joint_membership, 0x0006_0000);
        assert_eq!(retained.final_p, expected_final_p);
        assert!(retained.reduced_margin);
        assert_eq!(
            retained.directional_validity,
            [
                StageBDirectionalValidity::ReproducedRegionCoversFinalP,
                StageBDirectionalValidity::Fragment,
            ]
        );
        assert!(!retained.model_interval_available(1));
        assert_eq!(retained.directional_pooled_intervals[1], [0, 0]);
        assert!(matches!(
            action,
            ScheduledCommissionTickAction::Terminal {
                reply: ReplyPayload::VelocitySweepTerminal { terminal, .. },
                ..
            } if terminal.recovery_unavailable
        ));
        let resume = preflight_stage_c_request(request(), &preflight_config(), 9, &stage_c_slot)
            .expect("retained authority must admit the next matching command")
            .expect("retained authority must route directly to Stage C");
        assert_eq!(resume.plan.final_p, retained.final_p);
        assert_eq!(resume.plan.digest(), retained.identity.plan_digest);
    });
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
        STAGE_B_BUILD_REVISION,
        plan.schema_revision,
        plan.authority.stage_b_plan_digest,
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
