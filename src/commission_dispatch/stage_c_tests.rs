use super::stage_c::{
    STAGE_C_CAUSE_REPRODUCTION_MISMATCH, StageCPreflightRejection, apply_stage_c_terminal,
    preflight_stage_c_request,
};
use super::{CommissionOutcome, ReplyPayload, drive_commission_result_with_stages};
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use crate::commissioning::outer::velocity::integral_evidence::{
    ExactRateInterval, IntegralRunEvidence,
};
use crate::commissioning::outer::velocity::integral_plan::{
    STAGE_C_EVIDENCE_SCHEMA_REVISION, StageCAuthority, plan_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    StageCAuthoritySummary, StageCCandidateSlot, StageCSlotState,
};
use crate::commissioning::outer::velocity::reproduction::{
    STAGE_B_BUILD_REVISION, StageBDirectionalValidity,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    RequestedVelocitySource, VelocitySweepRequest, plan_velocity_sweep,
};
use crate::commissioning::types::{VelocityIntegralOutcome, VelocityIntegralTerminal};

const CHANNEL: u8 = 0;
const PWM_HZ: u32 = 25_000;
const TORQUE_LIMIT: u16 = 2_816;
const COUNTS_PER_REV: u32 = 4_000;

fn request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 2_929,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 500,
        settle_travel_reserve_mrev: 100,
        negative_position_headroom_mrev: 2_000,
        positive_position_headroom_mrev: 2_000,
        max_duration_ms: 2_000,
    }
}

fn authority() -> StageCAuthority {
    StageCAuthority {
        channel: CHANNEL,
        stage_b_plan_digest: stage_b_plan().digest(),
        final_p: 1_448,
        directional_memberships: [0x0038_0000; 2],
        joint_membership: 0x0038_0000,
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
    plan_velocity_integral_response(
        authority(),
        stage_b.execution_plan(stage_b.digest()),
        OuterMotionBudget::test_budget(500, 2_929, 2_000),
        PWM_HZ,
        COUNTS_PER_REV,
    )
    .expect("valid Stage-C plan")
}

fn retained_authority() -> StageCAuthoritySummary {
    let plan = stage_c_plan();
    StageCAuthoritySummary::from_plan(
        CHANNEL,
        STAGE_B_BUILD_REVISION,
        STAGE_C_EVIDENCE_SCHEMA_REVISION,
        authority().stage_b_plan_digest,
        plan.execution_plan(),
        authority(),
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
        recovery_unavailable: false,
        recovered_with_current_headroom: false,
        recovery_quantization_exposed: plan.recovery_quantization_exposed,
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

    let preflight = preflight_stage_c_request(
        request(),
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        CHANNEL,
        PWM_HZ,
        7,
        &slot,
    )
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

    let rejection = preflight_stage_c_request(
        changed,
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        CHANNEL,
        PWM_HZ,
        8,
        &slot,
    )
    .expect_err("changed frozen plan must reject before motion");

    assert!(matches!(
        rejection,
        StageCPreflightRejection::RejectedPlanMismatch { .. }
    ));
    assert_eq!(slot.authority(), Some(retained));
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
    let second = apply_stage_c_terminal(second_terminal, &mut slot);
    assert_eq!(second.terminal.outcome, VelocityIntegralOutcome::Complete);
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
        selected_memberships: [0x0038_0000; 3],
        nominated_p: 1_448,
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

#[test]
fn outer_safety_timing_terminal_order_composite_preserves_intermediate_state() {
    pollster::block_on(async {
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
fn reproduced_joint_authority_with_fragment_survives_suppressed_continuation() {
    pollster::block_on(async {
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
        terminal.selected_memberships[1] = 0x0018_0000;
        terminal.selected_memberships[2] = 0x0018_0000;
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
            .gain_at(19)
            .unwrap()
            .raw();
        assert_eq!(retained.directional_memberships[1], 0x0018_0000);
        assert_eq!(retained.joint_membership, 0x0018_0000);
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
        let resume = preflight_stage_c_request(
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            CHANNEL,
            PWM_HZ,
            9,
            &stage_c_slot,
        )
        .expect("retained authority must admit the next matching command")
        .expect("retained authority must route directly to Stage C");
        assert_eq!(resume.plan.final_p, retained.final_p);
        assert_eq!(resume.plan.digest(), retained.identity.plan_digest);
    });
}

#[test]
fn outer_safety_timing_terminal_order_stage_c_resume() {
    pollster::block_on(async {
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
        let preflight = preflight_stage_c_request(
            request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            CHANNEL,
            PWM_HZ,
            9,
            &stage_c_slot,
        )
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

        let outcome = drive_commission_result_with_stages(
            crate::commissioning::types::StepResult::VelocityIntegralTerminal(terminal(
                VelocityIntegralOutcome::CompleteCandidate,
                sufficient_evidence(0),
            )),
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
