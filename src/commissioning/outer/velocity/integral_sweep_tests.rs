use super::integral_plan::VelocityIntegralExecutionPlan;
use super::integral_sweep::{
    INTEGRAL_SLOT_COUNT, advance_integral_cursor, integral_fault_terminal, integral_i_at,
    integral_slot_direction, step_velocity_integral_with_trace,
};
use super::sweep_plan::RequestedVelocitySource;
use crate::autotune::CommissionError;
use crate::autotune::TransientBuffer;
use crate::autotune::velocity_measurement::StationarityExclusion;
use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
use crate::commissioning::outer::velocity::integral_evidence::ExactRateInterval;
use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, VelocityIntegralData, VelocityIntegralState,
};
use crate::commissioning::types::{StepResult, VelocityIntegralEvent, VelocityIntegralOutcome};
use foci_tmc4671::pid::{PidGain, PidGainPair};
use foci_tmc4671::registers::{PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET};

fn plan() -> VelocityIntegralExecutionPlan {
    VelocityIntegralExecutionPlan {
        plan_digest: 7,
        final_p: 1_448,
        planned_velocity_mrev_s: 2_929,
        target_velocity_rpm: 176,
        pwm_hz: 25_000,
        i_start: 5,
        positive_rung_count: 5,
        family_size: 28,
        moving_stroke_us: 512_000,
        zero_settle_us: 500_000,
        analysis_budget_us: 236_000,
        maximum_workflow_ms: 69_888,
        hard_torque_limit: 2_816,
        usable_torque_limit: 2_534,
    }
}

#[test]
fn schedule_has_two_anchors_and_the_exact_positive_i_ladder() {
    let plan = plan();
    let emitted: [u16; 7] = core::array::from_fn(|index| {
        integral_i_at(plan, index as u8).expect("declared rung must exist")
    });
    assert_eq!(emitted, [0, 5, 10, 20, 40, 80, 0]);
    assert_eq!(integral_i_at(plan, 7), None);
}

#[test]
fn every_rung_uses_the_fixed_frfrfrfr_slot_order() {
    assert_eq!(INTEGRAL_SLOT_COUNT, 8);
    assert_eq!(
        core::array::from_fn::<_, 8, _>(|slot| integral_slot_direction(slot as u8).unwrap()),
        [1, -1, 1, -1, 1, -1, 1, -1]
    );
    assert_eq!(integral_slot_direction(8), None);

    let mut rung = 0;
    let mut slot = 0;
    for expected_slot in 1..8 {
        assert!(!advance_integral_cursor(&mut rung, &mut slot, 7));
        assert_eq!((rung, slot), (0, expected_slot));
    }
    assert!(!advance_integral_cursor(&mut rung, &mut slot, 7));
    assert_eq!((rung, slot), (1, 0));
    for _ in 0..47 {
        advance_integral_cursor(&mut rung, &mut slot, 7);
    }
    assert!(advance_integral_cursor(&mut rung, &mut slot, 7));
    assert_eq!((rung, slot), (7, 0));
}

#[test]
fn plan_p_and_timing_remain_frozen_across_every_rung() {
    let plan = plan();
    for rung in 0..7 {
        assert!(integral_i_at(plan, rung).is_some());
        assert_eq!(plan.final_p, 1_448);
        assert_eq!(plan.moving_stroke_us, 512_000);
        assert_eq!(plan.zero_settle_us, 500_000);
        assert_eq!(plan.analysis_budget_us, 236_000);
    }
}

fn phase() -> OuterPhase {
    let plan = plan();
    OuterPhase::VelocityIntegral {
        state: VelocityIntegralState::Setup,
        data: VelocityIntegralData::new(plan),
        safety: OuterSafetyLimits::new(
            plan.hard_torque_limit,
            OuterMotionBudget {
                requested_velocity_mrev_s: 2_929,
                machine_velocity_ceiling_mrev_s: 10_000,
                requested_velocity_source: RequestedVelocitySource::Defaulted,
                max_stroke_travel_mrev: 2_000,
                settle_travel_reserve_mrev: 100,
                negative_position_headroom_mrev: 50_000,
                positive_position_headroom_mrev: 50_000,
                max_duration_ms: 60_000,
            },
        ),
    }
}

#[test]
fn setup_and_first_command_preserve_plan_before_motion_and_clear_integrator() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(2_816);

    assert!(matches!(
        pollster::block_on(step_velocity_integral_with_trace(
            &mut phase,
            &mut workspace,
            &mut backend,
        )),
        StepResult::ReadyNow
    ));
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));

    assert!(matches!(
        pollster::block_on(step_velocity_integral_with_trace(
            &mut phase,
            &mut workspace,
            &mut backend,
        )),
        StepResult::ReadyNow
    ));
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_P_VELOCITY_I.addr()
                && *value == PidGainPair::new(PidGain::new(1_448).unwrap(), PidGain::ZERO)
                    .register_value())
    }));

    pollster::block_on(step_velocity_integral_with_trace(
        &mut phase,
        &mut workspace,
        &mut backend,
    ));
    assert_eq!(backend.pid_velocity_error_sum_clears, 1);
    assert!(matches!(
        pollster::block_on(step_velocity_integral_with_trace(
            &mut phase,
            &mut workspace,
            &mut backend,
        )),
        StepResult::At(1_000)
    ));
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value == 176)
    }));
}

#[test]
fn opening_anchor_exclusion_short_circuits_before_positive_i_motion() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_classes = [VelocityObservationClass::Inert; 8];
    data.rung_intervals = [ExactRateInterval::EMPTY; 8];
    let mut backend = MockBackend::new();

    assert!(matches!(
        pollster::block_on(step_velocity_integral_with_trace(
            &mut phase,
            &mut workspace,
            &mut backend,
        )),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Rung(_))
    ));
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Terminal,
            ..
        }
    ));
    let StepResult::VelocityIntegralTerminal(terminal) = pollster::block_on(
        step_velocity_integral_with_trace(&mut phase, &mut workspace, &mut backend),
    ) else {
        panic!("opening-anchor failure must produce the dedicated terminal")
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(terminal.emitted_rungs, 1);
    assert_eq!(terminal.emitted_observations, 0);
}

#[test]
fn excluded_closing_bookend_does_not_discard_a_sufficient_curve() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_index = data.plan.positive_rung_count + 1;
    data.opening_direction_mask = 3;
    data.eligible_masks = [0b111; 2];
    for (index, value) in [10_i32, 20, -20, -10].into_iter().enumerate() {
        assert!(workspace.write_workspace_word(index, value as u32));
    }
    for rung in 0..3 {
        let forward = 4 + rung * 2;
        let reverse = 4 + (30 + rung) * 2;
        for (index, value) in [(forward, 1_i32), (forward + 1, 2)] {
            assert!(workspace.write_workspace_word(index, value as u32));
        }
        for (index, value) in [(reverse, -2_i32), (reverse + 1, -1)] {
            assert!(workspace.write_workspace_word(index, value as u32));
        }
    }
    data.rung_classes =
        [VelocityObservationClass::StationarityExcluded(StationarityExclusion::NoBlockingPlateau);
            8];
    data.rung_intervals = [ExactRateInterval::EMPTY; 8];
    let mut backend = MockBackend::new();

    pollster::block_on(step_velocity_integral_with_trace(
        &mut phase,
        &mut workspace,
        &mut backend,
    ));
    let StepResult::VelocityIntegralTerminal(terminal) = pollster::block_on(
        step_velocity_integral_with_trace(&mut phase, &mut workspace, &mut backend),
    ) else {
        panic!("sufficient curve must reach the dedicated terminal")
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::CompleteCandidate);
    assert_eq!(terminal.evidence.bookend, [None, None]);
}

#[test]
fn stage_c_safety_fault_uses_the_dedicated_terminal() {
    let phase = phase();
    let StepResult::VelocityIntegralTerminal(terminal) =
        integral_fault_terminal(&phase, CommissionError::SafetyEnvelopeViolation, None)
    else {
        panic!("Stage-C fault must not fall through to the generic terminal")
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        CommissionError::SafetyEnvelopeViolation.status_code()
    );
    assert_eq!(terminal.expected_observations, 56);
    assert_eq!(terminal.expected_rungs, 7);
}
