use super::current_guard::{VelocityCurrentObserver, VelocityLimitEvidenceCapability};
use super::integral_plan::VelocityIntegralExecutionPlan;
use super::integral_sweep::{
    INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE, INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY,
    INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY_AFTER_SUFFICIENCY, INTEGRAL_SLOT_COUNT,
    advance_integral_cursor, apply_integral_current_headroom, integral_fault_terminal,
    integral_i_at, integral_recovery_current_contact_policy, integral_slot_direction,
    opening_recovery_lower_rate_q, positive_directional_floors_exist,
    step_velocity_integral_with_trace,
};
use super::recovery::{
    ExactU64Parts, RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryStage,
    RungOriginRecoveryOutcome, RungOriginRecoveryReport,
};
use super::sweep_plan::RequestedVelocitySource;
use crate::autotune::CommissionError;
use crate::autotune::TransientBuffer;
use crate::autotune::velocity_measurement::{StationarityExclusion, StationarityResult};
use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
use crate::commissioning::outer::state::{
    PositiveIRestBoundary, VelocitySettleMetrics, VelocitySettleTransition,
};
use crate::commissioning::outer::velocity::integral_evidence::ExactRateInterval;
use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, VelocityIntegralData, VelocityIntegralState,
};
use crate::commissioning::types::{StepResult, VelocityIntegralEvent, VelocityIntegralOutcome};
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET};

#[test]
fn velocity_integral_outcome_codes_preserve_rest_boundary_causes() {
    assert_eq!(INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY, 9);
    assert_eq!(
        INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY_AFTER_SUFFICIENCY,
        10
    );
}

fn plan() -> VelocityIntegralExecutionPlan {
    VelocityIntegralExecutionPlan {
        plan_digest: 7,
        final_p: 1_448,
        planned_velocity_mrev_s: 2_929,
        target_velocity_rpm: 176,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
        i_start: 1,
        positive_rung_count: 5,
        family_size: 28,
        moving_stroke_us: 512_000,
        zero_settle_us: 500_000,
        analysis_budget_us: 236_000,
        maximum_workflow_ms: 69_888,
        recovery_origin_band_counts: 1_000,
        recovery_nominal_slot_us: 1_816_958,
        recovery_maximum_slot_us: 3_000_000,
        recovery_slot_count: 7,
        hard_torque_limit: 2_816,
        usable_torque_limit: 2_534,
        limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
        recovery_quantization_exposed: true,
        probe_constrained_test_point: false,
    }
}

#[test]
fn schedule_has_two_anchors_and_the_exact_positive_i_ladder() {
    let mut plan = plan();
    plan.positive_rung_count = 12;
    plan.family_size = 56;
    plan.recovery_slot_count = 14;
    let emitted: [u16; 14] = core::array::from_fn(|index| {
        integral_i_at(plan, index as u8).expect("declared rung must exist")
    });
    assert_eq!(
        emitted,
        [0, 1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024, 0]
    );
    assert_eq!(integral_i_at(plan, 14), None);
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

#[test]
fn integral_recovery_inverts_opening_deficits_in_the_encoder_rate_domain() {
    let mut workspace = TransientBuffer::new();
    for (index, value) in [1_000_000_i32, 2_000_000, -2_000_000, -1_000_000]
        .into_iter()
        .enumerate()
    {
        assert!(workspace.write_workspace_word(index, value as u32));
    }

    let forward = opening_recovery_lower_rate_q(&workspace, 3, 176, 4_000, 0)
        .unwrap()
        .unwrap();
    let reverse = opening_recovery_lower_rate_q(&workspace, 3, 176, 4_000, 1)
        .unwrap()
        .unwrap();

    assert_eq!(forward, 9_733_333);
    assert_eq!(reverse, 9_733_333);
}

#[test]
fn integral_recovery_rejects_absent_or_zero_straddling_opening_authority() {
    let mut workspace = TransientBuffer::new();
    for (index, value) in [-1_i32, 11_733_334, -11_733_334, 1].into_iter().enumerate() {
        assert!(workspace.write_workspace_word(index, value as u32));
    }

    assert_eq!(
        opening_recovery_lower_rate_q(&workspace, 0, 176, 4_000, 0),
        Ok(None)
    );
    assert_eq!(
        opening_recovery_lower_rate_q(&workspace, 3, 176, 4_000, 0),
        Ok(None)
    );
    assert_eq!(
        opening_recovery_lower_rate_q(&workspace, 3, 176, 4_000, 1),
        Ok(None)
    );
}

#[test]
fn integral_recovery_follows_the_emitted_opening_anchor_and_restores_i_ladder_order() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = core::array::from_fn(|slot| {
        if slot.is_multiple_of(2) {
            ExactRateInterval::new(1_000_000, 2_000_000).unwrap()
        } else {
            ExactRateInterval::new(-2_000_000, -1_000_000).unwrap()
        }
    });
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    safety.unwrapped_position = 1_001;
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Rung(_))
    ));
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::RecoverRung(_),
            ..
        }
    ));
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_P_VELOCITY_I.addr()
                && *value == PidGainPair::new(PidPGain::new(1_448).unwrap(), PidIGain::ZERO)
                    .register_value())
    }));
}

#[test]
fn integral_recovery_preserves_probe_clamp() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = core::array::from_fn(|slot| {
        if slot.is_multiple_of(2) {
            ExactRateInterval::new(1_000_000, 2_000_000).unwrap()
        } else {
            ExactRateInterval::new(-2_000_000, -1_000_000).unwrap()
        }
    });
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.hard_torque_limit)),
        data.plan.limit_evidence_capability,
    );
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    safety.unwrapped_position = 1_001;
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Rung(_))
    ));
    let OuterPhase::VelocityIntegral { data, .. } = phase else {
        unreachable!()
    };
    assert_eq!(
        data.current_observer.clamp_limit(),
        data.plan.usable_torque_limit
    );
}

#[test]
fn integral_recovery_containment_is_cause_five_before_the_bookend() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_index = 1;
    data.opening_direction_mask = 3;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = [ExactRateInterval::new(10, 20).unwrap(); 8];
    for (index, value) in [-1_i32, 11_733_334, -11_733_334, 1].into_iter().enumerate() {
        assert!(workspace.write_workspace_word(index, value as u32));
    }
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    safety.unwrapped_position = 1_001;
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Rung(_))
    ));
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Recovery(report))
            if report.report.outcome
                == super::recovery::RungOriginRecoveryOutcome::ContainedNoRate
    ));
    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("contained intermediate recovery must terminate Stage C");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(terminal.cause, 5);
    assert!(!terminal.recovery_unavailable);
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));
}

#[test]
fn partial_integral_current_terminus_does_not_schedule_a_bookend() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 3;

    assert!(!positive_directional_floors_exist(data));
    apply_integral_current_headroom(state, data);

    assert_eq!(data.current_terminus, Some(2));
    assert_eq!(data.rung_index, 3);
    assert!(matches!(state, VelocityIntegralState::Terminal));
}

#[test]
fn post_sufficiency_current_terminus_restores_origin_before_bookend() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 3;
    data.eligible_masks = [0b111; 2];
    data.current_terminus = Some(2);
    assert!(positive_directional_floors_exist(data));
    *state = VelocityIntegralState::FinalizeRecovery(RungOriginRecoveryReport {
        stage: RecoveryStage::Integral,
        rung_index: 3,
        p_raw: 1_448,
        planned_velocity_mrev_s: 2_929,
        start_offset_counts: 1_500,
        closest_offset_counts: 900,
        final_offset_counts: 900,
        origin_band_counts: 1_000,
        lower_rate_q: ExactU64Parts::from_u64(10_000_000),
        moving_duration_us: 100_000,
        settle_duration_us: 500_000,
        total_duration_us: 600_000,
        peak_torque_target_abs: 1_000,
        binding_source: RecoveryBindingSource::SettledRate,
        outcome: RungOriginRecoveryOutcome::Recovered,
    });
    let mut workspace = TransientBuffer::new();
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Recovery(_))
    ));
    let OuterPhase::VelocityIntegral { state, data, .. } = phase else {
        unreachable!()
    };
    assert_eq!(data.rung_index, data.plan.positive_rung_count + 1);
    assert!(matches!(state, VelocityIntegralState::WriteGains));
}

#[test]
fn recovered_with_current_headroom_advances_to_the_required_bookend() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 3;
    data.eligible_masks = [0b111; 2];
    data.current_terminus = Some(2);
    *state = VelocityIntegralState::FinalizeRecovery(RungOriginRecoveryReport {
        stage: RecoveryStage::Integral,
        rung_index: 3,
        p_raw: 1_448,
        planned_velocity_mrev_s: 2_929,
        start_offset_counts: 1_834,
        closest_offset_counts: 998,
        final_offset_counts: 998,
        origin_band_counts: 1_000,
        lower_rate_q: ExactU64Parts::from_u64(9_174_528),
        moving_duration_us: 100_000,
        settle_duration_us: 500_000,
        total_duration_us: 600_000,
        peak_torque_target_abs: 2_534,
        binding_source: RecoveryBindingSource::CurrentHeadroom,
        outcome: RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom,
    });
    let mut workspace = TransientBuffer::new();
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Recovery(_))
    ));
    let OuterPhase::VelocityIntegral { state, data, .. } = phase else {
        unreachable!()
    };
    assert!(data.recovered_with_current_headroom);
    assert!(!data.recovery_unavailable);
    assert_eq!(data.rung_index, data.plan.positive_rung_count + 1);
    assert!(matches!(state, VelocityIntegralState::WriteGains));
}

#[test]
fn recovery_current_contact_policy_is_limited_to_the_final_positive_recovery() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { data, .. } = &mut phase else {
        unreachable!()
    };

    data.rung_index = 0;
    assert_eq!(
        integral_recovery_current_contact_policy(data),
        RecoveryCurrentContactPolicy::StopImmediately
    );

    data.rung_index = 1;
    assert_eq!(
        integral_recovery_current_contact_policy(data),
        RecoveryCurrentContactPolicy::StopImmediately
    );

    data.eligible_masks = [0b111; 2];
    assert_eq!(
        integral_recovery_current_contact_policy(data),
        RecoveryCurrentContactPolicy::StopImmediately
    );

    data.rung_index = data.plan.positive_rung_count;
    assert_eq!(
        integral_recovery_current_contact_policy(data),
        RecoveryCurrentContactPolicy::ProvePassiveSettle
    );

    data.rung_index = 2;
    data.current_terminus = Some(1);
    assert_eq!(
        integral_recovery_current_contact_policy(data),
        RecoveryCurrentContactPolicy::ProvePassiveSettle
    );

    data.current_terminus = None;
    data.positive_i_rest_boundary_after_sufficiency = true;
    assert_eq!(
        integral_recovery_current_contact_policy(data),
        RecoveryCurrentContactPolicy::ProvePassiveSettle
    );

    data.rung_index = data.plan.positive_rung_count + 1;
    assert_eq!(
        integral_recovery_current_contact_policy(data),
        RecoveryCurrentContactPolicy::StopImmediately
    );
}

#[test]
fn positive_i_rest_rejection_enters_cleanup_before_any_new_motion() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    data.rung_index = 1;
    data.slot = 0;
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    data.settle_min_displacement_counts = 0;
    data.settle_max_displacement_counts = 2;
    for _ in 0..256 {
        data.observations.push(1, 1_000).unwrap();
    }
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    *state = VelocityIntegralState::AnalyzeSettle {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    let event = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            event => break event,
        }
    };

    assert!(matches!(
        event,
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Current(_))
    ));
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::BeginPositiveIRestCleanup,
            ..
        }
    ));
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_P_VELOCITY_I.addr()
                && *value == PidGainPair::new(PidPGain::new(1_448).unwrap(), PidIGain::ZERO)
                    .register_value())
    }));
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::At(1_000)
    ));
    assert_eq!(backend.pid_velocity_error_sum_clears, 1);
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_P_VELOCITY_I.addr()
                && *value == PidGainPair::new(PidPGain::new(1_448).unwrap(), PidIGain::ZERO)
                    .register_value())
    }));
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::PositiveIRestCleanupCapture {
                next_sample_us: 1_000,
                deadline_us: 500_000,
            },
            ..
        }
    ));

    for sample in 1..=500_u64 {
        backend.elapsed_us_value = sample * 1_000;
        step(&mut phase, &mut workspace, &mut backend);
    }
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::AnalyzePositiveIRestCleanup {
                analysis_deadline_us: 736_000,
            },
            ..
        }
    ));
}

#[test]
fn positive_i_remains_programmed_while_the_scored_rest_is_acquired() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 1;
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    *state = VelocityIntegralState::CommandZero;
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));

    assert_eq!(backend.pid_velocity_error_sum_clears, 1);
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, _)
            if *address == PID_VELOCITY_P_VELOCITY_I.addr())
    }));
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::AnalyzeMoving { .. },
            ..
        }
    ));
}

#[test]
fn settle_transition_metrics_retain_exact_first_and_last_context() {
    let mut metrics = VelocitySettleMetrics::EMPTY;

    metrics.observe(0, 1_001).unwrap();
    metrics.observe(2, 999).unwrap();
    metrics.observe(-1, 1_000).unwrap();

    assert_eq!(metrics.sample_count, 3);
    assert_eq!(metrics.duration_us, 3_000);
    assert_eq!(metrics.net_displacement_counts, 1);
    assert_eq!(metrics.minimum_displacement_counts, 0);
    assert_eq!(metrics.maximum_displacement_counts, 2);
    assert_eq!(metrics.transition_count, 2);
    assert_eq!(
        metrics.first_transition,
        VelocitySettleTransition {
            sample_index: 2,
            elapsed_us: 2_000,
            delta_counts: 2,
            displacement_counts: 2,
        }
    );
    assert_eq!(
        metrics.last_transition,
        VelocitySettleTransition {
            sample_index: 3,
            elapsed_us: 3_000,
            delta_counts: -1,
            displacement_counts: 1,
        }
    );
}

#[test]
fn stage_c_terminal_causes_reserve_bookend_and_rest_boundary_values() {
    assert_eq!(super::integral_sweep::INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE, 8);
    assert_eq!(
        super::integral_sweep::INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY,
        9
    );
    assert_eq!(
        super::integral_sweep::INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY_AFTER_SUFFICIENCY,
        10
    );
}

fn seed_boundary(data: &mut VelocityIntegralData, rung_index: u8, slot: u8) {
    let observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    data.rung_index = rung_index;
    data.slot = slot;
    data.positive_i_rest_boundary = Some(PositiveIRestBoundary {
        rung_index,
        slot,
        i_raw: integral_i_at(data.plan, rung_index).unwrap(),
        direction: integral_slot_direction(slot).unwrap() as i8,
        active_stationarity: StationarityResult::Excluded(
            StationarityExclusion::StructuredResidual,
        ),
        active_settle: VelocitySettleMetrics::EMPTY,
        zero_phase_current: observer
            .evidence(super::current_guard::VelocityCurrentZeroPhase::Braking)
            .unwrap(),
        cleanup_confirmed: false,
    });
    data.current_observer = observer;
}

#[test]
fn successful_cleanup_before_directional_floors_is_cause_nine() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    seed_boundary(data, 1, 3);
    data.opening_direction_mask = 3;
    for (index, value) in [10_i32, 20, -20, -10].into_iter().enumerate() {
        assert!(workspace.write_workspace_word(index, value as u32));
    }
    for _ in 0..256 {
        data.observations.push(0, 1_000).unwrap();
    }
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    while matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ) && !matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Terminal,
            ..
        }
    ) {}

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("successful pre-floor cleanup must terminate with boundary evidence");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(terminal.cause, 9);
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));
}

#[test]
fn failed_i_zero_cleanup_remains_a_fault() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    seed_boundary(data, 1, 3);
    data.settle_min_displacement_counts = 0;
    data.settle_max_displacement_counts = 2;
    for _ in 0..256 {
        data.observations.push(1, 1_000).unwrap();
    }
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    let terminal = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            StepResult::VelocityIntegralTerminal(terminal) => break terminal,
            other => panic!("unexpected cleanup result: {other:?}"),
        }
    };

    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        CommissionError::VelocityValidationFailed.status_code()
    );
}

#[test]
fn current_headroom_precedes_positive_i_rest_boundary_classification() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 1;
    data.slot = 0;
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    data.current_observer
        .observe_moving_output(data.plan.usable_torque_limit);
    data.observation_current_headroom = true;
    data.settle_min_displacement_counts = 0;
    data.settle_max_displacement_counts = 2;
    for _ in 0..256 {
        data.observations.push(1, 1_000).unwrap();
    }
    *state = VelocityIntegralState::AnalyzeSettle {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    let event = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            event => break event,
        }
    };

    assert!(matches!(
        event,
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Current(_))
    ));
    let OuterPhase::VelocityIntegral { state, data, .. } = phase else {
        unreachable!()
    };
    assert!(matches!(state, VelocityIntegralState::Terminal));
    assert!(data.positive_i_rest_boundary.is_none());
}

#[test]
fn successful_post_floor_cleanup_restores_origin_before_the_bookend() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    seed_sufficient_curve(data, &mut workspace);
    seed_boundary(data, 4, 3);
    for _ in 0..256 {
        data.observations.push(0, 1_000).unwrap();
    }
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    while matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ) && !matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::RecoverRung(_),
            ..
        }
    ) {}

    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    assert!(data.positive_i_rest_boundary_after_sufficiency);
    assert_eq!(data.eligible_masks, [0b111; 2]);
    assert!(matches!(state, VelocityIntegralState::RecoverRung(_)));
    *state = VelocityIntegralState::FinalizeRecovery(RungOriginRecoveryReport {
        stage: RecoveryStage::Integral,
        rung_index: data.rung_index,
        p_raw: data.plan.final_p,
        planned_velocity_mrev_s: data.plan.planned_velocity_mrev_s,
        start_offset_counts: 1_500,
        closest_offset_counts: 900,
        final_offset_counts: 900,
        origin_band_counts: 1_000,
        lower_rate_q: ExactU64Parts::from_u64(10_000_000),
        moving_duration_us: 100_000,
        settle_duration_us: 500_000,
        total_duration_us: 600_000,
        peak_torque_target_abs: 1_000,
        binding_source: RecoveryBindingSource::SettledRate,
        outcome: RungOriginRecoveryOutcome::Recovered,
    });

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Recovery(_))
    ));
    let OuterPhase::VelocityIntegral { state, data, .. } = phase else {
        unreachable!()
    };
    assert_eq!(data.rung_index, data.plan.positive_rung_count + 1);
    assert!(matches!(state, VelocityIntegralState::WriteGains));
}

#[test]
fn sufficient_boundary_run_uses_cause_ten_after_the_required_bookend() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    seed_sufficient_curve(data, &mut workspace);
    seed_boundary(data, 4, 3);
    data.positive_i_rest_boundary_after_sufficiency = true;
    data.bookend_available_mask = 3;
    for (index, value) in [100_i32, 200, -200, -100].into_iter().enumerate() {
        assert!(workspace.write_workspace_word(124 + index, value as u32));
    }
    *state = VelocityIntegralState::Terminal;
    let mut backend = MockBackend::new();

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("a sufficient boundary run must retain the response candidate");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::CompleteCandidate);
    assert_eq!(terminal.cause, 10);
}

fn phase() -> OuterPhase {
    let plan = plan();
    OuterPhase::VelocityIntegral {
        state: VelocityIntegralState::Setup,
        data: VelocityIntegralData::new(7, plan),
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

fn step(
    phase: &mut OuterPhase,
    workspace: &mut TransientBuffer,
    backend: &mut MockBackend,
) -> StepResult {
    pollster::block_on(step_velocity_integral_with_trace(phase, workspace, backend))
}

fn seed_sufficient_curve(data: &mut VelocityIntegralData, workspace: &mut TransientBuffer) {
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
}

#[test]
fn setup_and_first_command_preserve_plan_before_motion_and_clear_integrator() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(2_816);

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_P_VELOCITY_I.addr()
                && *value == PidGainPair::new(PidPGain::new(1_448).unwrap(), PidIGain::ZERO)
                    .register_value())
    }));

    step(&mut phase, &mut workspace, &mut backend);
    assert_eq!(backend.pid_velocity_error_sum_clears, 1);
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
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
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Rung(_))
    ));
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Terminal,
            ..
        }
    ));
    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("opening-anchor failure must produce the dedicated terminal")
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(terminal.emitted_rungs, 1);
    assert_eq!(terminal.emitted_observations, 0);
}

#[test]
fn excluded_closing_bookend_makes_the_curve_inconclusive() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_index = data.plan.positive_rung_count + 1;
    seed_sufficient_curve(data, &mut workspace);
    data.rung_classes =
        [VelocityObservationClass::StationarityExcluded(StationarityExclusion::NoBlockingPlateau);
            8];
    data.rung_intervals = [ExactRateInterval::EMPTY; 8];
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    let mut backend = MockBackend::new();

    step(&mut phase, &mut workspace, &mut backend);
    step(&mut phase, &mut workspace, &mut backend);
    step(&mut phase, &mut workspace, &mut backend);
    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("bookend insufficiency must reach the dedicated terminal")
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(terminal.cause, INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE);
    assert_eq!(terminal.evidence.bookend, [None, None]);
}

#[test]
fn contained_recovery_after_an_eligible_bookend_is_nonblocking() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral {
        state,
        data,
        safety,
    } = &mut phase
    else {
        unreachable!()
    };
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_index = data.plan.positive_rung_count + 1;
    seed_sufficient_curve(data, &mut workspace);
    for (index, value) in [-1_i32, 11_733_334, -11_733_334, 1].into_iter().enumerate() {
        assert!(workspace.write_workspace_word(index, value as u32));
    }
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = core::array::from_fn(|slot| {
        if slot % 2 == 0 {
            ExactRateInterval::new(100, 200).unwrap()
        } else {
            ExactRateInterval::new(-200, -100).unwrap()
        }
    });
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    safety.unwrapped_position = 1_001;
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Rung(_))
    ));
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Recovery(report))
            if report.report.outcome
                == super::recovery::RungOriginRecoveryOutcome::ContainedNoRate
    ));
    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("contained final recovery must preserve the Stage-C terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::CompleteCandidate);
    assert_eq!(terminal.cause, 0);
    assert!(terminal.recovery_unavailable);
    assert_eq!(
        terminal.evidence.bookend,
        [
            Some(ExactRateInterval::new(100, 200).unwrap()),
            Some(ExactRateInterval::new(-200, -100).unwrap()),
        ]
    );
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
