use super::integral_plan::IntegralSlotOrder;
use super::integral_plan::{INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US, VelocityIntegralExecutionPlan};
use super::integral_sweep::{
    INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE, INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY,
    INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY_AFTER_SUFFICIENCY, INTEGRAL_SLOT_COUNT,
    advance_integral_cursor, analyze_rung, combined_slot_transition_us, integral_fault_terminal,
    integral_i_at, integral_recovery_current_contact_policy, integral_rung_count,
    integral_slot_direction, opening_recovery_lower_rate_q, plan_integral_recovery,
    pooled_deficit_interval, positive_directional_floors_exist, record_diverted_target_terminus,
    record_integral_observation_rest_selection, record_integral_recovery_rest,
    record_integral_recovery_rest_selection, record_integral_target_if_reached, retain_rung,
    step_velocity_integral_with_trace, terminate_on_current_headroom,
};
use super::recovery::{
    ExactU64Parts, RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryRestAnalysis,
    RecoveryStabilizationSummary, RecoveryStage, RungOriginRecoveryConfig,
    RungOriginRecoveryOutcome, RungOriginRecoveryReport, RungOriginRecoveryRuntime,
    plan_rung_origin_recovery,
};
use super::sweep_plan::{RequestedVelocitySource, VELOCITY_SWEEP_SAMPLE_PERIOD_US};
use super::target_ramp::{RampRate, TargetRamp};
use super::velocity_clamp::{VelocityCurrentObserver, VelocityLimitEvidenceCapability};
use crate::autotune::CommissionError;
use crate::autotune::TmcMechanicalRpm;
use crate::autotune::TransientBuffer;
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::velocity_measurement::confidence_multiplier_95_df15;
use crate::autotune::velocity_measurement::{
    RestSelectionDecision, RestSelectionDisposition, RestStructuralCode, StationarityExclusion,
    StationarityResult,
};
use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
use crate::commissioning::outer::state::{
    PositiveIRestBoundary, VelocitySettleMetrics, VelocitySettleTransition,
};
use crate::commissioning::outer::velocity::integral_evidence::{
    ExactRateInterval, IntegralObservationClass, IntegralRungClass, IntegralRungEvidence,
    IntegralRungKind,
};
use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, VelocityIntegralRuntime,
    VelocityIntegralState,
};
use crate::commissioning::types::{
    IntegralOutcomeCause, RestSelectionOwner, RungOriginRecoveryEvidence, StepResult,
    VelocityIntegralEvent, VelocityIntegralOutcome, VelocityIntegralTerminal,
};
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    MODE_RAMP_MODE_MOTION, PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_TORQUE_FLUX_LIMITS,
    PID_VELOCITY_OFFSET, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

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
        slot_order: IntegralSlotOrder::ForwardFirst,
        plan_digest: 7,
        fixed_p: 1_448,
        planned_velocity_mrev_s: 2_929,
        target_velocity_rpm: 176,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
        i_start: 1,
        positive_rung_count: 5,
        i_top_cap: 0,
        family_size: 28,
        ramp_milli_rpm_per_ms: 24_000,
        ramp_us: 8_000,
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
fn breakaway_target_terminus_stays_sticky_once_recorded() {
    let mut plan = plan();
    plan.target_velocity_rpm = 264;
    plan.encoder_counts_per_rev = 4_000;
    plan.positive_rung_count = 13;
    let mut data = VelocityIntegralRuntime::new(1, plan);
    data.rung_index = 3;

    assert!(
        !record_integral_target_if_reached(
            &mut data,
            [
                ExactRateInterval::new(300_000, 400_000).unwrap(),
                ExactRateInterval::new(-400_000, -300_000).unwrap(),
            ],
        )
        .unwrap()
    );
    assert_eq!(data.target_terminus, None);

    data.rung_index = 4;
    assert!(
        record_integral_target_if_reached(
            &mut data,
            [
                ExactRateInterval::new(-100_000, 340_000).unwrap(),
                ExactRateInterval::new(-340_000, 100_000).unwrap(),
            ],
        )
        .unwrap()
    );
    assert_eq!(data.target_terminus, Some(3));

    data.rung_index = 5;
    assert!(
        record_integral_target_if_reached(
            &mut data,
            [
                ExactRateInterval::new(-200_000, 300_000).unwrap(),
                ExactRateInterval::new(-300_000, 200_000).unwrap(),
            ],
        )
        .unwrap()
    );
    assert_eq!(data.target_terminus, Some(3));
}

#[test]
fn breakaway_records_target_terminus_when_target_reached() {
    let mut plan = plan();
    plan.target_velocity_rpm = 264;
    plan.encoder_counts_per_rev = 4_000;
    plan.positive_rung_count = 13;
    let mut data = VelocityIntegralRuntime::new(1, plan);
    data.rung_index = 4;

    let reached = record_integral_target_if_reached(
        &mut data,
        [
            ExactRateInterval::new(-100_000, 340_000).unwrap(),
            ExactRateInterval::new(-340_000, 100_000).unwrap(),
        ],
    )
    .expect("valid intervals");

    assert!(reached);
    assert_eq!(data.target_terminus, Some(3));
}

#[test]
fn breakaway_leaves_terminus_none_on_sufficiency_without_target() {
    let mut plan = plan();
    plan.target_velocity_rpm = 264;
    plan.encoder_counts_per_rev = 4_000;
    plan.positive_rung_count = 13;
    let mut data = VelocityIntegralRuntime::new(1, plan);
    data.rung_index = 3;

    let reached = record_integral_target_if_reached(
        &mut data,
        [
            ExactRateInterval::new(300_000, 400_000).unwrap(),
            ExactRateInterval::new(-400_000, -300_000).unwrap(),
        ],
    )
    .expect("valid intervals");

    assert!(!reached);
    assert_eq!(data.target_terminus, None);
}

#[test]
fn target_terminus_recorder_reports_target_rate_invalid_for_a_non_positive_target() {
    let mut plan = plan();
    plan.target_velocity_rpm = 0;
    plan.encoder_counts_per_rev = 4_000;
    plan.positive_rung_count = 13;
    let mut data = VelocityIntegralRuntime::new(1, plan);
    data.rung_index = 4;

    let error = record_integral_target_if_reached(
        &mut data,
        [
            ExactRateInterval::new(-100_000, 340_000).unwrap(),
            ExactRateInterval::new(-340_000, 100_000).unwrap(),
        ],
    )
    .unwrap_err();

    assert_eq!(error, CommissionError::TargetRateInvalid);
}

#[test]
fn plan_integral_recovery_reports_rung_index_out_of_range_for_a_rung_past_the_schedule() {
    let mut phase = phase();
    let workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { data, safety, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = integral_rung_count(data.plan).expect("declared plan has a rung count");

    let error = plan_integral_recovery(data, &workspace, safety).unwrap_err();

    assert_eq!(error, CommissionError::RungIndexOutOfRange);
}

#[test]
fn plan_integral_recovery_reports_target_rate_invalid_for_a_negative_target() {
    let mut phase = phase();
    let workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { data, safety, .. } = &mut phase else {
        unreachable!()
    };
    safety.configure_integral_encoder_resolution(4_000).unwrap();
    data.plan.target_velocity_rpm = -1;
    data.rung_index = 0;

    let error = plan_integral_recovery(data, &workspace, safety).unwrap_err();

    assert_eq!(error, CommissionError::TargetRateInvalid);
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

fn completed_recovery_rest_analysis() -> RecoveryRestAnalysis {
    RecoveryRestAnalysis {
        exact_velocity_interval_q: [-17, 29],
        start_offset_counts: -1_212,
        closest_offset_counts: -947,
        final_offset_counts: -947,
        planned_return_distance_counts: 212,
        first_in_band_offset_counts: -1_000,
        displacement_to_band_counts: 212,
        entry_to_band_duration_us: 20_609,
        lead_in: RecoveryStabilizationSummary {
            net_displacement_counts: 42,
            displacement_span_counts: 43,
        },
        scored_net_displacement_counts: 0,
        scored_min_displacement_counts: 0,
        scored_max_displacement_counts: 0,
        p_raw: 1_448,
        peak_torque_target_abs: 1_560,
        stage: RecoveryStage::Integral,
        rung_index: 4,
        stationarity_code: 0,
        interval_present: 1,
        current_contact_code: 0,
        disposition: 0,
    }
}

fn completed_rest_selection() -> RestSelectionDecision {
    RestSelectionDecision {
        structural_codes: [RestStructuralCode::Usable; 3],
        intervals_q: [[-17, 29]; 3],
        interval_present_mask: 0b111,
        usable_mask: 0b111,
        zero_compatible_mask: 0b111,
        selected_mask: 0b100,
        selected_suffix_len: 256,
        disposition: RestSelectionDisposition::StatisticalAcceptance,
        retained_sample_count: 256,
    }
}

fn reset_scored_rest(data: &mut VelocityIntegralRuntime) {
    data.analysis.reset_for_scored_rest(
        data.settle_min_displacement_counts >= -1
            && data.settle_max_displacement_counts <= 1
            && i16::from(data.settle_max_displacement_counts)
                - i16::from(data.settle_min_displacement_counts)
                <= 1,
    );
}

fn completed_recovery_report() -> RungOriginRecoveryReport {
    RungOriginRecoveryReport {
        stage: RecoveryStage::Integral,
        rung_index: 4,
        p_raw: 1_448,
        planned_velocity_mrev_s: 4_394,
        start_offset_counts: -1_212,
        closest_offset_counts: -947,
        final_offset_counts: -947,
        origin_band_counts: 1_000,
        lower_rate_q: ExactU64Parts::from_u64(12_827_648),
        moving_duration_us: 20_609,
        settle_duration_us: 1_000_000,
        total_duration_us: 1_020_609,
        peak_torque_target_abs: 1_560,
        binding_source: RecoveryBindingSource::SettledRate,
        outcome: RungOriginRecoveryOutcome::Recovered,
    }
}

#[test]
fn live_integral_schedule_records_one_selector_per_observation() {
    let mut live_plan = plan();
    live_plan.positive_rung_count = 13;
    live_plan.recovery_slot_count = 15;
    let mut data = VelocityIntegralRuntime::new(31, live_plan);
    let decision = completed_rest_selection();
    let mut emitted = 0_u16;

    for rung_index in 0..15 {
        for slot in 0..INTEGRAL_SLOT_COUNT {
            data.rung_index = rung_index;
            data.slot = slot;
            let owner = if rung_index == 0 || rung_index == 14 {
                RestSelectionOwner::IntegralAnchor
            } else {
                RestSelectionOwner::IntegralPositiveObservation
            };
            let StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(report)) =
                record_integral_observation_rest_selection(&mut data, decision, owner)
            else {
                panic!("every ordinary Integral rest decision must emit Event 151");
            };
            emitted += 1;
            assert_eq!(report.evidence_sequence, emitted);
            assert_eq!(report.owner, owner);
            assert_eq!(report.rung_index, rung_index);
            assert_eq!(report.direction, slot % 2);
            assert_eq!(report.slot, slot);
        }
    }

    assert_eq!(live_plan.recovery_slot_count, 15);
    assert_eq!(emitted, 120);
    assert_eq!(data.integrity.next_evidence_sequence(), 121);

    for rung_index in 0..live_plan.recovery_slot_count {
        let StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(report)) =
            record_integral_recovery_rest_selection(
                &mut data,
                decision,
                RecoveryStage::Integral,
                rung_index,
            )
        else {
            panic!("every completed Integral recovery rest decision must emit Event 151");
        };
        assert_eq!(report.evidence_sequence, 121 + u16::from(rung_index));
        assert_eq!(report.owner, RestSelectionOwner::IntegralRecovery);
        assert_eq!(report.rung_index, rung_index);
        assert_eq!([report.direction, report.slot], [u8::MAX; 2]);
    }
    assert_eq!(data.integrity.next_evidence_sequence(), 136);
}

#[test]
fn integral_places_rest_diagnostics_immediately_before_recovery() {
    let integral_plan = plan();
    let mut integral = VelocityIntegralRuntime::new(17, integral_plan);
    let before = integral.integrity.digest();
    let StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(selection)) =
        record_integral_recovery_rest_selection(
            &mut integral,
            completed_rest_selection(),
            RecoveryStage::Integral,
            4,
        )
    else {
        panic!("Integral must emit its selector diagnostic")
    };
    let StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RecoveryRest(diagnostic)) =
        record_integral_recovery_rest(&mut integral, completed_recovery_rest_analysis())
    else {
        panic!("Integral must emit its recovery-rest diagnostic")
    };
    assert_eq!(diagnostic.run_sequence, 17);
    assert_eq!(selection.evidence_sequence, 1);
    assert_eq!(diagnostic.evidence_sequence, 2);
    assert_eq!(diagnostic.plan_digest, integral.plan.plan_digest);
    assert_eq!(selection.plan_digest, diagnostic.plan_digest);
    assert_eq!(selection.stage as u8, diagnostic.stage as u8);
    assert_eq!(selection.rung_index, diagnostic.rung_index);
    assert_eq!(diagnostic.stage, RecoveryStage::Integral);
    assert_eq!(diagnostic.rung_index, 4);
    let recovery = RungOriginRecoveryEvidence {
        run_sequence: 17,
        evidence_sequence: integral.integrity.next_evidence_sequence(),
        report: completed_recovery_report(),
    };
    let VelocityIntegralEvent::Recovery(recovery) = integral
        .integrity
        .record(VelocityIntegralEvent::Recovery(recovery))
    else {
        unreachable!()
    };
    assert_eq!(recovery.evidence_sequence, diagnostic.evidence_sequence + 1);
    assert_ne!(integral.integrity.digest(), before);
}

#[test]
fn every_rung_uses_the_fixed_frfrfrfr_slot_order() {
    assert_eq!(INTEGRAL_SLOT_COUNT, 8);
    assert_eq!(
        core::array::from_fn::<_, 8, _>(|slot| integral_slot_direction(
            IntegralSlotOrder::ForwardFirst,
            slot as u8
        )
        .unwrap()),
        [1, -1, 1, -1, 1, -1, 1, -1]
    );
    assert_eq!(
        integral_slot_direction(IntegralSlotOrder::ForwardFirst, 8),
        None
    );

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
        assert_eq!(plan.fixed_p, 1_448);
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
fn opening_recovery_forward_settled_rate_overflowing_u64_reports_target_rate_invalid() {
    let mut workspace = TransientBuffer::new();
    for index in 0..4 {
        assert!(workspace.write_workspace_word(index, 0));
    }

    let error = opening_recovery_lower_rate_q(&workspace, 3, i32::MAX, u32::MAX, 0).unwrap_err();

    assert_eq!(error, CommissionError::TargetRateInvalid);
}

#[test]
fn opening_recovery_reverse_settled_rate_overflowing_u64_reports_target_rate_invalid() {
    let mut workspace = TransientBuffer::new();
    for index in 0..4 {
        assert!(workspace.write_workspace_word(index, 0));
    }

    let error = opening_recovery_lower_rate_q(&workspace, 3, i32::MAX, u32::MAX, 1).unwrap_err();

    assert_eq!(error, CommissionError::TargetRateInvalid);
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
        panic!("contained intermediate recovery must terminate Integral");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(terminal.cause, IntegralOutcomeCause::Engine(5));
    assert!(!terminal.recovery_unavailable);
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));
}

#[test]
fn completed_integral_recovery_rest_rejection_is_cause_53() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rest_unconfirmed = true;
    *state = VelocityIntegralState::Terminal;
    let mut backend = MockBackend::new();

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("completed Integral recovery rest rejection must terminate");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code())
    );
    assert!(!terminal.recovery_unavailable);
}

#[test]
fn partial_integral_current_terminus_does_not_schedule_a_bookend() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 3;

    assert!(!positive_directional_floors_exist(data));
    terminate_on_current_headroom(state, data);

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
    assert!(matches!(
        state,
        VelocityIntegralState::PrepareMovingSlot { reset_rung: true }
    ));
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
    assert!(matches!(
        state,
        VelocityIntegralState::PrepareMovingSlot { reset_rung: true }
    ));
}

fn contained_report(rung_index: u8) -> RungOriginRecoveryReport {
    RungOriginRecoveryReport {
        stage: RecoveryStage::Integral,
        rung_index,
        p_raw: 1_448,
        planned_velocity_mrev_s: 2_929,
        start_offset_counts: 900,
        closest_offset_counts: 900,
        final_offset_counts: 900,
        origin_band_counts: 1_000,
        lower_rate_q: ExactU64Parts::from_u64(490_000),
        moving_duration_us: 0,
        settle_duration_us: 0,
        total_duration_us: 0,
        peak_torque_target_abs: 0,
        binding_source: RecoveryBindingSource::Deadline,
        outcome: RungOriginRecoveryOutcome::ContainedDeadline,
    }
}

struct RungKindCase {
    rung_index: u8,
    current_terminus: Option<u8>,
    positive_i_rest_boundary_after_sufficiency: bool,
    expect_recovery_failure: bool,
    expect_recovery_unavailable: bool,
}

#[test]
fn finalize_recovery_resolves_a_contained_outcome_without_a_fault_at_every_rung_kind() {
    let cases = [
        RungKindCase {
            rung_index: 6,
            current_terminus: None,
            positive_i_rest_boundary_after_sufficiency: false,
            expect_recovery_failure: false,
            expect_recovery_unavailable: true,
        },
        RungKindCase {
            rung_index: 3,
            current_terminus: Some(2),
            positive_i_rest_boundary_after_sufficiency: false,
            expect_recovery_failure: false,
            expect_recovery_unavailable: true,
        },
        RungKindCase {
            rung_index: 3,
            current_terminus: None,
            positive_i_rest_boundary_after_sufficiency: true,
            expect_recovery_failure: false,
            expect_recovery_unavailable: true,
        },
        RungKindCase {
            rung_index: 0,
            current_terminus: None,
            positive_i_rest_boundary_after_sufficiency: false,
            expect_recovery_failure: true,
            expect_recovery_unavailable: false,
        },
        RungKindCase {
            rung_index: 3,
            current_terminus: None,
            positive_i_rest_boundary_after_sufficiency: false,
            expect_recovery_failure: true,
            expect_recovery_unavailable: false,
        },
    ];
    for case in cases {
        let rung_index = case.rung_index;
        let mut phase = phase();
        let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
            unreachable!()
        };
        data.rung_index = rung_index;
        data.current_terminus = case.current_terminus;
        data.positive_i_rest_boundary_after_sufficiency =
            case.positive_i_rest_boundary_after_sufficiency;
        *state = VelocityIntegralState::FinalizeRecovery(contained_report(rung_index));
        let mut workspace = TransientBuffer::new();
        let mut backend = MockBackend::new();

        assert!(
            matches!(
                step(&mut phase, &mut workspace, &mut backend),
                StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Recovery(_))
            ),
            "rung_index {rung_index}"
        );
        let OuterPhase::VelocityIntegral { state, data, .. } = phase else {
            unreachable!()
        };
        assert_eq!(
            data.recovery_failure, case.expect_recovery_failure,
            "rung_index {rung_index}"
        );
        assert_eq!(
            data.recovery_unavailable, case.expect_recovery_unavailable,
            "rung_index {rung_index}"
        );
        assert!(
            matches!(state, VelocityIntegralState::Terminal),
            "rung_index {rung_index}: {state:?}"
        );
        assert!(!backend.spi_log.iter().any(|operation| {
            matches!(operation, SpiOp::Write(address, value)
                if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
        }));
    }
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
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzeSettle {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    let mut saw_rest_selection = false;
    let event = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(_)) => {
                saw_rest_selection = true;
            }
            event => break event,
        }
    };

    assert!(saw_rest_selection);
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
fn moving_exit_zeroes_i_and_target_before_clearing_the_scored_rest() {
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
    *state = final_zero_ramp_tick(data, 0);
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));

    let p = PidPGain::new(1_448).unwrap();
    assert_eq!(
        logged_writes(&backend).as_slice(),
        &[
            (
                PID_VELOCITY_P_VELOCITY_I.addr(),
                PidGainPair::new(p, PidIGain::ZERO).register_value(),
            ),
            (PID_VELOCITY_TARGET.addr(), 0),
            (
                PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                u32::from(PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr),
            ),
            (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
        ]
    );
    assert_eq!(backend.pid_velocity_error_sum_clears, 1);
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
fn integral_terminal_causes_reserve_bookend_and_rest_boundary_values() {
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

fn seed_boundary(data: &mut VelocityIntegralRuntime, rung_index: u8, slot: u8) {
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
        direction: integral_slot_direction(IntegralSlotOrder::ForwardFirst, slot).unwrap() as i8,
        active_stationarity: StationarityResult::Excluded(
            StationarityExclusion::StructuredResidual,
        ),
        active_settle: VelocitySettleMetrics::EMPTY,
        zero_phase_current: observer
            .evidence(super::velocity_clamp::VelocityCurrentZeroPhase::Braking)
            .unwrap(),
        cleanup_confirmed: false,
    });
    data.current_observer = observer;
}

#[test]
fn begin_positive_i_rest_cleanup_reports_missing_capture_state_without_a_boundary() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    assert!(data.positive_i_rest_boundary.is_none());
    *state = VelocityIntegralState::BeginPositiveIRestCleanup;
    let mut backend = MockBackend::new();

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("a missing boundary must emit the Integral integrity terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::MissingCaptureState.status_code())
    );
}

#[test]
fn begin_positive_i_rest_cleanup_reports_deadline_overflow_for_the_sample_grid() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    seed_boundary(data, 1, 3);
    *state = VelocityIntegralState::BeginPositiveIRestCleanup;
    let mut backend = MockBackend::new();
    backend.elapsed_us_value = u64::MAX;

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("sample-grid overflow must emit the Integral integrity terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::ScheduleDeadlineOverflow.status_code())
    );
}

#[test]
fn begin_positive_i_rest_cleanup_reports_deadline_overflow_for_the_settle_window() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    seed_boundary(data, 1, 3);
    *state = VelocityIntegralState::BeginPositiveIRestCleanup;
    let mut backend = MockBackend::new();
    backend.elapsed_us_value = u64::MAX - u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US);

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("settle-window overflow must emit the Integral integrity terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::ScheduleDeadlineOverflow.status_code())
    );
}

#[test]
fn positive_i_rest_cleanup_capture_reports_deadline_overflow_for_the_analysis_budget() {
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
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    seed_boundary(data, 1, 3);
    data.previous_encoder_raw = 0;
    data.previous_read_us = u64::MAX - 1_000;
    *state = VelocityIntegralState::PositiveIRestCleanupCapture {
        next_sample_us: 0,
        deadline_us: 0,
    };
    let mut backend = MockBackend::new();
    backend.elapsed_us_value = u64::MAX;

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("analysis-budget overflow must emit the Integral integrity terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::ScheduleDeadlineOverflow.status_code())
    );
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
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    while !matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Terminal,
            ..
        }
    ) {
        assert!(matches!(
            step(&mut phase, &mut workspace, &mut backend),
            StepResult::ReadyNow | StepResult::VelocityIntegralEvent(_)
        ));
    }

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("successful pre-floor cleanup must terminate with boundary evidence");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(terminal.cause, IntegralOutcomeCause::Engine(9));
    assert!(!backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
    }));
}

#[test]
fn completed_i_zero_cleanup_rejection_is_inconclusive_rest() {
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
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    let mut diagnostic = None;
    let terminal = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(event)) => {
                diagnostic = Some(event);
            }
            StepResult::VelocityIntegralTerminal(terminal) => break terminal,
            other => panic!("unexpected cleanup result: {other:?}"),
        }
    };

    assert_eq!(
        diagnostic
            .expect("completed cleanup analysis must emit Event 151")
            .owner,
        RestSelectionOwner::IntegralCleanup
    );
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code())
    );
    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert_eq!(
        data.rest_unconfirmed_owner,
        Some(RestSelectionOwner::IntegralCleanup)
    );
}

#[test]
fn integral_anchor_and_bookend_unconfirmed_rest_use_dedicated_cause() {
    for closing_bookend in [false, true] {
        let mut phase = phase();
        let mut workspace = TransientBuffer::new();
        let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
            unreachable!()
        };
        data.rung_index = if closing_bookend {
            data.plan.positive_rung_count + 1
        } else {
            0
        };
        data.current_observer = VelocityCurrentObserver::new(
            current_limit(u32::from(data.plan.usable_torque_limit)),
            VelocityLimitEvidenceCapability::BestEffortPolling,
        );
        data.settle_min_displacement_counts = 0;
        data.settle_max_displacement_counts = 2;
        for _ in 0..256 {
            data.observations.push(1, 1_000).unwrap();
        }
        reset_scored_rest(data);
        *state = VelocityIntegralState::AnalyzeSettle {
            analysis_deadline_us: 236_000,
        };
        let mut backend = MockBackend::new();

        let mut diagnostic = None;
        let terminal = loop {
            match step(&mut phase, &mut workspace, &mut backend) {
                StepResult::ReadyNow => {}
                StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(event)) => {
                    diagnostic = Some(event);
                }
                StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Current(_)) => {}
                StepResult::VelocityIntegralTerminal(terminal) => break terminal,
                other => panic!("unexpected generic rest result: {other:?}"),
            }
        };

        let diagnostic = diagnostic.expect("completed Integral rest must emit Event 151");
        assert_eq!(diagnostic.owner, RestSelectionOwner::IntegralAnchor);
        assert_eq!(diagnostic.slot, 0);
        assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
        assert_eq!(
            terminal.cause,
            IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code())
        );
        let OuterPhase::VelocityIntegral { data, .. } = &phase else {
            unreachable!()
        };
        assert_eq!(
            data.rest_unconfirmed_owner,
            Some(RestSelectionOwner::IntegralAnchor)
        );
    }
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
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzeSettle {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    let mut saw_rest_selection = false;
    let event = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(_)) => {
                saw_rest_selection = true;
            }
            event => break event,
        }
    };

    assert!(saw_rest_selection);
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
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    while !matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::RecoverRung(_),
            ..
        }
    ) {
        assert!(matches!(
            step(&mut phase, &mut workspace, &mut backend),
            StepResult::ReadyNow | StepResult::VelocityIntegralEvent(_)
        ));
    }

    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    assert!(data.positive_i_rest_boundary_after_sufficiency);
    assert_eq!(data.eligible_masks, [0b111; 2]);
    assert!(matches!(state, VelocityIntegralState::RecoverRung(_)));
    *state = VelocityIntegralState::FinalizeRecovery(RungOriginRecoveryReport {
        stage: RecoveryStage::Integral,
        rung_index: data.rung_index,
        p_raw: data.plan.fixed_p,
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
    assert!(matches!(
        state,
        VelocityIntegralState::PrepareMovingSlot { reset_rung: true }
    ));
}

#[test]
fn breakaway_rest_boundary_sufficiency_records_target_terminus() {
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
    data.plan.target_velocity_rpm = 264;
    data.plan.encoder_counts_per_rev = 4_000;
    seed_sufficient_curve(data, &mut workspace);
    seed_boundary(data, 4, 3);
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = core::array::from_fn(|slot| {
        if slot.is_multiple_of(2) {
            ExactRateInterval::new(-100_000, 340_000).unwrap()
        } else {
            ExactRateInterval::new(-340_000, 100_000).unwrap()
        }
    });
    for _ in 0..256 {
        data.observations.push(0, 1_000).unwrap();
    }
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    while !matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::RecoverRung(_),
            ..
        }
    ) {
        assert!(matches!(
            step(&mut phase, &mut workspace, &mut backend),
            StepResult::ReadyNow | StepResult::VelocityIntegralEvent(_)
        ));
    }

    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert!(data.positive_i_rest_boundary_after_sufficiency);
    assert_eq!(data.target_terminus, Some(3));
}

#[test]
fn breakaway_rest_boundary_sufficiency_leaves_terminus_none_without_target() {
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
    data.plan.target_velocity_rpm = 264;
    data.plan.encoder_counts_per_rev = 4_000;
    seed_sufficient_curve(data, &mut workspace);
    seed_boundary(data, 4, 3);
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = core::array::from_fn(|slot| {
        if slot.is_multiple_of(2) {
            ExactRateInterval::new(300_000, 400_000).unwrap()
        } else {
            ExactRateInterval::new(-400_000, -300_000).unwrap()
        }
    });
    for _ in 0..256 {
        data.observations.push(0, 1_000).unwrap();
    }
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    while !matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::RecoverRung(_),
            ..
        }
    ) {
        assert!(matches!(
            step(&mut phase, &mut workspace, &mut backend),
            StepResult::ReadyNow | StepResult::VelocityIntegralEvent(_)
        ));
    }

    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert!(data.positive_i_rest_boundary_after_sufficiency);
    assert_eq!(data.target_terminus, None);
}

#[test]
fn breakaway_analyze_rung_records_target_terminus_when_target_reached() {
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
    data.plan.target_velocity_rpm = 264;
    data.plan.encoder_counts_per_rev = 4_000;
    *state = VelocityIntegralState::AnalyzeRung;
    data.rung_index = 1;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = core::array::from_fn(|slot| {
        if slot.is_multiple_of(2) {
            ExactRateInterval::new(-100_000, 340_000).unwrap()
        } else {
            ExactRateInterval::new(-340_000, 100_000).unwrap()
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

    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert_eq!(data.target_terminus, Some(0));
}

#[test]
fn breakaway_clean_sufficiency_preserves_recorded_terminus_through_continuation() {
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
    data.target_terminus = Some(3);
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
        StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Recovery(_))
    ));
    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("clean sufficiency close must reach the terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::FirstRunRetained);
    assert_eq!(terminal.cause, IntegralOutcomeCause::Engine(0));
    assert_eq!(terminal.rest_boundary_rung_plus_one, 4);
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
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::FirstRunRetained);
    assert_eq!(terminal.cause, IntegralOutcomeCause::Engine(10));
}

#[test]
fn rest_rejection_after_sufficiency_is_discriminated_from_a_partial_acquisition() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    seed_sufficient_curve(data, &mut workspace);
    data.bookend_available_mask = 3;
    for (index, value) in [100_i32, 200, -200, -100].into_iter().enumerate() {
        assert!(workspace.write_workspace_word(124 + index, value as u32));
    }
    data.rest_unconfirmed = true;
    data.rest_unconfirmed_owner = Some(RestSelectionOwner::IntegralRecovery);
    *state = VelocityIntegralState::Terminal;
    let mut backend = MockBackend::new();

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("a rest rejection after sufficiency must still terminate Integral");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code())
    );
    assert!(terminal.rest_rejection_after_sufficiency);
    assert_eq!(
        terminal.rest_rejection_owner,
        Some(RestSelectionOwner::IntegralRecovery)
    );
}

#[test]
fn rest_rejection_before_sufficiency_leaves_the_discriminator_unset() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rest_unconfirmed = true;
    data.rest_unconfirmed_owner = Some(RestSelectionOwner::IntegralRecovery);
    *state = VelocityIntegralState::Terminal;
    let mut backend = MockBackend::new();

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("completed Integral recovery rest rejection must terminate");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code())
    );
    assert!(!terminal.rest_rejection_after_sufficiency);
    assert_eq!(terminal.rest_rejection_owner, None);
}

/// The zero ramp's final tick: a one-tick ramp down from the plan's target.
fn final_zero_ramp_tick(
    data: &mut VelocityIntegralRuntime,
    entry_transition_us: u32,
) -> VelocityIntegralState {
    let target_rpm = data.plan.target_velocity_rpm;
    data.last_target_rpm = target_rpm;
    let one_tick = RampRate::from_milli_rpm_per_ms(target_rpm.unsigned_abs() * 1_000)
        .expect("nonzero ramp rate");
    VelocityIntegralState::RampZero {
        ramp: TargetRamp::new(target_rpm, 0, one_tick),
        next_tick_us: 0,
        entry_transition_us,
    }
}

fn phase() -> OuterPhase {
    let plan = plan();
    OuterPhase::VelocityIntegral {
        state: VelocityIntegralState::Setup,
        data: VelocityIntegralRuntime::new(7, plan),
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
                homing_speed_mrev_s: 4_375,
                max_accel_mrev_s2: 200_000,
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

fn logged_writes(backend: &MockBackend) -> heapless::Vec<(u8, u32), 16> {
    let mut writes = heapless::Vec::new();
    for operation in &backend.spi_log {
        if let SpiOp::Write(address, value) = operation {
            writes.push((*address, *value)).unwrap();
        }
    }
    writes
}

fn seed_sufficient_curve(data: &mut VelocityIntegralRuntime, workspace: &mut TransientBuffer) {
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
    assert_eq!(
        logged_writes(&backend).last(),
        Some(&(PID_VELOCITY_TARGET.addr(), 24))
    );
}

#[test]
fn every_moving_slot_entry_zeroes_clears_and_reprograms_the_declared_i() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 1;
    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(2_816);

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    backend.spi_log.clear();
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));

    let p = PidPGain::new(1_448).unwrap();
    assert_eq!(
        logged_writes(&backend).as_slice(),
        &[
            (
                PID_VELOCITY_P_VELOCITY_I.addr(),
                PidGainPair::new(p, PidIGain::ZERO).register_value(),
            ),
            (
                PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                u32::from(PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr),
            ),
            (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
            (
                PID_VELOCITY_P_VELOCITY_I.addr(),
                PidGainPair::new(p, PidIGain::new(1).unwrap()).register_value(),
            ),
            (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0002),
        ]
    );
}

#[test]
fn sweep_start_clears_the_velocity_offset() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(2_816);

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));

    assert!(
        backend.spi_log.iter().any(|operation| matches!(
            operation,
            SpiOp::Write(address, value)
                if *address == PID_VELOCITY_OFFSET.addr() && *value == 0
        )),
        "sweep start must write PID_VELOCITY_OFFSET=0"
    );
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::PrepareMovingSlot { reset_rung: true },
            ..
        }
    ));
}

#[test]
fn slot_advance_reprograms_i_without_resetting_prior_evidence() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 1;
    data.slot = 0;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = [ExactRateInterval::new(10, 20).unwrap(); 8];
    *state = VelocityIntegralState::Advance;
    let mut backend = MockBackend::new();

    for expected_slot in 1..INTEGRAL_SLOT_COUNT {
        backend.spi_log.clear();
        assert!(matches!(
            step(&mut phase, &mut workspace, &mut backend),
            StepResult::ReadyNow
        ));
        assert!(matches!(
            step(&mut phase, &mut workspace, &mut backend),
            StepResult::ReadyNow
        ));
        let writes = logged_writes(&backend);
        assert!(writes.iter().any(|(address, value)| {
            *address == PID_VELOCITY_P_VELOCITY_I.addr()
                && *value
                    == PidGainPair::new(PidPGain::new(1_448).unwrap(), PidIGain::new(1).unwrap())
                        .register_value()
        }));
        let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
            unreachable!()
        };
        assert_eq!(data.slot, expected_slot);
        assert_eq!(data.rung_classes, [VelocityObservationClass::Eligible; 8]);
        assert_eq!(
            data.rung_intervals,
            [ExactRateInterval::new(10, 20).unwrap(); 8]
        );
        *state = VelocityIntegralState::Advance;
    }
}

#[test]
fn rung_reset_clears_only_per_rung_observation_state() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 1;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = [ExactRateInterval::new(10, 20).unwrap(); 8];
    data.eligible_masks = [0b101; 2];
    *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert_eq!(
        data.rung_classes,
        [VelocityObservationClass::InvalidEvidenceFault; 8]
    );
    assert_eq!(data.rung_intervals, [ExactRateInterval::EMPTY; 8]);
    assert_eq!(data.eligible_masks, [0b101; 2]);
}

#[test]
fn entry_and_exit_transaction_failures_fault_before_motion_or_evidence() {
    let entry_failures = [
        (PID_VELOCITY_P_VELOCITY_I.addr(), 0),
        (PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(), 0),
        (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
        (PID_VELOCITY_P_VELOCITY_I.addr(), 1),
        (MODE_RAMP_MODE_MOTION.addr(), 0),
    ];
    for failure in entry_failures {
        let mut phase = phase();
        let mut workspace = TransientBuffer::new();
        let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
            unreachable!()
        };
        data.rung_index = 1;
        *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
        let mut backend = MockBackend::new();
        backend.fail_on_write_occurrence = Some(failure);

        let StepResult::VelocityIntegralTerminal(terminal) =
            step(&mut phase, &mut workspace, &mut backend)
        else {
            panic!("entry failure must emit the Integral integrity terminal");
        };
        assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
        assert!(!backend.spi_log.iter().any(|operation| {
            matches!(operation, SpiOp::Write(address, value)
                if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
        }));
    }

    let exit_failures = [
        (PID_VELOCITY_P_VELOCITY_I.addr(), 0),
        (PID_VELOCITY_TARGET.addr(), 0),
        (PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(), 0),
        (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
    ];
    for failure in exit_failures {
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
        *state = final_zero_ramp_tick(data, 0);
        let mut backend = MockBackend::new();
        backend.fail_on_write_occurrence = Some(failure);

        let StepResult::VelocityIntegralTerminal(terminal) =
            step(&mut phase, &mut workspace, &mut backend)
        else {
            panic!("exit failure must emit the Integral integrity terminal");
        };
        assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
        let OuterPhase::VelocityIntegral { data, .. } = &phase else {
            unreachable!()
        };
        assert_eq!(
            data.rung_classes[0],
            VelocityObservationClass::InvalidEvidenceFault
        );
        assert_eq!(data.rung_intervals[0], ExactRateInterval::EMPTY);
    }
}

#[test]
fn slot_transition_timing_starts_scored_rest_after_the_exit_clear() {
    assert_eq!(INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US, 20_000);
    assert_eq!(combined_slot_transition_us(500, 400), Some(900));
    assert_eq!(combined_slot_transition_us(19_600, 400), Some(20_000));
    assert_eq!(combined_slot_transition_us(19_601, 400), None);

    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.rung_index = 1;
    *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
    let mut backend = MockBackend::new();
    backend.spi_elapsed_step_us = 100;

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    assert!(matches!(
        state,
        VelocityIntegralState::RampTarget {
            entry_transition_us: 500,
            ..
        }
    ));
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    *state = final_zero_ramp_tick(data, 500);
    let before_exit_writes = backend.elapsed_us_value;

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::ReadyNow
    ));
    let exit_write_count = logged_writes(&backend)
        .len()
        .checked_sub(5)
        .expect("entry writes are present");
    assert_eq!(exit_write_count, 4);
    assert_eq!(backend.elapsed_us_value - before_exit_writes, 500);
    let OuterPhase::VelocityIntegral { state, data, .. } = &phase else {
        unreachable!()
    };
    assert!(matches!(
        state,
        VelocityIntegralState::AnalyzeMoving {
            settle_deadline_us,
            ..
        } if *settle_deadline_us == backend.elapsed_us_value + u64::from(data.plan.zero_settle_us)
    ));
}

#[test]
fn prepare_zero_settle_reports_deadline_overflow_for_the_settle_window() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    *state = final_zero_ramp_tick(data, 0);
    let mut backend = MockBackend::new();
    backend.elapsed_us_value = u64::MAX;

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("settle-deadline overflow must emit the Integral integrity terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::ScheduleDeadlineOverflow.status_code())
    );
}

#[test]
fn prepare_zero_settle_reports_deadline_overflow_for_the_analysis_budget() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.plan.zero_settle_us = 100;
    data.plan.analysis_budget_us = 500_000_000;
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    *state = final_zero_ramp_tick(data, 0);
    let mut backend = MockBackend::new();
    backend.elapsed_us_value = u64::MAX - 100;

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("analysis-budget overflow must emit the Integral integrity terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::ScheduleDeadlineOverflow.status_code())
    );
}

#[test]
fn settle_capture_reports_deadline_overflow_for_the_analysis_budget() {
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
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    data.previous_encoder_raw = 0;
    data.previous_read_us = u64::MAX - 1_000;
    *state = VelocityIntegralState::SettleCapture {
        next_sample_us: 0,
        deadline_us: 0,
    };
    let mut backend = MockBackend::new();
    backend.elapsed_us_value = u64::MAX;

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("analysis-budget overflow must emit the Integral integrity terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::ScheduleDeadlineOverflow.status_code())
    );
}

#[test]
fn excessive_slot_transition_time_emits_an_observation_gap_fault() {
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
    *state = final_zero_ramp_tick(data, 19_601);
    let mut backend = MockBackend::new();
    backend.spi_elapsed_step_us = 100;

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("transition overrun must emit the Integral safety terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal
            .safety_fault
            .expect("observation-gap detail is required")
            .reason,
        crate::commissioning::types::OUTER_SAFETY_FAULT_OBSERVATION_GAP
    );
}

#[test]
fn opening_anchor_uses_the_declared_operational_clamp() {
    let mut phase = phase();
    let mut workspace = TransientBuffer::new();
    let OuterPhase::VelocityIntegral { data, .. } = &mut phase else {
        unreachable!()
    };
    data.plan.usable_torque_limit = data.plan.hard_torque_limit;
    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(2_816);

    for _ in 0..2 {
        assert!(matches!(
            step(&mut phase, &mut workspace, &mut backend),
            StepResult::ReadyNow
        ));
    }
    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::At(1_000)
    ));
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_TORQUE_FLUX_LIMITS.addr() && *value == 2_816)
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
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Engine(INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE)
    );
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
        panic!("contained final recovery must preserve the Integral terminal");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::FirstRunRetained);
    assert_eq!(terminal.cause, IntegralOutcomeCause::Engine(0));
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
fn integral_safety_fault_uses_the_dedicated_terminal() {
    let phase = phase();
    let StepResult::VelocityIntegralTerminal(terminal) =
        integral_fault_terminal(&phase, CommissionError::SafetyEnvelopeViolation, None)
    else {
        panic!("Integral fault must not fall through to the generic terminal")
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::SafetyEnvelopeViolation.status_code())
    );
    assert_eq!(terminal.expected_observations, 56);
    assert_eq!(terminal.expected_rungs, 7);
}

#[test]
fn rung_aggregation_follows_the_declared_slot_order() {
    let slot_order = IntegralSlotOrder::ForwardFirst;
    let mut plan = plan();
    plan.slot_order = slot_order;
    let mut data = VelocityIntegralRuntime::new(1, plan);
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = core::array::from_fn(|slot| {
        let travels_forward = slot_order.direction_index(slot as u8) == 0;
        if travels_forward {
            ExactRateInterval::new(100, 110).unwrap()
        } else {
            ExactRateInterval::new(-110, -100).unwrap()
        }
    });

    let forward = analyze_rung(&data, VelocityDirection::Forward).unwrap();
    let reverse = analyze_rung(&data, VelocityDirection::Reverse).unwrap();

    assert!(
        forward.interval.unwrap().low_q > 0,
        "{slot_order:?} forward read a reverse slot"
    );
    assert!(
        reverse.interval.unwrap().high_q < 0,
        "{slot_order:?} reverse read a forward slot"
    );
}

#[test]
fn unconfirmed_recovery_rest_ends_the_integral_run() {
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

    let start_offset_counts = 1_500_i32;
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    safety.configure_velocity_envelope(4_394).unwrap();
    let raw = start_offset_counts.rem_euclid(4_000) as u32;
    safety.set_encoder_baseline(raw).unwrap();
    safety.unwrapped_position = start_offset_counts;
    safety.arm_safety_observation(0).unwrap();

    let recovery_plan =
        plan_rung_origin_recovery(safety.budget, safety.counts_per_rev(), 4_394).unwrap();
    let clamp_limit = PidTorqueFluxLimit::new(u32::from(data.plan.usable_torque_limit)).unwrap();
    data.current_observer
        .reset(clamp_limit, data.plan.limit_evidence_capability);
    let runtime = RungOriginRecoveryRuntime::new(
        RungOriginRecoveryConfig {
            stage: RecoveryStage::Integral,
            rung_index: data.rung_index,
            p: PidPGain::new(data.plan.fixed_p).unwrap(),
            target_velocity_rpm: TmcMechanicalRpm::from_register(264),
            planned_velocity_mrev_s: 4_394,
            start_offset_counts,
            lower_rate_q: Some(14_000_000),
            authority_source: RecoveryBindingSource::SettledRate,
            current_contact_policy: integral_recovery_current_contact_policy(data),
        },
        recovery_plan,
    )
    .unwrap();
    *state = VelocityIntegralState::RecoverRung(runtime);

    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(u32::from(data.plan.usable_torque_limit));
    backend.mechanical_encoder_position_counts_value = raw;

    let terminal = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            StepResult::At(deadline_us) => {
                backend.elapsed_us_value = deadline_us;
                let drift = i32::try_from(deadline_us / 1_000).unwrap_or(i32::MAX);
                let offset = start_offset_counts.saturating_sub(drift);
                backend.mechanical_encoder_position_counts_value = offset.rem_euclid(4_000) as u32;
            }
            StepResult::VelocityIntegralEvent(_) => {}
            StepResult::VelocityIntegralTerminal(terminal) => break terminal,
            other => panic!("unexpected recovery rest result: {other:?}"),
        }
    };
    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert!(data.rest_unconfirmed);
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code())
    );
}

#[test]
fn positive_observation_unconfirmed_rest_reports_its_diagnostic_owner() {
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
    data.settle_min_displacement_counts = 0;
    data.settle_max_displacement_counts = 2;
    for _ in 0..256 {
        data.observations.push(1, 1_000).unwrap();
    }
    reset_scored_rest(data);
    *state = VelocityIntegralState::AnalyzeSettle {
        analysis_deadline_us: 236_000,
    };
    let mut backend = MockBackend::new();

    let diagnostic = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(event)) => {
                break event;
            }
            StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Current(_)) => {}
            other => panic!("unexpected positive-observation rest result: {other:?}"),
        }
    };

    assert_eq!(
        diagnostic.owner,
        RestSelectionOwner::IntegralPositiveObservation
    );
}

#[test]
fn integral_i_at_response_ladder_returns_the_half_octave_rung() {
    let mut test_plan = plan();
    test_plan.i_top_cap = 1_310;
    test_plan.positive_rung_count = 13;
    for (positive_index, expected) in [1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 724, 1_024]
        .iter()
        .enumerate()
    {
        assert_eq!(
            integral_i_at(test_plan, positive_index as u8 + 1),
            Some(*expected)
        );
    }
}

#[test]
fn integral_i_at_breakaway_response_ladder_never_returns_724() {
    let mut test_plan = plan();
    test_plan.i_top_cap = 0;
    test_plan.positive_rung_count = 12;
    let total = integral_rung_count(test_plan).unwrap();
    for index in 0..total {
        assert_ne!(integral_i_at(test_plan, index), Some(724));
    }
}

fn eligible_evidence(intervals: [Option<(i32, i32)>; 4], usable_mask: u8) -> IntegralRungEvidence {
    IntegralRungEvidence {
        direction: VelocityDirection::Forward,
        kind: IntegralRungKind::Positive,
        classification: IntegralRungClass::EligibleConsensus,
        interval: None,
        components: [ExactRateInterval::EMPTY; 2],
        component_count: 0,
        observation_classes: [IntegralObservationClass::Usable; 4],
        observation_intervals: intervals
            .map(|interval| interval.map(|(low, high)| ExactRateInterval::new(low, high).unwrap())),
        usable_mask,
        included_mask: usable_mask,
    }
}

#[test]
fn pooled_interval_uses_floor_and_ceil_of_the_exact_mean_with_zero_width() {
    let evidence = eligible_evidence([Some((3, 3)), Some((4, 4)), Some((6, 6)), None], 0b0111);
    let pooled = pooled_deficit_interval(&evidence, 56, 12).unwrap().unwrap();
    assert_eq!((pooled.low_q, pooled.high_q), (4, 5));
}

#[test]
fn pooled_interval_floors_toward_negative_infinity_for_a_negative_mean() {
    let evidence = eligible_evidence(
        [Some((-3, -3)), Some((-4, -4)), Some((-6, -6)), None],
        0b0111,
    );
    let pooled = pooled_deficit_interval(&evidence, 56, 12).unwrap().unwrap();
    assert_eq!((pooled.low_q, pooled.high_q), (-5, -4));
}

#[test]
fn pooled_half_width_follows_the_two_k_multiplier() {
    let evidence = eligible_evidence([Some((-500, 500)), None, None, None], 0b0001);
    let m_family = confidence_multiplier_95_df15(56).unwrap();
    let expected = |k: u8| -> i32 {
        let m_2k = confidence_multiplier_95_df15(2 * u16::from(k)).unwrap();
        libm::ceilf((m_2k * libm::sqrtf(1_000_000.0)) / (2.0 * m_family)) as i32
    };
    let twelve = pooled_deficit_interval(&evidence, 56, 12).unwrap().unwrap();
    let six = pooled_deficit_interval(&evidence, 56, 6).unwrap().unwrap();
    assert_eq!(twelve.high_q, expected(12));
    assert_eq!(six.high_q, expected(6));
    assert_eq!((twelve.high_q, six.high_q), (450, 409));
}

#[test]
fn pooled_interval_pools_the_usable_set_not_the_included_set() {
    let base = [
        Some((0, 100)),
        Some((0, 100)),
        Some((0, 100)),
        Some((400, 500)),
    ];
    let mut four = eligible_evidence(base, 0b1111);
    four.included_mask = 0b0111;
    let mut three = eligible_evidence(base, 0b0111);
    three.observation_intervals[3] = Some(ExactRateInterval::new(400, 500).unwrap());
    let only_three = eligible_evidence(
        [Some((0, 100)), Some((0, 100)), Some((0, 100)), None],
        0b0111,
    );
    let four = pooled_deficit_interval(&four, 56, 12).unwrap().unwrap();
    let three = pooled_deficit_interval(&three, 56, 12).unwrap().unwrap();
    let only_three = pooled_deficit_interval(&only_three, 56, 12)
        .unwrap()
        .unwrap();
    assert_ne!(four, three);
    assert_eq!(three, only_three);
}

#[test]
fn pooled_interval_is_absent_without_eligible_consensus() {
    let mut evidence = eligible_evidence([Some((0, 1)); 4], 0b1111);
    evidence.classification = IntegralRungClass::AmbiguousConsensus;
    assert_eq!(pooled_deficit_interval(&evidence, 56, 12).unwrap(), None);
}

fn regression_plan() -> VelocityIntegralExecutionPlan {
    let mut plan = plan();
    plan.target_velocity_rpm = 264;
    plan.encoder_counts_per_rev = 4_000;
    plan.positive_rung_count = 12;
    plan.family_size = 56;
    plan
}

fn regression_pair_evidence() -> [IntegralRungEvidence; 2] {
    let forward = [
        Some((-369_600, 404_800)),
        Some((-404_800, 404_800)),
        Some((-440_000, 404_800)),
        Some((-492_800, 422_400)),
    ];
    let reverse = forward.map(|interval| interval.map(|(low, high)| (-high, -low)));
    let mut fwd = eligible_evidence(forward, 0b1111);
    fwd.interval = Some(ExactRateInterval::new(-369_600, 404_800).unwrap());
    let mut rev = eligible_evidence(reverse, 0b1111);
    rev.direction = VelocityDirection::Reverse;
    rev.interval = Some(ExactRateInterval::new(-404_800, 369_600).unwrap());
    [fwd, rev]
}

#[test]
fn pooled_decision_passes_where_the_consensus_component_fails() {
    let plan = regression_plan();
    let [fwd, rev] = regression_pair_evidence();

    let mut legacy = VelocityIntegralRuntime::new(1, plan);
    legacy.rung_index = 10;
    assert!(
        !record_integral_target_if_reached(
            &mut legacy,
            [fwd.interval.unwrap(), rev.interval.unwrap()]
        )
        .unwrap()
    );
    assert_eq!(legacy.target_terminus, None);

    let mut pooled = VelocityIntegralRuntime::new(1, plan);
    pooled.rung_index = 10;
    let intervals = [
        pooled_deficit_interval(&fwd, 56, 12).unwrap().unwrap(),
        pooled_deficit_interval(&rev, 56, 12).unwrap().unwrap(),
    ];
    assert!(record_integral_target_if_reached(&mut pooled, intervals).unwrap());
    assert_eq!(pooled.target_terminus, Some(9));
}

#[test]
fn pooled_decision_still_rejects_a_mean_below_the_target() {
    let forward = [
        Some((52_800, 827_200)),
        Some((-52_800, 756_800)),
        Some((-105_600, 739_200)),
        Some((-176_000, 739_200)),
    ];
    let reverse = forward.map(|interval| interval.map(|(low, high)| (-high, -low)));
    let fwd = eligible_evidence(forward, 0b1111);
    let mut rev = eligible_evidence(reverse, 0b1111);
    rev.direction = VelocityDirection::Reverse;
    let plan = regression_plan();

    let mut legacy = VelocityIntegralRuntime::new(1, plan);
    legacy.rung_index = 10;
    let components = [
        ExactRateInterval::new(52_800, 739_200).unwrap(),
        ExactRateInterval::new(-739_200, -52_800).unwrap(),
    ];
    assert!(!record_integral_target_if_reached(&mut legacy, components).unwrap());
    assert_eq!(legacy.target_terminus, None);

    let mut data = VelocityIntegralRuntime::new(1, plan);
    data.rung_index = 10;
    let intervals = [
        pooled_deficit_interval(&fwd, 56, 12).unwrap().unwrap(),
        pooled_deficit_interval(&rev, 56, 12).unwrap().unwrap(),
    ];
    assert!(!record_integral_target_if_reached(&mut data, intervals).unwrap());
    assert_eq!(data.target_terminus, None);
}

#[test]
fn live_retain_rung_records_the_terminus_from_the_pooled_interval() {
    let mut data = VelocityIntegralRuntime::new(1, regression_plan());
    data.rung_index = 10;
    let mut workspace = TransientBuffer::new();

    retain_rung(&mut data, &mut workspace, regression_pair_evidence()).unwrap();

    assert_eq!(data.target_terminus, Some(9));
    assert_eq!(data.eligible_masks, [1 << 9, 1 << 9]);
}

#[test]
fn diverted_path_records_the_same_terminus_from_retained_rung_intervals() {
    let mut data = VelocityIntegralRuntime::new(1, regression_plan());
    data.rung_index = 10;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    for (direction, evidence) in regression_pair_evidence().iter().enumerate() {
        for (index, interval) in evidence.observation_intervals.iter().enumerate() {
            let slot = usize::from(data.plan.slot_order.slot_for(index, direction));
            data.rung_intervals[slot] = interval.unwrap();
        }
    }

    record_diverted_target_terminus(&mut data).unwrap();

    assert_eq!(data.target_terminus, Some(9));
}

#[test]
fn mixed_classification_rung_takes_no_decision() {
    let [fwd, mut rev] = regression_pair_evidence();
    rev.classification = IntegralRungClass::AmbiguousConsensus;
    rev.interval = None;
    let mut data = VelocityIntegralRuntime::new(1, regression_plan());
    data.rung_index = 10;
    let mut workspace = TransientBuffer::new();

    retain_rung(&mut data, &mut workspace, [fwd, rev]).unwrap();

    assert_eq!(data.target_terminus, None);
    assert_eq!(data.eligible_masks, [1 << 9, 0]);
}

#[test]
fn anchor_and_bookend_rungs_never_pool() {
    let plan = regression_plan();
    let mut workspace = TransientBuffer::new();

    let mut anchor = VelocityIntegralRuntime::new(1, plan);
    anchor.rung_index = 0;
    let evidence = regression_pair_evidence().map(|mut evidence| {
        evidence.kind = IntegralRungKind::OpeningAnchor;
        evidence
    });
    retain_rung(&mut anchor, &mut workspace, evidence).unwrap();
    assert_eq!(anchor.target_terminus, None);
    assert_eq!(anchor.opening_direction_mask, 3);

    let mut bookend = VelocityIntegralRuntime::new(1, plan);
    bookend.rung_index = plan.positive_rung_count + 1;
    let evidence = regression_pair_evidence().map(|mut evidence| {
        evidence.kind = IntegralRungKind::ClosingBookend;
        evidence
    });
    retain_rung(&mut bookend, &mut workspace, evidence).unwrap();
    assert_eq!(bookend.target_terminus, None);
    assert_eq!(bookend.bookend_available_mask, 3);
}

struct IntegralStroke {
    phase: OuterPhase,
    workspace: TransientBuffer,
    backend: MockBackend,
    position_counts: f64,
}

impl IntegralStroke {
    /// Runs Setup for `plan` and leaves the sweep at the first moving slot of `rung_index`.
    fn at_rung(plan: VelocityIntegralExecutionPlan, rung_index: u8) -> Self {
        let mut stroke = Self {
            phase: phase(),
            workspace: TransientBuffer::new(),
            backend: MockBackend::new(),
            position_counts: 0.0,
        };
        let OuterPhase::VelocityIntegral { data, .. } = &mut stroke.phase else {
            unreachable!()
        };
        *data = VelocityIntegralRuntime::new(7, plan);
        stroke.backend.current_limit_value = current_limit(u32::from(plan.hard_torque_limit));
        assert!(matches!(
            step(
                &mut stroke.phase,
                &mut stroke.workspace,
                &mut stroke.backend
            ),
            StepResult::ReadyNow
        ));
        stroke.data_mut().rung_index = rung_index;
        stroke
    }

    fn state(&self) -> &VelocityIntegralState {
        let OuterPhase::VelocityIntegral { state, .. } = &self.phase else {
            panic!("the integral phase ended: {:?}", self.phase)
        };
        state
    }

    fn data(&self) -> &VelocityIntegralRuntime {
        let OuterPhase::VelocityIntegral { data, .. } = &self.phase else {
            panic!("the integral phase ended")
        };
        data
    }

    fn data_mut(&mut self) -> &mut VelocityIntegralRuntime {
        let OuterPhase::VelocityIntegral { data, .. } = &mut self.phase else {
            panic!("the integral phase ended")
        };
        data
    }

    fn now_us(&self) -> u64 {
        self.backend.elapsed_us_value
    }

    fn written_target(&self) -> i32 {
        self.backend
            .registers
            .get(&PID_VELOCITY_TARGET.addr())
            .map_or(0, |value| *value as i32)
    }

    fn target_writes(&self) -> std::vec::Vec<i32> {
        self.backend
            .spi_log
            .iter()
            .filter_map(|operation| match operation {
                SpiOp::Write(address, value) if *address == PID_VELOCITY_TARGET.addr() => {
                    Some(*value as i32)
                }
                _ => None,
            })
            .collect()
    }

    /// Steps once, through the outer dispatcher when `dispatched`; on `At` the plant moves at
    /// `counts_per_ms(state, written target)` until the deadline.
    fn step(
        &mut self,
        dispatched: bool,
        mut counts_per_ms: impl FnMut(&VelocityIntegralState, i32) -> f64,
    ) -> StepResult {
        let result = if dispatched {
            let mut trace_summary_seqs = crate::commissioning::outer::TraceSummarySeqs::new();
            pollster::block_on(crate::commissioning::outer::step_outer_with_trace(
                &mut self.phase,
                &mut self.workspace,
                &mut trace_summary_seqs,
                &mut self.backend,
            ))
        } else {
            step(&mut self.phase, &mut self.workspace, &mut self.backend)
        };
        if let StepResult::At(deadline_us) = result {
            let rate = match &self.phase {
                OuterPhase::VelocityIntegral { state, .. } => {
                    counts_per_ms(state, self.written_target())
                }
                _ => 0.0,
            };
            let elapsed_ms = deadline_us.saturating_sub(self.now_us()) as f64 / 1_000.0;
            self.position_counts += rate * elapsed_ms;
            self.backend.elapsed_us_value = self.backend.elapsed_us_value.max(deadline_us);
            let position = libm::floor(self.position_counts) as i32;
            self.backend.mechanical_encoder_position_counts_value = position as u32;
            self.backend.set_abn_decoder_count(position);
        }
        result
    }
}

fn tracking_plant(_state: &VelocityIntegralState, written_rpm: i32) -> f64 {
    f64::from(written_rpm) * 4_000.0 / 60_000.0
}

#[test]
fn integral_response_fraction_is_unchanged_by_the_ramp_on_a_settled_plant() {
    let mut stroke = IntegralStroke::at_rung(plan(), 1);
    let pattern = [10.0, 11.0, 10.0, 11.0, 11.0, 10.0];
    let mut capture_ticks = 0_usize;

    let summary = loop {
        let result = stroke.step(false, |state, written| match state {
            VelocityIntegralState::Capture { .. } => {
                capture_ticks += 1;
                pattern[capture_ticks % pattern.len()]
            }
            _ => 0.5 * tracking_plant(state, written),
        });
        if let StepResult::VelocityIntegralEvent(VelocityIntegralEvent::Observation(report)) =
            result
        {
            break report.summary;
        }
        assert!(
            !matches!(result, StepResult::VelocityIntegralTerminal(_)),
            "the stroke ended before its observation: {result:?}"
        );
    };

    assert_eq!(summary.classification, VelocityObservationClass::Eligible);
    assert_eq!(summary.settled_rate_mean_q, 10_496_093);
    assert_eq!(summary.settled_rate_interval_q, [10_445_549, 10_546_637]);
    assert_eq!(summary.deficit_interval_q, [1_186_696, 1_287_785]);
    assert_eq!(summary.drift_ratio_ppm, 657);
}

fn ramp_plan() -> VelocityIntegralExecutionPlan {
    let mut plan = plan();
    plan.target_velocity_rpm = 274;
    plan.planned_velocity_mrev_s = 4_567;
    plan.ramp_milli_rpm_per_ms = 60_000;
    plan.ramp_us = 5_000;
    plan
}

fn set_pid_output(backend: &mut MockBackend, value: u32) {
    let _ = backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        value,
    );
}

#[test]
fn integral_stroke_ramps_up_and_down_and_capture_starts_at_the_ramp_end() {
    let mut stroke = IntegralStroke::at_rung(ramp_plan(), 1);
    let mut capture_tick_us = None;
    let mut settle_tick_us = None;

    while !matches!(stroke.state(), VelocityIntegralState::AnalyzeMoving { .. }) {
        let tick_us = stroke.now_us();
        stroke.step(false, tracking_plant);
        match stroke.state() {
            VelocityIntegralState::Capture {
                next_sample_us,
                deadline_us,
                ..
            } if capture_tick_us.is_none() => {
                capture_tick_us = Some(tick_us);
                assert_eq!(stroke.target_writes().last(), Some(&274));
                assert_eq!(*next_sample_us, tick_us + 1_000);
                assert_eq!(*deadline_us, tick_us + 512_000);
                assert_eq!(stroke.data().observations.len(), 0);
            }
            VelocityIntegralState::AnalyzeMoving {
                settle_deadline_us, ..
            } => {
                settle_tick_us = Some(tick_us);
                assert_eq!(*settle_deadline_us, tick_us + 500_000);
            }
            _ => {}
        }
    }

    let targets = stroke.target_writes();
    assert_eq!(targets, [60, 120, 180, 240, 274, 214, 154, 94, 34, 0]);
    let capture_tick_us = capture_tick_us.expect("the stroke reached its capture");
    assert_eq!(capture_tick_us, 4_000);
    assert_eq!(settle_tick_us, Some(capture_tick_us + 512_000 + 4_000));
    #[cfg(feature = "trace")]
    assert_eq!(stroke.backend.velocity_trace_observations.len(), 512);
}

#[test]
fn a_zero_ramp_rate_fails_the_integral_stroke_before_any_target_write() {
    let mut plan = ramp_plan();
    plan.ramp_milli_rpm_per_ms = 0;
    let mut stroke = IntegralStroke::at_rung(plan, 1);

    let StepResult::VelocityIntegralTerminal(terminal) = stroke.step(false, tracking_plant) else {
        panic!("a zero ramp rate must end the integral run");
    };

    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Error(CommissionError::CampaignPlanInvalid.status_code())
    );
    assert!(stroke.target_writes().iter().all(|target| *target == 0));
}

#[test]
fn integral_ramp_phase_pid_output_is_not_scored_into_the_moving_peak() {
    let mut stroke = IntegralStroke::at_rung(ramp_plan(), 1);
    set_pid_output(&mut stroke.backend, 900);
    while !matches!(stroke.state(), VelocityIntegralState::Capture { .. }) {
        stroke.step(false, tracking_plant);
    }
    set_pid_output(&mut stroke.backend, 100);

    while !matches!(stroke.state(), VelocityIntegralState::RampZero { .. }) {
        stroke.step(false, tracking_plant);
    }

    assert_eq!(stroke.data().current_observer.moving_pid_output_peak(), 100);
}

#[test]
fn integral_cleanup_zero_write_is_immediate() {
    let mut stroke = IntegralStroke::at_rung(ramp_plan(), 1);
    let mut jumped = false;

    let terminal = loop {
        let result = stroke.step(false, |state, written| {
            if written == 120 && !jumped {
                jumped = true;
                1_200.0
            } else {
                tracking_plant(state, written)
            }
        });
        if let StepResult::VelocityIntegralTerminal(terminal) = result {
            break terminal;
        }
    };

    assert_eq!(stroke.target_writes(), [60, 120, 0]);
    assert_eq!(
        terminal.safety_fault.map(|fault| fault.reason),
        Some(crate::commissioning::types::OUTER_SAFETY_FAULT_QUARTER_TURN)
    );
}

#[test]
fn integral_actionable_status_at_the_moving_boundary_zeroes_the_target_at_once() {
    let mut stroke = IntegralStroke::at_rung(ramp_plan(), 1);
    while !matches!(stroke.state(), VelocityIntegralState::RampZero { .. }) {
        stroke.step(false, tracking_plant);
    }
    let _ = stroke.backend.registers.insert(
        foci_tmc4671::registers::STATUS_FLAGS.addr(),
        crate::autotune::current_loop_actionable_status_mask(stroke.backend.feedback_config),
    );

    let result = stroke.step(false, tracking_plant);

    assert!(matches!(result, StepResult::VelocityIntegralTerminal(_)));
    assert_eq!(stroke.target_writes(), [60, 120, 180, 240, 274, 0]);
}

fn run_dispatched_fault(trigger_write: i32) -> (IntegralStroke, VelocityIntegralTerminal) {
    let mut stroke = IntegralStroke::at_rung(ramp_plan(), 1);
    let mut jumped = false;

    for _ in 0..10_000 {
        let result = stroke.step(true, |state, written| {
            if written == trigger_write && !jumped {
                jumped = true;
                900.0
            } else {
                tracking_plant(state, written)
            }
        });
        if let StepResult::VelocityIntegralTerminal(terminal) = result {
            return (stroke, terminal);
        }
    }
    panic!("the dispatched integral stroke did not fault");
}

#[test]
fn an_encoder_velocity_fault_during_the_integral_ramp_is_caught_by_the_safety_sample() {
    let (stroke, terminal) = run_dispatched_fault(120);

    assert_eq!(stroke.target_writes(), [60, 120, 0]);
    assert_eq!(
        terminal.safety_fault.map(|fault| fault.reason),
        Some(crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY)
    );
}

#[test]
fn an_encoder_velocity_fault_during_the_integral_zero_ramp_is_caught_by_the_safety_sample() {
    let (stroke, terminal) = run_dispatched_fault(154);

    assert_eq!(
        stroke.target_writes(),
        [60, 120, 180, 240, 274, 214, 154, 0]
    );
    assert_eq!(
        terminal.safety_fault.map(|fault| fault.reason),
        Some(crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY)
    );
}

#[test]
fn a_failure_after_a_moving_sample_reports_the_sample_read_time_as_run_completion() {
    let selector_addr = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
        .addr
        .addr();
    let mut stroke = IntegralStroke::at_rung(ramp_plan(), 1);
    let mut capture_steps = 0_u32;

    let terminal = loop {
        if matches!(stroke.state(), VelocityIntegralState::Capture { .. }) {
            capture_steps += 1;
            if capture_steps > 3 {
                let selector_writes = stroke
                    .backend
                    .spi_log
                    .iter()
                    .filter(|op| matches!(op, SpiOp::Write(addr, _) if *addr == selector_addr))
                    .count();
                stroke.backend.fail_on_write_occurrence =
                    Some((selector_addr, selector_writes + 1));
            }
        }
        if let StepResult::VelocityIntegralTerminal(terminal) = stroke.step(false, tracking_plant) {
            break terminal;
        }
    };

    let selector_values: std::vec::Vec<u32> = stroke
        .backend
        .spi_log
        .iter()
        .filter_map(|op| match op {
            SpiOp::Write(addr, value) if *addr == selector_addr => Some(*value),
            _ => None,
        })
        .collect();
    let [.., input_select, output_select] = selector_values[..] else {
        panic!("the sample selects the input then the output register");
    };
    assert_eq!(
        (input_select, output_select),
        (
            u32::from(foci_tmc4671::registers::INTERIM_PIDIN_TARGET_TORQUE.sub_addr),
            u32::from(foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE.sub_addr)
        ),
        "the failed write must be the output selector, after the input read succeeded"
    );
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert!(terminal.run_completed_us > terminal.run_started_us);
    assert_eq!(terminal.run_completed_us, stroke.now_us());
}
