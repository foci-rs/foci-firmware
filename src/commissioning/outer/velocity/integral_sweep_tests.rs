use super::integral_plan::IntegralSlotOrder;
use super::integral_plan::{
    AcceptanceSource, INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US, VelocityIntegralExecutionPlan,
};
use super::integral_sweep::{
    INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE, INTEGRAL_CAUSE_FIXED_GAIN_REST_OWNERSHIP,
    INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY,
    INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY_AFTER_SUFFICIENCY, INTEGRAL_SLOT_COUNT,
    advance_integral_cursor, amplitude_rung_mean, analyze_rung, apply_integral_current_headroom,
    combined_slot_transition_us, fixed_gain_amplitude_report, fixed_gain_amplitude_terminal,
    integral_fault_terminal, integral_i_at, integral_recovery_current_contact_policy,
    integral_rung_count, integral_rung_kind, integral_slot_direction, integral_target_rpm_at,
    is_fixed_gain_amplitude, opening_recovery_lower_rate_q, plan_integral_recovery,
    positive_directional_floors_exist, record_integral_observation_rest_selection,
    record_integral_recovery_rest, record_integral_recovery_rest_selection,
    record_integral_target_if_reached, step_velocity_integral_with_trace,
    store_amplitude_rung_mean,
};
use super::recovery::{
    ExactU64Parts, RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryRestAnalysis,
    RecoveryStabilizationSummary, RecoveryStage, RungOriginRecoveryConfig,
    RungOriginRecoveryOutcome, RungOriginRecoveryReport, RungOriginRecoveryRuntime,
    plan_rung_origin_recovery,
};
use super::sweep_plan::{RequestedVelocitySource, VELOCITY_SWEEP_SAMPLE_PERIOD_US};
use super::velocity_clamp::{VelocityCurrentObserver, VelocityLimitEvidenceCapability};
use crate::autotune::CommissionError;
use crate::autotune::TmcMechanicalRpm;
use crate::autotune::TransientBuffer;
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::velocity_measurement::{
    RestSelectionDecision, RestSelectionDisposition, RestStructuralCode, StationarityExclusion,
    StationarityResult,
};
use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
use crate::commissioning::outer::state::{
    PositiveIRestBoundary, VelocitySettleMetrics, VelocitySettleTransition,
};
use crate::commissioning::outer::velocity::fixed_gain_amplitude_evidence::FixedGainAmplitudeAvailability;
use crate::commissioning::outer::velocity::integral_evidence::{
    ExactRateInterval, IntegralRungKind,
};
use crate::commissioning::outer::velocity::integral_plan::VelocityIntegralAcquisitionKind;
use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, VelocityIntegralRuntime,
    VelocityIntegralState,
};
use crate::commissioning::types::{
    FixedGainAmplitudeCause, FixedGainAmplitudeEvent, FixedGainAmplitudeOutcome,
    IntegralOutcomeCause, RestSelectionOwner, RungOriginRecoveryEvidence, StepResult,
    VelocityIntegralEvent, VelocityIntegralOutcome,
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
        acceptance_plan_digest: 0,
        qualifier_bits: 0,
        fixed_p: 1_448,
        planned_velocity_mrev_s: 2_929,
        target_velocity_rpm: 176,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
        i_start: 1,
        positive_rung_count: 5,
        i_top_cap: 0,
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
        acquisition_kind: super::integral_plan::VelocityIntegralAcquisitionKind::ResponseLadder,
        acceptance_source: super::integral_plan::AcceptanceSource::NonBreakaway,
        amplitude_targets_rpm: [0; 5],
        selected_response_intervals_q: [[0; 2]; 2],
        recovery_lower_rate_q: [13_217_792, 12_827_648],
        recovery_quantization_exposed: true,
        probe_constrained_test_point: false,
    }
}

#[test]
fn breakaway_target_terminus_stays_sticky_once_recorded() {
    let mut plan = plan();
    plan.acceptance_source = AcceptanceSource::Breakaway;
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
    plan.acceptance_source = AcceptanceSource::Breakaway;
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
    plan.acceptance_source = AcceptanceSource::Breakaway;
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
fn non_breakaway_ladder_does_not_record_a_terminus() {
    let mut plan = plan();
    assert_eq!(plan.acceptance_source, AcceptanceSource::NonBreakaway);
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

    assert!(!reached);
    assert_eq!(data.target_terminus, None);
}

#[test]
fn target_terminus_recorder_reports_target_rate_invalid_for_a_non_positive_target() {
    let mut plan = plan();
    plan.acceptance_source = AcceptanceSource::Breakaway;
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

#[test]
fn response_and_amplitude_plans_expose_distinct_exact_schedules() {
    let response = plan();
    assert_eq!(integral_rung_count(response), Some(7));
    assert_eq!(integral_target_rpm_at(response, 0), Some(176));
    assert_eq!(
        core::array::from_fn::<_, 7, _>(|rung| { integral_rung_kind(response, rung as u8) }),
        [
            IntegralRungKind::OpeningAnchor,
            IntegralRungKind::Positive,
            IntegralRungKind::Positive,
            IntegralRungKind::Positive,
            IntegralRungKind::Positive,
            IntegralRungKind::Positive,
            IntegralRungKind::ClosingBookend,
        ]
    );

    let mut ascending = plan();
    ascending.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    ascending.i_start = 1_024;
    ascending.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    assert_eq!(integral_rung_count(ascending), Some(5));
    assert_eq!(
        core::array::from_fn::<_, 5, _>(|rung| integral_i_at(ascending, rung as u8).unwrap()),
        [1_024; 5]
    );
    assert_eq!(
        core::array::from_fn::<_, 5, _>(|rung| {
            integral_target_rpm_at(ascending, rung as u8).unwrap()
        }),
        [16, 33, 66, 132, 263]
    );
    assert_eq!(
        core::array::from_fn::<_, 5, _>(|rung| { integral_rung_kind(ascending, rung as u8) }),
        [IntegralRungKind::FixedGainAmplitude; 5]
    );

    let mut descending = ascending;
    descending.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending;
    descending.amplitude_targets_rpm = [263, 132, 66, 33, 16];
    assert_eq!(
        core::array::from_fn::<_, 5, _>(|rung| {
            integral_target_rpm_at(descending, rung as u8).unwrap()
        }),
        [263, 132, 66, 33, 16]
    );
    assert_eq!(integral_i_at(descending, 5), None);
    assert_eq!(integral_target_rpm_at(descending, 5), None);
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
fn live_amplitude_schedule_records_one_selector_per_completed_rest_decision() {
    let mut amplitude_plan = plan();
    amplitude_plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    amplitude_plan.i_start = 1_024;
    amplitude_plan.positive_rung_count = 5;
    amplitude_plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    let mut data = VelocityIntegralRuntime::new(41, amplitude_plan);
    let decision = completed_rest_selection();

    for amplitude_index in 0..5 {
        for slot in 0..INTEGRAL_SLOT_COUNT {
            data.rung_index = amplitude_index;
            data.slot = slot;
            let StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::RestSelection(report)) =
                record_integral_observation_rest_selection(
                    &mut data,
                    decision,
                    RestSelectionOwner::FixedGainAmplitudeObservation,
                )
            else {
                panic!("every amplitude observation rest decision must emit Event 151");
            };
            let expected_sequence =
                1 + u16::from(amplitude_index) * u16::from(INTEGRAL_SLOT_COUNT) + u16::from(slot);
            assert_eq!(report.evidence_sequence, expected_sequence);
            assert_eq!(
                report.owner,
                RestSelectionOwner::FixedGainAmplitudeObservation
            );
            assert_eq!(report.rung_index, amplitude_index);
            assert_eq!(report.direction, slot % 2);
            assert_eq!(report.slot, slot);
        }
    }
    assert_eq!(data.integrity.next_evidence_sequence(), 41);

    for amplitude_index in 0..5 {
        let StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::RestSelection(report)) =
            record_integral_recovery_rest_selection(
                &mut data,
                decision,
                RecoveryStage::Integral,
                amplitude_index,
            )
        else {
            panic!("every completed amplitude recovery rest decision must emit Event 151");
        };
        assert_eq!(report.evidence_sequence, 41 + u16::from(amplitude_index));
        assert_eq!(report.owner, RestSelectionOwner::FixedGainAmplitudeRecovery);
        assert_eq!(report.rung_index, amplitude_index);
        assert_eq!([report.direction, report.slot], [u8::MAX; 2]);
    }
    assert_eq!(data.integrity.next_evidence_sequence(), 46);
}

#[test]
fn integral_and_amplitude_place_rest_diagnostics_immediately_before_recovery() {
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

    let mut amplitude_plan = plan();
    amplitude_plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    amplitude_plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    let mut amplitude = VelocityIntegralRuntime::new(23, amplitude_plan);
    let StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::RestSelection(selection)) =
        record_integral_recovery_rest_selection(
            &mut amplitude,
            completed_rest_selection(),
            RecoveryStage::Integral,
            4,
        )
    else {
        panic!("amplitude must emit its selector diagnostic")
    };
    let StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::RecoveryRest(diagnostic)) =
        record_integral_recovery_rest(&mut amplitude, completed_recovery_rest_analysis())
    else {
        panic!("amplitude must emit its recovery-rest diagnostic")
    };
    assert_eq!(diagnostic.run_sequence, 23);
    assert_eq!(selection.evidence_sequence, 1);
    assert_eq!(diagnostic.evidence_sequence, 2);
    assert_eq!(selection.plan_digest, diagnostic.plan_digest);
    assert_eq!(selection.stage as u8, diagnostic.stage as u8);
    assert_eq!(selection.rung_index, diagnostic.rung_index);
    let recovery = RungOriginRecoveryEvidence {
        run_sequence: 23,
        evidence_sequence: amplitude.integrity.next_evidence_sequence(),
        report: completed_recovery_report(),
    };
    let FixedGainAmplitudeEvent::Recovery(recovery) = amplitude
        .integrity
        .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::Recovery(recovery))
    else {
        unreachable!()
    };
    assert_eq!(recovery.evidence_sequence, diagnostic.evidence_sequence + 1);
}

#[test]
fn amplitude_fold_retains_exact_consensus_location_and_masks() {
    let mut amplitude = plan();
    amplitude.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    amplitude.i_start = 1_024;
    amplitude.positive_rung_count = 5;
    amplitude.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    let mut data = VelocityIntegralRuntime::new(1, amplitude);
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = [
        ExactRateInterval::new(100, 110).unwrap(),
        ExactRateInterval::new(-110, -100).unwrap(),
        ExactRateInterval::new(102, 112).unwrap(),
        ExactRateInterval::new(-112, -102).unwrap(),
        ExactRateInterval::new(103, 113).unwrap(),
        ExactRateInterval::new(-113, -103).unwrap(),
        ExactRateInterval::new(104, 114).unwrap(),
        ExactRateInterval::new(-114, -104).unwrap(),
    ];
    let mut workspace = TransientBuffer::new();
    for (slot, mean) in [100, -100, 102, -102, 104, -104, 106, -106]
        .into_iter()
        .enumerate()
    {
        store_amplitude_rung_mean(&mut workspace, slot, mean).unwrap();
    }
    data.rung_blocking_levels = 0x2244_3322;

    let report = fixed_gain_amplitude_report(&mut data, &workspace).unwrap();
    assert_eq!(report.amplitude_index, 0);
    assert_eq!(report.target_rpm, 16);
    assert_eq!(report.directions[0].response_interval_q, Some([103, 112]));
    assert_eq!(report.directions[1].response_interval_q, Some([-112, -103]));
    assert_eq!(report.directions[0].consensus.included_mask, 0b1111);
    assert_eq!(report.directions[0].excluded_mask, 0);
    assert_eq!(report.directions[0].robust_location_q, Some(103));
    assert_eq!(report.directions[1].robust_location_q, Some(-103));
    assert_eq!(report.directions[0].blocking_levels, [2, 3, 4, 2]);
    assert_eq!(
        [data.opening_direction_mask, data.bookend_available_mask],
        [1; 2]
    );
    assert_eq!(data.eligible_masks, [1; 2]);
}

#[test]
fn amplitude_fold_emits_a_dedicated_result() {
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
    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    data.plan.i_start = 1_024;
    data.plan.positive_rung_count = 5;
    data.plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    data.plan.selected_response_intervals_q = [[0x1234, 0], [0x55aa, 0]];
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = [ExactRateInterval::new(100, 110).unwrap(); 8];
    for slot in 0..8 {
        store_amplitude_rung_mean(&mut workspace, slot, 105).unwrap();
    }
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    *state = VelocityIntegralState::AnalyzeRung;
    let mut backend = MockBackend::new();

    let StepResult::FixedGainAmplitudeEvent(
        crate::commissioning::types::FixedGainAmplitudeEvent::Amplitude(report),
    ) = step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("amplitude fold must emit a dedicated amplitude event")
    };
    assert_eq!(report.evidence_sequence, 1);
    assert_eq!(report.report.amplitude_index, 0);

    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.opening_direction_mask = 0b11111;
    data.bookend_available_mask = 0b11111;
    data.eligible_masks = [0b00111; 2];
    data.emitted_observations = 40;
    data.emitted_rungs = 5;
    *state = VelocityIntegralState::Terminal;

    let StepResult::FixedGainAmplitudeTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("amplitude completion must emit a dedicated terminal")
    };
    assert_eq!(terminal.outcome, FixedGainAmplitudeOutcome::Complete);
    assert_eq!(terminal.emitted_amplitudes, 5);
}

#[test]
fn amplitude_recovery_uses_the_retained_top_point_after_a_low_amplitude_precision_floor() {
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
    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    data.plan.planned_velocity_mrev_s = 4_394;
    data.plan.target_velocity_rpm = 263;
    data.plan.i_start = 1_024;
    data.plan.positive_rung_count = 5;
    data.plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    data.plan.recovery_lower_rate_q = [13_217_792, 12_827_648];
    data.rung_classes = core::array::from_fn(|slot| {
        if slot.is_multiple_of(2) || slot < 4 {
            VelocityObservationClass::Eligible
        } else {
            VelocityObservationClass::StationarityExcluded(StationarityExclusion::NoBlockingPlateau)
        }
    });
    data.rung_intervals = core::array::from_fn(|slot| {
        if slot.is_multiple_of(2) {
            ExactRateInterval::new(800_000, 900_000).unwrap()
        } else if slot < 4 {
            ExactRateInterval::new(-900_000, -800_000).unwrap()
        } else {
            ExactRateInterval::EMPTY
        }
    });
    for slot in 0..8 {
        store_amplitude_rung_mean(
            &mut workspace,
            slot,
            if slot.is_multiple_of(2) {
                850_000
            } else {
                -850_000
            },
        )
        .unwrap();
    }
    *safety = OuterSafetyLimits::new(
        data.plan.hard_torque_limit,
        OuterMotionBudget {
            requested_velocity_mrev_s: 4_394,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Explicit,
            max_stroke_travel_mrev: 2_500,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 50_000,
            positive_position_headroom_mrev: 50_000,
            max_duration_ms: 3_000,
            homing_speed_mrev_s: 4_375,
            max_accel_mrev_s2: 200_000,
        },
    );
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    safety.unwrapped_position = -1_834;
    *state = VelocityIntegralState::AnalyzeRung;
    let mut backend = MockBackend::new();

    let StepResult::FixedGainAmplitudeEvent(
        crate::commissioning::types::FixedGainAmplitudeEvent::Amplitude(report),
    ) = step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("the low-amplitude precision floor must not invalidate recovery authority")
    };
    assert_eq!(
        report.report.directions[1].availability,
        FixedGainAmplitudeAvailability::BelowPrecisionFloor
    );
    assert!(matches!(
        phase,
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::RecoverRung(_),
            ..
        }
    ));
}

#[test]
fn amplitude_terminals_distinguish_sufficiency_recovery_and_integrity() {
    let mut amplitude = plan();
    amplitude.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    amplitude.acceptance_plan_digest = 0x1234;
    amplitude.qualifier_bits = 0x55aa;
    amplitude.positive_rung_count = 5;
    amplitude.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    let mut data = VelocityIntegralRuntime::new(1, amplitude);
    data.opening_direction_mask = 0b11111;
    data.bookend_available_mask = 0b11111;
    data.eligible_masks = [0b00111; 2];
    data.emitted_observations = 40;
    data.emitted_rungs = 5;

    let complete = fixed_gain_amplitude_terminal(&data, false, None);
    assert_eq!(complete.outcome, FixedGainAmplitudeOutcome::Complete);
    assert_eq!(complete.cause, FixedGainAmplitudeCause::None);
    assert_eq!(complete.acceptance_plan_digest, 0x1234);
    assert_eq!(complete.qualifier_bits, 0x55aa);
    assert_eq!(complete.unattempted_masks, [0; 2]);

    data.eligible_masks[1] = 0b00011;
    let insufficient = fixed_gain_amplitude_terminal(&data, false, None);
    assert_eq!(
        insufficient.outcome,
        FixedGainAmplitudeOutcome::Inconclusive
    );
    assert_eq!(
        insufficient.cause,
        FixedGainAmplitudeCause::InsufficientSharedFloor
    );

    data.eligible_masks[1] = 0b00111;
    data.recovery_unavailable = true;
    let recovery = fixed_gain_amplitude_terminal(&data, false, None);
    assert_eq!(recovery.outcome, FixedGainAmplitudeOutcome::Inconclusive);
    assert_eq!(recovery.cause, FixedGainAmplitudeCause::RecoveryUnavailable);

    data.recovery_unavailable = false;
    data.rest_unconfirmed = true;
    let rest_unconfirmed = fixed_gain_amplitude_terminal(&data, false, None);
    assert_eq!(
        rest_unconfirmed.outcome,
        FixedGainAmplitudeOutcome::Inconclusive
    );
    assert_eq!(
        rest_unconfirmed.cause,
        FixedGainAmplitudeCause::VelocityRestNotConfirmed
    );
    assert_eq!(rest_unconfirmed.attempted_masks, [0b1_1111; 2]);
    assert_eq!(rest_unconfirmed.eligible_masks, [0b00111; 2]);

    let fault = fixed_gain_amplitude_terminal(&data, true, None);
    assert_eq!(fault.outcome, FixedGainAmplitudeOutcome::Fault);
    assert_eq!(fault.cause, FixedGainAmplitudeCause::EvidenceIntegrity);
}

#[test]
fn non_amplitude_plan_faults_on_evidence_integrity() {
    let mut data = VelocityIntegralRuntime::new(1, plan());
    data.opening_direction_mask = 0b11111;
    data.bookend_available_mask = 0b11111;
    data.eligible_masks = [0b00111; 2];
    data.emitted_observations = 40;
    data.emitted_rungs = 5;

    let terminal = fixed_gain_amplitude_terminal(&data, false, None);
    assert_eq!(terminal.outcome, FixedGainAmplitudeOutcome::Fault);
    assert_eq!(terminal.cause, FixedGainAmplitudeCause::EvidenceIntegrity);
    assert_eq!(terminal.acceptance_plan_digest, 0);
    assert_eq!(terminal.qualifier_bits, 0);
}

#[test]
fn both_amplitude_orders_end_an_observation_rest_rejection_with_cause_53() {
    for acquisition_kind in [
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending,
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending,
    ] {
        let mut phase = phase();
        let mut workspace = TransientBuffer::new();
        let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
            unreachable!()
        };
        data.plan.acquisition_kind = acquisition_kind;
        data.plan.i_start = 1_024;
        data.plan.positive_rung_count = 5;
        data.plan.amplitude_targets_rpm = match acquisition_kind {
            VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending => [16, 33, 66, 132, 263],
            VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending => [263, 132, 66, 33, 16],
            _ => unreachable!(),
        };
        data.plan.selected_response_intervals_q = [[0x1234, 0], [0x55aa, 0]];
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
                StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::RestSelection(
                    event,
                )) => {
                    diagnostic = Some(event);
                }
                StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::Current(_)) => {}
                StepResult::FixedGainAmplitudeTerminal(terminal) => break terminal,
                other => panic!("unexpected amplitude rest result: {other:?}"),
            }
        };

        let diagnostic = diagnostic.expect("amplitude rest analysis must emit Event 151");
        assert_eq!(
            diagnostic.owner,
            RestSelectionOwner::FixedGainAmplitudeObservation
        );
        assert_eq!(terminal.outcome, FixedGainAmplitudeOutcome::Inconclusive);
        assert_eq!(
            terminal.cause,
            FixedGainAmplitudeCause::VelocityRestNotConfirmed
        );
        assert_eq!(terminal.attempted_masks, [0; 2]);
        assert_eq!(terminal.unattempted_masks, [0b1_1111; 2]);
        let OuterPhase::VelocityIntegral { data, .. } = &phase else {
            unreachable!()
        };
        assert_eq!(
            data.rest_unconfirmed_owner,
            Some(RestSelectionOwner::FixedGainAmplitudeObservation)
        );
    }
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
    apply_integral_current_headroom(state, data);

    assert_eq!(data.current_terminus, Some(2));
    assert_eq!(data.rung_index, 3);
    assert!(matches!(state, VelocityIntegralState::Terminal));
}

#[test]
fn amplitude_current_continuation_follows_only_remaining_lower_amplitudes() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    data.plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    data.rung_index = 2;
    apply_integral_current_headroom(state, data);
    assert_eq!(data.rung_index, 2);
    assert!(matches!(state, VelocityIntegralState::Terminal));

    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending;
    data.plan.amplitude_targets_rpm = [263, 132, 66, 33, 16];
    data.current_terminus = None;
    data.rung_index = 2;
    apply_integral_current_headroom(state, data);
    assert_eq!(data.rung_index, 3);
    assert!(matches!(
        state,
        VelocityIntegralState::PrepareMovingSlot { reset_rung: true }
    ));
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
fn finalize_recovery_resolves_a_contained_fixed_gain_amplitude_rung_as_recovery_failure() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
        unreachable!()
    };
    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    data.plan.positive_rung_count = 5;
    data.rung_index = 2;
    *state = VelocityIntegralState::FinalizeRecovery(contained_report(2));
    let mut workspace = TransientBuffer::new();
    let mut backend = MockBackend::new();

    assert!(matches!(
        step(&mut phase, &mut workspace, &mut backend),
        StepResult::FixedGainAmplitudeEvent(_)
    ));
    let OuterPhase::VelocityIntegral { state, data, .. } = phase else {
        unreachable!()
    };
    assert!(data.recovery_failure);
    assert!(!data.recovery_unavailable);
    assert!(matches!(state, VelocityIntegralState::Terminal));
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
    *state = VelocityIntegralState::PrepareZeroSettle {
        entry_transition_us: 0,
    };
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
    data.plan.acceptance_source = AcceptanceSource::Breakaway;
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
    data.plan.acceptance_source = AcceptanceSource::Breakaway;
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
    data.plan.acceptance_source = AcceptanceSource::Breakaway;
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
    data.plan.acceptance_source = AcceptanceSource::Breakaway;
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

#[test]
fn amplitude_owner_reaching_evidence_terminal_is_reported_as_integrity_fault() {
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
    data.rest_unconfirmed_owner = Some(RestSelectionOwner::FixedGainAmplitudeRecovery);
    *state = VelocityIntegralState::Terminal;
    let mut backend = MockBackend::new();

    let StepResult::VelocityIntegralTerminal(terminal) =
        step(&mut phase, &mut workspace, &mut backend)
    else {
        panic!("an Amplitude owner on this path must still terminate Integral");
    };
    assert_eq!(terminal.outcome, VelocityIntegralOutcome::Fault);
    assert_eq!(
        terminal.cause,
        IntegralOutcomeCause::Engine(INTEGRAL_CAUSE_FIXED_GAIN_REST_OWNERSHIP)
    );
    assert!(!terminal.rest_rejection_after_sufficiency);
    assert_eq!(terminal.rest_rejection_owner, None);
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
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, value)
            if *address == PID_VELOCITY_TARGET.addr() && *value == 176)
    }));
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
fn amplitude_slots_program_fixed_i_and_exact_signed_amplitudes() {
    for (rung, expected_target) in [16_i32, 33, 66, 132, 263].into_iter().enumerate() {
        for slot in 0..INTEGRAL_SLOT_COUNT {
            let mut phase = phase();
            let mut workspace = TransientBuffer::new();
            let OuterPhase::VelocityIntegral { state, data, .. } = &mut phase else {
                unreachable!()
            };
            data.plan.acquisition_kind =
                VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
            data.plan.i_start = 1_024;
            data.plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
            data.plan.positive_rung_count = 5;
            data.rung_index = rung as u8;
            data.slot = slot;
            *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: false };
            let mut backend = MockBackend::new();
            backend.current_limit_value = current_limit(2_816);

            assert!(matches!(
                step(&mut phase, &mut workspace, &mut backend),
                StepResult::ReadyNow
            ));
            assert!(logged_writes(&backend).iter().any(|(address, value)| {
                *address == PID_VELOCITY_P_VELOCITY_I.addr()
                    && *value
                        == PidGainPair::new(
                            PidPGain::new(1_448).unwrap(),
                            PidIGain::new(1_024).unwrap(),
                        )
                        .register_value()
            }));

            assert!(matches!(
                step(&mut phase, &mut workspace, &mut backend),
                StepResult::At(_)
            ));
            let signed_target = if slot.is_multiple_of(2) {
                expected_target
            } else {
                -expected_target
            };
            assert!(logged_writes(&backend).iter().any(|(address, value)| {
                *address == PID_VELOCITY_TARGET.addr()
                    && *value == u32::from_ne_bytes(signed_target.to_ne_bytes())
            }));
        }
    }
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
    data.rung_blocking_levels = 0x1234_1234;
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
        assert_eq!(data.rung_blocking_levels, 0x1234_1234);
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
    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    data.plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    data.rung_index = 1;
    data.rung_classes = [VelocityObservationClass::Eligible; 8];
    data.rung_intervals = [ExactRateInterval::new(10, 20).unwrap(); 8];
    for (slot, mean) in [10, 20, 30, 40, 50, 60, 70, 80].into_iter().enumerate() {
        store_amplitude_rung_mean(&mut workspace, slot, mean).unwrap();
    }
    data.rung_blocking_levels = 0x1234_1234;
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
    assert_eq!(
        core::array::from_fn::<_, 8, _>(|slot| amplitude_rung_mean(&workspace, slot).unwrap()),
        [0; 8]
    );
    assert_eq!(data.rung_blocking_levels, 0);
    assert_eq!(data.eligible_masks, [0b101; 2]);
}

#[test]
fn amplitude_rung_mean_accessors_report_out_of_range_slot() {
    let mut workspace = TransientBuffer::new();

    let store_error =
        store_amplitude_rung_mean(&mut workspace, usize::from(INTEGRAL_SLOT_COUNT), 0).unwrap_err();
    assert_eq!(store_error, CommissionError::RungIndexOutOfRange);

    let read_error = amplitude_rung_mean(&workspace, usize::from(INTEGRAL_SLOT_COUNT)).unwrap_err();
    assert_eq!(read_error, CommissionError::RungIndexOutOfRange);
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
        *state = VelocityIntegralState::PrepareZeroSettle {
            entry_transition_us: 0,
        };
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
        VelocityIntegralState::CommandTarget {
            entry_transition_us: 500
        }
    ));
    data.current_observer = VelocityCurrentObserver::new(
        current_limit(u32::from(data.plan.usable_torque_limit)),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    *state = VelocityIntegralState::PrepareZeroSettle {
        entry_transition_us: 500,
    };
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
    *state = VelocityIntegralState::PrepareZeroSettle {
        entry_transition_us: 0,
    };
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
    *state = VelocityIntegralState::PrepareZeroSettle {
        entry_transition_us: 0,
    };
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
    *state = VelocityIntegralState::PrepareZeroSettle {
        entry_transition_us: 19_601,
    };
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
fn amplitude_safety_fault_uses_the_amplitude_terminal() {
    let mut phase = phase();
    let OuterPhase::VelocityIntegral { data, .. } = &mut phase else {
        unreachable!()
    };
    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    data.plan.positive_rung_count = 5;
    data.plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];

    let StepResult::FixedGainAmplitudeTerminal(terminal) =
        integral_fault_terminal(&phase, CommissionError::SafetyEnvelopeViolation, None)
    else {
        panic!("amplitude fault must not fall through to the generic terminal")
    };
    assert_eq!(terminal.outcome, FixedGainAmplitudeOutcome::Fault);
    assert_eq!(terminal.cause, FixedGainAmplitudeCause::EvidenceIntegrity);
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
fn recovery_rest_rejection_records_the_amplitude_owner() {
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
    data.plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
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

    let mut saw_rest_selection = false;
    let mut saw_recovery_rest = false;
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
            StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::RestSelection(_)) => {
                assert!(
                    !saw_recovery_rest,
                    "RestSelection must be observed before RecoveryRest"
                );
                saw_rest_selection = true;
            }
            StepResult::FixedGainAmplitudeEvent(FixedGainAmplitudeEvent::RecoveryRest(_)) => {
                saw_recovery_rest = true;
            }
            StepResult::FixedGainAmplitudeEvent(_) => {}
            StepResult::VelocityIntegralTerminal(terminal) => break Some(terminal),
            StepResult::FixedGainAmplitudeTerminal(_) => break None,
            other => panic!("unexpected recovery rest result: {other:?}"),
        }
    };
    assert!(saw_rest_selection, "RestSelection event was never emitted");
    assert!(saw_recovery_rest, "RecoveryRest event was never emitted");

    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert!(data.rest_unconfirmed);
    assert_eq!(
        data.rest_unconfirmed_owner,
        Some(RestSelectionOwner::FixedGainAmplitudeRecovery)
    );
    if let Some(terminal) = terminal {
        assert_eq!(terminal.outcome, VelocityIntegralOutcome::Inconclusive);
        assert_eq!(
            terminal.cause,
            IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code())
        );
    }
}

#[test]
fn non_amplitude_recover_rung_rest_unconfirmed_is_attributed_to_integral_not_amplitude() {
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
    assert!(!is_fixed_gain_amplitude(data.plan));
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

    let mut saw_rest_selection = false;
    let mut saw_recovery_rest = false;
    let terminal = loop {
        match step(&mut phase, &mut workspace, &mut backend) {
            StepResult::ReadyNow => {}
            StepResult::At(deadline_us) => {
                backend.elapsed_us_value = deadline_us;
                let drift = i32::try_from(deadline_us / 1_000).unwrap_or(i32::MAX);
                let offset = start_offset_counts.saturating_sub(drift);
                backend.mechanical_encoder_position_counts_value = offset.rem_euclid(4_000) as u32;
            }
            StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RestSelection(_)) => {
                assert!(
                    !saw_recovery_rest,
                    "RestSelection must be observed before RecoveryRest"
                );
                saw_rest_selection = true;
            }
            StepResult::VelocityIntegralEvent(VelocityIntegralEvent::RecoveryRest(_)) => {
                saw_recovery_rest = true;
            }
            StepResult::VelocityIntegralEvent(_) => {}
            StepResult::VelocityIntegralTerminal(terminal) => break terminal,
            other => {
                panic!(
                    "non-amplitude plan must never produce an FixedGainAmplitude result: {other:?}"
                )
            }
        }
    };
    assert!(saw_rest_selection, "RestSelection event was never emitted");
    assert!(saw_recovery_rest, "RecoveryRest event was never emitted");

    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        unreachable!()
    };
    assert!(data.rest_unconfirmed);
    assert_eq!(
        data.rest_unconfirmed_owner,
        Some(RestSelectionOwner::IntegralRecovery)
    );
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
    test_plan.acceptance_source = AcceptanceSource::NonBreakaway;
    test_plan.acquisition_kind = VelocityIntegralAcquisitionKind::ResponseLadder;
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
    test_plan.acceptance_source = AcceptanceSource::Breakaway;
    test_plan.i_top_cap = 0;
    test_plan.positive_rung_count = 12;
    test_plan.acquisition_kind = VelocityIntegralAcquisitionKind::ResponseLadder;
    let total = integral_rung_count(test_plan).unwrap();
    for index in 0..total {
        assert_ne!(integral_i_at(test_plan, index), Some(724));
    }
}

#[test]
fn integral_i_at_fixed_gain_amplitude_behavior_unchanged_by_i_top_cap_field() {
    let mut test_plan = plan();
    test_plan.i_start = 512;
    test_plan.acquisition_kind = VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending;
    test_plan.i_top_cap = 0;
    test_plan.positive_rung_count = 5;
    test_plan.amplitude_targets_rpm = [16, 33, 66, 132, 263];
    let total = integral_rung_count(test_plan).unwrap();
    for index in 0..total {
        assert_eq!(integral_i_at(test_plan, index), Some(test_plan.i_start));
    }
}
