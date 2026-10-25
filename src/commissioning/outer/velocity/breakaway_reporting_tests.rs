use foci_tmc4671::pid::PidPGain;

use super::super::band_confirmation::{
    BandConfirmationFailureCause, CONFIRMATION_BAND_LOWER_PERCENT, CONFIRMATION_BAND_UPPER_PERCENT,
    ConfirmationDirectionalInput, DiscoveryObservationAtom, DiscoveryRungNominationBounds,
    plan_band_confirmation,
};
use super::super::breakaway_campaign::{
    AdditiveDiscoveryOutcome, BandConfirmationOutcome, NominationEvidence,
    ObservationClassifierInputs,
};
use super::super::breakaway_plan::{
    AdditiveDiscoveryRequest, BreakawayProbeContext, plan_additive_discovery_ladder,
    plan_upward_breakaway_probe,
};
use super::super::breakaway_search::BreakawayMotionClass;
use super::super::sweep_consensus::{CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus};
use super::super::sweep_evidence::VelocityObservationClass;
use super::*;
use crate::autotune::CommissionError;
use crate::commissioning::types::OuterSafetyFault;

fn gain(raw: u16) -> PidPGain {
    PidPGain::new(raw).expect("raw gain within PidPGain range")
}

fn probe_context() -> BreakawayProbeContext {
    BreakawayProbeContext {
        workflow_digest: 0xA1B2_C3D4_E5F6_0718,
        target_velocity_rpm: 600,
        hard_torque_limit: 2_816,
        usable_torque_limit: 2_048,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
    }
}

fn zero_safety_fault() -> OuterSafetyFault {
    OuterSafetyFault {
        reason: 0,
        max_travel_mrev: 0,
        max_velocity_mrev_s: 0,
        max_duration_ms: 0,
        direction_mask: 0,
        delta_counts: 0,
        dt_us: 0,
        velocity_counts_per_ms: 0,
        velocity_cap_counts_per_ms: 0,
        position_counts: 0,
        position_window_counts: 0,
        elapsed_us: 0,
        duration_cap_us: 0,
    }
}

fn atom(
    direction: VelocityDirection,
    mean_rate_q: i64,
    variance_of_mean_q2: u128,
    target_rate_q: i64,
) -> DiscoveryObservationAtom {
    DiscoveryObservationAtom {
        mean_rate_q,
        variance_of_mean_q2,
        target_rate_q,
        direction,
    }
}

const TARGET: i64 = 1_000_000;
const RUNG18_MEAN: i64 = 750_000;
const RUNG18_VARIANCE: u128 = 58_752_225;

fn full_consensus() -> DirectionalConsensus {
    DirectionalConsensus {
        classification: crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    }
}

fn uniform_observations(
    direction: VelocityDirection,
    mean_rate_q: i64,
    variance_of_mean_q2: u128,
    target_rate_q: i64,
) -> [Option<DiscoveryObservationAtom>; CONSENSUS_OBSERVATIONS_PER_DIRECTION] {
    [Some(atom(
        direction,
        mean_rate_q,
        variance_of_mean_q2,
        target_rate_q,
    )); 4]
}

fn confirmation_input(
    direction: VelocityDirection,
    mean_rate_q: i64,
    variance_of_mean_q2: u128,
    target_rate_q: i64,
) -> ConfirmationDirectionalInput {
    ConfirmationDirectionalInput {
        consensus: full_consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: uniform_observations(
            direction,
            mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
        ),
    }
}

#[test]
fn probe_plan_report_carries_plan_digest_and_endpoints() {
    let p_start = gain(1);
    let p_top = gain(1024);
    let plan =
        plan_upward_breakaway_probe(p_start, p_top, probe_context()).expect("valid probe plan");

    let report = probe_plan_report(&plan, p_start, p_top, 7, 3);

    assert_eq!(report.run_sequence, 7);
    assert_eq!(report.evidence_sequence, 3);
    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.max_observations, plan.maximum_observations);
    assert_eq!(report.search_count, plan.search_count);
    assert_eq!(report.p_start_raw, 1);
    assert_eq!(report.p_top_raw, 1024);
    assert_eq!(report.workflow_digest, plan.context.workflow_digest);
    assert_eq!(report.target_velocity_rpm, plan.context.target_velocity_rpm);
    assert_eq!(report.hard_torque_limit, plan.context.hard_torque_limit);
    assert_eq!(report.usable_torque_limit, plan.context.usable_torque_limit);
    assert_eq!(report.pwm_hz, plan.context.pwm_hz);
    assert_eq!(
        report.encoder_counts_per_rev,
        plan.context.encoder_counts_per_rev
    );
    assert_eq!(report.motion_threshold_counts, plan.motion_threshold_counts);
    assert_eq!(report.max_capture_interval_us, plan.max_capture_interval_us);
}

#[test]
fn probe_resolved_report_resolves_the_grid_gain() {
    let plan = plan_upward_breakaway_probe(gain(1), gain(1024), probe_context())
        .expect("valid probe plan");
    let report = probe_resolved_report(&plan, 3, 17, 1, 0);

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.rung_index, 3);
    assert_eq!(report.breakaway_p_raw, plan.search_gains[3].raw());
    assert_eq!(report.motion_threshold_counts, plan.motion_threshold_counts);
    assert_eq!(report.observation_count, 17);
}

#[test]
fn breakaway_terminal_cause_wire_values_preserve_reserved_probe_slots() {
    let causes = [
        BreakawayTerminalCause::ProbeNoRepeatableMotionAtAuthority,
        BreakawayTerminalCause::ProbeAuthorityFailure,
        BreakawayTerminalCause::ProbeSafetyFault,
        BreakawayTerminalCause::DiscoveryTargetBandSkipped,
        BreakawayTerminalCause::DiscoveryCurrentHeadroomBeforeBand,
        BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
        BreakawayTerminalCause::DiscoveryAdditiveLadderResolutionBudgetExceeded,
        BreakawayTerminalCause::DiscoveryEvidenceExcluded,
        BreakawayTerminalCause::DiscoverySafetyFault,
        BreakawayTerminalCause::ConfirmationTargetBandUnconfirmableFromDiscovery,
        BreakawayTerminalCause::ConfirmationTargetBandNotReachedAtAuthority,
        BreakawayTerminalCause::ConfirmationResponseLocation,
        BreakawayTerminalCause::ConfirmationExcessiveUncertainty,
        BreakawayTerminalCause::ConfirmationLossOfConsensus,
        BreakawayTerminalCause::ConfirmationCurrentAuthority,
        BreakawayTerminalCause::ConfirmationEvidenceExcluded,
        BreakawayTerminalCause::ConfirmationSafetyFault,
        BreakawayTerminalCause::Accepted,
        BreakawayTerminalCause::ProbeExcursionEvidenceInvalid,
    ];

    assert_eq!(BreakawayTerminalCause::None as u8, 0);
    assert_eq!(
        causes.map(|cause| cause as u8),
        [
            1, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
        ]
    );
}

#[test]
fn probe_terminal_cause_maps_every_non_found_variant() {
    use super::super::breakaway_campaign::BreakawayProbeOutcome;

    let cases = [
        (
            BreakawayProbeOutcome::NoRepeatableMotionAtAuthority,
            BreakawayTerminalCause::ProbeNoRepeatableMotionAtAuthority,
        ),
        (
            BreakawayProbeOutcome::ExcursionEvidenceInvalid,
            BreakawayTerminalCause::ProbeExcursionEvidenceInvalid,
        ),
        (
            BreakawayProbeOutcome::AuthorityFailure,
            BreakawayTerminalCause::ProbeAuthorityFailure,
        ),
    ];
    for (outcome, expected) in cases {
        let (cause, has_fault) = probe_terminal_cause(&outcome);
        assert_eq!(cause, expected, "{outcome:?}");
        assert_eq!(has_fault, 0);
    }

    let (cause, has_fault) = probe_terminal_cause(&BreakawayProbeOutcome::SafetyFault {
        error: CommissionError::SafetyEnvelopeViolation,
        safety: Some(zero_safety_fault()),
    });
    assert_eq!(cause, BreakawayTerminalCause::ProbeSafetyFault);
    assert_eq!(has_fault, 1);

    let (cause, has_fault) = probe_terminal_cause(&BreakawayProbeOutcome::SafetyFault {
        error: CommissionError::Spi,
        safety: None,
    });
    assert_eq!(cause, BreakawayTerminalCause::ProbeInternalFault);
    assert_eq!(has_fault, 0);
}

#[test]
fn probe_terminal_report_carries_plan_identity_and_cause() {
    use super::super::breakaway_campaign::BreakawayProbeOutcome;

    let plan = plan_upward_breakaway_probe(gain(1), gain(1024), probe_context())
        .expect("valid probe plan");
    let report = probe_terminal_report(&plan, &BreakawayProbeOutcome::AuthorityFailure, 9, 4);

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::ProbeAuthorityFailure as u8
    );
    assert_eq!(report.has_safety_fault, 0);
    assert_eq!(report.fault_checkpoint, 0);

    crate::diagnostic::begin_breakaway_campaign();
    crate::diagnostic::checkpoint_breakaway_campaign(
        crate::diagnostic::BreakawayCheckpoint::ProbeMovingEncoderSample,
    );
    let report = probe_terminal_report(
        &plan,
        &BreakawayProbeOutcome::SafetyFault {
            error: crate::autotune::CommissionError::Spi,
            safety: None,
        },
        9,
        6,
    );
    assert_eq!(
        report.fault_checkpoint,
        crate::diagnostic::BreakawayCheckpoint::ProbeMovingEncoderSample as u8
    );

    crate::diagnostic::checkpoint_breakaway_campaign(
        crate::diagnostic::BreakawayCheckpoint::ProbeExcursionIntervalInvalid,
    );
    let report = probe_terminal_report(
        &plan,
        &BreakawayProbeOutcome::ExcursionEvidenceInvalid,
        9,
        7,
    );
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::ProbeExcursionEvidenceInvalid as u8
    );
    assert_eq!(
        report.fault_checkpoint,
        crate::diagnostic::BreakawayCheckpoint::ProbeExcursionIntervalInvalid as u8
    );
}

fn discovery_fixture() -> (
    AdditiveDiscoveryRequest,
    super::super::breakaway_plan::AdditiveDiscoveryPlan,
) {
    let request = AdditiveDiscoveryRequest {
        floor: gain(64),
        breakaway: gain(91),
        authority_ceiling: gain(1024),
        observations_per_direction: 4,
        max_duration_ms: 2_000,
        workflow_envelope_ms: 508_000,
        probe_plan_digest: 0xabcd_ef01_2345_6789,
    };
    let plan = plan_additive_discovery_ladder(request).expect("valid discovery plan");
    (request, plan)
}

#[test]
fn discovery_plan_report_carries_floor_breakaway_ceiling_and_step() {
    let (request, plan) = discovery_fixture();

    let report = discovery_plan_report(&request, false, &plan, 2, 0);

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.prior_plan_digest, request.probe_plan_digest);
    assert_eq!(
        report.schema_revision,
        super::super::breakaway_plan::BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION
    );
    assert_eq!(report.family_size, plan.confidence_family_size);
    assert_eq!(report.rung_count, plan.rung_count);
    assert_eq!(report.floor_p_raw, 64);
    assert_eq!(report.floor_origin, 0);
    assert_eq!(report.breakaway_p_raw, 91);
    assert_eq!(report.ceiling_p_raw, 1024);
    assert_eq!(report.maximum_workflow_ms, request.workflow_envelope_ms);
    assert!(report.first_additive_step_raw > 0);

    let breakaway_position = plan.gains[..usize::from(plan.rung_count)]
        .iter()
        .position(|g| g.raw() == 91)
        .expect("breakaway present in resolved ladder");
    let expected_step = u32::from(plan.gains[breakaway_position + 1].raw())
        - u32::from(plan.gains[breakaway_position].raw());
    assert_eq!(report.first_additive_step_raw, expected_step);
}

#[test]
fn discovery_plan_report_marks_clamped_floor_origin() {
    let (request, plan) = discovery_fixture();
    let report = discovery_plan_report(&request, true, &plan, 2, 0);
    assert_eq!(report.floor_origin, 1);
}

#[test]
fn raw_observation_report_carries_atom_inputs_verbatim() {
    let source = atom(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );

    let collected = CollectedObservation {
        atom: source,
        rung_index: 0,
        slot_index: 6,
        classifier_inputs: ObservationClassifierInputs {
            stationarity_code: 3,
            settled_rate_low_q: -11,
            settled_rate_high_q: 22,
            deficit_low_q: -33,
            deficit_high_q: 44,
            predicted_torque_target_abs: 55,
            moving_pid_output_peak: 51,
            usable_torque_limit: 66,
            hard_torque_limit: 77,
            disturbance_present: true,
            disturbance_low_q16: -88,
            disturbance_high_q16: 99,
            drift_ratio_ppm: 123_456,
            post_fit_lag_one_state: 2,
        },
        stroke_min_displacement_counts: -63,
        stroke_max_displacement_counts: 127,
        capture_max_interval_us: 2_001,
    };
    let report = raw_observation_report(
        0x1122_3344_5566_7788,
        BreakawayPhase::ProportionalBandConfirmation,
        &collected,
        7,
        9,
    );

    assert_eq!(report.run_sequence, 7);
    assert_eq!(report.evidence_sequence, 9);
    assert_eq!(report.plan_digest, 0x1122_3344_5566_7788);
    assert_eq!(
        report.phase,
        BreakawayPhase::ProportionalBandConfirmation as u8
    );
    assert_eq!(report.rung_index, 0);
    assert_eq!(report.slot_index, 6);
    assert_eq!(
        report.direction,
        direction_report_code(VelocityDirection::Reverse)
    );
    assert_eq!(report.mean_rate_q, -RUNG18_MEAN);
    assert_eq!(report.variance_of_mean_q2, RUNG18_VARIANCE);
    assert_eq!(report.target_rate_q, TARGET);
    assert_eq!(report.stationarity_code, 3);
    assert_eq!(report.settled_rate_low_q, -11);
    assert_eq!(report.settled_rate_high_q, 22);
    assert_eq!(report.deficit_low_q, -33);
    assert_eq!(report.deficit_high_q, 44);
    assert_eq!(report.predicted_torque_target_abs, 55);
    assert_eq!(report.usable_torque_limit, 66);
    assert_eq!(report.hard_torque_limit, 77);
    assert_eq!(report.disturbance_present, 1);
    assert_eq!(report.disturbance_low_q16, -88);
    assert_eq!(report.disturbance_high_q16, 99);
    assert_eq!(report.stroke_min_displacement_counts, -63);
    assert_eq!(report.stroke_max_displacement_counts, 127);
    assert_eq!(report.capture_max_interval_us, 2_001);
}

fn nominatable_bounds(
    p_raw: u16,
    lower_percent: f32,
    upper_percent: f32,
) -> DiscoveryRungNominationBounds {
    DiscoveryRungNominationBounds {
        p: gain(p_raw),
        lower_percent,
        upper_percent,
        nominatable: true,
        step_peak_permille: 0,
    }
}

#[test]
fn discovery_rung_margin_report_computes_margin_and_half_width() {
    let bounds = [nominatable_bounds(362, 71.0, 79.0)];

    let report = discovery_rung_margin_report(&bounds, 0, 0x11, 3, 2).expect("nominatable rung");

    assert_eq!(report.plan_digest, 0x11);
    assert_eq!(report.p_raw, 362);
    assert_eq!(report.lower_percent_milli, 71_000);
    assert_eq!(report.upper_percent_milli, 79_000);
    assert_eq!(report.margin_percent_milli, 1_000);
    assert_eq!(report.half_width_percent_milli, 4_000);
    assert_eq!(report.adjacent_rate_step_percent_milli, 0);
}

#[test]
fn discovery_rung_margin_report_computes_adjacent_step_from_previous_midpoint() {
    let bounds = [
        nominatable_bounds(362, 71.0, 79.0),
        nominatable_bounds(512, 73.0, 77.0),
    ];

    let second = discovery_rung_margin_report(&bounds, 1, 0x11, 3, 2).expect("nominatable rung");

    assert_eq!(second.adjacent_rate_step_percent_milli, 0);

    let bounds_shifted = [
        nominatable_bounds(362, 71.0, 79.0),
        nominatable_bounds(512, 76.0, 80.0),
    ];
    let shifted =
        discovery_rung_margin_report(&bounds_shifted, 1, 0x11, 3, 2).expect("nominatable rung");
    assert_eq!(shifted.adjacent_rate_step_percent_milli, 3_000);
}

#[test]
fn discovery_rung_margin_report_rejects_non_nominatable_and_out_of_range() {
    let bounds = [DiscoveryRungNominationBounds {
        p: gain(362),
        lower_percent: 0.0,
        upper_percent: 0.0,
        nominatable: false,
        step_peak_permille: 0,
    }];

    assert!(discovery_rung_margin_report(&bounds, 0, 0x11, 3, 2).is_none());
    assert!(discovery_rung_margin_report(&bounds, 5, 0x11, 3, 2).is_none());
}

#[test]
fn rung_zero_diagnostic_report_flags_each_direction_independently() {
    let (_, plan) = discovery_fixture();

    let report = rung_zero_diagnostic_report(
        &plan,
        BreakawayMotionClass::Inert,
        BreakawayMotionClass::Moving,
        4,
        1,
    );

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.forward_moved, 0);
    assert_eq!(report.reverse_moved, 1);
}

#[test]
fn discovery_terminal_cause_maps_every_non_selected_variant() {
    let cases = [
        (
            AdditiveDiscoveryOutcome::TargetBandSkipped,
            BreakawayTerminalCause::DiscoveryTargetBandSkipped,
        ),
        (
            AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand,
            BreakawayTerminalCause::DiscoveryCurrentHeadroomBeforeBand,
        ),
        (
            AdditiveDiscoveryOutcome::InsufficientAdditiveSpan,
            BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
        ),
        (
            AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded,
            BreakawayTerminalCause::DiscoveryAdditiveLadderResolutionBudgetExceeded,
        ),
        (
            AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded {
                error: CommissionError::VelocityRestNotConfirmed,
            },
            BreakawayTerminalCause::DiscoveryEvidenceExcluded,
        ),
    ];
    for (outcome, expected) in cases {
        let (cause, has_fault) = discovery_terminal_cause(&outcome);
        assert_eq!(cause, expected, "{outcome:?}");
        assert_eq!(has_fault, 0);
    }

    let (cause, has_fault) = discovery_terminal_cause(&AdditiveDiscoveryOutcome::SafetyFault {
        error: CommissionError::SafetyEnvelopeViolation,
        safety: Some(zero_safety_fault()),
    });
    assert_eq!(cause, BreakawayTerminalCause::DiscoverySafetyFault);
    assert_eq!(has_fault, 1);
}

#[test]
fn discovery_terminal_report_matches_the_field_level_assembly() {
    let (_, plan) = discovery_fixture();

    let outcome = AdditiveDiscoveryOutcome::InsufficientAdditiveSpan;
    let via_outcome = discovery_terminal_report(&plan, &outcome, 0, 0, 6, 2);
    let via_fields = discovery_terminal_fields(
        &plan,
        DiscoveryTerminalEvidence {
            cause: BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
            has_safety_fault: 0,
            collected_count: 0,
            stop_rung_index: 0,
            error_code: 0,
        },
        6,
        2,
    );

    assert_eq!(via_outcome, via_fields);
}

#[test]
fn discovery_safety_fault_terminal_reports_the_rungs_already_collected() {
    let (_, plan) = discovery_fixture();

    let outcome = AdditiveDiscoveryOutcome::SafetyFault {
        error: CommissionError::SafetyEnvelopeViolation,
        safety: Some(zero_safety_fault()),
    };

    let report = discovery_terminal_report(&plan, &outcome, 3, 5, 6, 2);

    assert_eq!(report.collected_count, 3);
    assert_eq!(report.stop_rung_index, 5);
    assert_eq!(report.has_safety_fault, 1);
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::DiscoverySafetyFault as u8
    );
}

#[test]
fn confirmation_plan_report_carries_band_and_margin() {
    let plan = plan_band_confirmation(0x22, gain(724)).expect("valid confirmation plan");
    let nomination = NominatedConfirmationCandidate {
        p: gain(724),
        margin_percent: 4.5,
        step_peak_permille: 0,
        overshoot_excluded_rungs: 0,
    };

    let report = confirmation_plan_report(&plan, &nomination, 8, 0);

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.prior_plan_digest, 0x22);
    assert_eq!(report.family_size, plan.family_size);
    assert_eq!(report.nominated_p_raw, 724);
    assert_eq!(report.band_lower_percent, CONFIRMATION_BAND_LOWER_PERCENT);
    assert_eq!(report.band_upper_percent, CONFIRMATION_BAND_UPPER_PERCENT);
    assert_eq!(report.nominated_margin_percent_milli, 4_500);
}

#[test]
fn confirmation_observation_report_matches_evaluate_band_confirmation_math() {
    let observation = atom(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );

    let report =
        confirmation_observation_report(0x33, 0, VelocityDirection::Forward, observation, 1, 0)
            .expect("representable observation");

    assert_eq!(report.plan_digest, 0x33);
    assert_eq!(
        report.direction,
        direction_report_code(VelocityDirection::Forward)
    );

    let multiplier = confidence_multiplier_95_df15(8).expect("family 8 within table");
    let relative_se = 7_665.0_f32 / 750_000.0;
    let half_width_fraction = multiplier * relative_se;
    let expected_lower_milli =
        ((75.0 * (1.0 - half_width_fraction) * 1_000.0).floor() / 1.0) as i32;
    let expected_upper_milli = ((75.0 * (1.0 + half_width_fraction) * 1_000.0).ceil() / 1.0) as i32;
    assert!(
        (report.response_lower_percent_milli - expected_lower_milli).abs() <= 1,
        "lower {} vs expected {expected_lower_milli}",
        report.response_lower_percent_milli
    );
    assert!(
        (report.response_upper_percent_milli - expected_upper_milli).abs() <= 1,
        "upper {} vs expected {expected_upper_milli}",
        report.response_upper_percent_milli
    );
    let expected_se_permille = (relative_se * 1_000.0).round() as u16;
    assert_eq!(
        report.relative_standard_error_permille,
        expected_se_permille
    );

    let forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    assert!(
        super::super::band_confirmation::evaluate_band_confirmation(gain(724), forward, reverse)
            .is_ok()
    );
}

#[test]
fn confirmation_observation_report_rejects_non_representable_mean() {
    let observation = atom(VelocityDirection::Forward, 0, RUNG18_VARIANCE, TARGET);
    assert!(
        confirmation_observation_report(0x33, 0, VelocityDirection::Forward, observation, 1, 0)
            .is_none()
    );
}

#[test]
fn confirmation_evaluated_terminal_report_accepted_carries_masks_and_gain() {
    let plan = plan_band_confirmation(0x44, gain(724)).expect("valid confirmation plan");
    let forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );

    let report =
        confirmation_evaluated_terminal_report(&plan, &forward, &reverse, Some(724), None, 10, 6);

    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::Accepted as u8
    );
    assert_eq!(report.accepted, 1);
    assert_eq!(report.confirmed_p_raw, 724);
    assert_eq!(report.forward_collected_mask, 0b1111);
    assert_eq!(report.forward_eligible_mask, 0b1111);
    assert_eq!(report.forward_included_mask, 0b1111);
    assert_eq!(report.reverse_collected_mask, 0b1111);
    assert!(report.max_relative_se_permille > 0);
    assert!(report.required_relative_se_permille > 0);
    assert_eq!(report.has_safety_fault, 0);
}

#[test]
fn confirmation_evaluated_terminal_report_inconclusive_carries_failure_cause() {
    let plan = plan_band_confirmation(0x44, gain(724)).expect("valid confirmation plan");
    let forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );

    let report = confirmation_evaluated_terminal_report(
        &plan,
        &forward,
        &reverse,
        None,
        Some(BandConfirmationFailureCause::ExcessiveUncertainty),
        10,
        6,
    );

    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::ConfirmationExcessiveUncertainty as u8
    );
    assert_eq!(report.accepted, 0);
    assert_eq!(report.confirmed_p_raw, 0);
}

#[test]
fn confirmation_partial_evidence_terminal_report_carries_the_collected_masks() {
    let plan = plan_band_confirmation(0x44, gain(724)).expect("valid confirmation plan");
    let forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    let reverse = ConfirmationDirectionalInput {
        consensus: DirectionalConsensus {
            classification: crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass::IncompleteRung,
            components: [[0; 2]; 2],
            component_count: 0,
            collected_mask: 0,
            eligible_mask: 0,
            included_mask: 0,
        },
        classes: [VelocityObservationClass::InvalidEvidenceFault; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };

    let report = confirmation_partial_evidence_terminal_report(
        &plan,
        &forward,
        &reverse,
        BreakawayTerminalCause::ConfirmationEvidenceExcluded,
        0,
        10,
        6,
    );

    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::ConfirmationEvidenceExcluded as u8
    );
    assert_eq!(report.accepted, 0);
    assert_eq!(report.confirmed_p_raw, 0);
    assert_eq!(report.has_safety_fault, 0);
    assert_eq!(report.forward_collected_mask, 0b1111);
    assert_eq!(report.forward_eligible_mask, 0b1111);
    assert_eq!(report.forward_included_mask, 0b1111);
    assert!(report.max_relative_se_permille > 0);
    assert_eq!(report.reverse_collected_mask, 0);
    assert_eq!(report.reverse_eligible_mask, 0);
    assert_eq!(report.reverse_included_mask, 0);
}

#[test]
fn confirmation_partial_evidence_terminal_report_carries_the_safety_fault_flag() {
    let plan = plan_band_confirmation(0x44, gain(724)).expect("valid confirmation plan");
    let forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    let reverse = ConfirmationDirectionalInput {
        consensus: DirectionalConsensus {
            classification: crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass::IncompleteRung,
            components: [[0; 2]; 2],
            component_count: 0,
            collected_mask: 0,
            eligible_mask: 0,
            included_mask: 0,
        },
        classes: [VelocityObservationClass::InvalidEvidenceFault; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };

    let report = confirmation_partial_evidence_terminal_report(
        &plan,
        &forward,
        &reverse,
        BreakawayTerminalCause::ConfirmationSafetyFault,
        1,
        10,
        6,
    );

    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::ConfirmationSafetyFault as u8
    );
    assert_eq!(report.has_safety_fault, 1);
    assert_eq!(report.forward_eligible_mask, 0b1111);
}

#[test]
fn confirmation_zero_stroke_terminal_report_zeroes_every_mask() {
    let report = confirmation_zero_stroke_terminal_report(
        0x55,
        0x66,
        8,
        &BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery,
        11,
        0,
    );

    assert_eq!(report.plan_digest, 0x55);
    assert_eq!(report.prior_plan_digest, 0x66);
    assert_eq!(report.family_size, 8);
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::ConfirmationTargetBandUnconfirmableFromDiscovery as u8
    );
    assert_eq!(report.forward_collected_mask, 0);
    assert_eq!(report.reverse_included_mask, 0);
    assert_eq!(report.accepted, 0);
    assert_eq!(report.has_safety_fault, 0);
}

#[test]
fn confirmation_zero_stroke_cause_maps_every_zero_stroke_variant() {
    let cases = [
        (
            BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery,
            BreakawayTerminalCause::ConfirmationTargetBandUnconfirmableFromDiscovery,
        ),
        (
            BandConfirmationOutcome::TargetBandNoTransitionCapableCandidate,
            BreakawayTerminalCause::ConfirmationNoTransitionCapableCandidate,
        ),
        (
            BandConfirmationOutcome::TargetBandNotReachedAtAuthority,
            BreakawayTerminalCause::ConfirmationTargetBandNotReachedAtAuthority,
        ),
        (
            BandConfirmationOutcome::ConfirmationEvidenceExcluded,
            BreakawayTerminalCause::ConfirmationEvidenceExcluded,
        ),
    ];
    for (outcome, expected) in cases {
        let (cause, has_fault) = confirmation_zero_stroke_cause(&outcome);
        assert_eq!(cause, expected, "{outcome:?}");
        assert_eq!(has_fault, 0);
    }

    assert_eq!(
        BreakawayTerminalCause::ConfirmationNoTransitionCapableCandidate as u8,
        27
    );

    for (failure, expected) in [
        (
            BandConfirmationFailureCause::ResponseLocation,
            BreakawayTerminalCause::ConfirmationResponseLocation,
        ),
        (
            BandConfirmationFailureCause::ExcessiveUncertainty,
            BreakawayTerminalCause::ConfirmationExcessiveUncertainty,
        ),
        (
            BandConfirmationFailureCause::IncompleteObservations,
            BreakawayTerminalCause::ConfirmationLossOfConsensus,
        ),
        (
            BandConfirmationFailureCause::CurrentAuthority,
            BreakawayTerminalCause::ConfirmationCurrentAuthority,
        ),
    ] {
        let outcome = BandConfirmationOutcome::TargetBandConfirmationInconclusive(failure);
        let (cause, has_fault) = confirmation_zero_stroke_cause(&outcome);
        assert_eq!(cause, expected, "{failure:?}");
        assert_eq!(has_fault, 0);
    }

    let (cause, has_fault) =
        confirmation_zero_stroke_cause(&BandConfirmationOutcome::SafetyFault {
            error: CommissionError::Spi,
            safety: Some(zero_safety_fault()),
        });
    assert_eq!(cause, BreakawayTerminalCause::ConfirmationSafetyFault);
    assert_eq!(has_fault, 1);
}

#[test]
fn no_rung_below_ceiling_maps_to_cause_29_in_the_confirmation_phase() {
    let outcome = BandConfirmationOutcome::TargetBandNoRungBelowCeiling {
        ceiling: 500,
        excluded: 1,
    };

    let (cause, has_fault) = confirmation_zero_stroke_cause(&outcome);
    let report = campaign_terminal_report(
        BreakawayPhase::ProportionalBandConfirmation,
        cause,
        false,
        0,
        0,
        NominationEvidence::default(),
        12,
        9,
    );

    assert_eq!(
        cause,
        BreakawayTerminalCause::ConfirmationNoRungBelowCeiling
    );
    assert_eq!(has_fault, 0);
    assert_eq!(report.terminal_cause, 29);
    assert_eq!(report.accepted, 0);
    assert_eq!(report.error_code, 0);
}

#[test]
fn no_damped_candidate_maps_to_cause_28_in_the_confirmation_phase() {
    let outcome = BandConfirmationOutcome::TargetBandNoDampedCandidate { excluded: 3 };

    let (cause, has_fault) = confirmation_zero_stroke_cause(&outcome);
    let report = campaign_terminal_report(
        BreakawayPhase::ProportionalBandConfirmation,
        cause,
        false,
        0,
        0,
        NominationEvidence {
            nominated_step_peak_permille: 0,
            overshoot_excluded_rungs: 3,
        },
        12,
        9,
    );

    assert_eq!(cause, BreakawayTerminalCause::ConfirmationNoDampedCandidate);
    assert_eq!(
        BreakawayTerminalCause::ConfirmationNoDampedCandidate as u8,
        28
    );
    assert_eq!(has_fault, 0);
    assert_eq!(
        report.phase,
        BreakawayPhase::ProportionalBandConfirmation as u8
    );
    assert_eq!(report.terminal_cause, 28);
    assert_eq!(report.accepted, 0);
    assert_eq!(report.error_code, 0);
}

#[test]
fn campaign_terminal_report_carries_the_nomination_evidence() {
    let report = campaign_terminal_report(
        BreakawayPhase::ProportionalBandConfirmation,
        BreakawayTerminalCause::ConfirmationResponseLocation,
        false,
        0,
        0,
        NominationEvidence {
            nominated_step_peak_permille: 1_390,
            overshoot_excluded_rungs: 2,
        },
        12,
        9,
    );

    assert_eq!(report.nominated_step_peak_permille, 1_390);
    assert_eq!(report.overshoot_excluded_rungs, 2);
}

#[test]
fn campaign_terminal_report_carries_owning_phase_and_integral_digest() {
    let report = campaign_terminal_report(
        BreakawayPhase::ProportionalBandConfirmation,
        BreakawayTerminalCause::Accepted,
        true,
        0x7788,
        0,
        NominationEvidence::default(),
        12,
        9,
    );

    assert_eq!(
        report.phase,
        BreakawayPhase::ProportionalBandConfirmation as u8
    );
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::Accepted as u8
    );
    assert_eq!(report.accepted, 1);
    assert_eq!(report.integral_plan_digest, 0x7788);
}

#[test]
fn campaign_terminal_report_zeroes_integral_digest_on_failure() {
    let report = campaign_terminal_report(
        BreakawayPhase::BreakawayProbe,
        BreakawayTerminalCause::ProbeAuthorityFailure,
        false,
        0,
        0,
        NominationEvidence::default(),
        12,
        9,
    );

    assert_eq!(report.accepted, 0);
    assert_eq!(report.integral_plan_digest, 0);
}

#[test]
fn campaign_terminal_report_names_the_owning_discovery_phase() {
    let report = campaign_terminal_report(
        BreakawayPhase::ProportionalResponseDiscovery,
        BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
        false,
        0,
        0,
        NominationEvidence::default(),
        13,
        1,
    );

    assert_eq!(
        report.phase,
        BreakawayPhase::ProportionalResponseDiscovery as u8
    );
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan as u8
    );
}

#[test]
fn required_half_width_fraction_matches_the_public_band_edges() {
    let lower = f32::from(CONFIRMATION_BAND_LOWER_PERCENT);
    let upper = f32::from(CONFIRMATION_BAND_UPPER_PERCENT);
    let derived = (upper - lower) / (upper + lower);
    assert!((derived - REQUIRED_HALF_WIDTH_FRACTION).abs() < 1.0e-6);
}
