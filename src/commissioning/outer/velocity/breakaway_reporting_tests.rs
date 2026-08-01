//! Tests for the breakaway-campaign wire-record assembly functions.
//!
//! Numeric fixtures reuse the rung-17/rung-18 measurements from
//! `band_confirmation_tests.rs` (spec section 2) where a physically grounded
//! value matters, so the reported percent/SE fields are checked against the
//! same production formulas the confirmation acceptance gate itself uses.

use foci_tmc4671::pid::PidPGain;

use super::super::band_confirmation::{
    BandConfirmationFailureCause, CONFIRMATION_BAND_LOWER_PERCENT, CONFIRMATION_BAND_UPPER_PERCENT,
    ConfirmationDirectionalInput, DiscoveryObservationAtom, DiscoveryRungNominationBounds,
    plan_band_confirmation,
};
use super::super::breakaway_campaign::{
    AdditiveDiscoveryOutcome, BandConfirmationOutcome, ObservationClassifierInputs,
};
use super::super::breakaway_plan::{
    AdditiveDiscoveryRequest, BreakawayProbeContext, plan_additive_discovery_ladder,
    plan_breakaway_probe,
};
use super::super::breakaway_search::{BreakawayMotionClass, DirectionalBreakaway};
use super::super::sweep_consensus::{CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus};
use super::super::sweep_evidence::VelocityObservationClass;
use super::*;
use crate::autotune::CommissionError;
use crate::commissioning::types::OuterSafetyFault;

fn gain(raw: u16) -> PidPGain {
    PidPGain::new(raw).expect("raw gain within PidPGain range")
}

/// Canonical acquisition context for probe-plan fixtures; exact values are
/// arbitrary but fixed.
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
/// Rung 18 (gain 724): relative SE exactly 1.022% at mean 750,000 (`r = 75%`).
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

// --- Probe plan --------------------------------------------------------

#[test]
fn probe_plan_report_carries_plan_digest_and_endpoints() {
    let p_start = gain(1);
    let p_top = gain(1024);
    let plan = plan_breakaway_probe(p_start, p_top, probe_context()).expect("valid probe plan");

    let report = probe_plan_report(&plan, p_start, p_top, 7, 3);

    assert_eq!(report.run_sequence, 7);
    assert_eq!(report.evidence_sequence, 3);
    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.family_size, plan.confidence_family_size);
    assert_eq!(report.max_observations, plan.maximum_observations);
    assert_eq!(report.search_count, plan.search_count);
    assert_eq!(report.p_start_raw, 1);
    assert_eq!(report.p_top_raw, 1024);
}

// --- Directional breakaway ----------------------------------------------

#[test]
fn directional_breakaway_report_resolves_gains_from_grid_indices() {
    let plan =
        plan_breakaway_probe(gain(1), gain(1024), probe_context()).expect("valid probe plan");
    let breakaway = DirectionalBreakaway {
        inert_index: Some(2),
        moving_index: 3,
        observations: 5,
    };

    let report = directional_breakaway_report(&plan, VelocityDirection::Forward, &breakaway, 1, 0);

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(
        report.direction,
        direction_report_code(VelocityDirection::Forward)
    );
    assert_eq!(report.inert_present, 1);
    assert_eq!(report.inert_p_raw, plan.search_gains[2].raw());
    assert_eq!(report.moving_p_raw, plan.search_gains[3].raw());
    assert_eq!(report.observations, 5);
}

#[test]
fn directional_breakaway_report_clears_inert_when_clamped_at_breakaway() {
    let plan =
        plan_breakaway_probe(gain(1), gain(1024), probe_context()).expect("valid probe plan");
    let breakaway = DirectionalBreakaway {
        inert_index: None,
        moving_index: 0,
        observations: 2,
    };

    let report = directional_breakaway_report(&plan, VelocityDirection::Reverse, &breakaway, 1, 1);

    assert_eq!(report.inert_present, 0);
    assert_eq!(report.inert_p_raw, 0);
    assert_eq!(report.moving_p_raw, plan.search_gains[0].raw());
}

// --- Probe terminal ------------------------------------------------------

#[test]
fn probe_terminal_cause_maps_every_non_found_variant() {
    use super::super::breakaway_campaign::BreakawayProbeOutcome;

    let cases = [
        (
            BreakawayProbeOutcome::NoMotionAtAuthority,
            BreakawayTerminalCause::ProbeNoMotionAtAuthority,
        ),
        (
            BreakawayProbeOutcome::UnstableBreakaway,
            BreakawayTerminalCause::ProbeUnstableBreakaway,
        ),
        (
            BreakawayProbeOutcome::DirectionalBreakawayDisagreement,
            BreakawayTerminalCause::ProbeDirectionalBreakawayDisagreement,
        ),
        (
            BreakawayProbeOutcome::ProbeEvidenceExcluded,
            BreakawayTerminalCause::ProbeEvidenceExcluded,
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
    assert_eq!(cause, BreakawayTerminalCause::ProbeSafetyFault);
    assert_eq!(has_fault, 0);
}

#[test]
fn probe_terminal_report_carries_plan_identity_and_cause() {
    use super::super::breakaway_campaign::BreakawayProbeOutcome;

    let plan =
        plan_breakaway_probe(gain(1), gain(1024), probe_context()).expect("valid probe plan");
    let report = probe_terminal_report(&plan, &BreakawayProbeOutcome::AuthorityFailure, 9, 4);

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.family_size, plan.confidence_family_size);
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::ProbeAuthorityFailure as u8
    );
    assert_eq!(report.has_safety_fault, 0);
}

// --- Discovery plan and ladder -------------------------------------------

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
        workflow_envelope_ms: 300_000,
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
        super::super::combined_plan::BREAKAWAY_STAGE_B_SCHEMA_REVISION
    );
    assert_eq!(report.family_size, plan.confidence_family_size);
    assert_eq!(report.rung_count, plan.rung_count);
    assert_eq!(report.floor_p_raw, 64);
    assert_eq!(report.floor_origin, 0);
    assert_eq!(report.breakaway_p_raw, 91);
    assert_eq!(report.ceiling_p_raw, 1024);
    assert_eq!(report.maximum_workflow_ms, request.workflow_envelope_ms);
    assert!(report.first_additive_step_raw > 0);

    // The reconstructed step matches the realized gap between the breakaway
    // rung and the rung immediately after it in the resolved ladder.
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
            usable_torque_limit: 66,
            hard_torque_limit: 77,
            disturbance_present: true,
            disturbance_low_q16: -88,
            disturbance_high_q16: 99,
        },
    };
    let report = raw_observation_report(
        0x1122_3344_5566_7788,
        BreakawayPhase::Confirmation,
        &collected,
        7,
        9,
    );

    // The raw atom's family-free inputs pass through byte-for-byte -- this is
    // what lets an offline decoder recompute r/SE/L/U/M_i and the [70, 80]
    // containment itself rather than trusting the firmware's reduced bounds.
    assert_eq!(report.run_sequence, 7);
    assert_eq!(report.evidence_sequence, 9);
    assert_eq!(report.plan_digest, 0x1122_3344_5566_7788);
    assert_eq!(report.phase, BreakawayPhase::Confirmation as u8);
    assert_eq!(report.rung_index, 0);
    assert_eq!(report.slot_index, 6);
    assert_eq!(
        report.direction,
        direction_report_code(VelocityDirection::Reverse)
    );
    assert_eq!(report.mean_rate_q, -RUNG18_MEAN);
    assert_eq!(report.variance_of_mean_q2, RUNG18_VARIANCE);
    assert_eq!(report.target_rate_q, TARGET);
    // The frozen classify/consensus inputs also pass through verbatim, so the
    // offline decoder derives the consensus masks itself.
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
}

// --- Discovery margins -----------------------------------------------------

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
    // margin = min(71 - 70, 80 - 79) = min(1, 1) = 1 percent
    assert_eq!(report.margin_percent_milli, 1_000);
    // half-width = (79 - 71) / 2 = 4 percent
    assert_eq!(report.half_width_percent_milli, 4_000);
    assert_eq!(report.adjacent_rate_step_permille, 0);
}

#[test]
fn discovery_rung_margin_report_computes_adjacent_step_from_previous_midpoint() {
    let bounds = [
        nominatable_bounds(362, 71.0, 79.0), // midpoint 75.0
        nominatable_bounds(512, 73.0, 77.0), // midpoint 75.0
    ];

    let second = discovery_rung_margin_report(&bounds, 1, 0x11, 3, 2).expect("nominatable rung");

    assert_eq!(second.adjacent_rate_step_permille, 0);

    let bounds_shifted = [
        nominatable_bounds(362, 71.0, 79.0), // midpoint 75.0
        nominatable_bounds(512, 76.0, 80.0), // midpoint 78.0
    ];
    let shifted =
        discovery_rung_margin_report(&bounds_shifted, 1, 0x11, 3, 2).expect("nominatable rung");
    assert_eq!(shifted.adjacent_rate_step_permille, 3_000);
}

#[test]
fn discovery_rung_margin_report_rejects_non_nominatable_and_out_of_range() {
    let bounds = [DiscoveryRungNominationBounds {
        p: gain(362),
        lower_percent: 0.0,
        upper_percent: 0.0,
        nominatable: false,
    }];

    assert!(discovery_rung_margin_report(&bounds, 0, 0x11, 3, 2).is_none());
    assert!(discovery_rung_margin_report(&bounds, 5, 0x11, 3, 2).is_none());
}

// --- Rung-zero diagnostic --------------------------------------------------

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

// --- Discovery terminal -----------------------------------------------------

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
            AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded,
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
    let via_outcome = discovery_terminal_report(&plan, &outcome, 6, 2);
    let via_fields = discovery_terminal_fields(
        &plan,
        BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
        0,
        0,
        6,
        2,
    );

    assert_eq!(via_outcome, via_fields);
}

// --- Confirmation plan ------------------------------------------------------

#[test]
fn confirmation_plan_report_carries_band_and_margin() {
    let plan = plan_band_confirmation(0x22, gain(724)).expect("valid confirmation plan");
    let nomination = NominatedConfirmationCandidate {
        p: gain(724),
        margin_percent: 4.5,
    };

    let report = confirmation_plan_report(&plan, &nomination, 8, 0);

    assert_eq!(report.plan_digest, plan.plan_digest);
    assert_eq!(report.prior_plan_digest, 0x22);
    assert_eq!(report.family_size, plan.family_size);
    assert_eq!(report.candidate_p_raw, 724);
    assert_eq!(report.band_lower_percent, CONFIRMATION_BAND_LOWER_PERCENT);
    assert_eq!(report.band_upper_percent, CONFIRMATION_BAND_UPPER_PERCENT);
    assert_eq!(report.nominated_margin_percent_milli, 4_500);
}

// --- Confirmation observations ----------------------------------------------

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

    // response_percent = 75%, relative SE = sqrt(58_752_225) / 750_000 =
    // 7665 / 750_000 = 1.022% exactly. h = m(8) * SE, m(8) = 3.177104
    // (`CONFIDENCE_MULTIPLIERS_95_DF15[7]`, family-8 index).
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

    // The same atom, judged by the real acceptance gate at family 8 with
    // three identical siblings per direction, must accept: this is exactly
    // the physical claim the reported interval makes.
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
    // Forward direction with a non-positive mean is not representable.
    let observation = atom(VelocityDirection::Forward, 0, RUNG18_VARIANCE, TARGET);
    assert!(
        confirmation_observation_report(0x33, 0, VelocityDirection::Forward, observation, 1, 0)
            .is_none()
    );
}

// --- Confirmation terminal ----------------------------------------------------

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

/// FIX D(b): `ConfirmationEvidenceExcluded` after earlier strokes already
/// folded evidence must report that partial evidence, not the empty fixture
/// [`confirmation_zero_stroke_terminal_report`] reserves for the outcomes
/// reachable before any stroke ever ran.
#[test]
fn confirmation_partial_evidence_terminal_report_carries_the_collected_masks() {
    let plan = plan_band_confirmation(0x44, gain(724)).expect("valid confirmation plan");
    // Only the forward direction ran a stroke before exclusion; reverse never
    // started, so its masks are honestly still zero.
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
    // The forward direction's real evidence survives, unlike the all-zero
    // zero-stroke path.
    assert_eq!(report.forward_collected_mask, 0b1111);
    assert_eq!(report.forward_eligible_mask, 0b1111);
    assert_eq!(report.forward_included_mask, 0b1111);
    assert!(report.max_relative_se_permille > 0);
    // The reverse direction genuinely never ran, so its masks are still zero
    // -- distinct from the forward direction's real partial evidence.
    assert_eq!(report.reverse_collected_mask, 0);
    assert_eq!(report.reverse_eligible_mask, 0);
    assert_eq!(report.reverse_included_mask, 0);
}

/// A mid-block `SafetyFault`'s retained-evidence flag must survive the
/// partial-evidence path exactly like it does on the zero-stroke path
/// ([`confirmation_zero_stroke_terminal_report_zeroes_every_mask`] and
/// friends): `has_safety_fault` is caller-supplied, not hardcoded, so it is
/// not silently dropped when the reporter switches from the empty fixture to
/// real collected evidence.
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
    // The forward direction's real evidence still survives on the safety-fault
    // path, not just the exclusion path.
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
            BandConfirmationFailureCause::LossOfConsensus,
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

// --- Campaign terminal --------------------------------------------------------

#[test]
fn campaign_terminal_report_carries_owning_phase_and_stage_c_digest() {
    let report = campaign_terminal_report(
        BreakawayPhase::Confirmation,
        BreakawayTerminalCause::Accepted,
        true,
        0x7788,
        12,
        9,
    );

    assert_eq!(report.phase, BreakawayPhase::Confirmation as u8);
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::Accepted as u8
    );
    assert_eq!(report.accepted, 1);
    assert_eq!(report.stage_c_plan_digest, 0x7788);
}

#[test]
fn campaign_terminal_report_zeroes_stage_c_digest_on_failure() {
    let report = campaign_terminal_report(
        BreakawayPhase::Probe,
        BreakawayTerminalCause::ProbeAuthorityFailure,
        false,
        0,
        12,
        9,
    );

    assert_eq!(report.accepted, 0);
    assert_eq!(report.stage_c_plan_digest, 0);
}

#[test]
fn campaign_terminal_report_names_the_owning_discovery_phase() {
    let report = campaign_terminal_report(
        BreakawayPhase::Discovery,
        BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
        false,
        0,
        13,
        1,
    );

    assert_eq!(report.phase, BreakawayPhase::Discovery as u8);
    assert_eq!(
        report.terminal_cause,
        BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan as u8
    );
}

// --- Required-SE derivation self-check ---------------------------------------

#[test]
fn required_half_width_fraction_matches_the_public_band_edges() {
    let lower = f32::from(CONFIRMATION_BAND_LOWER_PERCENT);
    let upper = f32::from(CONFIRMATION_BAND_UPPER_PERCENT);
    let derived = (upper - lower) / (upper + lower);
    assert!((derived - REQUIRED_HALF_WIDTH_FRACTION).abs() < 1.0e-6);
}
