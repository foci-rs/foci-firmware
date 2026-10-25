use super::super::recovery::commanded_side_lower_rate_q;
use super::super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
};
use super::super::sweep_evidence::{VelocityObservationClass, direction_report_code};
use super::super::sweep_plan::{MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US, ZERO_SETTLE_US};
use super::{
    BandConfirmationContractError, BandConfirmationFailureCause, BandConfirmationPlan,
    BandConfirmationPlanError, CONFIRMATION_ACCEPTANCE_RULE_ALL_EIGHT_CONTAINED,
    CONFIRMATION_CAPTURE_PROFILE_EXISTING, CONFIRMATION_FAMILY_SIZE,
    CONFIRMATION_OBSERVATIONS_PER_DIRECTION, CONFIRMATION_SLOT_ORDER, ConfirmationDirectionalInput,
    DiscoveryObservationAtom, DiscoveryRungNominationBounds, DiscoveryRungNominationInput,
    NOMINATION_SCREEN_C_SIGMA, NominationRejection, TargetBandConfirmationInconclusive,
    TargetBandUnconfirmableFromDiscovery, commanded_side_mean_rate_q, evaluate_band_confirmation,
    nominate_confirmation_candidate, nominate_confirmation_candidate_from_bounds,
    nominate_transition_capable_candidate_from_bounds, observation_statistic,
    plan_band_confirmation, rung_confirmable_margin_percent,
    rung_directional_pooled_bounds_percent, rung_nomination_bounds, rung_response_bounds_percent,
    rung_step_peak_permille, validate_fixed_contract,
};
use crate::autotune::velocity_measurement::{VelocityDirection, confidence_multiplier_95_df15};
use foci_tmc4671::pid::PidPGain;

const TARGET: i64 = 1_000_000;
const RUNG18_MEAN: i64 = 750_000;
const RUNG18_VARIANCE: u128 = 58_752_225;
const RUNG17_MEAN: i64 = 750_000;
const RUNG17_VARIANCE: u128 = 207_129_664;

fn gain(raw: u16) -> PidPGain {
    PidPGain::new(raw).expect("raw gain within PidPGain range")
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

fn variance_for_relative_se(commanded_magnitude: i64, relative_se: f64) -> u128 {
    let standard_error_of_mean = commanded_magnitude as f64 * relative_se;
    (standard_error_of_mean * standard_error_of_mean).round() as u128
}

fn full_consensus() -> DirectionalConsensus {
    DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
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

fn uniform_rung(
    p_raw: u16,
    forward_mean_rate_q: i64,
    variance_of_mean_q2: u128,
    target_rate_q: i64,
) -> DiscoveryRungNominationInput {
    DiscoveryRungNominationInput {
        p: gain(p_raw),
        forward_consensus: full_consensus(),
        forward_observations: uniform_observations(
            VelocityDirection::Forward,
            forward_mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
        ),
        reverse_consensus: full_consensus(),
        reverse_observations: uniform_observations(
            VelocityDirection::Reverse,
            -forward_mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
        ),
    }
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

fn approx(actual: f32, expected: f32, epsilon: f32) -> bool {
    (actual - expected).abs() <= epsilon
}

#[test]
fn family_eight_multiplier_matches_spec() {
    assert_eq!(
        confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE),
        Ok(3.177104)
    );
}

#[test]
fn c_sigma_matches_spec() {
    assert_eq!(NOMINATION_SCREEN_C_SIGMA, 1.4373055);
}

#[test]
fn forward_observation_requires_strictly_positive_mean() {
    assert_eq!(
        commanded_side_mean_rate_q(VelocityDirection::Forward, 100),
        Some(100)
    );
    assert_eq!(
        commanded_side_mean_rate_q(VelocityDirection::Forward, 0),
        None
    );
    assert_eq!(
        commanded_side_mean_rate_q(VelocityDirection::Forward, -100),
        None
    );
}

#[test]
fn reverse_observation_requires_strictly_negative_mean() {
    assert_eq!(
        commanded_side_mean_rate_q(VelocityDirection::Reverse, -100),
        Some(100)
    );
    assert_eq!(
        commanded_side_mean_rate_q(VelocityDirection::Reverse, 0),
        None
    );
    assert_eq!(
        commanded_side_mean_rate_q(VelocityDirection::Reverse, 100),
        None
    );
}

#[test]
fn observation_statistic_rejects_nonrepresentable_means() {
    assert!(observation_statistic(atom(VelocityDirection::Forward, -100, 1, TARGET)).is_err());
    assert!(observation_statistic(atom(VelocityDirection::Forward, 0, 1, TARGET)).is_err());
    assert!(observation_statistic(atom(VelocityDirection::Reverse, 100, 1, TARGET)).is_err());
}

#[test]
fn rung_eighteen_class_noise_is_nominatable_at_family_eight() {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE).unwrap();
    let rung = uniform_rung(724, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    let margin = rung_confirmable_margin_percent(&rung, multiplier).expect("nominatable");
    assert!(margin > 0.0, "expected positive margin, got {margin}");
    assert!(approx(margin, 1.4998, 0.01), "margin was {margin}");
}

#[test]
fn rung_seventeen_class_noise_is_excluded_once_variance_inflation_applies() {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE).unwrap();
    let rung = uniform_rung(512, RUNG17_MEAN, RUNG17_VARIANCE, TARGET);
    let margin = rung_confirmable_margin_percent(&rung, multiplier)
        .expect("nominatable in principle, but excluded on margin");
    assert!(
        margin < 0.0,
        "expected c_sigma to push rung 17 out once inflated, got {margin}"
    );
}

#[test]
fn nomination_selects_the_confirmable_rung_and_skips_the_excluded_one() {
    let rungs = [
        uniform_rung(512, RUNG17_MEAN, RUNG17_VARIANCE, TARGET),
        uniform_rung(724, RUNG18_MEAN, RUNG18_VARIANCE, TARGET),
    ];
    let nominated = nominate_confirmation_candidate(&rungs).expect("rung-18-class noise nominates");
    assert_eq!(nominated.p, gain(724));
    assert!(nominated.margin_percent > 0.0);
}

#[test]
fn nomination_selects_the_highest_admissible_rung_not_the_equal_noise_maximizer() {
    let relative_se = 0.01_f64;
    let candidates = [70_u16, 72, 74, 75, 76, 78, 80];
    let rungs: heapless::Vec<DiscoveryRungNominationInput, 8> = candidates
        .iter()
        .map(|&response_percent| {
            let mean = TARGET * i64::from(response_percent) / 100;
            let variance = variance_for_relative_se(mean, relative_se);
            uniform_rung(response_percent * 10, mean, variance, TARGET)
        })
        .collect();

    let nominated = nominate_confirmation_candidate(&rungs).expect("some rungs are admissible");
    assert_eq!(nominated.p, gain(760));
}

#[test]
fn nomination_selects_the_higher_p_rung_even_when_it_is_noisier() {
    let noisy_mean = 750_000;
    let noisy_variance = variance_for_relative_se(noisy_mean, 0.012);
    let noisy_rung = uniform_rung(700, noisy_mean, noisy_variance, TARGET);

    let quiet_mean = 730_000;
    let quiet_variance = variance_for_relative_se(quiet_mean, 0.004);
    let quiet_rung = uniform_rung(690, quiet_mean, quiet_variance, TARGET);

    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE).unwrap();
    let noisy_margin = rung_confirmable_margin_percent(&noisy_rung, multiplier).unwrap();
    let quiet_margin = rung_confirmable_margin_percent(&quiet_rung, multiplier).unwrap();
    assert!(
        quiet_margin > noisy_margin,
        "quiet={quiet_margin} noisy={noisy_margin}"
    );
    assert!(
        noisy_margin >= 0.0,
        "noisy rung must be band-admissible: {noisy_margin}"
    );
    assert!(
        quiet_margin >= 0.0,
        "quiet rung must be band-admissible: {quiet_margin}"
    );

    let nominated =
        nominate_confirmation_candidate(&[noisy_rung, quiet_rung]).expect("nominatable");
    assert_eq!(nominated.p, gain(700));
}

#[test]
fn nomination_selects_the_higher_p_rung_regardless_of_input_order() {
    let low_p_rung = uniform_rung(300, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    let high_p_rung = uniform_rung(500, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);

    let nominated =
        nominate_confirmation_candidate(&[low_p_rung, high_p_rung]).expect("nominatable");
    assert_eq!(nominated.p, gain(500));

    let nominated_reordered =
        nominate_confirmation_candidate(&[high_p_rung, low_p_rung]).expect("nominatable");
    assert_eq!(nominated_reordered.p, gain(500));
}

#[test]
fn nomination_selects_highest_band_admissible_rung_not_best_margin() {
    let center = DiscoveryRungNominationBounds {
        p: gain(400),
        nominatable: true,
        lower_percent: 74.0,
        upper_percent: 76.0,
        step_peak_permille: 0,
    };
    let top = DiscoveryRungNominationBounds {
        p: gain(480),
        nominatable: true,
        lower_percent: 71.0,
        upper_percent: 79.0,
        step_peak_permille: 0,
    };
    let out_of_band = DiscoveryRungNominationBounds {
        p: gain(520),
        nominatable: true,
        lower_percent: 69.0,
        upper_percent: 78.0,
        step_peak_permille: 0,
    };

    let nominated = nominate_confirmation_candidate_from_bounds(&[center, top, out_of_band])
        .expect("a band-admissible rung exists");
    assert_eq!(nominated.p, gain(480));
}

#[test]
fn transition_ceiling_excludes_the_over_ceiling_top_rung() {
    let capable = DiscoveryRungNominationBounds {
        p: gain(863),
        nominatable: true,
        lower_percent: 72.0,
        upper_percent: 78.0,
        step_peak_permille: 0,
    };
    let over_ceiling = DiscoveryRungNominationBounds {
        p: gain(925),
        nominatable: true,
        lower_percent: 72.0,
        upper_percent: 78.0,
        step_peak_permille: 0,
    };
    let nominated =
        nominate_transition_capable_candidate_from_bounds(&[capable, over_ceiling], 897)
            .expect("a transition-capable in-band rung exists");
    assert_eq!(nominated.p, gain(863));
}

#[test]
fn transition_ceiling_rejects_when_every_in_band_rung_is_over_ceiling() {
    let over_ceiling = DiscoveryRungNominationBounds {
        p: gain(925),
        nominatable: true,
        lower_percent: 72.0,
        upper_percent: 78.0,
        step_peak_permille: 0,
    };
    assert_eq!(
        nominate_transition_capable_candidate_from_bounds(&[over_ceiling], 897),
        Err(NominationRejection::NoTransitionCapableCandidate)
    );
}

#[test]
fn transition_ceiling_reports_unconfirmable_when_no_rung_is_band_admissible() {
    let out_of_band = DiscoveryRungNominationBounds {
        p: gain(863),
        nominatable: true,
        lower_percent: 69.0,
        upper_percent: 78.0,
        step_peak_permille: 0,
    };
    assert_eq!(
        nominate_transition_capable_candidate_from_bounds(&[out_of_band], 897),
        Err(NominationRejection::Unconfirmable)
    );
}

#[test]
fn an_unconstrained_ceiling_nominates_the_highest_rung_like_the_primitive() {
    let lower = DiscoveryRungNominationBounds {
        p: gain(863),
        nominatable: true,
        lower_percent: 72.0,
        upper_percent: 78.0,
        step_peak_permille: 0,
    };
    let top = DiscoveryRungNominationBounds {
        p: gain(925),
        nominatable: true,
        lower_percent: 72.0,
        upper_percent: 78.0,
        step_peak_permille: 0,
    };
    let nominated = nominate_transition_capable_candidate_from_bounds(&[lower, top], u16::MAX)
        .expect("the highest in-band rung");
    assert_eq!(nominated.p, gain(925));
}

#[test]
fn zero_or_wrong_sign_mean_disqualifies_the_whole_rung() {
    let mut zero_mean_rung = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    zero_mean_rung.forward_observations[0] =
        Some(atom(VelocityDirection::Forward, 0, RUNG18_VARIANCE, TARGET));
    assert_eq!(
        nominate_confirmation_candidate(&[zero_mean_rung]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );

    let mut wrong_sign_rung = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    wrong_sign_rung.forward_observations[0] = Some(atom(
        VelocityDirection::Forward,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    ));
    assert_eq!(
        nominate_confirmation_candidate(&[wrong_sign_rung]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );
}

#[test]
fn non_eligible_consensus_cannot_nominate() {
    let mut ambiguous_forward = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    ambiguous_forward.forward_consensus.classification =
        DirectionalConsensusClass::AmbiguousConsensus;
    assert_eq!(
        nominate_confirmation_candidate(&[ambiguous_forward]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );

    let mut insufficient_reverse = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    insufficient_reverse.reverse_consensus.classification =
        DirectionalConsensusClass::InsufficientConsensus;
    assert_eq!(
        nominate_confirmation_candidate(&[insufficient_reverse]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );
}

#[test]
fn missing_included_observation_disqualifies_the_rung() {
    let mut rung = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    rung.forward_observations[2] = None;
    assert_eq!(
        nominate_confirmation_candidate(&[rung]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );
}

#[test]
fn nomination_envelope_ignores_observations_outside_the_included_mask() {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE).unwrap();
    let baseline = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    let baseline_margin = rung_confirmable_margin_percent(&baseline, multiplier).unwrap();

    let mut rung = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    rung.forward_observations[3] = Some(atom(VelocityDirection::Forward, 1, 1, TARGET));

    rung.forward_consensus.included_mask = 0b0111;
    let margin_excluded = rung_confirmable_margin_percent(&rung, multiplier).expect("nominatable");
    assert!(approx(margin_excluded, baseline_margin, 1e-3));

    rung.forward_consensus.included_mask = 0b1111;
    let margin_included = rung_confirmable_margin_percent(&rung, multiplier).expect("nominatable");
    assert!(
        margin_included < margin_excluded,
        "including the outlier must widen the envelope and shrink the margin"
    );
}

#[test]
fn confirmation_requires_complete_masks_in_both_directions() {
    let mut incomplete_forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    incomplete_forward.consensus.eligible_mask = 0b0111;
    incomplete_forward.consensus.included_mask = 0b0111;
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );

    let result = evaluate_band_confirmation(gain(724), incomplete_forward, reverse);
    assert_eq!(
        result,
        Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::IncompleteObservations
        ))
    );
}

#[test]
fn current_headroom_exclusion_is_reported_as_current_authority() {
    let mut forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    forward.consensus.eligible_mask = 0b0111;
    forward.consensus.included_mask = 0b0111;
    forward.classes[3] = VelocityObservationClass::CurrentHeadroom;
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );

    let result = evaluate_band_confirmation(gain(724), forward, reverse);
    assert_eq!(
        result,
        Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::CurrentAuthority
        ))
    );
}

#[test]
fn other_exclusion_reasons_are_reported_as_incomplete_observations() {
    let mut forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    forward.consensus.eligible_mask = 0b0111;
    forward.consensus.included_mask = 0b0111;
    forward.classes[3] = VelocityObservationClass::HardClipFault;
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );

    let result = evaluate_band_confirmation(gain(724), forward, reverse);
    assert_eq!(
        result,
        Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::IncompleteObservations
        ))
    );
}

#[test]
fn confirmation_accepts_a_split_disturbance_consensus_when_all_eight_are_contained() {
    const CAPTURED_TARGET: i64 = 5_800_000;
    const FORWARD_CAPTURE: [(i64, u128); CONSENSUS_OBSERVATIONS_PER_DIRECTION] = [
        (4_296_875, 276_693_012),
        (4_371_093, 178_161_750),
        (4_390_625, 113_933_569),
        (4_367_187, 158_691_350),
    ];
    const REVERSE_CAPTURE: [(i64, u128); CONSENSUS_OBSERVATIONS_PER_DIRECTION] = [
        (-4_324_218, 202_435_151),
        (-4_375_000, 2_083_333_368),
        (-4_472_656, 422_167_472),
        (-4_390_625, 2_067_053_436),
    ];

    fn captured_direction(
        direction: VelocityDirection,
        capture: [(i64, u128); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        components: [[i32; 2]; 2],
    ) -> ConfirmationDirectionalInput {
        ConfirmationDirectionalInput {
            consensus: DirectionalConsensus {
                classification: DirectionalConsensusClass::AmbiguousConsensus,
                components,
                component_count: 2,
                collected_mask: 0b1111,
                eligible_mask: 0b1111,
                included_mask: 0,
            },
            classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            observations: capture.map(|(mean_rate_q, variance_of_mean_q2)| {
                Some(atom(
                    direction,
                    mean_rate_q,
                    variance_of_mean_q2,
                    CAPTURED_TARGET,
                ))
            }),
        }
    }

    let forward = captured_direction(
        VelocityDirection::Forward,
        FORWARD_CAPTURE,
        [[12_076_487, 12_514_347], [12_574_937, 12_757_354]],
    );
    let reverse = captured_direction(
        VelocityDirection::Reverse,
        REVERSE_CAPTURE,
        [[-13_188_038, -12_404_134], [-12_075_051, -10_943_001]],
    );

    let result = evaluate_band_confirmation(gain(2258), forward, reverse);

    assert_eq!(
        result.map(|acceptance| acceptance.nominated_p()),
        Ok(gain(2258))
    );
}

#[test]
fn confirmation_accepts_exact_boundary_responses() {
    let at_lower_edge_forward = confirmation_input(VelocityDirection::Forward, 700_000, 0, TARGET);
    let at_lower_edge_reverse = confirmation_input(VelocityDirection::Reverse, -700_000, 0, TARGET);
    assert!(
        evaluate_band_confirmation(gain(400), at_lower_edge_forward, at_lower_edge_reverse).is_ok()
    );

    let at_upper_edge_forward = confirmation_input(VelocityDirection::Forward, 800_000, 0, TARGET);
    let at_upper_edge_reverse = confirmation_input(VelocityDirection::Reverse, -800_000, 0, TARGET);
    assert!(
        evaluate_band_confirmation(gain(400), at_upper_edge_forward, at_upper_edge_reverse).is_ok()
    );
}

#[test]
fn confirmation_rejects_one_unit_escapes_at_each_edge() {
    let below_lower_edge = confirmation_input(VelocityDirection::Forward, 699_999, 0, TARGET);
    let at_lower_edge = confirmation_input(VelocityDirection::Reverse, -700_000, 0, TARGET);
    assert_eq!(
        evaluate_band_confirmation(gain(400), below_lower_edge, at_lower_edge),
        Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::ResponseLocation
        ))
    );

    let at_lower_edge_forward = confirmation_input(VelocityDirection::Forward, 700_000, 0, TARGET);
    let above_upper_edge = confirmation_input(VelocityDirection::Reverse, -800_001, 0, TARGET);
    assert_eq!(
        evaluate_band_confirmation(gain(400), at_lower_edge_forward, above_upper_edge),
        Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::ResponseLocation
        ))
    );
}

#[test]
fn confirmation_reports_excessive_uncertainty_when_containment_is_structurally_impossible() {
    let huge_variance = variance_for_relative_se(750_000, 0.05);
    let forward = confirmation_input(VelocityDirection::Forward, 750_000, huge_variance, TARGET);
    let reverse = confirmation_input(VelocityDirection::Reverse, -750_000, huge_variance, TARGET);
    assert_eq!(
        evaluate_band_confirmation(gain(724), forward, reverse),
        Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::ExcessiveUncertainty
        ))
    );
}

#[test]
fn rung_seventeen_class_noise_passes_real_confirmation_at_its_own_narrow_window() {
    let forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG17_MEAN,
        RUNG17_VARIANCE,
        TARGET,
    );
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG17_MEAN,
        RUNG17_VARIANCE,
        TARGET,
    );
    let result = evaluate_band_confirmation(gain(512), forward, reverse);
    assert!(
        result.is_ok(),
        "expected acceptance within the narrow rung-17 confirmable window, got {result:?}"
    );
}

#[test]
fn rung_seventeen_class_noise_fails_confirmation_just_outside_its_narrow_window() {
    let mean_at_seventy_four_point_five_percent = TARGET * 745 / 1000;
    let forward = confirmation_input(
        VelocityDirection::Forward,
        mean_at_seventy_four_point_five_percent,
        RUNG17_VARIANCE,
        TARGET,
    );
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG17_MEAN,
        RUNG17_VARIANCE,
        TARGET,
    );
    assert_eq!(
        evaluate_band_confirmation(gain(512), forward, reverse),
        Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::ResponseLocation
        ))
    );
}

#[test]
fn confirmation_family_size_is_always_eight_never_four_or_sixteen() {
    let plan = plan_band_confirmation(42, gain(724)).unwrap();
    assert_eq!(plan.family_size, CONFIRMATION_FAMILY_SIZE);
    assert_eq!(plan.family_size, 8);
    assert_ne!(plan.family_size, 4);
    assert_ne!(plan.family_size, 16);
}

#[test]
fn confirmation_acceptance_uses_the_family_eight_multiplier_not_four_or_sixteen() {
    let family_four = confidence_multiplier_95_df15(4).unwrap();
    let family_eight = confidence_multiplier_95_df15(8).unwrap();
    let family_sixteen = confidence_multiplier_95_df15(16).unwrap();
    assert_ne!(family_eight, family_four);
    assert_ne!(family_eight, family_sixteen);

    let statistic = observation_statistic(atom(
        VelocityDirection::Forward,
        RUNG17_MEAN,
        RUNG17_VARIANCE,
        TARGET,
    ))
    .expect("representable rung-17-class observation");

    let eight_half_width = family_eight * statistic.relative_standard_error;
    let sixteen_half_width = family_sixteen * statistic.relative_standard_error;
    assert!(
        eight_half_width <= 1.0 / 15.0,
        "family 8 must keep rung 17 containable, half-width {eight_half_width}"
    );
    assert!(
        sixteen_half_width > eight_half_width,
        "family 16 must widen every interval relative to family 8"
    );

    let forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG17_MEAN,
        RUNG17_VARIANCE,
        TARGET,
    );
    let reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG17_MEAN,
        RUNG17_VARIANCE,
        TARGET,
    );
    assert!(evaluate_band_confirmation(gain(512), forward, reverse).is_ok());
}

#[test]
fn empty_discovery_ladder_yields_zero_stroke_outcome() {
    assert_eq!(
        nominate_confirmation_candidate(&[]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );
}

#[test]
fn no_confirmable_rung_yields_zero_stroke_outcome_and_blocks_plan_construction() {
    let rungs = [
        uniform_rung(500, RUNG17_MEAN, RUNG17_VARIANCE, TARGET),
        uniform_rung(600, RUNG17_MEAN, RUNG17_VARIANCE, TARGET),
    ];
    assert_eq!(
        nominate_confirmation_candidate(&rungs),
        Err(TargetBandUnconfirmableFromDiscovery)
    );

    assert_eq!(
        plan_band_confirmation(1, PidPGain::ZERO),
        Err(BandConfirmationPlanError::InvalidRequest)
    );
}

#[test]
fn confirmation_evaluation_carries_no_hidden_state_across_independent_calls() {
    let failing_forward = confirmation_input(VelocityDirection::Forward, 699_999, 0, TARGET);
    let failing_reverse = confirmation_input(VelocityDirection::Reverse, -700_000, 0, TARGET);
    let first = evaluate_band_confirmation(gain(400), failing_forward, failing_reverse);
    assert!(first.is_err());

    let passing_forward = confirmation_input(
        VelocityDirection::Forward,
        RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    let passing_reverse = confirmation_input(
        VelocityDirection::Reverse,
        -RUNG18_MEAN,
        RUNG18_VARIANCE,
        TARGET,
    );
    let second = evaluate_band_confirmation(gain(724), passing_forward, passing_reverse);
    let accepted =
        second.expect("rung-18-class evidence confirms within the 70-80% band, as above");
    assert_eq!(accepted.nominated_p(), gain(724));
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");
    let relative_se = 7_665.0_f32 / 750_000.0;
    let half_width_fraction = multiplier * relative_se;
    let magnitude_low = libm::roundf(750_000.0 * (1.0 - half_width_fraction)) as i32;
    let magnitude_high = libm::roundf(750_000.0 * (1.0 + half_width_fraction)) as i32;
    assert_eq!(
        accepted.held_out_response_intervals_q,
        [
            [magnitude_low, magnitude_high],
            [-magnitude_high, -magnitude_low]
        ]
    );

    let [forward_interval, reverse_interval] = accepted.held_out_response_intervals_q;
    assert!(
        commanded_side_lower_rate_q(
            VelocityDirection::Forward,
            i64::from(forward_interval[0]),
            i64::from(forward_interval[1]),
        )
        .is_some(),
        "forward interval {forward_interval:?} must satisfy commanded_side_lower_rate_q"
    );
    assert!(
        commanded_side_lower_rate_q(
            VelocityDirection::Reverse,
            i64::from(reverse_interval[0]),
            i64::from(reverse_interval[1]),
        )
        .is_some(),
        "reverse interval {reverse_interval:?} must satisfy commanded_side_lower_rate_q"
    );
}

#[test]
fn confirmation_acceptance_retains_held_out_response_intervals() {
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
    let acceptance = evaluate_band_confirmation(gain(724), forward, reverse)
        .expect("rung-18-class evidence confirms within the 70-80% band");
    for [low, high] in acceptance.held_out_response_intervals_q {
        assert!(low <= high);
    }
    assert_ne!(acceptance.held_out_response_intervals_q, [[0, 0], [0, 0]]);
    let [forward_interval, reverse_interval] = acceptance.held_out_response_intervals_q;
    assert!(
        commanded_side_lower_rate_q(
            VelocityDirection::Forward,
            i64::from(forward_interval[0]),
            i64::from(forward_interval[1]),
        )
        .is_some()
    );
    assert!(
        commanded_side_lower_rate_q(
            VelocityDirection::Reverse,
            i64::from(reverse_interval[0]),
            i64::from(reverse_interval[1]),
        )
        .is_some()
    );
}

#[test]
fn confirmation_acceptance_carries_confidence_and_band_position() {
    const MEAN_RATE_Q: i64 = 770_000;

    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");
    let half_width_fraction = 1.0_f64 / 30.0;
    let relative_se = half_width_fraction / f64::from(multiplier);
    let variance = variance_for_relative_se(MEAN_RATE_Q, relative_se);

    let forward = confirmation_input(VelocityDirection::Forward, MEAN_RATE_Q, variance, TARGET);
    let reverse = confirmation_input(VelocityDirection::Reverse, -MEAN_RATE_Q, variance, TARGET);

    let acceptance = evaluate_band_confirmation(gain(724), forward, reverse)
        .expect("77% response at half the containable half-width accepts");

    assert_eq!(acceptance.band_position_q, 45874);
    assert_eq!(acceptance.confidence_q, 32767);
    assert_ne!(acceptance.band_position_q, acceptance.confidence_q);
}

#[test]
fn inconclusive_confirmation_carries_no_new_candidate_p() {
    let forward = confirmation_input(VelocityDirection::Forward, 699_999, 0, TARGET);
    let reverse = confirmation_input(VelocityDirection::Reverse, -700_000, 0, TARGET);
    let TargetBandConfirmationInconclusive(cause) =
        evaluate_band_confirmation(gain(400), forward, reverse).unwrap_err();
    assert_eq!(cause, BandConfirmationFailureCause::ResponseLocation);
}

#[test]
fn confirmation_plan_binds_discovery_digest_and_candidate_p_into_its_digest() {
    let plan_a = plan_band_confirmation(111, gain(724)).unwrap();
    let plan_b = plan_band_confirmation(222, gain(724)).unwrap();
    let plan_c = plan_band_confirmation(111, gain(512)).unwrap();

    assert_ne!(
        plan_a.plan_digest, plan_b.plan_digest,
        "discovery digest must be bound"
    );
    assert_ne!(
        plan_a.plan_digest, plan_c.plan_digest,
        "candidate P must be bound"
    );
    assert_eq!(plan_a.family_size, CONFIRMATION_FAMILY_SIZE);
    assert_eq!(
        plan_a.observations_per_direction,
        CONFIRMATION_OBSERVATIONS_PER_DIRECTION
    );
    assert_eq!(plan_a.band_lower_percent, 70);
    assert_eq!(plan_a.band_upper_percent, 80);
    assert_eq!(
        plan_a.acceptance_rule,
        CONFIRMATION_ACCEPTANCE_RULE_ALL_EIGHT_CONTAINED
    );
    assert_eq!(
        plan_a.capture_profile,
        CONFIRMATION_CAPTURE_PROFILE_EXISTING
    );
}

#[test]
fn confirmation_plan_rejects_zero_candidate_gain() {
    assert_eq!(
        plan_band_confirmation(1, PidPGain::ZERO),
        Err(BandConfirmationPlanError::InvalidRequest)
    );
}

fn digest_with_slot_order(plan: &BandConfirmationPlan, slot_order: &[VelocityDirection; 8]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    macro_rules! hash_value {
        ($value:expr) => {
            for byte in $value.to_le_bytes() {
                hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
            }
        };
    }
    hash_value!(plan.discovery_plan_digest);
    hash_value!(plan.nominated_p.raw());
    hash_value!(plan.capture_profile);
    hash_value!(plan.family_size);
    hash_value!(plan.observations_per_direction);
    hash_value!(plan.moving_stroke_us);
    hash_value!(plan.zero_settle_us);
    hash_value!(plan.settle_analysis_budget_us);
    hash_value!(plan.band_lower_percent);
    hash_value!(plan.band_upper_percent);
    hash_value!(plan.acceptance_rule);
    for direction in slot_order {
        hash_value!(direction_report_code(*direction));
    }
    hash
}

#[test]
fn validate_fixed_contract_accepts_a_well_formed_plan() {
    let plan = plan_band_confirmation(111, gain(724)).unwrap();
    assert_eq!(validate_fixed_contract(&plan), Ok(()));
    assert_eq!(
        digest_with_slot_order(&plan, &CONFIRMATION_SLOT_ORDER),
        plan.plan_digest
    );
}

#[test]
fn validate_fixed_contract_rejects_a_tampered_timing_field() {
    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.moving_stroke_us = MOVING_STROKE_US.wrapping_add(1);
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::Timing)
    );

    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.zero_settle_us = ZERO_SETTLE_US.wrapping_add(1);
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::Timing)
    );

    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.settle_analysis_budget_us = SETTLE_ANALYSIS_BUDGET_US.wrapping_add(1);
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::Timing)
    );
}

#[test]
fn validate_fixed_contract_rejects_a_tampered_band_edge() {
    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.band_lower_percent = 69;
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::BandEdge)
    );

    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.band_upper_percent = 81;
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::BandEdge)
    );
}

#[test]
fn validate_fixed_contract_rejects_a_tampered_observations_per_direction() {
    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.observations_per_direction = CONFIRMATION_OBSERVATIONS_PER_DIRECTION - 1;
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::ObservationsPerDirection)
    );
}

#[test]
fn validate_fixed_contract_rejects_a_tampered_family_size() {
    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.family_size = CONFIRMATION_FAMILY_SIZE + 1;
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::FamilySize)
    );
}

#[test]
fn validate_fixed_contract_rejects_a_tampered_capture_profile() {
    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.capture_profile = CONFIRMATION_CAPTURE_PROFILE_EXISTING + 1;
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::CaptureProfile)
    );
}

#[test]
fn validate_fixed_contract_rejects_a_tampered_acceptance_rule() {
    let mut plan = plan_band_confirmation(111, gain(724)).unwrap();
    plan.acceptance_rule = CONFIRMATION_ACCEPTANCE_RULE_ALL_EIGHT_CONTAINED + 1;
    plan.plan_digest = plan.digest();
    assert_eq!(
        validate_fixed_contract(&plan),
        Err(BandConfirmationContractError::AcceptanceRule)
    );
}

#[test]
fn validate_fixed_contract_rejects_a_slot_order_tampered_digest() {
    let plan = plan_band_confirmation(111, gain(724)).unwrap();
    let reversed = [
        VelocityDirection::Reverse,
        VelocityDirection::Reverse,
        VelocityDirection::Reverse,
        VelocityDirection::Reverse,
        VelocityDirection::Forward,
        VelocityDirection::Forward,
        VelocityDirection::Forward,
        VelocityDirection::Forward,
    ];
    let mut tampered = plan;
    tampered.plan_digest = digest_with_slot_order(&plan, &reversed);
    assert_ne!(tampered.plan_digest, plan.plan_digest);
    assert_eq!(
        validate_fixed_contract(&tampered),
        Err(BandConfirmationContractError::DigestMismatch)
    );
}

fn bounds_for(
    rungs: &[DiscoveryRungNominationInput],
) -> heapless::Vec<DiscoveryRungNominationBounds, 8> {
    rungs
        .iter()
        .map(|rung| rung_nomination_bounds(rung, 0))
        .collect()
}

#[test]
fn reduced_bounds_nominate_the_same_candidate_and_margin_as_raw_atoms() {
    let rungs = [
        uniform_rung(512, RUNG17_MEAN, RUNG17_VARIANCE, TARGET),
        uniform_rung(724, RUNG18_MEAN, RUNG18_VARIANCE, TARGET),
    ];

    let raw_atom_result = nominate_confirmation_candidate(&rungs);
    let bounds = bounds_for(&rungs);
    let reduced_result = nominate_confirmation_candidate_from_bounds(&bounds);

    assert_eq!(
        raw_atom_result, reduced_result,
        "the reduced-bounds path must select the same candidate and margin as the raw-atom path"
    );
    let nominated = raw_atom_result.expect("rung-18-class noise nominates");
    assert_eq!(nominated.p, gain(724));
}

#[test]
fn reduced_bounds_agree_with_raw_atoms_on_the_higher_p_selection_when_noisier() {
    let noisy_mean = 750_000;
    let noisy_variance = variance_for_relative_se(noisy_mean, 0.012);
    let noisy_rung = uniform_rung(700, noisy_mean, noisy_variance, TARGET);

    let quiet_mean = 730_000;
    let quiet_variance = variance_for_relative_se(quiet_mean, 0.004);
    let quiet_rung = uniform_rung(690, quiet_mean, quiet_variance, TARGET);

    let rungs = [noisy_rung, quiet_rung];
    let raw_atom_result = nominate_confirmation_candidate(&rungs);
    let bounds = bounds_for(&rungs);
    let reduced_result = nominate_confirmation_candidate_from_bounds(&bounds);

    assert_eq!(raw_atom_result, reduced_result);
    assert_eq!(raw_atom_result.unwrap().p, gain(700));
}

#[test]
fn reduced_bounds_select_the_higher_p_rung_regardless_of_order_like_raw_atoms() {
    let low_p_rung = uniform_rung(300, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    let high_p_rung = uniform_rung(500, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    let rungs = [low_p_rung, high_p_rung];

    let raw_atom_result = nominate_confirmation_candidate(&rungs);
    let bounds = bounds_for(&rungs);
    let reduced_result = nominate_confirmation_candidate_from_bounds(&bounds);

    assert_eq!(raw_atom_result, reduced_result);
    assert_eq!(raw_atom_result.unwrap().p, gain(500));

    let reordered = [high_p_rung, low_p_rung];
    let raw_atom_reordered = nominate_confirmation_candidate(&reordered);
    let bounds_reordered = bounds_for(&reordered);
    let reduced_reordered = nominate_confirmation_candidate_from_bounds(&bounds_reordered);
    assert_eq!(raw_atom_reordered, reduced_reordered);
    assert_eq!(raw_atom_reordered.unwrap().p, gain(500));
}

#[test]
fn reduced_bounds_agree_with_raw_atoms_at_the_borderline_rejected_rung() {
    let rung = uniform_rung(512, RUNG17_MEAN, RUNG17_VARIANCE, TARGET);
    let bounds = rung_nomination_bounds(&rung, 0);
    assert!(
        bounds.nominatable,
        "rung-17-class noise has valid consensus/atoms, just negative margin"
    );

    let raw_atom_result = nominate_confirmation_candidate(&[rung]);
    let reduced_result = nominate_confirmation_candidate_from_bounds(&[bounds]);
    assert_eq!(raw_atom_result, reduced_result);
    assert_eq!(raw_atom_result, Err(TargetBandUnconfirmableFromDiscovery));
}

#[test]
fn empty_bounds_list_yields_zero_stroke_outcome() {
    assert_eq!(
        nominate_confirmation_candidate_from_bounds(&[]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );
}

#[test]
fn non_eligible_consensus_rung_is_not_nominatable_in_reduced_bounds() {
    let mut ambiguous_forward = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    ambiguous_forward.forward_consensus.classification =
        DirectionalConsensusClass::AmbiguousConsensus;

    let bounds = rung_nomination_bounds(&ambiguous_forward, 0);
    assert!(
        !bounds.nominatable,
        "a rung whose forward consensus is not EligibleConsensus must not be nominatable"
    );
    assert_eq!(
        nominate_confirmation_candidate_from_bounds(&[bounds]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );
}

#[test]
fn zero_or_wrong_sign_mean_rung_is_not_nominatable_in_reduced_bounds() {
    let mut zero_mean_rung = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    zero_mean_rung.forward_observations[0] =
        Some(atom(VelocityDirection::Forward, 0, RUNG18_VARIANCE, TARGET));

    let bounds = rung_nomination_bounds(&zero_mean_rung, 0);
    assert!(
        !bounds.nominatable,
        "a zero mean is not a representable response, so the rung is not nominatable"
    );
    assert_eq!(
        nominate_confirmation_candidate_from_bounds(&[bounds]),
        Err(TargetBandUnconfirmableFromDiscovery)
    );
}

#[test]
fn non_nominatable_rung_is_excluded_even_alongside_a_nominatable_one() {
    let mut ambiguous_forward = uniform_rung(400, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);
    ambiguous_forward.forward_consensus.classification =
        DirectionalConsensusClass::AmbiguousConsensus;
    let nominatable_rung = uniform_rung(724, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);

    let bounds = [
        rung_nomination_bounds(&ambiguous_forward, 0),
        rung_nomination_bounds(&nominatable_rung, 0),
    ];
    let nominated = nominate_confirmation_candidate_from_bounds(&bounds)
        .expect("the non-nominatable rung must not block the nominatable one");
    assert_eq!(nominated.p, gain(724));
}

fn mixed_precision_rung(p_raw: u16) -> DiscoveryRungNominationInput {
    let tight_variance = variance_for_relative_se(RUNG18_MEAN, 0.01);
    let loose_variance = variance_for_relative_se(RUNG18_MEAN, 0.10);
    let forward_observations = [
        Some(atom(
            VelocityDirection::Forward,
            RUNG18_MEAN,
            tight_variance,
            TARGET,
        )),
        Some(atom(
            VelocityDirection::Forward,
            RUNG18_MEAN,
            tight_variance,
            TARGET,
        )),
        Some(atom(
            VelocityDirection::Forward,
            RUNG18_MEAN,
            tight_variance,
            TARGET,
        )),
        Some(atom(
            VelocityDirection::Forward,
            RUNG18_MEAN,
            loose_variance,
            TARGET,
        )),
    ];
    let reverse_observations = [
        Some(atom(
            VelocityDirection::Reverse,
            -RUNG18_MEAN,
            tight_variance,
            TARGET,
        )),
        Some(atom(
            VelocityDirection::Reverse,
            -RUNG18_MEAN,
            tight_variance,
            TARGET,
        )),
        Some(atom(
            VelocityDirection::Reverse,
            -RUNG18_MEAN,
            tight_variance,
            TARGET,
        )),
        Some(atom(
            VelocityDirection::Reverse,
            -RUNG18_MEAN,
            loose_variance,
            TARGET,
        )),
    ];
    DiscoveryRungNominationInput {
        p: gain(p_raw),
        forward_consensus: full_consensus(),
        forward_observations,
        reverse_consensus: full_consensus(),
        reverse_observations,
    }
}

#[test]
fn one_loose_observation_widens_the_union_past_the_band_but_not_the_pooled_estimate() {
    let rung = mixed_precision_rung(700);
    let multiplier = 2.0_f32;

    let (union_low, union_high) =
        rung_response_bounds_percent(&rung, multiplier, 1.0, false).expect("union is judgeable");
    assert!(
        union_low < 70.0 || union_high > 80.0,
        "the loose observation alone must widen the union outside 70-80: [{union_low}, {union_high}]"
    );

    let [
        (pooled_forward_low, pooled_forward_high),
        (pooled_reverse_low, pooled_reverse_high),
    ] = rung_directional_pooled_bounds_percent(&rung, multiplier, 1.0)
        .expect("directional pooling is judgeable");
    assert!(
        pooled_forward_low >= 70.0 && pooled_forward_high <= 80.0,
        "three tight observations must dominate the pooled forward estimate: \
         [{pooled_forward_low}, {pooled_forward_high}]"
    );
    assert!(
        pooled_reverse_low >= 70.0 && pooled_reverse_high <= 80.0,
        "three tight observations must dominate the pooled reverse estimate: \
         [{pooled_reverse_low}, {pooled_reverse_high}]"
    );
}

#[test]
fn nomination_still_uses_the_union_unaffected_by_the_pooled_method() {
    let rung = mixed_precision_rung(700);
    let bounds = rung_nomination_bounds(&rung, 0);
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");
    let (union_low, union_high) =
        rung_response_bounds_percent(&rung, multiplier, NOMINATION_SCREEN_C_SIGMA, true)
            .expect("union is judgeable");
    assert_eq!(bounds.lower_percent, union_low);
    assert_eq!(bounds.upper_percent, union_high);
}

#[test]
fn one_direction_outside_the_pooled_band_leaves_the_other_directions_result_visible() {
    let mut rung = mixed_precision_rung(700);
    for observation in rung.reverse_observations.iter_mut().flatten() {
        observation.mean_rate_q = -(RUNG18_MEAN * 6 / 10);
    }

    let [(forward_low, forward_high), (_reverse_low, reverse_high)] =
        rung_directional_pooled_bounds_percent(&rung, 2.0, 1.0)
            .expect("directional pooling is judgeable even when one direction misses the band");
    assert!(forward_low >= 70.0 && forward_high <= 80.0);
    assert!(
        reverse_high < 70.0,
        "60% reverse response must miss the band: {reverse_high}"
    );
}

#[test]
fn directional_pooling_is_not_judgeable_for_a_non_eligible_direction() {
    let mut rung = mixed_precision_rung(700);
    rung.reverse_consensus.classification = DirectionalConsensusClass::AmbiguousConsensus;
    assert_eq!(
        rung_directional_pooled_bounds_percent(&rung, 2.0, 1.0),
        None
    );
}

#[test]
fn directional_pooling_is_not_judgeable_for_a_missing_included_observation() {
    let mut rung = mixed_precision_rung(700);
    rung.forward_observations[0] = None;
    assert_eq!(
        rung_directional_pooled_bounds_percent(&rung, 2.0, 1.0),
        None
    );
}

#[test]
fn directional_pooling_is_not_judgeable_for_a_zero_mean_observation() {
    let mut rung = mixed_precision_rung(700);
    rung.forward_observations[0] = Some(atom(
        VelocityDirection::Forward,
        0,
        variance_for_relative_se(RUNG18_MEAN, 0.01),
        TARGET,
    ));
    assert_eq!(
        rung_directional_pooled_bounds_percent(&rung, 2.0, 1.0),
        None
    );
}

#[test]
fn directional_pooling_treats_identical_zero_variance_observations_as_one_exact_point() {
    let full_consensus = full_consensus();
    let observations = [Some(atom(VelocityDirection::Forward, RUNG18_MEAN, 0, TARGET)); 4];
    let rung = DiscoveryRungNominationInput {
        p: gain(700),
        forward_consensus: full_consensus,
        forward_observations: observations,
        reverse_consensus: full_consensus,
        reverse_observations: [Some(atom(VelocityDirection::Reverse, -RUNG18_MEAN, 0, TARGET)); 4],
    };
    let [(forward_low, forward_high), _] = rung_directional_pooled_bounds_percent(&rung, 2.0, 1.0)
        .expect("identical exact observations are judgeable");
    assert_eq!(forward_low, 75.0);
    assert_eq!(forward_high, 75.0);
}

#[test]
fn directional_pooling_rejects_conflicting_zero_variance_observations() {
    let mut rung = mixed_precision_rung(700);
    rung.forward_observations[0] = Some(atom(VelocityDirection::Forward, RUNG18_MEAN, 0, TARGET));
    rung.forward_observations[1] =
        Some(atom(VelocityDirection::Forward, RUNG18_MEAN + 1, 0, TARGET));
    assert_eq!(
        rung_directional_pooled_bounds_percent(&rung, 2.0, 1.0),
        None
    );
}

const FORWARD_PEAKS: [u16; 4] = [1200, 1390, 1250, 1500];
const REVERSE_PEAKS: [u16; 4] = [1600, 1100, 1300, 1380];

#[test]
fn rung_step_peak_is_the_max_over_included_slots_of_both_directions() {
    let peak = rung_step_peak_permille(0b0111, &FORWARD_PEAKS, 0b1110, &REVERSE_PEAKS);

    assert_eq!(peak, 1390);
}

#[test]
fn an_excluded_slot_does_not_raise_the_rung_peak() {
    let with_excluded_high =
        rung_step_peak_permille(0b0111, &FORWARD_PEAKS, 0b1110, &REVERSE_PEAKS);
    let all_included = rung_step_peak_permille(0b1111, &FORWARD_PEAKS, 0b1111, &REVERSE_PEAKS);

    assert_eq!(with_excluded_high, 1390);
    assert_eq!(all_included, 1600);
}

#[test]
fn an_all_unscored_rung_reports_zero() {
    let peak = rung_step_peak_permille(0b1111, &[0; 4], 0b1111, &[0; 4]);

    assert_eq!(peak, 0);
}

#[test]
fn rung_nomination_bounds_carries_the_step_peak() {
    let rung = uniform_rung(512, RUNG18_MEAN, RUNG18_VARIANCE, TARGET);

    let bounds = rung_nomination_bounds(&rung, 1234);

    assert_eq!(bounds.step_peak_permille, 1234);
}
