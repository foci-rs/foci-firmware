use crate::autotune::velocity_measurement::{
    PlateauSelection, RateBlock, StationarityExclusion, StationarityResult, StationarySuffix,
    VelocityDirection,
};
use foci_tmc4671::pid::PidPGain;

use super::integral_evidence::{
    ExactRateInterval, IntegralInconclusiveKind, IntegralObservationClass, IntegralRunEvidence,
    IntegralRunSufficiency, IntegralRungClass, IntegralRungKind, StageCEvidenceError,
    analyze_integral_direction, classify_integral_observation,
};
use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, VelocityObservationSummary,
};

fn stationary() -> StationarityResult {
    StationarityResult::Usable(StationarySuffix {
        selection: PlateauSelection {
            level: 1,
            block_count: 16,
            variance_of_mean_q2: 4,
            exact_totals: RateBlock {
                delta_counts: 1,
                elapsed_us: 1_000,
                observation_count: 64,
            },
        },
        suffix_len: 64,
        slope_q: 0,
        slope_half_width_q: 1,
        lag_one_q: 0,
        lag_one_half_width_q: 1,
        residual_magnitude_q2: 1,
        span_x2_us: 0,
        drift_ratio_ppm: u32::MAX,
        tested_suffixes: 1,
    })
}

fn summary(
    direction: VelocityDirection,
    classification: VelocityObservationClass,
    deficit: [i64; 2],
) -> VelocityObservationSummary {
    VelocityObservationSummary {
        direction,
        p: PidPGain::new(1_448).unwrap(),
        classification,
        settled_rate_interval_q: match direction {
            VelocityDirection::Forward => [10, 20],
            VelocityDirection::Reverse => [-20, -10],
        },
        settled_rate_mean_q: match direction {
            VelocityDirection::Forward => 15,
            VelocityDirection::Reverse => -15,
        },
        deficit_interval_q: deficit,
        disturbance: None,
        disturbance_interval: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: 100,
        drift_ratio_ppm: u32::MAX,
        initial_lead_in: false,
    }
}

fn four(
    direction: VelocityDirection,
    classes: [VelocityObservationClass; 4],
    intervals: [[i64; 2]; 4],
) -> [VelocityObservationSummary; 4] {
    core::array::from_fn(|index| summary(direction, classes[index], intervals[index]))
}

fn eligible(direction: VelocityDirection, low: i64, high: i64) -> VelocityObservationSummary {
    summary(direction, VelocityObservationClass::Eligible, [low, high])
}

fn interval(low_q: i32, high_q: i32) -> ExactRateInterval {
    ExactRateInterval::new(low_q, high_q).unwrap()
}

#[test]
fn intrinsic_stage_c_classification_never_requires_disturbance() {
    let input = VelocityObservationInput {
        direction: VelocityDirection::Forward,
        p: PidPGain::new(1_448).unwrap(),
        stationarity: stationary(),
        settled_rate_low_q: 10,
        settled_rate_high_q: 20,
        settled_rate_mean_q: 15,
        variance_of_mean_q2: 0,
        deficit_low_q: -20,
        deficit_high_q: -10,
        disturbance: None,
        disturbance_variance_q2: u64::MAX,
        predicted_torque_target_abs: 100,
        usable_torque_limit: 2_534,
        hard_torque_limit: 2_816,
        initial_lead_in: false,
        drift_ratio_ppm: u32::MAX,
        post_fit_lag_one_state: 2,
    };

    let summary = classify_integral_observation(input);
    assert_eq!(summary.classification, VelocityObservationClass::Eligible);
    assert_eq!(summary.deficit_interval_q, [-20, -10]);
    assert_eq!(summary.disturbance, None);
    assert_eq!(summary.disturbance_interval, None);
    assert_eq!(summary.disturbance_variance_q2, 0);
}

#[test]
fn structured_motion_remains_ineligible_despite_target_response() {
    let input = VelocityObservationInput {
        direction: VelocityDirection::Forward,
        p: PidPGain::new(1_024).unwrap(),
        stationarity: StationarityResult::Excluded(StationarityExclusion::ResolvedSlope),
        settled_rate_low_q: 17_248_000,
        settled_rate_high_q: 17_600_000,
        settled_rate_mean_q: 17_424_000,
        variance_of_mean_q2: 0,
        deficit_low_q: 0,
        deficit_high_q: 352_000,
        disturbance: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: 2_534,
        usable_torque_limit: 2_534,
        hard_torque_limit: 2_816,
        initial_lead_in: false,
        drift_ratio_ppm: u32::MAX,
        post_fit_lag_one_state: 0,
    };

    assert_eq!(
        classify_integral_observation(input).classification,
        VelocityObservationClass::StationarityExcluded(StationarityExclusion::ResolvedSlope)
    );
}

#[test]
fn opening_anchor_requires_command_derived_undertracking_sign() {
    let forward = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::OpeningAnchor,
        [
            eligible(VelocityDirection::Forward, 100, 200),
            eligible(VelocityDirection::Forward, 110, 190),
            eligible(VelocityDirection::Forward, 120, 180),
            eligible(VelocityDirection::Forward, 130, 170),
        ],
    )
    .unwrap();
    let reverse = analyze_integral_direction(
        VelocityDirection::Reverse,
        IntegralRungKind::OpeningAnchor,
        [
            eligible(VelocityDirection::Reverse, -200, -100),
            eligible(VelocityDirection::Reverse, -190, -110),
            eligible(VelocityDirection::Reverse, -180, -120),
            eligible(VelocityDirection::Reverse, -170, -130),
        ],
    )
    .unwrap();

    assert_eq!(forward.classification, IntegralRungClass::EligibleConsensus);
    assert_eq!(forward.interval, Some(interval(120, 180)));
    assert_eq!(reverse.classification, IntegralRungClass::EligibleConsensus);
    assert_eq!(reverse.interval, Some(interval(-180, -120)));

    let unresolved = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::OpeningAnchor,
        [
            eligible(VelocityDirection::Forward, -1, 2),
            eligible(VelocityDirection::Forward, -2, 3),
            eligible(VelocityDirection::Forward, -3, 4),
            eligible(VelocityDirection::Forward, -4, 5),
        ],
    )
    .unwrap();
    assert_eq!(
        unresolved.classification,
        IntegralRungClass::InsufficientConsensus
    );
    assert_eq!(
        unresolved.observation_classes,
        [IntegralObservationClass::SignUnresolved; 4]
    );
}

#[test]
fn positive_rungs_reuse_depth_three_consensus_without_a_sign_gate() {
    let all_usable = [VelocityObservationClass::Eligible; 4];
    let consensus = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::Positive,
        four(
            VelocityDirection::Forward,
            all_usable,
            [[0, 10], [2, 9], [3, 8], [4, 7]],
        ),
    )
    .unwrap();
    assert_eq!(
        consensus.classification,
        IntegralRungClass::EligibleConsensus
    );
    assert_eq!(consensus.interval, Some(interval(3, 8)));
    assert_eq!(consensus.included_mask, 0b1111);

    let three_usable = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::Positive,
        four(
            VelocityDirection::Forward,
            [
                VelocityObservationClass::Eligible,
                VelocityObservationClass::Eligible,
                VelocityObservationClass::DeficitUnresolved,
                VelocityObservationClass::StationarityExcluded(
                    StationarityExclusion::StructuredResidual,
                ),
            ],
            [[0, 10], [2, 9], [3, 8], [4, 7]],
        ),
    )
    .unwrap();
    assert_eq!(
        three_usable.classification,
        IntegralRungClass::EligibleConsensus
    );
    assert_eq!(three_usable.interval, Some(interval(3, 8)));
    assert_eq!(three_usable.included_mask, 0b0111);

    let two_usable = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::Positive,
        four(
            VelocityDirection::Forward,
            [
                VelocityObservationClass::Eligible,
                VelocityObservationClass::DeficitUnresolved,
                VelocityObservationClass::Inert,
                VelocityObservationClass::StationarityExcluded(
                    StationarityExclusion::ResolvedSlope,
                ),
            ],
            [[0, 10], [2, 9], [3, 8], [4, 7]],
        ),
    )
    .unwrap();
    assert_eq!(
        two_usable.classification,
        IntegralRungClass::InsufficientConsensus
    );
}

#[test]
fn ambiguity_and_signed_overshoot_are_reported_without_teleology() {
    let ambiguous = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::Positive,
        four(
            VelocityDirection::Forward,
            [VelocityObservationClass::Eligible; 4],
            [[0, 4], [0, 10], [0, 10], [6, 10]],
        ),
    )
    .unwrap();
    assert_eq!(
        ambiguous.classification,
        IntegralRungClass::AmbiguousConsensus
    );
    assert_eq!(ambiguous.components, [interval(0, 4), interval(6, 10)]);

    let overshoot = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::Positive,
        [
            summary(
                VelocityDirection::Forward,
                VelocityObservationClass::DeficitUnresolved,
                [-20, -10],
            ),
            summary(
                VelocityDirection::Forward,
                VelocityObservationClass::DeficitUnresolved,
                [-19, -11],
            ),
            summary(
                VelocityDirection::Forward,
                VelocityObservationClass::DeficitUnresolved,
                [-18, -12],
            ),
            summary(
                VelocityDirection::Forward,
                VelocityObservationClass::DeficitUnresolved,
                [-17, -13],
            ),
        ],
    )
    .unwrap();
    assert_eq!(
        overshoot.classification,
        IntegralRungClass::EligibleConsensus
    );
    assert_eq!(overshoot.interval, Some(interval(-18, -12)));
}

#[test]
fn stage_c_requires_both_bookend_directions() {
    let mut run = IntegralRunEvidence::new([interval(100, 200), interval(-200, -100)]);
    for rung in [0, 2, 4] {
        run.retain_positive(0, rung, interval(10 + rung as i32, 20 + rung as i32))
            .unwrap();
    }
    for rung in [1, 2, 3] {
        run.retain_positive(1, rung, interval(-20 - rung as i32, -10 - rung as i32))
            .unwrap();
    }
    assert_eq!(
        run.assess(5).unwrap(),
        IntegralRunSufficiency::Inconclusive {
            eligible_masks: [0b1_0101, 0b0_1110],
            cause: IntegralInconclusiveKind::BookendUnavailable,
            bookend_available_mask: 0,
            current_terminus: None,
        }
    );
    run.bookend = [Some(interval(101, 201)), None];

    assert_eq!(run.eligible_masks, [0b1_0101, 0b0_1110]);
    assert_eq!(
        run.assess(5).unwrap(),
        IntegralRunSufficiency::Inconclusive {
            eligible_masks: [0b1_0101, 0b0_1110],
            cause: IntegralInconclusiveKind::BookendUnavailable,
            bookend_available_mask: 0b01,
            current_terminus: None,
        }
    );

    run.bookend[1] = Some(interval(-201, -101));
    assert_eq!(
        run.assess(5).unwrap(),
        IntegralRunSufficiency::Sufficient {
            eligible_masks: [0b1_0101, 0b0_1110],
            bookend_available_mask: 0b11,
            current_terminus: None,
        }
    );
}

#[test]
fn current_bound_only_blocks_before_the_directional_floor() {
    let mut insufficient = IntegralRunEvidence::new([interval(100, 200), interval(-200, -100)]);
    for direction in 0..2 {
        for rung in 0..2 {
            insufficient
                .retain_positive(direction, rung, interval(10, 20))
                .unwrap();
        }
    }
    insufficient.current_terminus = Some(2);
    assert_eq!(
        insufficient.assess(5).unwrap(),
        IntegralRunSufficiency::Inconclusive {
            eligible_masks: [0b11, 0b11],
            cause: IntegralInconclusiveKind::CurrentBoundBeforeSufficiency,
            bookend_available_mask: 0,
            current_terminus: Some(2),
        }
    );

    let mut sufficient = insufficient;
    sufficient.retain_positive(0, 2, interval(10, 20)).unwrap();
    sufficient.retain_positive(1, 2, interval(10, 20)).unwrap();
    sufficient.bookend = [Some(interval(100, 200)), Some(interval(-200, -100))];
    assert!(matches!(
        sufficient.assess(5).unwrap(),
        IntegralRunSufficiency::Sufficient {
            current_terminus: Some(2),
            ..
        }
    ));
}

#[test]
fn invalid_rate_width_and_mask_bits_fail_closed() {
    let overflow = analyze_integral_direction(
        VelocityDirection::Forward,
        IntegralRungKind::Positive,
        [
            eligible(
                VelocityDirection::Forward,
                i64::from(i32::MAX) + 1,
                i64::from(i32::MAX) + 2,
            ),
            eligible(VelocityDirection::Forward, 1, 2),
            eligible(VelocityDirection::Forward, 1, 2),
            eligible(VelocityDirection::Forward, 1, 2),
        ],
    );
    assert_eq!(overflow, Err(StageCEvidenceError::Conversion));

    let mut run = IntegralRunEvidence::new([interval(100, 200), interval(-200, -100)]);
    run.eligible_masks = [1 << 5, 0];
    assert_eq!(run.assess(5), Err(StageCEvidenceError::MaskOutOfRange));
}
