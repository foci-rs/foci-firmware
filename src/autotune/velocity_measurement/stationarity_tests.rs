use core::cell::Cell;

use super::{
    RestExclusion, RestSelectionDisposition, RestStructuralCode, RestSuffixEvaluation,
    SettledRateContext, StationarityExclusion, StationarityMetrics, StationarityResult,
    StationarityWork, StationarityWorkPoll, VELOCITY_SUFFIX_LENGTHS, VelocityObservationSource,
    analyze_stationarity_diagnostics, analyze_stationarity_with_blocking_variance,
    classify_moving_metrics, classify_rest_metrics, drift_ratio_ppm, select_causal_rest_suffixes,
    select_stationary_suffix,
};
use crate::autotune::velocity_measurement::{DyadicBlocker, RawVelocityObservation};

const LEVELS: usize = 9;

fn observations<const N: usize>(delta: impl Fn(usize) -> i32) -> [RawVelocityObservation; N] {
    core::array::from_fn(|index| RawVelocityObservation::new(delta(index), 1_000).unwrap())
}

fn analyze_stationarity(
    observations: &[RawVelocityObservation],
    block_observations: usize,
    family_size: u16,
) -> Result<super::StationarityMetrics, StationarityExclusion> {
    analyze_stationarity_with_blocking_variance(observations, block_observations, family_size, 0)
}

fn usable_rest_suffix(interval: [i64; 2]) -> RestSuffixEvaluation {
    RestSuffixEvaluation {
        structural_code: RestStructuralCode::Usable,
        interval_present: true,
        interval_q: interval,
    }
}

fn excluded_rest_suffix(
    structural_code: RestStructuralCode,
    interval: Option<[i64; 2]>,
) -> RestSuffixEvaluation {
    RestSuffixEvaluation {
        structural_code,
        interval_present: interval.is_some(),
        interval_q: interval.unwrap_or([0, 0]),
    }
}

#[test]
fn causal_rest_selector_matches_literal_historical_decisions() {
    struct Case {
        name: &'static str,
        suffixes: [RestSuffixEvaluation; 3],
        fallback: bool,
        selected_suffix_len: u16,
        disposition: RestSelectionDisposition,
    }

    let cases = [
        Case {
            name: "X replacement combined",
            suffixes: [
                usable_rest_suffix([0, 0]),
                usable_rest_suffix([-13_058, 44_450]),
                usable_rest_suffix([5_840, 48_970]),
            ],
            fallback: false,
            selected_suffix_len: 128,
            disposition: RestSelectionDisposition::StatisticalAcceptance,
        },
        Case {
            name: "Y final combined",
            suffixes: [
                usable_rest_suffix([-26_318, 57_862]),
                usable_rest_suffix([-1_053, 79_545]),
                usable_rest_suffix([8_900, 69_408]),
            ],
            fallback: false,
            selected_suffix_len: 128,
            disposition: RestSelectionDisposition::StatisticalAcceptance,
        },
        Case {
            name: "Y retry",
            suffixes: [
                usable_rest_suffix([0, 0]),
                usable_rest_suffix([-28_876, 13_216]),
                usable_rest_suffix([-85_717, -317]),
            ],
            fallback: false,
            selected_suffix_len: 128,
            disposition: RestSelectionDisposition::StatisticalAcceptance,
        },
        Case {
            name: "X 300 ms",
            suffixes: [
                usable_rest_suffix([-26_350, 57_830]),
                usable_rest_suffix([33_244, 217_674]),
                usable_rest_suffix([43_470, 175_680]),
            ],
            fallback: false,
            selected_suffix_len: 64,
            disposition: RestSelectionDisposition::StatisticalAcceptance,
        },
        Case {
            name: "P=724",
            suffixes: [
                excluded_rest_suffix(
                    RestStructuralCode::ResolvedSlope,
                    Some([-511_681, -148_633]),
                ),
                usable_rest_suffix([-615_426, -262_274]),
                usable_rest_suffix([-445_779, -117_587]),
            ],
            fallback: false,
            selected_suffix_len: 0,
            disposition: RestSelectionDisposition::Rejected,
        },
    ];

    for case in cases {
        let decision = select_causal_rest_suffixes(case.suffixes, case.fallback);
        assert_eq!(
            decision.selected_suffix_len, case.selected_suffix_len,
            "{}",
            case.name
        );
        assert_eq!(decision.disposition, case.disposition, "{}", case.name);
    }
}

#[test]
fn slot_157_recent_directional_motion_cannot_be_masked_by_older_suffixes() {
    let mut recent_deltas = [0_i8; 64];
    for sample_index in [410, 422, 439, 448, 453, 457] {
        recent_deltas[sample_index - 396] = 1;
    }
    let decision = select_causal_rest_suffixes(
        [
            usable_rest_suffix([7_977, 181_103]),
            usable_rest_suffix([-89, 109_923]),
            usable_rest_suffix([-7_780, 62_582]),
        ],
        false,
    );

    assert_eq!(recent_deltas.iter().copied().map(i16::from).sum::<i16>(), 6);
    assert_eq!(recent_deltas[457 - 396], 1);
    assert_eq!(decision.selected_suffix_len, 0);
    assert_eq!(decision.selected_mask, 0);
    assert_eq!(decision.zero_compatible_mask, 0b110);
    assert_eq!(decision.disposition, RestSelectionDisposition::Rejected);
}

#[test]
fn causal_rest_selector_treats_closed_interval_endpoints_as_zero_compatible() {
    for interval in [[0, 1], [-1, 0]] {
        let decision = select_causal_rest_suffixes(
            [
                usable_rest_suffix(interval),
                excluded_rest_suffix(RestStructuralCode::NoBlockingPlateau, None),
                excluded_rest_suffix(RestStructuralCode::NoBlockingPlateau, None),
            ],
            false,
        );

        assert_eq!(decision.selected_suffix_len, 64);
        assert_eq!(decision.zero_compatible_mask, 0b001);
        assert_eq!(
            decision.disposition,
            RestSelectionDisposition::StatisticalAcceptance
        );
    }
}

#[test]
fn one_count_fallback_applies_only_when_the_recent_chain_does_not_start() {
    let recent_motion = [
        usable_rest_suffix([1, 2]),
        usable_rest_suffix([-1, 1]),
        usable_rest_suffix([-1, 1]),
    ];
    let fallback = select_causal_rest_suffixes(recent_motion, true);
    assert_eq!(fallback.selected_suffix_len, 0);
    assert_eq!(
        fallback.disposition,
        RestSelectionDisposition::OneCountFallbackAcceptance
    );

    let statistical = select_causal_rest_suffixes(
        [
            usable_rest_suffix([-1, 1]),
            usable_rest_suffix([-1, 1]),
            usable_rest_suffix([1, 2]),
        ],
        true,
    );
    assert_eq!(statistical.selected_suffix_len, 128);
    assert_eq!(
        statistical.disposition,
        RestSelectionDisposition::StatisticalAcceptance
    );
}

#[test]
fn every_selected_rest_suffix_has_a_contiguous_compatible_prefix() {
    for usable_mask in 0_u8..8 {
        for zero_mask in 0_u8..8 {
            let suffixes = core::array::from_fn(|index| {
                if usable_mask & (1 << index) == 0 {
                    excluded_rest_suffix(RestStructuralCode::ResolvedSlope, Some([1, 2]))
                } else if zero_mask & (1 << index) != 0 {
                    usable_rest_suffix([-1, 1])
                } else {
                    usable_rest_suffix([1, 2])
                }
            });
            let decision = select_causal_rest_suffixes(suffixes, false);
            if decision.selected_mask == 0 {
                continue;
            }
            let selected_index = decision.selected_mask.trailing_zeros() as usize;
            let required_prefix = (1_u8 << (selected_index + 1)) - 1;
            assert_eq!(decision.usable_mask & required_prefix, required_prefix);
            assert_eq!(
                decision.zero_compatible_mask & required_prefix,
                required_prefix
            );
        }
    }
}

#[test]
fn suffix_family_is_fixed_and_dyadic() {
    assert_eq!(VELOCITY_SUFFIX_LENGTHS, [64, 128, 256]);
}

#[test]
fn scored_rest_state_stays_inside_the_existing_storage_seam() {
    assert_eq!(core::mem::size_of::<StationarityWork>(), 256);
    assert!(core::mem::size_of::<super::StationarityWorkOutput>() <= 128);
}

#[test]
fn stationarity_diagnostics_preserve_the_binding_verdict() {
    let samples = observations::<256>(|index| [1, 0, 1, 1, 0][index % 5]);
    let mut binding_blocker = DyadicBlocker::<LEVELS>::new();
    let binding = select_stationary_suffix(&samples, &mut binding_blocker).unwrap();
    let mut diagnostic_blocker = DyadicBlocker::<LEVELS>::new();

    let diagnostics =
        analyze_stationarity_diagnostics(&samples[..], &mut diagnostic_blocker).unwrap();

    assert_eq!(diagnostics.binding, binding);
    assert_eq!(
        diagnostics.suffixes.map(|item| item.unwrap().suffix_len),
        [64, 128, 256]
    );
    assert!(diagnostics.suffixes.iter().all(|item| {
        let diagnostic = item.unwrap();
        diagnostic.usable_plateau && diagnostic.slope_passed && diagnostic.lag_one_passed
    }));
}

#[test]
fn stationarity_diagnostics_report_longer_suffix_after_binding_lag_rejection() {
    let samples = observations::<256>(|index| {
        if index >= 192 {
            if (index / 4) % 2 == 0 { 10 } else { -10 }
        } else {
            [1, -1, 0, -1, 1][index % 5]
        }
    });
    let mut blocker = DyadicBlocker::<LEVELS>::new();

    let diagnostics = analyze_stationarity_diagnostics(&samples[..], &mut blocker).unwrap();

    let short = diagnostics.suffixes[0].unwrap();
    let medium = diagnostics.suffixes[1].unwrap();
    assert!(!short.lag_one_passed);
    assert!(short.lag_one_rejection_margin_q < 0);
    assert!(medium.slope_passed);
    assert!(medium.lag_one_passed);
    assert_eq!(
        diagnostics.binding,
        StationarityResult::Excluded(StationarityExclusion::StructuredResidual)
    );
}

#[test]
fn stationarity_diagnostics_report_missing_plateaus_without_fabricating_intervals() {
    let samples = observations::<64>(|_| 1);
    let mut blocker = DyadicBlocker::<2>::new();

    let diagnostics = analyze_stationarity_diagnostics(&samples[..], &mut blocker).unwrap();

    let short = diagnostics.suffixes[0].unwrap();
    assert!(!short.usable_plateau);
    assert_eq!(short.settled_rate_interval_q, [0, 0]);
    assert!(diagnostics.suffixes[1].is_none());
    assert!(diagnostics.suffixes[2].is_none());
}

#[test]
fn flat_tail_selects_the_longest_supported_suffix() {
    let samples = observations::<256>(|_| 1);
    let mut blocker = DyadicBlocker::<LEVELS>::new();

    let result = select_stationary_suffix(&samples, &mut blocker).unwrap();

    let StationarityResult::Usable(suffix) = result else {
        panic!("flat tail was excluded: {result:?}");
    };
    assert_eq!(suffix.suffix_len, 256);
    assert_eq!(suffix.tested_suffixes, 3);
    assert_eq!(suffix.slope_q, 0);
    assert!(suffix.selection.block_count >= 16);
}

#[test]
fn cooperative_work_matches_synchronous_selection_across_multiple_polls() {
    let samples = observations::<256>(|index| [1, 0, 1, 1, 0][index % 5]);
    let mut synchronous_blocker = DyadicBlocker::<LEVELS>::new();
    let expected = select_stationary_suffix(&samples, &mut synchronous_blocker).unwrap();
    let mut cooperative_blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();
    let mut polls = 0;

    let actual = loop {
        polls += 1;
        match work.poll(&samples[..], &mut cooperative_blocker).unwrap() {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete { result, .. } => break result,
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("moving mode returned a scored-rest result")
            }
        }
    };

    assert_eq!(actual, expected);
    assert!(polls > 1, "a full capture must not be analyzed in one poll");
}

#[test]
fn cooperative_polling_yields_the_same_span_as_the_synchronous_path() {
    // The streaming state machine must not compute a different regression span
    // from the synchronous selection, or the same capture would classify two ways
    // depending on which path read it.
    //
    // The fixture is the one the neighbouring selection test already uses: a
    // repeating pattern whose fitted slope is zero, so it passes the significance
    // gate still in force at this point in the plan. Task 4 has not landed yet, and
    // a decelerating fixture would be rejected before the span is ever compared.
    let samples = observations::<256>(|index| [1, 0, 1, 1, 0][index % 5]);

    let mut synchronous_blocker = DyadicBlocker::<LEVELS>::new();
    let expected = select_stationary_suffix(&samples, &mut synchronous_blocker).unwrap();
    let StationarityResult::Usable(expected_suffix) = expected else {
        panic!("fixture must produce a usable suffix");
    };

    let mut cooperative_blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();
    let actual = loop {
        match work.poll(&samples[..], &mut cooperative_blocker).unwrap() {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete { result, .. } => break result,
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("moving mode returned a scored-rest result")
            }
        }
    };
    let StationarityResult::Usable(actual_suffix) = actual else {
        panic!("cooperative path must produce a usable suffix");
    };

    assert_eq!(actual_suffix.span_x2_us, expected_suffix.span_x2_us);
    assert_ne!(
        actual_suffix.span_x2_us, 0,
        "a zero span would pass vacuously"
    );
}

#[test]
fn cooperative_poll_limits_raw_visits_for_two_ms_safety_cadence() {
    struct CountingSource<'a> {
        samples: &'a [RawVelocityObservation],
        visits: Cell<usize>,
    }

    impl VelocityObservationSource for CountingSource<'_> {
        fn observation_len(&self) -> usize {
            self.samples.len()
        }

        fn observation(&self, index: usize) -> Option<RawVelocityObservation> {
            self.visits.set(self.visits.get() + 1);
            self.samples.get(index).copied()
        }
    }

    let samples = observations::<256>(|index| [1, 0, 1, 1, 0][index % 5]);
    let source = CountingSource {
        samples: &samples,
        visits: Cell::new(0),
    };
    let mut blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();

    loop {
        source.visits.set(0);
        let poll = work.poll(&source, &mut blocker).unwrap();
        assert!(
            source.visits.get() <= 8,
            "one cooperative poll visited {} raw observations",
            source.visits.get()
        );
        if matches!(poll, StationarityWorkPoll::Complete { .. }) {
            break;
        }
    }
}

#[test]
fn cooperative_rest_work_emits_one_diagnostic_then_reconstructs_its_witness() {
    struct CountingSource<'a> {
        samples: &'a [RawVelocityObservation],
        visits: Cell<usize>,
    }

    impl VelocityObservationSource for CountingSource<'_> {
        fn observation_len(&self) -> usize {
            self.samples.len()
        }

        fn observation(&self, index: usize) -> Option<RawVelocityObservation> {
            self.visits.set(self.visits.get() + 1);
            self.samples.get(index).copied()
        }
    }

    let samples = observations::<256>(|_| 0);
    let source = CountingSource {
        samples: &samples,
        visits: Cell::new(0),
    };
    let mut blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();
    work.reset_for_scored_rest(false);
    let mut diagnostic = None;
    let mut diagnostic_count = 0;
    let mut polls = 0_usize;

    let completed = loop {
        source.visits.set(0);
        polls += 1;
        match work.poll(&source, &mut blocker).unwrap() {
            StationarityWorkPoll::Pending => {
                let visit_limit = if diagnostic_count == 0 { 8 } else { 16 };
                assert!(source.visits.get() <= visit_limit);
            }
            StationarityWorkPoll::RestDiagnostic(decision) => {
                assert!(source.visits.get() <= 8);
                diagnostic_count += 1;
                diagnostic = Some(decision);
            }
            StationarityWorkPoll::RestComplete(result) => {
                assert!(source.visits.get() <= 16);
                break result;
            }
            StationarityWorkPoll::Complete { .. } => {
                panic!("rest mode returned a moving-rate result")
            }
        }
    };

    let diagnostic = diagnostic.expect("rest analysis must expose its decision");
    let selected = completed
        .selected
        .expect("statistical acceptance must reconstruct its witness");
    assert_eq!(diagnostic_count, 1);
    assert_eq!(diagnostic.interval_present_mask, 0b111);
    assert_eq!(diagnostic.usable_mask, 0b111);
    assert_eq!(diagnostic.zero_compatible_mask, 0b111);
    assert_eq!(diagnostic.selected_mask, 0b100);
    assert_eq!(diagnostic.selected_suffix_len, 256);
    assert_eq!(
        diagnostic.disposition,
        RestSelectionDisposition::StatisticalAcceptance
    );
    assert_eq!(selected.suffix_len, 256);
    assert_eq!(
        completed.exact_velocity_interval_q,
        Some(diagnostic.intervals_q[2])
    );
    assert_eq!(completed.disposition, diagnostic.disposition);
    assert!(
        polls <= 236,
        "diagnostic and witness reconstruction used {polls} cooperative polls"
    );
}

#[test]
fn scored_rest_rejection_retains_only_its_recent_structural_witness() {
    let samples = observations::<256>(|_| 1);

    for (fallback, expected) in [
        (true, RestSelectionDisposition::OneCountFallbackAcceptance),
        (false, RestSelectionDisposition::Rejected),
    ] {
        let mut blocker = DyadicBlocker::<LEVELS>::new();
        let mut work = StationarityWork::new();
        work.reset_for_scored_rest(fallback);
        let mut saw_diagnostic = false;
        let mut recent_structural_code = None;

        loop {
            match work.poll(&samples[..], &mut blocker).unwrap() {
                StationarityWorkPoll::Pending => {}
                StationarityWorkPoll::RestDiagnostic(decision) => {
                    saw_diagnostic = true;
                    assert_eq!(decision.selected_suffix_len, 0);
                    assert_eq!(decision.disposition, expected);
                    assert_eq!(decision.usable_mask, 0b111);
                    assert_eq!(decision.zero_compatible_mask, 0);
                    recent_structural_code = Some(decision.structural_codes[0]);
                }
                StationarityWorkPoll::RestComplete(result) => {
                    assert!(saw_diagnostic);
                    assert_eq!(result.disposition, expected);
                    if expected == RestSelectionDisposition::Rejected {
                        assert_eq!(result.selected.map(|suffix| suffix.suffix_len), Some(64));
                    } else {
                        assert_eq!(result.selected, None);
                    }
                    assert_eq!(result.exact_velocity_interval_q, None);
                    assert_eq!(
                        result.recent_structural_code,
                        recent_structural_code.unwrap()
                    );
                    break;
                }
                StationarityWorkPoll::Complete { .. } => {
                    panic!("rest mode returned a moving-rate result")
                }
            }
        }
    }
}

#[test]
fn cooperative_structured_diagnostics_match_synchronous_evidence_with_bounded_visits() {
    struct CountingSource<'a> {
        samples: &'a [RawVelocityObservation],
        visits: Cell<usize>,
    }

    impl VelocityObservationSource for CountingSource<'_> {
        fn observation_len(&self) -> usize {
            self.samples.len()
        }

        fn observation(&self, index: usize) -> Option<RawVelocityObservation> {
            self.visits.set(self.visits.get() + 1);
            self.samples.get(index).copied()
        }
    }

    let samples = observations::<256>(|index| {
        if index >= 192 {
            if (index / 4) % 2 == 0 { 10 } else { -10 }
        } else {
            [1, -1, 0, -1, 1][index % 5]
        }
    });
    let mut synchronous_blocker = DyadicBlocker::<LEVELS>::new();
    let expected =
        analyze_stationarity_diagnostics(&samples[..], &mut synchronous_blocker).unwrap();
    let source = CountingSource {
        samples: &samples,
        visits: Cell::new(0),
    };
    let mut cooperative_blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();
    work.reset_with_structured_diagnostics();

    let actual = loop {
        source.visits.set(0);
        let poll = work.poll(&source, &mut cooperative_blocker).unwrap();
        assert!(
            source.visits.get() <= 8,
            "one cooperative diagnostic poll visited {} raw observations",
            source.visits.get()
        );
        match poll {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete { result, .. } => break result,
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("structured mode returned a scored-rest result")
            }
        }
    };

    let short = expected.suffixes[0].unwrap();
    let mut expected_common = short.settled_rate_interval_q;
    let mut expected_mask = 0_u8;
    for (index, diagnostic) in expected.suffixes.iter().enumerate() {
        let Some(diagnostic) = diagnostic else {
            continue;
        };
        if diagnostic.slope_passed && diagnostic.lag_one_passed {
            expected_mask |= 1 << index;
            if index != 0 {
                expected_common[0] = expected_common[0].max(diagnostic.settled_rate_interval_q[0]);
                expected_common[1] = expected_common[1].min(diagnostic.settled_rate_interval_q[1]);
            }
        }
    }
    let structured = work
        .structured_evidence()
        .expect("the longer passing suffix must remain observable");

    assert_eq!(actual, expected.binding);
    assert_eq!(structured.passing_suffix_mask, expected_mask);
    assert_eq!(structured.common_rate_interval_q, expected_common);
    assert_eq!(
        structured.binding_slope_margin_q,
        short.slope_rejection_margin_q
    );
    assert_eq!(
        structured.binding_lag_one_margin_q,
        short.lag_one_rejection_margin_q
    );
}

#[test]
fn cooperative_work_preserves_first_nonstationary_extension_rule() {
    let samples = observations::<256>(|index| {
        if index >= 192 {
            1
        } else if index >= 128 {
            -100
        } else {
            1
        }
    });
    let mut synchronous_blocker = DyadicBlocker::<LEVELS>::new();
    let expected = select_stationary_suffix(&samples, &mut synchronous_blocker).unwrap();
    let mut cooperative_blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();

    let actual = loop {
        match work.poll(&samples[..], &mut cooperative_blocker).unwrap() {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete { result, .. } => break result,
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("moving mode returned a scored-rest result")
            }
        }
    };

    assert_eq!(actual, expected);
}

#[test]
fn quantized_flat_tail_remains_stationary() {
    let samples = observations::<256>(|index| [1, 0, 1, 1, 0][index % 5]);
    let mut blocker = DyadicBlocker::<LEVELS>::new();

    assert!(matches!(
        select_stationary_suffix(&samples, &mut blocker),
        Ok(StationarityResult::Usable(_))
    ));
}

#[test]
fn positive_and_negative_drift_are_rejected() {
    // The rejection reason moved from ResolvedSlope to ExcessiveDrift when the
    // moving classifier stopped gating on slope significance. The intent is
    // unchanged and the fixture is untouched: a monotonic ramp in either
    // direction must still be rejected. It is nowhere near the 200_000 ppm
    // boundary either. The rate runs from 0 to 63 counts per sample about a mean
    // of 31.5, so the drift across the span is roughly 2_000_000 ppm of the
    // settled rate, an order of magnitude past the bound.
    for sign in [1, -1] {
        let samples = observations::<64>(|index| sign * i32::try_from(index).unwrap());
        let exclusion = analyze_stationarity(&samples, 1, 3).unwrap_err();
        assert_eq!(exclusion, StationarityExclusion::ExcessiveDrift);
    }
}

#[test]
fn curvature_with_zero_average_slope_has_structured_residuals() {
    // A parabola centered on the suffix has zero average slope but a large
    // lag-one residual correlation, so this exercises the lag-one gate, not the
    // drift bound. Its fitted drift ratio passes the 200_000 ppm bound, but not
    // by a wide margin.
    //
    // The drift check runs before the lag-one check, so a future change to the
    // blocking level or span selection that pushes this fixture's ratio over the
    // bound flips the expected exclusion from StructuredResidual to
    // ExcessiveDrift rather than failing loudly. No figure is quoted here on
    // purpose: the ratio depends on which fit path is measured, and three
    // separate derivations of it disagreed.
    let samples = observations::<64>(|index| {
        let centered = i32::try_from(index).unwrap() - 31;
        centered * centered / 16
    });

    assert_eq!(
        analyze_stationarity(&samples, 1, 3),
        Err(StationarityExclusion::StructuredResidual)
    );
}

#[test]
fn positive_and_negative_lag_one_structure_are_rejected() {
    let positive = observations::<64>(|index| if (index / 4) % 2 == 0 { 10 } else { -10 });
    let negative = observations::<64>(|index| if index % 2 == 0 { 10 } else { -10 });

    assert_eq!(
        analyze_stationarity(&positive, 1, 3),
        Err(StationarityExclusion::StructuredResidual)
    );
    assert_eq!(
        analyze_stationarity(&negative, 1, 3),
        Err(StationarityExclusion::StructuredResidual)
    );
}

#[test]
fn lag_one_check_documents_the_bounded_higher_lag_limit() {
    let samples = observations::<64>(|index| match index % 4 {
        0 | 1 => 10,
        _ => -10,
    });

    assert!(analyze_stationarity(&samples, 1, 3).is_ok());
}

#[test]
fn fewer_than_sixteen_blocks_is_excluded() {
    let samples = observations::<15>(|_| 1);

    assert_eq!(
        analyze_stationarity(&samples, 1, 3),
        Err(StationarityExclusion::TooFewDecorrelatedBlocks)
    );
}

#[test]
fn blocker_variance_cross_check_can_only_widen_slope_uncertainty() {
    let samples = observations::<64>(|_| 1);
    let residual_only = analyze_stationarity(&samples, 1, 3).unwrap();
    let cross_checked =
        analyze_stationarity_with_blocking_variance(&samples, 1, 3, 1_000_000).unwrap();

    assert!(cross_checked.slope_half_width_q > residual_only.slope_half_width_q);
}

#[test]
fn no_two_doubling_plateau_is_distinct() {
    let samples = observations::<64>(|_| 1);
    let mut blocker = DyadicBlocker::<2>::new();

    assert_eq!(
        select_stationary_suffix(&samples, &mut blocker),
        Ok(StationarityResult::Excluded(
            StationarityExclusion::NoBlockingPlateau
        ))
    );
}

#[test]
fn first_nonstationary_extension_stops_without_stitching() {
    let samples = observations::<256>(|index| {
        if index >= 192 {
            1
        } else if index >= 128 {
            -100
        } else {
            1
        }
    });
    let mut blocker = DyadicBlocker::<LEVELS>::new();

    let result = select_stationary_suffix(&samples, &mut blocker).unwrap();
    let StationarityResult::Usable(suffix) = result else {
        panic!("short stationary suffix was excluded: {result:?}");
    };
    assert_eq!(suffix.suffix_len, 64);
    assert_eq!(suffix.tested_suffixes, 2);
}

/// A tail rejected by the slope or lag-one gate must retain the statistics that
/// rejected it. Without this the failing observations are the only ones carrying
/// no evidence, which is backwards for a diagnostic instrument.
#[test]
fn a_gate_rejected_tail_retains_the_margins_that_rejected_it() {
    // A monotonic ramp in per-sample displacement cannot be stationary.
    let samples = observations::<256>(|index| 1 + (index as i32) / 8);
    let mut blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();

    let (result, rejection) = loop {
        match work.poll(&samples[..], &mut blocker).unwrap() {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete {
                result, rejection, ..
            } => break (result, rejection),
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("moving mode returned a scored-rest result")
            }
        }
    };

    assert!(
        matches!(result, StationarityResult::Excluded(_)),
        "a ramp must not be certified stationary, got {result:?}"
    );
    let rejection = rejection.expect("a gate rejection must report its margins");
    assert_eq!(
        rejection.suffix_len, 64,
        "the family is tried shortest first"
    );
    assert!(
        !rejection.slope_passed || !rejection.lag_one_passed,
        "at least one gate must report failure"
    );
    assert!(
        rejection.slope_rejection_margin_q < 0 || rejection.lag_one_rejection_margin_q < 0,
        "the failing gate must carry a negative margin"
    );
}

/// Nothing was rejected, so there is no rejection to report.
#[test]
fn a_usable_tail_reports_no_rejection() {
    let samples = observations::<256>(|index| [1, 0, 1, 1, 0][index % 5]);
    let mut blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();

    let (result, rejection) = loop {
        match work.poll(&samples[..], &mut blocker).unwrap() {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete {
                result, rejection, ..
            } => break (result, rejection),
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("moving mode returned a scored-rest result")
            }
        }
    };

    assert!(matches!(result, StationarityResult::Usable(_)));
    assert!(rejection.is_none());
}

/// Structured mode must report its rejection too.
///
/// Stage C resets with structured diagnostics, so a rejected tail there took the
/// structured completion path, which returned no rejection at all. The gate
/// statistics were therefore absent from exactly the acquisition that needed
/// them: four ResolvedSlope exclusions in the 2026-07-29 mirrored capture all
/// carried a clear presence flag.
#[test]
fn a_structured_mode_rejection_still_reports_its_margins() {
    let samples = observations::<256>(|index| 1 + (index as i32) / 8);
    let mut blocker = DyadicBlocker::<LEVELS>::new();
    let mut work = StationarityWork::new();
    work.reset_with_structured_diagnostics();

    let (result, rejection) = loop {
        match work.poll(&samples[..], &mut blocker).unwrap() {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete {
                result, rejection, ..
            } => break (result, rejection),
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("structured mode returned a scored-rest result")
            }
        }
    };

    assert!(
        matches!(result, StationarityResult::Excluded(_)),
        "a ramp must not be certified stationary, got {result:?}"
    );
    let rejection = rejection.expect("structured rejection must report its margins");
    assert!(
        !rejection.slope_passed || !rejection.lag_one_passed,
        "at least one gate must report failure"
    );
    assert!(
        rejection.slope_rejection_margin_q < 0 || rejection.lag_one_rejection_margin_q < 0,
        "the failing gate must carry a negative margin"
    );
}

#[test]
fn drift_ratio_is_exact_integer_parts_per_million() {
    // 10% of rate over the span: slope 1_000_000 rate units per second across a
    // 2 second span is 2_000_000 of drift against a rate of 20_000_000.
    assert_eq!(drift_ratio_ppm(1_000_000, 4_000_000, 20_000_000), 100_000);

    // Sign is irrelevant on both inputs; only magnitudes matter.
    assert_eq!(drift_ratio_ppm(-1_000_000, 4_000_000, 20_000_000), 100_000);
    assert_eq!(drift_ratio_ppm(1_000_000, 4_000_000, -20_000_000), 100_000);
}

#[test]
fn drift_ratio_rounds_up_so_a_boundary_case_cannot_round_into_passing() {
    // Exactly 200_000 ppm must stay 200_000 and pass.
    assert_eq!(drift_ratio_ppm(200_000, 2_000_000, 1_000_000), 200_000);

    // Under floor division this would truncate to 0 and wrongly pass.
    assert_eq!(drift_ratio_ppm(1, 3, 1_000_000), 1);
}

#[test]
fn drift_ratio_saturates_rather_than_wrapping() {
    assert_eq!(drift_ratio_ppm(i64::MAX, u64::MAX, 1), u32::MAX);
}

#[test]
fn drift_ratio_matches_a_hand_computed_case_at_capture_scale() {
    // Magnitudes from rung 0 slot 3 of
    // foc-192-stepper-y-rejection-slope-20260729-221033.bin, at a 60 ms span.
    // The capture does not report the block level on the rejection path, so the
    // real span is unknown; this pins the arithmetic at a realistic scale, not
    // the observation's actual ratio.
    assert_eq!(drift_ratio_ppm(-8_823_529, 2 * 60_000, 13_750_000), 38_503);
}

struct MetricsFixture {
    slope_q: i64,
    slope_half_width_q: u64,
    span_x2_us: u64,
    mean_rate_q: i64,
    lag_one_passed: bool,
}

fn metrics_fixture(fixture: MetricsFixture) -> StationarityMetrics {
    StationarityMetrics {
        slope_q: fixture.slope_q,
        slope_half_width_q: fixture.slope_half_width_q,
        lag_one_q: 0,
        lag_one_half_width_q: 1_000_000,
        residual_magnitude_q2: 0,
        slope_passed: fixture.slope_q.unsigned_abs() <= fixture.slope_half_width_q,
        lag_one_passed: fixture.lag_one_passed,
        slope_rejection_margin_q: 0,
        lag_one_rejection_margin_q: 0,
        span_x2_us: fixture.span_x2_us,
        drift_ratio_ppm: drift_ratio_ppm(fixture.slope_q, fixture.span_x2_us, fixture.mean_rate_q),
    }
}

fn metrics_with_ratio(ratio: u32) -> StationarityMetrics {
    let mut metrics = metrics_fixture(MetricsFixture {
        slope_q: 0,
        slope_half_width_q: 1,
        span_x2_us: 0,
        mean_rate_q: 1_000_000,
        lag_one_passed: true,
    });
    metrics.drift_ratio_ppm = ratio;
    metrics
}

fn settled(mean_rate_q: i64, half_width_rate_q: u64) -> SettledRateContext {
    SettledRateContext {
        mean_rate_q,
        half_width_rate_q,
    }
}

#[test]
fn a_moving_stroke_gates_on_drift_not_on_significance() {
    // Steep but noisy: the significance test passes because the half-width is
    // large, yet the rate moves far enough across the span that no single value
    // describes it. The two gates are different predicates, not a loosening.
    let metrics = metrics_fixture(MetricsFixture {
        slope_q: 5_000_000,
        slope_half_width_q: 9_000_000,
        span_x2_us: 2 * 1_000_000,
        mean_rate_q: 10_000_000,
        lag_one_passed: true,
    });

    assert!(metrics.slope_passed, "fixture must pass the old gate");
    assert_eq!(
        classify_moving_metrics(metrics, settled(10_000_000, 1_000)),
        Err(StationarityExclusion::ExcessiveDrift)
    );
}

#[test]
fn zero_target_rest_keeps_the_significance_test() {
    // At rest the true slope really is zero, so proving it is the right question
    // and the drift bound's denominator would collapse with the mean.
    let metrics = metrics_fixture(MetricsFixture {
        slope_q: 12_000,
        slope_half_width_q: 5_000,
        span_x2_us: 2 * 64_000,
        mean_rate_q: 3,
        lag_one_passed: true,
    });

    assert_eq!(
        classify_rest_metrics(metrics),
        Err(RestExclusion::ResolvedSlope)
    );
}

#[test]
fn an_inert_observation_bypasses_the_drift_gate() {
    // sweep_evidence.rs evaluates StationarityExcluded before Inert, so a
    // zero-rate observation reaches the classifier first. Rejecting it would
    // relabel 15 P-ladder rungs from "did not move" to "excessive drift".
    let metrics = metrics_fixture(MetricsFixture {
        slope_q: 400,
        slope_half_width_q: 1_000_000,
        span_x2_us: 2 * 64_000,
        mean_rate_q: 900,
        lag_one_passed: true,
    });

    let settled = settled(900, 1_500); // interval contains zero
    let classified = classify_moving_metrics(metrics, settled)
        .expect("inert observations must not be excluded for drift");
    assert_eq!(classified.drift_ratio_ppm, u32::MAX);
}

/// A capture whose 64-sample tail is stationary and whose 128-sample tail is not.
///
/// The same shape `first_nonstationary_extension_stops_without_stitching` uses:
/// a stationary recent tail preceded by a large step, so the family stops at 64
/// after testing 128.
fn suffix_64_passes_128_fails() -> [RawVelocityObservation; 256] {
    observations::<256>(|index| {
        if index >= 192 {
            1
        } else if index >= 128 {
            -100
        } else {
            1
        }
    })
}

fn poll_to_completion(
    work: &mut StationarityWork,
    observations: &[RawVelocityObservation],
    blocker: &mut DyadicBlocker<LEVELS>,
) -> (StationarityResult, u32) {
    loop {
        match work.poll(observations, blocker).unwrap() {
            StationarityWorkPoll::Pending => {}
            StationarityWorkPoll::Complete {
                result,
                drift_ratio_ppm,
                ..
            } => break (result, drift_ratio_ppm),
            StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                panic!("moving mode returned a scored-rest result")
            }
        }
    }
}

#[test]
fn a_usable_completion_reports_the_selected_suffix_ratio_not_a_later_candidate() {
    // Suffix 64 passes, 128 fails. The reported ratio must be 64's.
    let observations = suffix_64_passes_128_fails();
    let mut work = StationarityWork::default();
    let mut blocker = DyadicBlocker::<LEVELS>::new();
    let (result, envelope_ratio) = poll_to_completion(&mut work, &observations, &mut blocker);
    let StationarityResult::Usable(suffix) = result else {
        panic!("expected a usable suffix");
    };
    assert_eq!(suffix.suffix_len, 64);
    assert_eq!(envelope_ratio, suffix.drift_ratio_ppm);
}

#[test]
fn the_drift_boundary_is_inclusive() {
    let settled = settled(1_000_000, 1);
    for (ratio, expect_ok) in [(199_999u32, true), (200_000, true), (200_001, false)] {
        let metrics = metrics_with_ratio(ratio);
        let outcome = classify_moving_metrics(metrics, settled);
        assert_eq!(outcome.is_ok(), expect_ok, "ratio {ratio}");
    }
}
