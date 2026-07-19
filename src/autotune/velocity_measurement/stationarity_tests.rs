use core::cell::Cell;

use super::{
    StationarityExclusion, StationarityResult, StationarityWork, StationarityWorkPoll,
    VELOCITY_SUFFIX_LENGTHS, VelocityObservationSource, analyze_stationarity_diagnostics,
    analyze_stationarity_with_blocking_variance, select_stationary_suffix,
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

#[test]
fn suffix_family_is_fixed_and_dyadic() {
    assert_eq!(VELOCITY_SUFFIX_LENGTHS, [64, 128, 256]);
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
            StationarityWorkPoll::Complete(result) => break result,
        }
    };

    assert_eq!(actual, expected);
    assert!(polls > 1, "a full capture must not be analyzed in one poll");
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
        if matches!(poll, StationarityWorkPoll::Complete(_)) {
            break;
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
            StationarityWorkPoll::Complete(result) => break result,
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
            StationarityWorkPoll::Complete(result) => break result,
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
fn positive_and_negative_drift_are_resolved() {
    for sign in [1, -1] {
        let samples = observations::<64>(|index| sign * i32::try_from(index).unwrap());
        let exclusion = analyze_stationarity(&samples, 1, 3).unwrap_err();
        assert_eq!(exclusion, StationarityExclusion::ResolvedSlope);
    }
}

#[test]
fn curvature_with_zero_average_slope_has_structured_residuals() {
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
