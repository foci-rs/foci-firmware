use super::super::sweep_evidence::VelocityObservationClass;
use super::super::sweep_evidence::VelocityObservationInput;
use super::super::sweep_plan::{MAX_SWEEP_RUNGS, MIN_STRUCTURAL_RUNGS};
use super::{
    BelowBoundaryDisposition, BreakawaySearchCursor, BreakawaySearchError, BreakawaySearchStep,
    DirectionalBreakaway, ProbeProductivityClass, classify_breakaway_observation,
    resolve_campaign_breakaway,
};
use crate::autotune::velocity_measurement::{
    PlateauSelection, RateBlock, StationarityExclusion, StationarityResult, StationarySuffix,
    VelocityDirection,
};
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
use foci_tmc4671::pid::PidPGain;

/// Maximum probed grid indices retained per simulated directional search. The
/// audited worst case is [`super::super::breakaway_plan::PROBE_MAX_OBSERVATIONS_PER_DIRECTION`]
/// (8); the trace capacity leaves headroom so a runaway stub overflows loudly
/// instead of silently truncating.
const TRACE_CAPACITY: usize = 16;
const INERT: ProbeProductivityClass =
    ProbeProductivityClass::BelowProductive(BelowBoundaryDisposition::Inert);
const PRODUCTIVE: ProbeProductivityClass = ProbeProductivityClass::Productive;

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

#[test]
fn excessive_drift_productivity_preserves_probe_local_precedence() {
    let mut transient = base_input(VelocityDirection::Forward);
    transient.stationarity = StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift);
    transient.disturbance = None;
    transient.disturbance_variance_q2 = 0;
    transient.drift_ratio_ppm = 200_001;
    transient.post_fit_lag_one_state = 2;

    assert_eq!(
        classify_breakaway_observation(transient),
        Ok(ProbeProductivityClass::BelowProductive(
            BelowBoundaryDisposition::Transient
        ))
    );

    let mut inert = transient;
    inert.settled_rate_low_q = -1;
    inert.settled_rate_high_q = 1;
    assert_eq!(
        classify_breakaway_observation(inert),
        Ok(ProbeProductivityClass::BelowProductive(
            BelowBoundaryDisposition::Inert
        ))
    );

    let mut structured = transient;
    structured.post_fit_lag_one_state = 1;
    assert_eq!(
        classify_breakaway_observation(structured),
        Err(BreakawaySearchError::ProbeEvidenceExcluded(
            StationarityExclusion::StructuredResidual
        ))
    );

    let mut unavailable = transient;
    unavailable.post_fit_lag_one_state = 0;
    assert_eq!(
        classify_breakaway_observation(unavailable),
        Err(BreakawaySearchError::InvalidEvidence)
    );

    let mut authority = transient;
    authority.predicted_torque_target_abs = authority.usable_torque_limit;
    assert_eq!(
        classify_breakaway_observation(authority),
        Err(BreakawaySearchError::AuthorityFailure)
    );
}

fn interval(low: i32, high: i32) -> EquivalentTorqueTargetInterval {
    EquivalentTorqueTargetInterval::new(
        EquivalentTorqueTargetQ16::from_raw(low),
        EquivalentTorqueTargetQ16::from_raw(high),
    )
    .unwrap()
}

/// A well-formed forward moving observation; individual fields are overridden per
/// table case.
fn base_input(direction: VelocityDirection) -> VelocityObservationInput {
    let (settled_rate_low_q, settled_rate_high_q, deficit_low_q, deficit_high_q, point, ci) =
        match direction {
            VelocityDirection::Forward => (10, 20, 2, 4, 100, interval(90, 110)),
            VelocityDirection::Reverse => (-20, -10, -4, -2, -100, interval(-110, -90)),
        };
    VelocityObservationInput {
        direction,
        p: PidPGain::new(350).unwrap(),
        stationarity: stationary(),
        settled_rate_low_q,
        settled_rate_high_q,
        settled_rate_mean_q: (settled_rate_low_q + settled_rate_high_q) / 2,
        variance_of_mean_q2: 4,
        deficit_low_q,
        deficit_high_q,
        disturbance: Some((EquivalentTorqueTargetQ16::from_raw(point), ci)),
        disturbance_variance_q2: 4,
        predicted_torque_target_abs: 500,
        usable_torque_limit: 900,
        hard_torque_limit: 1_000,
        initial_lead_in: false,
        drift_ratio_ppm: u32::MAX,
        post_fit_lag_one_state: 2,
    }
}

// --- RED table: sign-correct interval classification ----------------------------

#[test]
fn observation_classification_table() {
    struct Case {
        name: &'static str,
        input: VelocityObservationInput,
        expected: Result<ProbeProductivityClass, BreakawaySearchError>,
    }

    let interval_contains_zero = {
        let mut input = base_input(VelocityDirection::Forward);
        input.settled_rate_low_q = -5;
        input.settled_rate_high_q = 5;
        input
    };
    let correct_sign_forward = base_input(VelocityDirection::Forward);
    let correct_sign_reverse = base_input(VelocityDirection::Reverse);
    let wrong_sign = {
        let mut input = base_input(VelocityDirection::Forward);
        input.settled_rate_low_q = -20;
        input.settled_rate_high_q = -5;
        input
    };
    let hard_clip = {
        let mut input = base_input(VelocityDirection::Forward);
        input.predicted_torque_target_abs = input.hard_torque_limit;
        input
    };
    let stationarity_exclusion = {
        let mut input = base_input(VelocityDirection::Forward);
        input.stationarity = StationarityResult::Excluded(StationarityExclusion::ResolvedSlope);
        input
    };
    let invalid_evidence = {
        let mut input = base_input(VelocityDirection::Forward);
        input.settled_rate_low_q = 20;
        input.settled_rate_high_q = 10;
        input
    };
    let current_headroom = {
        let mut input = base_input(VelocityDirection::Forward);
        input.predicted_torque_target_abs = input.usable_torque_limit;
        input
    };

    let cases = [
        Case {
            name: "interval contains zero -> Inert",
            input: interval_contains_zero,
            expected: Ok(INERT),
        },
        Case {
            name: "correct-sign excludes zero (forward) -> Moving",
            input: correct_sign_forward,
            expected: Ok(PRODUCTIVE),
        },
        Case {
            name: "correct-sign excludes zero (reverse) -> Moving",
            input: correct_sign_reverse,
            expected: Ok(PRODUCTIVE),
        },
        Case {
            name: "wrong-sign excludes zero -> SafetyFault",
            input: wrong_sign,
            expected: Err(BreakawaySearchError::SafetyFault(
                VelocityObservationClass::WrongDirectionFault,
            )),
        },
        Case {
            name: "hard clip -> SafetyFault",
            input: hard_clip,
            expected: Err(BreakawaySearchError::SafetyFault(
                VelocityObservationClass::HardClipFault,
            )),
        },
        Case {
            name: "stationarity exclusion -> ProbeEvidenceExcluded",
            input: stationarity_exclusion,
            expected: Err(BreakawaySearchError::ProbeEvidenceExcluded(
                StationarityExclusion::ResolvedSlope,
            )),
        },
        Case {
            name: "invalid evidence -> InvalidEvidence",
            input: invalid_evidence,
            expected: Err(BreakawaySearchError::InvalidEvidence),
        },
        Case {
            name: "current headroom -> AuthorityFailure",
            input: current_headroom,
            expected: Err(BreakawaySearchError::AuthorityFailure),
        },
    ];

    for case in cases {
        assert_eq!(
            classify_breakaway_observation(case.input),
            case.expected,
            "case failed: {}",
            case.name
        );
    }
}

// --- Binary-search state machine -------------------------------------------------

/// Drive a directional search against a monotone simulated axis and return the
/// full sequence of probed grid indices alongside the resolved outcome.
///
/// `true_breakaway` is the smallest index that reads `Moving`; `None` means every
/// index in `0..rung_count` reads `Inert` (the axis never moves within the grid).
fn simulate_directional_search(
    rung_count: u8,
    true_breakaway: Option<u8>,
) -> (
    heapless::Vec<u8, TRACE_CAPACITY>,
    Result<DirectionalBreakaway, BreakawaySearchError>,
) {
    let (mut cursor, first_index) =
        BreakawaySearchCursor::start(rung_count).expect("valid rung count");
    let mut trace: heapless::Vec<u8, TRACE_CAPACITY> = heapless::Vec::new();
    let mut next_index = first_index;
    loop {
        trace
            .push(next_index)
            .expect("simulated search stayed within the trace capacity");
        let class = match true_breakaway {
            Some(threshold) if next_index >= threshold => PRODUCTIVE,
            _ => INERT,
        };
        match cursor.advance(Ok(class)) {
            Ok(BreakawaySearchStep::Probe(index)) => next_index = index,
            Ok(BreakawaySearchStep::Resolved(breakaway)) => return (trace, Ok(breakaway)),
            Err(error) => return (trace, Err(error)),
        }
    }
}

#[test]
fn bisection_confirms_each_bracket_endpoint_exactly_once_after_convergence() {
    let (trace, result) = simulate_directional_search(20, Some(14));
    let breakaway = result.expect("monotone axis resolves");

    assert_eq!(breakaway.moving_index, 14);
    assert_eq!(breakaway.inert_index, Some(13));
    assert_eq!(usize::from(breakaway.observations), trace.len());

    // Every probed index before the mandatory bracket repeat is distinct: nothing
    // is retested until the search has already converged on a bracket.
    let confirm_len = 2; // both 13 and 14 are representable here.
    let bisection_prefix = &trace[..trace.len() - confirm_len];
    for (position, index) in bisection_prefix.iter().enumerate() {
        assert!(
            !bisection_prefix[..position].contains(index),
            "index {index} retested during bisection: {trace:?}"
        );
    }
    // The mandatory repeat confirms inert (13) then moving (14).
    assert_eq!(&trace[trace.len() - confirm_len..], [13, 14]);
}

#[test]
fn all_inert_search_reports_no_productive_response_at_authority() {
    let (trace, result) = simulate_directional_search(20, None);
    assert_eq!(
        result,
        Err(BreakawaySearchError::NoProductiveResponseAtAuthority)
    );
    assert!(trace.len() <= 8, "trace: {trace:?}");
}

#[test]
fn all_moving_search_is_left_censored_at_the_grid_floor() {
    let (trace, result) = simulate_directional_search(20, Some(0));
    let breakaway = result.expect("monotone axis resolves");

    assert_eq!(breakaway.moving_index, 0);
    assert_eq!(breakaway.inert_index, None);
    // Only one representable endpoint: the mandatory repeat confirms index 0 once.
    assert_eq!(trace.last(), Some(&0));
    assert_eq!(usize::from(breakaway.observations), trace.len());
}

#[test]
fn arithmetic_boundary_at_grid_floor_reports_the_missing_bracket_side() {
    for rung_count in [MIN_STRUCTURAL_RUNGS as u8, MAX_SWEEP_RUNGS as u8] {
        let (_, result) = simulate_directional_search(rung_count, Some(0));
        let breakaway = result.expect("monotone axis resolves");
        assert_eq!(breakaway.inert_index, None, "rung_count={rung_count}");
        assert_eq!(breakaway.moving_index, 0, "rung_count={rung_count}");
    }
}

#[test]
fn exhaustive_search_resolves_every_monotone_axis_within_the_eight_observation_budget() {
    for rung_count in MIN_STRUCTURAL_RUNGS..=MAX_SWEEP_RUNGS {
        let rung_count = rung_count as u8;
        for true_breakaway in 0..=rung_count {
            let breakaway_option = if true_breakaway == rung_count {
                None
            } else {
                Some(true_breakaway)
            };
            let (trace, result) = simulate_directional_search(rung_count, breakaway_option);
            assert!(
                trace.len() <= 8,
                "rung_count={rung_count} breakaway={true_breakaway:?} trace={trace:?}"
            );
            match breakaway_option {
                None => {
                    assert_eq!(
                        result,
                        Err(BreakawaySearchError::NoProductiveResponseAtAuthority),
                        "rung_count={rung_count}"
                    );
                }
                Some(expected_moving) => {
                    let breakaway = result.unwrap_or_else(|error| {
                        panic!(
                            "rung_count={rung_count} breakaway={expected_moving} failed: {error:?}"
                        )
                    });
                    assert_eq!(
                        breakaway.moving_index, expected_moving,
                        "rung_count={rung_count}"
                    );
                    assert_eq!(
                        breakaway.inert_index,
                        expected_moving.checked_sub(1),
                        "rung_count={rung_count}"
                    );
                    assert_eq!(usize::from(breakaway.observations), trace.len());
                }
            }
        }
    }
}

#[test]
fn full_max_sweep_rungs_grid_resolves_within_the_eight_observation_budget() {
    // The full MAX_SWEEP_RUNGS grid is the worst case the family-16 probe budget was
    // raised to cover: 6 binary-search bisections plus the 2 mandatory
    // bracket-confirmation repeats.
    let rung_count = MAX_SWEEP_RUNGS as u8;
    for true_breakaway in 0..rung_count {
        let (trace, result) = simulate_directional_search(rung_count, Some(true_breakaway));
        let breakaway =
            result.unwrap_or_else(|error| panic!("breakaway={true_breakaway} failed: {error:?}"));
        assert_eq!(breakaway.moving_index, true_breakaway);
        assert!(
            trace.len() <= 8,
            "breakaway={true_breakaway} trace={trace:?}"
        );
    }
}

#[test]
fn contradictory_inert_repeat_yields_unstable_breakaway() {
    let (mut cursor, mut next_index) = BreakawaySearchCursor::start(20).unwrap();
    // Bisect down to the 13/14 bracket exactly as in the deterministic trace above.
    let scripted_bisection = [(10, false), (15, true), (13, false), (14, true)];
    for (expected_index, moving) in scripted_bisection {
        assert_eq!(next_index, expected_index);
        let class = if moving { PRODUCTIVE } else { INERT };
        match cursor.advance(Ok(class)).unwrap() {
            BreakawaySearchStep::Probe(index) => next_index = index,
            BreakawaySearchStep::Resolved(_) => panic!("resolved before the mandatory repeat"),
        }
    }
    // The bisection converged and the cursor now expects the inert repeat at 13.
    assert_eq!(next_index, 13);
    let outcome = cursor.advance(Ok(PRODUCTIVE));
    assert_eq!(outcome, Err(BreakawaySearchError::UnstableBreakaway));
}

#[test]
fn contradictory_moving_repeat_yields_unstable_breakaway() {
    let (mut cursor, mut next_index) = BreakawaySearchCursor::start(20).unwrap();
    let scripted_bisection = [(10, false), (15, true), (13, false), (14, true)];
    for (expected_index, moving) in scripted_bisection {
        assert_eq!(next_index, expected_index);
        let class = if moving { PRODUCTIVE } else { INERT };
        match cursor.advance(Ok(class)).unwrap() {
            BreakawaySearchStep::Probe(index) => next_index = index,
            BreakawaySearchStep::Resolved(_) => panic!("resolved before the mandatory repeat"),
        }
    }
    assert_eq!(next_index, 13);
    match cursor.advance(Ok(INERT)).unwrap() {
        BreakawaySearchStep::Probe(index) => next_index = index,
        BreakawaySearchStep::Resolved(_) => panic!("resolved before the moving repeat"),
    }
    assert_eq!(next_index, 14);
    let outcome = cursor.advance(Ok(INERT));
    assert_eq!(outcome, Err(BreakawaySearchError::UnstableBreakaway));
}

#[test]
fn lower_endpoint_repeat_accumulates_inert_and_transient_dispositions() {
    let (mut cursor, mut next_index) = BreakawaySearchCursor::start(20).unwrap();
    let scripted_bisection = [(10, INERT), (15, PRODUCTIVE), (13, INERT), (14, PRODUCTIVE)];
    for (expected_index, class) in scripted_bisection {
        assert_eq!(next_index, expected_index);
        match cursor.advance(Ok(class)).unwrap() {
            BreakawaySearchStep::Probe(index) => next_index = index,
            BreakawaySearchStep::Resolved(_) => panic!("resolved before the mandatory repeat"),
        }
    }
    assert_eq!(next_index, 13);
    match cursor
        .advance(Ok(ProbeProductivityClass::BelowProductive(
            BelowBoundaryDisposition::Transient,
        )))
        .unwrap()
    {
        BreakawaySearchStep::Probe(index) => next_index = index,
        BreakawaySearchStep::Resolved(_) => panic!("resolved before the productive repeat"),
    }
    assert_eq!(next_index, 14);
    let BreakawaySearchStep::Resolved(resolved) = cursor.advance(Ok(PRODUCTIVE)).unwrap() else {
        panic!("the confirmed productive endpoint must resolve the bracket");
    };
    assert_eq!(resolved.lower_disposition_mask, 0b11);
}

// --- FIX E: all-inert bisection must confirm the authority boundary once ---------

/// Drive an all-inert directional search up to (but not through) the mandatory
/// authority-boundary repeat, and return the cursor positioned to receive that
/// repeat's classified outcome.
///
/// Bisection never retests an index while narrowing the window (each fold
/// strictly increases `lo`, so the next midpoint is always new), so the first
/// repeated probe index in the returned sequence is unambiguously the
/// authority-boundary repeat, not a coincidental bisection re-test.
fn all_inert_cursor_at_the_repeat(rung_count: u8) -> (BreakawaySearchCursor, u8) {
    let (mut cursor, mut next_index) = BreakawaySearchCursor::start(rung_count).unwrap();
    loop {
        let previous_index = next_index;
        match cursor.advance(Ok(INERT)).unwrap() {
            BreakawaySearchStep::Probe(index) => next_index = index,
            BreakawaySearchStep::Resolved(breakaway) => {
                panic!("an all-inert grid must never resolve a moving bracket: {breakaway:?}")
            }
        }
        if next_index == previous_index {
            // The repeat targets the same index the last bisection probe just
            // tested; that repeated index is what the caller's next `advance`
            // call is meant to confirm or contradict.
            break;
        }
    }
    (cursor, next_index)
}

#[test]
fn all_inert_repeat_re_probes_the_authority_endpoint() {
    let rung_count = 20;
    let (_cursor, pending_index) = all_inert_cursor_at_the_repeat(rung_count);
    assert_eq!(
        pending_index,
        rung_count - 1,
        "the mandatory repeat must target the top rung, the sole representable \
         boundary endpoint when every rung tested inert"
    );
}

#[test]
fn all_inert_confirming_repeat_yields_no_productive_response_at_authority() {
    let rung_count = 20;
    let (mut cursor, pending_index) = all_inert_cursor_at_the_repeat(rung_count);
    assert_eq!(pending_index, rung_count - 1);
    let outcome = cursor.advance(Ok(INERT));
    assert_eq!(
        outcome,
        Err(BreakawaySearchError::NoProductiveResponseAtAuthority)
    );
}

#[test]
fn all_inert_contradictory_repeat_yields_unstable_breakaway() {
    let rung_count = 20;
    let (mut cursor, pending_index) = all_inert_cursor_at_the_repeat(rung_count);
    assert_eq!(pending_index, rung_count - 1);
    // The repeat now reads moving: the earlier inert read at the top rung was not
    // reproducible, so the search must not silently accept it.
    let outcome = cursor.advance(Ok(PRODUCTIVE));
    assert_eq!(outcome, Err(BreakawaySearchError::UnstableBreakaway));
}

#[test]
fn a_safety_fault_short_circuits_the_search_immediately() {
    let (mut cursor, _first_index) = BreakawaySearchCursor::start(20).unwrap();
    let outcome = cursor.advance(Err(BreakawaySearchError::SafetyFault(
        VelocityObservationClass::WrongDirectionFault,
    )));
    assert_eq!(
        outcome,
        Err(BreakawaySearchError::SafetyFault(
            VelocityObservationClass::WrongDirectionFault
        ))
    );
}

#[test]
fn zero_rung_count_is_rejected_before_any_probe() {
    assert_eq!(
        BreakawaySearchCursor::start(0).map(|(_, index)| index),
        Err(BreakawaySearchError::InvalidRequest)
    );
}

#[test]
fn the_full_max_sweep_rungs_grid_is_accepted_before_any_probe() {
    // The observation budget was raised specifically so the full MAX_SWEEP_RUNGS
    // grid needs no pre-motion refusal; see
    // full_max_sweep_rungs_grid_resolves_within_the_eight_observation_budget for the
    // budget proof.
    assert!(BreakawaySearchCursor::start(MAX_SWEEP_RUNGS as u8).is_ok());
}

// --- Campaign breakaway resolution ------------------------------------------------

fn directional_breakaway(inert_index: Option<u8>, moving_index: u8) -> DirectionalBreakaway {
    DirectionalBreakaway {
        inert_index,
        moving_index,
        observations: 7,
        lower_disposition_mask: u8::from(inert_index.is_some()),
    }
}

#[test]
fn one_grid_step_directional_agreement_resolves_to_the_lower_endpoint() {
    let forward = directional_breakaway(Some(9), 10);
    let reverse = directional_breakaway(Some(10), 11);

    let resolved = resolve_campaign_breakaway(forward, reverse).unwrap();

    assert_eq!(resolved.breakaway_index, 10);
    assert_eq!(resolved.forward, forward);
    assert_eq!(resolved.reverse, reverse);
}

#[test]
fn exact_directional_agreement_resolves_to_the_shared_endpoint() {
    let forward = directional_breakaway(Some(9), 10);
    let reverse = directional_breakaway(Some(9), 10);

    let resolved = resolve_campaign_breakaway(forward, reverse).unwrap();

    assert_eq!(resolved.breakaway_index, 10);
}

#[test]
fn two_grid_step_disagreement_is_rejected() {
    let forward = directional_breakaway(Some(9), 10);
    let reverse = directional_breakaway(Some(11), 12);

    assert_eq!(
        resolve_campaign_breakaway(forward, reverse),
        Err(BreakawaySearchError::DirectionalBreakawayDisagreement)
    );
}

#[test]
fn campaign_resolution_takes_the_lower_endpoint_regardless_of_direction() {
    let forward = directional_breakaway(Some(10), 11);
    let reverse = directional_breakaway(Some(9), 10);

    let resolved = resolve_campaign_breakaway(forward, reverse).unwrap();

    assert_eq!(resolved.breakaway_index, 10);
}
