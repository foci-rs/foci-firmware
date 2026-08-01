use super::super::breakaway_reporting::BreakawayTerminalCause;
use super::super::sweep_consensus::DirectionalConsensusClass;
use super::*;
use crate::autotune::TmcMechanicalRpm;
use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::breakaway_plan::{
    BreakawayProbeContext, plan_breakaway_probe,
};
use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;
use foci_tmc4671::pid::PidPGain;

fn gain(raw: u16) -> PidPGain {
    PidPGain::new(raw).expect("raw gain within PidPGain range")
}

fn test_budget() -> OuterMotionBudget {
    OuterMotionBudget {
        requested_velocity_mrev_s: 6_000,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_000,
        settle_travel_reserve_mrev: 100,
        negative_position_headroom_mrev: 2_000,
        positive_position_headroom_mrev: 2_000,
        max_duration_ms: 60_000,
    }
}

fn test_safety() -> OuterSafetyLimits {
    let mut safety = OuterSafetyLimits::new(current_limit(2_448).raw(), test_budget());
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .expect("valid encoder resolution");
    safety.set_encoder_baseline(0).expect("baseline configured");
    safety
        .arm_safety_observation(0)
        .expect("safety observation armed");
    safety
}

fn test_backend() -> MockBackend {
    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(2_448);
    backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
    backend
}

/// A search grid realized from `gain(1), gain(1_024)`: 20 post-dedupe rungs. Shared
/// by every test below so none of them re-derive the half-octave generator's output.
fn test_plan() -> BreakawayProbePlan {
    plan_breakaway_probe(gain(1), gain(1_024), test_probe_context()).expect("valid probe plan")
}

/// Canonical acquisition context for the shared probe fixture; exact values are
/// arbitrary but fixed.
fn test_probe_context() -> BreakawayProbeContext {
    BreakawayProbeContext {
        workflow_digest: 0xA1B2_C3D4_E5F6_0718,
        target_velocity_rpm: 600,
        hard_torque_limit: 2_816,
        usable_torque_limit: 2_048,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
    }
}

/// Drive a complete probe campaign against a scripted `MockBackend`.
///
/// `is_moving(direction, index)` decides, for the direction and post-dedupe grid
/// index about to be tested, whether that stroke's scripted encoder deltas produce
/// a sign-correct commanded-side settled rate (`true`) or hold still (`false`). The
/// zero-target settle window always holds still, so every scripted stroke confirms
/// rest. `target_velocity_rpm_raw` is small (100) in every test except the
/// `AuthorityFailure` one, which needs a target large enough that its predicted
/// torque crosses the usable-current boundary at the top of the grid.
fn run_probe_with_target(
    plan: BreakawayProbePlan,
    target_velocity_rpm_raw: i32,
    mut is_moving: impl FnMut(VelocityDirection, u8) -> bool,
) -> (BreakawayProbeOutcome, MockBackend) {
    let target_velocity_rpm = TmcMechanicalRpm::from_register(target_velocity_rpm_raw);
    let mut session =
        BreakawayProbeSession::new(plan, target_velocity_rpm, 4_394).expect("valid probe session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BreakawayProbeState::PlannedProbe;
    let mut safety = test_safety();
    let mut backend = test_backend();

    let mut encoder_count: i32 = 0;
    let mut stroke_moving = false;
    let mut stroke_sign = 1_i32;

    for _ in 0..200_000 {
        if matches!(state, BreakawayProbeState::CommandTarget) {
            stroke_moving = is_moving(session.direction, session.pending_index);
            stroke_sign = direction_sign(session.direction);
        }
        let is_capture_phase = matches!(state, BreakawayProbeState::Capture { .. });
        let result = pollster::block_on(step_breakaway_probe(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            BreakawayProbeStep::ReadyNow => {}
            BreakawayProbeStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if is_capture_phase && stroke_moving {
                    encoder_count += stroke_sign;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            BreakawayProbeStep::Done(outcome) => return (outcome, backend),
        }
    }
    panic!("probe run exceeded its bounded step budget");
}

/// [`run_probe_with_target`] at the shared small target every test but
/// `AuthorityFailure` uses.
fn run_probe(
    plan: BreakawayProbePlan,
    is_moving: impl FnMut(VelocityDirection, u8) -> bool,
) -> (BreakawayProbeOutcome, MockBackend) {
    run_probe_with_target(plan, 100, is_moving)
}

// --- RED: no motion is reachable before the plan is admitted ------------------

#[test]
fn no_register_traffic_happens_before_the_planned_probe_state_advances() {
    let plan = test_plan();
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session =
        BreakawayProbeSession::new(plan, target_velocity_rpm, 4_394).expect("valid session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BreakawayProbeState::PlannedProbe;
    let mut safety = test_safety();
    let mut backend = test_backend();

    let result = pollster::block_on(step_breakaway_probe(
        &mut state,
        &mut workspace,
        &mut session,
        &mut safety,
        &mut backend,
    ));

    assert!(matches!(result, BreakawayProbeStep::ReadyNow));
    assert_eq!(state, BreakawayProbeState::WriteGain);
    assert!(
        backend.spi_log.is_empty(),
        "PlannedProbe must not touch the backend"
    );
}

// --- BreakawayFound: both directions agree ------------------------------------

#[test]
fn agreeing_directional_breakaways_resolve_the_campaign() {
    let plan = test_plan();
    let target_index = 10;

    let (outcome, _) = run_probe(plan, |_direction, index| index >= target_index);

    let BreakawayProbeOutcome::BreakawayFound(resolved) = outcome else {
        panic!("expected BreakawayFound, got {outcome:?}");
    };
    assert_eq!(resolved.breakaway_index, target_index);
    assert_eq!(resolved.forward.moving_index, target_index);
    assert_eq!(resolved.reverse.moving_index, target_index);
}

// --- NoMotionAtAuthority: never moving anywhere on the grid -------------------

#[test]
fn never_moving_reaches_no_motion_at_authority() {
    let plan = test_plan();

    let (outcome, _) = run_probe(plan, |_direction, _index| false);

    assert!(matches!(
        outcome,
        BreakawayProbeOutcome::NoMotionAtAuthority
    ));
}

// --- UnstableBreakaway: bracket repeat contradicts the first read -------------

#[test]
fn a_contradictory_bracket_repeat_reports_unstable_breakaway() {
    let plan = test_plan();
    let target_index = 10;
    // The first `moving_index` read at `target_index` is Moving; flip the verdict
    // for every later observation at the same index so the mandatory repeat
    // contradicts it.
    let mut seen_target = false;

    let (outcome, _) = run_probe(plan, move |_direction, index| {
        if index == target_index {
            let first_read = !seen_target;
            seen_target = true;
            first_read
        } else {
            index > target_index
        }
    });

    assert!(matches!(outcome, BreakawayProbeOutcome::UnstableBreakaway));
}

// --- DirectionalBreakawayDisagreement: endpoints differ by more than one step -

#[test]
fn directional_breakaways_more_than_one_step_apart_disagree() {
    let plan = test_plan();
    // Forward's cursor resolves around index 4; reverse (driven independently,
    // after forward completes) resolves around index 12. The endpoints differ by
    // more than one grid step, so the campaign must not pick a side.
    let (outcome, _) = run_probe(plan, |direction, index| match direction {
        VelocityDirection::Forward => index >= 4,
        VelocityDirection::Reverse => index >= 12,
    });

    assert!(matches!(
        outcome,
        BreakawayProbeOutcome::DirectionalBreakawayDisagreement
    ));
}

#[test]
fn directional_breakaways_one_step_apart_take_the_lower_endpoint() {
    let plan = test_plan();
    // Forward resolves at index 4; reverse resolves at index 5 -- exactly one
    // grid step apart, which resolves to the lower endpoint rather than erroring.
    let (outcome, _) = run_probe(plan, |direction, index| match direction {
        VelocityDirection::Forward => index >= 4,
        VelocityDirection::Reverse => index >= 5,
    });

    let BreakawayProbeOutcome::BreakawayFound(resolved) = outcome else {
        panic!("expected BreakawayFound, got {outcome:?}");
    };
    assert_eq!(resolved.breakaway_index, 4);
    assert_eq!(resolved.forward.moving_index, 4);
    assert_eq!(resolved.reverse.moving_index, 5);
}

// --- WrongDirectionFault short-circuits before origin-recovery -----------------

#[test]
fn wrong_direction_short_circuits_without_recovering() {
    let plan = test_plan();
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session =
        BreakawayProbeSession::new(plan, target_velocity_rpm, 4_394).expect("valid session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BreakawayProbeState::PlannedProbe;
    let mut safety = test_safety();
    let mut backend = test_backend();
    let mut encoder_count: i32 = 0;

    loop {
        let is_capture_phase = matches!(state, BreakawayProbeState::Capture { .. });
        let result = pollster::block_on(step_breakaway_probe(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            BreakawayProbeStep::ReadyNow => {}
            BreakawayProbeStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                // The first stroke commands Forward. Move consistently in the
                // opposite direction so the settled-rate CI excludes zero on the
                // wrong side -- WrongDirectionFault, never a moving/inert verdict.
                if is_capture_phase {
                    encoder_count -= 1;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            BreakawayProbeStep::Done(outcome) => {
                let BreakawayProbeOutcome::SafetyFault { error, safety } = outcome else {
                    panic!("expected a safety fault, got {outcome:?}");
                };
                assert_eq!(error, CommissionError::VelocityValidationFailed);
                assert!(safety.is_none());
                break;
            }
        }
    }

    // Exactly one stroke's worth of velocity-target writes: the signed command from
    // CommandTarget and the zero from CommandZero. A third write -- necessarily
    // nonzero -- would be the origin-recovery motion this fault must never reach.
    let target_writes: std::vec::Vec<u32> = backend
        .spi_log
        .iter()
        .filter_map(|op| match op {
            SpiOp::Write(addr, value) if *addr == PID_VELOCITY_TARGET.addr() => Some(*value),
            _ => None,
        })
        .collect();
    assert_eq!(
        target_writes.len(),
        2,
        "no recovery stroke may follow a safety fault: {target_writes:?}"
    );
    assert_ne!(
        target_writes[0], 0,
        "the first write is the commanded stroke"
    );
    assert_eq!(target_writes[1], 0, "the second write is CommandZero");

    // Exactly one gain write: recovery's own `WriteGains` state would add a second.
    let gain_writes = backend
        .spi_log
        .iter()
        .filter(
            |op| matches!(op, SpiOp::Write(addr, _) if *addr == PID_VELOCITY_P_VELOCITY_I.addr()),
        )
        .count();
    assert_eq!(
        gain_writes, 1,
        "no recovery gain write may follow a safety fault"
    );
}

// --- AuthorityFailure: predicted torque crosses the usable-current boundary ----

#[test]
fn predicted_current_headroom_reports_authority_failure_and_does_not_advance() {
    let plan = test_plan();
    // At the top grid rung (index 19, gain 1024) and this elevated target, the
    // predicted torque (`p * target / 256` = `1024 * 560 / 256` = 2240) sits inside
    // `[usable_torque_limit, hard_torque_limit)` = `[2203, 2448)`: above the 9/10
    // usable-current boundary but below the hard clip. Every lower rung's predicted
    // torque stays comfortably under 2203, so only index 19 crosses. Scripting only
    // index 19 as genuinely moving means the classifier only reaches the
    // `CurrentHeadroom` check (which requires `moving == true`) on that one stroke.
    let (outcome, _) = run_probe_with_target(plan, 560, |_direction, index| index >= 19);

    assert!(matches!(outcome, BreakawayProbeOutcome::AuthorityFailure));
}

// --- ProbeEvidenceExcluded: the zero-target settle never confirms rest ---------

#[test]
fn an_unconfirmed_settle_reports_probe_evidence_excluded() {
    let plan = test_plan();
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session =
        BreakawayProbeSession::new(plan, target_velocity_rpm, 4_394).expect("valid session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BreakawayProbeState::PlannedProbe;
    let mut safety = test_safety();
    let mut backend = test_backend();
    let mut encoder_count: i32 = 0;

    loop {
        let is_settle_phase = matches!(state, BreakawayProbeState::SettleCapture { .. });
        let result = pollster::block_on(step_breakaway_probe(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            BreakawayProbeStep::ReadyNow => {}
            BreakawayProbeStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                // Hold still during the moving capture (Inert is fine -- this test
                // is about the settle window, not the moving classification), then
                // drift continuously once the zero-target settle starts so rest is
                // never confirmed.
                if is_settle_phase {
                    encoder_count += 1;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            BreakawayProbeStep::Done(outcome) => {
                assert!(matches!(
                    outcome,
                    BreakawayProbeOutcome::ProbeEvidenceExcluded
                ));
                break;
            }
        }
    }
}

// --- Existing safety faults are returned unchanged -----------------------------

#[test]
fn safety_faults_propagate_unchanged() {
    let plan = test_plan();
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session =
        BreakawayProbeSession::new(plan, target_velocity_rpm, 4_394).expect("valid session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BreakawayProbeState::PlannedProbe;
    let mut safety = test_safety();
    let mut backend = test_backend();

    // Force a quarter-turn fault on the very first Capture sample by jumping the
    // encoder by a full quarter turn between baseline and the first read.
    backend.set_abn_decoder_count(0);
    loop {
        let result = pollster::block_on(step_breakaway_probe(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            BreakawayProbeStep::ReadyNow => {}
            BreakawayProbeStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if matches!(state, BreakawayProbeState::Capture { .. }) {
                    backend.set_abn_decoder_count(1_500);
                }
            }
            BreakawayProbeStep::Done(outcome) => {
                let BreakawayProbeOutcome::SafetyFault { error, safety } = outcome else {
                    panic!("expected a safety fault, got {outcome:?}");
                };
                assert_eq!(error, CommissionError::SafetyEnvelopeViolation);
                let fault = safety.expect("quarter-turn fault carries an OuterSafetyFault");
                assert_eq!(fault.reason, OUTER_SAFETY_FAULT_QUARTER_TURN);
                break;
            }
        }
    }
}

// ================================================================================
// Additive discovery driver
// ================================================================================

/// The additive discovery ladder shared by every discovery test below.
///
/// Floor 181, breakaway 256, ceiling 574 -- the same floor/breakaway span as
/// `breakaway_plan_tests.rs`'s own `generous_request` fixture, narrowed to a
/// ceiling that resolves to exactly `MIN_STRUCTURAL_RUNGS` = 5 rungs
/// (`[181, 256, 362, 468, 574]`) once `workflow_envelope_ms` is tightened to
/// exclude every finer (6+ rung) candidate: at 8 slots/rung and the fixed
/// per-primitive duration, 5 rungs need 49_920 ms and 6 rungs need 59_904 ms, so
/// 50_000 ms admits only 5. Keeping the ladder small keeps every test's bounded
/// step budget small too.
fn discovery_request() -> AdditiveDiscoveryRequest {
    AdditiveDiscoveryRequest {
        floor: gain(181),
        breakaway: gain(256),
        authority_ceiling: gain(574),
        observations_per_direction: 4,
        max_duration_ms: u16::MAX,
        workflow_envelope_ms: 50_000,
        probe_plan_digest: 0,
    }
}

/// Drive a complete additive discovery campaign against a scripted `MockBackend`.
///
/// `response_numerator(direction, rung_index)` returns the per-tick encoder delta,
/// in encoder counts, for the stroke about to run. Every test below shares
/// `target_velocity_rpm_raw = 100` and `counts_per_rev = 4_000` (`test_backend`'s
/// default), so the commanded-side rate is exactly `100 * 4_000 / 60_000 = 20 / 3`
/// counts per 1 ms tick: a numerator of `5` is exactly 75% of target (inside the
/// confirmable 70-80% band), `2` is 30% (below it), and `8` is 120% (above it).
/// `current_limit_raw` lets headroom-focused tests narrow the usable-current
/// boundary without perturbing this ratio -- current headroom depends only on
/// `p * target_rpm`, never on the encoder script.
fn run_discovery(
    request: AdditiveDiscoveryRequest,
    current_limit_raw: u32,
    mut response_numerator: impl FnMut(VelocityDirection, u8) -> i32,
) -> (
    AdditiveDiscoveryOutcome,
    AdditiveDiscoverySession,
    MockBackend,
) {
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session = AdditiveDiscoverySession::new(request, false, target_velocity_rpm, 4_394)
        .expect("valid discovery session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = AdditiveDiscoveryState::PlannedDiscovery;
    let mut safety = test_safety();
    let mut backend = test_backend();
    backend.current_limit_value = current_limit(current_limit_raw);

    let mut encoder_count: i32 = 0;
    let mut stroke_numerator = 0_i32;
    let mut stroke_sign = 1_i32;

    for _ in 0..1_000_000 {
        if matches!(state, AdditiveDiscoveryState::CommandTarget) {
            stroke_numerator = response_numerator(session.direction, session.rung_index);
            stroke_sign = direction_sign(session.direction);
        }
        let is_capture_phase = matches!(state, AdditiveDiscoveryState::Capture { .. });
        let result = pollster::block_on(step_additive_discovery(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            AdditiveDiscoveryStep::ReadyNow => {}
            AdditiveDiscoveryStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if is_capture_phase {
                    encoder_count += stroke_sign * stroke_numerator;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            AdditiveDiscoveryStep::Done(outcome) => return (outcome, session, backend),
        }
    }
    panic!("discovery run exceeded its bounded step budget");
}

// --- RED: no discovery motion is reachable before the plan is admitted --------

#[test]
fn no_discovery_motion_happens_before_the_planned_discovery_state_advances() {
    let request = discovery_request();
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session = AdditiveDiscoverySession::new(request, false, target_velocity_rpm, 4_394)
        .expect("valid discovery session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = AdditiveDiscoveryState::PlannedDiscovery;
    let mut safety = test_safety();
    let mut backend = test_backend();

    let result = pollster::block_on(step_additive_discovery(
        &mut state,
        &mut workspace,
        &mut session,
        &mut safety,
        &mut backend,
    ));

    assert!(matches!(result, AdditiveDiscoveryStep::ReadyNow));
    assert_eq!(state, AdditiveDiscoveryState::WriteGain);
    assert!(
        backend.spi_log.is_empty(),
        "PlannedDiscovery must not touch the backend"
    );
}

// --- Plan-time terminals: no motion at all -------------------------------------

#[test]
fn insufficient_additive_span_is_a_plan_time_terminal_with_no_motion() {
    // Matches `insufficient_additive_span_rejects_a_ceiling_one_step_above_breakaway`
    // in `breakaway_plan_tests.rs`: a ceiling one grid step above breakaway cannot
    // hold five structural rungs.
    let request = AdditiveDiscoveryRequest {
        authority_ceiling: gain(257),
        ..discovery_request()
    };
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);

    let outcome = AdditiveDiscoverySession::new(request, false, target_velocity_rpm, 4_394)
        .expect_err("a ceiling one step above breakaway cannot hold five structural rungs");

    assert!(matches!(
        outcome,
        AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
    ));
}

#[test]
fn additive_ladder_resolution_budget_exceeded_is_a_plan_time_terminal_with_no_motion() {
    // Matches `additive_ladder_resolution_budget_boundary` in
    // `breakaway_plan_tests.rs`: ceiling 3_437 pushes the first additive gap one
    // gain unit past the provisional resolution guard at every candidate rung
    // count, including the finest (32).
    let request = AdditiveDiscoveryRequest {
        authority_ceiling: gain(3_437),
        workflow_envelope_ms: u32::MAX,
        ..discovery_request()
    };
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);

    let outcome = AdditiveDiscoverySession::new(request, false, target_velocity_rpm, 4_394)
        .expect_err("every candidate's first additive gap exceeds the resolution guard");

    assert!(matches!(
        outcome,
        AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded
    ));
}

// --- TargetBandSelected: at least one rung's own response is in 70-80% --------

#[test]
fn an_in_band_rung_produces_target_band_selected() {
    let (outcome, _session, _) =
        run_discovery(discovery_request(), 2_448, |_direction, rung_index| {
            if rung_index == 0 {
                5 // 75%: contained in 70-80%, retained.
            } else {
                8 // 120%: confidently above 80%, stopping exploration here.
            }
        });

    let AdditiveDiscoveryOutcome::TargetBandSelected(collected) = outcome else {
        panic!("expected TargetBandSelected, got {outcome:?}");
    };
    assert_eq!(collected.count(), 1);
    assert_eq!(collected.as_slice()[0].p.raw(), 181);
}

// --- Rung-zero diagnostic: retained regardless of its own InBand verdict ------

#[test]
fn a_moving_rung_zero_retains_its_moving_classification() {
    let (outcome, session, _) =
        run_discovery(discovery_request(), 2_448, |_direction, rung_index| {
            if rung_index == 0 {
                5 // 75%: moving, and InBand -- both facts must be retained independently.
            } else {
                8 // 120%: confidently above 80%, stopping exploration here.
            }
        });
    assert!(matches!(
        outcome,
        AdditiveDiscoveryOutcome::TargetBandSelected(_)
    ));
    let (forward, reverse) = session
        .rung_zero_classification
        .expect("rung zero was walked before the campaign stopped");
    assert_eq!(
        forward,
        crate::commissioning::outer::velocity::breakaway_search::BreakawayMotionClass::Moving
    );
    assert_eq!(
        reverse,
        crate::commissioning::outer::velocity::breakaway_search::BreakawayMotionClass::Moving
    );
}

#[test]
fn an_inert_rung_zero_retains_its_inert_classification() {
    let (outcome, session, _) =
        run_discovery(discovery_request(), 2_448, |_direction, rung_index| {
            if rung_index == 0 {
                0 // No motion at all: inert, and NOT InBand -- discarded from `in_band`,
            // but the diagnostic classification must still survive the reset.
            } else {
                8 // 120%: confidently above 80%, stopping exploration here.
            }
        });
    assert!(matches!(
        outcome,
        AdditiveDiscoveryOutcome::TargetBandSkipped
    ));
    let (forward, reverse) = session
        .rung_zero_classification
        .expect("rung zero was walked before the campaign stopped");
    assert_eq!(
        forward,
        crate::commissioning::outer::velocity::breakaway_search::BreakawayMotionClass::Inert
    );
    assert_eq!(
        reverse,
        crate::commissioning::outer::velocity::breakaway_search::BreakawayMotionClass::Inert
    );
}

// --- Live emission: a discovery terminal bundles every in-band rung's margin --

#[test]
fn a_discovery_terminal_bundles_every_in_band_rung_margin_and_the_rung_zero_diagnostic() {
    let (outcome, session, _) =
        run_discovery(discovery_request(), 2_448, |_direction, rung_index| {
            if rung_index == 0 {
                5 // 75%: contained in 70-80%, retained.
            } else {
                8 // 120%: confidently above 80%, stopping exploration here.
            }
        });
    let inputs = stage_c_inputs(2_448);
    let mut evidence_sequence = 3_u16;
    let advance = discovery_done_advance(outcome, &session, &inputs, &mut evidence_sequence);
    let CampaignAdvance::Emit { now, next } = advance else {
        panic!("expected an Emit advance");
    };
    let BreakawayCampaignEvent::DiscoveryTerminal {
        terminal,
        rungs,
        rung_zero,
    } = now
    else {
        panic!("expected the discovery terminal, got {now:?}");
    };
    assert_eq!(
        rungs.count(),
        1,
        "rung zero's 75% response is the sole in-band rung"
    );
    assert_eq!(rungs.as_slice()[0].p.raw(), 181);
    assert!(
        rung_zero.is_some(),
        "rung zero was walked and its classification retained"
    );
    assert_eq!(terminal.evidence_sequence, 3);
    // TargetBandSelected routes cleanly into confirmation: no campaign-level
    // closure record is due yet.
    assert!(matches!(
        next,
        EmitContinuation::EnterPhase(BreakawayCampaignPhase::Confirmation { .. })
    ));
}

// --- TargetBandSkipped: response crosses 70->80 with no in-band rung ----------

#[test]
fn a_response_crossing_the_band_between_rungs_produces_target_band_skipped() {
    let (outcome, _session, _) =
        run_discovery(discovery_request(), 2_448, |_direction, rung_index| {
            if rung_index == 0 {
                2 // 30%: clearly below the band.
            } else {
                8 // 120%: confidently above the band -- no in-band rung was ever found.
            }
        });

    assert!(matches!(
        outcome,
        AdditiveDiscoveryOutcome::TargetBandSkipped
    ));
}

// --- CurrentHeadroomBeforeBand: current authority binds first -----------------

#[test]
fn current_headroom_before_any_in_band_rung_reports_current_headroom_before_band() {
    // current_limit_raw = 250 makes only the ceiling rung (574) predict a torque
    // inside [225, 250) -- every earlier rung's predicted torque stays well below
    // that usable-current boundary regardless of the encoder script.
    let (outcome, _session, _) =
        run_discovery(discovery_request(), 250, |_direction, rung_index| {
            if rung_index < 4 {
                2 // 30%: clearly below the band, never contained.
            } else {
                1 // any nonzero motion -- current headroom fires from predicted torque.
            }
        });

    assert!(matches!(
        outcome,
        AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
    ));
}

// --- Current headroom above an already-safe candidate retains it --------------

#[test]
fn current_headroom_after_an_in_band_rung_retains_the_candidate() {
    let (outcome, _session, _) =
        run_discovery(discovery_request(), 250, |_direction, rung_index| {
            match rung_index {
                2 => 5, // 75%: contained in 70-80%, retained.
                4 => 1, // any nonzero motion -- current headroom fires at the ceiling rung.
                _ => 2, // 30%: clearly below the band.
            }
        });

    let AdditiveDiscoveryOutcome::TargetBandSelected(collected) = outcome else {
        panic!(
            "expected TargetBandSelected (headroom above a safe candidate must retain it), \
             got {outcome:?}"
        );
    };
    assert_eq!(collected.count(), 1);
    assert_eq!(collected.as_slice()[0].p.raw(), 362);
}

// --- WrongDirectionFault short-circuits before origin-recovery -----------------

#[test]
fn wrong_direction_short_circuits_discovery_without_recovering() {
    let request = discovery_request();
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session = AdditiveDiscoverySession::new(request, false, target_velocity_rpm, 4_394)
        .expect("valid discovery session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = AdditiveDiscoveryState::PlannedDiscovery;
    let mut safety = test_safety();
    let mut backend = test_backend();
    let mut encoder_count: i32 = 0;

    loop {
        let is_capture_phase = matches!(state, AdditiveDiscoveryState::Capture { .. });
        let result = pollster::block_on(step_additive_discovery(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            AdditiveDiscoveryStep::ReadyNow => {}
            AdditiveDiscoveryStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                // The first stroke commands Forward. Move consistently in the
                // opposite direction so the settled-rate CI excludes zero on the
                // wrong side -- WrongDirectionFault, never a moving/inert verdict.
                if is_capture_phase {
                    encoder_count -= 1;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            AdditiveDiscoveryStep::Done(outcome) => {
                let AdditiveDiscoveryOutcome::SafetyFault { error, safety } = outcome else {
                    panic!("expected a safety fault, got {outcome:?}");
                };
                assert_eq!(error, CommissionError::VelocityValidationFailed);
                assert!(safety.is_none());
                break;
            }
        }
    }

    // Exactly one stroke's worth of velocity-target writes: the signed command from
    // CommandTarget and the zero from CommandZero. A third write -- necessarily
    // nonzero -- would be the origin-recovery motion this fault must never reach.
    let target_writes: std::vec::Vec<u32> = backend
        .spi_log
        .iter()
        .filter_map(|op| match op {
            SpiOp::Write(addr, value) if *addr == PID_VELOCITY_TARGET.addr() => Some(*value),
            _ => None,
        })
        .collect();
    assert_eq!(
        target_writes.len(),
        2,
        "no recovery stroke may follow a safety fault: {target_writes:?}"
    );
    assert_ne!(
        target_writes[0], 0,
        "the first write is the commanded stroke"
    );
    assert_eq!(target_writes[1], 0, "the second write is CommandZero");

    // Exactly one gain write: recovery's own `WriteGains` state would add a second.
    let gain_writes = backend
        .spi_log
        .iter()
        .filter(
            |op| matches!(op, SpiOp::Write(addr, _) if *addr == PID_VELOCITY_P_VELOCITY_I.addr()),
        )
        .count();
    assert_eq!(
        gain_writes, 1,
        "no recovery gain write may follow a safety fault"
    );
}

// --- DiscoveryEvidenceExcluded: the zero-target settle never confirms rest ----

#[test]
fn an_unconfirmed_settle_reports_discovery_evidence_excluded() {
    let request = discovery_request();
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let mut session = AdditiveDiscoverySession::new(request, false, target_velocity_rpm, 4_394)
        .expect("valid discovery session");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = AdditiveDiscoveryState::PlannedDiscovery;
    let mut safety = test_safety();
    let mut backend = test_backend();
    let mut encoder_count: i32 = 0;

    loop {
        let is_settle_phase = matches!(state, AdditiveDiscoveryState::SettleCapture { .. });
        let result = pollster::block_on(step_additive_discovery(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            AdditiveDiscoveryStep::ReadyNow => {}
            AdditiveDiscoveryStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                // Hold still during the moving capture (Inert is fine -- this test
                // is about the settle window, not the moving classification), then
                // drift continuously once the zero-target settle starts so rest is
                // never confirmed.
                if is_settle_phase {
                    encoder_count += 1;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            AdditiveDiscoveryStep::Done(outcome) => {
                assert!(matches!(
                    outcome,
                    AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded
                ));
                break;
            }
        }
    }
}

// ================================================================================
// Held-out confirmation driver
// ================================================================================

fn confirmation_plan(p_raw: u16) -> BandConfirmationPlan {
    plan_band_confirmation(42, gain(p_raw)).expect("valid confirmation plan")
}

/// A single collected in-band discovery rung whose own mean/variance
/// reproduce rung 18's spec-section-2 measurement (relative SE 1.022% at
/// 75% response), the same fixture `band_confirmation_tests.rs`'s own
/// `rung_eighteen_class_noise_is_nominatable_at_family_eight` proves
/// nominatable at family eight. `p_raw` only labels which gain nomination
/// selects; it does not affect the mean/variance numbers.
fn confirmable_discovery_rungs(p_raw: u16) -> CollectedInBandRungs {
    const MEAN: i64 = 750_000;
    const VARIANCE: u128 = 58_752_225;
    const TARGET: i64 = 1_000_000;
    let full_consensus = DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    };
    let forward_atom = DiscoveryObservationAtom {
        mean_rate_q: MEAN,
        variance_of_mean_q2: VARIANCE,
        target_rate_q: TARGET,
        direction: VelocityDirection::Forward,
    };
    let reverse_atom = DiscoveryObservationAtom {
        mean_rate_q: -MEAN,
        variance_of_mean_q2: VARIANCE,
        target_rate_q: TARGET,
        direction: VelocityDirection::Reverse,
    };
    let rung = DiscoveryRungNominationInput {
        p: gain(p_raw),
        forward_consensus: full_consensus,
        forward_observations: [Some(forward_atom); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        reverse_consensus: full_consensus,
        reverse_observations: [Some(reverse_atom); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let mut collected = CollectedInBandRungs::new();
    collected
        .push(rung_nomination_bounds(&rung))
        .expect("one rung fits");
    collected
}

/// Drive a complete confirmation campaign against a scripted `MockBackend`,
/// starting from [`begin_band_confirmation`] (so nomination and plan-freeze
/// are exercised too, not just the stroke loop).
///
/// `response_numerator(direction, stroke_index)` returns the per-tick encoder
/// delta for the stroke about to run, matching `run_discovery`'s own
/// convention: at `target_velocity_rpm_raw = 100` and `counts_per_rev =
/// 4_000` (`test_backend`'s default), the commanded-side rate is exactly
/// `100 * 4_000 / 60_000 = 20 / 3` counts per 1 ms tick, so a numerator of `5`
/// is exactly 75% of target (inside the confirmable 70-80% band) and `4` is
/// 60% (below it). Returns the terminal outcome, the backend, the number of
/// `WriteGain` entries observed (exactly one per confirmation stroke,
/// unaffected by however many register writes origin-recovery performs
/// internally), and the sequence of commanded directions in stroke order.
fn run_confirmation(
    discovery_outcome: AdditiveDiscoveryOutcome,
    current_limit_raw: u32,
    mut response_numerator: impl FnMut(VelocityDirection, u8) -> i32,
) -> (
    BandConfirmationOutcome,
    MockBackend,
    u32,
    std::vec::Vec<VelocityDirection>,
) {
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let entry = begin_band_confirmation(discovery_outcome, 42, target_velocity_rpm, 4_394);
    let mut session = match entry {
        BandConfirmationEntry::Ready(session) => session,
        BandConfirmationEntry::Done(outcome) => {
            panic!("expected confirmation motion to begin, got early terminal {outcome:?}")
        }
    };
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BandConfirmationState::PlannedConfirmation;
    let mut safety = test_safety();
    let mut backend = test_backend();
    backend.current_limit_value = current_limit(current_limit_raw);

    let mut encoder_count: i32 = 0;
    let mut stroke_numerator = 0_i32;
    let mut stroke_sign = 1_i32;
    let mut write_gain_count = 0_u32;
    let mut direction_sequence: std::vec::Vec<VelocityDirection> = std::vec::Vec::new();

    for _ in 0..1_000_000 {
        if matches!(state, BandConfirmationState::WriteGain) {
            write_gain_count += 1;
            direction_sequence.push(session.current_direction());
        }
        if matches!(state, BandConfirmationState::CommandTarget) {
            stroke_numerator =
                response_numerator(session.current_direction(), session.stroke_index);
            stroke_sign = direction_sign(session.current_direction());
        }
        let is_capture_phase = matches!(state, BandConfirmationState::Capture { .. });
        let result = pollster::block_on(step_band_confirmation(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            BandConfirmationStep::ReadyNow => {}
            BandConfirmationStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if is_capture_phase {
                    encoder_count += stroke_sign * stroke_numerator;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            BandConfirmationStep::Done(outcome) => {
                return (outcome, backend, write_gain_count, direction_sequence);
            }
        }
    }
    panic!("confirmation run exceeded its bounded step budget");
}

// --- RED: no confirmation motion is reachable before the plan is frozen -------

#[test]
fn no_confirmation_motion_happens_before_the_planned_confirmation_state_advances() {
    let plan = confirmation_plan(724);
    let mut session =
        BandConfirmationSession::new(plan, 5.0, TmcMechanicalRpm::from_register(100), 4_394);
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BandConfirmationState::PlannedConfirmation;
    let mut safety = test_safety();
    let mut backend = test_backend();

    let result = pollster::block_on(step_band_confirmation(
        &mut state,
        &mut workspace,
        &mut session,
        &mut safety,
        &mut backend,
    ));

    assert!(matches!(result, BandConfirmationStep::ReadyNow));
    assert_eq!(state, BandConfirmationState::WriteGain);
    assert!(
        backend.spi_log.is_empty(),
        "PlannedConfirmation must not touch the backend"
    );
}

// --- RED: TargetBandUnconfirmableFromDiscovery spends zero strokes -----------

#[test]
fn target_band_unconfirmable_from_discovery_spends_zero_confirmation_strokes() {
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);

    // An empty in-band rung collection: `nominate_confirmation_candidate`
    // returns `Err(TargetBandUnconfirmableFromDiscovery)` on an empty slice
    // (see `band_confirmation_tests.rs::empty_discovery_ladder_yields_zero_stroke_outcome`).
    let entry = begin_band_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(CollectedInBandRungs::new()),
        42,
        target_velocity_rpm,
        4_394,
    );

    // `begin_band_confirmation` never takes a backend parameter at all, so
    // zero confirmation strokes is a structural guarantee, not merely an
    // observed one -- there is no way for this call to have touched a
    // backend.
    assert!(matches!(
        entry,
        BandConfirmationEntry::Done(BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery)
    ));
}

// --- TargetBandNotReachedAtAuthority: consumed from discovery's own terminal --

#[test]
fn target_band_not_reached_at_authority_consumes_current_headroom_before_band() {
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);

    let entry = begin_band_confirmation(
        AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand,
        42,
        target_velocity_rpm,
        4_394,
    );

    assert!(matches!(
        entry,
        BandConfirmationEntry::Done(BandConfirmationOutcome::TargetBandNotReachedAtAuthority)
    ));
}

// --- Accept: all eight held-out strokes contained -----------------------------

#[test]
fn an_all_eight_contained_run_accepts_with_the_candidate_p_and_the_frozen_plan() {
    let (outcome, _backend, write_gain_count, direction_sequence) = run_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(724)),
        2_448,
        |_direction, _stroke_index| 5, // 75%: comfortably inside 70-80% both ways.
    );

    let BandConfirmationOutcome::Accepted { acceptance, plan } = outcome else {
        panic!("expected Accepted, got {outcome:?}");
    };
    assert_eq!(acceptance.candidate_p(), gain(724));
    assert_eq!(plan.candidate_p, gain(724));
    assert_eq!(plan.discovery_plan_digest, 42);

    assert_eq!(
        write_gain_count, 8,
        "exactly eight strokes must be driven on the accept path"
    );
    assert_eq!(
        direction_sequence,
        std::vec![
            VelocityDirection::Forward,
            VelocityDirection::Forward,
            VelocityDirection::Forward,
            VelocityDirection::Forward,
            VelocityDirection::Reverse,
            VelocityDirection::Reverse,
            VelocityDirection::Reverse,
            VelocityDirection::Reverse,
        ],
        "CONFIRMATION_SLOT_ORDER is four forward strokes then four reverse strokes"
    );
}

// --- Inconclusive: response location -----------------------------------------

#[test]
fn an_out_of_band_response_reports_inconclusive_response_location() {
    let (outcome, _backend, write_gain_count, _directions) = run_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(724)),
        2_448,
        |_direction, _stroke_index| 4, // 60%: clearly below the 70-80% band.
    );

    assert_eq!(
        outcome_cause(&outcome),
        Some(BandConfirmationFailureCause::ResponseLocation)
    );
    assert_eq!(
        write_gain_count, 8,
        "no second-candidate retry may follow an inconclusive result -- the fixed \
         eight-stroke budget is spent exactly once"
    );
}

// --- Inconclusive: current authority ------------------------------------------

#[test]
fn current_headroom_across_the_block_reports_inconclusive_current_authority() {
    // Matches `run_discovery`'s own `current_headroom_before_any_in_band_rung...`
    // fixture exactly: current_limit_raw = 250 and p = 574 predict a torque of
    // 225, landing inside the usable-current boundary regardless of the
    // encoder script. Unlike discovery, `p` and `target_velocity_rpm` are both
    // frozen across the whole eight-stroke block, so this fires identically
    // on every stroke rather than only once current authority is reached.
    let (outcome, _backend, write_gain_count, _directions) = run_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(574)),
        250,
        |_direction, _stroke_index| 1, // any nonzero motion -- headroom fires from predicted torque.
    );

    assert_eq!(
        outcome_cause(&outcome),
        Some(BandConfirmationFailureCause::CurrentAuthority)
    );
    assert_eq!(write_gain_count, 8);
}

// --- Inconclusive: loss of consensus ------------------------------------------

#[test]
fn one_inert_stroke_without_headroom_reports_inconclusive_loss_of_consensus() {
    let (outcome, _backend, write_gain_count, _directions) = run_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(724)),
        2_448,
        |_direction, stroke_index| {
            if stroke_index == 1 {
                0 // holds still: Inert, not Eligible -- breaks the complete mask.
            } else {
                5 // 75%: every other stroke is a clean in-band Eligible observation.
            }
        },
    );

    assert_eq!(
        outcome_cause(&outcome),
        Some(BandConfirmationFailureCause::LossOfConsensus)
    );
    assert_eq!(write_gain_count, 8);
}

// --- Inconclusive: excessive uncertainty --------------------------------------

#[test]
fn excessive_uncertainty_reports_inconclusive_excessive_uncertainty() {
    // Manufacturing genuine sample-to-sample variance through the full
    // stationarity/blocking pipeline is exactly the statistical territory
    // `evaluate_band_confirmation`'s own math already exhaustively covers in
    // `band_confirmation_tests.rs` (see its
    // `confirmation_reports_excessive_uncertainty_when_containment_is_structurally_impossible`).
    // This test instead proves THIS driver's own wiring
    // (`confirmation_evaluation_outcome`) correctly relays that cause, using
    // the exact same relative-SE-5%-at-75% fixture directly, mirroring that
    // test file's own `confirmation_input`-style construction rather than
    // driving a `MockBackend` through the full moving-capture pipeline.
    let plan = confirmation_plan(724);
    const HUGE_VARIANCE: u128 = 1_406_250_000; // (750_000 * 0.05)^2
    let full_consensus = DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    };
    let forward = ConfirmationDirectionalInput {
        consensus: full_consensus,
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q: 750_000,
            variance_of_mean_q2: HUGE_VARIANCE,
            target_rate_q: 1_000_000,
            direction: VelocityDirection::Forward,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let reverse = ConfirmationDirectionalInput {
        consensus: full_consensus,
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q: -750_000,
            variance_of_mean_q2: HUGE_VARIANCE,
            target_rate_q: 1_000_000,
            direction: VelocityDirection::Reverse,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };

    let outcome = confirmation_evaluation_outcome(
        evaluate_band_confirmation(plan.candidate_p, forward, reverse),
        plan,
    );

    assert_eq!(
        outcome_cause(&outcome),
        Some(BandConfirmationFailureCause::ExcessiveUncertainty)
    );
}

/// Extract the failure cause from an inconclusive outcome, or `None`.
fn outcome_cause(outcome: &BandConfirmationOutcome) -> Option<BandConfirmationFailureCause> {
    match outcome {
        BandConfirmationOutcome::TargetBandConfirmationInconclusive(cause) => Some(*cause),
        _ => None,
    }
}

// --- WrongDirectionFault short-circuits before origin-recovery -----------------

#[test]
fn wrong_direction_short_circuits_confirmation_without_recovering() {
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let entry = begin_band_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(724)),
        42,
        target_velocity_rpm,
        4_394,
    );
    let mut session = match entry {
        BandConfirmationEntry::Ready(session) => session,
        BandConfirmationEntry::Done(outcome) => {
            panic!("expected confirmation motion to begin, got early terminal {outcome:?}")
        }
    };
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BandConfirmationState::PlannedConfirmation;
    let mut safety = test_safety();
    let mut backend = test_backend();
    let mut encoder_count: i32 = 0;

    loop {
        let result = pollster::block_on(step_band_confirmation(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            BandConfirmationStep::ReadyNow => {}
            BandConfirmationStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                // The first stroke commands Forward. Move consistently in the
                // opposite direction so the settled-rate CI excludes zero on
                // the wrong side -- WrongDirectionFault, never a moving/inert
                // verdict.
                encoder_count -= 1;
                backend.set_abn_decoder_count(encoder_count);
            }
            BandConfirmationStep::Done(outcome) => {
                let BandConfirmationOutcome::SafetyFault { error, safety } = outcome else {
                    panic!("expected a safety fault, got {outcome:?}");
                };
                assert_eq!(error, CommissionError::VelocityValidationFailed);
                assert!(safety.is_none());
                break;
            }
        }
    }

    // Exactly one stroke's worth of velocity-target writes: the signed
    // command from CommandTarget and the zero from CommandZero. A third
    // write -- necessarily nonzero -- would be the origin-recovery motion
    // this fault must never reach.
    let target_writes: std::vec::Vec<u32> = backend
        .spi_log
        .iter()
        .filter_map(|op| match op {
            SpiOp::Write(addr, value) if *addr == PID_VELOCITY_TARGET.addr() => Some(*value),
            _ => None,
        })
        .collect();
    assert_eq!(
        target_writes.len(),
        2,
        "no recovery stroke may follow a safety fault: {target_writes:?}"
    );
    assert_ne!(
        target_writes[0], 0,
        "the first write is the commanded stroke"
    );
    assert_eq!(target_writes[1], 0, "the second write is CommandZero");

    // Exactly one gain write: recovery's own `WriteGains` state would add a
    // second.
    let gain_writes = backend
        .spi_log
        .iter()
        .filter(
            |op| matches!(op, SpiOp::Write(addr, _) if *addr == PID_VELOCITY_P_VELOCITY_I.addr()),
        )
        .count();
    assert_eq!(
        gain_writes, 1,
        "no recovery gain write may follow a safety fault"
    );
}

// --- ConfirmationEvidenceExcluded: the zero-target settle never confirms rest -

#[test]
fn an_unconfirmed_settle_reports_confirmation_evidence_excluded() {
    let target_velocity_rpm = TmcMechanicalRpm::from_register(100);
    let entry = begin_band_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(724)),
        42,
        target_velocity_rpm,
        4_394,
    );
    let mut session = match entry {
        BandConfirmationEntry::Ready(session) => session,
        BandConfirmationEntry::Done(outcome) => {
            panic!("expected confirmation motion to begin, got early terminal {outcome:?}")
        }
    };
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut state = BandConfirmationState::PlannedConfirmation;
    let mut safety = test_safety();
    let mut backend = test_backend();
    let mut encoder_count: i32 = 0;

    loop {
        let is_settle_phase = matches!(state, BandConfirmationState::SettleCapture { .. });
        let result = pollster::block_on(step_band_confirmation(
            &mut state,
            &mut workspace,
            &mut session,
            &mut safety,
            &mut backend,
        ));
        match result {
            BandConfirmationStep::ReadyNow => {}
            BandConfirmationStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                // Hold still during the moving capture (Inert is fine -- this
                // test is about the settle window, not the moving
                // classification), then drift continuously once the
                // zero-target settle starts so rest is never confirmed.
                if is_settle_phase {
                    encoder_count += 1;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            BandConfirmationStep::Done(outcome) => {
                assert!(matches!(
                    outcome,
                    BandConfirmationOutcome::ConfirmationEvidenceExcluded
                ));
                break;
            }
        }
    }
}

// ================================================================================
// 6e-1: discovery RAM reduction
// ================================================================================

/// `CollectedInBandRungs` must have shrunk from the raw-atom representation's
/// 32 * 448 B = 14,352 B to a few hundred bytes: 32 rungs of the compact
/// `DiscoveryRungNominationBounds` record (`p: PidPGain`, two `f32` bounds,
/// and a `bool`), not the raw per-direction observation atoms and consensus.
#[test]
fn collected_in_band_rungs_shrinks_to_the_compact_bounds_representation() {
    use core::mem::size_of;

    let compact_record_size = size_of::<DiscoveryRungNominationBounds>();
    let raw_atom_rung_size = size_of::<DiscoveryRungNominationInput>();
    let collected_size = size_of::<CollectedInBandRungs>();

    // Pinned exact sizes (not just bounds) so a future change that grows either
    // representation is caught immediately rather than drifting under a loose
    // inequality. `DiscoveryRungNominationBounds` is `p: PidPGain` (u16) + two
    // `f32` bounds + a `bool`, rounded up to 4-byte alignment: 12 B, not the
    // ~12-13 B estimate's upper edge. `CollectedInBandRungs` is 32 rungs of
    // that record plus a `u8` count, rounded up to 4-byte alignment: 388 B,
    // down from the raw-atom representation's 32 * 448 B = 14,352 B.
    assert_eq!(compact_record_size, 12);
    assert_eq!(raw_atom_rung_size, 448);
    assert_eq!(collected_size, 388);
    assert!(
        collected_size < MAX_SWEEP_RUNGS * raw_atom_rung_size,
        "the reduction must actually shrink retained discovery evidence"
    );
}

// ================================================================================
// 6e-2a: campaign orchestrator + Stage-C token route
// ================================================================================

use super::{
    CampaignAdvance, EmitContinuation, confirmation_terminal, discovery_done_advance,
    route_confirmed_accept, route_discovery_outcome,
};
use crate::commissioning::outer::state::OuterPhase;
use crate::commissioning::outer::velocity::current_guard::VelocityLimitEvidenceCapability;
use crate::commissioning::outer::velocity::sweep_plan::current_gain_span;
use crate::commissioning::types::StepResult;

/// A well-formed campaign request at the shared small target.
fn campaign_request() -> BreakawayCampaignRequest {
    BreakawayCampaignRequest {
        target_velocity_rpm: TmcMechanicalRpm::from_register(100),
        planned_velocity_mrev_s: 4_394,
        hard_torque_limit: 2_448,
        limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
        max_duration_ms: 3_000,
        nominal_workflow_ms: 400_000,
        maximum_workflow_ms: 450_000,
        channel: 0,
        run_sequence: 7,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
    }
}

/// Frozen inputs for the Stage-C token route, with a caller-chosen usable torque
/// limit so a test can either admit or refuse the confirmed P at plan time.
fn stage_c_inputs(usable_torque_limit: u16) -> BreakawayCampaignInputs {
    BreakawayCampaignInputs {
        p_start: gain(1),
        p_top: gain(1_024),
        target_velocity_rpm: TmcMechanicalRpm::from_register(100),
        planned_velocity_mrev_s: 4_394,
        hard_torque_limit: 2_448,
        usable_torque_limit,
        limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
        observations_per_direction: 4,
        max_duration_ms: 3_000,
        workflow_envelope_ms: 450_000,
        channel: 0,
        run_sequence: 7,
        // Match `test_backend`'s live PWM (default_for_test => 25 kHz) and
        // `test_safety`'s live encoder resolution (4_000), so the Stage-C gate's
        // plan-time-vs-live physical-context guard admits the confirmed P.
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
        ceiling_binding_source:
            crate::commissioning::outer::velocity::sweep_plan::CEILING_BINDING_CURRENT_LIMIT,
    }
}

/// Drive a campaign against a scripted `MockBackend`, applying the probe's
/// `is_moving` convention while the campaign is in its probe phase. Returns the
/// campaign (for phase inspection) and the terminal step, if one is reached
/// before the phase-transition sentinel `stop_at_discovery` fires.
fn run_campaign_probe(
    request: BreakawayCampaignRequest,
    stop_at_discovery: bool,
    mut is_moving: impl FnMut(VelocityDirection, u8) -> bool,
) -> (BreakawayCampaign, Option<BreakawayCampaignOutcome>) {
    let mut campaign = BreakawayCampaign::new(request).expect("valid campaign request");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut safety = test_safety();
    let mut backend = test_backend();

    let mut encoder_count: i32 = 0;
    let mut stroke_moving = false;
    let mut stroke_sign = 1_i32;

    for _ in 0..1_000_000 {
        if let BreakawayCampaignPhase::Probe {
            state: BreakawayProbeState::CommandTarget,
            session,
        } = campaign.phase()
        {
            stroke_moving = is_moving(session.direction, session.pending_index);
            stroke_sign = direction_sign(session.direction);
        }
        let is_capture_phase = matches!(
            campaign.phase(),
            BreakawayCampaignPhase::Probe {
                state: BreakawayProbeState::Capture { .. },
                ..
            }
        );
        let step = pollster::block_on(step_breakaway_campaign(
            &mut campaign,
            &mut workspace,
            &mut safety,
            &mut backend,
        ));
        match step {
            BreakawayCampaignStep::ReadyNow | BreakawayCampaignStep::Event(_) => {
                if stop_at_discovery
                    && matches!(campaign.phase(), BreakawayCampaignPhase::Discovery { .. })
                {
                    return (campaign, None);
                }
            }
            BreakawayCampaignStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if is_capture_phase && stroke_moving {
                    encoder_count += stroke_sign;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            BreakawayCampaignStep::Done(outcome) => return (campaign, Some(outcome)),
        }
    }
    panic!("campaign run exceeded its bounded step budget");
}

/// Drive a campaign exactly like [`run_campaign_probe`], additionally
/// collecting every emitted [`BreakawayCampaignEvent`] in transition order.
fn run_campaign_probe_collecting_events(
    request: BreakawayCampaignRequest,
    stop_at_discovery: bool,
    mut is_moving: impl FnMut(VelocityDirection, u8) -> bool,
) -> (
    BreakawayCampaign,
    Option<BreakawayCampaignOutcome>,
    heapless::Vec<BreakawayCampaignEvent, 64>,
) {
    let mut campaign = BreakawayCampaign::new(request).expect("valid campaign request");
    let mut workspace = BreakawayProbeWorkspace::new();
    let mut safety = test_safety();
    let mut backend = test_backend();
    let mut events: heapless::Vec<BreakawayCampaignEvent, 64> = heapless::Vec::new();

    let mut encoder_count: i32 = 0;
    let mut stroke_moving = false;
    let mut stroke_sign = 1_i32;

    for _ in 0..1_000_000 {
        if let BreakawayCampaignPhase::Probe {
            state: BreakawayProbeState::CommandTarget,
            session,
        } = campaign.phase()
        {
            stroke_moving = is_moving(session.direction, session.pending_index);
            stroke_sign = direction_sign(session.direction);
        }
        let is_capture_phase = matches!(
            campaign.phase(),
            BreakawayCampaignPhase::Probe {
                state: BreakawayProbeState::Capture { .. },
                ..
            }
        );
        let step = pollster::block_on(step_breakaway_campaign(
            &mut campaign,
            &mut workspace,
            &mut safety,
            &mut backend,
        ));
        match step {
            BreakawayCampaignStep::ReadyNow => {
                if stop_at_discovery
                    && matches!(campaign.phase(), BreakawayCampaignPhase::Discovery { .. })
                {
                    return (campaign, None, events);
                }
            }
            BreakawayCampaignStep::Event(event) => {
                events
                    .push(event)
                    .expect("test campaign emits fewer than 64 events");
            }
            BreakawayCampaignStep::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if is_capture_phase && stroke_moving {
                    encoder_count += stroke_sign;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            BreakawayCampaignStep::Done(outcome) => return (campaign, Some(outcome), events),
        }
    }
    panic!("campaign run exceeded its bounded step budget");
}

// --- RED: no moving state is reachable before its Planned event ---------------

#[test]
fn a_new_campaign_starts_planned_and_its_first_step_issues_no_motion() {
    let mut campaign = BreakawayCampaign::new(campaign_request()).expect("valid campaign request");
    assert!(matches!(
        campaign.phase(),
        BreakawayCampaignPhase::Probe {
            state: BreakawayProbeState::PlannedProbe,
            ..
        }
    ));

    let mut workspace = BreakawayProbeWorkspace::new();
    let mut safety = test_safety();
    let mut backend = test_backend();
    let step = pollster::block_on(step_breakaway_campaign(
        &mut campaign,
        &mut workspace,
        &mut safety,
        &mut backend,
    ));

    let BreakawayCampaignStep::Event(BreakawayCampaignEvent::ProbePlanned(report)) = step else {
        panic!("expected the probe-planned event, got {step:?}");
    };
    assert_eq!(report.p_start_raw, campaign_request_p_start_raw());
    assert!(matches!(
        campaign.phase(),
        BreakawayCampaignPhase::Probe {
            state: BreakawayProbeState::WriteGain,
            ..
        }
    ));
    assert!(
        backend.spi_log.is_empty(),
        "PlannedProbe must not touch the backend before its first stroke"
    );
}

/// `p_start` the shared `campaign_request()` resolves to, independently
/// derived via the same `current_gain_span` the campaign itself calls.
fn campaign_request_p_start_raw() -> u16 {
    let (p_start, _p_top, _usable, _ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(100), 2_448)
            .expect("the shared campaign_request's target/limit resolve a gain span");
    p_start.raw()
}

// --- Probe -> Discovery: a resolved breakaway freezes the discovery plan -------

#[test]
fn a_resolved_probe_breakaway_advances_the_campaign_into_the_discovery_phase() {
    // A breakaway high on the grid keeps the additive discovery span (breakaway
    // -> p_top) inside the ladder's resolution budget, so discovery is planned
    // and entered rather than refused.
    let (campaign, terminal) =
        run_campaign_probe(campaign_request(), true, |_direction, index| index >= 18);

    assert!(
        terminal.is_none(),
        "the campaign must reach discovery, not a terminal: {terminal:?}"
    );
    let BreakawayCampaignPhase::Discovery { state, session } = campaign.phase() else {
        panic!("expected the discovery phase, got {:?}", campaign.phase());
    };
    // Each moving state is reachable only after its Planned event: discovery is
    // entered at PlannedDiscovery with no stroke issued yet.
    assert_eq!(*state, AdditiveDiscoveryState::PlannedDiscovery);
    // The discovery ceiling is the retained plan-time p_top, not a probe-observed
    // clamp: the ladder's exact final anchor is the campaign's p_top.
    let (_p_start, p_top, _usable, _ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(100), 2_448).unwrap();
    assert_eq!(
        session.plan.gains[usize::from(session.plan.rung_count) - 1],
        p_top
    );
}

// --- Live emission: probe -> discovery reports plan, brackets, plan, in order -

#[test]
fn a_resolved_probe_breakaway_emits_its_plan_brackets_and_the_discovery_plan_in_order() {
    let (_campaign, terminal, events) =
        run_campaign_probe_collecting_events(campaign_request(), true, |_direction, index| {
            index >= 18
        });

    assert!(
        terminal.is_none(),
        "the campaign must reach discovery, not a terminal: {terminal:?}"
    );
    assert_eq!(
        events.len(),
        3,
        "expected probe-planned, directional-breakaway, discovery-planned: {events:?}"
    );
    let BreakawayCampaignEvent::ProbePlanned(probe_planned) = &events[0] else {
        panic!("expected the probe plan first, got {:?}", events[0]);
    };
    // The probe plan is frozen and reported BEFORE its first stroke: no SPI
    // traffic precedes it (`no_register_traffic_happens_before_the_planned_probe_state_advances`
    // covers the same invariant at the probe-driver level).
    assert_eq!(probe_planned.evidence_sequence, 0);

    let BreakawayCampaignEvent::DirectionalBreakaway { forward, reverse } = &events[1] else {
        panic!(
            "expected the resolved breakaway brackets second, got {:?}",
            events[1]
        );
    };
    assert_eq!(forward.plan_digest, probe_planned.plan_digest);
    assert_eq!(reverse.plan_digest, probe_planned.plan_digest);
    assert_eq!(forward.direction, 0);
    assert_eq!(reverse.direction, 1);

    let BreakawayCampaignEvent::DiscoveryPlanned {
        planned,
        ceiling_source,
        plan,
    } = &events[2]
    else {
        panic!("expected the discovery plan third, got {:?}", events[2]);
    };
    assert_eq!(planned.prior_plan_digest, probe_planned.plan_digest);
    assert_eq!(ceiling_source.plan_digest, planned.plan_digest);
    assert_eq!(
        ceiling_source.binding_source,
        crate::commissioning::outer::velocity::sweep_plan::CEILING_BINDING_CURRENT_LIMIT
    );
    assert_eq!(plan.plan_digest, planned.plan_digest);
}

// --- Live emission: a probe failure reports its terminal, then the campaign's -

#[test]
fn a_probe_failure_emits_its_terminal_then_the_campaign_closure_record() {
    let (_campaign, terminal, events) =
        run_campaign_probe_collecting_events(campaign_request(), false, |_direction, _index| false);

    assert!(terminal.is_some(), "a never-moving probe must terminate");
    // ProbePlanned, then ProbeTerminal, then the campaign's own closure record;
    // the campaign's Done outcome itself is returned the tick after that.
    assert_eq!(events.len(), 3, "unexpected event sequence: {events:?}");
    assert!(matches!(events[0], BreakawayCampaignEvent::ProbePlanned(_)));
    let BreakawayCampaignEvent::ProbeTerminal(probe_terminal) = &events[1] else {
        panic!("expected the probe terminal second, got {:?}", events[1]);
    };
    assert_ne!(
        probe_terminal.terminal_cause, 0,
        "a NoMotionAtAuthority terminal must carry a nonzero cause"
    );
    let BreakawayCampaignEvent::CampaignTerminal(campaign_terminal) = &events[2] else {
        panic!(
            "expected the campaign closure record third, got {:?}",
            events[2]
        );
    };
    assert_eq!(
        campaign_terminal.terminal_cause,
        probe_terminal.terminal_cause
    );
    assert_eq!(campaign_terminal.phase, BreakawayPhase::Probe as u8);
    assert_eq!(campaign_terminal.accepted, 0);
}

// --- Probe failure -> campaign terminal, never Stage C ------------------------

#[test]
fn a_probe_that_never_moves_terminates_the_campaign_without_reaching_stage_c() {
    let (_campaign, terminal) =
        run_campaign_probe(campaign_request(), false, |_direction, _index| false);

    let Some(outcome) = terminal else {
        panic!("a never-moving probe must terminate the campaign");
    };
    assert!(
        matches!(outcome, BreakawayCampaignOutcome::Terminal { .. }),
        "no Stage-C plan may be constructed on a probe-failure path: {outcome:?}"
    );
}

// --- Accept -> Stage C via the token route ------------------------------------

/// Produce a genuine `(acceptance, plan)` by driving a full held-out
/// confirmation block that accepts at the given candidate gain.
fn accepted_confirmation(candidate_raw: u16) -> (BandConfirmationAcceptance, BandConfirmationPlan) {
    let (outcome, _backend, _writes, _dirs) = run_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(candidate_raw)),
        2_448,
        |_direction, _stroke_index| 5, // 75%: comfortably inside the 70-80% band.
    );
    match outcome {
        BandConfirmationOutcome::Accepted { acceptance, plan } => (acceptance, plan),
        other => panic!("expected an accepted confirmation, got {other:?}"),
    }
}

#[test]
fn an_accepted_confirmation_routes_through_the_token_chain_to_a_stage_c_plan() {
    let (_p_start, _p_top, usable, _ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(100), 2_448).unwrap();
    let (acceptance, plan) = accepted_confirmation(512);
    let inputs = stage_c_inputs(usable);
    let safety = test_safety();
    let backend = test_backend();

    let outcome = route_confirmed_accept(acceptance, plan, &inputs, &safety, &backend);

    let BreakawayCampaignOutcome::StageCPlanReady(stage_c_plan) = outcome else {
        panic!("an accepted, transition-capable P must yield a Stage-C plan: {outcome:?}");
    };
    // The confirmed P is admitted unchanged, and the authority's digest is the
    // accepted confirmation plan's digest (the token chain, not a fabricated
    // value).
    assert_eq!(stage_c_plan.final_p, 512);
    assert_eq!(stage_c_plan.authority.stage_b_plan_digest, plan.plan_digest);
    assert_eq!(
        stage_c_plan.schema_revision,
        crate::commissioning::outer::velocity::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION
    );
}

// --- Accept whose P fails 6a safety refuses Stage C WITHOUT motion -------------

#[test]
fn an_accept_whose_confirmed_p_fails_6a_safety_refuses_stage_c_without_motion() {
    let (acceptance, plan) = accepted_confirmation(1_024);
    // A torque authority of 1 cannot carry any nonzero I prefix at the confirmed
    // P: `plan_breakaway_velocity_integral_response` refuses at plan time.
    let inputs = stage_c_inputs(1);
    let safety = test_safety();
    let mut backend = test_backend();
    let writes_before = backend.spi_log.len();

    let outcome = route_confirmed_accept(acceptance, plan, &inputs, &safety, &backend);

    assert!(
        matches!(outcome, BreakawayCampaignOutcome::Terminal { .. }),
        "a non-transition-capable confirmed P must refuse Stage C: {outcome:?}"
    );
    // The token route touches no backend excitation: refusal is a pure plan-time
    // decision, so no motion (or any register traffic) can have happened.
    let _ = &mut backend;
    assert_eq!(backend.spi_log.len(), writes_before);
    assert!(backend.spi_log.is_empty());
}

// --- Non-accept confirmation terminals never carry a Stage-C plan -------------

#[test]
fn no_non_accept_confirmation_terminal_carries_a_stage_c_plan() {
    for outcome in [
        BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery,
        BandConfirmationOutcome::TargetBandNotReachedAtAuthority,
        BandConfirmationOutcome::TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::ResponseLocation,
        ),
        BandConfirmationOutcome::ConfirmationEvidenceExcluded,
        BandConfirmationOutcome::SafetyFault {
            error: CommissionError::SafetyEnvelopeViolation,
            safety: None,
        },
    ] {
        assert!(
            matches!(
                confirmation_terminal(outcome),
                BreakawayCampaignOutcome::Terminal { .. }
            ),
            "a non-accept confirmation outcome must not build a Stage-C plan"
        );
    }
}

// --- FIX D(b): mid-block exclusion reports the partial evidence collected -----

#[test]
fn confirmation_evidence_excluded_after_a_recorded_stroke_reports_partial_evidence_not_empty() {
    let plan = confirmation_plan(724);
    let mut session =
        BandConfirmationSession::new(plan, 5.0, TmcMechanicalRpm::from_register(100), 4_394);
    // Simulate one earlier forward stroke that succeeded before a later stroke
    // was excluded, folding a real eligible observation into slot 0 exactly as
    // `BandConfirmationState::Advance` does.
    let atom = DiscoveryObservationAtom {
        mean_rate_q: 750_000,
        variance_of_mean_q2: 58_752_225,
        target_rate_q: 1_000_000,
        direction: VelocityDirection::Forward,
    };
    session
        .forward
        .record(0, VelocityObservationClass::Eligible, None, atom);

    let inputs = stage_c_inputs(2_448);
    let safety = test_safety();
    let backend = test_backend();
    let mut evidence_sequence = 0_u16;
    let advance = confirmation_done_advance(
        BandConfirmationOutcome::ConfirmationEvidenceExcluded,
        &session,
        &inputs,
        &safety,
        &backend,
        &mut evidence_sequence,
    );
    let CampaignAdvance::Emit { now, next } = advance else {
        panic!("expected an Emit advance");
    };
    let BreakawayCampaignEvent::ConfirmationTerminal {
        terminal,
        forward,
        reverse,
    } = now
    else {
        panic!("expected the confirmation terminal, got {now:?}");
    };

    assert_eq!(
        terminal.terminal_cause,
        BreakawayTerminalCause::ConfirmationEvidenceExcluded as u8
    );
    assert_eq!(terminal.accepted, 0);
    // The forward direction's one recorded stroke survives in the reported
    // evidence -- unlike the pre-fix behavior, which routed every exclusion
    // through the all-zero zero-stroke fixture and falsely claimed no
    // confirmation work happened.
    assert_eq!(forward.observations[0], Some(atom));
    assert_eq!(
        terminal.forward_eligible_mask, 0b0001,
        "only the one recorded slot is eligible"
    );
    // The reverse direction genuinely never ran a stroke: its eligible/included
    // masks are honestly zero, distinct from the forward direction's real
    // partial evidence. (`collected_mask` is unconditionally full per
    // `DirectionAccumulator::consensus`'s own always-called-after-every-slot
    // contract -- unaffected by this fix, which is about not discarding the
    // atoms/classes/eligible-mask evidence that *was* actually collected.)
    assert_eq!(
        reverse.observations,
        [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION]
    );
    assert_eq!(terminal.reverse_eligible_mask, 0);
    assert_eq!(terminal.reverse_included_mask, 0);

    let EmitContinuation::Terminal {
        done: BreakawayCampaignOutcome::Terminal { error, safety },
        ..
    } = next
    else {
        panic!("a non-accept confirmation outcome must be a Terminal step");
    };
    assert_eq!(error, CommissionError::VelocityValidationFailed);
    assert!(safety.is_none());
}

// --- 6d routing: an impossible discovery outcome maps to InvalidSchedule -------

#[test]
fn an_impossible_discovery_outcome_reaching_confirmation_maps_to_invalid_schedule() {
    let inputs = stage_c_inputs(2_448);

    // `TargetBandSkipped` is a discovery-owned terminal that must never reach
    // confirmation routing; if it does, it maps to the InvalidSchedule safety
    // fault rather than silently proceeding.
    let mut evidence_sequence = 0_u16;
    let advance = route_discovery_outcome(
        AdditiveDiscoveryOutcome::TargetBandSkipped,
        42,
        &inputs,
        &mut evidence_sequence,
    );
    let EmitContinuation::Terminal {
        done: BreakawayCampaignOutcome::Terminal { error, safety },
        ..
    } = advance
    else {
        panic!("an impossible discovery outcome must be a Terminal step");
    };
    assert_eq!(error, CommissionError::InvalidSchedule);
    assert!(safety.is_none());
}

// --- Accept hands off through the engine adapter into VelocityIntegral ---------

#[test]
fn an_accepted_campaign_hands_off_to_the_velocity_integral_phase() {
    let candidate: u16 = 512;
    let (_p_start, _p_top, usable, _ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(100), 2_448).unwrap();

    // Independently compute the exact Stage-C plan the token route must produce
    // for this accepted candidate, using the same frozen inputs the hand-off
    // will use, so the assertion checks the hand-off carried that exact plan.
    let (expected_acceptance, expected_plan) = accepted_confirmation(candidate);
    let BreakawayCampaignOutcome::StageCPlanReady(expected_stage_c) = route_confirmed_accept(
        expected_acceptance,
        expected_plan,
        &stage_c_inputs(usable),
        &test_safety(),
        &test_backend(),
    ) else {
        panic!("the accepted, transition-capable candidate must produce a Stage-C plan");
    };
    let expected_execution_plan = expected_stage_c.execution_plan();

    // Build a campaign sitting on a fresh confirmation block for the same
    // candidate, wrapped in the engine's OuterPhase, and drive it THROUGH the
    // engine adapter (not `route_confirmed_accept` directly) until the accept
    // hands off.
    let BandConfirmationEntry::Ready(session) = begin_band_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(candidate)),
        42,
        TmcMechanicalRpm::from_register(100),
        4_394,
    ) else {
        panic!("the crafted discovery rungs must nominate a confirmable candidate");
    };
    let mut phase = OuterPhase::BreakawayCampaign {
        campaign: BreakawayCampaign {
            inputs: stage_c_inputs(usable),
            phase: BreakawayCampaignPhase::Confirmation {
                state: BandConfirmationState::PlannedConfirmation,
                session,
            },
            evidence_sequence: 0,
            pending_event: None,
            pending_done: None,
        },
        workspace: BreakawayProbeWorkspace::new(),
        safety: test_safety(),
    };

    let mut backend = test_backend();
    backend.current_limit_value = current_limit(2_448);
    let mut encoder_count: i32 = 0;
    let mut stroke_numerator = 0_i32;
    let mut stroke_sign = 1_i32;
    let mut handed_off = false;

    for _ in 0..2_000_000 {
        if let OuterPhase::BreakawayCampaign { campaign, .. } = &phase {
            if let BreakawayCampaignPhase::Confirmation { state, session } = campaign.phase() {
                if matches!(state, BandConfirmationState::CommandTarget) {
                    stroke_numerator = 5; // 75%: inside the confirmable band both ways.
                    stroke_sign = direction_sign(session.current_direction());
                }
            }
        }
        let is_capture_phase = matches!(
            &phase,
            OuterPhase::BreakawayCampaign { campaign, .. }
                if matches!(
                    campaign.phase(),
                    BreakawayCampaignPhase::Confirmation {
                        state: BandConfirmationState::Capture { .. },
                        ..
                    }
                )
        );
        let result =
            pollster::block_on(step_breakaway_campaign_with_trace(&mut phase, &mut backend));
        match result {
            StepResult::ReadyNow | StepResult::BreakawayCampaignEvent(_) => {
                if matches!(phase, OuterPhase::VelocityIntegral { .. }) {
                    handed_off = true;
                    break;
                }
            }
            StepResult::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if is_capture_phase {
                    encoder_count += stroke_sign * stroke_numerator;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            StepResult::OuterFailed(error) => {
                panic!("the accept path must not fail the outer phase: {error:?}");
            }
            other => panic!("unexpected engine step result on the accept path: {other:?}"),
        }
    }

    assert!(
        handed_off,
        "an accepted confirmation must hand off to the VelocityIntegral phase"
    );
    let OuterPhase::VelocityIntegral { data, .. } = &phase else {
        panic!("the phase must have become VelocityIntegral after the accept");
    };
    // The hand-off mirrors begin_stage_c_continuation_with_plan end to end: the
    // Stage-C data carries the campaign's run_sequence and the exact execution
    // plan the token route produced, not merely a freshly constructed one.
    assert_eq!(data.integrity.run_sequence(), 7);
    assert_eq!(data.plan, expected_execution_plan);
}

// --- Live emission: acceptance reports the confirmation terminal, then the ----
// --- campaign's Stage-C digest record, before the Stage-C hand-off ------------

#[test]
fn an_accepted_campaign_emits_its_confirmation_terminal_then_the_stage_c_digest_record() {
    let candidate: u16 = 512;
    let (_p_start, _p_top, usable, _ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(100), 2_448).unwrap();
    let BandConfirmationEntry::Ready(session) = begin_band_confirmation(
        AdditiveDiscoveryOutcome::TargetBandSelected(confirmable_discovery_rungs(candidate)),
        42,
        TmcMechanicalRpm::from_register(100),
        4_394,
    ) else {
        panic!("the crafted discovery rungs must nominate a confirmable candidate");
    };
    let mut phase = OuterPhase::BreakawayCampaign {
        campaign: BreakawayCampaign {
            inputs: stage_c_inputs(usable),
            phase: BreakawayCampaignPhase::Confirmation {
                state: BandConfirmationState::PlannedConfirmation,
                session,
            },
            evidence_sequence: 0,
            pending_event: None,
            pending_done: None,
        },
        workspace: BreakawayProbeWorkspace::new(),
        safety: test_safety(),
    };

    let mut backend = test_backend();
    backend.current_limit_value = current_limit(2_448);
    let mut encoder_count: i32 = 0;
    let mut stroke_numerator = 0_i32;
    let mut stroke_sign = 1_i32;
    let mut events: heapless::Vec<BreakawayCampaignEvent, 16> = heapless::Vec::new();
    let mut handed_off = false;

    for _ in 0..2_000_000 {
        if let OuterPhase::BreakawayCampaign { campaign, .. } = &phase {
            if let BreakawayCampaignPhase::Confirmation { state, session } = campaign.phase() {
                if matches!(state, BandConfirmationState::CommandTarget) {
                    stroke_numerator = 5; // 75%: inside the confirmable band both ways.
                    stroke_sign = direction_sign(session.current_direction());
                }
            }
        }
        let is_capture_phase = matches!(
            &phase,
            OuterPhase::BreakawayCampaign { campaign, .. }
                if matches!(
                    campaign.phase(),
                    BreakawayCampaignPhase::Confirmation {
                        state: BandConfirmationState::Capture { .. },
                        ..
                    }
                )
        );
        let result =
            pollster::block_on(step_breakaway_campaign_with_trace(&mut phase, &mut backend));
        match result {
            StepResult::ReadyNow => {
                if matches!(phase, OuterPhase::VelocityIntegral { .. }) {
                    handed_off = true;
                    break;
                }
            }
            StepResult::BreakawayCampaignEvent(event) => {
                events
                    .push(event)
                    .expect("test campaign emits fewer than 16 events");
            }
            StepResult::At(deadline) => {
                backend.elapsed_us_value = deadline;
                if is_capture_phase {
                    encoder_count += stroke_sign * stroke_numerator;
                }
                backend.set_abn_decoder_count(encoder_count);
            }
            StepResult::OuterFailed(error) => {
                panic!("the accept path must not fail the outer phase: {error:?}");
            }
            other => panic!("unexpected engine step result on the accept path: {other:?}"),
        }
    }

    assert!(handed_off, "an accepted confirmation must hand off");
    let [.., second_to_last, last] = events.as_slice() else {
        panic!("expected at least a confirmation terminal and a campaign terminal");
    };
    let BreakawayCampaignEvent::ConfirmationTerminal { terminal, .. } = second_to_last else {
        panic!("expected the confirmation terminal second-to-last, got {second_to_last:?}");
    };
    assert_eq!(terminal.accepted, 1);
    assert_eq!(terminal.confirmed_p_raw, candidate);
    let BreakawayCampaignEvent::CampaignTerminal(campaign_terminal) = last else {
        panic!("expected the campaign's Stage-C digest record last, got {last:?}");
    };
    assert_eq!(campaign_terminal.accepted, 1);
    assert_ne!(
        campaign_terminal.stage_c_plan_digest, 0,
        "an accepted campaign's closure record must carry the Stage-C plan digest"
    );
    assert_eq!(campaign_terminal.phase, BreakawayPhase::Confirmation as u8);
}

// ================================================================================
// FIX B: recovery slot reservation must cover every recovered observation
// ================================================================================

/// The drivers recover after every observation, not once per rung: two probe
/// directions at up to [`PROBE_MAX_OBSERVATIONS_PER_DIRECTION`] observations
/// each, every discovery rung driven in both directions at
/// [`OBSERVATIONS_PER_DIRECTION_PER_RUNG`], and the confirmation family. Pin
/// the concrete value (280) so a silent regression to an under-reserving
/// constant is caught, and re-derive it from the same named constants the
/// implementation uses so the assertion tracks any future change to those
/// constants rather than drifting into a second hardcoded magic number.
#[test]
fn recovery_slot_count_covers_every_recovered_observation() {
    let derived = 2 * u32::from(PROBE_MAX_OBSERVATIONS_PER_DIRECTION)
        + MAX_SWEEP_RUNGS as u32 * 2 * u32::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG)
        + u32::from(CONFIRMATION_FAMILY_SIZE);

    assert_eq!(BreakawayCampaign::RECOVERY_SLOT_COUNT, 280);
    assert_eq!(BreakawayCampaign::RECOVERY_SLOT_COUNT, derived);
}

// ================================================================================
// FIX D(a): a global outer-safety fault mid-campaign still closes the campaign
// ================================================================================

fn zero_safety_fault(reason: u8) -> OuterSafetyFault {
    OuterSafetyFault {
        reason,
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

/// `dispatch::step_outer_with_trace`'s own global outer-safety check -- an
/// invalid budget, a tripped envelope, or a check I/O error -- runs OUTSIDE
/// any of the three drivers' internal state machines. Before this fix, that
/// check routed a `BreakawayCampaign` phase straight to `OuterFailed` with no
/// campaign-closure record ever emitted, even though offline replay (foci-trace
/// `velocity_measurement.py`) requires a closure record for every campaign.
///
/// `breakaway_campaign_fault_terminal` is the function `dispatch.rs` now calls
/// for that case; this drives it directly (mirroring `sweep_fault_terminal`'s
/// own untested-in-isolation shape, since `dispatch.rs` has no fixture for a
/// tripped outer-safety envelope) and confirms the deferred closure record
/// actually drains on the next tick, carrying the owning phase and safety
/// cause, before the following tick resolves to the real `Terminal` outcome.
#[test]
fn breakaway_campaign_fault_terminal_defers_a_campaign_closure_before_outer_failed() {
    let campaign = BreakawayCampaign::new(campaign_request()).expect("valid campaign request");
    assert!(matches!(
        campaign.phase(),
        BreakawayCampaignPhase::Probe { .. }
    ));
    let mut phase = OuterPhase::BreakawayCampaign {
        campaign,
        workspace: BreakawayProbeWorkspace::new(),
        safety: test_safety(),
    };
    let fault = zero_safety_fault(crate::commissioning::types::OUTER_SAFETY_FAULT_QUARTER_TURN);

    let step = breakaway_campaign_fault_terminal(
        &mut phase,
        CommissionError::SafetyEnvelopeViolation,
        Some(fault),
    );
    assert!(
        matches!(step, StepResult::ReadyNow),
        "the closure record is deferred, not returned on this tick: {step:?}"
    );

    let OuterPhase::BreakawayCampaign {
        mut campaign,
        mut workspace,
        mut safety,
    } = phase
    else {
        panic!("phase must still be a breakaway campaign");
    };

    // Next tick: the deferred campaign-closure record drains first, before any
    // driver runs. This is the record offline replay requires.
    let closure_step = pollster::block_on(step_breakaway_campaign(
        &mut campaign,
        &mut workspace,
        &mut safety,
        &mut test_backend(),
    ));
    let BreakawayCampaignStep::Event(BreakawayCampaignEvent::CampaignTerminal(closure)) =
        closure_step
    else {
        panic!("expected the deferred campaign-closure record, got {closure_step:?}");
    };
    assert_eq!(
        closure.phase,
        BreakawayPhase::Probe as u8,
        "the closure record must name the phase that was active at the fault"
    );
    assert_eq!(
        closure.terminal_cause,
        BreakawayTerminalCause::ProbeSafetyFault as u8
    );
    assert_eq!(closure.accepted, 0);

    // The tick after that resolves the deferred `Done`, carrying the original
    // error and safety fault through to the real `OuterFailed`/
    // `FailedWithSafety` transition at the dispatch boundary.
    let done_step = pollster::block_on(step_breakaway_campaign(
        &mut campaign,
        &mut workspace,
        &mut safety,
        &mut test_backend(),
    ));
    let BreakawayCampaignStep::Done(BreakawayCampaignOutcome::Terminal {
        error,
        safety: safety_fault,
    }) = done_step
    else {
        panic!("expected the deferred Terminal outcome, got {done_step:?}");
    };
    assert_eq!(error, CommissionError::SafetyEnvelopeViolation);
    assert_eq!(safety_fault, Some(fault));
}

/// Same fault path, but while a discovery phase owns the campaign: the closure
/// record's phase and terminal cause must track whichever phase was actually
/// active, not always report `Probe`.
#[test]
fn breakaway_campaign_fault_terminal_names_the_owning_discovery_phase() {
    let session = AdditiveDiscoverySession::new(
        discovery_request(),
        false,
        TmcMechanicalRpm::from_register(100),
        4_394,
    )
    .expect("valid discovery session");
    let mut campaign = BreakawayCampaign::new(campaign_request()).expect("valid campaign request");
    // Directly overwrite the campaign's active phase (private-field access from
    // the `tests` submodule): the fix under test reads `campaign.phase()` to
    // decide the owning phase, and the probe/discovery/confirmation drivers'
    // own physical stroke simulation is not what this test is about.
    campaign.phase = BreakawayCampaignPhase::Discovery {
        state: AdditiveDiscoveryState::PlannedDiscovery,
        session,
    };
    let mut phase = OuterPhase::BreakawayCampaign {
        campaign,
        workspace: BreakawayProbeWorkspace::new(),
        safety: test_safety(),
    };

    let step = breakaway_campaign_fault_terminal(&mut phase, CommissionError::Spi, None);
    assert!(matches!(step, StepResult::ReadyNow));

    let OuterPhase::BreakawayCampaign {
        mut campaign,
        mut workspace,
        mut safety,
    } = phase
    else {
        panic!("phase must still be a breakaway campaign");
    };
    let closure_step = pollster::block_on(step_breakaway_campaign(
        &mut campaign,
        &mut workspace,
        &mut safety,
        &mut test_backend(),
    ));
    let BreakawayCampaignStep::Event(BreakawayCampaignEvent::CampaignTerminal(closure)) =
        closure_step
    else {
        panic!("expected the deferred campaign-closure record, got {closure_step:?}");
    };
    assert_eq!(closure.phase, BreakawayPhase::Discovery as u8);
    assert_eq!(
        closure.terminal_cause,
        BreakawayTerminalCause::DiscoverySafetyFault as u8
    );
}
