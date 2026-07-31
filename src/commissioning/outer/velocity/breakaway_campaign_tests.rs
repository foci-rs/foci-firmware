use super::*;
use crate::autotune::TmcMechanicalRpm;
use crate::commission_dispatch::test_support::{MockBackend, current_limit};
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::breakaway_plan::plan_breakaway_probe;
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
    plan_breakaway_probe(gain(1), gain(1_024)).expect("valid probe plan")
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
