//! Engine-level velocity-workflow phase emission.
//!
//! Every outer workflow names itself on the wire: the engine reports the
//! active [`CommissionPhase`] the first time it becomes current, and again at
//! each in-engine boundary, so the host can follow an autotune run without
//! inferring the workflow from evidence traffic.

use super::{CommissioningEngine, EngineMode, OuterPhase};
use crate::autotune::CommissionPhase;
use crate::commission_dispatch::test_support::{
    MockBackend, breakaway_campaign_engine, breakaway_confirmation_for_test,
    robustness_reversal_engine, sample_entry_snapshot, sample_outer_params,
};
use crate::commissioning::OuterAutotuneInputs;
use crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan;
use crate::commissioning::types::StepResult;
use crate::trace::events::{EVENT_COMMISSION_PHASE_END, EVENT_COMMISSION_PHASE_START};
use foci_firmware_macros::fixture_test;

/// Upper bound on cooperative ticks a fixture run may take before the test
/// gives up. Every fixture below terminates far inside this.
const MAX_DRIVE_TICKS: usize = 2_000_000;

const COUNTS_PER_REV: u32 = 4_000;
const PWM_HZ: u32 = 25_000;
const TORQUE_LIMIT: u16 = 2_816;

fn phase_of(result: &StepResult) -> Option<CommissionPhase> {
    match result {
        StepResult::PhaseChanged(phase) => Some(*phase),
        _ => None,
    }
}

fn is_terminal(result: &StepResult) -> bool {
    matches!(
        result,
        StepResult::VelocityIntegralTerminal(_)
            | StepResult::FixedGainAmplitudeTerminal(_)
            | StepResult::RobustnessReversalTerminal(_)
            | StepResult::OuterDone(_)
            | StepResult::OuterFailed(_)
            | StepResult::InnerFailed(_)
    )
}

async fn drive_to_completion(
    engine: &mut CommissioningEngine,
    backend: &mut MockBackend,
) -> heapless::Vec<CommissionPhase, 8> {
    let mut phases = heapless::Vec::new();
    for _ in 0..MAX_DRIVE_TICKS {
        let result = engine.step(backend).await;
        if let Some(phase) = phase_of(&result) {
            phases
                .push(phase)
                .expect("a fixture run reports fewer than eight phase transitions");
        }
        if let StepResult::At(deadline) = result {
            backend.elapsed_us_value = deadline;
        }
        if is_terminal(&result) {
            return phases;
        }
    }
    panic!("the fixture engine did not terminate within {MAX_DRIVE_TICKS} ticks");
}

fn outer_backend() -> MockBackend {
    let mut backend = MockBackend::new();
    backend.mechanical_encoder_quadrature_counts_per_rev_value = COUNTS_PER_REV;
    backend
}

/// Outer parameters whose motion budget reproduces the velocity the plan
/// fixtures below freeze, so the engine's safety envelope agrees with them.
fn plan_matched_outer_params() -> OuterAutotuneInputs {
    OuterAutotuneInputs {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
        ..sample_outer_params()
    }
}

fn plan_request() -> crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest {
    let params = plan_matched_outer_params();
    crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest {
        requested_velocity_mrev_s: params.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: params.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: params.requested_velocity_source,
        max_stroke_travel_mrev: params.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: params.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: params.negative_position_headroom_mrev,
        positive_position_headroom_mrev: params.positive_position_headroom_mrev,
        max_duration_ms: params.max_duration_ms,
    }
}

/// One genuine breakaway Stage-C plan, minted through the real
/// confirmation-token path a live campaign uses.
fn response_ladder_plan() -> VelocityIntegralPlan {
    use crate::commissioning::outer::OuterMotionBudget;
    use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
    use crate::commissioning::outer::velocity::integral_plan::{
        AcquisitionContext, plan_retained_breakaway_velocity_integral_response,
    };
    use crate::commissioning::outer::velocity::sweep_plan::plan_stage_b_velocity_sweep;

    let request = plan_request();
    let stage_b = plan_stage_b_velocity_sweep(
        request,
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .expect("a valid Stage-B sweep");
    let mut context = AcquisitionContext::from(stage_b.execution_plan(stage_b.digest()));
    context.channel = 0;
    context.plan_time_pwm_hz = PWM_HZ;
    context.plan_time_encoder_counts_per_rev = COUNTS_PER_REV;

    let confirmation = breakaway_confirmation_for_test();
    plan_retained_breakaway_velocity_integral_response(
        confirmation.token,
        confirmation.confirmation_plan.candidate_p.raw(),
        context,
        confirmation.held_out_response_intervals_q,
        confirmation.confidence_q,
        confirmation.band_position_q,
        OuterMotionBudget {
            requested_velocity_mrev_s: request.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: request.requested_velocity_source,
            max_stroke_travel_mrev: request.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: request.negative_position_headroom_mrev,
            positive_position_headroom_mrev: request.positive_position_headroom_mrev,
            max_duration_ms: request.max_duration_ms,
        },
        PWM_HZ,
        COUNTS_PER_REV,
    )
    .expect("a valid retained breakaway Stage-C plan")
}

fn response_ladder_engine_with_params(params: OuterAutotuneInputs) -> CommissioningEngine {
    use crate::commission_dispatch::stage_c::StageCPreflight;
    use crate::commissioning::outer::velocity::integral_reproduction::StageCPlanIdentity;

    let plan = response_ladder_plan();
    CommissioningEngine::new_outer_for_tmc_with_stage_c_preflight(
        0,
        crate::state::tmc::tmc_channel0(),
        params,
        sample_entry_snapshot(),
        StageCPreflight {
            plan,
            identity: StageCPlanIdentity {
                channel: 0,
                build_features: 0,
                schema_revision: plan.schema_revision,
                plan_digest: plan.digest(),
            },
            run_sequence: 9,
        },
    )
}

fn response_ladder_engine() -> CommissioningEngine {
    response_ladder_engine_with_params(plan_matched_outer_params())
}

fn fixed_gain_amplitude_engine() -> CommissioningEngine {
    use crate::commission_dispatch::fixed_gain_amplitude::FixedGainAmplitudePreflight;
    use crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::{
        FixedGainAmplitudeOrder, plan_fixed_gain_amplitude,
    };
    use crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptanceMatrixProgress, AcceptancePoint, StageCPlanIdentity,
    };
    use crate::commissioning::outer::velocity::reproduction::BUILD_FEATURES;

    let current = response_ladder_plan();
    let point = AcceptancePoint {
        identity: StageCPlanIdentity {
            channel: 0,
            build_features: BUILD_FEATURES,
            schema_revision: current.schema_revision,
            plan_digest: current.digest(),
        },
        selected_p: current.final_p,
        selected_i: current.positive_rungs()[0],
        target_velocity_rpm: current.target_velocity_rpm,
        selected_response_intervals_q: current.authority.directional_pooled_intervals,
        confidence_q: 0,
        band_position_q: 0,
        qualifier_bits: 0,
        token: None,
        progress: AcceptanceMatrixProgress::ReadyAscending,
    };
    let plan = plan_fixed_gain_amplitude(
        point,
        plan_request(),
        FixedGainAmplitudeOrder::Ascending,
        IntegralSlotOrder::ForwardFirst,
        current,
    )
    .expect("a genuine breakaway point carries amplitude acceptance authority");

    CommissioningEngine::new_outer_for_tmc_with_fixed_gain_amplitude_preflight(
        0,
        crate::state::tmc::tmc_channel0(),
        plan_matched_outer_params(),
        sample_entry_snapshot(),
        FixedGainAmplitudePreflight {
            plan,
            run_sequence: 11,
        },
    )
}

fn trace_phase_ids(backend: &MockBackend) -> heapless::Vec<(u16, i32), 32> {
    backend
        .trace_events
        .iter()
        .filter(|event| {
            event.event_id == EVENT_COMMISSION_PHASE_START
                || event.event_id == EVENT_COMMISSION_PHASE_END
        })
        .map(|event| (event.event_id, event.payload[0]))
        .collect()
}

fn phase_code(phase: CommissionPhase) -> i32 {
    i32::from(phase.to_u8())
}

/// Replace the outer phase in place exactly as an accepted breakaway
/// confirmation does (`route_confirmed_accept`), so the engine sees the same
/// underneath-the-stepper hand-off a live campaign produces.
///
/// A stand-in, not the seam: `BreakawayCampaign`'s fields are private to its own
/// module, so a mid-confirmation campaign cannot be built here. The hand-off
/// driven through the live campaign, and the event ordering it produces, is
/// covered by `breakaway_campaign_tests`'
/// `the_engine_reports_one_stage_c_plan_before_any_integral_evidence`; what this
/// buys is the trace-span pairing below, which that test does not inspect.
fn hand_off_to_response_ladder(engine: &mut CommissioningEngine) {
    use crate::commissioning::outer::{VelocityIntegralData, VelocityIntegralState};

    let EngineMode::Outer { phase, .. } = &mut engine.engine_mode else {
        panic!("the fixture engine must be an outer engine");
    };
    let OuterPhase::BreakawayCampaign { safety, .. } = phase else {
        panic!("the hand-off starts from the breakaway campaign phase");
    };
    let safety = *safety;
    *phase = OuterPhase::VelocityIntegral {
        state: VelocityIntegralState::Setup,
        data: VelocityIntegralData::new(7, response_ladder_plan().execution_plan()),
        safety,
    };
}

#[fixture_test]
async fn breakaway_run_emits_entry_then_its_integral_boundary() {
    let mut engine = breakaway_campaign_engine();
    let mut backend = outer_backend();

    let entry = engine.step(&mut backend).await;
    assert_eq!(
        phase_of(&entry),
        Some(CommissionPhase::BreakawayProportionalAcquisition),
        "the first outer tick must name the campaign's workflow"
    );
    let spi_after_entry = backend.spi_log.len();

    // The entry event rides the span `emit_trace_start` already opened, so it
    // must not close and reopen it.
    assert_eq!(
        trace_phase_ids(&backend),
        &[(
            EVENT_COMMISSION_PHASE_START,
            phase_code(CommissionPhase::BreakawayProportionalAcquisition)
        )][..]
    );

    let second = engine.step(&mut backend).await;
    assert_eq!(
        phase_of(&second),
        None,
        "the entry phase must be reported once, not on every tick"
    );
    assert!(
        backend.spi_log.len() >= spi_after_entry,
        "the campaign stepper must run once the entry phase is reported"
    );

    hand_off_to_response_ladder(&mut engine);
    backend.trace_events.clear();

    let boundary = engine.step(&mut backend).await;
    assert_eq!(
        phase_of(&boundary),
        Some(CommissionPhase::IntegralGainResponseCharacterization),
        "the in-place hand-off must report the integral workflow"
    );
    assert_eq!(
        trace_phase_ids(&backend),
        &[
            (
                EVENT_COMMISSION_PHASE_END,
                phase_code(CommissionPhase::BreakawayProportionalAcquisition)
            ),
            (
                EVENT_COMMISSION_PHASE_START,
                phase_code(CommissionPhase::IntegralGainResponseCharacterization)
            ),
        ][..],
        "a boundary closes the old span and opens the new one exactly once"
    );

    let after_boundary = engine.step(&mut backend).await;
    assert_eq!(
        phase_of(&after_boundary),
        None,
        "the boundary phase must be reported once"
    );
}

/// The synthetic phase tick reports the transition instead of advancing the
/// stepper, so no evidence the stepper would have produced is lost -- it
/// arrives on the following tick.
#[fixture_test]
async fn the_phase_tick_does_not_advance_the_stepper() {
    let mut engine = response_ladder_engine();
    let mut backend = outer_backend();

    let entry = engine.step(&mut backend).await;
    assert_eq!(
        phase_of(&entry),
        Some(CommissionPhase::IntegralGainResponseCharacterization)
    );
    assert!(
        backend.spi_log.is_empty(),
        "the phase tick must not run the Stage-C stepper: {:?}",
        backend.spi_log
    );

    let EngineMode::Outer {
        phase: OuterPhase::VelocityIntegral { state, .. },
        ..
    } = &engine.engine_mode
    else {
        panic!("the Stage-C phase must still be resident");
    };
    assert!(
        matches!(
            state,
            crate::commissioning::outer::VelocityIntegralState::Setup
        ),
        "the Stage-C stepper must still be parked at Setup"
    );
}

#[fixture_test]
async fn direct_response_ladder_emits_only_its_initial_phase() {
    let mut engine = response_ladder_engine();
    let mut backend = outer_backend();

    assert_eq!(
        drive_to_completion(&mut engine, &mut backend).await,
        &[CommissionPhase::IntegralGainResponseCharacterization][..]
    );
}

#[fixture_test]
async fn fixed_gain_amplitude_reports_fixed_gain_amplitude_validation() {
    let mut engine = fixed_gain_amplitude_engine();
    let mut backend = outer_backend();

    assert_eq!(
        drive_to_completion(&mut engine, &mut backend).await,
        &[CommissionPhase::FixedGainAmplitudeValidation][..]
    );
}

#[fixture_test]
async fn robustness_reversal_reports_its_own_phase() {
    let mut engine = robustness_reversal_engine();
    let mut backend = outer_backend();

    assert_eq!(
        drive_to_completion(&mut engine, &mut backend).await,
        &[CommissionPhase::ReversalStandstillRobustnessGate][..]
    );
}

/// A workflow whose very first stepper tick faults still names itself first:
/// the phase event is selected before the stepper is dispatched at all.
#[fixture_test]
async fn first_step_failure_emits_initial_phase_before_terminal() {
    let mut engine = response_ladder_engine_with_params(OuterAutotuneInputs {
        max_duration_ms: 0,
        ..plan_matched_outer_params()
    });
    let mut backend = outer_backend();

    let entry = engine.step(&mut backend).await;
    assert_eq!(
        phase_of(&entry),
        Some(CommissionPhase::IntegralGainResponseCharacterization)
    );

    let terminal = engine.step(&mut backend).await;
    assert!(
        is_terminal(&terminal),
        "an invalid motion budget must fault the first stepper tick: {terminal:?}"
    );
}

/// Reporting the workflow must never come before the e-stop check: a step
/// taken under a shutdown request aborts, it does not name a phase.
#[fixture_test]
async fn a_shutdown_request_aborts_before_any_phase_is_reported() {
    let mut engine = response_ladder_engine();
    let mut backend = outer_backend();
    backend.shutdown_requested = true;

    let result = engine.step(&mut backend).await;
    assert!(
        matches!(
            result,
            StepResult::OuterFailed(crate::autotune::CommissionError::Shutdown)
        ),
        "a shutdown request must abort the step: {result:?}"
    );
}

/// Inner commissioning reports its own transitions from the stepper; the
/// outer-only synthetic selection must leave that path untouched.
#[fixture_test]
async fn inner_commissioning_reports_no_synthetic_entry_phase() {
    let mut engine =
        CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced);
    let mut backend = MockBackend::new();

    let first = engine.step(&mut backend).await;
    assert_eq!(
        phase_of(&first),
        None,
        "an inner engine must not synthesize an entry phase event"
    );
}
