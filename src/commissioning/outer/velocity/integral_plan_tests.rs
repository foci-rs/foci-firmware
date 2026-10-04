use super::band_confirmation::{
    BandConfirmationFailureCause, BandConfirmationPlan, ConfirmationDirectionalInput,
    DiscoveryObservationAtom, TargetBandConfirmationInconclusive, evaluate_band_confirmation,
    plan_band_confirmation, validate_fixed_contract,
};
use super::breakaway_plan::{
    AdditiveDiscoveryRequest, AdditiveLadderPlanError, BreakawayProbeContext, GridFloorOrigin,
    PROBE_MAX_OBSERVATIONS, UpwardBreakawayProbePlan, plan_additive_discovery_ladder,
    plan_upward_breakaway_probe, resolve_grid_floor,
};
use super::integral_plan::{
    AcquisitionContext, BREAKAWAY_INTEGRAL_SCHEMA_REVISION, BreakawayAuthorityBindingError,
    BreakawayIntegralEvidence, ConfirmedAcquisitionDigest, INTEGRAL_EVIDENCE_SCHEMA_REVISION,
    INTEGRAL_MAX_POSITIVE_RUNGS, INTEGRAL_MIN_POSITIVE_I, INTEGRAL_OBSERVATIONS_PER_RUNG,
    IntegralAcquisitionAuthority, IntegralPlanPhysics, IntegralSlotOrder, VelocityIntegralPlan,
    VelocityIntegralPlanError, bind_breakaway_integral_authority, breakaway_confirmation_slots_us,
    breakaway_discovery_slots_us, breakaway_probe_slots_us, breakaway_workflow_envelope_ms,
    breakaway_workflow_envelope_us, build_integral_ladder, confidence_family_size,
    integral_gain_at, integral_top_gap_candidate, max_transition_capable_p, native_rung_count,
    native_velocity_i_at, plan_breakaway_velocity_integral_response, recovery_quantization_exposed,
    transition_capable_integral_gain, velocity_i_tau_us,
};
use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
};
use super::sweep_evidence::VelocityObservationClass;
use super::sweep_plan::{
    MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MIN_STRUCTURAL_RUNGS, MOVING_STROKE_US,
    RequestedVelocitySource, SETTLE_ANALYSIS_BUDGET_US, VelocitySweepRequest, ZERO_SETTLE_US,
    plan_velocity_sweep,
};
use super::workflow_plan::{CommissioningWorkflowPlan, VelocityAutotuneWorkflowKind};
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::commissioning::outer::velocity::reproduction::ProportionalDirectionalValidity;
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};
use foci_tmc4671::pid::PidPGain;

fn proportional_request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    }
}

fn authority() -> IntegralAcquisitionAuthority {
    IntegralAcquisitionAuthority {
        channel: 0,
        acquisition: breakaway_token_for(breakaway_confirmation_plan(breakaway_gain(512))),
        directional_memberships: [0x0007_0000, 0x0007_0000],
        proportional_joint_membership: 0x0007_0000,
        joint_membership: 0x0007_0000,
        directional_validity: [ProportionalDirectionalValidity::ReproducedRegionCoversFinalP; 2],
        reduced_margin: false,
        directional_pooled_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
        confidence_q: 0,
        band_position_q: 0,
    }
}

fn proportional_plan() -> super::sweep_plan::VelocitySweepPlan {
    plan_velocity_sweep(
        proportional_request(),
        4_000,
        2_816,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap()
}

fn motion_budget() -> OuterMotionBudget {
    OuterMotionBudget {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
        homing_speed_mrev_s: 4_375,
        max_accel_mrev_s2: 200_000,
    }
}

fn breakaway_gain(raw: u16) -> PidPGain {
    PidPGain::new(raw).expect("raw gain within PidPGain range")
}

fn breakaway_probe_context() -> BreakawayProbeContext {
    BreakawayProbeContext {
        workflow_digest: 0xA1B2_C3D4_E5F6_0718,
        target_velocity_rpm: 600,
        hard_torque_limit: 2_816,
        usable_torque_limit: 2_048,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
    }
}

fn breakaway_probe_and_floor() -> (UpwardBreakawayProbePlan, PidPGain, PidPGain) {
    let p_start = breakaway_gain(1);
    let p_top = breakaway_gain(1_024);
    let probe_plan =
        plan_upward_breakaway_probe(p_start, p_top, breakaway_probe_context()).unwrap();
    let breakaway_index = probe_plan.search_gains[..usize::from(probe_plan.search_count)]
        .iter()
        .position(|gain| gain.raw() == 256)
        .expect("half-octave grid includes 256") as u8;
    let (floor, origin) =
        resolve_grid_floor(p_start, p_top, probe_plan.search_count, breakaway_index).unwrap();
    assert_eq!(origin, GridFloorOrigin::Predecessor);
    let breakaway = probe_plan.search_gains[usize::from(breakaway_index)];
    (probe_plan, floor, breakaway)
}

fn breakaway_discovery_request(
    floor: PidPGain,
    breakaway: PidPGain,
    probe_plan_digest: u64,
    workflow_envelope_ms: u32,
) -> AdditiveDiscoveryRequest {
    AdditiveDiscoveryRequest {
        floor,
        breakaway,
        authority_ceiling: breakaway_gain(1_024),
        observations_per_direction: 4,
        max_duration_ms: 3_000,
        workflow_envelope_ms,
        probe_plan_digest,
    }
}

fn full_directional_consensus() -> DirectionalConsensus {
    DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    }
}

fn accepted_confirmation_evidence() -> (ConfirmationDirectionalInput, ConfirmationDirectionalInput)
{
    let mean_rate_q: i64 = 750_000;
    let variance_of_mean_q2: u128 = 58_752_225;
    let target_rate_q: i64 = 1_000_000;
    let forward = ConfirmationDirectionalInput {
        consensus: full_directional_consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Forward,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let reverse = ConfirmationDirectionalInput {
        consensus: full_directional_consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q: -mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Reverse,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    (forward, reverse)
}

#[test]
fn native_q4_12_geometry_derives_the_fixed_prefix_and_octaves() {
    assert_eq!(INTEGRAL_EVIDENCE_SCHEMA_REVISION, 17);
    assert_eq!(INTEGRAL_MIN_POSITIVE_I, 1);
    assert_eq!(velocity_i_tau_us(1, 25_000), Some(41_943_040));
    assert_eq!(velocity_i_tau_us(1_024, 25_000), Some(40_960));

    let ladder = build_integral_ladder(
        25_000,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )
    .unwrap();

    assert_eq!(ladder.i_start, 1);
    assert_eq!(ladder.i_top_cap, 1_310);
    assert_eq!(
        ladder.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );

    let doubled_pwm = build_integral_ladder(50_000, 512_000, 64_000, 32_767).unwrap();
    assert_eq!(doubled_pwm.i_start, 1);
    assert_eq!(doubled_pwm.i_top_cap, 655);
    assert_eq!(
        doubled_pwm.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512]
    );

    let shorter_stroke = build_integral_ladder(25_000, 256_000, 64_000, 32_767).unwrap();
    assert_eq!(shorter_stroke.i_start, 1);
    assert_eq!(shorter_stroke.i_top_cap, 1_310);
    assert_eq!(
        shorter_stroke.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );
}

#[test]
fn recovery_quantization_exposure_is_exact_and_checked() {
    assert_eq!(
        recovery_quantization_exposed(724, 264, 4_000, 2_534),
        Ok(false)
    );
    assert_eq!(
        recovery_quantization_exposed(1_968, 264, 4_000, 2_534),
        Ok(false)
    );
    assert_eq!(
        recovery_quantization_exposed(1_969, 264, 4_000, 2_534),
        Ok(true)
    );
    assert_eq!(recovery_quantization_exposed(1, 0, 4, 256), Ok(false));
    assert_eq!(recovery_quantization_exposed(1, 0, 4, 255), Ok(true));
    assert_eq!(
        recovery_quantization_exposed(1, i32::MIN, 1, 1),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
}

#[test]
fn integral_accumulation_ratio_tracks_pwm_and_stroke_geometry() {
    fn stroke_cycles(pwm_hz: u32, stroke_us: u32) -> u64 {
        u64::from(pwm_hz) * u64::from(stroke_us) / 1_000_000
    }

    let current_cycles = stroke_cycles(25_000, 512_000);
    assert_eq!(current_cycles, 12_800);
    assert_eq!(
        (
            current_cycles,
            crate::autotune::ADVANCED_PI_I_Q4_12_NUMERATOR
        ),
        (12_800, 1_048_576)
    );
    assert_eq!(
        [1_u16, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
            .map(|gain| current_cycles * u64::from(gain)),
        [
            12_800, 25_600, 38_400, 51_200, 102_400, 204_800, 409_600, 819_200, 1_638_400,
            3_276_800, 6_553_600, 13_107_200,
        ]
    );
    assert_eq!(stroke_cycles(50_000, 512_000), 25_600);
    assert_eq!(stroke_cycles(25_000, 256_000), 6_400);
}

#[test]
fn top_cap_is_not_appended_as_a_non_octave_rung() {
    let ladder = build_integral_ladder(25_000, 512_000, 64_000, 8_192).unwrap();
    assert_eq!(ladder.i_top_cap, 1_310);
    assert_eq!(ladder.rungs.last(), Some(&1_024));
    assert!(!ladder.rungs.contains(&1_310));
}

#[test]
fn q4_12_deployment_bridge_is_evidence_not_native_geometry() {
    let deployment_bridge = [16_u16, 32, 64, 128, 256, 512, 1_024];
    let current =
        build_integral_ladder(25_000, MOVING_STROKE_US, MIN_SETTLE_CAPTURE_US, 32_767).unwrap();

    assert_eq!(
        current.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );
    assert_ne!(current.rungs.as_slice(), deployment_bridge);
}

#[test]
fn confidence_family_counts_both_anchors() {
    assert_eq!(INTEGRAL_MAX_POSITIVE_RUNGS, 30);
    assert_eq!(INTEGRAL_OBSERVATIONS_PER_RUNG, 8);
    assert_eq!(confidence_family_size(12), Ok(56));
    assert_eq!(confidence_family_size(30), Ok(128));
    assert_eq!(
        confidence_family_size(31),
        Err(VelocityIntegralPlanError::ConfidenceFamilyTooLarge)
    );
}

#[test]
fn integral_plan_freezes_geometry_and_conservative_duration() {
    let plan = build_test_breakaway_plan();

    assert_eq!(plan.schema_revision, BREAKAWAY_INTEGRAL_SCHEMA_REVISION);
    assert_eq!(plan.fixed_p, 512);
    assert_eq!(
        plan.positive_rungs(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 724, 1_024]
    );
    assert_eq!(plan.family_size, 60);
    assert_eq!(plan.total_rung_count, 15);
    assert_eq!(plan.expected_observations, 120);
    assert_eq!(plan.moving_stroke_us, 512_000);
    assert_eq!(plan.zero_settle_us, 500_000);
    assert_eq!(plan.analysis_budget_us, 236_000);
    assert_eq!(plan.recovery_origin_band_counts, 1_000);
    assert_eq!(plan.recovery_nominal_slot_us, 1_816_958);
    assert_eq!(plan.recovery_maximum_slot_us, 3_000_000);
    assert_eq!(plan.recovery_slot_count, 15);
    assert_eq!(plan.nominal_workflow_ms, 177_751);
    assert_eq!(plan.maximum_workflow_ms, 195_496);
    let base_us = u32::from(plan.total_rung_count)
        * INTEGRAL_OBSERVATIONS_PER_RUNG
        * (MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US);
    let recovery_us = u32::from(plan.total_rung_count) * plan.recovery_nominal_slot_us;
    assert_eq!(
        plan.nominal_workflow_ms,
        (base_us + recovery_us + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US).div_ceil(1_000)
    );
}

fn build_test_breakaway_plan() -> VelocityIntegralPlan {
    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(512));
    plan_breakaway_velocity_integral_response(
        breakaway_token_for(confirmation_plan),
        confirmation_plan,
        breakaway_acquisition_context(),
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
            confidence_q: 0,
            band_position_q: 0,
        },
        IntegralPlanPhysics {
            motion_budget: motion_budget(),
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
        },
    )
    .expect("the campaign's own Integral plan")
}

#[test]
fn breakaway_plan_includes_the_half_octave_rung_and_real_cap() {
    let plan = build_test_breakaway_plan();
    assert_eq!(plan.i_top_cap, 1_310);
    assert_eq!(
        plan.positive_rungs(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 724, 1_024]
    );
    assert_eq!(plan.positive_rung_count, 13);
}

#[test]
fn execution_plan_copies_i_top_cap() {
    let breakaway = build_test_breakaway_plan().execution_plan();
    assert_eq!(breakaway.i_top_cap, 1_310);
}

#[test]
fn workflow_shape_resume_composes_its_digest_from_duration_arithmetic() {
    let integral = build_test_breakaway_plan();

    let resume = CommissioningWorkflowPlan::new(
        9,
        VelocityAutotuneWorkflowKind::Resume,
        0,
        integral.maximum_workflow_ms,
    )
    .unwrap();

    assert_eq!(resume.maximum_workflow_ms, integral.maximum_workflow_ms);
    assert_eq!(
        CommissioningWorkflowPlan::new(
            9,
            VelocityAutotuneWorkflowKind::Resume,
            1,
            integral.maximum_workflow_ms,
        ),
        Err(VelocityIntegralPlanError::InvalidRequest),
        "Resume names no Proportional duration"
    );
    assert_eq!(
        CommissioningWorkflowPlan::new(9, VelocityAutotuneWorkflowKind::Resume, 0, 0),
        Err(VelocityIntegralPlanError::InvalidRequest),
        "Resume must name an Integral duration"
    );
    let different_sequence = CommissioningWorkflowPlan::new(
        10,
        VelocityAutotuneWorkflowKind::Resume,
        0,
        integral.maximum_workflow_ms,
    )
    .unwrap();
    assert_ne!(resume.digest(), different_sequence.digest());
}

#[test]
fn invalid_geometry_rejects_before_motion() {
    assert_eq!(velocity_i_tau_us(0, 25_000), None);
    assert_eq!(velocity_i_tau_us(5, 0), None);
    assert_eq!(
        build_integral_ladder(25_000, 512_000, 64_000, 2),
        Err(VelocityIntegralPlanError::InsufficientStructuralSpan)
    );
    assert_eq!(
        build_integral_ladder(25_000, 512_000, 64_000, 160)
            .unwrap()
            .rungs
            .as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128]
    );
    assert_eq!(
        build_integral_ladder(1, 1_000_000, 2, u16::MAX),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
}

#[test]
fn model_interval_availability_is_typed_by_directional_validity() {
    let availability = [
        (ProportionalDirectionalValidity::NoRegion, false),
        (ProportionalDirectionalValidity::Fragment, false),
        (ProportionalDirectionalValidity::RegionNotReproduced, false),
        (
            ProportionalDirectionalValidity::ReproducedRegionCoversFinalP,
            true,
        ),
        (
            ProportionalDirectionalValidity::ReproducedRegionTransferred,
            true,
        ),
    ];
    let mut value = authority();
    for (validity, expected) in availability {
        value.directional_validity[0] = validity;
        assert_eq!(value.model_interval_available(0), expected);
    }
    assert!(!value.model_interval_available(2));
}

const SLOT_COUNT: u8 = INTEGRAL_OBSERVATIONS_PER_RUNG as u8;

#[test]
fn slot_and_direction_mappings_are_mutual_inverses() {
    for order in IntegralSlotOrder::ALL {
        for slot in 0..SLOT_COUNT {
            let direction = order.direction_index(slot);
            let observation = order.observation_index(slot);
            assert!(
                direction < 2,
                "{order:?} slot {slot} direction out of range"
            );
            assert!(
                observation < 4,
                "{order:?} slot {slot} observation out of range"
            );
            assert_eq!(
                order.slot_for(observation, direction),
                slot,
                "{order:?} slot {slot} did not round-trip"
            );
        }

        for observation in 0..4 {
            for direction in 0..2 {
                let slot = order.slot_for(observation, direction);
                assert!(slot < SLOT_COUNT, "{order:?} produced slot {slot}");
                assert_eq!(
                    order.direction_index(slot),
                    direction,
                    "{order:?} direction"
                );
                assert_eq!(
                    order.observation_index(slot),
                    observation,
                    "{order:?} observation"
                );
            }
        }
    }
}

#[test]
fn each_slot_order_is_a_permutation_of_all_slots() {
    for order in IntegralSlotOrder::ALL {
        let mut seen = [false; SLOT_COUNT as usize];
        for observation in 0..4 {
            for direction in 0..2 {
                let slot = usize::from(order.slot_for(observation, direction));
                assert!(!seen[slot], "{order:?} reused slot {slot}");
                seen[slot] = true;
            }
        }
        assert!(
            seen.iter().all(|used| *used),
            "{order:?} left a slot unused"
        );
    }
}

#[test]
fn forward_first_preserves_the_historical_parity_mapping() {
    let order = IntegralSlotOrder::ForwardFirst;
    for slot in 0..SLOT_COUNT {
        let expected_direction = usize::from(slot % 2);
        assert_eq!(order.direction_index(slot), expected_direction);
        assert_eq!(
            order.signed_direction(slot),
            if slot.is_multiple_of(2) { 1 } else { -1 }
        );
    }
}

#[test]
fn workflow_kind_discriminants_are_frozen() {
    assert_eq!(VelocityAutotuneWorkflowKind::Resume as u8, 0);
    assert_eq!(
        VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral as u8,
        3
    );
    assert_eq!(
        VelocityAutotuneWorkflowKind::ReversalStandstillRobustnessGate as u8,
        4
    );
    assert_eq!(VelocityAutotuneWorkflowKind::PositionTune as u8, 5);

    assert_eq!(
        CommissioningWorkflowPlan::new(
            1,
            VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral,
            1,
            1
        ),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
}

#[test]
fn new_breakaway_reserves_the_command_level_envelope_before_probe_motion() {
    let envelope = CommissioningWorkflowPlan::new_breakaway(1, 400_000, 450_000).unwrap();
    assert_eq!(
        envelope.shape,
        VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral
    );
    assert_eq!(envelope.run_sequence, 1);
    assert_eq!(envelope.nominal_workflow_ms, 400_000);
    assert_eq!(envelope.maximum_workflow_ms, 450_000);

    assert_eq!(
        CommissioningWorkflowPlan::new_breakaway(1, 0, 450_000),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
    assert_eq!(
        CommissioningWorkflowPlan::new_breakaway(1, 450_000, 0),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
    assert_eq!(
        CommissioningWorkflowPlan::new_breakaway(1, 450_001, 450_000),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
}

#[test]
fn breakaway_workflow_reserves_upward_probe() {
    let primitive_us = MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US;
    assert_eq!(primitive_us, 1_248_000);
    assert_eq!(PROBE_MAX_OBSERVATIONS, 128);
    assert_eq!(breakaway_probe_slots_us(), Ok(159_744_000));
}

#[test]
fn discovery_slots_reserve_one_fresh_rest_window_per_observation() {
    let discovery_primitive_us =
        MOVING_STROKE_US + 2 * (ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US);
    assert_eq!(discovery_primitive_us, 1_984_000);
    assert_eq!(breakaway_discovery_slots_us(1), Ok(15_872_000));
}

#[test]
fn discovery_slot_duration_is_monotonic_across_every_resolved_rung_count() {
    let mut previous = 0_u64;
    for rung_count in (MIN_STRUCTURAL_RUNGS as u8)..=(MAX_SWEEP_RUNGS as u8) {
        let duration = breakaway_discovery_slots_us(rung_count).unwrap();
        assert!(
            duration > previous,
            "rung_count {rung_count} did not strictly increase: {duration} <= {previous}"
        );
        previous = duration;
    }
    assert_eq!(
        breakaway_discovery_slots_us(MAX_SWEEP_RUNGS as u8),
        Ok(507_904_000)
    );
}

#[test]
fn confirmation_is_eight_strokes_at_the_declared_family_size() {
    assert_eq!(breakaway_confirmation_slots_us(), Ok(9_984_000));
}

#[test]
fn envelope_sums_microseconds_across_every_phase_and_rounds_outward_once() {
    let recovery_slot_count = 56_u32;
    let recovery_maximum_slot_us = 3_000_000_u32;
    let integral_maximum_us = 182_512_000_u64;

    let total_us = breakaway_workflow_envelope_us(
        recovery_slot_count,
        recovery_maximum_slot_us,
        integral_maximum_us,
    )
    .unwrap();
    let expected_us = breakaway_probe_slots_us().unwrap()
        + u64::from(recovery_slot_count) * u64::from(recovery_maximum_slot_us)
        + breakaway_discovery_slots_us(MAX_SWEEP_RUNGS as u8).unwrap()
        + breakaway_confirmation_slots_us().unwrap()
        + integral_maximum_us;
    assert_eq!(total_us, expected_us);

    let total_ms = breakaway_workflow_envelope_ms(
        recovery_slot_count,
        recovery_maximum_slot_us,
        integral_maximum_us,
    )
    .unwrap();
    assert_eq!(total_ms, u32::try_from(total_us.div_ceil(1_000)).unwrap());
    assert!(total_ms < u32::MAX, "envelope must fit u32 milliseconds");

    let workflow = CommissioningWorkflowPlan::new_breakaway(1, total_ms, total_ms).unwrap();
    assert_eq!(workflow.maximum_workflow_ms, total_ms);
}

#[test]
fn envelope_arithmetic_is_checked_against_overflow() {
    assert_eq!(
        breakaway_workflow_envelope_us(0, 0, u64::MAX),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
    assert_eq!(
        breakaway_workflow_envelope_us(u32::MAX, u32::MAX, u64::MAX),
        Err(VelocityIntegralPlanError::Arithmetic)
    );

    let huge_but_u64_representable = 5_000_000_000_000_u64;
    assert!(breakaway_workflow_envelope_us(0, 0, huge_but_u64_representable).is_ok());
    assert_eq!(
        breakaway_workflow_envelope_ms(0, 0, huge_but_u64_representable),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
}

#[test]
fn discovery_phase_plan_must_fit_inside_the_reserved_envelope() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();

    let fits = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ));
    assert!(
        fits.is_ok(),
        "worst-case discovery must fit its own reserved envelope: {fits:?}"
    );

    let too_small = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        1_000,
    ));
    assert_eq!(
        too_small,
        Err(AdditiveLadderPlanError::WorkflowEnvelopeExceeded)
    );
}

#[test]
fn breakaway_digest_chain_binds_each_successive_plan() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();

    let discovery_plan = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ))
    .unwrap();
    assert_eq!(discovery_plan.probe_plan_digest, probe_plan.plan_digest);

    let nominated_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, nominated_p).unwrap();
    assert_eq!(
        confirmation_plan.discovery_plan_digest,
        discovery_plan.plan_digest
    );

    let (forward, reverse) = accepted_confirmation_evidence();
    let acceptance = evaluate_band_confirmation(nominated_p, forward, reverse)
        .expect("rung-18-class evidence confirms within the 70-80% band");
    assert_eq!(acceptance.nominated_p(), nominated_p);

    let bound = bind_breakaway_integral_authority(
        confirmation_plan.plan_digest,
        confirmation_plan,
        Ok(acceptance),
    )
    .unwrap();

    assert_eq!(bound.get(), confirmation_plan.plan_digest);
    assert_ne!(bound.get(), discovery_plan.plan_digest);
    assert_ne!(bound.get(), probe_plan.plan_digest);
    assert_ne!(discovery_plan.plan_digest, probe_plan.plan_digest);

    let authority = IntegralAcquisitionAuthority::new_breakaway(
        0,
        bound,
        acceptance.held_out_response_intervals_q,
        acceptance.confidence_q,
        acceptance.band_position_q,
    );
    assert_eq!(authority.acquisition, bound);
    assert_eq!(authority.acquisition.get(), confirmation_plan.plan_digest);
    assert_eq!(authority.proportional_joint_membership, 0);
    assert_eq!(authority.joint_membership, 0);
    assert_eq!(
        authority.directional_pooled_intervals,
        acceptance.held_out_response_intervals_q
    );
    assert_ne!(authority.directional_pooled_intervals, [[0, 0], [0, 0]]);
}

#[test]
fn new_breakaway_authority_stores_confirmation_intervals() {
    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(724));
    let token = breakaway_token_for(confirmation_plan);
    let intervals = [[3_i32, 9], [-9, -3]];

    let authority = IntegralAcquisitionAuthority::new_breakaway(0, token, intervals, 0, 0);

    assert_eq!(authority.directional_pooled_intervals, intervals);
}

#[test]
fn integral_authority_rejects_a_discovery_or_probe_digest() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();
    let discovery_plan = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ))
    .unwrap();
    let nominated_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, nominated_p).unwrap();
    let (forward, reverse) = accepted_confirmation_evidence();
    let acceptance = evaluate_band_confirmation(nominated_p, forward, reverse).unwrap();

    assert_eq!(
        bind_breakaway_integral_authority(
            discovery_plan.plan_digest,
            confirmation_plan,
            Ok(acceptance),
        ),
        Err(BreakawayAuthorityBindingError::AcquisitionDigestMismatch)
    );
    assert_eq!(
        bind_breakaway_integral_authority(
            probe_plan.plan_digest,
            confirmation_plan,
            Ok(acceptance)
        ),
        Err(BreakawayAuthorityBindingError::AcquisitionDigestMismatch)
    );
}

#[test]
fn integral_authority_rejects_a_failed_confirmation_even_with_a_matching_digest() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();
    let discovery_plan = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ))
    .unwrap();
    let nominated_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, nominated_p).unwrap();

    let failure =
        TargetBandConfirmationInconclusive(BandConfirmationFailureCause::ResponseLocation);
    assert_eq!(
        bind_breakaway_integral_authority(
            confirmation_plan.plan_digest,
            confirmation_plan,
            Err(failure)
        ),
        Err(BreakawayAuthorityBindingError::ConfirmationNotAccepted)
    );
}

#[test]
fn integral_authority_rejects_an_acceptance_for_a_different_candidate() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();
    let discovery_plan = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ))
    .unwrap();
    let nominated_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, nominated_p).unwrap();

    let (forward, reverse) = accepted_confirmation_evidence();
    let mismatched_acceptance = evaluate_band_confirmation(breakaway_gain(512), forward, reverse)
        .expect("rung-18-class evidence confirms within the 70-80% band");
    assert_eq!(mismatched_acceptance.nominated_p(), breakaway_gain(512));
    assert_eq!(
        bind_breakaway_integral_authority(
            confirmation_plan.plan_digest,
            confirmation_plan,
            Ok(mismatched_acceptance),
        ),
        Err(BreakawayAuthorityBindingError::AcceptanceCandidateMismatch)
    );
}

#[test]
fn breakaway_plan_digest_reflects_the_half_octave_insertion() {
    let plan = build_test_breakaway_plan();
    assert_eq!(plan.digest(), 16_233_484_769_674_924_850);
}

fn breakaway_acquisition_context() -> AcquisitionContext {
    let proportional = proportional_plan();
    let mut context = AcquisitionContext::from(proportional.execution_plan(proportional.digest()));
    context.plan_time_pwm_hz = 25_000;
    context.plan_time_encoder_counts_per_rev = 4_000;
    context
}

fn breakaway_confirmation_plan(nominated_p: PidPGain) -> BandConfirmationPlan {
    plan_band_confirmation(0x1111_2222_3333_4444, nominated_p).unwrap()
}

fn breakaway_token_for(confirmation_plan: BandConfirmationPlan) -> ConfirmedAcquisitionDigest {
    let (forward, reverse) = accepted_confirmation_evidence();
    let acceptance =
        evaluate_band_confirmation(confirmation_plan.nominated_p, forward, reverse).unwrap();
    bind_breakaway_integral_authority(
        confirmation_plan.plan_digest,
        confirmation_plan,
        Ok(acceptance),
    )
    .unwrap()
}

fn breakaway_intervals_for(confirmation_plan: BandConfirmationPlan) -> [[i32; 2]; 2] {
    let (forward, reverse) = accepted_confirmation_evidence();
    evaluate_band_confirmation(confirmation_plan.nominated_p, forward, reverse)
        .unwrap()
        .held_out_response_intervals_q
}

#[test]
fn retained_breakaway_authority_regenerates_its_exact_plan() {
    use crate::commission_dispatch::integral::{
        IntegralPreflightConfig, install_authority_from_plan, preflight_integral_request,
    };
    use crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;

    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(512));
    let plan = plan_breakaway_velocity_integral_response(
        breakaway_token_for(confirmation_plan),
        confirmation_plan,
        breakaway_acquisition_context(),
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
            confidence_q: 0,
            band_position_q: 0,
        },
        IntegralPlanPhysics {
            motion_budget: motion_budget(),
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
        },
    )
    .expect("the campaign's own Integral plan");
    let mut slot = IntegralRetentionSlot::new();
    install_authority_from_plan(0, &mut slot, plan);

    let preflight = preflight_integral_request(
        proportional_request(),
        &IntegralPreflightConfig {
            encoder_counts_per_rev: 4_000,
            torque_limit: 2_816,
            limit_evidence_capability: super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            channel: 0,
            pwm_hz: 25_000,
        },
        11,
        &slot,
    )
    .expect("retained breakaway authority is resumable")
    .expect("the slot is not empty");

    assert_eq!(preflight.plan, plan);
    assert_eq!(preflight.identity.plan_digest, plan.digest());
    assert_eq!(
        preflight.identity.schema_revision,
        BREAKAWAY_INTEGRAL_SCHEMA_REVISION
    );
    assert_eq!(preflight.run_sequence, 11);
}

#[test]
fn a_resume_under_changed_physical_context_is_refused() {
    use crate::commission_dispatch::integral::{
        IntegralPreflightConfig, IntegralPreflightRejection, install_authority_from_plan,
        preflight_integral_request,
    };
    use crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;

    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(512));
    let plan = plan_breakaway_velocity_integral_response(
        breakaway_token_for(confirmation_plan),
        confirmation_plan,
        breakaway_acquisition_context(),
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
            confidence_q: 0,
            band_position_q: 0,
        },
        IntegralPlanPhysics {
            motion_budget: motion_budget(),
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
        },
    )
    .expect("the campaign's own Integral plan");
    let mut slot = IntegralRetentionSlot::new();
    install_authority_from_plan(0, &mut slot, plan);
    let config = |pwm_hz, encoder_counts_per_rev| IntegralPreflightConfig {
        encoder_counts_per_rev,
        torque_limit: 2_816,
        limit_evidence_capability: super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        channel: 0,
        pwm_hz,
    };

    for (pwm_hz, counts) in [(26_000, 4_000), (25_000, 8_000)] {
        let rejection =
            preflight_integral_request(proportional_request(), &config(pwm_hz, counts), 13, &slot)
                .expect_err("a moved physical context must not resume");
        assert!(matches!(rejection, IntegralPreflightRejection::InvalidPlan));
    }

    assert!(
        preflight_integral_request(proportional_request(), &config(25_000, 4_000), 13, &slot)
            .expect("the retained context is unchanged")
            .is_some()
    );
}

#[test]
fn a_changed_request_still_cannot_resume_retained_breakaway_authority() {
    use crate::commission_dispatch::integral::{
        IntegralPreflightConfig, IntegralPreflightRejection, install_authority_from_plan,
        preflight_integral_request,
    };
    use crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;

    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(512));
    let plan = plan_breakaway_velocity_integral_response(
        breakaway_token_for(confirmation_plan),
        confirmation_plan,
        breakaway_acquisition_context(),
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
            confidence_q: 0,
            band_position_q: 0,
        },
        IntegralPlanPhysics {
            motion_budget: motion_budget(),
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
        },
    )
    .expect("the campaign's own Integral plan");
    let mut slot = IntegralRetentionSlot::new();
    let retained = install_authority_from_plan(0, &mut slot, plan);
    let mut changed = proportional_request();
    changed.max_stroke_travel_mrev += 1;

    let rejection = preflight_integral_request(
        changed,
        &IntegralPreflightConfig {
            encoder_counts_per_rev: 4_000,
            torque_limit: 2_816,
            limit_evidence_capability: super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            channel: 0,
            pwm_hz: 25_000,
        },
        12,
        &slot,
    )
    .expect_err("a changed budget must not resume the retained plan");

    assert!(matches!(
        rejection,
        IntegralPreflightRejection::RejectedPlanMismatch { .. }
    ));
    assert_eq!(slot.authority(), Some(retained));
}

#[test]
fn breakaway_integral_admits_the_confirmed_p_unchanged() {
    let nominated_p = breakaway_gain(512);
    let confirmation_plan = breakaway_confirmation_plan(nominated_p);
    let token = breakaway_token_for(confirmation_plan);
    let context = breakaway_acquisition_context();

    let plan = plan_breakaway_velocity_integral_response(
        token,
        confirmation_plan,
        context,
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
            confidence_q: 0,
            band_position_q: 0,
        },
        IntegralPlanPhysics {
            motion_budget: motion_budget(),
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
        },
    )
    .expect("the exact confirmed P passes every plan-time safety check");

    assert_eq!(plan.schema_revision, BREAKAWAY_INTEGRAL_SCHEMA_REVISION);
    assert_eq!(plan.fixed_p, nominated_p.raw());
    assert_eq!(plan.authority.acquisition, token);
    assert_eq!(
        plan.authority.acquisition.get(),
        confirmation_plan.plan_digest
    );
    assert_eq!(plan.authority.proportional_joint_membership, 0);
    assert_eq!(plan.authority.joint_membership, 0);
    assert!(!plan.probe_constrained_test_point);
    assert_eq!(
        plan.positive_rungs(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 724, 1_024]
    );
}

#[test]
fn breakaway_integral_refuses_without_motion_when_the_confirmed_p_fails_current_authority() {
    let nominated_p = breakaway_gain(1_024);
    let confirmation_plan = breakaway_confirmation_plan(nominated_p);
    let token = breakaway_token_for(confirmation_plan);
    let mut context = breakaway_acquisition_context();
    context.usable_torque_limit = 1;

    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 25_000,
                encoder_counts_per_rev: 4_000
            }
        ),
        Err(
            VelocityIntegralPlanError::ConfirmedGainNotTransitionCapable(
                confirmation_plan.plan_digest
            )
        )
    );
}

#[test]
fn max_transition_capable_p_is_the_inverse_of_the_admission_gate() {
    let ceiling = max_transition_capable_p(322, 4_000, 25_000, MOVING_STROKE_US, 2_534)
        .expect("valid acquisition context");
    assert_eq!(ceiling, 1_614);
    assert!(
        transition_capable_integral_gain(ceiling, 322, 4_000, 25_000, MOVING_STROKE_US, 2_534)
            .expect("valid inputs")
    );
    assert!(
        !transition_capable_integral_gain(ceiling + 1, 322, 4_000, 25_000, MOVING_STROKE_US, 2_534)
            .expect("valid inputs")
    );
}

#[test]
fn max_transition_capable_p_rises_as_target_rpm_falls() {
    assert_eq!(
        max_transition_capable_p(264, 4_000, 25_000, MOVING_STROKE_US, 2_534)
            .expect("valid acquisition context"),
        1_898
    );
}

#[test]
fn max_transition_capable_p_does_not_halve_when_the_pwm_carrier_doubles() {
    let at_25_khz = max_transition_capable_p(322, 4_000, 25_000, MOVING_STROKE_US, 2_301)
        .expect("valid acquisition context");
    let at_50_khz = max_transition_capable_p(322, 4_000, 50_000, MOVING_STROKE_US, 2_301)
        .expect("valid acquisition context");
    assert_eq!(at_50_khz, 1_416);
    assert!(
        at_50_khz > 731,
        "band-admissible candidates must be transition-capable"
    );
    assert!(
        u32::from(at_50_khz) * 100 > u32::from(at_25_khz) * 95,
        "carrier rate must not dominate the ceiling"
    );
}

#[test]
fn max_transition_capable_p_rejects_an_invalid_context() {
    assert_eq!(
        max_transition_capable_p(322, 0, 25_000, MOVING_STROKE_US, 2_534),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
    assert_eq!(
        max_transition_capable_p(322, 4_000, 25_000, MOVING_STROKE_US, 0),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
}

#[test]
fn max_transition_capable_p_is_zero_when_nothing_is_capable() {
    let ceiling = max_transition_capable_p(322, 4_000, 25_000, MOVING_STROKE_US, 1)
        .expect("valid acquisition context");
    assert_eq!(ceiling, 0);
    assert!(
        !transition_capable_integral_gain(1, 322, 4_000, 25_000, MOVING_STROKE_US, 1)
            .expect("valid inputs")
    );
}

#[test]
fn breakaway_integral_refuses_without_motion_when_the_fixed_schedule_exceeds_the_budget() {
    let nominated_p = breakaway_gain(512);
    let confirmation_plan = breakaway_confirmation_plan(nominated_p);
    let token = breakaway_token_for(confirmation_plan);
    let mut context = breakaway_acquisition_context();
    context.planned_velocity_mrev_s = 1;

    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 25_000,
                encoder_counts_per_rev: 4_000
            }
        ),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
}

#[test]
fn breakaway_integral_rejects_a_confirmation_plan_whose_contract_does_not_validate() {
    let nominated_p = breakaway_gain(512);
    let confirmation_plan = breakaway_confirmation_plan(nominated_p);
    let token = breakaway_token_for(confirmation_plan);
    let context = breakaway_acquisition_context();

    let mut tampered_digest = confirmation_plan;
    tampered_digest.plan_digest ^= 1;
    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            tampered_digest,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 25_000,
                encoder_counts_per_rev: 4_000
            }
        ),
        Err(VelocityIntegralPlanError::InvalidConfirmationPlan)
    );

    let mut tampered_family = confirmation_plan;
    tampered_family.family_size = 4;
    tampered_family.plan_digest = tampered_family.digest();
    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            tampered_family,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 25_000,
                encoder_counts_per_rev: 4_000
            }
        ),
        Err(VelocityIntegralPlanError::InvalidConfirmationPlan)
    );
}

#[test]
fn breakaway_integral_rejects_a_token_minted_for_a_different_confirmation_plan() {
    let nominated_p = breakaway_gain(512);
    let confirmation_plan_a = breakaway_confirmation_plan(nominated_p);
    let token_a = breakaway_token_for(confirmation_plan_a);
    let confirmation_plan_b = plan_band_confirmation(0x5555_6666_7777_8888, nominated_p).unwrap();
    let context = breakaway_acquisition_context();

    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token_a,
            confirmation_plan_b,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan_a),
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 25_000,
                encoder_counts_per_rev: 4_000
            }
        ),
        Err(VelocityIntegralPlanError::MismatchedAuthority)
    );
}

#[test]
fn breakaway_integral_requires_a_token_matching_the_confirmation_plan() {
    let forged_plan = breakaway_confirmation_plan(breakaway_gain(384));
    assert!(
        validate_fixed_contract(&forged_plan).is_ok(),
        "the forged plan is contract-valid; only the token binding stops it"
    );

    let genuine_plan = breakaway_confirmation_plan(breakaway_gain(512));
    let genuine_token = breakaway_token_for(genuine_plan);
    assert_ne!(genuine_token.get(), forged_plan.plan_digest);

    assert_eq!(
        plan_breakaway_velocity_integral_response(
            genuine_token,
            forged_plan,
            breakaway_acquisition_context(),
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: breakaway_intervals_for(genuine_plan),
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 25_000,
                encoder_counts_per_rev: 4_000
            }
        ),
        Err(VelocityIntegralPlanError::MismatchedAuthority)
    );
}

#[test]
fn breakaway_integral_refuses_when_plan_time_physical_context_differs_from_live() {
    let nominated_p = breakaway_gain(512);
    let confirmation_plan = breakaway_confirmation_plan(nominated_p);
    let token = breakaway_token_for(confirmation_plan);

    let context = breakaway_acquisition_context();
    let intervals = breakaway_intervals_for(confirmation_plan);
    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: intervals,
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 50_000,
                encoder_counts_per_rev: 4_000
            }
        ),
        Err(VelocityIntegralPlanError::PhysicalContextMismatch)
    );

    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: intervals,
                confidence_q: 0,
                band_position_q: 0
            },
            IntegralPlanPhysics {
                motion_budget: motion_budget(),
                pwm_hz: 25_000,
                encoder_counts_per_rev: 8_000
            }
        ),
        Err(VelocityIntegralPlanError::PhysicalContextMismatch)
    );
}

#[test]
fn integral_top_gap_candidate_at_todays_cap_is_724() {
    assert_eq!(integral_top_gap_candidate(1_310), Some(724));
}

#[test]
fn integral_top_gap_candidate_none_for_zero_sentinel() {
    assert_eq!(integral_top_gap_candidate(0), None);
}

#[test]
fn integral_top_gap_candidate_none_when_lower_octave_below_four() {
    assert_eq!(integral_top_gap_candidate(4), None);
    assert_eq!(integral_top_gap_candidate(7), None);
}

#[test]
fn integral_top_gap_candidate_present_at_the_smallest_eligible_cap() {
    assert_eq!(integral_top_gap_candidate(8), Some(6));
}

#[test]
fn native_rung_count_matches_native_velocity_i_at_walk() {
    assert_eq!(native_rung_count(1_310), 12);
    assert_eq!(native_rung_count(4), 4);
}

#[test]
fn integral_gain_at_with_zero_cap_matches_native_velocity_i_at_unaugmented() {
    for index in 0..12 {
        assert_eq!(integral_gain_at(0, index), native_velocity_i_at(index));
    }
    assert_eq!(integral_gain_at(0, 200), None);
}

#[test]
fn integral_gain_at_inserts_the_candidate_before_the_final_native_rung() {
    let expected = [1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 724, 1024];
    for (index, value) in expected.iter().enumerate() {
        assert_eq!(integral_gain_at(1_310, index as u8), Some(*value));
    }
    assert_eq!(integral_gain_at(1_310, 13), None);
}

#[test]
fn integral_gain_at_final_rung_closing_bookend_and_first_out_of_range() {
    assert_eq!(integral_gain_at(1_310, 12), Some(1_024));
    assert_eq!(integral_gain_at(1_310, 13), None);
}

#[test]
fn integral_gain_at_zero_sentinel_is_unbounded_like_the_pre_existing_behavior() {
    assert_eq!(integral_gain_at(0, 12), Some(2_048));
}

#[test]
fn schema_17_still_hashes_i_top_cap_in_its_digest() {
    let mut plan = build_test_breakaway_plan();
    plan.schema_revision = INTEGRAL_EVIDENCE_SCHEMA_REVISION;
    assert_eq!(plan.schema_revision, 17);
    let baseline = plan.digest();
    plan.i_top_cap = plan.i_top_cap.wrapping_add(1);
    assert_ne!(plan.digest(), baseline);
}

#[test]
fn schema_16_breakaway_plan_also_hashes_i_top_cap_changes_in_its_digest() {
    let mut plan = build_test_breakaway_plan();
    assert_eq!(plan.schema_revision, BREAKAWAY_INTEGRAL_SCHEMA_REVISION);
    let baseline = plan.digest();
    plan.i_top_cap = plan.i_top_cap.wrapping_add(1);
    assert_ne!(plan.digest(), baseline);
}

#[test]
fn other_schema_revisions_still_ignore_i_top_cap_changes_in_their_digest() {
    let mut plan = build_test_breakaway_plan();
    plan.schema_revision = 3;
    let baseline = plan.digest();
    plan.i_top_cap = 12_345;
    assert_eq!(plan.digest(), baseline);
}
