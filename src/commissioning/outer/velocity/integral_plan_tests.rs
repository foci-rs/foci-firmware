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
use super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION;
use super::integral_plan::{
    AcceptanceSource, AcquisitionContext, BreakawayAuthorityBindingError,
    ConfirmedAcquisitionDigest, INTEGRAL_EVIDENCE_SCHEMA_REVISION, INTEGRAL_MAX_POSITIVE_RUNGS,
    INTEGRAL_MIN_POSITIVE_I, INTEGRAL_OBSERVATIONS_PER_RUNG, IntegralAcquisitionAuthority,
    IntegralSlotOrder, VelocityIntegralPlan, VelocityIntegralPlanError,
    bind_breakaway_integral_authority, breakaway_confirmation_slots_us,
    breakaway_discovery_slots_us, breakaway_probe_slots_us, breakaway_workflow_envelope_ms,
    breakaway_workflow_envelope_us, build_integral_ladder, confidence_family_size,
    integral_gain_at, integral_top_gap_candidate, integral_workflow_maximum_ms,
    max_transition_capable_p, native_rung_count, native_velocity_i_at,
    plan_breakaway_velocity_integral_response, recovery_quantization_exposed, response_horizon,
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
use crate::commissioning::outer::velocity::reproduction::StageBDirectionalValidity;
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};
use crate::commissioning::types::ResponseHorizonClass;
use foci_tmc4671::pid::PidPGain;

fn stage_b_request() -> VelocitySweepRequest {
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
        stage_b_joint_membership: 0x0007_0000,
        joint_membership: 0x0007_0000,
        directional_validity: [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2],
        reduced_margin: false,
        directional_pooled_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
        confidence_q: 0,
        band_position_q: 0,
    }
}

fn stage_b_plan() -> super::sweep_plan::VelocitySweepPlan {
    plan_velocity_sweep(
        stage_b_request(),
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
    }
}

// --- Breakaway campaign fixtures --------------------------------------------
//
// The half-octave search grid from 1 to 1024 is worked out exactly in spec
// section 1: 1, 2, 3, 4, 6, 8, 12, 16, 23, 32, 46, 64, 91, 128, 181, 256, 362,
// 512, 724, 1024. Gain 256 (index 15) has a genuine predecessor, 181 (index
// 14), so it exercises `GridFloorOrigin::Predecessor` rather than the clamped
// edge case.

fn breakaway_gain(raw: u16) -> PidPGain {
    PidPGain::new(raw).expect("raw gain within PidPGain range")
}

/// Canonical acquisition context for the probe fixture; exact values are
/// arbitrary but fixed.
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

/// Build the probe plan and resolve its floor/breakaway pair around gain 256.
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

/// Rung-18-class evidence from spec section 2 (relative SE 1.022% at 75%
/// response), reproduced exactly via integer `mean_rate_q` /
/// `variance_of_mean_q2` so the production formulas compute the same
/// containment as the worked example, not a hand-rounded approximation.
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
fn response_horizon_classifies_anchors_and_the_64_to_128_boundary() {
    assert_eq!(
        response_horizon(0, 25_000, 512_000),
        Some((0, ResponseHorizonClass::NotApplicable))
    );
    assert_eq!(
        response_horizon(64, 25_000, 512_000),
        Some((655_360, ResponseHorizonClass::SubHorizon))
    );
    assert_eq!(
        response_horizon(128, 25_000, 512_000),
        Some((327_680, ResponseHorizonClass::HorizonResolved))
    );
}

#[test]
fn recovery_quantization_exposure_is_exact_and_checked() {
    assert_eq!(
        recovery_quantization_exposed(724, 264, 4_000, 25_000, 2_534),
        Ok(false)
    );
    assert_eq!(
        recovery_quantization_exposed(1_024, 264, 4_000, 25_000, 2_534),
        Ok(true)
    );
    assert_eq!(recovery_quantization_exposed(256, 0, 1, 1, 60), Ok(false));
    assert_eq!(
        recovery_quantization_exposed(1, i32::MIN, 1, 1, 1),
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
fn preflight_duration_reserves_the_fixed_cleanup_window() {
    assert_eq!(integral_workflow_maximum_ms(25_000, 3_000), Ok(182_512));
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
    // family_size = confidence_family_size(13) = (13 + 2) * INTEGRAL_OBSERVATIONS_PER_DIRECTION.
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

/// One genuine breakaway-schema plan, built through the real
/// [`plan_breakaway_velocity_integral_response`] path used by the breakaway
/// fixtures below (see [`breakaway_acquisition_context`]).
fn build_test_breakaway_plan() -> VelocityIntegralPlan {
    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(512));
    plan_breakaway_velocity_integral_response(
        breakaway_token_for(confirmation_plan),
        confirmation_plan,
        breakaway_acquisition_context(),
        breakaway_intervals_for(confirmation_plan),
        0,
        0,
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("the campaign's own Stage-C plan")
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

/// `CommissioningWorkflowPlan::new` admits exactly `Resume`
/// (`stage_b_ms == 0 && integral_ms != 0`); every other shape it can name is
/// rejected -- the fixed-gain-amplitude and breakaway shapes have their own
/// dedicated constructors instead (see
/// `workflow_kind_discriminants_are_a_clean_zero_through_four_sequence`).
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
        "Resume names no Stage-B duration"
    );
    assert_eq!(
        CommissioningWorkflowPlan::new(9, VelocityAutotuneWorkflowKind::Resume, 0, 0),
        Err(VelocityIntegralPlanError::InvalidRequest),
        "Resume must name a Stage-C duration"
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
        (StageBDirectionalValidity::NoRegion, false),
        (StageBDirectionalValidity::Fragment, false),
        (StageBDirectionalValidity::RegionNotReproduced, false),
        (
            StageBDirectionalValidity::ReproducedRegionCoversFinalP,
            true,
        ),
        (StageBDirectionalValidity::ReproducedRegionTransferred, true),
    ];
    let mut value = authority();
    for (validity, expected) in availability {
        value.directional_validity[0] = validity;
        assert_eq!(value.model_interval_available(0), expected);
    }
    assert!(!value.model_interval_available(2));
}

const SLOT_COUNT: u8 = INTEGRAL_OBSERVATIONS_PER_RUNG as u8;

/// The slot index and the travel direction must round-trip in both directions.
///
/// `rung_classes` and `rung_intervals` are written by raw slot index and read
/// back by (observation, direction) in `fixed_gain_amplitude_report`. If the two
/// mappings ever disagree, forward evidence is filed as reverse and the run is
/// silently wrong rather than failing. This pins them together.
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

/// Every slot must be used exactly once, so no observation is dropped or doubled.
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

/// The default order must keep the historical parity mapping byte for byte.
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
fn workflow_kind_discriminants_are_a_clean_zero_through_four_sequence() {
    assert_eq!(VelocityAutotuneWorkflowKind::Resume as u8, 0);
    assert_eq!(
        VelocityAutotuneWorkflowKind::FixedGainAmplitudeAscending as u8,
        1
    );
    assert_eq!(
        VelocityAutotuneWorkflowKind::FixedGainAmplitudeDescending as u8,
        2
    );
    assert_eq!(
        VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral as u8,
        3
    );
    assert_eq!(
        VelocityAutotuneWorkflowKind::ReversalStandstillRobustnessGate as u8,
        4
    );

    // The breakaway shape has its own dedicated constructor, matching the
    // existing precedent set by the fixed-gain-amplitude shapes: it is not
    // constructible through the generic two-duration `new`.
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

// --- Duration accounting ----------------------------------------------------

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
    // The worst case is exactly 32 rungs x 8 slots, matching spec section 8's
    // "32 rungs x 8 slots" line.
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
    let integral_maximum_us =
        u64::from(integral_workflow_maximum_ms(25_000, 3_000).unwrap()) * 1_000;

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
    // Rounding happens exactly once: rounding each term separately first
    // would never produce a smaller total than rounding the exact sum.
    assert!(total_ms < u32::MAX, "envelope must fit u32 milliseconds");

    let workflow = CommissioningWorkflowPlan::new_breakaway(1, total_ms, total_ms).unwrap();
    assert_eq!(workflow.maximum_workflow_ms, total_ms);
}

#[test]
fn envelope_arithmetic_is_checked_against_overflow() {
    // `integral_maximum_us` alone large enough to overflow the running `u64`
    // sum must be rejected, not wrapped.
    assert_eq!(
        breakaway_workflow_envelope_us(0, 0, u64::MAX),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
    // The recovery term (`recovery_slot_count * recovery_maximum_slot_us`)
    // pushed to its extreme, combined with a near-maximal Stage-C term, must
    // also be rejected.
    assert_eq!(
        breakaway_workflow_envelope_us(u32::MAX, u32::MAX, u64::MAX),
        Err(VelocityIntegralPlanError::Arithmetic)
    );

    // A microsecond total that fits `u64` but whose outward millisecond
    // rounding would exceed `u32::MAX` must be rejected by the `_ms`
    // wrapper, never silently truncated.
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

// --- Digest chain ------------------------------------------------------------

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

    // The token is the only way to fill a breakaway authority's acquisition:
    // `IntegralAcquisitionAuthority::new_breakaway` takes a `ConfirmedAcquisitionDigest`,
    // not a bare `u64`, so this call could not have compiled with a hand-picked
    // digest, only with the value the gate above actually validated.
    let authority = IntegralAcquisitionAuthority::new_breakaway(
        0,
        bound,
        acceptance.held_out_response_intervals_q,
        acceptance.confidence_q,
        acceptance.band_position_q,
    );
    assert_eq!(authority.acquisition, bound);
    assert_eq!(authority.acquisition.get(), confirmation_plan.plan_digest);
    assert_eq!(authority.stage_b_joint_membership, 0);
    assert_eq!(authority.joint_membership, 0);
    // The digest chain also carries the held-out block's own pooled response
    // interval through to the authority, not a placeholder zero.
    assert_eq!(
        authority.directional_pooled_intervals,
        acceptance.held_out_response_intervals_q
    );
    assert_ne!(authority.directional_pooled_intervals, [[0, 0], [0, 0]]);
}

/// Isolated from any real confirmation math: `new_breakaway` is a pure
/// storage constructor for whatever `held_out_response_intervals_q` its
/// caller supplies, so an arbitrary well-formed literal proves the plumbing
/// without depending on `evaluate_band_confirmation`'s actual pooled values
/// (already covered above, end to end, by
/// `breakaway_digest_chain_binds_each_successive_plan`).
#[test]
fn new_breakaway_authority_stores_confirmation_intervals() {
    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(724));
    let token = breakaway_token_for(confirmation_plan);
    let intervals = [[3_i32, 9], [-9, -3]];

    let authority = IntegralAcquisitionAuthority::new_breakaway(0, token, intervals, 0, 0);

    assert_eq!(authority.directional_pooled_intervals, intervals);
}

// `ConfirmedAcquisitionDigest`'s field is private to `integral_plan.rs`.
// This file, `integral_plan_tests.rs`, is a *sibling* module declared in
// `mod.rs` (`pub mod integral_plan; #[cfg(test)] mod integral_plan_tests;`),
// not a descendant of `integral_plan`, so it has no access to that private
// field: neither `ConfirmedAcquisitionDigest(42)` nor
// `IntegralAcquisitionAuthority::new_breakaway(0, ConfirmedAcquisitionDigest(42))` would
// compile in this file (the compiler's own suggestion when the digest chain
// test above was still returning a bare `u64` was literally to wrap it in
// `ConfirmedAcquisitionDigest(..)` here, followed by a note that doing so
// requires changing the field's privacy -- i.e. it does not compile as
// written). There is no `compile_fail` doctest for this: the type is
// `pub(crate)`, invisible outside the crate entirely, so an external doctest
// binary can't even name it. The guarantee is structural, checked by every
// build of this crate: every call site in every module other than
// `integral_plan.rs` must obtain a `ConfirmedAcquisitionDigest` from
// `bind_breakaway_integral_authority`, with no other route to one.

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

    // The discovery digest is offered where the confirmation digest belongs.
    assert_eq!(
        bind_breakaway_integral_authority(
            discovery_plan.plan_digest,
            confirmation_plan,
            Ok(acceptance),
        ),
        Err(BreakawayAuthorityBindingError::AcquisitionDigestMismatch)
    );
    // The probe digest is rejected the same way.
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

    // The confirmation plan's own digest is well formed and matches exactly,
    // but the held-out block did not accept: Integral must still be refused.
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

    // A `BandConfirmationAcceptance` cannot be forged from this sibling module
    // (its field is private to `band_confirmation`), so the mismatched
    // acceptance is a genuine held-out pass -- for a *different* candidate gain,
    // 512 -- produced through `evaluate_band_confirmation`. Binding it against a
    // plan whose candidate is 724 is the mismatch under test.
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

// --- Legacy replay compatibility ---------------------------------------------

/// Pins a breakaway-sourced plan's exact digest. Breakaway plans now also
/// receive the half-octave insertion, so this value changes whenever the
/// ladder generation changes -- unlike the earlier ReproducedStageB-only
/// gating, there is no longer an untouched path to separately pin.
#[test]
fn breakaway_plan_digest_reflects_the_half_octave_insertion() {
    let plan = build_test_breakaway_plan();
    assert_eq!(plan.digest(), 106_042_503_528_755_426);
}

// --- Breakaway Stage-C constructor -------------------------------------------

/// Physical acquisition context for the breakaway fixtures below, built from
/// the same [`stage_b_plan`] fixture the legacy tests use so the confirmed
/// gain 512 is known (from `integral_plan_freezes_geometry_and_conservative_duration`)
/// to pass [`super::integral_plan::transition_capable_integral_gain`] under
/// `pwm_hz = 25_000`, `encoder_counts_per_rev = 4_000`.
fn breakaway_acquisition_context() -> AcquisitionContext {
    let stage_b = stage_b_plan();
    let mut context = AcquisitionContext::from(stage_b.execution_plan(stage_b.digest()));
    // The breakaway gate refuses if the plan-time physical context diverges from
    // the live pwm/encoder it is handed; every fixture below passes 25 kHz /
    // 4_000, so freeze the same values here.
    context.plan_time_pwm_hz = 25_000;
    context.plan_time_encoder_counts_per_rev = 4_000;
    context
}

fn breakaway_confirmation_plan(nominated_p: PidPGain) -> BandConfirmationPlan {
    plan_band_confirmation(0x1111_2222_3333_4444, nominated_p).unwrap()
}

/// Mint the breakaway confirmation token for `confirmation_plan` through the
/// real [`bind_breakaway_integral_authority`] gate, exactly as a live campaign
/// would, rather than fabricating the digest. The token is the only input the
/// Stage-C gate accepts as authority.
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

/// The same accepted held-out block's pooled response interval, for the
/// fixture callers that mint a token with [`breakaway_token_for`] and also
/// need [`plan_breakaway_velocity_integral_response`]'s
/// `held_out_response_intervals_q` argument. A live campaign sources both
/// values from the one [`BandConfirmationAcceptance`] it already holds
/// (`breakaway_campaign.rs`); this recomputes the same accepted evidence
/// rather than threading a tuple through every fixture call site.
fn breakaway_intervals_for(confirmation_plan: BandConfirmationPlan) -> [[i32; 2]; 2] {
    let (forward, reverse) = accepted_confirmation_evidence();
    evaluate_band_confirmation(confirmation_plan.nominated_p, forward, reverse)
        .unwrap()
        .held_out_response_intervals_q
}

/// A campaign's Stage-C plan must be resumable from retained authority alone.
///
/// The breakaway campaign has no Stage-B phase, so nothing recomputes a sweep
/// plan for it. Before this the preflight compared the retained confirmation
/// token against a freshly planned Stage-B digest and refused every resume,
/// which made the `CompleteCandidate` to `Complete` promotion unreachable for
/// the only campaign that produces candidates.
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
        breakaway_intervals_for(confirmation_plan),
        0,
        0,
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("the campaign's own Stage-C plan");
    let mut slot = IntegralRetentionSlot::new();
    install_authority_from_plan(0, &mut slot, plan);

    let preflight = preflight_integral_request(
        stage_b_request(),
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

/// The plan-digest check is the only identity guard on the resume path, so it
/// has to carry the weight the Stage-B digest comparison carries elsewhere.
/// Live PWM and encoder resolution must be re-checked, not assumed.
///
/// The plan digest cannot catch this on its own: a plan rebuilt from retained
/// context hashes the retained PWM and encoder values again, so it still
/// matches the retained identity even when the live configuration has moved.
/// Execution then does its target-rate math against the stale resolution.
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
        breakaway_intervals_for(confirmation_plan),
        0,
        0,
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("the campaign's own Stage-C plan");
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
            preflight_integral_request(stage_b_request(), &config(pwm_hz, counts), 13, &slot)
                .expect_err("a moved physical context must not resume");
        assert!(matches!(rejection, IntegralPreflightRejection::InvalidPlan));
    }

    // The unchanged context still resumes, so the guard is not simply refusing
    // everything.
    assert!(
        preflight_integral_request(stage_b_request(), &config(25_000, 4_000), 13, &slot)
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
        breakaway_intervals_for(confirmation_plan),
        0,
        0,
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("the campaign's own Stage-C plan");
    let mut slot = IntegralRetentionSlot::new();
    let retained = install_authority_from_plan(0, &mut slot, plan);
    let mut changed = stage_b_request();
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
        breakaway_intervals_for(confirmation_plan),
        0,
        0,
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("the exact confirmed P passes every plan-time safety check");

    assert_eq!(plan.schema_revision, BREAKAWAY_INTEGRAL_SCHEMA_REVISION);
    assert_eq!(plan.fixed_p, nominated_p.raw());
    assert_eq!(plan.authority.acquisition, token);
    assert_eq!(
        plan.authority.acquisition.get(),
        confirmation_plan.plan_digest
    );
    assert_eq!(plan.authority.stage_b_joint_membership, 0);
    assert_eq!(plan.authority.joint_membership, 0);
    assert!(!plan.probe_constrained_test_point);
    // The existing non-combined ladder, not the exact-cap combined shape.
    // Breakaway plans
    // now also receive the half-octave rung (724, between 512 and 1024).
    assert_eq!(
        plan.positive_rungs(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 724, 1_024]
    );
}

/// `acquisition_kind == ResponseLadder` alone cannot isolate breakaway: a
/// live combined workflow and an accepted breakaway confirmation both report
/// that same kind. The `acceptance_source` discriminator, resolved from the
/// Stage-C authority, is what `records_target_terminus` keys on instead. The
/// non-breakaway half of the contract -- it must not over-admit a plan that
/// carries no target evidence -- is covered by
/// `non_breakaway_ladder_does_not_record_a_terminus` in
/// `integral_sweep_tests`.
#[test]
fn execution_plan_reports_acceptance_source_from_authority() {
    let confirmation_plan = breakaway_confirmation_plan(breakaway_gain(512));
    let token = breakaway_token_for(confirmation_plan);
    let breakaway = plan_breakaway_velocity_integral_response(
        token,
        confirmation_plan,
        breakaway_acquisition_context(),
        breakaway_intervals_for(confirmation_plan),
        0,
        0,
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("the campaign's own Stage-C plan")
    .execution_plan();
    assert_eq!(breakaway.acceptance_source, AcceptanceSource::Breakaway);
    assert!(breakaway.records_target_terminus());
}

/// No reselection, no fallback: the constructor takes exactly one candidate
/// gain and either admits it or refuses. There is no alternate-candidate
/// parameter for it to fall back to, so a failing confirmed gain can only
/// ever produce a refusal, never a plan built around a substitute P.
#[test]
fn breakaway_integral_refuses_without_motion_when_the_confirmed_p_fails_current_authority() {
    let nominated_p = breakaway_gain(1_024);
    let confirmation_plan = breakaway_confirmation_plan(nominated_p);
    let token = breakaway_token_for(confirmation_plan);
    let mut context = breakaway_acquisition_context();
    // A torque authority this small cannot carry any nonzero I prefix at the
    // Q4.12 gain scale, regardless of the confirmed P: this simultaneously
    // stands in for both "transition capability" and "current authority" in
    // the brief's checklist, because `transition_capable_integral_gain`
    // evaluates them as one combined datasheet-derived guard.
    context.usable_torque_limit = 1;

    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            breakaway_intervals_for(confirmation_plan),
            0,
            0,
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(
            VelocityIntegralPlanError::ConfirmedGainNotTransitionCapable(
                confirmation_plan.plan_digest
            )
        )
    );
}

/// The nomination ceiling is the exact monotone inverse of the Stage-C
/// admission gate: the returned P passes `transition_capable_integral_gain` and
/// the next-higher P fails it. Values are a live acquisition context
/// (322 RPM, usable torque 2534, 4000 cpr, 25 kHz), where the ceiling is 897 --
/// below the confirmed P=925 that discovery nominated and Integral refused.
#[test]
fn max_transition_capable_p_is_the_inverse_of_the_admission_gate() {
    let ceiling = max_transition_capable_p(322, 4_000, 25_000, MOVING_STROKE_US, 2_534)
        .expect("valid acquisition context");
    assert_eq!(ceiling, 897);
    assert!(
        transition_capable_integral_gain(ceiling, 322, 4_000, 25_000, MOVING_STROKE_US, 2_534)
            .expect("valid inputs")
    );
    assert!(
        !transition_capable_integral_gain(ceiling + 1, 322, 4_000, 25_000, MOVING_STROKE_US, 2_534)
            .expect("valid inputs")
    );
}

/// The ceiling tracks the acquisition context: a lower target RPM raises it.
/// At 264 RPM the largest transition-capable P is 979, matching the Stage-C
/// admission audit's worked example.
#[test]
fn max_transition_capable_p_rises_as_target_rpm_falls() {
    assert_eq!(
        max_transition_capable_p(264, 4_000, 25_000, MOVING_STROKE_US, 2_534)
            .expect("valid acquisition context"),
        979
    );
}

/// An invalid acquisition context is a hard error, not a `0` ceiling: the
/// caller must be able to tell "no gain is capable" (ceiling 0) apart from
/// "the context is malformed" (Err), so a should-never planning failure is
/// reported as an internal fault rather than masked as a legitimate refusal.
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

/// When even `p = 1` would clip the probe clamp, no gain is transition-capable
/// and the ceiling is 0, so nomination has nothing to nominate.
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
    // The confirmed P still passes transition capability; only the recovery
    // travel time this implies -- driven by a near-stalled planned velocity
    // against the declared stroke travel -- no longer fits the declared
    // per-primitive duration budget.
    context.planned_velocity_mrev_s = 1;

    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            breakaway_intervals_for(confirmation_plan),
            0,
            0,
            motion_budget(),
            25_000,
            4_000,
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
            breakaway_intervals_for(confirmation_plan),
            0,
            0,
            motion_budget(),
            25_000,
            4_000,
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
            breakaway_intervals_for(confirmation_plan),
            0,
            0,
            motion_budget(),
            25_000,
            4_000,
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
            breakaway_intervals_for(confirmation_plan_a),
            0,
            0,
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MismatchedAuthority)
    );
}

/// The round-2 acceptance criterion: there is NO in-crate path that reaches
/// breakaway Stage-C plan construction without a genuine
/// [`ConfirmedAcquisitionDigest`] token. The gate's only authority input is the
/// token, which can be minted solely by [`bind_breakaway_integral_authority`]
/// from a real `Ok(BandConfirmationAcceptance)` -- itself unforgeable. A caller
/// can still fabricate a self-consistent `BandConfirmationPlan` for an arbitrary
/// gain (all fixed fields correct, `plan_digest = plan.digest()`), but cannot
/// mint a token whose value equals that forged plan's digest without a genuine
/// all-eight acceptance of THAT plan, so the plan-to-token binding refuses it.
#[test]
fn breakaway_integral_requires_a_token_matching_the_confirmation_plan() {
    // A forged plan for an arbitrary candidate, contract-valid and
    // digest-self-consistent -- exactly what an in-crate attacker could build.
    let forged_plan = breakaway_confirmation_plan(breakaway_gain(384));
    assert!(
        validate_fixed_contract(&forged_plan).is_ok(),
        "the forged plan is contract-valid; only the token binding stops it"
    );

    // The only token the attacker can actually mint is for a DIFFERENT,
    // genuinely accepted plan; its digest cannot equal the forged plan's.
    let genuine_plan = breakaway_confirmation_plan(breakaway_gain(512));
    let genuine_token = breakaway_token_for(genuine_plan);
    assert_ne!(genuine_token.get(), forged_plan.plan_digest);

    // Pairing the only obtainable token with the forged plan is refused: no
    // Stage-C plan for gain 384 is ever constructed. The token is the gate's
    // sole authority input -- there is no `IntegralAcquisitionAuthority` parameter through
    // which a forged authority could enter -- so this is the whole attack
    // surface, and it is closed. (A token genuinely minted for the forged plan
    // would require driving `evaluate_band_confirmation` to accept THAT plan,
    // which is the real confirmation gate, not a bypass; the admit-on-genuine
    // path is covered by `breakaway_integral_admits_the_confirmed_p_unchanged`.)
    assert_eq!(
        plan_breakaway_velocity_integral_response(
            genuine_token,
            forged_plan,
            breakaway_acquisition_context(),
            breakaway_intervals_for(genuine_plan),
            0,
            0,
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MismatchedAuthority)
    );
}

#[test]
fn breakaway_integral_refuses_when_plan_time_physical_context_differs_from_live() {
    let nominated_p = breakaway_gain(512);
    let confirmation_plan = breakaway_confirmation_plan(nominated_p);
    let token = breakaway_token_for(confirmation_plan);

    // Plan-time PWM (25 kHz) but a live PWM of 50 kHz: confirmation evidence was
    // gathered under a different timebase, so the gate refuses without motion.
    let context = breakaway_acquisition_context();
    let intervals = breakaway_intervals_for(confirmation_plan);
    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            intervals,
            0,
            0,
            motion_budget(),
            50_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::PhysicalContextMismatch)
    );

    // Live encoder resolution differs from plan time: same fail-closed refusal.
    assert_eq!(
        plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            intervals,
            0,
            0,
            motion_budget(),
            25_000,
            8_000,
        ),
        Err(VelocityIntegralPlanError::PhysicalContextMismatch)
    );
}

#[test]
fn integral_top_gap_candidate_at_todays_cap_is_724() {
    // upper_octave=1024 (largest native rung <= 1310), lower_octave=512.
    assert_eq!(integral_top_gap_candidate(1_310), Some(724));
}

#[test]
fn integral_top_gap_candidate_none_for_zero_sentinel() {
    assert_eq!(integral_top_gap_candidate(0), None);
}

#[test]
fn integral_top_gap_candidate_none_when_lower_octave_below_four() {
    // cap=4: upper_octave=4, lower_octave=2 -- below the prefix guard.
    assert_eq!(integral_top_gap_candidate(4), None);
    // cap=7: upper_octave=4 still (8 > 7), lower_octave=2 -- still guarded.
    assert_eq!(integral_top_gap_candidate(7), None);
}

#[test]
fn integral_top_gap_candidate_present_at_the_smallest_eligible_cap() {
    // cap=8: upper_octave=8, lower_octave=4 (eligible), candidate=6.
    assert_eq!(integral_top_gap_candidate(8), Some(6));
}

#[test]
fn native_rung_count_matches_native_velocity_i_at_walk() {
    assert_eq!(native_rung_count(1_310), 12); // 1,2,3,4,8,16,32,64,128,256,512,1024
    assert_eq!(native_rung_count(4), 4); // 1,2,3,4
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
    // native_rung_count(1310) = 12, indices 0..=11 are 1,2,3,4,...,512,1024.
    // Augmented: indices 0..=9 unchanged (1..256), index 10 = 724 (was 512's
    // successor slot -- 512 itself is index 9, unchanged), wait: the
    // insertion happens immediately before the FINAL native rung (1024,
    // index 11), so index 11 becomes 724 and a new index 12 becomes 1024.
    let expected = [1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 724, 1024];
    for (index, value) in expected.iter().enumerate() {
        assert_eq!(integral_gain_at(1_310, index as u8), Some(*value));
    }
    assert_eq!(integral_gain_at(1_310, 13), None);
}

#[test]
fn integral_gain_at_final_rung_closing_bookend_and_first_out_of_range() {
    // "Final positive rung" per the spec's boundary-test requirement: index
    // 12 (1024) is the last positive rung; index 13 is out of range.
    assert_eq!(integral_gain_at(1_310, 12), Some(1_024));
    assert_eq!(integral_gain_at(1_310, 13), None);
}

#[test]
fn integral_gain_at_zero_sentinel_is_unbounded_like_the_pre_existing_behavior() {
    // integral_gain_at itself does not bound the sentinel case (matching
    // native_velocity_i_at's own pre-existing unbounded behavior) --
    // guarding against the round-5 mistake (a Breakaway plan whose
    // positive_rung_count disagrees with what integral_gain_at can produce)
    // is Task 4's responsibility (the count and the accessor must be
    // derived from the same construction), checked by
    // breakaway_plan_includes_the_half_octave_rung_and_real_cap
    // and breakaway_plan_i_top_cap_is_zero_and_rungs_are_unaugmented in
    // integral_plan_tests.rs, and by integral_i_at's own
    // `rung_index >= total` bound in integral_sweep.rs, which is what
    // actually prevents a caller from ever reaching index 12 on a 12-rung
    // Breakaway plan.
    assert_eq!(integral_gain_at(0, 12), Some(2_048));
}

/// Schema 17 no longer has a live producer (it was `plan_velocity_integral_response`'s
/// exclusive schema, removed with `IntegralAcquisition::ReproducedStageB`), but
/// `digest()`'s schema-16-or-17 gate stays for historical trace-capture
/// replay compatibility, so this checks the surviving `17` arm directly on a
/// plan with the schema field overridden.
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
