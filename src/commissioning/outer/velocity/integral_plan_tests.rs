use super::band_confirmation::{
    BandConfirmationAcceptance, BandConfirmationFailureCause, ConfirmationDirectionalInput,
    DiscoveryObservationAtom, TargetBandConfirmationInconclusive, evaluate_band_confirmation,
    plan_band_confirmation,
};
use super::breakaway_plan::{
    AdditiveDiscoveryRequest, AdditiveLadderPlanError, BreakawayProbePlan, GridFloorOrigin,
    plan_additive_discovery_ladder, plan_breakaway_probe, resolve_grid_floor,
};
use super::integral_plan::{
    BreakawayAuthorityBindingError, CommissioningWorkflowPlan, IntegralSlotOrder,
    STAGE_C_EVIDENCE_SCHEMA_REVISION, STAGE_C_MAX_POSITIVE_RUNGS, STAGE_C_MIN_POSITIVE_I,
    STAGE_C_OBSERVATIONS_PER_RUNG, StageCAuthority, StageCPlanShape, VelocityIntegralPlanError,
    bind_breakaway_stage_c_authority, breakaway_confirmation_slots_us,
    breakaway_discovery_slots_us, breakaway_probe_slots_us, breakaway_workflow_envelope_ms,
    breakaway_workflow_envelope_us, build_integral_ladder, confidence_family_size,
    integral_workflow_maximum_ms, plan_velocity_integral_response, recovery_quantization_exposed,
    response_horizon, velocity_i_tau_us,
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

fn authority() -> StageCAuthority {
    StageCAuthority {
        channel: 0,
        stage_b_plan_digest: 0x0123_4567_89ab_cdef,
        directional_memberships: [0x0007_0000, 0x0007_0000],
        stage_b_joint_membership: 0x0007_0000,
        joint_membership: 0x0007_0000,
        directional_validity: [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2],
        reduced_margin: false,
        directional_pooled_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
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

/// Build the probe plan and resolve its floor/breakaway pair around gain 256.
fn breakaway_probe_and_floor() -> (BreakawayProbePlan, PidPGain, PidPGain) {
    let p_start = breakaway_gain(1);
    let p_top = breakaway_gain(1_024);
    let probe_plan = plan_breakaway_probe(p_start, p_top).unwrap();
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
    assert_eq!(STAGE_C_EVIDENCE_SCHEMA_REVISION, 7);
    assert_eq!(STAGE_C_MIN_POSITIVE_I, 1);
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
    assert_eq!(STAGE_C_MAX_POSITIVE_RUNGS, 30);
    assert_eq!(STAGE_C_OBSERVATIONS_PER_RUNG, 8);
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
fn stage_c_plan_freezes_geometry_and_conservative_duration() {
    let stage_b = stage_b_plan();
    let mut authority = authority();
    authority.stage_b_plan_digest = stage_b.digest();
    let plan = plan_velocity_integral_response(
        authority,
        stage_b.execution_plan(stage_b.digest()),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();

    assert_eq!(plan.schema_revision, STAGE_C_EVIDENCE_SCHEMA_REVISION);
    assert_eq!(plan.final_p, 512);
    assert_eq!(
        plan.positive_rungs(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024]
    );
    assert_eq!(plan.family_size, 56);
    assert_eq!(plan.total_rung_count, 14);
    assert_eq!(plan.expected_observations, 112);
    assert_eq!(plan.moving_stroke_us, 512_000);
    assert_eq!(plan.zero_settle_us, 500_000);
    assert_eq!(plan.analysis_budget_us, 236_000);
    assert_eq!(plan.recovery_origin_band_counts, 1_000);
    assert_eq!(plan.recovery_nominal_slot_us, 1_816_958);
    assert_eq!(plan.recovery_maximum_slot_us, 3_000_000);
    assert_eq!(plan.recovery_slot_count, 14);
    assert_eq!(plan.nominal_workflow_ms, 165_950);
    assert_eq!(plan.maximum_workflow_ms, 182_512);
    let base_us = u32::from(plan.total_rung_count)
        * STAGE_C_OBSERVATIONS_PER_RUNG
        * (MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US);
    let recovery_us = u32::from(plan.total_rung_count) * plan.recovery_nominal_slot_us;
    assert_eq!(
        plan.nominal_workflow_ms,
        (base_us + recovery_us + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US).div_ceil(1_000)
    );
}

#[test]
fn workflow_shape_does_not_change_either_stage_identity() {
    let stage_b = stage_b_plan();
    let stage_b_digest = stage_b.digest();
    let mut authority = authority();
    authority.stage_b_plan_digest = stage_b_digest;
    let stage_c = plan_velocity_integral_response(
        authority,
        stage_b.execution_plan(stage_b_digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();
    let stage_c_digest = stage_c.digest();

    let candidate =
        CommissioningWorkflowPlan::new(7, StageCPlanShape::StageB, stage_b.maximum_workflow_ms, 0)
            .unwrap();
    let composite = CommissioningWorkflowPlan::new(
        8,
        StageCPlanShape::StageBThenStageC,
        stage_b.maximum_workflow_ms,
        stage_c.maximum_workflow_ms,
    )
    .unwrap();
    let resume = CommissioningWorkflowPlan::new(
        9,
        StageCPlanShape::StageCResume,
        0,
        stage_c.maximum_workflow_ms,
    )
    .unwrap();

    assert_eq!(candidate.maximum_workflow_ms, 298_632);
    assert_eq!(composite.maximum_workflow_ms, 481_144);
    assert_eq!(resume.maximum_workflow_ms, 182_512);
    assert_ne!(candidate.digest(), composite.digest());
    assert_ne!(composite.digest(), resume.digest());
    assert_eq!(StageCPlanShape::StageB as u8, 0);
    assert_eq!(StageCPlanShape::StageBThenStageC as u8, 1);
    assert_eq!(StageCPlanShape::StageCResume as u8, 2);
    assert_eq!(stage_b.digest(), stage_b_digest);
    assert_eq!(stage_c.digest(), stage_c_digest);
    assert_eq!(stage_c.execution_plan().plan_digest, stage_c_digest);
}

#[test]
fn invalid_geometry_and_missing_authority_reject_before_motion() {
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

    let stage_b = stage_b_plan();
    let digest = stage_b.digest();
    let mut missing = authority();
    missing.stage_b_plan_digest = digest;
    missing.stage_b_joint_membership = 0;
    missing.joint_membership = 0;
    assert_eq!(
        plan_velocity_integral_response(
            missing,
            stage_b.execution_plan(digest),
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MissingAuthority)
    );
    assert_eq!(
        plan_velocity_integral_response(
            authority(),
            super::sweep_plan::VelocitySweepExecutionPlan::EMPTY,
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MissingStageBPlan)
    );
    let mut mismatched = authority();
    mismatched.stage_b_plan_digest = digest.wrapping_add(1);
    assert_eq!(
        plan_velocity_integral_response(
            mismatched,
            stage_b.execution_plan(digest),
            motion_budget(),
            25_000,
            4_000,
        ),
        Err(VelocityIntegralPlanError::MismatchedAuthority)
    );
}

#[test]
fn stage_c_accepts_local_authority_without_model() {
    let stage_b = stage_b_plan();
    let digest = stage_b.digest();
    let mut local = authority();
    local.stage_b_plan_digest = digest;
    local.stage_b_joint_membership = 0x0003_0000;
    local.joint_membership = 0x0003_0000;
    local.directional_memberships = [0x0003_0000, 0];
    local.directional_pooled_intervals = [[0, 0]; 2];
    local.directional_validity = [
        StageBDirectionalValidity::Fragment,
        StageBDirectionalValidity::NoRegion,
    ];
    local.reduced_margin = true;

    let plan = plan_velocity_integral_response(
        local,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .expect("joint operability authorizes Stage C without a cross-P model");

    assert!(!plan.authority.model_interval_available(0));
    assert!(!plan.authority.model_interval_available(1));
    assert!(plan.authority.reduced_margin);
    assert_eq!(
        plan.authority.directional_validity,
        [
            StageBDirectionalValidity::Fragment,
            StageBDirectionalValidity::NoRegion,
        ]
    );
}

#[test]
fn stage_c_model_interval_availability_is_typed_and_hashed() {
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

    let stage_b = stage_b_plan();
    let digest = stage_b.digest();
    let mut baseline = authority();
    baseline.stage_b_plan_digest = digest;
    let baseline = plan_velocity_integral_response(
        baseline,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();

    let mut transferred = authority();
    transferred.stage_b_plan_digest = digest;
    transferred.directional_memberships[0] = 0x001c_0000;
    let transferred = plan_velocity_integral_response(
        transferred,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();
    assert_ne!(baseline.digest(), transferred.digest());

    let mut reduced = authority();
    reduced.stage_b_plan_digest = digest;
    reduced.stage_b_joint_membership = 0x0003_0000;
    reduced.joint_membership = 0x0003_0000;
    let reduced = plan_velocity_integral_response(
        reduced,
        stage_b.execution_plan(digest),
        motion_budget(),
        25_000,
        4_000,
    )
    .unwrap();
    assert_ne!(baseline.digest(), reduced.digest());
}

const SLOT_COUNT: u8 = STAGE_C_OBSERVATIONS_PER_RUNG as u8;

/// The slot index and the travel direction must round-trip in both directions.
///
/// `rung_classes` and `rung_intervals` are written by raw slot index and read
/// back by (observation, direction) in `acceptance_amplitude_report`. If the two
/// mappings ever disagree, forward evidence is filed as reverse and the run is
/// silently wrong rather than failing. This pins them together.
#[test]
fn slot_and_direction_mappings_are_mutual_inverses() {
    for order in [
        IntegralSlotOrder::ForwardFirst,
        IntegralSlotOrder::ReverseFirst,
    ] {
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
    for order in [
        IntegralSlotOrder::ForwardFirst,
        IntegralSlotOrder::ReverseFirst,
    ] {
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

/// Mirroring swaps which end of the span each direction's tail lands on.
#[test]
fn reverse_first_inverts_every_slot_direction() {
    let forward_first = IntegralSlotOrder::ForwardFirst;
    let reverse_first = IntegralSlotOrder::ReverseFirst;
    for slot in 0..SLOT_COUNT {
        assert_ne!(
            forward_first.direction_index(slot),
            reverse_first.direction_index(slot)
        );
        assert_eq!(
            forward_first.signed_direction(slot),
            -reverse_first.signed_direction(slot)
        );
        // The observation an ordinal belongs to does not move; only its direction.
        assert_eq!(
            forward_first.observation_index(slot),
            reverse_first.observation_index(slot)
        );
    }
}

/// The paired order runs F,R,R,F,F,R,R,F so each direction takes both roles.
///
/// Under strict alternation a stroke's direction, whether it travels away from or
/// back to the origin, and the direction of the stroke before it are all locked
/// together. Pairing inverts the outbound/preceded-by relation relative to the
/// alternating orders, which is what separates them.
#[test]
fn paired_order_gives_each_direction_both_outbound_and_inbound_roles() {
    let order = IntegralSlotOrder::PairedOutAndBack;
    let expected = [0_usize, 1, 1, 0, 0, 1, 1, 0];
    for slot in 0..SLOT_COUNT {
        assert_eq!(
            order.direction_index(slot),
            expected[usize::from(slot)],
            "slot {slot}"
        );
    }

    // Even slots leave the origin, odd slots return to it, in every order. Under
    // the alternating orders an outbound stroke always follows an
    // opposite-direction stroke; under pairing it follows a same-direction one.
    for slot in (0..SLOT_COUNT).step_by(2) {
        let previous = if slot == 0 { SLOT_COUNT - 1 } else { slot - 1 };
        assert_eq!(
            order.direction_index(slot),
            order.direction_index(previous),
            "paired outbound slot {slot} must follow its own direction"
        );
        assert_ne!(
            IntegralSlotOrder::ForwardFirst.direction_index(slot),
            IntegralSlotOrder::ForwardFirst.direction_index(previous),
            "alternating outbound slot {slot} must follow the opposite direction"
        );
    }

    // Both excursion ends are used, so absolute position varies within a role.
    assert_eq!(order.direction_index(0), 0, "slot 0 leaves toward forward");
    assert_eq!(order.direction_index(2), 1, "slot 2 leaves toward reverse");
}

// --- StageCPlanShape::BreakawaySeededPThenI ---------------------------------

#[test]
fn breakaway_seeded_p_then_i_appends_without_renumbering_the_existing_shapes() {
    assert_eq!(StageCPlanShape::StageB as u8, 0);
    assert_eq!(StageCPlanShape::StageBThenStageC as u8, 1);
    assert_eq!(StageCPlanShape::StageCResume as u8, 2);
    assert_eq!(StageCPlanShape::CombinedPThenI as u8, 3);
    assert_eq!(StageCPlanShape::AcceptanceMatrixAscending as u8, 4);
    assert_eq!(StageCPlanShape::AcceptanceMatrixDescending as u8, 5);
    assert_eq!(StageCPlanShape::BreakawaySeededPThenI as u8, 6);

    // The breakaway shape has its own dedicated constructor, matching the
    // existing precedent set by `CombinedPThenI` and the acceptance-matrix
    // shapes: it is not constructible through the generic two-duration `new`.
    assert_eq!(
        CommissioningWorkflowPlan::new(1, StageCPlanShape::BreakawaySeededPThenI, 1, 1),
        Err(VelocityIntegralPlanError::InvalidRequest)
    );
}

#[test]
fn new_breakaway_reserves_the_command_level_envelope_before_probe_motion() {
    let envelope = CommissioningWorkflowPlan::new_breakaway(1, 400_000, 450_000).unwrap();
    assert_eq!(envelope.shape, StageCPlanShape::BreakawaySeededPThenI);
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
fn worst_case_probe_is_sixteen_primitives_at_19_968_seconds() {
    let primitive_us = MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US;
    assert_eq!(primitive_us, 1_248_000);
    assert_eq!(breakaway_probe_slots_us(), Ok(19_968_000));
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
        Ok(319_488_000)
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
    let stage_c_maximum_us =
        u64::from(integral_workflow_maximum_ms(25_000, 3_000).unwrap()) * 1_000;

    let total_us = breakaway_workflow_envelope_us(
        recovery_slot_count,
        recovery_maximum_slot_us,
        stage_c_maximum_us,
    )
    .unwrap();
    let expected_us = breakaway_probe_slots_us().unwrap()
        + u64::from(recovery_slot_count) * u64::from(recovery_maximum_slot_us)
        + breakaway_discovery_slots_us(MAX_SWEEP_RUNGS as u8).unwrap()
        + breakaway_confirmation_slots_us().unwrap()
        + stage_c_maximum_us;
    assert_eq!(total_us, expected_us);

    let total_ms = breakaway_workflow_envelope_ms(
        recovery_slot_count,
        recovery_maximum_slot_us,
        stage_c_maximum_us,
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
    // `stage_c_maximum_us` alone large enough to overflow the running `u64`
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

    let candidate_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, candidate_p).unwrap();
    assert_eq!(
        confirmation_plan.discovery_plan_digest,
        discovery_plan.plan_digest
    );

    let (forward, reverse) = accepted_confirmation_evidence();
    let acceptance = evaluate_band_confirmation(candidate_p, forward, reverse)
        .expect("rung-18-class evidence confirms within the 70-80% band");
    assert_eq!(acceptance.candidate_p, candidate_p);

    let bound = bind_breakaway_stage_c_authority(
        confirmation_plan.plan_digest,
        confirmation_plan,
        Ok(acceptance),
    )
    .unwrap();

    assert_eq!(bound.get(), confirmation_plan.plan_digest);
    assert_ne!(bound.get(), discovery_plan.plan_digest);
    assert_ne!(bound.get(), probe_plan.plan_digest);
    assert_ne!(discovery_plan.plan_digest, probe_plan.plan_digest);

    // The token is the only way to fill a breakaway authority's
    // `stage_b_plan_digest`: `StageCAuthority::new_breakaway` takes a
    // `ConfirmedAcquisitionDigest`, not a bare `u64`, so this call could not
    // have compiled with a hand-picked digest, only with the value the gate
    // above actually validated.
    let authority = StageCAuthority::new_breakaway(0, bound);
    assert_eq!(authority.stage_b_plan_digest, confirmation_plan.plan_digest);
    assert_eq!(authority.stage_b_joint_membership, 0);
    assert_eq!(authority.joint_membership, 0);
}

// `ConfirmedAcquisitionDigest`'s field is private to `integral_plan.rs`.
// This file, `integral_plan_tests.rs`, is a *sibling* module declared in
// `mod.rs` (`pub mod integral_plan; #[cfg(test)] mod integral_plan_tests;`),
// not a descendant of `integral_plan`, so it has no access to that private
// field: neither `ConfirmedAcquisitionDigest(42)` nor
// `StageCAuthority::new_breakaway(0, ConfirmedAcquisitionDigest(42))` would
// compile in this file (the compiler's own suggestion when the digest chain
// test above was still returning a bare `u64` was literally to wrap it in
// `ConfirmedAcquisitionDigest(..)` here, followed by a note that doing so
// requires changing the field's privacy -- i.e. it does not compile as
// written). There is no `compile_fail` doctest for this: the type is
// `pub(crate)`, invisible outside the crate entirely, so an external doctest
// binary can't even name it. The guarantee is structural, checked by every
// build of this crate: every call site in every module other than
// `integral_plan.rs` must obtain a `ConfirmedAcquisitionDigest` from
// `bind_breakaway_stage_c_authority`, with no other route to one.

#[test]
fn stage_c_authority_rejects_a_discovery_or_probe_digest() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();
    let discovery_plan = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ))
    .unwrap();
    let candidate_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, candidate_p).unwrap();
    let (forward, reverse) = accepted_confirmation_evidence();
    let acceptance = evaluate_band_confirmation(candidate_p, forward, reverse).unwrap();

    // The discovery digest is offered where the confirmation digest belongs.
    assert_eq!(
        bind_breakaway_stage_c_authority(
            discovery_plan.plan_digest,
            confirmation_plan,
            Ok(acceptance),
        ),
        Err(BreakawayAuthorityBindingError::AcquisitionDigestMismatch)
    );
    // The probe digest is rejected the same way.
    assert_eq!(
        bind_breakaway_stage_c_authority(probe_plan.plan_digest, confirmation_plan, Ok(acceptance)),
        Err(BreakawayAuthorityBindingError::AcquisitionDigestMismatch)
    );
}

#[test]
fn stage_c_authority_rejects_a_failed_confirmation_even_with_a_matching_digest() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();
    let discovery_plan = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ))
    .unwrap();
    let candidate_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, candidate_p).unwrap();

    // The confirmation plan's own digest is well formed and matches exactly,
    // but the held-out block did not accept: Stage C must still be refused.
    let failure =
        TargetBandConfirmationInconclusive(BandConfirmationFailureCause::ResponseLocation);
    assert_eq!(
        bind_breakaway_stage_c_authority(
            confirmation_plan.plan_digest,
            confirmation_plan,
            Err(failure)
        ),
        Err(BreakawayAuthorityBindingError::ConfirmationNotAccepted)
    );
}

#[test]
fn stage_c_authority_rejects_an_acceptance_for_a_different_candidate() {
    let (probe_plan, floor, breakaway) = breakaway_probe_and_floor();
    let discovery_plan = plan_additive_discovery_ladder(breakaway_discovery_request(
        floor,
        breakaway,
        probe_plan.plan_digest,
        320_000,
    ))
    .unwrap();
    let candidate_p = breakaway_gain(724);
    let confirmation_plan =
        plan_band_confirmation(discovery_plan.plan_digest, candidate_p).unwrap();

    let mismatched_acceptance = BandConfirmationAcceptance {
        candidate_p: breakaway_gain(512),
    };
    assert_eq!(
        bind_breakaway_stage_c_authority(
            confirmation_plan.plan_digest,
            confirmation_plan,
            Ok(mismatched_acceptance),
        ),
        Err(BreakawayAuthorityBindingError::AcceptanceCandidateMismatch)
    );
}
