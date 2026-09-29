use super::band_confirmation::{
    BandConfirmationPlan, ConfirmationDirectionalInput, DiscoveryObservationAtom,
    evaluate_band_confirmation, plan_band_confirmation,
};
use super::fixed_gain_amplitude_plan::{
    FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION, FIXED_GAIN_AMPLITUDE_TARGETS, FixedGainAmplitudeOrder,
    FixedGainAmplitudePlanError, derive_fixed_gain_amplitude_targets, plan_fixed_gain_amplitude,
    schedule_order_code,
};
use super::integral_plan::{
    AcceptanceSource, AcquisitionContext, BreakawayIntegralEvidence, ConfirmedAcquisitionDigest,
    IntegralPlanPhysics, IntegralSlotOrder, VelocityIntegralAcquisitionKind,
    bind_breakaway_integral_authority, plan_retained_breakaway_velocity_integral_response,
};
use super::integral_reporting::plan_recovery_flags;
use super::integral_reproduction::{
    AcceptancePoint, FixedGainAmplitudeProgress, IntegralPlanIdentity,
};
use super::reproduction::BUILD_FEATURES;
use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
};
use super::sweep_evidence::VelocityObservationClass;
use super::sweep_plan::{
    RequestedVelocitySource, VelocitySweepRequest, plan_proportional_velocity_sweep,
};
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::commissioning::outer::OuterMotionBudget;
use foci_tmc4671::pid::PidPGain;

const COUNTS_PER_REV: u32 = 4_000;
const PWM_HZ: u32 = 25_000;
const TORQUE_LIMIT: u16 = 2_816;

fn breakaway_request() -> VelocitySweepRequest {
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

fn breakaway_motion_budget() -> OuterMotionBudget {
    let request = breakaway_request();
    OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
        homing_speed_mrev_s: 0,
        max_accel_mrev_s2: 0,
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

fn breakaway_plan() -> super::integral_plan::VelocityIntegralPlan {
    let proportional = plan_proportional_velocity_sweep(
        breakaway_request(),
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let mut context = AcquisitionContext::from(proportional.execution_plan(proportional.digest()));
    context.channel = 0;
    context.plan_time_pwm_hz = PWM_HZ;
    context.plan_time_encoder_counts_per_rev = COUNTS_PER_REV;

    let confirmation_plan = breakaway_confirmation_plan(PidPGain::new(512).unwrap());
    let token = breakaway_token_for(confirmation_plan);
    plan_retained_breakaway_velocity_integral_response(
        token,
        confirmation_plan.nominated_p.raw(),
        context,
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: breakaway_intervals_for(confirmation_plan),
            confidence_q: 0,
            band_position_q: 0,
        },
        IntegralPlanPhysics {
            motion_budget: breakaway_motion_budget(),
            pwm_hz: PWM_HZ,
            encoder_counts_per_rev: COUNTS_PER_REV,
        },
    )
    .expect("the campaign's own Integral plan")
}

fn breakaway_point(plan: super::integral_plan::VelocityIntegralPlan) -> AcceptancePoint {
    AcceptancePoint {
        identity: IntegralPlanIdentity {
            channel: 0,
            build_features: BUILD_FEATURES,
            schema_revision: plan.schema_revision,
            plan_digest: plan.digest(),
        },
        selected_p: plan.fixed_p,
        selected_i: plan.positive_rungs()[0],
        target_velocity_rpm: plan.target_velocity_rpm,
        selected_response_intervals_q: plan.authority.directional_pooled_intervals,
        confidence_q: 0,
        band_position_q: 0,
        qualifier_bits: 0x0123,
        token: None,
        progress: FixedGainAmplitudeProgress::ReadyAscending,
    }
}

#[test]
fn amplitude_accepts_a_genuine_breakaway_authority_point() {
    let current = breakaway_plan();
    assert_eq!(
        current.schema_revision,
        super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION
    );
    let accepted = plan_fixed_gain_amplitude(
        breakaway_point(current),
        breakaway_request(),
        FixedGainAmplitudeOrder::Ascending,
        IntegralSlotOrder::ForwardFirst,
        current,
    )
    .expect("a genuine breakaway point carries amplitude acceptance authority");
    assert!(accepted.recovery_lower_rate_q.iter().all(|&rate| rate > 0));
    assert_eq!(
        accepted.acceptance_identity.schema_revision,
        super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION
    );
}

#[test]
fn amplitude_rejects_authority_point_with_selected_i_out_of_range() {
    let current = breakaway_plan();
    assert!(
        !current.positive_rungs().contains(&u16::MAX),
        "the ladder never contains u16::MAX"
    );
    let mut invalid = breakaway_point(current);
    invalid.selected_i = u16::MAX;

    assert_eq!(
        plan_fixed_gain_amplitude(
            invalid,
            breakaway_request(),
            FixedGainAmplitudeOrder::Ascending,
            IntegralSlotOrder::ForwardFirst,
            current,
        ),
        Err(FixedGainAmplitudePlanError::IdentityMismatch)
    );
}

#[test]
fn live_amplitude_geometry_and_orders_are_exact() {
    assert_eq!(FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION, 6);
    assert_eq!(FixedGainAmplitudeOrder::Ascending as u8, 1);
    assert_eq!(FixedGainAmplitudeOrder::Descending as u8, 2);
    assert_eq!(FIXED_GAIN_AMPLITUDE_TARGETS, 5);
    let current = breakaway_plan();
    let ascending = plan_fixed_gain_amplitude(
        breakaway_point(current),
        breakaway_request(),
        FixedGainAmplitudeOrder::Ascending,
        IntegralSlotOrder::ForwardFirst,
        current,
    )
    .unwrap();
    assert_eq!(ascending.target_rpm, [16, 33, 66, 132, 263]);
    let executed_mrev_s_milli = ascending
        .target_rpm
        .map(|rpm| (u64::try_from(rpm).unwrap() * 1_000_000).div_ceil(60));
    let stroke_travel_mrev_milli = ascending
        .target_rpm
        .map(|rpm| (u64::try_from(rpm).unwrap() * 512_000).div_ceil(60));
    assert_eq!(
        executed_mrev_s_milli,
        [266_667, 550_000, 1_100_000, 2_200_000, 4_383_334]
    );
    assert_eq!(
        stroke_travel_mrev_milli,
        [136_534, 281_600, 563_200, 1_126_400, 2_244_267]
    );
    assert!(
        stroke_travel_mrev_milli
            .into_iter()
            .all(|travel| travel <= 2_250_000)
    );
    assert_eq!(ascending.amplitude_count, 5);
    assert_eq!(ascending.expected_observations, 40);
    assert_eq!(ascending.family_size, 20);
    assert_eq!(ascending.recovery_slot_count, 5);
    assert_eq!(ascending.recovery_lower_rate_q, [725_648, 725_648]);
    assert_eq!(ascending.nominal_workflow_ms, 60_541);
    assert_eq!(ascending.maximum_workflow_ms, 66_456);
    assert_eq!(
        (
            super::combined_plan::PROPORTIONAL_PLAN_SCHEMA_REVISION,
            super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION,
            FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION,
        ),
        (14, 18, 6)
    );
    assert_eq!([160 + 23, 120 + 15, 40 + 5], [183, 135, 45]);
    assert_eq!((183 + 135 + 45) * 78, 28_314);

    let descending = plan_fixed_gain_amplitude(
        breakaway_point(current),
        breakaway_request(),
        FixedGainAmplitudeOrder::Descending,
        IntegralSlotOrder::ForwardFirst,
        current,
    )
    .unwrap();
    assert_eq!(descending.target_rpm, [263, 132, 66, 33, 16]);
    assert_eq!(
        descending
            .target_rpm
            .map(|rpm| (u64::try_from(rpm).unwrap() * 512_000).div_ceil(60)),
        [2_244_267, 1_126_400, 563_200, 281_600, 136_534]
    );
    assert_ne!(ascending.digest(), descending.digest());
}

#[test]
fn historical_schema_three_keeps_the_stabilized_recovery_digest_input() {
    let current = breakaway_plan();
    let mut historical = plan_fixed_gain_amplitude(
        breakaway_point(current),
        breakaway_request(),
        FixedGainAmplitudeOrder::Ascending,
        IntegralSlotOrder::ForwardFirst,
        current,
    )
    .unwrap();
    historical.schema_revision = 3;

    assert_eq!(historical.digest(), 0xa0c7_1872_d0c7_a971);
}

#[test]
fn target_derivation_rounds_half_up_and_rejects_unsupported_geometry() {
    assert_eq!(
        derive_fixed_gain_amplitude_targets(263),
        Ok([16, 33, 66, 132, 263])
    );
    assert_eq!(
        derive_fixed_gain_amplitude_targets(257),
        Ok([16, 32, 64, 129, 257])
    );
    assert_eq!(
        derive_fixed_gain_amplitude_targets(7),
        Err(FixedGainAmplitudePlanError::UnsupportedTargets)
    );
    assert_eq!(
        derive_fixed_gain_amplitude_targets(8),
        Err(FixedGainAmplitudePlanError::UnsupportedTargets)
    );
    assert_eq!(
        derive_fixed_gain_amplitude_targets(i32::MAX),
        Err(FixedGainAmplitudePlanError::Arithmetic)
    );
}

#[test]
fn compact_execution_plan_carries_fixed_i_and_literal_targets() {
    let current = breakaway_plan();
    let ascending = plan_fixed_gain_amplitude(
        breakaway_point(current),
        breakaway_request(),
        FixedGainAmplitudeOrder::Ascending,
        IntegralSlotOrder::ForwardFirst,
        current,
    )
    .unwrap();
    let execution = ascending.execution_plan();

    assert_eq!(
        execution.acquisition_kind,
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
    );
    assert_eq!(execution.acceptance_source, AcceptanceSource::NonBreakaway);
    assert!(!execution.records_target_terminus());
    assert_eq!(execution.i_start, 1);
    assert_eq!(execution.amplitude_targets_rpm, [16, 33, 66, 132, 263]);
    assert_eq!(execution.positive_rung_count, 5);
    assert_eq!(execution.family_size, 20);
    assert_eq!(execution.target_velocity_rpm, 263);
    assert_eq!(execution.planned_velocity_mrev_s, 6_666);
    assert_eq!(execution.recovery_lower_rate_q, [725_648, 725_648]);
    assert_eq!(
        execution.acceptance_plan_digest,
        ascending.acceptance_identity.plan_digest
    );
    assert_eq!(execution.qualifier_bits, ascending.qualifier_bits);
    assert_eq!(execution.selected_response_intervals_q, [[0, 0], [0, 0]]);
    assert_eq!(core::mem::size_of_val(&execution), 128);
}

#[test]
fn amplitude_plan_rejects_identity_and_request_drift() {
    let current = breakaway_plan();
    let mut stale = breakaway_point(current);
    stale.identity.plan_digest ^= 1;
    assert_eq!(
        plan_fixed_gain_amplitude(
            stale,
            breakaway_request(),
            FixedGainAmplitudeOrder::Ascending,
            IntegralSlotOrder::ForwardFirst,
            current
        ),
        Err(FixedGainAmplitudePlanError::IdentityMismatch)
    );

    let mut changed_request = breakaway_request();
    changed_request.max_duration_ms += 1;
    assert_eq!(
        plan_fixed_gain_amplitude(
            breakaway_point(current),
            changed_request,
            FixedGainAmplitudeOrder::Ascending,
            IntegralSlotOrder::ForwardFirst,
            current,
        ),
        Err(FixedGainAmplitudePlanError::IdentityMismatch)
    );
}

#[test]
fn amplitude_plan_rejects_wrong_sided_recovery_authority() {
    let current = breakaway_plan();
    let genuine = breakaway_point(current).selected_response_intervals_q;
    for intervals in [
        [[0, genuine[0][1]], genuine[1]],
        [genuine[0], [genuine[1][0], 0]],
    ] {
        let mut invalid = breakaway_point(current);
        invalid.selected_response_intervals_q = intervals;
        assert_eq!(
            plan_fixed_gain_amplitude(
                invalid,
                breakaway_request(),
                FixedGainAmplitudeOrder::Ascending,
                IntegralSlotOrder::ForwardFirst,
                current,
            ),
            Err(FixedGainAmplitudePlanError::IdentityMismatch)
        );
    }
}

#[test]
fn amplitude_digest_binds_every_explicit_plan_dimension() {
    let current = breakaway_plan();
    let base = plan_fixed_gain_amplitude(
        breakaway_point(current),
        breakaway_request(),
        FixedGainAmplitudeOrder::Ascending,
        IntegralSlotOrder::ForwardFirst,
        current,
    )
    .unwrap();
    let mutate = |change: fn(&mut super::fixed_gain_amplitude_plan::FixedGainAmplitudePlan)| {
        let mut changed = base;
        change(&mut changed);
        assert_ne!(base.digest(), changed.digest());
    };
    mutate(|plan| plan.selected_p += 1);
    mutate(|plan| plan.selected_i += 1);
    mutate(|plan| plan.target_rpm[0] += 1);
    mutate(|plan| plan.pwm_hz += 1);
    mutate(|plan| plan.encoder_counts_per_rev += 1);
    mutate(|plan| plan.moving_stroke_us += 1);
    mutate(|plan| plan.zero_settle_us += 1);
    mutate(|plan| plan.analysis_budget_us += 1);
    mutate(|plan| plan.family_size += 1);
    mutate(|plan| plan.hard_torque_limit += 1);
    mutate(|plan| plan.usable_torque_limit += 1);
    mutate(|plan| plan.recovery_origin_band_counts += 1);
    mutate(|plan| plan.recovery_lower_rate_q[0] += 1);
    mutate(|plan| plan.recovery_lower_rate_q[1] += 1);
    mutate(|plan| plan.recovery_maximum_slot_us += 1);
    mutate(|plan| plan.nominal_workflow_ms += 1);
    mutate(|plan| plan.maximum_workflow_ms += 1);
    mutate(|plan| plan.schema_revision += 1);
}

#[test]
fn schedule_order_code_preserves_the_historical_byte() {
    for order in [
        FixedGainAmplitudeOrder::Ascending,
        FixedGainAmplitudeOrder::Descending,
    ] {
        assert_eq!(
            schedule_order_code(order, IntegralSlotOrder::ForwardFirst),
            order as u8,
            "forward-first must not change the wire byte"
        );
    }
}

#[test]
fn forward_first_leaves_the_integral_plan_digest_untouched() {
    let plan = breakaway_plan();
    let mut explicit = plan;
    explicit.slot_order = IntegralSlotOrder::ForwardFirst;

    assert_eq!(plan.digest(), explicit.digest());
    assert_eq!(
        plan.schema_revision,
        super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION,
        "the schema must not move for a mirrored run"
    );
}

#[test]
fn plan_flags_leave_the_reserved_slot_order_bits_zero() {
    let plan = breakaway_plan();
    let older_bits = plan_recovery_flags(&plan) & 0b011;

    assert_eq!(plan.slot_order, IntegralSlotOrder::ForwardFirst);
    let flags = plan_recovery_flags(&plan);
    assert_eq!(flags & 0b1100, 0, "forward-first must leave bits 2-3 zero");
    assert_eq!(flags & 0b011, older_bits, "older bits must be undisturbed");
}
