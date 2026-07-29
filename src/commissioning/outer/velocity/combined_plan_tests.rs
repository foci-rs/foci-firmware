use super::combined_plan::{
    COMBINED_STAGE_B_SCHEMA_REVISION, COMBINED_STAGE_C_SCHEMA_REVISION, CombinedPlanError,
    CombinedRateInterval, CombinedTargetStatus, FIRST_COMBINED_STAGE_C_SCHEMA_REVISION,
    build_combined_integral_ladder, combined_workflow_geometry, integral_deficit_reaches_target,
    integral_response_reaches_target, proportional_response_in_initial_band,
    select_proportional_response,
};
use super::integral_plan::{
    CommissioningWorkflowPlan, STAGE_C_EVIDENCE_SCHEMA_REVISION, StageCPlanShape,
    build_integral_ladder, plan_combined_velocity_integral_response,
};
use super::operating_region::{BoundRounding, CompactSignedRate};
use super::sweep_consensus::DirectionalConsensusClass;
use super::sweep_evidence::CompactDirectionalRungReport;
use super::sweep_plan::{
    MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US, RequestedVelocitySource, VelocitySweepRequest,
    plan_combined_velocity_sweep, plan_historical_combined_velocity_sweep,
};
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_I, OuterMotionBudget};
use foci_tmc4671::pid::PidPGain;

fn response_report(p: u16, low_q: i64, high_q: i64) -> CompactDirectionalRungReport {
    CompactDirectionalRungReport {
        p: PidPGain::new(p).unwrap(),
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0; 2]; 2],
        pooled: None,
        settled_rate_bounds_q: [
            CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
            CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
        ],
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
        operable_count: 4,
        flags: 0,
    }
}

fn motion_budget(request: VelocitySweepRequest) -> OuterMotionBudget {
    OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
    }
}

#[test]
fn corrected_y_intervals_select_p1024_without_model_membership() {
    let candidate_p724 = [
        CombinedRateInterval::new(11_562_496, 11_719_168).unwrap(),
        CombinedRateInterval::new(-11_218_944, -11_187_200).unwrap(),
    ];
    let candidate_p1024 = [
        CombinedRateInterval::new(13_359_104, 13_492_224).unwrap(),
        CombinedRateInterval::new(-12_968_960, -12_921_856).unwrap(),
    ];
    let reproduction_p724 = [
        CombinedRateInterval::new(11_593_728, 11_750_400).unwrap(),
        CombinedRateInterval::new(-11_359_744, -11_328_000).unwrap(),
    ];
    let reproduction_p1024 = [
        CombinedRateInterval::new(13_390_336, 13_539_328).unwrap(),
        CombinedRateInterval::new(-13_000_192, -12_937_216).unwrap(),
    ];

    assert!(!proportional_response_in_initial_band(264, 4_000, candidate_p724).unwrap());
    assert!(proportional_response_in_initial_band(264, 4_000, candidate_p1024).unwrap());
    assert!(!proportional_response_in_initial_band(264, 4_000, reproduction_p724).unwrap());
    assert!(proportional_response_in_initial_band(264, 4_000, reproduction_p1024).unwrap());
    assert!(!proportional_response_in_initial_band(263, 4_000, reproduction_p724).unwrap());
    assert!(proportional_response_in_initial_band(263, 4_000, reproduction_p1024).unwrap());
}

#[test]
fn response_boundaries_are_closed_signed_and_checked() {
    let target_low = 12_320_000;
    let target_high = 14_080_000;
    let touching = [
        CombinedRateInterval::new(target_low, target_high).unwrap(),
        CombinedRateInterval::new(-target_high, -target_low).unwrap(),
    ];
    assert!(proportional_response_in_initial_band(264, 4_000, touching).unwrap());

    let below = [
        CombinedRateInterval::new(target_low - 1, target_high).unwrap(),
        touching[1],
    ];
    assert!(!proportional_response_in_initial_band(264, 4_000, below).unwrap());

    let wrong_reverse_sign = [
        touching[0],
        CombinedRateInterval::new(1, target_low).unwrap(),
    ];
    assert_eq!(
        proportional_response_in_initial_band(264, 4_000, wrong_reverse_sign),
        Err(CombinedPlanError::InvalidInterval)
    );
    assert_eq!(
        CombinedRateInterval::new(1, 0),
        Err(CombinedPlanError::InvalidInterval)
    );
}

#[test]
fn integral_target_uses_the_lower_response_endpoint_and_preserves_overshoot() {
    let exact_target = [
        CombinedRateInterval::new(17_248_000, 17_700_000).unwrap(),
        CombinedRateInterval::new(-18_000_000, -17_248_000).unwrap(),
    ];
    assert!(integral_response_reaches_target(264, 4_000, exact_target).unwrap());

    let below = [
        CombinedRateInterval::new(17_247_999, 17_700_000).unwrap(),
        exact_target[1],
    ];
    assert!(!integral_response_reaches_target(264, 4_000, below).unwrap());

    let overflow = [
        exact_target[0],
        CombinedRateInterval::new(i64::MIN, -17_248_000).unwrap(),
    ];
    assert_eq!(
        integral_response_reaches_target(264, 4_000, overflow),
        Err(CombinedPlanError::Arithmetic)
    );
}

#[test]
fn signed_deficit_target_uses_the_conservative_directional_endpoints() {
    let below = [
        CombinedRateInterval::new(300_000, 400_000).unwrap(),
        CombinedRateInterval::new(-400_000, -300_000).unwrap(),
    ];
    assert!(!integral_deficit_reaches_target(264, 4_000, below).unwrap());

    let reached = [
        CombinedRateInterval::new(-100_000, 340_000).unwrap(),
        CombinedRateInterval::new(-340_000, 100_000).unwrap(),
    ];
    assert!(integral_deficit_reaches_target(264, 4_000, reached).unwrap());
}

#[test]
fn combined_geometry_appends_the_exact_cap_without_changing_history() {
    assert_eq!(COMBINED_STAGE_B_SCHEMA_REVISION, 13);
    assert_eq!(COMBINED_STAGE_C_SCHEMA_REVISION, 11);
    assert_eq!(STAGE_C_EVIDENCE_SCHEMA_REVISION, 7);
    assert_eq!(StageCPlanShape::CombinedPThenI as u8, 3);

    let historical = build_integral_ladder(
        25_000,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )
    .unwrap();
    assert_eq!(historical.rungs.last(), Some(&1_024));

    let combined = build_combined_integral_ladder(
        25_000,
        MOVING_STROKE_US,
        MIN_SETTLE_CAPTURE_US,
        OUTER_MAX_VELOCITY_I,
    )
    .unwrap();
    assert_eq!(combined.i_top_cap, 1_310);
    assert_eq!(
        combined.rungs.as_slice(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024, 1_310]
    );
}

#[test]
fn combined_workflow_uses_exact_unrounded_phase_durations() {
    let request = VelocitySweepRequest {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    };
    let stage_b = plan_combined_velocity_sweep(
        request,
        4_000,
        2_816,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let mut historical_stage_b = stage_b;
    historical_stage_b.schema_revision = 10;
    historical_stage_b.recovery_nominal_slot_us = 1_816_958;
    let historical = combined_workflow_geometry(historical_stage_b, 25_000).unwrap();
    let geometry = combined_workflow_geometry(stage_b, 25_000).unwrap();

    assert_eq!(historical.combined_nominal_ms, 451_573);
    assert_eq!(historical.combined_maximum_ms, 496_528);
    assert_eq!(geometry.positive_i_rungs, 13);
    assert_eq!(geometry.total_i_rungs, 15);
    assert_eq!(geometry.i_observations, 120);
    assert_eq!(geometry.i_family_size, 60);
    assert_eq!(geometry.i_recovery_slots, 15);
    assert_eq!(geometry.i_nominal_ms, 187_651);
    assert_eq!(geometry.i_maximum_ms, 197_896);
    assert_eq!(geometry.combined_nominal_ms, 470_573);
    assert_eq!(geometry.combined_maximum_ms, 496_528);

    let workflow = CommissioningWorkflowPlan::new_combined(
        17,
        geometry.combined_nominal_ms,
        geometry.combined_maximum_ms,
    )
    .unwrap();
    assert_eq!(workflow.shape, StageCPlanShape::CombinedPThenI);
    assert_eq!(workflow.nominal_workflow_ms, 470_573);
    assert_eq!(workflow.maximum_workflow_ms, 496_528);
}

#[test]
fn combined_stage_c_plan_uses_only_the_live_selected_response() {
    let request = VelocitySweepRequest {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    };
    let stage_b = plan_combined_velocity_sweep(
        request,
        4_000,
        2_816,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let historical_stage_b = plan_historical_combined_velocity_sweep(
        request,
        4_000,
        2_816,
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let selected = select_proportional_response(
        response_report(1_024, 13_359_104, 13_492_224),
        response_report(1_024, -12_968_960, -12_921_856),
        264,
        4_000,
    )
    .unwrap()
    .unwrap();

    let plan = plan_combined_velocity_integral_response(
        selected,
        stage_b.execution_plan(stage_b.digest()),
        motion_budget(request),
        25_000,
        4_000,
    )
    .unwrap();

    assert_eq!(plan.schema_revision, COMBINED_STAGE_C_SCHEMA_REVISION);
    assert_eq!(plan.final_p, 1_024);
    assert_eq!(plan.combined_selected_response, Some(selected));
    assert_eq!(
        plan.execution_plan().selected_response_intervals_q,
        [[13_359_104, 13_492_224], [-12_968_960, -12_921_856]]
    );
    #[cfg(feature = "trace-velocity-sweep")]
    {
        let trace = super::integral_reporting::encode_velocity_integral_plan_trace(7, plan);
        for endpoint in [13_359_104_i32, 13_492_224, -12_968_960, -12_921_856] {
            assert!(
                trace[0]
                    .as_bytes()
                    .windows(4)
                    .any(|word| word == endpoint.to_le_bytes())
            );
        }
        assert!(trace[1].as_bytes().windows(4).any(|word| {
            word == super::integral_plan::INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US.to_le_bytes()
        }));
    }
    let mut historical = plan;
    historical.schema_revision = FIRST_COMBINED_STAGE_C_SCHEMA_REVISION;
    historical.authority.stage_b_plan_digest = historical_stage_b.digest();
    historical.target_velocity_rpm = historical_stage_b.target_velocity_rpm.raw();
    historical.nominal_workflow_ms = 177_751;
    historical.maximum_workflow_ms = 195_496;
    historical.recovery_nominal_slot_us = 1_816_958;
    assert_eq!(historical.digest(), 0x8674_2390_3e5e_a3f2);
    assert_eq!(plan.authority.joint_membership, 0);
    assert_eq!(
        plan.positive_rungs(),
        &[1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1_024, 1_310]
    );
    assert_eq!(plan.family_size, 60);
    assert_eq!(plan.total_rung_count, 15);
    assert_eq!(plan.recovery_nominal_slot_us, 2_316_958);
    assert_eq!(plan.nominal_workflow_ms, 187_651);
    assert_eq!(plan.maximum_workflow_ms, 197_896);
}

#[test]
fn combined_target_status_keeps_distinct_reportable_termini() {
    assert_ne!(
        CombinedTargetStatus::TargetReached,
        CombinedTargetStatus::TargetReachedSparse
    );
    assert_ne!(
        CombinedTargetStatus::CurrentHeadroom,
        CombinedTargetStatus::TargetNotReachedAtCap
    );
}
