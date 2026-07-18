use super::{
    CLAMP_ENVELOPE, CLAMP_TRAVEL, CURRENT_HEADROOM_DEN, CURRENT_HEADROOM_NUM,
    EFFECTIVE_CEILING_ENCODER, EFFECTIVE_CEILING_MACHINE, ENVELOPE_ABSOLUTE_MARGIN_MREV_S,
    ENVELOPE_PROPORTIONAL_DEN, ENVELOPE_PROPORTIONAL_NUM, MAX_SAFETY_OBSERVATION_GAP_US,
    MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MIN_STRUCTURAL_RUNGS, MOVING_STROKE_US,
    OBSERVATIONS_PER_DIRECTION_PER_RUNG, RequestedVelocitySource, SETTLE_ANALYSIS_BUDGET_US,
    VelocityPlanError, VelocitySweepRequest, ZERO_SETTLE_US, build_gain_ladder, current_gain_span,
    envelope_cap, plan_velocity_sweep, representability_ceilings, travel_cap,
};

fn request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 10_000,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_000,
        settle_travel_reserve_mrev: 200,
        negative_position_headroom_mrev: 50_000,
        positive_position_headroom_mrev: 50_000,
        max_duration_ms: 60_000,
    }
}

#[test]
fn inverse_envelope_uses_both_named_margins() {
    assert_eq!(ENVELOPE_PROPORTIONAL_NUM, 3);
    assert_eq!(ENVELOPE_PROPORTIONAL_DEN, 2);
    assert_eq!(ENVELOPE_ABSOLUTE_MARGIN_MREV_S, 2_000);
    assert_eq!(envelope_cap(10_000), Ok(6_666));
    assert_eq!(envelope_cap(4_000), Ok(2_000));
    assert_eq!(envelope_cap(2_000), Err(VelocityPlanError::InvalidRequest));
}

#[test]
fn plan_reports_machine_and_encoder_ceiling_provenance() {
    let machine_plan = plan_velocity_sweep(request(), 4_000, 25_000, 1_000).unwrap();
    assert_eq!(machine_plan.effective_ceiling_mrev_s, 10_000);
    assert_eq!(
        machine_plan.effective_ceiling_source,
        EFFECTIVE_CEILING_MACHINE
    );
    assert_ne!(machine_plan.clamp_flags & CLAMP_ENVELOPE, 0);

    let mut encoder_limited = request();
    encoder_limited.machine_velocity_ceiling_mrev_s = 200_000;
    encoder_limited.requested_velocity_mrev_s = 200_000;
    let encoder_plan = plan_velocity_sweep(encoder_limited, 4_000, 25_000, 1_000).unwrap();
    assert_eq!(
        encoder_plan.effective_ceiling_source,
        EFFECTIVE_CEILING_ENCODER
    );
    assert!(encoder_plan.effective_ceiling_mrev_s < 125_000);
}

#[test]
fn every_firmware_representability_term_is_enumerated() {
    let limits = representability_ceilings(4_000, 25_000).unwrap();
    assert!(limits.encoder_observable_mrev_s > 80_000);
    assert!(limits.tmc_target_mrev_s > limits.encoder_observable_mrev_s);
    assert_eq!(limits.tmc_target_mrev_s, limits.tmc_feedback_mrev_s);
    assert!(limits.fixed_point_mrev_s >= limits.tmc_target_mrev_s);
}

#[test]
fn encoder_observability_tracks_the_measured_three_ms_gap_bound() {
    assert_eq!(MAX_SAFETY_OBSERVATION_GAP_US, 3_000);
    let limits = representability_ceilings(4_000, 25_000).unwrap();
    assert!((83_000..84_000).contains(&limits.encoder_observable_mrev_s));
}

#[test]
fn travel_reserve_is_subtracted_before_the_active_duration_cap() {
    assert_eq!(travel_cap(1_000, 100, 256_000), Ok(3_515));
    assert_eq!(travel_cap(1_000, 100, 512_000), Ok(1_757));
    assert_eq!(
        travel_cap(1_000, 1_000, 256_000),
        Err(VelocityPlanError::InvalidRequest)
    );
    assert_eq!(
        travel_cap(2, 1, 2_000_000),
        Err(VelocityPlanError::ZeroMovingTravelAmplitude)
    );
}

#[test]
fn travel_binding_has_a_named_clamp_cause() {
    let mut limited = request();
    limited.max_stroke_travel_mrev = 300;
    limited.settle_travel_reserve_mrev = 100;
    let plan = plan_velocity_sweep(limited, 4_000, 25_000, 1_000).unwrap();
    assert_ne!(plan.clamp_flags & CLAMP_TRAVEL, 0);
    assert_eq!(plan.used_velocity_mrev_s, 781);
}

#[test]
fn current_span_uses_quantum_start_and_inclusive_nine_tenths_top() {
    let (p_start, p_top, usable) = current_gain_span(100, 1_000).unwrap();
    assert_eq!(CURRENT_HEADROOM_NUM, 9);
    assert_eq!(CURRENT_HEADROOM_DEN, 10);
    assert_eq!(usable, 900);
    assert_eq!(p_start.raw(), 3);
    assert_eq!(p_top.raw(), 2_304);
    assert_eq!(u32::from(p_top.raw()) * 100 / 256, u32::from(usable));
    assert!(usable < 1_000);
}

#[test]
fn anchored_half_octaves_do_not_compound_approximation() {
    let ladder = build_gain_ladder(350, 1_500).unwrap();
    let raw: heapless::Vec<u16, MAX_SWEEP_RUNGS> = ladder.iter().map(|gain| gain.raw()).collect();
    assert_eq!(raw.as_slice(), &[350, 495, 700, 990, 1_400, 1_500]);
}

#[test]
fn low_gain_collisions_are_deduplicated_and_top_is_explicit() {
    let powers = build_gain_ladder(1, 16).unwrap();
    let bounded = build_gain_ladder(10, 90).unwrap();
    assert_eq!(
        powers
            .iter()
            .map(|gain| gain.raw())
            .collect::<heapless::Vec<_, 32>>()
            .as_slice(),
        &[1, 2, 4, 8, 16]
    );
    assert_eq!(
        bounded
            .iter()
            .map(|gain| gain.raw())
            .collect::<heapless::Vec<_, 32>>()
            .as_slice(),
        &[10, 15, 20, 30, 40, 60, 80, 90]
    );
}

#[test]
fn structural_floor_counts_distinct_post_dedupe_gains() {
    assert_eq!(MIN_STRUCTURAL_RUNGS, 5);
    assert!(build_gain_ladder(1, 8).is_err());
    assert_eq!(build_gain_ladder(1, 16).unwrap().len(), 5);
}

#[test]
fn maximum_domain_stays_inside_the_audited_two_k_family() {
    let ladder = build_gain_ladder(2, 32_767).unwrap();
    assert!(ladder.len() <= MAX_SWEEP_RUNGS);
    assert!(ladder.len() * usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG) <= 64);
}

#[test]
fn plan_freezes_exact_ladder_and_workflow_duration() {
    let plan = plan_velocity_sweep(request(), 4_000, 25_000, 1_000).unwrap();
    let expected_ms = u32::from(plan.rung_count)
        * 4
        * (MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US)
        / 1_000;
    assert_eq!(plan.gains[0], plan.p_start);
    assert_eq!(plan.gains[usize::from(plan.rung_count) - 1], plan.p_top);
    assert_eq!(plan.observations_per_direction, 2);
    assert_eq!(plan.nominal_workflow_ms, expected_ms);
    assert_eq!(plan.maximum_workflow_ms, expected_ms);
}

#[test]
fn analysis_budget_preserves_the_shortest_scored_rest_suffix() {
    assert_eq!(ZERO_SETTLE_US, 500_000);
    assert_eq!(MIN_SETTLE_CAPTURE_US, 64_000);
    assert_eq!(SETTLE_ANALYSIS_BUDGET_US, 236_000);
    assert!(SETTLE_ANALYSIS_BUDGET_US + MIN_SETTLE_CAPTURE_US <= ZERO_SETTLE_US);
}

#[test]
fn host_default_primitive_budget_does_not_cap_total_workflow() {
    let mut host_request = request();
    host_request.max_duration_ms = 3_000;

    let plan = plan_velocity_sweep(host_request, 4_000, 25_000, 1_000).unwrap();

    assert!(plan.maximum_workflow_ms > u32::from(host_request.max_duration_ms));
}

#[test]
fn primitive_budget_must_cover_one_moving_and_settle_window() {
    let mut too_short = request();
    too_short.max_duration_ms = 455;

    assert_eq!(
        plan_velocity_sweep(too_short, 4_000, 25_000, 1_000),
        Err(VelocityPlanError::InvalidRequest)
    );
}
