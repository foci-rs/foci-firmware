use super::super::sweep_plan::{MAX_SWEEP_RUNGS, MIN_STRUCTURAL_RUNGS, VelocityPlanError};
use super::{
    AdditiveDiscoveryRequest, AdditiveLadderPlanError, BreakawayProbeContext, BreakawayProbePlan,
    GridFloorOrigin, MAX_ADDITIVE_GAIN_RATIO_DEN, MAX_ADDITIVE_GAIN_RATIO_NUM,
    PROBE_CONFIDENCE_FAMILY_SIZE, PROBE_MAX_CAPTURE_INTERVAL_US, PROBE_MAX_OBSERVATIONS,
    PROBE_MAX_OBSERVATIONS_PER_DIRECTION, PROBE_MOTION_REVOLUTION_DENOMINATOR,
    motion_threshold_counts, plan_additive_discovery_ladder, plan_breakaway_probe,
    plan_upward_breakaway_probe, realize_candidate_additive_ladder, resolve_grid_floor,
    satisfies_additive_ratio_guard,
};
use foci_tmc4671::pid::PidPGain;

fn gain(raw: u16) -> PidPGain {
    PidPGain::new(raw).expect("raw gain within PidPGain range")
}

/// Canonical acquisition context for probe-plan tests. Its exact values are
/// arbitrary but fixed, so digest self-consistency and cross-context digest
/// difference are both reproducible.
fn probe_context() -> BreakawayProbeContext {
    BreakawayProbeContext {
        workflow_digest: 0xA1B2_C3D4_E5F6_0718,
        target_velocity_rpm: 600,
        hard_torque_limit: 2_816,
        usable_torque_limit: 2_048,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
    }
}

fn generous_request(
    floor: u16,
    breakaway: u16,
    ceiling: u16,
    probe_plan_digest: u64,
) -> AdditiveDiscoveryRequest {
    AdditiveDiscoveryRequest {
        floor: gain(floor),
        breakaway: gain(breakaway),
        authority_ceiling: gain(ceiling),
        observations_per_direction: 4,
        max_duration_ms: u16::MAX,
        workflow_envelope_ms: u32::MAX,
        probe_plan_digest,
    }
}

// --- Probe plan: realized grid, family, and observation budget -----------------

#[test]
fn probe_plan_matches_the_realized_half_octave_grid() {
    let plan = plan_breakaway_probe(gain(1), gain(1_024), probe_context()).unwrap();

    let realized: heapless::Vec<u16, MAX_SWEEP_RUNGS> = plan
        .search_gains
        .iter()
        .take(usize::from(plan.search_count))
        .map(|gain| gain.raw())
        .collect();
    assert_eq!(
        realized.as_slice(),
        &[
            1, 2, 3, 4, 6, 8, 12, 16, 23, 32, 46, 64, 91, 128, 181, 256, 362, 512, 724, 1_024,
        ]
    );
    assert_eq!(plan.search_count, 20);
    assert_eq!(plan.confidence_family_size, PROBE_CONFIDENCE_FAMILY_SIZE);
    assert_eq!(
        plan.maximum_observations,
        PROBE_MAX_OBSERVATIONS_PER_DIRECTION
    );
    assert_ne!(plan.plan_digest, 0);
    assert_eq!(plan.plan_digest, plan.digest());
}

#[test]
fn probe_plan_digest_matches_the_foci_trace_fixture() {
    let mut search_gains = [gain(1); MAX_SWEEP_RUNGS];
    search_gains[..4].copy_from_slice(&[gain(3), gain(5), gain(6), gain(9)]);
    let plan = BreakawayProbePlan {
        search_gains,
        search_count: 4,
        confidence_family_size: 16,
        maximum_observations: 8,
        context: BreakawayProbeContext {
            target_velocity_rpm: -600,
            ..probe_context()
        },
        plan_digest: 0,
    };

    assert_eq!(plan.digest(), 0x2581_c2da_8114_9f64);
}

#[test]
fn probe_digest_changes_when_workflow_digest_or_acquisition_context_differs() {
    let base = plan_breakaway_probe(gain(1), gain(1_024), probe_context()).unwrap();

    // The workflow digest binds the command-level envelope into the probe.
    let mut altered = probe_context();
    altered.workflow_digest ^= 0x1;
    let plan = plan_breakaway_probe(gain(1), gain(1_024), altered).unwrap();
    assert_ne!(
        base.plan_digest, plan.plan_digest,
        "workflow digest must be bound into the probe digest"
    );

    // Every physical acquisition-context field must move the digest too.
    let mut altered = probe_context();
    altered.target_velocity_rpm += 1;
    let plan = plan_breakaway_probe(gain(1), gain(1_024), altered).unwrap();
    assert_ne!(base.plan_digest, plan.plan_digest, "target velocity");

    let mut altered = probe_context();
    altered.hard_torque_limit += 1;
    let plan = plan_breakaway_probe(gain(1), gain(1_024), altered).unwrap();
    assert_ne!(base.plan_digest, plan.plan_digest, "hard torque limit");

    let mut altered = probe_context();
    altered.usable_torque_limit += 1;
    let plan = plan_breakaway_probe(gain(1), gain(1_024), altered).unwrap();
    assert_ne!(base.plan_digest, plan.plan_digest, "usable torque limit");

    let mut altered = probe_context();
    altered.pwm_hz += 1;
    let plan = plan_breakaway_probe(gain(1), gain(1_024), altered).unwrap();
    assert_ne!(base.plan_digest, plan.plan_digest, "pwm frequency");

    let mut altered = probe_context();
    altered.encoder_counts_per_rev += 1;
    let plan = plan_breakaway_probe(gain(1), gain(1_024), altered).unwrap();
    assert_ne!(base.plan_digest, plan.plan_digest, "encoder counts per rev");
}

#[test]
fn probe_budgets_exactly_eight_observations_per_direction() {
    assert_eq!(PROBE_MAX_OBSERVATIONS_PER_DIRECTION, 8);
    assert_eq!(PROBE_CONFIDENCE_FAMILY_SIZE, 16);
    assert_eq!(u16::from(PROBE_MAX_OBSERVATIONS_PER_DIRECTION) * 2, 16);
}

#[test]
fn probe_plan_rejects_a_span_too_thin_for_five_structural_rungs() {
    assert_eq!(
        plan_breakaway_probe(gain(1), gain(4), probe_context()),
        Err(VelocityPlanError::InsufficientStructuralSpan)
    );
}

#[test]
fn upward_probe_plan_freezes_excursion_and_observation_bounds() {
    let plan = plan_upward_breakaway_probe(gain(1), gain(1_024), probe_context()).unwrap();

    assert_eq!(PROBE_MOTION_REVOLUTION_DENOMINATOR, 64);
    assert_eq!(PROBE_MAX_CAPTURE_INTERVAL_US, 2_000);
    assert_eq!(PROBE_MAX_OBSERVATIONS, 128);
    assert_eq!(plan.motion_threshold_counts, 63);
    assert_eq!(plan.max_capture_interval_us, 2_000);
    assert_eq!(plan.maximum_observations, 128);
    assert_eq!(plan.search_count, 20);
    assert_eq!(plan.plan_digest, plan.digest());
}

#[test]
fn upward_probe_motion_threshold_uses_checked_ceiling_division() {
    assert_eq!(motion_threshold_counts(4_000), Ok(63));
    assert_eq!(motion_threshold_counts(64), Ok(1));
    assert_eq!(motion_threshold_counts(1), Ok(1));
    assert_eq!(
        motion_threshold_counts(0),
        Err(VelocityPlanError::InvalidRequest)
    );
}

#[test]
fn upward_probe_digest_binds_excursion_contract_and_physical_context() {
    let base = plan_upward_breakaway_probe(gain(1), gain(1_024), probe_context()).unwrap();

    let mut threshold_changed = base;
    threshold_changed.motion_threshold_counts += 1;
    assert_ne!(base.plan_digest, threshold_changed.digest());

    let mut interval_changed = base;
    interval_changed.max_capture_interval_us += 1;
    assert_ne!(base.plan_digest, interval_changed.digest());

    let mut budget_changed = base;
    budget_changed.maximum_observations -= 1;
    assert_ne!(base.plan_digest, budget_changed.digest());

    let mut context = probe_context();
    context.encoder_counts_per_rev = 8_192;
    let context_changed = plan_upward_breakaway_probe(gain(1), gain(1_024), context).unwrap();
    assert_ne!(base.plan_digest, context_changed.plan_digest);
}

// --- Predecessor lookup, arithmetic-floor clamping, missing-bracket code -------

#[test]
fn predecessor_lookup_returns_the_prior_grid_rung() {
    // Grid 1..1024: index 15 is 256, whose predecessor is 181 (index 14).
    let (floor, origin) = resolve_grid_floor(gain(1), gain(1_024), 20, 15).unwrap();
    assert_eq!(floor.raw(), 181);
    assert_eq!(origin, GridFloorOrigin::Predecessor);
}

#[test]
fn arithmetic_floor_clamps_when_breakaway_is_the_smallest_rung() {
    let (floor, origin) = resolve_grid_floor(gain(1), gain(1_024), 20, 0).unwrap();
    assert_eq!(floor.raw(), 1);
    assert_eq!(origin, GridFloorOrigin::ClampedAtBreakaway);
}

#[test]
fn resolve_grid_floor_rejects_an_out_of_range_breakaway_index() {
    assert_eq!(
        resolve_grid_floor(gain(1), gain(1_024), 20, 20),
        Err(VelocityPlanError::InvalidRequest)
    );
}

// --- Per-n candidate enumeration, pinned for the representative geometry -------

#[test]
fn every_candidate_rung_count_matches_the_representative_geometry() {
    // floor=181, breakaway=256, ceiling=1088 (docs/specs/2026-07-31-foc-213-...).
    // step and realized rung count for every n from 5 through 32.
    const EXPECTED: [(u8, u32, u8); 28] = [
        (5, 278, 5),
        (6, 208, 6),
        (7, 167, 7),
        (8, 139, 8),
        (9, 119, 9),
        (10, 104, 10),
        (11, 93, 11),
        (12, 84, 12),
        (13, 76, 13),
        (14, 70, 14),
        (15, 64, 15),
        (16, 60, 16),
        (17, 56, 17),
        (18, 52, 18),
        (19, 49, 19),
        (20, 47, 20),
        (21, 44, 21),
        (22, 42, 22),
        (23, 40, 23),
        (24, 38, 24),
        (25, 37, 25),
        (26, 35, 26),
        (27, 34, 27),
        (28, 32, 28),
        (29, 31, 29),
        (30, 30, 30),
        (31, 29, 31),
        (32, 28, 32),
    ];

    for (n, expected_step, expected_rung_count) in EXPECTED {
        let candidate = realize_candidate_additive_ladder(n, 181, 256, 1_088).unwrap();
        assert_eq!(candidate.step, u32::from(expected_step), "n = {n}");
        assert_eq!(candidate.rung_count, expected_rung_count, "n = {n}");
        assert_eq!(
            candidate.gains[usize::from(expected_rung_count) - 1],
            1_088,
            "n = {n}"
        );
    }
}

#[test]
fn additive_step_dedupes_the_floor_when_it_equals_breakaway() {
    // Grid floor was clamped to breakaway (arithmetic-floor case): the ladder must
    // not carry a duplicate leading entry.
    let candidate = realize_candidate_additive_ladder(32, 256, 256, 1_088).unwrap();
    assert_eq!(candidate.gains[0], 256);
    assert_ne!(candidate.gains[1], 256);
    assert_eq!(candidate.rung_count, 31);
    assert_eq!(candidate.gains[30], 1_088);
}

// --- Resolved additive ladder: the representative case --------------------------

#[test]
fn plan_additive_discovery_ladder_selects_the_representative_ladder() {
    // The realized 256/181 grid bracket ratio exceeds the provisional guard
    // (1.4143646 > 1.4141), proving the guard does not apply to it.
    assert!(256_u32 * MAX_ADDITIVE_GAIN_RATIO_DEN > 181_u32 * MAX_ADDITIVE_GAIN_RATIO_NUM);

    let request = generous_request(181, 256, 1_088, 0xabcd_1234);
    let plan = plan_additive_discovery_ladder(request).unwrap();

    assert_eq!(plan.rung_count, 32);
    assert_eq!(plan.confidence_family_size, 128);
    assert_eq!(plan.observations_per_direction, 4);
    assert_eq!(plan.probe_plan_digest, 0xabcd_1234);
    assert_eq!(plan.gains[0].raw(), 181);
    assert_eq!(plan.gains[1].raw(), 256);
    assert_eq!(plan.gains[2].raw(), 284);
    assert_eq!(plan.gains[31].raw(), 1_088);
    assert_eq!(
        plan.gains[usize::from(plan.rung_count) - 1].raw(),
        request.authority_ceiling.raw(),
        "final gain must anchor exactly on the authority ceiling"
    );

    let expected: [u16; 32] = [
        181, 256, 284, 312, 340, 368, 396, 424, 452, 480, 508, 536, 564, 592, 620, 648, 676, 704,
        732, 760, 788, 816, 844, 872, 900, 928, 956, 984, 1_012, 1_040, 1_068, 1_088,
    ];
    let realized: heapless::Vec<u16, MAX_SWEEP_RUNGS> = plan
        .gains
        .iter()
        .take(usize::from(plan.rung_count))
        .map(|gain| gain.raw())
        .collect();
    assert_eq!(realized.as_slice(), &expected);

    assert_ne!(plan.plan_digest, 0);
    assert_eq!(plan.plan_digest, plan.digest());
}

#[test]
fn plan_digest_binds_the_probe_plan_that_located_breakaway() {
    let first = plan_additive_discovery_ladder(generous_request(181, 256, 1_088, 1)).unwrap();
    let second = plan_additive_discovery_ladder(generous_request(181, 256, 1_088, 2)).unwrap();

    assert_eq!(first.probe_plan_digest, 1);
    assert_eq!(second.probe_plan_digest, 2);
    assert_ne!(first.plan_digest, second.plan_digest);
}

#[test]
fn digest_ignores_padding_and_unused_gain_slots() {
    let mut first = plan_additive_discovery_ladder(generous_request(181, 256, 1_088, 7)).unwrap();
    let mut second = first;
    // Corrupt only the padding beyond rung_count; the digest must not change.
    for slot in first.gains.iter_mut().skip(usize::from(first.rung_count)) {
        *slot = gain(1);
    }
    for slot in second.gains.iter_mut().skip(usize::from(second.rung_count)) {
        *slot = PidPGain::ZERO;
    }
    assert_eq!(first.digest(), second.digest());
}

// --- Checked arithmetic, exact ceiling anchoring, near-max domain --------------

#[test]
fn plan_additive_discovery_ladder_stays_checked_near_the_maximum_domain() {
    let request = generous_request(16_384, 24_576, 32_767, 42);
    let plan = plan_additive_discovery_ladder(request).unwrap();

    assert_eq!(plan.rung_count, 32);
    assert_eq!(plan.gains[2].raw(), 24_850);
    assert_eq!(plan.gains[31].raw(), 32_767);
}

#[test]
fn additive_ratio_guard_uses_exact_cross_multiplication() {
    // breakaway = 256; max first-gap that still satisfies 181/128 is exactly 362
    // (256 * 181 = 46_336 = 362 * 128).
    assert!(satisfies_additive_ratio_guard(362, 256));
    assert!(!satisfies_additive_ratio_guard(363, 256));
}

// --- Terminal rejections before motion ------------------------------------------

#[test]
fn insufficient_additive_span_rejects_a_ceiling_one_step_above_breakaway() {
    let request = generous_request(181, 256, 257, 0);
    assert_eq!(
        plan_additive_discovery_ladder(request),
        Err(AdditiveLadderPlanError::InsufficientAdditiveSpan)
    );
}

#[test]
fn invalid_request_rejects_a_floor_above_breakaway() {
    let request = generous_request(300, 256, 1_088, 0);
    assert_eq!(
        plan_additive_discovery_ladder(request),
        Err(AdditiveLadderPlanError::InvalidRequest)
    );
}

#[test]
fn invalid_request_rejects_zero_inputs() {
    let mut request = generous_request(181, 256, 1_088, 0);
    request.observations_per_direction = 0;
    assert_eq!(
        plan_additive_discovery_ladder(request),
        Err(AdditiveLadderPlanError::InvalidRequest)
    );

    let mut request = generous_request(181, 256, 1_088, 0);
    request.max_duration_ms = 0;
    assert_eq!(
        plan_additive_discovery_ladder(request),
        Err(AdditiveLadderPlanError::InvalidRequest)
    );
}

#[test]
fn confidence_family_table_overflow_rejects_every_candidate() {
    let mut request = generous_request(181, 256, 1_088, 0);
    request.observations_per_direction = 27; // 5 rungs * 27 = 135 already exceeds 128.
    assert_eq!(
        plan_additive_discovery_ladder(request),
        Err(AdditiveLadderPlanError::ConfidenceFamilyTooLarge)
    );
}

#[test]
fn primitive_duration_rejection_fires_before_the_candidate_search() {
    let mut request = generous_request(181, 256, 1_088, 0);
    request.max_duration_ms = 1;
    assert_eq!(
        plan_additive_discovery_ladder(request),
        Err(AdditiveLadderPlanError::PrimitiveDurationExceeded)
    );
}

#[test]
fn command_envelope_rejection_fires_when_every_candidate_overruns_it() {
    let mut request = generous_request(181, 256, 1_088, 0);
    request.workflow_envelope_ms = 0;
    assert_eq!(
        plan_additive_discovery_ladder(request),
        Err(AdditiveLadderPlanError::WorkflowEnvelopeExceeded)
    );
}

#[test]
fn additive_ladder_resolution_budget_boundary() {
    // breakaway = 256, floor = 181. At n = 32 (the finest resolution), ceiling =
    // 3_436 lands the first additive gap exactly at the provisional limit (362);
    // ceiling = 3_437 pushes it one gain unit past the limit (363).
    let passing = generous_request(181, 256, 3_436, 0);
    let plan = plan_additive_discovery_ladder(passing).unwrap();
    assert_eq!(plan.gains[2].raw(), 362);

    let failing = generous_request(181, 256, 3_437, 0);
    assert_eq!(
        plan_additive_discovery_ladder(failing),
        Err(AdditiveLadderPlanError::AdditiveLadderResolutionBudgetExceeded)
    );
}

// --- Regression guard: the superseded `breakaway - floor` step rule ------------

#[test]
fn the_superseded_breakaway_minus_floor_rule_fails_its_own_budget() {
    // docs/specs/2026-07-31-foc-213-breakaway-seeded-p-ladder.md section 5: deriving
    // the additive step from `breakaway - floor` is structurally invalid. This pins
    // the two documented failure shapes so the rule cannot be silently reintroduced.
    let ceiling = 1_088_u32;

    // breakaway = 91, floor = 64 (predecessor on the 1..1024 grid): step = 27,
    // yielding 39 rungs, which exceeds MAX_SWEEP_RUNGS.
    let breakaway = 91_u32;
    let floor = 64_u32;
    let step = breakaway - floor;
    assert_eq!(step, 27);
    let rung_count = 2 + (ceiling - breakaway).div_ceil(step);
    assert_eq!(rung_count, 39);
    assert!(usize::try_from(rung_count).unwrap() > MAX_SWEEP_RUNGS);

    // breakaway = 1024, floor = 724 (predecessor): step = 300, yielding 3 rungs,
    // which falls below MIN_STRUCTURAL_RUNGS.
    let breakaway = 1_024_u32;
    let floor = 724_u32;
    let step = breakaway - floor;
    assert_eq!(step, 300);
    let rung_count = 2 + (ceiling - breakaway).div_ceil(step);
    assert_eq!(rung_count, 3);
    assert!(usize::try_from(rung_count).unwrap() < MIN_STRUCTURAL_RUNGS);
}
