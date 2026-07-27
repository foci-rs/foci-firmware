use super::integral_evidence::{ExactRateInterval, IntegralRunEvidence, StageCEvidenceError};
use super::integral_plan::{
    STAGE_C_EVIDENCE_SCHEMA_REVISION, VelocityIntegralExecutionPlan, VelocityIntegralPlan,
};
use super::integral_reproduction::{
    StageCAuthoritySummary, StageCCandidateSlot, StageCCandidateSummary, StageCPlanIdentity,
    StageCReproductionDecision, StageCRestBoundaryReference, StageCSlotState, closed_intersection,
    compare_integral_runs, delta_e, signed_divergence_gap,
};
use super::reproduction::{STAGE_B_BUILD_REVISION, StageBDirectionalValidity};

fn interval(low_q: i32, high_q: i32) -> ExactRateInterval {
    ExactRateInterval::new(low_q, high_q).unwrap()
}

fn identity(channel: u8, digest: u64) -> StageCPlanIdentity {
    StageCPlanIdentity {
        channel,
        build_revision: 0x1234_5678,
        schema_revision: STAGE_C_EVIDENCE_SCHEMA_REVISION,
        plan_digest: digest,
    }
}

fn authority(channel: u8, digest: u64) -> StageCAuthoritySummary {
    StageCAuthoritySummary {
        identity: identity(channel, digest),
        stage_b_plan_digest: 0x0123_4567_89ab_cdef,
        final_p: 1_448,
        directional_memberships: [0x0038_0000; 2],
        stage_b_joint_membership: 0x0038_0000,
        joint_membership: 0x0038_0000,
        directional_validity: [
            StageBDirectionalValidity::ReproducedRegionCoversFinalP,
            StageBDirectionalValidity::ReproducedRegionTransferred,
        ],
        reduced_margin: true,
        directional_pooled_intervals: [[10, 20], [-20, -10]],
        planned_velocity_mrev_s: 2_929,
        target_velocity_rpm: 176,
        pwm_hz: 25_000,
        encoder_counts_per_rev: 4_000,
        positive_rung_count: 6,
        family_size: 32,
        maximum_workflow_ms: 79_872,
        probe_constrained_test_point: false,
    }
}

fn evidence(
    opening: [ExactRateInterval; 2],
    masks: [u32; 2],
    offsets: [i32; 2],
) -> IntegralRunEvidence {
    let mut evidence = IntegralRunEvidence::new(opening);
    for direction in 0..2 {
        for rung in 0..6 {
            if masks[direction] & (1 << rung) != 0 {
                let low = opening[direction].low_q + offsets[direction] + rung * 10;
                evidence
                    .retain_positive(direction, rung as usize, interval(low, low + 4))
                    .unwrap();
            }
        }
    }
    evidence
}

fn candidate(
    authority: StageCAuthoritySummary,
    evidence: IntegralRunEvidence,
    run: u64,
) -> StageCCandidateSummary {
    StageCCandidateSummary {
        authority,
        evidence,
        rest_boundary: None,
        run_completed_us: run * 1_000_000,
        run_digest: run,
    }
}

#[test]
fn boundary_references_are_reported_but_never_gate_reproduction() {
    let authority = authority(0, 7);
    let evidence = evidence(
        [interval(100, 110), interval(-110, -100)],
        [0b001111, 0b111100],
        [20, -40],
    );
    let previous_boundary = StageCRestBoundaryReference {
        positive_rung_index: 3,
        slot: 4,
    };
    let current_boundary = StageCRestBoundaryReference {
        positive_rung_index: 5,
        slot: 1,
    };
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(authority);
    let mut first = candidate(authority, evidence, 1);
    first.rest_boundary = Some(previous_boundary);
    assert!(matches!(
        slot.observe_sufficient(first),
        StageCReproductionDecision::CompleteCandidate(_)
    ));

    let mut second = candidate(authority, evidence, 2);
    second.rest_boundary = Some(current_boundary);
    let StageCReproductionDecision::Complete(delta) = slot.observe_sufficient(second) else {
        panic!("boundary identity must not gate an otherwise reproduced curve");
    };
    assert_eq!(delta.previous_rest_boundary, Some(previous_boundary));
    assert_eq!(delta.current_rest_boundary, Some(current_boundary));
    assert!(delta.reproduces());

    let mut none_then_present = StageCCandidateSlot::new();
    none_then_present.install_authority(authority);
    assert!(matches!(
        none_then_present.observe_sufficient(candidate(authority, evidence, 3)),
        StageCReproductionDecision::CompleteCandidate(_)
    ));
    let mut present = candidate(authority, evidence, 4);
    present.rest_boundary = Some(current_boundary);
    let StageCReproductionDecision::Complete(delta) = none_then_present.observe_sufficient(present)
    else {
        panic!("boundary presence must remain report-only");
    };
    assert_eq!(delta.previous_rest_boundary, None);
    assert_eq!(delta.current_rest_boundary, Some(current_boundary));

    for boundary in [None, Some(previous_boundary)] {
        let mut equal_boundaries = StageCCandidateSlot::new();
        equal_boundaries.install_authority(authority);
        let mut first = candidate(authority, evidence, 5);
        first.rest_boundary = boundary;
        assert!(matches!(
            equal_boundaries.observe_sufficient(first),
            StageCReproductionDecision::CompleteCandidate(_)
        ));
        let mut second = candidate(authority, evidence, 6);
        second.rest_boundary = boundary;
        let StageCReproductionDecision::Complete(delta) =
            equal_boundaries.observe_sufficient(second)
        else {
            panic!("equal boundary references must not gate reproduction");
        };
        assert_eq!(delta.previous_rest_boundary, boundary);
        assert_eq!(delta.current_rest_boundary, boundary);
    }
}

#[test]
fn authority_summary_preserves_model_validity_and_reduced_margin() {
    let retained = authority(0, 7);
    let rebuilt = retained.stage_c_authority();

    assert_eq!(rebuilt.directional_validity, retained.directional_validity);
    assert_eq!(rebuilt.reduced_margin, retained.reduced_margin);
    assert!(retained.model_interval_available(0));
    assert!(retained.model_interval_available(1));
    assert!(!retained.model_interval_available(2));
}

#[test]
fn delta_subtraction_overlap_and_divergence_are_exact() {
    assert_eq!(
        delta_e(interval(120, 140), interval(100, 110)),
        Ok(interval(10, 40))
    );
    assert_eq!(
        delta_e(interval(-140, -120), interval(-110, -100)),
        Ok(interval(-40, -10))
    );
    assert_eq!(
        delta_e(interval(i32::MAX, i32::MAX), interval(i32::MIN, i32::MIN)),
        Err(StageCEvidenceError::Conversion)
    );
    assert_eq!(
        closed_intersection(interval(0, 5), interval(5, 10)),
        Some(interval(5, 5))
    );
    assert_eq!(closed_intersection(interval(0, 4), interval(5, 10)), None);
    assert_eq!(
        signed_divergence_gap(interval(0, 4), interval(7, 10)),
        Ok(3)
    );
    assert_eq!(
        signed_divergence_gap(interval(7, 10), interval(0, 4)),
        Ok(-3)
    );
    assert_eq!(
        signed_divergence_gap(interval(0, 5), interval(5, 10)),
        Ok(0)
    );
}

#[test]
fn sparse_directional_cores_reproduce_without_cross_direction_gates() {
    let previous = evidence(
        [interval(100, 110), interval(-110, -100)],
        [0b10_1011, 0b11_1000],
        [20, -40],
    );
    let current = evidence(
        [interval(500, 510), interval(-510, -500)],
        [0b10_1011, 0b11_1000],
        [20, -40],
    );

    let delta = compare_integral_runs(previous, current, 6).unwrap();
    assert_eq!(delta.reproduced_masks, [0b10_1011, 0b11_1000]);
    assert_eq!(delta.divergent_masks, [0, 0]);
    assert!(delta.reproduces());
    assert_eq!(
        closed_intersection(
            delta_e(previous.positive[0][0], previous.opening[0]).unwrap(),
            delta_e(current.positive[0][0], current.opening[0]).unwrap(),
        ),
        Some(interval(10, 24))
    );
    assert_eq!(
        closed_intersection(
            delta_e(previous.positive[1][3], previous.opening[1]).unwrap(),
            delta_e(current.positive[1][3], current.opening[1]).unwrap(),
        ),
        Some(interval(-20, -6))
    );
}

#[test]
fn eligibility_movement_and_response_divergence_are_separate() {
    let previous = evidence(
        [interval(100, 110), interval(-110, -100)],
        [0b00_1111, 0b11_1100],
        [20, -40],
    );
    let mut current = evidence(
        [interval(100, 110), interval(-110, -100)],
        [0b01_1110, 0b11_1100],
        [20, -40],
    );
    current.positive[1][4] = interval(100, 110);

    let delta = compare_integral_runs(previous, current, 6).unwrap();
    assert_eq!(delta.previous_only_masks, [0b000001, 0]);
    assert_eq!(delta.current_only_masks, [0b010000, 0]);
    assert_eq!(delta.divergent_masks, [0, 0b010000]);
    assert_eq!(
        signed_divergence_gap(
            delta_e(previous.positive[1][4], previous.opening[1]).unwrap(),
            delta_e(current.positive[1][4], current.opening[1]).unwrap(),
        ),
        Ok(196)
    );
    assert!(delta.reproduces());
}

#[test]
fn two_reproduced_rungs_or_out_of_range_masks_refuse_reproduction() {
    let two = evidence(
        [interval(0, 1), interval(0, 1)],
        [0b000011, 0b000011],
        [10, 10],
    );
    let delta = compare_integral_runs(two, two, 6).unwrap();
    assert!(!delta.reproduces());

    let mut invalid = two;
    invalid.eligible_masks[0] = 1 << 6;
    assert_eq!(
        compare_integral_runs(invalid, two, 6),
        Err(StageCEvidenceError::MaskOutOfRange)
    );
}

#[test]
fn authority_only_lifecycle_preserves_authority_on_safe_insufficiency() {
    let authority = authority(0, 7);
    let mut slot = StageCCandidateSlot::new();
    assert_eq!(slot.state(), StageCSlotState::Empty);
    slot.install_authority(authority);
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);

    slot.observe_inconclusive();
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);
    slot.observe_host_error();
    slot.preserve_for_same_channel_calibration(0);
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);

    slot.observe_fault();
    assert_eq!(slot.state(), StageCSlotState::Empty);
}

#[test]
fn candidate_completes_mismatches_replaces_and_inconclusive_demotes() {
    let authority = authority(0, 7);
    let first_evidence = evidence(
        [interval(100, 110), interval(-110, -100)],
        [0b001111, 0b111100],
        [20, -40],
    );
    let first = candidate(authority, first_evidence, 1);
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(authority);
    assert!(matches!(
        slot.observe_sufficient(first),
        StageCReproductionDecision::CompleteCandidate(_)
    ));
    assert_eq!(slot.state(), StageCSlotState::Candidate);

    let mut divergent = first_evidence;
    for direction in 0..2 {
        for rung in 0..6 {
            if divergent.eligible_masks[direction] & (1 << rung) != 0 {
                divergent.positive[direction][rung] = interval(1_000, 1_010);
            }
        }
    }
    let second = candidate(authority, divergent, 2);
    assert!(matches!(
        slot.observe_sufficient(second),
        StageCReproductionDecision::InconclusiveReproductionMismatch(_)
    ));
    assert_eq!(slot.candidate().unwrap().run_digest, 2);

    slot.observe_inconclusive();
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);
    assert_eq!(slot.authority(), Some(authority));

    let fresh = candidate(authority, first_evidence, 3);
    assert!(matches!(
        slot.observe_sufficient(fresh),
        StageCReproductionDecision::CompleteCandidate(_)
    ));
    let matching = candidate(authority, first_evidence, 4);
    assert!(matches!(
        slot.observe_sufficient(matching),
        StageCReproductionDecision::Complete(_)
    ));
    assert_eq!(slot.state(), StageCSlotState::Empty);
}

#[test]
fn plan_mismatch_and_invalidation_follow_the_authority_lifecycle() {
    let authority = authority(0, 7);
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(authority);
    assert!(matches!(
        slot.preflight(identity(0, 8)),
        Some(StageCReproductionDecision::RejectedPlanMismatch { .. })
    ));
    assert_eq!(slot.authority(), Some(authority));

    slot.invalidate_for_channel(0);
    assert_eq!(slot.state(), StageCSlotState::AuthorityOnly);
    slot.invalidate_for_channel(1);
    assert_eq!(slot.state(), StageCSlotState::Empty);

    slot.install_authority(authority);
    slot.invalidate_for_unrelated_commissioning();
    assert_eq!(slot.state(), StageCSlotState::Empty);
    assert_eq!(StageCCandidateSlot::new().state(), StageCSlotState::Empty);
}

#[test]
fn schema_one_candidate_rejects_before_motion_without_losing_authority() {
    let current = authority(0, 7);
    let mut retained = current;
    retained.identity.schema_revision = 1;
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained);

    assert!(matches!(
        slot.preflight(current.identity),
        Some(StageCReproductionDecision::RejectedPlanMismatch { .. })
    ));
    assert_eq!(slot.authority(), Some(retained));
}

#[test]
fn q8_8_package_authority_rejects_before_stage_c_motion() {
    let mut retained = authority(0, 7);
    retained.identity.build_revision = 0x7553_6758;
    let mut current = retained.identity;
    current.build_revision = STAGE_B_BUILD_REVISION;
    let mut slot = StageCCandidateSlot::new();
    slot.install_authority(retained);

    assert!(matches!(
        slot.preflight(current),
        Some(StageCReproductionDecision::RejectedPlanMismatch { .. })
    ));
    assert_eq!(slot.authority(), Some(retained));
}

#[test]
fn stage_c_candidate_size_is_measured_without_padding() {
    assert_eq!(core::mem::size_of::<VelocityIntegralPlan>(), 224);
    assert_eq!(core::mem::size_of::<VelocityIntegralExecutionPlan>(), 88);
    assert_eq!(core::mem::size_of::<StageCAuthoritySummary>(), 88);
    assert_eq!(
        core::mem::size_of::<crate::commissioning::types::VelocityIntegralTerminal>(),
        632
    );
    assert!(core::mem::size_of::<StageCCandidateSlot>() <= 640);
    assert_eq!(core::mem::size_of::<StageCCandidateSlot>(), 640);
    assert_eq!(
        core::mem::size_of::<super::integral_reproduction::StageCReproductionDelta>(),
        596
    );
}
