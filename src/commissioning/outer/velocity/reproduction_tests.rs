use super::{
    STAGE_B_BUILD_REVISION, STAGE_B_EVIDENCE_SCHEMA_REVISION, StageBCandidateSlot,
    StageBCandidateSummary, StageBPlanIdentity, StageBReproductionDecision, const_fnv1a32,
};

fn identity(channel: u8, plan_digest: u64) -> StageBPlanIdentity {
    StageBPlanIdentity {
        channel,
        build_revision: STAGE_B_BUILD_REVISION,
        schema_revision: STAGE_B_EVIDENCE_SCHEMA_REVISION,
        plan_digest,
    }
}

fn summary(channel: u8, run_digest: u64) -> StageBCandidateSummary {
    StageBCandidateSummary {
        identity: identity(channel, 0x1234),
        forward_membership: 0b0001_1100,
        reverse_membership: 0b0011_1000,
        joint_membership: 0b0111_0000,
        nominated_p: 1_448,
        forward_interval: [100, 120],
        reverse_interval: [-130, -100],
        run_started_us: run_digest * 1_000,
        run_completed_us: run_digest * 1_000 + 40_000,
        run_digest,
    }
}

#[test]
fn reproduction_fnv_vectors_and_build_revision_are_stable() {
    assert_eq!(const_fnv1a32(b""), 0x811c_9dc5);
    assert_eq!(const_fnv1a32(b"a"), 0xe40c_292c);
    assert_ne!(STAGE_B_BUILD_REVISION, 0);
}

#[test]
fn reproduction_empty_slot_becomes_candidate_then_completes() {
    let mut slot = StageBCandidateSlot::new();
    let first = summary(0, 1);
    let second = summary(0, 2);
    assert_eq!(
        slot.observe_sufficient(first),
        StageBReproductionDecision::CompleteCandidate(first)
    );
    let StageBReproductionDecision::Complete(delta) = slot.observe_sufficient(second) else {
        panic!("matching evidence must complete")
    };
    assert_eq!(delta.previous_run_digest, 1);
    assert_eq!(delta.current_run_digest, 2);
    assert_eq!(delta.previous_memberships, delta.current_memberships);
    assert!(slot.candidate().is_none());
}

#[test]
fn reproduction_timestamps_are_context_not_match_predicates() {
    let mut slot = StageBCandidateSlot::new();
    let first = summary(0, 1);
    let mut second = summary(0, 2);
    second.run_started_us = 9_000_000;
    second.run_completed_us = 9_999_999;
    slot.observe_sufficient(first);
    assert!(matches!(
        slot.observe_sufficient(second),
        StageBReproductionDecision::Complete(_)
    ));
}

#[test]
fn reproduction_mismatch_reports_and_replaces_candidate() {
    for mutate in 0..5 {
        let mut slot = StageBCandidateSlot::new();
        let first = summary(0, 1);
        let mut second = summary(0, 2);
        match mutate {
            0 => second.forward_membership ^= 1,
            1 => second.reverse_membership ^= 1,
            2 => second.joint_membership ^= 1,
            3 => second.nominated_p += 1,
            4 => second.forward_interval = [121, 130],
            _ => unreachable!(),
        }
        slot.observe_sufficient(first);
        let StageBReproductionDecision::InconclusiveReproductionMismatch(delta) =
            slot.observe_sufficient(second)
        else {
            panic!("changed evidence must be a reproduction mismatch")
        };
        assert_eq!(delta.current_run_digest, 2);
        assert_eq!(slot.candidate(), Some(second));
    }
}

#[test]
fn reproduction_overlapping_intervals_match_without_requiring_equality() {
    let mut slot = StageBCandidateSlot::new();
    let first = summary(0, 1);
    let mut second = summary(0, 2);
    second.forward_interval = [110, 130];
    second.reverse_interval = [-110, -90];
    slot.observe_sufficient(first);
    assert!(matches!(
        slot.observe_sufficient(second),
        StageBReproductionDecision::Complete(_)
    ));
}

#[test]
fn reproduction_plan_mismatch_is_pre_motion_and_preserves_candidate() {
    let mut slot = StageBCandidateSlot::new();
    let retained = summary(0, 1);
    slot.observe_sufficient(retained);
    let requested = identity(0, 0x9999);
    assert!(matches!(
        slot.preflight(requested),
        Some(StageBReproductionDecision::RejectedPlanMismatch { .. })
    ));
    assert_eq!(slot.candidate(), Some(retained));
}

#[test]
fn reproduction_rejects_retained_pair_schema_before_n4_motion() {
    let mut slot = StageBCandidateSlot::new();
    let mut retained = summary(0, 1);
    retained.identity.schema_revision = 1;
    slot.observe_sufficient(retained);

    let requested = identity(0, retained.identity.plan_digest);
    let Some(StageBReproductionDecision::RejectedPlanMismatch {
        retained: retained_identity,
        requested: requested_identity,
    }) = slot.preflight(requested)
    else {
        panic!("the n=2 evidence schema must not compare with an n=4 plan")
    };
    assert_eq!(retained_identity.schema_revision, 1);
    assert_eq!(requested_identity.schema_revision, 2);
    assert_eq!(slot.candidate(), Some(retained));
}

#[test]
fn reproduction_channel_and_commissioning_invalidation_clear_ram_state() {
    let mut slot = StageBCandidateSlot::new();
    slot.observe_sufficient(summary(0, 1));
    assert_eq!(
        slot.invalidate_for_channel(1),
        StageBReproductionDecision::Cleared
    );
    assert!(slot.candidate().is_none());

    slot.observe_sufficient(summary(0, 2));
    assert_eq!(
        slot.invalidate_for_unrelated_commissioning(),
        StageBReproductionDecision::Cleared
    );
    assert!(slot.candidate().is_none());
}

#[test]
fn reproduction_insufficient_or_fault_clears_and_default_is_empty() {
    let mut slot = StageBCandidateSlot::default();
    assert!(slot.candidate().is_none());
    slot.observe_sufficient(summary(0, 1));
    slot.clear();
    assert!(slot.candidate().is_none());
}
