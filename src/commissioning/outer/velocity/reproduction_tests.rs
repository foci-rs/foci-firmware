use super::{
    STAGE_B_BUILD_REVISION, STAGE_B_EVIDENCE_SCHEMA_REVISION,
    STAGE_B_REPRO_REASON_FORWARD_CORE_TOO_SMALL, STAGE_B_REPRO_REASON_FORWARD_POOLED_DISJOINT,
    STAGE_B_REPRO_REASON_JOINT_CORE_EMPTY, STAGE_B_REPRO_REASON_REVERSE_CORE_TOO_SMALL,
    STAGE_B_REPRO_REASON_REVERSE_POOLED_DISJOINT, StageBCandidateSlot, StageBCandidateSummary,
    StageBDirectionalValidity, StageBPlanIdentity, StageBReproductionDecision, const_fnv1a32,
    summary_geometry_valid,
};
use crate::commissioning::outer::velocity::operating_region::CoverageKind;

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
        forward_membership: 0b1_1100,
        reverse_membership: 0b1_1100,
        joint_membership: 0b1_1100,
        nominated_p: 724,
        p_start: 256,
        p_top: 1_024,
        rung_count: 5,
        forward_interval: [100, 120],
        reverse_interval: [-130, -100],
        forward_common_interval: [104, 115],
        reverse_common_interval: [-125, -109],
        run_completed_us: run_digest * 1_000 + 40_000,
        run_digest,
    }
}

fn compare(
    previous: StageBCandidateSummary,
    current: StageBCandidateSummary,
) -> StageBReproductionDecision {
    let mut slot = StageBCandidateSlot::new();
    assert!(matches!(
        slot.observe_sufficient(previous),
        StageBReproductionDecision::CompleteCandidate(_)
    ));
    slot.observe_sufficient(current)
}

#[test]
fn reproduction_fnv_vectors_and_build_revision_are_stable() {
    assert_eq!(const_fnv1a32(b""), 0x811c_9dc5);
    assert_eq!(const_fnv1a32(b"a"), 0xe40c_292c);
    assert_eq!(STAGE_B_EVIDENCE_SCHEMA_REVISION, 8);
    assert_eq!(STAGE_B_BUILD_REVISION, 0x05e2_1f75);
}

#[test]
fn q8_8_package_candidate_rejects_before_current_build_comparison() {
    let mut slot = StageBCandidateSlot::new();
    let mut retained = summary(0, 1);
    retained.identity.build_revision = 0x7553_6758;
    slot.observe_sufficient(retained);

    let requested = identity(0, retained.identity.plan_digest);
    assert!(matches!(
        slot.preflight(requested),
        Some(StageBReproductionDecision::RejectedPlanMismatch { .. })
    ));
    assert_eq!(slot.candidate(), Some(retained));
}

#[test]
fn reproduction_core_exact_membership_has_zero_boundary_evidence() {
    let StageBReproductionDecision::Complete(delta) = compare(summary(0, 1), summary(0, 2)) else {
        panic!("exact memberships must reproduce")
    };
    for membership in delta.memberships {
        assert_eq!(membership.previous, membership.current);
        assert_eq!(membership.core, membership.previous);
        assert_eq!(membership.previous_only, 0);
        assert_eq!(membership.current_only, 0);
        assert_eq!(membership.low_delta, 0);
        assert_eq!(membership.high_delta, 0);
    }
    assert_eq!(delta.final_p, 724);
    assert_eq!(delta.selected_joint.member_mask, 0b1_1100);
    assert_eq!(delta.selected_joint.first_rung, 2);
    assert_eq!(delta.selected_joint.member_count, 3);
    assert_eq!(
        delta.directional_validity,
        [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2]
    );
    assert_eq!(delta.reason_mask, 0);
}

fn split_union_summary(run_digest: u64) -> StageBCandidateSummary {
    let mut value = summary(0, run_digest);
    value.forward_membership = 0x001c_0000;
    value.reverse_membership = 0x0018_0000;
    value.joint_membership = 0x0019_8000;
    value.nominated_p = 256;
    value.p_start = 1;
    value.p_top = 2_457;
    value.rung_count = 23;
    value
}

#[test]
fn summary_geometry_valid_accepts_noncontiguous_joint_union() {
    assert!(summary_geometry_valid(split_union_summary(1)));
}

#[test]
fn reproduction_delta_splits_noncontiguous_joint_core_before_nomination() {
    let union = 0x0019_8000_u32;
    let forbidden_first = union.trailing_zeros() as u8;
    let forbidden_count = union.count_ones() as u8;
    let forbidden_rung = forbidden_first + (forbidden_count - 1) / 2;
    assert_eq!(forbidden_rung, 16);
    assert_eq!(
        crate::commissioning::outer::velocity::sweep_plan::gain_at(1, 2_457, 23, forbidden_rung,)
            .unwrap()
            .raw(),
        362
    );

    let StageBReproductionDecision::Complete(delta) =
        compare(split_union_summary(1), split_union_summary(2))
    else {
        panic!("the shared upper component must reproduce independently")
    };
    assert_eq!(delta.memberships[2].core, union);
    assert_eq!(delta.selected_joint.member_mask, 0x0018_0000);
    assert_eq!(delta.selected_joint.first_rung, 19);
    assert_eq!(delta.selected_joint.member_count, 2);
    assert_eq!(delta.final_p, 1_024);
    assert!(delta.reduced_margin);
    assert_eq!(
        delta.directional_validity,
        [
            StageBDirectionalValidity::ReproducedRegionCoversFinalP,
            StageBDirectionalValidity::Fragment,
        ]
    );
}

#[test]
fn reproduction_core_edge_expansion_and_p_flap_resolve_from_joint_core() {
    let previous = summary(0, 1);
    let mut current = summary(0, 2);
    current.forward_membership = 0b1_1110;
    current.joint_membership = 0b1_1110;
    current.nominated_p = 512;

    let StageBReproductionDecision::Complete(delta) = compare(previous, current) else {
        panic!("a sufficient shared core must reproduce")
    };
    assert_eq!(delta.memberships[0].core, 0b1_1100);
    assert_eq!(delta.memberships[0].previous_only, 0);
    assert_eq!(delta.memberships[0].current_only, 0b10);
    assert_eq!(delta.memberships[0].low_delta, -1);
    assert_eq!(delta.memberships[0].high_delta, 0);
    assert_eq!(delta.memberships[2].core, 0b1_1100);
    assert_eq!(delta.previous_provisional_p, 724);
    assert_eq!(delta.current_provisional_p, 512);
    assert_eq!(delta.final_p, 724);
}

#[test]
fn reproduction_core_reports_every_independent_mismatch_reason() {
    let previous = summary(0, 1);
    let mut current = summary(0, 2);
    current.forward_membership = 0b1_1000;
    current.reverse_membership = 0b1_1000;
    current.joint_membership = 0b11;
    current.forward_interval = [121, 130];
    current.reverse_interval = [-99, -90];

    let StageBReproductionDecision::InconclusiveReproductionMismatch(delta) =
        compare(previous, current)
    else {
        panic!("all five independent failures must mismatch")
    };
    assert_eq!(
        delta.reason_mask,
        STAGE_B_REPRO_REASON_FORWARD_CORE_TOO_SMALL
            | STAGE_B_REPRO_REASON_REVERSE_CORE_TOO_SMALL
            | STAGE_B_REPRO_REASON_JOINT_CORE_EMPTY
            | STAGE_B_REPRO_REASON_FORWARD_POOLED_DISJOINT
            | STAGE_B_REPRO_REASON_REVERSE_POOLED_DISJOINT
    );
}

#[test]
fn reproduction_core_reports_each_mismatch_reason_independently() {
    let previous = summary(0, 1);
    let cases = [
        (
            {
                let mut current = summary(0, 2);
                current.forward_membership = 0b1_1000;
                current
            },
            STAGE_B_REPRO_REASON_FORWARD_CORE_TOO_SMALL,
            0,
            StageBDirectionalValidity::Fragment,
        ),
        (
            {
                let mut current = summary(0, 2);
                current.reverse_membership = 0b1_1000;
                current
            },
            STAGE_B_REPRO_REASON_REVERSE_CORE_TOO_SMALL,
            1,
            StageBDirectionalValidity::Fragment,
        ),
        (
            {
                let mut current = summary(0, 2);
                current.forward_interval = [121, 130];
                current
            },
            STAGE_B_REPRO_REASON_FORWARD_POOLED_DISJOINT,
            0,
            StageBDirectionalValidity::RegionNotReproduced,
        ),
        (
            {
                let mut current = summary(0, 2);
                current.reverse_interval = [-99, -90];
                current
            },
            STAGE_B_REPRO_REASON_REVERSE_POOLED_DISJOINT,
            1,
            StageBDirectionalValidity::RegionNotReproduced,
        ),
    ];

    for (current, expected_reason, direction, expected_validity) in cases {
        let StageBReproductionDecision::Complete(delta) = compare(previous, current) else {
            panic!("directional model failure must remain non-gating evidence")
        };
        assert_eq!(delta.reason_mask, expected_reason);
        assert_eq!(delta.directional_validity[direction], expected_validity);
    }

    let mut no_joint = summary(0, 2);
    no_joint.joint_membership = 0b11;
    let StageBReproductionDecision::InconclusiveReproductionMismatch(delta) =
        compare(previous, no_joint)
    else {
        panic!("empty shared joint operability must remain a mismatch")
    };
    assert_eq!(delta.reason_mask, STAGE_B_REPRO_REASON_JOINT_CORE_EMPTY);
}

#[test]
fn reproduction_core_empty_common_is_reported_but_does_not_gate() {
    let previous = summary(0, 1);
    let mut current = summary(0, 2);
    current.forward_common_interval = [120, 130];

    let StageBReproductionDecision::Complete(delta) = compare(previous, current) else {
        panic!("common-interval emptiness is non-gating")
    };
    assert_eq!(delta.intervals[0].conservative_common, [120, 115]);
    assert!(!delta.intervals[0].conservative_common_nonempty);
}

#[test]
fn reproduction_core_one_rung_joint_core_has_reduced_margin() {
    let mut previous = summary(0, 1);
    let mut current = summary(0, 2);
    previous.joint_membership = 0b0011;
    current.joint_membership = 0b0110;

    let StageBReproductionDecision::Complete(delta) = compare(previous, current) else {
        panic!("a one-rung joint core remains operable")
    };
    assert_eq!(delta.memberships[2].core, 0b0010);
    assert_eq!(delta.selected_joint.member_mask, 0b0010);
    assert_eq!(delta.selected_joint.first_rung, 1);
    assert_eq!(delta.selected_joint.member_count, 1);
    assert_eq!(delta.final_p, 362);
    assert!(delta.reduced_margin);
}

#[test]
fn reproduction_core_recomputes_full_and_transferred_coverage() {
    let mut previous = summary(0, 1);
    let mut current = summary(0, 2);
    previous.p_top = 2_048;
    previous.rung_count = 7;
    current.p_top = 2_048;
    current.rung_count = 7;
    previous.reverse_membership = 0b111_0000;
    current.reverse_membership = 0b111_0000;

    let StageBReproductionDecision::Complete(delta) = compare(previous, current) else {
        panic!("transferred coverage is evidence, not a gate")
    };
    assert_eq!(delta.coverage[0].kind, CoverageKind::FullCoverage);
    assert_eq!(delta.coverage[0].signed_rung_distance, 0);
    assert_eq!(delta.coverage[1].kind, CoverageKind::TransferredCoverage);
    assert_eq!(
        delta.directional_validity[1],
        StageBDirectionalValidity::ReproducedRegionTransferred
    );
    assert_eq!(delta.coverage[1].signed_rung_distance, 1);
    assert_eq!(
        (
            delta.coverage[1].gain_ratio_num,
            delta.coverage[1].gain_ratio_den
        ),
        (1_024, 724)
    );
}

#[test]
fn reproduction_core_integrity_fault_clears_noncontiguous_evidence() {
    let previous = summary(0, 1);
    let mut current = summary(0, 2);
    current.forward_membership = 0b1_0100;
    let mut slot = StageBCandidateSlot::new();
    slot.observe_sufficient(previous);

    assert_eq!(
        slot.observe_sufficient(current),
        StageBReproductionDecision::EvidenceIntegrityFault
    );
    assert!(slot.candidate().is_none());
}

#[test]
fn reproduction_core_mismatch_replaces_candidate_and_complete_clears_it() {
    let mut slot = StageBCandidateSlot::new();
    let first = summary(0, 1);
    let mut mismatch = summary(0, 2);
    mismatch.joint_membership = 0b11;
    slot.observe_sufficient(first);
    assert!(matches!(
        slot.observe_sufficient(mismatch),
        StageBReproductionDecision::InconclusiveReproductionMismatch(_)
    ));
    assert_eq!(slot.candidate(), Some(mismatch));

    let mut matching = summary(0, 3);
    matching.joint_membership = 0b11;
    assert!(matches!(
        slot.observe_sufficient(matching),
        StageBReproductionDecision::Complete(_)
    ));
    assert!(slot.candidate().is_none());
}

#[test]
fn reproduction_directional_validity_reports_no_region_and_endpoint_touch() {
    let mut no_region = summary(0, 1);
    no_region.forward_membership = 0;
    no_region.forward_interval = [0, 0];
    no_region.forward_common_interval = [0, 0];
    let StageBReproductionDecision::Complete(delta) = compare(no_region, summary(0, 2)) else {
        panic!("absent directional evidence must not gate joint authority")
    };
    assert_eq!(
        delta.directional_validity[0],
        StageBDirectionalValidity::NoRegion
    );
    assert_eq!(delta.memberships[0].low_delta, 0);
    assert_eq!(delta.memberships[0].high_delta, 0);

    let previous = summary(0, 3);
    let mut current = summary(0, 4);
    current.forward_interval = [120, 130];
    let StageBReproductionDecision::Complete(delta) = compare(previous, current) else {
        panic!("closed intervals touching at one endpoint reproduce")
    };
    assert_eq!(delta.intervals[0].pooled_overlap, [120, 120]);
    assert_eq!(
        delta.directional_validity[0],
        StageBDirectionalValidity::ReproducedRegionCoversFinalP
    );
}

#[test]
fn reproduction_completion_timestamp_is_context_not_a_match_predicate() {
    let first = summary(0, 1);
    let mut second = summary(0, 2);
    second.run_completed_us = 9_999_999;
    assert!(matches!(
        compare(first, second),
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
fn reproduction_rejects_schema_seven_candidate_before_motion() {
    let mut slot = StageBCandidateSlot::new();
    let mut retained = summary(0, 1);
    retained.identity.schema_revision = 7;
    slot.observe_sufficient(retained);

    let requested = identity(0, retained.identity.plan_digest);
    let Some(StageBReproductionDecision::RejectedPlanMismatch {
        retained: retained_identity,
        requested: requested_identity,
    }) = slot.preflight(requested)
    else {
        panic!("schema-7 evidence must not compare with schema 8")
    };
    assert_eq!(retained_identity.schema_revision, 7);
    assert_eq!(requested_identity.schema_revision, 8);
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
fn reproduction_slot_stays_within_the_fixed_board_allowance() {
    assert_eq!(core::mem::size_of::<StageBCandidateSlot>(), 96);
}
