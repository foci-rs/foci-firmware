use super::operating_region::{
    BoundRounding, CompactSignedRate, CoverageKind, DirectionalRegionClosure,
    DirectionalRegionFold, DirectionalRegionKind, DirectionalRegionRef, HandoffTrackers,
    JointOperableClosure, JointOperableFold, MIN_DIRECTIONAL_REGION_RUNGS, choose_joint_nomination,
    observe_directional_rung, observe_joint_rung, select_directional_region,
};
use super::sweep_evidence::{VelocityRungClass, VelocityRungVerdict};
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
use foci_tmc4671::pid::PidGain;

fn interval(low: i32, high: i32) -> EquivalentTorqueTargetInterval {
    EquivalentTorqueTargetInterval::new(
        EquivalentTorqueTargetQ16::from_raw(low),
        EquivalentTorqueTargetQ16::from_raw(high),
    )
    .unwrap()
}

fn verdict(p: u16, low: i32, high: i32, mean: i64) -> VelocityRungVerdict {
    VelocityRungVerdict {
        direction: VelocityDirection::Forward,
        p: PidGain::new(p).unwrap(),
        classification: VelocityRungClass::Eligible,
        disturbance_interval: Some(interval(low, high)),
        disturbance_intervals: [Some(interval(low, high)); 2],
        disturbance_points: [Some(EquivalentTorqueTargetQ16::from_raw((low + high) / 2)); 2],
        disturbance_variances_q2: [4, 4],
        settled_rate_means_q: [mean - 1, mean + 1],
        settled_rate_intervals_q: [[mean - 3, mean + 2], [mean - 2, mean + 3]],
        reproduced_moving: true,
        operable: true,
        initial_lead_in: false,
    }
}

fn ineligible(p: u16, class: VelocityRungClass) -> VelocityRungVerdict {
    let mut rung = verdict(p, 0, 1, 0);
    rung.classification = class;
    rung.disturbance_interval = None;
    rung.disturbance_intervals = [None; 2];
    rung.disturbance_points = [None; 2];
    rung
}

#[test]
fn compact_signed_rate_encloses_positive_and_negative_bounds() {
    for value in [
        i64::MIN / 2,
        -32_769,
        -32_768,
        -1,
        0,
        1,
        32_767,
        32_768,
        i64::MAX / 2,
    ] {
        let low = CompactSignedRate::encode(value, BoundRounding::TowardNegativeInfinity);
        let high = CompactSignedRate::encode(value, BoundRounding::TowardPositiveInfinity);
        assert!(low.decode() <= value, "lower bound missed {value}");
        assert!(high.decode() >= value, "upper bound missed {value}");
    }
}

#[test]
fn fold_seeds_appends_and_reports_exact_region_evidence() {
    let mut fold = DirectionalRegionFold::new(VelocityDirection::Forward);
    assert!(observe_directional_rung(&mut fold, 2, verdict(256, 90, 120, 10_000)).is_none());
    assert!(observe_directional_rung(&mut fold, 3, verdict(362, 95, 115, 10_010)).is_none());
    assert!(observe_directional_rung(&mut fold, 4, verdict(512, 100, 110, 9_990)).is_none());

    let region = fold.finish(DirectionalRegionClosure::EndOfLadder).unwrap();

    assert_eq!(region.kind, DirectionalRegionKind::Valid);
    assert_eq!(region.member_count, MIN_DIRECTIONAL_REGION_RUNGS);
    assert_eq!(region.member_mask, 0b1_1100);
    assert_eq!((region.first_rung, region.last_rung), (2, 4));
    assert_eq!((region.p_low.raw(), region.p_high.raw()), (256, 512));
    assert_eq!(region.common_intersection, interval(100, 110));
    assert_eq!(region.tested_intersection, interval(100, 110));
    assert_eq!(
        (
            region.settled_rate_mean_min_q,
            region.settled_rate_mean_max_q
        ),
        (9_989, 10_011)
    );
    assert_eq!(
        (
            region.settled_rate_envelope_low_q,
            region.settled_rate_envelope_high_q
        ),
        (9_987, 10_013)
    );
    assert!(region.pooled.estimate.raw() >= 100);
    assert!(region.pooled.estimate.raw() <= 105);
}

#[test]
fn narrowed_intersection_closes_and_disjoint_rung_reseeds_immediately() {
    let mut fold = DirectionalRegionFold::new(VelocityDirection::Forward);
    assert!(observe_directional_rung(&mut fold, 0, verdict(100, 0, 10, 1)).is_none());
    assert!(observe_directional_rung(&mut fold, 1, verdict(150, 5, 15, 2)).is_none());

    let closed = observe_directional_rung(&mut fold, 2, verdict(225, 11, 20, 3)).unwrap();

    assert_eq!(closed.kind, DirectionalRegionKind::Fragment);
    assert_eq!(closed.member_mask, 0b0011);
    assert_eq!(closed.common_intersection, interval(5, 10));
    assert_eq!(closed.tested_intersection, interval(5, 10));
    assert_eq!(closed.closure, DirectionalRegionClosure::DisjointInterval);
    assert_eq!(closed.boundary_rung, Some(2));
    let reseeded = fold.finish(DirectionalRegionClosure::EndOfLadder).unwrap();
    assert_eq!(reseeded.member_mask, 0b0100);
}

#[test]
fn ineligible_rung_separates_candidates_and_preserves_fragments() {
    let mut fold = DirectionalRegionFold::new(VelocityDirection::Forward);
    assert!(observe_directional_rung(&mut fold, 0, verdict(100, 0, 10, 1)).is_none());
    let first = observe_directional_rung(
        &mut fold,
        1,
        ineligible(150, VelocityRungClass::DeficitUnresolved),
    )
    .unwrap();
    assert_eq!(first.member_mask, 0b0001);
    assert_eq!(first.closure, DirectionalRegionClosure::IneligibleGap);
    assert_eq!(first.boundary_rung, Some(1));
    assert!(observe_directional_rung(&mut fold, 2, verdict(225, 20, 30, 3)).is_none());
    assert!(observe_directional_rung(&mut fold, 3, verdict(320, 22, 28, 4)).is_none());
    let second = fold.finish(DirectionalRegionClosure::EndOfLadder).unwrap();
    assert_eq!(second.member_mask, 0b1100);
    assert_eq!(second.kind, DirectionalRegionKind::Fragment);
}

#[test]
fn current_and_fault_termini_are_distinct_closures() {
    let mut current = DirectionalRegionFold::new(VelocityDirection::Forward);
    assert!(observe_directional_rung(&mut current, 0, verdict(100, 0, 10, 1)).is_none());
    let closed = observe_directional_rung(
        &mut current,
        1,
        ineligible(150, VelocityRungClass::CurrentHeadroom),
    )
    .unwrap();
    assert_eq!(closed.closure, DirectionalRegionClosure::CurrentHeadroom);

    let mut fault = DirectionalRegionFold::new(VelocityDirection::Forward);
    assert!(observe_directional_rung(&mut fault, 0, verdict(100, 0, 10, 1)).is_none());
    let closed =
        observe_directional_rung(&mut fault, 1, ineligible(150, VelocityRungClass::Fault)).unwrap();
    assert_eq!(closed.closure, DirectionalRegionClosure::SafetyTerminus);
}

#[test]
fn first_fit_counterexample_remains_two_fragments() {
    let intervals = [(0, 2), (1, 10), (9, 11), (10, 12)];
    let mut fold = DirectionalRegionFold::new(VelocityDirection::Forward);
    let mut closed = [None, None];
    for (index, (low, high)) in intervals.into_iter().enumerate() {
        if let Some(region) = observe_directional_rung(
            &mut fold,
            index as u8,
            verdict(100 + index as u16, low, high, index as i64),
        ) {
            closed[0] = Some(region);
        }
    }
    closed[1] = fold.finish(DirectionalRegionClosure::EndOfLadder);
    let closed = closed.map(Option::unwrap);

    assert_eq!(closed[0].member_mask, 0b0011);
    assert_eq!(closed[1].member_mask, 0b1100);
    assert!(
        closed
            .iter()
            .all(|region| region.kind == DirectionalRegionKind::Fragment)
    );
}

fn gains() -> [PidGain; 7] {
    [256, 362, 512, 724, 1024, 1448, 2048].map(|raw| PidGain::new(raw).unwrap())
}

#[test]
fn joint_operability_ignores_deficit_resolution_but_rejects_limits_and_faults() {
    let mut fold = JointOperableFold::new();
    let mut forward = ineligible(256, VelocityRungClass::DeficitUnresolved);
    let mut reverse = forward;
    reverse.direction = VelocityDirection::Reverse;
    forward.operable = true;
    reverse.operable = true;
    assert!(observe_joint_rung(&mut fold, 0, forward, reverse).is_none());

    let mut current = forward;
    current.classification = VelocityRungClass::CurrentHeadroom;
    current.operable = false;
    let closed = observe_joint_rung(&mut fold, 1, current, reverse).unwrap();
    assert_eq!(closed.member_mask, 0b1);
    assert_eq!(closed.closure, JointOperableClosure::CurrentHeadroom);

    let mut fault_fold = JointOperableFold::new();
    assert!(observe_joint_rung(&mut fault_fold, 0, forward, reverse).is_none());
    let mut fault = reverse;
    fault.classification = VelocityRungClass::Fault;
    fault.operable = false;
    let closed = observe_joint_rung(&mut fault_fold, 1, forward, fault).unwrap();
    assert_eq!(closed.closure, JointOperableClosure::SafetyTerminus);
}

#[test]
fn joint_fold_reports_maximal_contiguous_regions_and_single_rung_margin() {
    let mut fold = JointOperableFold::new();
    let mut forward = verdict(256, 0, 10, 1);
    let mut reverse = forward;
    reverse.direction = VelocityDirection::Reverse;
    for index in 0..3 {
        forward.p = gains()[index];
        reverse.p = gains()[index];
        assert!(observe_joint_rung(&mut fold, index as u8, forward, reverse).is_none());
    }
    let closed = fold.finish(JointOperableClosure::EndOfLadder).unwrap();
    assert_eq!(closed.member_mask, 0b111);
    assert_eq!(
        (closed.first_rung, closed.last_rung, closed.member_count),
        (0, 2, 3)
    );

    let nomination = choose_joint_nomination(None, closed, &gains(), gains()[0], gains()[6]);
    assert_eq!(nomination.rung_index, 1);
    assert_eq!(nomination.p.raw(), 362);
    assert!(!nomination.reduced_margin);

    let single = super::operating_region::JointOperableRegion {
        member_mask: 1 << 4,
        first_rung: 4,
        last_rung: 4,
        member_count: 1,
        closure: JointOperableClosure::EndOfLadder,
    };
    assert!(choose_joint_nomination(None, single, &gains(), gains()[0], gains()[6]).reduced_margin);
}

#[test]
fn joint_nomination_uses_count_ratio_distance_and_lower_p_ties() {
    let candidates = [
        super::operating_region::JointOperableRegion {
            member_mask: 0b000_0111,
            first_rung: 0,
            last_rung: 2,
            member_count: 3,
            closure: JointOperableClosure::IneligibleGap,
        },
        super::operating_region::JointOperableRegion {
            member_mask: 0b111_0000,
            first_rung: 4,
            last_rung: 6,
            member_count: 3,
            closure: JointOperableClosure::EndOfLadder,
        },
        super::operating_region::JointOperableRegion {
            member_mask: 0b001_1110,
            first_rung: 1,
            last_rung: 4,
            member_count: 4,
            closure: JointOperableClosure::IneligibleGap,
        },
    ];
    let mut selected = None;
    for candidate in candidates {
        selected = Some(choose_joint_nomination(
            selected,
            candidate,
            &gains(),
            gains()[0],
            gains()[6],
        ));
    }
    let selected = selected.unwrap();
    assert_eq!(selected.joint.member_count, 4);
    assert_eq!(selected.rung_index, 2);

    let lower = super::operating_region::JointOperableRegion {
        member_mask: 0b000_1100,
        first_rung: 2,
        last_rung: 3,
        member_count: 2,
        closure: JointOperableClosure::IneligibleGap,
    };
    let upper = super::operating_region::JointOperableRegion {
        member_mask: 0b011_0000,
        first_rung: 4,
        last_rung: 5,
        member_count: 2,
        closure: JointOperableClosure::EndOfLadder,
    };
    let lower_nomination = choose_joint_nomination(None, lower, &gains(), gains()[0], gains()[6]);
    let tied = choose_joint_nomination(
        Some(lower_nomination),
        upper,
        &gains(),
        gains()[0],
        gains()[6],
    );
    assert_eq!(tied.joint.member_mask, lower.member_mask);
}

fn reference(mask: u32, low: i32, high: i32) -> DirectionalRegionRef {
    DirectionalRegionRef {
        member_mask: mask,
        pooled_interval: [low, high],
        selected_edge_rate: CompactSignedRate::encode(10, BoundRounding::TowardPositiveInfinity),
    }
}

#[test]
fn handoff_selection_prefers_containment_then_nearest_then_lower_region() {
    let lower = reference(0b000_0110, 1, 2);
    let containing = reference(0b001_1100, 3, 4);
    let upper = reference(0b110_0000, 5, 6);

    let selected = select_directional_region(None, lower, 3, &gains(), 0).unwrap();
    assert_eq!(selected.coverage, CoverageKind::TransferredCoverage);
    assert_eq!(
        (selected.gain_ratio_num, selected.gain_ratio_den),
        (724, 512)
    );
    assert_eq!(selected.settled_rate_difference_q, 10);
    let selected = select_directional_region(Some(selected), upper, 3, &gains(), 0).unwrap();
    assert_eq!(selected.region.member_mask, lower.member_mask);
    let selected = select_directional_region(Some(selected), containing, 3, &gains(), 0).unwrap();
    assert_eq!(selected.coverage, CoverageKind::FullCoverage);
    assert_eq!(selected.region.member_mask, containing.member_mask);
}

#[test]
fn handoff_trackers_promote_later_joint_candidate_without_full_regions() {
    let mut trackers = HandoffTrackers::new();
    let first = reference(0b000_0111, 1, 2);
    let later = reference(0b111_0000, 3, 4);
    trackers.observe_directional(VelocityDirection::Forward, first, 1, 1);
    trackers.observe_directional(VelocityDirection::Reverse, first, 1, 1);
    trackers.promote_active();
    trackers.begin_active_nomination(5);
    trackers.observe_directional(VelocityDirection::Forward, later, 5, 1);
    trackers.observe_directional(VelocityDirection::Reverse, later, 5, 1);
    trackers.promote_active();

    assert_eq!(
        trackers
            .best(VelocityDirection::Forward, 5, &gains(), 0)
            .unwrap()
            .region
            .member_mask,
        later.member_mask
    );
    assert_eq!(
        trackers
            .best(VelocityDirection::Reverse, 5, &gains(), 0)
            .unwrap()
            .region
            .member_mask,
        later.member_mask
    );

    let joint = super::operating_region::JointOperableRegion {
        member_mask: 0b111_0000,
        first_rung: 4,
        last_rung: 6,
        member_count: 3,
        closure: JointOperableClosure::EndOfLadder,
    };
    let nomination = choose_joint_nomination(None, joint, &gains(), gains()[0], gains()[6]);
    let handoff = trackers.handoff(nomination, &gains(), [0, 0]).unwrap();
    assert_eq!(handoff.coverage_mask, 3);
}
