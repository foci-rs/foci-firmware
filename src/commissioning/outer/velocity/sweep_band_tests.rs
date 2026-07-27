use super::{
    BandClosure, DirectionalDisturbanceBand, JointIdentifiableGeometry, VelocityRungClass,
    VelocityRungVerdict, VelocitySweepSufficiency, assemble_directional_band,
    assess_sweep_sufficiency,
};
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
use crate::commissioning::outer::velocity::combined_plan::select_proportional_response;
use crate::commissioning::outer::velocity::operating_region::{BoundRounding, CompactSignedRate};
use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
use crate::commissioning::outer::velocity::sweep_evidence::CompactVariance;
use crate::commissioning::outer::velocity::sweep_evidence::{
    CollectedRungEvidence, CompactConsensusObservation, CompactDirectionalRungReport,
    VelocityObservationClass, VelocityObservationSummary, analyze_directional_rung,
};
use foci_tmc4671::pid::PidPGain;

fn interval(low: i32, high: i32) -> EquivalentTorqueTargetInterval {
    EquivalentTorqueTargetInterval::new(
        EquivalentTorqueTargetQ16::from_raw(low),
        EquivalentTorqueTargetQ16::from_raw(high),
    )
    .unwrap()
}

fn verdict(p: u16, class: VelocityRungClass, low: i32, high: i32) -> VelocityRungVerdict {
    let eligible = class == VelocityRungClass::Eligible;
    VelocityRungVerdict {
        direction: VelocityDirection::Forward,
        p: PidPGain::new(p).unwrap(),
        classification: class,
        disturbance_interval: eligible.then(|| interval(low, high)),
        disturbance_intervals: eligible
            .then(|| [Some(interval(low, high)); 2])
            .unwrap_or([None; 2]),
        disturbance_points: eligible
            .then(|| {
                let middle = low + (high - low) / 2;
                [Some(EquivalentTorqueTargetQ16::from_raw(middle)); 2]
            })
            .unwrap_or([None; 2]),
        disturbance_variances_q2: [CompactVariance::encode(4); 2],
        settled_rate_bounds_q: [
            CompactSignedRate::encode(1_000, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(1_001, BoundRounding::TowardPositiveInfinity),
            CompactSignedRate::encode(990, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(1_011, BoundRounding::TowardPositiveInfinity),
        ],
        reproduced_moving: matches!(
            class,
            VelocityRungClass::Eligible
                | VelocityRungClass::DeficitUnresolved
                | VelocityRungClass::CurrentHeadroom
        ),
        operable: matches!(
            class,
            VelocityRungClass::Eligible | VelocityRungClass::DeficitUnresolved
        ),
        initial_lead_in: false,
    }
}

fn reverse(mut rung: VelocityRungVerdict) -> VelocityRungVerdict {
    rung.direction = VelocityDirection::Reverse;
    rung
}

fn summary(slot: u8, class: VelocityObservationClass) -> VelocityObservationSummary {
    let eligible = class == VelocityObservationClass::Eligible;
    let direction = if slot.is_multiple_of(2) {
        VelocityDirection::Forward
    } else {
        VelocityDirection::Reverse
    };
    let low = 100 + i32::from(slot);
    let high = 120 + i32::from(slot);
    VelocityObservationSummary {
        direction,
        p: PidPGain::new(512).unwrap(),
        classification: class,
        settled_rate_interval_q: [1_000 + i64::from(slot), 1_010 + i64::from(slot)],
        settled_rate_mean_q: 1_005 + i64::from(slot),
        deficit_interval_q: [100, 200],
        disturbance: eligible.then(|| EquivalentTorqueTargetQ16::from_raw((low + high) / 2)),
        disturbance_interval: eligible.then(|| interval(low, high)),
        disturbance_variance_q2: 4,
        predicted_torque_target_abs: 100,
        initial_lead_in: slot == 0,
    }
}

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

#[test]
fn combined_selection_consumes_compact_consensus_not_model_membership() {
    let p724 = select_proportional_response(
        response_report(724, 11_562_496, 11_719_168),
        response_report(724, -11_218_944, -11_187_200),
        264,
        4_000,
    )
    .unwrap();
    let p1024 = select_proportional_response(
        response_report(1_024, 13_359_104, 13_492_224),
        response_report(1_024, -12_968_960, -12_921_856),
        264,
        4_000,
    )
    .unwrap();

    assert_eq!(p724, None);
    let selected = p1024.expect("P=1024 lies wholly inside the initial band");
    assert_eq!(selected.p.raw(), 1_024);
    assert_eq!(
        selected.response_intervals_q(),
        [[13_359_104, 13_492_224], [-12_968_960, -12_921_856],]
    );
}

#[test]
fn compact_consensus_collects_eight_slots_without_aliasing() {
    assert!(core::mem::size_of::<CompactConsensusObservation>() <= 16);
    assert_eq!(core::mem::size_of::<CollectedRungEvidence>(), 144);

    let mut collected = CollectedRungEvidence::new();
    for slot in 0u8..8 {
        collected
            .retain(slot, summary(slot, VelocityObservationClass::Eligible))
            .unwrap();
    }

    assert_eq!(collected.collected_mask(VelocityDirection::Forward), 0b1111);
    assert_eq!(collected.collected_mask(VelocityDirection::Reverse), 0b1111);
    for slot in 0u8..8 {
        let direction = if slot.is_multiple_of(2) {
            VelocityDirection::Forward
        } else {
            VelocityDirection::Reverse
        };
        let index = usize::from(slot / 2);
        assert_eq!(
            collected.classification(direction, index),
            Some(VelocityObservationClass::Eligible)
        );
        assert_eq!(
            collected
                .disturbance_interval(direction, index)
                .unwrap()
                .low()
                .raw(),
            100 + i32::from(slot)
        );
    }
}

#[test]
fn compact_consensus_preserves_ineligible_class_without_an_interval() {
    let mut collected = CollectedRungEvidence::new();
    collected
        .retain(0, summary(0, VelocityObservationClass::Inert))
        .unwrap();

    assert_eq!(
        collected.classification(VelocityDirection::Forward, 0),
        Some(VelocityObservationClass::Inert)
    );
    assert_eq!(
        collected.disturbance_interval(VelocityDirection::Forward, 0),
        None
    );
}

#[test]
fn compact_consensus_rate_bounds_ignore_excluded_minimum_and_maximum() {
    fn collected_with_outlier(point: i32) -> CollectedRungEvidence {
        let mut collected = CollectedRungEvidence::new();
        for (observation, slot) in [0u8, 2, 4, 6].into_iter().enumerate() {
            let mut item = summary(slot, VelocityObservationClass::Eligible);
            let (low, high, disturbance) = if observation == 3 {
                (point, point, point)
            } else {
                (0, 10, 5)
            };
            item.disturbance = Some(EquivalentTorqueTargetQ16::from_raw(disturbance));
            item.disturbance_interval = Some(interval(low, high));
            collected.retain(slot, item).unwrap();
        }
        collected
    }

    let low_outlier = analyze_directional_rung(
        &collected_with_outlier(1_000_000),
        VelocityDirection::Forward,
        PidPGain::new(512).unwrap(),
        176,
        4_000,
    )
    .unwrap();
    let high_outlier = analyze_directional_rung(
        &collected_with_outlier(-1_000_000),
        VelocityDirection::Forward,
        PidPGain::new(512).unwrap(),
        176,
        4_000,
    )
    .unwrap();

    assert_eq!(low_outlier.report.included_mask, 0b0111);
    assert_eq!(high_outlier.report.included_mask, 0b0111);
    assert_eq!(
        low_outlier.report.settled_rate_bounds_q,
        high_outlier.report.settled_rate_bounds_q
    );
}

#[test]
fn band_is_maximal_contiguous_and_never_stitches_after_closure() {
    let rungs = [
        verdict(100, VelocityRungClass::Inert, 0, 0),
        verdict(200, VelocityRungClass::Eligible, 90, 130),
        verdict(300, VelocityRungClass::Eligible, 100, 125),
        verdict(400, VelocityRungClass::Eligible, 105, 120),
        verdict(500, VelocityRungClass::DeficitUnresolved, 0, 0),
        verdict(600, VelocityRungClass::Eligible, 106, 118),
    ];
    let band = assemble_directional_band(VelocityDirection::Forward, &rungs);
    assert_eq!(band.p_low.unwrap().raw(), 200);
    assert_eq!(band.p_high.unwrap().raw(), 400);
    assert_eq!(band.eligible_rungs, 3);
    assert_eq!(band.closure, BandClosure::DeficitResolution);
    assert_eq!(band.common_intersection, Some(interval(105, 120)));
    assert!(band.sufficient);
}

#[test]
fn lower_edge_reports_left_censoring_and_ambiguity() {
    let left = [
        verdict(100, VelocityRungClass::Eligible, 90, 110),
        verdict(150, VelocityRungClass::Eligible, 95, 108),
        verdict(225, VelocityRungClass::Eligible, 98, 105),
    ];
    let left_band = assemble_directional_band(VelocityDirection::Forward, &left);
    assert!(left_band.left_censored);
    assert!(left_band.last_inert.is_none());
    let ambiguous = [
        verdict(100, VelocityRungClass::Inert, 0, 0),
        verdict(150, VelocityRungClass::Ambiguous, 0, 0),
        verdict(225, VelocityRungClass::Eligible, 90, 110),
        verdict(320, VelocityRungClass::Eligible, 95, 108),
        verdict(450, VelocityRungClass::Eligible, 98, 105),
    ];
    let band = assemble_directional_band(VelocityDirection::Forward, &ambiguous);
    assert_eq!(band.last_inert.unwrap().raw(), 100);
    assert_eq!(band.first_moving.unwrap().raw(), 225);
    assert_eq!(band.ambiguous_edge.unwrap().raw(), 150);
}

#[test]
fn empty_common_intersection_closes_without_stitching() {
    let rungs = [
        verdict(100, VelocityRungClass::Eligible, 90, 110),
        verdict(150, VelocityRungClass::Eligible, 100, 120),
        verdict(225, VelocityRungClass::Eligible, 111, 130),
        verdict(320, VelocityRungClass::Eligible, 101, 109),
    ];
    let band = assemble_directional_band(VelocityDirection::Forward, &rungs);
    assert_eq!(band.eligible_rungs, 2);
    assert_eq!(band.closure, BandClosure::Invariance);
    assert!(!band.sufficient);
}

#[test]
fn pooling_uses_one_unit_variance_floor_and_keeps_intersection_separate() {
    let mut precise = verdict(100, VelocityRungClass::Eligible, 80, 120);
    precise.disturbance_points = [Some(EquivalentTorqueTargetQ16::from_raw(100)); 2];
    precise.disturbance_variances_q2 = [CompactVariance::encode(0); 2];
    let mut noisy = verdict(150, VelocityRungClass::Eligible, 90, 210);
    noisy.disturbance_points = [Some(EquivalentTorqueTargetQ16::from_raw(200)); 2];
    noisy.disturbance_variances_q2 = [CompactVariance::encode(100); 2];
    let third = verdict(225, VelocityRungClass::Eligible, 95, 115);
    let band = assemble_directional_band(VelocityDirection::Forward, &[precise, noisy, third]);
    let pooled = band.pooled.unwrap();
    assert!(pooled.estimate.raw() < 120);
    assert_eq!(band.common_intersection, Some(interval(95, 115)));
    assert_ne!(pooled.interval, band.common_intersection.unwrap());
    assert_eq!(pooled.variance_floor_observations, 2);
}

fn sufficient_band(
    direction: VelocityDirection,
    low: u16,
    high: u16,
) -> DirectionalDisturbanceBand {
    let base = [
        verdict(low, VelocityRungClass::Eligible, 90, 110),
        verdict((low + high) / 2, VelocityRungClass::Eligible, 95, 108),
        verdict(high, VelocityRungClass::Eligible, 98, 105),
    ];
    let directed = base.map(|rung| {
        if direction == VelocityDirection::Reverse {
            reverse(rung)
        } else {
            rung
        }
    });
    assemble_directional_band(direction, &directed)
}

#[test]
fn joint_geometry_preserves_asymmetry_and_empty_intersection() {
    let forward = sufficient_band(VelocityDirection::Forward, 100, 300);
    let reverse_overlap = sufficient_band(VelocityDirection::Reverse, 200, 500);
    let reverse_gap = sufficient_band(VelocityDirection::Reverse, 400, 600);
    let overlap = JointIdentifiableGeometry::from_bands(&forward, &reverse_overlap);
    assert_eq!(overlap.low.unwrap().raw(), 200);
    assert_eq!(overlap.high.unwrap().raw(), 300);
    assert!(overlap.has_joint_band);
    let gap = JointIdentifiableGeometry::from_bands(&forward, &reverse_gap);
    assert!(!gap.has_joint_band);
    assert_eq!(gap.gap_low.unwrap().raw(), 300);
    assert_eq!(gap.gap_high.unwrap().raw(), 400);
}

#[test]
fn sufficiency_distinguishes_complete_partial_and_total_inconclusive() {
    let forward = sufficient_band(VelocityDirection::Forward, 100, 300);
    let reverse_band = sufficient_band(VelocityDirection::Reverse, 200, 500);
    let insufficient = assemble_directional_band(
        VelocityDirection::Reverse,
        &[reverse(verdict(100, VelocityRungClass::Inert, 0, 0))],
    );
    assert!(matches!(
        assess_sweep_sufficiency(&forward, &reverse_band),
        VelocitySweepSufficiency::Complete { .. }
    ));
    assert!(matches!(
        assess_sweep_sufficiency(&forward, &insufficient),
        VelocitySweepSufficiency::Inconclusive {
            sufficient_direction_mask: 1,
            ..
        }
    ));
    assert!(matches!(
        assess_sweep_sufficiency(&insufficient, &insufficient),
        VelocitySweepSufficiency::Inconclusive {
            sufficient_direction_mask: 0,
            ..
        }
    ));
}
