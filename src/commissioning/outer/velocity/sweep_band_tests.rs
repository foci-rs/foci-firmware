use super::{
    BandClosure, DirectionalDisturbanceBand, JointIdentifiableGeometry, VelocityRungClass,
    VelocityRungVerdict, VelocitySweepSufficiency, assemble_directional_band,
    assess_sweep_sufficiency,
};
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

fn verdict(p: u16, class: VelocityRungClass, low: i32, high: i32) -> VelocityRungVerdict {
    let eligible = class == VelocityRungClass::Eligible;
    VelocityRungVerdict {
        direction: VelocityDirection::Forward,
        p: PidGain::new(p).unwrap(),
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
        disturbance_variances_q2: [4, 4],
        reproduced_moving: matches!(
            class,
            VelocityRungClass::Eligible
                | VelocityRungClass::DeficitUnresolved
                | VelocityRungClass::CurrentHeadroom
        ),
        initial_lead_in: false,
    }
}

fn reverse(mut rung: VelocityRungVerdict) -> VelocityRungVerdict {
    rung.direction = VelocityDirection::Reverse;
    rung
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
    precise.disturbance_variances_q2 = [0, 0];
    let mut noisy = verdict(150, VelocityRungClass::Eligible, 90, 210);
    noisy.disturbance_points = [Some(EquivalentTorqueTargetQ16::from_raw(200)); 2];
    noisy.disturbance_variances_q2 = [100, 100];
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
