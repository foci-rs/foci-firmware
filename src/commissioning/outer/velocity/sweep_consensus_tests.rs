use super::sweep_consensus::{DirectionalConsensusClass, depth_three_consensus};
use super::sweep_evidence::VelocityObservationClass;
use crate::autotune::velocity_measurement::StationarityExclusion;
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};

fn interval(low: i32, high: i32) -> Option<EquivalentTorqueTargetInterval> {
    Some(
        EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(low),
            EquivalentTorqueTargetQ16::from_raw(high),
        )
        .unwrap(),
    )
}

fn consensus(
    collected_mask: u8,
    classes: [VelocityObservationClass; 4],
    intervals: [[i32; 2]; 4],
) -> super::sweep_consensus::DirectionalConsensus {
    depth_three_consensus(
        collected_mask,
        classes,
        intervals.map(|[low, high]| interval(low, high)),
    )
}

fn components(result: &super::sweep_consensus::DirectionalConsensus) -> &[[i32; 2]] {
    &result.components[..usize::from(result.component_count)]
}

#[test]
fn four_agree_over_the_depth_three_union() {
    let result = consensus(
        0b1111,
        [VelocityObservationClass::Eligible; 4],
        [[0, 10], [2, 9], [3, 8], [4, 7]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::EligibleConsensus
    );
    assert_eq!(components(&result), [[3, 8]]);
    assert_eq!(result.eligible_mask, 0b1111);
    assert_eq!(result.included_mask, 0b1111);
}

#[test]
fn one_disjoint_outlier_is_excluded_from_the_unique_component() {
    let result = consensus(
        0b1111,
        [VelocityObservationClass::Eligible; 4],
        [[0, 10], [2, 9], [3, 8], [20, 30]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::EligibleConsensus
    );
    assert_eq!(components(&result), [[3, 8]]);
    assert_eq!(result.eligible_mask, 0b1111);
    assert_eq!(result.included_mask, 0b0111);
}

#[test]
fn disconnected_depth_three_components_are_ambiguous() {
    let result = consensus(
        0b1111,
        [VelocityObservationClass::Eligible; 4],
        [[0, 4], [0, 10], [0, 10], [6, 10]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::AmbiguousConsensus
    );
    assert_eq!(components(&result), [[0, 4], [6, 10]]);
    assert_eq!(result.included_mask, 0);
}

#[test]
fn a_fourth_eligible_observation_can_split_a_settled_component() {
    let result = consensus(
        0b1111,
        [
            VelocityObservationClass::Eligible,
            VelocityObservationClass::Eligible,
            VelocityObservationClass::Eligible,
            VelocityObservationClass::StationarityExcluded(StationarityExclusion::ExcessiveDrift),
        ],
        [[0, 4], [0, 10], [0, 10], [6, 10]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::EligibleConsensus
    );
    assert_eq!(components(&result), [[0, 4]]);
}

#[test]
fn closed_endpoint_components_merge_to_one_point() {
    let result = consensus(
        0b1111,
        [VelocityObservationClass::Eligible; 4],
        [[0, 5], [5, 10], [0, 5], [5, 10]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::EligibleConsensus
    );
    assert_eq!(components(&result), [[5, 5]]);
    assert_eq!(result.included_mask, 0b1111);
}

#[test]
fn exactly_three_eligible_observations_require_their_common_intersection() {
    let result = consensus(
        0b1111,
        [
            VelocityObservationClass::Eligible,
            VelocityObservationClass::Eligible,
            VelocityObservationClass::Eligible,
            VelocityObservationClass::Inert,
        ],
        [[0, 10], [2, 9], [3, 8], [4, 7]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::EligibleConsensus
    );
    assert_eq!(components(&result), [[3, 8]]);
    assert_eq!(result.eligible_mask, 0b0111);
    assert_eq!(result.included_mask, 0b0111);
}

#[test]
fn two_eligible_observations_remain_insufficient() {
    let result = consensus(
        0b1111,
        [
            VelocityObservationClass::Eligible,
            VelocityObservationClass::Eligible,
            VelocityObservationClass::Inert,
            VelocityObservationClass::Inert,
        ],
        [[0, 10], [2, 9], [3, 8], [4, 7]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::InsufficientConsensus
    );
    assert_eq!(result.eligible_mask, 0b0011);
    assert_eq!(result.included_mask, 0);
}

#[test]
fn three_collected_observations_are_an_incomplete_rung() {
    let result = consensus(
        0b0111,
        [VelocityObservationClass::Eligible; 4],
        [[0, 10], [2, 9], [3, 8], [4, 7]],
    );

    assert_eq!(
        result.classification,
        DirectionalConsensusClass::IncompleteRung
    );
    assert_eq!(result.collected_mask, 0b0111);
    assert_eq!(result.included_mask, 0);
}
