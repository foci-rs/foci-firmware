//! Exact depth-three interval consensus for one velocity-sweep direction.

use super::sweep_evidence::VelocityObservationClass;
use crate::autotune::EquivalentTorqueTargetInterval;

/// Number of same-direction observations consumed by the consensus rule.
pub const CONSENSUS_OBSERVATIONS_PER_DIRECTION: usize = 4;

const FULL_OBSERVATION_MASK: u8 = 0b1111;
const MAX_CONSENSUS_COMPONENTS: usize = 2;
const TRIPLES: [u8; 4] = [0b0111, 0b1011, 0b1101, 0b1110];

/// Directional consensus outcome for one complete gain rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum DirectionalConsensusClass {
    /// One connected depth-three component supports a model interval.
    EligibleConsensus,
    /// More than one component exists, so firmware cannot choose one.
    AmbiguousConsensus,
    /// No depth-three component exists.
    InsufficientConsensus,
    /// Fewer than four declared observations were collected.
    IncompleteRung,
}

/// Exact bounded consensus evidence for one direction and gain rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalConsensus {
    /// Consensus classification.
    pub classification: DirectionalConsensusClass,
    /// Exact connected components as inclusive signed Q16.16 raw bounds.
    pub components: [[i32; 2]; MAX_CONSENSUS_COMPONENTS],
    /// Number of populated entries in `components`.
    pub component_count: u8,
    /// Four-bit mask of observations collected by the declared schedule.
    pub collected_mask: u8,
    /// Four-bit mask of intrinsically eligible observations.
    pub eligible_mask: u8,
    /// Four-bit mask of eligible observations included in the unique component.
    pub included_mask: u8,
}

/// Compute the union of all closed three-observation intersections.
///
/// The four corrected observation intervals are the only statistical atoms.
/// This function performs exact deterministic set arithmetic and never changes
/// an observation's intrinsic classification.
pub fn depth_three_consensus(
    collected_mask: u8,
    classes: [VelocityObservationClass; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    intervals: [Option<EquivalentTorqueTargetInterval>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
) -> DirectionalConsensus {
    let collected_mask = collected_mask & FULL_OBSERVATION_MASK;
    let eligible_mask = classes
        .iter()
        .enumerate()
        .filter(|(index, class)| {
            collected_mask & (1u8 << index) != 0 && **class == VelocityObservationClass::Eligible
        })
        .fold(0u8, |mask, (index, _)| mask | (1u8 << index));

    if collected_mask != FULL_OBSERVATION_MASK {
        return empty_consensus(
            DirectionalConsensusClass::IncompleteRung,
            collected_mask,
            eligible_mask,
        );
    }
    if eligible_mask.count_ones() < 3 {
        return empty_consensus(
            DirectionalConsensusClass::InsufficientConsensus,
            collected_mask,
            eligible_mask,
        );
    }

    let mut intersections = [[0i32; 2]; TRIPLES.len()];
    let mut intersection_count = 0usize;
    for triple in TRIPLES {
        if let Some(intersection) = intersect_triple(triple, eligible_mask, intervals) {
            intersections[intersection_count] = intersection;
            intersection_count += 1;
        }
    }
    sort_intervals(&mut intersections, intersection_count);
    let (components, component_count) = merge_intersections(intersections, intersection_count);

    let classification = match component_count {
        0 => DirectionalConsensusClass::InsufficientConsensus,
        1 => DirectionalConsensusClass::EligibleConsensus,
        _ => DirectionalConsensusClass::AmbiguousConsensus,
    };
    let included_mask = if component_count == 1 {
        included_observations(eligible_mask, intervals, components[0])
    } else {
        0
    };

    DirectionalConsensus {
        classification,
        components,
        component_count: component_count as u8,
        collected_mask,
        eligible_mask,
        included_mask,
    }
}

/// Return the stable wire/report code for a consensus class.
pub const fn consensus_class_report_code(class: DirectionalConsensusClass) -> u8 {
    match class {
        DirectionalConsensusClass::EligibleConsensus => 0,
        DirectionalConsensusClass::AmbiguousConsensus => 1,
        DirectionalConsensusClass::InsufficientConsensus => 2,
        DirectionalConsensusClass::IncompleteRung => 3,
    }
}

fn empty_consensus(
    classification: DirectionalConsensusClass,
    collected_mask: u8,
    eligible_mask: u8,
) -> DirectionalConsensus {
    DirectionalConsensus {
        classification,
        components: [[0; 2]; MAX_CONSENSUS_COMPONENTS],
        component_count: 0,
        collected_mask,
        eligible_mask,
        included_mask: 0,
    }
}

fn intersect_triple(
    mask: u8,
    eligible_mask: u8,
    intervals: [Option<EquivalentTorqueTargetInterval>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
) -> Option<[i32; 2]> {
    if mask & eligible_mask != mask {
        return None;
    }
    let mut low = i32::MIN;
    let mut high = i32::MAX;
    for (index, interval) in intervals.iter().enumerate() {
        if mask & (1u8 << index) == 0 {
            continue;
        }
        let interval = interval.as_ref()?;
        low = low.max(interval.low().raw());
        high = high.min(interval.high().raw());
    }
    (low <= high).then_some([low, high])
}

fn sort_intervals(intervals: &mut [[i32; 2]; TRIPLES.len()], count: usize) {
    for index in 1..count {
        let mut current = index;
        while current > 0 && interval_precedes(intervals[current], intervals[current - 1]) {
            intervals.swap(current, current - 1);
            current -= 1;
        }
    }
}

const fn interval_precedes(first: [i32; 2], second: [i32; 2]) -> bool {
    first[0] < second[0] || (first[0] == second[0] && first[1] < second[1])
}

fn merge_intersections(
    intersections: [[i32; 2]; TRIPLES.len()],
    intersection_count: usize,
) -> ([[i32; 2]; MAX_CONSENSUS_COMPONENTS], usize) {
    let mut components = [[0i32; 2]; MAX_CONSENSUS_COMPONENTS];
    let mut component_count = 0usize;
    for intersection in intersections.into_iter().take(intersection_count) {
        if component_count == 0 {
            components[0] = intersection;
            component_count = 1;
            continue;
        }

        let previous = component_count - 1;
        if intersection[0] <= components[previous][1] {
            components[previous][1] = components[previous][1].max(intersection[1]);
            continue;
        }

        assert!(
            component_count < MAX_CONSENSUS_COMPONENTS,
            "four depth-three interval intersections have at most two components"
        );
        components[component_count] = intersection;
        component_count += 1;
    }
    (components, component_count)
}

fn included_observations(
    eligible_mask: u8,
    intervals: [Option<EquivalentTorqueTargetInterval>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    component: [i32; 2],
) -> u8 {
    intervals
        .iter()
        .enumerate()
        .filter(|(index, interval)| {
            if eligible_mask & (1u8 << index) == 0 {
                return false;
            }
            interval.is_some_and(|interval| {
                interval.low().raw() <= component[1] && component[0] <= interval.high().raw()
            })
        })
        .fold(0u8, |mask, (index, _)| mask | (1u8 << index))
}
