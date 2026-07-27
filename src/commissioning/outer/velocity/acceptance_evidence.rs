//! Exact per-amplitude evidence for fixed-I velocity confidence matrices.

use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    depth_three_consensus,
};
use super::sweep_evidence::VelocityObservationClass;

/// Number of amplitudes in one immutable matrix.
pub const ACCEPTANCE_AMPLITUDE_COUNT: usize = 5;
const FULL_AMPLITUDE_MASK: u8 = (1 << ACCEPTANCE_AMPLITUDE_COUNT) - 1;

/// Availability of one amplitude and direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum AcceptanceAmplitudeAvailability {
    /// A unique depth-three response interval is available.
    Eligible = 0,
    /// The low-amplitude acquisition could not resolve a unique response.
    BelowPrecisionFloor = 1,
    /// Multiple depth-three components exist.
    Ambiguous = 2,
    /// The acquisition was complete but insufficient away from the declared floor.
    Insufficient = 3,
    /// At least one observation reached the usable-current boundary.
    CurrentHeadroom = 4,
    /// At least one observation carried a safety or evidence-integrity fault.
    Fault = 5,
    /// The declared order ended before this amplitude.
    Unattempted = 6,
}

/// Inputs for one direction's four-observation fold.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcceptanceDirectionInput {
    /// Exact signed target in encoder rate-Q.
    pub signed_target_rate_q: i64,
    /// Four-bit schedule-completion mask.
    pub collected_mask: u8,
    /// Intrinsic observation classes decided before consensus.
    pub classes: [VelocityObservationClass; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    /// Exact signed settled-rate intervals.
    pub intervals: [Option<[i32; 2]>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    /// Exact signed settled-rate means for intrinsically usable observations.
    pub means_q: [Option<i32>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    /// Selected blocking level for each observation.
    pub blocking_levels: [u8; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    /// Whether insufficiency at this amplitude defines the lower precision floor.
    pub precision_floor_candidate: bool,
}

/// Exact direction-level result for one matrix amplitude.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AcceptanceDirectionalEvidence {
    /// Availability classification independent of other amplitudes.
    pub availability: AcceptanceAmplitudeAvailability,
    /// Exact consensus components and observation masks.
    pub consensus: DirectionalConsensus,
    /// Signed target in encoder rate-Q.
    pub signed_target_rate_q: i64,
    /// Unique response interval, when eligible.
    pub response_interval_q: Option<[i32; 2]>,
    /// Robust location over intrinsically usable observation means.
    pub robust_location_q: Option<i32>,
    /// Signed target-minus-response interval, when eligible.
    pub deficit_interval_q: Option<[i64; 2]>,
    /// Four-bit eligible observations excluded from the unique component.
    pub excluded_mask: u8,
    /// Per-observation blocking selections.
    pub blocking_levels: [u8; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
}

/// Exact report for both directions at one frozen target.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AcceptanceAmplitudeReport {
    /// Zero-based index in the immutable order.
    pub amplitude_index: u8,
    /// Exact frozen mechanical-RPM target.
    pub target_rpm: i16,
    /// Forward and reverse direction evidence.
    pub directions: [AcceptanceDirectionalEvidence; 2],
}

/// Exact five-bit availability masks for one run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AcceptanceRunMasks {
    /// Amplitudes whose schedules began.
    pub attempted: [u8; 2],
    /// Amplitudes with unique depth-three consensus.
    pub eligible: [u8; 2],
    /// Amplitudes ending on current headroom.
    pub current_terminus: [u8; 2],
    /// Declared amplitudes not reached.
    pub unattempted: [u8; 2],
}

/// Checked evidence-construction failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AcceptanceEvidenceError {
    /// A supplied closed interval was inverted.
    InvalidInterval,
    /// Checked integer arithmetic exceeded its representation.
    Arithmetic,
    /// Five-bit masks contradicted schedule ownership.
    InvalidMasks,
}

/// Compute the robust location of three or four intrinsically usable means.
pub fn robust_rate_location(
    means_q: [Option<i32>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
) -> Result<Option<i32>, AcceptanceEvidenceError> {
    let mut values = [0_i32; CONSENSUS_OBSERVATIONS_PER_DIRECTION];
    let mut count = 0;
    for value in means_q.into_iter().flatten() {
        values[count] = value;
        count += 1;
    }
    for index in 1..count {
        let mut current = index;
        while current > 0 && values[current] < values[current - 1] {
            values.swap(current, current - 1);
            current -= 1;
        }
    }
    match count {
        3 => Ok(Some(values[1])),
        4 => {
            let difference = i64::from(values[2]) - i64::from(values[1]);
            let midpoint = i64::from(values[1]) + difference / 2;
            i32::try_from(midpoint)
                .map(Some)
                .map_err(|_| AcceptanceEvidenceError::Arithmetic)
        }
        _ => Ok(None),
    }
}

/// Fold four intrinsic observations into one exact directional amplitude result.
pub fn classify_acceptance_amplitude(
    input: AcceptanceDirectionInput,
) -> Result<AcceptanceDirectionalEvidence, AcceptanceEvidenceError> {
    if input
        .intervals
        .iter()
        .flatten()
        .any(|interval| interval[0] > interval[1])
    {
        return Err(AcceptanceEvidenceError::InvalidInterval);
    }
    let consensus = depth_three_consensus(input.collected_mask, input.classes, input.intervals);
    let fault = input.classes.iter().any(|class| {
        matches!(
            class,
            VelocityObservationClass::WrongDirectionFault
                | VelocityObservationClass::HardClipFault
                | VelocityObservationClass::InvalidEvidenceFault
        )
    });
    let current = input
        .classes
        .contains(&VelocityObservationClass::CurrentHeadroom);
    let availability = if fault {
        AcceptanceAmplitudeAvailability::Fault
    } else if current {
        AcceptanceAmplitudeAvailability::CurrentHeadroom
    } else {
        match consensus.classification {
            DirectionalConsensusClass::EligibleConsensus => {
                AcceptanceAmplitudeAvailability::Eligible
            }
            DirectionalConsensusClass::AmbiguousConsensus => {
                AcceptanceAmplitudeAvailability::Ambiguous
            }
            DirectionalConsensusClass::InsufficientConsensus
            | DirectionalConsensusClass::IncompleteRung
                if input.precision_floor_candidate =>
            {
                AcceptanceAmplitudeAvailability::BelowPrecisionFloor
            }
            DirectionalConsensusClass::InsufficientConsensus
            | DirectionalConsensusClass::IncompleteRung => {
                AcceptanceAmplitudeAvailability::Insufficient
            }
        }
    };
    let response_interval_q = (availability == AcceptanceAmplitudeAvailability::Eligible)
        .then_some(consensus.components[0]);
    let deficit_interval_q = response_interval_q.map(|interval| {
        [
            input.signed_target_rate_q - i64::from(interval[1]),
            input.signed_target_rate_q - i64::from(interval[0]),
        ]
    });
    Ok(AcceptanceDirectionalEvidence {
        availability,
        consensus,
        signed_target_rate_q: input.signed_target_rate_q,
        response_interval_q,
        robust_location_q: robust_rate_location(input.means_q)?,
        deficit_interval_q,
        excluded_mask: consensus.eligible_mask & !consensus.included_mask,
        blocking_levels: input.blocking_levels,
    })
}

/// Validate one run's exact masks and test the three-amplitude directional floor.
pub fn acceptance_run_sufficient(
    masks: AcceptanceRunMasks,
) -> Result<bool, AcceptanceEvidenceError> {
    for direction in 0..2 {
        let attempted = masks.attempted[direction];
        let eligible = masks.eligible[direction];
        let current = masks.current_terminus[direction];
        let unattempted = masks.unattempted[direction];
        if attempted & !FULL_AMPLITUDE_MASK != 0
            || eligible & !FULL_AMPLITUDE_MASK != 0
            || current & !FULL_AMPLITUDE_MASK != 0
            || unattempted & !FULL_AMPLITUDE_MASK != 0
            || eligible & !attempted != 0
            || current & !attempted != 0
            || attempted & unattempted != 0
            || attempted | unattempted != FULL_AMPLITUDE_MASK
        {
            return Err(AcceptanceEvidenceError::InvalidMasks);
        }
    }
    Ok(masks.eligible.iter().all(|mask| mask.count_ones() >= 3))
}
