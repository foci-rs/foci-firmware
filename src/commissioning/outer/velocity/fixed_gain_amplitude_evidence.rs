//! Exact per-amplitude evidence for fixed-I fixed-gain amplitude validation runs.

use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    depth_three_consensus,
};
use super::sweep_evidence::VelocityObservationClass;

/// Number of amplitudes in one immutable run.
pub const FIXED_GAIN_AMPLITUDE_COUNT: usize = 5;
const FULL_AMPLITUDE_MASK: u8 = (1 << FIXED_GAIN_AMPLITUDE_COUNT) - 1;

/// Availability of one amplitude and direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FixedGainAmplitudeAvailability {
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
pub struct FixedGainAmplitudeDirectionInput {
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

/// Exact direction-level result for one amplitude.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FixedGainAmplitudeDirectionalEvidence {
    /// Availability classification independent of other amplitudes.
    pub availability: FixedGainAmplitudeAvailability,
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
pub struct FixedGainAmplitudeReport {
    /// Zero-based index in the immutable order.
    pub amplitude_index: u8,
    /// Exact frozen mechanical-RPM target.
    pub target_rpm: i16,
    /// Forward and reverse direction evidence.
    pub directions: [FixedGainAmplitudeDirectionalEvidence; 2],
}

/// Exact five-bit availability masks for one run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FixedGainAmplitudeRunMasks {
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
pub enum FixedGainAmplitudeEvidenceError {
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
) -> Result<Option<i32>, FixedGainAmplitudeEvidenceError> {
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
            // Unreachable: values is sorted ascending, so
            // values[1] <= midpoint <= values[2] always holds (midpoint is
            // an average of the two, floored toward values[1]), and both
            // bounds are themselves valid i32 values. i32::try_from(midpoint)
            // can therefore never fail; FixedGainAmplitudeEvidenceError::
            // Arithmetic is unreachable from this, its only construction
            // site.
            i32::try_from(midpoint)
                .map(Some)
                .map_err(|_| FixedGainAmplitudeEvidenceError::Arithmetic)
        }
        _ => Ok(None),
    }
}

/// Fold four intrinsic observations into one exact directional amplitude result.
pub fn classify_fixed_gain_amplitude(
    input: FixedGainAmplitudeDirectionInput,
) -> Result<FixedGainAmplitudeDirectionalEvidence, FixedGainAmplitudeEvidenceError> {
    if input
        .intervals
        .iter()
        .flatten()
        .any(|interval| interval[0] > interval[1])
    {
        return Err(FixedGainAmplitudeEvidenceError::InvalidInterval);
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
        FixedGainAmplitudeAvailability::Fault
    } else if current {
        FixedGainAmplitudeAvailability::CurrentHeadroom
    } else {
        match consensus.classification {
            DirectionalConsensusClass::EligibleConsensus => {
                FixedGainAmplitudeAvailability::Eligible
            }
            DirectionalConsensusClass::AmbiguousConsensus => {
                FixedGainAmplitudeAvailability::Ambiguous
            }
            DirectionalConsensusClass::InsufficientConsensus
            | DirectionalConsensusClass::IncompleteRung
                if input.precision_floor_candidate =>
            {
                FixedGainAmplitudeAvailability::BelowPrecisionFloor
            }
            DirectionalConsensusClass::InsufficientConsensus
            | DirectionalConsensusClass::IncompleteRung => {
                FixedGainAmplitudeAvailability::Insufficient
            }
        }
    };
    let response_interval_q = (availability == FixedGainAmplitudeAvailability::Eligible)
        .then_some(consensus.components[0]);
    let deficit_interval_q = response_interval_q.map(|interval| {
        [
            input.signed_target_rate_q - i64::from(interval[1]),
            input.signed_target_rate_q - i64::from(interval[0]),
        ]
    });
    Ok(FixedGainAmplitudeDirectionalEvidence {
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
pub fn fixed_gain_amplitude_run_sufficient(
    masks: FixedGainAmplitudeRunMasks,
) -> Result<bool, FixedGainAmplitudeEvidenceError> {
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
            // Unreachable via the sole production call path
            // (fixed_gain_amplitude_terminal, integral_sweep.rs): every
            // write to attempted/eligible/current_terminus under
            // FixedGainAmplitude*-acquisition semantics is bound-safe --
            // fixed_gain_amplitude_report's loop, CommandTarget, and the
            // current-headroom analysis block all only ever OR in a bit
            // derived from a rung_index already proven < 5 by an earlier
            // ?-propagated array lookup in the same state cycle -- and
            // eligible/current are always ORed with the same bit attempted
            // just received, so attempted ⊆ full_mask and
            // eligible/current ⊆ attempted hold unconditionally. unattempted
            // is full_mask & !attempted, an algebraic complement rather than
            // an independently tracked value, making the disjointness/union
            // clauses above tautological given that. ResponseLadder
            // acquisition reuses these same struct fields for unrelated
            // plain-rung-index/direction-flag semantics, but is
            // dispatch-level mutually exclusive with FixedGainAmplitude*
            // (VelocityIntegralState::Terminal branches on
            // is_fixed_gain_amplitude before either semantics' read/write
            // path can run), so it cannot contaminate these fields.
            //
            // This function is `pub` and takes masks directly (as is
            // FixedGainAmplitudeRunMasks's every field), so a caller outside
            // fixed_gain_amplitude_terminal -- including test code, or any
            // future caller -- can still construct inconsistent masks and
            // trigger this arm. It stays as a live defensive check against
            // direct callers, not dead code.
            return Err(FixedGainAmplitudeEvidenceError::InvalidMasks);
        }
    }
    Ok(masks.eligible.iter().all(|mask| mask.count_ones() >= 3))
}
