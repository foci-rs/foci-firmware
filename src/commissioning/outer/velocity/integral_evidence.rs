//! Per-run Integral velocity-integral evidence in the encoder-rate domain.

use crate::autotune::velocity_measurement::{StationarityResult, VelocityDirection};

use super::integral_plan::INTEGRAL_MAX_POSITIVE_RUNGS;
use super::sweep_consensus::{DirectionalConsensusClass, depth_three_consensus};
use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, VelocityObservationSummary,
};

const DIRECTION_COUNT: usize = 2;
const OBSERVATIONS_PER_DIRECTION: usize = 4;

/// Exact inclusive encoder-rate-Q interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ExactRateInterval {
    /// Inclusive lower bound.
    pub low_q: i32,
    /// Inclusive upper bound.
    pub high_q: i32,
}

impl ExactRateInterval {
    /// Zero-valued placeholder used only where a mask carries presence.
    pub const EMPTY: Self = Self {
        low_q: 0,
        high_q: 0,
    };

    /// Construct a checked closed interval.
    pub const fn new(low_q: i32, high_q: i32) -> Result<Self, IntegralEvidenceError> {
        if low_q > high_q {
            Err(IntegralEvidenceError::InvalidInterval)
        } else {
            Ok(Self { low_q, high_q })
        }
    }

    const fn bounds(self) -> [i32; 2] {
        [self.low_q, self.high_q]
    }
}

/// Intrinsic Integral interpretation of one observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralObservationClass {
    /// Stationary commanded-direction motion yielded usable rate evidence.
    Usable,
    /// An anchor deficit did not resolve on the command-derived side of zero.
    SignUnresolved,
    /// Commanded-direction motion was not resolved.
    Inert,
    /// The inclusive usable-current boundary was reached.
    CurrentHeadroom,
    /// Target-independent stationarity excluded the observation.
    StationarityExcluded,
    /// A structural observation fault occurred.
    Fault,
}

/// Semantic role of one Integral I rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralRungKind {
    /// Opening I=0 operating-point precondition and baseline.
    OpeningAnchor,
    /// Positive-I response point; deficit sign is unrestricted.
    Positive,
    /// Closing I=0 drift bookend.
    ClosingBookend,
    /// Fixed-I production fixed-gain amplitude.
    FixedGainAmplitude,
}

/// Directional result for one complete four-observation rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralRungClass {
    /// One unique depth-three component is eligible.
    EligibleConsensus,
    /// Two depth-three components exist and firmware refuses to choose.
    AmbiguousConsensus,
    /// Fewer than three compatible usable observations exist.
    InsufficientConsensus,
    /// At least one observation reached the usable-current boundary.
    CurrentHeadroom,
    /// At least one observation carried a safety or evidence-integrity fault.
    Fault,
}

/// Auditable direction-level result for one Integral rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralRungEvidence {
    /// Commanded mechanical direction.
    pub direction: VelocityDirection,
    /// Rung role controlling only the anchor sign predicate.
    pub kind: IntegralRungKind,
    /// Directional rung classification.
    pub classification: IntegralRungClass,
    /// Unique eligible interval, when present.
    pub interval: Option<ExactRateInterval>,
    /// Zero, one, or two exact depth-three components.
    pub components: [ExactRateInterval; 2],
    /// Number of populated components.
    pub component_count: u8,
    /// Intrinsic classifications of all four observations.
    pub observation_classes: [IntegralObservationClass; OBSERVATIONS_PER_DIRECTION],
    /// Raw intervals for every intrinsically usable observation.
    pub observation_intervals: [Option<ExactRateInterval>; OBSERVATIONS_PER_DIRECTION],
    /// Four-bit usable-observation mask.
    pub usable_mask: u8,
    /// Four-bit mask supporting the unique component.
    pub included_mask: u8,
}

/// Why a safe Integral run lacks the structural evidence needed for pairing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralInconclusiveKind {
    /// One or both directions contain fewer than three eligible positive rungs.
    TooFewEligibleRungs,
    /// Current headroom ended the ladder before both directional floors existed.
    CurrentBoundBeforeSufficiency,
    /// One or both required closing I=0 anchors were unavailable.
    BookendUnavailable,
}

/// Per-run Integral handoff sufficiency; this is not I acceptance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralRunSufficiency {
    /// The run can become or compare against an Integral candidate.
    Sufficient {
        /// Per-direction positive-rung membership.
        eligible_masks: [u32; DIRECTION_COUNT],
        /// Bit zero is forward and bit one is reverse.
        bookend_available_mask: u8,
        /// Exact positive-rung index that reached usable current.
        current_terminus: Option<u8>,
    },
    /// Measurement stayed safe but cannot enter reproduction.
    Inconclusive {
        /// Per-direction positive-rung membership.
        eligible_masks: [u32; DIRECTION_COUNT],
        /// Exact structural insufficiency.
        cause: IntegralInconclusiveKind,
        /// Bit zero is the forward bookend and bit one is the reverse bookend.
        bookend_available_mask: u8,
        /// Exact positive-rung current terminus, when present.
        current_terminus: Option<u8>,
    },
}

/// Exact raw Integral curve retained before anchor-relative comparison.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct IntegralRunEvidence {
    /// Opening I=0 intervals in forward, reverse order.
    pub opening: [ExactRateInterval; DIRECTION_COUNT],
    /// Positive-I intervals in direction-major order; masks carry presence.
    pub positive: [[ExactRateInterval; INTEGRAL_MAX_POSITIVE_RUNGS]; DIRECTION_COUNT],
    /// Per-direction positive-I eligibility masks.
    pub eligible_masks: [u32; DIRECTION_COUNT],
    /// Optional closing I=0 intervals.
    pub bookend: [Option<ExactRateInterval>; DIRECTION_COUNT],
    /// First positive rung reaching the inclusive usable-current boundary.
    pub current_terminus: Option<u8>,
}

impl IntegralRunEvidence {
    /// Begin a run only after both opening anchors pass.
    pub const fn new(opening: [ExactRateInterval; DIRECTION_COUNT]) -> Self {
        Self {
            opening,
            positive: [[ExactRateInterval::EMPTY; INTEGRAL_MAX_POSITIVE_RUNGS]; DIRECTION_COUNT],
            eligible_masks: [0; DIRECTION_COUNT],
            bookend: [None; DIRECTION_COUNT],
            current_terminus: None,
        }
    }

    /// Retain one unique positive-I consensus interval.
    pub fn retain_positive(
        &mut self,
        direction: usize,
        rung: usize,
        interval: ExactRateInterval,
    ) -> Result<(), IntegralEvidenceError> {
        if direction >= DIRECTION_COUNT || rung >= INTEGRAL_MAX_POSITIVE_RUNGS {
            return Err(IntegralEvidenceError::MaskOutOfRange);
        }
        let bit = 1_u32
            .checked_shl(u32::try_from(rung).map_err(|_| IntegralEvidenceError::MaskOutOfRange)?)
            .ok_or(IntegralEvidenceError::MaskOutOfRange)?;
        if self.eligible_masks[direction] & bit != 0 {
            return Err(IntegralEvidenceError::DuplicateRung);
        }
        self.positive[direction][rung] = interval;
        self.eligible_masks[direction] |= bit;
        Ok(())
    }

    /// Check the exact per-run precondition for a two-run comparison.
    pub fn assess(
        self,
        positive_rung_count: usize,
    ) -> Result<IntegralRunSufficiency, IntegralEvidenceError> {
        if positive_rung_count == 0 || positive_rung_count > INTEGRAL_MAX_POSITIVE_RUNGS {
            return Err(IntegralEvidenceError::MaskOutOfRange);
        }
        let valid_mask = (1_u32 << positive_rung_count) - 1;
        if self
            .eligible_masks
            .iter()
            .any(|mask| mask & !valid_mask != 0)
            || self
                .current_terminus
                .is_some_and(|rung| usize::from(rung) >= positive_rung_count)
        {
            return Err(IntegralEvidenceError::MaskOutOfRange);
        }
        let sufficient = self
            .eligible_masks
            .iter()
            .all(|mask| mask.count_ones() >= 3);
        let bookend_available_mask =
            u8::from(self.bookend[0].is_some()) | (u8::from(self.bookend[1].is_some()) << 1);
        if sufficient {
            if bookend_available_mask == 0b11 {
                Ok(IntegralRunSufficiency::Sufficient {
                    eligible_masks: self.eligible_masks,
                    bookend_available_mask,
                    current_terminus: self.current_terminus,
                })
            } else {
                Ok(IntegralRunSufficiency::Inconclusive {
                    eligible_masks: self.eligible_masks,
                    cause: IntegralInconclusiveKind::BookendUnavailable,
                    bookend_available_mask,
                    current_terminus: self.current_terminus,
                })
            }
        } else {
            Ok(IntegralRunSufficiency::Inconclusive {
                eligible_masks: self.eligible_masks,
                cause: if self.current_terminus.is_some() {
                    IntegralInconclusiveKind::CurrentBoundBeforeSufficiency
                } else {
                    IntegralInconclusiveKind::TooFewEligibleRungs
                },
                bookend_available_mask,
                current_terminus: self.current_terminus,
            })
        }
    }
}

/// Evidence-integrity failure while building one Integral curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralEvidenceError {
    /// A raw interval had reversed bounds.
    InvalidInterval,
    /// An observation's direction, gain, or interval was inconsistent.
    MismatchedObservation,
    /// An exact i64 rate bound did not fit Integral's retained i32 domain.
    Conversion,
    /// A direction, rung, mask bit, or terminus exceeded the declared ladder.
    MaskOutOfRange,
    /// The same direction/rung cell was retained twice.
    DuplicateRung,
}

/// Classify one Integral observation without constructing or requiring D_eq.
pub fn classify_integral_observation(
    input: VelocityObservationInput,
) -> VelocityObservationSummary {
    let valid_intervals = input.settled_rate_low_q <= input.settled_rate_high_q
        && input.deficit_low_q <= input.deficit_high_q
        && input.usable_torque_limit <= input.hard_torque_limit;
    let wrong_direction = match input.direction {
        VelocityDirection::Forward => input.settled_rate_high_q < 0,
        VelocityDirection::Reverse => input.settled_rate_low_q > 0,
    };
    let moving = match input.direction {
        VelocityDirection::Forward => input.settled_rate_low_q > 0,
        VelocityDirection::Reverse => input.settled_rate_high_q < 0,
    };
    let classification = if !valid_intervals {
        VelocityObservationClass::InvalidEvidenceFault
    } else if input.predicted_torque_target_abs >= input.hard_torque_limit {
        VelocityObservationClass::HardClipFault
    } else if wrong_direction {
        VelocityObservationClass::WrongDirectionFault
    } else if let StationarityResult::Excluded(reason) = input.stationarity {
        VelocityObservationClass::StationarityExcluded(reason)
    } else if !moving {
        VelocityObservationClass::Inert
    } else if input.predicted_torque_target_abs >= input.usable_torque_limit {
        VelocityObservationClass::CurrentHeadroom
    } else {
        VelocityObservationClass::Eligible
    };
    VelocityObservationSummary {
        direction: input.direction,
        p: input.p,
        classification,
        settled_rate_interval_q: [input.settled_rate_low_q, input.settled_rate_high_q],
        settled_rate_mean_q: input.settled_rate_mean_q,
        deficit_interval_q: [input.deficit_low_q, input.deficit_high_q],
        disturbance: None,
        disturbance_interval: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: input.predicted_torque_target_abs,
        drift_ratio_ppm: input.drift_ratio_ppm,
        initial_lead_in: input.initial_lead_in,
    }
}

/// Classify four same-direction observations without constructing D_eq.
pub fn analyze_integral_direction(
    direction: VelocityDirection,
    kind: IntegralRungKind,
    observations: [VelocityObservationSummary; OBSERVATIONS_PER_DIRECTION],
) -> Result<IntegralRungEvidence, IntegralEvidenceError> {
    let p = observations[0].p;
    if observations.iter().any(|observation| {
        observation.direction != direction
            || observation.p != p
            || observation.settled_rate_interval_q[0] > observation.settled_rate_interval_q[1]
            || observation.deficit_interval_q[0] > observation.deficit_interval_q[1]
    }) {
        return Err(IntegralEvidenceError::MismatchedObservation);
    }

    let mut observation_classes = [IntegralObservationClass::Fault; OBSERVATIONS_PER_DIRECTION];
    let mut observation_intervals = [None; OBSERVATIONS_PER_DIRECTION];
    for (index, observation) in observations.iter().enumerate() {
        let class = classify_observation(direction, kind, *observation);
        observation_classes[index] = class;
        if class == IntegralObservationClass::Usable {
            let low_q = i32::try_from(observation.deficit_interval_q[0])
                .map_err(|_| IntegralEvidenceError::Conversion)?;
            let high_q = i32::try_from(observation.deficit_interval_q[1])
                .map_err(|_| IntegralEvidenceError::Conversion)?;
            observation_intervals[index] = Some(ExactRateInterval::new(low_q, high_q)?);
        }
    }

    let mut result = IntegralRungEvidence {
        direction,
        kind,
        classification: IntegralRungClass::InsufficientConsensus,
        interval: None,
        components: [ExactRateInterval::EMPTY; 2],
        component_count: 0,
        observation_classes,
        observation_intervals,
        usable_mask: 0,
        included_mask: 0,
    };
    if observation_classes.contains(&IntegralObservationClass::Fault) {
        result.classification = IntegralRungClass::Fault;
        return Ok(result);
    }
    if observation_classes.contains(&IntegralObservationClass::CurrentHeadroom) {
        result.classification = IntegralRungClass::CurrentHeadroom;
        return Ok(result);
    }

    let consensus_classes = core::array::from_fn(|index| {
        if observation_classes[index] == IntegralObservationClass::Usable {
            VelocityObservationClass::Eligible
        } else {
            VelocityObservationClass::Inert
        }
    });
    let raw_intervals =
        observation_intervals.map(|interval| interval.map(ExactRateInterval::bounds));
    let consensus = depth_three_consensus(0b1111, consensus_classes, raw_intervals);
    result.usable_mask = consensus.eligible_mask;
    result.included_mask = consensus.included_mask;
    result.component_count = consensus.component_count;
    for (destination, bounds) in result.components.iter_mut().zip(consensus.components) {
        *destination = ExactRateInterval::new(bounds[0], bounds[1])?;
    }
    result.classification = match consensus.classification {
        DirectionalConsensusClass::EligibleConsensus => IntegralRungClass::EligibleConsensus,
        DirectionalConsensusClass::AmbiguousConsensus => IntegralRungClass::AmbiguousConsensus,
        DirectionalConsensusClass::InsufficientConsensus
        | DirectionalConsensusClass::IncompleteRung => IntegralRungClass::InsufficientConsensus,
    };
    if result.classification == IntegralRungClass::EligibleConsensus {
        result.interval = Some(result.components[0]);
    }
    Ok(result)
}

fn classify_observation(
    direction: VelocityDirection,
    kind: IntegralRungKind,
    observation: VelocityObservationSummary,
) -> IntegralObservationClass {
    match observation.classification {
        VelocityObservationClass::WrongDirectionFault
        | VelocityObservationClass::HardClipFault
        | VelocityObservationClass::InvalidEvidenceFault => IntegralObservationClass::Fault,
        VelocityObservationClass::StationarityExcluded(_) => {
            IntegralObservationClass::StationarityExcluded
        }
        VelocityObservationClass::Inert => IntegralObservationClass::Inert,
        VelocityObservationClass::CurrentHeadroom => IntegralObservationClass::CurrentHeadroom,
        VelocityObservationClass::Eligible | VelocityObservationClass::DeficitUnresolved => {
            let sign_resolved = match direction {
                VelocityDirection::Forward => observation.deficit_interval_q[0] > 0,
                VelocityDirection::Reverse => observation.deficit_interval_q[1] < 0,
            };
            if kind == IntegralRungKind::Positive || sign_resolved {
                IntegralObservationClass::Usable
            } else {
                IntegralObservationClass::SignUnresolved
            }
        }
    }
}
