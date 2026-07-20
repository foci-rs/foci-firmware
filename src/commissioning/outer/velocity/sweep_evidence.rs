use crate::autotune::velocity_measurement::{
    StationarityExclusion, StationarityResult, VelocityDirection,
};
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
use foci_tmc4671::pid::PidGain;

use super::operating_region::{BoundRounding, CompactSignedRate, PooledDisturbanceAccumulator};
use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensusClass, depth_three_consensus,
};

const CONSENSUS_DIRECTION_COUNT: usize = 2;
const CONSENSUS_SLOT_COUNT: usize =
    CONSENSUS_OBSERVATIONS_PER_DIRECTION * CONSENSUS_DIRECTION_COUNT;
const STATUS_CLASS_BITS: u32 = 4;
const STATUS_CLASS_MASK: u32 = 0x0f;
const STATUS_COLLECTED_SHIFT: u32 = 16;
const STATUS_COLLECTED_MASK: u32 = 0x0f << STATUS_COLLECTED_SHIFT;
const STATUS_INITIAL_LEAD_IN: u32 = 1 << 20;

/// Directional report flag indicating three independently operable observations.
pub const DIRECTIONAL_RUNG_FLAG_OPERABLE: u8 = 1 << 0;
/// Directional report flag indicating the exceptional initial lead-in observation.
pub const DIRECTIONAL_RUNG_FLAG_INITIAL_LEAD_IN: u8 = 1 << 1;

/// Stable report code for one directional-region kind.
pub const fn directional_region_kind_code(
    kind: super::operating_region::DirectionalRegionKind,
) -> u8 {
    match kind {
        super::operating_region::DirectionalRegionKind::Valid => 0,
        super::operating_region::DirectionalRegionKind::Fragment => 1,
    }
}

/// Stable report code for one directional-region closure.
pub const fn directional_region_closure_code(
    closure: super::operating_region::DirectionalRegionClosure,
) -> u8 {
    match closure {
        super::operating_region::DirectionalRegionClosure::DisjointInterval => 0,
        super::operating_region::DirectionalRegionClosure::IneligibleGap => 1,
        super::operating_region::DirectionalRegionClosure::CurrentHeadroom => 2,
        super::operating_region::DirectionalRegionClosure::SafetyTerminus => 3,
        super::operating_region::DirectionalRegionClosure::EndOfLadder => 4,
    }
}

/// Stable report code for one joint-operable-region closure.
pub const fn joint_operable_closure_code(
    closure: super::operating_region::JointOperableClosure,
) -> u8 {
    match closure {
        super::operating_region::JointOperableClosure::IneligibleGap => 0,
        super::operating_region::JointOperableClosure::CurrentHeadroom => 1,
        super::operating_region::JointOperableClosure::SafetyTerminus => 2,
        super::operating_region::JointOperableClosure::EndOfLadder => 3,
    }
}

/// Stable report code for one directional handoff coverage kind.
pub const fn coverage_kind_code(kind: super::operating_region::CoverageKind) -> u8 {
    match kind {
        super::operating_region::CoverageKind::FullCoverage => 0,
        super::operating_region::CoverageKind::TransferredCoverage => 1,
    }
}

/// Conservatively upper-rounded variance retained only until its rung is folded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CompactVariance {
    mantissa: u16,
    shift: u8,
}

impl CompactVariance {
    /// Encode an unsigned variance without ever understating it.
    pub fn encode(value: u64) -> Self {
        for shift in 0..=48 {
            let denominator = 1_u64 << shift;
            let rounded = value.div_ceil(denominator);
            if let Ok(mantissa) = u16::try_from(rounded) {
                return Self { mantissa, shift };
            }
        }
        unreachable!("a u64 always fits a u16 mantissa with a binary shift")
    }

    /// Decode the conservative variance upper bound.
    pub const fn decode(self) -> u64 {
        (self.mantissa as u64) << self.shift
    }
}

const _: () = assert!(core::mem::size_of::<CompactVariance>() <= 4);

/// Inputs already collected for one fixed-direction velocity observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityObservationInput {
    /// Commanded mechanical direction.
    pub direction: VelocityDirection,
    /// Velocity proportional gain used for the observation.
    pub p: PidGain,
    /// Target-independent stationary-suffix result.
    pub stationarity: StationarityResult,
    /// Inclusive lower settled-rate confidence edge.
    pub settled_rate_low_q: i64,
    /// Inclusive upper settled-rate confidence edge.
    pub settled_rate_high_q: i64,
    /// Exact time-weighted settled-rate mean.
    pub settled_rate_mean_q: i64,
    /// Inclusive lower signed-deficit confidence edge.
    pub deficit_low_q: i64,
    /// Inclusive upper signed-deficit confidence edge.
    pub deficit_high_q: i64,
    /// Controller-domain disturbance point and interval, when representable.
    pub disturbance: Option<(EquivalentTorqueTargetQ16, EquivalentTorqueTargetInterval)>,
    /// Controller-domain disturbance variance used for fixed-effect pooling.
    pub disturbance_variance_q2: u64,
    /// Predicted absolute velocity-controller torque target.
    pub predicted_torque_target_abs: u16,
    /// Inclusive current-headroom terminus.
    pub usable_torque_limit: u16,
    /// Hard controller-output clip boundary.
    pub hard_torque_limit: u16,
    /// Whether this observation lacks the normal reversal lead-in.
    pub initial_lead_in: bool,
}

/// Analysis-only classification for one completed observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityObservationClass {
    /// All predicates passed and the observation may contribute to D_eq.
    Eligible,
    /// The settled-rate interval still includes zero.
    Inert,
    /// Motion is resolved but the signed deficit is not.
    DeficitUnresolved,
    /// The usable current-headroom boundary was reached inclusively.
    CurrentHeadroom,
    /// Target-independent stationarity evidence was unusable.
    StationarityExcluded(StationarityExclusion),
    /// Settled motion is significantly opposite the command.
    WrongDirectionFault,
    /// The hard controller-output clip was reached.
    HardClipFault,
    /// Supplied interval or controller-domain evidence was inconsistent.
    InvalidEvidenceFault,
}

/// Stable report code for one signed mechanical direction.
pub const fn direction_report_code(direction: VelocityDirection) -> u8 {
    match direction {
        VelocityDirection::Forward => 0,
        VelocityDirection::Reverse => 1,
    }
}

/// Stable report code for one observation classification.
pub const fn observation_class_report_code(class: VelocityObservationClass) -> u8 {
    match class {
        VelocityObservationClass::Eligible => 0,
        VelocityObservationClass::Inert => 1,
        VelocityObservationClass::DeficitUnresolved => 2,
        VelocityObservationClass::CurrentHeadroom => 3,
        VelocityObservationClass::StationarityExcluded(reason) => 4 + reason as u8,
        VelocityObservationClass::WrongDirectionFault => 9,
        VelocityObservationClass::HardClipFault => 10,
        VelocityObservationClass::InvalidEvidenceFault => 11,
    }
}

/// Auditable result for one completed observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityObservationSummary {
    /// Commanded direction.
    pub direction: VelocityDirection,
    /// Gain used for the observation.
    pub p: PidGain,
    /// Observation-level classification.
    pub classification: VelocityObservationClass,
    /// Settled-rate confidence interval.
    pub settled_rate_interval_q: [i64; 2],
    /// Exact time-weighted settled-rate mean.
    pub settled_rate_mean_q: i64,
    /// Signed-deficit confidence interval.
    pub deficit_interval_q: [i64; 2],
    /// Controller-domain disturbance point.
    pub disturbance: Option<EquivalentTorqueTargetQ16>,
    /// Controller-domain disturbance interval.
    pub disturbance_interval: Option<EquivalentTorqueTargetInterval>,
    /// Controller-domain variance, with zero retained for the pooling floor.
    pub disturbance_variance_q2: u64,
    /// Predicted absolute controller output.
    pub predicted_torque_target_abs: u16,
    /// Whether this was the exceptional first campaign observation.
    pub initial_lead_in: bool,
}

/// Fixed-size controller-domain evidence retained for one observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(C)]
pub struct CompactConsensusObservation {
    /// Inclusive disturbance lower edge in signed Q16.16 counts.
    pub disturbance_low_q16: i32,
    /// Inclusive disturbance upper edge in signed Q16.16 counts.
    pub disturbance_high_q16: i32,
    /// Truncated disturbance point in signed Q16.16 counts.
    pub disturbance_point_q16: i32,
    /// Conservative compact controller-domain variance.
    pub variance: CompactVariance,
}

impl CompactConsensusObservation {
    const EMPTY: Self = Self {
        disturbance_low_q16: 0,
        disturbance_high_q16: 0,
        disturbance_point_q16: 0,
        variance: CompactVariance {
            mantissa: 0,
            shift: 0,
        },
    };

    fn from_summary(summary: VelocityObservationSummary) -> Result<Self, VelocityEvidenceError> {
        if summary.classification != VelocityObservationClass::Eligible {
            return Ok(Self::EMPTY);
        }
        let point = summary
            .disturbance
            .ok_or(VelocityEvidenceError::InvalidConsensus)?;
        let interval = summary
            .disturbance_interval
            .ok_or(VelocityEvidenceError::InvalidConsensus)?;
        Ok(Self {
            disturbance_low_q16: interval.low().raw(),
            disturbance_high_q16: interval.high().raw(),
            disturbance_point_q16: point.raw(),
            variance: CompactVariance::encode(summary.disturbance_variance_q2),
        })
    }

    fn interval(self) -> Option<EquivalentTorqueTargetInterval> {
        EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(self.disturbance_low_q16),
            EquivalentTorqueTargetQ16::from_raw(self.disturbance_high_q16),
        )
        .ok()
    }
}

const _: () = assert!(core::mem::size_of::<CompactConsensusObservation>() <= 16);

/// Complete compact evidence for all eight observations at one gain rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(C)]
pub struct CollectedRungEvidence {
    observations: [CompactConsensusObservation; CONSENSUS_SLOT_COUNT],
    direction_status: [u32; CONSENSUS_DIRECTION_COUNT],
}

impl CollectedRungEvidence {
    /// Construct an empty fixed-rung collector.
    pub const fn new() -> Self {
        Self {
            observations: [CompactConsensusObservation::EMPTY; CONSENSUS_SLOT_COUNT],
            direction_status: [0; CONSENSUS_DIRECTION_COUNT],
        }
    }

    /// Retain one intrinsic observation at its declared global slot.
    pub fn retain(
        &mut self,
        slot: u8,
        summary: VelocityObservationSummary,
    ) -> Result<(), VelocityEvidenceError> {
        let slot = usize::from(slot);
        if slot >= CONSENSUS_SLOT_COUNT {
            return Err(VelocityEvidenceError::MismatchedObservation);
        }
        let direction = slot % CONSENSUS_DIRECTION_COUNT;
        let observation = slot / CONSENSUS_DIRECTION_COUNT;
        let expected_direction = if direction == 0 {
            VelocityDirection::Forward
        } else {
            VelocityDirection::Reverse
        };
        if summary.direction != expected_direction || self.is_collected(direction, observation) {
            return Err(VelocityEvidenceError::MismatchedObservation);
        }

        self.observations[slot] = CompactConsensusObservation::from_summary(summary)?;
        let class_shift = u32::try_from(observation).unwrap() * STATUS_CLASS_BITS;
        self.direction_status[direction] |=
            u32::from(observation_class_report_code(summary.classification)) << class_shift;
        self.direction_status[direction] |= 1 << (STATUS_COLLECTED_SHIFT + class_shift / 4);
        if summary.initial_lead_in {
            self.direction_status[direction] |= STATUS_INITIAL_LEAD_IN;
        }
        Ok(())
    }

    /// Return the four-bit collected mask for one signed direction.
    pub const fn collected_mask(&self, direction: VelocityDirection) -> u8 {
        ((self.direction_status[direction_index(direction)] & STATUS_COLLECTED_MASK)
            >> STATUS_COLLECTED_SHIFT) as u8
    }

    /// Return one retained intrinsic class, or `None` when the slot was not collected.
    pub fn classification(
        &self,
        direction: VelocityDirection,
        observation: usize,
    ) -> Option<VelocityObservationClass> {
        self.directional_set(direction).classification(observation)
    }

    /// Return one retained eligible disturbance interval.
    pub fn disturbance_interval(
        &self,
        direction: VelocityDirection,
        observation: usize,
    ) -> Option<EquivalentTorqueTargetInterval> {
        self.directional_set(direction)
            .disturbance_interval(observation)
    }

    /// Extract the fixed four-observation set for one direction.
    pub fn directional_set(&self, direction: VelocityDirection) -> DirectionalObservationSet {
        let direction_index = direction_index(direction);
        let mut observations =
            [CompactConsensusObservation::EMPTY; CONSENSUS_OBSERVATIONS_PER_DIRECTION];
        for (observation, destination) in observations.iter_mut().enumerate() {
            *destination = self.observations[observation * 2 + direction_index];
        }
        DirectionalObservationSet {
            observations,
            status: self.direction_status[direction_index],
        }
    }

    const fn is_collected(&self, direction: usize, observation: usize) -> bool {
        self.direction_status[direction] & (1 << (STATUS_COLLECTED_SHIFT + observation as u32)) != 0
    }
}

impl Default for CollectedRungEvidence {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<CollectedRungEvidence>() <= 136);

/// Compact four-observation evidence retained for one direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(C)]
pub struct DirectionalObservationSet {
    observations: [CompactConsensusObservation; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    status: u32,
}

impl DirectionalObservationSet {
    fn collected_mask(self) -> u8 {
        ((self.status & STATUS_COLLECTED_MASK) >> STATUS_COLLECTED_SHIFT) as u8
    }

    fn classification(self, observation: usize) -> Option<VelocityObservationClass> {
        if observation >= CONSENSUS_OBSERVATIONS_PER_DIRECTION
            || self.status & (1 << (STATUS_COLLECTED_SHIFT + observation as u32)) == 0
        {
            return None;
        }
        let shift = observation as u32 * STATUS_CLASS_BITS;
        observation_class_from_report_code(((self.status >> shift) & STATUS_CLASS_MASK) as u8)
    }

    fn disturbance_interval(self, observation: usize) -> Option<EquivalentTorqueTargetInterval> {
        if self.classification(observation)? != VelocityObservationClass::Eligible {
            return None;
        }
        self.observations.get(observation)?.interval()
    }

    fn initial_lead_in(self) -> bool {
        self.status & STATUS_INITIAL_LEAD_IN != 0
    }
}

const _: () = assert!(core::mem::size_of::<DirectionalObservationSet>() <= 68);

/// Compact directional result consumed by region folds and wire reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CompactDirectionalRungReport {
    /// Gain shared by the four observations.
    pub p: PidGain,
    /// Depth-three consensus classification.
    pub classification: DirectionalConsensusClass,
    /// Zero, one, or two exact consensus components.
    pub components: [[i32; 2]; 2],
    /// Fixed-effect pool when a unique component exists.
    pub pooled: Option<PooledDisturbance>,
    /// Conservative included-only mean and confidence rate bounds.
    pub settled_rate_bounds_q: [CompactSignedRate; 4],
    /// Four-bit collected mask.
    pub collected_mask: u8,
    /// Four-bit intrinsically eligible mask.
    pub eligible_mask: u8,
    /// Four-bit unique-component inclusion mask.
    pub included_mask: u8,
    /// Number of independently operable observations.
    pub operable_count: u8,
    /// Directional operability and initial-lead-in flags.
    pub flags: u8,
}

impl CompactDirectionalRungReport {
    /// Whether at least three observations independently support operation.
    pub const fn operable(self) -> bool {
        self.flags & DIRECTIONAL_RUNG_FLAG_OPERABLE != 0
    }

    /// Whether this direction contains the campaign's exceptional first observation.
    pub const fn initial_lead_in(self) -> bool {
        self.flags & DIRECTIONAL_RUNG_FLAG_INITIAL_LEAD_IN != 0
    }

    /// Number of reported exact consensus components.
    pub const fn component_count(self) -> u8 {
        match self.classification {
            DirectionalConsensusClass::EligibleConsensus => 1,
            DirectionalConsensusClass::AmbiguousConsensus => 2,
            DirectionalConsensusClass::InsufficientConsensus
            | DirectionalConsensusClass::IncompleteRung => 0,
        }
    }
}

/// Directional report plus mergeable exact pool state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalRungAnalysis {
    /// Compact report retained after folding.
    pub report: CompactDirectionalRungReport,
    /// Exact fixed-effect accumulator merged into a directional region.
    pub pooled_accumulator: PooledDisturbanceAccumulator,
}

/// Analysis-only classification for one same-direction observation pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityRungClass {
    /// Both observations passed and their corrected D_eq intervals overlap.
    Eligible,
    /// Both observations independently remain indistinguishable from rest.
    Inert,
    /// Both observations move, but at least one deficit remains unresolved.
    DeficitUnresolved,
    /// The pair reached the inclusive usable-current boundary.
    CurrentHeadroom,
    /// Both observations passed individually but their D_eq intervals are disjoint.
    DisjointPair,
    /// The two observations disagree on a required predicate.
    Ambiguous,
    /// A non-motion quality predicate excluded the pair.
    Excluded,
    /// At least one observation produced a structural fault.
    Fault,
}

/// Stable report code for one reproduced-rung classification.
pub const fn rung_class_report_code(class: VelocityRungClass) -> u8 {
    match class {
        VelocityRungClass::Eligible => 0,
        VelocityRungClass::Inert => 1,
        VelocityRungClass::DeficitUnresolved => 2,
        VelocityRungClass::CurrentHeadroom => 3,
        VelocityRungClass::DisjointPair => 4,
        VelocityRungClass::Ambiguous => 5,
        VelocityRungClass::Excluded => 6,
        VelocityRungClass::Fault => 7,
    }
}

/// Reproduced evidence retained for one direction at one gain rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityRungVerdict {
    /// Direction shared by both observations.
    pub direction: VelocityDirection,
    /// Gain shared by both observations.
    pub p: PidGain,
    /// Pair-level classification.
    pub classification: VelocityRungClass,
    /// Intersection of corrected observation intervals when reproduced.
    pub disturbance_interval: Option<EquivalentTorqueTargetInterval>,
    /// Individual corrected intervals retained for auditable pooling.
    pub disturbance_intervals: [Option<EquivalentTorqueTargetInterval>; 2],
    /// Individual points retained for later fixed-effect pooling.
    pub disturbance_points: [Option<EquivalentTorqueTargetQ16>; 2],
    /// Individual variances retained for later fixed-effect pooling.
    pub disturbance_variances_q2: [CompactVariance; 2],
    /// Conservative mean-min, mean-max, interval-low, and interval-high rate bounds.
    pub settled_rate_bounds_q: [CompactSignedRate; 4],
    /// Whether both observations resolved commanded-side motion.
    pub reproduced_moving: bool,
    /// Whether all four unchanged operating-point predicates passed.
    pub operable: bool,
    /// Whether either observation carried the rung-zero lead-in marker.
    pub initial_lead_in: bool,
}

/// Failure to combine two summaries that do not describe one rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityEvidenceError {
    /// A slot, direction, or gain did not match the declared rung.
    MismatchedObservation,
    /// Intrinsic eligible evidence lacked a required controller-domain value.
    InvalidConsensus,
    /// Checked reconstruction or pooling arithmetic failed.
    Arithmetic,
}

/// Classify one already-executed observation without returning a motion directive.
pub fn classify_velocity_observation(
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
    let deficit_resolved = match input.direction {
        VelocityDirection::Forward => input.deficit_low_q > 0,
        VelocityDirection::Reverse => input.deficit_high_q < 0,
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
    } else if !deficit_resolved {
        VelocityObservationClass::DeficitUnresolved
    } else if input.disturbance.is_none() {
        VelocityObservationClass::InvalidEvidenceFault
    } else {
        VelocityObservationClass::Eligible
    };
    let (disturbance, disturbance_interval) =
        input.disturbance.map_or((None, None), |(point, interval)| {
            (Some(point), Some(interval))
        });

    VelocityObservationSummary {
        direction: input.direction,
        p: input.p,
        classification,
        settled_rate_interval_q: [input.settled_rate_low_q, input.settled_rate_high_q],
        settled_rate_mean_q: input.settled_rate_mean_q,
        deficit_interval_q: [input.deficit_low_q, input.deficit_high_q],
        disturbance,
        disturbance_interval,
        disturbance_variance_q2: input.disturbance_variance_q2,
        predicted_torque_target_abs: input.predicted_torque_target_abs,
        initial_lead_in: input.initial_lead_in,
    }
}

/// Combine two independent same-direction observations into one rung verdict.
pub fn reproduce_velocity_rung(
    first: VelocityObservationSummary,
    second: VelocityObservationSummary,
) -> Result<VelocityRungVerdict, VelocityEvidenceError> {
    if first.direction != second.direction || first.p != second.p {
        return Err(VelocityEvidenceError::MismatchedObservation);
    }
    let first_moving = observation_resolves_motion(first.classification);
    let second_moving = observation_resolves_motion(second.classification);
    let reproduced_moving = first_moving && second_moving;
    let operable = observation_is_operable(first.classification)
        && observation_is_operable(second.classification);
    let mut interval = None;
    let classification = match (first.classification, second.classification) {
        (VelocityObservationClass::Eligible, VelocityObservationClass::Eligible) => {
            interval = intersect_intervals(first.disturbance_interval, second.disturbance_interval);
            if interval.is_some() {
                VelocityRungClass::Eligible
            } else {
                VelocityRungClass::DisjointPair
            }
        }
        (VelocityObservationClass::Inert, VelocityObservationClass::Inert) => {
            VelocityRungClass::Inert
        }
        (
            VelocityObservationClass::DeficitUnresolved,
            VelocityObservationClass::DeficitUnresolved,
        ) => VelocityRungClass::DeficitUnresolved,
        (VelocityObservationClass::CurrentHeadroom, VelocityObservationClass::CurrentHeadroom) => {
            VelocityRungClass::CurrentHeadroom
        }
        (left, right) if observation_is_fault(left) || observation_is_fault(right) => {
            VelocityRungClass::Fault
        }
        (
            VelocityObservationClass::StationarityExcluded(_),
            VelocityObservationClass::StationarityExcluded(_),
        ) => VelocityRungClass::Excluded,
        _ => VelocityRungClass::Ambiguous,
    };

    Ok(VelocityRungVerdict {
        direction: first.direction,
        p: first.p,
        classification,
        disturbance_interval: interval,
        disturbance_intervals: [first.disturbance_interval, second.disturbance_interval],
        disturbance_points: [first.disturbance, second.disturbance],
        disturbance_variances_q2: [
            CompactVariance::encode(first.disturbance_variance_q2),
            CompactVariance::encode(second.disturbance_variance_q2),
        ],
        settled_rate_bounds_q: [
            CompactSignedRate::encode(
                first.settled_rate_mean_q.min(second.settled_rate_mean_q),
                BoundRounding::TowardNegativeInfinity,
            ),
            CompactSignedRate::encode(
                first.settled_rate_mean_q.max(second.settled_rate_mean_q),
                BoundRounding::TowardPositiveInfinity,
            ),
            CompactSignedRate::encode(
                first.settled_rate_interval_q[0].min(second.settled_rate_interval_q[0]),
                BoundRounding::TowardNegativeInfinity,
            ),
            CompactSignedRate::encode(
                first.settled_rate_interval_q[1].max(second.settled_rate_interval_q[1]),
                BoundRounding::TowardPositiveInfinity,
            ),
        ],
        reproduced_moving,
        operable,
        initial_lead_in: first.initial_lead_in || second.initial_lead_in,
    })
}

/// Derive one direction's consensus, pool, operability, and rate bounds.
pub fn analyze_directional_rung(
    collected: &CollectedRungEvidence,
    direction: VelocityDirection,
    p: PidGain,
    target_rpm: i32,
    counts_per_rev: u32,
) -> Result<DirectionalRungAnalysis, VelocityEvidenceError> {
    analyze_directional_set(
        collected.directional_set(direction),
        direction,
        p,
        target_rpm,
        counts_per_rev,
    )
}

/// Derive one direction after the opposite direction has already been folded.
pub fn analyze_directional_set(
    set: DirectionalObservationSet,
    direction: VelocityDirection,
    p: PidGain,
    target_rpm: i32,
    counts_per_rev: u32,
) -> Result<DirectionalRungAnalysis, VelocityEvidenceError> {
    if p == PidGain::ZERO || counts_per_rev == 0 || target_rpm == 0 {
        return Err(VelocityEvidenceError::Arithmetic);
    }
    let classes = core::array::from_fn(|index| {
        set.classification(index)
            .unwrap_or(VelocityObservationClass::InvalidEvidenceFault)
    });
    let intervals = core::array::from_fn(|index| set.disturbance_interval(index));
    let consensus = depth_three_consensus(set.collected_mask(), classes, intervals);
    let operable_count = classes
        .iter()
        .enumerate()
        .filter(|(index, class)| {
            set.collected_mask() & (1 << index) != 0 && observation_is_operable(**class)
        })
        .count() as u8;
    let mut flags = u8::from(operable_count >= 3) * DIRECTIONAL_RUNG_FLAG_OPERABLE;
    if set.initial_lead_in() {
        flags |= DIRECTIONAL_RUNG_FLAG_INITIAL_LEAD_IN;
    }

    let mut pooled_accumulator = PooledDisturbanceAccumulator::new();
    let mut variance_floor_observations = 0u8;
    let mut rate_bounds = [0i64; 4];
    let mut have_rate_bounds = false;
    if consensus.classification == DirectionalConsensusClass::EligibleConsensus {
        let target_magnitude = i32::try_from(target_rpm.unsigned_abs())
            .map_err(|_| VelocityEvidenceError::Arithmetic)?;
        let signed_target_rpm = match direction {
            VelocityDirection::Forward => target_magnitude,
            VelocityDirection::Reverse => -target_magnitude,
        };
        for index in 0..CONSENSUS_OBSERVATIONS_PER_DIRECTION {
            if consensus.included_mask & (1 << index) == 0 {
                continue;
            }
            let observation = set.observations[index];
            let interval = observation
                .interval()
                .ok_or(VelocityEvidenceError::InvalidConsensus)?;
            let point = EquivalentTorqueTargetQ16::from_raw(observation.disturbance_point_q16);
            let variance = observation.variance.decode();
            variance_floor_observations = variance_floor_observations
                .checked_add(u8::from(variance == 0))
                .ok_or(VelocityEvidenceError::Arithmetic)?;
            pooled_accumulator.add(point, interval, variance);

            let point_cell = disturbance_point_cell(observation.disturbance_point_q16)?;
            let mean_bounds = rate_bounds_from_disturbance_cell(
                signed_target_rpm,
                counts_per_rev,
                p.raw(),
                point_cell,
            )?;
            let confidence_bounds = rate_bounds_from_disturbance_cell(
                signed_target_rpm,
                counts_per_rev,
                p.raw(),
                [
                    observation.disturbance_low_q16,
                    observation.disturbance_high_q16,
                ],
            )?;
            if have_rate_bounds {
                rate_bounds[0] = rate_bounds[0].min(mean_bounds[0]);
                rate_bounds[1] = rate_bounds[1].max(mean_bounds[1]);
                rate_bounds[2] = rate_bounds[2].min(confidence_bounds[0]);
                rate_bounds[3] = rate_bounds[3].max(confidence_bounds[1]);
            } else {
                rate_bounds = [
                    mean_bounds[0],
                    mean_bounds[1],
                    confidence_bounds[0],
                    confidence_bounds[1],
                ];
                have_rate_bounds = true;
            }
        }
        if !have_rate_bounds {
            return Err(VelocityEvidenceError::InvalidConsensus);
        }
    }

    let pooled = if consensus.classification == DirectionalConsensusClass::EligibleConsensus {
        pooled_accumulator
            .finish(variance_floor_observations)
            .ok_or(VelocityEvidenceError::Arithmetic)
            .map(Some)?
    } else {
        None
    };
    let compact_rate_bounds = if have_rate_bounds {
        [
            CompactSignedRate::encode(rate_bounds[0], BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(rate_bounds[1], BoundRounding::TowardPositiveInfinity),
            CompactSignedRate::encode(rate_bounds[2], BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(rate_bounds[3], BoundRounding::TowardPositiveInfinity),
        ]
    } else {
        [CompactSignedRate::ZERO; 4]
    };

    Ok(DirectionalRungAnalysis {
        report: CompactDirectionalRungReport {
            p,
            classification: consensus.classification,
            components: consensus.components,
            pooled,
            settled_rate_bounds_q: compact_rate_bounds,
            collected_mask: consensus.collected_mask,
            eligible_mask: consensus.eligible_mask,
            included_mask: consensus.included_mask,
            operable_count,
            flags,
        },
        pooled_accumulator,
    })
}

fn rate_bounds_from_disturbance_cell(
    target_rpm: i32,
    counts_per_rev: u32,
    p_raw: u16,
    disturbance_cell_q16: [i32; 2],
) -> Result<[i64; 2], VelocityEvidenceError> {
    if counts_per_rev == 0 || p_raw == 0 || disturbance_cell_q16[0] > disturbance_cell_q16[1] {
        return Err(VelocityEvidenceError::Arithmetic);
    }
    let target_term = i128::from(target_rpm)
        .checked_mul(i128::from(counts_per_rev))
        .and_then(|value| value.checked_mul(1_000))
        .and_then(|value| value.checked_mul(i128::from(p_raw)))
        .and_then(|value| value.checked_mul(65_536))
        .ok_or(VelocityEvidenceError::Arithmetic)?;
    let denominator = i128::from(p_raw)
        .checked_mul(65_536)
        .and_then(|value| value.checked_mul(60))
        .ok_or(VelocityEvidenceError::Arithmetic)?;
    let numerator = |disturbance: i32| {
        i128::from(disturbance)
            .checked_mul(256)
            .and_then(|value| value.checked_mul(i128::from(counts_per_rev)))
            .and_then(|value| value.checked_mul(1_000))
            .and_then(|value| target_term.checked_sub(value))
            .ok_or(VelocityEvidenceError::Arithmetic)
    };
    let low = floor_ratio(numerator(disturbance_cell_q16[1])?, denominator)
        .ok_or(VelocityEvidenceError::Arithmetic)?;
    let high = ceil_ratio(numerator(disturbance_cell_q16[0])?, denominator)
        .ok_or(VelocityEvidenceError::Arithmetic)?;
    Ok([
        i64::try_from(low).map_err(|_| VelocityEvidenceError::Arithmetic)?,
        i64::try_from(high).map_err(|_| VelocityEvidenceError::Arithmetic)?,
    ])
}

fn disturbance_point_cell(point: i32) -> Result<[i32; 2], VelocityEvidenceError> {
    match point.cmp(&0) {
        core::cmp::Ordering::Greater => point
            .checked_add(1)
            .map(|high| [point, high])
            .ok_or(VelocityEvidenceError::Arithmetic),
        core::cmp::Ordering::Less => point
            .checked_sub(1)
            .map(|low| [low, point])
            .ok_or(VelocityEvidenceError::Arithmetic),
        core::cmp::Ordering::Equal => Ok([-1, 1]),
    }
}

fn floor_ratio(numerator: i128, denominator: i128) -> Option<i128> {
    let quotient = numerator.checked_div(denominator)?;
    let remainder = numerator.checked_rem(denominator)?;
    if remainder != 0 && numerator < 0 {
        quotient.checked_sub(1)
    } else {
        Some(quotient)
    }
}

fn ceil_ratio(numerator: i128, denominator: i128) -> Option<i128> {
    let quotient = numerator.checked_div(denominator)?;
    let remainder = numerator.checked_rem(denominator)?;
    if remainder != 0 && numerator > 0 {
        quotient.checked_add(1)
    } else {
        Some(quotient)
    }
}

const fn direction_index(direction: VelocityDirection) -> usize {
    match direction {
        VelocityDirection::Forward => 0,
        VelocityDirection::Reverse => 1,
    }
}

fn observation_class_from_report_code(code: u8) -> Option<VelocityObservationClass> {
    Some(match code {
        0 => VelocityObservationClass::Eligible,
        1 => VelocityObservationClass::Inert,
        2 => VelocityObservationClass::DeficitUnresolved,
        3 => VelocityObservationClass::CurrentHeadroom,
        4 => {
            VelocityObservationClass::StationarityExcluded(StationarityExclusion::NoBlockingPlateau)
        }
        5 => VelocityObservationClass::StationarityExcluded(
            StationarityExclusion::TooFewDecorrelatedBlocks,
        ),
        6 => VelocityObservationClass::StationarityExcluded(StationarityExclusion::ResolvedSlope),
        7 => VelocityObservationClass::StationarityExcluded(
            StationarityExclusion::StructuredResidual,
        ),
        8 => VelocityObservationClass::StationarityExcluded(StationarityExclusion::Arithmetic),
        9 => VelocityObservationClass::WrongDirectionFault,
        10 => VelocityObservationClass::HardClipFault,
        11 => VelocityObservationClass::InvalidEvidenceFault,
        _ => return None,
    })
}

fn observation_is_operable(classification: VelocityObservationClass) -> bool {
    matches!(
        classification,
        VelocityObservationClass::Eligible | VelocityObservationClass::DeficitUnresolved
    )
}

fn observation_resolves_motion(classification: VelocityObservationClass) -> bool {
    matches!(
        classification,
        VelocityObservationClass::Eligible
            | VelocityObservationClass::DeficitUnresolved
            | VelocityObservationClass::CurrentHeadroom
    )
}

fn observation_is_fault(classification: VelocityObservationClass) -> bool {
    matches!(
        classification,
        VelocityObservationClass::WrongDirectionFault
            | VelocityObservationClass::HardClipFault
            | VelocityObservationClass::InvalidEvidenceFault
    )
}

fn intersect_intervals(
    first: Option<EquivalentTorqueTargetInterval>,
    second: Option<EquivalentTorqueTargetInterval>,
) -> Option<EquivalentTorqueTargetInterval> {
    let first = first?;
    let second = second?;
    let low = first.low().raw().max(second.low().raw());
    let high = first.high().raw().min(second.high().raw());
    EquivalentTorqueTargetInterval::new(
        EquivalentTorqueTargetQ16::from_raw(low),
        EquivalentTorqueTargetQ16::from_raw(high),
    )
    .ok()
}

/// Reason the maximal contiguous directional band stopped growing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BandClosure {
    /// Every executed rung remained in the contiguous band.
    EndOfLadder,
    /// No same-direction pair confirmed motion.
    NoMovingEdge,
    /// The corrected deficit ceased to exclude zero.
    DeficitResolution,
    /// Observation reproduction or stationarity left a quality gap.
    ReproductionGap,
    /// Adding the next rung emptied the common D_eq intersection.
    Invariance,
    /// The inclusive usable-current boundary ended the evidence span.
    CurrentHeadroom,
    /// Structural evidence was invalid or unsafe.
    Fault,
}

/// Stable report code for one directional-band closure.
pub const fn band_closure_report_code(closure: BandClosure) -> u8 {
    match closure {
        BandClosure::EndOfLadder => 0,
        BandClosure::NoMovingEdge => 1,
        BandClosure::DeficitResolution => 2,
        BandClosure::ReproductionGap => 3,
        BandClosure::Invariance => 4,
        BandClosure::CurrentHeadroom => 5,
        BandClosure::Fault => 6,
    }
}

/// Fixed-effect pooled disturbance estimate kept separate from invariance evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PooledDisturbance {
    /// Inverse-variance weighted controller-domain point.
    pub estimate: EquivalentTorqueTargetQ16,
    /// Outward-rounded weighted interval.
    pub interval: EquivalentTorqueTargetInterval,
    /// Number of observations whose zero variance used the one-unit floor.
    pub variance_floor_observations: u8,
}

/// Maximal contiguous D_eq-identifiable band for one signed direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalDisturbanceBand {
    /// Signed direction represented by this band.
    pub direction: VelocityDirection,
    /// Last confirmed inert gain below the moving edge.
    pub last_inert: Option<PidGain>,
    /// First gain with reproduced commanded-side motion.
    pub first_moving: Option<PidGain>,
    /// Ambiguous gain observed around the lower edge.
    pub ambiguous_edge: Option<PidGain>,
    /// Whether the first structural gain already moved.
    pub left_censored: bool,
    /// First D_eq-eligible gain.
    pub p_low: Option<PidGain>,
    /// Last contiguous D_eq-eligible gain.
    pub p_high: Option<PidGain>,
    /// Condition that closed the band.
    pub closure: BandClosure,
    /// Common interval compatible with every included rung.
    pub common_intersection: Option<EquivalentTorqueTargetInterval>,
    /// Fixed-effect pooled estimate over included observations.
    pub pooled: Option<PooledDisturbance>,
    /// Number of contiguous eligible rungs.
    pub eligible_rungs: u8,
    /// Number of independently corrected observations pooled.
    pub eligible_observations: u16,
    /// Handoff sufficiency, not gain acceptance.
    pub sufficient: bool,
}

/// Cross-direction P-band geometry without merging signed disturbance values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct JointIdentifiableGeometry {
    /// Lower jointly-identifiable gain when bands overlap.
    pub low: Option<PidGain>,
    /// Upper jointly-identifiable gain when bands overlap.
    pub high: Option<PidGain>,
    /// Lower edge of the gap when bands do not overlap.
    pub gap_low: Option<PidGain>,
    /// Upper edge of the gap when bands do not overlap.
    pub gap_high: Option<PidGain>,
    /// Whether a shared identifiable P exists.
    pub has_joint_band: bool,
}

impl JointIdentifiableGeometry {
    /// Derive shared P geometry while retaining the two directional bands.
    pub fn from_bands(
        forward: &DirectionalDisturbanceBand,
        reverse: &DirectionalDisturbanceBand,
    ) -> Self {
        let (Some(forward_low), Some(forward_high), Some(reverse_low), Some(reverse_high)) =
            (forward.p_low, forward.p_high, reverse.p_low, reverse.p_high)
        else {
            return Self {
                low: None,
                high: None,
                gap_low: None,
                gap_high: None,
                has_joint_band: false,
            };
        };
        let low = if forward_low.raw() >= reverse_low.raw() {
            forward_low
        } else {
            reverse_low
        };
        let high = if forward_high.raw() <= reverse_high.raw() {
            forward_high
        } else {
            reverse_high
        };
        if low.raw() <= high.raw() {
            Self {
                low: Some(low),
                high: Some(high),
                gap_low: None,
                gap_high: None,
                has_joint_band: true,
            }
        } else {
            Self {
                low: None,
                high: None,
                gap_low: Some(high),
                gap_high: Some(low),
                has_joint_band: false,
            }
        }
    }
}

/// Why otherwise-safe evidence could not advance to the next stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityInconclusiveKind {
    /// Exactly one direction met the handoff threshold.
    PartialDirection,
    /// Neither direction met the handoff threshold.
    Total,
}

/// Bidirectional evidence-sufficiency result; this is not gain acceptance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocitySweepSufficiency {
    /// Both directions contain enough evidence to hand off.
    Complete {
        /// Shared P geometry, which may be empty without blocking handoff.
        joint: JointIdentifiableGeometry,
    },
    /// Measurement stayed safe but one or both directions lack evidence.
    Inconclusive {
        /// Bit zero is forward and bit one is reverse.
        sufficient_direction_mask: u8,
        /// Partial-versus-total diagnostic.
        kind: VelocityInconclusiveKind,
        /// Shared geometry retained for the detailed report.
        joint: JointIdentifiableGeometry,
    },
}

/// Assemble the maximal contiguous band from already-executed rung verdicts.
pub fn assemble_directional_band(
    direction: VelocityDirection,
    rungs: &[VelocityRungVerdict],
) -> DirectionalDisturbanceBand {
    let mut band = DirectionalBandAccumulator::new(direction);
    for (index, rung) in rungs.iter().enumerate() {
        band.observe(index, *rung);
    }
    band.finish()
}

/// Classify bidirectional handoff sufficiency without applying an acceptance threshold.
pub fn assess_sweep_sufficiency(
    forward: &DirectionalDisturbanceBand,
    reverse: &DirectionalDisturbanceBand,
) -> VelocitySweepSufficiency {
    let joint = JointIdentifiableGeometry::from_bands(forward, reverse);
    let mask = u8::from(forward.sufficient) | (u8::from(reverse.sufficient) << 1);
    if mask == 3 {
        VelocitySweepSufficiency::Complete { joint }
    } else {
        VelocitySweepSufficiency::Inconclusive {
            sufficient_direction_mask: mask,
            kind: if mask == 0 {
                VelocityInconclusiveKind::Total
            } else {
                VelocityInconclusiveKind::PartialDirection
            },
            joint,
        }
    }
}

/// Fixed-memory streaming accumulator for one directional disturbance band.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalBandAccumulator {
    band: DirectionalDisturbanceBand,
    closed: bool,
    pooled: PooledDisturbanceAccumulator,
    variance_floor_observations: u8,
}

impl DirectionalBandAccumulator {
    /// Construct an empty directional band accumulator.
    pub const fn new(direction: VelocityDirection) -> Self {
        Self {
            band: DirectionalDisturbanceBand {
                direction,
                last_inert: None,
                first_moving: None,
                ambiguous_edge: None,
                left_censored: false,
                p_low: None,
                p_high: None,
                closure: BandClosure::NoMovingEdge,
                common_intersection: None,
                pooled: None,
                eligible_rungs: 0,
                eligible_observations: 0,
                sufficient: false,
            },
            closed: false,
            pooled: PooledDisturbanceAccumulator::new(),
            variance_floor_observations: 0,
        }
    }

    /// Fold one already-executed rung into the maximal contiguous band.
    pub fn observe(&mut self, index: usize, rung: VelocityRungVerdict) {
        if self.closed {
            return;
        }
        if rung.direction != self.band.direction || rung.classification == VelocityRungClass::Fault
        {
            self.band.closure = BandClosure::Fault;
            self.closed = true;
            return;
        }
        if self.band.first_moving.is_none() {
            if rung.classification == VelocityRungClass::Inert {
                self.band.last_inert = Some(rung.p);
                return;
            }
            if !rung.reproduced_moving {
                self.band.ambiguous_edge = Some(rung.p);
                return;
            }
            self.band.first_moving = Some(rung.p);
            self.band.left_censored = index == 0;
        }
        if rung.classification != VelocityRungClass::Eligible {
            self.band.closure = closure_for(rung.classification);
            self.closed = true;
            return;
        }
        let Some(rung_interval) = rung.disturbance_interval else {
            self.band.closure = BandClosure::Fault;
            self.closed = true;
            return;
        };
        let next_common = self
            .band
            .common_intersection
            .map_or(Some(rung_interval), |current| {
                intersect_intervals(Some(current), Some(rung_interval))
            });
        let Some(common) = next_common else {
            self.band.closure = BandClosure::Invariance;
            self.closed = true;
            return;
        };
        self.band.common_intersection = Some(common);
        self.band.p_low.get_or_insert(rung.p);
        self.band.p_high = Some(rung.p);
        self.band.eligible_rungs = self.band.eligible_rungs.saturating_add(1);
        for observation in 0..2 {
            self.add_observation(
                rung.disturbance_points[observation],
                rung.disturbance_intervals[observation],
                rung.disturbance_variances_q2[observation].decode(),
            );
        }
    }

    fn add_observation(
        &mut self,
        point: Option<EquivalentTorqueTargetQ16>,
        interval: Option<EquivalentTorqueTargetInterval>,
        variance_q2: u64,
    ) {
        let (Some(point), Some(interval)) = (point, interval) else {
            self.band.closure = BandClosure::Fault;
            self.closed = true;
            return;
        };
        self.variance_floor_observations = self
            .variance_floor_observations
            .saturating_add(u8::from(variance_q2 == 0));
        self.pooled.add(point, interval, variance_q2);
        self.band.eligible_observations = self.band.eligible_observations.saturating_add(1);
    }

    /// Return the current reportable band without consuming the accumulator.
    pub fn snapshot(self) -> DirectionalDisturbanceBand {
        self.finish()
    }

    fn finish(mut self) -> DirectionalDisturbanceBand {
        if !self.closed && self.band.first_moving.is_some() {
            self.band.closure = BandClosure::EndOfLadder;
        }
        self.band.pooled = self.pooled.finish(self.variance_floor_observations);
        self.band.sufficient = self.band.first_moving.is_some()
            && self.band.eligible_rungs >= 3
            && self.band.pooled.is_some();
        self.band
    }
}

fn closure_for(classification: VelocityRungClass) -> BandClosure {
    match classification {
        VelocityRungClass::DeficitUnresolved => BandClosure::DeficitResolution,
        VelocityRungClass::CurrentHeadroom => BandClosure::CurrentHeadroom,
        VelocityRungClass::Fault => BandClosure::Fault,
        VelocityRungClass::Eligible => BandClosure::EndOfLadder,
        VelocityRungClass::Inert
        | VelocityRungClass::DisjointPair
        | VelocityRungClass::Ambiguous
        | VelocityRungClass::Excluded => BandClosure::ReproductionGap,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        VelocityEvidenceError, VelocityObservationClass, VelocityObservationInput,
        VelocityRungClass, classify_velocity_observation, disturbance_point_cell,
        rate_bounds_from_disturbance_cell, reproduce_velocity_rung,
    };
    use crate::autotune::velocity_measurement::{
        PlateauSelection, RateBlock, StationarityExclusion, StationarityResult, StationarySuffix,
        VelocityDirection,
    };
    use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
    use foci_tmc4671::pid::PidGain;

    fn stationary() -> StationarityResult {
        StationarityResult::Usable(StationarySuffix {
            selection: PlateauSelection {
                level: 1,
                block_count: 16,
                variance_of_mean_q2: 4,
                exact_totals: RateBlock {
                    delta_counts: 1,
                    elapsed_us: 1_000,
                    observation_count: 64,
                },
            },
            suffix_len: 64,
            slope_q: 0,
            slope_half_width_q: 1,
            lag_one_q: 0,
            lag_one_half_width_q: 1,
            residual_magnitude_q2: 1,
            tested_suffixes: 1,
        })
    }

    fn interval(low: i32, high: i32) -> EquivalentTorqueTargetInterval {
        EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(low),
            EquivalentTorqueTargetQ16::from_raw(high),
        )
        .unwrap()
    }

    #[test]
    fn compact_consensus_rate_inversion_is_outward_and_sign_correct() {
        assert_eq!(
            rate_bounds_from_disturbance_cell(176, 4_000, 256, [0, 0]),
            Ok([11_733_333, 11_733_334])
        );
        assert_eq!(
            rate_bounds_from_disturbance_cell(176, 4_000, 256, [65_536, 65_536]),
            Ok([11_666_666, 11_666_667])
        );
        assert_eq!(
            rate_bounds_from_disturbance_cell(176, 4_000, 256, [-65_536, -65_536]),
            Ok([11_800_000, 11_800_000])
        );
    }

    #[test]
    fn compact_consensus_point_cells_cover_positive_negative_and_zero_truncation() {
        assert_eq!(disturbance_point_cell(8), Ok([8, 9]));
        assert_eq!(disturbance_point_cell(-8), Ok([-9, -8]));
        assert_eq!(disturbance_point_cell(0), Ok([-1, 1]));
        assert_eq!(
            disturbance_point_cell(i32::MAX),
            Err(VelocityEvidenceError::Arithmetic)
        );
        assert_eq!(
            disturbance_point_cell(i32::MIN),
            Err(VelocityEvidenceError::Arithmetic)
        );
    }

    fn input(direction: VelocityDirection) -> VelocityObservationInput {
        let (settled_rate_low_q, settled_rate_high_q, deficit_low_q, deficit_high_q, point, ci) =
            match direction {
                VelocityDirection::Forward => (10, 20, 2, 4, 100, interval(90, 110)),
                VelocityDirection::Reverse => (-20, -10, -4, -2, -100, interval(-110, -90)),
            };
        VelocityObservationInput {
            direction,
            p: PidGain::new(350).unwrap(),
            stationarity: stationary(),
            settled_rate_low_q,
            settled_rate_high_q,
            settled_rate_mean_q: (settled_rate_low_q + settled_rate_high_q) / 2,
            deficit_low_q,
            deficit_high_q,
            disturbance: Some((EquivalentTorqueTargetQ16::from_raw(point), ci)),
            disturbance_variance_q2: 4,
            predicted_torque_target_abs: 500,
            usable_torque_limit: 900,
            hard_torque_limit: 1_000,
            initial_lead_in: false,
        }
    }

    #[test]
    fn commanded_side_motion_and_deficit_are_eligible_per_direction() {
        let forward = classify_velocity_observation(input(VelocityDirection::Forward));
        let mut reverse_input = input(VelocityDirection::Reverse);
        reverse_input.settled_rate_low_q = -20;
        reverse_input.settled_rate_high_q = -10;
        reverse_input.deficit_low_q = -4;
        reverse_input.deficit_high_q = -2;
        reverse_input.disturbance = Some((
            EquivalentTorqueTargetQ16::from_raw(-100),
            interval(-110, -90),
        ));
        let reverse = classify_velocity_observation(reverse_input);

        assert_eq!(forward.classification, VelocityObservationClass::Eligible);
        assert_eq!(reverse.classification, VelocityObservationClass::Eligible);
    }

    #[test]
    fn opposite_sign_motion_is_a_fault() {
        let mut wrong = input(VelocityDirection::Forward);
        wrong.settled_rate_low_q = -20;
        wrong.settled_rate_high_q = -10;

        assert_eq!(
            classify_velocity_observation(wrong).classification,
            VelocityObservationClass::WrongDirectionFault
        );
    }

    #[test]
    fn zero_crossing_motion_and_deficit_are_distinct_exclusions() {
        let mut inert = input(VelocityDirection::Forward);
        inert.settled_rate_low_q = -1;
        let mut unresolved = input(VelocityDirection::Forward);
        unresolved.deficit_low_q = -1;

        assert_eq!(
            classify_velocity_observation(inert).classification,
            VelocityObservationClass::Inert
        );
        assert_eq!(
            classify_velocity_observation(unresolved).classification,
            VelocityObservationClass::DeficitUnresolved
        );
    }

    #[test]
    fn stationarity_exclusion_survives_observation_classification() {
        let mut excluded = input(VelocityDirection::Forward);
        excluded.stationarity = StationarityResult::Excluded(StationarityExclusion::ResolvedSlope);

        assert_eq!(
            classify_velocity_observation(excluded).classification,
            VelocityObservationClass::StationarityExcluded(StationarityExclusion::ResolvedSlope)
        );
    }

    #[test]
    fn current_headroom_is_inclusive_and_hard_clip_remains_a_fault() {
        let mut headroom = input(VelocityDirection::Forward);
        headroom.predicted_torque_target_abs = 900;
        let mut clip = input(VelocityDirection::Forward);
        clip.predicted_torque_target_abs = 1_000;

        assert_eq!(
            classify_velocity_observation(headroom).classification,
            VelocityObservationClass::CurrentHeadroom
        );
        assert_eq!(
            classify_velocity_observation(clip).classification,
            VelocityObservationClass::HardClipFault
        );
    }

    #[test]
    fn reproduced_pair_requires_two_passes_and_overlapping_intervals() {
        let first = classify_velocity_observation(input(VelocityDirection::Forward));
        let mut second_input = input(VelocityDirection::Forward);
        second_input.disturbance =
            Some((EquivalentTorqueTargetQ16::from_raw(105), interval(100, 120)));
        let second = classify_velocity_observation(second_input);
        let reproduced = reproduce_velocity_rung(first, second).unwrap();
        assert_eq!(reproduced.classification, VelocityRungClass::Eligible);
        assert_eq!(reproduced.disturbance_interval, Some(interval(100, 110)));
        assert!(reproduced.operable);

        let mut disjoint_input = input(VelocityDirection::Forward);
        disjoint_input.disturbance =
            Some((EquivalentTorqueTargetQ16::from_raw(200), interval(190, 210)));
        let disjoint =
            reproduce_velocity_rung(first, classify_velocity_observation(disjoint_input)).unwrap();
        assert_eq!(disjoint.classification, VelocityRungClass::DisjointPair);
        assert!(disjoint.operable);

        let mut inert_input = input(VelocityDirection::Forward);
        inert_input.settled_rate_low_q = -1;
        let one_pass =
            reproduce_velocity_rung(first, classify_velocity_observation(inert_input)).unwrap();
        assert_eq!(one_pass.classification, VelocityRungClass::Ambiguous);
        assert!(!one_pass.operable);
    }

    #[test]
    fn initial_lead_in_is_preserved_for_rung_zero_audit() {
        let mut initial = input(VelocityDirection::Forward);
        initial.initial_lead_in = true;
        let summary = classify_velocity_observation(initial);

        assert!(summary.initial_lead_in);
    }
}

#[cfg(test)]
#[path = "sweep_band_tests.rs"]
mod band_tests;
