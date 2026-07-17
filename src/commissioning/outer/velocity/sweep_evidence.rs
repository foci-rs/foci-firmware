use crate::autotune::velocity_measurement::{
    StationarityExclusion, StationarityResult, VelocityDirection,
};
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
use foci_tmc4671::pid::PidGain;

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
    pub disturbance_variances_q2: [u64; 2],
    /// Whether both observations resolved commanded-side motion.
    pub reproduced_moving: bool,
    /// Whether either observation carried the rung-zero lead-in marker.
    pub initial_lead_in: bool,
}

/// Failure to combine two summaries that do not describe one rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityEvidenceError {
    /// Direction or gain differed between the two observations.
    MismatchedPair,
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
        return Err(VelocityEvidenceError::MismatchedPair);
    }
    let first_moving = observation_resolves_motion(first.classification);
    let second_moving = observation_resolves_motion(second.classification);
    let reproduced_moving = first_moving && second_moving;
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
            first.disturbance_variance_q2,
            second.disturbance_variance_q2,
        ],
        reproduced_moving,
        initial_lead_in: first.initial_lead_in || second.initial_lead_in,
    })
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
    let mut band = BandAccumulator::new(direction);
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

struct BandAccumulator {
    band: DirectionalDisturbanceBand,
    closed: bool,
    weighted_point: i128,
    weighted_low: i128,
    weighted_high: i128,
    weight_sum: i128,
    variance_floor_observations: u8,
}

impl BandAccumulator {
    fn new(direction: VelocityDirection) -> Self {
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
            weighted_point: 0,
            weighted_low: 0,
            weighted_high: 0,
            weight_sum: 0,
            variance_floor_observations: 0,
        }
    }

    fn observe(&mut self, index: usize, rung: VelocityRungVerdict) {
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
                rung.disturbance_variances_q2[observation],
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
        let variance = variance_q2.max(1);
        self.variance_floor_observations = self
            .variance_floor_observations
            .saturating_add(u8::from(variance_q2 == 0));
        let weight = ((1_u64 << 32) / variance).max(1);
        let weight = i128::from(weight);
        self.weighted_point += i128::from(point.raw()) * weight;
        self.weighted_low += i128::from(interval.low().raw()) * weight;
        self.weighted_high += i128::from(interval.high().raw()) * weight;
        self.weight_sum += weight;
        self.band.eligible_observations = self.band.eligible_observations.saturating_add(1);
    }

    fn finish(mut self) -> DirectionalDisturbanceBand {
        if !self.closed && self.band.first_moving.is_some() {
            self.band.closure = BandClosure::EndOfLadder;
        }
        if self.weight_sum > 0 {
            let estimate = i32::try_from(self.weighted_point / self.weight_sum).ok();
            let low =
                floor_ratio(self.weighted_low, self.weight_sum).and_then(|v| i32::try_from(v).ok());
            let high =
                ceil_ratio(self.weighted_high, self.weight_sum).and_then(|v| i32::try_from(v).ok());
            if let (Some(estimate), Some(low), Some(high)) = (estimate, low, high) {
                let estimate = EquivalentTorqueTargetQ16::from_raw(estimate);
                if let Ok(interval) = EquivalentTorqueTargetInterval::new(
                    EquivalentTorqueTargetQ16::from_raw(low),
                    EquivalentTorqueTargetQ16::from_raw(high),
                ) {
                    self.band.pooled = Some(PooledDisturbance {
                        estimate,
                        interval,
                        variance_floor_observations: self.variance_floor_observations,
                    });
                }
            }
        }
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

fn floor_ratio(numerator: i128, denominator: i128) -> Option<i128> {
    let quotient = numerator.checked_div(denominator)?;
    let remainder = numerator.checked_rem(denominator)?;
    (remainder != 0 && numerator < 0)
        .then(|| quotient.checked_sub(1))
        .flatten()
        .or(Some(quotient))
}

fn ceil_ratio(numerator: i128, denominator: i128) -> Option<i128> {
    let quotient = numerator.checked_div(denominator)?;
    let remainder = numerator.checked_rem(denominator)?;
    (remainder != 0 && numerator > 0)
        .then(|| quotient.checked_add(1))
        .flatten()
        .or(Some(quotient))
}

#[cfg(test)]
mod tests {
    use super::{
        VelocityObservationClass, VelocityObservationInput, VelocityRungClass,
        classify_velocity_observation, reproduce_velocity_rung,
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

        let mut disjoint_input = input(VelocityDirection::Forward);
        disjoint_input.disturbance =
            Some((EquivalentTorqueTargetQ16::from_raw(200), interval(190, 210)));
        let disjoint =
            reproduce_velocity_rung(first, classify_velocity_observation(disjoint_input)).unwrap();
        assert_eq!(disjoint.classification, VelocityRungClass::DisjointPair);

        let mut inert_input = input(VelocityDirection::Forward);
        inert_input.settled_rate_low_q = -1;
        let one_pass =
            reproduce_velocity_rung(first, classify_velocity_observation(inert_input)).unwrap();
        assert_eq!(one_pass.classification, VelocityRungClass::Ambiguous);
    }

    #[test]
    fn initial_lead_in_is_preserved_for_rung_zero_audit() {
        let mut initial = input(VelocityDirection::Forward);
        initial.initial_lead_in = true;
        let summary = classify_velocity_observation(initial);

        assert!(summary.initial_lead_in);
    }
}
