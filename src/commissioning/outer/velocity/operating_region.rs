use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
use foci_tmc4671::pid::PidGain;

use super::sweep_evidence::{PooledDisturbance, VelocityRungClass, VelocityRungVerdict};

/// Minimum contiguous rung population supporting a directional constant model.
pub const MIN_DIRECTIONAL_REGION_RUNGS: u8 = 3;

/// Conservative direction used when compacting one signed rate bound.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum BoundRounding {
    /// Decode no greater than the original value.
    TowardNegativeInfinity,
    /// Decode no less than the original value.
    TowardPositiveInfinity,
}

/// Conservatively rounded signed rate bound with a binary scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CompactSignedRate {
    /// Signed compact mantissa.
    pub mantissa: i16,
    /// Binary left shift applied while decoding.
    pub shift: u8,
    /// Direction used if compaction discarded low bits.
    pub rounding: BoundRounding,
}

impl CompactSignedRate {
    /// Encode one signed value while preserving the requested conservative bound.
    pub fn encode(value: i64, rounding: BoundRounding) -> Self {
        for shift in 0..=48 {
            let denominator = 1_i128 << shift;
            let value = i128::from(value);
            let quotient = value / denominator;
            let remainder = value % denominator;
            let rounded = match rounding {
                BoundRounding::TowardNegativeInfinity if value < 0 && remainder != 0 => {
                    quotient - 1
                }
                BoundRounding::TowardPositiveInfinity if value > 0 && remainder != 0 => {
                    quotient + 1
                }
                _ => quotient,
            };
            if let Ok(mantissa) = i16::try_from(rounded) {
                return Self {
                    mantissa,
                    shift,
                    rounding,
                };
            }
        }
        unreachable!("an i64 always fits an i16 mantissa with a binary shift")
    }

    /// Decode the conservative signed bound.
    pub const fn decode(self) -> i64 {
        (self.mantissa as i64) << self.shift
    }
}

const _: () = assert!(core::mem::size_of::<CompactSignedRate>() <= 4);

/// Candidate classification under the deterministic first-fit fold.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum DirectionalRegionKind {
    /// At least three contiguous eligible rungs share one disturbance value.
    Valid,
    /// One or two eligible rungs were preserved as boundary evidence.
    Fragment,
}

/// Reason one directional candidate stopped growing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum DirectionalRegionClosure {
    /// The next eligible interval was disjoint from the running intersection.
    DisjointInterval,
    /// An ineligible rung separated otherwise eligible evidence.
    IneligibleGap,
    /// The inclusive usable-current boundary ended the ladder.
    CurrentHeadroom,
    /// A fault or safety terminus ended the ladder.
    SafetyTerminus,
    /// The predeclared ladder ended normally.
    EndOfLadder,
}

/// Fixed-effect inverse-variance pooling state with no retained observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PooledDisturbanceAccumulator {
    weighted_point: i128,
    weighted_low: i128,
    weighted_high: i128,
    weight_sum: i128,
}

impl PooledDisturbanceAccumulator {
    /// Construct an empty accumulator.
    pub const fn new() -> Self {
        Self {
            weighted_point: 0,
            weighted_low: 0,
            weighted_high: 0,
            weight_sum: 0,
        }
    }

    /// Add one observation using the delivered one-unit variance floor.
    pub fn add(
        &mut self,
        point: EquivalentTorqueTargetQ16,
        interval: EquivalentTorqueTargetInterval,
        variance_q2: u64,
    ) {
        let variance = variance_q2.max(1);
        let weight = i128::from(((1_u64 << 32) / variance).max(1));
        self.weighted_point += i128::from(point.raw()) * weight;
        self.weighted_low += i128::from(interval.low().raw()) * weight;
        self.weighted_high += i128::from(interval.high().raw()) * weight;
        self.weight_sum += weight;
    }

    /// Finish the fixed-effect estimate with outward-rounded interval edges.
    pub fn finish(self, variance_floor_observations: u8) -> Option<PooledDisturbance> {
        if self.weight_sum == 0 {
            return None;
        }
        let estimate = i32::try_from(self.weighted_point / self.weight_sum).ok()?;
        let low = i32::try_from(floor_ratio(self.weighted_low, self.weight_sum)?).ok()?;
        let high = i32::try_from(ceil_ratio(self.weighted_high, self.weight_sum)?).ok()?;
        let interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(low),
            EquivalentTorqueTargetQ16::from_raw(high),
        )
        .ok()?;
        Some(PooledDisturbance {
            estimate: EquivalentTorqueTargetQ16::from_raw(estimate),
            interval,
            variance_floor_observations,
        })
    }
}

impl Default for PooledDisturbanceAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<PooledDisturbanceAccumulator>() <= 64);

/// Complete evidence emitted when one directional candidate closes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalRegionSummary {
    /// Signed mechanical direction.
    pub direction: VelocityDirection,
    /// Exact contiguous rung-membership bitmap.
    pub member_mask: u32,
    /// First member rung index.
    pub first_rung: u8,
    /// Last member rung index.
    pub last_rung: u8,
    /// First member gain.
    pub p_low: PidGain,
    /// Last member gain.
    pub p_high: PidGain,
    /// Exact member population.
    pub member_count: u8,
    /// Valid region or retained fragment.
    pub kind: DirectionalRegionKind,
    /// Final common disturbance intersection.
    pub common_intersection: EquivalentTorqueTargetInterval,
    /// Fixed-effect pooled disturbance estimate.
    pub pooled: PooledDisturbance,
    /// Conservative minimum member settled-rate mean.
    pub settled_rate_mean_min_q: i64,
    /// Conservative maximum member settled-rate mean.
    pub settled_rate_mean_max_q: i64,
    /// Conservative minimum member settled-rate confidence edge.
    pub settled_rate_envelope_low_q: i64,
    /// Conservative maximum member settled-rate confidence edge.
    pub settled_rate_envelope_high_q: i64,
    /// Why this candidate closed.
    pub closure: DirectionalRegionClosure,
    /// Rung that forced closure, when closure preceded ladder end.
    pub boundary_rung: Option<u8>,
    /// Final narrowed intersection tested at the boundary.
    pub tested_intersection: EquivalentTorqueTargetInterval,
}

/// Fixed-memory active state for one directional first-fit fold.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalRegionFold {
    pooled: PooledDisturbanceAccumulator,
    common_intersection: EquivalentTorqueTargetInterval,
    rate_bounds: [CompactSignedRate; 4],
    p_low: PidGain,
    p_high: PidGain,
    first_rung: u8,
    member_count: u8,
    flags: u8,
    variance_floor_observations: u8,
}

impl DirectionalRegionFold {
    const ACTIVE: u8 = 1;
    const REVERSE: u8 = 2;

    /// Construct an empty fold for one signed direction.
    pub const fn new(direction: VelocityDirection) -> Self {
        let flags = match direction {
            VelocityDirection::Forward => 0,
            VelocityDirection::Reverse => Self::REVERSE,
        };
        let zero_rate = CompactSignedRate {
            mantissa: 0,
            shift: 0,
            rounding: BoundRounding::TowardNegativeInfinity,
        };
        Self {
            pooled: PooledDisturbanceAccumulator::new(),
            common_intersection: EquivalentTorqueTargetInterval::point(
                EquivalentTorqueTargetQ16::from_raw(0),
            ),
            rate_bounds: [zero_rate; 4],
            p_low: PidGain::ZERO,
            p_high: PidGain::ZERO,
            first_rung: 0,
            member_count: 0,
            flags,
            variance_floor_observations: 0,
        }
    }

    /// Close the active candidate at a declared ladder terminus.
    pub fn finish(
        &mut self,
        closure: DirectionalRegionClosure,
    ) -> Option<DirectionalRegionSummary> {
        self.close(closure, None)
    }

    fn direction(&self) -> VelocityDirection {
        if self.flags & Self::REVERSE == 0 {
            VelocityDirection::Forward
        } else {
            VelocityDirection::Reverse
        }
    }

    fn is_active(&self) -> bool {
        self.flags & Self::ACTIVE != 0
    }

    fn clear(&mut self) {
        let direction = self.direction();
        *self = Self::new(direction);
    }

    fn seed(&mut self, rung_index: u8, rung: VelocityRungVerdict) -> bool {
        let Some(interval) = rung.disturbance_interval else {
            return false;
        };
        self.common_intersection = interval;
        self.p_low = rung.p;
        self.p_high = rung.p;
        self.first_rung = rung_index;
        self.member_count = 1;
        self.flags |= Self::ACTIVE;
        self.set_rate_bounds(rung);
        self.add_rung_to_pool(rung)
    }

    fn append(&mut self, rung: VelocityRungVerdict, intersection: EquivalentTorqueTargetInterval) {
        self.common_intersection = intersection;
        self.p_high = rung.p;
        self.member_count = self.member_count.saturating_add(1);
        self.extend_rate_bounds(rung);
        let _ = self.add_rung_to_pool(rung);
    }

    fn add_rung_to_pool(&mut self, rung: VelocityRungVerdict) -> bool {
        for observation in 0..2 {
            let (Some(point), Some(interval)) = (
                rung.disturbance_points[observation],
                rung.disturbance_intervals[observation],
            ) else {
                return false;
            };
            let variance = rung.disturbance_variances_q2[observation];
            self.variance_floor_observations = self
                .variance_floor_observations
                .saturating_add(u8::from(variance == 0));
            self.pooled.add(point, interval, variance);
        }
        true
    }

    fn set_rate_bounds(&mut self, rung: VelocityRungVerdict) {
        let means = rung.settled_rate_means_q;
        let intervals = rung.settled_rate_intervals_q;
        self.rate_bounds = [
            CompactSignedRate::encode(
                means[0].min(means[1]),
                BoundRounding::TowardNegativeInfinity,
            ),
            CompactSignedRate::encode(
                means[0].max(means[1]),
                BoundRounding::TowardPositiveInfinity,
            ),
            CompactSignedRate::encode(
                intervals[0][0].min(intervals[1][0]),
                BoundRounding::TowardNegativeInfinity,
            ),
            CompactSignedRate::encode(
                intervals[0][1].max(intervals[1][1]),
                BoundRounding::TowardPositiveInfinity,
            ),
        ];
    }

    fn extend_rate_bounds(&mut self, rung: VelocityRungVerdict) {
        let values = [
            rung.settled_rate_means_q[0].min(rung.settled_rate_means_q[1]),
            rung.settled_rate_means_q[0].max(rung.settled_rate_means_q[1]),
            rung.settled_rate_intervals_q[0][0].min(rung.settled_rate_intervals_q[1][0]),
            rung.settled_rate_intervals_q[0][1].max(rung.settled_rate_intervals_q[1][1]),
        ];
        if values[0] < self.rate_bounds[0].decode() {
            self.rate_bounds[0] =
                CompactSignedRate::encode(values[0], BoundRounding::TowardNegativeInfinity);
        }
        if values[1] > self.rate_bounds[1].decode() {
            self.rate_bounds[1] =
                CompactSignedRate::encode(values[1], BoundRounding::TowardPositiveInfinity);
        }
        if values[2] < self.rate_bounds[2].decode() {
            self.rate_bounds[2] =
                CompactSignedRate::encode(values[2], BoundRounding::TowardNegativeInfinity);
        }
        if values[3] > self.rate_bounds[3].decode() {
            self.rate_bounds[3] =
                CompactSignedRate::encode(values[3], BoundRounding::TowardPositiveInfinity);
        }
    }

    fn close(
        &mut self,
        closure: DirectionalRegionClosure,
        boundary_rung: Option<u8>,
    ) -> Option<DirectionalRegionSummary> {
        if !self.is_active() {
            return None;
        }
        let pooled = self.pooled.finish(self.variance_floor_observations)?;
        let member_mask = contiguous_mask(self.first_rung, self.member_count);
        let summary = DirectionalRegionSummary {
            direction: self.direction(),
            member_mask,
            first_rung: self.first_rung,
            last_rung: self.first_rung + self.member_count - 1,
            p_low: self.p_low,
            p_high: self.p_high,
            member_count: self.member_count,
            kind: if self.member_count >= MIN_DIRECTIONAL_REGION_RUNGS {
                DirectionalRegionKind::Valid
            } else {
                DirectionalRegionKind::Fragment
            },
            common_intersection: self.common_intersection,
            pooled,
            settled_rate_mean_min_q: self.rate_bounds[0].decode(),
            settled_rate_mean_max_q: self.rate_bounds[1].decode(),
            settled_rate_envelope_low_q: self.rate_bounds[2].decode(),
            settled_rate_envelope_high_q: self.rate_bounds[3].decode(),
            closure,
            boundary_rung,
            tested_intersection: self.common_intersection,
        };
        self.clear();
        Some(summary)
    }
}

const _: () = assert!(core::mem::size_of::<DirectionalRegionFold>() <= 96);

/// Fold one executed rung, returning a candidate closed by this observation.
pub fn observe_directional_rung(
    fold: &mut DirectionalRegionFold,
    rung_index: u8,
    rung: VelocityRungVerdict,
) -> Option<DirectionalRegionSummary> {
    if rung.direction != fold.direction() {
        return fold.close(DirectionalRegionClosure::SafetyTerminus, Some(rung_index));
    }
    if rung.classification != VelocityRungClass::Eligible {
        let closure = match rung.classification {
            VelocityRungClass::CurrentHeadroom => DirectionalRegionClosure::CurrentHeadroom,
            VelocityRungClass::Fault => DirectionalRegionClosure::SafetyTerminus,
            _ => DirectionalRegionClosure::IneligibleGap,
        };
        return fold.close(closure, Some(rung_index));
    }
    let Some(rung_interval) = rung.disturbance_interval else {
        return fold.close(DirectionalRegionClosure::SafetyTerminus, Some(rung_index));
    };
    if !fold.is_active() {
        let _ = fold.seed(rung_index, rung);
        return None;
    }
    let contiguous = rung_index == fold.first_rung.saturating_add(fold.member_count);
    let intersection = contiguous
        .then(|| intersect(fold.common_intersection, rung_interval))
        .flatten();
    if let Some(intersection) = intersection {
        fold.append(rung, intersection);
        return None;
    }
    let closure = if contiguous {
        DirectionalRegionClosure::DisjointInterval
    } else {
        DirectionalRegionClosure::IneligibleGap
    };
    let closed = fold.close(closure, Some(rung_index));
    let _ = fold.seed(rung_index, rung);
    closed
}

fn intersect(
    first: EquivalentTorqueTargetInterval,
    second: EquivalentTorqueTargetInterval,
) -> Option<EquivalentTorqueTargetInterval> {
    EquivalentTorqueTargetInterval::new(
        EquivalentTorqueTargetQ16::from_raw(first.low().raw().max(second.low().raw())),
        EquivalentTorqueTargetQ16::from_raw(first.high().raw().min(second.high().raw())),
    )
    .ok()
}

fn contiguous_mask(first: u8, count: u8) -> u32 {
    if count == 32 {
        u32::MAX
    } else {
        ((1_u32 << count) - 1) << first
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
