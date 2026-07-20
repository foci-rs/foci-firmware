use crate::autotune::velocity_measurement::VelocityDirection;
use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
use foci_tmc4671::pid::PidGain;

use super::sweep_consensus::DirectionalConsensusClass;
use super::sweep_evidence::{
    CompactDirectionalRungReport, DirectionalRungAnalysis, PooledDisturbance, VelocityEvidenceError,
};

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
    /// Exact zero used before a rung supplies a representative rate.
    pub const ZERO: Self = Self {
        mantissa: 0,
        shift: 0,
        rounding: BoundRounding::TowardNegativeInfinity,
    };

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

    /// Merge another exact fixed-effect accumulator with checked arithmetic.
    pub fn merge(&mut self, other: Self) -> Option<()> {
        self.weighted_point = self.weighted_point.checked_add(other.weighted_point)?;
        self.weighted_low = self.weighted_low.checked_add(other.weighted_low)?;
        self.weighted_high = self.weighted_high.checked_add(other.weighted_high)?;
        self.weight_sum = self.weight_sum.checked_add(other.weight_sum)?;
        Some(())
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

    fn seed(
        &mut self,
        rung_index: u8,
        analysis: DirectionalRungAnalysis,
    ) -> Result<(), VelocityEvidenceError> {
        let interval = report_interval(analysis.report)?;
        self.common_intersection = interval;
        self.p_low = analysis.report.p;
        self.p_high = analysis.report.p;
        self.first_rung = rung_index;
        self.member_count = 1;
        self.flags |= Self::ACTIVE;
        self.set_rate_bounds(analysis.report);
        self.variance_floor_observations = analysis
            .report
            .pooled
            .ok_or(VelocityEvidenceError::InvalidConsensus)?
            .variance_floor_observations;
        self.pooled
            .merge(analysis.pooled_accumulator)
            .ok_or(VelocityEvidenceError::Arithmetic)
    }

    fn append(
        &mut self,
        analysis: DirectionalRungAnalysis,
        intersection: EquivalentTorqueTargetInterval,
    ) -> Result<(), VelocityEvidenceError> {
        self.common_intersection = intersection;
        self.p_high = analysis.report.p;
        self.member_count = self.member_count.saturating_add(1);
        self.extend_rate_bounds(analysis.report);
        self.variance_floor_observations = self
            .variance_floor_observations
            .checked_add(
                analysis
                    .report
                    .pooled
                    .ok_or(VelocityEvidenceError::InvalidConsensus)?
                    .variance_floor_observations,
            )
            .ok_or(VelocityEvidenceError::Arithmetic)?;
        self.pooled
            .merge(analysis.pooled_accumulator)
            .ok_or(VelocityEvidenceError::Arithmetic)
    }

    fn set_rate_bounds(&mut self, report: CompactDirectionalRungReport) {
        self.rate_bounds = report.settled_rate_bounds_q;
    }

    fn extend_rate_bounds(&mut self, report: CompactDirectionalRungReport) {
        let values = report.settled_rate_bounds_q.map(CompactSignedRate::decode);
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

/// Why one maximal contiguous jointly operable region closed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum JointOperableClosure {
    /// One direction ceased to satisfy the unchanged operating predicates.
    IneligibleGap,
    /// The usable-current boundary ended the shared region.
    CurrentHeadroom,
    /// A fault or safety terminus ended the shared region.
    SafetyTerminus,
    /// The predeclared ladder ended normally.
    EndOfLadder,
}

/// One maximal contiguous region where a shared P is operable in both directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct JointOperableRegion {
    /// Exact contiguous rung-membership bitmap.
    pub member_mask: u32,
    /// First member rung index.
    pub first_rung: u8,
    /// Last member rung index.
    pub last_rung: u8,
    /// Exact member population.
    pub member_count: u8,
    /// Why the region closed.
    pub closure: JointOperableClosure,
}

/// Fixed-memory fold for jointly operable rungs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct JointOperableFold {
    member_mask: u32,
    best_member_mask: u32,
    first_rung: u8,
    member_count: u8,
    active: bool,
    best_closure: JointOperableClosure,
}

impl JointOperableFold {
    /// Construct an empty joint-operability fold.
    pub const fn new() -> Self {
        Self {
            member_mask: 0,
            best_member_mask: 0,
            first_rung: 0,
            member_count: 0,
            active: false,
            best_closure: JointOperableClosure::EndOfLadder,
        }
    }

    /// Close the active region at a declared terminus.
    pub fn finish(&mut self, closure: JointOperableClosure) -> Option<JointOperableRegion> {
        self.close(closure)
    }

    /// Retain only the deterministically preferred closed joint membership.
    pub fn consider_best(
        &mut self,
        candidate: JointOperableRegion,
        gains: &[PidGain],
        p_start: PidGain,
        p_top: PidGain,
    ) {
        let current = self
            .best_region()
            .map(|region| choose_joint_nomination(None, region, gains, p_start, p_top));
        let selected = choose_joint_nomination(current, candidate, gains, p_start, p_top);
        self.best_member_mask = selected.joint.member_mask;
        self.best_closure = selected.joint.closure;
    }

    /// Return the retained best joint nomination.
    pub fn best_nomination(
        &self,
        gains: &[PidGain],
        p_start: PidGain,
        p_top: PidGain,
    ) -> Option<ProvisionalPNomination> {
        self.best_region()
            .map(|region| choose_joint_nomination(None, region, gains, p_start, p_top))
    }

    /// Return the current active lower-middle nomination.
    pub fn active_nomination(
        &self,
        gains: &[PidGain],
        p_start: PidGain,
        p_top: PidGain,
    ) -> Option<ProvisionalPNomination> {
        self.active_region()
            .map(|region| choose_joint_nomination(None, region, gains, p_start, p_top))
    }

    fn best_region(&self) -> Option<JointOperableRegion> {
        (self.best_member_mask != 0)
            .then(|| region_from_mask(self.best_member_mask, self.best_closure))
    }

    fn active_region(&self) -> Option<JointOperableRegion> {
        self.active
            .then(|| region_from_mask(self.member_mask, JointOperableClosure::EndOfLadder))
    }

    fn seed(&mut self, rung_index: u8) {
        self.member_mask = 1_u32 << rung_index;
        self.first_rung = rung_index;
        self.member_count = 1;
        self.active = true;
    }

    fn append(&mut self, rung_index: u8) {
        self.member_mask |= 1_u32 << rung_index;
        self.member_count = self.member_count.saturating_add(1);
    }

    fn close(&mut self, closure: JointOperableClosure) -> Option<JointOperableRegion> {
        if !self.active {
            return None;
        }
        let region = JointOperableRegion {
            member_mask: self.member_mask,
            first_rung: self.first_rung,
            last_rung: self.first_rung + self.member_count - 1,
            member_count: self.member_count,
            closure,
        };
        self.member_mask = 0;
        self.first_rung = 0;
        self.member_count = 0;
        self.active = false;
        Some(region)
    }
}

impl Default for JointOperableFold {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<JointOperableFold>() <= 16);

/// Fold one forward/reverse rung pair without consulting D_eq eligibility.
pub fn observe_joint_rung(
    fold: &mut JointOperableFold,
    rung_index: u8,
    forward: CompactDirectionalRungReport,
    reverse: CompactDirectionalRungReport,
) -> Option<JointOperableRegion> {
    let matching_gain = forward.p == reverse.p;
    if matching_gain && forward.operable() && reverse.operable() {
        if !fold.active {
            fold.seed(rung_index);
            return None;
        }
        if rung_index == fold.first_rung.saturating_add(fold.member_count) {
            fold.append(rung_index);
            return None;
        }
        let closed = fold.close(JointOperableClosure::IneligibleGap);
        fold.seed(rung_index);
        return closed;
    }
    let closure = if !matching_gain {
        JointOperableClosure::SafetyTerminus
    } else {
        JointOperableClosure::IneligibleGap
    };
    fold.close(closure)
}

/// Deterministic provisional operating point selected from one joint region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ProvisionalPNomination {
    /// Selected joint-operability evidence.
    pub joint: JointOperableRegion,
    /// Lower-middle rung index.
    pub rung_index: u8,
    /// Gain at the selected rung.
    pub p: PidGain,
    /// Q16 ratio distance from structural P_start.
    pub distance_to_start_num: u32,
    /// Q16 ratio distance from structural P_top.
    pub distance_to_top_num: u32,
    /// Single-rung regions carry reduced I-ladder margin.
    pub reduced_margin: bool,
}

/// Compare one closed joint region with the current deterministic nomination.
pub fn choose_joint_nomination(
    current: Option<ProvisionalPNomination>,
    candidate: JointOperableRegion,
    gains: &[PidGain],
    p_start: PidGain,
    p_top: PidGain,
) -> ProvisionalPNomination {
    let rung_index = candidate.first_rung + (candidate.member_count - 1) / 2;
    let p = gains[usize::from(rung_index)];
    let proposed = ProvisionalPNomination {
        joint: candidate,
        rung_index,
        p,
        distance_to_start_num: ratio_q16(p.raw(), p_start.raw()),
        distance_to_top_num: ratio_q16(p_top.raw(), p.raw()),
        reduced_margin: candidate.member_count == 1,
    };
    let Some(current) = current else {
        return proposed;
    };
    if candidate.member_count != current.joint.member_count {
        return if candidate.member_count > current.joint.member_count {
            proposed
        } else {
            current
        };
    }
    match compare_margin(p, current.p, p_start, p_top) {
        core::cmp::Ordering::Greater => proposed,
        core::cmp::Ordering::Less => current,
        core::cmp::Ordering::Equal if p.raw() < current.p.raw() => proposed,
        core::cmp::Ordering::Equal => current,
    }
}

/// Compact region identity retained by bounded handoff selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalRegionRef {
    /// Exact membership joining this reference to its causal region record.
    pub member_mask: u32,
    /// Exact pooled Q16.16 disturbance interval.
    pub pooled_interval: [i32; 2],
    /// Conservative source-edge settled rate for transfer reporting.
    pub selected_edge_rate: CompactSignedRate,
}

impl DirectionalRegionRef {
    const EMPTY: Self = Self {
        member_mask: 0,
        pooled_interval: [0, 0],
        selected_edge_rate: CompactSignedRate::ZERO,
    };

    /// Compact one valid causal region for bounded handoff selection.
    pub fn from_summary(summary: DirectionalRegionSummary, nominated_rung: u8) -> Option<Self> {
        if summary.kind != DirectionalRegionKind::Valid {
            return None;
        }
        let (edge_rate, rounding) = if nominated_rung > summary.last_rung {
            (
                summary.settled_rate_mean_max_q,
                BoundRounding::TowardPositiveInfinity,
            )
        } else {
            (
                summary.settled_rate_mean_min_q,
                BoundRounding::TowardNegativeInfinity,
            )
        };
        Some(Self {
            member_mask: summary.member_mask,
            pooled_interval: [
                summary.pooled.interval.low().raw(),
                summary.pooled.interval.high().raw(),
            ],
            selected_edge_rate: CompactSignedRate::encode(edge_rate, rounding),
        })
    }
}

const _: () = assert!(core::mem::size_of::<DirectionalRegionRef>() <= 16);

/// Relationship between a selected directional model and the nominated P.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CoverageKind {
    /// The directional model region contains the nominated rung.
    FullCoverage,
    /// The nearest directional model region transfers across a declared gap.
    TransferredCoverage,
}

/// Compact directional selection and transfer context.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct SelectedDirectionalRegion {
    /// Selected compact source-region reference.
    pub region: DirectionalRegionRef,
    /// Direct or transferred coverage.
    pub coverage: CoverageKind,
    /// Signed source-edge distance from the nominated rung.
    pub signed_rung_distance: i8,
    /// Larger gain in the source/target ratio.
    pub gain_ratio_num: u16,
    /// Smaller gain in the source/target ratio.
    pub gain_ratio_den: u16,
    /// Source-edge minus nominated settled rate.
    pub settled_rate_difference_q: i64,
}

/// Complete Stage B evidence handoff without applying a gain-acceptance policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StageBHandoff {
    /// Firmware-owned provisional P nomination.
    pub nomination: ProvisionalPNomination,
    /// Forward then reverse selected directional model evidence.
    pub directional: [SelectedDirectionalRegion; 2],
    /// Bit set for each direction with full nominated-P coverage.
    pub coverage_mask: u8,
}

/// Compare one directional region with the current nearest selection.
pub fn select_directional_region(
    current: Option<SelectedDirectionalRegion>,
    candidate: DirectionalRegionRef,
    nominated_rung: u8,
    gains: &[PidGain],
    nominated_rate_q: i64,
) -> Option<SelectedDirectionalRegion> {
    if candidate.member_mask == 0 {
        return current;
    }
    let proposed = selected_directional(candidate, nominated_rung, gains, nominated_rate_q);
    let Some(current) = current else {
        return Some(proposed);
    };
    let proposed_distance = proposed.signed_rung_distance.unsigned_abs();
    let current_distance = current.signed_rung_distance.unsigned_abs();
    if proposed.coverage == CoverageKind::FullCoverage
        && current.coverage != CoverageKind::FullCoverage
        || proposed_distance < current_distance
        || (proposed_distance == current_distance
            && first_member(candidate.member_mask) < first_member(current.region.member_mask))
    {
        Some(proposed)
    } else {
        Some(current)
    }
}

fn selected_directional(
    region: DirectionalRegionRef,
    nominated_rung: u8,
    gains: &[PidGain],
    nominated_rate_q: i64,
) -> SelectedDirectionalRegion {
    let first = first_member(region.member_mask);
    let last = last_member(region.member_mask);
    let (coverage, distance, edge) = if nominated_rung < first {
        (
            CoverageKind::TransferredCoverage,
            i16::from(first) - i16::from(nominated_rung),
            first,
        )
    } else if nominated_rung > last {
        (
            CoverageKind::TransferredCoverage,
            i16::from(last) - i16::from(nominated_rung),
            last,
        )
    } else {
        (CoverageKind::FullCoverage, 0, nominated_rung)
    };
    let source_p = gains
        .get(usize::from(edge))
        .copied()
        .unwrap_or(PidGain::ZERO)
        .raw();
    let target_p = gains
        .get(usize::from(nominated_rung))
        .copied()
        .unwrap_or(PidGain::ZERO)
        .raw();
    let (gain_ratio_num, gain_ratio_den) = if coverage == CoverageKind::FullCoverage {
        (1, 1)
    } else {
        (source_p.max(target_p), source_p.min(target_p).max(1))
    };
    SelectedDirectionalRegion {
        region,
        coverage,
        signed_rung_distance: distance as i8,
        gain_ratio_num,
        gain_ratio_den,
        settled_rate_difference_q: if coverage == CoverageKind::FullCoverage {
            0
        } else {
            region.selected_edge_rate.decode() - nominated_rate_q
        },
    }
}

/// Six compact references supporting active and selected joint nominations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct HandoffTrackers {
    best: [DirectionalRegionRef; 2],
    active: [DirectionalRegionRef; 2],
    latest_closed: [DirectionalRegionRef; 2],
}

impl HandoffTrackers {
    /// Construct empty bounded handoff trackers.
    pub const fn new() -> Self {
        Self {
            best: [DirectionalRegionRef::EMPTY; 2],
            active: [DirectionalRegionRef::EMPTY; 2],
            latest_closed: [DirectionalRegionRef::EMPTY; 2],
        }
    }

    /// Observe one closed valid directional region against both live nominations.
    pub fn observe_directional(
        &mut self,
        direction: VelocityDirection,
        region: DirectionalRegionRef,
        active_rung: u8,
        best_rung: u8,
    ) {
        let index = direction_index(direction);
        self.latest_closed[index] = region;
        self.active[index] = choose_ref(self.active[index], region, active_rung);
        self.best[index] = choose_ref(self.best[index], region, best_rung);
    }

    /// Re-evaluate active selections when the active lower-middle rung advances.
    pub fn move_active_nomination(&mut self, rung_index: u8) {
        for index in 0..2 {
            self.active[index] =
                choose_ref(self.active[index], self.latest_closed[index], rung_index);
        }
    }

    /// Start tracking a later joint candidate without retaining its predecessors.
    pub fn begin_active_nomination(&mut self, rung_index: u8) {
        for index in 0..2 {
            self.active[index] =
                choose_ref(self.best[index], self.latest_closed[index], rung_index);
        }
    }

    /// Promote the active candidate's selections when it becomes the best joint region.
    pub fn promote_active(&mut self) {
        self.best = self.active;
    }

    /// Return the selected compact reference for one direction.
    pub fn best(
        &self,
        direction: VelocityDirection,
        nominated_rung: u8,
        gains: &[PidGain],
        nominated_rate_q: i64,
    ) -> Option<SelectedDirectionalRegion> {
        let selected = self.best[direction_index(direction)];
        (selected.member_mask != 0)
            .then(|| selected_directional(selected, nominated_rung, gains, nominated_rate_q))
    }

    /// Bit set for each direction with at least one retained valid model region.
    pub fn model_direction_mask(&self) -> u8 {
        u8::from(self.best[0].member_mask != 0) | (u8::from(self.best[1].member_mask != 0) << 1)
    }

    /// Build the complete bidirectional handoff for the selected nomination.
    pub fn handoff(
        &self,
        nomination: ProvisionalPNomination,
        gains: &[PidGain],
        nominated_rates_q: [i64; 2],
    ) -> Option<StageBHandoff> {
        let forward = self.best(
            VelocityDirection::Forward,
            nomination.rung_index,
            gains,
            nominated_rates_q[0],
        )?;
        let reverse = self.best(
            VelocityDirection::Reverse,
            nomination.rung_index,
            gains,
            nominated_rates_q[1],
        )?;
        let coverage_mask = u8::from(forward.coverage == CoverageKind::FullCoverage)
            | (u8::from(reverse.coverage == CoverageKind::FullCoverage) << 1);
        Some(StageBHandoff {
            nomination,
            directional: [forward, reverse],
            coverage_mask,
        })
    }
}

impl Default for HandoffTrackers {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<HandoffTrackers>() <= 96);

/// Bounded runtime state replacing the two legacy global-band accumulators.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OperatingRegionRuntime {
    /// Independent forward and reverse model-region folds.
    pub directional: [DirectionalRegionFold; 2],
    /// Joint operability fold independent of model validity.
    pub joint: JointOperableFold,
    /// Bounded handoff selections.
    pub handoff: HandoffTrackers,
}

impl OperatingRegionRuntime {
    /// Construct empty Stage B operating-region state.
    pub const fn new() -> Self {
        Self {
            directional: [
                DirectionalRegionFold::new(VelocityDirection::Forward),
                DirectionalRegionFold::new(VelocityDirection::Reverse),
            ],
            joint: JointOperableFold::new(),
            handoff: HandoffTrackers::new(),
        }
    }
}

impl Default for OperatingRegionRuntime {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<OperatingRegionRuntime>() <= 304);

/// Fold one executed rung, returning a candidate closed by this observation.
pub fn observe_directional_rung(
    fold: &mut DirectionalRegionFold,
    rung_index: u8,
    analysis: DirectionalRungAnalysis,
) -> Result<Option<DirectionalRegionSummary>, VelocityEvidenceError> {
    if analysis.report.classification != DirectionalConsensusClass::EligibleConsensus {
        return Ok(fold.close(DirectionalRegionClosure::IneligibleGap, Some(rung_index)));
    }
    let rung_interval = report_interval(analysis.report)?;
    if !fold.is_active() {
        fold.seed(rung_index, analysis)?;
        return Ok(None);
    }
    let contiguous = rung_index == fold.first_rung.saturating_add(fold.member_count);
    let intersection = contiguous
        .then(|| intersect(fold.common_intersection, rung_interval))
        .flatten();
    if let Some(intersection) = intersection {
        fold.append(analysis, intersection)?;
        return Ok(None);
    }
    let closure = if contiguous {
        DirectionalRegionClosure::DisjointInterval
    } else {
        DirectionalRegionClosure::IneligibleGap
    };
    let closed = fold.close(closure, Some(rung_index));
    fold.seed(rung_index, analysis)?;
    Ok(closed)
}

fn report_interval(
    report: CompactDirectionalRungReport,
) -> Result<EquivalentTorqueTargetInterval, VelocityEvidenceError> {
    if report.classification != DirectionalConsensusClass::EligibleConsensus
        || report.pooled.is_none()
    {
        return Err(VelocityEvidenceError::InvalidConsensus);
    }
    EquivalentTorqueTargetInterval::new(
        EquivalentTorqueTargetQ16::from_raw(report.components[0][0]),
        EquivalentTorqueTargetQ16::from_raw(report.components[0][1]),
    )
    .map_err(|_| VelocityEvidenceError::InvalidConsensus)
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

fn ratio_q16(numerator: u16, denominator: u16) -> u32 {
    (u32::from(numerator) << 16) / u32::from(denominator.max(1))
}

fn compare_margin(
    left: PidGain,
    right: PidGain,
    p_start: PidGain,
    p_top: PidGain,
) -> core::cmp::Ordering {
    let left_margin = margin_fraction(left, p_start, p_top);
    let right_margin = margin_fraction(right, p_start, p_top);
    (u64::from(left_margin.0) * u64::from(right_margin.1))
        .cmp(&(u64::from(right_margin.0) * u64::from(left_margin.1)))
}

fn margin_fraction(p: PidGain, p_start: PidGain, p_top: PidGain) -> (u16, u16) {
    if u32::from(p.raw()) * u32::from(p.raw()) <= u32::from(p_start.raw()) * u32::from(p_top.raw())
    {
        (p.raw(), p_start.raw())
    } else {
        (p_top.raw(), p.raw())
    }
}

fn direction_index(direction: VelocityDirection) -> usize {
    match direction {
        VelocityDirection::Forward => 0,
        VelocityDirection::Reverse => 1,
    }
}

fn choose_ref(
    current: DirectionalRegionRef,
    candidate: DirectionalRegionRef,
    nominated_rung: u8,
) -> DirectionalRegionRef {
    let current_distance = ref_distance(current, nominated_rung);
    let candidate_distance = ref_distance(candidate, nominated_rung);
    if current.member_mask == 0
        || candidate_distance < current_distance
        || (candidate_distance == current_distance
            && first_member(candidate.member_mask) < first_member(current.member_mask))
    {
        candidate
    } else {
        current
    }
}

fn ref_distance(region: DirectionalRegionRef, nominated_rung: u8) -> u8 {
    if region.member_mask == 0 {
        return u8::MAX;
    }
    let first = first_member(region.member_mask);
    let last = last_member(region.member_mask);
    if nominated_rung < first {
        first - nominated_rung
    } else {
        nominated_rung.saturating_sub(last)
    }
}

fn first_member(mask: u32) -> u8 {
    mask.trailing_zeros() as u8
}

fn last_member(mask: u32) -> u8 {
    (u32::BITS - 1 - mask.leading_zeros()) as u8
}

fn region_from_mask(mask: u32, closure: JointOperableClosure) -> JointOperableRegion {
    let first_rung = first_member(mask);
    let last_rung = last_member(mask);
    JointOperableRegion {
        member_mask: mask,
        first_rung,
        last_rung,
        member_count: mask.count_ones() as u8,
        closure,
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
