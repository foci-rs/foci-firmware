//! Pure, host-testable planning for the breakaway probe and additive discovery.
//!
//! Nothing here moves the axis. This module builds immutable plans:
//!
//! - [`UpwardBreakawayProbePlan`]: the realized half-octave grid plus the
//!   physical-excursion contract for the pair-and-repeat upward walk.
//! - [`AdditiveDiscoveryPlan`]: the fixed-step ladder that spends the discovery
//!   sweep's rung budget around the measured breakaway instead of across the whole
//!   current-representable span.
//!
//! The additive ladder is deliberately **not** built by re-anchoring
//! [`super::sweep_plan::build_gain_ladder`] at the breakaway gain. That generator
//! anchors its geometric rounding to its supplied start, so re-anchoring at a
//! breakaway gain that is itself a half-octave grid value silently produces a
//! different set of rungs and loses the measured value. See
//! `docs/specs/2026-07-31-foc-213-breakaway-seeded-p-ladder.md` section 1 for the
//! worked example.

use foci_tmc4671::pid::PidPGain;

use super::sweep_plan::{
    MAX_SWEEP_RUNGS, MIN_STRUCTURAL_RUNGS, MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US,
    SWEEP_DIRECTION_COUNT, VelocityPlanError, ZERO_SETTLE_US, build_gain_ladder, gain_at,
};

/// Numerator of the provisional maximum additive first-gap ratio guard.
///
/// **Provisional, conservative placeholder.** This is the nominal half-octave
/// generator factor (`181/128 ~ 1.4141`), not a hardware-verified bound. It refuses
/// additive first gaps coarser than the generator's own step, so a low-breakaway
/// ladder is rejected before motion rather than run and mis-tuned. The production
/// value is a Task 11 hardware-characterization deliverable derived from the
/// discovery sweep's first hardware run; realized grid ratios (the exempt
/// `floor -> breakaway` bracket) can exceed this nominal factor, and the additive
/// portion may ultimately tolerate a different bound once characterized.
pub const MAX_ADDITIVE_GAIN_RATIO_NUM: u32 = 181;
/// Denominator of the provisional maximum additive first-gap ratio guard.
///
/// See [`MAX_ADDITIVE_GAIN_RATIO_NUM`] for the provisional-value rationale.
pub const MAX_ADDITIVE_GAIN_RATIO_DEN: u32 = 128;

/// Mechanical-revolution divisor used by the physical-excursion probe.
pub const PROBE_MOTION_REVOLUTION_DENOMINATOR: u32 = 64;
/// Largest baseline-to-sample or sample-to-sample interval accepted by the probe.
pub const PROBE_MAX_CAPTURE_INTERVAL_US: u32 = 2_000;
/// Maximum observations consumed by a 32-rung pair-and-repeat upward walk.
pub const PROBE_MAX_OBSERVATIONS: u8 = 128;

/// Command-level workflow envelope and physical acquisition context a breakaway
/// probe is planned under.
///
/// Threaded from the orchestrator into [`plan_upward_breakaway_probe`] and hashed
/// into [`UpwardBreakawayProbePlan::digest`], so the whole downstream digest chain --
/// probe -> discovery (binds the probe digest) -> confirmation (binds the
/// discovery digest) -> Stage C (binds the confirmation digest) -- is pinned to
/// the exact command-level envelope and the physical parameters it was planned
/// against, not merely to the search geometry. These are the same
/// `AcquisitionContext`-shaped values the Stage-C tail consumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbeContext {
    /// Digest of the command-level [`super::integral_plan::CommissioningWorkflowPlan`]
    /// envelope this campaign reserved before probe motion.
    pub workflow_digest: u64,
    /// Frozen signed-target magnitude in mechanical RPM register units.
    pub target_velocity_rpm: i32,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
    /// PWM carrier frequency the campaign was planned under, in Hz.
    pub pwm_hz: u32,
    /// Encoder counts per mechanical revolution the campaign was planned under.
    pub encoder_counts_per_rev: u32,
}

/// Immutable, pre-motion plan for the physical-excursion upward probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct UpwardBreakawayProbePlan {
    /// Post-dedupe anchored half-octave search grid; unused entries are zero.
    pub search_gains: [PidPGain; MAX_SWEEP_RUNGS],
    /// Number of valid entries in `search_gains`.
    pub search_count: u8,
    /// Minimum commanded-direction excursion that counts as physical motion.
    pub motion_threshold_counts: u32,
    /// Maximum accepted interval between consecutive encoder captures.
    pub max_capture_interval_us: u32,
    /// Maximum observations permitted across the whole upward walk.
    pub maximum_observations: u8,
    /// Command-level workflow envelope and physical acquisition context.
    pub context: BreakawayProbeContext,
    /// Digest of every frozen field and valid realized search gain.
    pub plan_digest: u64,
}

impl UpwardBreakawayProbePlan {
    /// Hash the realized grid, excursion contract, budget, and physical context.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        macro_rules! hash_value {
            ($value:expr) => {
                for byte in $value.to_le_bytes() {
                    hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
                }
            };
        }
        hash_value!(self.search_count);
        for gain in self
            .search_gains
            .iter()
            .take(usize::from(self.search_count))
        {
            hash_value!(gain.raw());
        }
        hash_value!(self.motion_threshold_counts);
        hash_value!(self.max_capture_interval_us);
        hash_value!(self.maximum_observations);
        hash_value!(self.context.workflow_digest);
        hash_value!(self.context.target_velocity_rpm);
        hash_value!(self.context.hard_torque_limit);
        hash_value!(self.context.usable_torque_limit);
        hash_value!(self.context.pwm_hz);
        hash_value!(self.context.encoder_counts_per_rev);
        hash
    }
}

/// Compute the probe threshold as an exact checked ceiling division.
pub(crate) fn motion_threshold_counts(counts_per_rev: u32) -> Result<u32, VelocityPlanError> {
    if counts_per_rev == 0 {
        return Err(VelocityPlanError::InvalidRequest);
    }
    Ok(counts_per_rev.div_ceil(PROBE_MOTION_REVOLUTION_DENOMINATOR))
}

/// Build the immutable physical-excursion upward-probe plan.
pub fn plan_upward_breakaway_probe(
    p_start: PidPGain,
    p_top: PidPGain,
    context: BreakawayProbeContext,
) -> Result<UpwardBreakawayProbePlan, VelocityPlanError> {
    let ladder = build_gain_ladder(p_start.raw(), p_top.raw())?;
    let search_count = u8::try_from(ladder.len()).map_err(|_| VelocityPlanError::Arithmetic)?;
    let mut search_gains = [PidPGain::ZERO; MAX_SWEEP_RUNGS];
    search_gains[..ladder.len()].copy_from_slice(ladder.as_slice());

    let mut plan = UpwardBreakawayProbePlan {
        search_gains,
        search_count,
        motion_threshold_counts: motion_threshold_counts(context.encoder_counts_per_rev)?,
        max_capture_interval_us: PROBE_MAX_CAPTURE_INTERVAL_US,
        maximum_observations: PROBE_MAX_OBSERVATIONS,
        context,
        plan_digest: 0,
    };
    plan.plan_digest = plan.digest();
    Ok(plan)
}

/// Origin of the diagnostic floor placed one search-grid step below breakaway.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum GridFloorOrigin {
    /// A distinct predecessor grid rung exists below breakaway; it is the
    /// diagnostic floor.
    Predecessor,
    /// Breakaway is already the smallest representable grid rung. Floor is clamped
    /// to breakaway itself, and no distinct lower diagnostic rung was representable.
    ClampedAtBreakaway,
}

/// Resolve the additive ladder's diagnostic floor from the probe's search grid.
///
/// `breakaway_index` is the post-dedupe index of the resolved campaign breakaway
/// within the probe's `search_gains`. When breakaway is already the grid's smallest
/// rung (index zero, the arithmetic floor), the floor clamps to breakaway itself and
/// [`GridFloorOrigin::ClampedAtBreakaway`] reports that no lower diagnostic rung was
/// representable. Rung zero moving in that case is a diagnostic, not a failure.
pub fn resolve_grid_floor(
    p_start: PidPGain,
    p_top: PidPGain,
    rung_count: u8,
    breakaway_index: u8,
) -> Result<(PidPGain, GridFloorOrigin), VelocityPlanError> {
    let breakaway = gain_at(p_start.raw(), p_top.raw(), rung_count, breakaway_index)
        .ok_or(VelocityPlanError::InvalidRequest)?;
    if breakaway_index == 0 {
        return Ok((breakaway, GridFloorOrigin::ClampedAtBreakaway));
    }
    let predecessor_index = breakaway_index - 1;
    let floor = gain_at(p_start.raw(), p_top.raw(), rung_count, predecessor_index)
        .ok_or(VelocityPlanError::InvalidRequest)?;
    Ok((floor, GridFloorOrigin::Predecessor))
}

/// Frozen inputs to the additive discovery ladder planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdditiveDiscoveryRequest {
    /// Diagnostic floor: the predecessor grid rung below breakaway, or breakaway
    /// itself when [`resolve_grid_floor`] reports [`GridFloorOrigin::ClampedAtBreakaway`].
    pub floor: PidPGain,
    /// Resolved campaign breakaway gain (the lower of the two directional endpoints).
    pub breakaway: PidPGain,
    /// Most restrictive applicable authority ceiling; the ladder's exact final anchor.
    pub authority_ceiling: PidPGain,
    /// Discovery per-rung, per-direction observation count.
    pub observations_per_direction: u16,
    /// Host-declared per-primitive safety cap; every rung's primitive must fit it.
    pub max_duration_ms: u16,
    /// Aggregate discovery workflow duration reserved by the command-level plan
    /// before probe motion.
    pub workflow_envelope_ms: u32,
    /// Digest of the probe plan whose search located `breakaway`.
    pub probe_plan_digest: u64,
}

/// Immutable, pre-motion plan for the additive discovery sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdditiveDiscoveryPlan {
    /// Resolved additive ladder: floor, breakaway, fixed-step rungs, exact ceiling
    /// anchor; unused entries are zero.
    pub gains: [PidPGain; MAX_SWEEP_RUNGS],
    /// Number of valid entries in `gains`.
    pub rung_count: u8,
    /// Discovery per-rung, per-direction observation count.
    pub observations_per_direction: u16,
    /// `rung_count * observations_per_direction`, bounded by the audited table.
    pub confidence_family_size: u16,
    /// Digest of the probe plan whose search located this ladder's breakaway.
    pub probe_plan_digest: u64,
    /// Digest of every frozen field and each valid `gains` entry.
    pub plan_digest: u64,
}

impl AdditiveDiscoveryPlan {
    /// Hash every frozen field and each valid gain-ladder entry exactly once.
    ///
    /// Padding and unused `gains` slots beyond `rung_count` are never hashed.
    /// `probe_plan_digest` is hashed to bind this plan to the search that produced
    /// its breakaway.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        macro_rules! hash_value {
            ($value:expr) => {
                for byte in $value.to_le_bytes() {
                    hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
                }
            };
        }
        hash_value!(self.rung_count);
        for gain in self.gains.iter().take(usize::from(self.rung_count)) {
            hash_value!(gain.raw());
        }
        hash_value!(self.observations_per_direction);
        hash_value!(self.confidence_family_size);
        hash_value!(self.probe_plan_digest);
        hash
    }
}

/// Failure to construct a resolved additive discovery ladder before motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AdditiveLadderPlanError {
    /// A required input was zero or inconsistently ordered.
    InvalidRequest,
    /// Checked planner arithmetic exceeded its representation.
    Arithmetic,
    /// No candidate rung count can hold the minimum structural span: every
    /// realizable ladder dedupes below [`MIN_STRUCTURAL_RUNGS`].
    InsufficientAdditiveSpan,
    /// Every candidate's confidence family exceeded the audited table.
    ConfidenceFamilyTooLarge,
    /// The fixed per-rung primitive duration exceeds the host-declared per-primitive
    /// safety cap.
    PrimitiveDurationExceeded,
    /// Every surviving candidate's aggregate workflow duration exceeded the reserved
    /// command-level envelope.
    WorkflowEnvelopeExceeded,
    /// Every surviving candidate's first additive gap exceeded the provisional
    /// [`MAX_ADDITIVE_GAIN_RATIO_NUM`] / [`MAX_ADDITIVE_GAIN_RATIO_DEN`] guard.
    AdditiveLadderResolutionBudgetExceeded,
}

/// One realized additive ladder for a specific candidate rung-count target.
///
/// Exposed at `pub(crate)` visibility so tests can enumerate every candidate `n`
/// the planner considers without re-deriving the generator arithmetic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CandidateAdditiveLadder {
    /// Fixed additive step for this candidate: `ceil((ceiling - breakaway) / (n - 2))`.
    pub(crate) step: u32,
    /// Realized post-dedupe gains, in ascending order; unused entries are zero.
    pub(crate) gains: [u32; MAX_SWEEP_RUNGS],
    /// Number of valid entries in `gains`. May be less than the requested `n` when
    /// the final step overshoots the ceiling and dedupe collapses the tail.
    pub(crate) rung_count: u8,
    /// First rung strictly above breakaway: `min(breakaway + step, ceiling)`.
    pub(crate) first_additive_gain: u32,
}

/// Realize one candidate additive ladder for a requested structural rung count.
///
/// Implements the normative generator from
/// `docs/specs/2026-07-31-foc-213-breakaway-seeded-p-ladder.md` section 5: reserve
/// two rungs for `floor` and the appended exact `ceiling`, spend the rest on a fixed
/// additive step, and anchor the final rung exactly on `ceiling`. `floor` is deduped
/// against `breakaway` when the grid floor was clamped
/// ([`GridFloorOrigin::ClampedAtBreakaway`]).
pub(crate) fn realize_candidate_additive_ladder(
    rung_count_target: u8,
    floor_raw: u32,
    breakaway_raw: u32,
    ceiling_raw: u32,
) -> Result<CandidateAdditiveLadder, AdditiveLadderPlanError> {
    let remaining_intervals = rung_count_target
        .checked_sub(2)
        .ok_or(AdditiveLadderPlanError::Arithmetic)?;
    let remaining_intervals = u32::from(remaining_intervals);
    if remaining_intervals == 0 {
        return Err(AdditiveLadderPlanError::Arithmetic);
    }
    let span = ceiling_raw
        .checked_sub(breakaway_raw)
        .ok_or(AdditiveLadderPlanError::Arithmetic)?;
    let step = span.div_ceil(remaining_intervals);
    let first_additive_gain = breakaway_raw
        .checked_add(step)
        .ok_or(AdditiveLadderPlanError::Arithmetic)?
        .min(ceiling_raw);

    let mut gains = [0_u32; MAX_SWEEP_RUNGS];
    let mut rung_count = 0_usize;
    if floor_raw != breakaway_raw {
        gains[rung_count] = floor_raw;
        rung_count += 1;
    }
    gains[rung_count] = breakaway_raw;
    rung_count += 1;

    let mut current = breakaway_raw;
    loop {
        let next = current
            .checked_add(step)
            .ok_or(AdditiveLadderPlanError::Arithmetic)?;
        if next >= ceiling_raw {
            if gains[rung_count - 1] != ceiling_raw {
                if rung_count >= MAX_SWEEP_RUNGS {
                    return Err(AdditiveLadderPlanError::Arithmetic);
                }
                gains[rung_count] = ceiling_raw;
                rung_count += 1;
            }
            break;
        }
        if rung_count >= MAX_SWEEP_RUNGS {
            return Err(AdditiveLadderPlanError::Arithmetic);
        }
        gains[rung_count] = next;
        rung_count += 1;
        current = next;
    }

    Ok(CandidateAdditiveLadder {
        step,
        gains,
        rung_count: u8::try_from(rung_count).map_err(|_| AdditiveLadderPlanError::Arithmetic)?,
        first_additive_gain,
    })
}

/// Whether a candidate's first additive gap satisfies the provisional resolution
/// guard, checked as exact integer cross-multiplication with no rounding of its own.
///
/// The grid `floor -> breakaway` bracket is exempt: this only ever compares the
/// first rung strictly above breakaway against breakaway itself.
fn satisfies_additive_ratio_guard(first_additive_gain: u32, breakaway_raw: u32) -> bool {
    let left = u64::from(first_additive_gain) * u64::from(MAX_ADDITIVE_GAIN_RATIO_DEN);
    let right = u64::from(breakaway_raw) * u64::from(MAX_ADDITIVE_GAIN_RATIO_NUM);
    left <= right
}

struct BestCandidate {
    rung_count_target: u8,
    confidence_family_size: u16,
    ladder: CandidateAdditiveLadder,
}

/// Plan the resolved additive discovery ladder from a breakaway probe result.
///
/// Enumerates every candidate rung count from [`MIN_STRUCTURAL_RUNGS`] through
/// [`MAX_SWEEP_RUNGS`] rather than assuming monotonicity, because integer rounding
/// and dedupe can break it. Rejects candidates that dedupe below the structural
/// minimum, exceed the audited confidence-family table, exceed the aggregate
/// workflow envelope, or whose first additive gap exceeds the provisional
/// [`MAX_ADDITIVE_GAIN_RATIO_NUM`] / [`MAX_ADDITIVE_GAIN_RATIO_DEN`] guard. Among
/// survivors, chooses the smallest first additive gap, breaking exact ties toward
/// the higher rung count.
pub fn plan_additive_discovery_ladder(
    request: AdditiveDiscoveryRequest,
) -> Result<AdditiveDiscoveryPlan, AdditiveLadderPlanError> {
    let floor_raw = u32::from(request.floor.raw());
    let breakaway_raw = u32::from(request.breakaway.raw());
    let ceiling_raw = u32::from(request.authority_ceiling.raw());

    if floor_raw == 0
        || breakaway_raw == 0
        || ceiling_raw == 0
        || request.observations_per_direction == 0
        || request.max_duration_ms == 0
    {
        return Err(AdditiveLadderPlanError::InvalidRequest);
    }
    if floor_raw > breakaway_raw {
        return Err(AdditiveLadderPlanError::InvalidRequest);
    }
    if breakaway_raw >= ceiling_raw {
        return Err(AdditiveLadderPlanError::InsufficientAdditiveSpan);
    }

    let primitive_us = MOVING_STROKE_US
        .checked_add(ZERO_SETTLE_US)
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(AdditiveLadderPlanError::Arithmetic)?;
    if primitive_us.div_ceil(1_000) > u32::from(request.max_duration_ms) {
        return Err(AdditiveLadderPlanError::PrimitiveDurationExceeded);
    }
    let slot_count = u32::from(request.observations_per_direction)
        .checked_mul(u32::from(SWEEP_DIRECTION_COUNT))
        .ok_or(AdditiveLadderPlanError::Arithmetic)?;

    let mut best: Option<BestCandidate> = None;
    let mut saw_structural_survivor = false;
    let mut saw_family_survivor = false;
    let mut saw_envelope_survivor = false;

    for rung_count_target in MIN_STRUCTURAL_RUNGS..=MAX_SWEEP_RUNGS {
        let rung_count_target =
            u8::try_from(rung_count_target).map_err(|_| AdditiveLadderPlanError::Arithmetic)?;
        let ladder = realize_candidate_additive_ladder(
            rung_count_target,
            floor_raw,
            breakaway_raw,
            ceiling_raw,
        )?;

        if usize::from(ladder.rung_count) < MIN_STRUCTURAL_RUNGS {
            continue;
        }
        saw_structural_survivor = true;

        let confidence_family_size = u16::from(ladder.rung_count)
            .checked_mul(request.observations_per_direction)
            .ok_or(AdditiveLadderPlanError::Arithmetic)?;
        if confidence_family_size
            > crate::autotune::velocity_measurement::MAX_CONFIDENCE_FAMILY_SIZE
        {
            continue;
        }
        saw_family_survivor = true;

        let workflow_us = u32::from(ladder.rung_count)
            .checked_mul(slot_count)
            .and_then(|value| value.checked_mul(primitive_us))
            .ok_or(AdditiveLadderPlanError::Arithmetic)?;
        let workflow_ms = workflow_us.div_ceil(1_000);
        if workflow_ms > request.workflow_envelope_ms {
            continue;
        }
        saw_envelope_survivor = true;

        if !satisfies_additive_ratio_guard(ladder.first_additive_gain, breakaway_raw) {
            continue;
        }

        let is_better = match &best {
            None => true,
            Some(current_best) => {
                ladder.first_additive_gain < current_best.ladder.first_additive_gain
                    || (ladder.first_additive_gain == current_best.ladder.first_additive_gain
                        && rung_count_target > current_best.rung_count_target)
            }
        };
        if is_better {
            best = Some(BestCandidate {
                rung_count_target,
                confidence_family_size,
                ladder,
            });
        }
    }

    let best = best.ok_or(if !saw_structural_survivor {
        AdditiveLadderPlanError::InsufficientAdditiveSpan
    } else if !saw_family_survivor {
        AdditiveLadderPlanError::ConfidenceFamilyTooLarge
    } else if !saw_envelope_survivor {
        AdditiveLadderPlanError::WorkflowEnvelopeExceeded
    } else {
        AdditiveLadderPlanError::AdditiveLadderResolutionBudgetExceeded
    })?;

    let mut gains = [PidPGain::ZERO; MAX_SWEEP_RUNGS];
    for (slot, &raw_gain) in gains
        .iter_mut()
        .zip(best.ladder.gains.iter())
        .take(usize::from(best.ladder.rung_count))
    {
        let raw = u16::try_from(raw_gain).map_err(|_| AdditiveLadderPlanError::Arithmetic)?;
        *slot = PidPGain::new(raw).map_err(|_| AdditiveLadderPlanError::Arithmetic)?;
    }

    let mut plan = AdditiveDiscoveryPlan {
        gains,
        rung_count: best.ladder.rung_count,
        observations_per_direction: request.observations_per_direction,
        confidence_family_size: best.confidence_family_size,
        probe_plan_digest: request.probe_plan_digest,
        plan_digest: 0,
    };
    plan.plan_digest = plan.digest();
    Ok(plan)
}

#[cfg(test)]
#[path = "breakaway_plan_tests.rs"]
mod tests;
