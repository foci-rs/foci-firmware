//! Canonical wire reporting for the breakaway-seeded campaign.
//!
//! Every function here is pure: it takes the same frozen plan and evidence
//! types the probe, additive discovery, and held-out confirmation drivers in
//! [`super::breakaway_campaign`] already produce, and assembles the
//! `commissioning::types` records those drivers can hand a future caller to
//! report. Nothing here drives motion or touches a backend, and nothing here
//! is wired into the live campaign step functions yet -- that wiring is a
//! later task's job. This module exists so the wire shape and the digest and
//! terminal-cause bookkeeping are frozen and tested independently of that
//! wiring.
//!
//! [`BreakawayPhase`] names which of the three committed drivers owns a
//! record. [`BreakawayTerminalCause`] is the single wire taxonomy spanning
//! every probe, discovery, and confirmation terminal variant, so a host
//! decoder needs exactly one cause table instead of three.
//!
//! Nothing outside this module's own tests calls into any of these assembly
//! functions yet -- wiring them into the live campaign step functions is a
//! later task's job (see the module-level docs above) -- so, matching
//! [`super::breakaway_campaign`]'s own
//! `#![cfg_attr(not(test), allow(dead_code))]`, dead-code warnings are
//! suppressed outside test builds.
#![cfg_attr(not(test), allow(dead_code))]

use foci_tmc4671::pid::PidPGain;

use crate::autotune::velocity_measurement::{VelocityDirection, confidence_multiplier_95_df15};
use crate::commissioning::types::{
    AdditiveDiscoveryPlanned, AdditiveDiscoveryTerminal, BandConfirmationObservation,
    BandConfirmationPlanned, BandConfirmationTerminal, BreakawayCampaignTerminal,
    BreakawayDirectionalBreakaway, BreakawayDiscoveryRungMargin, BreakawayLadderRung,
    BreakawayProbePlanned, BreakawayProbeTerminal, BreakawayRungZeroDiagnostic,
};

use super::band_confirmation::{
    BandConfirmationFailureCause, BandConfirmationPlan, CONFIRMATION_BAND_LOWER_PERCENT,
    CONFIRMATION_BAND_UPPER_PERCENT, CONFIRMATION_FAMILY_SIZE, ConfirmationDirectionalInput,
    DiscoveryObservationAtom, DiscoveryRungNominationBounds, NominatedConfirmationCandidate,
    observation_interval_percent, observation_statistic,
};
use super::breakaway_campaign::{
    AdditiveDiscoveryOutcome, BandConfirmationOutcome, BreakawayProbeOutcome,
};
use super::breakaway_plan::{AdditiveDiscoveryPlan, AdditiveDiscoveryRequest, BreakawayProbePlan};
use super::breakaway_search::{BreakawayMotionClass, DirectionalBreakaway};
use super::sweep_evidence::direction_report_code;

/// Phase that owns a breakaway-campaign wire record or terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum BreakawayPhase {
    /// The two-direction bisection probe.
    Probe = 0,
    /// The additive discovery ladder.
    Discovery = 1,
    /// The held-out eight-stroke confirmation block.
    Confirmation = 2,
}

/// Unified wire terminal-cause taxonomy spanning every probe, discovery, and
/// confirmation terminal variant, plus the campaign-level accept outcome.
///
/// One shared table so a host decoder does not need three separate cause
/// enums with overlapping wire codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum BreakawayTerminalCause {
    /// No failure: the phase either succeeded or has not yet terminated.
    None = 0,
    /// [`BreakawayProbeOutcome::NoMotionAtAuthority`].
    ProbeNoMotionAtAuthority = 1,
    /// [`BreakawayProbeOutcome::UnstableBreakaway`].
    ProbeUnstableBreakaway = 2,
    /// [`BreakawayProbeOutcome::DirectionalBreakawayDisagreement`].
    ProbeDirectionalBreakawayDisagreement = 3,
    /// [`BreakawayProbeOutcome::ProbeEvidenceExcluded`].
    ProbeEvidenceExcluded = 4,
    /// [`BreakawayProbeOutcome::AuthorityFailure`].
    ProbeAuthorityFailure = 5,
    /// [`BreakawayProbeOutcome::SafetyFault`].
    ProbeSafetyFault = 6,
    /// [`AdditiveDiscoveryOutcome::TargetBandSkipped`].
    DiscoveryTargetBandSkipped = 7,
    /// [`AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand`].
    DiscoveryCurrentHeadroomBeforeBand = 8,
    /// [`AdditiveDiscoveryOutcome::InsufficientAdditiveSpan`].
    DiscoveryInsufficientAdditiveSpan = 9,
    /// [`AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded`].
    DiscoveryAdditiveLadderResolutionBudgetExceeded = 10,
    /// [`AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded`].
    DiscoveryEvidenceExcluded = 11,
    /// [`AdditiveDiscoveryOutcome::SafetyFault`].
    DiscoverySafetyFault = 12,
    /// [`BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery`].
    ConfirmationTargetBandUnconfirmableFromDiscovery = 13,
    /// [`BandConfirmationOutcome::TargetBandNotReachedAtAuthority`].
    ConfirmationTargetBandNotReachedAtAuthority = 14,
    /// [`BandConfirmationFailureCause::ResponseLocation`].
    ConfirmationResponseLocation = 15,
    /// [`BandConfirmationFailureCause::ExcessiveUncertainty`].
    ConfirmationExcessiveUncertainty = 16,
    /// [`BandConfirmationFailureCause::LossOfConsensus`].
    ConfirmationLossOfConsensus = 17,
    /// [`BandConfirmationFailureCause::CurrentAuthority`].
    ConfirmationCurrentAuthority = 18,
    /// [`BandConfirmationOutcome::ConfirmationEvidenceExcluded`].
    ConfirmationEvidenceExcluded = 19,
    /// [`BandConfirmationOutcome::SafetyFault`].
    ConfirmationSafetyFault = 20,
    /// [`BandConfirmationOutcome::Accepted`], or the campaign-level Stage-C hand-off.
    Accepted = 21,
}

/// Relative standard error above which no achievable response location could
/// satisfy 70-80% containment: `(80 - 70) / (80 + 70) = 1 / 15`.
///
/// Independently derived here from the public band-edge constants (spec
/// section 7, "The confirmable window is narrower than the band") rather than
/// imported, because [`super::band_confirmation`]'s own copy of this bound is
/// a private constant. `breakaway_terminal_cause_constants_match_band_edges`
/// below pins this literal against those same public constants so the two
/// cannot silently drift.
const REQUIRED_HALF_WIDTH_FRACTION: f32 = 1.0 / 15.0;

/// Largest representable magnitude for a `_percent_milli` wire field: 1000%,
/// far beyond any real response percentage but small enough that the cast to
/// `i32` below is always exact.
const PERCENT_MILLI_SATURATION_BOUND: f32 = 1_000_000.0;

fn percent_to_milli(value: f32) -> i32 {
    let scaled = libm::roundf(value * 1_000.0);
    if scaled >= PERCENT_MILLI_SATURATION_BOUND {
        PERCENT_MILLI_SATURATION_BOUND as i32
    } else if scaled <= -PERCENT_MILLI_SATURATION_BOUND {
        -(PERCENT_MILLI_SATURATION_BOUND as i32)
    } else {
        scaled as i32
    }
}

fn relative_se_to_permille(value: f32) -> u16 {
    let scaled = libm::roundf(value * 1_000.0);
    if scaled >= f32::from(u16::MAX) {
        u16::MAX
    } else if scaled <= 0.0 {
        0
    } else {
        scaled as u16
    }
}

/// Map a non-`BreakawayFound` probe outcome to its unified terminal cause and
/// whether it carries retained safety-fault evidence.
pub(crate) fn probe_terminal_cause(
    outcome: &BreakawayProbeOutcome,
) -> (BreakawayTerminalCause, u8) {
    match outcome {
        BreakawayProbeOutcome::BreakawayFound(_) => (BreakawayTerminalCause::None, 0),
        BreakawayProbeOutcome::NoMotionAtAuthority => {
            (BreakawayTerminalCause::ProbeNoMotionAtAuthority, 0)
        }
        BreakawayProbeOutcome::UnstableBreakaway => {
            (BreakawayTerminalCause::ProbeUnstableBreakaway, 0)
        }
        BreakawayProbeOutcome::DirectionalBreakawayDisagreement => (
            BreakawayTerminalCause::ProbeDirectionalBreakawayDisagreement,
            0,
        ),
        BreakawayProbeOutcome::ProbeEvidenceExcluded => {
            (BreakawayTerminalCause::ProbeEvidenceExcluded, 0)
        }
        BreakawayProbeOutcome::AuthorityFailure => {
            (BreakawayTerminalCause::ProbeAuthorityFailure, 0)
        }
        BreakawayProbeOutcome::SafetyFault { safety, .. } => (
            BreakawayTerminalCause::ProbeSafetyFault,
            u8::from(safety.is_some()),
        ),
    }
}

/// Map a discovery outcome to its unified terminal cause and whether it
/// carries retained safety-fault evidence.
///
/// [`AdditiveDiscoveryOutcome::TargetBandSelected`] is the discovery success
/// path into confirmation, not a failure, so it maps to
/// [`BreakawayTerminalCause::None`].
pub(crate) fn discovery_terminal_cause(
    outcome: &AdditiveDiscoveryOutcome,
) -> (BreakawayTerminalCause, u8) {
    match outcome {
        AdditiveDiscoveryOutcome::TargetBandSelected(_) => (BreakawayTerminalCause::None, 0),
        AdditiveDiscoveryOutcome::TargetBandSkipped => {
            (BreakawayTerminalCause::DiscoveryTargetBandSkipped, 0)
        }
        AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand => (
            BreakawayTerminalCause::DiscoveryCurrentHeadroomBeforeBand,
            0,
        ),
        AdditiveDiscoveryOutcome::InsufficientAdditiveSpan => {
            (BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan, 0)
        }
        AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => (
            BreakawayTerminalCause::DiscoveryAdditiveLadderResolutionBudgetExceeded,
            0,
        ),
        AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded => {
            (BreakawayTerminalCause::DiscoveryEvidenceExcluded, 0)
        }
        AdditiveDiscoveryOutcome::SafetyFault { safety, .. } => (
            BreakawayTerminalCause::DiscoverySafetyFault,
            u8::from(safety.is_some()),
        ),
    }
}

/// Map a zero-stroke confirmation outcome to its unified terminal cause and
/// whether it carries retained safety-fault evidence.
///
/// Only the confirmation outcomes reachable WITHOUT running
/// [`super::band_confirmation::evaluate_band_confirmation`] apply here:
/// [`BandConfirmationOutcome::Accepted`] and
/// [`BandConfirmationOutcome::TargetBandConfirmationInconclusive`] carry their
/// own evaluated masks and are reported by
/// [`confirmation_evaluated_terminal_report`] instead.
pub(crate) fn confirmation_zero_stroke_cause(
    outcome: &BandConfirmationOutcome,
) -> (BreakawayTerminalCause, u8) {
    match outcome {
        BandConfirmationOutcome::Accepted { .. } => (BreakawayTerminalCause::Accepted, 0),
        BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery => (
            BreakawayTerminalCause::ConfirmationTargetBandUnconfirmableFromDiscovery,
            0,
        ),
        BandConfirmationOutcome::TargetBandNotReachedAtAuthority => (
            BreakawayTerminalCause::ConfirmationTargetBandNotReachedAtAuthority,
            0,
        ),
        BandConfirmationOutcome::TargetBandConfirmationInconclusive(cause) => {
            (confirmation_failure_cause(*cause), 0)
        }
        BandConfirmationOutcome::ConfirmationEvidenceExcluded => {
            (BreakawayTerminalCause::ConfirmationEvidenceExcluded, 0)
        }
        BandConfirmationOutcome::SafetyFault { safety, .. } => (
            BreakawayTerminalCause::ConfirmationSafetyFault,
            u8::from(safety.is_some()),
        ),
    }
}

const fn confirmation_failure_cause(cause: BandConfirmationFailureCause) -> BreakawayTerminalCause {
    match cause {
        BandConfirmationFailureCause::ResponseLocation => {
            BreakawayTerminalCause::ConfirmationResponseLocation
        }
        BandConfirmationFailureCause::ExcessiveUncertainty => {
            BreakawayTerminalCause::ConfirmationExcessiveUncertainty
        }
        BandConfirmationFailureCause::LossOfConsensus => {
            BreakawayTerminalCause::ConfirmationLossOfConsensus
        }
        BandConfirmationFailureCause::CurrentAuthority => {
            BreakawayTerminalCause::ConfirmationCurrentAuthority
        }
    }
}

/// Report the immutable probe plan before the first probe stroke.
pub(crate) fn probe_plan_report(
    plan: &BreakawayProbePlan,
    p_start: PidPGain,
    p_top: PidPGain,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayProbePlanned {
    BreakawayProbePlanned {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        family_size: plan.confidence_family_size,
        max_observations: plan.maximum_observations,
        search_count: plan.search_count,
        p_start_raw: p_start.raw(),
        p_top_raw: p_top.raw(),
    }
}

/// Report one direction's resolved inert/moving bracket, resolving its
/// post-dedupe grid indices back to gains through the owning probe plan.
pub(crate) fn directional_breakaway_report(
    plan: &BreakawayProbePlan,
    direction: VelocityDirection,
    breakaway: &DirectionalBreakaway,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayDirectionalBreakaway {
    let inert_p_raw = breakaway
        .inert_index
        .and_then(|index| plan.search_gains.get(usize::from(index)))
        .map_or(0, |gain| gain.raw());
    let moving_p_raw = plan
        .search_gains
        .get(usize::from(breakaway.moving_index))
        .map_or(0, |gain| gain.raw());
    BreakawayDirectionalBreakaway {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        direction: direction_report_code(direction),
        inert_present: u8::from(breakaway.inert_index.is_some()),
        inert_p_raw,
        moving_p_raw,
        observations: breakaway.observations,
    }
}

/// Report a probe campaign terminal that did not resolve a breakaway.
pub(crate) fn probe_terminal_report(
    plan: &BreakawayProbePlan,
    outcome: &BreakawayProbeOutcome,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayProbeTerminal {
    let (cause, has_safety_fault) = probe_terminal_cause(outcome);
    BreakawayProbeTerminal {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        family_size: plan.confidence_family_size,
        terminal_cause: cause as u8,
        has_safety_fault,
    }
}

/// Fixed additive step from the resolved breakaway to the first ladder rung
/// strictly above it, reconstructed from the resolved ladder's own adjacent
/// gains rather than a retained planner intermediate.
fn first_additive_step_raw(plan: &AdditiveDiscoveryPlan, breakaway_raw: u16) -> u32 {
    let gains = &plan.gains[..usize::from(plan.rung_count)];
    let Some(position) = gains.iter().position(|gain| gain.raw() == breakaway_raw) else {
        return 0;
    };
    match gains.get(position + 1) {
        Some(next) => u32::from(next.raw()).saturating_sub(u32::from(breakaway_raw)),
        None => 0,
    }
}

/// Report the immutable additive discovery plan before the first discovery
/// stroke: chosen floor, breakaway, ceiling, and additive step.
pub(crate) fn discovery_plan_report(
    request: &AdditiveDiscoveryRequest,
    floor_origin_clamped: bool,
    plan: &AdditiveDiscoveryPlan,
    run_sequence: u32,
    evidence_sequence: u16,
) -> AdditiveDiscoveryPlanned {
    AdditiveDiscoveryPlanned {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        prior_plan_digest: plan.probe_plan_digest,
        family_size: plan.confidence_family_size,
        rung_count: plan.rung_count,
        observations_per_direction: plan.observations_per_direction,
        floor_p_raw: request.floor.raw(),
        floor_origin: u8::from(floor_origin_clamped),
        breakaway_p_raw: request.breakaway.raw(),
        ceiling_p_raw: request.authority_ceiling.raw(),
        first_additive_step_raw: first_additive_step_raw(plan, request.breakaway.raw()),
        maximum_workflow_ms: request.workflow_envelope_ms,
    }
}

/// Report one rung of the complete resolved additive ladder.
///
/// Returns `None` for an out-of-range or unused index, so a caller can drive
/// this across `0..plan.rung_count` without a separate bounds check.
pub(crate) fn ladder_rung_report(
    plan: &AdditiveDiscoveryPlan,
    rung_index: u8,
    run_sequence: u32,
    evidence_sequence: u16,
) -> Option<BreakawayLadderRung> {
    if rung_index >= plan.rung_count {
        return None;
    }
    let gain = plan.gains.get(usize::from(rung_index))?;
    Some(BreakawayLadderRung {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        rung_index,
        p_raw: gain.raw(),
    })
}

/// Report one in-band discovery rung's nomination bounds and pointwise margin
/// `M_i`, plus its achieved rate step from the previous collected in-band
/// rung's own midpoint response.
///
/// `bounds` is the collected in-band set in collection (ascending rung) order
/// -- see [`CollectedInBandRungs::as_slice`]. Returns `None` for an
/// out-of-range index or a non-nominatable entry (which carries no meaningful
/// bounds, see [`DiscoveryRungNominationBounds::nominatable`]).
pub(crate) fn discovery_rung_margin_report(
    bounds: &[DiscoveryRungNominationBounds],
    collected_index: u8,
    plan_digest: u64,
    run_sequence: u32,
    evidence_sequence: u16,
) -> Option<BreakawayDiscoveryRungMargin> {
    let entry = bounds.get(usize::from(collected_index))?;
    if !entry.nominatable {
        return None;
    }
    let lower_percent_milli = percent_to_milli(entry.lower_percent);
    let upper_percent_milli = percent_to_milli(entry.upper_percent);
    let lower_band_milli = i32::from(CONFIRMATION_BAND_LOWER_PERCENT) * 1_000;
    let upper_band_milli = i32::from(CONFIRMATION_BAND_UPPER_PERCENT) * 1_000;
    let margin_percent_milli =
        (lower_percent_milli - lower_band_milli).min(upper_band_milli - upper_percent_milli);
    let half_width_percent_milli = (upper_percent_milli - lower_percent_milli) / 2;
    let adjacent_rate_step_permille = if collected_index == 0 {
        0
    } else {
        match bounds.get(usize::from(collected_index) - 1) {
            Some(previous) if previous.nominatable => {
                let previous_mid = (percent_to_milli(previous.lower_percent)
                    + percent_to_milli(previous.upper_percent))
                    / 2;
                let current_mid = (lower_percent_milli + upper_percent_milli) / 2;
                current_mid - previous_mid
            }
            _ => 0,
        }
    };
    Some(BreakawayDiscoveryRungMargin {
        run_sequence,
        evidence_sequence,
        plan_digest,
        collected_index,
        p_raw: entry.p.raw(),
        lower_percent_milli,
        upper_percent_milli,
        margin_percent_milli,
        half_width_percent_milli,
        adjacent_rate_step_permille,
    })
}

/// Report whether the additive ladder's rung zero (the diagnostic floor)
/// itself showed motion (spec section 6: reported purely as evidence, never a
/// failure).
///
/// `forward`/`reverse` are the rung-zero observations' own motion
/// classifications. The current additive discovery driver
/// ([`super::breakaway_campaign::step_additive_discovery`]) does not retain
/// rung zero's classification once the ladder walk advances past it unless
/// rung zero happens to be collected as in-band; live emission of this record
/// needs that classification threaded out of the driver as an explicit
/// caller-visible value, which is a wiring change outside this reporting
/// module's scope.
pub(crate) fn rung_zero_diagnostic_report(
    plan: &AdditiveDiscoveryPlan,
    forward: BreakawayMotionClass,
    reverse: BreakawayMotionClass,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayRungZeroDiagnostic {
    BreakawayRungZeroDiagnostic {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        forward_moved: u8::from(matches!(forward, BreakawayMotionClass::Moving)),
        reverse_moved: u8::from(matches!(reverse, BreakawayMotionClass::Moving)),
    }
}

/// Assemble a discovery terminal record from its already-classified fields.
///
/// Split out from [`discovery_terminal_report`] so the record-shaping logic
/// is directly testable with an explicit `collected_count`, independent of
/// constructing a live [`CollectedInBandRungs`] (whose constructor is private
/// to [`super::breakaway_campaign`]).
pub(crate) fn discovery_terminal_fields(
    plan: &AdditiveDiscoveryPlan,
    cause: BreakawayTerminalCause,
    has_safety_fault: u8,
    collected_count: u8,
    run_sequence: u32,
    evidence_sequence: u16,
) -> AdditiveDiscoveryTerminal {
    AdditiveDiscoveryTerminal {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        prior_plan_digest: plan.probe_plan_digest,
        family_size: plan.confidence_family_size,
        terminal_cause: cause as u8,
        collected_count,
        has_safety_fault,
    }
}

/// Report a discovery campaign terminal.
///
/// `outcome` may be [`AdditiveDiscoveryOutcome::TargetBandSelected`] (the
/// success path into confirmation, `terminal_cause == None`) or any
/// discovery-owned failure terminal.
pub(crate) fn discovery_terminal_report(
    plan: &AdditiveDiscoveryPlan,
    outcome: &AdditiveDiscoveryOutcome,
    run_sequence: u32,
    evidence_sequence: u16,
) -> AdditiveDiscoveryTerminal {
    let (cause, has_safety_fault) = discovery_terminal_cause(outcome);
    let collected_count = match outcome {
        AdditiveDiscoveryOutcome::TargetBandSelected(rungs) => rungs.count(),
        AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
        | AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded
        | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded
        | AdditiveDiscoveryOutcome::SafetyFault { .. } => 0,
    };
    discovery_terminal_fields(
        plan,
        cause,
        has_safety_fault,
        collected_count,
        run_sequence,
        evidence_sequence,
    )
}

/// Report the immutable held-out confirmation plan before the first
/// confirmation stroke, including the nomination screen's margin for the
/// nominated candidate.
pub(crate) fn confirmation_plan_report(
    plan: &BandConfirmationPlan,
    nomination: &NominatedConfirmationCandidate,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BandConfirmationPlanned {
    BandConfirmationPlanned {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        prior_plan_digest: plan.discovery_plan_digest,
        family_size: plan.family_size,
        observations_per_direction: plan.observations_per_direction,
        candidate_p_raw: plan.candidate_p.raw(),
        band_lower_percent: plan.band_lower_percent,
        band_upper_percent: plan.band_upper_percent,
        capture_profile: plan.capture_profile,
        acceptance_rule: plan.acceptance_rule,
        nominated_margin_percent_milli: percent_to_milli(nomination.margin_percent),
    }
}

/// Report one fresh held-out confirmation stroke's response interval.
///
/// Reuses [`observation_interval_percent`] with `c_sigma = 1.0` and
/// `round_outward = true` -- exactly the parameters
/// [`super::band_confirmation::evaluate_band_confirmation`]'s own inline
/// computation uses for real held-out data (no nomination-style inflation),
/// so this record's interval is provably identical to the one acceptance
/// judged. Returns `None` when the observation's mean is zero, wrong-signed,
/// or otherwise not representable as a candidate response (the same
/// condition that fails acceptance with `ResponseLocation`).
pub(crate) fn confirmation_observation_report(
    plan_digest: u64,
    slot_index: u8,
    direction: VelocityDirection,
    atom: DiscoveryObservationAtom,
    run_sequence: u32,
    evidence_sequence: u16,
) -> Option<BandConfirmationObservation> {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE).ok()?;
    let (lower, upper) = observation_interval_percent(atom, multiplier, 1.0, true)?;
    let statistic = observation_statistic(atom).ok()?;
    Some(BandConfirmationObservation {
        run_sequence,
        evidence_sequence,
        plan_digest,
        slot_index,
        direction: direction_report_code(direction),
        response_lower_percent_milli: percent_to_milli(lower),
        response_upper_percent_milli: percent_to_milli(upper),
        relative_standard_error_permille: relative_se_to_permille(
            statistic.relative_standard_error,
        ),
    })
}

/// Report a confirmation terminal for the case where all eight strokes ran --
/// [`super::band_confirmation::evaluate_band_confirmation`]'s own input
/// shape -- whether accepted or judged
/// [`super::band_confirmation::TargetBandConfirmationInconclusive`].
///
/// For the zero-stroke confirmation terminals (no evaluation ever ran), use
/// [`confirmation_zero_stroke_terminal_report`] instead.
pub(crate) fn confirmation_evaluated_terminal_report(
    plan: &BandConfirmationPlan,
    forward: &ConfirmationDirectionalInput,
    reverse: &ConfirmationDirectionalInput,
    accepted_p_raw: Option<u16>,
    failure_cause: Option<BandConfirmationFailureCause>,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BandConfirmationTerminal {
    let (terminal_cause, accepted, confirmed_p_raw) = match (accepted_p_raw, failure_cause) {
        (Some(p_raw), _) => (BreakawayTerminalCause::Accepted, 1_u8, p_raw),
        (None, Some(cause)) => (confirmation_failure_cause(cause), 0_u8, 0_u16),
        (None, None) => (BreakawayTerminalCause::None, 0_u8, 0_u16),
    };
    confirmation_terminal_from_collected_evidence(
        plan,
        forward,
        reverse,
        ConfirmationTerminalOutcome {
            terminal_cause,
            accepted,
            confirmed_p_raw,
            run_sequence,
            evidence_sequence,
        },
    )
}

/// Report a confirmation terminal reached mid-block: earlier strokes already
/// folded evidence into `forward`/`reverse` before the outcome under
/// `terminal_cause` cut the block short (for example
/// [`BandConfirmationOutcome::ConfirmationEvidenceExcluded`] after one or more
/// prior strokes succeeded).
///
/// Unlike [`confirmation_zero_stroke_terminal_report`], this reports the
/// masks and observations actually collected rather than all-zero fields, so
/// the terminal does not falsely claim no confirmation work happened. Shares
/// the same mask/uncertainty assembly as
/// [`confirmation_evaluated_terminal_report`] because both report whatever
/// evidence `forward`/`reverse` actually hold; only the not-yet-evaluated
/// `terminal_cause` and the always-unaccepted outcome differ.
pub(crate) fn confirmation_partial_evidence_terminal_report(
    plan: &BandConfirmationPlan,
    forward: &ConfirmationDirectionalInput,
    reverse: &ConfirmationDirectionalInput,
    terminal_cause: BreakawayTerminalCause,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BandConfirmationTerminal {
    confirmation_terminal_from_collected_evidence(
        plan,
        forward,
        reverse,
        ConfirmationTerminalOutcome {
            terminal_cause,
            accepted: 0,
            confirmed_p_raw: 0,
            run_sequence,
            evidence_sequence,
        },
    )
}

/// Resolved outcome fields for a confirmation terminal that reports evidence
/// actually collected in `forward`/`reverse`, bundled so
/// [`confirmation_terminal_from_collected_evidence`] stays within the
/// project's five-parameter guideline.
struct ConfirmationTerminalOutcome {
    terminal_cause: BreakawayTerminalCause,
    accepted: u8,
    confirmed_p_raw: u16,
    run_sequence: u32,
    evidence_sequence: u16,
}

/// Shared mask/uncertainty assembly for a confirmation terminal that reports
/// whatever evidence `forward`/`reverse` actually collected, whether the
/// block ran to completion ([`confirmation_evaluated_terminal_report`]) or
/// was cut short after some strokes ran
/// ([`confirmation_partial_evidence_terminal_report`]).
fn confirmation_terminal_from_collected_evidence(
    plan: &BandConfirmationPlan,
    forward: &ConfirmationDirectionalInput,
    reverse: &ConfirmationDirectionalInput,
    outcome: ConfirmationTerminalOutcome,
) -> BandConfirmationTerminal {
    let ConfirmationTerminalOutcome {
        terminal_cause,
        accepted,
        confirmed_p_raw,
        run_sequence,
        evidence_sequence,
    } = outcome;
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE).unwrap_or(0.0);
    let max_relative_se = forward
        .observations
        .iter()
        .chain(reverse.observations.iter())
        .filter_map(|atom| *atom)
        .filter_map(|atom| observation_statistic(atom).ok())
        .fold(0.0_f32, |worst, statistic| {
            worst.max(statistic.relative_standard_error)
        });
    let required_relative_se = if multiplier > 0.0 {
        REQUIRED_HALF_WIDTH_FRACTION / multiplier
    } else {
        0.0
    };
    BandConfirmationTerminal {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        prior_plan_digest: plan.discovery_plan_digest,
        family_size: plan.family_size,
        terminal_cause: terminal_cause as u8,
        forward_collected_mask: forward.consensus.collected_mask,
        forward_eligible_mask: forward.consensus.eligible_mask,
        forward_included_mask: forward.consensus.included_mask,
        reverse_collected_mask: reverse.consensus.collected_mask,
        reverse_eligible_mask: reverse.consensus.eligible_mask,
        reverse_included_mask: reverse.consensus.included_mask,
        accepted,
        confirmed_p_raw,
        max_relative_se_permille: relative_se_to_permille(max_relative_se),
        required_relative_se_permille: relative_se_to_permille(required_relative_se),
        has_safety_fault: 0,
    }
}

/// Report a confirmation terminal reached before any confirmation stroke, or
/// through the safety short-circuit: zero strokes were spent, so every mask
/// is honestly zero.
pub(crate) fn confirmation_zero_stroke_terminal_report(
    plan_digest: u64,
    prior_plan_digest: u64,
    family_size: u16,
    outcome: &BandConfirmationOutcome,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BandConfirmationTerminal {
    let (terminal_cause, has_safety_fault) = confirmation_zero_stroke_cause(outcome);
    BandConfirmationTerminal {
        run_sequence,
        evidence_sequence,
        plan_digest,
        prior_plan_digest,
        family_size,
        terminal_cause: terminal_cause as u8,
        forward_collected_mask: 0,
        forward_eligible_mask: 0,
        forward_included_mask: 0,
        reverse_collected_mask: 0,
        reverse_eligible_mask: 0,
        reverse_included_mask: 0,
        accepted: 0,
        confirmed_p_raw: 0,
        max_relative_se_permille: 0,
        required_relative_se_permille: 0,
        has_safety_fault,
    }
}

/// Report the canonical closure record for one complete breakaway campaign.
pub(crate) fn campaign_terminal_report(
    phase: BreakawayPhase,
    terminal_cause: BreakawayTerminalCause,
    accepted: bool,
    stage_c_plan_digest: u64,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayCampaignTerminal {
    BreakawayCampaignTerminal {
        run_sequence,
        evidence_sequence,
        phase: phase as u8,
        terminal_cause: terminal_cause as u8,
        accepted: u8::from(accepted),
        stage_c_plan_digest,
    }
}

#[cfg(test)]
#[path = "breakaway_reporting_tests.rs"]
mod tests;
