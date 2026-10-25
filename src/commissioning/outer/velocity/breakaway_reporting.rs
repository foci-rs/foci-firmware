use foci_tmc4671::pid::PidPGain;

use crate::autotune::velocity_measurement::{VelocityDirection, confidence_multiplier_95_df15};
#[cfg(any(test, feature = "trace"))]
use crate::commissioning::types::BandConfirmationObservation;
use crate::commissioning::types::{
    AdditiveDiscoveryPlanned, AdditiveDiscoveryTerminal, BandConfirmationPlanned,
    BandConfirmationTerminal, BreakawayCampaignTerminal, BreakawayDiscoveryRungMargin,
    BreakawayProbeDecision, BreakawayProbePlanned, BreakawayProbeResolved, BreakawayProbeTerminal,
    BreakawayRawObservation, BreakawayRungZeroDiagnostic,
};

use super::breakaway_plan::BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION;

use super::band_confirmation::{
    BandConfirmationFailureCause, BandConfirmationPlan, CONFIRMATION_BAND_LOWER_PERCENT,
    CONFIRMATION_BAND_UPPER_PERCENT, CONFIRMATION_FAMILY_SIZE, ConfirmationDirectionalInput,
    DiscoveryRungNominationBounds, NominatedConfirmationCandidate, observation_statistic,
};
#[cfg(any(test, feature = "trace"))]
use super::band_confirmation::{DiscoveryObservationAtom, observation_interval_percent};
use super::breakaway_campaign::{
    AdditiveDiscoveryOutcome, BandConfirmationOutcome, BreakawayProbeOutcome, CollectedObservation,
    NominationEvidence,
};
use super::breakaway_plan::{
    AdditiveDiscoveryPlan, AdditiveDiscoveryRequest, UpwardBreakawayProbePlan,
};
use super::breakaway_search::BreakawayMotionClass;
use super::sweep_evidence::direction_report_code;

pub(crate) use crate::commissioning::vocabulary_generated::BreakawayPhase;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum BreakawayTerminalCause {
    None = 0,
    ProbeNoRepeatableMotionAtAuthority = 1,
    ProbeAuthorityFailure = 5,
    ProbeSafetyFault = 6,
    DiscoveryTargetBandSkipped = 7,
    DiscoveryCurrentHeadroomBeforeBand = 8,
    DiscoveryInsufficientAdditiveSpan = 9,
    DiscoveryAdditiveLadderResolutionBudgetExceeded = 10,
    DiscoveryEvidenceExcluded = 11,
    DiscoverySafetyFault = 12,
    ConfirmationTargetBandUnconfirmableFromDiscovery = 13,
    ConfirmationTargetBandNotReachedAtAuthority = 14,
    ConfirmationResponseLocation = 15,
    ConfirmationExcessiveUncertainty = 16,
    ConfirmationLossOfConsensus = 17,
    ConfirmationCurrentAuthority = 18,
    ConfirmationEvidenceExcluded = 19,
    ConfirmationSafetyFault = 20,
    Accepted = 21,
    ProbeExcursionEvidenceInvalid = 22,
    ProbeInternalFault = 23,
    DiscoveryInternalFault = 24,
    ConfirmationInternalFault = 25,
    ConfirmationIntegralPlanRefused = 26,
    ConfirmationNoTransitionCapableCandidate = 27,
    /// Every band-admissible, transition-capable rung exceeded the step peak bound.
    ConfirmationNoDampedCandidate = 28,
}

const REQUIRED_HALF_WIDTH_FRACTION: f32 = 1.0 / 15.0;

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

pub(crate) fn probe_terminal_cause(
    outcome: &BreakawayProbeOutcome,
) -> (BreakawayTerminalCause, u8) {
    match outcome {
        BreakawayProbeOutcome::BreakawayFound(_) => (BreakawayTerminalCause::None, 0),
        BreakawayProbeOutcome::NoRepeatableMotionAtAuthority => (
            BreakawayTerminalCause::ProbeNoRepeatableMotionAtAuthority,
            0,
        ),
        BreakawayProbeOutcome::ExcursionEvidenceInvalid => {
            (BreakawayTerminalCause::ProbeExcursionEvidenceInvalid, 0)
        }
        BreakawayProbeOutcome::AuthorityFailure => {
            (BreakawayTerminalCause::ProbeAuthorityFailure, 0)
        }
        BreakawayProbeOutcome::SafetyFault { safety, .. } => (
            if safety.is_some() {
                BreakawayTerminalCause::ProbeSafetyFault
            } else {
                BreakawayTerminalCause::ProbeInternalFault
            },
            u8::from(safety.is_some()),
        ),
    }
}

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
        AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { .. } => {
            (BreakawayTerminalCause::DiscoveryEvidenceExcluded, 0)
        }
        AdditiveDiscoveryOutcome::SafetyFault { safety, .. } => (
            if safety.is_some() {
                BreakawayTerminalCause::DiscoverySafetyFault
            } else {
                BreakawayTerminalCause::DiscoveryInternalFault
            },
            u8::from(safety.is_some()),
        ),
    }
}

pub(crate) fn confirmation_zero_stroke_cause(
    outcome: &BandConfirmationOutcome,
) -> (BreakawayTerminalCause, u8) {
    match outcome {
        BandConfirmationOutcome::Accepted { .. } => (BreakawayTerminalCause::Accepted, 0),
        BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery => (
            BreakawayTerminalCause::ConfirmationTargetBandUnconfirmableFromDiscovery,
            0,
        ),
        BandConfirmationOutcome::TargetBandNoTransitionCapableCandidate => (
            BreakawayTerminalCause::ConfirmationNoTransitionCapableCandidate,
            0,
        ),
        BandConfirmationOutcome::TargetBandNoDampedCandidate { .. } => {
            (BreakawayTerminalCause::ConfirmationNoDampedCandidate, 0)
        }
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
            if safety.is_some() {
                BreakawayTerminalCause::ConfirmationSafetyFault
            } else {
                BreakawayTerminalCause::ConfirmationInternalFault
            },
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
        BandConfirmationFailureCause::IncompleteObservations => {
            BreakawayTerminalCause::ConfirmationLossOfConsensus
        }
        BandConfirmationFailureCause::CurrentAuthority => {
            BreakawayTerminalCause::ConfirmationCurrentAuthority
        }
    }
}

pub(crate) fn probe_plan_report(
    plan: &UpwardBreakawayProbePlan,
    p_start: PidPGain,
    p_top: PidPGain,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayProbePlanned {
    BreakawayProbePlanned {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        max_observations: plan.maximum_observations,
        search_count: plan.search_count,
        p_start_raw: p_start.raw(),
        p_top_raw: p_top.raw(),
        workflow_digest: plan.context.workflow_digest,
        target_velocity_rpm: plan.context.target_velocity_rpm,
        hard_torque_limit: plan.context.hard_torque_limit,
        usable_torque_limit: plan.context.usable_torque_limit,
        pwm_hz: plan.context.pwm_hz,
        encoder_counts_per_rev: plan.context.encoder_counts_per_rev,
        motion_threshold_counts: plan.motion_threshold_counts,
        max_capture_interval_us: plan.max_capture_interval_us,
    }
}

pub(crate) fn probe_resolved_report(
    plan: &UpwardBreakawayProbePlan,
    rung_index: u8,
    observation_count: u8,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayProbeResolved {
    let breakaway_p_raw = plan
        .search_gains
        .get(usize::from(rung_index))
        .map_or(0, |gain| gain.raw());
    BreakawayProbeResolved {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        rung_index,
        breakaway_p_raw,
        motion_threshold_counts: plan.motion_threshold_counts,
        observation_count,
    }
}

pub(crate) fn probe_terminal_report(
    plan: &UpwardBreakawayProbePlan,
    outcome: &BreakawayProbeOutcome,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayProbeTerminal {
    let (cause, has_safety_fault) = probe_terminal_cause(outcome);
    let fault_checkpoint = if matches!(
        outcome,
        BreakawayProbeOutcome::SafetyFault { .. } | BreakawayProbeOutcome::ExcursionEvidenceInvalid
    ) {
        crate::diagnostic::active_breakaway_checkpoint().map_or(0, |checkpoint| checkpoint as u8)
    } else {
        0
    };
    BreakawayProbeTerminal {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        terminal_cause: cause as u8,
        has_safety_fault,
        fault_checkpoint,
    }
}

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
        schema_revision: BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION,
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

pub(crate) fn raw_observation_report(
    plan_digest: u64,
    phase: BreakawayPhase,
    collected: &CollectedObservation,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayRawObservation {
    let atom = collected.atom;
    let inputs = collected.classifier_inputs;
    BreakawayRawObservation {
        run_sequence,
        evidence_sequence,
        plan_digest,
        phase: phase as u8,
        rung_index: collected.rung_index,
        slot_index: collected.slot_index,
        direction: direction_report_code(atom.direction),
        mean_rate_q: atom.mean_rate_q,
        variance_of_mean_q2: atom.variance_of_mean_q2,
        target_rate_q: atom.target_rate_q,
        stationarity_code: inputs.stationarity_code,
        settled_rate_low_q: inputs.settled_rate_low_q,
        settled_rate_high_q: inputs.settled_rate_high_q,
        deficit_low_q: inputs.deficit_low_q,
        deficit_high_q: inputs.deficit_high_q,
        predicted_torque_target_abs: inputs.predicted_torque_target_abs,
        usable_torque_limit: inputs.usable_torque_limit,
        hard_torque_limit: inputs.hard_torque_limit,
        disturbance_present: u8::from(inputs.disturbance_present),
        disturbance_low_q16: inputs.disturbance_low_q16,
        disturbance_high_q16: inputs.disturbance_high_q16,
        drift_ratio_ppm: inputs.drift_ratio_ppm,
        post_fit_lag_one_state: inputs.post_fit_lag_one_state,
        stroke_min_displacement_counts: collected.stroke_min_displacement_counts,
        stroke_max_displacement_counts: collected.stroke_max_displacement_counts,
        capture_max_interval_us: collected.capture_max_interval_us,
        moving_pid_output_peak: inputs.moving_pid_output_peak,
    }
}

pub(crate) struct ProbeDecisionReportInput {
    pub(crate) direction: VelocityDirection,
    pub(crate) rung_index: u8,
    pub(crate) disposition: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RecoveryRestRejectedInput {
    pub(crate) decision: crate::autotune::velocity_measurement::RestSelectionDecision,
    pub(crate) stage: u8,
    pub(crate) rung_index: u8,
    pub(crate) slot_index: u8,
    pub(crate) direction: VelocityDirection,
    pub(crate) p_raw: u16,
    pub(crate) scored_net_displacement_counts: i8,
    pub(crate) scored_min_displacement_counts: i8,
    pub(crate) scored_max_displacement_counts: i8,
}

pub(crate) fn recovery_rest_rejected_report(
    plan_digest: u64,
    phase: BreakawayPhase,
    input: RecoveryRestRejectedInput,
    run_sequence: u32,
    evidence_sequence: u16,
) -> crate::commissioning::types::BreakawayRecoveryRestRejected {
    let decision = input.decision;
    crate::commissioning::types::BreakawayRecoveryRestRejected {
        run_sequence,
        evidence_sequence,
        plan_digest,
        phase: phase as u8,
        stage: input.stage,
        rung_index: input.rung_index,
        slot_index: input.slot_index,
        direction: direction_report_code(input.direction),
        p_raw: input.p_raw,
        structural_codes: [
            decision.structural_codes[0] as u8,
            decision.structural_codes[1] as u8,
            decision.structural_codes[2] as u8,
        ],
        interval_present_mask: decision.interval_present_mask,
        usable_mask: decision.usable_mask,
        zero_compatible_mask: decision.zero_compatible_mask,
        selected_mask: decision.selected_mask,
        selected_suffix_len: decision.selected_suffix_len,
        disposition: decision.disposition as u8,
        intervals_q: decision.intervals_q,
        scored_net_displacement_counts: input.scored_net_displacement_counts,
        scored_min_displacement_counts: input.scored_min_displacement_counts,
        scored_max_displacement_counts: input.scored_max_displacement_counts,
    }
}

pub(crate) fn recovery_origin_mismatch_report(
    plan_digest: u64,
    phase: BreakawayPhase,
    evidence: super::recovery::RecoveryOriginMismatchEvidence,
    slot_index: u8,
    direction: VelocityDirection,
    run_sequence: u32,
    evidence_sequence: u16,
) -> crate::commissioning::types::BreakawayRecoveryOriginMismatch {
    crate::commissioning::types::BreakawayRecoveryOriginMismatch {
        run_sequence,
        evidence_sequence,
        plan_digest,
        phase: phase as u8,
        stage: evidence.stage as u8,
        rung_index: evidence.rung_index,
        slot_index,
        direction: direction_report_code(direction),
        p_raw: evidence.p_raw,
        expected_start_offset_counts: evidence.expected_counts,
        observed_position_counts: evidence.observed_counts,
        delta_counts: evidence
            .observed_counts
            .saturating_sub(evidence.expected_counts),
        position_window_counts: evidence.position_window,
    }
}

pub(crate) fn probe_decision_report(
    plan_digest: u64,
    input: ProbeDecisionReportInput,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayProbeDecision {
    BreakawayProbeDecision {
        run_sequence,
        evidence_sequence,
        plan_digest,
        direction: direction_report_code(input.direction),
        rung_index: input.rung_index,
        disposition: input.disposition,
    }
}

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
    let adjacent_rate_step_percent_milli = if collected_index == 0 {
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
        adjacent_rate_step_percent_milli,
    })
}

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

pub(crate) struct DiscoveryTerminalEvidence {
    pub(crate) cause: BreakawayTerminalCause,
    pub(crate) has_safety_fault: u8,
    pub(crate) collected_count: u8,
    pub(crate) stop_rung_index: u8,
    pub(crate) error_code: u8,
}

pub(crate) fn discovery_terminal_fields(
    plan: &AdditiveDiscoveryPlan,
    evidence: DiscoveryTerminalEvidence,
    run_sequence: u32,
    evidence_sequence: u16,
) -> AdditiveDiscoveryTerminal {
    AdditiveDiscoveryTerminal {
        run_sequence,
        evidence_sequence,
        plan_digest: plan.plan_digest,
        prior_plan_digest: plan.probe_plan_digest,
        family_size: plan.confidence_family_size,
        terminal_cause: evidence.cause as u8,
        collected_count: evidence.collected_count,
        has_safety_fault: evidence.has_safety_fault,
        stop_rung_index: evidence.stop_rung_index,
        error_code: evidence.error_code,
    }
}

pub(crate) fn discovery_terminal_report(
    plan: &AdditiveDiscoveryPlan,
    outcome: &AdditiveDiscoveryOutcome,
    session_collected_count: u8,
    session_rung_index: u8,
    run_sequence: u32,
    evidence_sequence: u16,
) -> AdditiveDiscoveryTerminal {
    let (cause, has_safety_fault) = discovery_terminal_cause(outcome);
    let error_code = match outcome {
        AdditiveDiscoveryOutcome::SafetyFault { error, .. }
        | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { error } => error.status_code(),
        AdditiveDiscoveryOutcome::TargetBandSelected(_)
        | AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
        | AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => 0,
    };
    let collected_count = match outcome {
        AdditiveDiscoveryOutcome::TargetBandSelected(rungs) => rungs.count(),
        AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
        | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { .. }
        | AdditiveDiscoveryOutcome::SafetyFault { .. } => session_collected_count,
        AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => 0,
    };
    let stop_rung_index = match outcome {
        AdditiveDiscoveryOutcome::TargetBandSelected(_)
        | AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
        | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { .. }
        | AdditiveDiscoveryOutcome::SafetyFault { .. } => session_rung_index,
        AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => 0,
    };
    discovery_terminal_fields(
        plan,
        DiscoveryTerminalEvidence {
            cause,
            has_safety_fault,
            collected_count,
            stop_rung_index,
            error_code,
        },
        run_sequence,
        evidence_sequence,
    )
}

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
        nominated_p_raw: plan.nominated_p.raw(),
        band_lower_percent: plan.band_lower_percent,
        band_upper_percent: plan.band_upper_percent,
        capture_profile: plan.capture_profile,
        acceptance_rule: plan.acceptance_rule,
        nominated_margin_percent_milli: percent_to_milli(nomination.margin_percent),
    }
}

#[cfg(any(test, feature = "trace"))]
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
            has_safety_fault: 0,
        },
    )
}

pub(crate) fn confirmation_partial_evidence_terminal_report(
    plan: &BandConfirmationPlan,
    forward: &ConfirmationDirectionalInput,
    reverse: &ConfirmationDirectionalInput,
    terminal_cause: BreakawayTerminalCause,
    has_safety_fault: u8,
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
            has_safety_fault,
        },
    )
}

struct ConfirmationTerminalOutcome {
    terminal_cause: BreakawayTerminalCause,
    accepted: u8,
    confirmed_p_raw: u16,
    run_sequence: u32,
    evidence_sequence: u16,
    has_safety_fault: u8,
}

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
        has_safety_fault,
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
        has_safety_fault,
    }
}

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

#[allow(clippy::too_many_arguments)]
pub(crate) fn campaign_terminal_report(
    phase: BreakawayPhase,
    terminal_cause: BreakawayTerminalCause,
    accepted: bool,
    integral_plan_digest: u64,
    error_code: u8,
    nomination: NominationEvidence,
    run_sequence: u32,
    evidence_sequence: u16,
) -> BreakawayCampaignTerminal {
    BreakawayCampaignTerminal {
        run_sequence,
        evidence_sequence,
        phase: phase as u8,
        terminal_cause: terminal_cause as u8,
        accepted: u8::from(accepted),
        integral_plan_digest,
        error_code,
        nominated_step_peak_permille: nomination.nominated_step_peak_permille,
        overshoot_excluded_rungs: nomination.overshoot_excluded_rungs,
    }
}

#[cfg(test)]
#[path = "breakaway_reporting_tests.rs"]
mod tests;
