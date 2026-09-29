//! Dispatch-side preflight and terminal application for the reversal-standstill robustness gate.

use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptancePoint, IntegralRetentionSlot,
};
use crate::commissioning::outer::velocity::robustness_cycles::ROBUSTNESS_CYCLES_PER_DIRECTION;
use crate::commissioning::outer::velocity::robustness_evidence::{
    RobustnessDirectionSummary, RobustnessEvidence,
};
use crate::commissioning::outer::velocity::robustness_metrics::{
    ROBUSTNESS_LEGS_PER_CYCLE, ROBUSTNESS_MAX_RETRIES, ROBUSTNESS_MEASUREMENT_BUDGET_US,
};
use crate::commissioning::outer::velocity::robustness_reporting::ROBUSTNESS_SCHEMA_REVISION;
use crate::commissioning::outer::velocity::sweep_plan::ZERO_SETTLE_US;
use crate::commissioning::types::{RobustnessOutcome, RobustnessReversalTerminal};

const ROBUSTNESS_DIRECTION_COUNT: u32 = 2;

const ROBUSTNESS_TAIL_GRACE_CYCLES: u32 = 1;

const MICROS_PER_MILLI: u32 = 1_000;

/// No direction tripped a threshold gate and no direction was inconclusive.
pub const ROBUSTNESS_CAUSE_PASS: u8 = 0;
/// At least one direction's `reconvergence_time_us` exceeded its threshold.
pub const ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED: u8 = 1;
/// At least one direction's `settled_residual_q` exceeded its threshold.
pub const ROBUSTNESS_CAUSE_RESIDUAL_EXCEEDED: u8 = 2;
/// At least one direction's `recovery_iae_q` exceeded its threshold.
pub const ROBUSTNESS_CAUSE_IAE_EXCEEDED: u8 = 3;
/// At least one direction never reached a sustained band-entry dwell within
/// its measurement budget.
pub const ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE: u8 = 4;
/// No candidate survived acceptance to be probed for robustness.
pub const ROBUSTNESS_CAUSE_NO_ACCEPTED_CANDIDATE: u8 = 5;
/// Envelope/velocity-limit safety fault passthrough; carries a populated `safety_fault`.
pub const ROBUSTNESS_CAUSE_SAFETY_FAULT: u8 = 6;
/// Accepted-point identity mismatch, set at terminal application.
pub const ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY: u8 = 7;
/// The rest after a conclusive cycle was never confirmed, even after one re-opened window.
pub const ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED: u8 = 9;
/// Two conclusive tail cycles occurred in one reversal direction, latching a tail rejection.
pub const ROBUSTNESS_CAUSE_TAIL_REPEATED: u8 = 10;
/// The origin recovery preceding an establish retry left the axis where the next leg does not fit.
pub const ROBUSTNESS_CAUSE_ORIGIN_NOT_RECOVERED: u8 = 11;
/// A register write or read failed.
pub const ROBUSTNESS_CAUSE_IO: u8 = 12;
/// A safety-envelope precondition failed to arm.
pub const ROBUSTNESS_CAUSE_SAFETY_ARM_FAILED: u8 = 13;
/// A recovery plan or reversal schedule could not be constructed or does not fit the window.
pub const ROBUSTNESS_CAUSE_RECOVERY_PLAN_INVALID: u8 = 14;
/// The retained P/I gain pair was invalid.
pub const ROBUSTNESS_CAUSE_INVALID_GAINS: u8 = 15;
/// A checked deadline computation overflowed.
pub const ROBUSTNESS_CAUSE_DEADLINE_OVERFLOW: u8 = 16;
/// A stationarity analysis deadline was exceeded before it concluded.
pub const ROBUSTNESS_CAUSE_ANALYSIS_TIMEOUT: u8 = 17;
/// A stationarity or scored-rest-confirmation poll returned an internal error.
pub const ROBUSTNESS_CAUSE_ANALYSIS_POLL_FAILED: u8 = 18;
/// Reversal-leg scoring failed.
pub const ROBUSTNESS_CAUSE_SCORING_FAILED: u8 = 19;
/// A confirmed rest arrived with neither a pending cycle nor a held route to
/// recover from.
pub const ROBUSTNESS_CAUSE_UNEXPECTED_STATE: u8 = 20;
/// The origin-recovery worker reported an internal fault.
pub const ROBUSTNESS_CAUSE_ORIGIN_RECOVERY_FAULT: u8 = 21;
/// The outer dispatch aborted the run for a reason outside this phase's step loop.
pub const ROBUSTNESS_CAUSE_DISPATCH_ABORT: u8 = 22;

/// Definitive rejection of a robustness-reversal preflight.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RobustnessPreflightRejection {
    NoAcceptedCandidate,
}

/// Immutable robustness-reversal plan accepted before any probe motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RobustnessPreflight {
    pub point: AcceptancePoint,
    pub run_sequence: u32,
}

/// Validate a retained accepted point without mutating retention.
pub fn preflight_robustness_reversal(
    run_sequence: u32,
    integral_candidate: &IntegralRetentionSlot,
) -> Result<RobustnessPreflight, RobustnessPreflightRejection> {
    let point = integral_candidate
        .acceptance_point()
        .ok_or(RobustnessPreflightRejection::NoAcceptedCandidate)?;
    Ok(RobustnessPreflight {
        point,
        run_sequence,
    })
}

/// Build one definitive zero-motion robustness terminal from admission policy.
pub fn failed_robustness_terminal(
    run_sequence: u32,
    rejection: RobustnessPreflightRejection,
) -> RobustnessReversalTerminal {
    let cause = match rejection {
        RobustnessPreflightRejection::NoAcceptedCandidate => ROBUSTNESS_CAUSE_NO_ACCEPTED_CANDIDATE,
    };
    RobustnessReversalTerminal {
        schema_revision: ROBUSTNESS_SCHEMA_REVISION,
        run_sequence,
        outcome: RobustnessOutcome::Failed,
        cause,
        selected_p: 0,
        selected_i: 0,
        target_velocity_rpm: 0,
        plant_rate_q: 0,
        iae_max_q: 0,
        evidence: zero_robustness_evidence(),
        run_started_us: 0,
        run_completed_us: 0,
        safety_fault: None,
    }
}

/// Apply one reversal-robustness terminal after confirming the retained
/// accepted point's identity has not changed underneath the probe.
///
/// On identity mismatch (or an emptied slot) the terminal is `Failed`/`EVIDENCE_INTEGRITY`.
pub fn apply_robustness_terminal(
    mut terminal: RobustnessReversalTerminal,
    integral_candidate: &IntegralRetentionSlot,
) -> RobustnessReversalTerminal {
    let Some(point) = integral_candidate.acceptance_point() else {
        terminal.outcome = RobustnessOutcome::Failed;
        terminal.cause = ROBUSTNESS_CAUSE_NO_ACCEPTED_CANDIDATE;
        return terminal;
    };
    let identity_ok = point.selected_p == terminal.selected_p
        && point.selected_i == terminal.selected_i
        && point.target_velocity_rpm == terminal.target_velocity_rpm;
    if !identity_ok {
        terminal.outcome = RobustnessOutcome::Failed;
        terminal.cause = ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY;
    }
    terminal
}

pub(crate) fn robustness_worst_case_duration_ms(recovery_maximum_slot_us: u32) -> u32 {
    let cycles_per_direction = ROBUSTNESS_CYCLES_PER_DIRECTION as u32
        + u32::from(ROBUSTNESS_MAX_RETRIES)
        + ROBUSTNESS_TAIL_GRACE_CYCLES;
    let worst_case_cycles = cycles_per_direction * ROBUSTNESS_DIRECTION_COUNT;
    let per_cycle_envelope_ms =
        (ROBUSTNESS_MEASUREMENT_BUDGET_US / MICROS_PER_MILLI) * ROBUSTNESS_LEGS_PER_CYCLE;
    let probe_ms = u64::from(worst_case_cycles) * u64::from(per_cycle_envelope_ms);

    let unwinds = u64::from(worst_case_cycles) * 2;
    let unwind_slot_ms = (u64::from(recovery_maximum_slot_us) + 2 * u64::from(ZERO_SETTLE_US))
        .div_ceil(u64::from(MICROS_PER_MILLI));

    u32::try_from(probe_ms + unwinds * unwind_slot_ms).unwrap_or(u32::MAX)
}

pub(crate) fn robustness_nominal_duration_ms() -> u32 {
    let nominal_cycles = ROBUSTNESS_CYCLES_PER_DIRECTION as u32 * ROBUSTNESS_DIRECTION_COUNT;
    let per_cycle_envelope_ms =
        (ROBUSTNESS_MEASUREMENT_BUDGET_US / MICROS_PER_MILLI) * (ROBUSTNESS_LEGS_PER_CYCLE - 1);
    nominal_cycles * per_cycle_envelope_ms
}

fn zero_robustness_evidence() -> RobustnessEvidence {
    let direction = RobustnessDirectionSummary {
        median_reconvergence_ms: 0,
        max_reconvergence_ms: 0,
        median_forward_settle_ms: 0,
        overshoot_peak_counts: 0,
        valid_cycles: 0,
        trip_count: 0,
        retry_count: 0,
        inconclusive: false,
    };
    RobustnessEvidence {
        directions: [direction, direction],
        residual_medians_q: [0, 0],
        iae_medians_qs: [0, 0],
        dac_rms_medians_q: [0, 0],
        cycles: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
        tail_counts: [0, 0],
        displaced_tails: [None, None],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        FixedGainAmplitudeProgress, IntegralPlanIdentity,
    };

    fn sample_acceptance_point() -> AcceptancePoint {
        AcceptancePoint {
            identity: IntegralPlanIdentity {
                channel: 0,
                build_features: 0x1234,
                schema_revision: 1,
                plan_digest: 0xfeed_beef,
            },
            selected_p: 1_024,
            selected_i: 512,
            target_velocity_rpm: 176,
            selected_response_intervals_q: [[10, 20], [-20, -10]],
            confidence_q: 0,
            band_position_q: 0,
            qualifier_bits: 0,
            token: None,
            progress: FixedGainAmplitudeProgress::ReadyAscending,
        }
    }

    fn sample_terminal(point: AcceptancePoint) -> RobustnessReversalTerminal {
        RobustnessReversalTerminal {
            schema_revision: ROBUSTNESS_SCHEMA_REVISION,
            run_sequence: 1,
            outcome: RobustnessOutcome::Complete,
            cause: ROBUSTNESS_CAUSE_PASS,
            selected_p: point.selected_p,
            selected_i: point.selected_i,
            target_velocity_rpm: point.target_velocity_rpm,
            plant_rate_q: 60_000,
            iae_max_q: 120_000_000_000,
            evidence: zero_robustness_evidence(),
            run_started_us: 10,
            run_completed_us: 20,
            safety_fault: None,
        }
    }

    #[test]
    fn preflight_refuses_when_no_accepted_point() {
        let slot = IntegralRetentionSlot::new();
        assert_eq!(
            preflight_robustness_reversal(1, &slot),
            Err(RobustnessPreflightRejection::NoAcceptedCandidate)
        );
        let t = failed_robustness_terminal(1, RobustnessPreflightRejection::NoAcceptedCandidate);
        assert_eq!(t.outcome, RobustnessOutcome::Failed);
        assert_eq!(t.cause, ROBUSTNESS_CAUSE_NO_ACCEPTED_CANDIDATE);
    }

    #[test]
    fn preflight_reads_point_without_mutating_progress() {
        let mut slot = IntegralRetentionSlot::new();
        slot.install_acceptance_point(sample_acceptance_point());
        let before = slot.acceptance_point().unwrap().progress;
        let pre = preflight_robustness_reversal(1, &slot).expect("has point");
        assert_eq!(pre.point.selected_p, sample_acceptance_point().selected_p);
        assert_eq!(slot.acceptance_point().unwrap().progress, before);
    }

    #[test]
    fn apply_terminal_passes_through_on_identity_match() {
        let point = sample_acceptance_point();
        let mut slot = IntegralRetentionSlot::new();
        slot.install_acceptance_point(point);
        let terminal = sample_terminal(point);

        let applied = apply_robustness_terminal(terminal, &slot);

        assert_eq!(applied.outcome, RobustnessOutcome::Complete);
        assert_eq!(applied.cause, ROBUSTNESS_CAUSE_PASS);
    }

    #[test]
    fn apply_terminal_flags_evidence_integrity_on_identity_mismatch() {
        let point = sample_acceptance_point();
        let mut slot = IntegralRetentionSlot::new();
        slot.install_acceptance_point(point);
        let mut terminal = sample_terminal(point);
        terminal.selected_p = point.selected_p.wrapping_add(1);

        let applied = apply_robustness_terminal(terminal, &slot);

        assert_eq!(applied.outcome, RobustnessOutcome::Failed);
        assert_eq!(applied.cause, ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY);
    }

    #[test]
    fn apply_terminal_flags_evidence_integrity_on_target_velocity_mismatch() {
        let point = sample_acceptance_point();
        let mut slot = IntegralRetentionSlot::new();
        slot.install_acceptance_point(point);
        let mut terminal = sample_terminal(point);
        terminal.target_velocity_rpm = point.target_velocity_rpm.wrapping_add(1);

        let applied = apply_robustness_terminal(terminal, &slot);

        assert_eq!(applied.outcome, RobustnessOutcome::Failed);
        assert_eq!(applied.cause, ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY);
    }

    #[test]
    fn apply_terminal_flags_no_accepted_candidate_when_slot_empty() {
        let point = sample_acceptance_point();
        let slot = IntegralRetentionSlot::new();
        let terminal = sample_terminal(point);

        let applied = apply_robustness_terminal(terminal, &slot);

        assert_eq!(applied.outcome, RobustnessOutcome::Failed);
        assert_eq!(applied.cause, ROBUSTNESS_CAUSE_NO_ACCEPTED_CANDIDATE);
    }

    #[test]
    fn worst_case_run_duration_reserves_one_unwind_and_two_rests_per_retry_and_direction() {
        assert_eq!(robustness_worst_case_duration_ms(3_000_000), 216_000);
        assert!(robustness_nominal_duration_ms() <= robustness_worst_case_duration_ms(0));
    }
}
