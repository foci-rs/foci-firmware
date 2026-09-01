//! Dispatch-side preflight and terminal application for the
//! reversal-standstill robustness gate.
//!
//! Holds the `ROBUSTNESS_CAUSE_*` namespace (consumed by
//! `robustness_evidence::summary_cause`) plus the read-only preflight, the
//! failed-terminal builder, and the terminal-application identity check.
//! Both `preflight_robustness_reversal` and `apply_robustness_terminal`
//! take `&StageCCandidateSlot`, never `&mut`: this phase probes the
//! already-accepted operating point but never advances or closes
//! `AcceptanceMatrixProgress`.

use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptancePoint, StageCCandidateSlot,
};
use crate::commissioning::outer::velocity::robustness_cycles::ROBUSTNESS_CYCLES_PER_DIRECTION;
use crate::commissioning::outer::velocity::robustness_evidence::{
    RobustnessDirectionSummary, RobustnessEvidence,
};
use crate::commissioning::outer::velocity::robustness_metrics::{
    ROBUSTNESS_LEGS_PER_CYCLE, ROBUSTNESS_MAX_RETRIES, ROBUSTNESS_MEASUREMENT_BUDGET_US,
};
use crate::commissioning::outer::velocity::robustness_reporting::ROBUSTNESS_SCHEMA_REVISION;
use crate::commissioning::types::{RobustnessOutcome, RobustnessReversalTerminal};

/// Reversal directions probed per run: forward->reverse and
/// reverse->forward. Mirrors `robustness_reversal::DIRECTION_COUNT`,
/// duplicated here so this module's worst-case arithmetic does not need a
/// cross-module reach for a single scalar.
const ROBUSTNESS_DIRECTION_COUNT: u32 = 2;

/// Bounded per-direction tail-grace re-run budget: the cycle ledger grants
/// exactly one free re-run for the first conclusive cycle whose
/// reconvergence time crosses the tail threshold; a second tail cycle
/// latches the rejection instead of retrying.
const ROBUSTNESS_TAIL_GRACE_CYCLES: u32 = 1;

/// Microseconds per millisecond, for the envelope's unit conversion.
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
/// Envelope/velocity-limit safety fault passthrough only. Carries a populated
/// `safety_fault` field; internal (non-envelope) faults use
/// `ROBUSTNESS_CAUSE_INTERNAL_FAULT` so the host can tell them apart.
pub const ROBUSTNESS_CAUSE_SAFETY_FAULT: u8 = 6;
/// Accepted-point identity mismatch, set at terminal application (Task 6).
/// Never returned by `summary_cause`.
pub const ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY: u8 = 7;
/// Internal (non-envelope) fault: SPI/encoder I/O, analysis deadline, invalid
/// gains, or ring overflow. Distinct from `ROBUSTNESS_CAUSE_SAFETY_FAULT`,
/// which is reserved for a real envelope/velocity-limit trip carrying an
/// `OuterSafetyFault`. Set at the fault terminal, never by `summary_cause`.
pub const ROBUSTNESS_CAUSE_INTERNAL_FAULT: u8 = 8;
/// The between-legs standstill was never confirmed, even after the one bounded
/// re-opened rest window. An operator-actionable subset of the internal-fault
/// space (outcome stays `Failed`); every other internal fault -- SPI/encoder
/// I/O, analysis deadline, invalid gains, ring overflow -- stays on
/// `ROBUSTNESS_CAUSE_INTERNAL_FAULT`. Set at the fault terminal, never by
/// `summary_cause`.
pub const ROBUSTNESS_CAUSE_REST_NOT_CONFIRMED: u8 = 9;
/// Two conclusive tail cycles occurred in one reversal direction, latching a
/// tail rejection. Distinct from `ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED`
/// (a slow per-direction median) so the host can tell a slow center from a
/// heavy tail. Set at terminal assembly; never returned by `summary_cause`.
pub const ROBUSTNESS_CAUSE_TAIL_REPEATED: u8 = 10;

/// Definitive rejection of a robustness-reversal preflight.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RobustnessPreflightRejection {
    /// No candidate survived acceptance to be probed for robustness.
    NoAcceptedCandidate,
}

/// Immutable robustness-reversal plan accepted before any probe motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RobustnessPreflight {
    /// The retained accepted operating point read for this probe.
    pub point: AcceptancePoint,
    /// Caller-assigned trace sequence.
    pub run_sequence: u32,
}

/// Validate a retained accepted point without mutating retention.
///
/// Reads [`StageCCandidateSlot::acceptance_point`] only; never calls
/// `advance_matrix_progress`/`close_matrix_progress`, so a failed or
/// completed reversal probe cannot itself move the acceptance-matrix
/// lifecycle forward.
pub fn preflight_robustness_reversal(
    run_sequence: u32,
    stage_c_candidate: &StageCCandidateSlot,
) -> Result<RobustnessPreflight, RobustnessPreflightRejection> {
    let point = stage_c_candidate
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
        // No accepted point exists on this rejection path, so no plant
        // response was ever identified to construct a threshold from.
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
/// Takes `&StageCCandidateSlot` (shared, read-only): this phase never
/// advances or closes `AcceptanceMatrixProgress`, so it cannot take a
/// mutable slot reference. On identity mismatch (or an emptied slot) the
/// terminal is stamped `Failed`/`EVIDENCE_INTEGRITY` -- distinct from
/// `SAFETY_FAULT`, which is reserved for envelope faults.
pub fn apply_robustness_terminal(
    mut terminal: RobustnessReversalTerminal,
    stage_c_candidate: &StageCCandidateSlot,
) -> RobustnessReversalTerminal {
    let identity_ok = stage_c_candidate.acceptance_point().is_some_and(|point| {
        point.selected_p == terminal.selected_p
            && point.selected_i == terminal.selected_i
            && point.target_velocity_rpm == terminal.target_velocity_rpm
    });
    if !identity_ok {
        terminal.outcome = RobustnessOutcome::Failed;
        terminal.cause = ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY;
    }
    terminal
}

/// Worst-case wall-clock duration of a full reversal-robustness run, in
/// milliseconds.
///
/// Bounds the run at 16 total probe cycles: per direction, the
/// `ROBUSTNESS_CYCLES_PER_DIRECTION` measurement slots plus
/// `ROBUSTNESS_MAX_RETRIES` bounded inconclusive retries plus the single
/// tail-grace re-run, doubled for both reversal directions. Each cycle
/// budgets `ROBUSTNESS_LEGS_PER_CYCLE` legs (establish, reversal, rest, and
/// the one re-opened rest a rejected rest is granted) at
/// `ROBUSTNESS_MEASUREMENT_BUDGET_US` apiece -- the same budget
/// `robustness_reversal::open_zero_window` uses for each rest window, so all
/// four legs share one constant. This is a conservative envelope: a cycle
/// that reaches band entry early finishes a leg well under its full budget.
/// The result is asserted inside the u16 millisecond field the host's
/// duration-budget wire format supplies.
pub(crate) fn robustness_worst_case_duration_ms() -> u32 {
    let cycles_per_direction = ROBUSTNESS_CYCLES_PER_DIRECTION as u32
        + u32::from(ROBUSTNESS_MAX_RETRIES)
        + ROBUSTNESS_TAIL_GRACE_CYCLES;
    let worst_case_cycles = cycles_per_direction * ROBUSTNESS_DIRECTION_COUNT;

    let per_cycle_envelope_ms =
        (ROBUSTNESS_MEASUREMENT_BUDGET_US / MICROS_PER_MILLI) * ROBUSTNESS_LEGS_PER_CYCLE;

    let worst_case_duration_ms = worst_case_cycles * per_cycle_envelope_ms;
    assert!(
        worst_case_duration_ms <= u32::from(u16::MAX),
        "robustness worst-case duration must fit the host's u16 duration-budget field"
    );
    worst_case_duration_ms
}

/// Clean-run duration estimate of a full reversal-robustness run, in
/// milliseconds: the host's `nominal_workflow_ms`.
///
/// Every measurement slot fills on its first attempt, so no inconclusive
/// retries and no tail grace, and each cycle rests once with no re-opened rest
/// window (`ROBUSTNESS_LEGS_PER_CYCLE - 1` legs). This is only a disclosure
/// hint; the host arms its run timeout off `robustness_worst_case_duration_ms`.
pub(crate) fn robustness_nominal_duration_ms() -> u32 {
    let nominal_cycles = ROBUSTNESS_CYCLES_PER_DIRECTION as u32 * ROBUSTNESS_DIRECTION_COUNT;
    let per_cycle_envelope_ms =
        (ROBUSTNESS_MEASUREMENT_BUDGET_US / MICROS_PER_MILLI) * (ROBUSTNESS_LEGS_PER_CYCLE - 1);
    nominal_cycles * per_cycle_envelope_ms
}

/// All-zero robustness evidence for a terminal built before any probe motion.
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
        cycles: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
        tail_counts: [0, 0],
        displaced_tails: [None, None],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptanceMatrixProgress, StageCPlanIdentity,
    };

    fn sample_acceptance_point() -> AcceptancePoint {
        AcceptancePoint {
            identity: StageCPlanIdentity {
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
            progress: AcceptanceMatrixProgress::ReadyAscending,
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
        let slot = StageCCandidateSlot::new(); // Empty
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
        let mut slot = StageCCandidateSlot::new();
        slot.install_acceptance_point(sample_acceptance_point()); // helper; progress ReadyAscending
        let before = slot.acceptance_point().unwrap().progress;
        let pre = preflight_robustness_reversal(1, &slot).expect("has point");
        assert_eq!(pre.point.selected_p, sample_acceptance_point().selected_p);
        assert_eq!(slot.acceptance_point().unwrap().progress, before); // untouched
    }

    #[test]
    fn apply_terminal_passes_through_on_identity_match() {
        let point = sample_acceptance_point();
        let mut slot = StageCCandidateSlot::new();
        slot.install_acceptance_point(point);
        let terminal = sample_terminal(point);

        let applied = apply_robustness_terminal(terminal, &slot);

        assert_eq!(applied.outcome, RobustnessOutcome::Complete);
        assert_eq!(applied.cause, ROBUSTNESS_CAUSE_PASS);
    }

    #[test]
    fn apply_terminal_flags_evidence_integrity_on_identity_mismatch() {
        let point = sample_acceptance_point();
        let mut slot = StageCCandidateSlot::new();
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
        let mut slot = StageCCandidateSlot::new();
        slot.install_acceptance_point(point);
        let mut terminal = sample_terminal(point);
        // Identical gains, different target: a re-installed candidate must not
        // pass the identity check as unchanged.
        terminal.target_velocity_rpm = point.target_velocity_rpm.wrapping_add(1);

        let applied = apply_robustness_terminal(terminal, &slot);

        assert_eq!(applied.outcome, RobustnessOutcome::Failed);
        assert_eq!(applied.cause, ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY);
    }

    #[test]
    fn apply_terminal_flags_evidence_integrity_when_slot_empty() {
        let point = sample_acceptance_point();
        let slot = StageCCandidateSlot::new(); // Empty
        let terminal = sample_terminal(point);

        let applied = apply_robustness_terminal(terminal, &slot);

        assert_eq!(applied.outcome, RobustnessOutcome::Failed);
        assert_eq!(applied.cause, ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY);
    }

    #[test]
    fn worst_case_run_duration_fits_the_u16_budget_field() {
        // Per direction: 5 measurement slots + 4 bounded inconclusive
        // retries + 1 tail-grace re-run = 10 cycles; both directions = 20.
        // Each cycle budgets 4 legs (establish, reversal, rest, and the one
        // re-opened rest) at the 500 ms measurement budget apiece:
        // 20 * 4 * 500 = 40_000 ms.
        let expected_ms: u32 = 40_000;

        let duration_ms = robustness_worst_case_duration_ms();

        assert_eq!(duration_ms, expected_ms);
        assert!(duration_ms <= u32::from(u16::MAX));
    }
}
