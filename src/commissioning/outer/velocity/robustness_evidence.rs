//! Per-direction reversal-robustness evidence for the robustness gate.
//!
//! [`RobustnessDirectionSummary`] is the schema-2 wire summary a terminal
//! reports: median/max reconvergence, median forward-settle, an
//! overshoot-peak count, and per-direction valid-cycle/trip/retry counts
//! reduced from a K-cycle probe run. [`RobustnessEvidence`] bundles both
//! directions' summaries with the off-terminal residual/IAE medians and the
//! per-cycle wire triples. [`summary_cause`] derives the summary
//! `ROBUSTNESS_CAUSE_*` code a terminal reply reports from both directions'
//! tripped masks and inconclusive flags.

use super::robustness_cycles::ROBUSTNESS_CYCLES_PER_DIRECTION;
use super::robustness_metrics::{TRIP_IAE, TRIP_RESIDUAL, TRIP_TIME, worse_of};
use super::robustness_reporting::CycleWire;
use crate::commission_dispatch::robustness::{
    ROBUSTNESS_CAUSE_IAE_EXCEEDED, ROBUSTNESS_CAUSE_PASS,
    ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED,
    ROBUSTNESS_CAUSE_RESIDUAL_EXCEEDED,
};

/// One reversal direction's cycle-summary robustness evidence, as reported
/// on the wire.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct RobustnessDirectionSummary {
    /// Median reconvergence time across valid cycles, in milliseconds.
    pub median_reconvergence_ms: u16,
    /// Maximum reconvergence time across valid cycles, in milliseconds.
    pub max_reconvergence_ms: u16,
    /// Median forward-settle time across valid cycles, in milliseconds.
    pub median_forward_settle_ms: u16,
    /// Saturating count of cycles whose settled tail overshot the target
    /// before converging.
    pub overshoot_peak_counts: u16,
    /// Count of cycle slots that filled with conclusive evidence.
    pub valid_cycles: u8,
    /// 1 if this direction contributed a rejection (a median threshold trip or
    /// a latched tail), else 0. Not a per-cycle tally: the median gate keeps no
    /// per-cycle trip count.
    pub trip_count: u8,
    /// Count of bounded inconclusive retries this direction consumed.
    pub retry_count: u8,
    /// True when this direction never reached the minimum valid-cycle floor.
    pub inconclusive: bool,
}

/// Both reversal directions' robustness evidence for one candidate.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct RobustnessEvidence {
    /// Index 0 = forward->reverse, index 1 = reverse->forward.
    pub directions: [RobustnessDirectionSummary; 2],
    /// Median settled residual rate error per direction, in rate-Q units.
    /// Off-terminal: carried for the detailed reply, not the compact wire
    /// terminal.
    pub residual_medians_q: [i32; 2],
    /// Median recovery IAE per direction, in rate-Q seconds. Off-terminal:
    /// carried for the detailed reply, not the compact wire terminal.
    pub iae_medians_qs: [i32; 2],
    /// Per-cycle wire triples for each direction, in cycle order. An unfilled
    /// slot is `None`. Off-terminal: carried by the per-cycle evidence reply,
    /// not encoded into the compact 39-byte terminal. `pub(crate)` because
    /// [`CycleWire`] is a crate-internal wire type.
    pub(crate) cycles: [[Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
}

/// Derive the primary `ROBUSTNESS_CAUSE_*` from both directions' tripped
/// masks and inconclusive flags.
///
/// Trips-first, to agree with the outcome mapping in `complete_terminal`
/// (any trip -> `Rejected`): from the OR of both directions' tripped masks
/// (`worse_of`), the first tripped threshold cause in time > residual > IAE
/// priority; otherwise `RECONVERGENCE_INCONCLUSIVE` if either direction is
/// inconclusive; otherwise `PASS`. A mixed result (one direction tripped,
/// the other inconclusive) therefore reports the trip cause, matching
/// `outcome = Rejected` rather than aliasing the inconclusive path.
///
/// Takes the tripped masks and inconclusive flags directly rather than a
/// [`RobustnessEvidence`] so it stays pure over the terminal's schema-2
/// summary fields, which no longer carry a per-direction tripped bitmask
/// (only the derived [`RobustnessDirectionSummary::trip_count`]).
///
/// Never returns `ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY` or
/// `ROBUSTNESS_CAUSE_INTERNAL_FAULT` -- those causes are set only at
/// terminal application and the fault path, not by this summary derivation.
pub(crate) fn summary_cause(
    tripped_a: u8,
    tripped_b: u8,
    inconclusive_a: bool,
    inconclusive_b: bool,
) -> u8 {
    let combined = worse_of(tripped_a, tripped_b);
    if combined & TRIP_TIME != 0 {
        ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED
    } else if combined & TRIP_RESIDUAL != 0 {
        ROBUSTNESS_CAUSE_RESIDUAL_EXCEEDED
    } else if combined & TRIP_IAE != 0 {
        ROBUSTNESS_CAUSE_IAE_EXCEEDED
    } else if inconclusive_a || inconclusive_b {
        ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE
    } else {
        ROBUSTNESS_CAUSE_PASS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn summary_cause_prefers_trip_priority_then_inconclusive() {
        use crate::commission_dispatch::robustness::*; // ROBUSTNESS_CAUSE_*
        use crate::commissioning::outer::velocity::robustness_metrics::{TRIP_IAE, TRIP_TIME};

        // All clean -> PASS.
        assert_eq!(summary_cause(0, 0, false, false), ROBUSTNESS_CAUSE_PASS);

        // Inconclusive with no trip -> INCONCLUSIVE.
        assert_eq!(
            summary_cause(0, 0, false, true),
            ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE
        );

        // A trip outranks IAE within the same direction (time > IAE).
        assert_eq!(
            summary_cause(0, TRIP_TIME | TRIP_IAE, false, false),
            ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED
        );

        // Mixed: one direction tripped, the other inconclusive -> the TRIP
        // cause, matching outcome = Rejected (trips dominate the outcome).
        assert_eq!(
            summary_cause(TRIP_TIME | TRIP_IAE, 0, false, true),
            ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED
        );
    }
}
