//! Per-direction reversal-robustness evidence for the robustness gate.
//!
//! [`RobustnessDirectionSummary`] is the per-direction wire summary of a K-cycle probe run;
//! [`RobustnessEvidence`] bundles both directions with the off-terminal medians and
//! per-cycle triples.

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
    /// 1 if this direction contributed a rejection (median threshold trip or latched tail),
    /// else 0.
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
    pub residual_medians_q: [i32; 2],
    /// Median recovery IAE per direction, in rate-Q seconds.
    pub iae_medians_qs: [i32; 2],
    /// Median settled dAC_rms per direction, in raw rate-Q units.
    pub dac_rms_medians_q: [i32; 2],
    /// Per-cycle wire triples for each direction, in cycle order; an unfilled slot is `None`.
    pub(crate) cycles: [[Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
    /// Count of conclusive tail cycles per direction.
    pub tail_counts: [u8; 2],
    /// The grace-retried (displaced) tail cycle per direction, if any.
    pub(crate) displaced_tails: [Option<CycleWire>; 2],
}

/// Derive the primary `ROBUSTNESS_CAUSE_*` from both directions tripped masks
/// and inconclusive flags.
///
/// Priority: first tripped threshold cause (time > residual > IAE), then
/// `RECONVERGENCE_INCONCLUSIVE` if either direction is inconclusive, else `PASS`.
/// Never returns `ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY` or the internal-fault causes.
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
        use crate::commission_dispatch::robustness::*;
        use crate::commissioning::outer::velocity::robustness_metrics::{TRIP_IAE, TRIP_TIME};

        assert_eq!(summary_cause(0, 0, false, false), ROBUSTNESS_CAUSE_PASS);

        assert_eq!(
            summary_cause(0, 0, false, true),
            ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE
        );

        assert_eq!(
            summary_cause(0, TRIP_TIME | TRIP_IAE, false, false),
            ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED
        );

        assert_eq!(
            summary_cause(TRIP_TIME | TRIP_IAE, 0, false, true),
            ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED
        );
    }

    #[test]
    fn summary_cause_reports_iae_when_only_iae_trips() {
        use crate::commission_dispatch::robustness::*;
        use crate::commissioning::outer::velocity::robustness_metrics::TRIP_IAE;

        assert_eq!(
            summary_cause(TRIP_IAE, 0, false, false),
            ROBUSTNESS_CAUSE_IAE_EXCEEDED
        );
    }
}
