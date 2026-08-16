//! Per-direction reversal-robustness evidence for the robustness gate.
//!
//! Pairs one [`super::robustness_metrics::ReversalMetrics`] measurement with
//! its tripped-threshold bitmask, retry count, and inconclusive flag for each
//! reversal direction, and derives the summary `ROBUSTNESS_CAUSE_*` code that
//! a terminal reply reports.

use super::robustness_metrics::{ReversalMetrics, TRIP_IAE, TRIP_RESIDUAL, TRIP_TIME, worse_of};
use crate::commission_dispatch::robustness::{
    ROBUSTNESS_CAUSE_IAE_EXCEEDED, ROBUSTNESS_CAUSE_PASS,
    ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE, ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED,
    ROBUSTNESS_CAUSE_RESIDUAL_EXCEEDED,
};

/// One reversal direction's robustness probe outcome.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
#[allow(dead_code)]
pub struct RobustnessDirectionEvidence {
    /// The measured reconvergence/residual/IAE metrics for this direction.
    pub metrics: ReversalMetrics,
    /// `TRIP_TIME | TRIP_RESIDUAL | TRIP_IAE` bitmask of threshold gates
    /// this direction failed, independently of one another.
    pub tripped: u8,
    /// Number of bounded retries consumed by this direction's probe.
    pub retry_count: u8,
    /// True when the probe never reached a sustained band-entry dwell
    /// within its measurement budget, even after retries.
    pub inconclusive: bool,
}

/// Both reversal directions' robustness evidence for one candidate.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
#[allow(dead_code)]
pub struct RobustnessEvidence {
    /// Index 0 = forward->reverse, index 1 = reverse->forward.
    pub directions: [RobustnessDirectionEvidence; 2],
}

/// Derive the primary `ROBUSTNESS_CAUSE_*` for `ev`.
///
/// Priority: `RECONVERGENCE_INCONCLUSIVE` if either direction is
/// inconclusive; otherwise the first tripped threshold cause, in
/// time > residual > IAE priority, from the OR of both directions' tripped
/// masks (`worse_of`); otherwise `PASS`.
///
/// Never returns `ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY` -- that cause is set
/// only at terminal application (Task 6), not by this summary derivation.
#[allow(dead_code)]
pub(crate) fn summary_cause(ev: &RobustnessEvidence) -> u8 {
    if ev.directions.iter().any(|direction| direction.inconclusive) {
        return ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE;
    }
    let combined = worse_of(ev.directions[0].tripped, ev.directions[1].tripped);
    if combined & TRIP_TIME != 0 {
        ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED
    } else if combined & TRIP_RESIDUAL != 0 {
        ROBUSTNESS_CAUSE_RESIDUAL_EXCEEDED
    } else if combined & TRIP_IAE != 0 {
        ROBUSTNESS_CAUSE_IAE_EXCEEDED
    } else {
        ROBUSTNESS_CAUSE_PASS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const METRICS_OK: ReversalMetrics = ReversalMetrics {
        reconvergence_time_us: 50_000,
        settled_residual_q: 100,
        recovery_iae_q: 500_000,
        forward_settle_time_us: 20_000,
        reconvergence_ratio_ppm: 0,
    };

    #[test]
    fn summary_cause_prefers_inconclusive_then_trip_priority() {
        use crate::commission_dispatch::robustness::*; // ROBUSTNESS_CAUSE_*
        use crate::commissioning::outer::velocity::robustness_metrics::{TRIP_IAE, TRIP_TIME};
        let clean = RobustnessDirectionEvidence {
            metrics: METRICS_OK,
            tripped: 0,
            retry_count: 0,
            inconclusive: false,
        };
        assert_eq!(
            summary_cause(&RobustnessEvidence {
                directions: [clean, clean]
            }),
            ROBUSTNESS_CAUSE_PASS
        );
        let inconclusive = RobustnessDirectionEvidence {
            inconclusive: true,
            ..clean
        };
        assert_eq!(
            summary_cause(&RobustnessEvidence {
                directions: [clean, inconclusive]
            }),
            ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE
        );
        let time_tripped = RobustnessDirectionEvidence {
            tripped: TRIP_TIME | TRIP_IAE,
            ..clean
        };
        assert_eq!(
            summary_cause(&RobustnessEvidence {
                directions: [clean, time_tripped]
            }),
            ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED
        );
    }
}
