//! Pure commissioning trace sampling policy helpers.
//!
//! This module owns only board-agnostic decisions. It does not own USB queues,
//! timestamps, source identity, SPI reads, or trace counters.

use crate::autotune::PhaseId;
use crate::commissioning::types::EngineKind;

/// Minimum remaining commissioning tick budget required before USB trace SPI
/// sampling may start.
pub const COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US: u64 = 250;

/// Board-local action selected by the shared commissioning trace policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionTraceSampleDecision {
    /// Trace is not configured; skip silently and do not count a drop.
    SkipUnconfigured,
    /// Trace is configured but the current tick has too little budget left.
    DropForBudget,
    /// Trace is configured and budget remains for SPI sampling.
    Sample,
}

/// Return whether a commissioning phase should emit closed-loop entry samples.
pub const fn commission_trace_phase_samples_closed_loop_entry(
    engine_kind: EngineKind,
    phase_id: PhaseId,
) -> bool {
    matches!(engine_kind, EngineKind::Commission) && matches!(phase_id, PhaseId::ClosedLoopEntry)
}

/// Decide whether a board wrapper should sample, drop, or skip.
pub const fn commission_trace_sample_decision(
    trace_configured: bool,
    past_deadline: bool,
    remaining_budget_us: u64,
) -> CommissionTraceSampleDecision {
    if !trace_configured {
        return CommissionTraceSampleDecision::SkipUnconfigured;
    }
    if past_deadline || remaining_budget_us < COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US {
        return CommissionTraceSampleDecision::DropForBudget;
    }
    CommissionTraceSampleDecision::Sample
}

#[cfg(test)]
mod tests {
    use super::{
        CommissionTraceSampleDecision, commission_trace_phase_samples_closed_loop_entry,
        commission_trace_sample_decision,
    };
    use crate::autotune::PhaseId;
    use crate::commissioning::types::EngineKind;

    #[test]
    fn closed_loop_entry_samples_only_stage1_commission() {
        assert!(commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Commission,
            PhaseId::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Calibrate,
            PhaseId::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Selftest,
            PhaseId::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::OuterTune,
            PhaseId::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Commission,
            PhaseId::ElectricalId,
        ));
    }

    #[test]
    fn sample_decision_preserves_openffboard_budget_policy() {
        assert_eq!(
            commission_trace_sample_decision(false, false, 1_000),
            CommissionTraceSampleDecision::SkipUnconfigured,
        );
        assert_eq!(
            commission_trace_sample_decision(true, true, 1_000),
            CommissionTraceSampleDecision::DropForBudget,
        );
        assert_eq!(
            commission_trace_sample_decision(true, false, 249),
            CommissionTraceSampleDecision::DropForBudget,
        );
        assert_eq!(
            commission_trace_sample_decision(true, false, 250),
            CommissionTraceSampleDecision::Sample,
        );
        assert_eq!(
            commission_trace_sample_decision(true, false, 251),
            CommissionTraceSampleDecision::Sample,
        );
    }
}
