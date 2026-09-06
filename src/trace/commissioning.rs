//! Pure commissioning trace sampling phase helpers.
//!
//! This module owns only board-agnostic commissioning phase selection. It does
//! not own USB queues, timestamps, source identity, SPI reads, trace counters,
//! or sample budget policy.

use crate::autotune::CommissionPhase;
use crate::commissioning::types::EngineKind;

/// Return whether a commissioning phase should emit closed-loop entry samples.
pub const fn commission_trace_phase_samples_closed_loop_entry(
    engine_kind: EngineKind,
    phase_id: CommissionPhase,
) -> bool {
    matches!(engine_kind, EngineKind::Commission)
        && matches!(phase_id, CommissionPhase::ClosedLoopEntry)
}

#[cfg(test)]
mod tests {
    use super::commission_trace_phase_samples_closed_loop_entry;
    use crate::autotune::CommissionPhase;
    use crate::commissioning::types::EngineKind;

    #[test]
    fn closed_loop_entry_samples_only_stage1_commission() {
        assert!(commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Commission,
            CommissionPhase::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Calibrate,
            CommissionPhase::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Selftest,
            CommissionPhase::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::OuterTune,
            CommissionPhase::ClosedLoopEntry,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_entry(
            EngineKind::Commission,
            CommissionPhase::ElectricalId,
        ));
    }
}
