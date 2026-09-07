//! Pure commissioning trace sampling phase helpers.
//!
//! This module owns only board-agnostic commissioning phase selection. It does
//! not own USB queues, timestamps, source identity, SPI reads, trace counters,
//! or sample budget policy.

use crate::autotune::CommissionPhase;
use crate::commissioning::types::EngineKind;

/// Return whether a commissioning phase should emit closed-loop entry samples.
pub const fn commission_trace_phase_samples_closed_loop_activation(
    engine_kind: EngineKind,
    phase_id: CommissionPhase,
) -> bool {
    matches!(engine_kind, EngineKind::Commission)
        && matches!(phase_id, CommissionPhase::ClosedLoopActivation)
}

#[cfg(test)]
mod tests {
    use super::commission_trace_phase_samples_closed_loop_activation;
    use crate::autotune::CommissionPhase;
    use crate::commissioning::types::EngineKind;

    #[test]
    fn closed_loop_activation_samples_only_stage1_commission() {
        assert!(commission_trace_phase_samples_closed_loop_activation(
            EngineKind::Commission,
            CommissionPhase::ClosedLoopActivation,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_activation(
            EngineKind::Calibrate,
            CommissionPhase::ClosedLoopActivation,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_activation(
            EngineKind::Selftest,
            CommissionPhase::ClosedLoopActivation,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_activation(
            EngineKind::OuterTune,
            CommissionPhase::ClosedLoopActivation,
        ));
        assert!(!commission_trace_phase_samples_closed_loop_activation(
            EngineKind::Commission,
            CommissionPhase::ElectricalIdentification,
        ));
    }
}
