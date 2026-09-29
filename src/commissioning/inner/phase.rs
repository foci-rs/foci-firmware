//! Phase 3: Phase wiring validation.
//! both coils. A reversed dominance indicates swapped motor phases.

use super::{CoilObservation, EncoderCheckState, InnerPhase};
use crate::autotune::{CommissionError, CommissionPhase, InnerSequence};
use crate::commissioning::types::StepResult;

pub(crate) fn step_phase_wiring(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    coil_a: CoilObservation,
    coil_b: CoilObservation,
) -> StepResult {
    if coil_a.other.unsigned_abs() >= coil_a.expected.unsigned_abs() {
        if matches!(sequence, InnerSequence::Selftest) {
            selftest_results.set(
                crate::selftest::SelftestStage::PhaseWiring,
                crate::selftest::SelftestStageValue::fail(0),
            );
        }
        return super::fail_inner(phase, CommissionError::PhaseFault);
    }
    if coil_b.other.unsigned_abs() >= coil_b.expected.unsigned_abs() {
        if matches!(sequence, InnerSequence::Selftest) {
            selftest_results.set(
                crate::selftest::SelftestStage::PhaseWiring,
                crate::selftest::SelftestStageValue::fail(0),
            );
        }
        return super::fail_inner(phase, CommissionError::PhaseFault);
    }

    if matches!(sequence, InnerSequence::Selftest) {
        selftest_results.set(
            crate::selftest::SelftestStage::PhaseWiring,
            crate::selftest::SelftestStageValue {
                status: 0,
                value: 0,
            },
        );
    }

    *phase = InnerPhase::EncoderCheck {
        state: EncoderCheckState::Setup,
    };
    StepResult::PhaseChanged(CommissionPhase::EncoderCheck)
}
