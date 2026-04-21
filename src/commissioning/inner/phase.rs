//! Phase 3: Phase wiring validation.
//!
//! Reads the coil-check observations and confirms that the current on the
//! expected ADC channel dominates the cross-talk on the other channel for
//! both coils. A reversed dominance indicates swapped motor phases.

use crate::autotune::{CommissionError, InnerSequence, PhaseId};
use crate::commission::{CoilObservation, EncoderCheckState, InnerPhase};
use crate::commissioning::types::StepResult;

/// Validate phase wiring from coil test observations.
///
/// Current on the expected ADC channel must exceed current on
/// the other channel. If reversed, the motor phases are swapped.
pub(crate) fn step_phase_wiring(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    coil_a: CoilObservation,
    coil_b: CoilObservation,
) -> StepResult {
    // Phase wiring check: expected channel must dominate.
    if coil_a.other.unsigned_abs() >= coil_a.expected.unsigned_abs() {
        if matches!(sequence, InnerSequence::Selftest) {
            selftest_results.set(4, crate::selftest::SelftestStageValue::fail(0));
        }
        return super::fail_inner(phase, CommissionError::PhaseFault);
    }
    if coil_b.other.unsigned_abs() >= coil_b.expected.unsigned_abs() {
        if matches!(sequence, InnerSequence::Selftest) {
            selftest_results.set(4, crate::selftest::SelftestStageValue::fail(0));
        }
        return super::fail_inner(phase, CommissionError::PhaseFault);
    }

    if matches!(sequence, InnerSequence::Selftest) {
        selftest_results.set(
            4,
            crate::selftest::SelftestStageValue {
                status: 0,
                value: 0,
            },
        );
    }

    // Transition to encoder check.
    *phase = InnerPhase::EncoderCheck {
        state: EncoderCheckState::Setup,
    };
    StepResult::PhaseChanged(PhaseId::EncoderCheck)
}
