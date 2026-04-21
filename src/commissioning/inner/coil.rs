//! Phase 2: Coil connectivity check.
//!
//! Tests coil A then coil B via `UQ_UD_EXT` (MODE=8). Validates current
//! exceeds `MOTOR_CURRENT_THRESHOLD` on the expected ADC channel and
//! carries the observations forward into the phase-wiring check.

use crate::autotune::{CommissionError, InnerSequence, PhaseId};
use crate::commission::{CoilCheckState, CoilObservation, CoilTestState, InnerPhase};
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Minimum ADC current to consider a coil present (raw ADC units).
const MOTOR_CURRENT_THRESHOLD: u16 = 200;

/// Settling time after applying coil test voltage (microseconds).
const COIL_SETTLE_US: u64 = 100_000;

/// Settling time after disabling coil test (microseconds).
const COIL_COOLDOWN_US: u64 = 50_000;

/// Advance coil check sub-state.
///
/// Tests coil A then coil B via `UQ_UD_EXT` (MODE=8). Validates
/// current exceeds `MOTOR_CURRENT_THRESHOLD` on the expected
/// ADC channel. Carries observations forward for phase wiring
/// check.
pub(crate) async fn step_coil_check<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    // Extract mutable fields from phase.
    let (state, coil_a_obs) = match phase {
        InnerPhase::CoilCheck { state, coil_a } => (state, coil_a),
        _ => return StepResult::Continue,
    };

    let is_coil_b = matches!(state, CoilCheckState::CoilB(_));

    match state {
        CoilCheckState::CoilA(coil_test) | CoilCheckState::CoilB(coil_test) => match coil_test {
            CoilTestState::Setup { phase: coil_phase } => {
                let p = *coil_phase;
                // Set PHI_E source to external.
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                // Angle: 0 for coil A (D-axis), 16384 for coil B (Q-axis).
                let phi_e = if p == 0 { 0u32 } else { 16384 };
                if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                // Apply voltage.
                let voltage = backend.excitation_voltage() as u32;
                if backend
                    .write_register(UQ_UD_EXT, voltage & 0xFFFF)
                    .await
                    .is_err()
                {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 8)
                    .await
                    .is_err()
                {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                let deadline = backend.elapsed_us().wrapping_add(COIL_SETTLE_US);
                *coil_test = CoilTestState::Settle {
                    phase: p,
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            CoilTestState::Settle {
                phase: coil_phase,
                deadline_us,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    *coil_test = CoilTestState::Read { phase: *coil_phase };
                }
                StepResult::Continue
            }

            CoilTestState::Read { phase: coil_phase } => {
                let p = *coil_phase;
                let raw = match backend.read_register(ADC_IWY_IUX).await {
                    Ok(v) => v,
                    Err(_) => {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                };

                // Stop motor and disable.
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                let _ = backend.write_register(UQ_UD_EXT, 0).await;
                backend.set_enable_pin(false);

                let i_ux = (raw & 0xFFFF) as i16;
                let i_wy = ((raw >> 16) & 0xFFFF) as i16;

                let obs = if p == 0 {
                    CoilObservation {
                        expected: i_ux,
                        other: i_wy,
                    }
                } else {
                    CoilObservation {
                        expected: i_wy,
                        other: i_ux,
                    }
                };

                // Validate coil presence.
                if obs.expected.unsigned_abs() <= MOTOR_CURRENT_THRESHOLD {
                    if matches!(sequence, InnerSequence::Selftest) {
                        let stage = if is_coil_b { 3 } else { 2 };
                        selftest_results.set(stage, crate::selftest::SelftestStageValue::fail(0));
                    }
                    return super::fail_inner(phase, CommissionError::CoilFault);
                }

                let deadline = backend.elapsed_us().wrapping_add(COIL_COOLDOWN_US);
                *coil_test = CoilTestState::Cooldown {
                    phase: p,
                    deadline_us: deadline,
                };

                // Store observation.
                if !is_coil_b {
                    if matches!(sequence, InnerSequence::Selftest) {
                        selftest_results.set(
                            2,
                            crate::selftest::SelftestStageValue::coil_current(obs.expected),
                        );
                    }
                    *coil_a_obs = Some(obs);
                } else {
                    if matches!(sequence, InnerSequence::Selftest) {
                        selftest_results.set(
                            3,
                            crate::selftest::SelftestStageValue::coil_current(obs.expected),
                        );
                    }
                    // Both coils tested. Transition to PhaseWiringCheck.
                    let a = coil_a_obs.unwrap_or(CoilObservation {
                        expected: 0,
                        other: 0,
                    });
                    *phase = InnerPhase::PhaseWiringCheck {
                        coil_a: a,
                        coil_b: obs,
                    };
                    return StepResult::PhaseChanged(PhaseId::PhaseWiringCheck);
                }
                StepResult::Continue
            }

            CoilTestState::Cooldown {
                phase: _coil_phase,
                deadline_us,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    // Coil A done, move to coil B.
                    *state = CoilCheckState::CoilB(CoilTestState::Setup { phase: 1 });
                }
                StepResult::Continue
            }
        },
    }
}
