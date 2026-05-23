//! Phase 2: Coil connectivity check.
//!
//! Tests coil A then coil B via `UQ_UD_EXT` (MODE=8). Validates current
//! exceeds `MOTOR_CURRENT_THRESHOLD` on the expected ADC channel and
//! carries the observations forward into the phase-wiring check.

use super::{CoilCheckState, CoilObservation, CoilTestState, InnerPhase};
use crate::autotune::{CommissionError, InnerSequence, PhaseId};
use crate::commissioning::types::{CommissioningBackend, StepResult};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics};

const DIAG_COIL_CHECK_A_SAMPLE: u8 = 1;
const DIAG_COIL_CHECK_B_SAMPLE: u8 = 2;

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
    diagnostics: &mut CommissionDiagnostics,
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

                let sample_failed = obs.expected.unsigned_abs() <= MOTOR_CURRENT_THRESHOLD;
                diagnostics.push(CommissionDiagnostic {
                    phase: PhaseId::CoilCheck.to_u8(),
                    code: if p == 0 {
                        DIAG_COIL_CHECK_A_SAMPLE
                    } else {
                        DIAG_COIL_CHECK_B_SAMPLE
                    },
                    status: if sample_failed { 1 } else { 0 },
                    value0: obs.expected as u16 as u32,
                    value1: obs.other as u16 as u32,
                    value2: raw,
                });

                // Validate coil presence.
                if sample_failed {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::{CommissionProfile, InnerSequence};
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::types::StepResult;
    use crate::selftest::CommissionDiagnostics;
    use foci_tmc4671::registers::ADC_IWY_IUX;

    #[test]
    fn coil_fault_records_raw_current_sample() {
        pollster::block_on(async {
            let mut phase = InnerPhase::CoilCheck {
                state: CoilCheckState::CoilA(CoilTestState::Read { phase: 0 }),
                coil_a: None,
            };
            let mut buf = crate::autotune::TransientBuffer::new();
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            let raw = ((-20i16 as u16 as u32) << 16) | 120u16 as u32;
            backend.registers.insert(ADC_IWY_IUX.addr(), raw).unwrap();

            let result = super::super::step_inner(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::CoilFault)
            ));
            let items: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(items.len(), 1);
            assert_eq!(items[0].phase, PhaseId::CoilCheck.to_u8());
            assert_eq!(items[0].status, 1);
            assert_eq!(items[0].value0, 120);
            assert_eq!(items[0].value1, -20i16 as u16 as u32);
            assert_eq!(items[0].value2, raw);
        });
    }
}
