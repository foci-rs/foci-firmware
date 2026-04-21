//! Phase 1: ADC offset calibration.
//!
//! Configures TMC4671 for zero-current ADC sampling (PWM_CHOP=0), collects
//! 128 samples one per `step()` call, validates the resulting offsets, and
//! programs them back into `ADC_I0/I1_SCALE_OFFSET`.

use crate::autotune::{CommissionError, InnerSequence, PhaseId};
use crate::commission::{
    AdcCalState, CoilCheckState, CoilTestState, EncoderCheckState, InnerPhase,
};
use crate::commissioning::types::{AdcCalResult, CommissioningBackend, StepResult};

/// Number of ADC samples for offset calibration.
const ADC_CAL_SAMPLES: u16 = 128;

/// Settling time after enabling ADC outputs (microseconds).
const ADC_CAL_SETTLE_US: u64 = 250_000;

/// Valid ADC offset range (centered around ~33000).
const ADC_OFFSET_MIN: u16 = 30_000;
const ADC_OFFSET_MAX: u16 = 36_000;

/// Advance ADC calibration sub-state.
///
/// Ported from `selftest_adc_calibrate`: configures TMC4671 for
/// zero-current ADC sampling (PWM_CHOP=0), collects 128 samples
/// one per step() call, validates offsets, programs them back.
pub(crate) async fn step_adc_calibrate<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    selftest_results: &mut crate::selftest::SelftestResults,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let cal_state = match phase {
        InnerPhase::AdcCalibrate(cal_state) => cal_state,
        _ => return StepResult::Continue,
    };

    match cal_state {
        AdcCalState::Setup => {
            // Stop motor, disable outputs, set voltage limit.
            backend.set_enable_pin(false);
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, 0)
                .await
                .is_err()
                || backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, 1024)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Verify voltage limit write.
            match backend.read_register(PIDOUT_UQ_UD_LIMITS).await {
                Ok(1024) => {}
                _ => return super::fail_inner(phase, CommissionError::Spi),
            }

            let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
            let _ = backend.write_register(UQ_UD_EXT, 0).await;

            // PWM config: chopping disabled for calibration.
            let _ = backend.write_register(PWM_POLARITIES, 0).await;
            let _ = backend
                .write_register(PWM_MAXCNT, backend.pwm_maxcnt() as u32)
                .await;
            let _ = backend
                .write_register(PWM_BBM_H_BBM_L, backend.bbm_register())
                .await;
            if backend.write_register(PWM_SV_CHOP, 0).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Verify MODE=0 and PWM_SV_CHOP=0.
            let mode_ok = backend
                .read_register(MODE_RAMP_MODE_MOTION)
                .await
                .is_ok_and(|v| v == 0);
            let chop_ok = backend
                .read_register(PWM_SV_CHOP)
                .await
                .is_ok_and(|v| v == 0);
            if !mode_ok || !chop_ok {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // ADC configuration.
            let _ = backend
                .write_register(DSADC_MDEC_B_MDEC_A, backend.dsadc_mdec_register())
                .await;
            let _ = backend
                .write_register(ADC_I_SELECT, (1 << 28) | (1 << 8))
                .await;

            // Motor type: 2-phase stepper, 50 pole pairs.
            let _ = backend
                .write_register(MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50)
                .await;

            // Clear offsets, enable outputs.
            let _ = backend.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
            let _ = backend.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;
            backend.set_enable_pin(true);

            let deadline = backend.elapsed_us().wrapping_add(ADC_CAL_SETTLE_US);
            *cal_state = AdcCalState::Settle {
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        AdcCalState::Settle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                // Deadline passed (unsigned comparison).
                *cal_state = AdcCalState::Sample {
                    sum_i0: 0,
                    sum_i1: 0,
                    count: 0,
                };
            }
            StepResult::Continue
        }

        AdcCalState::Sample {
            sum_i0,
            sum_i1,
            count,
        } => {
            // One sample per step() call (~1ms between samples).
            match backend.read_register(ADC_IWY_IUX).await {
                Ok(raw) => {
                    let i_ux = (raw & 0xFFFF) as i16 as i32;
                    let i_wy = ((raw >> 16) & 0xFFFF) as i16 as i32;
                    *sum_i0 += i_ux;
                    *sum_i1 += i_wy;
                    *count += 1;
                }
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }
            if *count >= ADC_CAL_SAMPLES {
                *cal_state = AdcCalState::Finalize {
                    sum_i0: *sum_i0,
                    sum_i1: *sum_i1,
                };
            }
            StepResult::Continue
        }

        AdcCalState::Finalize { sum_i0, sum_i1 } => {
            let n = ADC_CAL_SAMPLES as i32;
            let offset_i0 = (*sum_i0 / n) as i16 as u16;
            let offset_i1 = (*sum_i1 / n) as i16 as u16;

            // Validate offsets in expected range.
            if !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i0)
                || !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i1)
            {
                if matches!(sequence, InnerSequence::Selftest) {
                    selftest_results.set(1, crate::selftest::SelftestStageValue::fail(0));
                }
                return super::fail_inner(phase, CommissionError::AdcFault);
            }

            if matches!(sequence, InnerSequence::Selftest) {
                selftest_results.set(
                    1,
                    crate::selftest::SelftestStageValue::adc_offsets(offset_i0, offset_i1),
                );
            }

            // Program calibrated offsets.
            let _ = backend
                .write_register(ADC_I0_SCALE_OFFSET, (256 << 16) | offset_i0 as u32)
                .await;
            let _ = backend
                .write_register(ADC_I1_SCALE_OFFSET, (256 << 16) | offset_i1 as u32)
                .await;

            // Re-enable PWM chopping.
            let _ = backend.write_register(PWM_SV_CHOP, 7).await;

            // Capture encoder count for the final calibration result.
            let encoder_count = backend.read_encoder_count().await;
            let result = AdcCalResult {
                adc_i0: offset_i0,
                adc_i1: offset_i1,
                encoder_count,
            };
            *adc_cal = Some(result.clone());

            // Route based on sequence.
            match sequence {
                InnerSequence::Calibrate => {
                    // Skip coil/wiring diagnostics, but still run the
                    // encoder direction sweep used by commissioning so
                    // the rotor is moved before D-axis alignment.
                    *phase = InnerPhase::EncoderCheck {
                        state: EncoderCheckState::Setup,
                    };
                    StepResult::PhaseChanged(PhaseId::EncoderCheck)
                }
                InnerSequence::Selftest | InnerSequence::Commission => {
                    // Run full diagnostics.
                    *phase = InnerPhase::CoilCheck {
                        state: CoilCheckState::CoilA(CoilTestState::Setup { phase: 0 }),
                        coil_a: None,
                    };
                    StepResult::PhaseChanged(PhaseId::CoilCheck)
                }
            }
        }
    }
}
