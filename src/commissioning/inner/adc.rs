//! Phase 1: ADC offset calibration.
//!
//! Configures TMC4671 for zero-current ADC sampling (PWM_CHOP=0), collects
//! 128 samples one per `step()` call, validates the resulting offsets, and
//! programs them back into `ADC_I0/I1_SCALE_OFFSET`.

use super::{
    AdcCalState, AdcResidualStage, CoilCheckState, CoilTestState, EncoderCheckState, InnerPhase,
};
use crate::autotune::{CommissionError, InnerSequence, PhaseId};
use crate::commissioning::types::{
    ADC_RESIDUAL_STAGE_CHOP_OFF, ADC_RESIDUAL_STAGE_CHOP_ON, AdcCalResult, AdcResidualEvidence,
    CommissioningBackend, StepResult,
};

/// Number of ADC samples for offset calibration.
const ADC_CALIBRATION_SAMPLES: u16 = 128;

/// Settling time after enabling ADC outputs (microseconds).
const ADC_CALIBRATION_SETTLE_US: u64 = 250_000;

/// Valid ADC offset range (centered around ~33000).
const ADC_OFFSET_MIN: u16 = 30_000;
const ADC_OFFSET_MAX: u16 = 36_000;

fn residual_stage_code(stage: AdcResidualStage) -> u8 {
    match stage {
        AdcResidualStage::ChopOff => ADC_RESIDUAL_STAGE_CHOP_OFF,
        AdcResidualStage::ChopOn => ADC_RESIDUAL_STAGE_CHOP_ON,
    }
}

fn residual_stage_chop_value(stage: AdcResidualStage) -> u32 {
    match stage {
        AdcResidualStage::ChopOff => 0,
        AdcResidualStage::ChopOn => 7,
    }
}

fn signed_lower_half(raw: u32) -> i16 {
    (raw & 0xFFFF) as u16 as i16
}

fn signed_upper_half(raw: u32) -> i16 {
    ((raw >> 16) & 0xFFFF) as u16 as i16
}

fn mean_i16(sum: i32, count: u16) -> i16 {
    let count = i32::from(count.max(1));
    (sum / count).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

#[allow(clippy::too_many_arguments)]
fn residual_evidence(
    stage: AdcResidualStage,
    count: u16,
    pwm_sv_chop: u8,
    pwm_bbm: u32,
    adc_i0_scale_offset: u32,
    adc_i1_scale_offset: u32,
    adc_iux_sum: i32,
    adc_iwy_sum: i32,
    pid_flux_sum: i32,
    pid_torque_sum: i32,
) -> AdcResidualEvidence {
    AdcResidualEvidence {
        stage: residual_stage_code(stage),
        sample_count: count.min(u16::from(u8::MAX)) as u8,
        pwm_sv_chop,
        pwm_bbm,
        adc_i0_scale_offset,
        adc_i1_scale_offset,
        adc_iux_mean_count: mean_i16(adc_iux_sum, count),
        adc_iwy_mean_count: mean_i16(adc_iwy_sum, count),
        pid_flux_mean_count: mean_i16(pid_flux_sum, count),
        pid_torque_mean_count: mean_i16(pid_torque_sum, count),
    }
}

/// Advance ADC calibration sub-state.
///
/// Configures TMC4671 for zero-current ADC sampling (PWM_CHOP=0),
/// collects 128 samples one per `step()` call, validates the resulting
/// offsets, and programs them back.
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
            if backend
                .read_register_verified(MODE_RAMP_MODE_MOTION, 0)
                .await
                .is_err()
                || backend
                    .read_register_verified(PWM_SV_CHOP, 0)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // ADC configuration.
            let _ = backend
                .write_register(DSADC_MDEC_B_MDEC_A, backend.dsadc_mdec_register())
                .await;
            let _ = backend
                .write_register(ADC_I_SELECT, backend.tmc_config().adc_i_select)
                .await;

            // Motor type: 2-phase stepper, 50 pole pairs. The MOTOR_TYPE write
            // must be verified before storing the sentinel — silently continuing
            // past a failed write here would let arm_motor_electrical see a
            // sentinel that doesn't match what's actually on the chip and trip
            // the safety check, but worse, MOTOR_TYPE=0 plus stepper PWM
            // excitation is the AGENTS.md "destroys steppers" case.
            const MOTOR_TYPE_VALUE: u32 = (2 << 16) | 50;
            if backend
                .write_register_verified(MOTOR_TYPE_N_POLE_PAIRS, MOTOR_TYPE_VALUE)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            backend.store_motor_sentinel(MOTOR_TYPE_VALUE);

            // Clear offsets, enable outputs.
            let _ = backend.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
            let _ = backend.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;
            backend.set_enable_pin(true);

            let deadline = backend.elapsed_us().wrapping_add(ADC_CALIBRATION_SETTLE_US);
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
            if *count >= ADC_CALIBRATION_SAMPLES {
                *cal_state = AdcCalState::Finalize {
                    sum_i0: *sum_i0,
                    sum_i1: *sum_i1,
                };
            }
            StepResult::Continue
        }

        AdcCalState::Finalize { sum_i0, sum_i1 } => {
            let n = ADC_CALIBRATION_SAMPLES as i32;
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
            let adc_i0_scale_offset = (256 << 16) | offset_i0 as u32;
            let adc_i1_scale_offset = (256 << 16) | offset_i1 as u32;
            if backend
                .write_register_verified(ADC_I0_SCALE_OFFSET, adc_i0_scale_offset)
                .await
                .is_err()
                || backend
                    .write_register_verified(ADC_I1_SCALE_OFFSET, adc_i1_scale_offset)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Keep the controller in open-loop zero-voltage state so the
            // residual reads measure ADC/bridge bias instead of a PI loop
            // nulling the error at a zero-current target.
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, 0)
                .await
                .is_err()
                || backend
                    .write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .is_err()
                || backend.write_register(UQ_UD_EXT, 0).await.is_err()
                || backend
                    .write_register_verified(PWM_SV_CHOP, 0)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let deadline = backend.elapsed_us().wrapping_add(ADC_CALIBRATION_SETTLE_US);
            *cal_state = AdcCalState::ResidualSettle {
                stage: AdcResidualStage::ChopOff,
                offset_i0,
                offset_i1,
                chop0: None,
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        AdcCalState::ResidualSettle {
            stage,
            offset_i0,
            offset_i1,
            chop0,
            deadline_us,
        } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }
            let pwm_sv_chop = match backend.read_register(PWM_SV_CHOP).await {
                Ok(value) if value == residual_stage_chop_value(*stage) => value as u8,
                _ => return super::fail_inner(phase, CommissionError::Spi),
            };
            let pwm_bbm = match backend.read_register(PWM_BBM_H_BBM_L).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let adc_i0_scale_offset = match backend.read_register(ADC_I0_SCALE_OFFSET).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let adc_i1_scale_offset = match backend.read_register(ADC_I1_SCALE_OFFSET).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            *cal_state = AdcCalState::ResidualSample {
                stage: *stage,
                offset_i0: *offset_i0,
                offset_i1: *offset_i1,
                adc_i0_scale_offset,
                adc_i1_scale_offset,
                pwm_sv_chop,
                pwm_bbm,
                adc_iux_sum: 0,
                adc_iwy_sum: 0,
                pid_flux_sum: 0,
                pid_torque_sum: 0,
                count: 0,
                chop0: *chop0,
            };
            StepResult::Continue
        }

        AdcCalState::ResidualSample {
            stage,
            offset_i0,
            offset_i1,
            adc_i0_scale_offset,
            adc_i1_scale_offset,
            pwm_sv_chop,
            pwm_bbm,
            adc_iux_sum,
            adc_iwy_sum,
            pid_flux_sum,
            pid_torque_sum,
            count,
            chop0,
        } => {
            let adc_raw = match backend.read_register(ADC_IWY_IUX).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let pid_raw = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            *adc_iux_sum += i32::from(signed_lower_half(adc_raw));
            *adc_iwy_sum += i32::from(signed_upper_half(adc_raw));
            *pid_flux_sum += i32::from(signed_lower_half(pid_raw));
            *pid_torque_sum += i32::from(signed_upper_half(pid_raw));
            *count += 1;

            if *count < ADC_CALIBRATION_SAMPLES {
                return StepResult::Continue;
            }

            let evidence = residual_evidence(
                *stage,
                *count,
                *pwm_sv_chop,
                *pwm_bbm,
                *adc_i0_scale_offset,
                *adc_i1_scale_offset,
                *adc_iux_sum,
                *adc_iwy_sum,
                *pid_flux_sum,
                *pid_torque_sum,
            );

            match *stage {
                AdcResidualStage::ChopOff => {
                    if backend
                        .write_register_verified(PWM_SV_CHOP, 7)
                        .await
                        .is_err()
                    {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                    let deadline = backend.elapsed_us().wrapping_add(ADC_CALIBRATION_SETTLE_US);
                    *cal_state = AdcCalState::ResidualSettle {
                        stage: AdcResidualStage::ChopOn,
                        offset_i0: *offset_i0,
                        offset_i1: *offset_i1,
                        chop0: Some(evidence),
                        deadline_us: deadline,
                    };
                    StepResult::Continue
                }
                AdcResidualStage::ChopOn => {
                    let encoder_count = backend.read_mechanical_encoder_position_counts().await;
                    let result = AdcCalResult {
                        adc_i0: *offset_i0,
                        adc_i1: *offset_i1,
                        encoder_count,
                        encoder_alignment_electrical_residual_counts: 0,
                        encoder_alignment_stability_counts: 0,
                        encoder_alignment_movement_counts: 0,
                        encoder_alignment_min_movement_counts: 0,
                        encoder_alignment_counts_per_electrical_rev: 0,
                        adc_residual_chop0: chop0.unwrap_or(AdcResidualEvidence::EMPTY),
                        adc_residual_chop7: evidence,
                    };
                    *adc_cal = Some(result.clone());

                    match sequence {
                        InnerSequence::Calibrate => {
                            *phase = InnerPhase::EncoderCheck {
                                state: EncoderCheckState::Setup,
                            };
                            StepResult::PhaseChanged(PhaseId::EncoderCheck)
                        }
                        InnerSequence::Selftest | InnerSequence::Commission => {
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::types::StepResult;

    fn offset_sum(offset: u16) -> i32 {
        i32::from(offset as i16) * i32::from(ADC_CALIBRATION_SAMPLES)
    }

    fn pack_signed_halves(low: i16, high: i16) -> u32 {
        ((high as u16 as u32) << 16) | low as u16 as u32
    }

    #[test]
    fn setup_writes_board_configured_adc_i_select() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.tmc_config.adc_i_select = 0x1234_5678;
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Setup);
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            assert!(
                backend.spi_log.iter().any(|op| matches!(
                    op,
                    SpiOp::Write(addr, 0x1234_5678)
                        if *addr == foci_tmc4671::registers::ADC_I_SELECT.addr()
                )),
                "ADC setup must use the board-provided ADC_I_SELECT value"
            );
        });
    }

    #[test]
    fn adc_cal_stores_sentinel_after_motor_type_write() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Setup);
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            assert_eq!(
                backend.last_motor_sentinel_stored.get(),
                Some((2 << 16) | 50),
            );
        });
    }

    #[test]
    fn finalize_fails_when_programmed_adc_offset_does_not_verify() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override = Some((
                foci_tmc4671::registers::ADC_I0_SCALE_OFFSET.addr(),
                (256 << 16) | 32_999,
            ));
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Finalize {
                sum_i0: offset_sum(33_000),
                sum_i1: offset_sum(33_100),
            });
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(adc_cal.is_none());
        });
    }

    #[test]
    fn adc_cal_records_open_loop_residuals_before_and_after_chop_restore() {
        pollster::block_on(async {
            use crate::commissioning::types::{
                ADC_RESIDUAL_STAGE_CHOP_OFF, ADC_RESIDUAL_STAGE_CHOP_ON,
            };
            use foci_tmc4671::registers::{ADC_IWY_IUX, PID_TORQUE_FLUX_ACTUAL, PWM_BBM_H_BBM_L};

            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(ADC_IWY_IUX.addr(), pack_signed_halves(12, -34))
                .unwrap();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_signed_halves(5, -7))
                .unwrap();
            backend
                .registers
                .insert(PWM_BBM_H_BBM_L.addr(), 0x0009_0009)
                .unwrap();

            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Finalize {
                sum_i0: offset_sum(33_000),
                sum_i1: offset_sum(33_100),
            });
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let first = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(first, StepResult::Continue));
            assert!(adc_cal.is_none());

            backend.elapsed_us_value = backend.elapsed_us_value.saturating_add(250_000);
            let settled = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(settled, StepResult::Continue));

            for _ in 0..ADC_CALIBRATION_SAMPLES {
                let sample = step_adc_calibrate(
                    &mut phase,
                    InnerSequence::Calibrate,
                    &mut adc_cal,
                    &mut selftest_results,
                    &mut backend,
                )
                .await;
                assert!(matches!(sample, StepResult::Continue));
            }

            backend.elapsed_us_value = backend.elapsed_us_value.saturating_add(250_000);
            let settled = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(settled, StepResult::Continue));

            let mut result = StepResult::Continue;
            for _ in 0..ADC_CALIBRATION_SAMPLES {
                result = step_adc_calibrate(
                    &mut phase,
                    InnerSequence::Calibrate,
                    &mut adc_cal,
                    &mut selftest_results,
                    &mut backend,
                )
                .await;
            }

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::EncoderCheck)
            ));
            let cal = adc_cal.expect("ADC calibration result should be recorded");
            assert_eq!(cal.adc_residual_chop0.stage, ADC_RESIDUAL_STAGE_CHOP_OFF);
            assert_eq!(cal.adc_residual_chop0.sample_count, 128);
            assert_eq!(cal.adc_residual_chop0.pwm_sv_chop, 0);
            assert_eq!(cal.adc_residual_chop0.pwm_bbm, 0x0009_0009);
            assert_eq!(
                cal.adc_residual_chop0.adc_i0_scale_offset,
                (256 << 16) | 33_000
            );
            assert_eq!(
                cal.adc_residual_chop0.adc_i1_scale_offset,
                (256 << 16) | 33_100
            );
            assert_eq!(cal.adc_residual_chop0.adc_iux_mean_count, 12);
            assert_eq!(cal.adc_residual_chop0.adc_iwy_mean_count, -34);
            assert_eq!(cal.adc_residual_chop0.pid_flux_mean_count, 5);
            assert_eq!(cal.adc_residual_chop0.pid_torque_mean_count, -7);

            assert_eq!(cal.adc_residual_chop7.stage, ADC_RESIDUAL_STAGE_CHOP_ON);
            assert_eq!(cal.adc_residual_chop7.sample_count, 128);
            assert_eq!(cal.adc_residual_chop7.pwm_sv_chop, 7);
            assert_eq!(cal.adc_residual_chop7.adc_iux_mean_count, 12);
            assert_eq!(cal.adc_residual_chop7.pid_flux_mean_count, 5);
        });
    }
}
