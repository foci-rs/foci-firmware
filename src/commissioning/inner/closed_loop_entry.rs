//! Phase 8: Closed-loop entry.
//!
//! Final phase before motion: writes the inner-loop (flux/torque/velocity/
//! position) PID gains and velocity limit, raises the post-calibration
//! voltage limit, locks `PHI_E_SELECTION` to the ABN encoder, zeros the
//! position registers and PID integrators, programs `STEP_WIDTH` and
//! biquad filters, enters position mode, and runs a 30 ms two-sample
//! stability check that aborts on runaway or drift.

use crate::autotune::{ClosedLoopEntryState, CommissionError, CommissionStatus, InnerSequence};
use crate::commission::InnerPhase;
use crate::commissioning::types::{
    AdcCalResult, CommissionResult, CommissioningBackend, InnerResult, StepResult,
};

/// Advance closed-loop entry sub-state by one cooperative tick.
pub(crate) async fn step_closed_loop_entry<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (
        cle_state,
        flux_p,
        flux_i,
        torque_p,
        torque_i,
        velocity_p,
        velocity_i,
        position_p,
        position_i,
        velocity_limit,
        velocity_filter_hz,
        torque_filter_hz,
        position_filter_hz,
        flux_filter_hz,
        inner_result,
    ) = match phase {
        InnerPhase::ClosedLoopEntry {
            state,
            flux_p,
            flux_i,
            torque_p,
            torque_i,
            velocity_p,
            velocity_i,
            position_p,
            position_i,
            velocity_limit,
            velocity_filter_hz,
            torque_filter_hz,
            position_filter_hz,
            flux_filter_hz,
            inner_result,
        } => (
            state,
            *flux_p,
            *flux_i,
            *torque_p,
            *torque_i,
            *velocity_p,
            *velocity_i,
            *position_p,
            *position_i,
            *velocity_limit,
            *velocity_filter_hz,
            *torque_filter_hz,
            *position_filter_hz,
            *flux_filter_hz,
            inner_result,
        ),
        _ => return StepResult::Continue,
    };

    match cle_state {
        ClosedLoopEntryState::WriteGains => {
            // 1. Flux and torque PID gains.
            let flux_reg = (flux_p as u32) << 16 | flux_i as u32;
            let torque_reg = (torque_p as u32) << 16 | torque_i as u32;
            if backend
                .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                .await
                .is_err()
                || backend
                    .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // 2. Velocity PID gains.
            let velocity_reg = (velocity_p as u32) << 16 | velocity_i as u32;
            if backend
                .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // 3. Position PID gains.
            let position_reg = (position_p as u32) << 16 | position_i as u32;
            if backend
                .write_register(PID_POSITION_P_POSITION_I, position_reg)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *cle_state = ClosedLoopEntryState::WriteLimits;
            StepResult::Continue
        }

        ClosedLoopEntryState::WriteLimits => {
            // 4. Velocity limit.
            if backend
                .write_register(PID_VELOCITY_LIMIT, velocity_limit)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // 5. Raise voltage limit for actual operation.
            // ADC cal sets it to 1024 as a safety measure.
            if backend
                .write_register(PIDOUT_UQ_UD_LIMITS, 16000)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            let voltage_limit_readback = backend.read_register(PIDOUT_UQ_UD_LIMITS).await;
            match voltage_limit_readback {
                Ok(16000) => {}
                _ => {
                    defmt::error!("closed-loop: PIDOUT_UQ_UD_LIMITS readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            *cle_state = ClosedLoopEntryState::VerifyPhiE;
            StepResult::Continue
        }

        ClosedLoopEntryState::VerifyPhiE => {
            // Set PHI_E_SELECTION to ABN encoder (3). For Calibrate this
            // is already 3 from EncoderAlign::Commit; for Commission the
            // intervening ElectricalId/CurrentTune phases reset it to 1
            // (external phi_e for voltage-mode sweeps).
            if backend.write_register(PHI_E_SELECTION, 3).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            match backend.read_register(PHI_E_SELECTION).await {
                Ok(3) => {}
                Ok(val) => {
                    defmt::error!("closed-loop: PHI_E_SELECTION={}, expected 3", val);
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            *cle_state = ClosedLoopEntryState::ZeroAndClear;
            StepResult::Continue
        }

        ClosedLoopEntryState::ZeroAndClear => {
            // 6. Zero position (also zeros target per datasheet 0x6B).
            if backend
                .write_register(PID_POSITION_ACTUAL, 0)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Clear PID integrators. These must succeed — entering
            // position mode with stale error sums causes transients.
            let integrators_ok = async {
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_TORQUE_ERROR_SUM,
                    0,
                )
                .await?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_FLUX_ERROR_SUM,
                    0,
                )
                .await?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    0,
                )
                .await?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_POSITION_ERROR_SUM,
                    0,
                )
                .await?;
                backend
                    .write_register(STATUS_FLAGS, 0)
                    .await
                    .map_err(|_| ())
            };
            if integrators_ok.await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *cle_state = ClosedLoopEntryState::WritePositionConfig;
            StepResult::Continue
        }

        ClosedLoopEntryState::WritePositionConfig => {
            // 7. Explicit POSITION_SELECTION=0 (phi_e).
            if backend.write_register(POSITION_SELECTION, 0).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // 8. Write STEP_WIDTH for hardware step/dir input.
            let step_width = backend.step_width();
            if step_width == 0 {
                defmt::error!("STEP_WIDTH=0, aborting closed-loop entry");
                return super::fail_inner(phase, CommissionError::Spi);
            }
            if backend
                .write_register(STEP_WIDTH, step_width)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            // Readback verification.
            match backend.read_register(STEP_WIDTH).await {
                Ok(val) if val == step_width => {}
                _ => {
                    defmt::error!("closed-loop: STEP_WIDTH readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }
            defmt::info!("STEP_WIDTH(0x78) = {} written", step_width);

            *cle_state = ClosedLoopEntryState::WriteFilters;
            StepResult::Continue
        }

        ClosedLoopEntryState::WriteFilters => {
            let pwm_maxcnt = backend.pwm_maxcnt();

            // Best-effort filter writes (warn on failure, don't abort).
            if velocity_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_velocity(
                    backend,
                    velocity_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_V_ENABLE,
                    0,
                )
                .await;
            }

            if torque_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_torque(
                    backend,
                    torque_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_T_ENABLE,
                    0,
                )
                .await;
            }

            if position_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_position(
                    backend,
                    position_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_X_ENABLE,
                    0,
                )
                .await;
            }

            if flux_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_flux(
                    backend,
                    flux_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_F_ENABLE,
                    0,
                )
                .await;
            }

            *cle_state = ClosedLoopEntryState::EnterPositionMode;
            StepResult::Continue
        }

        ClosedLoopEntryState::EnterPositionMode => {
            // MODE_PID_TYPE=1 (bit 31), MODE_MOTION=3 (position).
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            // Readback verification.
            match backend.read_register(MODE_RAMP_MODE_MOTION).await {
                Ok(0x8000_0003) => {}
                _ => {
                    defmt::error!("closed-loop: MODE_RAMP_MODE_MOTION readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            // Schedule stability check after 10ms.
            let now = backend.elapsed_us();
            *cle_state = ClosedLoopEntryState::StabilityCheck1 {
                position_1: 0,
                deadline_us: now + 10_000,
            };
            StepResult::Continue
        }

        ClosedLoopEntryState::StabilityCheck1 {
            position_1: _,
            deadline_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::Continue;
            }

            // Read first position sample.
            match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(val) => {
                    let pos1 = val as i32;
                    let now = backend.elapsed_us();
                    *cle_state = ClosedLoopEntryState::StabilityCheck2 {
                        position_1: pos1,
                        deadline_us: now + 20_000,
                    };
                }
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }
            StepResult::Continue
        }

        ClosedLoopEntryState::StabilityCheck2 {
            position_1,
            deadline_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::Continue;
            }

            // Read second position sample and check for runaway.
            let pos2 = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(val) => val as i32,
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };

            // Runaway detection: if position error is large and growing,
            // the feedback sign is wrong.
            const RUNAWAY_THRESHOLD: u32 = 32760; // ~40 steps
            if pos2.unsigned_abs() > RUNAWAY_THRESHOLD
                && pos2.unsigned_abs() > position_1.unsigned_abs()
            {
                defmt::error!(
                    "closed-loop: runaway detected: pos1={}, pos2={}",
                    position_1,
                    pos2
                );
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Stability check: position must not be drifting.
            // Threshold of 2 encoder counts between 10ms and 30ms
            // readings indicates the motor is holding position.
            const STABILITY_THRESHOLD: u32 = 2;
            let drift = (pos2.wrapping_sub(*position_1)).unsigned_abs();
            if drift > STABILITY_THRESHOLD {
                defmt::error!(
                    "closed-loop: unstable: pos1={}, pos2={}, drift={}",
                    position_1,
                    pos2,
                    drift
                );
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            defmt::info!("closed-loop position mode active (engine)");

            match sequence {
                InnerSequence::Calibrate => {
                    let adc_result = adc_cal.clone().unwrap_or(AdcCalResult {
                        adc_i0: 0,
                        adc_i1: 0,
                        encoder_count: 0,
                    });
                    StepResult::CalibrateDone(adc_result)
                }
                InnerSequence::Commission => {
                    let inner = inner_result.take().unwrap_or(InnerResult {
                        status: CommissionStatus::Accepted,
                        r_mohm: 0,
                        l_uh: 0,
                        flux_p: 0,
                        flux_i: 0,
                        torque_p: 0,
                        torque_i: 0,
                        lambda_us: 0,
                        theta_e_us: 0,
                        ringing_count: 0,
                        bandwidth_hz: 0,
                    });
                    StepResult::CommissionDone(CommissionResult {
                        inner,
                        fallback_velocity_p: velocity_p,
                        fallback_velocity_i: velocity_i,
                        fallback_position_p: position_p,
                        fallback_position_i: position_i,
                        fallback_velocity_limit: velocity_limit,
                    })
                }
                InnerSequence::Selftest => {
                    // Selftest never reaches ClosedLoopEntry.
                    unreachable!()
                }
            }
        }
    }
}
