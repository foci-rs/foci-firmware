//! Phase 4: Encoder presence/direction check and D-axis alignment.
//!
//! Drives the rotor with current-regulated open-loop excitation, sweeps
//! through electrical angles to confirm encoder counts move, then locks
//! `PHI_E_SELECTION` to the ABN decoder before any closed-loop operation.

use crate::autotune::{ClosedLoopEntryState, CommissionError, InnerSequence, PhaseId};
use crate::commission::{
    ElectricalIdData, ElectricalIdState, EncoderAlignState, EncoderCheckState, InnerPhase,
};
use crate::commissioning::types::{AdcCalResult, CommissioningBackend, StepResult};

/// Number of PHI_E steps for encoder direction sweep.
const ENCODER_DIRECTION_STEPS: u16 = 50;

/// Final PHI_E_EXT value for encoder direction sweep.
///
/// Keep this away from 0/65536 so the sweep has a non-zero final rotor
/// displacement. A full electrical revolution can end at the same stable
/// angle it started from, producing a false ENCODER_FAULT.
const ENCODER_DIRECTION_TARGET_PHI_E: u32 = 32768;

/// Per-step dwell time for encoder direction sweep (microseconds).
const ENCODER_DIRECTION_STEP_US: u64 = 10_000;

/// Settling time after applying encoder alignment voltage.
const ENCODER_ALIGNMENT_SETTLE_US: u64 = 500_000;

/// Stability double-read delay after encoder alignment settle.
const ENCODER_ALIGNMENT_STABILITY_US: u64 = 10_000;

/// Allowed encoder jitter during the alignment double-read.
const ENCODER_ALIGNMENT_STABILITY_COUNTS: u32 = 2;

/// Voltage limit used only while applying encoder movement current.
const ENCODER_ALIGNMENT_VOLTAGE_LIMIT: u32 = 2000;

/// Minimum D-axis current target used while forcing encoder movement.
const ENCODER_MOVEMENT_CURRENT_MIN: u32 = 100;

/// Maximum D-axis current target used while forcing encoder movement.
const ENCODER_MOVEMENT_CURRENT_MAX: u32 = 1000;

/// Start current-regulated open-loop encoder movement.
///
/// Direct voltage mode (`MODE=8` + `UQ_UD_EXT`) is useful for coil
/// current measurements, but it can fail to move a loaded rotor far
/// enough for encoder validation. The dev open-loop path uses this
/// current-regulated pattern: external PHI_E, a D-axis flux target,
/// current-loop gains, and torque/flux current limits.
pub(crate) async fn start_encoder_current_drive<B: CommissioningBackend>(
    backend: &mut B,
    phi_e: u32,
) -> Result<(), CommissionError> {
    use foci_tmc4671::registers::*;

    let current_limit = backend.current_limit() & 0xFFFF;
    if current_limit == 0 {
        return Err(CommissionError::NoCurrent);
    }

    let target = (current_limit / 2)
        .clamp(ENCODER_MOVEMENT_CURRENT_MIN, ENCODER_MOVEMENT_CURRENT_MAX)
        .min(current_limit) as i16;
    let defaults = crate::pid::DEFAULT_INNER_GAINS;
    let flux_reg = (defaults.flux_p as u32) << 16 | defaults.flux_i as u32;
    let torque_reg = (defaults.torque_p as u32) << 16 | defaults.torque_i as u32;
    let target_reg: u32 = PidTorqueFluxTarget::new()
        .with_pid_flux_target(target)
        .with_pid_torque_target(0)
        .into();

    backend.set_enable_pin(false);
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_TORQUE_FLUX_TARGET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(UQ_UD_EXT, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_FLUX_P_FLUX_I, flux_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_TORQUE_FLUX_LIMITS, backend.current_limit())
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PIDOUT_UQ_UD_LIMITS, ENCODER_ALIGNMENT_VOLTAGE_LIMIT)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PHI_E_SELECTION, 1)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PHI_E_EXT, phi_e)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_TORQUE_FLUX_TARGET, target_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0001)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend.set_enable_pin(true);
    Ok(())
}

/// Stop current-regulated open-loop encoder movement.
pub(crate) async fn stop_encoder_current_drive<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), CommissionError> {
    use foci_tmc4671::registers::*;

    backend
        .write_register(PID_TORQUE_FLUX_TARGET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(UQ_UD_EXT, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend.set_enable_pin(false);
    Ok(())
}

/// Advance encoder check sub-state.
///
/// Sweeps to a displaced electrical angle via PHI_E_EXT in
/// resumable sub-steps, then checks whether the encoder count changed.
pub(crate) async fn step_encoder_check<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let state = match phase {
        InnerPhase::EncoderCheck { state } => state,
        _ => return StepResult::Continue,
    };

    match state {
        EncoderCheckState::Setup => {
            let count_start = backend.read_encoder_count().await as i32;

            if let Err(error) = start_encoder_current_drive(backend, 0).await {
                return super::fail_inner(phase, error);
            }

            let deadline = backend.elapsed_us().wrapping_add(ENCODER_DIRECTION_STEP_US);
            *state = EncoderCheckState::Sweep {
                step: 0,
                deadline_us: deadline,
                count_start,
            };
            StepResult::Continue
        }

        EncoderCheckState::Sweep {
            step,
            deadline_us,
            count_start,
        } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                // Not yet time for next step.
                return StepResult::Continue;
            }

            *step += 1;
            if *step > ENCODER_DIRECTION_STEPS {
                let cs = *count_start;
                *state = EncoderCheckState::Finalize { count_start: cs };
                return StepResult::Continue;
            }

            // Advance PHI_E_EXT toward a non-zero final angle. Ending
            // at 65536/0 can return the rotor to the start angle and
            // make a working encoder look stationary.
            let phi = (ENCODER_DIRECTION_TARGET_PHI_E as u64 * *step as u64
                / ENCODER_DIRECTION_STEPS as u64) as u32;
            let _ = backend.write_register(PHI_E_EXT, phi).await;

            *deadline_us = backend.elapsed_us().wrapping_add(ENCODER_DIRECTION_STEP_US);
            StepResult::Continue
        }

        EncoderCheckState::Finalize { count_start } => {
            let count_end = backend.read_encoder_count().await as i32;

            // Stop and disable (always, regardless of result).
            let _ = stop_encoder_current_drive(backend).await;

            let delta = count_end.wrapping_sub(*count_start);
            defmt::info!(
                "encoder-dir: start={} end={} delta={}",
                *count_start,
                count_end,
                delta
            );
            if delta == 0 {
                defmt::warn!(
                    "encoder-dir FAIL: start={} end={} delta=0",
                    *count_start,
                    count_end
                );
                if matches!(sequence, InnerSequence::Selftest) {
                    selftest_results.set(5, crate::selftest::SelftestStageValue::fail(0));
                }
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }

            // All sequences that reach EncoderCheck proceed to
            // EncoderAlign. Use the final sweep count as the alignment
            // baseline because the rotor is still held at the displaced
            // sweep angle; EncoderAlign::Setup will drive back to PHI=0.
            if matches!(sequence, InnerSequence::Selftest) {
                // Raw encoder delta magnitude; direction captured in stage 6.
                selftest_results.set(
                    5,
                    crate::selftest::SelftestStageValue {
                        status: 0,
                        value: delta.unsigned_abs(),
                    },
                );
                // Stage 6: encoder direction (0=forward, 1=reversed).
                // Reversed when the encoder count decreases as PHI_E increases.
                let reversed = delta < 0;
                selftest_results.set(
                    6,
                    crate::selftest::SelftestStageValue {
                        status: 0,
                        value: if reversed { 1 } else { 0 },
                    },
                );
            }

            // Direction detected. Transition to EncoderAlign, which
            // locks PHI_E_SELECTION to ABN at a known D-axis zero before
            // any later closed-loop or electrical-ID work.
            // Direction value (0=forward, 1=reversed) is carried as
            // commissioning-local data; persistence is outside this plan.
            *phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::Setup {
                    count_before: count_end as u32,
                },
            };
            StepResult::PhaseChanged(PhaseId::EncoderAlign)
        }
    }
}

/// Advance encoder D-axis alignment sub-state.
///
/// Applies current-regulated open-loop drive, verifies the encoder
/// moved and settled, zeros ABN count/offset while still driving, then
/// locks PHI_E to ABN before stopping the drive.
pub(crate) async fn step_encoder_align<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    _selftest_results: &mut crate::selftest::SelftestResults,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let state = match phase {
        InnerPhase::EncoderAlign { state } => state,
        _ => return StepResult::Continue,
    };

    match state {
        EncoderAlignState::Setup { count_before } => {
            if backend.current_limit() == 0 || backend.step_width() == 0 {
                return super::fail_inner(phase, CommissionError::NoCurrent);
            }

            if let Err(error) = start_encoder_current_drive(backend, 0).await {
                return super::fail_inner(phase, error);
            }

            let phi_ok = backend
                .read_register(PHI_E_SELECTION)
                .await
                .is_ok_and(|v| v == 1);
            let mode_ok = backend
                .read_register(MODE_RAMP_MODE_MOTION)
                .await
                .is_ok_and(|v| v == 0x8000_0001);
            if !phi_ok || !mode_ok {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            backend.set_enable_pin(true);

            let step_width = backend.step_width().max(1);
            let min_movement = (65536u32 / (20 * step_width)).max(2);
            let deadline = backend
                .elapsed_us()
                .wrapping_add(ENCODER_ALIGNMENT_SETTLE_US);
            *state = EncoderAlignState::Settle {
                count_before: *count_before,
                deadline_us: deadline,
                min_movement,
            };
            StepResult::Continue
        }

        EncoderAlignState::Settle {
            count_before,
            deadline_us,
            min_movement,
        } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                *state = EncoderAlignState::StabilityCheck1 {
                    count_before: *count_before,
                    min_movement: *min_movement,
                };
            }
            StepResult::Continue
        }

        EncoderAlignState::StabilityCheck1 {
            count_before,
            min_movement,
        } => {
            let count1 = match backend.read_register(ABN_DECODER_COUNT).await {
                Ok(v) => v,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let deadline = backend
                .elapsed_us()
                .wrapping_add(ENCODER_ALIGNMENT_STABILITY_US);
            *state = EncoderAlignState::StabilityCheck2 {
                count_before: *count_before,
                count1,
                deadline_us: deadline,
                min_movement: *min_movement,
            };
            StepResult::Continue
        }

        EncoderAlignState::StabilityCheck2 {
            count_before,
            count1,
            deadline_us,
            min_movement,
        } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let count2 = match backend.read_register(ABN_DECODER_COUNT).await {
                Ok(v) => v,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };

            let movement1 = (*count1 as i32 - *count_before as i32).unsigned_abs();
            let movement2 = (count2 as i32 - *count_before as i32).unsigned_abs();
            let movement = movement1.max(movement2);
            let stable = (*count1 as i32 - count2 as i32).unsigned_abs()
                <= ENCODER_ALIGNMENT_STABILITY_COUNTS;
            if movement < *min_movement || !stable {
                defmt::warn!(
                    "encoder-align FAIL: count_before={} count1={} count2={} \
                     movement={} min={} stable={}",
                    *count_before,
                    *count1,
                    count2,
                    movement,
                    *min_movement,
                    stable
                );
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }

            *state = EncoderAlignState::Commit { old_count: *count1 };
            StepResult::Continue
        }

        EncoderAlignState::Commit { old_count } => {
            if backend.write_register(ABN_DECODER_COUNT, 0).await.is_err()
                || backend
                    .write_register(ABN_DECODER_PHI_E_PHI_M_OFFSET, 0)
                    .await
                    .is_err()
                || backend.write_register(PHI_E_SELECTION, 3).await.is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let phi_ok = backend
                .read_register(PHI_E_SELECTION)
                .await
                .is_ok_and(|v| v == 3);
            if !phi_ok {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *state = EncoderAlignState::StopDrive {
                old_count: *old_count,
            };
            StepResult::Continue
        }

        EncoderAlignState::StopDrive { old_count } => {
            if let Err(error) = stop_encoder_current_drive(backend).await {
                return super::fail_inner(phase, error);
            }

            if let Some(result) = adc_cal.as_mut() {
                result.encoder_count = *old_count;
            }

            match sequence {
                InnerSequence::Calibrate => {
                    // Transition to ClosedLoopEntry with conservative
                    // default gains. The CalibrateDone handler in
                    // main.rs will overwrite current-loop gains with
                    // stored_gains (from prior commissioning) and
                    // re-apply filters from atomics.
                    let defaults = crate::pid::DEFAULT_INNER_GAINS;
                    let vel_limit = backend.operational_velocity_limit();
                    *phase = InnerPhase::ClosedLoopEntry {
                        state: ClosedLoopEntryState::WriteGains,
                        flux_p: defaults.flux_p,
                        flux_i: defaults.flux_i,
                        torque_p: defaults.torque_p,
                        torque_i: defaults.torque_i,
                        velocity_p: 1152, // 4.5 in Q8.8 (conservative default)
                        velocity_i: 0,
                        position_p: 640, // 2.5 in Q8.8 (conservative default)
                        position_i: 0,
                        velocity_limit: vel_limit,
                        velocity_filter_hz: 0,
                        torque_filter_hz: 0,
                        position_filter_hz: 0,
                        flux_filter_hz: 0,
                        inner_result: None,
                    };
                    StepResult::PhaseChanged(PhaseId::ClosedLoopEntry)
                }
                InnerSequence::Selftest | InnerSequence::Commission => {
                    let voltage = backend.excitation_voltage() as u32;
                    *phase = InnerPhase::ElectricalId {
                        state: ElectricalIdState::ResSetup { coil: 0 },
                        data: ElectricalIdData::new(voltage),
                    };
                    StepResult::PhaseChanged(PhaseId::ElectricalId)
                }
            }
        }
    }
}
