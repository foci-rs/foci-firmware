//! Phase 4: Encoder presence/direction check and D-axis alignment.
//!
//! Drives the rotor with current-regulated open-loop excitation, sweeps
//! through electrical angles to confirm encoder counts move, then locks
//! `PHI_E_SELECTION` to the ABN decoder before any closed-loop operation.

use super::{
    ElectricalIdData, ElectricalIdState, EncoderAlignState, EncoderCheckState, InnerPhase,
};
use crate::autotune::{ClosedLoopEntryState, CommissionError, InnerSequence, PhaseId};
use crate::commissioning::CalibrateEntryGains;
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain};
use crate::commissioning::types::{AdcCalResult, CommissioningBackend, StepResult};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics};

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
/// OpenFFBoard bringup motors use 50 pole pairs.
const ENCODER_ALIGNMENT_POLE_PAIRS: u32 = 50;

const ENCODER_READ_SAMPLES: usize = 3;
const ENCODER_READ_MAX_JITTER: u32 = 2;
const ENCODER_READ_ATTEMPTS: usize = 3;

const DIAG_ENCODER_READ_UNSTABLE: u8 = 1;
const DIAG_ENCODER_DIRECTION_RESULT: u8 = 2;
const DIAG_ENCODER_ALIGN_RESULT: u8 = 1;

fn encoder_alignment_min_movement(encoder_counts_per_rev: u32) -> u32 {
    let denominator = 20_u64 * u64::from(ENCODER_ALIGNMENT_POLE_PAIRS);
    let numerator = u64::from(encoder_counts_per_rev);
    ((numerator + denominator / 2) / denominator).max(2) as u32
}

fn saturating_u16(value: u32) -> u16 {
    value.min(u32::from(u16::MAX)) as u16
}

fn counts_per_electrical_rev(counts_per_rev: u32) -> u16 {
    saturating_u16((counts_per_rev / ENCODER_ALIGNMENT_POLE_PAIRS).max(1))
}

fn folded_electrical_residual_counts(count: u32, counts_per_electrical_rev: u16) -> u16 {
    let period = u32::from(counts_per_electrical_rev).max(1);
    let remainder = count % period;
    let folded = remainder.min(period.saturating_sub(remainder));
    saturating_u16(folded)
}

fn push_encoder_read_unstable(
    diagnostics: &mut CommissionDiagnostics,
    phase_id: PhaseId,
    samples: [u32; ENCODER_READ_SAMPLES],
) {
    diagnostics.push(CommissionDiagnostic {
        phase: phase_id.to_u8(),
        code: DIAG_ENCODER_READ_UNSTABLE,
        status: 1,
        value0: samples[0],
        value1: samples[1],
        value2: samples[2],
    });
}

fn push_encoder_direction_result(
    diagnostics: &mut CommissionDiagnostics,
    count_start: EncoderCount,
    count_end: EncoderCount,
    delta_abs: u32,
    failed: bool,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: PhaseId::EncoderCheck.to_u8(),
        code: DIAG_ENCODER_DIRECTION_RESULT,
        status: u8::from(failed),
        value0: count_start.raw_counts(),
        value1: count_end.raw_counts(),
        value2: delta_abs,
    });
}

fn push_encoder_align_result(
    diagnostics: &mut CommissionDiagnostics,
    movement: u32,
    min_movement: u32,
    stability: u32,
    failed: bool,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: PhaseId::EncoderAlign.to_u8(),
        code: DIAG_ENCODER_ALIGN_RESULT,
        status: u8::from(failed),
        value0: movement,
        value1: min_movement,
        value2: stability,
    });
}

/// Read `ABN_DECODER_COUNT` multiple times and return a stable value.
///
/// Reads `ENCODER_READ_SAMPLES` samples per attempt and accepts when the
/// pairwise wrap-aware spread is <= `ENCODER_READ_MAX_JITTER`. Retries up
/// to `ENCODER_READ_ATTEMPTS` times. Ambiguous half-turn evidence makes the
/// current attempt unstable instead of selecting an arbitrary direction.
///
/// Returns `CommissionError::EncoderFault` when attempts are exhausted.
/// Propagates `CommissionError::Spi` if the underlying register read fails.
async fn read_mechanical_encoder_position_counts_stable<B: CommissioningBackend>(
    backend: &mut B,
    diagnostics: Option<&mut CommissionDiagnostics>,
    phase_id: PhaseId,
) -> Result<EncoderCount, CommissionError> {
    use foci_tmc4671::registers::ABN_DECODER_COUNT;

    let domain = EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev())
        .map_err(|_| CommissionError::EncoderFault)?;
    let mut last_samples = [0u32; ENCODER_READ_SAMPLES];
    for _ in 0..ENCODER_READ_ATTEMPTS {
        let mut samples = [0u32; ENCODER_READ_SAMPLES];
        for sample in samples.iter_mut() {
            *sample = backend
                .read_register(ABN_DECODER_COUNT)
                .await
                .map_err(|_| CommissionError::Spi)?;
        }
        last_samples = samples;
        let counts = samples.map(|sample| domain.count(sample));
        let mut spread = 0;
        let mut ambiguous = false;
        for previous in counts {
            for current in counts {
                match domain.delta(previous, current) {
                    Ok(delta) => spread = spread.max(delta.absolute_counts()),
                    Err(_) => ambiguous = true,
                }
            }
        }
        if !ambiguous && spread <= ENCODER_READ_MAX_JITTER {
            return Ok(counts[ENCODER_READ_SAMPLES - 1]);
        }
    }
    if let Some(diagnostics) = diagnostics {
        push_encoder_read_unstable(diagnostics, phase_id, last_samples);
    }
    Err(CommissionError::EncoderFault)
}

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
    diagnostics: &mut CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let state = match phase {
        InnerPhase::EncoderCheck { state } => state,
        _ => return StepResult::Continue,
    };

    match state {
        EncoderCheckState::Setup => {
            let count_start = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                PhaseId::EncoderCheck,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => return super::fail_inner(phase, error),
            };

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
            let count_end = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                PhaseId::EncoderCheck,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, error);
                }
            };

            // Stop and disable (always, regardless of result).
            let _ = stop_encoder_current_drive(backend).await;

            let domain =
                match EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()) {
                    Ok(domain) => domain,
                    Err(_) => return super::fail_inner(phase, CommissionError::EncoderFault),
                };
            let delta = match domain.delta(*count_start, count_end) {
                Ok(delta) => delta,
                Err(_) => {
                    push_encoder_direction_result(diagnostics, *count_start, count_end, 0, true);
                    return super::fail_inner(phase, CommissionError::EncoderFault);
                }
            };
            defmt::info!(
                "encoder-dir: start={} end={} delta={}",
                count_start.raw_counts(),
                count_end.raw_counts(),
                delta.signed_counts()
            );
            if delta.signed_counts() == 0 {
                push_encoder_direction_result(
                    diagnostics,
                    *count_start,
                    count_end,
                    delta.absolute_counts(),
                    true,
                );
                defmt::warn!(
                    "encoder-dir FAIL: start={} end={} delta=0",
                    count_start.raw_counts(),
                    count_end.raw_counts()
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
                        value: delta.absolute_counts(),
                    },
                );
                // Stage 6: encoder direction (0=forward, 1=reversed).
                // Reversed when the encoder count decreases as PHI_E increases.
                let reversed = delta.signed_counts() < 0;
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
                    count_before: count_end,
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
    calibrate_entry_gains: Option<CalibrateEntryGains>,
    _selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let state = match phase {
        InnerPhase::EncoderAlign { state } => state,
        _ => return StepResult::Continue,
    };

    match state {
        EncoderAlignState::Setup { count_before } => {
            if EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()).is_err() {
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }
            if backend.current_limit() == 0 {
                return super::fail_inner(phase, CommissionError::NoCurrent);
            }

            if let Err(error) = start_encoder_current_drive(backend, 0).await {
                return super::fail_inner(phase, error);
            }

            if backend
                .read_register_verified(PHI_E_SELECTION, 1)
                .await
                .is_err()
                || backend
                    .read_register_verified(MODE_RAMP_MODE_MOTION, 0x8000_0001)
                    .await
                    .is_err()
            {
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            backend.set_enable_pin(true);

            let min_movement = encoder_alignment_min_movement(
                backend.mechanical_encoder_quadrature_counts_per_rev(),
            );
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
            let count1 = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                PhaseId::EncoderAlign,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, error);
                }
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

            let count2 = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                PhaseId::EncoderAlign,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, error);
                }
            };

            let domain =
                match EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()) {
                    Ok(domain) => domain,
                    Err(_) => {
                        let _ = stop_encoder_current_drive(backend).await;
                        return super::fail_inner(phase, CommissionError::EncoderFault);
                    }
                };
            let movement1 = domain.delta(*count_before, *count1);
            let movement2 = domain.delta(*count_before, count2);
            let stability = domain.delta(*count1, count2);
            let (movement1, movement2, stability) = match (movement1, movement2, stability) {
                (Ok(movement1), Ok(movement2), Ok(stability)) => (
                    movement1.absolute_counts(),
                    movement2.absolute_counts(),
                    stability.absolute_counts(),
                ),
                _ => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, CommissionError::EncoderFault);
                }
            };
            let movement = movement1.max(movement2);
            let stable = stability <= ENCODER_ALIGNMENT_STABILITY_COUNTS;
            if movement < *min_movement || !stable {
                push_encoder_align_result(diagnostics, movement, *min_movement, stability, true);
                defmt::warn!(
                    "encoder-align FAIL: count_before={} count1={} count2={} \
                     movement={} min={} stable={}",
                    count_before.raw_counts(),
                    count1.raw_counts(),
                    count2.raw_counts(),
                    movement,
                    *min_movement,
                    stable
                );
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }

            let counts_per_elec =
                counts_per_electrical_rev(backend.mechanical_encoder_quadrature_counts_per_rev());
            *state = EncoderAlignState::Commit {
                old_count: *count1,
                electrical_residual_counts: folded_electrical_residual_counts(
                    count1.raw_counts(),
                    counts_per_elec,
                ),
                stability_counts: saturating_u16(stability),
                movement_counts: saturating_u16(movement),
                min_movement_counts: saturating_u16(*min_movement),
                counts_per_electrical_rev: counts_per_elec,
            };
            StepResult::Continue
        }

        EncoderAlignState::Commit {
            old_count,
            electrical_residual_counts,
            stability_counts,
            movement_counts,
            min_movement_counts,
            counts_per_electrical_rev,
        } => {
            if backend.write_register(ABN_DECODER_COUNT, 0).await.is_err()
                || backend
                    .write_register(ABN_DECODER_PHI_E_PHI_M_OFFSET, 0)
                    .await
                    .is_err()
                || backend.write_register(PHI_E_SELECTION, 3).await.is_err()
            {
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .read_register_verified(PHI_E_SELECTION, 3)
                .await
                .is_err()
            {
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *state = EncoderAlignState::StopDrive {
                old_count: *old_count,
                electrical_residual_counts: *electrical_residual_counts,
                stability_counts: *stability_counts,
                movement_counts: *movement_counts,
                min_movement_counts: *min_movement_counts,
                counts_per_electrical_rev: *counts_per_electrical_rev,
            };
            StepResult::Continue
        }

        EncoderAlignState::StopDrive {
            old_count,
            electrical_residual_counts,
            stability_counts,
            movement_counts,
            min_movement_counts,
            counts_per_electrical_rev,
        } => {
            if let Err(error) = stop_encoder_current_drive(backend).await {
                return super::fail_inner(phase, error);
            }

            if let Some(result) = adc_cal.as_mut() {
                result.encoder_count = old_count.raw_counts();
                result.encoder_alignment_electrical_residual_counts = *electrical_residual_counts;
                result.encoder_alignment_stability_counts = *stability_counts;
                result.encoder_alignment_movement_counts = *movement_counts;
                result.encoder_alignment_min_movement_counts = *min_movement_counts;
                result.encoder_alignment_counts_per_electrical_rev = *counts_per_electrical_rev;
            }

            match sequence {
                InnerSequence::Calibrate => {
                    // Use the gain snapshot captured when calibration started.
                    // Falling back to defaults keeps direct unit construction
                    // and legacy callers conservative, but production re-enable
                    // paths should provide the host-stored tuned values.
                    let gains = calibrate_entry_gains.unwrap_or(CalibrateEntryGains {
                        inner: crate::pid::DEFAULT_INNER_GAINS,
                        velocity_p: 1152,
                        velocity_i: 0,
                        position_p: 640,
                        position_i: 0,
                        velocity_limit: backend.operational_velocity_limit(),
                        velocity_filter_hz: 0,
                        torque_filter_hz: 0,
                        position_filter_hz: 0,
                        flux_filter_hz: 0,
                    });
                    *phase = InnerPhase::ClosedLoopEntry {
                        state: ClosedLoopEntryState::WriteGains,
                        flux_p: gains.inner.flux_p,
                        flux_i: gains.inner.flux_i,
                        torque_p: gains.inner.torque_p,
                        torque_i: gains.inner.torque_i,
                        velocity_p: gains.velocity_p,
                        velocity_i: gains.velocity_i,
                        position_p: gains.position_p,
                        position_i: gains.position_i,
                        velocity_limit: gains.velocity_limit,
                        velocity_filter_hz: gains.velocity_filter_hz,
                        torque_filter_hz: gains.torque_filter_hz,
                        position_filter_hz: gains.position_filter_hz,
                        flux_filter_hz: gains.flux_filter_hz,
                        inner_result: None,
                    };
                    StepResult::PhaseChanged(PhaseId::ClosedLoopEntry)
                }
                InnerSequence::Selftest | InnerSequence::Commission => {
                    let voltage = backend.excitation_voltage() as u32;
                    *phase = InnerPhase::ElectricalId {
                        state: ElectricalIdState::ResistanceIdentify,
                        data: ElectricalIdData::new(voltage),
                    };
                    StepResult::PhaseChanged(PhaseId::ElectricalId)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, SHARED_STATE_MUTEX};

    fn encoder_count(raw_counts: u32) -> EncoderCount {
        EncoderDomain::new(4_000).unwrap().count(raw_counts)
    }

    #[test]
    fn encoder_alignment_min_movement_uses_encoder_count_scale() {
        assert_eq!(encoder_alignment_min_movement(4_000), 4);
    }

    #[test]
    fn stable_helper_accepts_when_all_samples_agree() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            let _ = backend
                .write_register(foci_tmc4671::registers::ABN_DECODER_COUNT, 1234)
                .await;
            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                None,
                PhaseId::EncoderCheck,
            )
            .await;
            assert_eq!(result.unwrap().raw_counts(), 1234);
        });
    }

    #[test]
    fn stable_read_accepts_samples_across_encoder_origin() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[3999, 0, 1]);
            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                None,
                PhaseId::EncoderCheck,
            )
            .await;
            assert!(
                result.is_ok(),
                "wrap-near samples should be accepted, got {:?}",
                result
            );
            assert_eq!(result.unwrap().raw_counts(), 1);
        });
    }

    #[test]
    fn stable_read_half_turn_ambiguity_exhausts_all_attempts() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[0, 2000, 0, 0, 2000, 0, 0, 2000, 0]);
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                Some(&mut diagnostics),
                PhaseId::EncoderCheck,
            )
            .await;

            assert_eq!(result, Err(CommissionError::EncoderFault));
            assert_eq!(
                backend.abn_decoder_count_pos,
                ENCODER_READ_SAMPLES * ENCODER_READ_ATTEMPTS
            );
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].code, DIAG_ENCODER_READ_UNSTABLE);
        });
    }

    #[test]
    fn stable_helper_rejects_after_exhausted_attempts() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[500, 600, 500, 600, 500, 600, 500, 600, 500]);
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                Some(&mut diagnostics),
                PhaseId::EncoderCheck,
            )
            .await;
            match result {
                Err(crate::autotune::CommissionError::EncoderFault) => {}
                other => panic!("expected EncoderFault, got {:?}", other),
            }
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].phase, PhaseId::EncoderCheck.to_u8());
            assert_eq!(events[0].code, DIAG_ENCODER_READ_UNSTABLE);
            assert_eq!(events[0].status, 1);
            assert_eq!(events[0].value0, 500);
            assert_eq!(events[0].value1, 600);
            assert_eq!(events[0].value2, 500);
        });
    }

    #[test]
    fn encoder_check_zero_delta_records_diagnostic() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(1234),
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[1234, 1234, 1234]);

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].phase, PhaseId::EncoderCheck.to_u8());
            assert_eq!(events[0].code, DIAG_ENCODER_DIRECTION_RESULT);
            assert_eq!(events[0].status, 1);
            assert_eq!(events[0].value0, 1234);
            assert_eq!(events[0].value1, 1234);
            assert_eq!(events[0].value2, 0);
        });
    }

    #[test]
    fn encoder_direction_origin_crossing_is_positive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(3998),
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[2, 2, 2]);

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Selftest,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::EncoderAlign)
            ));
            assert_eq!(selftest_results.stage(5).unwrap().value, 4);
            assert_eq!(selftest_results.stage(6).unwrap().value, 0);
        });
    }

    #[test]
    fn encoder_direction_half_turn_stops_current_without_committing_direction() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(0),
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[2000, 2000, 2000]);

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Selftest,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::EncoderFault)
            ));
            assert!(selftest_results.stage(6).is_none());
        });
    }

    #[test]
    fn invalid_encoder_domain_fails_before_direction_drive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Setup,
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 1000;
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(!backend.spi_log.iter().any(|op| matches!(
                op,
                crate::commission_dispatch::test_support::SpiOp::Write(_, _)
                    | crate::commission_dispatch::test_support::SpiOp::EnablePin(true)
            )));
        });
    }

    #[test]
    fn encoder_align_stability_accepts_wrap_boundary() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StabilityCheck2 {
                    count_before: encoder_count(3998),
                    count1: encoder_count(3999),
                    deadline_us: 0,
                    min_movement: 2,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[0, 0, 0]);

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            let InnerPhase::EncoderAlign {
                state:
                    EncoderAlignState::Commit {
                        old_count,
                        electrical_residual_counts: 1,
                        stability_counts: 1,
                        movement_counts: 2,
                        min_movement_counts: 2,
                        counts_per_electrical_rev: 80,
                    },
            } = phase
            else {
                panic!("valid wrap crossing should advance to alignment commit");
            };
            assert_eq!(old_count.raw_counts(), 3999);
        });
    }

    #[test]
    fn invalid_encoder_domain_fails_before_alignment_drive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::Setup {
                    count_before: encoder_count(10),
                },
            };
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;

            let result = step_encoder_align(
                &mut phase,
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
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(backend.spi_log.is_empty());
        });
    }

    #[test]
    fn active_alignment_spi_failures_stop_drive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            use foci_tmc4671::registers::{ABN_DECODER_COUNT, PHI_E_SELECTION};

            for case in 0..3 {
                let mut phase = if case == 0 {
                    InnerPhase::EncoderAlign {
                        state: EncoderAlignState::Setup {
                            count_before: encoder_count(10),
                        },
                    }
                } else {
                    InnerPhase::EncoderAlign {
                        state: EncoderAlignState::Commit {
                            old_count: encoder_count(10),
                            electrical_residual_counts: 1,
                            stability_counts: 1,
                            movement_counts: 4,
                            min_movement_counts: 2,
                            counts_per_electrical_rev: 80,
                        },
                    }
                };
                let mut backend = MockBackend::new();
                backend.current_limit_value = 1000;
                backend.enable_pin = case != 0;
                if case == 1 {
                    backend.fail_on_write = Some(ABN_DECODER_COUNT.addr());
                } else {
                    backend.read_override = Some((PHI_E_SELECTION.addr(), 0));
                }

                let result = step_encoder_align(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut None,
                    None,
                    &mut crate::selftest::SelftestResults::default(),
                    &mut CommissionDiagnostics::default(),
                    &mut backend,
                )
                .await;

                assert!(matches!(
                    result,
                    StepResult::InnerFailed(CommissionError::Spi)
                ));
                assert!(
                    !backend.enable_pin,
                    "case {case} left alignment drive enabled"
                );
            }
        });
    }

    #[test]
    fn encoder_align_rejection_records_movement_diagnostic() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StabilityCheck2 {
                    count_before: encoder_count(1000),
                    count1: encoder_count(1001),
                    deadline_us: 0,
                    min_movement: 4,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[1001, 1001, 1001]);

            let result = step_encoder_align(
                &mut phase,
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
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].phase, PhaseId::EncoderAlign.to_u8());
            assert_eq!(events[0].code, DIAG_ENCODER_ALIGN_RESULT);
            assert_eq!(events[0].status, 1);
            assert_eq!(events[0].value0, 1);
            assert_eq!(events[0].value1, 4);
            assert_eq!(events[0].value2, 0);
        });
    }

    #[test]
    fn encoder_alignment_half_turn_stops_drive_without_committing_alignment() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StabilityCheck2 {
                    count_before: encoder_count(0),
                    count1: encoder_count(2000),
                    deadline_us: 0,
                    min_movement: 2,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 77,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[2000, 2000, 2000]);

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::EncoderFault)
            ));
            assert_eq!(adc_cal.unwrap().encoder_count, 77);
        });
    }

    #[test]
    fn calibrate_stop_drive_enters_closed_loop_with_entry_gains() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StopDrive {
                    old_count: encoder_count(42),
                    electrical_residual_counts: 2,
                    stability_counts: 1,
                    movement_counts: 25,
                    min_movement_counts: 2,
                    counts_per_electrical_rev: 80,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            let gains = CalibrateEntryGains {
                inner: crate::pid::InnerLoopGains {
                    flux_p: 711,
                    flux_i: 159,
                    torque_p: 712,
                    torque_i: 160,
                },
                velocity_p: 1434,
                velocity_i: 2,
                position_p: 627,
                position_i: 1,
                velocity_limit: 500_000,
                velocity_filter_hz: 11,
                torque_filter_hz: 13,
                position_filter_hz: 17,
                flux_filter_hz: 19,
            };

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                Some(gains),
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::ClosedLoopEntry)
            ));
            assert_eq!(adc_cal.as_ref().unwrap().encoder_count, 42);
            let InnerPhase::ClosedLoopEntry {
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
                ..
            } = phase
            else {
                panic!("calibrate StopDrive should enter ClosedLoopEntry");
            };

            assert_eq!(flux_p, gains.inner.flux_p);
            assert_eq!(flux_i, gains.inner.flux_i);
            assert_eq!(torque_p, gains.inner.torque_p);
            assert_eq!(torque_i, gains.inner.torque_i);
            assert_eq!(velocity_p, gains.velocity_p);
            assert_eq!(velocity_i, gains.velocity_i);
            assert_eq!(position_p, gains.position_p);
            assert_eq!(position_i, gains.position_i);
            assert_eq!(velocity_limit, gains.velocity_limit);
            assert_eq!(velocity_filter_hz, gains.velocity_filter_hz);
            assert_eq!(torque_filter_hz, gains.torque_filter_hz);
            assert_eq!(position_filter_hz, gains.position_filter_hz);
            assert_eq!(flux_filter_hz, gains.flux_filter_hz);
        });
    }
}
