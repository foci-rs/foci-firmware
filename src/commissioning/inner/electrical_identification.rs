//! Phase 5: Electrical identification (R, L, transport delay).
//!
//! Per-coil resistance from steady-state current sampling, per-coil
//! inductance from a bounded square-wave dI/dt sweep, and electrical
//! transport delay from a 256-sample current transient. Assembles an
//! `ElectricalModel` and validates it before transitioning to
//! `CurrentTune` (commission flow) or emitting `SelftestDone` (selftest).

use super::{ElectricalIdState, InnerPhase};
use crate::autotune::{CommissionError, ElectricalModel, InnerSequence, PhaseId, TransientBuffer};
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Number of steady-state current samples for resistance measurement.
const RESISTANCE_SAMPLE_COUNT: u16 = 1000;

/// Sampling interval for resistance and transient captures (microseconds).
/// Matches PWM rate (~160us at 25 kHz) for Nyquist-rate current sampling.
///
/// Re-exported for the validation phase, which captures step responses at
/// the same cadence.
pub(crate) const ELECTRICAL_SAMPLE_PERIOD_US: u64 = 160;

/// Settling time after applying excitation voltage (microseconds).
const RESISTANCE_SETTLE_US: u64 = 100_000;

/// Cooldown time after disabling a per-coil measurement (microseconds).
const ELECTRICAL_COOLDOWN_US: u64 = 50_000;

/// Brief settle before starting di/dt toggling (microseconds).
const INDUCTANCE_PRE_SETTLE_US: u64 = 10_000;

/// Maximum di/dt toggle cycles per step() call.
/// Each cycle is ~2 SPI transfers (~22us at 8 MHz SPI).
/// Budget: ~30 cycles/step ≈ 660us, leaving margin within 1ms tick.
const INDUCTANCE_CYCLES_PER_STEP: u32 = 30;

/// Minimum average |delta_I| per di/dt cycle to consider inductance valid.
/// Below this, the current change is too small to extract L reliably.
const INDUCTANCE_MIN_DELTA: u32 = 5;

/// Number of transient samples for delay identification.
/// Matches TransientBuffer capacity. Re-exported because the current-loop
/// validation phase uses the same buffer length cap.
pub(crate) const DELAY_SAMPLE_COUNT: u16 = 256;

/// Minimum detectable current for delay onset (ADC units above baseline).
/// Must be above noise floor but sensitive enough to detect delay.
const DELAY_ONSET_THRESHOLD: u16 = 20;

/// Fraction of steady-state current for 63% rise detection (fixed-point).
/// 0.632 * 1024 ≈ 647 (Q10 format for integer math).
const RISE_63_PCT_Q10: u32 = 647;

/// Advance electrical identification sub-state.
///
/// Measures per-coil resistance (1000 samples at 160us in steady
/// state), per-coil inductance (square-wave dI/dt), and delay
/// (transient capture with two-point fit). Assembles an
/// `ElectricalModel` and validates before transitioning.
pub(crate) async fn step_electrical_identification<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    buf: &mut TransientBuffer,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (electrical_id_state, data) = match phase {
        InnerPhase::ElectricalId { state, data } => (state, data),
        _ => return StepResult::Continue,
    };

    match electrical_id_state {
        // ---------------------------------------------------------------
        // Resistance measurement (per coil)
        // ---------------------------------------------------------------
        ElectricalIdState::ResSetup { coil } => {
            let coil_idx = *coil;
            // Configure open-loop voltage mode.
            if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            let phi_e = if coil_idx == 0 { 0u32 } else { 16384 };
            if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            if backend
                .write_register(UQ_UD_EXT, data.voltage & 0xFFFF)
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

            let deadline = backend.elapsed_us().wrapping_add(RESISTANCE_SETTLE_US);
            *electrical_id_state = ElectricalIdState::ResSettle {
                coil: coil_idx,
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        ElectricalIdState::ResSettle { coil, deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                let next = backend
                    .elapsed_us()
                    .wrapping_add(ELECTRICAL_SAMPLE_PERIOD_US);
                *electrical_id_state = ElectricalIdState::ResSample {
                    coil: *coil,
                    next_sample_us: next,
                };
            }
            StepResult::Continue
        }

        ElectricalIdState::ResSample {
            coil,
            next_sample_us,
        } => {
            let coil_idx = *coil as usize;
            // Take as many samples as fit within this step's time budget.
            loop {
                if data.resistance_sample_count[coil_idx] >= RESISTANCE_SAMPLE_COUNT {
                    // All samples collected. Disable motor.
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    let _ = backend.write_register(UQ_UD_EXT, 0).await;
                    backend.set_enable_pin(false);
                    let deadline = backend.elapsed_us().wrapping_add(ELECTRICAL_COOLDOWN_US);
                    *electrical_id_state = ElectricalIdState::ResDisable {
                        coil: *coil,
                        deadline_us: deadline,
                    };
                    return StepResult::Continue;
                }
                // Wait for next sample deadline.
                if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    break; // Not yet time, yield.
                }
                // Take one sample.
                match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => {
                        let current = (actual & 0xFFFF) as i16;
                        data.resistance_current_sum[coil_idx] += current as i32;
                        data.resistance_sample_count[coil_idx] += 1;
                    }
                    Err(_) => {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                }
                *next_sample_us = next_sample_us.wrapping_add(ELECTRICAL_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        ElectricalIdState::ResDisable { coil, deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            let coil_idx = *coil;
            if coil_idx == 0 {
                // Move to coil B resistance.
                *electrical_id_state = ElectricalIdState::ResSetup { coil: 1 };
            } else {
                // Both resistance measurements done. Start inductance.
                *electrical_id_state = ElectricalIdState::IndSetup { coil: 0 };
            }
            StepResult::Continue
        }

        // ---------------------------------------------------------------
        // Inductance measurement (per coil, square-wave dI/dt)
        // ---------------------------------------------------------------
        ElectricalIdState::IndSetup { coil } => {
            let coil_idx = *coil;
            if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            let phi_e = if coil_idx == 0 { 0u32 } else { 16384 };
            if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            // Start at zero voltage.
            if backend.write_register(UQ_UD_EXT, 0).await.is_err() {
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

            let deadline = backend.elapsed_us().wrapping_add(INDUCTANCE_PRE_SETTLE_US);
            *electrical_id_state = ElectricalIdState::IndSettle {
                coil: coil_idx,
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        ElectricalIdState::IndSettle { coil, deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            // Read initial current before toggling.
            let initial_current = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(actual) => (actual & 0xFFFF) as i16,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let total_cycles = 5000u32; // From TmcConfig::didt_cycles default
            let start = backend.elapsed_us();
            *electrical_id_state = ElectricalIdState::IndToggle {
                coil: *coil,
                remaining_cycles: total_cycles,
                prev_current: initial_current,
                start_us: start,
            };
            StepResult::Continue
        }

        ElectricalIdState::IndToggle {
            coil,
            remaining_cycles,
            prev_current,
            start_us,
        } => {
            let coil_idx = *coil as usize;
            let voltage = data.voltage;
            let v_pos = voltage & 0xFFFF;
            let v_neg = (-(voltage as i32) as u32) & 0xFFFF;

            let mut cycles_this_step = 0u32;

            while *remaining_cycles > 0 && cycles_this_step < INDUCTANCE_CYCLES_PER_STEP {
                // Determine polarity based on which cycle we're on.
                let total_done = 5000u32.saturating_sub(*remaining_cycles);
                let v = if total_done % 2 == 0 { v_pos } else { v_neg };
                let _ = backend.write_register(UQ_UD_EXT, v).await;

                if let Ok(actual) = backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    let current = (actual & 0xFFFF) as i16;
                    let delta = (current as i32 - *prev_current as i32).unsigned_abs();
                    data.inductance_sum_delta[coil_idx] += delta as u64;
                    data.inductance_count[coil_idx] += 1;
                    *prev_current = current;
                }
                *remaining_cycles -= 1;
                cycles_this_step += 1;
            }

            if *remaining_cycles == 0 {
                // All cycles done. Record elapsed time.
                // Duration is bounded by INDUCTANCE_CYCLES * cycle time
                // (~200ms max), well within u32 us range.
                let end = backend.elapsed_us();
                data.inductance_elapsed_us[coil_idx] = (end - *start_us) as u32;

                // Disable motor.
                let _ = backend.write_register(UQ_UD_EXT, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                backend.set_enable_pin(false);

                let deadline = backend.elapsed_us().wrapping_add(ELECTRICAL_COOLDOWN_US);
                *electrical_id_state = ElectricalIdState::IndDisable {
                    coil: *coil,
                    deadline_us: deadline,
                };
            }
            StepResult::Continue
        }

        ElectricalIdState::IndDisable { coil, deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            let coil_idx = *coil;
            if coil_idx == 0 {
                // Move to coil B inductance.
                *electrical_id_state = ElectricalIdState::IndSetup { coil: 1 };
            } else {
                // Both inductance measurements done. Start delay capture.
                *electrical_id_state = ElectricalIdState::DelaySetup;
            }
            StepResult::Continue
        }

        // ---------------------------------------------------------------
        // Delay identification (transient capture)
        // ---------------------------------------------------------------
        ElectricalIdState::DelaySetup => {
            // Configure for step response on coil A (D-axis).
            if backend.write_register(PHI_E_SELECTION, 1).await.is_err()
                || backend.write_register(PHI_E_EXT, 0).await.is_err()
                || backend.write_register(UQ_UD_EXT, 0).await.is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, 8)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            backend.set_enable_pin(true);

            // Reset transient buffer for 160us sampling.
            buf.reset(ELECTRICAL_SAMPLE_PERIOD_US as u16);

            let deadline = backend.elapsed_us().wrapping_add(INDUCTANCE_PRE_SETTLE_US);
            *electrical_id_state = ElectricalIdState::DelaySettle {
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        ElectricalIdState::DelaySettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            *electrical_id_state = ElectricalIdState::DelayStep;
            StepResult::Continue
        }

        ElectricalIdState::DelayStep => {
            // Read baseline current (should be ~0 with UQ_UD_EXT=0).
            let baseline = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(actual) => (actual & 0xFFFF) as i16,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };

            // Apply voltage step.
            if backend
                .write_register(UQ_UD_EXT, data.voltage & 0xFFFF)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let next = backend
                .elapsed_us()
                .wrapping_add(ELECTRICAL_SAMPLE_PERIOD_US);
            *electrical_id_state = ElectricalIdState::DelaySample {
                baseline,
                next_sample_us: next,
            };
            StepResult::Continue
        }

        ElectricalIdState::DelaySample {
            baseline,
            next_sample_us,
        } => {
            // Capture samples into TransientBuffer at 160us intervals.
            loop {
                if buf.len() >= DELAY_SAMPLE_COUNT || buf.is_full() {
                    // Capture complete. Disable motor.
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    let _ = backend.write_register(UQ_UD_EXT, 0).await;
                    backend.set_enable_pin(false);
                    let deadline = backend.elapsed_us().wrapping_add(ELECTRICAL_COOLDOWN_US);
                    *electrical_id_state = ElectricalIdState::DelayDisable {
                        deadline_us: deadline,
                    };
                    return StepResult::Continue;
                }
                if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    break; // Not yet time, yield.
                }
                match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => {
                        // Store absolute current relative to baseline.
                        let current = (actual & 0xFFFF) as i16;
                        let relative = (current as i32 - *baseline as i32).unsigned_abs();
                        buf.push(relative);
                    }
                    Err(_) => {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                }
                *next_sample_us = next_sample_us.wrapping_add(ELECTRICAL_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        ElectricalIdState::DelayDisable { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            *electrical_id_state = ElectricalIdState::Validate;
            StepResult::Continue
        }

        // ---------------------------------------------------------------
        // Model assembly and validation
        // ---------------------------------------------------------------
        ElectricalIdState::Validate => {
            let voltage = data.voltage;

            // --- Resistance ---
            // Per-coil: R_int = V / |I_avg|
            let mut r_int_per_coil = [0u32; 2];
            let mut r_mohm_per_coil = [0u32; 2];
            for coil_idx in 0..2 {
                let count = data.resistance_sample_count[coil_idx] as i32;
                if count == 0 {
                    return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }
                let avg_current = (data.resistance_current_sum[coil_idx] / count) as i16;
                let abs_current = avg_current.unsigned_abs() as u32;
                if abs_current == 0 {
                    return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }
                r_int_per_coil[coil_idx] = voltage / abs_current;
                r_mohm_per_coil[coil_idx] = (voltage as u64 * 1000 / abs_current as u64) as u32;
            }
            // Average of both coils.
            let r_int = (r_int_per_coil[0] + r_int_per_coil[1]) / 2;
            let r_mohm = (r_mohm_per_coil[0] + r_mohm_per_coil[1]) / 2;
            if r_int == 0 {
                return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
            }

            // --- Inductance (from di/dt) ---
            // L_int = V * total_time_s / sum_delta
            // tau_e_us = L_int / R_int * 1e6
            // Since L_int = V * elapsed_us * 1e-6 / (sum_delta / count):
            //   L_int = V * elapsed_us * count / (sum_delta * 1e6)
            // And tau_e_us = L_int / R_int * 1e6:
            //   tau_e_us = V * elapsed_us * count / (sum_delta * R_int)
            let mut tau_per_coil = [0u32; 2];
            for (tau_result, ((count, sum_delta), elapsed)) in tau_per_coil
                .iter_mut()
                .zip(
                    data.inductance_count
                        .iter()
                        .zip(data.inductance_sum_delta.iter()),
                )
                .zip(data.inductance_elapsed_us.iter())
                .map(|((t, (c, s)), e)| (t, ((c, s), e)))
            {
                if *count == 0 || *sum_delta == 0 || *elapsed == 0 {
                    continue;
                }
                let avg_delta = (*sum_delta / *count as u64) as u32;
                if avg_delta < INDUCTANCE_MIN_DELTA {
                    continue;
                }
                // dt_per_cycle_us = elapsed / count
                // L_int = V * dt_per_cycle_us * 1e-6 / avg_delta
                // tau_us = L_int / R_int * 1e6 = V * dt_per_cycle_us / (avg_delta * R_int)
                let dt_per_cycle_us = *elapsed / *count;
                *tau_result = (voltage as u64 * dt_per_cycle_us as u64
                    / (avg_delta as u64 * r_int as u64)) as u32;
            }

            // Use the valid coil(s) for tau.
            let tau_e_us = match (tau_per_coil[0] > 0, tau_per_coil[1] > 0) {
                (true, true) => (tau_per_coil[0] + tau_per_coil[1]) / 2,
                (true, false) => tau_per_coil[0],
                (false, true) => tau_per_coil[1],
                (false, false) => {
                    return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }
            };

            // L_int from tau: L_int = R_int * tau_e_us (internal units,
            // such that tau_e_us = L_int / R_int holds directly).
            let l_int = r_int * tau_e_us;

            // Physical L: L_uh = R_mohm * tau_e_us / 1000
            // (since L = R*tau; mohm*us = nH; nH/1000 = uH)
            let l_uh = r_mohm as u64 * tau_e_us as u64 / 1000;

            if l_int == 0 || l_uh == 0 {
                return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
            }

            // --- Delay identification (from transient buffer) ---
            let samples = buf.samples();
            let sample_count = samples.len();

            // Find steady-state from last 10 samples.
            let steady_state = if sample_count >= 20 {
                let tail_start = sample_count - 10;
                let tail_sum: u32 = samples[tail_start..].iter().sum();
                tail_sum / 10
            } else if !samples.is_empty() {
                // Short capture: use last sample as approximation.
                samples[sample_count - 1]
            } else {
                return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
            };

            // Onset: first sample above noise threshold.
            // Result values are bounded by samples.len() * sample period
            // (< 256 * 160us = 41ms), well within u32 us range.
            let onset_threshold = DELAY_ONSET_THRESHOLD as u32;
            let mut theta_e_us = 0u32;
            for (i, &sample) in samples.iter().enumerate() {
                if sample > onset_threshold {
                    theta_e_us = (i as u64 * ELECTRICAL_SAMPLE_PERIOD_US) as u32;
                    break;
                }
            }

            // 63% rise: first sample >= 0.632 * steady_state.
            let rise_threshold = (steady_state as u64 * RISE_63_PCT_Q10 as u64 / 1024) as u32;
            let mut tau_crosscheck_us = 0u32;
            for (i, &sample) in samples.iter().enumerate() {
                if sample >= rise_threshold {
                    tau_crosscheck_us = (i as u64 * ELECTRICAL_SAMPLE_PERIOD_US) as u32;
                    break;
                }
            }

            // Default theta_e to at least one sample period if zero.
            if theta_e_us == 0 {
                theta_e_us = ELECTRICAL_SAMPLE_PERIOD_US as u32;
            }

            // --- Assemble ElectricalModel ---
            let model = ElectricalModel {
                r_int,
                l_int,
                tau_e_us,
                theta_e_us,
                r_mohm,
                l_uh: l_uh as u32,
                tau_e_crosscheck_us: tau_crosscheck_us,
            };

            match sequence {
                InnerSequence::Selftest => {
                    selftest_results.set(
                        7,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: r_mohm,
                        },
                    );
                    selftest_results.set(
                        8,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: l_uh as u32,
                        },
                    );
                    // Full safe-state restore matching restore_safe_state().
                    backend.set_enable_pin(false);
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                    let _ = backend.write_register(UQ_UD_EXT, 0).await;
                    let _ = backend.write_register(PWM_SV_CHOP, 0).await;
                    let _ = backend
                        .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, 0)
                        .await;
                    StepResult::SelftestDone
                }
                InnerSequence::Commission => {
                    // Transition to CurrentTune carrying the model.
                    *phase = InnerPhase::CurrentTune { model };
                    StepResult::PhaseChanged(PhaseId::CurrentTune)
                }
                InnerSequence::Calibrate => {
                    // Calibrate never reaches electrical ID.
                    unreachable!()
                }
            }
        }
    }
}
