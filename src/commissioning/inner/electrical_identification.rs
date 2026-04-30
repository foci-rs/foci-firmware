//! Phase 5: Electrical identification (R, L, transport delay).
//!
//! Per-coil resistance from steady-state current sampling, per-coil
//! inductance from a bounded square-wave dI/dt sweep, and electrical
//! transport delay from a 256-sample current transient. Assembles an
//! `ElectricalModel` and validates it before transitioning to
//! `CurrentTune` (commission flow) or emitting `SelftestDone` (selftest).

use super::{ElectricalIdData, ElectricalIdState, InnerPhase};
use crate::autotune::{
    CommissionError, ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel, InnerSequence, PhaseId,
    TransientBuffer,
};
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

/// Maximum allowed L/R tau vs transient tau mismatch (50%, in permille).
const TAU_CROSSCHECK_MAX_ERROR_PERMILLE: u32 = 500;

fn tau_crosscheck_valid(model: &ElectricalModel) -> bool {
    if model.tau_e_us == 0 || model.tau_e_crosscheck_us == 0 {
        return false;
    }
    let diff = model.tau_e_us.abs_diff(model.tau_e_crosscheck_us) as u64;
    diff * 1000 <= model.tau_e_us as u64 * TAU_CROSSCHECK_MAX_ERROR_PERMILLE as u64
}

fn sample_time_us(index: usize) -> u32 {
    crate::sample_timing::sample_elapsed_us(index, ELECTRICAL_SAMPLE_PERIOD_US as u32)
}

fn interpolate_threshold_crossing_us(samples: &[u32], threshold: u32) -> Option<u32> {
    let mut prev_sample = 0u32;
    let mut prev_time_us = 0u32;
    for (i, &sample) in samples.iter().enumerate() {
        let time_us = sample_time_us(i);
        if sample >= threshold {
            if i == 0 || sample <= prev_sample {
                return Some(time_us);
            }
            let span = sample - prev_sample;
            let above_prev = threshold.saturating_sub(prev_sample).min(span);
            let offset_us = above_prev as u64 * ELECTRICAL_SAMPLE_PERIOD_US / span as u64;
            return Some(prev_time_us + offset_us as u32);
        }
        prev_sample = sample;
        prev_time_us = time_us;
    }
    None
}

fn extrapolate_onset_to_baseline_us(samples: &[u32], threshold: u32) -> Option<u32> {
    let mut prev_sample = 0u32;
    let mut prev_time_us = 0u32;
    for (i, &sample) in samples.iter().enumerate() {
        let time_us = sample_time_us(i);
        if sample > threshold {
            if i == 0 || sample <= prev_sample {
                return Some(time_us);
            }
            let span = sample - prev_sample;
            let offset_us = prev_sample as u64 * ELECTRICAL_SAMPLE_PERIOD_US / span as u64;
            return Some(prev_time_us.saturating_sub(offset_us as u32).max(1));
        }
        prev_sample = sample;
        prev_time_us = time_us;
    }
    None
}

fn assemble_electrical_model(data: &ElectricalIdData, samples: &[u32]) -> Option<ElectricalModel> {
    let voltage = data.voltage;

    // --- Resistance ---
    // Per-coil: R_int = V / |I_avg|, stored with explicit fixed-point scale.
    let mut r_int_per_coil = [0u32; 2];
    let mut r_mohm_per_coil = [0u32; 2];
    for coil_idx in 0..2 {
        let count = data.resistance_sample_count[coil_idx] as i32;
        if count == 0 {
            return None;
        }
        let avg_current = data.resistance_current_sum[coil_idx] / count;
        let abs_current = avg_current.unsigned_abs();
        if abs_current == 0 {
            return None;
        }
        r_int_per_coil[coil_idx] =
            (voltage as u64 * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64 / abs_current as u64) as u32;
        r_mohm_per_coil[coil_idx] = (voltage as u64 * 1000 / abs_current as u64) as u32;
    }
    let r_int = (r_int_per_coil[0] + r_int_per_coil[1]) / 2;
    let r_mohm = (r_mohm_per_coil[0] + r_mohm_per_coil[1]) / 2;
    if r_int == 0 || r_mohm == 0 {
        return None;
    }

    // --- Inductance (from di/dt) ---
    // tau_us = V * dt_per_cycle_us / (avg_delta * R_int_unscaled)
    //        = V * dt_per_cycle_us * SCALE / (avg_delta * R_int_scaled)
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
        let dt_per_cycle_us = *elapsed / *count;
        *tau_result =
            (voltage as u64 * dt_per_cycle_us as u64 * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64
                / (avg_delta as u64 * r_int as u64)) as u32;
    }

    let tau_e_us = match (tau_per_coil[0] > 0, tau_per_coil[1] > 0) {
        (true, true) => (tau_per_coil[0] + tau_per_coil[1]) / 2,
        (true, false) => tau_per_coil[0],
        (false, true) => tau_per_coil[1],
        (false, false) => return None,
    };

    let l_int = (r_int as u64 * tau_e_us as u64 / ELECTRICAL_MODEL_RESISTANCE_SCALE as u64) as u32;
    let l_uh = r_mohm as u64 * tau_e_us as u64 / 1000;
    if l_int == 0 || l_uh == 0 {
        return None;
    }

    // --- Delay identification (from transient buffer) ---
    let sample_count = samples.len();
    let steady_state = if sample_count >= 20 {
        let tail_start = sample_count - 10;
        let tail_sum: u32 = samples[tail_start..].iter().sum();
        tail_sum / 10
    } else if !samples.is_empty() {
        samples[sample_count - 1]
    } else {
        return None;
    };

    let theta_e_us = extrapolate_onset_to_baseline_us(samples, DELAY_ONSET_THRESHOLD as u32)
        .unwrap_or(ELECTRICAL_SAMPLE_PERIOD_US as u32);

    let rise_threshold = (steady_state as u64 * RISE_63_PCT_Q10 as u64 / 1024) as u32;
    let rise_crossing_us = interpolate_threshold_crossing_us(samples, rise_threshold).unwrap_or(0);
    let tau_crosscheck_us = rise_crossing_us.saturating_sub(theta_e_us);

    Some(ElectricalModel {
        r_int,
        l_int,
        tau_e_us,
        theta_e_us,
        r_mohm,
        l_uh: l_uh as u32,
        tau_e_crosscheck_us: tau_crosscheck_us,
    })
}

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
            let total_cycles = backend.tmc_config().didt_cycles.max(1);
            let start = backend.elapsed_us();
            *electrical_id_state = ElectricalIdState::IndToggle {
                coil: *coil,
                total_cycles,
                remaining_cycles: total_cycles,
                prev_current: initial_current,
                start_us: start,
            };
            StepResult::Continue
        }

        ElectricalIdState::IndToggle {
            coil,
            total_cycles,
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
                // Cycle 0 starts from the zero-voltage settle state, so the
                // first delta is a zero-to-+V transition instead of a full
                // +V/-V swing. Keep didt_cycles high enough that this startup
                // transient cannot dominate the average.
                let total_done = total_cycles.saturating_sub(*remaining_cycles);
                let v = if total_done % 2 == 0 { v_pos } else { v_neg };
                if backend.write_register(UQ_UD_EXT, v).await.is_err() {
                    return super::fail_inner(phase, CommissionError::Spi);
                }

                let actual = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => actual,
                    Err(_) => return super::fail_inner(phase, CommissionError::Spi),
                };
                let current = (actual & 0xFFFF) as i16;
                let delta = (current as i32 - *prev_current as i32).unsigned_abs();
                data.inductance_sum_delta[coil_idx] += delta as u64;
                data.inductance_count[coil_idx] += 1;
                *prev_current = current;
                *remaining_cycles -= 1;
                cycles_this_step += 1;
            }

            if *remaining_cycles == 0 {
                // All cycles done. Record elapsed time.
                // Duration is bounded by INDUCTANCE_CYCLES * cycle time
                // (~200ms max), well within u32 us range.
                let end = backend.elapsed_us();
                data.inductance_elapsed_us[coil_idx] = end.wrapping_sub(*start_us) as u32;

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
            let Some(model) = assemble_electrical_model(data, buf.samples()) else {
                return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
            };
            if !tau_crosscheck_valid(&model) {
                return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
            }

            match sequence {
                InnerSequence::Selftest => {
                    selftest_results.set(
                        7,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: model.r_mohm,
                        },
                    );
                    selftest_results.set(
                        8,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: model.l_uh,
                        },
                    );
                    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::TransientBuffer;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::inner::ElectricalIdData;
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::registers::{PID_TORQUE_FLUX_ACTUAL, UQ_UD_EXT};

    fn representative_data() -> ElectricalIdData {
        let mut data = ElectricalIdData::new(512);
        data.resistance_current_sum = [300_000, 300_000];
        data.resistance_sample_count = [1000, 1000];
        data.inductance_sum_delta = [85_000, 85_000];
        data.inductance_count = [5000, 5000];
        data.inductance_elapsed_us = [490_000, 490_000];
        data
    }

    fn representative_transient() -> TransientBuffer {
        let mut buf = TransientBuffer::new();
        buf.reset(ELECTRICAL_SAMPLE_PERIOD_US as u16);
        for &sample in &[0, 10, 25, 90, 180, 260, 330, 380] {
            buf.push(sample);
        }
        for _ in 0..32 {
            buf.push(400);
        }
        buf
    }

    #[test]
    fn model_assembly_preserves_fractional_internal_resistance() {
        let data = representative_data();
        let buf = representative_transient();

        let model = assemble_electrical_model(&data, buf.samples()).unwrap();

        assert_eq!(model.r_int, 1706);
        assert!(
            (1725..=1735).contains(&model.tau_e_us),
            "tau should use R=1.706, not truncated R=1: {}",
            model.tau_e_us
        );
        assert!(
            (2940..=2960).contains(&model.l_int),
            "l_int should preserve the original internal L scale: {}",
            model.l_int
        );
    }

    #[test]
    fn model_assembly_uses_full_width_resistance_average() {
        let mut data = representative_data();
        data.resistance_current_sum = [40_000_000, 40_000_000];
        data.resistance_sample_count = [1000, 1000];
        let buf = representative_transient();

        let model = assemble_electrical_model(&data, buf.samples()).unwrap();

        assert_eq!(model.r_int, 12);
        assert_eq!(model.r_mohm, 12);
    }

    #[test]
    fn model_assembly_extrapolates_delay_onset_to_baseline() {
        let data = representative_data();
        let mut buf = TransientBuffer::new();
        buf.reset(ELECTRICAL_SAMPLE_PERIOD_US as u16);
        for &sample in &[0, 10, 25, 100, 300, 500, 650, 800] {
            buf.push(sample);
        }
        for _ in 0..12 {
            buf.push(1000);
        }

        let model = assemble_electrical_model(&data, buf.samples()).unwrap();

        assert!(
            (210..=220).contains(&model.theta_e_us),
            "theta should extrapolate the threshold crossing back to baseline: {}",
            model.theta_e_us
        );
        assert!(
            (880..=890).contains(&model.tau_e_crosscheck_us),
            "tau crosscheck should use interpolated 63 percent crossing minus onset: {}",
            model.tau_e_crosscheck_us
        );
    }

    #[test]
    fn validate_rejects_tau_crosscheck_mismatch() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::Validate,
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            buf.reset(ELECTRICAL_SAMPLE_PERIOD_US as u16);
            for &sample in &[25, 200, 700, 1000] {
                buf.push(sample);
            }
            for _ in 0..20 {
                buf.push(1000);
            }
            let mut backend = MockBackend::new();
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::ElectricalIdFailed)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::ElectricalIdFailed)
            ));
        });
    }

    #[test]
    fn inductance_phase_uses_configured_didt_cycles() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::IndSettle {
                    coil: 0,
                    deadline_us: 0,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.tmc_config.didt_cycles = 7;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            let remaining_cycles = match &phase {
                InnerPhase::ElectricalId {
                    state:
                        ElectricalIdState::IndToggle {
                            remaining_cycles, ..
                        },
                    ..
                } => *remaining_cycles,
                other => panic!("expected IndToggle, got {other:?}"),
            };
            assert_eq!(remaining_cycles, 7);

            backend.spi_log.clear();
            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            let mut uq_ud_writes = backend.spi_log.iter().filter_map(|op| match op {
                SpiOp::Write(addr, val) if *addr == UQ_UD_EXT.addr() => Some(*val),
                _ => None,
            });
            assert_eq!(uq_ud_writes.next(), Some(512));
            assert_eq!(uq_ud_writes.next(), Some(0xFE00));
            assert_eq!(uq_ud_writes.next(), Some(512));
        });
    }

    #[test]
    fn inductance_elapsed_time_handles_timer_wrap() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::IndToggle {
                    coil: 0,
                    total_cycles: 1,
                    remaining_cycles: 1,
                    prev_current: 0,
                    start_us: u64::MAX - 10,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 20;
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 10)
                .ok();
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            match &phase {
                InnerPhase::ElectricalId {
                    state: ElectricalIdState::IndDisable { .. },
                    data,
                } => {
                    assert_eq!(data.inductance_elapsed_us[0], 31);
                    assert_eq!(data.inductance_count[0], 1);
                }
                other => panic!("expected IndDisable, got {other:?}"),
            }
        });
    }

    #[test]
    fn inductance_toggle_write_error_fails_inner_phase() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::IndToggle {
                    coil: 0,
                    total_cycles: 7,
                    remaining_cycles: 7,
                    prev_current: 0,
                    start_us: 0,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(UQ_UD_EXT.addr());
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, InnerPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn inductance_toggle_read_error_fails_inner_phase() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::IndToggle {
                    coil: 0,
                    total_cycles: 7,
                    remaining_cycles: 7,
                    prev_current: 0,
                    start_us: 0,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.fail_on_read = Some(PID_TORQUE_FLUX_ACTUAL.addr());
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, InnerPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn step_inner_restores_safe_state_on_inner_failure() {
        pollster::block_on(async {
            // Drive through `step_inner` (the un-bypassable cleanup boundary)
            // so an SPI write failure mid-toggle leaves the chip de-energized
            // and the canonical safe-state register sequence applied.
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::IndToggle {
                    coil: 0,
                    total_cycles: 7,
                    remaining_cycles: 7,
                    prev_current: 0,
                    start_us: 0,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.fail_on_write = Some(UQ_UD_EXT.addr());
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut adc_cal = None;

            let result = crate::commissioning::inner::step_inner(
                &mut phase,
                &mut buf,
                crate::autotune::CommissionProfile::Balanced,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(!backend.enable_pin, "enable pin must be lowered");
            // The canonical safe-state writes must follow the failing UQ_UD_EXT
            // write. Collect the addresses written *after* the failure point.
            use foci_tmc4671::registers::*;
            let mut seen_failure = false;
            let mut after: std::vec::Vec<u8> = std::vec::Vec::new();
            for op in backend.spi_log.iter() {
                if let SpiOp::Write(addr, _) = op {
                    if !seen_failure && *addr == UQ_UD_EXT.addr() {
                        seen_failure = true;
                        continue;
                    }
                    if seen_failure {
                        after.push(*addr);
                    }
                }
            }
            for canonical in [
                MODE_RAMP_MODE_MOTION.addr(),
                PID_TORQUE_FLUX_TARGET.addr(),
                PWM_SV_CHOP.addr(),
                PID_VELOCITY_OFFSET.addr(),
                PIDOUT_UQ_UD_LIMITS.addr(),
            ] {
                assert!(
                    after.contains(&canonical),
                    "canonical safe-state register {:#x} not written after failure",
                    canonical
                );
            }
        });
    }
}
