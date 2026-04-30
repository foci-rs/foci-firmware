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

// ---------------------------------------------------------------------------
// Phase 1 model-assembly gates and warning bits.
// ---------------------------------------------------------------------------

/// Per-coil resistance disagreement >= 5 % (warning).
pub(crate) const INNER_WARN_BIT_RESISTANCE_MISMATCH: u8 = 1 << 0;
/// Per-coil tau/L disagreement >= 15 % (warning).
pub(crate) const INNER_WARN_BIT_TAU_MISMATCH: u8 = 1 << 1;
/// L/R tau vs transient tau residual >= 25 % (warning).
pub(crate) const INNER_WARN_BIT_TAU_RESIDUAL: u8 = 1 << 2;
/// theta_e_us >= tau_e_us / 4 (warning).
pub(crate) const INNER_WARN_BIT_THETA_TAU: u8 = 1 << 3;

/// Resistance below this is treated as a near-short.
const RESISTANCE_SHORT_MOHM: u32 = 100;
/// Resistance above this is treated as open-load or invalid scaling.
const RESISTANCE_OPEN_MOHM: u32 = 50_000;
/// Per-coil resistance disagreement fail threshold (permille).
const RESISTANCE_MISMATCH_FAIL_PERMILLE: u32 = 150;
/// Per-coil resistance disagreement warn threshold (permille).
const RESISTANCE_MISMATCH_WARN_PERMILLE: u32 = 50;
/// Per-coil tau disagreement fail threshold (permille).
const TAU_MISMATCH_FAIL_PERMILLE: u32 = 300;
/// Per-coil tau disagreement warn threshold (permille).
const TAU_MISMATCH_WARN_PERMILLE: u32 = 150;
/// L/R tau vs transient tau residual fail threshold (permille).
const TAU_RESIDUAL_FAIL_PERMILLE: u32 = 500;
/// L/R tau vs transient tau residual warn threshold (permille).
const TAU_RESIDUAL_WARN_PERMILLE: u32 = 250;

/// Successful model assembly: the assembled `ElectricalModel`, the
/// non-critical warning bitfield (bits 0–3 from the gates), and the
/// computed L/R-vs-transient residual in permille for the host to
/// persist.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct ModelAssembly {
    pub(crate) model: ElectricalModel,
    pub(crate) warning_flags: u8,
    pub(crate) tau_residual_permille: u16,
}

/// Reason a model assembly was rejected. Each variant logs distinct
/// `defmt::warn!` text and maps to `CommissionError::ElectricalIdFailed`
/// at the phase boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum ModelAssemblyError {
    /// At least one coil had zero samples or zero current.
    SampleCount,
    /// Both coils must produce non-zero tau; at least one was zero.
    SingleCoilTau,
    /// L_int or L_uh evaluated to zero after the per-coil tau computation.
    /// Distinct from `SampleCount` so RTT diagnostics name the actual
    /// failure (likely a fixed-point scale or rounding edge case).
    ModelScaleZero { l_int: u32, l_uh: u32 },
    /// Averaged resistance below short-circuit threshold.
    ResistanceShort { r_mohm: u32 },
    /// Averaged resistance above open-load threshold.
    ResistanceOpen { r_mohm: u32 },
    /// Per-coil resistance disagreement above hard threshold.
    ResistanceMismatch { permille: u32 },
    /// Per-coil tau disagreement above hard threshold.
    TauMismatch { permille: u32 },
    /// Transient capture produced no usable rise crossing, so the
    /// crosscheck tau is undefined. Distinct from a non-zero-but-bad
    /// residual so diagnostics can distinguish "didn't measure" from
    /// "measured wrong".
    TauCrosscheckUnmeasurable,
    /// L/R tau vs transient tau residual above hard threshold.
    TauResidual { permille: u32 },
    /// theta_e_us >= tau_e_us / 2.
    ThetaTooLarge { theta_us: u32, tau_us: u32 },
}

fn log_model_assembly_failure(err: ModelAssemblyError) {
    match err {
        ModelAssemblyError::SampleCount => {
            defmt::warn!("electrical id: no usable per-coil samples");
        }
        ModelAssemblyError::SingleCoilTau => {
            defmt::warn!("electrical id: only one coil produced non-zero tau");
        }
        ModelAssemblyError::ModelScaleZero { l_int, l_uh } => {
            defmt::warn!(
                "electrical id: model scale zero (l_int={}, l_uh={})",
                l_int,
                l_uh
            );
        }
        ModelAssemblyError::ResistanceShort { r_mohm } => {
            defmt::warn!("electrical id: r_mohm={} below short threshold", r_mohm);
        }
        ModelAssemblyError::ResistanceOpen { r_mohm } => {
            defmt::warn!("electrical id: r_mohm={} above open threshold", r_mohm);
        }
        ModelAssemblyError::ResistanceMismatch { permille } => {
            defmt::warn!("electrical id: coil R mismatch {} permille", permille);
        }
        ModelAssemblyError::TauMismatch { permille } => {
            defmt::warn!("electrical id: coil tau mismatch {} permille", permille);
        }
        ModelAssemblyError::TauCrosscheckUnmeasurable => {
            defmt::warn!("electrical id: tau crosscheck unmeasurable");
        }
        ModelAssemblyError::TauResidual { permille } => {
            defmt::warn!("electrical id: tau residual {} permille", permille);
        }
        ModelAssemblyError::ThetaTooLarge { theta_us, tau_us } => {
            defmt::warn!(
                "electrical id: theta={}us >= tau={}us / 2",
                theta_us,
                tau_us
            );
        }
    }
}

fn permille_diff(a: u32, b: u32) -> u32 {
    let larger = a.max(b);
    if larger == 0 {
        return 0;
    }
    let diff = a.abs_diff(b) as u64;
    (diff * 1000 / larger as u64) as u32
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

fn assemble_electrical_model(
    data: &ElectricalIdData,
    samples: &[u32],
) -> Result<ModelAssembly, ModelAssemblyError> {
    let voltage = data.voltage;

    // --- Per-coil resistance ---
    let mut r_int_per_coil = [0u32; 2];
    let mut r_mohm_per_coil = [0u32; 2];
    for coil_idx in 0..2 {
        let count = data.resistance_sample_count[coil_idx] as i32;
        if count == 0 {
            return Err(ModelAssemblyError::SampleCount);
        }
        let avg_current = data.resistance_current_sum[coil_idx] / count;
        let abs_current = avg_current.unsigned_abs();
        if abs_current == 0 {
            return Err(ModelAssemblyError::SampleCount);
        }
        r_int_per_coil[coil_idx] =
            (voltage as u64 * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64 / abs_current as u64) as u32;
        r_mohm_per_coil[coil_idx] = (voltage as u64 * 1000 / abs_current as u64) as u32;
    }
    let r_int = (r_int_per_coil[0] + r_int_per_coil[1]) / 2;
    let r_mohm = (r_mohm_per_coil[0] + r_mohm_per_coil[1]) / 2;

    // --- Per-coil tau (both coils must produce non-zero tau) ---
    //
    // Each coil's tau uses its OWN r_int_per_coil entry, not the averaged
    // r_int. Averaging here would mask coil-asymmetric R/L behavior: two
    // coils with identical L but different R would report identical tau
    // and the per-coil tau-mismatch gate would never fire. The per-coil
    // R disagreement gate independently catches resistance imbalance.
    let mut tau_per_coil = [0u32; 2];
    for (coil_idx, tau_slot) in tau_per_coil.iter_mut().enumerate() {
        let count = data.inductance_count[coil_idx];
        let sum_delta = data.inductance_sum_delta[coil_idx];
        let elapsed = data.inductance_elapsed_us[coil_idx];
        if count == 0 || sum_delta == 0 || elapsed == 0 {
            continue;
        }
        let avg_delta = (sum_delta / count as u64) as u32;
        if avg_delta < INDUCTANCE_MIN_DELTA {
            continue;
        }
        let dt_per_cycle_us = elapsed / count;
        let r_int_coil = r_int_per_coil[coil_idx].max(1);
        *tau_slot =
            (voltage as u64 * dt_per_cycle_us as u64 * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64
                / (avg_delta as u64 * r_int_coil as u64)) as u32;
    }
    if tau_per_coil[0] == 0 || tau_per_coil[1] == 0 {
        return Err(ModelAssemblyError::SingleCoilTau);
    }

    // --- Hard gates (in spec order) ---
    if r_mohm < RESISTANCE_SHORT_MOHM {
        return Err(ModelAssemblyError::ResistanceShort { r_mohm });
    }
    if r_mohm > RESISTANCE_OPEN_MOHM {
        return Err(ModelAssemblyError::ResistanceOpen { r_mohm });
    }
    let r_diff_permille = permille_diff(r_mohm_per_coil[0], r_mohm_per_coil[1]);
    if r_diff_permille >= RESISTANCE_MISMATCH_FAIL_PERMILLE {
        return Err(ModelAssemblyError::ResistanceMismatch {
            permille: r_diff_permille,
        });
    }
    let tau_diff_permille = permille_diff(tau_per_coil[0], tau_per_coil[1]);
    if tau_diff_permille >= TAU_MISMATCH_FAIL_PERMILLE {
        return Err(ModelAssemblyError::TauMismatch {
            permille: tau_diff_permille,
        });
    }

    let tau_e_us = (tau_per_coil[0] + tau_per_coil[1]) / 2;
    let l_int = (r_int as u64 * tau_e_us as u64 / ELECTRICAL_MODEL_RESISTANCE_SCALE as u64) as u32;
    let l_uh = r_mohm as u64 * tau_e_us as u64 / 1000;
    if l_int == 0 || l_uh == 0 {
        return Err(ModelAssemblyError::ModelScaleZero {
            l_int,
            l_uh: l_uh as u32,
        });
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
        return Err(ModelAssemblyError::SampleCount);
    };

    let theta_e_us = extrapolate_onset_to_baseline_us(samples, DELAY_ONSET_THRESHOLD as u32)
        .unwrap_or(ELECTRICAL_SAMPLE_PERIOD_US as u32);

    let rise_threshold = (steady_state as u64 * RISE_63_PCT_Q10 as u64 / 1024) as u32;
    let rise_crossing_us = interpolate_threshold_crossing_us(samples, rise_threshold).unwrap_or(0);
    let tau_e_crosscheck_us = rise_crossing_us.saturating_sub(theta_e_us);

    if tau_e_crosscheck_us == 0 {
        return Err(ModelAssemblyError::TauCrosscheckUnmeasurable);
    }
    // tau_e_us is guaranteed non-zero here (passed SingleCoilTau).
    let diff = tau_e_us.abs_diff(tau_e_crosscheck_us) as u64;
    let tau_residual_permille = ((diff * 1000 / tau_e_us as u64) as u32).min(1000);
    if tau_residual_permille >= TAU_RESIDUAL_FAIL_PERMILLE {
        return Err(ModelAssemblyError::TauResidual {
            permille: tau_residual_permille,
        });
    }
    if tau_e_us > 0 && theta_e_us >= tau_e_us / 2 {
        return Err(ModelAssemblyError::ThetaTooLarge {
            theta_us: theta_e_us,
            tau_us: tau_e_us,
        });
    }

    // --- Warning flags (same evaluation order as hard gates) ---
    let mut warning_flags: u8 = 0;
    if r_diff_permille >= RESISTANCE_MISMATCH_WARN_PERMILLE {
        warning_flags |= INNER_WARN_BIT_RESISTANCE_MISMATCH;
    }
    if tau_diff_permille >= TAU_MISMATCH_WARN_PERMILLE {
        warning_flags |= INNER_WARN_BIT_TAU_MISMATCH;
    }
    if tau_residual_permille >= TAU_RESIDUAL_WARN_PERMILLE {
        warning_flags |= INNER_WARN_BIT_TAU_RESIDUAL;
    }
    if tau_e_us > 0 && theta_e_us >= tau_e_us / 4 {
        warning_flags |= INNER_WARN_BIT_THETA_TAU;
    }

    Ok(ModelAssembly {
        model: ElectricalModel {
            r_int,
            l_int,
            tau_e_us,
            theta_e_us,
            r_mohm,
            l_uh: l_uh as u32,
            tau_e_crosscheck_us,
        },
        warning_flags,
        tau_residual_permille: tau_residual_permille as u16,
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
                *electrical_id_state = ElectricalIdState::InductanceSetup { coil: 0 };
            }
            StepResult::Continue
        }

        // ---------------------------------------------------------------
        // Inductance measurement (per coil, square-wave dI/dt)
        // ---------------------------------------------------------------
        ElectricalIdState::InductanceSetup { coil } => {
            // Bound-check `didt_cycles` before any energizing register write.
            // Phase 1 policy: warmup_cycles = max(8, didt_cycles / 20),
            // measured_cycles = didt_cycles - warmup_cycles. Reject if the
            // measured window would be too short to reduce noise, and reject
            // configurations that would energize the dI/dt sweep beyond ~1.1s.
            let didt_cycles = backend.tmc_config().didt_cycles;
            if didt_cycles > 50_000 {
                defmt::warn!("electrical id: didt_cycles={} > 50000 cap", didt_cycles);
                return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
            }
            let warmup_cycles = core::cmp::max(8, didt_cycles / 20);
            let measured_cycles = didt_cycles.saturating_sub(warmup_cycles);
            if measured_cycles < 64 {
                defmt::warn!(
                    "electrical id: didt_cycles={} leaves {} measured (need >=64)",
                    didt_cycles,
                    measured_cycles
                );
                return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
            }

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
            *electrical_id_state = ElectricalIdState::InductanceSettle {
                coil: coil_idx,
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        ElectricalIdState::InductanceSettle { coil, deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            // Read initial current before toggling.
            let initial_current = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(actual) => (actual & 0xFFFF) as i16,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            // Re-read `didt_cycles` here rather than threading it through
            // the state enum. The `CommissioningBackend` trait does not
            // promise `tmc_config()` immutability, but `TmcConfig` is
            // constructed once at engine creation and never mutated by the
            // commissioning code — this read returns the same value
            // `InductanceSetup` validated. The `.max(1)` keeps the
            // arithmetic well-defined even on an unexpected mutation.
            let didt_cycles = backend.tmc_config().didt_cycles.max(1);
            let warmup_cycles = core::cmp::max(8, didt_cycles / 20);
            let start = backend.elapsed_us();
            *electrical_id_state = ElectricalIdState::InductanceToggle {
                coil: *coil,
                total_cycles: didt_cycles,
                remaining_cycles: didt_cycles,
                prev_current: initial_current,
                start_us: start,
                warmup_cycles_remaining: warmup_cycles,
                accumulation_start_us: None,
            };
            StepResult::Continue
        }

        ElectricalIdState::InductanceToggle {
            coil,
            total_cycles,
            remaining_cycles,
            prev_current,
            start_us,
            warmup_cycles_remaining,
            accumulation_start_us,
        } => {
            let coil_idx = *coil as usize;
            let voltage = data.voltage;
            let v_pos = voltage & 0xFFFF;
            let v_neg = (-(voltage as i32) as u32) & 0xFFFF;

            let mut cycles_this_step = 0u32;

            while *remaining_cycles > 0 && cycles_this_step < INDUCTANCE_CYCLES_PER_STEP {
                // Polarity alternates by absolute cycle number so the warm-up
                // window still sees the same +V/-V toggle pattern that the
                // accumulation window relies on.
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
                if *warmup_cycles_remaining > 0 {
                    *warmup_cycles_remaining -= 1;
                    if *warmup_cycles_remaining == 0 {
                        // Next iteration is the first measured cycle.
                        *accumulation_start_us = Some(backend.elapsed_us());
                    }
                } else {
                    let delta = (current as i32 - *prev_current as i32).unsigned_abs();
                    data.inductance_sum_delta[coil_idx] += delta as u64;
                    data.inductance_count[coil_idx] += 1;
                }
                *prev_current = current;
                *remaining_cycles -= 1;
                cycles_this_step += 1;
            }

            if *remaining_cycles == 0 {
                // All cycles done. Elapsed is measured from the first
                // accumulated cycle (warm-up excluded).
                let end = backend.elapsed_us();
                let start = accumulation_start_us.unwrap_or(*start_us);
                data.inductance_elapsed_us[coil_idx] = end.wrapping_sub(start) as u32;

                // Disable motor.
                let _ = backend.write_register(UQ_UD_EXT, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                backend.set_enable_pin(false);

                let deadline = backend.elapsed_us().wrapping_add(ELECTRICAL_COOLDOWN_US);
                *electrical_id_state = ElectricalIdState::InductanceDisable {
                    coil: *coil,
                    deadline_us: deadline,
                };
            }
            StepResult::Continue
        }

        ElectricalIdState::InductanceDisable { coil, deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            let coil_idx = *coil;
            if coil_idx == 0 {
                // Move to coil B inductance.
                *electrical_id_state = ElectricalIdState::InductanceSetup { coil: 1 };
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
            let assembly = match assemble_electrical_model(data, buf.samples()) {
                Ok(assembly) => assembly,
                Err(err) => {
                    log_model_assembly_failure(err);
                    return super::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }
            };

            match sequence {
                InnerSequence::Selftest => {
                    selftest_results.set(
                        7,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: assembly.model.r_mohm,
                        },
                    );
                    selftest_results.set(
                        8,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: assembly.model.l_uh,
                        },
                    );
                    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
                    StepResult::SelftestDone
                }
                InnerSequence::Commission => {
                    // Carry the model + Phase 5 confidence flags into the
                    // current-tune phase. Bits 4–6 (retry / fallback / host
                    // default) are added later by current validation and
                    // host parameter resolution.
                    *phase = InnerPhase::CurrentTune {
                        model: assembly.model,
                        inner_warning_flags: assembly.warning_flags,
                        tau_residual_permille: assembly.tau_residual_permille,
                    };
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

    /// Per-coil resistance currents that yield r_mohm = 1706 (V=512, I=300).
    /// Inductance values produce tau_e ≈ 730 us, matching the
    /// `representative_transient` crosscheck so the new tau-residual gate
    /// passes.
    fn representative_data() -> ElectricalIdData {
        let mut data = ElectricalIdData::new(512);
        data.resistance_current_sum = [300_000, 300_000];
        data.resistance_sample_count = [1000, 1000];
        data.inductance_sum_delta = [200_000, 200_000];
        data.inductance_count = [5000, 5000];
        data.inductance_elapsed_us = [490_000, 490_000];
        data
    }

    /// Captures a step transient that yields theta_e ≈ 214 us and
    /// tau_e_crosscheck ≈ 730 us (steady state ≈ 400, threshold ≈ 252,
    /// crossing at index 5 between 180 and 260).
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
    fn model_assembly_uses_full_width_resistance_math() {
        let data = representative_data();
        let buf = representative_transient();

        let assembly = assemble_electrical_model(&data, buf.samples()).unwrap();

        assert_eq!(assembly.model.r_int, 1706);
        assert_eq!(assembly.model.r_mohm, 1706);
        assert!(
            (725..=740).contains(&assembly.model.tau_e_us),
            "tau should use R=1.706, not truncated: {}",
            assembly.model.tau_e_us
        );
        // l_int = r_int * tau / SCALE = 1706 * 730 / 1000 ≈ 1245.
        assert!(
            (1235..=1260).contains(&assembly.model.l_int),
            "l_int should preserve the internal L scale: {}",
            assembly.model.l_int
        );
    }

    #[test]
    fn model_assembly_extrapolates_delay_onset_to_baseline() {
        // Use a transient with steady state 1000 (threshold 632) and a
        // sufficiently slow ramp so tau_crosscheck stays close to tau_e.
        // Adjust per-coil tau via inductance data so the residual gate
        // does not fire.
        let mut data = ElectricalIdData::new(512);
        data.resistance_current_sum = [300_000, 300_000];
        data.resistance_sample_count = [1000, 1000];
        // Tune sum_delta so tau_e ≈ 887 us to match the transient
        // crosscheck below.
        // avg_delta = V * dt * SCALE / (tau * r_int)
        //           = 512 * 98 * 1000 / (887 * 1706) ≈ 33
        // sum_delta = 33 * 5000 = 165_000
        data.inductance_sum_delta = [165_000, 165_000];
        data.inductance_count = [5000, 5000];
        data.inductance_elapsed_us = [490_000, 490_000];

        let mut buf = TransientBuffer::new();
        buf.reset(ELECTRICAL_SAMPLE_PERIOD_US as u16);
        for &sample in &[0, 10, 25, 100, 300, 500, 650, 800] {
            buf.push(sample);
        }
        for _ in 0..12 {
            buf.push(1000);
        }

        let assembly = assemble_electrical_model(&data, buf.samples()).unwrap();

        assert!(
            (210..=220).contains(&assembly.model.theta_e_us),
            "theta should extrapolate the threshold crossing back to baseline: {}",
            assembly.model.theta_e_us
        );
        assert!(
            (880..=890).contains(&assembly.model.tau_e_crosscheck_us),
            "tau crosscheck should use interpolated 63 percent crossing minus onset: {}",
            assembly.model.tau_e_crosscheck_us
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
                state: ElectricalIdState::InductanceSettle {
                    coil: 0,
                    deadline_us: 0,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            // Section 2 bound check rejects didt_cycles below 64 + warmup.
            // 200 → warmup = max(8, 200/20) = 10, measured = 190 ≥ 64.
            backend.tmc_config.didt_cycles = 200;
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
                        ElectricalIdState::InductanceToggle {
                            remaining_cycles, ..
                        },
                    ..
                } => *remaining_cycles,
                other => panic!("expected InductanceToggle, got {other:?}"),
            };
            assert_eq!(remaining_cycles, 200);

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
            // Polarity still alternates by absolute cycle number across
            // both the warm-up and measured windows.
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
                state: ElectricalIdState::InductanceToggle {
                    coil: 0,
                    total_cycles: 1,
                    remaining_cycles: 1,
                    prev_current: 0,
                    start_us: u64::MAX - 10,
                    // No warm-up remaining: this single cycle is measured,
                    // and the elapsed window starts at the recorded
                    // accumulation_start_us, which mirrors start_us.
                    warmup_cycles_remaining: 0,
                    accumulation_start_us: Some(u64::MAX - 10),
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
                    state: ElectricalIdState::InductanceDisable { .. },
                    data,
                } => {
                    assert_eq!(data.inductance_elapsed_us[0], 31);
                    assert_eq!(data.inductance_count[0], 1);
                }
                other => panic!("expected InductanceDisable, got {other:?}"),
            }
        });
    }

    #[test]
    fn inductance_toggle_write_error_fails_inner_phase() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceToggle {
                    coil: 0,
                    total_cycles: 7,
                    remaining_cycles: 7,
                    prev_current: 0,
                    start_us: 0,
                    warmup_cycles_remaining: 0,
                    accumulation_start_us: Some(0),
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
                state: ElectricalIdState::InductanceToggle {
                    coil: 0,
                    total_cycles: 7,
                    remaining_cycles: 7,
                    prev_current: 0,
                    start_us: 0,
                    warmup_cycles_remaining: 0,
                    accumulation_start_us: Some(0),
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
                state: ElectricalIdState::InductanceToggle {
                    coil: 0,
                    total_cycles: 7,
                    remaining_cycles: 7,
                    prev_current: 0,
                    start_us: 0,
                    warmup_cycles_remaining: 0,
                    accumulation_start_us: Some(0),
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
            let mut after: heapless::Vec<u8, 32> = heapless::Vec::new();
            for op in backend.spi_log.iter() {
                if let SpiOp::Write(addr, _) = op {
                    if !seen_failure && *addr == UQ_UD_EXT.addr() {
                        seen_failure = true;
                        continue;
                    }
                    if seen_failure {
                        after.push(*addr).ok();
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

    #[test]
    fn inductance_setup_fails_when_didt_cycles_too_small() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSetup { coil: 0 },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            // 50 - max(8, 50/20=8) = 42 measured cycles, below the 64 floor.
            backend.tmc_config.didt_cycles = 50;
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
            assert!(!backend.enable_pin, "must not energize on rejected config");
        });
    }

    #[test]
    fn inductance_setup_fails_when_didt_cycles_overlarge() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSetup { coil: 0 },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.tmc_config.didt_cycles = 60_000; // > 50_000 cap
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
            assert!(!backend.enable_pin, "must not energize on rejected config");
        });
    }

    #[test]
    fn inductance_toggle_skips_warmup_cycles_in_accumulator() {
        pollster::block_on(async {
            // 4 cycles total, 2 of warm-up. The first 2 cycles must not
            // contribute to inductance_count or inductance_sum_delta; the
            // elapsed window starts at the elapsed_us captured between the
            // last warm-up read and the first measured read.
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceToggle {
                    coil: 0,
                    total_cycles: 4,
                    remaining_cycles: 4,
                    prev_current: 0,
                    start_us: 0,
                    warmup_cycles_remaining: 2,
                    accumulation_start_us: None,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 100;
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 50)
                .ok();
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let _ = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            match &phase {
                InnerPhase::ElectricalId {
                    state: ElectricalIdState::InductanceDisable { .. },
                    data,
                } => {
                    assert_eq!(data.inductance_count[0], 2);
                }
                other => panic!("expected InductanceDisable, got {other:?}"),
            }
        });
    }

    // -----------------------------------------------------------------
    // Phase 1 model-assembly hard gates
    // -----------------------------------------------------------------

    /// Build per-coil resistance data for currents `c0`/`c1` (in counts).
    /// The inductance arms are tuned to give per-coil tau ≈ 730us against
    /// the resulting `r_int` so other gates do not fire incidentally.
    fn data_with_per_coil_currents(c0: i32, c1: i32) -> ElectricalIdData {
        let mut data = ElectricalIdData::new(512);
        data.resistance_current_sum = [c0 * 1000, c1 * 1000];
        data.resistance_sample_count = [1000, 1000];
        let avg_abs = ((c0.unsigned_abs() + c1.unsigned_abs()) / 2).max(1);
        let r_int = (512u64 * 1000 / avg_abs as u64).max(1) as u32;
        let dt_us = 98u64;
        let target_tau_us = 730u64;
        // avg_delta = V * dt * SCALE / (tau * r_int)
        let avg_delta = (512u64 * dt_us * 1000 / (target_tau_us * r_int as u64)).max(1);
        let count: u32 = 5000;
        let sum_delta = avg_delta * count as u64;
        data.inductance_sum_delta = [sum_delta, sum_delta];
        data.inductance_count = [count, count];
        data.inductance_elapsed_us = [count * dt_us as u32, count * dt_us as u32];
        data
    }

    #[test]
    fn assemble_accepts_representative_data() {
        let data = representative_data();
        let buf = representative_transient();
        let assembly = assemble_electrical_model(&data, buf.samples()).unwrap();
        // Bit 3 (theta/tau warning) is expected because theta=214, tau≈730,
        // theta/tau = 0.293 ≥ 0.25. Bits 0–2 should be clean.
        assert_eq!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
        assert_eq!(assembly.warning_flags & INNER_WARN_BIT_TAU_MISMATCH, 0);
    }

    #[test]
    fn assemble_rejects_short_resistance() {
        // V=512, I=8000 → r_mohm = 64 (< 100 short threshold).
        let data = data_with_per_coil_currents(8000, 8000);
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples()).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::ResistanceShort { .. }));
    }

    #[test]
    fn assemble_rejects_open_resistance() {
        // V=512, I=10 → r_mohm = 51_200 (> 50_000 open threshold).
        // Use sum_delta tuned to keep per-coil tau non-zero so the
        // SingleCoilTau gate doesn't fire first.
        let mut data = ElectricalIdData::new(512);
        data.resistance_current_sum = [10_000, 10_000];
        data.resistance_sample_count = [1000, 1000];
        // avg_delta = 5 (just above INDUCTANCE_MIN_DELTA), tau ≈ 196 us.
        data.inductance_sum_delta = [25_000, 25_000];
        data.inductance_count = [5000, 5000];
        data.inductance_elapsed_us = [490_000, 490_000];
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples()).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::ResistanceOpen { .. }));
    }

    #[test]
    fn assemble_rejects_per_coil_resistance_disagreement() {
        // c0=300 vs c1=800 produces r_mohm of 1706 vs 640 (~62% mismatch).
        // Per-coil tau also diverges (tau uses per-coil R now), so absent
        // gate ordering this could fire TauMismatch instead. The spec
        // requires R-mismatch to be checked first.
        let data = data_with_per_coil_currents(300, 800);
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples()).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::ResistanceMismatch { .. }));
    }

    #[test]
    fn assemble_rejects_single_coil_tau() {
        let mut data = representative_data();
        data.inductance_sum_delta = [200_000, 0];
        data.inductance_count = [5000, 0];
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples()).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::SingleCoilTau));
    }

    #[test]
    fn assemble_rejects_zero_l_int_with_model_scale_zero() {
        // r_mohm = 100 (just at the short threshold; passes), avg_delta
        // tuned so per-coil tau rounds to 5us — high enough to clear
        // SingleCoilTau but low enough that l_int = r_int * tau / 1000
        // and l_uh = r_mohm * tau / 1000 both round to 0.
        // tau = V * dt * SCALE / (avg_delta * r_int)
        //     = 512 * 98 * 1000 / (avg_delta * 100)
        // avg_delta = 100_352 → tau ≈ 5us → l_int = l_uh = 0.
        let mut data = ElectricalIdData::new(512);
        data.resistance_current_sum = [5_120_000, 5_120_000];
        data.resistance_sample_count = [1000, 1000];
        data.inductance_sum_delta = [501_760_000, 501_760_000];
        data.inductance_count = [5000, 5000];
        data.inductance_elapsed_us = [490_000, 490_000];
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples()).unwrap_err();
        assert!(matches!(
            err,
            ModelAssemblyError::ModelScaleZero { l_int: 0, l_uh: 0 }
        ));
    }

    #[test]
    fn assemble_rejects_per_coil_tau_disagreement_with_equal_resistance() {
        // Equal per-coil resistance (both currents = 300) so the
        // resistance-mismatch gate cannot fire. Skewed sum_delta gives
        // large per-coil tau divergence, exercising the tau-mismatch
        // gate independently of resistance.
        let mut data = representative_data();
        data.inductance_sum_delta = [200_000, 600_000];
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples()).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::TauMismatch { .. }));
    }

    #[test]
    fn assemble_warns_on_borderline_resistance_mismatch() {
        // 300 vs 320 → ~6% (above warn 5%, below fail 15%).
        let data = data_with_per_coil_currents(300, 320);
        let buf = representative_transient();
        let assembly = assemble_electrical_model(&data, buf.samples()).unwrap();
        assert_ne!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
    }
}
