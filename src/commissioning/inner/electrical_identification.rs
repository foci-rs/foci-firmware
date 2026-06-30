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
use crate::commissioning::types::{
    CommissioningBackend, INDUCTANCE_FIT_POINT_COUNT, InductanceFitEvidence, InductanceFitPoint,
    StepResult,
};
use foci_tmc4671::registers::Register;

/// Sampling interval for resistance and transient captures (microseconds).
/// Matches PWM rate (~160us at 25 kHz) for Nyquist-rate current sampling.
///
/// Re-exported for the validation phase, which captures step responses at
/// the same cadence.
pub(crate) const ELECTRICAL_SAMPLE_PERIOD_US: u64 = 160;

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
#[allow(
    unfulfilled_lint_expectations,
    reason = "the staged helper references this constant before production capture/model assembly lands"
)]
#[cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "multi-amplitude inductance capture/model assembly tasks wire this into production",
    )
)]
const INDUCTANCE_FIT_MIN_USABLE_POINTS: u8 = 3;
#[allow(
    unfulfilled_lint_expectations,
    reason = "the staged helper references this constant before production capture/model assembly lands"
)]
#[cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "multi-amplitude inductance capture/model assembly tasks wire this into production",
    )
)]
const INDUCTANCE_FIT_MAX_DEADTIME_UD: i32 = 400;
#[allow(
    unfulfilled_lint_expectations,
    reason = "the staged helper references this constant before production capture/model assembly lands"
)]
#[cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "multi-amplitude inductance capture/model assembly tasks wire this into production",
    )
)]
const INDUCTANCE_FIT_MAX_RESIDUAL_PERMILLE: u16 = 120;

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

const DIAG_ELECTRICAL_ID_EXCITATION: u8 = 1;
const DIAG_ELECTRICAL_ID_RESISTANCE_A: u8 = 2;
const DIAG_ELECTRICAL_ID_RESISTANCE_B: u8 = 3;
const DIAG_ELECTRICAL_ID_INDUCTANCE_A: u8 = 4;
const DIAG_ELECTRICAL_ID_INDUCTANCE_B: u8 = 5;
const DIAG_ELECTRICAL_ID_TRANSIENT: u8 = 6;
const DIAG_ELECTRICAL_ID_REJECT_NO_SAMPLES: u8 = 20;
const DIAG_ELECTRICAL_ID_REJECT_SINGLE_COIL_TAU: u8 = 21;
const DIAG_ELECTRICAL_ID_REJECT_MODEL_SCALE_ZERO: u8 = 22;
// Codes 23–25 (resistance short/open/mismatch) are retired: the shared
// resistance runner now owns those quality gates.
const DIAG_ELECTRICAL_ID_REJECT_TAU_MISMATCH: u8 = 26;
const DIAG_ELECTRICAL_ID_REJECT_TAU_CROSSCHECK_UNMEASURABLE: u8 = 27;
const DIAG_ELECTRICAL_ID_REJECT_THETA_TOO_LARGE: u8 = 29;

// ---------------------------------------------------------------------------
// Phase 1 model-assembly gates and warning bits.
// ---------------------------------------------------------------------------

/// Per-coil resistance disagreement >= 5 % (warning).
pub(crate) const INNER_WARN_BIT_RESISTANCE_MISMATCH: u8 = 1 << 0;
/// Per-coil tau/L disagreement >= 15 % (warning).
pub(crate) const INNER_WARN_BIT_TAU_MISMATCH: u8 = 1 << 1;
/// theta_e_us >= tau_e_us / 4 (warning).
pub(crate) const INNER_WARN_BIT_THETA_TAU: u8 = 1 << 3;

/// Per-coil resistance disagreement warn threshold (permille).
/// The hard resistance gates now live in the shared runner; this remains as a
/// non-critical confidence warning computed from the runner's per-axis slopes.
const RESISTANCE_MISMATCH_WARN_PERMILLE: u32 = 50;
/// Per-coil tau disagreement fail threshold (permille).
const TAU_MISMATCH_FAIL_PERMILLE: u32 = 300;
/// Per-coil tau disagreement warn threshold (permille).
const TAU_MISMATCH_WARN_PERMILLE: u32 = 150;
/// Successful model assembly: the assembled `ElectricalModel`, the
/// non-critical warning bitfield, and the computed L/R-vs-transient residual
/// in permille for the host to persist.
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
    /// Per-coil tau disagreement above hard threshold.
    TauMismatch { permille: u32 },
    /// Transient capture produced no usable rise crossing, so the
    /// crosscheck tau is undefined. Distinct from a non-zero-but-bad
    /// residual so diagnostics can distinguish "didn't measure" from
    /// "measured wrong".
    TauCrosscheckUnmeasurable,
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
        ModelAssemblyError::TauMismatch { permille } => {
            defmt::warn!("electrical id: coil tau mismatch {} permille", permille);
        }
        ModelAssemblyError::TauCrosscheckUnmeasurable => {
            defmt::warn!("electrical id: tau crosscheck unmeasurable");
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

fn diagnostic_for_model_assembly_failure(
    err: ModelAssemblyError,
) -> crate::selftest::CommissionDiagnostic {
    let (code, value0, value1, value2) = match err {
        ModelAssemblyError::SampleCount => (DIAG_ELECTRICAL_ID_REJECT_NO_SAMPLES, 0, 0, 0),
        ModelAssemblyError::SingleCoilTau => (DIAG_ELECTRICAL_ID_REJECT_SINGLE_COIL_TAU, 0, 0, 0),
        ModelAssemblyError::ModelScaleZero { l_int, l_uh } => {
            (DIAG_ELECTRICAL_ID_REJECT_MODEL_SCALE_ZERO, l_int, l_uh, 0)
        }
        ModelAssemblyError::TauMismatch { permille } => (
            DIAG_ELECTRICAL_ID_REJECT_TAU_MISMATCH,
            permille,
            TAU_MISMATCH_FAIL_PERMILLE,
            0,
        ),
        ModelAssemblyError::TauCrosscheckUnmeasurable => (
            DIAG_ELECTRICAL_ID_REJECT_TAU_CROSSCHECK_UNMEASURABLE,
            0,
            0,
            0,
        ),
        ModelAssemblyError::ThetaTooLarge { theta_us, tau_us } => (
            DIAG_ELECTRICAL_ID_REJECT_THETA_TOO_LARGE,
            theta_us,
            tau_us,
            0,
        ),
    };
    crate::selftest::CommissionDiagnostic {
        phase: PhaseId::ElectricalId.to_u8(),
        code,
        status: 1,
        value0,
        value1,
        value2,
    }
}

fn permille_diff(a: u32, b: u32) -> u32 {
    let average = (a as u64 + b as u64) / 2;
    if average == 0 {
        return 0;
    }
    let diff = a.abs_diff(b) as u64;
    (diff * 1000 / average) as u32
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

fn electrical_id_measurement_diagnostic(
    code: u8,
    value0: u32,
    value1: u32,
    value2: u32,
) -> crate::selftest::CommissionDiagnostic {
    crate::selftest::CommissionDiagnostic {
        phase: PhaseId::ElectricalId.to_u8(),
        code,
        status: 0,
        value0,
        value1,
        value2,
    }
}

/// Per-coil resistance count-slope (milli-units) and RMSE (permille) from the
/// shared runner result. `None` until the resistance sweep has populated
/// `data.resistance_result`.
///
/// The count-slope is in the same unit space as the legacy `r_int` resistance
/// scale (`dV_count / dI_count * 1000`), so it feeds the per-coil tau and gain
/// math directly without conversion.
fn resistance_measurement(data: &ElectricalIdData, coil_idx: usize) -> Option<(u32, u32, u32)> {
    let result = data.resistance_result?;
    let axis = if coil_idx == 0 {
        result.axis0
    } else {
        result.axis1
    };
    let r_int = axis.r_count_slope_milli.max(0) as u32;
    if r_int == 0 {
        return None;
    }
    // `r_mohm` diagnostic value is the count-slope-milli reused as derived
    // display evidence; the runner owns the physical-scale gating.
    Some((r_int, r_int, u32::from(axis.selected_count)))
}

/// Per-coil resistance count-slope in the `ELECTRICAL_MODEL_RESISTANCE_SCALE`
/// (milli) unit space, taken from the runner's per-axis fit.
fn resistance_model_scale(data: &ElectricalIdData, coil_idx: usize) -> Option<u32> {
    let (r_int, _, _) = resistance_measurement(data, coil_idx)?;
    Some(r_int)
}

#[cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "multi-amplitude inductance capture/model assembly tasks wire this into production",
    )
)]
fn fit_inductance_points(
    points: &[InductanceFitPoint; INDUCTANCE_FIT_POINT_COUNT],
    r_int: u32,
) -> Option<InductanceFitEvidence> {
    if r_int == 0 {
        return None;
    }

    let mut n = 0i64;
    let mut sum_x = 0i64;
    let mut sum_y = 0i64;
    let mut sum_xx = 0i64;
    let mut sum_xy = 0i64;
    let mut sum_elapsed_us = 0u64;
    let mut sum_sample_count = 0u64;
    let mut min_effective_ud = i32::MAX;
    let mut selected_mask = 0u16;

    for (idx, point) in points.iter().enumerate() {
        if point.sample_count == 0 || point.avg_delta < INDUCTANCE_MIN_DELTA {
            continue;
        }
        if point.effective_ud <= 0 {
            continue;
        }
        let x = point.effective_ud as i64;
        let y = point.avg_delta as i64;
        n += 1;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_elapsed_us += u64::from(point.elapsed_us);
        sum_sample_count += u64::from(point.sample_count);
        min_effective_ud = min_effective_ud.min(point.effective_ud);
        selected_mask |= 1u16 << idx;
    }

    if n < i64::from(INDUCTANCE_FIT_MIN_USABLE_POINTS) || sum_sample_count == 0 {
        return None;
    }

    let denom = n * sum_xx - sum_x * sum_x;
    if denom <= 0 {
        return None;
    }

    let slope_milli = ((n * sum_xy - sum_x * sum_y) * 1000) / denom;
    if slope_milli <= 0 {
        return None;
    }
    let intercept_milli = ((sum_y * 1000) - slope_milli * sum_x) / n;
    let deadtime_ud =
        ((-intercept_milli) / slope_milli).clamp(i16::MIN as i64, i16::MAX as i64) as i16;
    if deadtime_ud < 0
        || i32::from(deadtime_ud) > INDUCTANCE_FIT_MAX_DEADTIME_UD
        || i32::from(deadtime_ud) >= min_effective_ud
    {
        return None;
    }

    let mut max_residual_permille = 0u16;
    for point in points.iter() {
        if point.sample_count == 0 || point.avg_delta < INDUCTANCE_MIN_DELTA {
            continue;
        }
        if point.effective_ud <= 0 {
            continue;
        }
        let predicted_milli = slope_milli * point.effective_ud as i64 + intercept_milli;
        if predicted_milli <= 0 {
            return None;
        }
        let actual_milli = point.avg_delta as i64 * 1000;
        let diff = actual_milli.abs_diff(predicted_milli);
        let residual = (diff * 1000 / predicted_milli as u64).min(1000) as u16;
        max_residual_permille = max_residual_permille.max(residual);
    }
    if max_residual_permille > INDUCTANCE_FIT_MAX_RESIDUAL_PERMILLE {
        return None;
    }

    let dt_per_cycle_us = sum_elapsed_us / sum_sample_count;
    if dt_per_cycle_us == 0 {
        return None;
    }

    let tau_us_u64 = dt_per_cycle_us * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64 * 1000
        / (slope_milli as u64 * r_int as u64);
    let tau_us = u32::try_from(tau_us_u64).ok()?;
    if tau_us == 0 {
        return None;
    }

    Some(InductanceFitEvidence {
        tau_us,
        old_tau_us: 0,
        slope_delta_per_ud_milli: slope_milli as u32,
        intercept_delta_milli: intercept_milli.clamp(i32::MIN as i64, i32::MAX as i64) as i32,
        deadtime_ud,
        residual_permille: max_residual_permille,
        usable_points: n as u8,
        selected_mask,
    })
}

fn inductance_measurement(data: &ElectricalIdData, coil_idx: usize) -> Option<(u32, u32, u32)> {
    let count = data.inductance_count[coil_idx];
    let sum_delta = data.inductance_sum_delta[coil_idx];
    let elapsed = data.inductance_elapsed_us[coil_idx];
    if count == 0 || sum_delta == 0 || elapsed == 0 {
        return None;
    }
    let avg_delta = (sum_delta / count as u64) as u32;
    let r_int = resistance_model_scale(data, coil_idx).unwrap_or(0);
    let tau_us = if avg_delta >= INDUCTANCE_MIN_DELTA && r_int != 0 {
        let dt_per_cycle_us = elapsed / count;
        (data.voltage as u64 * dt_per_cycle_us as u64 * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64
            / (avg_delta as u64 * r_int as u64)) as u32
    } else {
        0
    };
    Some((avg_delta, tau_us, count))
}

fn transient_measurement(samples: &[u32]) -> Option<(u32, u32, u32)> {
    let sample_count = samples.len();
    if sample_count == 0 {
        return None;
    }
    let steady_state = if sample_count >= 20 {
        let tail_start = sample_count - 10;
        let tail_sum: u32 = samples[tail_start..].iter().sum();
        tail_sum / 10
    } else {
        samples[sample_count - 1]
    };
    let theta_us = extrapolate_onset_to_baseline_us(samples, DELAY_ONSET_THRESHOLD as u32)
        .unwrap_or(ELECTRICAL_SAMPLE_PERIOD_US as u32);
    let rise_threshold = (steady_state as u64 * RISE_63_PCT_Q10 as u64 / 1024) as u32;
    let rise_crossing_us = interpolate_threshold_crossing_us(samples, rise_threshold).unwrap_or(0);
    let crosscheck_us = rise_crossing_us.saturating_sub(theta_us);
    Some((steady_state, theta_us, crosscheck_us))
}

fn record_electrical_id_measurement_diagnostics(
    data: &ElectricalIdData,
    samples: &[u32],
    didt_cycles: u32,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
) {
    diagnostics.push(electrical_id_measurement_diagnostic(
        DIAG_ELECTRICAL_ID_EXCITATION,
        data.voltage,
        didt_cycles,
        ELECTRICAL_SAMPLE_PERIOD_US as u32,
    ));
    for coil_idx in 0..2 {
        if let Some((avg_current, r_mohm, count)) = resistance_measurement(data, coil_idx) {
            diagnostics.push(electrical_id_measurement_diagnostic(
                if coil_idx == 0 {
                    DIAG_ELECTRICAL_ID_RESISTANCE_A
                } else {
                    DIAG_ELECTRICAL_ID_RESISTANCE_B
                },
                avg_current,
                r_mohm,
                count,
            ));
        }
    }
    for coil_idx in 0..2 {
        if let Some((avg_delta, tau_us, count)) = inductance_measurement(data, coil_idx) {
            diagnostics.push(electrical_id_measurement_diagnostic(
                if coil_idx == 0 {
                    DIAG_ELECTRICAL_ID_INDUCTANCE_A
                } else {
                    DIAG_ELECTRICAL_ID_INDUCTANCE_B
                },
                avg_delta,
                tau_us,
                count,
            ));
        }
    }
    if let Some((steady_state, theta_us, crosscheck_us)) = transient_measurement(samples) {
        diagnostics.push(electrical_id_measurement_diagnostic(
            DIAG_ELECTRICAL_ID_TRANSIENT,
            steady_state,
            theta_us,
            crosscheck_us,
        ));
    }
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
    configured_theta_e_us: u32,
) -> Result<ModelAssembly, ModelAssemblyError> {
    let voltage = data.voltage;

    // --- Per-coil resistance (from the shared multi-point runner) ---
    //
    // `r_int` and the per-coil entries come from the runner's count-space fit,
    // in `ELECTRICAL_MODEL_RESISTANCE_SCALE` (milli) units. The runner already
    // enforces the resistance quality gates (short/open/per-axis mismatch,
    // residual, drift); this path only translates its slopes into the model.
    let result = data
        .resistance_result
        .ok_or(ModelAssemblyError::SampleCount)?;
    let mut r_int_per_coil = [0u32; 2];
    let axis_results = [result.axis0, result.axis1];
    for (coil_idx, axis) in axis_results.iter().enumerate() {
        let r_int_coil = axis.r_count_slope_milli.max(0) as u32;
        if r_int_coil == 0 {
            return Err(ModelAssemblyError::SampleCount);
        }
        r_int_per_coil[coil_idx] = r_int_coil;
    }
    // Averaged model resistance is the run-selected count-slope.
    let r_int = result.selected_r_count_slope_milli.max(0) as u32;
    if r_int == 0 {
        return Err(ModelAssemblyError::SampleCount);
    }
    // `r_mohm` is derived display evidence only: the count-slope-milli reused
    // as a physical-display stand-in. The host applies real scale metadata.
    let r_mohm = r_int;

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
    //
    // Resistance short/open/per-axis-mismatch gates are owned by the shared
    // runner's count-space quality gates; only the inductance/delay gates run
    // here. The per-coil resistance disagreement is still surfaced as a
    // confidence warning below, computed from the runner's per-axis slopes.
    let r_diff_permille = permille_diff(r_int_per_coil[0], r_int_per_coil[1]);
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

    let transient_theta_e_us =
        extrapolate_onset_to_baseline_us(samples, DELAY_ONSET_THRESHOLD as u32)
            .unwrap_or(ELECTRICAL_SAMPLE_PERIOD_US as u32);

    let rise_threshold = (steady_state as u64 * RISE_63_PCT_Q10 as u64 / 1024) as u32;
    let rise_crossing_us = interpolate_threshold_crossing_us(samples, rise_threshold).unwrap_or(0);
    let tau_e_crosscheck_us = rise_crossing_us.saturating_sub(transient_theta_e_us);

    if tau_e_crosscheck_us == 0 {
        return Err(ModelAssemblyError::TauCrosscheckUnmeasurable);
    }
    // tau_e_us is guaranteed non-zero here (passed SingleCoilTau).
    let diff = tau_e_us.abs_diff(tau_e_crosscheck_us) as u64;
    let tau_residual_permille = ((diff * 1000 / tau_e_us as u64) as u32).min(1000);
    if configured_theta_e_us >= tau_e_us / 2 {
        return Err(ModelAssemblyError::ThetaTooLarge {
            theta_us: configured_theta_e_us,
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
    if configured_theta_e_us >= tau_e_us / 4 {
        warning_flags |= INNER_WARN_BIT_THETA_TAU;
    }

    Ok(ModelAssembly {
        model: ElectricalModel {
            r_int,
            l_int,
            tau_e_us,
            theta_e_us: configured_theta_e_us,
            r_mohm,
            l_uh: l_uh as u32,
            tau_e_crosscheck_us,
        },
        warning_flags,
        tau_residual_permille: tau_residual_permille as u16,
    })
}

/// Adapter that exposes a [`CommissioningBackend`] as a
/// [`crate::resistance_identification::ResistanceBackend`] for the shared
/// resistance runner during commissioning.
///
/// This is the sanctioned commissioning managed-enable exception to the
/// direct-enable rule: the runner owns enable-pin assertion, so
/// [`is_armed_for_open_loop_voltage`](Self::is_armed_for_open_loop_voltage)
/// reports `true` and [`set_enable_pin`](Self::set_enable_pin) forwards to the
/// real backend.
struct CommissionResistanceBackend<'a, B: CommissioningBackend> {
    backend: &'a mut B,
}

impl<B: CommissioningBackend> crate::resistance_identification::ResistanceBackend
    for CommissionResistanceBackend<'_, B>
{
    async fn read_register(
        &mut self,
        addr: u8,
    ) -> Result<u32, crate::resistance_identification::ResistanceBackendError> {
        self.backend
            .read_register(Register::new(addr))
            .await
            .map_err(|_| crate::resistance_identification::ResistanceBackendError::Spi)
    }

    async fn write_register(
        &mut self,
        addr: u8,
        value: u32,
    ) -> Result<(), crate::resistance_identification::ResistanceBackendError> {
        self.backend
            .write_register(Register::new(addr), value)
            .await
            .map_err(|_| crate::resistance_identification::ResistanceBackendError::Spi)
    }

    async fn delay_ms(&mut self, ms: u64) {
        // Cooperative wait through the backend monotonic. `delay_us` is the
        // sanctioned commissioning yield; loop in <=1s chunks to keep the u32
        // microsecond argument well within range for long settle windows.
        let mut remaining_ms = ms;
        while remaining_ms > 0 {
            let chunk_ms = remaining_ms.min(1_000);
            self.backend.delay_us((chunk_ms * 1_000) as u32).await;
            remaining_ms -= chunk_ms;
        }
    }

    fn elapsed_us(&self) -> u64 {
        self.backend.elapsed_us()
    }

    fn set_enable_pin(&mut self, enabled: bool) {
        self.backend.set_enable_pin(enabled);
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        true
    }

    fn profile(&self) -> crate::resistance_identification::ResistanceProfile {
        self.backend.tmc_config().resistance_profile(
            self.backend
                .tmc_config()
                .resistance_scale_metadata_validated,
        )
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        self.backend.encoder_counts_per_rev()
    }

    fn adc_offsets_ready(&self) -> bool {
        self.backend.adc_offsets_ready()
    }
}

/// Advance electrical identification sub-state.
///
/// Runs shared multi-point resistance identification, then per-coil
/// inductance (square-wave dI/dt) and delay (transient capture with a
/// two-point fit). Assembles an `ElectricalModel` and validates before
/// transitioning.
pub(crate) async fn step_electrical_identification<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    buf: &mut TransientBuffer,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (electrical_id_state, data) = match phase {
        InnerPhase::ElectricalId { state, data } => (state, data),
        _ => return StepResult::Continue,
    };

    match electrical_id_state {
        // ---------------------------------------------------------------
        // Resistance identification (shared multi-point runner)
        // ---------------------------------------------------------------
        ElectricalIdState::ResistanceIdentify => {
            // The shared runner owns the full count-space sweep and the
            // managed-enable exception (it asserts/clears the enable pin and
            // restores safe state on every exit path). On success, store the
            // result and hand off to the legacy inductance path; on any
            // quality-gate or backend failure, fail the inner phase.
            let result = crate::resistance_identification::run_resistance_identification(
                &mut CommissionResistanceBackend { backend },
                crate::resistance_identification::ResistanceRunMode::CommissioningManagedEnable,
            )
            .await;

            match result {
                Ok(resistance) => {
                    data.resistance_result = Some(resistance);
                    *electrical_id_state = ElectricalIdState::InductanceSetup { coil: 0 };
                    StepResult::Continue
                }
                Err(err) => {
                    defmt::warn!("resistance id failed: {}", err);
                    super::fail_inner(phase, CommissionError::ResistanceIdFailed)
                }
            }
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
            // The preceding resistance-identification phase leaves
            // PWM_SV_CHOP=0 on its safe-state exit (see
            // resistance_identification::runner::restore_safe_state). Re-enable
            // the chopper here so the dI/dt sweep is self-sufficient regardless
            // of the chopper state left behind, instead of silently measuring
            // zero current with no PWM drive.
            if backend.write_register(PWM_SV_CHOP, 7).await.is_err() {
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
                if *warmup_cycles_remaining == 0 && accumulation_start_us.is_none() {
                    *accumulation_start_us = Some(backend.elapsed_us());
                }
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
            let assembly = match assemble_electrical_model(
                data,
                buf.samples(),
                backend.tmc_config().electrical_model_theta_e_us,
            ) {
                Ok(assembly) => assembly,
                Err(err) => {
                    log_model_assembly_failure(err);
                    record_electrical_id_measurement_diagnostics(
                        data,
                        buf.samples(),
                        backend.tmc_config().didt_cycles,
                        diagnostics,
                    );
                    diagnostics.push(diagnostic_for_model_assembly_failure(err));
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
                    // host parameter resolution. The resistance result is
                    // carried so the final InnerResult can persist the
                    // count-space fit and the 64x audit comparison. It is
                    // `Some` here because `ResistanceIdentify` always runs
                    // before `Validate` in the commission sequence.
                    let resistance_result = data
                        .resistance_result
                        .expect("resistance result populated before Validate");
                    *phase = InnerPhase::CurrentTune {
                        model: assembly.model,
                        inner_warning_flags: assembly.warning_flags,
                        tau_residual_permille: assembly.tau_residual_permille,
                        resistance_result,
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
    use crate::commissioning::backend::CommissionBoardConfig;
    use crate::commissioning::inner::ElectricalIdData;
    use crate::commissioning::types::StepResult;
    use crate::resistance_identification::{
        ElectricalAxis, ResistanceAxisResult, ResistanceRunResult, ResistanceWarningFlags,
        SignedAnchorEvidence,
    };
    use foci_tmc4671::registers::{PID_TORQUE_FLUX_ACTUAL, UQ_UD_EXT};

    const TEST_THETA_E_US: u32 = 160;

    /// Build a synthetic per-axis fit at the given count-slope (milli-units).
    fn axis_result(axis: ElectricalAxis, slope_milli: i32) -> ResistanceAxisResult {
        ResistanceAxisResult {
            axis,
            r_count_slope_milli: slope_milli,
            slope_num: i64::from(slope_milli),
            slope_den: 1000,
            intercept_count: 0,
            rmse_permille: 0,
            selected_mask: 0b1111_1000,
            excluded_point_mask: 0b0000_0111,
            selected_count: 5,
            signed_anchor: Some(SignedAnchorEvidence {
                positive_current_count: 900,
                negative_current_count: -900,
                signed_count_slope_milli: slope_milli,
                asymmetry_permille: 0,
                repeated_positive_current_count: 900,
                drift_permille: 0,
            }),
            warning_flags: ResistanceWarningFlags::NONE,
        }
    }

    /// Build a synthetic two-axis resistance result with the given per-axis
    /// count-slopes (milli-units). The selected slope is their average, matching
    /// the runner's `evaluate_resistance_result` behavior.
    fn resistance_result(slope0_milli: i32, slope1_milli: i32) -> ResistanceRunResult {
        ResistanceRunResult {
            profile_version: 1,
            selected_r_count_slope_milli: (slope0_milli + slope1_milli) / 2,
            status_flags_or: 0,
            axis0: axis_result(ElectricalAxis::PHASE0, slope0_milli),
            axis1: axis_result(ElectricalAxis::PHASE90, slope1_milli),
            warning_flags: ResistanceWarningFlags::NONE,
        }
    }

    /// Per-coil resistance count-slopes of 1706 milli (V=512, I=300 → 1706).
    /// Inductance values produce tau_e ≈ 730 us, matching the
    /// `representative_transient` crosscheck so the tau-residual gate passes.
    fn representative_data() -> ElectricalIdData {
        let mut data = ElectricalIdData::new(512);
        data.resistance_result = Some(resistance_result(1706, 1706));
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

        let assembly = assemble_electrical_model(&data, buf.samples(), TEST_THETA_E_US).unwrap();

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
    fn inductance_fit_uses_slope_not_absolute_ud() {
        let r_int = 1000;
        let dt_us = 100;
        let target_tau_us = 1200;
        let deadtime_ud = 200i32;
        let uds = [512u16, 640, 768, 896];
        let mut points = [InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT];

        for (idx, ud) in uds.iter().copied().enumerate() {
            let effective = i32::from(ud) - deadtime_ud;
            let avg_delta =
                (effective as u64 * dt_us as u64 * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64
                    / (target_tau_us as u64 * r_int as u64)) as u32;
            points[idx] = InductanceFitPoint {
                ud,
                avg_delta,
                avg_current_count: 0,
                r_drop_ud: 0,
                effective_ud: i32::from(ud),
                sample_count: 100,
                elapsed_us: 10_000,
            };
        }

        let fit = fit_inductance_points(&points, r_int).expect("fit should pass");

        assert!((1190..=1210).contains(&fit.tau_us), "tau={}", fit.tau_us);
        assert!(
            (190..=210).contains(&fit.deadtime_ud),
            "deadtime={}",
            fit.deadtime_ud
        );
        assert_eq!(fit.usable_points, 4);
        assert!(
            fit.residual_permille <= 20,
            "residual={}",
            fit.residual_permille
        );
    }

    #[test]
    fn inductance_fit_subtracts_r_drop_before_slope() {
        let r_int = 1000;
        let dt_us = 100;
        let target_tau_us = 1200;
        let deadtime_ud = 200i32;
        let uds = [512u16, 640, 768, 896];
        let currents = [20i32, 60, 100, 140];
        let mut points = [InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT];

        for (idx, ud) in uds.iter().copied().enumerate() {
            let r_drop = r_int as i32 * currents[idx] / ELECTRICAL_MODEL_RESISTANCE_SCALE as i32;
            let effective = i32::from(ud) - r_drop - deadtime_ud;
            let avg_delta =
                (effective as u64 * dt_us as u64 * ELECTRICAL_MODEL_RESISTANCE_SCALE as u64
                    / (target_tau_us as u64 * r_int as u64)) as u32;
            points[idx] = InductanceFitPoint {
                ud,
                avg_delta,
                avg_current_count: currents[idx],
                r_drop_ud: r_drop,
                effective_ud: i32::from(ud) - r_drop,
                sample_count: 100,
                elapsed_us: 10_000,
            };
        }

        let fit = fit_inductance_points(&points, r_int).expect("fit should pass");

        assert!((1190..=1210).contains(&fit.tau_us), "tau={}", fit.tau_us);
        assert!(
            (190..=210).contains(&fit.deadtime_ud),
            "deadtime={}",
            fit.deadtime_ud
        );
        assert!(
            fit.residual_permille <= 20,
            "residual={}",
            fit.residual_permille
        );
    }

    #[test]
    fn inductance_fit_rejects_tau_overflow() {
        let mut points = [InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT];
        let uds = [5200u16, 6200, 7200, 8200];

        for (idx, ud) in uds.iter().copied().enumerate() {
            points[idx] = InductanceFitPoint {
                ud,
                avg_delta: idx as u32 + INDUCTANCE_MIN_DELTA,
                avg_current_count: 0,
                r_drop_ud: 0,
                effective_ud: i32::from(ud),
                sample_count: 1,
                elapsed_us: u32::MAX,
            };
        }

        assert!(fit_inductance_points(&points, 1).is_none());
    }

    #[test]
    fn model_assembly_uses_configured_theta_and_keeps_transient_crosscheck() {
        // Use a transient with steady state 1000 (threshold 632) and a
        // sufficiently slow ramp so tau_crosscheck stays close to tau_e.
        // Adjust per-coil tau via inductance data so the residual gate
        // does not fire.
        let mut data = ElectricalIdData::new(512);
        // r_count_slope_milli = 1706 (V=512, I=300 -> matches the old
        // resistance_current_sum=[300_000,300_000]/sample_count=[1000,1000]
        // single-point average this fixture replaced).
        data.resistance_result = Some(resistance_result(1706, 1706));
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

        let assembly = assemble_electrical_model(&data, buf.samples(), 320).unwrap();

        assert_eq!(assembly.model.theta_e_us, 320);
        assert!(
            (880..=890).contains(&assembly.model.tau_e_crosscheck_us),
            "tau crosscheck should use interpolated 63 percent crossing minus onset: {}",
            assembly.model.tau_e_crosscheck_us
        );
    }

    #[test]
    fn validate_reports_tau_crosscheck_mismatch_as_telemetry() {
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::CurrentTune)
            ));
            let InnerPhase::CurrentTune {
                model,
                inner_warning_flags,
                tau_residual_permille,
                resistance_result,
            } = phase
            else {
                panic!("expected CurrentTune, got {phase:?}");
            };
            assert_eq!(
                resistance_result.selected_r_count_slope_milli, 1706,
                "resistance result from Phase 5 should carry forward unchanged"
            );
            assert!(
                (725..=740).contains(&model.tau_e_us),
                "tau should be reported from the per-coil L/R model: {}",
                model.tau_e_us
            );
            assert!(
                (290..=305).contains(&model.tau_e_crosscheck_us),
                "crosscheck should come from the transient fixture: {}",
                model.tau_e_crosscheck_us
            );
            assert_eq!(inner_warning_flags & (1 << 2), 0);
            assert!(
                (580..=610).contains(&tau_residual_permille),
                "residual should capture L/R-vs-transient disagreement: {}",
                tau_residual_permille
            );
            assert_eq!(diagnostics.iter().count(), 0);
        });
    }

    /// Pack a `MOTOR_TYPE_N_POLE_PAIRS` value reporting a two-phase stepper,
    /// matching the runner's `InvalidMotorType` entry check.
    fn two_phase_motor_type_n_pole_pairs() -> u32 {
        foci_tmc4671::registers::MotorTypeNPolePairs::new()
            .with_motor_type(foci_tmc4671::registers::MotorType::TwoPhase as u8)
            .with_n_pole_pairs(50)
            .into()
    }

    /// Seed the power-stage readback registers the shared resistance runner
    /// verifies against `backend.tmc_config()`'s profile (`PWM_MAXCNT`,
    /// `PWM_BBM_H_BBM_L`, `DSADC_MDEC_B_MDEC_A`), plus `MOTOR_TYPE_N_POLE_PAIRS`.
    /// `PWM_SV_CHOP` is written and read back by the runner itself, so it does
    /// not need pre-seeding.
    fn seed_resistance_power_stage_registers(backend: &mut MockBackend) {
        use foci_tmc4671::registers::{
            DSADC_MDEC_B_MDEC_A, MOTOR_TYPE_N_POLE_PAIRS, PWM_BBM_H_BBM_L, PWM_MAXCNT,
        };
        let config = backend.tmc_config().clone();
        backend
            .registers
            .insert(
                MOTOR_TYPE_N_POLE_PAIRS.addr(),
                two_phase_motor_type_n_pole_pairs(),
            )
            .ok();
        backend
            .registers
            .insert(PWM_MAXCNT.addr(), u32::from(config.pwm_maxcnt))
            .ok();
        backend
            .registers
            .insert(
                PWM_BBM_H_BBM_L.addr(),
                (u32::from(config.bbm_h) << 8) | u32::from(config.bbm_l),
            )
            .ok();
        backend
            .registers
            .insert(
                DSADC_MDEC_B_MDEC_A.addr(),
                (u32::from(config.dsadc_mdec_b) << 16) | u32::from(config.dsadc_mdec_a),
            )
            .ok();
    }

    #[test]
    fn electrical_id_uses_resistance_runner_before_legacy_inductance() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;

            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::ResistanceIdentify,
                data: ElectricalIdData::new(u32::from(backend.tmc_config().excitation_voltage)),
            };
            let mut buf = TransientBuffer::default();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let step = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(step, StepResult::Continue),
                "expected Continue, got {step:?}"
            );
            assert!(
                matches!(
                    phase,
                    InnerPhase::ElectricalId {
                        state: ElectricalIdState::InductanceSetup { coil: 0 },
                        ..
                    }
                ),
                "expected InductanceSetup after a successful resistance run, got {phase:?}"
            );
            let InnerPhase::ElectricalId { data, .. } = &phase else {
                panic!("expected ElectricalId, got {phase:?}");
            };
            assert!(
                data.resistance_result.is_some(),
                "resistance result should be stored on success"
            );
        });
    }

    #[test]
    fn electrical_id_fails_when_resistance_quality_gate_fails() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            // No current scripting: ADC_IWY_IUX always reads back 0, so no
            // point clears `linear_current_threshold_count` and the shared
            // runner's fit rejects with `InsufficientLinearPoints`.

            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::ResistanceIdentify,
                data: ElectricalIdData::new(u32::from(backend.tmc_config().excitation_voltage)),
            };
            let mut buf = TransientBuffer::default();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let step = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(
                    step,
                    StepResult::InnerFailed(CommissionError::ResistanceIdFailed)
                ),
                "expected ResistanceIdFailed, got {step:?}"
            );
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
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
                &mut diagnostics,
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut adc_cal = None;

            let result = crate::commissioning::inner::step_inner(
                &mut phase,
                &mut buf,
                crate::autotune::CommissionProfile::Balanced,
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
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
    fn inductance_setup_re_enables_chopper_before_asserting_enable_pin() {
        // The resistance-identification phase that precedes InductanceSetup
        // leaves PWM_SV_CHOP=0 on its safe-state exit (see
        // resistance_identification::runner::restore_safe_state). InductanceSetup
        // must restore the chopper itself before energizing, or the dI/dt sweep
        // runs with no PWM drive and measures zero current.
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSetup { coil: 0 },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            // Model the chopper-disabled state the resistance runner leaves
            // behind on its commission-success exit path.
            let _ = backend
                .registers
                .insert(foci_tmc4671::registers::PWM_SV_CHOP.addr(), 0);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            assert!(backend.enable_pin, "expected the enable pin to be asserted");

            use foci_tmc4671::registers::PWM_SV_CHOP;
            let chop_write_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::Write(addr, 7) if *addr == PWM_SV_CHOP.addr()))
                .expect("expected a PWM_SV_CHOP=7 write");
            let enable_pin_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::EnablePin(true)))
                .expect("expected set_enable_pin(true)");
            assert!(
                chop_write_index < enable_pin_index,
                "expected PWM_SV_CHOP=7 before set_enable_pin(true), got chop at {} \
                 and enable at {}",
                chop_write_index,
                enable_pin_index
            );
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
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let _ = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
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

    #[test]
    fn inductance_toggle_starts_elapsed_window_on_first_measured_cycle() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceToggle {
                    coil: 0,
                    total_cycles: 31,
                    remaining_cycles: 31,
                    prev_current: 0,
                    start_us: 1_000,
                    warmup_cycles_remaining: 30,
                    accumulation_start_us: None,
                },
                data: ElectricalIdData::new(512),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 50)
                .ok();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let _ = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            match &phase {
                InnerPhase::ElectricalId {
                    state:
                        ElectricalIdState::InductanceToggle {
                            remaining_cycles,
                            warmup_cycles_remaining,
                            accumulation_start_us,
                            ..
                        },
                    data,
                } => {
                    assert_eq!(*remaining_cycles, 1);
                    assert_eq!(*warmup_cycles_remaining, 0);
                    assert_eq!(*accumulation_start_us, None);
                    assert_eq!(data.inductance_count[0], 0);
                }
                other => panic!("expected InductanceToggle, got {other:?}"),
            }

            backend.elapsed_us_value = 2_000;
            let _ = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            match &phase {
                InnerPhase::ElectricalId {
                    state: ElectricalIdState::InductanceDisable { .. },
                    data,
                } => {
                    assert_eq!(data.inductance_count[0], 1);
                    assert_eq!(data.inductance_elapsed_us[0], 0);
                }
                other => panic!("expected InductanceDisable, got {other:?}"),
            }
        });
    }

    // -----------------------------------------------------------------
    // Phase 1 model-assembly gates (inductance / delay)
    // -----------------------------------------------------------------

    /// Build model data for per-coil resistance currents `c0`/`c1` (counts).
    ///
    /// The per-axis count-slopes are `512000/c` (V=512 in count-slope-milli),
    /// and the inductance arms are tuned per coil to give tau ≈ 730us against
    /// each coil's own slope, so the tau-mismatch gate does not fire
    /// incidentally when only resistance differs.
    fn data_with_per_coil_currents(c0: i32, c1: i32) -> ElectricalIdData {
        let slope0 = (512_000 / c0.max(1)).max(1);
        let slope1 = (512_000 / c1.max(1)).max(1);
        let mut data = ElectricalIdData::new(512);
        data.resistance_result = Some(resistance_result(slope0, slope1));
        let dt_us = 98u64;
        let target_tau_us = 730u64;
        let count: u32 = 5000;
        // avg_delta = V * dt * SCALE / (tau * r_int_coil), tuned per coil.
        let avg_delta0 = (512u64 * dt_us * 1000 / (target_tau_us * slope0 as u64)).max(1);
        let avg_delta1 = (512u64 * dt_us * 1000 / (target_tau_us * slope1 as u64)).max(1);
        data.inductance_sum_delta = [avg_delta0 * count as u64, avg_delta1 * count as u64];
        data.inductance_count = [count, count];
        data.inductance_elapsed_us = [count * dt_us as u32, count * dt_us as u32];
        data
    }

    #[test]
    fn model_failure_diagnostic_codes_are_stable() {
        let single_tau = diagnostic_for_model_assembly_failure(ModelAssemblyError::SingleCoilTau);
        assert_eq!(single_tau.phase, PhaseId::ElectricalId.to_u8());
        assert_eq!(single_tau.code, 21);
        assert_eq!(single_tau.status, 1);

        let tau_mismatch = diagnostic_for_model_assembly_failure(ModelAssemblyError::TauMismatch {
            permille: 301,
        });
        assert_eq!(tau_mismatch.code, 26);
        assert_eq!(tau_mismatch.value0, 301);
    }

    #[test]
    fn permille_diff_uses_average_denominator_for_coil_disagreement() {
        // |130 - 100| / ((130 + 100) / 2) = 30 / 115 = 260 permille.
        assert_eq!(permille_diff(100, 130), 260);
    }

    #[test]
    fn assemble_accepts_representative_data() {
        let data = representative_data();
        let buf = representative_transient();
        let assembly = assemble_electrical_model(&data, buf.samples(), TEST_THETA_E_US).unwrap();
        // Configured theta=160, tau≈730, so the theta/tau warning is clean.
        assert_eq!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
        assert_eq!(assembly.warning_flags & INNER_WARN_BIT_TAU_MISMATCH, 0);
        assert_eq!(assembly.warning_flags & INNER_WARN_BIT_THETA_TAU, 0);
    }

    #[test]
    fn assemble_rejects_single_coil_tau() {
        let mut data = representative_data();
        data.inductance_sum_delta = [200_000, 0];
        data.inductance_count = [5000, 0];
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples(), TEST_THETA_E_US).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::SingleCoilTau));
    }

    #[test]
    fn assemble_rejects_zero_l_int_with_model_scale_zero() {
        // r_count_slope_milli = 100 (V=512, I=5120 -> matches the old
        // resistance_current_sum=[5_120_000,5_120_000]/sample_count=[1000,1000]
        // single-point average this fixture replaced). avg_delta tuned so
        // per-coil tau rounds to 5us — high enough to clear SingleCoilTau but
        // low enough that l_int = r_int * tau / 1000 and l_uh = r_mohm * tau /
        // 1000 both round to 0.
        // tau = V * dt * SCALE / (avg_delta * r_int)
        //     = 512 * 98 * 1000 / (avg_delta * 100)
        // avg_delta = 100_352 → tau ≈ 5us → l_int = l_uh = 0.
        let mut data = ElectricalIdData::new(512);
        data.resistance_result = Some(resistance_result(100, 100));
        data.inductance_sum_delta = [501_760_000, 501_760_000];
        data.inductance_count = [5000, 5000];
        data.inductance_elapsed_us = [490_000, 490_000];
        let buf = representative_transient();
        let err = assemble_electrical_model(&data, buf.samples(), TEST_THETA_E_US).unwrap_err();
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
        let err = assemble_electrical_model(&data, buf.samples(), TEST_THETA_E_US).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::TauMismatch { .. }));
    }

    #[test]
    fn assemble_warns_on_borderline_resistance_mismatch() {
        // 300 vs 320 → ~6% (above warn 5%, below fail 15%).
        let data = data_with_per_coil_currents(300, 320);
        let buf = representative_transient();
        let assembly = assemble_electrical_model(&data, buf.samples(), TEST_THETA_E_US).unwrap();
        assert_ne!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
    }
}
