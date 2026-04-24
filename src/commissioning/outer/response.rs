//! Outer-loop helpers: encoder math, mechanical-ID retry policy, and
//! response-ramp fallbacks.
//!
//! These are pure (or near-pure) helpers shared by the mechanical-ID,
//! velocity-tune, and velocity-validate submodules. They live here
//! rather than alongside any single phase because they are referenced
//! across multiple phases.

use super::{MechanicalIdData, MechanicalIdState, OuterDiagnostics, OuterPhase, OuterSafetyLimits};
use crate::autotune::{MechanicalModel, TransientBuffer, VelocityGains};

/// Torque step attempts as fractions of run_current.
///
/// The ladder climbs from a conservative first pass to retry with stronger
/// bounded torque if the encoder response is too quantized to identify a
/// useful mechanical model. Lowered from (25/40/60) to (15/25/40) after the
/// Stage 1 current-loop tuning pass was strengthened — the previous ladder
/// overshot the outer velocity cap on low-inertia rigs with the new gains.
pub(crate) const MECHANICAL_ID_TORQUE_STEP_PERCENT_0: u32 = 15;
pub(crate) const MECHANICAL_ID_TORQUE_STEP_PERCENT_1: u32 = 25;
pub(crate) const MECHANICAL_ID_TORQUE_STEP_PERCENT_2: u32 = 40;
pub(crate) const MECHANICAL_ID_MAX_ATTEMPTS: u8 = 3;

/// Sampling interval for encoder capture during mechanical ID (microseconds).
/// 1 kHz = 1000us between samples.
pub(crate) const MECHANICAL_ID_SAMPLE_PERIOD_US: u64 = 1000;

/// Velocity threshold (encoder counts per ms) above which a sample is
/// considered to mark the start of the rotor's response to the torque step.
pub(crate) const MECHANICAL_ID_VELOCITY_ONSET_THRESHOLD: i32 = 2;
/// Velocity threshold (encoder counts per ms) below which the zero-torque
/// post-switch settle is considered stationary enough to arm MechanicalId.
///
/// Kept as a dedicated alias so settle qualification can diverge from onset
/// detection later without a cross-cutting rename.
pub(crate) const MECHANICAL_ID_SETTLE_VELOCITY_THRESHOLD: i32 =
    MECHANICAL_ID_VELOCITY_ONSET_THRESHOLD;

/// Onset threshold converted to milli-counts/ms for comparison against the
/// time-normalized velocity estimator used by MechanicalId metric extraction.
pub(crate) const fn mechanical_id_velocity_onset_threshold_mcpms() -> i32 {
    MECHANICAL_ID_VELOCITY_ONSET_THRESHOLD * 1000
}

/// Quality gates for accepting mechanical-ID estimates.
pub(crate) const MECHANICAL_ID_MIN_ACCEL_COUNTS_PER_MS2: u32 = 2;
pub(crate) const MECHANICAL_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS: u32 = 4;
pub(crate) const MECHANICAL_ID_MIN_TRAVEL_COUNTS: u32 = 64;

/// Velocity-loop response-ramp fallback: target step in encoder counts/ms.
pub(crate) const VELOCITY_RESPONSE_STEP_TARGET: u32 = 4;

/// Velocity-loop response-ramp validation time constant (ms).
pub(crate) const VELOCITY_RESPONSE_VALIDATE_TAU_MS: u32 = 50;

// Default fallback velocity gains and the velocity cap currently live in
// `commission.rs` because the still-resident outer step functions reference
// them. They are re-exported here so the response helpers can name them
// without reaching across modules at every site.
pub(crate) use super::{
    OUTER_FALLBACK_VELOCITY_I, OUTER_FALLBACK_VELOCITY_P,
    RESPONSE_FALLBACK_VELOCITY_LIMIT_COUNTS_PER_MS,
};

/// Diagnostic warning code: low-confidence mechanical ID, response ramp
/// fallback was used.
pub(crate) const OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID: u8 = 1;

/// Wrap-aware encoder delta.
///
/// The ABN decoder counter wraps at `counts_per_rev` (PPR * 4). A raw
/// subtraction across the wrap boundary produces a delta near
/// `+/- counts_per_rev` instead of the true small motion. This normalizes
/// the delta to `[-half, +half)`.
pub(crate) fn encoder_delta(current: i32, previous: i32, counts_per_rev: i32) -> i32 {
    let half = counts_per_rev / 2;
    let raw = current.wrapping_sub(previous);
    if raw > half {
        raw - counts_per_rev
    } else if raw < -half {
        raw + counts_per_rev
    } else {
        raw
    }
}

/// Convert a wrap-aware encoder delta into counts per millisecond using the
/// actual elapsed sample time.
///
/// Returns 0 when `dt_us == 0` so same-tick initialization does not create
/// artificial infinite velocity.
pub(crate) fn counts_per_ms(delta_counts: i32, dt_us: u64) -> i32 {
    if dt_us == 0 {
        0
    } else {
        ((delta_counts as i64) * 1000 / dt_us as i64) as i32
    }
}

/// Adaptive torque step percentage for mechanical ID, indexed by retry attempt.
pub(crate) fn mechanical_id_torque_percent(attempt: u8) -> u32 {
    match attempt {
        0 => MECHANICAL_ID_TORQUE_STEP_PERCENT_0,
        1 => MECHANICAL_ID_TORQUE_STEP_PERCENT_1,
        _ => MECHANICAL_ID_TORQUE_STEP_PERCENT_2,
    }
}

/// Compute the torque step amplitude for the given attempt as a u16
/// PIDOUT value, clamped to at least 1.
pub(crate) fn mechanical_id_torque_step(current_limit: u32, attempt: u8) -> u16 {
    let percent = mechanical_id_torque_percent(attempt);
    (current_limit * percent / 100).max(1).min(u16::MAX as u32) as u16
}

/// Returns true if any of the captured mechanical-ID metrics fall below
/// the acceptance gates and the result is therefore considered low
/// confidence.
pub(crate) fn mechanical_id_low_confidence(
    fwd_accel: u32,
    rev_accel: u32,
    fwd_peak: u32,
    rev_peak: u32,
    fwd_travel: u32,
    rev_travel: u32,
) -> bool {
    fwd_accel.min(rev_accel) < MECHANICAL_ID_MIN_ACCEL_COUNTS_PER_MS2
        || fwd_peak.min(rev_peak) < MECHANICAL_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS
        || fwd_travel.min(rev_travel) < MECHANICAL_ID_MIN_TRAVEL_COUNTS
}

/// Try advancing to the next mechanical-ID retry attempt.
///
/// Resets the captured forward/reverse data, rewinds the state machine to
/// `Setup`, and returns `true`. Returns `false` once `MECHANICAL_ID_MAX_ATTEMPTS`
/// is exhausted; the caller is expected to take the response-ramp fallback
/// path in that case.
pub(crate) fn retry_mechanical_id(
    state: &mut MechanicalIdState,
    data: &mut MechanicalIdData,
    reason: &str,
) -> bool {
    if data.attempt + 1 >= MECHANICAL_ID_MAX_ATTEMPTS {
        return false;
    }

    data.attempt += 1;
    data.reset_captures();
    *state = MechanicalIdState::Setup;
    defmt::warn!(
        "mechanical-id: {} on attempt {}, retrying at {}% current",
        reason,
        data.attempt,
        mechanical_id_torque_percent(data.attempt)
    );
    true
}

/// Default velocity gains used by the response-ramp fallback.
pub(crate) fn response_velocity_gains() -> VelocityGains {
    VelocityGains {
        velocity_p: OUTER_FALLBACK_VELOCITY_P,
        velocity_i: OUTER_FALLBACK_VELOCITY_I,
        velocity_limit: RESPONSE_FALLBACK_VELOCITY_LIMIT_COUNTS_PER_MS as u32,
        effective_bandwidth_hz: 0,
        lambda_vel_ms: 0,
    }
}

/// Synthesize a coarse mechanical model from the captured response when
/// the SIMC tuning path is taken with low-confidence mechanical-ID data.
///
/// Returns the model and the average peak velocity (used downstream for
/// velocity-limit derivation).
pub(crate) fn response_mechanical_model(
    data: &MechanicalIdData,
    torque: u32,
    fwd_accel: i32,
    rev_accel: i32,
    fwd_peak: u32,
    rev_peak: u32,
) -> (MechanicalModel, u32) {
    let accel_avg =
        ((fwd_accel.unsigned_abs().max(1) + rev_accel.unsigned_abs().max(1)) / 2).max(1);
    let peak_avg = ((fwd_peak + rev_peak) / 2).max(1);
    let j_eff = (torque * 1000) / accel_avg;
    let b_eff = (torque * 1000) / peak_avg;
    let avg_delay_us = (data.forward_delay_us + data.reverse_delay_us) / 2;

    (
        MechanicalModel {
            j_eff,
            b_eff,
            tau_mech_ms: VELOCITY_RESPONSE_VALIDATE_TAU_MS,
            theta_mech_us: avg_delay_us,
            t_f: 0,
        },
        peak_avg,
    )
}

/// Response-ramp fallback when the slope estimator reported low confidence.
///
/// Substitutes J_eff with `J_EFF_CONSERVATIVE_FALLBACK` (the quantity
/// derived from the untrusted accel measurement). B_eff is derived
/// consistently from J_eff and the fixed `VELOCITY_RESPONSE_VALIDATE_TAU_MS`
/// so the j/b ratio remains in the bounded conservative band. Measured
/// peak velocity and measured delay flow through unchanged — they retain
/// their "motion happened" semantics.
pub(crate) fn response_mechanical_model_low_confidence(
    peak_avg: u32,
    avg_delay_us: u32,
) -> (MechanicalModel, u32) {
    use crate::commissioning::outer::slope_estimator::J_EFF_CONSERVATIVE_FALLBACK;

    let peak_avg = peak_avg.max(1);
    let j_eff = J_EFF_CONSERVATIVE_FALLBACK;
    let b_eff = (j_eff / VELOCITY_RESPONSE_VALIDATE_TAU_MS).max(1);
    (
        MechanicalModel {
            j_eff,
            b_eff,
            tau_mech_ms: VELOCITY_RESPONSE_VALIDATE_TAU_MS,
            theta_mech_us: avg_delay_us,
            t_f: 0,
        },
        peak_avg,
    )
}

/// Build a `VelocityTune` outer phase carrying the response-ramp
/// fallback flag in diagnostics.
pub(crate) fn response_velocity_phase(
    model: MechanicalModel,
    peak_velocity: u32,
    mut diagnostics: OuterDiagnostics,
    safety: OuterSafetyLimits,
) -> OuterPhase {
    diagnostics.warning_code = OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID;
    OuterPhase::VelocityTune {
        model,
        peak_velocity,
        diagnostics,
        safety,
    }
}

/// Time-aware metrics extracted from a MechanicalId transient capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct MechanicalIdMetrics {
    pub(crate) accel: crate::commissioning::outer::slope_estimator::SlopeEstimate,
    pub(crate) peak_velocity: i32,
    pub(crate) delay_us: u32,
    pub(crate) travel: u32,
}

/// Extract slope estimate, peak velocity, motion-onset delay, and total
/// absolute travel from a mechanical-ID transient capture.
///
/// Accel + confidence is delegated to `estimate_early_slope`; peak velocity,
/// delay, and travel are computed inline. The caller supplies the reference
/// encoder position and counts-per-revolution so all math is wrap-aware on
/// the ABN decoder boundary.
pub(crate) fn extract_mechanical_id_metrics(
    buf: &TransientBuffer,
    timestamps_us: &[u64],
    capture_start_us: u64,
    reference_position: i32,
    counts_per_rev: i32,
) -> MechanicalIdMetrics {
    use crate::commissioning::outer::slope_estimator::{SlopeEstimate, estimate_early_slope};

    let samples = buf.samples();
    debug_assert_eq!(timestamps_us.len(), samples.len());
    if samples.len() < 2 || timestamps_us.len() != samples.len() {
        defmt::error!(
            "mechanical-id: ts/sample length desync: {}/{}",
            timestamps_us.len(),
            samples.len()
        );
        return MechanicalIdMetrics {
            accel: SlopeEstimate::default(),
            peak_velocity: 0,
            delay_us: 0,
            travel: 0,
        };
    }

    // Accel + confidence is delegated.
    let accel = estimate_early_slope(
        buf,
        timestamps_us,
        capture_start_us,
        reference_position,
        counts_per_rev,
    );

    // Peak velocity, delay_us, and travel keep their existing extraction.
    let mut peak_velocity_mcpms: i32 = 0;
    let mut onset_idx: Option<usize> = None;
    let mut max_travel: u32 = 0;
    let mut unwrapped_travel: i32 = 0;

    for i in 0..samples.len() {
        let delta_counts = encoder_delta(
            samples[i] as i32,
            if i == 0 {
                reference_position
            } else {
                samples[i - 1] as i32
            },
            counts_per_rev,
        );
        let dt_us = if i == 0 {
            timestamps_us[i].saturating_sub(capture_start_us)
        } else {
            timestamps_us[i].saturating_sub(timestamps_us[i - 1])
        };
        let v_mcpms = if dt_us == 0 {
            0
        } else {
            ((delta_counts as i64) * 1_000_000 / dt_us as i64) as i32
        };
        unwrapped_travel += delta_counts;
        let travel = unwrapped_travel.unsigned_abs();
        if travel > max_travel {
            max_travel = travel;
        }
        if v_mcpms.unsigned_abs() > peak_velocity_mcpms.unsigned_abs() {
            peak_velocity_mcpms = v_mcpms;
        }
        if onset_idx.is_none()
            && v_mcpms.unsigned_abs() >= mechanical_id_velocity_onset_threshold_mcpms() as u32
        {
            onset_idx = Some(i);
        }
    }

    let delay_us = match onset_idx {
        Some(idx) => timestamps_us[idx].saturating_sub(capture_start_us) as u32,
        None => timestamps_us
            .last()
            .map(|ts| ts.saturating_sub(capture_start_us) as u32)
            .unwrap_or(0),
    };

    MechanicalIdMetrics {
        accel,
        peak_velocity: round_div_signed_i32(peak_velocity_mcpms, 1000),
        delay_us,
        travel: max_travel,
    }
}

fn round_div_signed_i32(numerator: i32, denominator: i32) -> i32 {
    round_div_signed_i128(numerator as i128, denominator as i128) as i32
}

fn round_div_signed_i128(numerator: i128, denominator: i128) -> i128 {
    if denominator == 0 {
        return 0;
    }
    let half = denominator.abs() / 2;
    if numerator >= 0 {
        (numerator + half) / denominator
    } else {
        (numerator - half) / denominator
    }
}

#[cfg(test)]
mod tests {
    use super::{extract_mechanical_id_metrics, response_velocity_gains};
    use crate::autotune::TransientBuffer;
    use crate::commissioning::outer::{
        MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, RESPONSE_FALLBACK_VELOCITY_LIMIT_COUNTS_PER_MS,
    };

    #[test]
    fn response_velocity_gains_keep_conservative_runtime_limit() {
        let gains = response_velocity_gains();
        assert_eq!(
            gains.velocity_limit,
            RESPONSE_FALLBACK_VELOCITY_LIMIT_COUNTS_PER_MS as u32
        );
        assert_ne!(
            gains.velocity_limit,
            MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS as u32
        );
    }

    #[test]
    fn extract_mechanical_id_metrics_estimates_initial_slope_not_window_peak() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for sample in [2_u32, 8, 18, 32, 50] {
            assert!(buf.push(sample));
        }

        let timestamps = [1000_u64, 2000, 3000, 4000, 5000];
        let metrics = extract_mechanical_id_metrics(&buf, &timestamps, 0, 0, 4000);

        assert_eq!(
            metrics.accel.accel,
            4, // was initial_accel
            "initial acceleration should match the fitted early slope"
        );
        assert!(
            metrics.accel.is_high_confidence(),
            "expected high confidence"
        );
        assert_eq!(metrics.peak_velocity, 18);
        assert_eq!(metrics.delay_us, 1000);
        assert_eq!(metrics.travel, 50);
    }

    #[test]
    fn extract_mechanical_id_metrics_is_stable_under_small_quantization_noise() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for sample in [2_u32, 8, 17, 33, 50] {
            assert!(buf.push(sample));
        }

        let timestamps = [1000_u64, 2000, 3000, 4000, 5000];
        let metrics = extract_mechanical_id_metrics(&buf, &timestamps, 0, 0, 4000);

        assert_eq!(
            metrics.accel.accel,
            4, // was initial_accel
            "least-squares fit should remain stable under small quantization perturbations"
        );
        assert!(
            metrics.accel.is_high_confidence(),
            "expected high confidence"
        );
        assert_eq!(metrics.peak_velocity, 17);
    }
}
