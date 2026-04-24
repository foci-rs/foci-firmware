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

/// Number of post-onset samples averaged to estimate initial acceleration.
pub(crate) const MECHANICAL_ID_ACCEL_SAMPLES: usize = 8;

/// Velocity threshold (encoder counts per ms) above which a sample is
/// considered to mark the start of the rotor's response to the torque step.
pub(crate) const MECHANICAL_ID_VELOCITY_ONSET_THRESHOLD: i32 = 2;

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
    fwd_accel: u32,
    rev_accel: u32,
    fwd_peak: u32,
    rev_peak: u32,
) -> (MechanicalModel, u32) {
    let accel_avg = ((fwd_accel.max(1) + rev_accel.max(1)) / 2).max(1);
    let peak_avg = ((fwd_peak + rev_peak) / 2).max(1);
    let j_eff = (torque * 1000) / accel_avg;
    let b_eff = (torque * 1000) / peak_avg;
    let avg_delay_samples = (data.forward_delay_samples + data.reverse_delay_samples) / 2;

    (
        MechanicalModel {
            j_eff,
            b_eff,
            tau_mech_ms: VELOCITY_RESPONSE_VALIDATE_TAU_MS,
            theta_mech_us: (avg_delay_samples as u64 * MECHANICAL_ID_SAMPLE_PERIOD_US) as u32,
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

/// Extract initial acceleration, peak velocity, motion-onset sample
/// index, and total absolute travel from a mechanical-ID transient capture.
///
/// Returns `(initial_accel, peak_velocity, delay_samples, max_travel)`.
/// The caller already knows the reference encoder position and
/// counts-per-revolution and supplies them here so the math can be
/// wrap-aware on the ABN decoder boundary.
pub(crate) fn extract_mechanical_id_metrics(
    buf: &TransientBuffer,
    reference_position: i32,
    counts_per_rev: i32,
) -> (i32, i32, u16, u32) {
    let samples = buf.samples();
    if samples.len() < 2 {
        return (0, 0, 0, 0);
    }

    // Convert absolute encoder samples into per-sample velocity using
    // wrap-aware deltas. The ABN decoder wraps at counts_per_rev, so
    // raw subtraction produces wrong results near the boundary.
    let velocity_at = |idx: usize| -> i32 {
        if idx == 0 {
            encoder_delta(samples[0] as i32, reference_position, counts_per_rev)
        } else {
            encoder_delta(samples[idx] as i32, samples[idx - 1] as i32, counts_per_rev)
        }
    };

    let mut peak_velocity: i32 = 0;
    let mut onset_idx: Option<usize> = None;
    let mut max_travel: u32 = 0;

    // Accumulate unwrapped travel from wrap-aware deltas.
    let mut unwrapped_travel: i32 = 0;

    // Find the first sample with real motion and track the signed peak
    // velocity. Mechanical ID runs against an unloaded motor, but current
    // buildup and static friction can leave a quiet prefix at the start of
    // the capture. Treating that prefix as the acceleration window caused
    // false zero-acceleration failures on hardware.
    for (i, &_sample) in samples.iter().enumerate() {
        let velocity = velocity_at(i);
        unwrapped_travel += velocity;
        let travel = unwrapped_travel.unsigned_abs();
        if travel > max_travel {
            max_travel = travel;
        }
        let abs_cur = velocity.unsigned_abs();
        let abs_peak = peak_velocity.unsigned_abs();
        if abs_cur > abs_peak {
            peak_velocity = velocity;
        }

        if onset_idx.is_none() && abs_cur >= MECHANICAL_ID_VELOCITY_ONSET_THRESHOLD as u32 {
            onset_idx = Some(i);
        }
    }

    let onset_idx = match onset_idx {
        Some(idx) => idx,
        None => return (0, peak_velocity, samples.len() as u16, max_travel),
    };
    let delay_samples = onset_idx as u16;

    // Estimate initial acceleration from the first few samples after
    // motion onset, preserving sign so the caller can compare forward and
    // reverse captures. We use the velocity step to the local peak rather
    // than an average over the whole window so integer quantization cannot
    // round a small but real acceleration down to zero.
    let previous_velocity = if onset_idx == 0 {
        0
    } else {
        velocity_at(onset_idx - 1)
    };

    let direction_positive = velocity_at(onset_idx) >= 0;
    let end = (onset_idx + MECHANICAL_ID_ACCEL_SAMPLES).min(samples.len());
    let mut window_peak = velocity_at(onset_idx);
    for i in onset_idx + 1..end {
        let velocity = velocity_at(i);
        if direction_positive {
            if velocity > window_peak {
                window_peak = velocity;
            }
        } else if velocity < window_peak {
            window_peak = velocity;
        }
    }

    let mut initial_accel = window_peak.wrapping_sub(previous_velocity);
    if initial_accel == 0 {
        initial_accel = velocity_at(onset_idx);
    }

    (initial_accel, peak_velocity, delay_samples, max_travel)
}

#[cfg(test)]
mod tests {
    use super::response_velocity_gains;
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
}
