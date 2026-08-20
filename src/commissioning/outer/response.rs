//! Outer-loop helpers: encoder math, mechanical-ID retry policy, and
//! response-ramp fallbacks.
//!
//! These are pure (or near-pure) helpers shared by the mechanical-ID,
//! velocity-tune, and velocity-validate submodules. They live here
//! rather than alongside any single phase because they are referenced
//! across multiple phases.

use super::{MechanicalIdData, MechanicalIdState, OuterDiagnostics, OuterPhase, OuterSafetyLimits};
use crate::autotune::{MechanicalModel, TransientBuffer, VelocityGains};
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain};
use foci_tmc4671::pid::{PidIGain, PidPGain};

/// Torque step attempts as fractions of run_current.
///
/// The ladder climbs from a conservative first pass to retry with stronger
/// bounded torque if the encoder response is too quantized to identify a
/// useful mechanical model.
pub(crate) const MECHANICAL_ID_TORQUE_STEP_PERCENT_0: u32 = 15;
pub(crate) const MECHANICAL_ID_TORQUE_STEP_PERCENT_1: u32 = 25;
pub(crate) const MECHANICAL_ID_TORQUE_STEP_PERCENT_2: u32 = 40;
pub(crate) const MECHANICAL_ID_MAX_ATTEMPTS: u8 = 3;

/// Sampling interval for encoder capture during mechanical ID (microseconds).
/// 1 kHz = 1000us between samples.
pub(crate) const MECHANICAL_ID_SAMPLE_PERIOD_US: u64 = 1000;

/// Velocity threshold below which the zero-torque post-switch settle is
/// considered stationary enough to arm MechanicalId.
///
/// The physical threshold is 0.5 rev/s, rounded up to whole counts/ms for
/// the settle path's integer velocity samples.
pub(crate) const fn mechanical_id_settle_velocity_threshold_counts_per_ms(
    counts_per_rev: u32,
) -> u32 {
    let threshold = counts_per_rev.div_ceil(2000);
    if threshold > 0 { threshold } else { 1 }
}

/// Onset threshold in milli-counts/ms for comparison against the
/// time-normalized velocity estimator used by MechanicalId metric extraction.
///
/// The physical threshold is 0.5 rev/s.
pub(crate) const fn mechanical_id_velocity_onset_threshold_mcpms(counts_per_rev: u32) -> u32 {
    let threshold = counts_per_rev.div_ceil(2);
    if threshold > 0 { threshold } else { 1 }
}

/// Quality gate for accepting mechanical-ID estimates. The minimum-peak-velocity
/// and minimum-travel gates moved to `super::limits` -- the shared outer-safety
/// envelope depends on them through `OuterSafetyLimits::budget_is_usable`.
pub(crate) const MECHANICAL_ID_MIN_ACCEL_COUNTS_PER_MS2: u32 = 2;

/// Velocity-loop response-ramp fallback: target step in encoder counts/ms.
pub(crate) const VELOCITY_RESPONSE_STEP_TARGET: u32 = 4;

/// Velocity-loop response-ramp validation time constant (ms).
pub(crate) const VELOCITY_RESPONSE_VALIDATE_TAU_MS: u32 = 50;

/// Initial P for response-ramp tuning.
///
/// This is deliberately lower than the runtime fallback. The ramp is a search:
/// start with a low-energy validation step, then increase P until the first
/// clean response is found and finally stop before the response turns unsafe.
pub(crate) const VELOCITY_RESPONSE_INITIAL_P: u16 = 128;
pub(crate) const VELOCITY_RESPONSE_INITIAL_GAIN: PidPGain =
    match PidPGain::new(VELOCITY_RESPONSE_INITIAL_P) {
        Ok(gain) => gain,
        Err(_) => panic!("response velocity P must be valid"),
    };
const OUTER_FALLBACK_VELOCITY_I_GAIN: PidIGain = match PidIGain::new(OUTER_FALLBACK_VELOCITY_I) {
    Ok(gain) => gain,
    Err(_) => panic!("response velocity I must be valid"),
};

const FULL_RESPONSE_TAIL_SAMPLES: usize = 4;
const FULL_RESPONSE_MIN_TAIL_SAMPLES: u8 = 4;
const FULL_RESPONSE_MAX_TAIL_RESIDUAL_PCT: u32 = 15;

// Default fallback velocity gains and the velocity cap currently live in
// `commission.rs` because the still-resident outer step functions reference
// them. They are re-exported here so the response helpers can name them
// without reaching across modules at every site.
pub(crate) use super::{OUTER_FALLBACK_VELOCITY_I, position_tune_step_counts};

/// Diagnostic warning code: low-confidence mechanical ID, response ramp
/// fallback was used.
pub(crate) const OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID: u8 = 1;

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
pub(crate) fn response_velocity_gains(counts_per_rev: u32) -> VelocityGains {
    VelocityGains {
        velocity_p: VELOCITY_RESPONSE_INITIAL_GAIN,
        velocity_i: OUTER_FALLBACK_VELOCITY_I_GAIN,
        velocity_limit: super::response_fallback_velocity_limit_counts_per_ms(counts_per_rev),
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
    diagnostics.mech_tau_defaulted = true;
    OuterPhase::VelocityTune {
        model,
        peak_velocity,
        diagnostics,
        safety,
    }
}

/// Time-aware metrics extracted from a MechanicalId transient capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct MechanicalIdMetrics {
    pub(crate) accel: crate::commissioning::outer::slope_estimator::SlopeEstimate,
    pub(crate) peak_velocity: i32,
    pub(crate) delay_us: u32,
    pub(crate) travel: u32,
}

/// Mechanical model estimate from the full torque-step velocity response.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub(crate) struct MechanicalResponseEstimate {
    pub(crate) j_eff: u32,
    pub(crate) b_eff: u32,
    pub(crate) tau_mech_ms: u32,
    pub(crate) theta_mech_us: u32,
    pub(crate) steady_velocity_cpms: i32,
    pub(crate) travel: u32,
    pub(crate) residual_mcpms: u32,
    pub(crate) fit_sample_count: u8,
    pub(crate) sign_match: bool,
}

impl MechanicalResponseEstimate {
    pub(crate) fn is_high_confidence(&self) -> bool {
        self.fit_sample_count >= FULL_RESPONSE_MIN_TAIL_SAMPLES
            && self.sign_match
            && self.steady_velocity_cpms.unsigned_abs()
                >= MECHANICAL_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS
            && self.travel >= MECHANICAL_ID_MIN_TRAVEL_COUNTS
            && self.residual_mcpms
                <= (self.steady_velocity_cpms.unsigned_abs()
                    * 1000
                    * FULL_RESPONSE_MAX_TAIL_RESIDUAL_PCT
                    / 100)
            && self.b_eff > 0
            && self.j_eff > 0
    }
}

/// Extract a first-order mechanical model from the whole torque-step response.
///
/// This path uses measured steady velocity for damping and the 63% rise time
/// for the time constant, so low-inertia rigs that reach steady velocity before
/// the early-slope estimator gets a clean fit can still produce a usable model.
pub(crate) fn extract_mechanical_response_model(
    buf: &TransientBuffer,
    timestamps_us: &[u64],
    capture_start_us: u64,
    reference_position: EncoderCount,
    encoder_domain: EncoderDomain,
    torque: u32,
) -> MechanicalResponseEstimate {
    let samples = buf.samples();
    debug_assert_eq!(timestamps_us.len(), samples.len());
    if samples.len() < 2 || timestamps_us.len() != samples.len() || torque == 0 {
        return MechanicalResponseEstimate::default();
    }

    let mut velocities_mcpms = [0_i32; crate::autotune::TRANSIENT_LEN];
    let mut peak_velocity_mcpms: i32 = 0;
    let mut onset_idx: Option<usize> = None;
    let mut unwrapped_travel: i64 = 0;
    let mut max_travel: u32 = 0;
    let threshold_mcpms =
        mechanical_id_velocity_onset_threshold_mcpms(encoder_domain.counts_per_rev());
    let mut previous_count = reference_position;

    for i in 0..samples.len() {
        let previous_ts = if i == 0 {
            capture_start_us
        } else {
            timestamps_us[i - 1]
        };
        let current_count = encoder_domain.count(samples[i]);
        let delta = match encoder_domain.delta(previous_count, current_count) {
            Ok(delta) => delta.signed_counts(),
            Err(_) => return MechanicalResponseEstimate::default(),
        };
        previous_count = current_count;
        let dt_us = timestamps_us[i].saturating_sub(previous_ts);
        let v_mcpms = if dt_us == 0 {
            0
        } else {
            ((delta as i64) * 1_000_000 / dt_us as i64) as i32
        };
        velocities_mcpms[i] = v_mcpms;

        unwrapped_travel += i64::from(delta);
        let travel = unwrapped_travel.unsigned_abs().min(u64::from(u32::MAX)) as u32;
        max_travel = max_travel.max(travel);
        if v_mcpms.unsigned_abs() > peak_velocity_mcpms.unsigned_abs() {
            peak_velocity_mcpms = v_mcpms;
        }
        if onset_idx.is_none() && v_mcpms.unsigned_abs() >= threshold_mcpms {
            onset_idx = Some(i);
        }
    }

    let onset_idx = match onset_idx {
        Some(idx) => idx,
        None => return MechanicalResponseEstimate::default(),
    };

    let tail_start = samples
        .len()
        .saturating_sub(FULL_RESPONSE_TAIL_SAMPLES)
        .max(onset_idx);
    let tail_len = samples.len() - tail_start;
    if tail_len < FULL_RESPONSE_MIN_TAIL_SAMPLES as usize {
        return MechanicalResponseEstimate::default();
    }

    let tail_sum: i64 = velocities_mcpms[tail_start..samples.len()]
        .iter()
        .map(|&v| v as i64)
        .sum();
    let steady_mcpms = round_div_signed_i128(tail_sum as i128, tail_len as i128) as i32;
    let steady_abs_mcpms = steady_mcpms.unsigned_abs();
    if steady_abs_mcpms < threshold_mcpms {
        return MechanicalResponseEstimate::default();
    }

    let mut residual_sq: i128 = 0;
    for &v in &velocities_mcpms[tail_start..samples.len()] {
        let err = v as i128 - steady_mcpms as i128;
        residual_sq = residual_sq.saturating_add(err * err);
    }
    let mean_sq = residual_sq / tail_len as i128;
    let residual_mcpms = mean_sq.max(0) as u128;
    let residual_mcpms = residual_mcpms.isqrt() as u32;

    let target_mcpms = (steady_abs_mcpms as u64 * 632 / 1000).max(threshold_mcpms as u64);
    let mut tau_cross_idx = onset_idx;
    for (offset, &v) in velocities_mcpms[onset_idx..samples.len()]
        .iter()
        .enumerate()
    {
        if v.unsigned_abs() as u64 >= target_mcpms {
            tau_cross_idx = onset_idx + offset;
            break;
        }
    }
    let tau_us = timestamps_us[tau_cross_idx].saturating_sub(timestamps_us[onset_idx]);
    let tau_mech_ms = ((tau_us + 500) / 1000).max(1) as u32;
    let steady_velocity_cpms = round_div_signed_i32(steady_mcpms, 1000);
    let steady_abs_cpms = steady_velocity_cpms.unsigned_abs().max(1);
    let b_eff = (torque * 1000 / steady_abs_cpms).max(1);
    let j_eff = b_eff.saturating_mul(tau_mech_ms).max(1);
    let sign_match = steady_mcpms.signum() == peak_velocity_mcpms.signum();

    MechanicalResponseEstimate {
        j_eff,
        b_eff,
        tau_mech_ms,
        theta_mech_us: timestamps_us[onset_idx].saturating_sub(capture_start_us) as u32,
        steady_velocity_cpms,
        travel: max_travel,
        residual_mcpms,
        fit_sample_count: tail_len as u8,
        sign_match,
    }
}

/// Extract slope estimate, peak velocity, motion-onset delay, and total
/// absolute travel from a mechanical-ID transient capture.
///
/// Accel + confidence is delegated to `estimate_early_slope`; peak velocity,
/// delay, and travel are computed inline. The caller supplies a normalized
/// reference position and its validated domain so all displacement uses the
/// shared shortest-path encoder arithmetic.
pub(crate) fn extract_mechanical_id_metrics(
    buf: &TransientBuffer,
    timestamps_us: &[u64],
    capture_start_us: u64,
    reference_position: EncoderCount,
    encoder_domain: EncoderDomain,
) -> MechanicalIdMetrics {
    use crate::commissioning::outer::slope_estimator::estimate_early_slope;

    let samples = buf.samples();
    debug_assert_eq!(timestamps_us.len(), samples.len());
    if samples.len() < 2 || timestamps_us.len() != samples.len() {
        defmt::error!(
            "mechanical-id: ts/sample length desync: {}/{}",
            timestamps_us.len(),
            samples.len()
        );
        return MechanicalIdMetrics::default();
    }

    // Accel + confidence is delegated.
    let accel = estimate_early_slope(
        buf,
        timestamps_us,
        capture_start_us,
        reference_position,
        encoder_domain,
    );

    // Peak velocity, delay_us, and travel keep their existing extraction.
    let mut peak_velocity_mcpms: i32 = 0;
    let mut onset_idx: Option<usize> = None;
    let mut max_travel: u32 = 0;
    let mut unwrapped_travel: i64 = 0;
    let mut previous_count = reference_position;

    for i in 0..samples.len() {
        let current_count = encoder_domain.count(samples[i]);
        let delta_counts = match encoder_domain.delta(previous_count, current_count) {
            Ok(delta) => delta.signed_counts(),
            Err(_) => return MechanicalIdMetrics::default(),
        };
        previous_count = current_count;
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
        unwrapped_travel += i64::from(delta_counts);
        let travel = unwrapped_travel.unsigned_abs().min(u64::from(u32::MAX)) as u32;
        if travel > max_travel {
            max_travel = travel;
        }
        if v_mcpms.unsigned_abs() > peak_velocity_mcpms.unsigned_abs() {
            peak_velocity_mcpms = v_mcpms;
        }
        if onset_idx.is_none()
            && v_mcpms.unsigned_abs()
                >= mechanical_id_velocity_onset_threshold_mcpms(encoder_domain.counts_per_rev())
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
    use super::{
        MechanicalIdMetrics, MechanicalResponseEstimate, mechanical_id_torque_percent,
        mechanical_id_velocity_onset_threshold_mcpms, position_tune_step_counts,
        response_velocity_gains,
    };
    use crate::autotune::TransientBuffer;
    use crate::commissioning::encoder_domain::EncoderDomain;
    use crate::commissioning::outer::MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS;

    fn extract_mechanical_id_metrics(
        buf: &TransientBuffer,
        timestamps_us: &[u64],
        capture_start_us: u64,
        reference_position: u32,
        counts_per_rev: u32,
    ) -> MechanicalIdMetrics {
        let domain = EncoderDomain::new(counts_per_rev).unwrap();
        super::extract_mechanical_id_metrics(
            buf,
            timestamps_us,
            capture_start_us,
            domain.count(reference_position),
            domain,
        )
    }

    fn extract_mechanical_response_model(
        buf: &TransientBuffer,
        timestamps_us: &[u64],
        capture_start_us: u64,
        reference_position: u32,
        counts_per_rev: u32,
        torque: u32,
    ) -> MechanicalResponseEstimate {
        let domain = EncoderDomain::new(counts_per_rev).unwrap();
        super::extract_mechanical_response_model(
            buf,
            timestamps_us,
            capture_start_us,
            domain.count(reference_position),
            domain,
            torque,
        )
    }

    #[test]
    fn response_velocity_gains_keep_conservative_runtime_limit() {
        let gains = response_velocity_gains(4_000);
        assert_eq!(gains.velocity_limit, 16);
        assert_ne!(
            gains.velocity_limit,
            MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS
        );
    }

    #[test]
    fn response_velocity_gains_scale_runtime_limit_from_encoder_resolution() {
        let gains = response_velocity_gains(8_000);

        assert_eq!(gains.velocity_limit, 32);
    }

    #[test]
    fn position_tune_step_scales_from_encoder_resolution() {
        assert_eq!(position_tune_step_counts(4_000), 8);
        assert_eq!(position_tune_step_counts(8_000), 16);
    }

    #[test]
    fn mechanical_id_onset_threshold_scales_from_encoder_resolution() {
        assert_eq!(mechanical_id_velocity_onset_threshold_mcpms(4_000), 2_000);
        assert_eq!(mechanical_id_velocity_onset_threshold_mcpms(8_000), 4_000);
    }

    #[test]
    fn mechanical_id_torque_ladder_starts_with_gentle_excitation() {
        assert_eq!(mechanical_id_torque_percent(0), 15);
        assert_eq!(mechanical_id_torque_percent(1), 25);
        assert_eq!(mechanical_id_torque_percent(2), 40);
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

    #[test]
    fn extract_mechanical_response_model_uses_full_transient() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        // Position samples generated from a response whose velocity rises
        // toward an 18 counts/ms steady state.
        for sample in [0_u32, 4, 12, 23, 36, 51, 67, 84, 102, 120, 138, 156] {
            assert!(buf.push(sample));
        }
        let timestamps = [
            1_000_u64, 2_000, 3_000, 4_000, 5_000, 6_000, 7_000, 8_000, 9_000, 10_000, 11_000,
            12_000,
        ];

        let estimate = extract_mechanical_response_model(&buf, &timestamps, 0, 0, 4000, 360);

        assert!(
            estimate.is_high_confidence(),
            "expected high-confidence full-response model: {:?}",
            estimate,
        );
        assert_eq!(estimate.steady_velocity_cpms, 18);
        assert_eq!(estimate.b_eff, 20_000);
        assert!((2..=6).contains(&estimate.tau_mech_ms));
        assert_eq!(estimate.j_eff, estimate.b_eff * estimate.tau_mech_ms);
    }

    #[test]
    fn response_metrics_preserve_forward_motion_across_encoder_origin() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for sample in [3_995_u32, 3_998, 3, 10, 19, 30, 43, 58] {
            assert!(buf.push(sample));
        }
        let timestamps = [0_u64, 1_000, 2_000, 3_000, 4_000, 5_000, 6_000, 7_000];

        let metrics = extract_mechanical_id_metrics(&buf, &timestamps, 0, 3_995, 4_000);

        assert!(metrics.accel.accel > 0);
        assert!(metrics.peak_velocity > 0);
        assert_eq!(metrics.travel, 63);
    }

    #[test]
    fn half_turn_discards_entire_response_before_later_samples() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for sample in [4_u32, 8, 2_008, 3_900, 3_800, 3_700, 3_600, 3_500] {
            assert!(buf.push(sample));
        }
        let timestamps = [1_000_u64, 2_000, 3_000, 4_000, 5_000, 6_000, 7_000, 8_000];

        let metrics = extract_mechanical_id_metrics(&buf, &timestamps, 0, 0, 4_000);
        let estimate = extract_mechanical_response_model(&buf, &timestamps, 0, 0, 4_000, 360);

        assert_eq!(metrics.accel, Default::default());
        assert_eq!(metrics.peak_velocity, 0);
        assert_eq!(metrics.delay_us, 0);
        assert_eq!(metrics.travel, 0);
        assert_eq!(estimate, Default::default());
    }
}
