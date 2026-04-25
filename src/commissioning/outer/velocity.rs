//! Phase 9: Velocity-loop SIMC gain synthesis.
//! Phase 10: Velocity-loop bidirectional step-response validation.
//!
//! Phase 9 (`step_velocity_tune`) is a thin pass-through over the SIMC
//! synthesis helper. Phase 10 (`step_velocity_validate`) performs the
//! actual closed-loop step responses, classifies them, and either
//! accepts the gains, retries with a relaxed lambda, or falls back to
//! conservative response-ramp tuning.

use super::{
    OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P, OUTER_WARNING_UNSAFE_SYNTH_GAINS, OuterPhase,
    POS_TUNE_INITIAL_P, POS_TUNE_STEP_COUNTS, PositionTuneData, PositionTuneState,
    VelocityTuneStrategy, VelocityValidateData, VelocityValidateState,
};
use crate::autotune::{
    CommissionError, CommissionProfile, OperatingMode, PhaseId, TransientBuffer, ValidationResult,
    VelocityValidation,
};
use crate::commissioning::outer::{fail_outer, response};
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Velocity step as a fraction of velocity_limit (15% = 15/100).
/// Spec says 10-20%.
const VELOCITY_VALIDATE_STEP_PERCENT: u32 = 15;

/// Sampling interval for velocity capture (microseconds).
/// 1 kHz matches mechanical ID and the SIMC validation cadence.
const VELOCITY_VALIDATE_SAMPLE_PERIOD_US: u64 = 1000;

/// Settling time after a velocity step (microseconds).
const VELOCITY_VALIDATE_SETTLE_US: u64 = 200_000;

/// Number of zero-speed samples used to detect chatter.
const VELOCITY_VALIDATE_CHATTER_WINDOW: u16 = 200;

/// Encoder-counts/ms threshold above which a zero-speed sample counts as chatter.
const VELOCITY_VALIDATE_CHATTER_THRESHOLD: u32 = 3;

/// Lambda multiplier (numerator/denominator) applied on a marginal retry.
const VELOCITY_RETRY_LAMBDA_MUL_NUM: u32 = 3;
const VELOCITY_RETRY_LAMBDA_MUL_DEN: u32 = 2;

/// Response-ramp P gain multiplier between attempts.
const VELOCITY_RESPONSE_P_MUL_NUM: u32 = 3;
const VELOCITY_RESPONSE_P_MUL_DEN: u32 = 2;
const VELOCITY_RESPONSE_BREAKAWAY_OVERSHOOT_COUNTS: u32 = 4;

/// Velocity mode with `MODE_PID_TYPE` set for the TMC4671 advanced PI structure.
const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;

fn counts_per_ms_to_tmc_velocity(counts_per_ms: u32, step_width: u32, f_pwm_hz: u32) -> u32 {
    if counts_per_ms == 0 || step_width == 0 || f_pwm_hz == 0 {
        return 0;
    }

    // TMC velocity registers use position-units per PWM period. Validation
    // metrics use encoder counts/ms, and STEP_WIDTH is position-units/count.
    let numerator = counts_per_ms as u64 * step_width as u64 * 1000;
    let raw = numerator.div_ceil(f_pwm_hz as u64);
    raw.min(u32::MAX as u64) as u32
}

fn response_next_velocity_p(current_p: u16) -> Option<u16> {
    let next_p = current_p as u32 * VELOCITY_RESPONSE_P_MUL_NUM / VELOCITY_RESPONSE_P_MUL_DEN;
    if next_p > OUTER_MAX_VELOCITY_P as u32 || next_p <= current_p as u32 {
        None
    } else {
        Some(next_p as u16)
    }
}

fn response_result_is_gain_limited(
    metrics: &VelocityValidation,
    fwd: &VelocityValidation,
    rev: &VelocityValidation,
    target: u32,
    total_samples: u16,
) -> bool {
    let sat_pct = if total_samples > 0 {
        (metrics.saturation_samples as u32 * 100) / total_samples as u32
    } else {
        0
    };

    let min_peak = fwd.peak_velocity.min(rev.peak_velocity);
    let max_peak = fwd.peak_velocity.max(rev.peak_velocity);
    let breakaway_stiction = min_peak < target
        && max_peak <= target.saturating_add(VELOCITY_RESPONSE_BREAKAWAY_OVERSHOOT_COUNTS);

    // Before the first passing response, only keep increasing P when the
    // capture looks under-driven. A one-sided breakaway hop is also
    // gain-limited: one direction moved a little, the other never overcame
    // stiction. Saturation/chatter still mean the gain is unsafe.
    (metrics.overshoot_pct <= 200 || breakaway_stiction)
        && sat_pct <= 10
        && !metrics.zero_speed_chatter
}

/// Phase 9: synthesize velocity gains and transition to validation.
///
/// If the mechanical-ID warning indicates low confidence, take the
/// response-ramp path with conservative defaults. Otherwise run the
/// SIMC synthesis; if it produces gains outside the safety envelope,
/// fall back to response-ramp tuning.
pub(crate) async fn step_velocity_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    backend: &mut B,
) -> StepResult {
    let (model, peak_velocity, mut diagnostics, safety) = match phase {
        OuterPhase::VelocityTune {
            model,
            peak_velocity,
            diagnostics,
            safety,
        } => (*model, *peak_velocity, *diagnostics, *safety),
        _ => return StepResult::Continue,
    };

    let (gains, strategy, step_target) =
        if diagnostics.warning_code == response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID {
            let gains = response::response_velocity_gains();
            (
                gains,
                VelocityTuneStrategy::ResponseRamp,
                response::VELOCITY_RESPONSE_STEP_TARGET,
            )
        } else {
            let config = backend.tmc_config();
            let step_width = backend.step_width();
            let gains = match crate::autotune::simc_velocity_gains(
                &model,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                peak_velocity,
                step_width,
                config,
            ) {
                Some(gains) => gains,
                None => {
                    defmt::warn!("velocity tune: synthesis returned None");
                    return fail_outer(phase, CommissionError::MechanicalIdFailed);
                }
            };

            diagnostics.synth_velocity_p = gains.velocity_p;
            diagnostics.synth_velocity_i = gains.velocity_i;

            if gains.velocity_p > OUTER_MAX_VELOCITY_P || gains.velocity_i > OUTER_MAX_VELOCITY_I {
                diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
                defmt::warn!(
                    "velocity tune: synthesized P={}, I={} outside envelope, using response tuning",
                    gains.velocity_p,
                    gains.velocity_i
                );
                (
                    response::response_velocity_gains(),
                    VelocityTuneStrategy::ResponseRamp,
                    response::VELOCITY_RESPONSE_STEP_TARGET,
                )
            } else {
                let step_target = if gains.velocity_limit > 0 {
                    (gains.velocity_limit * VELOCITY_VALIDATE_STEP_PERCENT / 100).max(1)
                } else {
                    return fail_outer(phase, CommissionError::VelocityValidationFailed);
                };
                (gains, VelocityTuneStrategy::Model, step_target)
            }
        };

    defmt::info!(
        "velocity tune: P={}, I={}, limit={}, bw={}Hz, lambda={}ms, strategy={}",
        gains.velocity_p,
        gains.velocity_i,
        gains.velocity_limit,
        gains.effective_bandwidth_hz,
        gains.lambda_vel_ms,
        match strategy {
            VelocityTuneStrategy::Model => "model",
            VelocityTuneStrategy::ResponseRamp => "response",
        }
    );

    let data = VelocityValidateData {
        gains,
        accepted_gains: gains,
        has_accepted_gains: false,
        model,
        strategy,
        attempt: 0,
        step_target,
        forward_metrics: None,
        reverse_metrics: None,
        max_chatter: 0,
        peak_velocity,
        prev_encoder_for_velocity: 0,
        saturation_count: 0,
        torque_limit: backend.current_limit() & 0xFFFF,
        operational_velocity_limit: backend.operational_velocity_limit(),
        diagnostics,
    };

    *phase = OuterPhase::VelocityValidate {
        state: VelocityValidateState::WriteGains,
        data,
        safety,
    };
    StepResult::PhaseChanged(PhaseId::VelocityValidate)
}

/// Phase 10: bidirectional velocity step-response validation.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_velocity_validate<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (state, data, safety) = match phase {
        OuterPhase::VelocityValidate {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::Continue,
    };

    match state {
        VelocityValidateState::WriteGains => {
            let step_width = backend.step_width();
            let f_pwm_hz = backend.tmc_config().pwm_freq_hz();
            let velocity_limit_reg =
                counts_per_ms_to_tmc_velocity(data.gains.velocity_limit, step_width, f_pwm_hz)
                    .max(1);
            let step_target_reg =
                counts_per_ms_to_tmc_velocity(data.step_target, step_width, f_pwm_hz).max(1);

            let vel_reg = ((data.gains.velocity_p as u32) << 16) | (data.gains.velocity_i as u32);
            if backend
                .write_register(PID_VELOCITY_P_VELOCITY_I, vel_reg)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            if backend
                .write_register(PID_VELOCITY_LIMIT, velocity_limit_reg)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            if backend
                .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_MODE)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;

            defmt::info!(
                "vel validate: limit={}cpms/{}tmc step={}cpms/{}tmc",
                data.gains.velocity_limit,
                velocity_limit_reg,
                data.step_target,
                step_target_reg
            );

            buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
            data.forward_metrics = None;
            data.reverse_metrics = None;
            data.max_chatter = 0;
            data.saturation_count = 0;

            *state = VelocityValidateState::ForwardStep;
            StepResult::Continue
        }

        VelocityValidateState::ForwardStep => {
            let target_reg = counts_per_ms_to_tmc_velocity(
                data.step_target,
                backend.step_width(),
                backend.tmc_config().pwm_freq_hz(),
            )
            .max(1);
            if backend
                .write_register(PID_VELOCITY_TARGET, target_reg)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            data.prev_encoder_for_velocity = backend.read_encoder_count().await;

            let next_sample_us = backend
                .elapsed_us()
                .wrapping_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US);
            *state = VelocityValidateState::ForwardCapture { next_sample_us };
            StepResult::Continue
        }

        VelocityValidateState::ForwardCapture { next_sample_us } => {
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let encoder_raw = backend.read_encoder_count().await;
            let velocity = (encoder_raw as i32)
                .wrapping_sub(data.prev_encoder_for_velocity as i32)
                .unsigned_abs();
            data.prev_encoder_for_velocity = encoder_raw;
            buf.push(velocity);

            if data.torque_limit > 0 {
                let sub = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
                let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                if let Ok(pidout) = backend.read_register(sub.data).await {
                    let commanded_torque = (pidout & 0xFFFF) as i16;
                    let abs_commanded = commanded_torque.unsigned_abs() as u32;
                    if abs_commanded >= data.torque_limit * 9 / 10 {
                        data.saturation_count += 1;
                    }
                }
            }

            if buf.is_full() {
                let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;

                let samples_slice = buf.samples();
                let head_len = samples_slice.len().min(16);
                let peak = samples_slice.iter().copied().max().unwrap_or(0);
                let metrics = crate::autotune::extract_velocity_metrics(
                    buf,
                    data.step_target,
                    0,
                    VELOCITY_VALIDATE_CHATTER_THRESHOLD,
                    VELOCITY_VALIDATE_CHATTER_WINDOW,
                );
                data.forward_metrics = Some(metrics);

                defmt::info!(
                    "vel fwd: target={} peak={} rise={}ms overshoot={} settle={}ms sat={} samples_head={:?}",
                    data.step_target,
                    peak,
                    metrics.rise_time_ms,
                    metrics.overshoot_pct,
                    metrics.settling_time_ms,
                    data.saturation_count,
                    &samples_slice[..head_len],
                );

                let deadline = backend
                    .elapsed_us()
                    .wrapping_add(VELOCITY_VALIDATE_SETTLE_US);
                *state = VelocityValidateState::ForwardSettle {
                    deadline_us: deadline,
                };
            } else {
                *next_sample_us = now.wrapping_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        VelocityValidateState::ForwardSettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
                *state = VelocityValidateState::ReverseStep;
            }
            StepResult::Continue
        }

        VelocityValidateState::ReverseStep => {
            let target_reg = counts_per_ms_to_tmc_velocity(
                data.step_target,
                backend.step_width(),
                backend.tmc_config().pwm_freq_hz(),
            )
            .max(1)
            .min(i32::MAX as u32) as i32;
            let neg_target = -target_reg;
            if backend
                .write_register(PID_VELOCITY_TARGET, neg_target as u32)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            data.prev_encoder_for_velocity = backend.read_encoder_count().await;

            let next_sample_us = backend
                .elapsed_us()
                .wrapping_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US);
            *state = VelocityValidateState::ReverseCapture { next_sample_us };
            StepResult::Continue
        }

        VelocityValidateState::ReverseCapture { next_sample_us } => {
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let encoder_raw = backend.read_encoder_count().await;
            let velocity = (encoder_raw as i32)
                .wrapping_sub(data.prev_encoder_for_velocity as i32)
                .unsigned_abs();
            data.prev_encoder_for_velocity = encoder_raw;
            buf.push(velocity);

            if data.torque_limit > 0 {
                let sub = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
                let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                if let Ok(pidout) = backend.read_register(sub.data).await {
                    let commanded_torque = (pidout & 0xFFFF) as i16;
                    let abs_commanded = commanded_torque.unsigned_abs() as u32;
                    if abs_commanded >= data.torque_limit * 9 / 10 {
                        data.saturation_count += 1;
                    }
                }
            }

            if buf.is_full() {
                let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;

                let samples_slice = buf.samples();
                let head_len = samples_slice.len().min(16);
                let peak = samples_slice.iter().copied().max().unwrap_or(0);
                let metrics = crate::autotune::extract_velocity_metrics(
                    buf,
                    data.step_target,
                    0,
                    VELOCITY_VALIDATE_CHATTER_THRESHOLD,
                    VELOCITY_VALIDATE_CHATTER_WINDOW,
                );
                data.reverse_metrics = Some(metrics);

                defmt::info!(
                    "vel rev: target={} peak={} rise={}ms overshoot={} settle={}ms sat={} samples_head={:?}",
                    data.step_target,
                    peak,
                    metrics.rise_time_ms,
                    metrics.overshoot_pct,
                    metrics.settling_time_ms,
                    data.saturation_count,
                    &samples_slice[..head_len],
                );

                let deadline = backend
                    .elapsed_us()
                    .wrapping_add(VELOCITY_VALIDATE_SETTLE_US);
                *state = VelocityValidateState::ReverseSettle {
                    deadline_us: deadline,
                };
            } else {
                *next_sample_us = now.wrapping_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        VelocityValidateState::ReverseSettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                data.prev_encoder_for_velocity = backend.read_encoder_count().await;
                buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
                let next_sample_us = backend
                    .elapsed_us()
                    .wrapping_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US);
                *state = VelocityValidateState::ChatterCheck {
                    next_sample_us,
                    remaining: VELOCITY_VALIDATE_CHATTER_WINDOW,
                };
            }
            StepResult::Continue
        }

        VelocityValidateState::ChatterCheck {
            next_sample_us,
            remaining,
        } => {
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let count = backend.read_encoder_count().await;
            let velocity = (count as i32).wrapping_sub(data.prev_encoder_for_velocity as i32);
            data.prev_encoder_for_velocity = count;
            let abs_velocity = velocity.unsigned_abs();
            if abs_velocity > data.max_chatter {
                data.max_chatter = abs_velocity;
            }

            *remaining -= 1;
            if *remaining == 0 {
                *state = VelocityValidateState::Classify;
            } else {
                *next_sample_us = now.wrapping_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        VelocityValidateState::Classify => {
            let fwd = data.forward_metrics.unwrap_or(VelocityValidation {
                rise_time_ms: 0,
                overshoot_pct: 0,
                peak_velocity: 0,
                settling_time_ms: 0,
                zero_speed_chatter: false,
                saturation_samples: 0,
                symmetry_pct: 0,
            });
            let rev = data.reverse_metrics.unwrap_or(VelocityValidation {
                rise_time_ms: 0,
                overshoot_pct: 0,
                peak_velocity: 0,
                settling_time_ms: 0,
                zero_speed_chatter: false,
                saturation_samples: 0,
                symmetry_pct: 0,
            });

            let symmetry =
                crate::autotune::compute_symmetry_pct(fwd.rise_time_ms, rev.rise_time_ms);

            let merged = VelocityValidation {
                rise_time_ms: fwd.rise_time_ms.max(rev.rise_time_ms),
                overshoot_pct: fwd.overshoot_pct.max(rev.overshoot_pct),
                peak_velocity: fwd.peak_velocity.max(rev.peak_velocity),
                settling_time_ms: fwd.settling_time_ms.max(rev.settling_time_ms),
                zero_speed_chatter: data.max_chatter > VELOCITY_VALIDATE_CHATTER_THRESHOLD,
                saturation_samples: data.saturation_count,
                symmetry_pct: symmetry,
            };

            let validation_tau_ms = if data.strategy == VelocityTuneStrategy::ResponseRamp {
                response::VELOCITY_RESPONSE_VALIDATE_TAU_MS
            } else {
                data.model.tau_mech_ms.max(1)
            };
            let total_samples = crate::autotune::TRANSIENT_LEN as u16 * 2;
            let result = crate::autotune::classify_velocity_validation(
                &merged,
                validation_tau_ms,
                total_samples,
            );

            defmt::info!(
                "vel classify: result={}, chatter={}, saturation={}, symmetry={}",
                match result {
                    ValidationResult::Pass => "pass",
                    ValidationResult::Marginal => "marginal",
                    ValidationResult::Fail => "fail",
                },
                data.max_chatter,
                data.saturation_count,
                symmetry
            );

            if data.strategy == VelocityTuneStrategy::ResponseRamp {
                match result {
                    ValidationResult::Pass => {
                        data.accepted_gains = data.gains;
                        data.has_accepted_gains = true;

                        if let Some(next_p) = response_next_velocity_p(data.gains.velocity_p) {
                            data.attempt = data.attempt.saturating_add(1);
                            data.gains.velocity_p = next_p;
                            data.gains.velocity_i = 0;
                            defmt::info!(
                                "vel response: clean response, trying P={}, I=0",
                                data.gains.velocity_p
                            );
                            *state = VelocityValidateState::WriteGains;
                        } else {
                            defmt::info!(
                                "vel response: cap reached, accepted P={}, I={}",
                                data.accepted_gains.velocity_p,
                                data.accepted_gains.velocity_i
                            );
                            let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                            let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                            let deadline = backend
                                .elapsed_us()
                                .wrapping_add(VELOCITY_VALIDATE_SETTLE_US);
                            *state = VelocityValidateState::SafeState {
                                deadline_us: deadline,
                            };
                        }
                    }
                    ValidationResult::Marginal | ValidationResult::Fail => {
                        let failed_p = data.gains.velocity_p;
                        if !data.has_accepted_gains
                            && response_result_is_gain_limited(
                                &merged,
                                &fwd,
                                &rev,
                                data.step_target,
                                total_samples,
                            )
                            && let Some(next_p) = response_next_velocity_p(data.gains.velocity_p)
                        {
                            data.attempt = data.attempt.saturating_add(1);
                            data.gains.velocity_p = next_p;
                            data.gains.velocity_i = 0;
                            defmt::info!(
                                "vel response: {} at P={} without passing gain, trying P={}",
                                match result {
                                    ValidationResult::Fail => "fail",
                                    _ => "marginal",
                                },
                                failed_p,
                                data.gains.velocity_p
                            );
                            *state = VelocityValidateState::WriteGains;
                            return StepResult::Continue;
                        }
                        if !data.has_accepted_gains {
                            defmt::warn!(
                                "vel response: {} at P={} with no passing response gain",
                                match result {
                                    ValidationResult::Fail => "fail",
                                    _ => "marginal",
                                },
                                failed_p
                            );
                            let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                            let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                            return fail_outer(phase, CommissionError::VelocityValidationFailed);
                        }

                        data.gains = data.accepted_gains;
                        defmt::warn!(
                            "vel response: {} at P={}, restoring passing P={}",
                            match result {
                                ValidationResult::Fail => "fail",
                                _ => "marginal",
                            },
                            failed_p,
                            data.accepted_gains.velocity_p
                        );
                        let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let deadline = backend
                            .elapsed_us()
                            .wrapping_add(VELOCITY_VALIDATE_SETTLE_US);
                        *state = VelocityValidateState::SafeState {
                            deadline_us: deadline,
                        };
                    }
                }
                return StepResult::Continue;
            }

            match result {
                ValidationResult::Pass => {
                    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    let deadline = backend
                        .elapsed_us()
                        .wrapping_add(VELOCITY_VALIDATE_SETTLE_US);
                    *state = VelocityValidateState::SafeState {
                        deadline_us: deadline,
                    };
                }
                ValidationResult::Marginal if data.attempt == 0 => {
                    defmt::info!("vel validate: marginal on attempt 0, retrying");
                    data.attempt = 1;

                    let config = backend.tmc_config();
                    let step_width = backend.step_width();
                    let mut retry_model = data.model;
                    let new_theta = data.model.theta_mech_us * VELOCITY_RETRY_LAMBDA_MUL_NUM
                        / VELOCITY_RETRY_LAMBDA_MUL_DEN;
                    retry_model.theta_mech_us = new_theta;

                    match crate::autotune::simc_velocity_gains(
                        &retry_model,
                        profile,
                        mode,
                        inner_lambda_us,
                        theta_e_us,
                        data.peak_velocity,
                        step_width,
                        config,
                    ) {
                        Some(mut new_gains) => {
                            data.diagnostics.synth_velocity_p = new_gains.velocity_p;
                            data.diagnostics.synth_velocity_i = new_gains.velocity_i;
                            if new_gains.velocity_p > OUTER_MAX_VELOCITY_P
                                || new_gains.velocity_i > OUTER_MAX_VELOCITY_I
                            {
                                data.diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
                                defmt::warn!(
                                    "vel retry: synthesized P={}, I={} outside envelope, using response tuning",
                                    new_gains.velocity_p,
                                    new_gains.velocity_i
                                );
                                new_gains = response::response_velocity_gains();
                                data.strategy = VelocityTuneStrategy::ResponseRamp;
                                data.step_target = response::VELOCITY_RESPONSE_STEP_TARGET;
                            }
                            defmt::info!(
                                "vel retry: P={}, I={}, lambda={}ms",
                                new_gains.velocity_p,
                                new_gains.velocity_i,
                                new_gains.lambda_vel_ms
                            );
                            data.gains = new_gains;
                            if data.strategy == VelocityTuneStrategy::Model {
                                data.step_target = (new_gains.velocity_limit
                                    * VELOCITY_VALIDATE_STEP_PERCENT
                                    / 100)
                                    .max(1);
                            }
                            *state = VelocityValidateState::WriteGains;
                        }
                        None => {
                            return fail_outer(phase, CommissionError::VelocityValidationFailed);
                        }
                    }
                }
                ValidationResult::Marginal | ValidationResult::Fail => {
                    defmt::warn!(
                        "vel validate: {} on attempt {}, trying response ramp",
                        match result {
                            ValidationResult::Fail => "fail",
                            _ => "marginal",
                        },
                        data.attempt
                    );
                    data.diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
                    data.gains = response::response_velocity_gains();
                    data.accepted_gains = data.gains;
                    data.has_accepted_gains = false;
                    data.strategy = VelocityTuneStrategy::ResponseRamp;
                    data.attempt = 0;
                    data.step_target = response::VELOCITY_RESPONSE_STEP_TARGET;
                    data.forward_metrics = None;
                    data.reverse_metrics = None;
                    data.max_chatter = 0;
                    data.saturation_count = 0;
                    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    *state = VelocityValidateState::WriteGains;
                }
            }
            StepResult::Continue
        }

        VelocityValidateState::SafeState { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                *state = VelocityValidateState::Finalize;
            }
            StepResult::Continue
        }

        VelocityValidateState::Finalize => {
            let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
            let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

            let carried_safety = *safety;
            let mut accepted_gains = data.gains;
            // Keep the bounded validation velocity limit in hardware for
            // Phase 11. Position tuning intentionally uses small, local
            // step responses; restoring the runtime limit here lets the
            // first position step violate the outer safety envelope.
            //
            // The final reported gains still carry the operational limit
            // so SAVE_CONFIG does not persist the commissioning test limit.
            accepted_gains.velocity_limit = data.operational_velocity_limit.max(1);
            let velocity_chatter = data.max_chatter > VELOCITY_VALIDATE_CHATTER_THRESHOLD;

            defmt::info!(
                "vel validate: accepted P={}, I={}, limit={}",
                accepted_gains.velocity_p,
                accepted_gains.velocity_i,
                accepted_gains.velocity_limit
            );

            // Position capture is measured in ABN counts, but
            // PID_POSITION_TARGET is written in TMC electrical-position
            // units. Convert the small motor-local test move through the
            // configured STEP_WIDTH so the command and measurement agree.
            let step_width = backend.step_width().max(1);
            let step_target_counts = POS_TUNE_STEP_COUNTS;
            let step_target_tmc = step_width.saturating_mul(step_target_counts);

            let j_eff = data.model.j_eff;
            let b_eff = data.model.b_eff;

            let pos_data = PositionTuneData {
                current_p: POS_TUNE_INITIAL_P,
                stable_p: 0,
                step_target_counts,
                step_target_tmc,
                step_start_counts: 0,
                step_start_tmc: 0,
                oscillation_at_minimum: false,
                velocity_gains: accepted_gains,
                velocity_chatter,
                current_ringing,
                current_bandwidth_hz,
                j_eff,
                b_eff,
                diagnostics: data.diagnostics,
            };

            *phase = OuterPhase::PositionTune {
                state: PositionTuneState::Setup,
                data: pos_data,
                safety: carried_safety,
            };
            StepResult::PhaseChanged(PhaseId::PositionTune)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{counts_per_ms_to_tmc_velocity, step_velocity_validate};
    use crate::autotune::{
        CommissionError, CommissionProfile, MechanicalModel, OperatingMode, TransientBuffer,
        VelocityGains, VelocityValidation,
    };
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::outer::{
        OuterDiagnostics, OuterPhase, OuterSafetyLimits, VelocityTuneStrategy,
        VelocityValidateData, VelocityValidateState, response,
    };
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::registers::{PID_VELOCITY_LIMIT, PID_VELOCITY_TARGET};

    fn zero_diagnostics() -> OuterDiagnostics {
        OuterDiagnostics {
            warning_code: 0,
            mech_torque_step: 0,
            mech_fwd_accel: 0,
            mech_rev_accel: 0,
            mech_fwd_peak_velocity: 0,
            mech_rev_peak_velocity: 0,
            mech_fwd_travel: 0,
            mech_rev_travel: 0,
            synth_velocity_p: 0,
            synth_velocity_i: 0,
            synth_position_p: 0,
            synth_position_i: 0,
        }
    }

    fn failing_velocity_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 0,
            overshoot_pct: 11_500,
            peak_velocity: 25,
            settling_time_ms: 256,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
        }
    }

    fn low_energy_marginal_velocity_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 0,
            overshoot_pct: 0,
            peak_velocity: 0,
            settling_time_ms: 256,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
        }
    }

    fn stiction_breakaway_metrics(peak_velocity: u32, overshoot_pct: u16) -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 0,
            overshoot_pct,
            settling_time_ms: 256,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
            peak_velocity,
        }
    }

    fn model_failed_velocity_phase() -> OuterPhase {
        let gains = VelocityGains {
            velocity_p: 4608,
            velocity_i: 1,
            velocity_limit: 14,
            effective_bandwidth_hz: 1,
            lambda_vel_ms: 137,
        };

        OuterPhase::VelocityValidate {
            state: VelocityValidateState::Classify,
            data: VelocityValidateData {
                gains,
                accepted_gains: gains,
                has_accepted_gains: false,
                model: MechanicalModel {
                    j_eff: 2_712_665,
                    b_eff: 20_451,
                    tau_mech_ms: 132,
                    theta_mech_us: 12_895,
                    t_f: 0,
                },
                strategy: VelocityTuneStrategy::Model,
                attempt: 0,
                step_target: 2,
                forward_metrics: Some(failing_velocity_metrics()),
                reverse_metrics: Some(failing_velocity_metrics()),
                max_chatter: 24,
                peak_velocity: 18,
                prev_encoder_for_velocity: 0,
                saturation_count: 484,
                torque_limit: 2448,
                operational_velocity_limit: 500_000,
                diagnostics: zero_diagnostics(),
            },
            safety: OuterSafetyLimits::new(0, 2448),
        }
    }

    #[test]
    fn velocity_validation_converts_counts_per_ms_to_tmc_velocity_units() {
        assert_eq!(counts_per_ms_to_tmc_velocity(4, 819, 25_000), 132);
        assert_eq!(counts_per_ms_to_tmc_velocity(16, 819, 25_000), 525);
    }

    #[test]
    fn velocity_validation_conversion_handles_zero_inputs() {
        assert_eq!(counts_per_ms_to_tmc_velocity(0, 819, 25_000), 0);
        assert_eq!(counts_per_ms_to_tmc_velocity(4, 0, 25_000), 0);
        assert_eq!(counts_per_ms_to_tmc_velocity(4, 819, 0), 0);
    }

    #[test]
    fn velocity_validation_writes_converted_tmc_limit_and_target() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: 1152,
                velocity_i: 0,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::WriteGains,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    strategy: VelocityTuneStrategy::Model,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: None,
                    reverse_metrics: None,
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: 0,
                    saturation_count: 0,
                    torque_limit: 0,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.step_width_value = 819;

            let _ = step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert_eq!(
                *backend
                    .registers
                    .get(&PID_VELOCITY_LIMIT.addr())
                    .expect("velocity limit should be written"),
                525
            );

            let _ = step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert_eq!(
                *backend
                    .registers
                    .get(&PID_VELOCITY_TARGET.addr())
                    .expect("velocity target should be written"),
                132
            );
        });
    }

    #[test]
    fn model_velocity_failure_falls_back_to_response_ramp() {
        pollster::block_on(async {
            let mut phase = model_failed_velocity_phase();
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert_eq!(data.gains.velocity_p, response::VELOCITY_RESPONSE_INITIAL_P);
                    assert_eq!(data.gains.velocity_i, 0);
                    assert_eq!(data.step_target, response::VELOCITY_RESPONSE_STEP_TARGET);
                }
                other => panic!("expected response-ramp retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_failure_after_model_fallback_terminates() {
        pollster::block_on(async {
            let mut phase = model_failed_velocity_phase();
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            match &mut phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    *state = VelocityValidateState::Classify;
                    data.forward_metrics = Some(failing_velocity_metrics());
                    data.reverse_metrics = Some(failing_velocity_metrics());
                    data.max_chatter = 24;
                    data.saturation_count = 484;
                }
                other => panic!("expected response-ramp validation, got {:?}", other),
            }

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::VelocityValidationFailed)
            ));
            assert!(matches!(
                phase,
                OuterPhase::Failed(CommissionError::VelocityValidationFailed)
            ));
        });
    }

    #[test]
    fn response_ramp_marginal_without_accepted_gain_keeps_searching() {
        pollster::block_on(async {
            let gains = response::response_velocity_gains();
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(low_energy_marginal_velocity_metrics()),
                    reverse_metrics: Some(low_energy_marginal_velocity_metrics()),
                    max_chatter: 0,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: 0,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    assert!(
                        data.gains.velocity_p > gains.velocity_p,
                        "low-energy response should advance P, got {} from {}",
                        data.gains.velocity_p,
                        gains.velocity_p
                    );
                }
                other => panic!("expected response-ramp search to continue, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_stiction_breakaway_without_accepted_gain_keeps_searching() {
        pollster::block_on(async {
            let gains = response::response_velocity_gains();
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(stiction_breakaway_metrics(8, 250)),
                    reverse_metrics: Some(stiction_breakaway_metrics(1, 0)),
                    max_chatter: 0,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: 0,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    assert!(
                        data.gains.velocity_p > gains.velocity_p,
                        "stiction breakaway should advance P, got {} from {}",
                        data.gains.velocity_p,
                        gains.velocity_p
                    );
                }
                other => panic!("expected response-ramp search to continue, got {:?}", other),
            }
        });
    }
}
