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

/// Velocity mode with `MODE_PID_TYPE` set for the TMC4671 advanced PI structure.
const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;

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
            let gains = match crate::autotune::simc_velocity_gains(
                &model,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                peak_velocity,
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
            let vel_reg = ((data.gains.velocity_p as u32) << 16) | (data.gains.velocity_i as u32);
            if backend
                .write_register(PID_VELOCITY_P_VELOCITY_I, vel_reg)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            if backend
                .write_register(PID_VELOCITY_LIMIT, data.gains.velocity_limit)
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

            buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
            data.forward_metrics = None;
            data.reverse_metrics = None;
            data.max_chatter = 0;
            data.saturation_count = 0;

            *state = VelocityValidateState::ForwardStep;
            StepResult::Continue
        }

        VelocityValidateState::ForwardStep => {
            if backend
                .write_register(PID_VELOCITY_TARGET, data.step_target)
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

                let metrics = crate::autotune::extract_velocity_metrics(
                    buf,
                    data.step_target,
                    0,
                    VELOCITY_VALIDATE_CHATTER_THRESHOLD,
                    VELOCITY_VALIDATE_CHATTER_WINDOW,
                );
                data.forward_metrics = Some(metrics);

                defmt::info!(
                    "vel fwd: rise={}ms, overshoot={}, settle={}ms",
                    metrics.rise_time_ms,
                    metrics.overshoot_pct,
                    metrics.settling_time_ms
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
            let neg_target = -(data.step_target as i32);
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

                let metrics = crate::autotune::extract_velocity_metrics(
                    buf,
                    data.step_target,
                    0,
                    VELOCITY_VALIDATE_CHATTER_THRESHOLD,
                    VELOCITY_VALIDATE_CHATTER_WINDOW,
                );
                data.reverse_metrics = Some(metrics);

                defmt::info!(
                    "vel rev: rise={}ms, overshoot={}, settle={}ms",
                    metrics.rise_time_ms,
                    metrics.overshoot_pct,
                    metrics.settling_time_ms
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
                settling_time_ms: 0,
                zero_speed_chatter: false,
                saturation_samples: 0,
                symmetry_pct: 0,
            });
            let rev = data.reverse_metrics.unwrap_or(VelocityValidation {
                rise_time_ms: 0,
                overshoot_pct: 0,
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
            let result = crate::autotune::classify_velocity_validation(
                &merged,
                validation_tau_ms,
                crate::autotune::TRANSIENT_LEN as u16 * 2,
            );

            defmt::info!(
                "vel classify: result={}, chatter={}, symmetry={}",
                match result {
                    ValidationResult::Pass => "pass",
                    ValidationResult::Marginal => "marginal",
                    ValidationResult::Fail => "fail",
                },
                data.max_chatter,
                symmetry
            );

            if data.strategy == VelocityTuneStrategy::ResponseRamp {
                match result {
                    ValidationResult::Pass => {
                        data.accepted_gains = data.gains;
                        data.has_accepted_gains = true;

                        let next_p = data.gains.velocity_p as u32 * VELOCITY_RESPONSE_P_MUL_NUM
                            / VELOCITY_RESPONSE_P_MUL_DEN;
                        if next_p > OUTER_MAX_VELOCITY_P as u32
                            || next_p <= data.gains.velocity_p as u32
                        {
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
                        } else {
                            data.attempt = data.attempt.saturating_add(1);
                            data.gains.velocity_p = next_p as u16;
                            data.gains.velocity_i = 0;
                            defmt::info!(
                                "vel response: clean response, trying P={}, I=0",
                                data.gains.velocity_p
                            );
                            *state = VelocityValidateState::WriteGains;
                        }
                    }
                    ValidationResult::Marginal | ValidationResult::Fail => {
                        let failed_p = data.gains.velocity_p;
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
                        "vel validate: {} on attempt {}",
                        match result {
                            ValidationResult::Fail => "fail",
                            _ => "marginal",
                        },
                        data.attempt
                    );
                    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return fail_outer(phase, CommissionError::VelocityValidationFailed);
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
