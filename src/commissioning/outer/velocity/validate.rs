//! Phase 10 velocity-loop bidirectional step-response validation.

use super::super::{
    OUTER_EVIDENCE_LOW_CONFIDENCE_MECH, OUTER_EVIDENCE_MOTION_BUDGET_CLAMPED,
    OUTER_EVIDENCE_TAU_DEFAULTED, OUTER_EVIDENCE_UNDER_DRIVEN_PROBE, OUTER_MAX_VELOCITY_I,
    OUTER_MAX_VELOCITY_P, OUTER_WARNING_UNSAFE_SYNTH_GAINS, OuterPhase, POS_TUNE_INITIAL_P,
    PositionTuneData, PositionTuneState, TraceSummarySeqs, VELOCITY_SEARCH_STOP_CLEAN,
    VELOCITY_SEARCH_STOP_LOW_CONFIDENCE, VELOCITY_SEARCH_STOP_UNDER_DRIVEN_BUDGET,
    VELOCITY_SEARCH_STOP_UNSAFE_BACKOFF, VelocityTuneStrategy, VelocityValidateData,
    VelocityValidateState, trace_pack_i16_pair, trace_pack_u16_pair, trace_summary_event,
};
use super::response_ramp;
use crate::autotune::{
    CommissionError, OperatingMode, Permille, PhaseId, TransientBuffer, ValidationResult,
    VelocityValidation,
};
use crate::commissioning::encoder_domain::{EncoderDomain, EncoderMathError};
use crate::commissioning::outer::{fail_outer, response};
use crate::commissioning::types::{CommissioningBackend, StepResult};
use crate::trace::events::{
    EVENT_COMMISSION_VELOCITY_VALIDATE_GAIN, EVENT_COMMISSION_VELOCITY_VALIDATE_LIMITS,
    EVENT_COMMISSION_VELOCITY_VALIDATE_METRICS, EVENT_COMMISSION_VELOCITY_VALIDATE_RESULT,
    TraceEventLevel,
};
use foci_tmc4671::pid::{PidGain, PidGainPair};

/// Velocity step as a fraction of velocity_limit (15% = 15/100).
/// Spec says 10-20%.
pub(super) const VELOCITY_VALIDATE_STEP_PERCENT: u32 = 15;

/// Sampling interval for velocity capture (microseconds).
/// 1 kHz matches mechanical ID and the SIMC validation cadence.
pub(crate) const VELOCITY_VALIDATE_SAMPLE_PERIOD_US: u64 = 1000;

/// Settling time after a velocity step (microseconds).
pub(crate) const VELOCITY_VALIDATE_SETTLE_US: u64 = 200_000;

/// Number of zero-speed samples used to detect chatter.
pub(crate) const VELOCITY_VALIDATE_CHATTER_WINDOW: u16 = 200;

/// Encoder-counts/ms threshold above which a zero-speed sample counts as chatter.
const VELOCITY_VALIDATE_CHATTER_THRESHOLD: u32 = 3;

/// Maximum register operations performed by one armed velocity-validation poll.
#[allow(dead_code)]
pub(super) const VELOCITY_VALIDATE_MAX_REGISTER_OPS_PER_ARM: usize = 4;

/// Lambda multiplier (numerator/denominator) applied on a marginal retry.
const VELOCITY_RETRY_LAMBDA_MUL_NUM: u32 = 3;
const VELOCITY_RETRY_LAMBDA_MUL_DEN: u32 = 2;

/// Velocity mode with `MODE_PID_TYPE` set for the TMC4671 advanced PI structure.
const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;

async fn stop_velocity_motion<B: CommissioningBackend>(
    backend: &mut B,
    safety: &mut super::super::OuterSafetyLimits,
) -> bool {
    use foci_tmc4671::registers::MODE_RAMP_MODE_MOTION;

    let target_ok = command_velocity_target(backend, safety, 0).await.is_ok();
    let mode_ok = backend
        .write_register(MODE_RAMP_MODE_MOTION, 0)
        .await
        .is_ok();
    target_ok && mode_ok
}

async fn command_velocity_target<B: CommissioningBackend>(
    backend: &mut B,
    safety: &mut super::super::OuterSafetyLimits,
    target: u32,
) -> Result<(), CommissionError> {
    if target != 0 && safety.primitive_start_us.is_some() {
        return Err(CommissionError::InvalidSchedule);
    }
    backend
        .write_register(foci_tmc4671::registers::PID_VELOCITY_TARGET, target)
        .await
        .map_err(|_| CommissionError::Spi)?;
    if target == 0 {
        safety.end_primitive();
        Ok(())
    } else {
        safety.begin_primitive(backend.elapsed_us())
    }
}

async fn clear_velocity_integrator<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), CommissionError> {
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        foci_tmc4671::registers::PID_ERROR_PID_VELOCITY_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)
}

fn encoder_rate_counts_per_ms_to_tmc_velocity_signed(
    encoder_counts_per_ms: i64,
    encoder_counts_per_rev: u32,
    f_pwm_hz: u32,
) -> i32 {
    if encoder_counts_per_ms == 0 || encoder_counts_per_rev == 0 || f_pwm_hz == 0 {
        return 0;
    }

    // TMC velocity registers use position-units per PWM period. Validation
    // metrics use encoder counts/ms. Convert through the exact mechanical
    // position scale, independent of the planner's physical STEP width.
    let numerator = i128::from(encoder_counts_per_ms) * 65_536 * 1_000;
    let denominator = i128::from(encoder_counts_per_rev) * i128::from(f_pwm_hz);
    let rounding = denominator / 2;
    let rounded = if numerator < 0 {
        (numerator - rounding) / denominator
    } else {
        (numerator + rounding) / denominator
    };
    rounded.clamp(i128::from(i32::MIN), i128::from(i32::MAX)) as i32
}

fn encoder_rate_magnitude_counts_per_ms_to_tmc_velocity(
    encoder_counts_per_ms: u64,
    encoder_counts_per_rev: u32,
    f_pwm_hz: u32,
) -> u32 {
    if encoder_counts_per_ms == 0 || encoder_counts_per_rev == 0 || f_pwm_hz == 0 {
        return 0;
    }

    let numerator = u128::from(encoder_counts_per_ms) * 65_536 * 1_000;
    let denominator = u128::from(encoder_counts_per_rev) * u128::from(f_pwm_hz);
    let rounded = (numerator + denominator / 2) / denominator;
    rounded.min(u128::from(u32::MAX)) as u32
}

fn velocity_settle_deadline(now_us: u64) -> Result<u64, CommissionError> {
    now_us
        .checked_add(VELOCITY_VALIDATE_SETTLE_US)
        .ok_or(CommissionError::InvalidSchedule)
}

fn encoder_counts_to_tmc_position_units(encoder_counts: i32, encoder_counts_per_rev: u32) -> i32 {
    if encoder_counts_per_rev == 0 {
        return 0;
    }

    let numerator = i128::from(encoder_counts) * 65_536;
    let denominator = i128::from(encoder_counts_per_rev);
    let rounding = denominator / 2;
    let rounded = if numerator < 0 {
        (numerator - rounding) / denominator
    } else {
        (numerator + rounding) / denominator
    };
    rounded.clamp(i128::from(i32::MIN), i128::from(i32::MAX)) as i32
}

fn encoder_delta_counts_per_ms(
    current: u32,
    previous: u32,
    counts_per_rev: u32,
    elapsed_us: u64,
) -> Result<u32, EncoderMathError> {
    encoder_delta_and_rate_counts_per_ms(current, previous, counts_per_rev, elapsed_us)
        .map(|(_, rate)| rate)
}

fn encoder_delta_and_rate_counts_per_ms(
    current: u32,
    previous: u32,
    counts_per_rev: u32,
    elapsed_us: u64,
) -> Result<(i32, u32), EncoderMathError> {
    let domain = EncoderDomain::new(counts_per_rev)?;
    let delta = domain.delta(domain.count(previous), domain.count(current))?;
    let delta_counts = delta.absolute_counts();
    if elapsed_us == 0 {
        return Ok((delta.signed_counts(), 0));
    }

    if delta_counts == 0 {
        return Ok((delta.signed_counts(), 0));
    }

    let numerator = delta_counts as u128 * 1000;
    let rate =
        ((numerator + elapsed_us as u128 / 2) / elapsed_us as u128).min(u32::MAX as u128) as u32;
    Ok((delta.signed_counts(), rate))
}

#[cfg(feature = "trace")]
fn record_velocity_trace_observation<B: CommissioningBackend>(
    backend: &mut B,
    data: &VelocityValidateData,
    delta_counts: i32,
    elapsed_us: u64,
    reverse: bool,
    saturated: bool,
) {
    let Ok(elapsed_us) = u32::try_from(elapsed_us) else {
        return;
    };
    let target_magnitude = i32::try_from(data.step_target).unwrap_or(i32::MAX);
    let stroke_base = u16::from(data.attempt).saturating_mul(2);
    backend.record_velocity_trace_observation(crate::trace::stream::VelocityTraceObservation {
        target_counts_per_ms: if reverse {
            -target_magnitude
        } else {
            target_magnitude
        },
        delta_counts,
        elapsed_us,
        stroke_index: stroke_base + u16::from(reverse),
        direction: if reverse { -1 } else { 1 },
        flags: if saturated {
            crate::trace::stream::VELOCITY_OBSERVATION_FLAG_SATURATED
        } else {
            0
        },
    });
}

fn record_velocity_validate_summary<B: CommissioningBackend>(
    backend: &mut B,
    trace_summary_seqs: &mut TraceSummarySeqs,
    data: &VelocityValidateData,
    fwd: &VelocityValidation,
    rev: &VelocityValidation,
    result_status: u8,
    level: TraceEventLevel,
) {
    let summary_seq = trace_summary_seqs.next_velocity_validate();
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_VELOCITY_VALIDATE_RESULT,
        summary_seq,
        i32::from(result_status),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_VELOCITY_VALIDATE_GAIN,
        summary_seq,
        trace_pack_u16_pair(
            u32::from(data.gains.velocity_p),
            u32::from(data.gains.velocity_i),
        ),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_VELOCITY_VALIDATE_METRICS,
        summary_seq,
        trace_pack_i16_pair(fwd.peak_velocity as i32, rev.peak_velocity as i32),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_VELOCITY_VALIDATE_LIMITS,
        summary_seq,
        trace_pack_u16_pair(u32::from(data.saturation_count), data.max_chatter),
    );
}

/// Phase 10: bidirectional velocity step-response validation.
#[allow(clippy::too_many_arguments)]
#[allow(dead_code)]
pub(crate) async fn step_velocity_validate<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    backend: &mut B,
) -> StepResult {
    let mut trace_summary_seqs = TraceSummarySeqs::new();
    step_velocity_validate_with_trace(
        phase,
        buf,
        mode,
        inner_lambda_us,
        theta_e_us,
        current_ringing,
        current_bandwidth_hz,
        &mut trace_summary_seqs,
        backend,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_velocity_validate_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    trace_summary_seqs: &mut TraceSummarySeqs,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (state, data, safety) = match phase {
        OuterPhase::VelocityValidate {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::ReadyNow,
    };

    match state {
        VelocityValidateState::WriteGains => {
            let encoder_counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
            let f_pwm_hz = backend.tmc_config().pwm_freq_hz();
            let velocity_limit_reg = encoder_rate_magnitude_counts_per_ms_to_tmc_velocity(
                u64::from(data.gains.velocity_limit),
                encoder_counts_per_rev,
                f_pwm_hz,
            )
            .max(1);
            let step_target_reg = encoder_rate_counts_per_ms_to_tmc_velocity_signed(
                i64::from(data.step_target),
                encoder_counts_per_rev,
                f_pwm_hz,
            )
            .max(1) as u32;

            let vel_reg =
                PidGainPair::new(data.gains.velocity_p, data.gains.velocity_i).register_value();
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

            if let Err(error) = command_velocity_target(backend, safety, 0).await {
                return fail_outer(phase, error);
            }

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

            *state = VelocityValidateState::ForwardClearIntegrator;
            StepResult::ReadyNow
        }

        VelocityValidateState::ForwardClearIntegrator => {
            if let Err(error) = clear_velocity_integrator(backend).await {
                return fail_outer(phase, error);
            }
            *state = VelocityValidateState::ForwardStep;
            StepResult::ReadyNow
        }

        VelocityValidateState::ForwardStep => {
            let target_reg = encoder_rate_counts_per_ms_to_tmc_velocity_signed(
                i64::from(data.step_target),
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                backend.tmc_config().pwm_freq_hz(),
            )
            .max(1) as u32;
            if let Err(error) = command_velocity_target(backend, safety, target_reg).await {
                return fail_outer(phase, error);
            }

            data.prev_encoder_for_velocity =
                match backend.try_read_mechanical_encoder_position_counts().await {
                    Ok(value) => Some(
                        safety
                            .encoder_domain
                            .expect("outer encoder domain configured before velocity validation")
                            .count(value),
                    ),
                    Err(_) => return fail_outer(phase, CommissionError::Spi),
                };

            let now = backend.elapsed_us();
            let Some(next_sample_us) = now.checked_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US) else {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            };
            *state = VelocityValidateState::ForwardCapture {
                next_sample_us,
                last_sample_us: now,
                ambiguity_retried: false,
                saturation_start: data.saturation_count,
            };
            StepResult::At(next_sample_us)
        }

        VelocityValidateState::ForwardCapture {
            next_sample_us,
            last_sample_us,
            ambiguity_retried,
            saturation_start,
        } => {
            let now = backend.elapsed_us();
            if now < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }

            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let prev_encoder = data
                .prev_encoder_for_velocity
                .expect("forward velocity baseline configured");
            let counts_per_rev = safety
                .encoder_domain
                .map(EncoderDomain::counts_per_rev)
                .unwrap_or(0);
            let Some(elapsed_us) = now.checked_sub(*last_sample_us) else {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            };
            let (_delta_counts, velocity) = match encoder_delta_and_rate_counts_per_ms(
                encoder_raw,
                prev_encoder.raw_counts(),
                counts_per_rev,
                elapsed_us,
            ) {
                Ok(velocity) => velocity,
                Err(EncoderMathError::InvalidCountsPerRev) => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    return fail_outer(phase, CommissionError::EncoderFault);
                }
                Err(EncoderMathError::AmbiguousHalfTurn) if !*ambiguity_retried => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    if let Err(error) = clear_velocity_integrator(backend).await {
                        return fail_outer(phase, error);
                    }
                    let target = encoder_rate_counts_per_ms_to_tmc_velocity_signed(
                        i64::from(data.step_target),
                        counts_per_rev,
                        backend.tmc_config().pwm_freq_hz(),
                    )
                    .max(1) as u32;
                    if let Err(error) = command_velocity_target(backend, safety, target).await {
                        return fail_outer(phase, error);
                    }
                    buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
                    data.forward_metrics = None;
                    data.saturation_count = *saturation_start;
                    data.prev_encoder_for_velocity = Some(
                        safety
                            .encoder_domain
                            .expect("validated domain")
                            .count(encoder_raw),
                    );
                    *ambiguity_retried = true;
                    let Some(deadline_us) = now.checked_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US)
                    else {
                        return fail_outer(phase, CommissionError::InvalidSchedule);
                    };
                    *next_sample_us = deadline_us;
                    *last_sample_us = now;
                    return StepResult::At(deadline_us);
                }
                Err(EncoderMathError::AmbiguousHalfTurn) => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    return fail_outer(phase, CommissionError::VelocityValidationFailed);
                }
            };
            data.prev_encoder_for_velocity = Some(
                safety
                    .encoder_domain
                    .expect("validated domain")
                    .count(encoder_raw),
            );
            buf.push(velocity);

            let mut _saturated = false;
            if data.torque_limit > 0 {
                let sub = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
                let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                if let Ok(pidout) = backend.read_register(sub.data).await {
                    let commanded_torque = (pidout & 0xFFFF) as i16;
                    let abs_commanded = commanded_torque.unsigned_abs() as u32;
                    if abs_commanded >= data.torque_limit * 9 / 10 {
                        data.saturation_count += 1;
                        _saturated = true;
                    }
                }
            }

            #[cfg(feature = "trace")]
            record_velocity_trace_observation(
                backend,
                data,
                _delta_counts,
                elapsed_us,
                false,
                _saturated,
            );

            if buf.is_full() {
                if let Err(error) = command_velocity_target(backend, safety, 0).await {
                    return fail_outer(phase, error);
                }

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
                    metrics.overshoot_permille.raw(),
                    metrics.settling_time_ms,
                    data.saturation_count,
                    &samples_slice[..head_len],
                );

                let Some(deadline) = backend
                    .elapsed_us()
                    .checked_add(VELOCITY_VALIDATE_SETTLE_US)
                else {
                    return fail_outer(phase, CommissionError::InvalidSchedule);
                };
                *state = VelocityValidateState::ForwardZeroClearIntegrator {
                    deadline_us: deadline,
                };
                StepResult::ReadyNow
            } else {
                let next_deadline_us = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
                    now,
                ) {
                    Ok(deadline_us) => deadline_us,
                    Err(error) => return fail_outer(phase, error),
                };
                *next_sample_us = next_deadline_us;
                *last_sample_us = now;
                StepResult::At(next_deadline_us)
            }
        }

        VelocityValidateState::ForwardZeroClearIntegrator { deadline_us } => {
            let deadline_us = *deadline_us;
            if let Err(error) = clear_velocity_integrator(backend).await {
                return fail_outer(phase, error);
            }
            *state = VelocityValidateState::ForwardSettle { deadline_us };
            StepResult::At(deadline_us)
        }

        VelocityValidateState::ForwardSettle { deadline_us } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
            *state = VelocityValidateState::ReverseClearIntegrator;
            StepResult::ReadyNow
        }

        VelocityValidateState::ReverseClearIntegrator => {
            if let Err(error) = clear_velocity_integrator(backend).await {
                return fail_outer(phase, error);
            }
            *state = VelocityValidateState::ReverseStep;
            StepResult::ReadyNow
        }

        VelocityValidateState::ReverseStep => {
            let target_reg = encoder_rate_counts_per_ms_to_tmc_velocity_signed(
                -i64::from(data.step_target),
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                backend.tmc_config().pwm_freq_hz(),
            )
            .min(-1);
            if let Err(error) = command_velocity_target(backend, safety, target_reg as u32).await {
                return fail_outer(phase, error);
            }

            data.prev_encoder_for_velocity =
                match backend.try_read_mechanical_encoder_position_counts().await {
                    Ok(value) => Some(
                        safety
                            .encoder_domain
                            .expect("outer encoder domain configured before velocity validation")
                            .count(value),
                    ),
                    Err(_) => return fail_outer(phase, CommissionError::Spi),
                };

            let now = backend.elapsed_us();
            let Some(next_sample_us) = now.checked_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US) else {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            };
            *state = VelocityValidateState::ReverseCapture {
                next_sample_us,
                last_sample_us: now,
                ambiguity_retried: false,
                saturation_start: data.saturation_count,
            };
            StepResult::At(next_sample_us)
        }

        VelocityValidateState::ReverseCapture {
            next_sample_us,
            last_sample_us,
            ambiguity_retried,
            saturation_start,
        } => {
            let now = backend.elapsed_us();
            if now < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }

            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let prev_encoder = data
                .prev_encoder_for_velocity
                .expect("reverse velocity baseline configured");
            let counts_per_rev = safety
                .encoder_domain
                .map(EncoderDomain::counts_per_rev)
                .unwrap_or(0);
            let Some(elapsed_us) = now.checked_sub(*last_sample_us) else {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            };
            let (_delta_counts, velocity) = match encoder_delta_and_rate_counts_per_ms(
                encoder_raw,
                prev_encoder.raw_counts(),
                counts_per_rev,
                elapsed_us,
            ) {
                Ok(velocity) => velocity,
                Err(EncoderMathError::InvalidCountsPerRev) => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    return fail_outer(phase, CommissionError::EncoderFault);
                }
                Err(EncoderMathError::AmbiguousHalfTurn) if !*ambiguity_retried => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    if let Err(error) = clear_velocity_integrator(backend).await {
                        return fail_outer(phase, error);
                    }
                    let target = encoder_rate_counts_per_ms_to_tmc_velocity_signed(
                        -i64::from(data.step_target),
                        counts_per_rev,
                        backend.tmc_config().pwm_freq_hz(),
                    )
                    .min(-1) as u32;
                    if let Err(error) = command_velocity_target(backend, safety, target).await {
                        return fail_outer(phase, error);
                    }
                    buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
                    data.reverse_metrics = None;
                    data.saturation_count = *saturation_start;
                    data.prev_encoder_for_velocity = Some(
                        safety
                            .encoder_domain
                            .expect("validated domain")
                            .count(encoder_raw),
                    );
                    *ambiguity_retried = true;
                    let Some(deadline_us) = now.checked_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US)
                    else {
                        return fail_outer(phase, CommissionError::InvalidSchedule);
                    };
                    *next_sample_us = deadline_us;
                    *last_sample_us = now;
                    return StepResult::At(deadline_us);
                }
                Err(EncoderMathError::AmbiguousHalfTurn) => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    return fail_outer(phase, CommissionError::VelocityValidationFailed);
                }
            };
            data.prev_encoder_for_velocity = Some(
                safety
                    .encoder_domain
                    .expect("validated domain")
                    .count(encoder_raw),
            );
            buf.push(velocity);

            let mut _saturated = false;
            if data.torque_limit > 0 {
                let sub = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
                let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                if let Ok(pidout) = backend.read_register(sub.data).await {
                    let commanded_torque = (pidout & 0xFFFF) as i16;
                    let abs_commanded = commanded_torque.unsigned_abs() as u32;
                    if abs_commanded >= data.torque_limit * 9 / 10 {
                        data.saturation_count += 1;
                        _saturated = true;
                    }
                }
            }

            #[cfg(feature = "trace")]
            record_velocity_trace_observation(
                backend,
                data,
                _delta_counts,
                elapsed_us,
                true,
                _saturated,
            );

            if buf.is_full() {
                if let Err(error) = command_velocity_target(backend, safety, 0).await {
                    return fail_outer(phase, error);
                }

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
                    metrics.overshoot_permille.raw(),
                    metrics.settling_time_ms,
                    data.saturation_count,
                    &samples_slice[..head_len],
                );

                let Some(deadline) = backend
                    .elapsed_us()
                    .checked_add(VELOCITY_VALIDATE_SETTLE_US)
                else {
                    return fail_outer(phase, CommissionError::InvalidSchedule);
                };
                *state = VelocityValidateState::ReverseZeroClearIntegrator {
                    deadline_us: deadline,
                };
                StepResult::ReadyNow
            } else {
                let next_deadline_us = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
                    now,
                ) {
                    Ok(deadline_us) => deadline_us,
                    Err(error) => return fail_outer(phase, error),
                };
                *next_sample_us = next_deadline_us;
                *last_sample_us = now;
                StepResult::At(next_deadline_us)
            }
        }

        VelocityValidateState::ReverseZeroClearIntegrator { deadline_us } => {
            let deadline_us = *deadline_us;
            if let Err(error) = clear_velocity_integrator(backend).await {
                return fail_outer(phase, error);
            }
            *state = VelocityValidateState::ReverseSettle { deadline_us };
            StepResult::At(deadline_us)
        }

        VelocityValidateState::ReverseSettle { deadline_us } => {
            let now = backend.elapsed_us();
            if now < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            data.prev_encoder_for_velocity =
                match backend.try_read_mechanical_encoder_position_counts().await {
                    Ok(value) => Some(
                        safety
                            .encoder_domain
                            .expect("outer encoder domain configured before velocity validation")
                            .count(value),
                    ),
                    Err(_) => return fail_outer(phase, CommissionError::Spi),
                };
            buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
            let Some(next_sample_us) = now.checked_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US) else {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            };
            *state = VelocityValidateState::ChatterCheck {
                next_sample_us,
                last_sample_us: now,
                remaining: VELOCITY_VALIDATE_CHATTER_WINDOW,
                ambiguity_retried: false,
            };
            StepResult::At(next_sample_us)
        }

        VelocityValidateState::ChatterCheck {
            next_sample_us,
            last_sample_us,
            remaining,
            ambiguity_retried,
        } => {
            let now = backend.elapsed_us();
            if now < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }

            let count = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let prev_encoder = data
                .prev_encoder_for_velocity
                .expect("chatter velocity baseline configured");
            let counts_per_rev = safety
                .encoder_domain
                .map(EncoderDomain::counts_per_rev)
                .unwrap_or(0);
            let Some(elapsed_us) = now.checked_sub(*last_sample_us) else {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            };
            let velocity = match encoder_delta_counts_per_ms(
                count,
                prev_encoder.raw_counts(),
                counts_per_rev,
                elapsed_us,
            ) {
                Ok(velocity) => velocity,
                Err(EncoderMathError::InvalidCountsPerRev) => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    return fail_outer(phase, CommissionError::EncoderFault);
                }
                Err(EncoderMathError::AmbiguousHalfTurn) if !*ambiguity_retried => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    if let Err(error) = clear_velocity_integrator(backend).await {
                        return fail_outer(phase, error);
                    }
                    data.max_chatter = 0;
                    data.prev_encoder_for_velocity = Some(
                        safety
                            .encoder_domain
                            .expect("validated domain")
                            .count(count),
                    );
                    *ambiguity_retried = true;
                    *remaining = VELOCITY_VALIDATE_CHATTER_WINDOW;
                    let Some(deadline_us) = now.checked_add(VELOCITY_VALIDATE_SAMPLE_PERIOD_US)
                    else {
                        return fail_outer(phase, CommissionError::InvalidSchedule);
                    };
                    *next_sample_us = deadline_us;
                    *last_sample_us = now;
                    return StepResult::At(deadline_us);
                }
                Err(EncoderMathError::AmbiguousHalfTurn) => {
                    if let Err(error) = command_velocity_target(backend, safety, 0).await {
                        return fail_outer(phase, error);
                    }
                    return fail_outer(phase, CommissionError::VelocityValidationFailed);
                }
            };
            data.prev_encoder_for_velocity = Some(
                safety
                    .encoder_domain
                    .expect("validated domain")
                    .count(count),
            );
            if velocity > data.max_chatter {
                data.max_chatter = velocity;
            }

            *remaining -= 1;
            if *remaining == 0 {
                *state = VelocityValidateState::Classify;
                StepResult::ReadyNow
            } else {
                let next_deadline_us = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
                    now,
                ) {
                    Ok(deadline_us) => deadline_us,
                    Err(error) => return fail_outer(phase, error),
                };
                *next_sample_us = next_deadline_us;
                *last_sample_us = now;
                StepResult::At(next_deadline_us)
            }
        }

        VelocityValidateState::Classify => {
            let fwd = data.forward_metrics.unwrap_or(VelocityValidation {
                rise_time_ms: 0,
                overshoot_permille: Permille::ZERO,
                peak_velocity: 0,
                settling_time_ms: 0,
                zero_speed_chatter: false,
                saturation_samples: 0,
                symmetry_permille: Permille::ZERO,
            });
            let rev = data.reverse_metrics.unwrap_or(VelocityValidation {
                rise_time_ms: 0,
                overshoot_permille: Permille::ZERO,
                peak_velocity: 0,
                settling_time_ms: 0,
                zero_speed_chatter: false,
                saturation_samples: 0,
                symmetry_permille: Permille::ZERO,
            });

            let symmetry_permille =
                crate::autotune::compute_symmetry_permille(fwd.rise_time_ms, rev.rise_time_ms);

            let merged = VelocityValidation {
                rise_time_ms: fwd.rise_time_ms.max(rev.rise_time_ms),
                overshoot_permille: fwd.overshoot_permille.max(rev.overshoot_permille),
                peak_velocity: fwd.peak_velocity.max(rev.peak_velocity),
                settling_time_ms: fwd.settling_time_ms.max(rev.settling_time_ms),
                zero_speed_chatter: data.max_chatter > VELOCITY_VALIDATE_CHATTER_THRESHOLD,
                saturation_samples: data.saturation_count,
                symmetry_permille,
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
            let stiffness = crate::autotune::classify_velocity_stiffness(
                &merged,
                result,
                data.stiffness_timebase_ms,
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
                symmetry_permille.raw()
            );

            let exhausted_response_budget = data.strategy == VelocityTuneStrategy::ResponseRamp
                && !data.has_accepted_gains
                && data.attempt >= response_ramp::VELOCITY_RESPONSE_MAX_RETRIES
                && result != ValidationResult::Pass;
            let (summary_level, summary_status) = if exhausted_response_budget {
                (
                    TraceEventLevel::Error,
                    CommissionError::VelocityValidationFailed.status_code(),
                )
            } else {
                (
                    match result {
                        ValidationResult::Pass => TraceEventLevel::Info,
                        ValidationResult::Marginal | ValidationResult::Fail => {
                            TraceEventLevel::Warn
                        }
                    },
                    0,
                )
            };
            record_velocity_validate_summary(
                backend,
                trace_summary_seqs,
                data,
                &fwd,
                &rev,
                summary_status,
                summary_level,
            );

            if data.strategy == VelocityTuneStrategy::ResponseRamp {
                match result {
                    ValidationResult::Pass => {
                        data.accepted_gains = data.gains;
                        data.has_accepted_gains = true;

                        if data.attempt < response_ramp::VELOCITY_RESPONSE_MAX_RETRIES
                            && let Some(next_p) =
                                response_ramp::response_next_velocity_p(data.gains.velocity_p)
                        {
                            data.attempt = data.attempt.saturating_add(1);
                            data.gains.velocity_p = next_p;
                            data.gains.velocity_i = PidGain::ZERO;
                            defmt::info!(
                                "vel response: clean response, trying P={}, I=0",
                                data.gains.velocity_p
                            );
                            *state = VelocityValidateState::WriteGains;
                            return StepResult::ReadyNow;
                        } else {
                            defmt::info!(
                                "vel response: cap reached, accepted P={}, I={}",
                                data.accepted_gains.velocity_p,
                                data.accepted_gains.velocity_i
                            );
                            if !stop_velocity_motion(backend, safety).await {
                                return fail_outer(phase, CommissionError::Spi);
                            }
                            let deadline = match velocity_settle_deadline(backend.elapsed_us()) {
                                Ok(deadline_us) => deadline_us,
                                Err(error) => return fail_outer(phase, error),
                            };
                            *state = VelocityValidateState::SafeState {
                                deadline_us: deadline,
                            };
                            return StepResult::At(deadline);
                        }
                    }
                    ValidationResult::Marginal | ValidationResult::Fail => {
                        let failed_p = data.gains.velocity_p.raw();
                        if !data.has_accepted_gains
                            && data.attempt >= response_ramp::VELOCITY_RESPONSE_MAX_RETRIES
                        {
                            defmt::warn!(
                                "vel response: {} at P={} exhausted retry budget ({})",
                                match result {
                                    ValidationResult::Fail => "fail",
                                    _ => "marginal",
                                },
                                data.gains.velocity_p,
                                data.attempt
                            );
                            if !stop_velocity_motion(backend, safety).await {
                                return fail_outer(phase, CommissionError::Spi);
                            }
                            return fail_outer(phase, CommissionError::VelocityValidationFailed);
                        }
                        if !data.has_accepted_gains
                            && response_ramp::response_result_is_gain_limited(
                                &merged,
                                &fwd,
                                &rev,
                                data.step_target,
                                total_samples,
                            )
                            && let Some(next_p) = response_ramp::response_next_gain_limited_p(
                                data.gains.velocity_p,
                                data.response_upper_p,
                            )
                        {
                            data.response_lower_p = failed_p;
                            data.attempt = data.attempt.saturating_add(1);
                            data.gains.velocity_p = next_p;
                            data.gains.velocity_i = PidGain::ZERO;
                            defmt::info!(
                                "vel response: {} at P={} without passing gain, trying P={}",
                                match result {
                                    ValidationResult::Fail => "fail",
                                    _ => "marginal",
                                },
                                data.gains.velocity_p,
                                data.gains.velocity_p
                            );
                            *state = VelocityValidateState::WriteGains;
                            return StepResult::ReadyNow;
                        }
                        if !data.has_accepted_gains {
                            let gain_limited = response_ramp::response_result_is_gain_limited(
                                &merged,
                                &fwd,
                                &rev,
                                data.step_target,
                                total_samples,
                            );
                            if gain_limited {
                                data.response_lower_p = failed_p;
                            } else {
                                data.response_upper_p = if data.response_upper_p == 0 {
                                    failed_p
                                } else {
                                    data.response_upper_p.min(failed_p)
                                };
                            }
                            if let Some(next_p) = response_ramp::response_midpoint_p(
                                data.response_lower_p,
                                data.response_upper_p,
                            ) {
                                data.attempt = data.attempt.saturating_add(1);
                                data.gains.velocity_p = next_p;
                                data.gains.velocity_i = PidGain::ZERO;
                                defmt::info!(
                                    "vel response: {} at P={} bracketed by P={}..{}, trying P={}",
                                    match result {
                                        ValidationResult::Fail => "fail",
                                        _ => "marginal",
                                    },
                                    failed_p,
                                    data.response_lower_p,
                                    data.response_upper_p,
                                    data.gains.velocity_p
                                );
                                *state = VelocityValidateState::WriteGains;
                                return StepResult::ReadyNow;
                            }
                            if data.response_lower_p > 0
                                && data.response_upper_p > data.response_lower_p
                                && let Some(next_target) = response_ramp::response_next_step_target(
                                    data.step_target,
                                    data.gains.velocity_limit,
                                )
                            {
                                let old_target = data.step_target;
                                let Some(next_p) = response_ramp::response_scaled_p_for_target(
                                    data.response_lower_p,
                                    old_target,
                                    next_target,
                                ) else {
                                    return fail_outer(
                                        phase,
                                        CommissionError::VelocityValidationFailed,
                                    );
                                };
                                data.gains.velocity_p = next_p;
                                data.gains.velocity_i = PidGain::ZERO;
                                data.step_target = next_target;
                                data.response_lower_p = 0;
                                data.response_upper_p = 0;
                                data.attempt = data.attempt.saturating_add(1);
                                defmt::info!(
                                    "vel response: no passing P bracket at target={}cpms, trying target={}cpms P={}",
                                    old_target,
                                    data.step_target,
                                    data.gains.velocity_p
                                );
                                *state = VelocityValidateState::WriteGains;
                                return StepResult::ReadyNow;
                            }
                            defmt::warn!(
                                "vel response: {} at P={} with no passing response gain",
                                match result {
                                    ValidationResult::Fail => "fail",
                                    _ => "marginal",
                                },
                                failed_p
                            );
                            if !stop_velocity_motion(backend, safety).await {
                                return fail_outer(phase, CommissionError::Spi);
                            }
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
                        if !stop_velocity_motion(backend, safety).await {
                            return fail_outer(phase, CommissionError::Spi);
                        }
                        let deadline = match velocity_settle_deadline(backend.elapsed_us()) {
                            Ok(deadline_us) => deadline_us,
                            Err(error) => return fail_outer(phase, error),
                        };
                        *state = VelocityValidateState::SafeState {
                            deadline_us: deadline,
                        };
                        return StepResult::At(deadline);
                    }
                }
            }

            if data.strategy == VelocityTuneStrategy::Model
                && result == ValidationResult::Pass
                && stiffness == crate::autotune::VelocityStiffnessVerdict::UnderDriven
            {
                data.accepted_gains = data.gains;
                data.has_accepted_gains = true;
                data.diagnostics.outer_evidence_flags |= OUTER_EVIDENCE_UNDER_DRIVEN_PROBE;
                if let Some(next_p) = response_ramp::response_next_velocity_p(data.gains.velocity_p)
                {
                    data.attempt = data.attempt.saturating_add(1);
                    data.gains.velocity_p = next_p;
                    data.gains.velocity_i = PidGain::ZERO;
                    defmt::info!(
                        "vel validate: under-driven model response, trying P={}, I=0",
                        data.gains.velocity_p
                    );
                    *state = VelocityValidateState::WriteGains;
                    return StepResult::ReadyNow;
                }

                data.velocity_search_stop_reason = VELOCITY_SEARCH_STOP_UNDER_DRIVEN_BUDGET;
                if !stop_velocity_motion(backend, safety).await {
                    return fail_outer(phase, CommissionError::Spi);
                }
                let deadline = match velocity_settle_deadline(backend.elapsed_us()) {
                    Ok(deadline_us) => deadline_us,
                    Err(error) => return fail_outer(phase, error),
                };
                *state = VelocityValidateState::SafeState {
                    deadline_us: deadline,
                };
                return StepResult::At(deadline);
            }

            match result {
                ValidationResult::Pass => {
                    data.velocity_search_stop_reason = VELOCITY_SEARCH_STOP_CLEAN;
                    if !stop_velocity_motion(backend, safety).await {
                        return fail_outer(phase, CommissionError::Spi);
                    }
                    let deadline = match velocity_settle_deadline(backend.elapsed_us()) {
                        Ok(deadline_us) => deadline_us,
                        Err(error) => return fail_outer(phase, error),
                    };
                    *state = VelocityValidateState::SafeState {
                        deadline_us: deadline,
                    };
                    StepResult::At(deadline)
                }
                ValidationResult::Marginal if data.attempt == 0 => {
                    defmt::info!("vel validate: marginal on attempt 0, retrying");
                    data.attempt = 1;

                    let config = backend.tmc_config();
                    let encoder_counts_per_rev =
                        backend.mechanical_encoder_quadrature_counts_per_rev();
                    let mut retry_model = data.model;
                    let new_theta = data.model.theta_mech_us * VELOCITY_RETRY_LAMBDA_MUL_NUM
                        / VELOCITY_RETRY_LAMBDA_MUL_DEN;
                    retry_model.theta_mech_us = new_theta;

                    match crate::autotune::simc_velocity_gains_with_lambda_multiplier(
                        &retry_model,
                        data.policy.effective_profile,
                        mode,
                        inner_lambda_us,
                        theta_e_us,
                        data.peak_velocity,
                        encoder_counts_per_rev,
                        config,
                        data.policy.lambda_multiplier,
                    ) {
                        Some(mut new_gains) => {
                            data.diagnostics.synth_velocity_p = new_gains.velocity_p.raw();
                            data.diagnostics.synth_velocity_i = new_gains.velocity_i.raw();
                            if new_gains.velocity_p.raw() > OUTER_MAX_VELOCITY_P
                                || new_gains.velocity_i.raw() > OUTER_MAX_VELOCITY_I
                            {
                                data.diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
                                defmt::warn!(
                                    "vel retry: synthesized P={}, I={} outside envelope, using response tuning",
                                    new_gains.velocity_p,
                                    new_gains.velocity_i
                                );
                                new_gains =
                                    response::response_velocity_gains(safety.counts_per_rev());
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
                            StepResult::ReadyNow
                        }
                        None => fail_outer(phase, CommissionError::VelocityValidationFailed),
                    }
                }
                ValidationResult::Marginal | ValidationResult::Fail if data.has_accepted_gains => {
                    let failed_p = data.gains.velocity_p;
                    data.gains = data.accepted_gains;
                    data.velocity_search_stop_reason = VELOCITY_SEARCH_STOP_UNSAFE_BACKOFF;
                    defmt::warn!(
                        "vel validate: {} at P={}, restoring passing P={}",
                        match result {
                            ValidationResult::Fail => "fail",
                            _ => "marginal",
                        },
                        failed_p,
                        data.accepted_gains.velocity_p
                    );
                    if !stop_velocity_motion(backend, safety).await {
                        return fail_outer(phase, CommissionError::Spi);
                    }
                    let deadline = match velocity_settle_deadline(backend.elapsed_us()) {
                        Ok(deadline_us) => deadline_us,
                        Err(error) => return fail_outer(phase, error),
                    };
                    *state = VelocityValidateState::SafeState {
                        deadline_us: deadline,
                    };
                    StepResult::At(deadline)
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
                    data.gains = response::response_velocity_gains(safety.counts_per_rev());
                    data.accepted_gains = data.gains;
                    data.has_accepted_gains = false;
                    data.response_lower_p = 0;
                    data.response_upper_p = 0;
                    data.strategy = VelocityTuneStrategy::ResponseRamp;
                    data.attempt = 0;
                    data.step_target = response::VELOCITY_RESPONSE_STEP_TARGET;
                    data.forward_metrics = None;
                    data.reverse_metrics = None;
                    data.max_chatter = 0;
                    data.saturation_count = 0;
                    if !stop_velocity_motion(backend, safety).await {
                        return fail_outer(phase, CommissionError::Spi);
                    }
                    *state = VelocityValidateState::WriteGains;
                    StepResult::ReadyNow
                }
            }
        }

        VelocityValidateState::SafeState { deadline_us } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            *state = VelocityValidateState::Finalize;
            StepResult::ReadyNow
        }

        VelocityValidateState::Finalize => {
            if !stop_velocity_motion(backend, safety).await {
                return fail_outer(phase, CommissionError::Spi);
            }

            if !data.mech_tau_trusted {
                data.diagnostics.warning_code =
                    response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID;
                data.velocity_search_stop_reason = VELOCITY_SEARCH_STOP_LOW_CONFIDENCE;
            }

            let carried_safety = *safety;
            data.diagnostics.stiffness_timebase_ms =
                data.stiffness_timebase_ms.min(u16::MAX as u32) as u16;
            data.diagnostics.velocity_search_stop_reason = data.velocity_search_stop_reason;
            data.diagnostics.motion_budget_mrev = carried_safety.motion_budget_mrev();
            if data.diagnostics.warning_code == response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID
            {
                data.diagnostics.outer_evidence_flags |= OUTER_EVIDENCE_LOW_CONFIDENCE_MECH;
            }
            if data.diagnostics.mech_tau_defaulted {
                data.diagnostics.outer_evidence_flags |= OUTER_EVIDENCE_TAU_DEFAULTED;
            }
            if carried_safety.budget_clamped_firmware_caps() {
                data.diagnostics.outer_evidence_flags |= OUTER_EVIDENCE_MOTION_BUDGET_CLAMPED;
            }

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

            // Position capture is measured in ABN counts, while the TMC target
            // uses 65536 mechanical position units per revolution.
            let step_target_counts = response::position_tune_step_counts(safety.counts_per_rev());
            let step_target_tmc = encoder_counts_to_tmc_position_units(
                step_target_counts.min(i32::MAX as u32) as i32,
                backend.mechanical_encoder_quadrature_counts_per_rev(),
            )
            .max(1) as u32;

            let j_eff = data.model.j_eff;
            let b_eff = data.model.b_eff;

            let pos_data = PositionTuneData {
                current_p: POS_TUNE_INITIAL_P,
                stable_p: 0,
                step_target_counts,
                step_target_tmc,
                step_start_counts: None,
                step_start_tmc: 0,
                oscillation_at_minimum: false,
                velocity_gains: accepted_gains,
                velocity_chatter,
                current_ringing,
                current_bandwidth_hz,
                j_eff,
                b_eff,
                torque_limit: data.torque_limit,
                saturation_count: 0,
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
    use super::{
        VELOCITY_RETRY_LAMBDA_MUL_DEN, VELOCITY_RETRY_LAMBDA_MUL_NUM,
        VELOCITY_VALIDATE_MAX_REGISTER_OPS_PER_ARM, VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
        command_velocity_target, encoder_counts_to_tmc_position_units, encoder_delta_counts_per_ms,
        encoder_rate_counts_per_ms_to_tmc_velocity_signed,
        encoder_rate_magnitude_counts_per_ms_to_tmc_velocity, record_velocity_validate_summary,
        step_velocity_validate,
    };
    use crate::autotune::{
        CommissionError, CommissionProfile, MechanicalModel, OperatingMode, Permille,
        TRANSIENT_LEN, TransientBuffer, VelocityGains, VelocityValidation,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain};
    use crate::commissioning::outer::{
        EffectiveVelocityPolicy, OuterDiagnostics, OuterPhase, OuterSafetyLimits, TraceSummarySeqs,
        VELOCITY_SEARCH_STOP_CLEAN, VELOCITY_SEARCH_STOP_UNSAFE_BACKOFF, VelocityTuneStrategy,
        VelocityValidateData, VelocityValidateState, response, trace_pack_i16_pair,
        trace_pack_u16_pair,
    };
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::pid::PidGain;
    use foci_tmc4671::registers::{
        PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_VELOCITY_LIMIT, PID_VELOCITY_P_VELOCITY_I,
        PID_VELOCITY_TARGET, SubRegister,
    };

    fn has_sub_register_write(ops: &[SpiOp], sub: SubRegister, value: u32) -> bool {
        ops.windows(2).any(|pair| {
            matches!(pair[0], SpiOp::Write(address, selector)
                if address == sub.addr.addr() && selector == u32::from(sub.sub_addr))
                && matches!(pair[1], SpiOp::Write(address, data)
                    if address == sub.data.addr() && data == value)
        })
    }

    fn gain(raw: u16) -> PidGain {
        PidGain::new(raw).expect("test gain must be in range")
    }

    fn encoder_count(raw_counts: u32) -> EncoderCount {
        EncoderDomain::new(4_000).unwrap().count(raw_counts)
    }

    fn clean_effective_policy() -> EffectiveVelocityPolicy {
        EffectiveVelocityPolicy {
            effective_profile: CommissionProfile::Balanced,
            lambda_multiplier: 1.0,
            conservative_required: false,
            lambda_widened: false,
        }
    }

    fn configured_safety() -> OuterSafetyLimits {
        let mut safety = OuterSafetyLimits::new(
            2448,
            crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
        );
        safety.configure_encoder_resolution(4_000).unwrap();
        safety.set_encoder_baseline(0).unwrap();
        safety
    }

    #[test]
    fn forward_and_reverse_velocity_targets_use_separate_primitive_timers() {
        pollster::block_on(async {
            let mut safety = configured_safety();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 100;

            assert!(
                command_velocity_target(&mut backend, &mut safety, 42)
                    .await
                    .is_ok()
            );
            assert_eq!(safety.primitive_start_us, Some(100));

            backend.elapsed_us_value = 200;
            assert!(
                command_velocity_target(&mut backend, &mut safety, 0)
                    .await
                    .is_ok()
            );
            assert_eq!(safety.primitive_start_us, None);

            backend.elapsed_us_value = 300;
            assert!(
                command_velocity_target(&mut backend, &mut safety, (-42_i32) as u32)
                    .await
                    .is_ok()
            );
            assert_eq!(safety.primitive_start_us, Some(300));
        });
    }

    #[test]
    fn failed_velocity_target_does_not_start_a_primitive_timer() {
        pollster::block_on(async {
            let mut safety = configured_safety();
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(PID_VELOCITY_TARGET.addr());

            assert!(matches!(
                command_velocity_target(&mut backend, &mut safety, 42).await,
                Err(CommissionError::Spi)
            ));
            assert_eq!(safety.primitive_start_us, None);
        });
    }

    #[test]
    fn velocity_captures_wait_for_the_exact_sample_deadline() {
        pollster::block_on(async {
            for point in [
                AmbiguousCapturePoint::Forward,
                AmbiguousCapturePoint::Reverse,
            ] {
                let mut phase =
                    capture_phase(VelocityTuneStrategy::Model, point, configured_safety());
                let mut buf = TransientBuffer::new();
                let mut backend = MockBackend::new();
                backend.elapsed_us_value = 999;

                let result = step_velocity_validate(
                    &mut phase,
                    &mut buf,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    &mut backend,
                )
                .await;

                assert!(matches!(result, StepResult::At(1_000)));
                assert!(backend.spi_log.is_empty());
            }
        });
    }

    #[test]
    fn velocity_settles_use_one_shot_deadlines() {
        pollster::block_on(async {
            let mut phase = capture_phase(
                VelocityTuneStrategy::Model,
                AmbiguousCapturePoint::Forward,
                configured_safety(),
            );
            if let OuterPhase::VelocityValidate { state, .. } = &mut phase {
                *state = VelocityValidateState::ForwardSettle { deadline_us: 1_000 };
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 999;

            let waiting = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(waiting, StepResult::At(1_000)));

            backend.elapsed_us_value = 1_000;
            let elapsed = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(elapsed, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                OuterPhase::VelocityValidate {
                    state: VelocityValidateState::ReverseClearIntegrator,
                    ..
                }
            ));

            if let OuterPhase::VelocityValidate { state, .. } = &mut phase {
                *state = VelocityValidateState::ReverseSettle { deadline_us: 2_000 };
            }
            backend.elapsed_us_value = 1_999;
            let reverse_waiting = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(reverse_waiting, StepResult::At(2_000)));

            backend.elapsed_us_value = 2_000;
            let reverse_elapsed = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(reverse_elapsed, StepResult::At(3_000)));
            assert!(matches!(
                phase,
                OuterPhase::VelocityValidate {
                    state: VelocityValidateState::ChatterCheck {
                        next_sample_us: 3_000,
                        last_sample_us: 2_000,
                        ..
                    },
                    ..
                }
            ));
        });
    }

    #[test]
    fn velocity_integrator_is_cleared_before_the_first_forward_step() {
        pollster::block_on(async {
            let mut phase = capture_phase(
                VelocityTuneStrategy::Model,
                AmbiguousCapturePoint::Forward,
                configured_safety(),
            );
            if let OuterPhase::VelocityValidate { state, .. } = &mut phase {
                *state = VelocityValidateState::WriteGains;
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let setup = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(setup, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                OuterPhase::VelocityValidate {
                    state: VelocityValidateState::ForwardClearIntegrator,
                    ..
                }
            ));

            backend.spi_log.clear();
            let clear = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(clear, StepResult::ReadyNow));
            assert!(has_sub_register_write(
                &backend.spi_log,
                PID_ERROR_PID_VELOCITY_ERROR_SUM,
                0
            ));
            assert!(matches!(
                phase,
                OuterPhase::VelocityValidate {
                    state: VelocityValidateState::ForwardStep,
                    ..
                }
            ));
        });
    }

    #[test]
    fn velocity_integrator_clear_failure_fails_closed() {
        pollster::block_on(async {
            let mut phase = capture_phase(
                VelocityTuneStrategy::Model,
                AmbiguousCapturePoint::Forward,
                configured_safety(),
            );
            if let OuterPhase::VelocityValidate { state, .. } = &mut phase {
                *state = VelocityValidateState::ForwardClearIntegrator;
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr());

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, OuterPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn completed_signed_steps_clear_the_integrator_before_settling() {
        pollster::block_on(async {
            for point in [
                AmbiguousCapturePoint::Forward,
                AmbiguousCapturePoint::Reverse,
            ] {
                let mut phase =
                    capture_phase(VelocityTuneStrategy::Model, point, configured_safety());
                let mut buf = TransientBuffer::new();
                buf.reset(VELOCITY_VALIDATE_SAMPLE_PERIOD_US as u16);
                for _ in 0..(TRANSIENT_LEN - 1) {
                    assert!(buf.push(0));
                }
                let mut backend = MockBackend::new();
                backend.elapsed_us_value = 1_000;
                backend.mechanical_encoder_position_counts_value = match point {
                    AmbiguousCapturePoint::Forward => 4,
                    AmbiguousCapturePoint::Reverse => 3_996,
                    AmbiguousCapturePoint::Chatter => unreachable!(),
                };

                let completed = step_velocity_validate(
                    &mut phase,
                    &mut buf,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    &mut backend,
                )
                .await;
                assert!(matches!(completed, StepResult::ReadyNow));
                assert!(
                    backend
                        .spi_log
                        .iter()
                        .any(|op| matches!(op, SpiOp::Write(address, 0)
                        if *address == PID_VELOCITY_TARGET.addr()))
                );
                assert!(!has_sub_register_write(
                    &backend.spi_log,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    0
                ));
                match (&point, &phase) {
                    (
                        AmbiguousCapturePoint::Forward,
                        OuterPhase::VelocityValidate {
                            state:
                                VelocityValidateState::ForwardZeroClearIntegrator {
                                    deadline_us: 201_000,
                                },
                            ..
                        },
                    )
                    | (
                        AmbiguousCapturePoint::Reverse,
                        OuterPhase::VelocityValidate {
                            state:
                                VelocityValidateState::ReverseZeroClearIntegrator {
                                    deadline_us: 201_000,
                                },
                            ..
                        },
                    ) => {}
                    other => panic!("unexpected post-capture state: {other:?}"),
                }

                backend.spi_log.clear();
                let cleared = step_velocity_validate(
                    &mut phase,
                    &mut buf,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    &mut backend,
                )
                .await;
                assert!(matches!(cleared, StepResult::At(201_000)));
                assert!(has_sub_register_write(
                    &backend.spi_log,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    0
                ));
                match (&point, &phase) {
                    (
                        AmbiguousCapturePoint::Forward,
                        OuterPhase::VelocityValidate {
                            state:
                                VelocityValidateState::ForwardSettle {
                                    deadline_us: 201_000,
                                },
                            ..
                        },
                    )
                    | (
                        AmbiguousCapturePoint::Reverse,
                        OuterPhase::VelocityValidate {
                            state:
                                VelocityValidateState::ReverseSettle {
                                    deadline_us: 201_000,
                                },
                            ..
                        },
                    ) => {}
                    other => panic!("unexpected cleared state: {other:?}"),
                }
            }
        });
    }

    #[test]
    fn chatter_capture_waits_and_preserves_the_sample_grid() {
        pollster::block_on(async {
            let mut phase = capture_phase(
                VelocityTuneStrategy::Model,
                AmbiguousCapturePoint::Chatter,
                configured_safety(),
            );
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 999;

            let waiting = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(waiting, StepResult::At(1_000)));
            assert!(backend.spi_log.is_empty());

            backend.elapsed_us_value = 1_600;
            let sampled = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(sampled, StepResult::At(2_000)));
            assert!(matches!(
                phase,
                OuterPhase::VelocityValidate {
                    state: VelocityValidateState::ChatterCheck {
                        next_sample_us: 2_000,
                        last_sample_us: 1_600,
                        remaining: 99,
                        ..
                    },
                    ..
                }
            ));
        });
    }

    #[test]
    fn velocity_safe_state_settle_uses_one_shot_deadline() {
        pollster::block_on(async {
            let mut phase = capture_phase(
                VelocityTuneStrategy::Model,
                AmbiguousCapturePoint::Forward,
                configured_safety(),
            );
            if let OuterPhase::VelocityValidate { state, .. } = &mut phase {
                *state = VelocityValidateState::SafeState { deadline_us: 1_000 };
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 999;

            let waiting = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(waiting, StepResult::At(1_000)));

            backend.elapsed_us_value = 1_000;
            let elapsed = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(elapsed, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                OuterPhase::VelocityValidate {
                    state: VelocityValidateState::Finalize,
                    ..
                }
            ));
        });
    }

    #[test]
    fn velocity_validation_respects_the_per_arm_register_bound() {
        pollster::block_on(async {
            let mut phase = capture_phase(
                VelocityTuneStrategy::Model,
                AmbiguousCapturePoint::Forward,
                configured_safety(),
            );
            if let OuterPhase::VelocityValidate { state, .. } = &mut phase {
                *state = VelocityValidateState::WriteGains;
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert_eq!(
                backend.spi_log.len(),
                VELOCITY_VALIDATE_MAX_REGISTER_OPS_PER_ARM
            );
        });
    }

    #[derive(Debug, Clone, Copy)]
    enum AmbiguousCapturePoint {
        Forward,
        Reverse,
        Chatter,
    }

    fn capture_phase(
        strategy: VelocityTuneStrategy,
        point: AmbiguousCapturePoint,
        safety: OuterSafetyLimits,
    ) -> OuterPhase {
        let gains = VelocityGains {
            velocity_p: gain(1_152),
            velocity_i: gain(7),
            velocity_limit: 16,
            effective_bandwidth_hz: 10,
            lambda_vel_ms: 50,
        };
        let accepted_gains = VelocityGains {
            velocity_p: gain(768),
            velocity_i: gain(3),
            ..gains
        };
        let state = match point {
            AmbiguousCapturePoint::Forward => VelocityValidateState::ForwardCapture {
                next_sample_us: 1_000,
                last_sample_us: 0,
                ambiguity_retried: false,
                saturation_start: 4,
            },
            AmbiguousCapturePoint::Reverse => VelocityValidateState::ReverseCapture {
                next_sample_us: 1_000,
                last_sample_us: 0,
                ambiguity_retried: false,
                saturation_start: 4,
            },
            AmbiguousCapturePoint::Chatter => VelocityValidateState::ChatterCheck {
                next_sample_us: 1_000,
                last_sample_us: 0,
                remaining: 100,
                ambiguity_retried: false,
            },
        };
        let metrics = VelocityValidation {
            rise_time_ms: 11,
            overshoot_permille: Permille::new(125),
            peak_velocity: 4,
            settling_time_ms: 17,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::new(900),
        };

        OuterPhase::VelocityValidate {
            state,
            data: VelocityValidateData {
                gains,
                accepted_gains,
                has_accepted_gains: true,
                response_lower_p: 512,
                response_upper_p: 1_536,
                model: MechanicalModel {
                    j_eff: 1,
                    b_eff: 1,
                    tau_mech_ms: 50,
                    theta_mech_us: 1,
                    t_f: 0,
                },
                policy: clean_effective_policy(),
                strategy,
                stiffness_timebase_ms: 50,
                mech_tau_trusted: true,
                velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                attempt: 3,
                step_target: 4,
                forward_metrics: Some(metrics),
                reverse_metrics: Some(metrics),
                max_chatter: 12,
                peak_velocity: 20,
                prev_encoder_for_velocity: Some(encoder_count(0)),
                saturation_count: 9,
                torque_limit: 0,
                operational_velocity_limit: 500_000,
                diagnostics: zero_diagnostics(),
            },
            safety,
        }
    }

    async fn assert_ambiguous_capture_retries_without_gain_search_mutation(
        strategy: VelocityTuneStrategy,
        point: AmbiguousCapturePoint,
    ) {
        let mut phase = capture_phase(strategy, point, configured_safety());
        let mut buf = TransientBuffer::new();
        if !matches!(point, AmbiguousCapturePoint::Chatter) {
            assert!(buf.push(99));
        }
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 1_000;
        backend.mechanical_encoder_position_counts_value = 2_000;

        let result = step_velocity_validate(
            &mut phase,
            &mut buf,
            OperatingMode::Nominal,
            0,
            0,
            false,
            0,
            &mut backend,
        )
        .await;

        match point {
            AmbiguousCapturePoint::Forward | AmbiguousCapturePoint::Reverse => {
                assert!(matches!(result, StepResult::At(2_000)));
            }
            AmbiguousCapturePoint::Chatter => {
                assert!(matches!(result, StepResult::At(2_000)));
            }
        }
        assert!(buf.samples().is_empty());
        assert!(backend.spi_log.iter().any(
            |op| matches!(op, SpiOp::Write(address, 0) if *address == PID_VELOCITY_TARGET.addr())
        ));
        assert!(has_sub_register_write(
            &backend.spi_log,
            PID_ERROR_PID_VELOCITY_ERROR_SUM,
            0
        ));
        match phase {
            OuterPhase::VelocityValidate { state, data, .. } => {
                assert_eq!(data.strategy, strategy);
                assert_eq!(data.attempt, 3);
                assert_eq!(data.gains.velocity_p.raw(), 1_152);
                assert_eq!(data.accepted_gains.velocity_p.raw(), 768);
                assert!(data.has_accepted_gains);
                assert_eq!(data.response_lower_p, 512);
                assert_eq!(data.response_upper_p, 1_536);
                match point {
                    AmbiguousCapturePoint::Forward => {
                        assert!(data.forward_metrics.is_none());
                        assert!(data.reverse_metrics.is_some());
                        assert_eq!(data.saturation_count, 4);
                        assert!(matches!(
                            state,
                            VelocityValidateState::ForwardCapture {
                                ambiguity_retried: true,
                                next_sample_us: 2_000,
                                last_sample_us: 1_000,
                                ..
                            }
                        ));
                    }
                    AmbiguousCapturePoint::Reverse => {
                        assert!(data.forward_metrics.is_some());
                        assert!(data.reverse_metrics.is_none());
                        assert_eq!(data.saturation_count, 4);
                        assert!(matches!(
                            state,
                            VelocityValidateState::ReverseCapture {
                                ambiguity_retried: true,
                                next_sample_us: 2_000,
                                last_sample_us: 1_000,
                                ..
                            }
                        ));
                    }
                    AmbiguousCapturePoint::Chatter => {
                        assert!(data.forward_metrics.is_some());
                        assert!(data.reverse_metrics.is_some());
                        assert_eq!(data.max_chatter, 0);
                        assert!(matches!(
                            state,
                            VelocityValidateState::ChatterCheck {
                                ambiguity_retried: true,
                                remaining: 200,
                                next_sample_us: 2_000,
                                last_sample_us: 1_000,
                            }
                        ));
                    }
                }
            }
            other => panic!("expected velocity validation retry, got {other:?}"),
        }
    }

    #[test]
    fn ambiguous_forward_capture_retries_for_both_tuning_strategies() {
        pollster::block_on(async {
            for strategy in [
                VelocityTuneStrategy::Model,
                VelocityTuneStrategy::ResponseRamp,
            ] {
                assert_ambiguous_capture_retries_without_gain_search_mutation(
                    strategy,
                    AmbiguousCapturePoint::Forward,
                )
                .await;
            }
        });
    }

    #[test]
    fn ambiguous_reverse_capture_retries_for_both_tuning_strategies() {
        pollster::block_on(async {
            for strategy in [
                VelocityTuneStrategy::Model,
                VelocityTuneStrategy::ResponseRamp,
            ] {
                assert_ambiguous_capture_retries_without_gain_search_mutation(
                    strategy,
                    AmbiguousCapturePoint::Reverse,
                )
                .await;
            }
        });
    }

    #[test]
    fn ambiguous_chatter_capture_retries_for_both_tuning_strategies() {
        pollster::block_on(async {
            for strategy in [
                VelocityTuneStrategy::Model,
                VelocityTuneStrategy::ResponseRamp,
            ] {
                assert_ambiguous_capture_retries_without_gain_search_mutation(
                    strategy,
                    AmbiguousCapturePoint::Chatter,
                )
                .await;
            }
        });
    }

    #[test]
    fn invalid_velocity_domain_commands_zero_and_fails_immediately() {
        pollster::block_on(async {
            for strategy in [
                VelocityTuneStrategy::Model,
                VelocityTuneStrategy::ResponseRamp,
            ] {
                let safety = OuterSafetyLimits::new(
                    2_448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                );
                let mut phase = capture_phase(strategy, AmbiguousCapturePoint::Forward, safety);
                let mut buf = TransientBuffer::new();
                let mut backend = MockBackend::new();
                backend.elapsed_us_value = 1_000;
                backend.mechanical_encoder_position_counts_value = 1;

                let result = step_velocity_validate(
                    &mut phase,
                    &mut buf,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    &mut backend,
                )
                .await;

                assert!(matches!(
                    result,
                    StepResult::OuterFailed(CommissionError::EncoderFault)
                ));
                assert!(buf.samples().is_empty());
                assert!(backend.spi_log.iter().any(|op| {
                    matches!(op, SpiOp::Write(address, 0) if *address == PID_VELOCITY_TARGET.addr())
                }));
            }
        });
    }

    #[test]
    fn repeated_velocity_ambiguity_commands_zero_and_exhausts_retry() {
        pollster::block_on(async {
            for strategy in [
                VelocityTuneStrategy::Model,
                VelocityTuneStrategy::ResponseRamp,
            ] {
                for point in [
                    AmbiguousCapturePoint::Forward,
                    AmbiguousCapturePoint::Reverse,
                    AmbiguousCapturePoint::Chatter,
                ] {
                    let mut phase = capture_phase(strategy, point, configured_safety());
                    if let OuterPhase::VelocityValidate { state, .. } = &mut phase {
                        match state {
                            VelocityValidateState::ForwardCapture {
                                ambiguity_retried, ..
                            }
                            | VelocityValidateState::ReverseCapture {
                                ambiguity_retried, ..
                            }
                            | VelocityValidateState::ChatterCheck {
                                ambiguity_retried, ..
                            } => *ambiguity_retried = true,
                            _ => unreachable!("fixture always starts in a capture state"),
                        }
                    }
                    let mut buf = TransientBuffer::new();
                    let mut backend = MockBackend::new();
                    backend.elapsed_us_value = 1_000;
                    backend.mechanical_encoder_position_counts_value = 2_000;

                    let result = step_velocity_validate(
                        &mut phase,
                        &mut buf,
                        OperatingMode::Nominal,
                        0,
                        0,
                        false,
                        0,
                        &mut backend,
                    )
                    .await;

                    assert!(matches!(
                        result,
                        StepResult::OuterFailed(CommissionError::VelocityValidationFailed)
                    ));
                    assert!(backend.spi_log.iter().any(|op| {
                        matches!(
                            op,
                            SpiOp::Write(address, 0)
                                if *address == PID_VELOCITY_TARGET.addr()
                        )
                    }));
                }
            }
        });
    }

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
            mech_tau_defaulted: false,
            synth_velocity_p: 0,
            synth_velocity_i: 0,
            synth_position_p: 0,
            synth_position_i: 0,
            outer_evidence_flags: 0,
            stiffness_timebase_ms: 0,
            velocity_search_stop_reason: 0,
            motion_budget_mrev: 0,
        }
    }

    fn failing_velocity_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 0,
            overshoot_permille: Permille::new(11_500),
            peak_velocity: 25,
            settling_time_ms: 256,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        }
    }

    fn marginal_model_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 10,
            overshoot_permille: Permille::new(300),
            peak_velocity: 20,
            settling_time_ms: 100,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        }
    }

    #[test]
    fn velocity_validate_summary_packs_exact_fragments() {
        let mut backend = MockBackend::new();
        let mut seqs = TraceSummarySeqs::new();
        let data = VelocityValidateData {
            gains: VelocityGains {
                velocity_p: gain(512),
                velocity_i: gain(32),
                velocity_limit: 10_000,
                effective_bandwidth_hz: 50,
                lambda_vel_ms: 20,
            },
            accepted_gains: VelocityGains {
                velocity_p: PidGain::ZERO,
                velocity_i: PidGain::ZERO,
                velocity_limit: 0,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            },
            has_accepted_gains: false,
            response_lower_p: 0,
            response_upper_p: 0,
            model: MechanicalModel {
                j_eff: 1,
                b_eff: 1,
                tau_mech_ms: 1,
                theta_mech_us: 1,
                t_f: 0,
            },
            policy: clean_effective_policy(),
            strategy: VelocityTuneStrategy::Model,
            stiffness_timebase_ms: 50,
            mech_tau_trusted: true,
            velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
            attempt: 0,
            step_target: 100,
            forward_metrics: None,
            reverse_metrics: None,
            max_chatter: 7,
            peak_velocity: 0,
            prev_encoder_for_velocity: None,
            saturation_count: 3,
            torque_limit: 1000,
            operational_velocity_limit: 20_000,
            diagnostics: zero_diagnostics(),
        };
        let fwd = VelocityValidation {
            peak_velocity: 123,
            ..failing_velocity_metrics()
        };
        let rev = VelocityValidation {
            peak_velocity: 45,
            ..failing_velocity_metrics()
        };

        record_velocity_validate_summary(
            &mut backend,
            &mut seqs,
            &data,
            &fwd,
            &rev,
            0,
            crate::trace::events::TraceEventLevel::Info,
        );

        assert_eq!(
            backend.trace_events[0].event_id,
            crate::trace::events::EVENT_COMMISSION_VELOCITY_VALIDATE_RESULT
        );
        assert_eq!(backend.trace_events[0].payload, [0, 0]);
        assert_eq!(
            backend.trace_events[1].event_id,
            crate::trace::events::EVENT_COMMISSION_VELOCITY_VALIDATE_GAIN
        );
        assert_eq!(
            backend.trace_events[1].payload,
            [0, trace_pack_u16_pair(512, 32)]
        );
        assert_eq!(
            backend.trace_events[2].event_id,
            crate::trace::events::EVENT_COMMISSION_VELOCITY_VALIDATE_METRICS
        );
        assert_eq!(
            backend.trace_events[2].payload,
            [0, trace_pack_i16_pair(123, 45)]
        );
        assert_eq!(
            backend.trace_events[3].event_id,
            crate::trace::events::EVENT_COMMISSION_VELOCITY_VALIDATE_LIMITS
        );
        assert_eq!(
            backend.trace_events[3].payload,
            [0, trace_pack_u16_pair(3, 7)]
        );
    }

    #[test]
    fn response_ramp_fail_does_not_advance_to_position_tune() {
        pollster::block_on(async {
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains: VelocityGains {
                        velocity_p: gain(1152),
                        velocity_i: PidGain::ZERO,
                        velocity_limit: 16,
                        effective_bandwidth_hz: 0,
                        lambda_vel_ms: 0,
                    },
                    accepted_gains: VelocityGains {
                        velocity_p: gain(1152),
                        velocity_i: PidGain::ZERO,
                        velocity_limit: 16,
                        effective_bandwidth_hz: 0,
                        lambda_vel_ms: 0,
                    },
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 0,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: Some(VelocityValidation {
                        rise_time_ms: 20,
                        overshoot_permille: Permille::new(5250),
                        peak_velocity: 25,
                        settling_time_ms: 256,
                        zero_speed_chatter: false,
                        saturation_samples: 0,
                        symmetry_permille: Permille::ZERO,
                    }),
                    reverse_metrics: Some(VelocityValidation {
                        rise_time_ms: 0,
                        overshoot_permille: Permille::new(5000),
                        peak_velocity: 24,
                        settling_time_ms: 256,
                        zero_speed_chatter: false,
                        saturation_samples: 0,
                        symmetry_permille: Permille::ZERO,
                    }),
                    max_chatter: 24,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: OuterDiagnostics {
                        warning_code: 1,
                        mech_torque_step: 367,
                        mech_fwd_accel: 0,
                        mech_rev_accel: 0,
                        mech_fwd_peak_velocity: 18,
                        mech_rev_peak_velocity: 18,
                        mech_fwd_travel: 3907,
                        mech_rev_travel: 3886,
                        mech_tau_defaulted: false,
                        synth_velocity_p: 0,
                        synth_velocity_i: 0,
                        synth_position_p: 0,
                        synth_position_i: 0,
                        outer_evidence_flags: 0,
                        stiffness_timebase_ms: 0,
                        velocity_search_stop_reason: 0,
                        motion_budget_mrev: 0,
                    },
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
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

    fn low_energy_marginal_velocity_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 0,
            overshoot_permille: Permille::ZERO,
            peak_velocity: 0,
            settling_time_ms: 256,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        }
    }

    fn stiction_breakaway_metrics(
        peak_velocity: u32,
        overshoot_permille: Permille,
    ) -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 0,
            overshoot_permille,
            settling_time_ms: 256,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
            peak_velocity,
        }
    }

    fn under_driven_pass_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 200,
            overshoot_permille: Permille::ZERO,
            peak_velocity: 20,
            settling_time_ms: 120,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        }
    }

    fn model_failed_velocity_phase() -> OuterPhase {
        let gains = VelocityGains {
            velocity_p: gain(4608),
            velocity_i: gain(1),
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
                response_lower_p: 0,
                response_upper_p: 0,
                model: MechanicalModel {
                    j_eff: 2_712_665,
                    b_eff: 20_451,
                    tau_mech_ms: 132,
                    theta_mech_us: 12_895,
                    t_f: 0,
                },
                policy: clean_effective_policy(),
                strategy: VelocityTuneStrategy::Model,
                stiffness_timebase_ms: 50,
                mech_tau_trusted: true,
                velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                attempt: 0,
                step_target: 2,
                forward_metrics: Some(failing_velocity_metrics()),
                reverse_metrics: Some(failing_velocity_metrics()),
                max_chatter: 24,
                peak_velocity: 18,
                prev_encoder_for_velocity: None,
                saturation_count: 484,
                torque_limit: 2448,
                operational_velocity_limit: 500_000,
                diagnostics: zero_diagnostics(),
            },
            safety: configured_safety(),
        }
    }

    #[test]
    fn marginal_retry_reuses_effective_policy() {
        pollster::block_on(async {
            let model = MechanicalModel {
                j_eff: 1_000_000,
                b_eff: 1_000,
                tau_mech_ms: 100,
                theta_mech_us: 20_000,
                t_f: 0,
            };
            let policy = EffectiveVelocityPolicy {
                effective_profile: CommissionProfile::Conservative,
                lambda_multiplier: 1.8,
                conservative_required: true,
                lambda_widened: true,
            };
            let initial_gains = VelocityGains {
                velocity_p: gain(1),
                velocity_i: gain(1),
                velocity_limit: 1_000,
                effective_bandwidth_hz: 1,
                lambda_vel_ms: 1,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains: initial_gains,
                    accepted_gains: initial_gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model,
                    policy,
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: 100,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 100,
                    forward_metrics: Some(marginal_model_metrics()),
                    reverse_metrics: Some(marginal_model_metrics()),
                    max_chatter: 0,
                    peak_velocity: 1_000,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2_448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
            let retry_model = MechanicalModel {
                theta_mech_us: model.theta_mech_us * VELOCITY_RETRY_LAMBDA_MUL_NUM
                    / VELOCITY_RETRY_LAMBDA_MUL_DEN,
                ..model
            };
            let expected = crate::autotune::simc_velocity_gains_with_lambda_multiplier(
                &retry_model,
                policy.effective_profile,
                OperatingMode::Nominal,
                1_000,
                100,
                1_000,
                4_000,
                &backend.tmc_config,
                policy.lambda_multiplier,
            )
            .expect("retry model should synthesize");

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                1_000,
                100,
                false,
                1_600,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.attempt, 1);
                    assert_eq!(data.gains, expected);
                }
                other => panic!("expected velocity retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn model_pass_under_driven_probes_higher_p() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(512),
                velocity_i: gain(4),
                velocity_limit: 40,
                effective_bandwidth_hz: 10,
                lambda_vel_ms: 50,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: Some(under_driven_pass_metrics()),
                    reverse_metrics: Some(under_driven_pass_metrics()),
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert!(data.has_accepted_gains);
                    assert_eq!(data.accepted_gains.velocity_p, gains.velocity_p);
                    assert!(data.gains.velocity_p.raw() > gains.velocity_p.raw());
                    assert_eq!(data.gains.velocity_i, PidGain::ZERO);
                }
                other => panic!("expected higher-P model probe, got {:?}", other),
            }
        });
    }

    #[test]
    fn model_probe_restores_last_clean_when_higher_p_fails() {
        pollster::block_on(async {
            let accepted_gains = VelocityGains {
                velocity_p: gain(512),
                velocity_i: gain(4),
                velocity_limit: 40,
                effective_bandwidth_hz: 10,
                lambda_vel_ms: 50,
            };
            let failing_gains = VelocityGains {
                velocity_p: gain(768),
                velocity_i: PidGain::ZERO,
                ..accepted_gains
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains: failing_gains,
                    accepted_gains,
                    has_accepted_gains: true,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 1,
                    step_target: 4,
                    forward_metrics: Some(failing_velocity_metrics()),
                    reverse_metrics: Some(failing_velocity_metrics()),
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(200_000)));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::SafeState { .. }));
                    assert_eq!(data.gains.velocity_p, accepted_gains.velocity_p);
                    assert_eq!(
                        data.velocity_search_stop_reason,
                        VELOCITY_SEARCH_STOP_UNSAFE_BACKOFF
                    );
                }
                other => panic!("expected accepted model gains restore, got {:?}", other),
            }
        });
    }

    #[test]
    fn low_confidence_tau_accepts_with_warning_not_production() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(512),
                velocity_i: gain(4),
                velocity_limit: 40,
                effective_bandwidth_hz: 10,
                lambda_vel_ms: 50,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Finalize,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: response::VELOCITY_RESPONSE_VALIDATE_TAU_MS,
                    mech_tau_trusted: false,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: None,
                    reverse_metrics: None,
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
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
                StepResult::PhaseChanged(crate::autotune::PhaseId::PositionTune)
            ));
            match phase {
                OuterPhase::PositionTune { data, .. } => {
                    assert_eq!(
                        data.diagnostics.warning_code,
                        response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID
                    );
                }
                other => panic!("expected PositionTune with warning, got {:?}", other),
            }
        });
    }

    #[test]
    fn defaulted_tau_accepts_with_warning_not_production() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(512),
                velocity_i: gain(4),
                velocity_limit: 40,
                effective_bandwidth_hz: 10,
                lambda_vel_ms: 50,
            };
            let mut diagnostics = zero_diagnostics();
            diagnostics.mech_tau_defaulted = true;
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Finalize,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 1000,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: response::VELOCITY_RESPONSE_VALIDATE_TAU_MS,
                    mech_tau_trusted: false,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: None,
                    reverse_metrics: None,
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics,
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
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
                StepResult::PhaseChanged(crate::autotune::PhaseId::PositionTune)
            ));
            match phase {
                OuterPhase::PositionTune { data, .. } => {
                    assert_eq!(
                        data.diagnostics.warning_code,
                        response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID
                    );
                }
                other => panic!("expected PositionTune with warning, got {:?}", other),
            }
        });
    }

    #[test]
    fn velocity_validation_converts_encoder_rate_through_exact_scale() {
        assert_eq!(
            encoder_rate_counts_per_ms_to_tmc_velocity_signed(4, 4_000, 25_000),
            3
        );
        assert_eq!(
            encoder_rate_counts_per_ms_to_tmc_velocity_signed(-4, 4_000, 25_000),
            -3
        );
        assert_eq!(
            encoder_rate_magnitude_counts_per_ms_to_tmc_velocity(16, 4_000, 25_000),
            10
        );
    }

    #[test]
    fn velocity_conversion_preserves_signed_and_unsigned_saturation_contracts() {
        assert_eq!(
            encoder_rate_counts_per_ms_to_tmc_velocity_signed(i64::MAX, 1, 1),
            i32::MAX
        );
        assert_eq!(
            encoder_rate_counts_per_ms_to_tmc_velocity_signed(i64::MIN, 1, 1),
            i32::MIN
        );
        assert_eq!(
            encoder_rate_magnitude_counts_per_ms_to_tmc_velocity(u64::MAX, 1, 1),
            u32::MAX
        );
    }

    #[test]
    fn position_target_converts_signed_encoder_counts_through_exact_scale() {
        assert_eq!(encoder_counts_to_tmc_position_units(31, 4_000), 508);
        assert_eq!(encoder_counts_to_tmc_position_units(-31, 4_000), -508);
    }

    #[test]
    fn velocity_validation_normalizes_encoder_delta_by_elapsed_time() {
        assert_eq!(
            encoder_delta_counts_per_ms(1_024, 1_000, 4_000, 1_600),
            Ok(15)
        );
        assert_eq!(
            encoder_delta_counts_per_ms(1_024, 1_000, 4_000, 1_000),
            Ok(24)
        );
        assert_eq!(
            encoder_delta_counts_per_ms(1_000, 1_000, 4_000, 1_600),
            Ok(0)
        );
        assert_eq!(encoder_delta_counts_per_ms(1_024, 1_000, 4_000, 0), Ok(0));
    }

    #[test]
    fn velocity_sample_preserves_origin_crossing_and_rejects_invalid_evidence() {
        assert_eq!(encoder_delta_counts_per_ms(5, 3_995, 4_000, 1_000), Ok(10));
        assert_eq!(
            encoder_delta_counts_per_ms(5, 3_995, 0, 1_000),
            Err(crate::commissioning::encoder_domain::EncoderMathError::InvalidCountsPerRev)
        );
        assert_eq!(
            encoder_delta_counts_per_ms(2_000, 0, 4_000, 1_000),
            Err(crate::commissioning::encoder_domain::EncoderMathError::AmbiguousHalfTurn)
        );
    }

    #[test]
    fn velocity_validation_capture_uses_elapsed_time_for_sample_velocity() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(1152),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::ForwardCapture {
                    next_sample_us: 1_000,
                    last_sample_us: 0,
                    ambiguity_retried: false,
                    saturation_start: 0,
                },
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: None,
                    reverse_metrics: None,
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: Some(encoder_count(1_000)),
                    saturation_count: 0,
                    torque_limit: 1_000,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_600;
            backend.mechanical_encoder_position_counts_value = 1_024;
            let torque_output = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
            backend
                .registers
                .insert(torque_output.data.addr(), 900)
                .unwrap();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(2_000)));
            assert_eq!(buf.samples(), &[15]);
            #[cfg(feature = "trace")]
            assert_eq!(
                backend.velocity_trace_observations.as_slice(),
                &[crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: 4,
                    delta_counts: 24,
                    elapsed_us: 1_600,
                    stroke_index: 0,
                    direction: 1,
                    flags: crate::trace::stream::VELOCITY_OBSERVATION_FLAG_SATURATED,
                }]
            );
            match &phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert_eq!(data.saturation_count, 1);
                    assert_eq!(
                        data.prev_encoder_for_velocity
                            .map(|count| count.raw_counts()),
                        Some(1_024)
                    );
                    assert!(matches!(
                        state,
                        VelocityValidateState::ForwardCapture {
                            next_sample_us: 2_000,
                            last_sample_us: 1_600,
                            ..
                        }
                    ));
                }
                other => panic!("expected velocity validate phase, got {:?}", other),
            }

            #[cfg(feature = "trace")]
            {
                let OuterPhase::VelocityValidate { state, data, .. } = &mut phase else {
                    unreachable!("velocity phase checked above");
                };
                *state = VelocityValidateState::ReverseCapture {
                    next_sample_us: 2_600,
                    last_sample_us: 1_600,
                    ambiguity_retried: false,
                    saturation_start: data.saturation_count,
                };
                data.prev_encoder_for_velocity = Some(encoder_count(1_024));
                backend.elapsed_us_value = 2_600;
                backend.mechanical_encoder_position_counts_value = 1_000;
                backend
                    .registers
                    .insert(torque_output.data.addr(), 0)
                    .unwrap();

                let reverse_result = step_velocity_validate(
                    &mut phase,
                    &mut buf,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    &mut backend,
                )
                .await;

                assert!(matches!(reverse_result, StepResult::At(3_600)));
                assert_eq!(buf.samples(), &[15, 24]);
                assert_eq!(
                    backend.velocity_trace_observations[1],
                    crate::trace::stream::VelocityTraceObservation {
                        target_counts_per_ms: -4,
                        delta_counts: -24,
                        elapsed_us: 1_000,
                        stroke_index: 1,
                        direction: -1,
                        flags: 0,
                    }
                );
            }
        });
    }

    #[test]
    fn velocity_forward_capture_cleanup_write_failure_fails_spi() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(1152),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut safety = OuterSafetyLimits::new(
                2448,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::ForwardCapture {
                    next_sample_us: 1_000,
                    last_sample_us: 0,
                    ambiguity_retried: false,
                    saturation_start: 0,
                },
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: None,
                    reverse_metrics: None,
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: Some(encoder_count(1_000)),
                    saturation_count: 0,
                    torque_limit: 0,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety,
            };
            let mut buf = TransientBuffer::new();
            buf.reset(1000);
            for _ in 0..(TRANSIENT_LEN - 1) {
                assert!(buf.push(0));
            }
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 1_004;
            backend.fail_on_write = Some(PID_VELOCITY_TARGET.addr());

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, OuterPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn velocity_validation_capture_uses_wrap_aware_encoder_delta() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(1152),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut safety = OuterSafetyLimits::new(
                2448,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::ForwardCapture {
                    next_sample_us: 1_000,
                    last_sample_us: 0,
                    ambiguity_retried: false,
                    saturation_start: 0,
                },
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: None,
                    reverse_metrics: None,
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: Some(encoder_count(3_995)),
                    saturation_count: 0,
                    torque_limit: 0,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 5;

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(2_000)));
            assert_eq!(buf.samples(), &[10]);
        });
    }

    #[test]
    fn velocity_validation_conversion_handles_zero_inputs() {
        assert_eq!(
            encoder_rate_counts_per_ms_to_tmc_velocity_signed(0, 4_000, 25_000),
            0
        );
        assert_eq!(
            encoder_rate_magnitude_counts_per_ms_to_tmc_velocity(4, 0, 25_000),
            0
        );
        assert_eq!(
            encoder_rate_magnitude_counts_per_ms_to_tmc_velocity(4, 4_000, 0),
            0
        );
    }

    #[test]
    fn velocity_validation_writes_encoder_scaled_rates_independent_of_step_width() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(1152),
                velocity_i: gain(341),
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
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::Model,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: None,
                    reverse_metrics: None,
                    max_chatter: 0,
                    peak_velocity: 20,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 0,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;

            let _ = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            let _ = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(has_sub_register_write(
                &backend.spi_log,
                PID_ERROR_PID_VELOCITY_ERROR_SUM,
                0
            ));

            assert_eq!(
                *backend
                    .registers
                    .get(&PID_VELOCITY_P_VELOCITY_I.addr())
                    .expect("velocity gains should be written"),
                (u32::from(gains.velocity_p) << 16) | u32::from(gains.velocity_i)
            );
            assert_eq!(
                *backend
                    .registers
                    .get(&PID_VELOCITY_LIMIT.addr())
                    .expect("velocity limit should be written"),
                10
            );

            let _ = step_velocity_validate(
                &mut phase,
                &mut buf,
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
                3
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
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert_eq!(
                        data.gains.velocity_p.raw(),
                        response::VELOCITY_RESPONSE_INITIAL_P
                    );
                    assert_eq!(data.gains.velocity_i, PidGain::ZERO);
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
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
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
            let gains = response::response_velocity_gains(4_000);
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(low_energy_marginal_velocity_metrics()),
                    reverse_metrics: Some(low_energy_marginal_velocity_metrics()),
                    max_chatter: 0,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    assert!(
                        data.gains.velocity_p.raw() > gains.velocity_p.raw(),
                        "low-energy response should advance P, got {} from {}",
                        data.gains.velocity_p.raw(),
                        gains.velocity_p.raw()
                    );
                }
                other => panic!("expected response-ramp search to continue, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_stiction_breakaway_without_accepted_gain_keeps_searching() {
        pollster::block_on(async {
            let gains = response::response_velocity_gains(4_000);
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: gains,
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(stiction_breakaway_metrics(8, Permille::new(250))),
                    reverse_metrics: Some(stiction_breakaway_metrics(1, Permille::ZERO)),
                    max_chatter: 0,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    assert!(
                        data.gains.velocity_p.raw() > gains.velocity_p.raw(),
                        "stiction breakaway should advance P, got {} from {}",
                        data.gains.velocity_p.raw(),
                        gains.velocity_p.raw()
                    );
                }
                other => panic!("expected response-ramp search to continue, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_brackets_between_underdriven_and_unsafe_without_accepted_gain() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(648),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: response::response_velocity_gains(4_000),
                    has_accepted_gains: false,
                    response_lower_p: 432,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(failing_velocity_metrics()),
                    reverse_metrics: Some(failing_velocity_metrics()),
                    max_chatter: 23,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 283,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    assert_eq!(data.response_lower_p, 432);
                    assert_eq!(data.response_upper_p, 648);
                    assert_eq!(data.gains.velocity_p.raw(), 540);
                }
                other => panic!("expected bracketed response-ramp search, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_raises_step_target_when_p_bracket_collapses_under_stiction() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(485),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: response::response_velocity_gains(4_000),
                    has_accepted_gains: false,
                    response_lower_p: 484,
                    response_upper_p: 486,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(stiction_breakaway_metrics(2, Permille::ZERO)),
                    reverse_metrics: Some(stiction_breakaway_metrics(2, Permille::ZERO)),
                    max_chatter: 1,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    assert_eq!(
                        data.step_target,
                        response::VELOCITY_RESPONSE_STEP_TARGET * 2
                    );
                    assert_eq!(data.response_lower_p, 0);
                    assert_eq!(data.response_upper_p, 0);
                    assert_eq!(data.gains.velocity_p.raw(), 243);
                }
                other => panic!(
                    "expected raised-target response-ramp search, got {:?}",
                    other
                ),
            }
        });
    }

    #[test]
    fn response_ramp_treats_one_sided_overshoot_as_stiction_at_higher_target() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(645),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: response::response_velocity_gains(4_000),
                    has_accepted_gains: false,
                    response_lower_p: 580,
                    response_upper_p: 774,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET * 2,
                    forward_metrics: Some(stiction_breakaway_metrics(13, Permille::new(250))),
                    reverse_metrics: Some(stiction_breakaway_metrics(2, Permille::ZERO)),
                    max_chatter: 0,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 1,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(!data.has_accepted_gains);
                    assert_eq!(data.response_lower_p, 645);
                    assert_eq!(data.response_upper_p, 774);
                    assert_eq!(data.gains.velocity_p.raw(), 709);
                }
                other => panic!("expected one-sided stiction search, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_raises_step_target_when_p_bracket_is_too_narrow_to_search() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(668),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: response::response_velocity_gains(4_000),
                    has_accepted_gains: false,
                    response_lower_p: 648,
                    response_upper_p: 688,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 0,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(stiction_breakaway_metrics(2, Permille::ZERO)),
                    reverse_metrics: Some(stiction_breakaway_metrics(2, Permille::ZERO)),
                    max_chatter: 1,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
                OperatingMode::Nominal,
                934,
                160,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            match phase {
                OuterPhase::VelocityValidate { state, data, .. } => {
                    assert!(matches!(state, VelocityValidateState::WriteGains));
                    assert_eq!(
                        data.step_target,
                        response::VELOCITY_RESPONSE_STEP_TARGET * 2
                    );
                    assert_eq!(data.response_lower_p, 0);
                    assert_eq!(data.response_upper_p, 0);
                    assert_eq!(data.gains.velocity_p.raw(), 334);
                }
                other => panic!("expected raised-target retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_stops_when_retry_budget_is_exhausted() {
        pollster::block_on(async {
            let gains = VelocityGains {
                velocity_p: gain(688),
                velocity_i: PidGain::ZERO,
                velocity_limit: 16,
                effective_bandwidth_hz: 0,
                lambda_vel_ms: 0,
            };
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains,
                    accepted_gains: response::response_velocity_gains(4_000),
                    has_accepted_gains: false,
                    response_lower_p: 648,
                    response_upper_p: 729,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 1,
                        t_f: 0,
                    },
                    policy: clean_effective_policy(),
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    stiffness_timebase_ms: 50,
                    mech_tau_trusted: true,
                    velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
                    attempt: 12,
                    step_target: response::VELOCITY_RESPONSE_STEP_TARGET,
                    forward_metrics: Some(failing_velocity_metrics()),
                    reverse_metrics: Some(failing_velocity_metrics()),
                    max_chatter: 23,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: None,
                    saturation_count: 277,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: zero_diagnostics(),
                },
                safety: configured_safety(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_velocity_validate(
                &mut phase,
                &mut buf,
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
}
