//! Phase 11: Position-loop P ramp-up.
//!
//! Starts from a conservative P, captures the position step response into
//! the transient buffer, runs oscillation detection, and either ramps P
//! upward by `POSITION_TUNE_P_MUL_NUM/DEN` or finalizes at the last
//! stable value when oscillation is observed or the cap is reached.
//! Response-tuned velocity gains use a tighter cap and disable position
//! integral.

use super::{
    OUTER_FALLBACK_POSITION_P, OUTER_MAX_POSITION_P, OuterPhase, POS_TUNE_INITIAL_P,
    PositionTuneState,
};
use crate::autotune::{CommissionError, OperatingMode, PhaseId, PositionGains, TransientBuffer};
use crate::commissioning::outer::fail_outer;
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Position-tune P-gain growth ratio between attempts.
const POSITION_TUNE_P_MUL_NUM: u32 = 3;
const POSITION_TUNE_P_MUL_DEN: u32 = 2;

/// Final position I gain as a percentage of the accepted P (5%).
const POSITION_TUNE_I_PERCENT: u32 = 5;

/// Position-tune ramp cap when SIMC velocity gains were accepted.
const POSITION_TUNE_MAX_P: u16 = OUTER_MAX_POSITION_P;

/// Position-tune ramp cap when the response-ramp velocity fallback was used.
const POSITION_RESPONSE_TUNE_MAX_P: u16 = OUTER_FALLBACK_POSITION_P;

/// Sample period for the position step response (microseconds).
const POSITION_TUNE_SAMPLE_PERIOD_US: u64 = 1000;

/// Settle period after returning to the starting position (microseconds).
const POSITION_TUNE_SETTLE_US: u64 = 200_000;

/// Position mode with `MODE_PID_TYPE` set, matching production closed-loop entry.
const OUTER_POSITION_MODE: u32 = 0x8000_0003;

/// Advance position-tune sub-state machine.
pub(crate) async fn step_position_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    mode: OperatingMode,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (state, data, safety) = match phase {
        OuterPhase::PositionTune {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::Continue,
    };

    match state {
        PositionTuneState::Setup => {
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, OUTER_POSITION_MODE)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            let pos_reg = (data.current_p as u32) << 16;
            if backend
                .write_register(PID_POSITION_P_POSITION_I, pos_reg)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            // Record both position domains. ABN counts are used for response
            // measurement; PID_POSITION_ACTUAL is the TMC target domain used
            // by PID_POSITION_TARGET.
            let encoder_raw = backend.read_encoder_count().await;
            data.step_start_counts = encoder_raw as i32;
            let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(v) => v as i32,
                Err(_) => {
                    return fail_outer(phase, CommissionError::Spi);
                }
            };
            data.step_start_tmc = position_raw;

            let target = position_raw.wrapping_add(data.step_target_tmc as i32);
            if backend
                .write_register(PID_POSITION_TARGET, target as u32)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            defmt::info!(
                "pos tune setup: P={} target_counts={} target_tmc={} torque_limit={} \
                 vel_gains P={} I={} lambda_ms={}",
                data.current_p,
                data.step_target_counts,
                data.step_target_tmc,
                data.torque_limit,
                data.velocity_gains.velocity_p,
                data.velocity_gains.velocity_i,
                data.velocity_gains.lambda_vel_ms,
            );

            buf.reset(POSITION_TUNE_SAMPLE_PERIOD_US as u16);
            data.saturation_count = 0;

            let next_sample_us = backend
                .elapsed_us()
                .wrapping_add(POSITION_TUNE_SAMPLE_PERIOD_US);
            *state = PositionTuneState::StepCapture { next_sample_us };
            StepResult::Continue
        }

        PositionTuneState::StepCapture { next_sample_us } => {
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let encoder_raw = backend.read_encoder_count().await;
            // Wrap-aware displacement from step start, normalized to
            // [-counts_per_rev/2, +counts_per_rev/2). Stored unsigned for
            // comparison against the unsigned step target_abs in the
            // oscillation detector.
            let delta = crate::commissioning::outer::response::encoder_delta(
                encoder_raw as i32,
                data.step_start_counts,
                safety.counts_per_rev,
            );
            buf.push(delta.unsigned_abs());

            if data.torque_limit > 0 {
                let sub = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
                let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                if let Ok(pidout) = backend.read_register(sub.data).await {
                    let commanded_torque = (pidout & 0xFFFF) as i16;
                    let abs_commanded = commanded_torque.unsigned_abs() as u32;
                    if abs_commanded >= data.torque_limit * 9 / 10 {
                        data.saturation_count = data.saturation_count.saturating_add(1);
                    }
                }
            }

            if buf.is_full() {
                let _ = backend
                    .write_register(PID_POSITION_TARGET, data.step_start_tmc as u32)
                    .await;

                let deadline = backend.elapsed_us().wrapping_add(POSITION_TUNE_SETTLE_US);
                *state = PositionTuneState::Settle {
                    deadline_us: deadline,
                };
            } else {
                *next_sample_us = now.wrapping_add(POSITION_TUNE_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        PositionTuneState::Settle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                *state = PositionTuneState::Analyze;
            }
            StepResult::Continue
        }

        PositionTuneState::Analyze => {
            let target_abs = data.step_target_counts;
            let diag = crate::autotune::analyze_position_oscillation(buf, target_abs);
            let oscillation = diag.oscillating;

            // Diagnostic dump: log the inputs that drove the bool plus three
            // 16-sample windows (head, around peak, tail) and the saturation
            // count from the capture so a cascade-saturation issue is
            // distinguishable from quantization-driven limit cycling.
            let samples_slice = buf.samples();
            let total = samples_slice.len();
            let head_end = total.min(16);
            let peak_idx = samples_slice
                .iter()
                .position(|&s| s == diag.peak)
                .unwrap_or(0);
            let mid_start = peak_idx.saturating_sub(8).min(total.saturating_sub(16));
            let mid_end = (mid_start + 16).min(total);
            let tail_start = total.saturating_sub(16);
            defmt::info!(
                "pos tune: P={} target_abs={} peak={} peak_idx={} overshoot={} crossings={} \
                 saturation={}",
                data.current_p,
                target_abs,
                diag.peak,
                peak_idx,
                diag.overshoot,
                diag.crossings,
                data.saturation_count,
            );
            defmt::info!("pos tune samples_head: {:?}", &samples_slice[..head_end]);
            defmt::info!(
                "pos tune samples_mid: {:?}",
                &samples_slice[mid_start..mid_end]
            );
            defmt::info!(
                "pos tune samples_tail: {:?}",
                &samples_slice[tail_start..total]
            );
            defmt::info!("pos tune: oscillation={}", oscillation);

            if oscillation {
                if data.stable_p == 0 {
                    data.oscillation_at_minimum = true;
                    defmt::warn!("pos tune: minimum P={} oscillates", data.current_p);
                }
                *state = PositionTuneState::Finalize;
            } else {
                data.stable_p = data.current_p;

                let response_tuned = data.velocity_gains.effective_bandwidth_hz == 0;
                let max_p = if response_tuned {
                    POSITION_RESPONSE_TUNE_MAX_P
                } else {
                    POSITION_TUNE_MAX_P
                };
                if data.current_p >= max_p {
                    defmt::info!("pos tune: P cap reached at {}", data.stable_p);
                    *state = PositionTuneState::Finalize;
                } else {
                    let new_p = (data.current_p as u32 * POSITION_TUNE_P_MUL_NUM
                        / POSITION_TUNE_P_MUL_DEN)
                        .min(max_p as u32) as u16;
                    if new_p <= data.current_p {
                        defmt::info!("pos tune: P cap reached at {}", data.stable_p);
                        *state = PositionTuneState::Finalize;
                        return StepResult::Continue;
                    }
                    data.current_p = new_p;

                    let pos_reg = (data.current_p as u32) << 16;
                    let _ = backend
                        .write_register(PID_POSITION_P_POSITION_I, pos_reg)
                        .await;

                    let encoder_raw = backend.read_encoder_count().await;
                    data.step_start_counts = encoder_raw as i32;
                    let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
                        Ok(v) => v as i32,
                        Err(_) => {
                            return fail_outer(phase, CommissionError::Spi);
                        }
                    };
                    data.step_start_tmc = position_raw;
                    let target = position_raw.wrapping_add(data.step_target_tmc as i32);
                    let _ = backend
                        .write_register(PID_POSITION_TARGET, target as u32)
                        .await;

                    buf.reset(POSITION_TUNE_SAMPLE_PERIOD_US as u16);
                    data.saturation_count = 0;
                    let next_sample_us = backend
                        .elapsed_us()
                        .wrapping_add(POSITION_TUNE_SAMPLE_PERIOD_US);
                    *state = PositionTuneState::StepCapture { next_sample_us };
                }
            }
            StepResult::Continue
        }

        PositionTuneState::Finalize => {
            let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

            if data.oscillation_at_minimum {
                return fail_outer(phase, CommissionError::PositionTuneFailed);
            }

            let final_p = data.stable_p;

            // Response-tuned velocity gains are deliberately conservative:
            // the tuner has not measured a steady-state position bias, so
            // leave position integral disabled to avoid windup on installed
            // mechanics.
            let response_tuned = data.velocity_gains.effective_bandwidth_hz == 0;
            let final_i = if response_tuned {
                0
            } else {
                match mode {
                    OperatingMode::Nominal | OperatingMode::HighInertia => {
                        (final_p as u32 * POSITION_TUNE_I_PERCENT / 100).max(1) as u16
                    }
                    OperatingMode::Unloaded => 0,
                }
            };

            defmt::info!("pos tune: final P={}, I={}", final_p, final_i);

            let carried_safety = *safety;
            let position_gains = PositionGains {
                position_p: final_p,
                position_i: final_i,
            };
            let velocity_gains = data.velocity_gains;
            let velocity_chatter = data.velocity_chatter;
            let current_ringing = data.current_ringing;
            let current_bandwidth_hz = data.current_bandwidth_hz;
            let j_eff = data.j_eff;
            let b_eff = data.b_eff;
            let diagnostics = data.diagnostics;
            // Position oscillation is "early" if the first stable P was
            // reached only at the minimum value (suggesting the system
            // is marginal for position control).
            let position_oscillation_early =
                final_p == POS_TUNE_INITIAL_P && data.current_p > POS_TUNE_INITIAL_P;

            *phase = OuterPhase::FilterTune {
                velocity_gains,
                position_gains,
                position_oscillation_early,
                velocity_chatter,
                current_ringing,
                current_bandwidth_hz,
                j_eff,
                b_eff,
                diagnostics,
                safety: carried_safety,
            };
            StepResult::PhaseChanged(PhaseId::FilterTune)
        }
    }
}
