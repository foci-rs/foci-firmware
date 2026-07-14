//! Phase 11: Position-loop P ramp-up.
//!
//! Starts from a conservative P, captures the position step response into
//! the transient buffer, runs oscillation detection, and either ramps P
//! upward by `POSITION_TUNE_P_MUL_NUM/DEN` or finalizes at the last
//! stable value when oscillation is observed or the cap is reached.
//! Response-tuned velocity gains use a tighter cap and disable position
//! integral.

use super::{
    OUTER_FALLBACK_POSITION_P, OUTER_MAX_POSITION_P, OuterPhase, OuterSafetyLimits,
    POS_TUNE_INITIAL_P, PositionTuneData, PositionTuneState, TraceSummarySeqs, trace_pack_u16_pair,
    trace_summary_event,
};
use crate::autotune::{CommissionError, OperatingMode, PhaseId, PositionGains, TransientBuffer};
use crate::commissioning::outer::fail_outer;
use crate::commissioning::types::{CommissioningBackend, StepResult};
use crate::trace::events::{
    EVENT_COMMISSION_POSITION_TUNE_LIMITS, EVENT_COMMISSION_POSITION_TUNE_METRICS,
    EVENT_COMMISSION_POSITION_TUNE_RESULT, EVENT_COMMISSION_POSITION_TUNE_TARGET, TraceEventLevel,
    u32_arg,
};

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

/// Maximum register operations performed by one armed position-tune poll.
#[allow(dead_code)]
pub(super) const POSITION_TUNE_MAX_REGISTER_OPS_PER_ARM: usize = 5;

/// Position mode with `MODE_PID_TYPE` set, matching production closed-loop entry.
const OUTER_POSITION_MODE: u32 = 0x8000_0003;

async fn command_position_step_target<B: CommissioningBackend>(
    backend: &mut B,
    safety: &mut OuterSafetyLimits,
    target: u32,
) -> Result<(), CommissionError> {
    if safety.primitive_start_us.is_some() {
        return Err(CommissionError::InvalidSchedule);
    }
    backend
        .write_register(foci_tmc4671::registers::PID_POSITION_TARGET, target)
        .await
        .map_err(|_| CommissionError::Spi)?;
    safety.begin_primitive(backend.elapsed_us())
}

async fn command_position_baseline_target<B: CommissioningBackend>(
    backend: &mut B,
    safety: &mut OuterSafetyLimits,
    target: u32,
) -> Result<(), CommissionError> {
    backend
        .write_register(foci_tmc4671::registers::PID_POSITION_TARGET, target)
        .await
        .map_err(|_| CommissionError::Spi)?;
    safety.end_primitive();
    Ok(())
}

fn record_position_tune_summary<B: CommissioningBackend>(
    backend: &mut B,
    trace_summary_seqs: &mut TraceSummarySeqs,
    data: &PositionTuneData,
    result_status: u8,
    level: TraceEventLevel,
) {
    let summary_seq = trace_summary_seqs.next_position_tune();
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_POSITION_TUNE_RESULT,
        summary_seq,
        i32::from(result_status),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_POSITION_TUNE_TARGET,
        summary_seq,
        u32_arg(data.step_target_counts),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_POSITION_TUNE_METRICS,
        summary_seq,
        trace_pack_u16_pair(u32::from(data.stable_p), u32::from(data.current_p)),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_POSITION_TUNE_LIMITS,
        summary_seq,
        trace_pack_u16_pair(
            u32::from(data.saturation_count),
            u32::from(data.oscillation_at_minimum as u8),
        ),
    );
}

/// Advance position-tune sub-state machine.
#[allow(dead_code)]
pub(crate) async fn step_position_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    mode: OperatingMode,
    backend: &mut B,
) -> StepResult {
    let mut trace_summary_seqs = TraceSummarySeqs::new();
    step_position_tune_with_trace(phase, buf, mode, &mut trace_summary_seqs, backend).await
}

pub(crate) async fn step_position_tune_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    mode: OperatingMode,
    trace_summary_seqs: &mut TraceSummarySeqs,
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
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            data.step_start_counts = Some(
                safety
                    .encoder_domain
                    .expect("outer encoder domain configured before position tuning")
                    .count(encoder_raw),
            );
            let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(v) => v as i32,
                Err(_) => {
                    return fail_outer(phase, CommissionError::Spi);
                }
            };
            data.step_start_tmc = position_raw;

            let target = position_raw.wrapping_add(data.step_target_tmc as i32);
            if let Err(error) = command_position_step_target(backend, safety, target as u32).await {
                return fail_outer(phase, error);
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

            let now = backend.elapsed_us();
            let Some(next_sample_us) = now.checked_add(POSITION_TUNE_SAMPLE_PERIOD_US) else {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            };
            *state = PositionTuneState::StepCapture { next_sample_us };
            StepResult::At(next_sample_us)
        }

        PositionTuneState::StepCapture { next_sample_us } => {
            let now = backend.elapsed_us();
            if now < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }

            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let encoder_domain = safety
                .encoder_domain
                .expect("outer encoder domain configured before position capture");
            let encoder_count = encoder_domain.count(encoder_raw);
            let delta = match encoder_domain.delta(
                data.step_start_counts
                    .expect("position step baseline configured"),
                encoder_count,
            ) {
                Ok(delta) => delta,
                Err(_) => {
                    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return fail_outer(phase, CommissionError::EncoderFault);
                }
            };
            buf.push(delta.absolute_counts());

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
                if let Err(error) =
                    command_position_baseline_target(backend, safety, data.step_start_tmc as u32)
                        .await
                {
                    return fail_outer(phase, error);
                }

                let Some(deadline) = backend.elapsed_us().checked_add(POSITION_TUNE_SETTLE_US)
                else {
                    return fail_outer(phase, CommissionError::InvalidSchedule);
                };
                *state = PositionTuneState::Settle {
                    deadline_us: deadline,
                };
                StepResult::At(deadline)
            } else {
                let next_deadline_us = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    POSITION_TUNE_SAMPLE_PERIOD_US,
                    now,
                ) {
                    Ok(deadline_us) => deadline_us,
                    Err(error) => return fail_outer(phase, error),
                };
                *next_sample_us = next_deadline_us;
                StepResult::At(next_deadline_us)
            }
        }

        PositionTuneState::Settle { deadline_us } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            *state = PositionTuneState::Analyze;
            StepResult::ReadyNow
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
                StepResult::ReadyNow
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
                    StepResult::ReadyNow
                } else {
                    record_position_tune_summary(
                        backend,
                        trace_summary_seqs,
                        data,
                        0,
                        TraceEventLevel::Warn,
                    );
                    let new_p = (data.current_p as u32 * POSITION_TUNE_P_MUL_NUM
                        / POSITION_TUNE_P_MUL_DEN)
                        .min(max_p as u32) as u16;
                    if new_p <= data.current_p {
                        defmt::info!("pos tune: P cap reached at {}", data.stable_p);
                        *state = PositionTuneState::Finalize;
                        StepResult::ReadyNow
                    } else {
                        data.current_p = new_p;

                        let pos_reg = (data.current_p as u32) << 16;
                        if backend
                            .write_register(PID_POSITION_P_POSITION_I, pos_reg)
                            .await
                            .is_err()
                        {
                            return fail_outer(phase, CommissionError::Spi);
                        }

                        let encoder_raw =
                            match backend.try_read_mechanical_encoder_position_counts().await {
                                Ok(value) => value,
                                Err(_) => return fail_outer(phase, CommissionError::Spi),
                            };
                        data.step_start_counts = Some(
                            safety
                                .encoder_domain
                                .expect("outer encoder domain configured before position tuning")
                                .count(encoder_raw),
                        );
                        let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
                            Ok(v) => v as i32,
                            Err(_) => {
                                return fail_outer(phase, CommissionError::Spi);
                            }
                        };
                        data.step_start_tmc = position_raw;
                        let target = position_raw.wrapping_add(data.step_target_tmc as i32);
                        if let Err(error) =
                            command_position_step_target(backend, safety, target as u32).await
                        {
                            return fail_outer(phase, error);
                        }

                        buf.reset(POSITION_TUNE_SAMPLE_PERIOD_US as u16);
                        data.saturation_count = 0;
                        let now = backend.elapsed_us();
                        let Some(next_sample_us) = now.checked_add(POSITION_TUNE_SAMPLE_PERIOD_US)
                        else {
                            return fail_outer(phase, CommissionError::InvalidSchedule);
                        };
                        *state = PositionTuneState::StepCapture { next_sample_us };
                        StepResult::At(next_sample_us)
                    }
                }
            }
        }

        PositionTuneState::Finalize => {
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, 0)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            if data.oscillation_at_minimum {
                record_position_tune_summary(
                    backend,
                    trace_summary_seqs,
                    data,
                    CommissionError::PositionTuneFailed.status_code(),
                    TraceEventLevel::Error,
                );
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

            record_position_tune_summary(
                backend,
                trace_summary_seqs,
                data,
                0,
                TraceEventLevel::Info,
            );
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::VelocityGains;
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::outer::OuterDiagnostics;

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

    fn position_capture_phase(start_counts: u32) -> OuterPhase {
        let mut safety = crate::commissioning::outer::OuterSafetyLimits::new(
            0,
            crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
        );
        safety.configure_encoder_resolution(4_000).unwrap();
        let domain = safety.encoder_domain.unwrap();
        OuterPhase::PositionTune {
            state: PositionTuneState::StepCapture {
                next_sample_us: 1_000,
            },
            data: PositionTuneData {
                current_p: 128,
                stable_p: 0,
                step_target_counts: 100,
                step_target_tmc: 1_638,
                step_start_counts: Some(domain.count(start_counts)),
                step_start_tmc: 0,
                oscillation_at_minimum: false,
                velocity_gains: VelocityGains {
                    velocity_p: 1,
                    velocity_i: 0,
                    velocity_limit: 1_000,
                    effective_bandwidth_hz: 50,
                    lambda_vel_ms: 20,
                },
                velocity_chatter: false,
                current_ringing: false,
                current_bandwidth_hz: 100,
                j_eff: 1,
                b_eff: 1,
                torque_limit: 0,
                saturation_count: 0,
                diagnostics: zero_diagnostics(),
            },
            safety,
        }
    }

    #[test]
    fn each_position_rung_uses_a_fresh_primitive_timer() {
        pollster::block_on(async {
            let mut safety = crate::commissioning::outer::OuterSafetyLimits::new(
                100,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 100;

            assert!(
                command_position_step_target(&mut backend, &mut safety, 1_000)
                    .await
                    .is_ok()
            );
            assert_eq!(safety.primitive_start_us, Some(100));

            backend.elapsed_us_value = 200;
            assert!(
                command_position_baseline_target(&mut backend, &mut safety, 0)
                    .await
                    .is_ok()
            );
            assert_eq!(safety.primitive_start_us, None);

            backend.elapsed_us_value = 300;
            assert!(
                command_position_step_target(&mut backend, &mut safety, 2_000)
                    .await
                    .is_ok()
            );
            assert_eq!(safety.primitive_start_us, Some(300));
        });
    }

    #[test]
    fn position_setup_schedules_capture_within_the_register_bound() {
        pollster::block_on(async {
            let mut phase = position_capture_phase(0);
            if let OuterPhase::PositionTune { state, .. } = &mut phase {
                *state = PositionTuneState::Setup;
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;

            assert!(matches!(result, StepResult::At(1_000)));
            assert_eq!(
                backend.spi_log.len(),
                POSITION_TUNE_MAX_REGISTER_OPS_PER_ARM
            );
            assert!(matches!(
                phase,
                OuterPhase::PositionTune {
                    state: PositionTuneState::StepCapture {
                        next_sample_us: 1_000
                    },
                    safety: OuterSafetyLimits {
                        primitive_start_us: Some(0),
                        ..
                    },
                    ..
                }
            ));
        });
    }

    #[test]
    fn position_capture_waits_and_preserves_the_sample_grid() {
        pollster::block_on(async {
            let mut phase = position_capture_phase(0);
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 999;

            let waiting =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;
            assert!(matches!(waiting, StepResult::At(1_000)));
            assert!(backend.spi_log.is_empty());

            backend.elapsed_us_value = 1_600;
            backend.mechanical_encoder_position_counts_value = 4;
            let sampled =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;
            assert!(matches!(sampled, StepResult::At(2_000)));
            assert!(matches!(
                phase,
                OuterPhase::PositionTune {
                    state: PositionTuneState::StepCapture {
                        next_sample_us: 2_000
                    },
                    ..
                }
            ));
        });
    }

    #[test]
    fn position_settle_uses_a_one_shot_deadline() {
        pollster::block_on(async {
            let mut phase = position_capture_phase(0);
            if let OuterPhase::PositionTune { state, .. } = &mut phase {
                *state = PositionTuneState::Settle { deadline_us: 1_000 };
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 999;

            let waiting =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;
            assert!(matches!(waiting, StepResult::At(1_000)));

            backend.elapsed_us_value = 1_000;
            let elapsed =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;
            assert!(matches!(elapsed, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                OuterPhase::PositionTune {
                    state: PositionTuneState::Analyze,
                    ..
                }
            ));
        });
    }

    #[test]
    fn next_position_rung_arms_its_exact_capture_deadline() {
        pollster::block_on(async {
            let mut phase = position_capture_phase(0);
            if let OuterPhase::PositionTune { state, .. } = &mut phase {
                *state = PositionTuneState::Analyze;
            }
            let mut buf = TransientBuffer::new();
            for _ in 0..crate::autotune::TRANSIENT_LEN {
                assert!(buf.push(100));
            }
            let mut backend = MockBackend::new();

            let result =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;

            assert!(matches!(result, StepResult::At(1_000)));
            assert!(matches!(
                phase,
                OuterPhase::PositionTune {
                    state: PositionTuneState::StepCapture {
                        next_sample_us: 1_000
                    },
                    ..
                }
            ));
        });
    }

    #[test]
    fn position_step_capture_accepts_origin_crossing() {
        pollster::block_on(async {
            let mut phase = position_capture_phase(3_998);
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let result =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;

            assert!(matches!(result, StepResult::At(2_000)));
            assert_eq!(buf.samples(), &[4]);
        });
    }

    #[test]
    fn position_step_half_turn_stops_without_appending_or_accepting_state() {
        pollster::block_on(async {
            let mut phase = position_capture_phase(0);
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2_000;

            let result =
                step_position_tune(&mut phase, &mut buf, OperatingMode::Nominal, &mut backend)
                    .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::EncoderFault)
            ));
            assert!(matches!(
                phase,
                OuterPhase::Failed(CommissionError::EncoderFault)
            ));
            assert_eq!(buf.len(), 0);
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                crate::commission_dispatch::test_support::SpiOp::Write(addr, 0)
                    if *addr == foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr()
            )));
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                crate::commission_dispatch::test_support::SpiOp::Write(addr, 0)
                    if *addr == foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr()
            )));
        });
    }

    #[test]
    fn position_tune_summary_packs_exact_fragments() {
        let mut backend = MockBackend::new();
        let mut seqs = TraceSummarySeqs::new();
        let data = PositionTuneData {
            current_p: 256,
            stable_p: 128,
            step_target_counts: 400,
            step_target_tmc: 800,
            step_start_counts: None,
            step_start_tmc: 0,
            oscillation_at_minimum: true,
            velocity_gains: VelocityGains {
                velocity_p: 512,
                velocity_i: 0,
                velocity_limit: 10_000,
                effective_bandwidth_hz: 50,
                lambda_vel_ms: 20,
            },
            velocity_chatter: false,
            current_ringing: false,
            current_bandwidth_hz: 100,
            j_eff: 1,
            b_eff: 2,
            torque_limit: 1000,
            saturation_count: 3,
            diagnostics: zero_diagnostics(),
        };

        record_position_tune_summary(
            &mut backend,
            &mut seqs,
            &data,
            CommissionError::PositionTuneFailed.status_code(),
            TraceEventLevel::Error,
        );

        assert_eq!(
            backend.trace_events[0].event_id,
            EVENT_COMMISSION_POSITION_TUNE_RESULT
        );
        assert_eq!(
            backend.trace_events[0].payload,
            [
                0,
                i32::from(CommissionError::PositionTuneFailed.status_code())
            ]
        );
        assert_eq!(
            backend.trace_events[1].event_id,
            EVENT_COMMISSION_POSITION_TUNE_TARGET
        );
        assert_eq!(backend.trace_events[1].payload, [0, u32_arg(400)]);
        assert_eq!(
            backend.trace_events[2].event_id,
            EVENT_COMMISSION_POSITION_TUNE_METRICS
        );
        assert_eq!(
            backend.trace_events[2].payload,
            [0, trace_pack_u16_pair(128, 256)]
        );
        assert_eq!(
            backend.trace_events[3].event_id,
            EVENT_COMMISSION_POSITION_TUNE_LIMITS
        );
        assert_eq!(
            backend.trace_events[3].payload,
            [0, trace_pack_u16_pair(3, 1)]
        );
    }
}
