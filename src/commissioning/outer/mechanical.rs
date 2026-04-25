//! Phase 8: Mechanical identification.
//!
//! Bidirectional torque-step procedure that estimates effective inertia,
//! damping, mechanical time constant, and transport delay from encoder
//! response. Forward and reverse captures are compared for symmetry; if
//! the captures fail the acceptance gates the phase retries with a
//! larger torque step (up to `MECHANICAL_ID_MAX_ATTEMPTS`) before
//! falling back to the response-ramp velocity tuning.

use super::{MechanicalIdState, OuterDiagnostics, OuterPhase};
use crate::autotune::{CommissionError, MechanicalModel, PhaseId, TransientBuffer};
use crate::commissioning::outer::{fail_outer, response};
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Settling time after removing torque (microseconds).
/// Allow the motor to decelerate and stop before reverse capture.
const MECHANICAL_ID_SETTLE_US: u64 = 500_000;

/// Phase-local settle tuning for the position->torque mode switch.
///
/// These values are still firmware constants today, but grouping them makes
/// it straightforward to promote them into a future board/profile tuning
/// surface without touching the state-machine logic.
struct MechanicalIdModeSwitchSettlePolicy {
    grace_us: u64,
    timeout_us: u64,
    stable_ticks: u8,
}

const MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY: MechanicalIdModeSwitchSettlePolicy =
    MechanicalIdModeSwitchSettlePolicy {
        // Ignore the immediate position->torque transient before sampling settle velocity.
        grace_us: 10_000,
        // Fail cleanly if the unloaded axis never comes to rest after the mode switch.
        timeout_us: 100_000,
        // Require a few consecutive quiet samples before arming MechanicalId safety.
        stable_ticks: 3,
    };

/// Forward/reverse agreement threshold (30% = 30/100).
/// If `|J_fwd - J_rev| / avg > 30%`, mechanical ID fails (or retries).
const MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT: u32 = 30;

/// Operational PID output voltage limit for outer commissioning.
///
/// Matches closed-loop entry so torque, velocity, and position tests are
/// not constrained by the conservative safe-state ceiling.
const OUTER_PIDOUT_LIMIT: u32 = 16000;

/// Torque mode with `MODE_PID_TYPE` set for the TMC4671 advanced PI structure.
///
/// Current-loop SIMC writes the advanced flux/torque PI registers, so
/// validation and mechanical ID must exercise that same controller path.
const CURRENT_TORQUE_MODE: u32 = 0x8000_0001;

/// Advance mechanical identification sub-state.
///
/// Bidirectional torque step procedure:
/// 1. Switch to torque mode with tuned current-loop gains active.
/// 2. Record starting position.
/// 3. Apply torque step, sample encoder at 1 kHz into the transient buffer.
/// 4. Return to zero torque, wait for settling.
/// 5. Repeat in opposite direction.
/// 6. Estimate `J_eff`, `B_eff`, `tau_mech`, `theta_mech`.
/// 7. Validate gates; on failure either retry with stronger torque or
///    fall back to response-ramp velocity tuning.
pub(crate) async fn step_mechanical_id<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (state, data, safety) = match phase {
        OuterPhase::MechanicalId {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::Continue,
    };

    match state {
        MechanicalIdState::Setup => {
            // Read current encoder position as the reference point.
            let encoder_raw = backend.read_encoder_count().await;
            let start_position = encoder_raw as i32;

            // Compute torque step from configured run_current.
            let current_limit = backend.current_limit() & 0xFFFF;
            let torque_step = if current_limit > 0 {
                response::mechanical_id_torque_step(current_limit, data.attempt)
            } else {
                return fail_outer(phase, CommissionError::NoCurrent);
            };

            // Initialize data with real values.
            data.start_position = start_position;
            data.torque_step = torque_step;

            // Zero torque before the position->torque mode switch so any stale
            // target from the prior closed-loop hold cannot bleed into setup.
            if backend
                .write_register(PID_TORQUE_FLUX_TARGET, 0)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            // Ensure we are in torque mode with the same advanced PI
            // controller structure used by the tuned current loop.
            if backend
                .write_register(PIDOUT_UQ_UD_LIMITS, OUTER_PIDOUT_LIMIT)
                .await
                .is_err()
                || backend.write_register(PWM_SV_CHOP, 7).await.is_err()
                || backend.write_register(PHI_E_SELECTION, 3).await.is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, CURRENT_TORQUE_MODE)
                    .await
                    .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            let phi_ok = backend
                .read_register(PHI_E_SELECTION)
                .await
                .is_ok_and(|v| v == 3);
            if !phi_ok {
                return fail_outer(phase, CommissionError::EncoderNotAligned);
            }

            backend.set_enable_pin(true);

            // Prepare transient buffer for forward capture.
            buf.reset(response::MECHANICAL_ID_SAMPLE_PERIOD_US as u16);

            // Read encoder PPR for wrap-aware delta computation.
            let ppr_raw = backend.read_register(ABN_DECODER_PPR).await.unwrap_or(4000);
            let counts_per_rev = (ppr_raw as i32).max(4);
            safety.counts_per_rev = counts_per_rev;

            // Seed the safety baseline from the post-switch position, but keep
            // safety disarmed until a brief zero-torque settle completes.
            let post_switch_pos = backend.read_encoder_count().await as i32;
            safety.prev_encoder_count = post_switch_pos;
            safety.unwrapped_position = 0;
            data.start_position = post_switch_pos;
            safety.torque_cap = 0;

            let now = backend.elapsed_us();
            safety.last_sample_us = now;
            *state = MechanicalIdState::PostSwitchSettle {
                earliest_sample_us: now + MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY.grace_us,
                timeout_deadline_us: now + MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY.timeout_us,
                stable_ticks: 0,
            };
            StepResult::PhaseChanged(PhaseId::MechanicalId)
        }

        MechanicalIdState::PostSwitchSettle {
            earliest_sample_us,
            timeout_deadline_us,
            stable_ticks,
        } => {
            let now = backend.elapsed_us();
            if now < *earliest_sample_us {
                return StepResult::Continue;
            }

            let settled_pos = backend.read_encoder_count().await as i32;
            let delta_counts = response::encoder_delta(
                settled_pos,
                safety.prev_encoder_count,
                safety.counts_per_rev,
            );
            let dt_us = if safety.last_sample_us == 0 {
                0
            } else {
                now.wrapping_sub(safety.last_sample_us)
            };
            let velocity_cpms = response::counts_per_ms(delta_counts, dt_us);
            safety.prev_encoder_count = settled_pos;
            safety.last_sample_us = now;

            if velocity_cpms.unsigned_abs() as i32
                <= response::MECHANICAL_ID_SETTLE_VELOCITY_THRESHOLD
            {
                *stable_ticks = stable_ticks.saturating_add(1);
            } else {
                *stable_ticks = 0;
            }

            if *stable_ticks >= MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY.stable_ticks {
                let current_limit = backend.current_limit() & 0xFFFF;
                safety.prev_encoder_count = settled_pos;
                safety.unwrapped_position = 0;
                safety.torque_cap = current_limit as u16;
                safety.last_sample_us = backend.elapsed_us();
                data.start_position = settled_pos;

                // Apply the first positive torque step immediately so the next
                // outer tick does not arm safety for an idle pre-excitation gap.
                let torque = data.torque_step as u32;
                let target = torque << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return fail_outer(phase, CommissionError::Spi);
                }

                let capture_start_us = backend.elapsed_us();
                data.reset_sample_timestamps();
                let next_sample_us =
                    capture_start_us.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);

                defmt::info!(
                    "mechanical-id: attempt={}, start pos={}, torque_step={}, current_limit={}",
                    data.attempt,
                    settled_pos,
                    data.torque_step,
                    current_limit
                );

                *state = MechanicalIdState::ForwardCapture {
                    next_sample_us,
                    capture_start_us,
                };
                return StepResult::Continue;
            }

            if now >= *timeout_deadline_us {
                defmt::warn!(
                    "mechanical-id: post-switch settle timed out at velocity={} (delta={} dt_us={})",
                    velocity_cpms.unsigned_abs(),
                    delta_counts,
                    dt_us,
                );
                return fail_outer(phase, CommissionError::SafetyEnvelopeViolation);
            }

            StepResult::Continue
        }

        MechanicalIdState::ForwardCapture {
            next_sample_us,
            capture_start_us,
        } => {
            // Check if it's time for the next sample.
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                // Not yet time for next sample.
                return StepResult::Continue;
            }

            // Sample encoder position.
            let encoder_raw = backend.read_encoder_count().await;
            buf.push(encoder_raw);
            data.push_sample_timestamp(now);

            if buf.is_full() {
                // Forward capture complete — zero torque.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                // Extract forward metrics from buffer before reset.
                let metrics = response::extract_mechanical_id_metrics(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    data.start_position,
                    safety.counts_per_rev,
                );
                data.fwd_slope = metrics.accel;
                data.forward_peak_velocity = metrics.peak_velocity;
                data.forward_delay_us = metrics.delay_us;
                data.forward_travel = metrics.travel;
                data.fwd_response = response::extract_mechanical_response_model(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    data.start_position,
                    safety.counts_per_rev,
                    data.torque_step as u32,
                );

                defmt::info!(
                    "mechanical-id fwd: accel={} peak_vel={} delay_us={} travel={} \
                     resid_counts={} rise_mcpms={} samples={} steady={} tau={}ms response_resid={}",
                    data.fwd_slope.accel,
                    data.forward_peak_velocity,
                    data.forward_delay_us,
                    data.forward_travel,
                    data.fwd_slope.residual_counts,
                    data.fwd_slope.velocity_rise_mcpms,
                    data.fwd_slope.fit_sample_count,
                    data.fwd_response.steady_velocity_cpms,
                    data.fwd_response.tau_mech_ms,
                    data.fwd_response.residual_mcpms,
                );

                let deadline = backend.elapsed_us().wrapping_add(MECHANICAL_ID_SETTLE_US);
                *state = MechanicalIdState::ForwardSettle {
                    deadline_us: deadline,
                };
            } else {
                *next_sample_us = now.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        MechanicalIdState::ForwardSettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                // Settled — prepare for reverse capture.
                buf.reset(response::MECHANICAL_ID_SAMPLE_PERIOD_US as u16);
                *state = MechanicalIdState::ReverseStart;
            }
            StepResult::Continue
        }

        MechanicalIdState::ReverseStart => {
            // Apply negative torque step.
            // Torque is signed i16 in upper 16 bits.
            let neg_torque = (-(data.torque_step as i16)) as u16;
            let target = (neg_torque as u32) << 16;
            if backend
                .write_register(PID_TORQUE_FLUX_TARGET, target)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            let capture_start_us = backend.elapsed_us();
            data.reset_sample_timestamps();
            let next_sample_us =
                capture_start_us.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            *state = MechanicalIdState::ReverseCapture {
                next_sample_us,
                capture_start_us,
            };
            StepResult::Continue
        }

        MechanicalIdState::ReverseCapture {
            next_sample_us,
            capture_start_us,
        } => {
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let encoder_raw = backend.read_encoder_count().await;
            buf.push(encoder_raw);
            data.push_sample_timestamp(now);

            if buf.is_full() {
                // Reverse capture complete — zero torque.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                // For reverse capture, use the position at capture start
                // (first sample) as the reference, not the original start.
                let reverse_start = if !buf.is_empty() {
                    buf.samples()[0] as i32
                } else {
                    data.start_position
                };
                let metrics = response::extract_mechanical_id_metrics(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    reverse_start,
                    safety.counts_per_rev,
                );
                data.rev_slope = metrics.accel;
                data.reverse_peak_velocity = metrics.peak_velocity.unsigned_abs() as i32;
                data.reverse_delay_us = metrics.delay_us;
                data.reverse_travel = metrics.travel;
                data.rev_response = response::extract_mechanical_response_model(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    reverse_start,
                    safety.counts_per_rev,
                    data.torque_step as u32,
                );

                defmt::info!(
                    "mechanical-id rev: accel={} peak_vel={} delay_us={} travel={} \
                     resid_counts={} rise_mcpms={} samples={} steady={} tau={}ms response_resid={}",
                    data.rev_slope.accel,
                    data.reverse_peak_velocity,
                    data.reverse_delay_us,
                    data.reverse_travel,
                    data.rev_slope.residual_counts,
                    data.rev_slope.velocity_rise_mcpms,
                    data.rev_slope.fit_sample_count,
                    data.rev_response.steady_velocity_cpms,
                    data.rev_response.tau_mech_ms,
                    data.rev_response.residual_mcpms,
                );

                let deadline = backend.elapsed_us().wrapping_add(MECHANICAL_ID_SETTLE_US);
                *state = MechanicalIdState::ReverseSettle {
                    deadline_us: deadline,
                };
            } else {
                *next_sample_us = now.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        MechanicalIdState::ReverseSettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                *state = MechanicalIdState::Estimate;
            }
            StepResult::Continue
        }

        MechanicalIdState::Estimate => {
            // Ensure motor is stopped.
            let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
            let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

            *state = MechanicalIdState::Finalize;
            StepResult::Continue
        }

        MechanicalIdState::Finalize => {
            // Assemble and validate mechanical model.
            let torque = data.torque_step as u32;

            let fwd_peak = data.forward_peak_velocity.unsigned_abs();
            let rev_peak = data.reverse_peak_velocity as u32;
            let peak_avg = (fwd_peak + rev_peak) / 2;
            let diagnostics = OuterDiagnostics::from_mechanical_id(data);

            if data.fwd_response.is_high_confidence() && data.rev_response.is_high_confidence() {
                let j_avg = (data.fwd_response.j_eff + data.rev_response.j_eff) / 2;
                let j_diff = data.fwd_response.j_eff.abs_diff(data.rev_response.j_eff);
                let asymmetry_pct = if j_avg == 0 {
                    100
                } else {
                    (j_diff * 100) / j_avg
                };
                if asymmetry_pct > MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT {
                    if response::retry_mechanical_id(state, data, "asymmetric full response") {
                        return StepResult::Continue;
                    }
                    defmt::warn!(
                        "mechanical-id: full-response asymmetry {}% exceeds {}%, using response fallback",
                        asymmetry_pct,
                        MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT
                    );
                    let avg_delay_us =
                        (data.fwd_response.theta_mech_us + data.rev_response.theta_mech_us) / 2;
                    let (model, peak_velocity) =
                        response::response_mechanical_model_low_confidence(peak_avg, avg_delay_us);
                    *phase = response::response_velocity_phase(
                        model,
                        peak_velocity,
                        diagnostics,
                        *safety,
                    );
                    return StepResult::PhaseChanged(PhaseId::VelocityTune);
                } else {
                    let b_eff = (data.fwd_response.b_eff + data.rev_response.b_eff) / 2;
                    let tau_mech_ms =
                        (data.fwd_response.tau_mech_ms + data.rev_response.tau_mech_ms) / 2;
                    let theta_mech_us =
                        (data.fwd_response.theta_mech_us + data.rev_response.theta_mech_us) / 2;
                    let model = MechanicalModel {
                        j_eff: j_avg.max(1),
                        b_eff: b_eff.max(1),
                        tau_mech_ms: tau_mech_ms.max(1),
                        theta_mech_us,
                        t_f: 0,
                    };
                    defmt::info!(
                        "mechanical-id full-response: J={} B={} tau={}ms theta={}us steady={}/{}",
                        model.j_eff,
                        model.b_eff,
                        model.tau_mech_ms,
                        model.theta_mech_us,
                        data.fwd_response.steady_velocity_cpms,
                        data.rev_response.steady_velocity_cpms,
                    );
                    let carried_safety = match phase {
                        OuterPhase::MechanicalId { safety, .. } => *safety,
                        _ => unreachable!(),
                    };
                    *phase = OuterPhase::VelocityTune {
                        model,
                        peak_velocity: peak_avg.max(1),
                        diagnostics,
                        safety: carried_safety,
                    };
                    return StepResult::PhaseChanged(PhaseId::VelocityTune);
                }
            }

            // Confidence gate: if either slope is low-confidence, fall back via
            // the response-ramp fallback using a conservative J_eff. The
            // slope estimator's is_high_confidence() replaces the old
            // MECHANICAL_ID_MIN_RETRY_ACCEL_COUNTS_PER_MS2 magic-number gate.
            if !data.fwd_slope.is_high_confidence() || !data.rev_slope.is_high_confidence() {
                defmt::warn!(
                    "mechanical-id: low-confidence slope \
                     (fwd: rise={} samples={} resid={}, rev: rise={} samples={} resid={}) \
                     -> response fallback",
                    data.fwd_slope.velocity_rise_mcpms,
                    data.fwd_slope.fit_sample_count,
                    data.fwd_slope.residual_counts,
                    data.rev_slope.velocity_rise_mcpms,
                    data.rev_slope.fit_sample_count,
                    data.rev_slope.residual_counts,
                );
                let avg_delay_us = (data.forward_delay_us + data.reverse_delay_us) / 2;
                let (model, peak_velocity) =
                    response::response_mechanical_model_low_confidence(peak_avg, avg_delay_us);
                *phase = response::response_velocity_phase(
                    model,
                    peak_velocity,
                    OuterDiagnostics::from_mechanical_id(data),
                    *safety,
                );
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            let fwd_accel = data.fwd_slope.accel;
            let rev_accel = data.rev_slope.accel;

            // Gate: both accelerations must be non-zero.
            if fwd_accel == 0 || rev_accel == 0 {
                if response::retry_mechanical_id(state, data, "zero acceleration") {
                    return StepResult::Continue;
                }
                defmt::warn!(
                    "mechanical-id: zero acceleration after retries, using response velocity tuning"
                );
                let (model, peak_velocity) = response::response_mechanical_model(
                    data, torque, fwd_accel, rev_accel, fwd_peak, rev_peak,
                );
                *phase = response::response_velocity_phase(
                    model,
                    peak_velocity,
                    OuterDiagnostics::from_mechanical_id(data),
                    *safety,
                );
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            // J_eff = torque / initial_acceleration.
            // Scale: torque is in DAC units, accel is in counts/ms^2.
            // We multiply torque by 1000 to get better fixed-point resolution.
            let j_eff_fwd = (torque * 1000) / fwd_accel.unsigned_abs().max(1);
            let j_eff_rev = (torque * 1000) / rev_accel.unsigned_abs().max(1);

            // Symmetry gate: |J_fwd - J_rev| / avg <= 30%.
            let j_avg = (j_eff_fwd + j_eff_rev) / 2;
            if j_avg == 0 {
                defmt::warn!("mechanical-id: zero average inertia");
                return fail_outer(phase, CommissionError::MechanicalIdFailed);
            }
            let j_diff = j_eff_fwd.abs_diff(j_eff_rev);
            let asymmetry_pct = (j_diff * 100) / j_avg;
            if asymmetry_pct > MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT {
                if response::retry_mechanical_id(state, data, "asymmetric response") {
                    return StepResult::Continue;
                }
                defmt::warn!(
                    "mechanical-id: asymmetry {}% exceeds {}% after retries, using response velocity tuning",
                    asymmetry_pct,
                    MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT
                );
                let (model, peak_velocity) = response::response_mechanical_model(
                    data, torque, fwd_accel, rev_accel, fwd_peak, rev_peak,
                );
                *phase = response::response_velocity_phase(
                    model,
                    peak_velocity,
                    OuterDiagnostics::from_mechanical_id(data),
                    *safety,
                );
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            let j_eff = j_avg;

            // B_eff = torque / steady_state_velocity, scaled to match J_eff units.
            // No steady-state velocity reached -> pure inertial load; small minimum
            // (1) avoids division by zero in tau below.
            let b_eff = (torque * 1000).checked_div(peak_avg).unwrap_or(1);

            // tau_mech = J_eff / B_eff in milliseconds. Default to 1000 ms when
            // b_eff is zero (matches the historical fall-back).
            let tau_mech_ms = j_eff.checked_div(b_eff).unwrap_or(1000);

            // theta_mech from delay samples (time from torque step to
            // first measurable velocity change).
            let theta_mech_us = (data.forward_delay_us + data.reverse_delay_us) / 2;

            let model = MechanicalModel {
                j_eff,
                b_eff,
                tau_mech_ms,
                theta_mech_us,
                t_f: 0, // Static friction not measured in this phase.
            };

            defmt::info!(
                "mechanical-id: J={}, B={}, tau={}ms, theta={}us",
                model.j_eff,
                model.b_eff,
                model.tau_mech_ms,
                model.theta_mech_us
            );

            if response::mechanical_id_low_confidence(
                fwd_accel.unsigned_abs(),
                rev_accel.unsigned_abs(),
                fwd_peak,
                rev_peak,
                data.forward_travel,
                data.reverse_travel,
            ) {
                if response::retry_mechanical_id(state, data, "low-confidence response") {
                    return StepResult::Continue;
                }

                defmt::warn!("mechanical-id: low-confidence data, using response velocity tuning");
                *phase = response::response_velocity_phase(model, peak_avg, diagnostics, *safety);
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            // Carry safety limits forward to VelocityTune.
            let carried_safety = match phase {
                OuterPhase::MechanicalId { safety, .. } => *safety,
                _ => unreachable!(),
            };

            *phase = OuterPhase::VelocityTune {
                model,
                peak_velocity: peak_avg,
                diagnostics,
                safety: carried_safety,
            };
            StepResult::PhaseChanged(PhaseId::VelocityTune)
        }
    }
}
