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

            // Initialize safety limits with real values.
            safety.prev_encoder_count = start_position;
            safety.torque_cap = current_limit as u16;

            // Initialize data with real values.
            data.start_position = start_position;
            data.torque_step = torque_step;

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

            // Zero torque target before starting capture.
            let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

            // Prepare transient buffer for forward capture.
            buf.reset(response::MECHANICAL_ID_SAMPLE_PERIOD_US as u16);

            // Read encoder PPR for wrap-aware delta computation.
            let ppr_raw = backend.read_register(ABN_DECODER_PPR).await.unwrap_or(4000);
            let counts_per_rev = (ppr_raw as i32).max(4);
            safety.counts_per_rev = counts_per_rev;

            // Re-read encoder after mode switch to prevent a false
            // velocity spike in the safety check. The transition from
            // position mode to torque mode can cause the motor to
            // drift, producing a large encoder delta on the next tick.
            let post_switch_pos = backend.read_encoder_count().await as i32;
            safety.prev_encoder_count = post_switch_pos;
            safety.unwrapped_position = 0;
            data.start_position = post_switch_pos;

            defmt::info!(
                "mechanical-id: attempt={}, start pos={}, torque_step={}, current_limit={}",
                data.attempt,
                post_switch_pos,
                torque_step,
                current_limit
            );

            *state = MechanicalIdState::ForwardStart;
            StepResult::PhaseChanged(PhaseId::MechanicalId)
        }

        MechanicalIdState::ForwardStart => {
            // Apply positive torque step.
            let torque = data.torque_step as u32;
            // PID_TORQUE_FLUX_TARGET: torque in [31:16], flux=0 in [15:0].
            let target = torque << 16;
            if backend
                .write_register(PID_TORQUE_FLUX_TARGET, target)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            let next_sample_us = backend
                .elapsed_us()
                .wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            *state = MechanicalIdState::ForwardCapture { next_sample_us };
            StepResult::Continue
        }

        MechanicalIdState::ForwardCapture { next_sample_us } => {
            // Check if it's time for the next sample.
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                // Not yet time for next sample.
                return StepResult::Continue;
            }

            // Sample encoder position.
            let encoder_raw = backend.read_encoder_count().await;
            buf.push(encoder_raw);

            if buf.is_full() {
                // Forward capture complete — zero torque.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                // Extract forward metrics from buffer before reset.
                let (initial_accel, peak_velocity, delay_samples, travel) =
                    response::extract_mechanical_id_metrics(
                        buf,
                        data.start_position,
                        safety.counts_per_rev,
                    );
                data.forward_initial_accel = initial_accel;
                data.forward_peak_velocity = peak_velocity;
                data.forward_delay_samples = delay_samples;
                data.forward_travel = travel;

                defmt::info!(
                    "mechanical-id fwd: accel={}, peak_vel={}, delay={}, travel={}",
                    initial_accel,
                    peak_velocity,
                    delay_samples,
                    travel
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

            let next_sample_us = backend
                .elapsed_us()
                .wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            *state = MechanicalIdState::ReverseCapture { next_sample_us };
            StepResult::Continue
        }

        MechanicalIdState::ReverseCapture { next_sample_us } => {
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let encoder_raw = backend.read_encoder_count().await;
            buf.push(encoder_raw);

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
                let (initial_accel, peak_velocity, delay_samples, travel) =
                    response::extract_mechanical_id_metrics(
                        buf,
                        reverse_start,
                        safety.counts_per_rev,
                    );
                // Store as absolute values for symmetry comparison.
                data.reverse_initial_accel = initial_accel.unsigned_abs() as i32;
                data.reverse_peak_velocity = peak_velocity.unsigned_abs() as i32;
                data.reverse_delay_samples = delay_samples;
                data.reverse_travel = travel;

                defmt::info!(
                    "mechanical-id rev: accel={}, peak_vel={}, delay={}, travel={}",
                    data.reverse_initial_accel,
                    data.reverse_peak_velocity,
                    delay_samples,
                    travel
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

            // Forward metrics (already absolute for accel).
            let fwd_accel = data.forward_initial_accel.unsigned_abs();
            let rev_accel = data.reverse_initial_accel as u32;

            let fwd_peak = data.forward_peak_velocity.unsigned_abs();
            let rev_peak = data.reverse_peak_velocity as u32;
            let peak_avg = (fwd_peak + rev_peak) / 2;

            // Gate: both accelerations must be positive and non-zero.
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
            let j_eff_fwd = (torque * 1000) / fwd_accel;
            let j_eff_rev = (torque * 1000) / rev_accel;

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

            // B_eff = torque / steady_state_velocity.
            // Scale to match J_eff units.
            let b_eff = if peak_avg > 0 {
                (torque * 1000) / peak_avg
            } else {
                // No steady-state velocity reached — pure inertial load.
                // Use a small minimum to avoid division by zero in tau.
                1
            };

            // tau_mech = J_eff / B_eff in milliseconds.
            let tau_mech_ms = if b_eff > 0 { j_eff / b_eff } else { 1000 };

            // theta_mech from delay samples (time from torque step to
            // first measurable velocity change).
            let avg_delay_samples = (data.forward_delay_samples + data.reverse_delay_samples) / 2;
            let theta_mech_us =
                (avg_delay_samples as u64 * response::MECHANICAL_ID_SAMPLE_PERIOD_US) as u32;

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

            let diagnostics = OuterDiagnostics::from_mechanical_id(data);
            if response::mechanical_id_low_confidence(
                fwd_accel,
                rev_accel,
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
