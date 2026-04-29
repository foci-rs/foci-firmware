//! Phases 6 and 7: Current-loop SIMC gain synthesis and step-response
//! validation.
//!
//! `step_current_tune` reads the electrical model from Phase 5 and applies
//! the SIMC tuning rule to derive flux/torque PI gains plus a step
//! amplitude for validation. `step_current_validate` then writes those
//! gains, captures bidirectional torque step responses, classifies them,
//! retries once if marginal, and falls back to conservative defaults on
//! failure.

use crate::autotune::{
    CaptureDirection, ClosedLoopEntryState, CommissionError, CommissionProfile, CommissionStatus,
    CurrentGains, ElectricalModel, InnerSequence, PhaseId, TransientBuffer, ValidationResult,
};
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::types::{CommissioningBackend, InnerResult, StepResult};

pub(crate) use crate::autotune::CurrentValidation;

/// Current-loop validation sub-state machine (Phase 7).
#[derive(Debug, defmt::Format)]
pub(crate) enum CurrentValidateState {
    /// Write PI gains to TMC4671 registers.
    WriteGains,
    /// Switch to torque mode and apply current step.
    StepSetup { direction: CaptureDirection },
    /// Wait for transient to develop before capturing.
    StepSettle {
        direction: CaptureDirection,
        deadline_us: u64,
    },
    /// Capture step response into TransientBuffer at 160us intervals.
    StepCapture {
        direction: CaptureDirection,
        next_sample_us: u64,
    },
    /// Extract metrics and classify response.
    Classify,
    /// Restore safe state before finalizing.
    SafeState { deadline_us: u64 },
    /// Emit final InnerResult.
    Finalize,
}

/// Accumulated data for current-loop validation.
#[derive(Debug, Clone, defmt::Format)]
pub(crate) struct CurrentValidateData {
    /// The electrical model from Phase 5.
    pub(crate) model: ElectricalModel,
    /// Currently applied gains.
    pub(crate) gains: CurrentGains,
    /// Validation attempt (0 = first, 1 = retry).
    pub(crate) attempt: u8,
    /// Current step amplitude (absolute torque/flux current target counts).
    pub(crate) step_amplitude: u16,
    /// Forward capture metrics (filled after forward capture).
    pub(crate) forward_metrics: Option<CurrentValidation>,
    /// Reverse capture metrics (filled after reverse capture).
    pub(crate) reverse_metrics: Option<CurrentValidation>,
    /// Final commissioning status to report.
    pub(crate) final_status: CommissionStatus,
    /// PID-output saturation counter: samples where the current PID's
    /// voltage output is at or near `PIDOUT_UQ_UD_LIMITS`.
    pub(crate) saturation_count: u16,
    /// Configured PIDOUT voltage limit for saturation detection.
    pub(crate) pidout_limit: u32,
}

/// Step amplitude as a fraction of run_current (15% = 15/100).
const VALIDATION_STEP_PERCENT: u32 = 15;

/// Settling time after applying or removing current step (microseconds).
const VALIDATION_SETTLE_US: u64 = 50_000;

/// PID output voltage limit used during current-loop validation.
///
/// ADC calibration leaves `PIDOUT_UQ_UD_LIMITS` at 1024 as a conservative
/// safety cap, but the measured validation captures can hit that cap before
/// reaching the current target. 2000 matches the bounded encoder-alignment
/// limit and remains well below the operational closed-loop limit.
const CURRENT_VALIDATE_PIDOUT_LIMIT: u32 = 2000;

/// `MODE_RAMP_MODE_MOTION` value for advanced PI torque mode.
///
/// High bit selects the advanced controller structure used by the tuned
/// current loop; low bits select torque mode.
const CURRENT_TORQUE_MODE: u32 = 0x8000_0001;

/// Thin phase: read the electrical model, compute SIMC gains, and
/// transition immediately to validation.
pub(crate) async fn step_current_tune<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    profile: CommissionProfile,
    backend: &mut B,
) -> StepResult {
    let model = match phase {
        InnerPhase::CurrentTune { model } => *model,
        _ => return StepResult::Continue,
    };

    let config = backend.tmc_config();
    let gains = crate::autotune::simc_current_gains(&model, profile, config);

    defmt::info!(
        "simc: R={} L={} tau_e={}us theta_e={}us lambda={}us → P={} I={} bw={}Hz",
        model.r_int,
        model.l_int,
        model.tau_e_us,
        model.theta_e_us,
        gains.lambda_us,
        gains.flux_p,
        gains.flux_i,
        gains.effective_bandwidth_hz,
    );

    // Compute step amplitude: 15% of run_current.
    let current_limit = backend.current_limit() & 0xFFFF;
    let step_amplitude = if current_limit > 0 {
        (current_limit * VALIDATION_STEP_PERCENT / 100).max(1) as u16
    } else {
        // No current limit configured — cannot validate.
        return super::fail_inner(phase, CommissionError::CurrentValidationFailed);
    };

    *phase = InnerPhase::CurrentValidate {
        state: CurrentValidateState::WriteGains,
        data: CurrentValidateData {
            model,
            gains,
            attempt: 0,
            step_amplitude,
            forward_metrics: None,
            reverse_metrics: None,
            final_status: CommissionStatus::Accepted,
            saturation_count: 0,
            pidout_limit: {
                // Read PIDOUT_UQ_UD_LIMITS for saturation detection.
                backend
                    .read_register(foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS)
                    .await
                    .unwrap_or(0)
                    & 0xFFFF
            },
        },
    };
    StepResult::PhaseChanged(PhaseId::CurrentValidate)
}

/// Advance current-loop validation sub-state.
///
/// Writes SIMC gains, runs bidirectional step-response captures,
/// classifies the response, retries once if marginal, and falls
/// back to conservative defaults on failure.
pub(crate) async fn step_current_validate<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    buf: &mut TransientBuffer,
    _profile: CommissionProfile,
    sequence: InnerSequence,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (validate_state, data) = match phase {
        InnerPhase::CurrentValidate { state, data } => (state, data),
        _ => return StepResult::Continue,
    };

    match validate_state {
        CurrentValidateState::WriteGains => {
            // Write gains to TMC4671 PI registers.
            // PID_FLUX_P_FLUX_I: P in [31:16], I in [15:0]
            let flux_reg = ((data.gains.flux_p as u32) << 16) | data.gains.flux_i as u32;
            let torque_reg = ((data.gains.torque_p as u32) << 16) | data.gains.torque_i as u32;

            if backend
                .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                .await
                .is_err()
                || backend
                    .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                    .await
                    .is_err()
                || backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, CURRENT_VALIDATE_PIDOUT_LIMIT)
                    .await
                    .is_err()
                || crate::commissioning::helpers::clear_current_integrators(backend)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            data.pidout_limit = CURRENT_VALIDATE_PIDOUT_LIMIT;

            // Clear any previous metrics for this attempt.
            data.forward_metrics = None;
            data.reverse_metrics = None;
            data.saturation_count = 0;

            *validate_state = CurrentValidateState::StepSetup {
                direction: CaptureDirection::Forward,
            };
            StepResult::Continue
        }

        CurrentValidateState::StepSetup { direction } => {
            let dir = *direction;
            // Switch to torque mode using the same advanced PI
            // controller structure as the tuned current loop.
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, CURRENT_TORQUE_MODE)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            backend.set_enable_pin(true);

            if crate::commissioning::helpers::clear_current_integrators(backend)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Apply current step via PID_TORQUE_FLUX_TARGET.
            // Torque target in [31:16], flux target in [15:0].
            let torque_val = match dir {
                CaptureDirection::Forward => data.step_amplitude as u32,
                CaptureDirection::Reverse => (-(data.step_amplitude as i16) as u16) as u32,
            };
            let target = torque_val << 16;
            if backend
                .write_register(PID_TORQUE_FLUX_TARGET, target)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Reset buffer for this capture.
            buf.reset(super::electrical_identification::ELECTRICAL_SAMPLE_PERIOD_US as u16);

            // The first sample is captured one period after the target write.
            // `extract_step_metrics` reports durations, not absolute crossing
            // timestamps, so this origin is intentionally not encoded in the
            // transient buffer.
            let next = backend
                .elapsed_us()
                .wrapping_add(super::electrical_identification::ELECTRICAL_SAMPLE_PERIOD_US);
            *validate_state = CurrentValidateState::StepCapture {
                direction: dir,
                next_sample_us: next,
            };
            StepResult::Continue
        }

        CurrentValidateState::StepSettle {
            direction,
            deadline_us,
        } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }
            // Settling done. Continue based on direction.
            let dir = *direction;
            match dir {
                CaptureDirection::Forward => {
                    // Move to reverse capture.
                    *validate_state = CurrentValidateState::StepSetup {
                        direction: CaptureDirection::Reverse,
                    };
                }
                CaptureDirection::Reverse => {
                    // Both captures done. Classify.
                    *validate_state = CurrentValidateState::Classify;
                }
            }
            StepResult::Continue
        }

        CurrentValidateState::StepCapture {
            direction,
            next_sample_us,
        } => {
            let dir = *direction;
            // Capture samples at 160us intervals.
            loop {
                if buf.is_full()
                    || buf.len() >= super::electrical_identification::DELAY_SAMPLE_COUNT
                {
                    // Capture complete. Extract metrics.
                    // Saturation is counted from the PID output (voltage),
                    // not from measured current, so disable the helper's
                    // sample-level saturation check.
                    let target = data.step_amplitude as u32;
                    let metrics = crate::autotune::extract_step_metrics(buf, target, 0);

                    match dir {
                        CaptureDirection::Forward => {
                            data.forward_metrics = Some(metrics);
                        }
                        CaptureDirection::Reverse => {
                            data.reverse_metrics = Some(metrics);
                        }
                    }

                    // Return to zero target and clear accumulated current
                    // error so the next capture starts from a defined
                    // controller state.
                    if backend
                        .write_register(PID_TORQUE_FLUX_TARGET, 0)
                        .await
                        .is_err()
                        || crate::commissioning::helpers::clear_current_integrators(backend)
                            .await
                            .is_err()
                    {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                    let deadline = backend.elapsed_us().wrapping_add(VALIDATION_SETTLE_US);
                    *validate_state = CurrentValidateState::StepSettle {
                        direction: dir,
                        deadline_us: deadline,
                    };
                    return StepResult::Continue;
                }
                if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    break; // Not yet time, yield.
                }
                match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => {
                        // PID_TORQUE_ACTUAL is in [31:16], PID_FLUX_ACTUAL in [15:0].
                        // We stepped the torque axis, so read the upper half.
                        // Normalize by direction so both captures produce
                        // positive values when the motor responds correctly.
                        // Wrong-sign current clamps to zero.
                        let torque_actual = ((actual >> 16) & 0xFFFF) as i16;
                        let sample = crate::autotune::normalize_capture_sample(torque_actual, dir);
                        buf.push(sample);
                    }
                    Err(_) => {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                }
                // Detect current-PID output saturation: read the limited
                // FOC voltage output (FOC_UQ_UD_LIMITED) and count when
                // it's near the PIDOUT_UQ_UD_LIMITS voltage ceiling.
                if data.pidout_limit > 0 {
                    let sub = foci_tmc4671::registers::INTERIM_FOC_UQ_UD_LIMITED;
                    let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                    if let Ok(foc_limited) = backend.read_register(sub.data).await {
                        // FOC_UQ_UD_LIMITED packs UQ in [31:16], UD in [15:0].
                        // Check the torque-axis voltage (UQ) since we stepped torque.
                        let uq = ((foc_limited >> 16) & 0xFFFF) as i16;
                        let abs_uq = uq.unsigned_abs() as u32;
                        if abs_uq >= data.pidout_limit * 9 / 10 {
                            data.saturation_count += 1;
                        }
                    }
                }
                *next_sample_us = next_sample_us
                    .wrapping_add(super::electrical_identification::ELECTRICAL_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        CurrentValidateState::Classify => {
            let forward = match data.forward_metrics {
                Some(m) => m,
                None => {
                    return super::fail_inner(phase, CommissionError::CurrentValidationFailed);
                }
            };
            let reverse = match data.reverse_metrics {
                Some(m) => m,
                None => {
                    return super::fail_inner(phase, CommissionError::CurrentValidationFailed);
                }
            };

            // Compute symmetry from forward and reverse tail levels
            // (steady-state current magnitude).
            let symmetry =
                crate::autotune::compute_tail_symmetry_pct(forward.tail_level, reverse.tail_level);

            // Use worst-case tail_level (lower = further from target).
            let worst_tail = forward.tail_level.min(reverse.tail_level);
            let target = data.step_amplitude as u32;
            let target_error = crate::autotune::compute_target_error_pct(worst_tail, target);

            let combined = CurrentValidation {
                symmetry_pct: symmetry,
                // Use worst-case of forward and reverse for other metrics.
                overshoot_pct: forward.overshoot_pct.max(reverse.overshoot_pct),
                ringing_count: forward.ringing_count.max(reverse.ringing_count),
                settling_time_us: forward.settling_time_us.max(reverse.settling_time_us),
                // Use firmware-counted PID-output saturation, not the
                // helper's current-level saturation.
                saturation_samples: data.saturation_count,
                rise_time_us: forward.rise_time_us,
                tail_level: worst_tail,
                target_error_pct: target_error,
            };

            let total_samples = crate::autotune::TRANSIENT_LEN as u16 * 2;
            let settle_reference_us =
                crate::autotune::current_settling_reference_us(&data.gains, backend.tmc_config());
            let result = crate::autotune::classify_current_validation(
                &combined,
                settle_reference_us,
                total_samples,
            );

            defmt::info!(
                "current validate: overshoot={}‰ ringing={} settle={}us sym={}‰ target_err={}‰ sat={}/{} tau_e={}us settle_ref={}us → {:?} (attempt={})",
                combined.overshoot_pct,
                combined.ringing_count,
                combined.settling_time_us,
                combined.symmetry_pct,
                combined.target_error_pct,
                combined.saturation_samples,
                total_samples,
                data.model.tau_e_us,
                settle_reference_us,
                result,
                data.attempt,
            );

            match result {
                ValidationResult::Pass => {
                    // Gains accepted.
                    data.final_status = if data.attempt > 0 {
                        CommissionStatus::AcceptedWithWarnings
                    } else {
                        CommissionStatus::Accepted
                    };
                }
                retry_result
                    if crate::autotune::should_retry_current_validation(
                        retry_result,
                        data.attempt,
                    ) =>
                {
                    // Retry: keep P damping and adjust I based on the
                    // measured tail response.
                    let target = data.step_amplitude as u32;
                    let settle_band = (target * 5 / 100).max(1);
                    let retry_gains = crate::autotune::current_validation_retry_gains(
                        data.gains,
                        target,
                        settle_band,
                        forward.tail_level,
                        reverse.tail_level,
                        combined.overshoot_pct,
                    );
                    if crate::autotune::should_accept_current_validation_without_retry(
                        &combined,
                        settle_reference_us,
                        total_samples,
                        data.gains,
                        retry_gains,
                    ) {
                        defmt::info!(
                            "current validate accept: marginal symmetry only, retry unchanged"
                        );
                        data.final_status = CommissionStatus::Accepted;
                    } else {
                        data.attempt = 1;
                        defmt::info!(
                            "current validate retry: previous={:?} lambda={}us → P={} I={} tails={}/{}",
                            retry_result,
                            retry_gains.lambda_us,
                            retry_gains.flux_p,
                            retry_gains.flux_i,
                            forward.tail_level,
                            reverse.tail_level,
                        );
                        data.gains = retry_gains;
                        *validate_state = CurrentValidateState::WriteGains;
                        return StepResult::Continue;
                    }
                }
                ValidationResult::Marginal | ValidationResult::Fail => {
                    // Fall back to conservative defaults.
                    data.gains = CurrentGains {
                        flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
                        flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
                        torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
                        torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
                        effective_bandwidth_hz: 0,
                        lambda_us: 0,
                    };
                    data.final_status = CommissionStatus::AcceptedWithWarnings;

                    // Write the default gains to TMC4671 so outer
                    // commissioning uses the reported conservative gains.
                    let flux_reg = ((data.gains.flux_p as u32) << 16) | data.gains.flux_i as u32;
                    let torque_reg =
                        ((data.gains.torque_p as u32) << 16) | data.gains.torque_i as u32;
                    let _ = backend.write_register(PID_FLUX_P_FLUX_I, flux_reg).await;
                    let _ = backend
                        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                        .await;
                }
            }

            // Restore safe state.
            let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
            let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
            backend.set_enable_pin(false);

            let deadline = backend.elapsed_us().wrapping_add(VALIDATION_SETTLE_US);
            *validate_state = CurrentValidateState::SafeState {
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        CurrentValidateState::SafeState { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }
            *validate_state = CurrentValidateState::Finalize;
            StepResult::Continue
        }

        CurrentValidateState::Finalize => {
            // Compute max ringing count from forward/reverse metrics.
            let ringing_count = match (data.forward_metrics, data.reverse_metrics) {
                (Some(fwd), Some(rev)) => fwd.ringing_count.max(rev.ringing_count),
                (Some(fwd), None) => fwd.ringing_count,
                (None, Some(rev)) => rev.ringing_count,
                (None, None) => 0,
            };

            let result = InnerResult {
                status: data.final_status,
                r_mohm: data.model.r_mohm,
                l_uh: data.model.l_uh,
                flux_p: data.gains.flux_p,
                flux_i: data.gains.flux_i,
                torque_p: data.gains.torque_p,
                torque_i: data.gains.torque_i,
                lambda_us: data.gains.lambda_us,
                theta_e_us: data.model.theta_e_us,
                ringing_count,
                bandwidth_hz: data.gains.effective_bandwidth_hz,
            };

            match sequence {
                InnerSequence::Commission => {
                    // Transition to ClosedLoopEntry with computed
                    // current-loop gains and fallback outer gains.
                    let vel_limit = backend.operational_velocity_limit();
                    *phase = InnerPhase::ClosedLoopEntry {
                        state: ClosedLoopEntryState::WriteGains,
                        flux_p: result.flux_p,
                        flux_i: result.flux_i,
                        torque_p: result.torque_p,
                        torque_i: result.torque_i,
                        // Conservative fallback outer gains.
                        velocity_p: 1152, // 4.5 in Q8.8
                        velocity_i: 0,
                        position_p: 640, // 2.5 in Q8.8
                        position_i: 0,
                        velocity_limit: vel_limit,
                        velocity_filter_hz: 0,
                        torque_filter_hz: 0,
                        position_filter_hz: 0,
                        flux_filter_hz: 0,
                        inner_result: Some(result),
                    };
                    StepResult::PhaseChanged(PhaseId::ClosedLoopEntry)
                }
                InnerSequence::Calibrate | InnerSequence::Selftest => {
                    unreachable!("Calibrate/Selftest never reach CurrentValidate")
                }
            }
        }
    }
}
