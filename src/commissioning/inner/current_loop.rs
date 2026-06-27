//! Phases 6 and 7: Current-loop SIMC gain synthesis and step-response
//! validation.
//!
//! `step_current_tune` reads the electrical model from Phase 5 and applies
//! the SIMC tuning rule to derive flux/torque PI gains plus a step
//! amplitude for validation. `step_current_validate` then writes those
//! gains, samples flux and torque current responses at short fixed delays,
//! classifies the per-axis evidence, and falls back to conservative defaults
//! on validation failure.

use crate::autotune::{
    CaptureDirection, ClosedLoopEntryState, CommissionError, CommissionProfile, CommissionStatus,
    CurrentAxisGate, CurrentAxisGateResult, CurrentAxisSample, CurrentGainSynthesisInput,
    CurrentGains, CurrentLoopEvidence, ElectricalModel, InnerSequence, PhaseId, TransientBuffer,
    classify_current_axis_sample,
};
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::types::{
    CommissioningBackend, CurrentValidationAxisEvidence, InnerResult, StepResult,
};

use super::impedance::{CommissioningImpedanceEvidence, run_current_gain_impedance};

/// Current-loop validation sub-state machine (Phase 7).
#[derive(Debug, defmt::Format)]
pub(crate) enum CurrentValidateState {
    /// Write PI gains to TMC4671 registers.
    WriteGains,
    /// Switch to torque mode and apply an axis current target.
    AxisStepSetup {
        axis: CurrentValidationAxis,
        sample_index: u8,
        sample_delay_ms: u16,
        direction: CaptureDirection,
    },
    /// Wait for the short sample delay to elapse.
    AxisSampleDelay {
        axis: CurrentValidationAxis,
        sample_index: u8,
        sample_delay_ms: u16,
        direction: CaptureDirection,
        deadline_us: u64,
    },
    /// Read the delayed current/voltage sample and append pair evidence.
    AxisSampleRead {
        axis: CurrentValidationAxis,
        sample_index: u8,
        sample_delay_ms: u16,
        direction: CaptureDirection,
    },
    /// Classify one axis after all configured sample delays.
    AxisClassify { axis: CurrentValidationAxis },
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
    /// Current validation attempt index carried in evidence.
    pub(crate) attempt: u8,
    /// Current step amplitude (absolute torque/flux current target counts).
    pub(crate) step_amplitude: u16,
    /// Final commissioning status to report.
    pub(crate) final_status: CommissionStatus,
    /// Configured PIDOUT voltage limit for saturation detection.
    pub(crate) pidout_limit: u32,
    /// Encoder count captured immediately before the active short sample target.
    pub(crate) sample_encoder_start: u32,
    /// Positive half of the active positive/negative delay pair.
    pub(crate) pending_positive_sample: Option<AxisDirectionalEvidence>,
    /// Number of populated flux-axis validation evidence items.
    pub(crate) current_flux_validation_count: u8,
    /// Flux-axis validation evidence by sample delay.
    pub(crate) current_flux_validation:
        [CurrentValidationAxisEvidence; crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
    /// Number of populated torque-axis validation evidence items.
    pub(crate) current_torque_validation_count: u8,
    /// Torque-axis validation evidence by sample delay.
    pub(crate) current_torque_validation:
        [CurrentValidationAxisEvidence; crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
    /// Warning bits accumulated through Stage 1 (bits 0–3 from Phase 5,
    /// bit 4 reserved for later retry policy, bit 5 ORed if gains fell back
    /// to defaults).
    pub(crate) inner_warning_flags: u8,
    /// L/R-vs-transient tau residual in permille from Phase 5.
    pub(crate) tau_residual_permille: u16,
    /// Shared multi-point resistance-identification result from Phase 5.
    pub(crate) resistance_result: crate::resistance_identification::ResistanceRunResult,
    /// Resistance count-slope (milli-units) consumed by the gain-formula path.
    /// Captured before any averaging so the final InnerResult can record the
    /// same-run 64x count-slope-milli audit counterpart against
    /// `selected_r_count_slope_milli`.
    pub(crate) resistance_gain_path_count_slope_milli: i32,
    /// Firmware-owned gain-source and axis-split evidence selected at tune time.
    pub(crate) current_loop_evidence: CurrentLoopEvidence,
    /// Accepted rotating-carrier impedance evidence, if it was used for synthesis.
    pub(crate) impedance_evidence: Option<CommissioningImpedanceEvidence>,
}

/// Current axis under short-sample validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum CurrentValidationAxis {
    /// Flux / D-axis current validation.
    Flux,
    /// Torque / Q-axis current validation.
    Torque,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct AxisDirectionalEvidence {
    gate: CurrentAxisGateResult,
    status_flags: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AxisSampleRegisters {
    pid_torque_flux_actual: u32,
    interim_foc_uq_ud_limited: u32,
    output_limit: u16,
    encoder_start: u32,
    encoder_end: u32,
    status_flags: u32,
}

/// Step amplitude as a fraction of run_current (15% = 15/100).
const VALIDATION_STEP_PERCENT: u32 = 15;

/// Settling time after removing validation current targets (microseconds).
const VALIDATION_SETTLE_US: u64 = 50_000;

const FLUX_VALIDATION_DELAYS_MS: [u16; crate::autotune::CURRENT_FLUX_VALIDATION_SAMPLE_COUNT] =
    [1, 2, 5];
const TORQUE_VALIDATION_DELAYS_MS: [u16; crate::autotune::CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT] =
    [1, 2];

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

fn validation_target_register(axis: CurrentValidationAxis, target: i16) -> u32 {
    match axis {
        CurrentValidationAxis::Flux => target as u16 as u32,
        CurrentValidationAxis::Torque => (target as u16 as u32) << 16,
    }
}

fn signed_lower_half(value: u32) -> i16 {
    crate::tmc::decode_signed_lower_half(value)
}

fn signed_upper_half(value: u32) -> i16 {
    crate::tmc::decode_signed_upper_half(value)
}

fn encoder_delta_counts(start: u32, end: u32) -> i32 {
    end.wrapping_sub(start) as i32
}

fn axis_sample_from_registers(
    axis: CurrentValidationAxis,
    target: i16,
    registers: AxisSampleRegisters,
) -> CurrentAxisSample {
    let (same_axis_actual, cross_axis_actual, output_limited) = match axis {
        CurrentValidationAxis::Flux => (
            signed_lower_half(registers.pid_torque_flux_actual),
            signed_upper_half(registers.pid_torque_flux_actual),
            signed_lower_half(registers.interim_foc_uq_ud_limited),
        ),
        CurrentValidationAxis::Torque => (
            signed_upper_half(registers.pid_torque_flux_actual),
            signed_lower_half(registers.pid_torque_flux_actual),
            signed_upper_half(registers.interim_foc_uq_ud_limited),
        ),
    };

    CurrentAxisSample {
        target,
        same_axis_actual,
        cross_axis_actual,
        output_limited,
        output_limit: registers.output_limit,
        encoder_delta_counts: encoder_delta_counts(registers.encoder_start, registers.encoder_end),
        status_flags: registers.status_flags,
    }
}

fn validation_gate(axis: CurrentValidationAxis, sample_delay_ms: u16) -> CurrentAxisGate {
    match (axis, sample_delay_ms) {
        (CurrentValidationAxis::Flux, 1) => CurrentAxisGate::openffboard_flux_1ms(),
        (CurrentValidationAxis::Flux, 2) => CurrentAxisGate::openffboard_flux_2ms(),
        (CurrentValidationAxis::Flux, _) => CurrentAxisGate::openffboard_flux_5ms(),
        (CurrentValidationAxis::Torque, 1) => CurrentAxisGate::openffboard_torque_1ms(),
        (CurrentValidationAxis::Torque, _) => CurrentAxisGate::openffboard_torque_2ms(),
    }
}

fn axis_validation_delays(axis: CurrentValidationAxis) -> &'static [u16] {
    match axis {
        CurrentValidationAxis::Flux => &FLUX_VALIDATION_DELAYS_MS,
        CurrentValidationAxis::Torque => &TORQUE_VALIDATION_DELAYS_MS,
    }
}

fn first_axis_step(axis: CurrentValidationAxis) -> CurrentValidateState {
    let sample_delay_ms = axis_validation_delays(axis)[0];
    CurrentValidateState::AxisStepSetup {
        axis,
        sample_index: 0,
        sample_delay_ms,
        direction: CaptureDirection::Forward,
    }
}

fn combine_axis_pair_evidence(
    attempt: u8,
    target: i16,
    sample_delay_ms: u16,
    positive: AxisDirectionalEvidence,
    negative: AxisDirectionalEvidence,
) -> CurrentValidationAxisEvidence {
    let status = merge_axis_pair_status(positive.gate.status, negative.gate.status);

    CurrentValidationAxisEvidence {
        status,
        attempt,
        target,
        sample_delay_ms,
        positive_response_permille: positive.gate.response_permille,
        negative_response_permille: negative.gate.response_permille,
        cross_axis_permille: positive
            .gate
            .cross_axis_permille
            .max(negative.gate.cross_axis_permille),
        voltage_output_permille: positive
            .gate
            .voltage_output_permille
            .max(negative.gate.voltage_output_permille),
        encoder_delta_counts: positive
            .gate
            .encoder_delta_counts
            .max(negative.gate.encoder_delta_counts),
        status_flags_or: positive.status_flags | negative.status_flags,
    }
}

fn merge_axis_pair_status(positive_status: u8, negative_status: u8) -> u8 {
    if status_priority(negative_status) > status_priority(positive_status) {
        negative_status
    } else {
        positive_status
    }
}

fn status_priority(status: u8) -> u8 {
    match status {
        crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS => 7,
        crate::autotune::CURRENT_AXIS_VALIDATION_SATURATION => 6,
        crate::autotune::CURRENT_AXIS_VALIDATION_MOTION => 5,
        crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS => 4,
        crate::autotune::CURRENT_AXIS_VALIDATION_WRONG_SIGN => 3,
        crate::autotune::CURRENT_AXIS_VALIDATION_ZERO_TARGET => 2,
        crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE
        | crate::autotune::CURRENT_AXIS_VALIDATION_HIGH_RESPONSE => 1,
        crate::autotune::CURRENT_AXIS_VALIDATION_PASS => 0,
        _ => 0,
    }
}

fn evidence_axis_passed(samples: &[CurrentValidationAxisEvidence], count: u8) -> bool {
    samples
        .iter()
        .take(count as usize)
        .all(|sample| sample.status == crate::autotune::CURRENT_AXIS_VALIDATION_PASS)
}

fn axis_failure_reason(axis: CurrentValidationAxis) -> u8 {
    match axis {
        CurrentValidationAxis::Flux => crate::autotune::CURRENT_LOOP_FAILURE_FLUX_VALIDATION,
        CurrentValidationAxis::Torque => crate::autotune::CURRENT_LOOP_FAILURE_TORQUE_VALIDATION,
    }
}

fn axis_failure_reason_for_samples(
    axis: CurrentValidationAxis,
    samples: &[CurrentValidationAxisEvidence],
    count: u8,
) -> u8 {
    let status = samples.iter().take(count as usize).fold(
        crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
        |worst, sample| merge_axis_pair_status(worst, sample.status),
    );

    match status {
        crate::autotune::CURRENT_AXIS_VALIDATION_SATURATION => {
            crate::autotune::CURRENT_LOOP_FAILURE_SATURATION
        }
        crate::autotune::CURRENT_AXIS_VALIDATION_MOTION => {
            crate::autotune::CURRENT_LOOP_FAILURE_MOTION
        }
        crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS => {
            crate::autotune::CURRENT_LOOP_FAILURE_STATUS_FLAGS
        }
        crate::autotune::CURRENT_AXIS_VALIDATION_PASS => axis_failure_reason(axis),
        _ => axis_failure_reason(axis),
    }
}

fn fallback_axis_failure_evidence(
    previous: CurrentLoopEvidence,
    failure_reason: u8,
) -> CurrentLoopEvidence {
    CurrentLoopEvidence::default_gains(failure_reason, false, previous.current_validation_axes)
}

#[cfg(test)]
fn fallback_current_loop_evidence(
    result: crate::autotune::ValidationResult,
    attempt: u8,
) -> CurrentLoopEvidence {
    let retry_exhausted = attempt > 0 && result != crate::autotune::ValidationResult::Pass;
    let failure_reason = if retry_exhausted {
        crate::autotune::CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED
    } else {
        crate::autotune::CURRENT_LOOP_FAILURE_TORQUE_VALIDATION
    };
    CurrentLoopEvidence::default_gains(failure_reason, retry_exhausted, 0)
}

/// Thin phase: read the electrical model, compute SIMC gains, and
/// transition immediately to validation.
pub(crate) async fn step_current_tune<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    profile: CommissionProfile,
    backend: &mut B,
) -> StepResult {
    let (model, inner_warning_flags, tau_residual_permille, resistance_result) = match phase {
        InnerPhase::CurrentTune {
            model,
            inner_warning_flags,
            tau_residual_permille,
            resistance_result,
        } => (
            *model,
            *inner_warning_flags,
            *tau_residual_permille,
            *resistance_result,
        ),
        _ => return StepResult::Continue,
    };

    let impedance_evidence = run_current_gain_impedance(backend).await.ok();
    let synthesis_input = match impedance_evidence {
        Some(evidence) => CurrentGainSynthesisInput::with_impedance_split(
            model,
            profile,
            evidence.measured_axis_split_permille,
            evidence.applied_axis_split_permille,
            evidence.high_inductance_axis,
        ),
        None => CurrentGainSynthesisInput::symmetric(model, profile),
    };
    let config = backend.tmc_config();
    let synthesis = crate::autotune::synthesize_current_gains(&synthesis_input, config);
    let (gains, mut current_loop_evidence) = match synthesis {
        Some(synthesis) => (synthesis.gains, synthesis.evidence),
        None => (
            CurrentGains {
                flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
                flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
                torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
                torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
                effective_bandwidth_hz: 0,
                lambda_us: 0,
            },
            CurrentLoopEvidence::failed(crate::autotune::CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS),
        ),
    };
    current_loop_evidence.current_validation_axes = 0;

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
            final_status: CommissionStatus::Accepted,
            pidout_limit: {
                // Read PIDOUT_UQ_UD_LIMITS for saturation detection.
                backend
                    .read_register(foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS)
                    .await
                    .unwrap_or(0)
                    & 0xFFFF
            },
            sample_encoder_start: 0,
            pending_positive_sample: None,
            current_flux_validation_count: 0,
            current_flux_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            inner_warning_flags,
            tau_residual_permille,
            resistance_result,
            // The gain-formula path consumes `model.r_int`, already in the
            // `ELECTRICAL_MODEL_RESISTANCE_SCALE` (milli) count-slope unit
            // space — identical to `selected_r_count_slope_milli`, so the 64x
            // mismatch is directly readable from the persisted artifacts.
            resistance_gain_path_count_slope_milli: model.r_int as i32,
            current_loop_evidence,
            impedance_evidence,
        },
    };
    StepResult::PhaseChanged(PhaseId::CurrentValidate)
}

/// Advance current-loop validation sub-state.
///
/// Writes SIMC gains, runs bidirectional short samples for each current axis,
/// classifies the per-delay evidence, and falls back to conservative defaults
/// on validation failure.
pub(crate) async fn step_current_validate<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    _buf: &mut TransientBuffer,
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

            // Clear any previous validation evidence for this attempt.
            data.pending_positive_sample = None;
            data.current_flux_validation_count = 0;
            data.current_flux_validation = [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
            data.current_torque_validation_count = 0;
            data.current_torque_validation = [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];

            *validate_state = first_axis_step(CurrentValidationAxis::Flux);
            StepResult::Continue
        }

        CurrentValidateState::AxisStepSetup {
            axis,
            sample_index,
            sample_delay_ms,
            direction,
        } => {
            let axis = *axis;
            let sample_index = *sample_index;
            let sample_delay_ms = *sample_delay_ms;
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

            data.sample_encoder_start = match backend.try_read_encoder_count().await {
                Ok(count) => count,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };

            let target = match dir {
                CaptureDirection::Forward => data.step_amplitude as i16,
                CaptureDirection::Reverse => -(data.step_amplitude as i16),
            };
            if backend
                .write_register(
                    PID_TORQUE_FLUX_TARGET,
                    validation_target_register(axis, target),
                )
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let deadline = backend
                .elapsed_us()
                .wrapping_add(u64::from(sample_delay_ms) * 1000);
            *validate_state = CurrentValidateState::AxisSampleDelay {
                axis,
                sample_index,
                sample_delay_ms,
                direction: dir,
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        CurrentValidateState::AxisSampleDelay {
            axis,
            sample_index,
            sample_delay_ms,
            direction,
            deadline_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::Continue;
            }
            *validate_state = CurrentValidateState::AxisSampleRead {
                axis: *axis,
                sample_index: *sample_index,
                sample_delay_ms: *sample_delay_ms,
                direction: *direction,
            };
            StepResult::Continue
        }

        CurrentValidateState::AxisSampleRead {
            axis,
            sample_index,
            sample_delay_ms,
            direction,
        } => {
            let axis = *axis;
            let sample_index = *sample_index;
            let sample_delay_ms = *sample_delay_ms;
            let dir = *direction;
            let pid_actual = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(actual) => actual,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };

            let sub = foci_tmc4671::registers::INTERIM_FOC_UQ_UD_LIMITED;
            if backend
                .write_register(sub.addr, sub.sub_addr as u32)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            let foc_limited = match backend.read_register(sub.data).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let status_flags = match backend.read_register(STATUS_FLAGS).await {
                Ok(flags) => flags,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let encoder_end = match backend.try_read_encoder_count().await {
                Ok(count) => count,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };

            let target = match dir {
                CaptureDirection::Forward => data.step_amplitude as i16,
                CaptureDirection::Reverse => -(data.step_amplitude as i16),
            };
            let sample = axis_sample_from_registers(
                axis,
                target,
                AxisSampleRegisters {
                    pid_torque_flux_actual: pid_actual,
                    interim_foc_uq_ud_limited: foc_limited,
                    output_limit: data.pidout_limit.min(u16::MAX as u32) as u16,
                    encoder_start: data.sample_encoder_start,
                    encoder_end,
                    status_flags,
                },
            );
            let gate =
                classify_current_axis_sample(&sample, &validation_gate(axis, sample_delay_ms));
            let directional = AxisDirectionalEvidence {
                gate,
                status_flags: sample.status_flags,
            };

            // Return to zero target and clear accumulated current error so
            // the next short sample starts from a defined controller state.
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

            match dir {
                CaptureDirection::Forward => {
                    data.pending_positive_sample = Some(directional);
                    *validate_state = CurrentValidateState::AxisStepSetup {
                        axis,
                        sample_index,
                        sample_delay_ms,
                        direction: CaptureDirection::Reverse,
                    };
                }
                CaptureDirection::Reverse => {
                    let positive = match data.pending_positive_sample.take() {
                        Some(sample) => sample,
                        None => {
                            return super::fail_inner(
                                phase,
                                CommissionError::CurrentValidationFailed,
                            );
                        }
                    };
                    let evidence = combine_axis_pair_evidence(
                        data.attempt,
                        data.step_amplitude as i16,
                        sample_delay_ms,
                        positive,
                        directional,
                    );
                    match axis {
                        CurrentValidationAxis::Flux => {
                            if let Some(slot) =
                                data.current_flux_validation.get_mut(sample_index as usize)
                            {
                                *slot = evidence;
                            }
                            data.current_flux_validation_count =
                                data.current_flux_validation_count.max(sample_index + 1);
                        }
                        CurrentValidationAxis::Torque => {
                            if let Some(slot) = data
                                .current_torque_validation
                                .get_mut(sample_index as usize)
                            {
                                *slot = evidence;
                            }
                            data.current_torque_validation_count =
                                data.current_torque_validation_count.max(sample_index + 1);
                        }
                    }

                    let next_index = sample_index + 1;
                    let delays = axis_validation_delays(axis);
                    if next_index as usize >= delays.len() {
                        *validate_state = CurrentValidateState::AxisClassify { axis };
                    } else {
                        *validate_state = CurrentValidateState::AxisStepSetup {
                            axis,
                            sample_index: next_index,
                            sample_delay_ms: delays[next_index as usize],
                            direction: CaptureDirection::Forward,
                        };
                    }
                }
            }
            StepResult::Continue
        }

        CurrentValidateState::AxisClassify { axis } => {
            let axis = *axis;
            let passed = match axis {
                CurrentValidationAxis::Flux => evidence_axis_passed(
                    &data.current_flux_validation,
                    data.current_flux_validation_count,
                ),
                CurrentValidationAxis::Torque => evidence_axis_passed(
                    &data.current_torque_validation,
                    data.current_torque_validation_count,
                ),
            };

            if passed {
                match axis {
                    CurrentValidationAxis::Flux => {
                        data.current_loop_evidence.current_validation_axes |=
                            crate::autotune::CURRENT_VALIDATION_AXIS_FLUX;
                        *validate_state = first_axis_step(CurrentValidationAxis::Torque);
                        return StepResult::Continue;
                    }
                    CurrentValidationAxis::Torque => {
                        data.current_loop_evidence.current_validation_axes |=
                            crate::autotune::CURRENT_VALIDATION_AXIS_TORQUE;
                        data.current_loop_evidence.failure_reason =
                            crate::autotune::CURRENT_LOOP_FAILURE_NONE;
                        data.final_status = if data.attempt > 0 {
                            data.inner_warning_flags |= 1 << 4;
                            CommissionStatus::AcceptedWithWarnings
                        } else {
                            CommissionStatus::Accepted
                        };
                    }
                }
            } else {
                let failure_reason = match axis {
                    CurrentValidationAxis::Flux => axis_failure_reason_for_samples(
                        axis,
                        &data.current_flux_validation,
                        data.current_flux_validation_count,
                    ),
                    CurrentValidationAxis::Torque => axis_failure_reason_for_samples(
                        axis,
                        &data.current_torque_validation,
                        data.current_torque_validation_count,
                    ),
                };
                data.current_loop_evidence =
                    fallback_axis_failure_evidence(data.current_loop_evidence, failure_reason);
                data.gains = CurrentGains {
                    flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
                    flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
                    torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
                    torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                };
                data.final_status = CommissionStatus::AcceptedWithWarnings;
                data.inner_warning_flags |= 1 << 5;

                let flux_reg = ((data.gains.flux_p as u32) << 16) | data.gains.flux_i as u32;
                let torque_reg = ((data.gains.torque_p as u32) << 16) | data.gains.torque_i as u32;
                let _ = backend.write_register(PID_FLUX_P_FLUX_I, flux_reg).await;
                let _ = backend
                    .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                    .await;
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
            let resistance = data.resistance_result;
            let axis0 = resistance.axis0;
            let axis1 = resistance.axis1;
            let impedance = data.impedance_evidence;
            let result = InnerResult {
                status: data.final_status,
                r_int: data.model.r_int,
                l_int: data.model.l_int,
                r_mohm: data.model.r_mohm,
                l_uh: data.model.l_uh,
                flux_p: data.gains.flux_p,
                flux_i: data.gains.flux_i,
                torque_p: data.gains.torque_p,
                torque_i: data.gains.torque_i,
                lambda_us: data.gains.lambda_us,
                theta_e_us: data.model.theta_e_us,
                ringing_count: 0,
                bandwidth_hz: data.gains.effective_bandwidth_hz,
                tau_e_us: data.model.tau_e_us,
                tau_e_crosscheck_us: data.model.tau_e_crosscheck_us,
                tau_residual_permille: data.tau_residual_permille,
                inner_warning_flags: data.inner_warning_flags,
                resistance_profile_version: resistance.profile_version,
                resistance_selected_count_slope_milli: resistance.selected_r_count_slope_milli,
                resistance_gain_path_count_slope_milli: data.resistance_gain_path_count_slope_milli,
                resistance_axis0_count_slope_milli: axis0.r_count_slope_milli,
                resistance_axis1_count_slope_milli: axis1.r_count_slope_milli,
                resistance_axis0_intercept_count: axis0.intercept_count,
                resistance_axis1_intercept_count: axis1.intercept_count,
                resistance_axis0_rmse_permille: axis0.rmse_permille,
                resistance_axis1_rmse_permille: axis1.rmse_permille,
                resistance_selected_mask_axis0: axis0.selected_mask,
                resistance_selected_mask_axis1: axis1.selected_mask,
                resistance_axis0_signed_count_slope_milli: axis0
                    .signed_anchor
                    .map_or(0, |anchor| anchor.signed_count_slope_milli),
                resistance_axis1_signed_count_slope_milli: axis1
                    .signed_anchor
                    .map_or(0, |anchor| anchor.signed_count_slope_milli),
                resistance_axis0_signed_asymmetry_permille: axis0
                    .signed_anchor
                    .map_or(0, |anchor| anchor.asymmetry_permille),
                resistance_axis1_signed_asymmetry_permille: axis1
                    .signed_anchor
                    .map_or(0, |anchor| anchor.asymmetry_permille),
                resistance_axis0_drift_permille: axis0
                    .signed_anchor
                    .map_or(0, |anchor| anchor.drift_permille),
                resistance_axis1_drift_permille: axis1
                    .signed_anchor
                    .map_or(0, |anchor| anchor.drift_permille),
                resistance_status_flags_or: resistance.status_flags_or,
                resistance_warning_flags: resistance.warning_flags.0,
                current_gains_source: data.current_loop_evidence.gains_source,
                axis_split_source: data.current_loop_evidence.axis_split_source,
                gains_tier: data.current_loop_evidence.gains_tier,
                measured_axis_split_permille: data
                    .current_loop_evidence
                    .measured_axis_split_permille,
                applied_axis_split_permille: data.current_loop_evidence.applied_axis_split_permille,
                axis_split_clamped: data.current_loop_evidence.axis_split_clamped,
                current_validation_axes: data.current_loop_evidence.current_validation_axes,
                retry_budget_exhausted: data.current_loop_evidence.retry_budget_exhausted,
                current_loop_failure_reason: data.current_loop_evidence.failure_reason,
                impedance_high_inductance_axis: impedance
                    .map_or(crate::autotune::CURRENT_AXIS_FLUX, |evidence| {
                        evidence.high_inductance_axis
                    }),
                impedance_selected_axis_max_count_per_ud_milli: impedance
                    .map_or(0, |evidence| evidence.selected_axis_max_count_per_ud_milli),
                impedance_selected_axis_min_count_per_ud_milli: impedance
                    .map_or(0, |evidence| evidence.selected_axis_min_count_per_ud_milli),
                impedance_saliency_permille: impedance
                    .map_or(0, |evidence| evidence.saliency_permille),
                impedance_fitted_axis_electrical_counts: impedance
                    .map_or(0, |evidence| evidence.fitted_axis_electrical_counts),
                impedance_residual_permille: impedance
                    .map_or(0, |evidence| evidence.residual_permille),
                impedance_warning_flags: impedance.map_or(0, |evidence| evidence.warning_flags),
                current_flux_validation_count: data.current_flux_validation_count,
                current_flux_validation: data.current_flux_validation,
                current_torque_validation_count: data.current_torque_validation_count,
                current_torque_validation: data.current_torque_validation,
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

#[cfg(test)]
mod tests {
    use crate::autotune::{
        AXIS_SPLIT_SOURCE_NONE, CURRENT_GAINS_SOURCE_DEFAULT, CURRENT_GAINS_TIER_DEFAULT,
        CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED, CURRENT_LOOP_FAILURE_STATUS_FLAGS,
        CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, CURRENT_VALIDATION_AXIS_FLUX, ValidationResult,
    };

    use super::*;

    #[test]
    fn fallback_evidence_reports_default_gains_and_validation_failure() {
        let evidence = fallback_current_loop_evidence(ValidationResult::Fail, 0);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_DEFAULT);
        assert_eq!(evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(evidence.gains_tier, CURRENT_GAINS_TIER_DEFAULT);
        assert_eq!(evidence.measured_axis_split_permille, 1000);
        assert_eq!(evidence.applied_axis_split_permille, 1000);
        assert_eq!(evidence.axis_split_clamped, 0);
        assert_eq!(evidence.current_validation_axes, 0);
        assert_eq!(evidence.retry_budget_exhausted, 0);
        assert_eq!(
            evidence.failure_reason,
            CURRENT_LOOP_FAILURE_TORQUE_VALIDATION
        );
    }

    #[test]
    fn fallback_evidence_reports_retry_exhaustion_after_retry_attempt() {
        let evidence = fallback_current_loop_evidence(ValidationResult::Marginal, 1);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_DEFAULT);
        assert_eq!(evidence.current_validation_axes, 0);
        assert_eq!(evidence.retry_budget_exhausted, 1);
        assert_eq!(
            evidence.failure_reason,
            CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED
        );
    }

    #[test]
    fn validation_failure_fallback_preserves_prior_axis_coverage() {
        let mut previous = CurrentLoopEvidence::measured_symmetric();
        previous.current_validation_axes = CURRENT_VALIDATION_AXIS_FLUX;

        let evidence =
            fallback_axis_failure_evidence(previous, CURRENT_LOOP_FAILURE_TORQUE_VALIDATION);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_DEFAULT);
        assert_eq!(
            evidence.current_validation_axes,
            CURRENT_VALIDATION_AXIS_FLUX
        );
        assert_eq!(
            evidence.failure_reason,
            CURRENT_LOOP_FAILURE_TORQUE_VALIDATION
        );
    }

    #[test]
    fn axis_failure_reason_preserves_later_actionable_failure() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[1] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS);

        let reason = axis_failure_reason_for_samples(CurrentValidationAxis::Torque, &samples, 2);

        assert_eq!(reason, CURRENT_LOOP_FAILURE_STATUS_FLAGS);
    }

    #[test]
    fn validation_target_register_packs_flux_and_clears_torque() {
        assert_eq!(
            validation_target_register(CurrentValidationAxis::Flux, -250),
            0x0000_ff06
        );
    }

    #[test]
    fn validation_target_register_packs_torque_and_clears_flux() {
        assert_eq!(
            validation_target_register(CurrentValidationAxis::Torque, 250),
            0x00fa_0000
        );
    }

    #[test]
    fn axis_sample_from_registers_reads_flux_halves() {
        let sample = axis_sample_from_registers(
            CurrentValidationAxis::Flux,
            250,
            AxisSampleRegisters {
                pid_torque_flux_actual: pack_halves(25, 240),
                interim_foc_uq_ud_limited: pack_halves(500, 300),
                output_limit: 2000,
                encoder_start: 1000,
                encoder_end: 1002,
                status_flags: 0x10,
            },
        );

        assert_eq!(sample.same_axis_actual, 240);
        assert_eq!(sample.cross_axis_actual, 25);
        assert_eq!(sample.output_limited, 300);
        assert_eq!(sample.encoder_delta_counts, 2);
        assert_eq!(sample.status_flags, 0x10);
    }

    #[test]
    fn axis_sample_from_registers_reads_torque_halves() {
        let sample = axis_sample_from_registers(
            CurrentValidationAxis::Torque,
            -250,
            AxisSampleRegisters {
                pid_torque_flux_actual: pack_halves(-240, 25),
                interim_foc_uq_ud_limited: pack_halves(-300, 500),
                output_limit: 2000,
                encoder_start: 1002,
                encoder_end: 1000,
                status_flags: 0,
            },
        );

        assert_eq!(sample.same_axis_actual, -240);
        assert_eq!(sample.cross_axis_actual, 25);
        assert_eq!(sample.output_limited, -300);
        assert_eq!(sample.encoder_delta_counts, -2);
    }

    #[test]
    fn combine_axis_pair_evidence_preserves_actionable_negative_failure() {
        let evidence = combine_axis_pair_evidence(
            0,
            250,
            2,
            axis_directional_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE),
            axis_directional_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS),
        );

        assert_eq!(
            evidence.status,
            crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS
        );
    }

    fn axis_directional_evidence(status: u8) -> AxisDirectionalEvidence {
        AxisDirectionalEvidence {
            gate: CurrentAxisGateResult {
                status,
                response_permille: 0,
                cross_axis_permille: 0,
                voltage_output_permille: 0,
                encoder_delta_counts: 0,
                max_cross_axis_permille: 150,
            },
            status_flags: if status == crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS {
                1
            } else {
                0
            },
        }
    }

    fn axis_validation_evidence(status: u8) -> CurrentValidationAxisEvidence {
        CurrentValidationAxisEvidence {
            status,
            ..CurrentValidationAxisEvidence::EMPTY
        }
    }

    fn pack_halves(upper: i16, lower: i16) -> u32 {
        ((upper as u16 as u32) << 16) | lower as u16 as u32
    }
}
