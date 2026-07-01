//! Phases 6 and 7: Current-loop SIMC gain synthesis and step-response
//! validation.
//!
//! `step_current_tune` reads the electrical model from Phase 5 and applies
//! the SIMC tuning rule to derive flux/torque PI gains plus a step
//! amplitude for validation. `step_current_validate` then writes those
//! gains, samples flux and torque current responses at short fixed delays,
//! classifies the per-axis evidence, and fails explicitly on invalid measured
//! current gains.

use crate::autotune::{
    CaptureDirection, ClosedLoopEntryState, CommissionError, CommissionProfile, CommissionStatus,
    CurrentAxisGate, CurrentAxisGateResult, CurrentAxisRetryDecision, CurrentAxisSample,
    CurrentGainSynthesisInput, CurrentGains, CurrentLoopEvidence, ElectricalModel, InnerSequence,
    PhaseId, TransientBuffer, classify_current_axis_sample, current_axis_retry_decision,
    retry_current_loop_gains,
};
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::types::{
    CommissionResult, CommissioningBackend, CurrentValidationAxisEvidence,
    INDUCTANCE_FIT_POINT_COUNT, InductanceFitEvidence, InductanceFitPoint, InnerResult, StepResult,
};

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
    /// Wait for physical current to decay after returning the target to zero.
    AxisSampleSettle {
        axis: CurrentValidationAxis,
        sample_index: u8,
        sample_delay_ms: u16,
        direction: CaptureDirection,
        directional: AxisDirectionalEvidence,
        deadline_us: u64,
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
    /// Candidate gains last written for validation, before safe-state restore.
    pub(crate) candidate_gains: CurrentGains,
    /// Retry attempt associated with `candidate_gains`.
    pub(crate) candidate_attempt: u8,
    /// Flux-axis validation retry attempt index carried in evidence.
    pub(crate) flux_attempt: u8,
    /// Torque-axis validation retry attempt index carried in evidence.
    pub(crate) torque_attempt: u8,
    /// Current step amplitude (absolute torque/flux current target counts).
    pub(crate) step_amplitude: u16,
    /// Final commissioning status to report.
    pub(crate) final_status: CommissionStatus,
    /// Configured PIDOUT voltage limit for saturation detection.
    pub(crate) pidout_limit: u32,
    /// Encoder counts per mechanical revolution for wrap-aware motion checks.
    pub(crate) encoder_counts_per_rev: u32,
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
    /// Warning bits accumulated through Stage 1.
    ///
    /// Bits 0, 1, and 3 come from Phase 5 model gates; bit 2 is unused. Bit 5
    /// is ORed if gains fell back to defaults.
    pub(crate) inner_warning_flags: u8,
    /// Accepted per-coil corrected inductance fit evidence.
    pub(crate) inductance_fit: [InductanceFitEvidence; 2],
    /// Per-amplitude points used by the accepted corrected inductance fits.
    pub(crate) inductance_fit_points: [[InductanceFitPoint; INDUCTANCE_FIT_POINT_COUNT]; 2],
    /// Shared multi-point resistance-identification result from Phase 5.
    pub(crate) resistance_result: crate::resistance_identification::ResistanceRunResult,
    /// Resistance count-slope (milli-units) consumed by the gain-formula path.
    /// Captured before any averaging so the final InnerResult can record the
    /// same-run 64x count-slope-milli audit counterpart against
    /// `selected_r_count_slope_milli`.
    pub(crate) resistance_gain_path_count_slope_milli: i32,
    /// Firmware-owned gain-source and axis-split evidence selected at tune time.
    pub(crate) current_loop_evidence: CurrentLoopEvidence,
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
    encoder_delta_counts: i32,
    cross_axis_peak_permille: u16,
    status_flags: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AxisSampleRegisters {
    pid_torque_flux_actual: u32,
    interim_foc_uq_ud_limited: u32,
    output_limit: u16,
    encoder_start: u32,
    encoder_end: u32,
    encoder_counts_per_rev: u32,
    status_flags: u32,
}

/// Step amplitude as a fraction of run_current (15% = 15/100).
const VALIDATION_STEP_PERCENT: u32 = 15;

/// Settling time after removing validation current targets (microseconds).
const VALIDATION_SETTLE_US: u64 = 50_000;

const FLUX_VALIDATION_DELAYS_MS: [u16; crate::autotune::CURRENT_FLUX_VALIDATION_SAMPLE_COUNT] =
    [1, 2, 5, 100];
const TORQUE_VALIDATION_DELAYS_MS: [u16; crate::autotune::CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT] =
    [0, 1, 2];
const SETTLED_FLUX_SAMPLE_DELAY_MS: u16 = 100;
const SETTLED_FLUX_AVERAGE_SAMPLES: usize = 4;
const SETTLED_FLUX_AVERAGE_INTERVAL_US: u32 = 250;

/// PID output voltage limit used during current-loop validation.
///
/// ADC calibration leaves `PIDOUT_UQ_UD_LIMITS` at 1024 as a conservative
/// safety cap, but the measured validation captures can hit that cap before
/// reaching the current target. 2000 matches the bounded encoder-alignment
/// limit and remains well below the operational closed-loop limit.
const CURRENT_VALIDATE_PIDOUT_LIMIT: u32 = 2000;
const CURRENT_SAFE_PIDOUT_LIMIT: u32 = 1024;

/// `MODE_RAMP_MODE_MOTION` value for advanced PI torque mode.
///
/// High bit selects the advanced controller structure used by the tuned
/// current loop; low bits select torque mode.
const CURRENT_TORQUE_MODE: u32 = 0x8000_0001;
const CURRENT_VALIDATE_PWM_SV_CHOP: u32 = 7;
const CURRENT_VALIDATE_PHI_E_SELECTION: u32 = 3;

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

fn encoder_delta_counts(start: u32, end: u32, counts_per_rev: u32) -> i32 {
    let counts_u32 = counts_per_rev.max(1);
    let counts = i64::from(counts_u32);
    let half = counts / 2;
    let start = i64::from(start % counts_u32);
    let end = i64::from(end % counts_u32);
    let raw = end - start;
    let delta = if raw > half {
        raw - counts
    } else if raw < -half {
        raw + counts
    } else {
        raw
    };
    delta.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
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
        encoder_delta_counts: encoder_delta_counts(
            registers.encoder_start,
            registers.encoder_end,
            registers.encoder_counts_per_rev,
        ),
        status_flags: registers.status_flags,
    }
}

fn cross_axis_permille(sample: &CurrentAxisSample) -> u16 {
    let target_abs = sample.target.unsigned_abs().max(1) as u32;
    ((sample.cross_axis_actual.unsigned_abs() as u32 * 1000) / target_abs).min(u16::MAX as u32)
        as u16
}

fn max_abs_i16(current: i16, candidate: i16) -> i16 {
    if candidate.unsigned_abs() > current.unsigned_abs() {
        candidate
    } else {
        current
    }
}

fn max_abs_i32(current: i32, candidate: i32) -> i32 {
    if candidate.unsigned_abs() > current.unsigned_abs() {
        candidate
    } else {
        current
    }
}

fn should_average_current_sample(axis: CurrentValidationAxis, sample_delay_ms: u16) -> bool {
    axis == CurrentValidationAxis::Flux && sample_delay_ms == SETTLED_FLUX_SAMPLE_DELAY_MS
}

fn validation_gate(
    config: &crate::config::TmcConfig,
    axis: CurrentValidationAxis,
    sample_delay_ms: u16,
) -> CurrentAxisGate {
    let max_encoder_delta_counts = match axis {
        CurrentValidationAxis::Flux => config.current_validation_flux_max_encoder_delta_counts,
        CurrentValidationAxis::Torque => config.current_validation_torque_max_encoder_delta_counts,
    };
    let (min_response_permille, max_response_permille) = match (axis, sample_delay_ms) {
        (CurrentValidationAxis::Flux, 1) => (250, 1250),
        (CurrentValidationAxis::Flux, 2) => (700, 1250),
        // The 5 ms flux sample remains bandwidth evidence. Response magnitude
        // is reported, but response-only misses do not block axis acceptance;
        // the 100 ms sample below is the steady-state closure gate.
        (CurrentValidationAxis::Flux, 5) => (850, 1150),
        (CurrentValidationAxis::Flux, _) => (850, 1150),
        // Zero-delay torque is an earliest-possible geometry discriminator.
        // Keep safety/status/cross-axis gates active, but do not make an
        // intentionally pre-settle response magnitude block commissioning.
        (CurrentValidationAxis::Torque, 0) => (0, 1250),
        (CurrentValidationAxis::Torque, 1) => (250, 1250),
        (CurrentValidationAxis::Torque, _) => (550, 1250),
    };
    CurrentAxisGate::commissioning_short_sample(
        min_response_permille,
        max_response_permille,
        max_encoder_delta_counts,
    )
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
        cross_axis_peak_permille: positive
            .cross_axis_peak_permille
            .max(negative.cross_axis_peak_permille),
        voltage_output_permille: positive
            .gate
            .voltage_output_permille
            .max(negative.gate.voltage_output_permille),
        encoder_delta_counts: positive
            .gate
            .encoder_delta_counts
            .max(negative.gate.encoder_delta_counts),
        positive_encoder_delta_counts: positive.encoder_delta_counts,
        negative_encoder_delta_counts: negative.encoder_delta_counts,
        status_flags_or: positive.status_flags | negative.status_flags,
    }
}

async fn read_axis_sample<B: CommissioningBackend>(
    backend: &mut B,
    axis: CurrentValidationAxis,
    target: i16,
    encoder_start: u32,
    output_limit: u32,
    encoder_counts_per_rev: u32,
) -> Result<CurrentAxisSample, ()> {
    use foci_tmc4671::registers::{
        INTERIM_FOC_UQ_UD_LIMITED, PID_TORQUE_FLUX_ACTUAL, STATUS_FLAGS,
    };

    let pid_actual = backend
        .read_register(PID_TORQUE_FLUX_ACTUAL)
        .await
        .map_err(|_| ())?;
    let sub = INTERIM_FOC_UQ_UD_LIMITED;
    backend
        .write_register(sub.addr, sub.sub_addr as u32)
        .await
        .map_err(|_| ())?;
    let foc_limited = backend.read_register(sub.data).await.map_err(|_| ())?;
    let status_flags = backend.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
    let encoder_end = backend.try_read_encoder_count().await.map_err(|_| ())?;

    Ok(axis_sample_from_registers(
        axis,
        target,
        AxisSampleRegisters {
            pid_torque_flux_actual: pid_actual,
            interim_foc_uq_ud_limited: foc_limited,
            output_limit: output_limit.min(u16::MAX as u32) as u16,
            encoder_start,
            encoder_end,
            encoder_counts_per_rev,
            status_flags,
        },
    ))
}

fn directional_evidence_from_sample<B: CommissioningBackend>(
    backend: &B,
    axis: CurrentValidationAxis,
    sample_delay_ms: u16,
    sample: CurrentAxisSample,
) -> AxisDirectionalEvidence {
    let gate = classify_current_axis_sample(
        &sample,
        &validation_gate(backend.tmc_config(), axis, sample_delay_ms),
    );

    AxisDirectionalEvidence {
        cross_axis_peak_permille: gate.cross_axis_permille,
        gate,
        encoder_delta_counts: sample.encoder_delta_counts,
        status_flags: sample.status_flags,
    }
}

async fn read_axis_directional_evidence<B: CommissioningBackend>(
    backend: &mut B,
    axis: CurrentValidationAxis,
    target: i16,
    sample_delay_ms: u16,
    encoder_start: u32,
    output_limit: u32,
    encoder_counts_per_rev: u32,
) -> Result<AxisDirectionalEvidence, ()> {
    let sample = read_axis_sample(
        backend,
        axis,
        target,
        encoder_start,
        output_limit,
        encoder_counts_per_rev,
    )
    .await?;

    Ok(directional_evidence_from_sample(
        backend,
        axis,
        sample_delay_ms,
        sample,
    ))
}

async fn read_axis_directional_evidence_for_gate<B: CommissioningBackend>(
    backend: &mut B,
    axis: CurrentValidationAxis,
    target: i16,
    sample_delay_ms: u16,
    encoder_start: u32,
    output_limit: u32,
    encoder_counts_per_rev: u32,
) -> Result<AxisDirectionalEvidence, ()> {
    if !should_average_current_sample(axis, sample_delay_ms) {
        return read_axis_directional_evidence(
            backend,
            axis,
            target,
            sample_delay_ms,
            encoder_start,
            output_limit,
            encoder_counts_per_rev,
        )
        .await;
    }

    let mut same_axis_total = 0i32;
    let mut cross_axis_total = 0i32;
    let mut output_limited_peak = 0i16;
    let mut encoder_delta_peak = 0i32;
    let mut status_flags_or = 0u32;
    let mut cross_axis_peak_permille = 0u16;

    for index in 0..SETTLED_FLUX_AVERAGE_SAMPLES {
        let sample = read_axis_sample(
            backend,
            axis,
            target,
            encoder_start,
            output_limit,
            encoder_counts_per_rev,
        )
        .await?;

        same_axis_total += i32::from(sample.same_axis_actual);
        cross_axis_total += i32::from(sample.cross_axis_actual);
        output_limited_peak = max_abs_i16(output_limited_peak, sample.output_limited);
        encoder_delta_peak = max_abs_i32(encoder_delta_peak, sample.encoder_delta_counts);
        status_flags_or |= sample.status_flags;
        cross_axis_peak_permille = cross_axis_peak_permille.max(cross_axis_permille(&sample));

        if index + 1 < SETTLED_FLUX_AVERAGE_SAMPLES {
            backend.delay_us(SETTLED_FLUX_AVERAGE_INTERVAL_US).await;
        }
    }

    let count = SETTLED_FLUX_AVERAGE_SAMPLES as i32;
    let averaged_sample = CurrentAxisSample {
        target,
        same_axis_actual: (same_axis_total / count) as i16,
        cross_axis_actual: (cross_axis_total / count) as i16,
        output_limited: output_limited_peak,
        output_limit: output_limit.min(u16::MAX as u32) as u16,
        encoder_delta_counts: encoder_delta_peak,
        status_flags: status_flags_or,
    };
    let mut evidence =
        directional_evidence_from_sample(backend, axis, sample_delay_ms, averaged_sample);
    evidence.cross_axis_peak_permille = cross_axis_peak_permille;
    Ok(evidence)
}

fn advance_after_directional_sample(
    data: &mut CurrentValidateData,
    validate_state: &mut CurrentValidateState,
    axis: CurrentValidationAxis,
    sample_index: u8,
    sample_delay_ms: u16,
    direction: CaptureDirection,
    directional: AxisDirectionalEvidence,
) -> Result<(), CommissionError> {
    match direction {
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
            let positive = data
                .pending_positive_sample
                .take()
                .ok_or(CommissionError::CurrentValidationFailed)?;
            let evidence = combine_axis_pair_evidence(
                axis_attempt(data, axis),
                data.step_amplitude as i16,
                sample_delay_ms,
                positive,
                directional,
            );
            match axis {
                CurrentValidationAxis::Flux => {
                    if let Some(slot) = data.current_flux_validation.get_mut(sample_index as usize)
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

    Ok(())
}

fn settle_after_directional_sample(
    validate_state: &mut CurrentValidateState,
    now_us: u64,
    axis: CurrentValidationAxis,
    sample_index: u8,
    sample_delay_ms: u16,
    direction: CaptureDirection,
    directional: AxisDirectionalEvidence,
) {
    *validate_state = CurrentValidateState::AxisSampleSettle {
        axis,
        sample_index,
        sample_delay_ms,
        direction,
        directional,
        deadline_us: now_us.wrapping_add(VALIDATION_SETTLE_US),
    };
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

fn evidence_axis_passed(
    axis: CurrentValidationAxis,
    samples: &[CurrentValidationAxisEvidence],
    count: u8,
) -> bool {
    samples.iter().take(count as usize).all(|sample| {
        axis_pass_status(axis, sample) == crate::autotune::CURRENT_AXIS_VALIDATION_PASS
    })
}

fn axis_pass_status(axis: CurrentValidationAxis, sample: &CurrentValidationAxisEvidence) -> u8 {
    // Flux short samples are bandwidth telemetry. The long-dwell flux sample
    // proves steady-state closure and is the sole hard orientation/coupling
    // gate; safety/status gates still apply at every delay. Torque is
    // motion-limited, so its cross/response shape is telemetry while
    // safety/status gates remain hard.
    let telemetry_status = match axis {
        CurrentValidationAxis::Flux => {
            sample.sample_delay_ms < 100 && is_transient_current_telemetry_status(sample.status)
        }
        CurrentValidationAxis::Torque => {
            (sample.sample_delay_ms == 0 && is_zero_delay_torque_telemetry_status(sample.status))
                || (sample.sample_delay_ms > 0
                    && is_transient_current_telemetry_status(sample.status))
        }
    };

    if telemetry_status {
        crate::autotune::CURRENT_AXIS_VALIDATION_PASS
    } else {
        sample.status
    }
}

fn is_response_magnitude_status(status: u8) -> bool {
    status == crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE
        || status == crate::autotune::CURRENT_AXIS_VALIDATION_HIGH_RESPONSE
}

fn is_transient_current_telemetry_status(status: u8) -> bool {
    is_response_magnitude_status(status)
        || status == crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS
}

fn is_zero_delay_torque_telemetry_status(status: u8) -> bool {
    is_response_magnitude_status(status)
        || status == crate::autotune::CURRENT_AXIS_VALIDATION_WRONG_SIGN
        || status == crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS
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
        |worst, sample| merge_axis_pair_status(worst, axis_pass_status(axis, sample)),
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

fn failed_axis_failure_evidence(
    previous: CurrentLoopEvidence,
    failure_reason: u8,
    retry_budget_exhausted: bool,
) -> CurrentLoopEvidence {
    let mut evidence = CurrentLoopEvidence::failed(failure_reason);
    evidence.candidate_gains_source = previous.candidate_gains_source;
    evidence.candidate_axis_split_source = previous.candidate_axis_split_source;
    evidence.candidate_gains_tier = previous.candidate_gains_tier;
    evidence.candidate_measured_axis_split_permille =
        previous.candidate_measured_axis_split_permille;
    evidence.candidate_applied_axis_split_permille = previous.candidate_applied_axis_split_permille;
    evidence.candidate_axis_split_clamped = previous.candidate_axis_split_clamped;
    evidence.current_validation_axes = previous.current_validation_axes;
    evidence.retry_budget_exhausted = retry_budget_exhausted as u8;
    evidence
}

#[cfg(test)]
fn failed_current_loop_evidence(failure_reason: u8, retry_exhausted: bool) -> CurrentLoopEvidence {
    failed_axis_failure_evidence(
        CurrentLoopEvidence::measured_symmetric(),
        failure_reason,
        retry_exhausted,
    )
}

fn disabled_state_safe_current_gains() -> CurrentGains {
    CurrentGains {
        flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
        flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
        torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
        torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
        effective_bandwidth_hz: 0,
        lambda_us: 0,
    }
}

fn axis_attempt(data: &CurrentValidateData, axis: CurrentValidationAxis) -> u8 {
    match axis {
        CurrentValidationAxis::Flux => data.flux_attempt,
        CurrentValidationAxis::Torque => data.torque_attempt,
    }
}

fn worst_axis_status(
    axis: CurrentValidationAxis,
    samples: &[CurrentValidationAxisEvidence],
    count: u8,
) -> u8 {
    samples.iter().take(count as usize).fold(
        crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
        |worst, sample| merge_axis_pair_status(worst, axis_pass_status(axis, sample)),
    )
}

fn axis_retry_decision_for_samples(
    axis: CurrentValidationAxis,
    samples: &[CurrentValidationAxisEvidence],
    count: u8,
    attempt: u8,
) -> CurrentAxisRetryDecision {
    let status = worst_axis_status(axis, samples, count);
    current_axis_retry_decision(
        status,
        attempt,
        status == crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS,
        status == crate::autotune::CURRENT_AXIS_VALIDATION_SATURATION,
        status == crate::autotune::CURRENT_AXIS_VALIDATION_MOTION,
    )
}

async fn restore_current_validation_safe_state<B: CommissioningBackend>(
    backend: &mut B,
    gains: CurrentGains,
) -> Result<(), ()> {
    use foci_tmc4671::registers::*;

    let mut failed = false;
    if backend
        .write_register(PID_TORQUE_FLUX_TARGET, 0)
        .await
        .is_err()
    {
        failed = true;
    }
    if crate::commissioning::helpers::clear_current_integrators(backend)
        .await
        .is_err()
    {
        failed = true;
    }

    let flux_reg = ((gains.flux_p as u32) << 16) | gains.flux_i as u32;
    let torque_reg = ((gains.torque_p as u32) << 16) | gains.torque_i as u32;
    if backend
        .write_register(PID_FLUX_P_FLUX_I, flux_reg)
        .await
        .is_err()
    {
        failed = true;
    }
    if backend
        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
        .await
        .is_err()
    {
        failed = true;
    }
    if backend
        .write_register(PIDOUT_UQ_UD_LIMITS, CURRENT_SAFE_PIDOUT_LIMIT)
        .await
        .is_err()
    {
        failed = true;
    }
    if backend
        .write_register(MODE_RAMP_MODE_MOTION, 0)
        .await
        .is_err()
    {
        failed = true;
    }
    if backend.write_register(UQ_UD_EXT, 0).await.is_err() {
        failed = true;
    }
    backend.set_enable_pin(false);

    if failed { Err(()) } else { Ok(()) }
}

/// Thin phase: read the electrical model, compute SIMC gains, and
/// transition immediately to validation.
pub(crate) async fn step_current_tune<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    profile: CommissionProfile,
    backend: &mut B,
) -> StepResult {
    let (model, inner_warning_flags, resistance_result, inductance_fit, inductance_fit_points) =
        match phase {
            InnerPhase::CurrentTune {
                model,
                inner_warning_flags,
                resistance_result,
                inductance_fit,
                inductance_fit_points,
            } => (
                *model,
                *inner_warning_flags,
                *resistance_result,
                *inductance_fit,
                *inductance_fit_points,
            ),
            _ => return StepResult::Continue,
        };

    let synthesis_input = CurrentGainSynthesisInput::symmetric(model, profile);
    let config = backend.tmc_config();
    let synthesis = crate::autotune::synthesize_current_gains(&synthesis_input, config);
    let (gains, mut current_loop_evidence) = match synthesis {
        Some(synthesis) => (synthesis.gains, synthesis.evidence),
        None => {
            let mut evidence =
                CurrentLoopEvidence::failed(crate::autotune::CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS);
            evidence.candidate_gains_source = crate::autotune::CURRENT_GAINS_SOURCE_DEFAULT;
            evidence.candidate_gains_tier = crate::autotune::CURRENT_GAINS_TIER_DEFAULT;
            (
                CurrentGains {
                    flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
                    flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
                    torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
                    torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                },
                evidence,
            )
        }
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
            candidate_gains: gains,
            candidate_attempt: 0,
            flux_attempt: 0,
            torque_attempt: 0,
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
            encoder_counts_per_rev: 1,
            sample_encoder_start: 0,
            pending_positive_sample: None,
            current_flux_validation_count: 0,
            current_flux_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            inner_warning_flags,
            inductance_fit,
            inductance_fit_points,
            resistance_result,
            // The gain-formula path consumes `model.r_int`, already in the
            // `ELECTRICAL_MODEL_RESISTANCE_SCALE` (milli) count-slope unit
            // space — identical to `selected_r_count_slope_milli`, so the 64x
            // mismatch is directly readable from the persisted artifacts.
            resistance_gain_path_count_slope_milli: model.r_int as i32,
            current_loop_evidence,
        },
    };
    StepResult::PhaseChanged(PhaseId::CurrentValidate)
}

/// Advance current-loop validation sub-state.
///
/// Writes SIMC gains, runs bidirectional short samples for each current axis,
/// classifies the per-delay evidence, and fails explicitly on invalid measured
/// current gains.
pub(crate) async fn step_current_validate<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    _buf: &mut TransientBuffer,
    _profile: CommissionProfile,
    sequence: InnerSequence,
    adc_cal: &Option<crate::commissioning::types::AdcCalResult>,
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
                || backend
                    .write_register(PWM_SV_CHOP, CURRENT_VALIDATE_PWM_SV_CHOP)
                    .await
                    .is_err()
                || backend
                    .write_register(PHI_E_SELECTION, CURRENT_VALIDATE_PHI_E_SELECTION)
                    .await
                    .is_err()
                || crate::commissioning::helpers::clear_current_integrators(backend)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            data.pidout_limit = CURRENT_VALIDATE_PIDOUT_LIMIT;
            data.encoder_counts_per_rev = match backend.read_register(ABN_DECODER_PPR).await {
                Ok(counts) => counts.max(1),
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };

            // Clear any previous validation evidence for this attempt.
            data.pending_positive_sample = None;
            data.current_flux_validation_count = 0;
            data.current_flux_validation = [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
            data.current_torque_validation_count = 0;
            data.current_torque_validation = [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
            data.current_loop_evidence.current_validation_axes = 0;

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
                || backend.write_register(STATUS_FLAGS, 0).await.is_err()
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

            if sample_delay_ms == 0 {
                let directional = match read_axis_directional_evidence_for_gate(
                    backend,
                    axis,
                    target,
                    sample_delay_ms,
                    data.sample_encoder_start,
                    data.pidout_limit,
                    data.encoder_counts_per_rev,
                )
                .await
                {
                    Ok(sample) => sample,
                    Err(()) => return super::fail_inner(phase, CommissionError::Spi),
                };

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

                settle_after_directional_sample(
                    validate_state,
                    backend.elapsed_us(),
                    axis,
                    sample_index,
                    sample_delay_ms,
                    dir,
                    directional,
                );

                return StepResult::Continue;
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

            let target = match dir {
                CaptureDirection::Forward => data.step_amplitude as i16,
                CaptureDirection::Reverse => -(data.step_amplitude as i16),
            };
            let directional = match read_axis_directional_evidence_for_gate(
                backend,
                axis,
                target,
                sample_delay_ms,
                data.sample_encoder_start,
                data.pidout_limit,
                data.encoder_counts_per_rev,
            )
            .await
            {
                Ok(sample) => sample,
                Err(()) => return super::fail_inner(phase, CommissionError::Spi),
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

            settle_after_directional_sample(
                validate_state,
                backend.elapsed_us(),
                axis,
                sample_index,
                sample_delay_ms,
                dir,
                directional,
            );
            StepResult::Continue
        }

        CurrentValidateState::AxisSampleSettle {
            axis,
            sample_index,
            sample_delay_ms,
            direction,
            directional,
            deadline_us,
        } => {
            let axis = *axis;
            let sample_index = *sample_index;
            let sample_delay_ms = *sample_delay_ms;
            let direction = *direction;
            let directional = *directional;
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            if let Err(err) = advance_after_directional_sample(
                data,
                validate_state,
                axis,
                sample_index,
                sample_delay_ms,
                direction,
                directional,
            ) {
                return super::fail_inner(phase, err);
            }
            StepResult::Continue
        }

        CurrentValidateState::AxisClassify { axis } => {
            let axis = *axis;
            let passed = match axis {
                CurrentValidationAxis::Flux => evidence_axis_passed(
                    CurrentValidationAxis::Flux,
                    &data.current_flux_validation,
                    data.current_flux_validation_count,
                ),
                CurrentValidationAxis::Torque => evidence_axis_passed(
                    CurrentValidationAxis::Torque,
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
                        data.final_status = CommissionStatus::Accepted;
                    }
                }
            } else {
                let (samples, count) = match axis {
                    CurrentValidationAxis::Flux => (
                        &data.current_flux_validation,
                        data.current_flux_validation_count,
                    ),
                    CurrentValidationAxis::Torque => (
                        &data.current_torque_validation,
                        data.current_torque_validation_count,
                    ),
                };
                let failure_reason = axis_failure_reason_for_samples(axis, samples, count);
                let retry_decision =
                    axis_retry_decision_for_samples(axis, samples, count, axis_attempt(data, axis));
                match retry_decision {
                    CurrentAxisRetryDecision::RetryHigherBandwidth
                    | CurrentAxisRetryDecision::RetryLowerBandwidth => {
                        data.gains = retry_current_loop_gains(data.gains, retry_decision, _profile);
                        let next_attempt = axis_attempt(data, axis).saturating_add(1);
                        data.candidate_gains = data.gains;
                        data.candidate_attempt = next_attempt;
                        data.flux_attempt = data.flux_attempt.max(next_attempt);
                        data.torque_attempt = data.torque_attempt.max(next_attempt);
                        *validate_state = CurrentValidateState::WriteGains;
                        return StepResult::Continue;
                    }
                    CurrentAxisRetryDecision::RetryBudgetExhausted
                    | CurrentAxisRetryDecision::Fail => {
                        let retry_exhausted =
                            retry_decision == CurrentAxisRetryDecision::RetryBudgetExhausted;
                        let mapped_failure_reason = if retry_exhausted {
                            crate::autotune::CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED
                        } else {
                            failure_reason
                        };
                        data.final_status =
                            CommissionStatus::Failed(CommissionError::CurrentValidationFailed);
                        data.current_loop_evidence = failed_axis_failure_evidence(
                            data.current_loop_evidence,
                            mapped_failure_reason,
                            retry_exhausted,
                        );
                        data.gains = disabled_state_safe_current_gains();
                    }
                }
            }

            // Restore safe state.
            let safe_restore_result =
                restore_current_validation_safe_state(backend, data.gains).await;
            if safe_restore_result.is_err()
                && data.final_status
                    != CommissionStatus::Failed(CommissionError::CurrentValidationFailed)
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

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
            let alignment = adc_cal.as_ref();
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
                candidate_gains_source: data.current_loop_evidence.candidate_gains_source,
                candidate_axis_split_source: data.current_loop_evidence.candidate_axis_split_source,
                candidate_gains_tier: data.current_loop_evidence.candidate_gains_tier,
                candidate_measured_axis_split_permille: data
                    .current_loop_evidence
                    .candidate_measured_axis_split_permille,
                candidate_applied_axis_split_permille: data
                    .current_loop_evidence
                    .candidate_applied_axis_split_permille,
                candidate_axis_split_clamped: data
                    .current_loop_evidence
                    .candidate_axis_split_clamped,
                candidate_flux_p: data.candidate_gains.flux_p,
                candidate_flux_i: data.candidate_gains.flux_i,
                candidate_torque_p: data.candidate_gains.torque_p,
                candidate_torque_i: data.candidate_gains.torque_i,
                candidate_attempt: data.candidate_attempt,
                lambda_us: data.gains.lambda_us,
                theta_e_us: data.model.theta_e_us,
                ringing_count: 0,
                bandwidth_hz: data.gains.effective_bandwidth_hz,
                tau_e_us: data.model.tau_e_us,
                inductance_fit: data.inductance_fit,
                inductance_fit_points: data.inductance_fit_points,
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
                encoder_alignment_count: alignment.map_or(0, |result| result.encoder_count),
                encoder_alignment_electrical_residual_counts: alignment.map_or(0, |result| {
                    result.encoder_alignment_electrical_residual_counts
                }),
                encoder_alignment_stability_counts: alignment
                    .map_or(0, |result| result.encoder_alignment_stability_counts),
                encoder_alignment_movement_counts: alignment
                    .map_or(0, |result| result.encoder_alignment_movement_counts),
                encoder_alignment_min_movement_counts: alignment
                    .map_or(0, |result| result.encoder_alignment_min_movement_counts),
                encoder_alignment_counts_per_electrical_rev: alignment.map_or(0, |result| {
                    result.encoder_alignment_counts_per_electrical_rev
                }),
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
                current_flux_validation_count: data.current_flux_validation_count,
                current_flux_validation: data.current_flux_validation,
                current_torque_validation_count: data.current_torque_validation_count,
                current_torque_validation: data.current_torque_validation,
            };

            match sequence {
                InnerSequence::Commission => {
                    if matches!(
                        result.status,
                        CommissionStatus::Failed(CommissionError::CurrentValidationFailed)
                    ) {
                        return StepResult::CommissionDone(CommissionResult {
                            inner: result,
                            fallback_velocity_p: 0,
                            fallback_velocity_i: 0,
                            fallback_position_p: 0,
                            fallback_position_i: 0,
                            fallback_velocity_limit: 0,
                        });
                    }

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
        AXIS_SPLIT_SOURCE_NONE, CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
        CURRENT_GAINS_TIER_NONE, CURRENT_LOOP_FAILURE_NONE, CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED,
        CURRENT_LOOP_FAILURE_STATUS_FLAGS, CURRENT_LOOP_FAILURE_TORQUE_VALIDATION,
        CURRENT_VALIDATION_AXIS_FLUX, CURRENT_VALIDATION_AXIS_TORQUE,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use foci_tmc4671::registers::{
        ABN_DECODER_PPR, INTERIM_FOC_UQ_UD_LIMITED, MODE_RAMP_MODE_MOTION, PHI_E_SELECTION,
        PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM, PID_FLUX_P_FLUX_I,
        PID_TORQUE_FLUX_ACTUAL, PID_TORQUE_FLUX_TARGET, PID_TORQUE_P_TORQUE_I, PIDOUT_UQ_UD_LIMITS,
        PWM_SV_CHOP, STATUS_FLAGS, UQ_UD_EXT,
    };

    use super::*;

    #[test]
    fn failed_evidence_reports_failed_gains_and_validation_failure() {
        let evidence = failed_current_loop_evidence(CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, false);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_FAILED);
        assert_eq!(evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(evidence.gains_tier, CURRENT_GAINS_TIER_NONE);
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
    fn failed_evidence_reports_retry_exhaustion_after_retry_attempt() {
        let evidence = failed_current_loop_evidence(CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED, true);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_FAILED);
        assert_eq!(evidence.current_validation_axes, 0);
        assert_eq!(evidence.retry_budget_exhausted, 1);
        assert_eq!(
            evidence.failure_reason,
            CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED
        );
    }

    #[test]
    fn current_tune_uses_measured_symmetric_phase1_evidence() {
        pollster::block_on(async {
            let mut phase = InnerPhase::CurrentTune {
                model: ElectricalModel {
                    r_int: 1000,
                    l_int: 2000,
                    tau_e_us: 2000,
                    theta_e_us: 100,
                    r_mohm: 1000,
                    l_uh: 2000,
                },
                inner_warning_flags: 0,
                resistance_result: resistance_result(),
                inductance_fit: [InductanceFitEvidence::EMPTY; 2],
                inductance_fit_points: [[InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT]; 2],
            };
            let mut backend = MockBackend::new();
            backend.current_limit_value = 4000;

            let result =
                step_current_tune(&mut phase, CommissionProfile::Balanced, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::CurrentValidate)
            ));
            match phase {
                InnerPhase::CurrentValidate { data, .. } => {
                    assert_eq!(
                        data.current_loop_evidence.axis_split_source,
                        AXIS_SPLIT_SOURCE_NONE
                    );
                    assert_eq!(
                        data.current_loop_evidence.gains_tier,
                        CURRENT_GAINS_TIER_MEASURED_SYMMETRIC
                    );
                    assert!(
                        backend.spi_log.iter().all(|op| matches!(op, SpiOp::Read)),
                        "current tune should not run power-stage writes"
                    );
                }
                other => panic!("expected CurrentValidate phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn validation_failure_failed_evidence_preserves_prior_axis_coverage() {
        let mut previous = CurrentLoopEvidence::measured_symmetric();
        previous.current_validation_axes = CURRENT_VALIDATION_AXIS_FLUX;

        let evidence =
            failed_axis_failure_evidence(previous, CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, false);

        assert_eq!(evidence.gains_source, CURRENT_GAINS_SOURCE_FAILED);
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
    fn demoted_short_response_does_not_trigger_retry() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 1;
        samples[0].status_flags_or = 0x7000_0080;
        samples[1] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[1].sample_delay_ms = 2;
        samples[1].status_flags_or = 0x7000_0080;

        let decision =
            axis_retry_decision_for_samples(CurrentValidationAxis::Torque, &samples, 2, 0);

        assert_eq!(decision, CurrentAxisRetryDecision::Fail);
    }

    #[test]
    fn flux_five_ms_response_floor_is_bandwidth_evidence() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 1;
        samples[1] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[1].sample_delay_ms = 2;
        samples[2] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[2].sample_delay_ms = 5;
        samples[3] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[3].sample_delay_ms = 100;

        assert!(evidence_axis_passed(
            CurrentValidationAxis::Flux,
            &samples,
            4
        ));
    }

    #[test]
    fn flux_two_ms_response_floor_is_bandwidth_evidence() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 1;
        samples[1] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[1].sample_delay_ms = 2;
        samples[2] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[2].sample_delay_ms = 5;
        samples[3] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[3].sample_delay_ms = 100;

        assert!(evidence_axis_passed(
            CurrentValidationAxis::Flux,
            &samples,
            4
        ));
    }

    #[test]
    fn torque_two_ms_response_floor_is_bandwidth_evidence() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 1;
        samples[1] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[1].sample_delay_ms = 2;

        assert!(evidence_axis_passed(
            CurrentValidationAxis::Torque,
            &samples,
            2
        ));
    }

    #[test]
    fn torque_one_ms_response_floor_is_bandwidth_evidence() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[0].sample_delay_ms = 1;
        samples[1] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
        samples[1].sample_delay_ms = 2;

        assert!(evidence_axis_passed(
            CurrentValidationAxis::Torque,
            &samples,
            2
        ));

        let decision =
            axis_retry_decision_for_samples(CurrentValidationAxis::Torque, &samples, 2, 0);
        assert_eq!(decision, CurrentAxisRetryDecision::Fail);
    }

    #[test]
    fn torque_two_ms_safety_fault_still_blocks_validation() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 1;
        samples[1] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS);
        samples[1].sample_delay_ms = 2;

        assert!(!evidence_axis_passed(
            CurrentValidationAxis::Torque,
            &samples,
            2
        ));
    }

    #[test]
    fn flux_short_cross_axis_is_telemetry_but_settled_cross_blocks() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 1;
        samples[1] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS);
        samples[1].sample_delay_ms = 2;
        samples[2] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS);
        samples[2].sample_delay_ms = 5;
        samples[3] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[3].sample_delay_ms = 100;

        assert!(evidence_axis_passed(
            CurrentValidationAxis::Flux,
            &samples,
            4
        ));

        samples[3] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS);
        samples[3].sample_delay_ms = 100;

        assert!(!evidence_axis_passed(
            CurrentValidationAxis::Flux,
            &samples,
            4
        ));
    }

    #[test]
    fn torque_cross_axis_is_telemetry_at_all_validation_delays() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 0;
        samples[1] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS);
        samples[1].sample_delay_ms = 1;
        samples[2] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS);
        samples[2].sample_delay_ms = 2;

        assert!(evidence_axis_passed(
            CurrentValidationAxis::Torque,
            &samples,
            3
        ));

        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS);
        samples[0].sample_delay_ms = 0;

        assert!(evidence_axis_passed(
            CurrentValidationAxis::Torque,
            &samples,
            3
        ));
    }

    #[test]
    fn demoted_gate_windows_preserve_safety_failures() {
        for status in [
            crate::autotune::CURRENT_AXIS_VALIDATION_WRONG_SIGN,
            crate::autotune::CURRENT_AXIS_VALIDATION_SATURATION,
            crate::autotune::CURRENT_AXIS_VALIDATION_MOTION,
            crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS,
        ] {
            let mut flux_sample = axis_validation_evidence(status);
            flux_sample.sample_delay_ms = 2;
            assert_eq!(
                axis_pass_status(CurrentValidationAxis::Flux, &flux_sample),
                status
            );

            let mut torque_sample = axis_validation_evidence(status);
            torque_sample.sample_delay_ms = 2;
            assert_eq!(
                axis_pass_status(CurrentValidationAxis::Torque, &torque_sample),
                status
            );
        }
    }

    #[test]
    fn flux_five_ms_safety_fault_still_blocks_validation() {
        let mut samples = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        samples[0] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[0].sample_delay_ms = 1;
        samples[1] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[1].sample_delay_ms = 2;
        samples[2] =
            axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_STATUS_FLAGS);
        samples[2].sample_delay_ms = 5;
        samples[3] = axis_validation_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        samples[3].sample_delay_ms = 100;

        assert!(!evidence_axis_passed(
            CurrentValidationAxis::Flux,
            &samples,
            4
        ));
    }

    #[test]
    fn successful_retry_is_evidence_not_commission_warning() {
        pollster::block_on(async {
            let mut phase = passed_after_torque_retry_phase();
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_current_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                &None,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            match phase {
                InnerPhase::CurrentValidate { state, data } => {
                    assert!(matches!(state, CurrentValidateState::SafeState { .. }));
                    assert_eq!(data.final_status, CommissionStatus::Accepted);
                    assert_eq!(data.inner_warning_flags & (1 << 4), 0);
                    assert_eq!(data.current_flux_validation[0].attempt, 1);
                    assert_eq!(data.current_torque_validation[0].attempt, 1);
                    assert_eq!(
                        data.current_loop_evidence.current_validation_axes,
                        CURRENT_VALIDATION_AXIS_FLUX | CURRENT_VALIDATION_AXIS_TORQUE
                    );
                    assert_eq!(
                        data.current_loop_evidence.failure_reason,
                        CURRENT_LOOP_FAILURE_NONE
                    );
                }
                other => panic!("expected CurrentValidate phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn failed_current_validation_finalize_returns_commission_result_evidence() {
        pollster::block_on(async {
            let mut phase = failed_current_validate_phase();
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            let adc_cal = Some(crate::commissioning::types::AdcCalResult {
                adc_i0: 0,
                adc_i1: 0,
                encoder_count: 163,
                encoder_alignment_electrical_residual_counts: 3,
                encoder_alignment_stability_counts: 1,
                encoder_alignment_movement_counts: 37,
                encoder_alignment_min_movement_counts: 2,
                encoder_alignment_counts_per_electrical_rev: 80,
            });

            let result = step_current_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                &adc_cal,
                &mut backend,
            )
            .await;

            match result {
                StepResult::CommissionDone(result) => {
                    assert_eq!(
                        result.inner.status,
                        CommissionStatus::Failed(CommissionError::CurrentValidationFailed)
                    );
                    assert_eq!(
                        result.inner.current_gains_source,
                        crate::autotune::CURRENT_GAINS_SOURCE_FAILED
                    );
                    assert_eq!(
                        result.inner.current_loop_failure_reason,
                        CURRENT_LOOP_FAILURE_TORQUE_VALIDATION
                    );
                    assert_eq!(
                        result.inner.current_validation_axes,
                        CURRENT_VALIDATION_AXIS_FLUX
                    );
                    assert_eq!(result.inner.encoder_alignment_count, 163);
                    assert_eq!(result.inner.encoder_alignment_electrical_residual_counts, 3);
                    assert_eq!(result.inner.encoder_alignment_stability_counts, 1);
                    assert_eq!(result.inner.encoder_alignment_counts_per_electrical_rev, 80);
                }
                other => panic!("expected failed CommissionDone, got {other:?}"),
            }
        });
    }

    #[test]
    fn safe_state_restore_writes_current_loop_sequence_in_order() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            let gains = CurrentGains {
                flux_p: 11,
                flux_i: 12,
                torque_p: 13,
                torque_i: 14,
                effective_bandwidth_hz: 0,
                lambda_us: 0,
            };

            restore_current_validation_safe_state(&mut backend, gains)
                .await
                .unwrap();

            let expected = [
                SpiOp::Write(PID_TORQUE_FLUX_TARGET.addr(), 0),
                SpiOp::Write(
                    PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr as u32,
                ),
                SpiOp::Write(PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
                SpiOp::Write(
                    PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr as u32,
                ),
                SpiOp::Write(PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
                SpiOp::Write(PID_FLUX_P_FLUX_I.addr(), (11 << 16) | 12),
                SpiOp::Write(PID_TORQUE_P_TORQUE_I.addr(), (13 << 16) | 14),
                SpiOp::Write(PIDOUT_UQ_UD_LIMITS.addr(), CURRENT_SAFE_PIDOUT_LIMIT),
                SpiOp::Write(MODE_RAMP_MODE_MOTION.addr(), 0),
                SpiOp::Write(UQ_UD_EXT.addr(), 0),
                SpiOp::EnablePin(false),
            ];

            assert_eq!(backend.spi_log.len(), expected.len());
            for (actual, expected) in backend.spi_log.iter().zip(expected.iter()) {
                assert_spi_op_eq(*actual, *expected);
            }
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn write_gains_restores_current_validation_drive_frame() {
        pollster::block_on(async {
            let mut phase = failed_current_validate_phase();
            if let InnerPhase::CurrentValidate { state, data } = &mut phase {
                *state = CurrentValidateState::WriteGains;
                data.gains = CurrentGains {
                    flux_p: 11,
                    flux_i: 12,
                    torque_p: 13,
                    torque_i: 14,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                };
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_current_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                &None,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            assert!(backend.spi_log.iter().any(|op| {
                matches!(op, SpiOp::Write(addr, value) if *addr == PWM_SV_CHOP.addr() && *value == 7)
            }));
            assert!(backend.spi_log.iter().any(|op| {
                matches!(op, SpiOp::Write(addr, value) if *addr == PHI_E_SELECTION.addr() && *value == 3)
            }));
            assert!(matches!(
                phase,
                InnerPhase::CurrentValidate {
                    state: CurrentValidateState::AxisStepSetup {
                        axis: CurrentValidationAxis::Flux,
                        sample_index: 0,
                        direction: CaptureDirection::Forward,
                        ..
                    },
                    ..
                }
            ));
        });
    }

    #[test]
    fn current_validation_runs_flux_then_torque_and_records_axis_evidence() {
        #[derive(Clone, Copy)]
        enum Action {
            StepSetup {
                axis: CurrentValidationAxis,
                direction: CaptureDirection,
                sample_delay_ms: u16,
                step_amplitude: u16,
                flux_count: u8,
                torque_count: u8,
                validation_axes: u8,
            },
            SampleDelay {
                deadline_us: u64,
            },
            SampleRead {
                axis: CurrentValidationAxis,
                direction: CaptureDirection,
            },
            Other,
            SafeState,
        }

        pollster::block_on(async {
            let mut phase = current_validate_phase();
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);
            let mut first_axis = None;
            let mut saw_flux_setup = false;
            let mut saw_torque_after_flux_pass = false;

            for _ in 0..120 {
                let action = match &phase {
                    InnerPhase::CurrentValidate {
                        state:
                            CurrentValidateState::AxisStepSetup {
                                axis,
                                direction,
                                sample_delay_ms,
                                ..
                            },
                        data,
                    } => Action::StepSetup {
                        axis: *axis,
                        direction: *direction,
                        sample_delay_ms: *sample_delay_ms,
                        step_amplitude: data.step_amplitude,
                        flux_count: data.current_flux_validation_count,
                        torque_count: data.current_torque_validation_count,
                        validation_axes: data.current_loop_evidence.current_validation_axes,
                    },
                    InnerPhase::CurrentValidate {
                        state: CurrentValidateState::AxisSampleDelay { deadline_us, .. },
                        ..
                    } => Action::SampleDelay {
                        deadline_us: *deadline_us,
                    },
                    InnerPhase::CurrentValidate {
                        state: CurrentValidateState::AxisSampleSettle { deadline_us, .. },
                        ..
                    } => Action::SampleDelay {
                        deadline_us: *deadline_us,
                    },
                    InnerPhase::CurrentValidate {
                        state:
                            CurrentValidateState::AxisSampleRead {
                                axis, direction, ..
                            },
                        ..
                    } => Action::SampleRead {
                        axis: *axis,
                        direction: *direction,
                    },
                    InnerPhase::CurrentValidate {
                        state: CurrentValidateState::SafeState { .. },
                        ..
                    } => Action::SafeState,
                    InnerPhase::CurrentValidate { .. } => Action::Other,
                    other => panic!("expected CurrentValidate phase, got {other:?}"),
                };

                match action {
                    Action::StepSetup {
                        axis,
                        direction,
                        sample_delay_ms,
                        step_amplitude,
                        flux_count,
                        torque_count,
                        validation_axes,
                    } => {
                        if first_axis.is_none() {
                            first_axis = Some(axis);
                        }
                        match axis {
                            CurrentValidationAxis::Flux => {
                                saw_flux_setup = true;
                                assert_eq!(torque_count, 0);
                            }
                            CurrentValidationAxis::Torque => {
                                assert!(saw_flux_setup);
                                assert_eq!(flux_count, 4);
                                if !saw_torque_after_flux_pass {
                                    assert_eq!(torque_count, 0);
                                }
                                assert_eq!(
                                    validation_axes & CURRENT_VALIDATION_AXIS_FLUX,
                                    CURRENT_VALIDATION_AXIS_FLUX
                                );
                                saw_torque_after_flux_pass = true;
                            }
                        }

                        if sample_delay_ms == 0 {
                            seed_axis_sample_registers(&mut backend, axis, direction);
                        }
                        backend.spi_log.clear();
                        let result = step_current_validate(
                            &mut phase,
                            &mut buf,
                            CommissionProfile::Balanced,
                            InnerSequence::Commission,
                            &None,
                            &mut backend,
                        )
                        .await;
                        assert!(matches!(result, StepResult::Continue));

                        let signed_target = match direction {
                            CaptureDirection::Forward => step_amplitude as i16,
                            CaptureDirection::Reverse => -(step_amplitude as i16),
                        };
                        let target_write = nonzero_validation_target_write(&backend.spi_log);
                        assert_eq!(
                            target_write,
                            validation_target_register(axis, signed_target)
                        );
                        match axis {
                            CurrentValidationAxis::Flux => assert_eq!(target_write >> 16, 0),
                            CurrentValidationAxis::Torque => assert_eq!(target_write & 0xffff, 0),
                        }
                    }
                    Action::SampleDelay { deadline_us } => {
                        backend.elapsed_us_value = deadline_us;
                        backend.spi_log.clear();
                        let result = step_current_validate(
                            &mut phase,
                            &mut buf,
                            CommissionProfile::Balanced,
                            InnerSequence::Commission,
                            &None,
                            &mut backend,
                        )
                        .await;
                        assert!(matches!(result, StepResult::Continue));
                    }
                    Action::SampleRead { axis, direction } => {
                        if axis == CurrentValidationAxis::Torque {
                            assert!(saw_torque_after_flux_pass);
                        }
                        seed_axis_sample_registers(&mut backend, axis, direction);
                        backend.spi_log.clear();
                        let result = step_current_validate(
                            &mut phase,
                            &mut buf,
                            CommissionProfile::Balanced,
                            InnerSequence::Commission,
                            &None,
                            &mut backend,
                        )
                        .await;
                        assert!(matches!(result, StepResult::Continue));
                    }
                    Action::Other => {
                        backend.spi_log.clear();
                        let result = step_current_validate(
                            &mut phase,
                            &mut buf,
                            CommissionProfile::Balanced,
                            InnerSequence::Commission,
                            &None,
                            &mut backend,
                        )
                        .await;
                        assert!(matches!(result, StepResult::Continue));
                    }
                    Action::SafeState => break,
                }
            }

            assert_eq!(first_axis, Some(CurrentValidationAxis::Flux));
            assert!(saw_torque_after_flux_pass);
            match phase {
                InnerPhase::CurrentValidate {
                    state: CurrentValidateState::SafeState { .. },
                    data,
                } => {
                    assert_eq!(
                        data.current_loop_evidence.current_validation_axes,
                        CURRENT_VALIDATION_AXIS_FLUX | CURRENT_VALIDATION_AXIS_TORQUE
                    );
                    assert_validation_evidence_delays(
                        &data.current_flux_validation,
                        data.current_flux_validation_count,
                        &[1, 2, 5, 100],
                    );
                    assert_validation_evidence_delays(
                        &data.current_torque_validation,
                        data.current_torque_validation_count,
                        &[0, 1, 2],
                    );
                }
                other => panic!("expected validation SafeState, got {other:?}"),
            }
        });
    }

    #[test]
    fn safe_state_restore_continues_after_early_spi_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.fail_on_write = Some(PID_TORQUE_FLUX_TARGET.addr());
            let gains = CurrentGains {
                flux_p: 11,
                flux_i: 12,
                torque_p: 13,
                torque_i: 14,
                effective_bandwidth_hz: 0,
                lambda_us: 0,
            };

            let result = restore_current_validation_safe_state(&mut backend, gains).await;

            assert!(result.is_err());
            assert!(!backend.enable_pin);
            assert!(backend.spi_log.iter().any(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, value)
                        if *addr == PID_FLUX_P_FLUX_I.addr() && *value == ((11 << 16) | 12)
                )
            }));
            assert!(backend.spi_log.iter().any(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, value)
                        if *addr == PIDOUT_UQ_UD_LIMITS.addr()
                            && *value == CURRENT_SAFE_PIDOUT_LIMIT
                )
            }));
            assert!(backend.spi_log.iter().any(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, value)
                        if *addr == UQ_UD_EXT.addr() && *value == 0
                )
            }));
            assert!(matches!(
                backend.spi_log.last(),
                Some(SpiOp::EnablePin(false))
            ));
        });
    }

    #[test]
    fn axis_step_setup_clears_status_flags_before_validation_target() {
        pollster::block_on(async {
            let mut phase = failed_current_validate_phase();
            if let InnerPhase::CurrentValidate { state, data } = &mut phase {
                *state = CurrentValidateState::AxisStepSetup {
                    axis: CurrentValidationAxis::Flux,
                    sample_index: 0,
                    sample_delay_ms: 1,
                    direction: CaptureDirection::Forward,
                };
                data.step_amplitude = 250;
            }
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_current_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                &None,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            let status_clear_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::Write(addr, 0) if *addr == STATUS_FLAGS.addr()))
                .expect("validation setup should clear latched status flags");
            let target_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, value)
                            if *addr == PID_TORQUE_FLUX_TARGET.addr() && *value == 250
                    )
                })
                .expect("validation setup should write the sample target");

            assert!(status_clear_index < target_index);
        });
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
                encoder_counts_per_rev: 4000,
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
                encoder_counts_per_rev: 4000,
                status_flags: 0,
            },
        );

        assert_eq!(sample.same_axis_actual, -240);
        assert_eq!(sample.cross_axis_actual, 25);
        assert_eq!(sample.output_limited, -300);
        assert_eq!(sample.encoder_delta_counts, -2);
    }

    #[test]
    fn current_validation_encoder_delta_uses_shortest_wrap_distance() {
        assert_eq!(encoder_delta_counts(3998, 2, 4000), 4);
        assert_eq!(encoder_delta_counts(2, 3998, 4000), -4);
        assert_eq!(encoder_delta_counts(100, 108, 4000), 8);
        assert_eq!(encoder_delta_counts(108, 100, 4000), -8);
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

    #[test]
    fn combine_axis_pair_evidence_preserves_signed_encoder_deltas() {
        let mut positive = axis_directional_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        positive.encoder_delta_counts = 7;
        positive.gate.encoder_delta_counts = 7;
        let mut negative = axis_directional_evidence(crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
        negative.encoder_delta_counts = -3;
        negative.gate.encoder_delta_counts = 3;

        let evidence = combine_axis_pair_evidence(0, 250, 2, positive, negative);

        assert_eq!(evidence.encoder_delta_counts, 7);
        assert_eq!(evidence.positive_encoder_delta_counts, 7);
        assert_eq!(evidence.negative_encoder_delta_counts, -3);
    }

    #[test]
    fn settled_flux_evidence_averages_single_cross_spike_and_reports_peak() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let _ = backend
                .registers
                .insert(INTERIM_FOC_UQ_UD_LIMITED.data.addr(), pack_halves(0, 100));
            let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0);
            backend.set_pid_torque_flux_actual_sequence(&[
                pack_halves(45, 250),
                pack_halves(5, 250),
                pack_halves(5, 250),
                pack_halves(5, 250),
            ]);

            let evidence = read_axis_directional_evidence_for_gate(
                &mut backend,
                CurrentValidationAxis::Flux,
                250,
                100,
                0,
                1024,
                4000,
            )
            .await
            .expect("settled flux averaging should read cleanly");

            assert_eq!(
                evidence.gate.status,
                crate::autotune::CURRENT_AXIS_VALIDATION_PASS
            );
            assert_eq!(evidence.gate.response_permille, 1000);
            assert_eq!(evidence.gate.cross_axis_permille, 60);
            assert_eq!(evidence.cross_axis_peak_permille, 180);
            assert_eq!(backend.pid_torque_flux_actual_pos, 4);
            assert_eq!(backend.delay_us_log.as_slice(), &[250, 250, 250]);
        });
    }

    #[test]
    fn settled_flux_evidence_fails_when_average_cross_stays_high() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let _ = backend
                .registers
                .insert(INTERIM_FOC_UQ_UD_LIMITED.data.addr(), pack_halves(0, 100));
            let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0);
            backend.set_pid_torque_flux_actual_sequence(&[
                pack_halves(45, 250),
                pack_halves(45, 250),
                pack_halves(45, 250),
                pack_halves(45, 250),
            ]);

            let evidence = read_axis_directional_evidence_for_gate(
                &mut backend,
                CurrentValidationAxis::Flux,
                250,
                100,
                0,
                1024,
                4000,
            )
            .await
            .expect("settled flux averaging should read cleanly");

            assert_eq!(
                evidence.gate.status,
                crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS
            );
            assert_eq!(evidence.gate.cross_axis_permille, 180);
            assert_eq!(evidence.cross_axis_peak_permille, 180);
        });
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
            encoder_delta_counts: 0,
            cross_axis_peak_permille: 0,
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

    fn nonzero_validation_target_write(log: &[SpiOp]) -> u32 {
        let mut found = None;
        for op in log {
            match *op {
                SpiOp::Write(addr, value)
                    if addr == PID_TORQUE_FLUX_TARGET.addr() && value != 0 =>
                {
                    assert!(
                        found.is_none(),
                        "expected one non-zero validation target write"
                    );
                    found = Some(value);
                }
                _ => {}
            }
        }
        found.expect("validation setup should write a non-zero current target")
    }

    fn seed_axis_sample_registers(
        backend: &mut MockBackend,
        axis: CurrentValidationAxis,
        direction: CaptureDirection,
    ) {
        let same_axis = match direction {
            CaptureDirection::Forward => 225,
            CaptureDirection::Reverse => -225,
        };
        let cross_axis = match direction {
            CaptureDirection::Forward => 5,
            CaptureDirection::Reverse => -5,
        };
        let output_limited = match direction {
            CaptureDirection::Forward => 100,
            CaptureDirection::Reverse => -100,
        };
        let pid_actual = match axis {
            CurrentValidationAxis::Flux => pack_halves(cross_axis, same_axis),
            CurrentValidationAxis::Torque => pack_halves(same_axis, cross_axis),
        };
        let foc_limited = match axis {
            CurrentValidationAxis::Flux => pack_halves(0, output_limited),
            CurrentValidationAxis::Torque => pack_halves(output_limited, 0),
        };

        let _ = backend
            .registers
            .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pid_actual);
        let _ = backend
            .registers
            .insert(INTERIM_FOC_UQ_UD_LIMITED.data.addr(), foc_limited);
        let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0);
    }

    fn assert_validation_evidence_delays(
        evidence: &[CurrentValidationAxisEvidence],
        count: u8,
        expected_delays: &[u16],
    ) {
        assert_eq!(count as usize, expected_delays.len());
        for (sample, expected_delay) in evidence.iter().zip(expected_delays.iter()) {
            assert_eq!(sample.status, crate::autotune::CURRENT_AXIS_VALIDATION_PASS);
            assert_eq!(sample.sample_delay_ms, *expected_delay);
        }
    }

    #[test]
    fn torque_zero_delay_response_magnitude_is_geometry_telemetry() {
        let mut sample = CurrentValidationAxisEvidence {
            status: crate::autotune::CURRENT_AXIS_VALIDATION_WRONG_SIGN,
            sample_delay_ms: 0,
            ..CurrentValidationAxisEvidence::EMPTY
        };

        assert_eq!(
            axis_pass_status(CurrentValidationAxis::Torque, &sample),
            crate::autotune::CURRENT_AXIS_VALIDATION_PASS
        );

        sample.status = crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE;
        assert_eq!(
            axis_pass_status(CurrentValidationAxis::Torque, &sample),
            crate::autotune::CURRENT_AXIS_VALIDATION_PASS
        );

        sample.status = crate::autotune::CURRENT_AXIS_VALIDATION_CROSS_AXIS;
        assert_eq!(
            axis_pass_status(CurrentValidationAxis::Torque, &sample),
            crate::autotune::CURRENT_AXIS_VALIDATION_PASS
        );

        sample.status = crate::autotune::CURRENT_AXIS_VALIDATION_SATURATION;
        assert_eq!(
            axis_pass_status(CurrentValidationAxis::Torque, &sample),
            crate::autotune::CURRENT_AXIS_VALIDATION_SATURATION
        );
    }

    fn current_validate_phase() -> InnerPhase {
        InnerPhase::CurrentValidate {
            state: CurrentValidateState::WriteGains,
            data: CurrentValidateData {
                model: ElectricalModel {
                    r_int: 1000,
                    l_int: 2000,
                    tau_e_us: 2000,
                    theta_e_us: 100,
                    r_mohm: 1000,
                    l_uh: 2000,
                },
                gains: CurrentGains {
                    flux_p: 11,
                    flux_i: 12,
                    torque_p: 13,
                    torque_i: 14,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                },
                candidate_gains: CurrentGains {
                    flux_p: 11,
                    flux_i: 12,
                    torque_p: 13,
                    torque_i: 14,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                },
                candidate_attempt: 0,
                flux_attempt: 0,
                torque_attempt: 0,
                step_amplitude: 250,
                final_status: CommissionStatus::Accepted,
                pidout_limit: CURRENT_SAFE_PIDOUT_LIMIT,
                encoder_counts_per_rev: 4000,
                sample_encoder_start: 0,
                pending_positive_sample: None,
                current_flux_validation_count: 0,
                current_flux_validation: [CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
                current_torque_validation_count: 0,
                current_torque_validation: [CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
                inner_warning_flags: 0,
                inductance_fit: [InductanceFitEvidence::EMPTY; 2],
                inductance_fit_points: [[InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT]; 2],
                resistance_result: resistance_result(),
                resistance_gain_path_count_slope_milli: 1000,
                current_loop_evidence: CurrentLoopEvidence::measured_symmetric(),
            },
        }
    }

    fn failed_current_validate_phase() -> InnerPhase {
        let mut evidence = CurrentLoopEvidence::failed(CURRENT_LOOP_FAILURE_TORQUE_VALIDATION);
        evidence.candidate_gains_source = crate::autotune::CURRENT_GAINS_SOURCE_MEASURED;
        evidence.candidate_gains_tier = crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC;
        evidence.current_validation_axes = CURRENT_VALIDATION_AXIS_FLUX;
        InnerPhase::CurrentValidate {
            state: CurrentValidateState::Finalize,
            data: CurrentValidateData {
                model: ElectricalModel {
                    r_int: 1000,
                    l_int: 2000,
                    tau_e_us: 2000,
                    theta_e_us: 100,
                    r_mohm: 1000,
                    l_uh: 2000,
                },
                gains: disabled_state_safe_current_gains(),
                candidate_gains: CurrentGains {
                    flux_p: 11,
                    flux_i: 12,
                    torque_p: 13,
                    torque_i: 14,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                },
                candidate_attempt: 1,
                flux_attempt: 1,
                torque_attempt: 1,
                step_amplitude: 250,
                final_status: CommissionStatus::Failed(CommissionError::CurrentValidationFailed),
                pidout_limit: CURRENT_SAFE_PIDOUT_LIMIT,
                encoder_counts_per_rev: 4000,
                sample_encoder_start: 0,
                pending_positive_sample: None,
                current_flux_validation_count: 1,
                current_flux_validation: [CurrentValidationAxisEvidence {
                    status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
                    attempt: 1,
                    target: 250,
                    sample_delay_ms: 2,
                    positive_response_permille: 980,
                    negative_response_permille: 970,
                    cross_axis_permille: 10,
                    cross_axis_peak_permille: 10,
                    voltage_output_permille: 100,
                    encoder_delta_counts: 0,
                    positive_encoder_delta_counts: 0,
                    negative_encoder_delta_counts: 0,
                    status_flags_or: 0,
                };
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
                current_torque_validation_count: 1,
                current_torque_validation: [CurrentValidationAxisEvidence {
                    status: crate::autotune::CURRENT_AXIS_VALIDATION_LOW_RESPONSE,
                    attempt: 1,
                    target: 250,
                    sample_delay_ms: 2,
                    positive_response_permille: 100,
                    negative_response_permille: 120,
                    cross_axis_permille: 10,
                    cross_axis_peak_permille: 10,
                    voltage_output_permille: 100,
                    encoder_delta_counts: 0,
                    positive_encoder_delta_counts: 0,
                    negative_encoder_delta_counts: 0,
                    status_flags_or: 0,
                };
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
                inner_warning_flags: 0,
                inductance_fit: [InductanceFitEvidence::EMPTY; 2],
                inductance_fit_points: [[InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT]; 2],
                resistance_result: resistance_result(),
                resistance_gain_path_count_slope_milli: 1000,
                current_loop_evidence: evidence,
            },
        }
    }

    fn passed_after_torque_retry_phase() -> InnerPhase {
        let mut evidence = CurrentLoopEvidence::measured_symmetric();
        evidence.current_validation_axes = CURRENT_VALIDATION_AXIS_FLUX;
        InnerPhase::CurrentValidate {
            state: CurrentValidateState::AxisClassify {
                axis: CurrentValidationAxis::Torque,
            },
            data: CurrentValidateData {
                model: ElectricalModel {
                    r_int: 1000,
                    l_int: 2000,
                    tau_e_us: 2000,
                    theta_e_us: 100,
                    r_mohm: 1000,
                    l_uh: 2000,
                },
                gains: CurrentGains {
                    flux_p: 11,
                    flux_i: 12,
                    torque_p: 13,
                    torque_i: 14,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                },
                candidate_gains: CurrentGains {
                    flux_p: 11,
                    flux_i: 12,
                    torque_p: 13,
                    torque_i: 14,
                    effective_bandwidth_hz: 0,
                    lambda_us: 0,
                },
                candidate_attempt: 1,
                flux_attempt: 1,
                torque_attempt: 1,
                step_amplitude: 250,
                final_status: CommissionStatus::Accepted,
                pidout_limit: CURRENT_SAFE_PIDOUT_LIMIT,
                encoder_counts_per_rev: 4000,
                sample_encoder_start: 0,
                pending_positive_sample: None,
                current_flux_validation_count: 1,
                current_flux_validation: [CurrentValidationAxisEvidence {
                    status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
                    attempt: 1,
                    target: 250,
                    sample_delay_ms: 2,
                    positive_response_permille: 980,
                    negative_response_permille: 970,
                    cross_axis_permille: 10,
                    cross_axis_peak_permille: 10,
                    voltage_output_permille: 100,
                    encoder_delta_counts: 0,
                    positive_encoder_delta_counts: 0,
                    negative_encoder_delta_counts: 0,
                    status_flags_or: 0,
                };
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
                current_torque_validation_count: 2,
                current_torque_validation: [CurrentValidationAxisEvidence {
                    status: crate::autotune::CURRENT_AXIS_VALIDATION_PASS,
                    attempt: 1,
                    target: 250,
                    sample_delay_ms: 2,
                    positive_response_permille: 780,
                    negative_response_permille: 680,
                    cross_axis_permille: 50,
                    cross_axis_peak_permille: 50,
                    voltage_output_permille: 450,
                    encoder_delta_counts: 15,
                    positive_encoder_delta_counts: 15,
                    negative_encoder_delta_counts: -8,
                    status_flags_or: 0x7000_8080,
                };
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
                inner_warning_flags: 0,
                inductance_fit: [InductanceFitEvidence::EMPTY; 2],
                inductance_fit_points: [[InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT]; 2],
                resistance_result: resistance_result(),
                resistance_gain_path_count_slope_milli: 1000,
                current_loop_evidence: evidence,
            },
        }
    }

    fn resistance_result() -> crate::resistance_identification::ResistanceRunResult {
        use crate::resistance_identification::{
            ElectricalAxis, ResistanceAxisResult, ResistanceRunResult, ResistanceWarningFlags,
        };

        fn axis_result(axis: ElectricalAxis) -> ResistanceAxisResult {
            ResistanceAxisResult {
                axis,
                r_count_slope_milli: 1000,
                slope_num: 1000,
                slope_den: 1000,
                intercept_count: 0,
                rmse_permille: 0,
                selected_mask: 0b1111_1000,
                excluded_point_mask: 0,
                selected_count: 5,
                signed_anchor: None,
                warning_flags: ResistanceWarningFlags::NONE,
            }
        }

        ResistanceRunResult {
            profile_version: 1,
            selected_r_count_slope_milli: 1000,
            status_flags_or: 0,
            axis0: axis_result(ElectricalAxis::PHASE0),
            axis1: axis_result(ElectricalAxis::PHASE90),
            warning_flags: ResistanceWarningFlags::NONE,
        }
    }

    fn assert_spi_op_eq(actual: SpiOp, expected: SpiOp) {
        match (actual, expected) {
            (SpiOp::Read, SpiOp::Read) => {}
            (
                SpiOp::Write(actual_addr, actual_value),
                SpiOp::Write(expected_addr, expected_value),
            ) => {
                assert_eq!((actual_addr, actual_value), (expected_addr, expected_value));
            }
            (SpiOp::EnablePin(actual), SpiOp::EnablePin(expected)) => {
                assert_eq!(actual, expected);
            }
            (actual, expected) => panic!("expected {expected:?}, got {actual:?}"),
        }
    }
}
