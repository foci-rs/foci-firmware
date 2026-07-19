//! Trace-gated disturbance-identification velocity sweep.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::state::VelocitySweepEvidenceState;
use crate::commissioning::outer::{OuterPhase, VelocitySweepState};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_CURRENT, OUTER_SAFETY_FAULT_OBSERVATION_GAP,
    OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault, StepResult, VelocitySweepEvent,
    VelocitySweepObservation, VelocitySweepOutcome, VelocitySweepPlanned, VelocitySweepRungReport,
    VelocitySweepTerminal,
};
use foci_tmc4671::pid::PidGainPair;
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, CONFIG_VELOCITY_METER_PPTM_MIN_POS_DEV, INTERIM_ACTUAL_VELOCITY_PPTM,
    INTERIM_PIDOUT_TARGET_TORQUE, MODE_RAMP_MODE_MOTION, PID_ERROR_PID_VELOCITY_ERROR_SUM,
    PID_TORQUE_FLUX_TARGET, PID_VELOCITY_ACTUAL, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use super::sweep_analysis::{
    poll_moving_observation_analysis, poll_settle_confirmation, retain_observation,
};
use super::sweep_evidence::VelocityObservationClass;
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US,
    RequestedVelocitySource, SETTLE_ANALYSIS_BUDGET_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US,
    ZERO_SETTLE_US,
};

const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;
const SWEEP_SLOT_COUNT: u8 = 4;
const SETTLE_TRACE_STROKE_MARKER: u16 = 1 << 15;
const SWEEP_CAUSE_NONE: u8 = 0;
const SWEEP_CAUSE_PARTIAL_DIRECTION: u8 = 1;
const SWEEP_CAUSE_TOTAL_INCONCLUSIVE: u8 = 2;
const SWEEP_CAUSE_INTEGRITY: u8 = 3;
const SWEEP_CAUSE_CURRENT_HEADROOM: u8 = 4;
const PID_VELOCITY_ACTUAL_TRACE_DECIMATION: u16 = 16;

#[derive(Clone, Copy)]
enum VelocityDiagnosticKind {
    PidVelocityActual,
    PptmActual,
}

fn slot_direction(slot: u8) -> i32 {
    if slot.is_multiple_of(2) { 1 } else { -1 }
}

fn velocity_diagnostic_kind(
    next_sample_us: u64,
    deadline_us: u64,
) -> Option<VelocityDiagnosticKind> {
    let remaining_sample_slots =
        deadline_us.saturating_sub(next_sample_us) / u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US);
    if !remaining_sample_slots.is_multiple_of(u64::from(PID_VELOCITY_ACTUAL_TRACE_DECIMATION)) {
        return None;
    }
    let diagnostic_slot = remaining_sample_slots / u64::from(PID_VELOCITY_ACTUAL_TRACE_DECIMATION);
    if diagnostic_slot.is_multiple_of(2) {
        Some(VelocityDiagnosticKind::PidVelocityActual)
    } else {
        Some(VelocityDiagnosticKind::PptmActual)
    }
}

async fn fail_sweep<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
    error: CommissionError,
) -> StepResult {
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    sweep_fault_terminal(phase, error, None)
}

async fn fail_sweep_with_safety<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
    fault: OuterSafetyFault,
) -> StepResult {
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    sweep_fault_terminal(phase, CommissionError::SafetyEnvelopeViolation, Some(fault))
}

pub(crate) fn sweep_fault_terminal(
    phase: &OuterPhase,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    let OuterPhase::VelocitySweep { data, .. } = phase else {
        return StepResult::OuterFailed(error);
    };
    sweep_fault_terminal_from_data(data, error, safety_fault)
}

fn sweep_fault_terminal_from_data(
    data: &super::super::VelocitySweepData,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    use super::sweep_evidence::assess_sweep_sufficiency;

    let forward = data.forward_band.snapshot();
    let reverse = data.reverse_band.snapshot();
    let sufficiency = assess_sweep_sufficiency(&forward, &reverse);
    let sufficient_direction_mask =
        u8::from(forward.sufficient) | (u8::from(reverse.sufficient) << 1);
    let (expected_observations, expected_rungs) = data.plan.map_or((0, 0), |plan| {
        (
            u16::from(plan.rung_count)
                .saturating_mul(plan.observations_per_direction)
                .saturating_mul(2),
            u16::from(plan.rung_count),
        )
    });
    StepResult::VelocitySweepTerminal(VelocitySweepTerminal {
        run_sequence: data.integrity.run_sequence(),
        forward,
        reverse,
        sufficiency,
        outcome: VelocitySweepOutcome::Fault,
        cause: error.status_code(),
        sufficient_direction_mask,
        expected_observations,
        emitted_observations: data.integrity.emitted_observations(),
        expected_rungs,
        emitted_rungs: data.integrity.emitted_rungs(),
        digest: data.integrity.digest(),
        safety_fault,
    })
}

fn sweep_evidence_terminal(
    data: &super::super::VelocitySweepData,
    current_headroom: bool,
) -> VelocitySweepTerminal {
    use super::sweep_evidence::{
        VelocityInconclusiveKind, VelocitySweepSufficiency, assess_sweep_sufficiency,
    };

    let plan = data
        .plan
        .expect("velocity sweep terminal requires an immutable plan");
    let forward = data.forward_band.snapshot();
    let reverse = data.reverse_band.snapshot();
    let sufficiency = assess_sweep_sufficiency(&forward, &reverse);
    let (mut outcome, mut cause, sufficient_direction_mask) = match sufficiency {
        VelocitySweepSufficiency::Complete { .. } => (
            VelocitySweepOutcome::Complete,
            if current_headroom {
                SWEEP_CAUSE_CURRENT_HEADROOM
            } else {
                SWEEP_CAUSE_NONE
            },
            3,
        ),
        VelocitySweepSufficiency::Inconclusive {
            sufficient_direction_mask,
            kind,
            ..
        } => (
            VelocitySweepOutcome::Inconclusive,
            if current_headroom {
                SWEEP_CAUSE_CURRENT_HEADROOM
            } else {
                match kind {
                    VelocityInconclusiveKind::PartialDirection => SWEEP_CAUSE_PARTIAL_DIRECTION,
                    VelocityInconclusiveKind::Total => SWEEP_CAUSE_TOTAL_INCONCLUSIVE,
                }
            },
            sufficient_direction_mask,
        ),
    };
    let expected_observations = u16::from(plan.rung_count)
        .saturating_mul(plan.observations_per_direction)
        .saturating_mul(2);
    let expected_rungs = u16::from(plan.rung_count);
    if !current_headroom
        && (data.integrity.emitted_observations() != expected_observations
            || data.integrity.emitted_rungs() != expected_rungs)
    {
        outcome = VelocitySweepOutcome::Fault;
        cause = SWEEP_CAUSE_INTEGRITY;
    }
    VelocitySweepTerminal {
        run_sequence: data.integrity.run_sequence(),
        forward,
        reverse,
        sufficiency,
        outcome,
        cause,
        sufficient_direction_mask,
        expected_observations,
        emitted_observations: data.integrity.emitted_observations(),
        expected_rungs,
        emitted_rungs: data.integrity.emitted_rungs(),
        digest: data.integrity.digest(),
        safety_fault: None,
    }
}

async fn clear_velocity_integrator<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), CommissionError> {
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_VELOCITY_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)
}

/// Advance the trace-gated velocity sweep by one cooperative state-machine step.
pub(crate) async fn step_velocity_sweep_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
) -> StepResult {
    let (state, data, safety) = match phase {
        OuterPhase::VelocitySweep {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::ReadyNow,
    };

    match state {
        VelocitySweepState::Setup => {
            if backend.is_shutdown_requested() {
                return sweep_fault_terminal(phase, CommissionError::Shutdown, None);
            }
            let encoder_counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
            let torque_limit = u16::from(backend.current_limit());
            let request = super::sweep_plan::VelocitySweepRequest {
                requested_velocity_mrev_s: safety.budget.requested_velocity_mrev_s,
                machine_velocity_ceiling_mrev_s: safety.budget.machine_velocity_ceiling_mrev_s,
                requested_velocity_source: safety.budget.requested_velocity_source,
                max_stroke_travel_mrev: safety.budget.max_stroke_travel_mrev,
                settle_travel_reserve_mrev: safety.budget.settle_travel_reserve_mrev,
                negative_position_headroom_mrev: safety.budget.negative_position_headroom_mrev,
                positive_position_headroom_mrev: safety.budget.positive_position_headroom_mrev,
                max_duration_ms: safety.budget.max_duration_ms,
            };
            let plan = match super::sweep_plan::plan_velocity_sweep(
                request,
                encoder_counts_per_rev,
                torque_limit,
            ) {
                Ok(plan) => plan,
                Err(_) => {
                    return sweep_fault_terminal(phase, CommissionError::InvalidSchedule, None);
                }
            };
            if backend
                .write_register(
                    CONFIG_VELOCITY_METER_PPTM_MIN_POS_DEV.addr,
                    u32::from(CONFIG_VELOCITY_METER_PPTM_MIN_POS_DEV.sub_addr),
                )
                .await
                .is_err()
            {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            let pptm_min_pos_dev = match backend
                .read_register(CONFIG_VELOCITY_METER_PPTM_MIN_POS_DEV.data)
                .await
            {
                Ok(value) => value,
                Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
            };
            if backend
                .write_register(
                    INTERIM_ACTUAL_VELOCITY_PPTM.addr,
                    u32::from(INTERIM_ACTUAL_VELOCITY_PPTM.sub_addr),
                )
                .await
                .is_err()
            {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            defmt::info!(
                "Velocity sweep read-only PPTM_MIN_POS_DEV={}",
                pptm_min_pos_dev
            );
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return sweep_fault_terminal(phase, CommissionError::Spi, None),
            };
            if safety
                .configure_velocity_sweep_encoder_resolution(encoder_counts_per_rev)
                .and_then(|_| safety.set_encoder_baseline(encoder_raw))
                .is_err()
                || safety
                    .configure_velocity_envelope(plan.planned_velocity_mrev_s)
                    .is_err()
            {
                return sweep_fault_terminal(phase, CommissionError::EncoderFault, None);
            }
            safety.torque_cap = torque_limit;
            if safety.arm_safety_observation(backend.elapsed_us()).is_err() {
                return sweep_fault_terminal(phase, CommissionError::InvalidSchedule, None);
            }
            data.plan = Some(plan);
            *state = VelocitySweepState::EmitPlan;
            StepResult::ReadyNow
        }
        VelocitySweepState::EmitPlan => {
            let Some(plan) = data.plan else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let report = VelocitySweepPlanned {
                run_sequence: data.integrity.run_sequence(),
                evidence_sequence: data.integrity.next_evidence_sequence(),
                plan_digest: plan.digest(),
                requested_velocity_mrev_s: plan.requested_velocity_mrev_s,
                requested_velocity_source: match plan.requested_velocity_source {
                    RequestedVelocitySource::Explicit => 0,
                    RequestedVelocitySource::Defaulted => 1,
                },
                planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
                effective_ceiling_mrev_s: plan.effective_ceiling_mrev_s,
                clamp_flags: plan.clamp_flags,
                effective_ceiling_source: plan.effective_ceiling_source,
                target_velocity_rpm: plan.target_velocity_rpm.raw(),
                p_start: plan.p_start.raw(),
                p_top: plan.p_top.raw(),
                rung_count: plan.rung_count,
                observations_per_direction: plan.observations_per_direction,
                moving_stroke_us: plan.moving_stroke_us,
                zero_settle_us: plan.zero_settle_us,
                nominal_workflow_ms: plan.nominal_workflow_ms,
                maximum_workflow_ms: plan.maximum_workflow_ms,
                max_stroke_travel_mrev: plan.max_stroke_travel_mrev,
                settle_travel_reserve_mrev: plan.settle_travel_reserve_mrev,
                negative_position_headroom_mrev: plan.negative_position_headroom_mrev,
                positive_position_headroom_mrev: plan.positive_position_headroom_mrev,
                hard_torque_limit: plan.hard_torque_limit,
                usable_torque_limit: plan.usable_torque_limit,
            };
            let event = data.integrity.record(VelocitySweepEvent::Planned(report));
            *state = VelocitySweepState::WriteGain;
            StepResult::VelocitySweepEvent(event)
        }
        VelocitySweepState::WriteGain => {
            let Some(plan) = data.plan else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let Some(p) = plan.gains.get(usize::from(data.rung_index)).copied() else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            if data.rung_index >= plan.rung_count
                || backend
                    .write_register(
                        PID_VELOCITY_P_VELOCITY_I,
                        PidGainPair::new(p, foci_tmc4671::pid::PidGain::ZERO).register_value(),
                    )
                    .await
                    .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_MODE)
                    .await
                    .is_err()
            {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            data.evidence = VelocitySweepEvidenceState::Empty;
            data.slot = 0;
            *state = VelocitySweepState::ClearSignedIntegrator;
            StepResult::ReadyNow
        }
        VelocitySweepState::ClearSignedIntegrator => {
            if clear_velocity_integrator(backend).await.is_err() {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            *state = VelocitySweepState::CommandTarget;
            StepResult::ReadyNow
        }
        VelocitySweepState::CommandTarget => {
            let Some(plan) = data.plan else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
            };
            let baseline_us = backend.elapsed_us();
            let target = match plan
                .target_velocity_rpm
                .with_direction(slot_direction(data.slot))
            {
                Ok(target) => target.raw(),
                Err(_) => {
                    return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
                }
            };
            if backend
                .write_register(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
                || safety.begin_primitive(backend.elapsed_us()).is_err()
            {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(MOVING_STROKE_US)) else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            data.observations.clear();
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = baseline_us;
            data.settle_displacement_counts = 0;
            data.settle_min_displacement_counts = 0;
            data.settle_max_displacement_counts = 0;
            data.peak_torque_target = 0;
            *state = VelocitySweepState::Capture {
                next_sample_us,
                deadline_us,
            };
            StepResult::At(next_sample_us)
        }
        VelocitySweepState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
            };
            let read_us = backend.elapsed_us();
            let elapsed_us = match read_us.checked_sub(data.previous_read_us) {
                Some(value @ 1..=MAX_SAFETY_OBSERVATION_GAP_US) => value,
                _ => {
                    let fault = safety.fault(
                        OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                        0,
                        read_us.saturating_sub(data.previous_read_us),
                        0,
                        0,
                    );
                    return fail_sweep_with_safety(phase, backend, fault).await;
                }
            };
            let domain = EncoderDomain::new(safety.counts_per_rev())
                .expect("sweep setup validates encoder domain");
            let delta = match domain.delta(
                domain.count(data.previous_encoder_raw),
                domain.count(encoder_raw),
            ) {
                Ok(delta) if delta.absolute_counts() < safety.quarter_turn_fault_counts => delta,
                _ => {
                    let fault = safety.fault(OUTER_SAFETY_FAULT_QUARTER_TURN, 0, elapsed_us, 0, 0);
                    return fail_sweep_with_safety(phase, backend, fault).await;
                }
            };
            let diagnostic_kind = velocity_diagnostic_kind(*next_sample_us, *deadline_us);
            let pid_velocity_actual = if matches!(
                diagnostic_kind,
                Some(VelocityDiagnosticKind::PidVelocityActual)
            ) {
                match backend.read_register(PID_VELOCITY_ACTUAL).await {
                    Ok(value) => Some(value as i32),
                    Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
                }
            } else {
                None
            };
            if backend
                .write_register(
                    INTERIM_PIDOUT_TARGET_TORQUE.addr,
                    u32::from(INTERIM_PIDOUT_TARGET_TORQUE.sub_addr),
                )
                .await
                .is_err()
            {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            let torque_target = match backend
                .read_register(INTERIM_PIDOUT_TARGET_TORQUE.data)
                .await
            {
                Ok(value) => (value as u16 as i16).unsigned_abs(),
                Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
            };
            data.peak_torque_target = data.peak_torque_target.max(torque_target);
            if data.peak_torque_target >= plan_hard_limit(data) {
                let fault = safety.fault(OUTER_SAFETY_FAULT_CURRENT, 0, elapsed_us, 0, 0);
                return fail_sweep_with_safety(phase, backend, fault).await;
            }
            if data
                .observations
                .push(delta.signed_counts(), elapsed_us as u16)
                .is_err()
            {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let (diagnostic_flag, velocity_diagnostic_value) = match diagnostic_kind {
                Some(VelocityDiagnosticKind::PidVelocityActual) => (
                    crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID,
                    pid_velocity_actual.expect("PID diagnostic was read before torque output"),
                ),
                Some(VelocityDiagnosticKind::PptmActual) => {
                    if backend
                        .write_register(
                            INTERIM_ACTUAL_VELOCITY_PPTM.addr,
                            u32::from(INTERIM_ACTUAL_VELOCITY_PPTM.sub_addr),
                        )
                        .await
                        .is_err()
                    {
                        return fail_sweep(phase, backend, CommissionError::Spi).await;
                    }
                    match backend
                        .read_register(INTERIM_ACTUAL_VELOCITY_PPTM.data)
                        .await
                    {
                        Ok(value) => (
                            crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID,
                            value as i32,
                        ),
                        Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
                    }
                }
                None => (0, 0),
            };
            backend.record_velocity_trace_observation(
                crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: slot_direction(data.slot).saturating_mul(
                        i32::try_from(
                            u64::from(plan_used_velocity(data))
                                .saturating_mul(u64::from(safety.counts_per_rev()))
                                / 1_000_000,
                        )
                        .unwrap_or(i32::MAX),
                    ),
                    delta_counts: delta.signed_counts(),
                    elapsed_us: elapsed_us as u32,
                    stroke_index: u16::from(data.rung_index) * 4 + u16::from(data.slot),
                    direction: slot_direction(data.slot) as i8,
                    flags: u8::from(data.peak_torque_target >= plan_usable_limit(data))
                        | diagnostic_flag,
                    peak_torque_target_abs: data.peak_torque_target,
                    velocity_diagnostic_value,
                },
            );
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = read_us;
            if read_us >= *deadline_us {
                *state = VelocitySweepState::CommandZero;
                StepResult::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        StepResult::At(*next_sample_us)
                    }
                    Err(error) => fail_sweep(phase, backend, error).await,
                }
            }
        }
        VelocitySweepState::CommandZero => {
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            *state = VelocitySweepState::ClearSettleIntegrator {
                settle_started_us: backend.elapsed_us(),
            };
            StepResult::ReadyNow
        }
        VelocitySweepState::ClearSettleIntegrator { settle_started_us } => {
            if clear_velocity_integrator(backend).await.is_err() {
                return fail_sweep(phase, backend, CommissionError::Spi).await;
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) = settle_started_us.checked_add(u64::from(ZERO_SETTLE_US))
            else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
            else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            data.analysis.reset();
            *state = VelocitySweepState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            StepResult::ReadyNow
        }
        VelocitySweepState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            let now_us = backend.elapsed_us();
            if now_us >= *analysis_deadline_us
                || now_us
                    .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                    .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return fail_sweep(phase, backend, CommissionError::AnalysisOverrun).await;
            }
            let analysis = match poll_moving_observation_analysis(data, safety.counts_per_rev()) {
                Ok(Some(analysis)) => analysis,
                Ok(None) => return StepResult::ReadyNow,
                Err(error) => return fail_sweep(phase, backend, error).await,
            };
            let report = VelocitySweepObservation {
                run_sequence: data.integrity.run_sequence(),
                evidence_sequence: data.integrity.next_evidence_sequence(),
                rung_index: data.rung_index,
                slot: data.slot,
                target_velocity_rpm: plan_target_velocity(data)
                    .saturating_mul(slot_direction(data.slot)),
                summary: analysis.summary,
                stationarity: analysis.stationarity,
                selected_delta_counts: analysis.selected_delta_counts,
                selected_elapsed_us: analysis.selected_elapsed_us,
                selected_level: analysis.selected_level,
                selected_blocks: analysis.selected_blocks,
                variance_of_mean_q2: analysis.variance_of_mean_q2,
                settled_rate_mean_q: analysis.settled_rate_mean_q,
            };
            let terminal_error = match analysis.summary.classification {
                VelocityObservationClass::WrongDirectionFault => {
                    Some(CommissionError::VelocityValidationFailed)
                }
                VelocityObservationClass::HardClipFault => {
                    Some(CommissionError::SafetyEnvelopeViolation)
                }
                VelocityObservationClass::InvalidEvidenceFault => {
                    Some(CommissionError::InvalidSchedule)
                }
                _ => None,
            };
            let current_headroom =
                analysis.summary.classification == VelocityObservationClass::CurrentHeadroom;
            if retain_observation(data, analysis.summary).is_err() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let settle_deadline = *settle_deadline_us;
            data.analysis.reset();
            data.observations.clear();
            *state = if let Some(error) = terminal_error {
                VelocitySweepState::FaultTerminal { error }
            } else {
                VelocitySweepState::BeginSettleCapture {
                    settle_deadline_us: settle_deadline,
                    current_headroom,
                }
            };
            let event = data
                .integrity
                .record(VelocitySweepEvent::Observation(report));
            StepResult::VelocitySweepEvent(event)
        }
        VelocitySweepState::BeginSettleCapture {
            settle_deadline_us,
            current_headroom,
        } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return fail_sweep(phase, backend, CommissionError::AnalysisOverrun).await;
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = baseline_us;
            data.settle_displacement_counts = 0;
            data.settle_min_displacement_counts = 0;
            data.settle_max_displacement_counts = 0;
            *state = VelocitySweepState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
                current_headroom: *current_headroom,
            };
            StepResult::At(next_sample_us)
        }
        VelocitySweepState::SettleCapture {
            next_sample_us,
            deadline_us,
            current_headroom,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_sweep(phase, backend, CommissionError::Spi).await,
            };
            let read_us = backend.elapsed_us();
            let elapsed_us = match read_us.checked_sub(data.previous_read_us) {
                Some(value @ 1..=MAX_SAFETY_OBSERVATION_GAP_US) => value,
                _ => {
                    let fault = safety.fault(
                        OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                        0,
                        read_us.saturating_sub(data.previous_read_us),
                        0,
                        0,
                    );
                    return fail_sweep_with_safety(phase, backend, fault).await;
                }
            };
            let domain = EncoderDomain::new(safety.counts_per_rev())
                .expect("sweep setup validates encoder domain");
            let delta = match domain.delta(
                domain.count(data.previous_encoder_raw),
                domain.count(encoder_raw),
            ) {
                Ok(delta) if delta.absolute_counts() < safety.quarter_turn_fault_counts => delta,
                _ => {
                    let fault = safety.fault(OUTER_SAFETY_FAULT_QUARTER_TURN, 0, elapsed_us, 0, 0);
                    return fail_sweep_with_safety(phase, backend, fault).await;
                }
            };
            data.settle_displacement_counts = i32::from(data.settle_displacement_counts)
                .saturating_add(delta.signed_counts())
                .clamp(-2, 2) as i8;
            data.settle_min_displacement_counts = data
                .settle_min_displacement_counts
                .min(data.settle_displacement_counts);
            data.settle_max_displacement_counts = data
                .settle_max_displacement_counts
                .max(data.settle_displacement_counts);
            if data
                .observations
                .push(delta.signed_counts(), elapsed_us as u16)
                .is_err()
            {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            backend.record_velocity_trace_observation(
                crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: 0,
                    delta_counts: delta.signed_counts(),
                    elapsed_us: elapsed_us as u32,
                    stroke_index: SETTLE_TRACE_STROKE_MARKER
                        | (u16::from(data.rung_index) * 4 + u16::from(data.slot)),
                    direction: 0,
                    flags: 0,
                    peak_torque_target_abs: 0,
                    velocity_diagnostic_value: 0,
                },
            );
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = read_us;
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
                };
                data.analysis.reset();
                *state = VelocitySweepState::AnalyzeSettle {
                    current_headroom: *current_headroom,
                    analysis_deadline_us,
                };
                StepResult::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        StepResult::At(*next_sample_us)
                    }
                    Err(error) => fail_sweep(phase, backend, error).await,
                }
            }
        }
        VelocitySweepState::AnalyzeSettle {
            current_headroom,
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return fail_sweep(phase, backend, CommissionError::AnalysisOverrun).await;
            }
            match poll_settle_confirmation(data) {
                Ok(Some(true)) => {}
                Ok(Some(false)) => {
                    return fail_sweep(phase, backend, CommissionError::VelocityValidationFailed)
                        .await;
                }
                Ok(None) => return StepResult::ReadyNow,
                Err(error) => return fail_sweep(phase, backend, error).await,
            }
            data.analysis.reset();
            data.observations.clear();
            safety.end_primitive();
            *state = if *current_headroom {
                VelocitySweepState::CurrentHeadroomTerminal
            } else {
                VelocitySweepState::Advance
            };
            StepResult::ReadyNow
        }
        VelocitySweepState::FaultTerminal { error } => {
            let error = *error;
            fail_sweep(phase, backend, error).await
        }
        VelocitySweepState::CurrentHeadroomTerminal => {
            StepResult::VelocitySweepTerminal(sweep_evidence_terminal(data, true))
        }
        VelocitySweepState::Advance => {
            let Some(plan) = data.plan else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            if data.slot + 1 < SWEEP_SLOT_COUNT {
                data.slot += 1;
                *state = VelocitySweepState::ClearSignedIntegrator;
            } else {
                let VelocitySweepEvidenceState::CompletePair { forward, reverse } = data.evidence
                else {
                    return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
                };
                data.forward_band
                    .observe(usize::from(data.rung_index), forward);
                data.reverse_band
                    .observe(usize::from(data.rung_index), reverse);
                let report = VelocitySweepRungReport {
                    run_sequence: data.integrity.run_sequence(),
                    evidence_sequence: data.integrity.next_evidence_sequence(),
                    rung_index: data.rung_index,
                    forward,
                    reverse,
                    forward_band: data.forward_band.snapshot(),
                    reverse_band: data.reverse_band.snapshot(),
                };
                let event = data.integrity.record(VelocitySweepEvent::Rung(report));
                if data.rung_index + 1 < plan.rung_count {
                    data.rung_index += 1;
                    data.slot = 0;
                    *state = VelocitySweepState::WriteGain;
                } else {
                    *state = VelocitySweepState::Terminal;
                }
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::Terminal => {
            if data.plan.is_none() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            StepResult::VelocitySweepTerminal(sweep_evidence_terminal(data, false))
        }
    }
}

fn plan_used_velocity(data: &super::super::VelocitySweepData) -> u32 {
    data.plan.map_or(0, |plan| plan.planned_velocity_mrev_s)
}

fn plan_target_velocity(data: &super::super::VelocitySweepData) -> i32 {
    data.plan.map_or(0, |plan| plan.target_velocity_rpm.raw())
}

fn plan_usable_limit(data: &super::super::VelocitySweepData) -> u16 {
    data.plan.map_or(0, |plan| plan.usable_torque_limit)
}

fn plan_hard_limit(data: &super::super::VelocitySweepData) -> u16 {
    data.plan.map_or(0, |plan| plan.hard_torque_limit)
}

#[cfg(test)]
mod tests {
    use crate::autotune::velocity_measurement::VelocityDirection;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;
    use crate::commissioning::outer::{OuterMotionBudget, OuterSafetyLimits, VelocitySweepData};

    use super::*;

    fn phase() -> OuterPhase {
        let budget = OuterMotionBudget {
            requested_velocity_mrev_s: 6_000,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 2_000,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: 2_000,
            positive_position_headroom_mrev: 2_000,
            max_duration_ms: 60_000,
        };
        OuterPhase::VelocitySweep {
            state: VelocitySweepState::Setup,
            data: VelocitySweepData::new(0),
            safety: OuterSafetyLimits::new(0, budget),
        }
    }

    fn enter_first_capture(phase: &mut OuterPhase, backend: &mut MockBackend) {
        backend.current_limit_value = current_limit(2_448);
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        let OuterPhase::VelocitySweep { data, .. } = phase else {
            panic!("velocity sweep should remain active");
        };
        data.plan.as_mut().unwrap().rung_count = 1;
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
    }

    fn assert_fail_closed_tail(backend: &MockBackend) {
        let writes: std::vec::Vec<(u8, u32)> = backend
            .spi_log
            .iter()
            .filter_map(|op| match op {
                SpiOp::Write(address, value) => Some((*address, *value)),
                SpiOp::Read | SpiOp::EnablePin(_) => None,
            })
            .collect();
        assert!(writes.ends_with(&[
            (PID_VELOCITY_TARGET.addr(), 0),
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (MODE_RAMP_MODE_MOTION.addr(), 0),
        ]));
    }

    fn assert_sweep_fault(result: StepResult, error: CommissionError) -> VelocitySweepTerminal {
        let StepResult::VelocitySweepTerminal(terminal) = result else {
            panic!("sweep failure must use the detailed terminal");
        };
        assert_eq!(terminal.outcome, VelocitySweepOutcome::Fault);
        assert_eq!(terminal.cause, error.status_code());
        terminal
    }

    #[test]
    fn plan_is_frozen_and_exposed_before_any_motion_write() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::ReadyNow
        ));
        let OuterPhase::VelocitySweep { state, data, .. } = &phase else {
            panic!("velocity sweep should remain active");
        };
        assert!(matches!(state, VelocitySweepState::EmitPlan));
        assert!(data.plan.is_some());
        let selector_writes: std::vec::Vec<_> = backend
            .spi_log
            .iter()
            .filter_map(|operation| match operation {
                SpiOp::Write(address, value) => Some((*address, *value)),
                SpiOp::Read | SpiOp::EnablePin(_) => None,
            })
            .collect();
        assert!(
            selector_writes.contains(&(
                foci_tmc4671::registers::CONFIG_VELOCITY_METER_PPTM_MIN_POS_DEV
                    .addr
                    .addr(),
                50,
            ))
        );
        assert!(
            selector_writes.contains(&(
                foci_tmc4671::registers::INTERIM_ACTUAL_VELOCITY_PPTM
                    .addr
                    .addr(),
                29,
            ))
        );
        assert!(!selector_writes.iter().any(|(address, _)| {
            *address
                == foci_tmc4671::registers::CONFIG_VELOCITY_METER_PPTM_MIN_POS_DEV
                    .data
                    .addr()
        }));
        assert!(!backend.spi_log.iter().any(|op| {
            matches!(op, SpiOp::Write(address, value)
                if *address == foci_tmc4671::registers::PID_VELOCITY_TARGET.addr()
                    && *value != 0)
        }));

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Planned(_))
        ));
        assert!(matches!(
            phase,
            OuterPhase::VelocitySweep {
                state: VelocitySweepState::WriteGain,
                ..
            }
        ));
    }

    #[test]
    fn setup_enforces_the_full_declared_sweep_position_windows() {
        let mut phase = phase();
        let OuterPhase::VelocitySweep { safety, .. } = &mut phase else {
            panic!("velocity sweep should be active");
        };
        safety.budget.max_stroke_travel_mrev = 2_500;
        safety.budget.negative_position_headroom_mrev = 3_000;
        safety.budget.positive_position_headroom_mrev = 4_000;
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::ReadyNow
        ));

        let OuterPhase::VelocitySweep { safety, .. } = phase else {
            panic!("velocity sweep should remain active");
        };
        assert_eq!(safety.position_window, 10_000);
        assert_eq!(safety.negative_position_window, 12_000);
        assert_eq!(safety.positive_position_window, 16_000);
    }

    #[test]
    fn invalid_plan_fails_before_motion() {
        let mut phase = phase();
        let mut backend = MockBackend::new();

        assert_sweep_fault(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            CommissionError::InvalidSchedule,
        );
        assert!(
            !backend
                .spi_log
                .iter()
                .any(|op| matches!(op, SpiOp::Write(_, _)))
        );
    }

    #[test]
    fn shutdown_and_gain_write_failure_restore_safe_targets() {
        let mut shutdown_phase = phase();
        let mut shutdown_backend = MockBackend::new();
        shutdown_backend.shutdown_requested = true;
        assert_sweep_fault(
            pollster::block_on(step_velocity_sweep_with_trace(
                &mut shutdown_phase,
                &mut shutdown_backend,
            )),
            CommissionError::Shutdown,
        );

        let mut spi_phase = phase();
        let mut spi_backend = MockBackend::new();
        spi_backend.current_limit_value = current_limit(2_448);
        pollster::block_on(step_velocity_sweep_with_trace(
            &mut spi_phase,
            &mut spi_backend,
        ));
        pollster::block_on(step_velocity_sweep_with_trace(
            &mut spi_phase,
            &mut spi_backend,
        ));
        spi_backend.fail_on_write = Some(PID_VELOCITY_P_VELOCITY_I.addr());
        assert_sweep_fault(
            pollster::block_on(step_velocity_sweep_with_trace(
                &mut spi_phase,
                &mut spi_backend,
            )),
            CommissionError::Spi,
        );
        assert_fail_closed_tail(&spi_backend);
    }

    #[test]
    fn moving_capture_faults_on_gap_quarter_turn_and_hard_clip() {
        for cause in 0..3 {
            let mut phase = phase();
            let mut backend = MockBackend::new();
            enter_first_capture(&mut phase, &mut backend);
            backend.elapsed_us_value = if cause == 0 {
                MAX_SAFETY_OBSERVATION_GAP_US + 1
            } else {
                1_000
            };
            backend.set_abn_decoder_count(if cause == 1 { 1_000 } else { 1 });
            if cause == 2 {
                let hard_limit = current_limit(2_448).raw();
                let _ = backend.registers.insert(
                    foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
                        .data
                        .addr(),
                    u32::from(hard_limit),
                );
            }

            let terminal = assert_sweep_fault(
                pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
                CommissionError::SafetyEnvelopeViolation,
            );
            let safety_fault = terminal
                .safety_fault
                .expect("structural motion fault must retain safety evidence");
            assert_eq!(
                safety_fault.reason,
                [
                    OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                    OUTER_SAFETY_FAULT_QUARTER_TURN,
                    OUTER_SAFETY_FAULT_CURRENT,
                ][cause]
            );
            assert_fail_closed_tail(&backend);
        }
    }

    #[test]
    fn moving_capture_alternates_diagnostics_without_raising_slot_cadence() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        let _ = backend.registers.insert(
            foci_tmc4671::registers::PID_VELOCITY_ACTUAL.addr(),
            (-7i32) as u32,
        );
        let _ = backend.registers.insert(
            foci_tmc4671::registers::INTERIM_ACTUAL_VELOCITY_PPTM
                .data
                .addr(),
            (-11i32) as u32,
        );

        let _ = finish_first_moving_capture(&mut phase, &mut backend, 1, 11);

        let OuterPhase::VelocitySweep { data, .. } = &phase else {
            panic!("velocity sweep should remain active");
        };
        assert_eq!(data.settle_displacement_counts, 0);
        assert_eq!(data.settle_min_displacement_counts, 0);
        assert_eq!(data.settle_max_displacement_counts, 0);

        let sampled: std::vec::Vec<_> = backend
            .velocity_trace_observations
            .iter()
            .enumerate()
            .filter(|(_, observation)| {
                observation.flags
                    & crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID
                    != 0
                    || observation.flags
                        & crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID
                        != 0
            })
            .collect();
        let moving_samples = usize::try_from(MOVING_STROKE_US / 1_000).unwrap();
        assert_eq!(sampled.len(), moving_samples / 16);
        assert_eq!(
            sampled
                .iter()
                .map(|(index, _)| *index)
                .collect::<std::vec::Vec<_>>(),
            (15..moving_samples)
                .step_by(16)
                .collect::<std::vec::Vec<_>>()
        );
        let pid_samples: std::vec::Vec<_> = sampled
            .iter()
            .filter(|(_, observation)| {
                observation.flags
                    & crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID
                    != 0
            })
            .collect();
        let pptm_samples: std::vec::Vec<_> = sampled
            .iter()
            .filter(|(_, observation)| {
                observation.flags
                    & crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID
                    != 0
            })
            .collect();
        assert_eq!(pid_samples.len(), moving_samples / 32);
        assert_eq!(pptm_samples.len(), moving_samples / 32);
        assert!(pid_samples.iter().all(|(_, observation)| {
            observation.velocity_diagnostic_value == -7
                && observation.flags
                    & crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID
                    == 0
        }));
        assert!(pptm_samples.iter().all(|(_, observation)| {
            observation.velocity_diagnostic_value == 11
                && observation.flags
                    & crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID
                    == 0
        }));
        assert!(
            backend
                .velocity_trace_observations
                .iter()
                .filter(|observation| observation.stroke_index & SETTLE_TRACE_STROKE_MARKER == 0)
                .all(|observation| observation.peak_torque_target_abs == 11)
        );
    }

    #[test]
    fn diagnostic_feedback_is_paired_with_the_following_torque_output() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        let _ = backend
            .registers
            .insert(foci_tmc4671::registers::PID_VELOCITY_ACTUAL.addr(), 389);
        backend.pid_velocity_actual_update_interim_on_read = Some((8, 320));

        let _ = finish_first_moving_capture(&mut phase, &mut backend, 1, 285);

        let last_pid_sample = backend
            .velocity_trace_observations
            .iter()
            .filter(|observation| {
                observation.stroke_index & SETTLE_TRACE_STROKE_MARKER == 0
                    && observation.flags
                        & crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID
                        != 0
            })
            .next_back()
            .expect("moving stroke must retain the final PID diagnostic");
        assert_eq!(last_pid_sample.velocity_diagnostic_value, 389);
        assert_eq!(last_pid_sample.peak_torque_target_abs, 320);
    }

    fn finish_first_moving_capture(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        delta_per_sample: i32,
        torque_target_abs: u16,
    ) -> StepResult {
        let mut encoder_count = if delta_per_sample < 0 { 2_000 } else { 0 };
        backend.set_abn_decoder_count(encoder_count);
        enter_first_capture(phase, backend);
        let _ = backend.registers.insert(
            INTERIM_PIDOUT_TARGET_TORQUE.data.addr(),
            u32::from(torque_target_abs),
        );
        for sample in 1..=u64::from(MOVING_STROKE_US / 1_000) {
            encoder_count += delta_per_sample;
            backend.elapsed_us_value = sample * 1_000;
            backend.set_abn_decoder_count(encoder_count);
            pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        }
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        poll_until_observation(phase, backend).0
    }

    fn poll_until_observation(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
    ) -> (StepResult, usize) {
        for poll_count in 1..=256 {
            let result = pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
            if matches!(
                result,
                StepResult::VelocitySweepEvent(VelocitySweepEvent::Observation(_))
            ) {
                assert!(
                    poll_count > 1,
                    "moving analysis must yield before emitting its observation"
                );
                return (result, poll_count);
            }
            assert!(matches!(result, StepResult::ReadyNow));
        }
        panic!("moving analysis did not finish within its bounded work count");
    }

    fn poll_until_settle_analysis_completes(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
    ) -> StepResult {
        for _ in 0..256 {
            let result = pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
            let still_analyzing = matches!(
                phase,
                OuterPhase::VelocitySweep {
                    state: VelocitySweepState::AnalyzeSettle { .. },
                    ..
                }
            );
            if !still_analyzing || !matches!(result, StepResult::ReadyNow) {
                return result;
            }
        }
        panic!("settle analysis did not finish within its bounded work count");
    }

    #[test]
    fn wrong_direction_observation_faults_before_another_stroke() {
        let mut phase = phase();
        let mut backend = MockBackend::new();

        let result = finish_first_moving_capture(&mut phase, &mut backend, -1, 1);
        let StepResult::VelocitySweepEvent(VelocitySweepEvent::Observation(report)) = result else {
            panic!("moving analysis must emit the faulting observation first");
        };
        assert_eq!(
            report.summary.classification,
            super::super::sweep_evidence::VelocityObservationClass::WrongDirectionFault
        );

        let terminal = assert_sweep_fault(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            CommissionError::VelocityValidationFailed,
        );
        assert_eq!(terminal.emitted_observations, 1);
        assert_eq!(terminal.emitted_rungs, 0);
        assert_fail_closed_tail(&backend);
    }

    #[test]
    fn current_headroom_terminates_after_confirmed_zero_settle() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        let usable_limit = current_limit(2_448).raw() * 9 / 10;

        let result = finish_first_moving_capture(&mut phase, &mut backend, 1, usable_limit);
        let StepResult::VelocitySweepEvent(VelocitySweepEvent::Observation(report)) = result else {
            panic!("moving analysis must emit the headroom observation first");
        };
        assert_eq!(
            report.summary.classification,
            super::super::sweep_evidence::VelocityObservationClass::CurrentHeadroom
        );

        let moving_samples = u64::from(MOVING_STROKE_US / 1_000);
        let encoder_count = moving_samples as i32;
        let result = pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        assert!(
            matches!(result, StepResult::At(deadline) if deadline == (moving_samples + 1) * 1_000)
        );
        for sample in moving_samples + 1..=moving_samples + u64::from(ZERO_SETTLE_US / 1_000) {
            backend.elapsed_us_value = sample * 1_000;
            backend.set_abn_decoder_count(encoder_count);
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        }
        let sampled_rest: std::vec::Vec<_> = backend
            .velocity_trace_observations
            .iter()
            .filter(|observation| {
                observation.stroke_index & SETTLE_TRACE_STROKE_MARKER != 0
                    && observation.flags
                        & (crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID
                            | crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID)
                        != 0
            })
            .collect();
        assert!(sampled_rest.is_empty());
        let final_rest = backend
            .velocity_trace_observations
            .iter()
            .rev()
            .find(|observation| observation.stroke_index & SETTLE_TRACE_STROKE_MARKER != 0)
            .expect("settle capture must emit raw observations");
        assert_eq!(
            final_rest.flags
                & (crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID
                    | crate::trace::stream::VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID),
            0
        );
        assert!(matches!(
            poll_until_settle_analysis_completes(&mut phase, &mut backend),
            StepResult::ReadyNow
        ));

        let StepResult::VelocitySweepTerminal(terminal) =
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend))
        else {
            panic!("current headroom must terminate without another signed stroke");
        };
        assert_eq!(terminal.outcome, VelocitySweepOutcome::Inconclusive);
        assert_eq!(terminal.cause, SWEEP_CAUSE_CURRENT_HEADROOM);
        assert_eq!(terminal.emitted_observations, 1);
        assert_eq!(terminal.emitted_rungs, 0);
        assert_eq!(
            backend
                .spi_log
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(address, value)
                    if *address == PID_VELOCITY_TARGET.addr() && *value != 0))
                .count(),
            1
        );
    }

    #[test]
    fn analysis_overrun_fails_instead_of_extending_settle() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        let OuterPhase::VelocitySweep { state, data, .. } = &mut phase else {
            panic!("velocity sweep should remain active");
        };
        data.plan.as_mut().unwrap().rung_count = 1;
        for _ in 0..256 {
            data.observations.push(1, 1_000).unwrap();
        }
        *state = VelocitySweepState::AnalyzeMoving {
            settle_deadline_us: 200_000,
            analysis_deadline_us: 1_000,
        };
        backend.elapsed_us_value = 1_000;

        assert_sweep_fault(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            CommissionError::AnalysisOverrun,
        );
        assert_fail_closed_tail(&backend);
    }

    #[test]
    fn unconfirmed_zero_target_settle_fails_closed() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        enter_first_capture(&mut phase, &mut backend);
        let mut encoder_count = 0i32;
        let moving_samples = u64::from(MOVING_STROKE_US / 1_000);
        for sample in 1..=moving_samples {
            encoder_count += 1;
            backend.elapsed_us_value = sample * 1_000;
            backend.set_abn_decoder_count(encoder_count);
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        }
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        poll_until_observation(&mut phase, &mut backend);
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        for sample in moving_samples + 1..=moving_samples + u64::from(ZERO_SETTLE_US / 1_000) {
            encoder_count += 1;
            backend.elapsed_us_value = sample * 1_000;
            backend.set_abn_decoder_count(encoder_count);
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        }

        assert_sweep_fault(
            poll_until_settle_analysis_completes(&mut phase, &mut backend),
            CommissionError::VelocityValidationFailed,
        );
        assert_fail_closed_tail(&backend);
    }

    #[test]
    fn one_rung_executes_fixed_f_r_f_r_strokes_without_early_stop() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        let OuterPhase::VelocitySweep { data, .. } = &mut phase else {
            panic!("velocity sweep should remain active");
        };
        data.plan.as_mut().unwrap().rung_count = 1;

        let plan_event =
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        let StepResult::VelocitySweepEvent(VelocitySweepEvent::Planned(plan_report)) = plan_event
        else {
            panic!("plan must be the first streamed event");
        };
        let mut evidence_sequences = std::vec![plan_report.evidence_sequence];
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        let mut encoder_count = 0i32;
        let mut now_us = 0u64;
        for slot in 0..4u8 {
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            let result =
                pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            assert!(matches!(result, StepResult::At(deadline) if deadline == now_us + 1_000));

            let direction = slot_direction(slot);
            for _ in 0..MOVING_STROKE_US / 1_000 {
                now_us += 1_000;
                encoder_count += direction;
                backend.elapsed_us_value = now_us;
                backend.set_abn_decoder_count(encoder_count);
                pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            }
            assert!(matches!(
                phase,
                OuterPhase::VelocitySweep {
                    state: VelocitySweepState::CommandZero,
                    ..
                }
            ));
            let OuterPhase::VelocitySweep { safety, .. } = &phase else {
                panic!("velocity sweep should remain active");
            };
            assert!(safety.primitive_start_us.is_some());

            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            let OuterPhase::VelocitySweep { safety, .. } = &phase else {
                panic!("velocity sweep should remain active");
            };
            assert!(
                safety.primitive_start_us.is_some(),
                "the local travel window must cover zero-target settle"
            );
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            let (result, _) = poll_until_observation(&mut phase, &mut backend);
            let StepResult::VelocitySweepEvent(VelocitySweepEvent::Observation(report)) = result
            else {
                panic!("moving analysis must stream one observation");
            };
            evidence_sequences.push(report.evidence_sequence);
            let result =
                pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            assert!(matches!(result, StepResult::At(deadline) if deadline == now_us + 1_000));
            for _ in 0..ZERO_SETTLE_US / 1_000 {
                now_us += 1_000;
                backend.elapsed_us_value = now_us;
                backend.set_abn_decoder_count(encoder_count);
                pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            }
            assert!(matches!(
                poll_until_settle_analysis_completes(&mut phase, &mut backend),
                StepResult::ReadyNow
            ));
            let OuterPhase::VelocitySweep { safety, .. } = &phase else {
                panic!("velocity sweep should remain active");
            };
            assert!(
                safety.primitive_start_us.is_none(),
                "confirmed rest ends the moving-plus-settle primitive"
            );
            let advance =
                pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
            if slot == 3 {
                let StepResult::VelocitySweepEvent(VelocitySweepEvent::Rung(report)) = advance
                else {
                    panic!("four observations must end with one rung verdict");
                };
                evidence_sequences.push(report.evidence_sequence);
            } else {
                assert!(matches!(advance, StepResult::ReadyNow));
            }
        }

        let OuterPhase::VelocitySweep { state, data, .. } = &phase else {
            panic!("velocity sweep should remain active");
        };
        assert!(matches!(state, VelocitySweepState::Terminal));
        let VelocitySweepEvidenceState::CompletePair { forward, reverse } = &data.evidence else {
            panic!("terminal rung should retain both directional verdicts");
        };
        assert_eq!(forward.direction, VelocityDirection::Forward);
        assert!(forward.initial_lead_in);
        assert_eq!(reverse.direction, VelocityDirection::Reverse);
        assert!(!reverse.initial_lead_in);
        assert_eq!(evidence_sequences, [0, 1, 2, 3, 4, 5]);
        let targets: std::vec::Vec<i32> = backend
            .spi_log
            .iter()
            .filter_map(|op| match op {
                SpiOp::Write(address, value)
                    if *address == PID_VELOCITY_TARGET.addr() && *value != 0 =>
                {
                    Some(*value as i32)
                }
                _ => None,
            })
            .collect();
        assert_eq!(targets.len(), 4);
        assert!(targets[0] > 0 && targets[1] < 0 && targets[2] > 0 && targets[3] < 0);
        let moving_samples = usize::try_from(MOVING_STROKE_US / 1_000).unwrap();
        assert_eq!(
            backend.capture_register_reads,
            4 * (1 + moving_samples + 1 + usize::try_from(ZERO_SETTLE_US / 1_000).unwrap())
        );
        assert_eq!(
            backend.velocity_controller_output_selects,
            4 * moving_samples
        );
        assert_eq!(
            backend
                .spi_log
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(address, 0)
                    if *address == PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr()))
                .count(),
            8
        );

        let StepResult::VelocitySweepTerminal(terminal) =
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend))
        else {
            panic!("fixed ladder must end with the detailed sweep terminal");
        };
        assert_eq!(terminal.expected_observations, 4);
        assert_eq!(terminal.emitted_observations, 4);
        assert_eq!(terminal.expected_rungs, 1);
        assert_eq!(terminal.emitted_rungs, 1);
        assert_eq!(terminal.outcome, VelocitySweepOutcome::Inconclusive);
        assert_ne!(terminal.digest, 0);
    }
}
