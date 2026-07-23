//! Trace-gated disturbance-identification velocity sweep.

use crate::autotune::CommissionError;
use crate::autotune::velocity_measurement::StructuredStationarityEvidence;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::state::VelocitySweepEvidenceState;
use crate::commissioning::outer::{OuterPhase, VelocitySweepState};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_CURRENT, OUTER_SAFETY_FAULT_OBSERVATION_GAP,
    OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault, RungOriginRecoveryEvidence, StepResult,
    StructuredBoundaryEvidence, VelocityDirectionalRegionReport, VelocityJointRegionReport,
    VelocitySweepEvent, VelocitySweepObservation, VelocitySweepOutcome, VelocitySweepRungReport,
    VelocitySweepTerminal,
};
use foci_tmc4671::pid::PidGainPair;
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, CONFIG_VELOCITY_METER_PPTM_MIN_POS_DEV, INTERIM_ACTUAL_VELOCITY_PPTM,
    INTERIM_PIDOUT_TARGET_TORQUE, MODE_RAMP_MODE_MOTION, PID_ERROR_PID_VELOCITY_ERROR_SUM,
    PID_TORQUE_FLUX_TARGET, PID_VELOCITY_ACTUAL, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use super::operating_region::{
    BoundRounding, CompactSignedRate, DirectionalRegionClosure, DirectionalRegionRef,
    DirectionalRegionSummary, JointOperableClosure, JointOperableRegion,
};
use super::recovery::{
    RecoveryRateAuthority, RecoveryStage, RungOriginRecoveryConfig, RungOriginRecoveryOutcome,
    RungOriginRecoveryResources, RungOriginRecoveryRuntime, RungOriginRecoveryStep,
    plan_rung_origin_recovery, step_rung_origin_recovery,
};
use super::sweep_analysis::{
    poll_moving_observation_analysis, poll_settle_confirmation, retain_observation,
};
use super::sweep_evidence::{
    CollectedRungEvidence, CompactDirectionalRungReport, DirectionalRungAnalysis,
    VelocityEvidenceError, VelocityObservationClass, analyze_directional_rung,
    analyze_directional_set, directional_recovery_lower_rate_q,
};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US,
    SETTLE_ANALYSIS_BUDGET_US, SWEEP_SLOT_COUNT, VELOCITY_SWEEP_SAMPLE_PERIOD_US,
    VelocitySweepExecutionPlan, ZERO_SETTLE_US,
};

const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;
const SETTLE_TRACE_STROKE_MARKER: u16 = 1 << 15;
const SWEEP_CAUSE_NONE: u8 = 0;
const SWEEP_CAUSE_NO_DIRECTIONAL_MODEL: u8 = 1;
const SWEEP_CAUSE_NO_JOINT_OPERABLE_REGION: u8 = 2;
const SWEEP_CAUSE_INTEGRITY: u8 = 3;
const SWEEP_CAUSE_CURRENT_HEADROOM: u8 = 4;
const SWEEP_CAUSE_RECOVERY: u8 = 7;
const PID_VELOCITY_ACTUAL_TRACE_DECIMATION: u16 = 16;

fn execution_gains(
    plan: VelocitySweepExecutionPlan,
) -> [foci_tmc4671::pid::PidGain; MAX_SWEEP_RUNGS] {
    let mut gains = [foci_tmc4671::pid::PidGain::ZERO; MAX_SWEEP_RUNGS];
    let mut index = 0;
    while index < plan.rung_count {
        if let Some(gain) = plan.gain_at(index) {
            gains[usize::from(index)] = gain;
        }
        index += 1;
    }
    gains
}

#[derive(Clone, Copy)]
enum VelocityDiagnosticKind {
    PidVelocityActual,
    PptmActual,
}

fn slot_direction(slot: u8) -> i32 {
    if slot.is_multiple_of(2) { 1 } else { -1 }
}

fn sweep_stroke_index(rung_index: u8, slot: u8) -> Option<u16> {
    u16::from(rung_index)
        .checked_mul(u16::from(SWEEP_SLOT_COUNT))
        .and_then(|base| base.checked_add(u16::from(slot)))
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

fn track_closed_directional_region(
    data: &mut super::super::VelocitySweepData,
    region: DirectionalRegionSummary,
) {
    let plan = data.plan;
    if !plan.is_present() {
        return;
    }
    let gains = execution_gains(plan);
    let active_rung = data
        .operating_regions
        .joint
        .active_nomination(
            &gains[..usize::from(plan.rung_count)],
            plan.p_start,
            plan.p_top,
        )
        .map_or(data.rung_index, |nomination| nomination.rung_index);
    let best_rung = data
        .operating_regions
        .joint
        .best_nomination(
            &gains[..usize::from(plan.rung_count)],
            plan.p_start,
            plan.p_top,
        )
        .map_or(active_rung, |nomination| nomination.rung_index);
    if let Some(reference) = DirectionalRegionRef::from_summary(region, active_rung) {
        data.operating_regions.handoff.observe_directional(
            region.direction,
            reference,
            active_rung,
            best_rung,
        );
    }
}

fn retain_joint_candidate(data: &mut super::super::VelocitySweepData, region: JointOperableRegion) {
    let plan = data.plan;
    if !plan.is_present() {
        return;
    }
    let gains = execution_gains(plan);
    let gains = &gains[..usize::from(plan.rung_count)];
    let previous = data
        .operating_regions
        .joint
        .best_nomination(gains, plan.p_start, plan.p_top)
        .map(|nomination| nomination.joint.member_mask);
    data.operating_regions
        .joint
        .consider_best(region, gains, plan.p_start, plan.p_top);
    let current = data
        .operating_regions
        .joint
        .best_nomination(gains, plan.p_start, plan.p_top)
        .map(|nomination| nomination.joint.member_mask);
    if current != previous && current == Some(region.member_mask) {
        data.operating_regions.handoff.promote_active();
        data.best_nomination_rates = data.active_nomination_rates;
    }
}

fn joint_region_report(
    data: &super::super::VelocitySweepData,
    region: JointOperableRegion,
) -> VelocityJointRegionReport {
    let plan = data.plan;
    assert!(
        plan.is_present(),
        "joint-region reporting requires an immutable plan"
    );
    let p_low = plan
        .gain_at(region.first_rung)
        .expect("joint-region first rung belongs to the immutable ladder")
        .raw();
    let p_high = plan
        .gain_at(region.last_rung)
        .expect("joint-region last rung belongs to the immutable ladder")
        .raw();
    VelocityJointRegionReport {
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        region,
        p_low,
        p_high,
    }
}

fn selected_handoff(
    data: &super::super::VelocitySweepData,
) -> Option<super::operating_region::StageBHandoff> {
    let plan = data.plan;
    if !plan.is_present() {
        return None;
    }
    let gains = execution_gains(plan);
    let gains = &gains[..usize::from(plan.rung_count)];
    let nomination =
        data.operating_regions
            .joint
            .best_nomination(gains, plan.p_start, plan.p_top)?;
    data.operating_regions.handoff.handoff(
        nomination,
        gains,
        data.best_nomination_rates.map(CompactSignedRate::decode),
    )
}

fn representative_rung_rates(
    forward: CompactDirectionalRungReport,
    reverse: CompactDirectionalRungReport,
) -> [CompactSignedRate; 2] {
    [forward, reverse].map(|rung| {
        let low = rung.settled_rate_bounds_q[0].decode();
        let high = rung.settled_rate_bounds_q[1].decode();
        let midpoint = (i128::from(low) + i128::from(high)) / 2;
        let midpoint = i64::try_from(midpoint).expect("the midpoint of two i64 rates fits i64");
        CompactSignedRate::encode(midpoint, BoundRounding::TowardNegativeInfinity)
    })
}

fn structured_boundary_evidence(
    evidence: StructuredStationarityEvidence,
) -> StructuredBoundaryEvidence {
    StructuredBoundaryEvidence {
        passing_suffix_mask: evidence.passing_suffix_mask,
        common_rate_interval_q: evidence.common_rate_interval_q,
        binding_slope_margin_q: evidence.binding_slope_margin_q,
        binding_lag_one_margin_q: evidence.binding_lag_one_margin_q,
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
    let model_direction_mask = data.operating_regions.handoff.model_direction_mask();
    let (expected_observations, expected_rungs) = if data.plan.is_present() {
        let plan = data.plan;
        (
            u16::from(plan.rung_count)
                .saturating_mul(plan.observations_per_direction)
                .saturating_mul(2),
            u16::from(plan.rung_count),
        )
    } else {
        (0, 0)
    };
    StepResult::VelocitySweepTerminal(VelocitySweepTerminal {
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        plan_digest: data.plan.plan_digest,
        outcome: VelocitySweepOutcome::Fault,
        cause: error.status_code(),
        recovery_unavailable: false,
        model_direction_mask,
        selected_memberships: [0; 3],
        nominated_p: 0,
        selected_intervals: [[0; 2]; 2],
        selected_common_intervals: [[0; 2]; 2],
        p_start: data.plan.p_start.raw(),
        p_top: data.plan.p_top.raw(),
        rung_count: data.plan.rung_count,
        coverage_mask: 0,
        directional_region_counts: data.integrity.directional_region_counts(),
        directional_fragment_counts: data.integrity.directional_fragment_counts(),
        expected_observations,
        emitted_observations: data.integrity.emitted_observations(),
        expected_rungs,
        emitted_rungs: data.integrity.emitted_rungs(),
        digest: data.integrity.digest(),
        run_started_us: data.run_started_us,
        run_completed_us: data.previous_read_us,
        safety_fault,
    })
}

fn sweep_evidence_terminal(
    data: &super::super::VelocitySweepData,
    current_headroom: bool,
    recovery_failure: bool,
) -> VelocitySweepTerminal {
    let plan = data.plan;
    assert!(
        plan.is_present(),
        "velocity sweep terminal requires an immutable plan"
    );
    let model_direction_mask = data.operating_regions.handoff.model_direction_mask();
    let handoff = selected_handoff(data);
    let (mut outcome, mut cause) = if recovery_failure {
        (VelocitySweepOutcome::Inconclusive, SWEEP_CAUSE_RECOVERY)
    } else if handoff.is_some() {
        (
            VelocitySweepOutcome::CompleteCandidate,
            if current_headroom {
                SWEEP_CAUSE_CURRENT_HEADROOM
            } else {
                SWEEP_CAUSE_NONE
            },
        )
    } else if model_direction_mask != 3 {
        (
            VelocitySweepOutcome::Inconclusive,
            SWEEP_CAUSE_NO_DIRECTIONAL_MODEL,
        )
    } else {
        (
            VelocitySweepOutcome::Inconclusive,
            SWEEP_CAUSE_NO_JOINT_OPERABLE_REGION,
        )
    };
    let expected_observations = u16::from(plan.rung_count)
        .saturating_mul(plan.observations_per_direction)
        .saturating_mul(2);
    let expected_rungs = u16::from(plan.rung_count);
    if !current_headroom
        && !recovery_failure
        && (data.integrity.emitted_observations() != expected_observations
            || data.integrity.emitted_rungs() != expected_rungs)
    {
        outcome = VelocitySweepOutcome::Fault;
        cause = SWEEP_CAUSE_INTEGRITY;
    }
    VelocitySweepTerminal {
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        plan_digest: plan.plan_digest,
        outcome,
        cause,
        recovery_unavailable: match data.evidence {
            VelocitySweepEvidenceState::RecoveryComplete(report) => matches!(
                report.outcome,
                RungOriginRecoveryOutcome::ContainedNoRate
                    | RungOriginRecoveryOutcome::ContainedDeadline
            ),
            VelocitySweepEvidenceState::Complete { .. } => current_headroom,
            _ => false,
        },
        model_direction_mask,
        selected_memberships: handoff.map_or([0; 3], |handoff| {
            [
                handoff.directional[0].region.member_mask,
                handoff.directional[1].region.member_mask,
                handoff.nomination.joint.member_mask,
            ]
        }),
        nominated_p: handoff.map_or(0, |handoff| handoff.nomination.p.raw()),
        selected_intervals: handoff.map_or([[0; 2]; 2], |handoff| {
            [
                handoff.directional[0].region.pooled_interval,
                handoff.directional[1].region.pooled_interval,
            ]
        }),
        selected_common_intervals: handoff.map_or([[0; 2]; 2], |handoff| {
            [
                handoff.directional[0].region.common_intersection,
                handoff.directional[1].region.common_intersection,
            ]
        }),
        p_start: plan.p_start.raw(),
        p_top: plan.p_top.raw(),
        rung_count: plan.rung_count,
        coverage_mask: handoff.map_or(0, |handoff| handoff.coverage_mask),
        directional_region_counts: data.integrity.directional_region_counts(),
        directional_fragment_counts: data.integrity.directional_fragment_counts(),
        expected_observations,
        emitted_observations: data.integrity.emitted_observations(),
        expected_rungs,
        emitted_rungs: data.integrity.emitted_rungs(),
        digest: data.integrity.digest(),
        run_started_us: data.run_started_us,
        run_completed_us: data.previous_read_us,
        safety_fault: None,
    }
}

pub(super) async fn clear_velocity_integrator<B: CommissioningBackend>(
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
            let generated_plan = if !data.plan.is_present() {
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
                match super::sweep_plan::plan_velocity_sweep(
                    request,
                    encoder_counts_per_rev,
                    torque_limit,
                ) {
                    Ok(plan) => {
                        let plan_digest = plan.digest();
                        data.plan = plan.execution_plan(plan_digest);
                        Some(plan)
                    }
                    Err(_) => {
                        return sweep_fault_terminal(phase, CommissionError::InvalidSchedule, None);
                    }
                }
            } else {
                None
            };
            let plan = data.plan;
            assert!(
                plan.is_present(),
                "velocity sweep setup requires an admitted plan"
            );
            if plan.hard_torque_limit != torque_limit {
                return sweep_fault_terminal(phase, CommissionError::InvalidSchedule, None);
            }
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
            let run_started_us = backend.elapsed_us();
            if safety.arm_safety_observation(run_started_us).is_err() {
                return sweep_fault_terminal(phase, CommissionError::InvalidSchedule, None);
            }
            data.run_started_us = run_started_us;
            *state = VelocitySweepState::WriteGain;
            if let Some(plan) = generated_plan {
                let report = plan.report(
                    data.integrity.run_sequence(),
                    data.integrity.next_evidence_sequence(),
                );
                let event = data.integrity.record(VelocitySweepEvent::Planned(report));
                StepResult::VelocitySweepEvent(event)
            } else {
                StepResult::ReadyNow
            }
        }
        VelocitySweepState::WriteGain => {
            let plan = data.plan;
            if !plan.is_present() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let Some(p) = plan.gain_at(data.rung_index) else {
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
            data.evidence = VelocitySweepEvidenceState::Collecting(CollectedRungEvidence::new());
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
            let plan = data.plan;
            if !plan.is_present() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
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
            let Some(stroke_index) = sweep_stroke_index(data.rung_index, data.slot) else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
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
                    stroke_index,
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
            data.analysis.reset_with_structured_diagnostics();
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
            let structured_boundary = analysis
                .structured_boundary
                .map(structured_boundary_evidence);
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
                structured_boundary,
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
            if retain_observation(data, analysis.summary, analysis.structured_boundary).is_err() {
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
            let Some(stroke_index) = sweep_stroke_index(data.rung_index, data.slot) else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            backend.record_velocity_trace_observation(
                crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: 0,
                    delta_counts: delta.signed_counts(),
                    elapsed_us: elapsed_us as u32,
                    stroke_index: SETTLE_TRACE_STROKE_MARKER | stroke_index,
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
            *state = if *current_headroom || data.slot + 1 >= SWEEP_SLOT_COUNT {
                VelocitySweepState::AnalyzeForwardRung {
                    current_headroom: *current_headroom,
                }
            } else {
                VelocitySweepState::Advance
            };
            StepResult::ReadyNow
        }
        VelocitySweepState::FaultTerminal { error } => {
            let error = *error;
            fail_sweep(phase, backend, error).await
        }
        VelocitySweepState::Advance => {
            if !data.plan.is_present() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            if data.slot + 1 < SWEEP_SLOT_COUNT {
                data.slot += 1;
                *state = VelocitySweepState::ClearSignedIntegrator;
            } else {
                *state = VelocitySweepState::AnalyzeForwardRung {
                    current_headroom: false,
                };
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::AnalyzeForwardRung { current_headroom } => {
            *state = VelocitySweepState::FoldForwardRung {
                current_headroom: *current_headroom,
            };
            StepResult::ReadyNow
        }
        VelocitySweepState::FoldForwardRung { current_headroom } => {
            let VelocitySweepEvidenceState::Collecting(collected) = data.evidence else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let plan = data.plan;
            let Some(p) = plan.gain_at(data.rung_index) else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let forward_recovery_rate = match directional_recovery_lower_rate_q(
                collected.directional_set(
                    crate::autotune::velocity_measurement::VelocityDirection::Forward,
                ),
                crate::autotune::velocity_measurement::VelocityDirection::Forward,
                p,
                plan.target_velocity_rpm.raw(),
                safety.counts_per_rev(),
            ) {
                Ok(rate) => rate.unwrap_or(CompactSignedRate::ZERO),
                Err(_) => {
                    return fail_sweep(phase, backend, CommissionError::VelocityValidationFailed)
                        .await;
                }
            };
            let reverse = collected
                .directional_set(crate::autotune::velocity_measurement::VelocityDirection::Reverse);
            let analysis = match analyze_directional_rung(
                &collected,
                crate::autotune::velocity_measurement::VelocityDirection::Forward,
                p,
                plan.target_velocity_rpm.raw(),
                safety.counts_per_rev(),
            ) {
                Ok(analysis) => analysis,
                Err(_) => {
                    return fail_sweep(phase, backend, CommissionError::VelocityValidationFailed)
                        .await;
                }
            };
            let forward = analysis.report;
            let closed = match observe_directional_rung_for_sweep(
                &mut data.operating_regions.directional[0],
                data.rung_index,
                analysis,
                *current_headroom,
            ) {
                Ok(closed) => closed,
                Err(_) => {
                    return fail_sweep(phase, backend, CommissionError::VelocityValidationFailed)
                        .await;
                }
            };
            data.evidence = VelocitySweepEvidenceState::ForwardFolded {
                forward,
                reverse,
                forward_recovery_rate,
            };
            *state = VelocitySweepState::AnalyzeReverseRung {
                current_headroom: *current_headroom,
            };
            if let Some(region) = closed {
                track_closed_directional_region(data, region);
                let event = data.integrity.record(VelocitySweepEvent::DirectionalRegion(
                    VelocityDirectionalRegionReport {
                        run_sequence: data.integrity.run_sequence(),
                        evidence_sequence: data.integrity.next_evidence_sequence(),
                        region,
                    },
                ));
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::AnalyzeReverseRung { current_headroom } => {
            *state = VelocitySweepState::FoldReverseRung {
                current_headroom: *current_headroom,
            };
            StepResult::ReadyNow
        }
        VelocitySweepState::FoldReverseRung { current_headroom } => {
            let VelocitySweepEvidenceState::ForwardFolded {
                forward,
                reverse,
                forward_recovery_rate,
            } = data.evidence
            else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let plan = data.plan;
            let Some(p) = plan.gain_at(data.rung_index) else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let reverse_recovery_rate = match directional_recovery_lower_rate_q(
                reverse,
                crate::autotune::velocity_measurement::VelocityDirection::Reverse,
                p,
                plan.target_velocity_rpm.raw(),
                safety.counts_per_rev(),
            ) {
                Ok(rate) => rate.unwrap_or(CompactSignedRate::ZERO),
                Err(_) => {
                    return fail_sweep(phase, backend, CommissionError::VelocityValidationFailed)
                        .await;
                }
            };
            let analysis = match analyze_directional_set(
                reverse,
                crate::autotune::velocity_measurement::VelocityDirection::Reverse,
                p,
                plan.target_velocity_rpm.raw(),
                safety.counts_per_rev(),
            ) {
                Ok(analysis) => analysis,
                Err(_) => {
                    return fail_sweep(phase, backend, CommissionError::VelocityValidationFailed)
                        .await;
                }
            };
            let reverse = analysis.report;
            let closed = match observe_directional_rung_for_sweep(
                &mut data.operating_regions.directional[1],
                data.rung_index,
                analysis,
                *current_headroom,
            ) {
                Ok(closed) => closed,
                Err(_) => {
                    return fail_sweep(phase, backend, CommissionError::VelocityValidationFailed)
                        .await;
                }
            };
            data.evidence = VelocitySweepEvidenceState::Complete {
                forward,
                reverse,
                recovery_rates: [forward_recovery_rate, reverse_recovery_rate],
            };
            *state = VelocitySweepState::FoldJointRung {
                current_headroom: *current_headroom,
            };
            if let Some(region) = closed {
                track_closed_directional_region(data, region);
                let event = data.integrity.record(VelocitySweepEvent::DirectionalRegion(
                    VelocityDirectionalRegionReport {
                        run_sequence: data.integrity.run_sequence(),
                        evidence_sequence: data.integrity.next_evidence_sequence(),
                        region,
                    },
                ));
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::FoldJointRung { current_headroom } => {
            let plan = data.plan;
            if !plan.is_present() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let VelocitySweepEvidenceState::Complete {
                forward, reverse, ..
            } = data.evidence
            else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let current_rates = representative_rung_rates(forward, reverse);
            let current_headroom = *current_headroom;
            *state = VelocitySweepState::EmitRung { current_headroom };
            if current_headroom {
                data.previous_rung_rates = current_rates;
                return StepResult::ReadyNow;
            }
            let gains = execution_gains(plan);
            let gains = &gains[..usize::from(plan.rung_count)];
            let previous_active =
                data.operating_regions
                    .joint
                    .active_nomination(gains, plan.p_start, plan.p_top);
            let closed = super::operating_region::observe_joint_rung(
                &mut data.operating_regions.joint,
                data.rung_index,
                forward,
                reverse,
            );
            let active =
                data.operating_regions
                    .joint
                    .active_nomination(gains, plan.p_start, plan.p_top);
            if let Some(region) = closed {
                retain_joint_candidate(data, region);
            }
            match (previous_active, active) {
                (None, Some(active)) => {
                    data.active_nomination_rates = current_rates;
                    data.operating_regions
                        .handoff
                        .begin_active_nomination(active.rung_index);
                }
                (Some(previous), Some(active)) if previous.rung_index != active.rung_index => {
                    data.active_nomination_rates = if active.rung_index == data.rung_index {
                        current_rates
                    } else {
                        data.previous_rung_rates
                    };
                    data.operating_regions
                        .handoff
                        .move_active_nomination(active.rung_index);
                }
                _ => {}
            }
            data.previous_rung_rates = current_rates;
            if let Some(region) = closed {
                let report = joint_region_report(data, region);
                let event = data
                    .integrity
                    .record(VelocitySweepEvent::JointRegion(report));
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::EmitRung { current_headroom } => {
            let plan = data.plan;
            if !plan.is_present() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let VelocitySweepEvidenceState::Complete {
                forward,
                reverse,
                recovery_rates,
            } = data.evidence
            else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let report = VelocitySweepRungReport {
                run_sequence: data.integrity.run_sequence(),
                evidence_sequence: data.integrity.next_evidence_sequence(),
                rung_index: data.rung_index,
                forward,
                reverse,
            };
            let event = data.integrity.record(VelocitySweepEvent::Rung(report));
            let complete_rung =
                forward.collected_mask == 0b1111 && reverse.collected_mask == 0b1111;
            let final_rung = *current_headroom || data.rung_index + 1 >= plan.rung_count;
            if *current_headroom && !complete_rung {
                *state = VelocitySweepState::FinishForward {
                    current_headroom: true,
                    recovery_failure: false,
                };
            } else {
                let recovery_plan = match plan_rung_origin_recovery(
                    safety.budget,
                    safety.counts_per_rev(),
                    plan.planned_velocity_mrev_s,
                ) {
                    Ok(recovery_plan) => recovery_plan,
                    Err(_) => {
                        return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
                    }
                };
                let compact_rate = if safety.unwrapped_position < 0 {
                    recovery_rates[0]
                } else {
                    recovery_rates[1]
                };
                let ordinary = (compact_rate != CompactSignedRate::ZERO)
                    .then_some(RecoveryRateAuthority::settled(compact_rate));
                let authority = RecoveryRateAuthority::select(ordinary, None);
                let runtime = match RungOriginRecoveryRuntime::new(
                    RungOriginRecoveryConfig {
                        stage: RecoveryStage::Proportional,
                        rung_index: data.rung_index,
                        p: plan
                            .gain_at(data.rung_index)
                            .expect("emitted rung belongs to the immutable ladder"),
                        target_velocity_rpm: plan.target_velocity_rpm,
                        planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
                        start_offset_counts: safety.unwrapped_position,
                        lower_rate_q: authority.lower_rate_q(),
                        authority_source: authority.source,
                    },
                    recovery_plan,
                ) {
                    Ok(runtime) => runtime,
                    Err(error) => return fail_sweep(phase, backend, error).await,
                };
                data.evidence = VelocitySweepEvidenceState::Recovering(runtime);
                *state = VelocitySweepState::RecoverRung {
                    current_headroom: *current_headroom,
                    final_rung,
                };
            }
            StepResult::VelocitySweepEvent(event)
        }
        VelocitySweepState::RecoverRung {
            current_headroom,
            final_rung,
        } => {
            let VelocitySweepEvidenceState::Recovering(runtime) = &mut data.evidence else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let mut resources = RungOriginRecoveryResources {
                safety,
                observations: &mut data.observations,
                blocker: &mut data.blocker,
                analysis: &mut data.analysis,
                settle_displacement_counts: &mut data.settle_displacement_counts,
                settle_min_displacement_counts: &mut data.settle_min_displacement_counts,
                settle_max_displacement_counts: &mut data.settle_max_displacement_counts,
            };
            match step_rung_origin_recovery(runtime, &mut resources, backend).await {
                RungOriginRecoveryStep::ReadyNow => StepResult::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => StepResult::At(deadline_us),
                RungOriginRecoveryStep::Finished(report)
                | RungOriginRecoveryStep::Contained(report) => {
                    data.evidence = VelocitySweepEvidenceState::RecoveryComplete(report);
                    *state = VelocitySweepState::FinalizeRecovery {
                        current_headroom: *current_headroom,
                        final_rung: *final_rung,
                    };
                    StepResult::ReadyNow
                }
                RungOriginRecoveryStep::Fault {
                    error: _,
                    safety: Some(fault),
                } => fail_sweep_with_safety(phase, backend, fault).await,
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => fail_sweep(phase, backend, error).await,
            }
        }
        VelocitySweepState::FinalizeRecovery {
            current_headroom,
            final_rung,
        } => {
            let VelocitySweepEvidenceState::RecoveryComplete(report) = data.evidence else {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let contained = matches!(
                report.outcome,
                RungOriginRecoveryOutcome::ContainedNoRate
                    | RungOriginRecoveryOutcome::ContainedDeadline
            );
            if *final_rung || contained {
                *state = VelocitySweepState::FinishForward {
                    current_headroom: *current_headroom,
                    recovery_failure: contained && !*final_rung,
                };
            } else {
                data.rung_index += 1;
                data.slot = 0;
                *state = VelocitySweepState::WriteGain;
            }
            let event =
                data.integrity
                    .record(VelocitySweepEvent::Recovery(RungOriginRecoveryEvidence {
                        run_sequence: data.integrity.run_sequence(),
                        evidence_sequence: data.integrity.next_evidence_sequence(),
                        report,
                    }));
            StepResult::VelocitySweepEvent(event)
        }
        VelocitySweepState::FinishForward {
            current_headroom,
            recovery_failure,
        } => {
            let closure = if *recovery_failure {
                DirectionalRegionClosure::SafetyTerminus
            } else if *current_headroom {
                DirectionalRegionClosure::CurrentHeadroom
            } else {
                DirectionalRegionClosure::EndOfLadder
            };
            let closed = data.operating_regions.directional[0].finish(closure);
            let current_headroom = *current_headroom;
            let recovery_failure = *recovery_failure;
            *state = VelocitySweepState::FinishReverse {
                current_headroom,
                recovery_failure,
            };
            if let Some(region) = closed {
                track_closed_directional_region(data, region);
                let event = data.integrity.record(VelocitySweepEvent::DirectionalRegion(
                    VelocityDirectionalRegionReport {
                        run_sequence: data.integrity.run_sequence(),
                        evidence_sequence: data.integrity.next_evidence_sequence(),
                        region,
                    },
                ));
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::FinishReverse {
            current_headroom,
            recovery_failure,
        } => {
            let closure = if *recovery_failure {
                DirectionalRegionClosure::SafetyTerminus
            } else if *current_headroom {
                DirectionalRegionClosure::CurrentHeadroom
            } else {
                DirectionalRegionClosure::EndOfLadder
            };
            let closed = data.operating_regions.directional[1].finish(closure);
            let current_headroom = *current_headroom;
            let recovery_failure = *recovery_failure;
            *state = VelocitySweepState::FinishJoint {
                current_headroom,
                recovery_failure,
            };
            if let Some(region) = closed {
                track_closed_directional_region(data, region);
                let event = data.integrity.record(VelocitySweepEvent::DirectionalRegion(
                    VelocityDirectionalRegionReport {
                        run_sequence: data.integrity.run_sequence(),
                        evidence_sequence: data.integrity.next_evidence_sequence(),
                        region,
                    },
                ));
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::FinishJoint {
            current_headroom,
            recovery_failure,
        } => {
            let closure = if *recovery_failure {
                JointOperableClosure::SafetyTerminus
            } else if *current_headroom {
                JointOperableClosure::CurrentHeadroom
            } else {
                JointOperableClosure::EndOfLadder
            };
            let closed = data.operating_regions.joint.finish(closure);
            let current_headroom = *current_headroom;
            let recovery_failure = *recovery_failure;
            *state = VelocitySweepState::EmitHandoff {
                current_headroom,
                recovery_failure,
            };
            if let Some(region) = closed {
                retain_joint_candidate(data, region);
                let report = joint_region_report(data, region);
                let event = data
                    .integrity
                    .record(VelocitySweepEvent::JointRegion(report));
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::EmitHandoff {
            current_headroom,
            recovery_failure,
        } => {
            if !data.plan.is_present() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let handoff = selected_handoff(data);
            let current_headroom = *current_headroom;
            let recovery_failure = *recovery_failure;
            *state = VelocitySweepState::Terminal {
                current_headroom,
                recovery_failure,
            };
            if let Some(handoff) = handoff {
                let event = data.integrity.record(VelocitySweepEvent::Handoff(
                    crate::commissioning::types::VelocityStageBHandoffReport {
                        run_sequence: data.integrity.run_sequence(),
                        evidence_sequence: data.integrity.next_evidence_sequence(),
                        handoff,
                    },
                ));
                return StepResult::VelocitySweepEvent(event);
            }
            StepResult::ReadyNow
        }
        VelocitySweepState::Terminal {
            current_headroom,
            recovery_failure,
        } => {
            if !data.plan.is_present() {
                return fail_sweep(phase, backend, CommissionError::InvalidSchedule).await;
            }
            StepResult::VelocitySweepTerminal(sweep_evidence_terminal(
                data,
                *current_headroom,
                *recovery_failure,
            ))
        }
    }
}

fn plan_used_velocity(data: &super::super::VelocitySweepData) -> u32 {
    data.plan.planned_velocity_mrev_s
}

fn plan_target_velocity(data: &super::super::VelocitySweepData) -> i32 {
    data.plan.target_velocity_rpm.raw()
}

fn plan_usable_limit(data: &super::super::VelocitySweepData) -> u16 {
    data.plan.usable_torque_limit
}

fn plan_hard_limit(data: &super::super::VelocitySweepData) -> u16 {
    data.plan.hard_torque_limit
}

fn observe_directional_rung_for_sweep(
    fold: &mut super::operating_region::DirectionalRegionFold,
    rung_index: u8,
    analysis: DirectionalRungAnalysis,
    current_headroom: bool,
) -> Result<Option<DirectionalRegionSummary>, VelocityEvidenceError> {
    if current_headroom {
        return Ok(None);
    }
    super::operating_region::observe_directional_rung(fold, rung_index, analysis)
}

#[cfg(test)]
mod tests {
    use crate::autotune::velocity_measurement::VelocityDirection;
    use crate::autotune::{
        CommissionError, CommissionProfile, EquivalentTorqueTargetInterval,
        EquivalentTorqueTargetQ16, OperatingMode, TransientBuffer,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::outer::velocity::operating_region::{
        BoundRounding, CompactSignedRate, DirectionalRegionClosure, DirectionalRegionFold,
        DirectionalRegionKind, DirectionalRegionSummary, JointOperableClosure, JointOperableRegion,
        PooledDisturbanceAccumulator,
    };
    use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
    use crate::commissioning::outer::velocity::sweep_evidence::{
        CompactDirectionalRungReport, DIRECTIONAL_RUNG_FLAG_OPERABLE, DirectionalRungAnalysis,
        PooledDisturbance,
    };
    use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;
    use crate::commissioning::outer::{OuterMotionBudget, OuterSafetyLimits, VelocitySweepData};
    use foci_tmc4671::pid::PidGain;

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

    fn dispatch_outer_tick(phase: &mut OuterPhase, backend: &mut MockBackend) -> StepResult {
        let mut buffer = TransientBuffer::new();
        pollster::block_on(crate::commissioning::outer::step_outer(
            phase,
            &mut buffer,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            0,
            0,
            false,
            0,
            0,
            backend,
        ))
    }

    fn enter_first_capture(phase: &mut OuterPhase, backend: &mut MockBackend) {
        backend.current_limit_value = current_limit(2_448);
        pollster::block_on(step_velocity_sweep_with_trace(phase, backend));
        let OuterPhase::VelocitySweep { data, .. } = phase else {
            panic!("velocity sweep should remain active");
        };
        data.plan.rung_count = 1;
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

    fn eligible_directional_analysis(p: u16) -> DirectionalRungAnalysis {
        let interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(100),
            EquivalentTorqueTargetQ16::from_raw(120),
        )
        .unwrap();
        let mut accumulator = PooledDisturbanceAccumulator::new();
        for point in [100, 110, 120] {
            accumulator.add(EquivalentTorqueTargetQ16::from_raw(point), interval, 4);
        }
        DirectionalRungAnalysis {
            report: CompactDirectionalRungReport {
                p: PidGain::new(p).unwrap(),
                classification: DirectionalConsensusClass::EligibleConsensus,
                components: [[100, 120], [0, 0]],
                pooled: accumulator.finish(0),
                settled_rate_bounds_q: [
                    CompactSignedRate::encode(1_000, BoundRounding::TowardNegativeInfinity),
                    CompactSignedRate::encode(1_010, BoundRounding::TowardPositiveInfinity),
                    CompactSignedRate::encode(990, BoundRounding::TowardNegativeInfinity),
                    CompactSignedRate::encode(1_020, BoundRounding::TowardPositiveInfinity),
                ],
                collected_mask: 0b1111,
                eligible_mask: 0b1111,
                included_mask: 0b1111,
                operable_count: 4,
                flags: DIRECTIONAL_RUNG_FLAG_OPERABLE,
            },
            pooled_accumulator: accumulator,
        }
    }

    fn install_completed_rung(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        rung_count: u8,
        current_headroom: bool,
        recovery_rates: [CompactSignedRate; 2],
        offset_counts: i32,
    ) {
        backend.current_limit_value = current_limit(2_448);
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(phase, backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Planned(_))
        ));
        let OuterPhase::VelocitySweep {
            state,
            data,
            safety,
        } = phase
        else {
            panic!("velocity sweep should remain active");
        };
        data.plan.rung_count = rung_count;
        let p = data.plan.gain_at(0).unwrap().raw();
        data.evidence = VelocitySweepEvidenceState::Complete {
            forward: eligible_directional_analysis(p).report,
            reverse: eligible_directional_analysis(p).report,
            recovery_rates,
        };
        safety.unwrapped_position = offset_counts;
        *state = VelocitySweepState::EmitRung { current_headroom };
    }

    fn selected_region(
        direction: VelocityDirection,
        pooled: [i32; 2],
        common: [i32; 2],
    ) -> DirectionalRegionSummary {
        let pooled_interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(pooled[0]),
            EquivalentTorqueTargetQ16::from_raw(pooled[1]),
        )
        .unwrap();
        let common_intersection = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(common[0]),
            EquivalentTorqueTargetQ16::from_raw(common[1]),
        )
        .unwrap();
        DirectionalRegionSummary {
            direction,
            member_mask: 0b111,
            first_rung: 0,
            last_rung: 2,
            p_low: PidGain::new(2).unwrap(),
            p_high: PidGain::new(4).unwrap(),
            member_count: 3,
            kind: DirectionalRegionKind::Valid,
            common_intersection,
            pooled: PooledDisturbance {
                estimate: EquivalentTorqueTargetQ16::from_raw((pooled[0] + pooled[1]) / 2),
                interval: pooled_interval,
                variance_floor_observations: 0,
            },
            settled_rate_mean_min_q: 990,
            settled_rate_mean_max_q: 1_020,
            settled_rate_envelope_low_q: 980,
            settled_rate_envelope_high_q: 1_030,
            closure: DirectionalRegionClosure::EndOfLadder,
            boundary_rung: None,
            tested_intersection: common_intersection,
        }
    }

    #[test]
    fn selected_region_common_intersection_reaches_terminal() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        let OuterPhase::VelocitySweep { data, .. } = &mut phase else {
            panic!("velocity sweep should remain active");
        };
        track_closed_directional_region(
            data,
            selected_region(VelocityDirection::Forward, [100, 140], [112, 126]),
        );
        track_closed_directional_region(
            data,
            selected_region(VelocityDirection::Reverse, [-140, -100], [-129, -115]),
        );
        retain_joint_candidate(
            data,
            JointOperableRegion {
                member_mask: 0b111,
                first_rung: 0,
                last_rung: 2,
                member_count: 3,
                closure: JointOperableClosure::EndOfLadder,
            },
        );
        let expected_ladder = [data.plan.p_start.raw(), data.plan.p_top.raw()];
        let expected_rung_count = data.plan.rung_count;

        let terminal = sweep_evidence_terminal(data, true, false);

        assert_eq!(terminal.selected_intervals, [[100, 140], [-140, -100]]);
        assert_eq!(
            terminal.selected_common_intervals,
            [[112, 126], [-129, -115]]
        );
        assert_eq!([terminal.p_start, terminal.p_top], expected_ladder);
        assert_eq!(terminal.rung_count, expected_rung_count);
    }

    #[test]
    fn current_headroom_incomplete_rung_closes_active_region_as_terminus() {
        let mut fold = DirectionalRegionFold::new(VelocityDirection::Forward);
        assert!(
            observe_directional_rung_for_sweep(
                &mut fold,
                0,
                eligible_directional_analysis(256),
                false,
            )
            .unwrap()
            .is_none()
        );

        let incomplete = DirectionalRungAnalysis {
            report: CompactDirectionalRungReport {
                p: PidGain::new(362).unwrap(),
                classification: DirectionalConsensusClass::IncompleteRung,
                components: [[0, 0]; 2],
                pooled: None,
                settled_rate_bounds_q: [CompactSignedRate::ZERO; 4],
                collected_mask: 0b0001,
                eligible_mask: 0,
                included_mask: 0,
                operable_count: 0,
                flags: 0,
            },
            pooled_accumulator: PooledDisturbanceAccumulator::new(),
        };
        assert!(
            observe_directional_rung_for_sweep(&mut fold, 1, incomplete, true)
                .unwrap()
                .is_none()
        );
        let closed = fold
            .finish(DirectionalRegionClosure::CurrentHeadroom)
            .unwrap();
        assert_eq!(closed.member_mask, 0b0001);
        assert_eq!(closed.boundary_rung, None);
        assert_eq!(closed.closure, DirectionalRegionClosure::CurrentHeadroom);
    }

    #[test]
    fn sweep_recovery_cause_is_unique_in_the_stage_b_namespace() {
        assert_eq!(SWEEP_CAUSE_RECOVERY, 7);
        assert!(
            ![
                SWEEP_CAUSE_NONE,
                SWEEP_CAUSE_NO_DIRECTIONAL_MODEL,
                SWEEP_CAUSE_NO_JOINT_OPERABLE_REGION,
                SWEEP_CAUSE_INTEGRITY,
                SWEEP_CAUSE_CURRENT_HEADROOM,
            ]
            .contains(&SWEEP_CAUSE_RECOVERY)
        );
    }

    #[test]
    fn sweep_recovery_containment_stops_before_the_next_gain() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        install_completed_rung(
            &mut phase,
            &mut backend,
            2,
            false,
            [CompactSignedRate::ZERO; 2],
            1_001,
        );

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Rung(_))
        ));
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::ReadyNow
        ));
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Recovery(report))
                if report.report.outcome == RungOriginRecoveryOutcome::ContainedNoRate
        ));
        assert!(matches!(
            phase,
            OuterPhase::VelocitySweep {
                state: VelocitySweepState::FinishForward {
                    current_headroom: false,
                    recovery_failure: true,
                },
                ..
            }
        ));
        assert!(!backend.spi_log.iter().any(|operation| {
            matches!(operation, SpiOp::Write(address, value)
                if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
        }));

        let terminal = loop {
            match pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)) {
                StepResult::VelocitySweepTerminal(terminal) => break terminal,
                StepResult::VelocitySweepEvent(_) | StepResult::ReadyNow => {}
                other => panic!("unexpected recovery-containment finalization result: {other:?}"),
            }
        };
        assert_eq!(terminal.outcome, VelocitySweepOutcome::Inconclusive);
        assert_eq!(terminal.cause, SWEEP_CAUSE_RECOVERY);
        assert_eq!(terminal.emitted_observations, 0);
        assert_eq!(terminal.emitted_rungs, 1);
    }

    #[test]
    fn terminalizing_sweep_skips_post_motion_safety_spi_read() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        install_completed_rung(
            &mut phase,
            &mut backend,
            2,
            false,
            [CompactSignedRate::ZERO; 2],
            1_001,
        );

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Rung(_))
        ));
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::ReadyNow
        ));
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Recovery(report))
                if report.report.outcome == RungOriginRecoveryOutcome::ContainedNoRate
        ));

        backend.elapsed_us_value = 2_000;
        backend.fail_on_read = Some(ABN_DECODER_COUNT.addr());
        let reads_before = backend
            .spi_log
            .iter()
            .filter(|operation| matches!(operation, SpiOp::Read))
            .count();

        let result = dispatch_outer_tick(&mut phase, &mut backend);

        assert!(matches!(result, StepResult::ReadyNow));
        assert!(matches!(
            phase,
            OuterPhase::VelocitySweep {
                state: VelocitySweepState::FinishReverse {
                    recovery_failure: true,
                    ..
                },
                ..
            }
        ));
        assert_eq!(
            backend
                .spi_log
                .iter()
                .filter(|operation| matches!(operation, SpiOp::Read))
                .count(),
            reads_before
        );
    }

    #[test]
    fn active_sweep_recovery_keeps_outer_safety_spi_read() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        install_completed_rung(
            &mut phase,
            &mut backend,
            2,
            false,
            [CompactSignedRate::encode(1_000_000, BoundRounding::TowardNegativeInfinity); 2],
            1_001,
        );
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Rung(_))
        ));
        assert!(matches!(
            phase,
            OuterPhase::VelocitySweep {
                state: VelocitySweepState::RecoverRung { .. },
                ..
            }
        ));

        backend.elapsed_us_value = 2_000;
        backend.fail_on_read = Some(ABN_DECODER_COUNT.addr());

        let result = dispatch_outer_tick(&mut phase, &mut backend);

        assert_sweep_fault(result, CommissionError::Spi);
    }

    #[test]
    fn completed_current_terminus_recovers_but_partial_terminus_skips() {
        let mut completed_phase = phase();
        let mut completed_backend = MockBackend::new();
        install_completed_rung(
            &mut completed_phase,
            &mut completed_backend,
            1,
            true,
            [CompactSignedRate::ZERO; 2],
            1_001,
        );
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(
                &mut completed_phase,
                &mut completed_backend,
            )),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Rung(_))
        ));
        assert!(matches!(
            completed_phase,
            OuterPhase::VelocitySweep {
                state: VelocitySweepState::RecoverRung {
                    current_headroom: true,
                    final_rung: true,
                },
                ..
            }
        ));

        let mut partial_phase = phase();
        let mut partial_backend = MockBackend::new();
        install_completed_rung(
            &mut partial_phase,
            &mut partial_backend,
            1,
            true,
            [CompactSignedRate::ZERO; 2],
            1_001,
        );
        let OuterPhase::VelocitySweep { data, .. } = &mut partial_phase else {
            panic!("velocity sweep should remain active");
        };
        let VelocitySweepEvidenceState::Complete { forward, .. } = &mut data.evidence else {
            panic!("test must begin from complete transient storage");
        };
        forward.collected_mask = 0b0001;
        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(
                &mut partial_phase,
                &mut partial_backend,
            )),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Rung(_))
        ));
        assert!(matches!(
            partial_phase,
            OuterPhase::VelocitySweep {
                state: VelocitySweepState::FinishForward {
                    current_headroom: true,
                    recovery_failure: false,
                },
                ..
            }
        ));
    }

    #[test]
    fn cooperative_structured_evidence_maps_without_reanalysis() {
        let boundary = structured_boundary_evidence(StructuredStationarityEvidence {
            passing_suffix_mask: 0b010,
            common_rate_interval_q: [5, 8],
            binding_slope_margin_q: 2,
            binding_lag_one_margin_q: -3,
        });

        assert_eq!(boundary.passing_suffix_mask, 0b010);
        assert_eq!(boundary.common_rate_interval_q, [5, 8]);
        assert_eq!(boundary.binding_slope_margin_q, 2);
        assert_eq!(boundary.binding_lag_one_margin_q, -3);
    }

    #[test]
    fn plan_is_frozen_and_exposed_before_any_motion_write() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Planned(_))
        ));
        let OuterPhase::VelocitySweep { state, data, .. } = &phase else {
            panic!("velocity sweep should remain active");
        };
        assert!(matches!(state, VelocitySweepState::WriteGain));
        assert!(data.plan.is_present());
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
            StepResult::VelocitySweepEvent(VelocitySweepEvent::Planned(_))
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

        let terminal = loop {
            match pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)) {
                StepResult::VelocitySweepTerminal(terminal) => break terminal,
                StepResult::VelocitySweepEvent(_) | StepResult::ReadyNow => {}
                other => panic!("unexpected current-headroom finalization result: {other:?}"),
            }
        };
        assert_eq!(terminal.outcome, VelocitySweepOutcome::Inconclusive);
        assert_eq!(terminal.cause, SWEEP_CAUSE_NO_DIRECTIONAL_MODEL);
        assert_eq!(terminal.emitted_observations, 1);
        assert_eq!(terminal.emitted_rungs, 1);
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
        data.plan.rung_count = 1;
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
    fn sweep_recovery_follows_one_rung_without_changing_its_evidence() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);
        let plan_event =
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        let OuterPhase::VelocitySweep { data, .. } = &mut phase else {
            panic!("velocity sweep should remain active");
        };
        data.plan.rung_count = 1;

        let StepResult::VelocitySweepEvent(VelocitySweepEvent::Planned(plan_report)) = plan_event
        else {
            panic!("plan must be the first streamed event");
        };
        let mut evidence_sequences = std::vec![plan_report.evidence_sequence];
        let mut observation_slots = std::vec::Vec::new();
        let mut observation_targets = std::vec::Vec::new();
        let mut terminal_rung = None;
        pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
        let mut encoder_count = 0i32;
        let mut now_us = 0u64;
        let mut maximum_analysis_polls = 0usize;
        for slot in 0..SWEEP_SLOT_COUNT {
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
            let (result, analysis_polls) = poll_until_observation(&mut phase, &mut backend);
            maximum_analysis_polls = maximum_analysis_polls.max(analysis_polls);
            let StepResult::VelocitySweepEvent(VelocitySweepEvent::Observation(report)) = result
            else {
                panic!("moving analysis must stream one observation");
            };
            evidence_sequences.push(report.evidence_sequence);
            observation_slots.push(report.slot);
            observation_targets.push(report.target_velocity_rpm);
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
            if slot + 1 == SWEEP_SLOT_COUNT {
                loop {
                    match pollster::block_on(step_velocity_sweep_with_trace(
                        &mut phase,
                        &mut backend,
                    )) {
                        StepResult::VelocitySweepEvent(VelocitySweepEvent::Rung(report)) => {
                            evidence_sequences.push(report.evidence_sequence);
                            terminal_rung = Some(report);
                            break;
                        }
                        StepResult::VelocitySweepEvent(event) => {
                            let sequence = match event {
                                VelocitySweepEvent::DirectionalRegion(report) => {
                                    report.evidence_sequence
                                }
                                VelocitySweepEvent::JointRegion(report) => report.evidence_sequence,
                                _ => panic!("unexpected pre-rung event"),
                            };
                            evidence_sequences.push(sequence);
                        }
                        StepResult::ReadyNow => {}
                        other => panic!("unexpected pre-rung result: {other:?}"),
                    }
                }
            } else {
                let advance =
                    pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend));
                assert!(matches!(advance, StepResult::ReadyNow));
            }
        }

        let OuterPhase::VelocitySweep { state, data, .. } = &phase else {
            panic!("velocity sweep should remain active");
        };
        assert!(matches!(
            state,
            VelocitySweepState::RecoverRung {
                current_headroom: false,
                final_rung: true,
            }
        ));
        assert!(matches!(
            data.evidence,
            VelocitySweepEvidenceState::Recovering(_)
        ));
        let terminal_rung = terminal_rung.expect("terminal rung must be emitted before recovery");
        assert!(terminal_rung.forward.initial_lead_in());
        assert!(!terminal_rung.reverse.initial_lead_in());

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::ReadyNow
        ));
        let OuterPhase::VelocitySweep { state, data, .. } = &phase else {
            panic!("velocity sweep should remain active");
        };
        assert!(matches!(state, VelocitySweepState::FinalizeRecovery { .. }));
        let VelocitySweepEvidenceState::RecoveryComplete(report) = data.evidence else {
            panic!("recovery result must remain available for causal reporting");
        };
        assert_eq!(
            report.outcome,
            super::super::recovery::RungOriginRecoveryOutcome::AlreadyCentered
        );
        let StepResult::VelocitySweepEvent(VelocitySweepEvent::Recovery(recovery)) =
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend))
        else {
            panic!("recovery must stream before the final fold");
        };
        evidence_sequences.push(recovery.evidence_sequence);
        assert_eq!(recovery.report, report);
        assert_eq!(evidence_sequences, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        assert_eq!(observation_slots, [0, 1, 2, 3, 4, 5, 6, 7]);
        assert!(
            observation_targets
                .iter()
                .enumerate()
                .all(|(index, target)| {
                    if index % 2 == 0 {
                        *target > 0
                    } else {
                        *target < 0
                    }
                })
        );
        let moving_samples = usize::try_from(MOVING_STROKE_US / 1_000).unwrap();
        assert_eq!(
            backend.capture_register_reads,
            usize::from(SWEEP_SLOT_COUNT)
                * (1 + moving_samples + 1 + usize::try_from(ZERO_SETTLE_US / 1_000).unwrap())
        );
        assert_eq!(
            backend.velocity_controller_output_selects,
            usize::from(SWEEP_SLOT_COUNT) * moving_samples
        );
        assert_eq!(
            backend.pid_velocity_error_sum_clears,
            2 * usize::from(SWEEP_SLOT_COUNT)
        );
        assert_eq!(maximum_analysis_polls, 168);
        assert!(
            maximum_analysis_polls
                <= usize::try_from(SETTLE_ANALYSIS_BUDGET_US / VELOCITY_SWEEP_SAMPLE_PERIOD_US)
                    .unwrap()
        );

        let terminal = loop {
            match pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)) {
                StepResult::VelocitySweepEvent(event) => {
                    let sequence = match event {
                        VelocitySweepEvent::Planned(report) => report.evidence_sequence,
                        VelocitySweepEvent::Observation(report) => report.evidence_sequence,
                        VelocitySweepEvent::Rung(report) => report.evidence_sequence,
                        VelocitySweepEvent::Recovery(report) => report.evidence_sequence,
                        VelocitySweepEvent::DirectionalRegion(report) => report.evidence_sequence,
                        VelocitySweepEvent::JointRegion(report) => report.evidence_sequence,
                        VelocitySweepEvent::Handoff(report) => report.evidence_sequence,
                    };
                    evidence_sequences.push(sequence);
                }
                StepResult::VelocitySweepTerminal(terminal) => break terminal,
                StepResult::ReadyNow => {}
                other => panic!("unexpected finalization result: {other:?}"),
            }
        };
        assert_eq!(terminal.expected_observations, u16::from(SWEEP_SLOT_COUNT));
        assert_eq!(terminal.emitted_observations, u16::from(SWEEP_SLOT_COUNT));
        assert_eq!(terminal.expected_rungs, 1);
        assert_eq!(terminal.emitted_rungs, 1);
        assert_eq!(terminal.outcome, VelocitySweepOutcome::Inconclusive);
        assert_ne!(terminal.digest, 0);
    }
}
