//! Cooperative fixed-P Stage-C velocity-integral acquisition.

use crate::autotune::velocity_measurement::{
    StationarityResult, VelocityDirection, estimate_confidence,
};
use crate::autotune::{CommissionError, TmcMechanicalRpm, TransientBuffer};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::{
    OuterPhase, OuterSafetyLimits, VelocityIntegralData, VelocityIntegralState,
};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_CURRENT, OUTER_SAFETY_FAULT_OBSERVATION_GAP,
    OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault, RungOriginRecoveryEvidence, StepResult,
    VelocityIntegralEvent, VelocityIntegralObservation, VelocityIntegralOutcome,
    VelocityIntegralRungReport, VelocityIntegralTerminal, VelocityPrimitiveCurrentEvidence,
};
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_TARGET, PID_VELOCITY_P_VELOCITY_I,
    PID_VELOCITY_TARGET,
};

use super::current_guard::{
    VelocityCurrentClampSession, VelocityCurrentContact, VelocityCurrentGuardError,
    VelocityCurrentZeroPhase, begin_velocity_current_primitive,
    finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    read_velocity_pid_input_abs, read_velocity_pid_output_abs,
};
use super::integral_evidence::{
    ExactRateInterval, IntegralInconclusiveKind, IntegralRunSufficiency, IntegralRungClass,
    IntegralRungEvidence, IntegralRungKind, analyze_integral_direction,
    classify_integral_observation,
};
use super::integral_plan::{
    STAGE_C_MAX_POSITIVE_RUNGS, VelocityIntegralExecutionPlan, native_velocity_i_at,
};
use super::recovery::{
    RecoveryBindingSource, RecoveryStage, RungOriginRecoveryConfig, RungOriginRecoveryOutcome,
    RungOriginRecoveryResources, RungOriginRecoveryRuntime, RungOriginRecoveryStep,
    plan_rung_origin_recovery, step_rung_origin_recovery,
};
use super::sweep::clear_velocity_integrator;
use super::sweep_analysis::{
    deficit_interval_q, poll_settle_confirmation_parts, poll_stationarity,
};
use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, VelocityObservationSummary,
};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MIN_SETTLE_CAPTURE_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US,
};

const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;
const SETTLE_TRACE_STROKE_MARKER: u16 = 1 << 15;
const INTEGRAL_CAUSE_NONE: u8 = 0;
const INTEGRAL_CAUSE_TOO_FEW_RUNGS: u8 = 1;
const INTEGRAL_CAUSE_CURRENT_BEFORE_SUFFICIENCY: u8 = 2;
const INTEGRAL_CAUSE_CURRENT_AFTER_SUFFICIENCY: u8 = 3;
const INTEGRAL_CAUSE_INTEGRITY: u8 = 4;
const INTEGRAL_CAUSE_RECOVERY: u8 = 5;
pub(super) const INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE: u8 = 8;
const OPENING_WORD_BASE: usize = 0;
const POSITIVE_WORD_BASE: usize = 4;
const BOOKEND_WORD_BASE: usize = POSITIVE_WORD_BASE + 2 * STAGE_C_MAX_POSITIVE_RUNGS * 2;

pub(super) const INTEGRAL_SLOT_COUNT: u8 = 8;

pub(super) fn integral_i_at(plan: VelocityIntegralExecutionPlan, rung_index: u8) -> Option<u16> {
    let total = plan.positive_rung_count.checked_add(2)?;
    if rung_index >= total {
        None
    } else if rung_index == 0 || rung_index + 1 == total {
        Some(0)
    } else {
        native_velocity_i_at(rung_index - 1)
    }
}

pub(super) const fn integral_slot_direction(slot: u8) -> Option<i32> {
    if slot >= INTEGRAL_SLOT_COUNT {
        None
    } else if slot.is_multiple_of(2) {
        Some(1)
    } else {
        Some(-1)
    }
}

pub(super) fn advance_integral_cursor(rung_index: &mut u8, slot: &mut u8, total_rungs: u8) -> bool {
    if *slot + 1 < INTEGRAL_SLOT_COUNT {
        *slot += 1;
        false
    } else {
        *slot = 0;
        *rung_index = rung_index.saturating_add(1);
        *rung_index >= total_rungs
    }
}

fn integral_rung_kind(plan: VelocityIntegralExecutionPlan, rung_index: u8) -> IntegralRungKind {
    if rung_index == 0 {
        IntegralRungKind::OpeningAnchor
    } else if rung_index == plan.positive_rung_count + 1 {
        IntegralRungKind::ClosingBookend
    } else {
        IntegralRungKind::Positive
    }
}

pub(super) fn apply_integral_current_headroom(
    state: &mut VelocityIntegralState,
    data: &mut VelocityIntegralData,
) {
    if matches!(
        integral_rung_kind(data.plan, data.rung_index),
        IntegralRungKind::Positive
    ) {
        data.current_terminus = data.rung_index.checked_sub(1);
    }
    data.recovery_unavailable = true;
    *state = VelocityIntegralState::Terminal;
}

pub(super) fn positive_directional_floors_exist(data: &VelocityIntegralData) -> bool {
    data.eligible_masks
        .iter()
        .all(|mask| mask.count_ones() >= 3)
}

fn integral_stroke_index(rung_index: u8, slot: u8) -> Option<u16> {
    u16::from(rung_index)
        .checked_mul(u16::from(INTEGRAL_SLOT_COUNT))
        .and_then(|base| base.checked_add(u16::from(slot)))
}

fn current_guard_error(error: VelocityCurrentGuardError) -> CommissionError {
    match error {
        VelocityCurrentGuardError::RegisterIo => CommissionError::Spi,
        VelocityCurrentGuardError::InvalidLimit
        | VelocityCurrentGuardError::ActionableStatus(_) => {
            CommissionError::SafetyEnvelopeViolation
        }
    }
}

fn reset_rung(data: &mut VelocityIntegralData) {
    data.slot = 0;
    data.rung_classes = [VelocityObservationClass::InvalidEvidenceFault; 8];
    data.rung_intervals = [ExactRateInterval::EMPTY; 8];
}

fn interval_word_index(direction: usize, rung: usize) -> Option<usize> {
    if direction >= 2 || rung >= STAGE_C_MAX_POSITIVE_RUNGS {
        None
    } else {
        Some(POSITIVE_WORD_BASE + (direction * STAGE_C_MAX_POSITIVE_RUNGS + rung) * 2)
    }
}

fn store_interval(
    workspace: &mut TransientBuffer,
    word_index: usize,
    interval: ExactRateInterval,
) -> Result<(), CommissionError> {
    if workspace.write_workspace_word(word_index, interval.low_q as u32)
        && workspace.write_workspace_word(word_index + 1, interval.high_q as u32)
    {
        Ok(())
    } else {
        Err(CommissionError::InvalidSchedule)
    }
}

fn load_interval(
    workspace: &TransientBuffer,
    word_index: usize,
) -> Result<ExactRateInterval, CommissionError> {
    ExactRateInterval::new(
        workspace
            .workspace_word(word_index)
            .ok_or(CommissionError::InvalidSchedule)? as i32,
        workspace
            .workspace_word(word_index + 1)
            .ok_or(CommissionError::InvalidSchedule)? as i32,
    )
    .map_err(|_| CommissionError::InvalidSchedule)
}

pub(super) fn opening_recovery_lower_rate_q(
    workspace: &TransientBuffer,
    opening_direction_mask: u8,
    target_velocity_rpm: i32,
    counts_per_rev: u32,
    direction: usize,
) -> Result<Option<u64>, CommissionError> {
    if opening_direction_mask != 3
        || target_velocity_rpm <= 0
        || counts_per_rev == 0
        || direction >= 2
    {
        return Ok(None);
    }
    let deficit = load_interval(workspace, OPENING_WORD_BASE + direction * 2)?;
    let signed_target_rpm = if direction == 0 {
        target_velocity_rpm
    } else {
        -target_velocity_rpm
    };
    let target_numerator = i128::from(signed_target_rpm)
        .checked_mul(i128::from(counts_per_rev))
        .and_then(|value| {
            value.checked_mul(i128::from(
                crate::autotune::velocity_measurement::VELOCITY_RATE_Q_SCALE,
            ))
        })
        .ok_or(CommissionError::InvalidSchedule)?;
    let settled_low = super::sweep_analysis::floor_div_60(
        target_numerator
            .checked_sub(i128::from(deficit.high_q) * 60)
            .ok_or(CommissionError::InvalidSchedule)?,
    )?;
    let settled_high = super::sweep_analysis::ceil_div_60(
        target_numerator
            .checked_sub(i128::from(deficit.low_q) * 60)
            .ok_or(CommissionError::InvalidSchedule)?,
    )?;
    match direction {
        0 if settled_low > 0 => u64::try_from(settled_low)
            .map(Some)
            .map_err(|_| CommissionError::InvalidSchedule),
        1 if settled_high < 0 => u64::try_from(-settled_high)
            .map(Some)
            .map_err(|_| CommissionError::InvalidSchedule),
        0 | 1 => Ok(None),
        _ => unreachable!("direction bounds are validated above"),
    }
}

fn plan_integral_recovery(
    data: &mut VelocityIntegralData,
    workspace: &TransientBuffer,
    safety: &OuterSafetyLimits,
) -> Result<RungOriginRecoveryRuntime, CommissionError> {
    let direction = usize::from(safety.unwrapped_position >= 0);
    let lower_rate_q = opening_recovery_lower_rate_q(
        workspace,
        data.opening_direction_mask,
        data.plan.target_velocity_rpm,
        safety.counts_per_rev(),
        direction,
    )?;
    let recovery_plan = plan_rung_origin_recovery(
        safety.budget,
        safety.counts_per_rev(),
        data.plan.planned_velocity_mrev_s,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    let clamp_limit = PidTorqueFluxLimit::new(u32::from(data.plan.hard_torque_limit))
        .map_err(|_| CommissionError::InvalidSchedule)?;
    data.current_observer
        .reset(clamp_limit, data.plan.limit_evidence_capability);
    RungOriginRecoveryRuntime::new(
        RungOriginRecoveryConfig {
            stage: RecoveryStage::Integral,
            rung_index: data.rung_index,
            p: PidPGain::new(data.plan.final_p).expect("validated Stage-C P remains nonzero"),
            target_velocity_rpm: TmcMechanicalRpm::from_register(data.plan.target_velocity_rpm),
            planned_velocity_mrev_s: data.plan.planned_velocity_mrev_s,
            start_offset_counts: safety.unwrapped_position,
            lower_rate_q,
            authority_source: if lower_rate_q.is_some() {
                RecoveryBindingSource::SettledRate
            } else {
                RecoveryBindingSource::MissingRate
            },
        },
        recovery_plan,
    )
}

fn materialize_run_evidence(
    data: &VelocityIntegralData,
    workspace: &TransientBuffer,
) -> Result<super::integral_evidence::IntegralRunEvidence, CommissionError> {
    let opening = if data.opening_direction_mask == 3 {
        [
            load_interval(workspace, OPENING_WORD_BASE)?,
            load_interval(workspace, OPENING_WORD_BASE + 2)?,
        ]
    } else {
        [ExactRateInterval::EMPTY; 2]
    };
    let mut evidence = super::integral_evidence::IntegralRunEvidence::new(opening);
    for direction in 0..2 {
        for rung in 0..usize::from(data.plan.positive_rung_count) {
            if data.eligible_masks[direction] & (1_u32 << rung) != 0 {
                let word_index =
                    interval_word_index(direction, rung).ok_or(CommissionError::InvalidSchedule)?;
                evidence
                    .retain_positive(direction, rung, load_interval(workspace, word_index)?)
                    .map_err(|_| CommissionError::InvalidSchedule)?;
            }
        }
        if data.bookend_available_mask & (1 << direction) != 0 {
            evidence.bookend[direction] =
                Some(load_interval(workspace, BOOKEND_WORD_BASE + direction * 2)?);
        }
    }
    evidence.current_terminus = data.current_terminus;
    Ok(evidence)
}

fn direction_for_slot(slot: u8) -> Result<VelocityDirection, CommissionError> {
    match integral_slot_direction(slot) {
        Some(1) => Ok(VelocityDirection::Forward),
        Some(-1) => Ok(VelocityDirection::Reverse),
        _ => Err(CommissionError::InvalidSchedule),
    }
}

fn terminal_from_data(
    data: &VelocityIntegralData,
    evidence: super::integral_evidence::IntegralRunEvidence,
    outcome: VelocityIntegralOutcome,
    cause: u8,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    StepResult::VelocityIntegralTerminal(VelocityIntegralTerminal {
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        plan_digest: data.plan.plan_digest,
        digest: data.integrity.digest(),
        outcome,
        cause,
        recovery_unavailable: data.recovery_unavailable,
        evidence,
        expected_observations: u16::from(data.plan.positive_rung_count.saturating_add(2))
            .saturating_mul(u16::from(INTEGRAL_SLOT_COUNT)),
        emitted_observations: data.integrity.emitted_observations(),
        expected_rungs: u16::from(data.plan.positive_rung_count.saturating_add(2)),
        emitted_rungs: data.integrity.emitted_rungs(),
        run_started_us: data.run_started_us,
        run_completed_us: data.previous_read_us,
        safety_fault,
    })
}

pub(crate) fn integral_fault_terminal(
    phase: &OuterPhase,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    let OuterPhase::VelocityIntegral { data, .. } = phase else {
        return StepResult::OuterFailed(error);
    };
    terminal_from_data(
        data,
        super::integral_evidence::IntegralRunEvidence::new([ExactRateInterval::EMPTY; 2]),
        VelocityIntegralOutcome::Fault,
        error.status_code(),
        safety_fault,
    )
}

async fn fail_integral<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
    error: CommissionError,
) -> StepResult {
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    integral_fault_terminal(phase, error, None)
}

async fn fail_integral_with_safety<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
    fault: OuterSafetyFault,
) -> StepResult {
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    integral_fault_terminal(phase, CommissionError::SafetyEnvelopeViolation, Some(fault))
}

fn compact_summary(
    data: &VelocityIntegralData,
    slot: usize,
    direction: VelocityDirection,
) -> Result<VelocityObservationSummary, CommissionError> {
    let p = PidPGain::new(data.plan.final_p).map_err(|_| CommissionError::InvalidSchedule)?;
    let interval = data.rung_intervals[slot];
    let direction_rate = match direction {
        VelocityDirection::Forward => [1, 1],
        VelocityDirection::Reverse => [-1, -1],
    };
    Ok(VelocityObservationSummary {
        direction,
        p,
        classification: data.rung_classes[slot],
        settled_rate_interval_q: direction_rate,
        settled_rate_mean_q: direction_rate[0],
        deficit_interval_q: [i64::from(interval.low_q), i64::from(interval.high_q)],
        disturbance: None,
        disturbance_interval: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: 0,
        initial_lead_in: data.rung_index == 0 && slot == 0,
    })
}

fn analyze_rung(
    data: &VelocityIntegralData,
    direction: VelocityDirection,
) -> Result<IntegralRungEvidence, CommissionError> {
    let offset = match direction {
        VelocityDirection::Forward => 0,
        VelocityDirection::Reverse => 1,
    };
    let observations = core::array::from_fn(|index| {
        compact_summary(data, index * 2 + offset, direction)
            .expect("validated compact Stage-C evidence reconstructs")
    });
    analyze_integral_direction(
        direction,
        integral_rung_kind(data.plan, data.rung_index),
        observations,
    )
    .map_err(|_| CommissionError::InvalidSchedule)
}

fn retain_rung(
    data: &mut VelocityIntegralData,
    workspace: &mut TransientBuffer,
    directions: [IntegralRungEvidence; 2],
) -> Result<(), CommissionError> {
    match integral_rung_kind(data.plan, data.rung_index) {
        IntegralRungKind::OpeningAnchor => {
            if directions
                .iter()
                .all(|evidence| evidence.classification == IntegralRungClass::EligibleConsensus)
            {
                let opening = directions.map(|evidence| {
                    evidence
                        .interval
                        .expect("eligible opening anchor has an exact interval")
                });
                store_interval(workspace, OPENING_WORD_BASE, opening[0])?;
                store_interval(workspace, OPENING_WORD_BASE + 2, opening[1])?;
                data.opening_direction_mask = 3;
            }
        }
        IntegralRungKind::Positive => {
            let positive_rung = usize::from(data.rung_index - 1);
            for (direction, evidence) in directions.iter().enumerate() {
                if let Some(interval) = evidence.interval {
                    let bit = 1_u32
                        .checked_shl(
                            u32::try_from(positive_rung)
                                .map_err(|_| CommissionError::InvalidSchedule)?,
                        )
                        .ok_or(CommissionError::InvalidSchedule)?;
                    if data.eligible_masks[direction] & bit != 0 {
                        return Err(CommissionError::InvalidSchedule);
                    }
                    let word_index = interval_word_index(direction, positive_rung)
                        .ok_or(CommissionError::InvalidSchedule)?;
                    store_interval(workspace, word_index, interval)?;
                    data.eligible_masks[direction] |= bit;
                }
            }
        }
        IntegralRungKind::ClosingBookend => {
            for (direction, evidence) in directions.iter().enumerate() {
                if let Some(interval) = evidence.interval {
                    store_interval(workspace, BOOKEND_WORD_BASE + direction * 2, interval)?;
                    data.bookend_available_mask |= 1 << direction;
                }
            }
        }
    }
    Ok(())
}

fn evidence_terminal(data: &VelocityIntegralData, workspace: &TransientBuffer) -> StepResult {
    let evidence = match materialize_run_evidence(data, workspace) {
        Ok(evidence) => evidence,
        Err(_) => {
            return terminal_from_data(
                data,
                super::integral_evidence::IntegralRunEvidence::new([ExactRateInterval::EMPTY; 2]),
                VelocityIntegralOutcome::Fault,
                INTEGRAL_CAUSE_INTEGRITY,
                None,
            );
        }
    };
    if data.recovery_failure {
        return terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::Inconclusive,
            INTEGRAL_CAUSE_RECOVERY,
            None,
        );
    }
    match evidence.assess(usize::from(data.plan.positive_rung_count)) {
        Ok(IntegralRunSufficiency::Sufficient {
            current_terminus, ..
        }) => terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::CompleteCandidate,
            if current_terminus.is_some() {
                INTEGRAL_CAUSE_CURRENT_AFTER_SUFFICIENCY
            } else {
                INTEGRAL_CAUSE_NONE
            },
            None,
        ),
        Ok(IntegralRunSufficiency::Inconclusive { cause, .. }) => terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::Inconclusive,
            if data.opening_direction_mask != 3 && data.observation_current_headroom {
                INTEGRAL_CAUSE_CURRENT_BEFORE_SUFFICIENCY
            } else {
                match cause {
                    IntegralInconclusiveKind::TooFewEligibleRungs => INTEGRAL_CAUSE_TOO_FEW_RUNGS,
                    IntegralInconclusiveKind::CurrentBoundBeforeSufficiency => {
                        INTEGRAL_CAUSE_CURRENT_BEFORE_SUFFICIENCY
                    }
                    IntegralInconclusiveKind::BookendUnavailable => {
                        INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE
                    }
                }
            },
            None,
        ),
        Err(_) => terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::Fault,
            INTEGRAL_CAUSE_INTEGRITY,
            None,
        ),
    }
}

fn moving_analysis(
    data: &mut VelocityIntegralData,
    counts_per_rev: u32,
) -> Result<Option<(VelocityObservationSummary, StationarityResult)>, CommissionError> {
    let Some(stationarity) =
        poll_stationarity(&mut data.analysis, &data.observations, &mut data.blocker)?
    else {
        return Ok(None);
    };
    let p = PidPGain::new(data.plan.final_p).map_err(|_| CommissionError::InvalidSchedule)?;
    let direction = direction_for_slot(data.slot)?;
    let mut input = VelocityObservationInput {
        direction,
        p,
        stationarity,
        settled_rate_low_q: 0,
        settled_rate_high_q: 0,
        settled_rate_mean_q: 0,
        deficit_low_q: 0,
        deficit_high_q: 0,
        disturbance: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: data.current_observer.moving_pid_output_peak(),
        usable_torque_limit: data.plan.usable_torque_limit,
        hard_torque_limit: data.plan.hard_torque_limit,
        initial_lead_in: data.rung_index == 0 && data.slot == 0,
    };
    if let StationarityResult::Usable(suffix) = stationarity {
        let confidence = estimate_confidence(suffix.selection, data.plan.family_size)
            .map_err(|_| CommissionError::VelocityValidationFailed)?;
        let mean_rate_q = suffix
            .selection
            .exact_totals
            .rate_q()
            .map_err(|_| CommissionError::VelocityValidationFailed)?;
        let half_width = i64::try_from(confidence.half_width_rate_q)
            .map_err(|_| CommissionError::VelocityValidationFailed)?;
        input.settled_rate_mean_q = mean_rate_q;
        input.settled_rate_low_q = mean_rate_q.saturating_sub(half_width);
        input.settled_rate_high_q = mean_rate_q.saturating_add(half_width);
        let signed_target = data
            .plan
            .target_velocity_rpm
            .checked_mul(integral_slot_direction(data.slot).unwrap_or(0))
            .ok_or(CommissionError::InvalidSchedule)?;
        (input.deficit_low_q, input.deficit_high_q) = deficit_interval_q(
            signed_target,
            counts_per_rev,
            input.settled_rate_low_q,
            input.settled_rate_high_q,
        )?;
    }
    Ok(Some((classify_integral_observation(input), stationarity)))
}

fn retain_analyzed_observation(
    data: &mut VelocityIntegralData,
    summary: VelocityObservationSummary,
) -> Result<(), CommissionError> {
    let interval = ExactRateInterval::new(
        i32::try_from(summary.deficit_interval_q[0])
            .map_err(|_| CommissionError::InvalidSchedule)?,
        i32::try_from(summary.deficit_interval_q[1])
            .map_err(|_| CommissionError::InvalidSchedule)?,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    data.rung_classes[usize::from(data.slot)] = summary.classification;
    data.rung_intervals[usize::from(data.slot)] = interval;
    Ok(())
}

/// Advance the trace-gated Stage-C response sweep by one cooperative state-machine step.
pub(crate) async fn step_velocity_integral_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    workspace: &mut TransientBuffer,
    backend: &mut B,
) -> StepResult {
    let (state, data, safety) = match phase {
        OuterPhase::VelocityIntegral {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::ReadyNow,
    };

    match state {
        VelocityIntegralState::Setup => {
            if backend.is_shutdown_requested() {
                return integral_fault_terminal(phase, CommissionError::Shutdown, None);
            }
            let plan = data.plan;
            let counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
            if plan.plan_digest == 0
                || plan.positive_rung_count < 3
                || plan.hard_torque_limit != u16::from(backend.current_limit())
            {
                return integral_fault_terminal(phase, CommissionError::InvalidSchedule, None);
            }
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return integral_fault_terminal(phase, CommissionError::Spi, None),
            };
            if safety
                .configure_velocity_sweep_encoder_resolution(counts_per_rev)
                .and_then(|_| safety.set_encoder_baseline(encoder_raw))
                .is_err()
                || safety
                    .configure_velocity_envelope(plan.planned_velocity_mrev_s)
                    .is_err()
            {
                return integral_fault_terminal(phase, CommissionError::EncoderFault, None);
            }
            safety.torque_cap = plan.hard_torque_limit;
            workspace.reset(0);
            let run_started_us = backend.elapsed_us();
            if safety.arm_safety_observation(run_started_us).is_err() {
                return integral_fault_terminal(phase, CommissionError::InvalidSchedule, None);
            }
            data.run_started_us = run_started_us;
            data.previous_read_us = run_started_us;
            *state = VelocityIntegralState::WriteGains;
            StepResult::ReadyNow
        }
        VelocityIntegralState::WriteGains => {
            let plan = data.plan;
            let Some(i_raw) = integral_i_at(plan, data.rung_index) else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let p = match PidPGain::new(plan.final_p) {
                Ok(value) => value,
                Err(_) => {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                }
            };
            let i = match PidIGain::new(i_raw) {
                Ok(value) => value,
                Err(_) => {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                }
            };
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, i).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_MODE)
                    .await
                    .is_err()
            {
                return fail_integral(phase, backend, CommissionError::Spi).await;
            }
            reset_rung(data);
            *state = VelocityIntegralState::ClearSignedIntegrator;
            StepResult::ReadyNow
        }
        VelocityIntegralState::ClearSignedIntegrator => {
            if clear_velocity_integrator(backend).await.is_err() {
                return fail_integral(phase, backend, CommissionError::Spi).await;
            }
            *state = VelocityIntegralState::CommandTarget;
            StepResult::ReadyNow
        }
        VelocityIntegralState::CommandTarget => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_integral(phase, backend, CommissionError::Spi).await,
            };
            let baseline_us = backend.elapsed_us();
            let target = match integral_slot_direction(data.slot)
                .and_then(|direction| data.plan.target_velocity_rpm.checked_mul(direction))
            {
                Some(value) => value,
                None => {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                }
            };
            let mut clamp_session = match VelocityCurrentClampSession::try_new(u32::from(
                data.plan.hard_torque_limit,
            )) {
                Ok(session) if session.probe_limit().raw() == data.plan.usable_torque_limit => {
                    session
                }
                Ok(_) | Err(_) => {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                }
            };
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return fail_integral(phase, backend, current_guard_error(error)).await;
            }
            let clamp_limit = clamp_session.probe_limit();
            if clamp_limit == PidTorqueFluxLimit::ZERO {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            if let Err(error) = begin_velocity_current_primitive(
                &mut data.current_observer,
                backend,
                clamp_limit,
                data.plan.limit_evidence_capability,
            )
            .await
            {
                return fail_integral(phase, backend, current_guard_error(error)).await;
            }
            if backend
                .write_register(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
                || safety.begin_primitive(backend.elapsed_us()).is_err()
            {
                return fail_integral(phase, backend, CommissionError::Spi).await;
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(data.plan.moving_stroke_us))
            else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            data.observations.clear();
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = baseline_us;
            data.observation_current_headroom = false;
            *state = VelocityIntegralState::Capture {
                next_sample_us,
                deadline_us,
            };
            StepResult::At(next_sample_us)
        }
        VelocityIntegralState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            if backend.elapsed_us() < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_integral(phase, backend, CommissionError::Spi).await,
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
                    return fail_integral_with_safety(phase, backend, fault).await;
                }
            };
            let domain = EncoderDomain::new(safety.counts_per_rev())
                .expect("Stage-C setup validates encoder domain");
            let delta = match domain.delta(
                domain.count(data.previous_encoder_raw),
                domain.count(encoder_raw),
            ) {
                Ok(delta) if delta.absolute_counts() < safety.quarter_turn_fault_counts => delta,
                _ => {
                    let fault = safety.fault(OUTER_SAFETY_FAULT_QUARTER_TURN, 0, elapsed_us, 0, 0);
                    return fail_integral_with_safety(phase, backend, fault).await;
                }
            };
            let torque_demand = match read_velocity_pid_input_abs(backend).await {
                Ok(value) => value,
                Err(error) => {
                    return fail_integral(phase, backend, current_guard_error(error)).await;
                }
            };
            if torque_demand > data.current_observer.clamp_limit() {
                let fault = safety.fault(OUTER_SAFETY_FAULT_CURRENT, 0, elapsed_us, 0, 0);
                return fail_integral_with_safety(phase, backend, fault).await;
            }
            let torque_target = match read_velocity_pid_output_abs(backend).await {
                Ok(value) => value,
                Err(error) => {
                    return fail_integral(phase, backend, current_guard_error(error)).await;
                }
            };
            data.current_observer.observe_moving_output(torque_target);
            let moving_peak = data.current_observer.moving_pid_output_peak();
            if data
                .observations
                .push(delta.signed_counts(), elapsed_us as u16)
                .is_err()
            {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let Some(stroke_index) = integral_stroke_index(data.rung_index, data.slot) else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            backend.record_velocity_trace_observation(
                crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: integral_slot_direction(data.slot)
                        .unwrap_or(0)
                        .saturating_mul(
                            i32::try_from(
                                u64::from(data.plan.planned_velocity_mrev_s)
                                    .saturating_mul(u64::from(safety.counts_per_rev()))
                                    / 1_000_000,
                            )
                            .unwrap_or(i32::MAX),
                        ),
                    delta_counts: delta.signed_counts(),
                    elapsed_us: elapsed_us as u32,
                    stroke_index,
                    direction: integral_slot_direction(data.slot).unwrap_or(0) as i8,
                    flags: u8::from(moving_peak >= data.plan.usable_torque_limit),
                    peak_torque_target_abs: moving_peak,
                    velocity_diagnostic_value: 0,
                },
            );
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = read_us;
            if read_us >= *deadline_us {
                *state = VelocityIntegralState::CommandZero;
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
                    Err(error) => fail_integral(phase, backend, error).await,
                }
            }
        }
        VelocityIntegralState::CommandZero => {
            let boundary =
                finish_velocity_current_moving_phase(&mut data.current_observer, backend).await;
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return fail_integral(phase, backend, CommissionError::Spi).await;
            }
            if let Err(error) = boundary {
                return fail_integral(phase, backend, current_guard_error(error)).await;
            }
            *state = VelocityIntegralState::ClearSettleIntegrator {
                settle_started_us: backend.elapsed_us(),
            };
            StepResult::ReadyNow
        }
        VelocityIntegralState::ClearSettleIntegrator { settle_started_us } => {
            if clear_velocity_integrator(backend).await.is_err() {
                return fail_integral(phase, backend, CommissionError::Spi).await;
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) =
                settle_started_us.checked_add(u64::from(data.plan.zero_settle_us))
            else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(data.plan.analysis_budget_us))
            else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            data.analysis.reset_with_structured_diagnostics();
            *state = VelocityIntegralState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            StepResult::ReadyNow
        }
        VelocityIntegralState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            let now_us = backend.elapsed_us();
            if now_us >= *analysis_deadline_us
                || now_us
                    .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                    .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return fail_integral(phase, backend, CommissionError::AnalysisOverrun).await;
            }
            let (summary, stationarity) = match moving_analysis(data, safety.counts_per_rev()) {
                Ok(Some(result)) => result,
                Ok(None) => return StepResult::ReadyNow,
                Err(error) => return fail_integral(phase, backend, error).await,
            };
            if retain_analyzed_observation(data, summary).is_err() {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            data.emitted_observations = data.emitted_observations.saturating_add(1);
            data.observation_current_headroom =
                summary.classification == VelocityObservationClass::CurrentHeadroom;
            let terminal_error = match summary.classification {
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
            let report = VelocityIntegralObservation {
                run_sequence: data.integrity.run_sequence(),
                evidence_sequence: data.integrity.next_evidence_sequence(),
                plan_digest: data.plan.plan_digest,
                rung_index: data.rung_index,
                slot: data.slot,
                i_raw: integral_i_at(data.plan, data.rung_index).unwrap_or(0),
                summary,
                stationarity,
            };
            let settle_deadline_us = *settle_deadline_us;
            data.analysis.reset();
            data.observations.clear();
            *state = if let Some(error) = terminal_error {
                VelocityIntegralState::FaultTerminal { error }
            } else {
                VelocityIntegralState::BeginSettleCapture { settle_deadline_us }
            };
            let event = data
                .integrity
                .record(VelocityIntegralEvent::Observation(report));
            StepResult::VelocityIntegralEvent(event)
        }
        VelocityIntegralState::BeginSettleCapture { settle_deadline_us } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_integral(phase, backend, CommissionError::Spi).await,
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return fail_integral(phase, backend, CommissionError::AnalysisOverrun).await;
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = baseline_us;
            data.settle_displacement_counts = 0;
            data.settle_min_displacement_counts = 0;
            data.settle_max_displacement_counts = 0;
            *state = VelocityIntegralState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
            };
            StepResult::At(next_sample_us)
        }
        VelocityIntegralState::SettleCapture {
            next_sample_us,
            deadline_us,
        } => {
            if backend.elapsed_us() < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_integral(phase, backend, CommissionError::Spi).await,
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
                    return fail_integral_with_safety(phase, backend, fault).await;
                }
            };
            let domain = EncoderDomain::new(safety.counts_per_rev())
                .expect("Stage-C setup validates encoder domain");
            let delta = match domain.delta(
                domain.count(data.previous_encoder_raw),
                domain.count(encoder_raw),
            ) {
                Ok(delta) if delta.absolute_counts() < safety.quarter_turn_fault_counts => delta,
                _ => {
                    let fault = safety.fault(OUTER_SAFETY_FAULT_QUARTER_TURN, 0, elapsed_us, 0, 0);
                    return fail_integral_with_safety(phase, backend, fault).await;
                }
            };
            let torque_demand = match read_velocity_pid_input_abs(backend).await {
                Ok(value) => value,
                Err(error) => {
                    return fail_integral(phase, backend, current_guard_error(error)).await;
                }
            };
            if torque_demand > data.current_observer.clamp_limit() {
                let fault = safety.fault(OUTER_SAFETY_FAULT_CURRENT, 0, elapsed_us, 0, 0);
                return fail_integral_with_safety(phase, backend, fault).await;
            }
            let torque_target = match read_velocity_pid_output_abs(backend).await {
                Ok(value) => value,
                Err(error) => {
                    return fail_integral(phase, backend, current_guard_error(error)).await;
                }
            };
            data.current_observer.observe_zero_output(torque_target);
            let zero_peak = data.current_observer.zero_pid_output_peak();
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
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let Some(stroke_index) = integral_stroke_index(data.rung_index, data.slot) else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            backend.record_velocity_trace_observation(
                crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: 0,
                    delta_counts: delta.signed_counts(),
                    elapsed_us: elapsed_us as u32,
                    stroke_index: SETTLE_TRACE_STROKE_MARKER | stroke_index,
                    direction: 0,
                    flags: 0,
                    peak_torque_target_abs: zero_peak,
                    velocity_diagnostic_value: 0,
                },
            );
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = read_us;
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(data.plan.analysis_budget_us))
                else {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                };
                data.analysis.reset();
                *state = VelocityIntegralState::AnalyzeSettle {
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
                    Err(error) => fail_integral(phase, backend, error).await,
                }
            }
        }
        VelocityIntegralState::AnalyzeSettle {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return fail_integral(phase, backend, CommissionError::AnalysisOverrun).await;
            }
            match poll_settle_confirmation_parts(
                &mut data.analysis,
                &data.observations,
                &mut data.blocker,
                data.settle_min_displacement_counts,
                data.settle_max_displacement_counts,
            ) {
                Ok(Some(true)) => {}
                Ok(Some(false)) => {
                    return fail_integral(
                        phase,
                        backend,
                        CommissionError::VelocityValidationFailed,
                    )
                    .await;
                }
                Ok(None) => return StepResult::ReadyNow,
                Err(error) => return fail_integral(phase, backend, error).await,
            }
            if let Err(error) =
                finish_velocity_current_zero_phase(&mut data.current_observer, backend).await
            {
                return fail_integral(phase, backend, current_guard_error(error)).await;
            }
            let current_evidence = match data
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
            {
                Ok(evidence) => evidence,
                Err(error) => {
                    return fail_integral(phase, backend, current_guard_error(error)).await;
                }
            };
            if current_evidence.contact == VelocityCurrentContact::Braking {
                data.observation_current_headroom = true;
            }
            data.analysis.reset();
            data.observations.clear();
            safety.end_primitive();
            if data.observation_current_headroom {
                if matches!(
                    integral_rung_kind(data.plan, data.rung_index),
                    IntegralRungKind::Positive
                ) && positive_directional_floors_exist(data)
                {
                    data.current_terminus = data.rung_index.checked_sub(1);
                    match plan_integral_recovery(data, workspace, safety) {
                        Ok(recovery) => *state = VelocityIntegralState::RecoverRung(recovery),
                        Err(error) => return fail_integral(phase, backend, error).await,
                    }
                } else {
                    apply_integral_current_headroom(state, data);
                }
            } else if data.slot + 1 < INTEGRAL_SLOT_COUNT {
                *state = VelocityIntegralState::Advance;
            } else {
                *state = VelocityIntegralState::AnalyzeRung;
            }
            let event = data.integrity.record(VelocityIntegralEvent::Current(
                VelocityPrimitiveCurrentEvidence {
                    run_sequence: data.integrity.run_sequence(),
                    evidence_sequence: data.integrity.next_evidence_sequence(),
                    stage: RecoveryStage::Integral as u8,
                    rung_index: data.rung_index,
                    slot: data.slot,
                    clamp_limit: current_evidence.clamp_limit,
                    clamp_readback: current_evidence.clamp_limit,
                    moving_pid_output_peak: current_evidence.moving_pid_output_peak,
                    zero_pid_output_peak: current_evidence.zero_pid_output_peak,
                    moving_status_flags: current_evidence.moving_status_flags,
                    zero_status_flags: current_evidence.zero_status_flags,
                    capability: current_evidence.capability as u8,
                    contact: current_evidence.contact as u8,
                },
            ));
            StepResult::VelocityIntegralEvent(event)
        }
        VelocityIntegralState::Advance => {
            let finished = advance_integral_cursor(
                &mut data.rung_index,
                &mut data.slot,
                data.plan.positive_rung_count.saturating_add(2),
            );
            if finished || data.slot == 0 {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            *state = VelocityIntegralState::ClearSignedIntegrator;
            StepResult::ReadyNow
        }
        VelocityIntegralState::AnalyzeRung => {
            let directions = match (
                analyze_rung(data, VelocityDirection::Forward),
                analyze_rung(data, VelocityDirection::Reverse),
            ) {
                (Ok(forward), Ok(reverse)) => [forward, reverse],
                _ => return fail_integral(phase, backend, CommissionError::InvalidSchedule).await,
            };
            if retain_rung(data, workspace, directions).is_err() {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            data.emitted_rungs = data.emitted_rungs.saturating_add(1);
            let report = VelocityIntegralRungReport {
                run_sequence: data.integrity.run_sequence(),
                evidence_sequence: data.integrity.next_evidence_sequence(),
                plan_digest: data.plan.plan_digest,
                rung_index: data.rung_index,
                i_raw: integral_i_at(data.plan, data.rung_index).unwrap_or(0),
                directions,
            };
            let rung_kind = integral_rung_kind(data.plan, data.rung_index);
            match rung_kind {
                IntegralRungKind::OpeningAnchor if data.opening_direction_mask != 3 => {
                    *state = VelocityIntegralState::Terminal;
                }
                IntegralRungKind::OpeningAnchor
                | IntegralRungKind::Positive
                | IntegralRungKind::ClosingBookend => {
                    let recovery = match plan_integral_recovery(data, workspace, safety) {
                        Ok(runtime) => runtime,
                        Err(error) => return fail_integral(phase, backend, error).await,
                    };
                    *state = VelocityIntegralState::RecoverRung(recovery);
                }
            }
            let event = data.integrity.record(VelocityIntegralEvent::Rung(report));
            StepResult::VelocityIntegralEvent(event)
        }
        VelocityIntegralState::RecoverRung(runtime) => {
            let mut resources = RungOriginRecoveryResources {
                safety,
                observations: &mut data.observations,
                blocker: &mut data.blocker,
                analysis: &mut data.analysis,
                settle_displacement_counts: &mut data.settle_displacement_counts,
                settle_min_displacement_counts: &mut data.settle_min_displacement_counts,
                settle_max_displacement_counts: &mut data.settle_max_displacement_counts,
                current_observer: &mut data.current_observer,
            };
            match step_rung_origin_recovery(runtime, &mut resources, backend).await {
                RungOriginRecoveryStep::ReadyNow => StepResult::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => StepResult::At(deadline_us),
                RungOriginRecoveryStep::Finished(report)
                | RungOriginRecoveryStep::Contained(report) => {
                    *state = VelocityIntegralState::FinalizeRecovery(report);
                    StepResult::ReadyNow
                }
                RungOriginRecoveryStep::Fault {
                    error: _,
                    safety: Some(fault),
                } => fail_integral_with_safety(phase, backend, fault).await,
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => fail_integral(phase, backend, error).await,
            }
        }
        VelocityIntegralState::FinalizeRecovery(report) => {
            let report = *report;
            let contained = matches!(
                report.outcome,
                RungOriginRecoveryOutcome::ContainedNoRate
                    | RungOriginRecoveryOutcome::ContainedDeadline
            );
            if report.outcome == RungOriginRecoveryOutcome::RecoveryCurrentHeadroom {
                apply_integral_current_headroom(state, data);
            } else {
                match integral_rung_kind(data.plan, data.rung_index) {
                    IntegralRungKind::ClosingBookend => {
                        data.recovery_unavailable = contained;
                        *state = VelocityIntegralState::Terminal;
                    }
                    IntegralRungKind::Positive if data.current_terminus.is_some() && contained => {
                        data.recovery_unavailable = true;
                        *state = VelocityIntegralState::Terminal;
                    }
                    IntegralRungKind::OpeningAnchor | IntegralRungKind::Positive if contained => {
                        data.recovery_failure = true;
                        *state = VelocityIntegralState::Terminal;
                    }
                    IntegralRungKind::Positive if data.current_terminus.is_some() => {
                        data.rung_index = data.plan.positive_rung_count.saturating_add(1);
                        reset_rung(data);
                        *state = VelocityIntegralState::WriteGains;
                    }
                    IntegralRungKind::OpeningAnchor | IntegralRungKind::Positive => {
                        data.rung_index = data.rung_index.saturating_add(1);
                        *state = VelocityIntegralState::WriteGains;
                    }
                }
            }
            let event = data.integrity.record(VelocityIntegralEvent::Recovery(
                RungOriginRecoveryEvidence {
                    run_sequence: data.integrity.run_sequence(),
                    evidence_sequence: data.integrity.next_evidence_sequence(),
                    report,
                },
            ));
            StepResult::VelocityIntegralEvent(event)
        }
        VelocityIntegralState::FaultTerminal { error } => {
            let error = *error;
            fail_integral(phase, backend, error).await
        }
        VelocityIntegralState::Terminal => evidence_terminal(data, workspace),
    }
}
