//! Cooperative fixed-P Stage-C velocity-integral acquisition.

use super::integral_plan::IntegralSlotOrder;
use crate::autotune::velocity_measurement::SuffixDiagnostic;
use crate::autotune::velocity_measurement::{
    RestStructuralCode, StationarityExclusion, StationarityResult, VelocityDirection,
    estimate_confidence,
};
use crate::autotune::{CommissionError, TmcMechanicalRpm, TransientBuffer};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::state::{PositiveIRestBoundary, VelocitySettleMetrics};
use crate::commissioning::outer::{
    OuterPhase, OuterSafetyLimits, VelocityIntegralRuntime, VelocityIntegralState,
};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    FixedGainAmplitudeCause, FixedGainAmplitudeEvent, FixedGainAmplitudeEvidence,
    FixedGainAmplitudeOutcome, FixedGainAmplitudeTerminal, IntegralOutcomeCause,
    OUTER_SAFETY_FAULT_CURRENT, OUTER_SAFETY_FAULT_OBSERVATION_GAP,
    OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault, RecoveryRestDiagnostic,
    RestSelectionDiagnostic, RestSelectionOwner, RungOriginRecoveryEvidence, StepResult,
    VelocityIntegralEvent, VelocityIntegralObservation, VelocityIntegralOutcome,
    VelocityIntegralRungReport, VelocityIntegralTerminal, VelocityPrimitiveCurrentEvidence,
};
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_TARGET, PID_VELOCITY_ACTUAL,
    PID_VELOCITY_OFFSET, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use super::combined_plan::{CombinedRateInterval, integral_deficit_reaches_target};
use super::fixed_gain_amplitude_evidence::{
    FIXED_GAIN_AMPLITUDE_COUNT, FixedGainAmplitudeAvailability, FixedGainAmplitudeDirectionInput,
    FixedGainAmplitudeReport, FixedGainAmplitudeRunMasks, classify_fixed_gain_amplitude,
    fixed_gain_amplitude_run_sufficient,
};
use super::integral_evidence::{
    ExactRateInterval, IntegralInconclusiveKind, IntegralRunSufficiency, IntegralRungClass,
    IntegralRungEvidence, IntegralRungKind, analyze_integral_direction,
    classify_integral_observation,
};
use super::integral_plan::{
    AcceptanceSource, INTEGRAL_MAX_POSITIVE_RUNGS, INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US,
    VelocityIntegralAcquisitionKind, VelocityIntegralExecutionPlan, integral_gain_at,
};
use super::recovery::{
    RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryRestAnalysis, RecoveryStage,
    RungOriginRecoveryConfig, RungOriginRecoveryOutcome, RungOriginRecoveryResources,
    RungOriginRecoveryRuntime, RungOriginRecoveryStep, plan_rung_origin_recovery,
    step_rung_origin_recovery,
};
use super::sweep_analysis::{
    ScoredRestPoll, bounded_rest_displacement, deficit_interval_q, poll_scored_rest_confirmation,
    poll_stationarity,
};
use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, VelocityObservationSummary,
};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MIN_SETTLE_CAPTURE_US, VELOCITY_SWEEP_SAMPLE_PERIOD_US,
};
use super::velocity_clamp::{
    VelocityCurrentClampSession, VelocityCurrentContact, VelocityCurrentGuardError,
    VelocityCurrentZeroPhase, begin_velocity_current_primitive,
    finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    read_velocity_pid_input_abs, read_velocity_pid_output_abs,
};
use super::{clear_velocity_error_sum, clear_velocity_offset};

const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;
#[cfg(feature = "trace")]
const SETTLE_TRACE_STROKE_MARKER: u16 = 1 << 15;
const INTEGRAL_CAUSE_NONE: u8 = 0;
const INTEGRAL_CAUSE_TOO_FEW_RUNGS: u8 = 1;
const INTEGRAL_CAUSE_CURRENT_BEFORE_SUFFICIENCY: u8 = 2;
const INTEGRAL_CAUSE_CURRENT_AFTER_SUFFICIENCY: u8 = 3;
pub(super) const INTEGRAL_CAUSE_INTEGRITY: u8 = 4;
const INTEGRAL_CAUSE_RECOVERY: u8 = 5;
pub(super) const INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE: u8 = 8;
pub(super) const INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY: u8 = 9;
pub(super) const INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY_AFTER_SUFFICIENCY: u8 = 10;
const OPENING_WORD_BASE: usize = 0;
const AMPLITUDE_MEAN_WORD_BASE: usize = 0;
const POSITIVE_WORD_BASE: usize = 4;
const BOOKEND_WORD_BASE: usize = POSITIVE_WORD_BASE + 2 * INTEGRAL_MAX_POSITIVE_RUNGS * 2;

pub(super) const INTEGRAL_SLOT_COUNT: u8 = 8;
const _: () = assert!(
    INTEGRAL_SLOT_COUNT as u32 == super::integral_plan::INTEGRAL_OBSERVATIONS_PER_RUNG,
    "the engine slot count and the plan observation count are the same quantity"
);

pub(super) fn integral_rung_count(plan: VelocityIntegralExecutionPlan) -> Option<u8> {
    match plan.acquisition_kind {
        VelocityIntegralAcquisitionKind::ResponseLadder => plan.positive_rung_count.checked_add(2),
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
        | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending => {
            u8::try_from(plan.amplitude_targets_rpm.len()).ok()
        }
    }
}

pub(crate) fn integral_i_at(plan: VelocityIntegralExecutionPlan, rung_index: u8) -> Option<u16> {
    let total = integral_rung_count(plan)?;
    if !matches!(
        plan.acquisition_kind,
        VelocityIntegralAcquisitionKind::ResponseLadder
    ) {
        return (rung_index < total).then_some(plan.i_start);
    }
    if rung_index >= total {
        None
    } else if rung_index == 0 || rung_index + 1 == total {
        Some(0)
    } else {
        let positive_index = rung_index - 1;
        integral_gain_at(plan.i_top_cap, positive_index)
    }
}

pub(super) fn integral_target_rpm_at(
    plan: VelocityIntegralExecutionPlan,
    rung_index: u8,
) -> Option<i32> {
    match plan.acquisition_kind {
        VelocityIntegralAcquisitionKind::ResponseLadder => {
            (rung_index < integral_rung_count(plan)?).then_some(plan.target_velocity_rpm)
        }
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
        | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending => plan
            .amplitude_targets_rpm
            .get(usize::from(rung_index))
            .copied()
            .map(i32::from),
    }
}

pub(super) const fn integral_slot_direction(order: IntegralSlotOrder, slot: u8) -> Option<i32> {
    if slot >= INTEGRAL_SLOT_COUNT {
        None
    } else {
        Some(order.signed_direction(slot))
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

pub(super) fn integral_rung_kind(
    plan: VelocityIntegralExecutionPlan,
    rung_index: u8,
) -> IntegralRungKind {
    let total = integral_rung_count(plan).unwrap_or(0);
    match plan.acquisition_kind {
        VelocityIntegralAcquisitionKind::ResponseLadder if rung_index == 0 => {
            IntegralRungKind::OpeningAnchor
        }
        VelocityIntegralAcquisitionKind::ResponseLadder if rung_index + 1 == total => {
            IntegralRungKind::ClosingBookend
        }
        VelocityIntegralAcquisitionKind::ResponseLadder => IntegralRungKind::Positive,
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
        | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending => {
            IntegralRungKind::FixedGainAmplitude
        }
    }
}

pub(super) fn apply_integral_current_headroom(
    state: &mut VelocityIntegralState,
    data: &mut VelocityIntegralRuntime,
) {
    if matches!(
        data.plan.acquisition_kind,
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
            | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending
    ) {
        if data.plan.acquisition_kind
            == VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending
        {
            data.rung_index = data.rung_index.saturating_add(1);
            if data.rung_index < integral_rung_count(data.plan).unwrap_or(0) {
                *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
                return;
            }
        }
        *state = VelocityIntegralState::Terminal;
        return;
    }
    if matches!(
        integral_rung_kind(data.plan, data.rung_index),
        IntegralRungKind::Positive
    ) {
        data.current_terminus = data.rung_index.checked_sub(1);
    }
    data.recovery_unavailable = true;
    *state = VelocityIntegralState::Terminal;
}

pub(super) fn positive_directional_floors_exist(data: &VelocityIntegralRuntime) -> bool {
    data.eligible_masks
        .iter()
        .all(|mask| mask.count_ones() >= 3)
}

pub(super) fn record_integral_target_if_reached(
    data: &mut VelocityIntegralRuntime,
    intervals: [ExactRateInterval; 2],
) -> Result<bool, CommissionError> {
    if !data.plan.records_target_terminus() {
        return Ok(false);
    }
    if data.target_terminus.is_some() {
        return Ok(true);
    }
    if !matches!(
        integral_rung_kind(data.plan, data.rung_index),
        IntegralRungKind::Positive
    ) {
        return Ok(false);
    }
    let deficits = intervals.map(|interval| {
        CombinedRateInterval::new(i64::from(interval.low_q), i64::from(interval.high_q))
            .map_err(|_| CommissionError::InvalidSchedule)
    });
    let deficits = [deficits[0]?, deficits[1]?];
    if integral_deficit_reaches_target(
        data.plan.target_velocity_rpm,
        data.plan.encoder_counts_per_rev,
        deficits,
    )
    .map_err(|_| CommissionError::TargetRateInvalid)?
    {
        data.target_terminus = data.rung_index.checked_sub(1);
        Ok(true)
    } else {
        Ok(false)
    }
}

/// Record the breakaway target terminus on a closure that bypasses
/// `retain_rung`.
///
/// The terminus recorder is otherwise wired only into `retain_rung`, reachable
/// only from `AnalyzeRung`. Breakaway's peak rung drives the native integral to
/// its cap, never settles, and closes through the positive-I-rest-boundary or
/// current-headroom diversion instead -- neither reaches `retain_rung`, so the
/// terminus for the target-reaching rung would never install. This mirrors the
/// positive-rung branch of `retain_rung`: it reconstructs both directional
/// deficit consensuses from the retained per-slot evidence and records the
/// terminus when the deficit reaches the target. It is a no-op unless the plan
/// records a terminus and both directions reach an eligible consensus, so it is
/// safe on a diversion that consumed a partially swept rung.
fn record_diverted_target_terminus(
    data: &mut VelocityIntegralRuntime,
) -> Result<(), CommissionError> {
    let forward = analyze_rung(data, VelocityDirection::Forward)?;
    let reverse = analyze_rung(data, VelocityDirection::Reverse)?;
    if let [Some(forward), Some(reverse)] = [forward.interval, reverse.interval] {
        record_integral_target_if_reached(data, [forward, reverse])?;
    }
    Ok(())
}

pub(super) fn integral_recovery_current_contact_policy(
    data: &VelocityIntegralRuntime,
) -> RecoveryCurrentContactPolicy {
    let closes_positive_ladder = data.rung_index == data.plan.positive_rung_count
        || data.current_terminus.is_some()
        || data.positive_i_rest_boundary_after_sufficiency;
    if matches!(
        integral_rung_kind(data.plan, data.rung_index),
        IntegralRungKind::Positive
    ) && positive_directional_floors_exist(data)
        && closes_positive_ladder
    {
        RecoveryCurrentContactPolicy::ProvePassiveSettle
    } else {
        RecoveryCurrentContactPolicy::StopImmediately
    }
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

fn reset_rung(
    data: &mut VelocityIntegralRuntime,
    workspace: &mut TransientBuffer,
) -> Result<(), CommissionError> {
    data.slot = 0;
    data.rung_classes = [VelocityObservationClass::InvalidEvidenceFault; 8];
    data.rung_intervals = [ExactRateInterval::EMPTY; 8];
    if is_fixed_gain_amplitude(data.plan) {
        for slot in 0..usize::from(INTEGRAL_SLOT_COUNT) {
            if !workspace.write_workspace_word(AMPLITUDE_MEAN_WORD_BASE + slot, 0) {
                return Err(CommissionError::EvidenceBufferFull);
            }
        }
    }
    data.rung_blocking_levels = 0;
    Ok(())
}

pub(super) fn store_amplitude_rung_mean(
    workspace: &mut TransientBuffer,
    slot: usize,
    value: i32,
) -> Result<(), CommissionError> {
    if slot >= usize::from(INTEGRAL_SLOT_COUNT)
        || !workspace.write_workspace_word(AMPLITUDE_MEAN_WORD_BASE + slot, value as u32)
    {
        Err(CommissionError::RungIndexOutOfRange)
    } else {
        Ok(())
    }
}

pub(super) fn amplitude_rung_mean(
    workspace: &TransientBuffer,
    slot: usize,
) -> Result<i32, CommissionError> {
    if slot >= usize::from(INTEGRAL_SLOT_COUNT) {
        return Err(CommissionError::RungIndexOutOfRange);
    }
    workspace
        .workspace_word(AMPLITUDE_MEAN_WORD_BASE + slot)
        .map(|value| value as i32)
        .ok_or(CommissionError::MissingCaptureState)
}

fn set_rung_blocking_level(
    data: &mut VelocityIntegralRuntime,
    slot: u8,
    level: u8,
) -> Result<(), CommissionError> {
    if slot >= INTEGRAL_SLOT_COUNT || level > 0x0f {
        return Err(CommissionError::RungIndexOutOfRange);
    }
    let shift = u32::from(slot) * 4;
    data.rung_blocking_levels =
        (data.rung_blocking_levels & !(0x0f << shift)) | (u32::from(level) << shift);
    Ok(())
}

fn rung_blocking_level(data: &VelocityIntegralRuntime, slot: usize) -> u8 {
    ((data.rung_blocking_levels >> (slot * 4)) & 0x0f) as u8
}

fn fixed_gain_amplitude_attempted_masks(data: &VelocityIntegralRuntime) -> [u8; 2] {
    [data.opening_direction_mask, data.bookend_available_mask]
}

fn set_fixed_gain_amplitude_attempted(
    data: &mut VelocityIntegralRuntime,
    direction: usize,
    bit: u8,
) {
    if direction == 0 {
        data.opening_direction_mask |= bit;
    } else {
        data.bookend_available_mask |= bit;
    }
}

fn fixed_gain_amplitude_current_masks(data: &VelocityIntegralRuntime) -> [u8; 2] {
    [
        data.current_terminus.unwrap_or(0),
        data.target_terminus.unwrap_or(0),
    ]
}

fn set_fixed_gain_amplitude_current(data: &mut VelocityIntegralRuntime, direction: usize, bit: u8) {
    let value = if direction == 0 {
        &mut data.current_terminus
    } else {
        &mut data.target_terminus
    };
    *value = Some(value.unwrap_or(0) | bit);
}

pub(super) fn is_fixed_gain_amplitude(plan: VelocityIntegralExecutionPlan) -> bool {
    matches!(
        plan.acquisition_kind,
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
            | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending
    )
}

fn fixed_gain_amplitude_target_rate_q(
    plan: VelocityIntegralExecutionPlan,
    rung_index: u8,
) -> Result<i64, CommissionError> {
    let target_rpm =
        integral_target_rpm_at(plan, rung_index).ok_or(CommissionError::InvalidSchedule)?;
    i64::from(target_rpm)
        .checked_mul(i64::from(plan.encoder_counts_per_rev))
        .and_then(|value| {
            value.checked_mul(crate::autotune::velocity_measurement::VELOCITY_RATE_Q_SCALE)
        })
        .map(|value| value / 60)
        .ok_or(CommissionError::InvalidSchedule)
}

fn acceptance_identity_and_qualifiers(plan: VelocityIntegralExecutionPlan) -> Option<(u64, u16)> {
    if !is_fixed_gain_amplitude(plan) {
        return None;
    }
    Some((plan.acceptance_plan_digest, plan.qualifier_bits))
}

pub(super) fn fixed_gain_amplitude_report(
    data: &mut VelocityIntegralRuntime,
    workspace: &TransientBuffer,
) -> Result<FixedGainAmplitudeReport, CommissionError> {
    let amplitude = usize::from(data.rung_index);
    let bit = 1_u8
        .checked_shl(u32::from(data.rung_index))
        .ok_or(CommissionError::InvalidSchedule)?;
    let target_rate_q = fixed_gain_amplitude_target_rate_q(data.plan, data.rung_index)?;
    let precision_floor_candidate = integral_target_rpm_at(data.plan, data.rung_index)
        == data
            .plan
            .amplitude_targets_rpm
            .iter()
            .copied()
            .min()
            .map(i32::from);
    let slot_order = data.plan.slot_order;
    let directions: [Result<_, CommissionError>; 2] = core::array::from_fn(|direction| {
        // Must mirror the write side: evidence is stored by raw slot index.
        let slot = |observation: usize| usize::from(slot_order.slot_for(observation, direction));
        let classes = core::array::from_fn(|observation| data.rung_classes[slot(observation)]);
        let intervals = core::array::from_fn(|observation| {
            (classes[observation] == VelocityObservationClass::Eligible).then(|| {
                let interval = data.rung_intervals[slot(observation)];
                [interval.low_q, interval.high_q]
            })
        });
        let means_q: [Result<Option<i32>, CommissionError>; 4] =
            core::array::from_fn(|observation| {
                (classes[observation] == VelocityObservationClass::Eligible)
                    .then(|| amplitude_rung_mean(workspace, slot(observation)))
                    .transpose()
            });
        let means_q = [means_q[0]?, means_q[1]?, means_q[2]?, means_q[3]?];
        let blocking_levels =
            core::array::from_fn(|observation| rung_blocking_level(data, slot(observation)));
        classify_fixed_gain_amplitude(FixedGainAmplitudeDirectionInput {
            signed_target_rate_q: if direction == 0 {
                target_rate_q
            } else {
                -target_rate_q
            },
            collected_mask: 0b1111,
            classes,
            intervals,
            means_q,
            blocking_levels,
            precision_floor_candidate,
        })
        .map_err(|_| CommissionError::InvalidSchedule)
    });
    let directions = [
        directions[0].map_err(|_| CommissionError::InvalidSchedule)?,
        directions[1].map_err(|_| CommissionError::InvalidSchedule)?,
    ];
    for (direction, evidence) in directions.iter().enumerate() {
        set_fixed_gain_amplitude_attempted(data, direction, bit);
        if evidence.availability == FixedGainAmplitudeAvailability::Eligible {
            data.eligible_masks[direction] |= u32::from(bit);
        }
        if evidence.availability == FixedGainAmplitudeAvailability::CurrentHeadroom {
            set_fixed_gain_amplitude_current(data, direction, bit);
        }
    }
    Ok(FixedGainAmplitudeReport {
        amplitude_index: u8::try_from(amplitude).map_err(|_| CommissionError::InvalidSchedule)?,
        target_rpm: i16::try_from(
            integral_target_rpm_at(data.plan, data.rung_index)
                .ok_or(CommissionError::InvalidSchedule)?,
        )
        .map_err(|_| CommissionError::InvalidSchedule)?,
        directions,
    })
}

pub(super) fn fixed_gain_amplitude_terminal(
    data: &VelocityIntegralRuntime,
    faulted: bool,
    safety_fault: Option<OuterSafetyFault>,
) -> FixedGainAmplitudeTerminal {
    let eligible_masks = [
        u8::try_from(data.eligible_masks[0])
            .expect("amplitude eligibility is written only through five-bit masks"),
        u8::try_from(data.eligible_masks[1])
            .expect("amplitude eligibility is written only through five-bit masks"),
    ];
    let full_mask = (1_u8 << FIXED_GAIN_AMPLITUDE_COUNT) - 1;
    let masks = FixedGainAmplitudeRunMasks {
        attempted: fixed_gain_amplitude_attempted_masks(data),
        eligible: eligible_masks,
        current_terminus: fixed_gain_amplitude_current_masks(data),
        unattempted: fixed_gain_amplitude_attempted_masks(data).map(|mask| full_mask & !mask),
    };
    let identity = acceptance_identity_and_qualifiers(data.plan);
    let (outcome, cause) = if faulted || identity.is_none() {
        (
            FixedGainAmplitudeOutcome::Fault,
            FixedGainAmplitudeCause::EvidenceIntegrity,
        )
    } else {
        match fixed_gain_amplitude_run_sufficient(masks) {
            Err(_) => (
                FixedGainAmplitudeOutcome::Fault,
                FixedGainAmplitudeCause::EvidenceIntegrity,
            ),
            Ok(_) if data.rest_unconfirmed => (
                FixedGainAmplitudeOutcome::Inconclusive,
                FixedGainAmplitudeCause::VelocityRestNotConfirmed,
            ),
            Ok(_) if data.recovery_failure || data.recovery_unavailable => (
                FixedGainAmplitudeOutcome::Inconclusive,
                FixedGainAmplitudeCause::RecoveryUnavailable,
            ),
            Ok(true) => (
                FixedGainAmplitudeOutcome::Complete,
                FixedGainAmplitudeCause::None,
            ),
            Ok(false) => (
                FixedGainAmplitudeOutcome::Inconclusive,
                FixedGainAmplitudeCause::InsufficientSharedFloor,
            ),
        }
    };
    let (acceptance_plan_digest, qualifier_bits) = identity.unwrap_or((0, 0));
    FixedGainAmplitudeTerminal {
        schema_revision: super::fixed_gain_amplitude_plan::FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION,
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        acceptance_plan_digest,
        plan_digest: data.plan.plan_digest,
        digest: data.integrity.digest(),
        outcome,
        cause,
        attempted_masks: masks.attempted,
        eligible_masks: masks.eligible,
        current_terminus_masks: masks.current_terminus,
        unattempted_masks: masks.unattempted,
        expected_observations: u16::from(INTEGRAL_SLOT_COUNT)
            * u16::try_from(FIXED_GAIN_AMPLITUDE_COUNT)
                .expect("fixed amplitude count fits terminal"),
        emitted_observations: data.emitted_observations,
        expected_amplitudes: u8::try_from(FIXED_GAIN_AMPLITUDE_COUNT)
            .expect("fixed amplitude count fits terminal"),
        emitted_amplitudes: u8::try_from(data.emitted_rungs)
            .expect("emitted amplitude count cannot exceed the five-rung plan"),
        qualifier_bits,
        run_started_us: data.run_started_us,
        run_completed_us: data.previous_read_us,
        safety_fault,
    }
}

fn transition_elapsed_us(started_us: u64, finished_us: u64) -> Result<u32, CommissionError> {
    let elapsed = finished_us
        .checked_sub(started_us)
        .ok_or(CommissionError::InvalidSchedule)?;
    u32::try_from(elapsed).map_err(|_| CommissionError::InvalidSchedule)
}

pub(super) fn combined_slot_transition_us(
    entry_transition_us: u32,
    exit_transition_us: u32,
) -> Option<u32> {
    let total = entry_transition_us.checked_add(exit_transition_us)?;
    if total > INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US {
        None
    } else {
        Some(total)
    }
}

fn interval_word_index(direction: usize, rung: usize) -> Option<usize> {
    if direction >= 2 || rung >= INTEGRAL_MAX_POSITIVE_RUNGS {
        None
    } else {
        Some(POSITIVE_WORD_BASE + (direction * INTEGRAL_MAX_POSITIVE_RUNGS + rung) * 2)
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
        Err(CommissionError::EvidenceBufferFull)
    }
}

fn load_interval(
    workspace: &TransientBuffer,
    word_index: usize,
) -> Result<ExactRateInterval, CommissionError> {
    ExactRateInterval::new(
        workspace
            .workspace_word(word_index)
            .ok_or(CommissionError::MissingCaptureState)? as i32,
        workspace
            .workspace_word(word_index + 1)
            .ok_or(CommissionError::MissingCaptureState)? as i32,
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
        .ok_or(CommissionError::TargetRateInvalid)?;
    let settled_low = super::sweep_analysis::floor_div_60(
        target_numerator
            .checked_sub(i128::from(deficit.high_q) * 60)
            .ok_or(CommissionError::TargetRateInvalid)?,
    )?;
    let settled_high = super::sweep_analysis::ceil_div_60(
        target_numerator
            .checked_sub(i128::from(deficit.low_q) * 60)
            .ok_or(CommissionError::TargetRateInvalid)?,
    )?;
    match direction {
        0 if settled_low > 0 => u64::try_from(settled_low)
            .map(Some)
            .map_err(|_| CommissionError::TargetRateInvalid),
        1 if settled_high < 0 => u64::try_from(-settled_high)
            .map(Some)
            .map_err(|_| CommissionError::TargetRateInvalid),
        0 | 1 => Ok(None),
        _ => unreachable!("direction bounds are validated above"),
    }
}

pub(super) fn plan_integral_recovery(
    data: &mut VelocityIntegralRuntime,
    workspace: &TransientBuffer,
    safety: &OuterSafetyLimits,
) -> Result<RungOriginRecoveryRuntime, CommissionError> {
    let direction = usize::from(safety.unwrapped_position >= 0);
    let target_velocity_rpm = if is_fixed_gain_amplitude(data.plan) {
        data.plan.target_velocity_rpm
    } else {
        integral_target_rpm_at(data.plan, data.rung_index)
            .ok_or(CommissionError::InvalidSchedule)?
    };
    let lower_rate_q = match data.plan.acquisition_kind {
        VelocityIntegralAcquisitionKind::ResponseLadder => opening_recovery_lower_rate_q(
            workspace,
            data.opening_direction_mask,
            data.plan.target_velocity_rpm,
            safety.counts_per_rev(),
            direction,
        )?,
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
        | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending => Some(
            *data
                .plan
                .recovery_lower_rate_q
                .get(direction)
                .filter(|value| **value != 0)
                .ok_or(CommissionError::InvalidSchedule)?,
        ),
    };
    let planned_velocity_mrev_s = if is_fixed_gain_amplitude(data.plan) {
        data.plan.planned_velocity_mrev_s
    } else {
        u32::try_from(target_velocity_rpm)
            .ok()
            .and_then(|target| target.checked_mul(1_000))
            .map(|target| target / 60)
            .ok_or(CommissionError::InvalidSchedule)?
    };
    let recovery_plan = plan_rung_origin_recovery(
        safety.budget,
        safety.counts_per_rev(),
        planned_velocity_mrev_s,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    let clamp_limit = PidTorqueFluxLimit::new(u32::from(data.plan.usable_torque_limit))
        .map_err(|_| CommissionError::InvalidSchedule)?;
    data.current_observer
        .reset(clamp_limit, data.plan.limit_evidence_capability);
    RungOriginRecoveryRuntime::new(
        RungOriginRecoveryConfig {
            stage: RecoveryStage::Integral,
            rung_index: data.rung_index,
            p: PidPGain::new(data.plan.fixed_p).expect("validated Stage-C P remains nonzero"),
            target_velocity_rpm: TmcMechanicalRpm::from_register(target_velocity_rpm),
            planned_velocity_mrev_s,
            start_offset_counts: safety.unwrapped_position,
            lower_rate_q,
            authority_source: if lower_rate_q.is_some() {
                RecoveryBindingSource::SettledRate
            } else {
                RecoveryBindingSource::MissingRate
            },
            current_contact_policy: integral_recovery_current_contact_policy(data),
        },
        recovery_plan,
    )
}

pub(super) fn record_integral_recovery_rest(
    data: &mut VelocityIntegralRuntime,
    analysis: RecoveryRestAnalysis,
) -> StepResult {
    let diagnostic = RecoveryRestDiagnostic {
        plan_digest: data.plan.plan_digest,
        exact_velocity_interval_q: analysis.exact_velocity_interval_q,
        run_sequence: data.integrity.run_sequence(),
        start_offset_counts: analysis.start_offset_counts,
        closest_offset_counts: analysis.closest_offset_counts,
        final_offset_counts: analysis.final_offset_counts,
        planned_return_distance_counts: analysis.planned_return_distance_counts,
        first_in_band_offset_counts: analysis.first_in_band_offset_counts,
        displacement_to_band_counts: analysis.displacement_to_band_counts,
        entry_to_band_duration_us: analysis.entry_to_band_duration_us,
        lead_in_net_displacement_counts: analysis.lead_in.net_displacement_counts,
        lead_in_displacement_span_counts: analysis.lead_in.displacement_span_counts,
        scored_net_displacement_counts: analysis.scored_net_displacement_counts,
        scored_min_displacement_counts: analysis.scored_min_displacement_counts,
        scored_max_displacement_counts: analysis.scored_max_displacement_counts,
        p_raw: analysis.p_raw,
        evidence_sequence: data.integrity.next_evidence_sequence(),
        peak_torque_target_abs: analysis.peak_torque_target_abs,
        stage: analysis.stage,
        rung_index: analysis.rung_index,
        stationarity_code: analysis.stationarity_code,
        interval_present: analysis.interval_present,
        current_contact_code: analysis.current_contact_code,
        disposition: analysis.disposition,
    };
    if is_fixed_gain_amplitude(data.plan) {
        let event = data
            .integrity
            .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::RecoveryRest(diagnostic));
        StepResult::FixedGainAmplitudeEvent(event)
    } else {
        let event = data
            .integrity
            .record(VelocityIntegralEvent::RecoveryRest(diagnostic));
        StepResult::VelocityIntegralEvent(event)
    }
}

pub(super) fn record_integral_recovery_rest_selection(
    data: &mut VelocityIntegralRuntime,
    decision: crate::autotune::velocity_measurement::RestSelectionDecision,
    stage: RecoveryStage,
    rung_index: u8,
) -> StepResult {
    let diagnostic = RestSelectionDiagnostic {
        intervals_q: decision.intervals_q,
        plan_digest: data.plan.plan_digest,
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        selected_suffix_len: decision.selected_suffix_len,
        structural_codes: decision.structural_codes,
        interval_present_mask: decision.interval_present_mask,
        usable_mask: decision.usable_mask,
        zero_compatible_mask: decision.zero_compatible_mask,
        selected_mask: decision.selected_mask,
        stage,
        owner: if is_fixed_gain_amplitude(data.plan) {
            RestSelectionOwner::FixedGainAmplitudeRecovery
        } else {
            RestSelectionOwner::IntegralRecovery
        },
        rung_index,
        direction: u8::MAX,
        slot: u8::MAX,
        disposition: decision.disposition,
        retained_sample_count: decision.retained_sample_count,
    };
    if is_fixed_gain_amplitude(data.plan) {
        let event = data
            .integrity
            .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::RestSelection(diagnostic));
        StepResult::FixedGainAmplitudeEvent(event)
    } else {
        let event = data
            .integrity
            .record(VelocityIntegralEvent::RestSelection(diagnostic));
        StepResult::VelocityIntegralEvent(event)
    }
}

pub(super) fn record_integral_observation_rest_selection(
    data: &mut VelocityIntegralRuntime,
    decision: crate::autotune::velocity_measurement::RestSelectionDecision,
    owner: RestSelectionOwner,
) -> StepResult {
    let diagnostic = RestSelectionDiagnostic {
        intervals_q: decision.intervals_q,
        plan_digest: data.plan.plan_digest,
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        selected_suffix_len: decision.selected_suffix_len,
        structural_codes: decision.structural_codes,
        interval_present_mask: decision.interval_present_mask,
        usable_mask: decision.usable_mask,
        zero_compatible_mask: decision.zero_compatible_mask,
        selected_mask: decision.selected_mask,
        stage: RecoveryStage::Integral,
        owner,
        rung_index: data.rung_index,
        direction: data.plan.slot_order.direction_index(data.slot) as u8,
        slot: data.slot,
        disposition: decision.disposition,
        retained_sample_count: decision.retained_sample_count,
    };
    if is_fixed_gain_amplitude(data.plan) {
        let event = data
            .integrity
            .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::RestSelection(diagnostic));
        StepResult::FixedGainAmplitudeEvent(event)
    } else {
        let event = data
            .integrity
            .record(VelocityIntegralEvent::RestSelection(diagnostic));
        StepResult::VelocityIntegralEvent(event)
    }
}

fn materialize_run_evidence(
    data: &VelocityIntegralRuntime,
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

fn direction_for_slot(
    order: IntegralSlotOrder,
    slot: u8,
) -> Result<VelocityDirection, CommissionError> {
    match integral_slot_direction(order, slot) {
        Some(1) => Ok(VelocityDirection::Forward),
        Some(-1) => Ok(VelocityDirection::Reverse),
        _ => Err(CommissionError::InvalidSchedule),
    }
}

pub(super) fn terminal_from_data(
    data: &VelocityIntegralRuntime,
    evidence: super::integral_evidence::IntegralRunEvidence,
    outcome: VelocityIntegralOutcome,
    cause: IntegralOutcomeCause,
    safety_fault: Option<OuterSafetyFault>,
    rest_rejection_after_sufficiency: bool,
    rest_rejection_owner: Option<RestSelectionOwner>,
) -> StepResult {
    StepResult::VelocityIntegralTerminal(VelocityIntegralTerminal {
        run_sequence: data.integrity.run_sequence(),
        evidence_sequence: data.integrity.next_evidence_sequence(),
        plan_digest: data.plan.plan_digest,
        digest: data.integrity.digest(),
        outcome,
        cause,
        breakaway_acceptance: data.plan.acceptance_source == AcceptanceSource::Breakaway,
        fixed_p: data.plan.fixed_p,
        recovery_unavailable: data.recovery_unavailable,
        recovered_with_current_headroom: data.recovered_with_current_headroom,
        recovery_quantization_exposed: data.plan.recovery_quantization_exposed,
        probe_constrained_test_point: data.plan.probe_constrained_test_point,
        rest_boundary_rung_plus_one: data.target_terminus.map_or(0, |rung| rung + 1),
        rest_boundary_slot_plus_one: 0,
        evidence,
        expected_observations: u16::from(integral_rung_count(data.plan).unwrap_or(0))
            .saturating_mul(u16::from(INTEGRAL_SLOT_COUNT)),
        emitted_observations: data.integrity.emitted_observations(),
        expected_rungs: u16::from(integral_rung_count(data.plan).unwrap_or(0)),
        emitted_rungs: data.integrity.emitted_rungs(),
        run_started_us: data.run_started_us,
        run_completed_us: data.previous_read_us,
        safety_fault,
        rest_rejection_after_sufficiency,
        rest_rejection_owner,
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
    if is_fixed_gain_amplitude(data.plan) {
        let _ = error;
        return StepResult::FixedGainAmplitudeTerminal(fixed_gain_amplitude_terminal(
            data,
            true,
            safety_fault,
        ));
    }
    terminal_from_data(
        data,
        super::integral_evidence::IntegralRunEvidence::new([ExactRateInterval::EMPTY; 2]),
        VelocityIntegralOutcome::Fault,
        IntegralOutcomeCause::Error(error.status_code()),
        safety_fault,
        false,
        None,
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
    data: &VelocityIntegralRuntime,
    slot: usize,
    direction: VelocityDirection,
) -> Result<VelocityObservationSummary, CommissionError> {
    let p = PidPGain::new(data.plan.fixed_p).map_err(|_| CommissionError::InvalidSchedule)?;
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
        drift_ratio_ppm: u32::MAX,
        initial_lead_in: data.rung_index == 0 && slot == 0,
    })
}

pub(super) fn analyze_rung(
    data: &VelocityIntegralRuntime,
    direction: VelocityDirection,
) -> Result<IntegralRungEvidence, CommissionError> {
    let direction_index = match direction {
        VelocityDirection::Forward => 0,
        VelocityDirection::Reverse => 1,
    };
    // Must mirror the write side: evidence is stored by raw slot index.
    let observations = core::array::from_fn(|index| {
        let slot = usize::from(data.plan.slot_order.slot_for(index, direction_index));
        compact_summary(data, slot, direction)
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
    data: &mut VelocityIntegralRuntime,
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
            if let [Some(forward), Some(reverse)] = [directions[0].interval, directions[1].interval]
            {
                record_integral_target_if_reached(data, [forward, reverse])?;
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
        IntegralRungKind::FixedGainAmplitude => {}
    }
    Ok(())
}

fn evidence_terminal(data: &VelocityIntegralRuntime, workspace: &TransientBuffer) -> StepResult {
    let evidence = match materialize_run_evidence(data, workspace) {
        Ok(evidence) => evidence,
        Err(_) => {
            return terminal_from_data(
                data,
                super::integral_evidence::IntegralRunEvidence::new([ExactRateInterval::EMPTY; 2]),
                VelocityIntegralOutcome::Fault,
                IntegralOutcomeCause::Engine(INTEGRAL_CAUSE_INTEGRITY),
                None,
                false,
                None,
            );
        }
    };
    let assessment = evidence.assess(usize::from(data.plan.positive_rung_count));
    if data.rest_unconfirmed {
        let sufficient_before_rest_rejection =
            matches!(assessment, Ok(IntegralRunSufficiency::Sufficient { .. }));
        if sufficient_before_rest_rejection {
            if matches!(
                data.rest_unconfirmed_owner,
                Some(RestSelectionOwner::FixedGainAmplitudeObservation)
                    | Some(RestSelectionOwner::FixedGainAmplitudeRecovery)
            ) {
                return terminal_from_data(
                    data,
                    evidence,
                    VelocityIntegralOutcome::Fault,
                    IntegralOutcomeCause::Engine(INTEGRAL_CAUSE_INTEGRITY),
                    None,
                    false,
                    None,
                );
            }
            return terminal_from_data(
                data,
                evidence,
                VelocityIntegralOutcome::Inconclusive,
                IntegralOutcomeCause::Error(
                    CommissionError::VelocityRestNotConfirmed.status_code(),
                ),
                None,
                true,
                data.rest_unconfirmed_owner,
            );
        }
        return terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::Inconclusive,
            IntegralOutcomeCause::Error(CommissionError::VelocityRestNotConfirmed.status_code()),
            None,
            false,
            None,
        );
    }
    if data.recovery_failure {
        return terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::Inconclusive,
            IntegralOutcomeCause::Engine(INTEGRAL_CAUSE_RECOVERY),
            None,
            false,
            None,
        );
    }
    match assessment {
        Ok(IntegralRunSufficiency::Sufficient {
            current_terminus, ..
        }) => terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::FirstRunRetained,
            IntegralOutcomeCause::Engine(if data.positive_i_rest_boundary_after_sufficiency {
                INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY_AFTER_SUFFICIENCY
            } else if current_terminus.is_some() {
                INTEGRAL_CAUSE_CURRENT_AFTER_SUFFICIENCY
            } else {
                INTEGRAL_CAUSE_NONE
            }),
            None,
            false,
            None,
        ),
        Ok(IntegralRunSufficiency::Inconclusive { cause, .. }) => terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::Inconclusive,
            IntegralOutcomeCause::Engine(
                if data.positive_i_rest_boundary.is_some()
                    && !data.positive_i_rest_boundary_after_sufficiency
                {
                    INTEGRAL_CAUSE_POSITIVE_I_REST_BOUNDARY
                } else if data.opening_direction_mask != 3 && data.observation_current_headroom {
                    INTEGRAL_CAUSE_CURRENT_BEFORE_SUFFICIENCY
                } else {
                    match cause {
                        IntegralInconclusiveKind::TooFewEligibleRungs => {
                            INTEGRAL_CAUSE_TOO_FEW_RUNGS
                        }
                        IntegralInconclusiveKind::CurrentBoundBeforeSufficiency => {
                            INTEGRAL_CAUSE_CURRENT_BEFORE_SUFFICIENCY
                        }
                        IntegralInconclusiveKind::BookendUnavailable => {
                            INTEGRAL_CAUSE_BOOKEND_UNAVAILABLE
                        }
                    }
                },
            ),
            None,
            false,
            None,
        ),
        Err(_) => terminal_from_data(
            data,
            evidence,
            VelocityIntegralOutcome::Fault,
            IntegralOutcomeCause::Engine(INTEGRAL_CAUSE_INTEGRITY),
            None,
            false,
            None,
        ),
    }
}

fn current_report(
    data: &mut VelocityIntegralRuntime,
    current_evidence: super::velocity_clamp::VelocityCurrentPhaseEvidence,
) -> VelocityPrimitiveCurrentEvidence {
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
    }
}

type MovingAnalysis = (
    VelocityObservationSummary,
    StationarityResult,
    Option<SuffixDiagnostic>,
);

fn moving_analysis(
    data: &mut VelocityIntegralRuntime,
    counts_per_rev: u32,
) -> Result<Option<MovingAnalysis>, CommissionError> {
    let Some((stationarity, rejection, drift_ratio_ppm)) =
        poll_stationarity(&mut data.analysis, &data.observations, &mut data.blocker)?
    else {
        return Ok(None);
    };
    let p = PidPGain::new(data.plan.fixed_p).map_err(|_| CommissionError::InvalidSchedule)?;
    let direction = direction_for_slot(data.plan.slot_order, data.slot)?;
    let mut input = VelocityObservationInput {
        direction,
        p,
        stationarity,
        settled_rate_low_q: 0,
        settled_rate_high_q: 0,
        settled_rate_mean_q: 0,
        variance_of_mean_q2: 0,
        deficit_low_q: 0,
        deficit_high_q: 0,
        disturbance: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: data.current_observer.moving_pid_output_peak(),
        usable_torque_limit: data.plan.usable_torque_limit,
        hard_torque_limit: data.plan.hard_torque_limit,
        initial_lead_in: data.rung_index == 0 && data.slot == 0,
        drift_ratio_ppm,
        post_fit_lag_one_state: 0,
    };
    if let StationarityResult::Usable(suffix) = stationarity {
        input.drift_ratio_ppm = suffix.drift_ratio_ppm;
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
        input.variance_of_mean_q2 = confidence.variance_of_mean_q2;
        input.post_fit_lag_one_state = 2;
        input.settled_rate_low_q = mean_rate_q.saturating_sub(half_width);
        input.settled_rate_high_q = mean_rate_q.saturating_add(half_width);
        let signed_target = integral_target_rpm_at(data.plan, data.rung_index)
            .ok_or(CommissionError::InvalidSchedule)?
            .checked_mul(integral_slot_direction(data.plan.slot_order, data.slot).unwrap_or(0))
            .ok_or(CommissionError::InvalidSchedule)?;
        (input.deficit_low_q, input.deficit_high_q) = deficit_interval_q(
            signed_target,
            counts_per_rev,
            input.settled_rate_low_q,
            input.settled_rate_high_q,
        )?;
    }
    Ok(Some((
        classify_integral_observation(input),
        stationarity,
        rejection,
    )))
}

fn retain_analyzed_observation(
    data: &mut VelocityIntegralRuntime,
    workspace: &mut TransientBuffer,
    summary: VelocityObservationSummary,
) -> Result<(), CommissionError> {
    let source_interval = match data.plan.acquisition_kind {
        VelocityIntegralAcquisitionKind::ResponseLadder => summary.deficit_interval_q,
        VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
        | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending => {
            summary.settled_rate_interval_q
        }
    };
    let interval = ExactRateInterval::new(
        i32::try_from(source_interval[0]).map_err(|_| CommissionError::InvalidSchedule)?,
        i32::try_from(source_interval[1]).map_err(|_| CommissionError::InvalidSchedule)?,
    )
    .map_err(|_| CommissionError::InvalidSchedule)?;
    data.rung_classes[usize::from(data.slot)] = summary.classification;
    data.rung_intervals[usize::from(data.slot)] = interval;
    if is_fixed_gain_amplitude(data.plan)
        && summary.classification == VelocityObservationClass::Eligible
    {
        store_amplitude_rung_mean(
            workspace,
            usize::from(data.slot),
            i32::try_from(summary.settled_rate_mean_q)
                .map_err(|_| CommissionError::InvalidSchedule)?,
        )?;
    }
    Ok(())
}

/// Advance the Stage-C response sweep by one cooperative state-machine step.
///
/// This step compiles into the default (production) build; only the raw
/// trace emission it may call into is gated behind the `trace` feature.
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
                || integral_rung_count(plan).is_none_or(|count| count < 3)
                || plan.hard_torque_limit != u16::from(backend.current_limit())
            {
                return integral_fault_terminal(phase, CommissionError::InvalidSchedule, None);
            }
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return integral_fault_terminal(phase, CommissionError::Spi, None),
            };
            if safety
                .configure_integral_encoder_resolution(counts_per_rev)
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
            let velocity_offset = backend
                .read_register(PID_VELOCITY_OFFSET)
                .await
                .map(|value| value as i32)
                .unwrap_or(i32::MIN);
            let velocity_actual = backend
                .read_capture_register(PID_VELOCITY_ACTUAL)
                .await
                .map(|value| value as i32)
                .unwrap_or(i32::MIN);
            defmt::info!(
                "integral sweep start: velocity_offset={} velocity_actual={}",
                velocity_offset,
                velocity_actual,
            );
            if clear_velocity_offset(backend).await.is_err() {
                return integral_fault_terminal(phase, CommissionError::Spi, None);
            }
            *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
            StepResult::ReadyNow
        }
        VelocityIntegralState::PrepareMovingSlot {
            reset_rung: should_reset_rung,
        } => {
            let plan = data.plan;
            let Some(i_raw) = integral_i_at(plan, data.rung_index) else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let p = match PidPGain::new(plan.fixed_p) {
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
            let transition_started_us = backend.elapsed_us();
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || clear_velocity_error_sum(backend).await.is_err()
                || backend
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
            let entry_transition_us =
                match transition_elapsed_us(transition_started_us, backend.elapsed_us()) {
                    Ok(value) => value,
                    Err(error) => return fail_integral(phase, backend, error).await,
                };
            if *should_reset_rung && let Err(error) = reset_rung(data, workspace) {
                return fail_integral(phase, backend, error).await;
            }
            *state = VelocityIntegralState::CommandTarget {
                entry_transition_us,
            };
            StepResult::ReadyNow
        }
        VelocityIntegralState::CommandTarget {
            entry_transition_us,
        } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_integral(phase, backend, CommissionError::Spi).await,
            };
            let baseline_us = backend.elapsed_us();
            let target =
                match integral_target_rpm_at(data.plan, data.rung_index).and_then(|target| {
                    integral_slot_direction(data.plan.slot_order, data.slot)
                        .and_then(|direction| target.checked_mul(direction))
                }) {
                    Some(value) => value,
                    None => {
                        return fail_integral(phase, backend, CommissionError::InvalidSchedule)
                            .await;
                    }
                };
            if is_fixed_gain_amplitude(data.plan) {
                let bit = match 1_u8.checked_shl(u32::from(data.rung_index)) {
                    Some(value) => value,
                    None => {
                        return fail_integral(phase, backend, CommissionError::InvalidSchedule)
                            .await;
                    }
                };
                let direction = data.plan.slot_order.direction_index(data.slot);
                set_fixed_gain_amplitude_attempted(data, direction, bit);
            }
            let mut clamp_session = match VelocityCurrentClampSession::try_new_with_limit(
                u32::from(data.plan.hard_torque_limit),
                u32::from(data.plan.usable_torque_limit),
            ) {
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
                entry_transition_us: *entry_transition_us,
            };
            StepResult::At(next_sample_us)
        }
        VelocityIntegralState::Capture {
            next_sample_us,
            deadline_us,
            entry_transition_us,
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
            let _moving_peak = data.current_observer.moving_pid_output_peak();
            if data
                .observations
                .push(delta.signed_counts(), elapsed_us as u16)
                .is_err()
            {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let Some(_stroke_index) = integral_stroke_index(data.rung_index, data.slot) else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            #[cfg(feature = "trace")]
            backend.record_velocity_trace_observation(
                crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: integral_slot_direction(data.plan.slot_order, data.slot)
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
                    stroke_index: _stroke_index,
                    direction: integral_slot_direction(data.plan.slot_order, data.slot).unwrap_or(0)
                        as i8,
                    flags: u8::from(_moving_peak >= data.plan.usable_torque_limit),
                    peak_torque_target_abs: _moving_peak,
                    velocity_diagnostic_value: 0,
                },
            );
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = read_us;
            if read_us >= *deadline_us {
                *state = VelocityIntegralState::PrepareZeroSettle {
                    entry_transition_us: *entry_transition_us,
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
        VelocityIntegralState::PrepareZeroSettle {
            entry_transition_us,
        } => {
            let boundary =
                finish_velocity_current_moving_phase(&mut data.current_observer, backend).await;
            let p = match PidPGain::new(data.plan.fixed_p) {
                Ok(value) => value,
                Err(_) => {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                }
            };
            let transition_started_us = backend.elapsed_us();
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(PID_VELOCITY_TARGET, 0)
                    .await
                    .is_err()
                || clear_velocity_error_sum(backend).await.is_err()
            {
                return fail_integral(phase, backend, CommissionError::Spi).await;
            }
            if let Err(error) = boundary {
                return fail_integral(phase, backend, current_guard_error(error)).await;
            }
            let now_us = backend.elapsed_us();
            let exit_transition_us = match transition_elapsed_us(transition_started_us, now_us) {
                Ok(value) => value,
                Err(error) => return fail_integral(phase, backend, error).await,
            };
            if combined_slot_transition_us(*entry_transition_us, exit_transition_us).is_none() {
                let fault = safety.fault(
                    OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                    0,
                    u64::from(*entry_transition_us) + u64::from(exit_transition_us),
                    0,
                    0,
                );
                return fail_integral_with_safety(phase, backend, fault).await;
            }
            let settle_started_us = now_us;
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
            let (summary, stationarity, rejection) =
                match moving_analysis(data, safety.counts_per_rev()) {
                    Ok(Some(result)) => result,
                    Ok(None) => return StepResult::ReadyNow,
                    Err(error) => return fail_integral(phase, backend, error).await,
                };
            if let StationarityResult::Usable(suffix) = stationarity
                && set_rung_blocking_level(data, data.slot, suffix.selection.level).is_err()
            {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            if retain_analyzed_observation(data, workspace, summary).is_err() {
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
                rejection: rejection.map(Into::into),
            };
            let settle_deadline_us = *settle_deadline_us;
            data.analysis.reset();
            data.observations.clear();
            *state = if let Some(error) = terminal_error {
                VelocityIntegralState::FaultTerminal { error }
            } else {
                VelocityIntegralState::BeginSettleCapture { settle_deadline_us }
            };
            if is_fixed_gain_amplitude(data.plan) {
                let event = data
                    .integrity
                    .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::Observation(report));
                StepResult::FixedGainAmplitudeEvent(event)
            } else {
                let event = data
                    .integrity
                    .record(VelocityIntegralEvent::Observation(report));
                StepResult::VelocityIntegralEvent(event)
            }
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
            data.settle_metrics = VelocitySettleMetrics::EMPTY;
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
            let _zero_peak = data.current_observer.zero_pid_output_peak();
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
                .settle_metrics
                .observe(delta.signed_counts(), elapsed_us as u32)
                .is_err()
            {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            if data
                .observations
                .push(delta.signed_counts(), elapsed_us as u16)
                .is_err()
            {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let Some(_stroke_index) = integral_stroke_index(data.rung_index, data.slot) else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            #[cfg(feature = "trace")]
            backend.record_velocity_trace_observation(
                crate::trace::stream::VelocityTraceObservation {
                    target_counts_per_ms: 0,
                    delta_counts: delta.signed_counts(),
                    elapsed_us: elapsed_us as u32,
                    stroke_index: SETTLE_TRACE_STROKE_MARKER | _stroke_index,
                    direction: 0,
                    flags: 0,
                    peak_torque_target_abs: _zero_peak,
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
                data.analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        data.settle_min_displacement_counts,
                        data.settle_max_displacement_counts,
                    ));
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
            let settle_result = match poll_scored_rest_confirmation(
                &mut data.analysis,
                &data.observations,
                &mut data.blocker,
            ) {
                Ok(ScoredRestPoll::Pending) => return StepResult::ReadyNow,
                Ok(ScoredRestPoll::Diagnostic(decision)) => {
                    let owner = match integral_rung_kind(data.plan, data.rung_index) {
                        IntegralRungKind::OpeningAnchor | IntegralRungKind::ClosingBookend => {
                            RestSelectionOwner::IntegralAnchor
                        }
                        IntegralRungKind::Positive => {
                            RestSelectionOwner::IntegralPositiveObservation
                        }
                        IntegralRungKind::FixedGainAmplitude => {
                            RestSelectionOwner::FixedGainAmplitudeObservation
                        }
                    };
                    return record_integral_observation_rest_selection(data, decision, owner);
                }
                Ok(ScoredRestPoll::Complete(result)) => result,
                Err(error) => return fail_integral(phase, backend, error).await,
            };
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
            if !settle_result.confirmed_rest
                && !data.observation_current_headroom
                && matches!(
                    integral_rung_kind(data.plan, data.rung_index),
                    IntegralRungKind::Positive
                )
            {
                let active_stationarity = match settle_result.selected {
                    Some(suffix) => StationarityResult::Usable(suffix),
                    None => {
                        StationarityResult::Excluded(match settle_result.recent_structural_code {
                            RestStructuralCode::NoBlockingPlateau => {
                                StationarityExclusion::NoBlockingPlateau
                            }
                            RestStructuralCode::TooFewDecorrelatedBlocks => {
                                StationarityExclusion::TooFewDecorrelatedBlocks
                            }
                            RestStructuralCode::ResolvedSlope => {
                                StationarityExclusion::ResolvedSlope
                            }
                            RestStructuralCode::StructuredResidual => {
                                StationarityExclusion::StructuredResidual
                            }
                            RestStructuralCode::Usable
                            | RestStructuralCode::Arithmetic
                            | RestStructuralCode::InsufficientHistory => {
                                return fail_integral(
                                    phase,
                                    backend,
                                    CommissionError::VelocityValidationFailed,
                                )
                                .await;
                            }
                        })
                    }
                };
                data.positive_i_rest_boundary = Some(PositiveIRestBoundary {
                    rung_index: data.rung_index,
                    slot: data.slot,
                    i_raw: integral_i_at(data.plan, data.rung_index).unwrap_or(0),
                    direction: integral_slot_direction(data.plan.slot_order, data.slot).unwrap_or(0)
                        as i8,
                    active_stationarity,
                    active_settle: data.settle_metrics,
                    zero_phase_current: current_evidence,
                    cleanup_confirmed: false,
                });
                *state = VelocityIntegralState::BeginPositiveIRestCleanup;
            } else if !settle_result.confirmed_rest && !data.observation_current_headroom {
                safety.end_primitive();
                data.rest_unconfirmed = true;
                data.rest_unconfirmed_owner =
                    Some(match integral_rung_kind(data.plan, data.rung_index) {
                        IntegralRungKind::OpeningAnchor | IntegralRungKind::ClosingBookend => {
                            RestSelectionOwner::IntegralAnchor
                        }
                        // The sibling if-branch above already claims every Positive rung.
                        IntegralRungKind::Positive => unreachable!(),
                        IntegralRungKind::FixedGainAmplitude => {
                            RestSelectionOwner::FixedGainAmplitudeObservation
                        }
                    });
                *state = VelocityIntegralState::Terminal;
            } else {
                safety.end_primitive();
            }
            if data.observation_current_headroom {
                if is_fixed_gain_amplitude(data.plan) {
                    let bit = match 1_u8.checked_shl(u32::from(data.rung_index)) {
                        Some(value) => value,
                        None => {
                            return fail_integral(phase, backend, CommissionError::InvalidSchedule)
                                .await;
                        }
                    };
                    set_fixed_gain_amplitude_current(
                        data,
                        data.plan.slot_order.direction_index(data.slot),
                        bit,
                    );
                    match plan_integral_recovery(data, workspace, safety) {
                        Ok(recovery) => *state = VelocityIntegralState::RecoverRung(recovery),
                        Err(error) => return fail_integral(phase, backend, error).await,
                    }
                } else if matches!(
                    integral_rung_kind(data.plan, data.rung_index),
                    IntegralRungKind::Positive
                ) && positive_directional_floors_exist(data)
                {
                    // Breakaway current-headroom exit is the sibling
                    // `retain_rung` bypass; record the target terminus before
                    // the current terminus consumes the rung.
                    if let Err(error) = record_diverted_target_terminus(data) {
                        return fail_integral(phase, backend, error).await;
                    }
                    data.current_terminus = data.rung_index.checked_sub(1);
                    match plan_integral_recovery(data, workspace, safety) {
                        Ok(recovery) => *state = VelocityIntegralState::RecoverRung(recovery),
                        Err(error) => return fail_integral(phase, backend, error).await,
                    }
                } else {
                    apply_integral_current_headroom(state, data);
                }
            } else if data.slot + 1 < INTEGRAL_SLOT_COUNT {
                if settle_result.confirmed_rest {
                    *state = VelocityIntegralState::Advance;
                }
            } else if settle_result.confirmed_rest {
                *state = VelocityIntegralState::AnalyzeRung;
            }
            let report = current_report(data, current_evidence);
            if is_fixed_gain_amplitude(data.plan) {
                let event = data
                    .integrity
                    .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::Current(report));
                StepResult::FixedGainAmplitudeEvent(event)
            } else {
                let event = data
                    .integrity
                    .record(VelocityIntegralEvent::Current(report));
                StepResult::VelocityIntegralEvent(event)
            }
        }
        VelocityIntegralState::BeginPositiveIRestCleanup => {
            if data.positive_i_rest_boundary.is_none() {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            let p = match PidPGain::new(data.plan.fixed_p) {
                Ok(value) => value,
                Err(_) => {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                }
            };
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || clear_velocity_error_sum(backend).await.is_err()
                || backend
                    .write_register(PID_VELOCITY_TARGET, 0)
                    .await
                    .is_err()
            {
                return fail_integral(phase, backend, CommissionError::Spi).await;
            }
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fail_integral(phase, backend, CommissionError::Spi).await,
            };
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(data.plan.zero_settle_us))
            else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = started_us;
            data.settle_displacement_counts = 0;
            data.settle_min_displacement_counts = 0;
            data.settle_max_displacement_counts = 0;
            data.settle_metrics = VelocitySettleMetrics::EMPTY;
            data.analysis.reset();
            data.observations.clear();
            *state = VelocityIntegralState::PositiveIRestCleanupCapture {
                next_sample_us,
                deadline_us,
            };
            StepResult::At(next_sample_us)
        }
        VelocityIntegralState::PositiveIRestCleanupCapture {
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
                .settle_metrics
                .observe(delta.signed_counts(), elapsed_us as u32)
                .is_err()
            {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            if data
                .observations
                .push(delta.signed_counts(), elapsed_us as u16)
                .is_err()
            {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            data.previous_encoder_raw = encoder_raw;
            data.previous_read_us = read_us;
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(data.plan.analysis_budget_us))
                else {
                    return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
                };
                data.analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        data.settle_min_displacement_counts,
                        data.settle_max_displacement_counts,
                    ));
                *state = VelocityIntegralState::AnalyzePositiveIRestCleanup {
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
        VelocityIntegralState::AnalyzePositiveIRestCleanup {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return fail_integral(phase, backend, CommissionError::AnalysisOverrun).await;
            }
            let cleanup = match poll_scored_rest_confirmation(
                &mut data.analysis,
                &data.observations,
                &mut data.blocker,
            ) {
                Ok(ScoredRestPoll::Pending) => return StepResult::ReadyNow,
                Ok(ScoredRestPoll::Diagnostic(decision)) => {
                    return record_integral_observation_rest_selection(
                        data,
                        decision,
                        RestSelectionOwner::IntegralCleanup,
                    );
                }
                Ok(ScoredRestPoll::Complete(result)) => result,
                Err(error) => return fail_integral(phase, backend, error).await,
            };
            let Some(boundary) = data.positive_i_rest_boundary.as_mut() else {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            };
            boundary.cleanup_confirmed = cleanup.confirmed_rest;
            if let Err(error) =
                finish_velocity_current_zero_phase(&mut data.current_observer, backend).await
            {
                return fail_integral(phase, backend, current_guard_error(error)).await;
            }
            let cleanup_current = match data
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
            {
                Ok(evidence) => evidence,
                Err(error) => {
                    return fail_integral(phase, backend, current_guard_error(error)).await;
                }
            };
            if cleanup_current.contact != VelocityCurrentContact::None {
                return fail_integral(phase, backend, CommissionError::SafetyEnvelopeViolation)
                    .await;
            }
            data.analysis.reset();
            data.observations.clear();
            safety.end_primitive();
            if !cleanup.confirmed_rest {
                data.rest_unconfirmed = true;
                data.rest_unconfirmed_owner = Some(RestSelectionOwner::IntegralCleanup);
                *state = VelocityIntegralState::Terminal;
                return StepResult::ReadyNow;
            }
            data.positive_i_rest_boundary_after_sufficiency =
                positive_directional_floors_exist(data);
            if data.positive_i_rest_boundary_after_sufficiency {
                // This is the breakaway peak-rung closure that never reaches
                // `retain_rung`; record the target terminus here so a
                // target-reaching, non-settling rung still installs it.
                if let Err(error) = record_diverted_target_terminus(data) {
                    return fail_integral(phase, backend, error).await;
                }
            }
            if data.positive_i_rest_boundary_after_sufficiency {
                match plan_integral_recovery(data, workspace, safety) {
                    Ok(recovery) => *state = VelocityIntegralState::RecoverRung(recovery),
                    Err(error) => return fail_integral(phase, backend, error).await,
                }
            } else {
                *state = VelocityIntegralState::Terminal;
            }
            StepResult::ReadyNow
        }
        VelocityIntegralState::Advance => {
            let finished = advance_integral_cursor(
                &mut data.rung_index,
                &mut data.slot,
                integral_rung_count(data.plan).unwrap_or(0),
            );
            if finished || data.slot == 0 {
                return fail_integral(phase, backend, CommissionError::InvalidSchedule).await;
            }
            *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: false };
            StepResult::ReadyNow
        }
        VelocityIntegralState::AnalyzeRung => {
            if is_fixed_gain_amplitude(data.plan) {
                let report = match fixed_gain_amplitude_report(data, workspace) {
                    Ok(report) => report,
                    Err(error) => return fail_integral(phase, backend, error).await,
                };
                data.emitted_rungs = data.emitted_rungs.saturating_add(1);
                let recovery = match plan_integral_recovery(data, workspace, safety) {
                    Ok(runtime) => runtime,
                    Err(error) => return fail_integral(phase, backend, error).await,
                };
                *state = VelocityIntegralState::RecoverRung(recovery);
                let evidence = FixedGainAmplitudeEvidence {
                    run_sequence: data.integrity.run_sequence(),
                    evidence_sequence: data.integrity.next_evidence_sequence(),
                    plan_digest: data.plan.plan_digest,
                    report,
                };
                let event = data
                    .integrity
                    .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::Amplitude(evidence));
                return StepResult::FixedGainAmplitudeEvent(event);
            }
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
            let i_raw = integral_i_at(data.plan, data.rung_index).unwrap_or(0);
            let (tau_us, horizon_class) = (
                0,
                crate::commissioning::types::ResponseHorizonClass::NotApplicable,
            );
            let report = VelocityIntegralRungReport {
                run_sequence: data.integrity.run_sequence(),
                evidence_sequence: data.integrity.next_evidence_sequence(),
                plan_digest: data.plan.plan_digest,
                rung_index: data.rung_index,
                i_raw,
                tau_us,
                moving_horizon_us: 0,
                horizon_class,
                directions,
                target_terminus_plus_one: data.target_terminus.map_or(0, |rung| rung + 1),
            };
            let rung_kind = integral_rung_kind(data.plan, data.rung_index);
            match rung_kind {
                IntegralRungKind::OpeningAnchor if data.opening_direction_mask != 3 => {
                    *state = VelocityIntegralState::Terminal;
                }
                IntegralRungKind::OpeningAnchor
                | IntegralRungKind::Positive
                | IntegralRungKind::ClosingBookend
                | IntegralRungKind::FixedGainAmplitude => {
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
                // Breakaway-only diagnostic; the fault it precedes arrives
                // on the following tick and is handled below unchanged.
                RungOriginRecoveryStep::OriginMismatch(_) => StepResult::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => StepResult::At(deadline_us),
                RungOriginRecoveryStep::RestSelection {
                    decision,
                    stage,
                    rung_index,
                } => record_integral_recovery_rest_selection(data, decision, stage, rung_index),
                RungOriginRecoveryStep::Finished(report)
                | RungOriginRecoveryStep::Contained(report) => {
                    *state = VelocityIntegralState::FinalizeRecovery(report);
                    StepResult::ReadyNow
                }
                RungOriginRecoveryStep::RestAnalysis(analysis) => {
                    record_integral_recovery_rest(data, analysis)
                }
                RungOriginRecoveryStep::RestUnconfirmed => {
                    data.rest_unconfirmed = true;
                    data.rest_unconfirmed_owner = Some(if is_fixed_gain_amplitude(data.plan) {
                        RestSelectionOwner::FixedGainAmplitudeRecovery
                    } else {
                        RestSelectionOwner::IntegralRecovery
                    });
                    *state = VelocityIntegralState::Terminal;
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
            if report.outcome == RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom {
                data.recovered_with_current_headroom = true;
                if matches!(
                    data.plan.acquisition_kind,
                    VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
                        | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending
                ) {
                    apply_integral_current_headroom(state, data);
                } else {
                    data.rung_index = data.plan.positive_rung_count.saturating_add(1);
                    *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
                }
            } else if report.outcome == RungOriginRecoveryOutcome::RecoveryCurrentHeadroom {
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
                    IntegralRungKind::Positive
                        if data.positive_i_rest_boundary_after_sufficiency && contained =>
                    {
                        data.recovery_unavailable = true;
                        *state = VelocityIntegralState::Terminal;
                    }
                    IntegralRungKind::OpeningAnchor | IntegralRungKind::Positive if contained => {
                        data.recovery_failure = true;
                        *state = VelocityIntegralState::Terminal;
                    }
                    IntegralRungKind::Positive
                        if data.positive_i_rest_boundary_after_sufficiency =>
                    {
                        data.rung_index = data.plan.positive_rung_count.saturating_add(1);
                        *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
                    }
                    IntegralRungKind::Positive if data.current_terminus.is_some() => {
                        data.rung_index = data.plan.positive_rung_count.saturating_add(1);
                        *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
                    }
                    IntegralRungKind::OpeningAnchor | IntegralRungKind::Positive => {
                        data.rung_index = data.rung_index.saturating_add(1);
                        *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
                    }
                    IntegralRungKind::FixedGainAmplitude if contained => {
                        data.recovery_failure = true;
                        *state = VelocityIntegralState::Terminal;
                    }
                    IntegralRungKind::FixedGainAmplitude
                        if fixed_gain_amplitude_current_masks(data) != [0; 2] =>
                    {
                        apply_integral_current_headroom(state, data);
                    }
                    IntegralRungKind::FixedGainAmplitude => {
                        data.rung_index = data.rung_index.saturating_add(1);
                        if data.rung_index >= integral_rung_count(data.plan).unwrap_or(0) {
                            *state = VelocityIntegralState::Terminal;
                        } else {
                            *state = VelocityIntegralState::PrepareMovingSlot { reset_rung: true };
                        }
                    }
                }
            }
            let recovery = RungOriginRecoveryEvidence {
                run_sequence: data.integrity.run_sequence(),
                evidence_sequence: data.integrity.next_evidence_sequence(),
                report,
            };
            if is_fixed_gain_amplitude(data.plan) {
                let event = data
                    .integrity
                    .record_fixed_gain_amplitude(FixedGainAmplitudeEvent::Recovery(recovery));
                StepResult::FixedGainAmplitudeEvent(event)
            } else {
                let event = data
                    .integrity
                    .record(VelocityIntegralEvent::Recovery(recovery));
                StepResult::VelocityIntegralEvent(event)
            }
        }
        VelocityIntegralState::FaultTerminal { error } => {
            let error = *error;
            fail_integral(phase, backend, error).await
        }
        VelocityIntegralState::Terminal if is_fixed_gain_amplitude(data.plan) => {
            StepResult::FixedGainAmplitudeTerminal(fixed_gain_amplitude_terminal(data, false, None))
        }
        VelocityIntegralState::Terminal => evidence_terminal(data, workspace),
    }
}

#[cfg(test)]
mod tests {
    use crate::autotune::{CommissionError, TRANSIENT_LEN, TransientBuffer};

    use super::{INTEGRAL_SLOT_COUNT, load_interval, set_rung_blocking_level, store_interval};
    use crate::commissioning::outer::VelocityIntegralRuntime;
    use crate::commissioning::outer::velocity::integral_evidence::ExactRateInterval;
    use crate::commissioning::outer::velocity::integral_plan::{
        AcceptanceSource, IntegralSlotOrder, VelocityIntegralAcquisitionKind,
        VelocityIntegralExecutionPlan,
    };
    use crate::commissioning::outer::velocity::velocity_clamp::VelocityLimitEvidenceCapability;

    fn plan() -> VelocityIntegralExecutionPlan {
        VelocityIntegralExecutionPlan {
            slot_order: IntegralSlotOrder::ForwardFirst,
            plan_digest: 7,
            acceptance_plan_digest: 0,
            qualifier_bits: 0,
            fixed_p: 1_448,
            planned_velocity_mrev_s: 2_929,
            target_velocity_rpm: 176,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            i_start: 1,
            positive_rung_count: 5,
            i_top_cap: 0,
            family_size: 28,
            moving_stroke_us: 512_000,
            zero_settle_us: 500_000,
            analysis_budget_us: 236_000,
            maximum_workflow_ms: 69_888,
            recovery_origin_band_counts: 1_000,
            recovery_nominal_slot_us: 1_816_958,
            recovery_maximum_slot_us: 3_000_000,
            recovery_slot_count: 7,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
            limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
            acquisition_kind: VelocityIntegralAcquisitionKind::ResponseLadder,
            acceptance_source: AcceptanceSource::NonBreakaway,
            amplitude_targets_rpm: [0; 5],
            selected_response_intervals_q: [[0; 2]; 2],
            recovery_lower_rate_q: [13_217_792, 12_827_648],
            recovery_quantization_exposed: true,
            probe_constrained_test_point: false,
        }
    }

    /// A slot or level outside its declared range is a caller bug, not a
    /// missing or exhausted workspace write -- must be reported as an
    /// out-of-range rung index.
    #[test]
    fn set_rung_blocking_level_rejects_out_of_range_slot_or_level() {
        let mut data = VelocityIntegralRuntime::new(1, plan());

        let slot_error = set_rung_blocking_level(&mut data, INTEGRAL_SLOT_COUNT, 0).unwrap_err();
        assert_eq!(slot_error, CommissionError::RungIndexOutOfRange);

        let level_error = set_rung_blocking_level(&mut data, 0, 0x10).unwrap_err();
        assert_eq!(level_error, CommissionError::RungIndexOutOfRange);
    }

    /// The second workspace word of an interval write lands one past the
    /// fixed `TransientBuffer` capacity: the write cannot be accepted, which
    /// is a distinct fault from a caller-supplied out-of-range index.
    #[test]
    fn store_interval_reports_evidence_buffer_full_at_the_workspace_boundary() {
        let mut workspace = TransientBuffer::new();
        let interval = ExactRateInterval::new(1, 2).unwrap();

        let error = store_interval(&mut workspace, TRANSIENT_LEN - 1, interval).unwrap_err();

        assert_eq!(error, CommissionError::EvidenceBufferFull);
    }

    /// Reading either half of an interval past the fixed `TransientBuffer`
    /// capacity finds no retained state there.
    #[test]
    fn load_interval_reports_missing_capture_state_past_the_workspace_boundary() {
        let workspace = TransientBuffer::new();

        let first_word_error = load_interval(&workspace, TRANSIENT_LEN).unwrap_err();
        assert_eq!(first_word_error, CommissionError::MissingCaptureState);

        let second_word_error = load_interval(&workspace, TRANSIENT_LEN - 1).unwrap_err();
        assert_eq!(second_word_error, CommissionError::MissingCaptureState);
    }
}
