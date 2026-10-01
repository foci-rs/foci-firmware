//! Canonical sequencing, integrity, and trace records for Integral.

use crate::ankyra::velocity_sweep_encoding::{
    VelocitySweepCanonicalRecord, encode_rate_values, extend_velocity_sweep_digest,
    upper_bound_i64, upper_bound_u128,
};
use crate::autotune::velocity_measurement::{StationarityExclusion, StationarityResult};
use crate::commissioning::types::{
    RestSelectionOwner, StationarityRejectionDiagnostic, VelocityIntegralEvent,
    VelocityIntegralObservation, VelocityIntegralOutcome, VelocityIntegralRungReport,
    VelocityIntegralTerminal,
};

use super::super::state::PositiveIRestBoundary;
use super::combined_plan::CombinedTargetStatus;
use super::integral_evidence::{IntegralObservationClass, IntegralRungClass, IntegralRungKind};
use super::integral_plan::{VelocityIntegralExecutionPlan, VelocityIntegralPlan};
use super::reproduction::BUILD_IDENTITY;

const FNV1A64_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
pub(crate) const PLAN_RECOVERY_QUANTIZATION_EXPOSED: u8 = 1 << 0;
pub(crate) const PLAN_PROBE_CONSTRAINED_TEST_POINT: u8 = 1 << 1;
pub(crate) const PLAN_SLOT_ORDER_SHIFT: u8 = 2;
pub(crate) const PLAN_SLOT_ORDER_MASK: u8 = 0b11 << PLAN_SLOT_ORDER_SHIFT;
pub(crate) const TERMINAL_RECOVERY_UNAVAILABLE: u8 = 1 << 0;
pub(crate) const TERMINAL_RECOVERED_WITH_CURRENT_HEADROOM: u8 = 1 << 1;
pub(crate) const TERMINAL_RECOVERY_QUANTIZATION_EXPOSED: u8 = 1 << 2;
pub(crate) const TERMINAL_PROBE_CONSTRAINED_TEST_POINT: u8 = 1 << 3;
pub(crate) const TERMINAL_REST_REJECTION_AFTER_SUFFICIENCY: u8 = 1 << 4;
pub(crate) const TERMINAL_REST_REJECTION_OWNER_SHIFT: u32 = 5;
pub(crate) const TERMINAL_REST_REJECTION_OWNER_MASK: u8 =
    0b011 << TERMINAL_REST_REJECTION_OWNER_SHIFT;

const fn terminal_carries_target(report: &VelocityIntegralTerminal) -> bool {
    report.breakaway_acceptance
}

pub(crate) const fn terminal_target_terminus(report: &VelocityIntegralTerminal) -> Option<u8> {
    if terminal_carries_target(report)
        && report.rest_boundary_slot_plus_one == 0
        && report.rest_boundary_rung_plus_one != 0
    {
        Some(report.rest_boundary_rung_plus_one - 1)
    } else {
        None
    }
}

pub(crate) fn terminal_target_status(
    report: &VelocityIntegralTerminal,
) -> Option<CombinedTargetStatus> {
    if !terminal_carries_target(report) {
        None
    } else if terminal_target_terminus(report).is_some() {
        Some(
            if report
                .evidence
                .eligible_masks
                .iter()
                .all(|mask| mask.count_ones() >= 3)
            {
                CombinedTargetStatus::TargetReached
            } else {
                CombinedTargetStatus::TargetReachedSparse
            },
        )
    } else if report.evidence.current_terminus.is_some() {
        Some(CombinedTargetStatus::CurrentHeadroom)
    } else if report.rest_boundary_slot_plus_one != 0
        || report.emitted_rungs < report.expected_rungs
    {
        None
    } else {
        Some(CombinedTargetStatus::TargetNotReachedAtCap)
    }
}

pub(crate) const fn plan_recovery_flags(plan: &VelocityIntegralPlan) -> u8 {
    let mut flags = 0;
    if plan.recovery_quantization_exposed {
        flags |= PLAN_RECOVERY_QUANTIZATION_EXPOSED;
    }
    if plan.probe_constrained_test_point {
        flags |= PLAN_PROBE_CONSTRAINED_TEST_POINT;
    }
    flags |= (plan.slot_order.report_code() << PLAN_SLOT_ORDER_SHIFT) & PLAN_SLOT_ORDER_MASK;
    flags
}

pub(crate) fn terminal_recovery_flags(report: &VelocityIntegralTerminal) -> u8 {
    let owner_code: u8 = match report.rest_rejection_owner {
        Some(RestSelectionOwner::IntegralAnchor) | None => 0b00,
        Some(RestSelectionOwner::IntegralPositiveObservation) => 0b01,
        Some(RestSelectionOwner::IntegralRecovery) => 0b10,
        Some(RestSelectionOwner::IntegralCleanup) => 0b11,
        Some(_) => 0b00,
    };
    (u8::from(report.recovery_unavailable) * TERMINAL_RECOVERY_UNAVAILABLE)
        | (u8::from(report.recovered_with_current_headroom)
            * TERMINAL_RECOVERED_WITH_CURRENT_HEADROOM)
        | (u8::from(report.recovery_quantization_exposed) * TERMINAL_RECOVERY_QUANTIZATION_EXPOSED)
        | (u8::from(report.probe_constrained_test_point) * TERMINAL_PROBE_CONSTRAINED_TEST_POINT)
        | (u8::from(report.rest_rejection_after_sufficiency)
            * TERMINAL_REST_REJECTION_AFTER_SUFFICIENCY)
        | if report.rest_rejection_after_sufficiency {
            (owner_code << TERMINAL_REST_REJECTION_OWNER_SHIFT) & TERMINAL_REST_REJECTION_OWNER_MASK
        } else {
            0
        }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocityIntegralIntegrity {
    run_sequence: u32,
    next_evidence_sequence: u16,
    emitted_observations: u16,
    emitted_rungs: u16,
    digest: u64,
}

impl VelocityIntegralIntegrity {
    pub(crate) fn new(run_sequence: u32, plan: VelocityIntegralExecutionPlan) -> Self {
        let digest = extend_velocity_sweep_digest(
            FNV1A64_OFFSET,
            &encode_velocity_integral_plan_marker(run_sequence, plan),
        );
        Self {
            run_sequence,
            next_evidence_sequence: 1,
            emitted_observations: 0,
            emitted_rungs: 0,
            digest,
        }
    }

    pub(crate) const fn run_sequence(self) -> u32 {
        self.run_sequence
    }

    pub(crate) const fn next_evidence_sequence(self) -> u16 {
        self.next_evidence_sequence
    }

    pub(crate) fn record(&mut self, event: VelocityIntegralEvent) -> VelocityIntegralEvent {
        match event {
            VelocityIntegralEvent::Observation(_) => {
                self.emitted_observations = self.emitted_observations.saturating_add(1);
            }
            VelocityIntegralEvent::Rung(_) => {
                self.emitted_rungs = self.emitted_rungs.saturating_add(1);
            }
            VelocityIntegralEvent::Recovery(_)
            | VelocityIntegralEvent::RecoveryRest(_)
            | VelocityIntegralEvent::RestSelection(_)
            | VelocityIntegralEvent::Current(_) => {}
        }
        self.digest =
            extend_velocity_sweep_digest(self.digest, &encode_velocity_integral_event(event));
        self.next_evidence_sequence = self.next_evidence_sequence.saturating_add(1);
        event
    }

    pub(crate) fn record_terminal_boundary(&mut self, record: &VelocitySweepCanonicalRecord) {
        self.digest = extend_velocity_sweep_digest(self.digest, record);
    }

    pub(crate) const fn emitted_observations(self) -> u16 {
        self.emitted_observations
    }

    pub(crate) const fn emitted_rungs(self) -> u16 {
        self.emitted_rungs
    }

    pub(crate) const fn digest(self) -> u64 {
        self.digest
    }
}

pub(crate) fn encode_velocity_integral_event(
    event: VelocityIntegralEvent,
) -> VelocitySweepCanonicalRecord {
    match event {
        VelocityIntegralEvent::Observation(report) => encode_observation(report),
        VelocityIntegralEvent::Rung(report) => encode_rung(report),
        VelocityIntegralEvent::Recovery(report) => {
            super::sweep_reporting::encode_rung_origin_recovery(report)
        }
        VelocityIntegralEvent::RecoveryRest(report) => {
            super::sweep_reporting::encode_recovery_rest(report)
        }
        VelocityIntegralEvent::RestSelection(report) => {
            super::sweep_reporting::encode_rest_selection(report)
                .expect("completed scored-rest diagnostic is coherent")
        }
        VelocityIntegralEvent::Current(report) => {
            crate::ankyra::velocity_sweep_encoding::encode_velocity_primitive_current_evidence(
                report,
            )
        }
    }
}

pub(crate) fn encode_velocity_integral_plan_trace(
    run_sequence: u32,
    plan: VelocityIntegralPlan,
) -> [VelocitySweepCanonicalRecord; 3] {
    let mut core = VelocitySweepCanonicalRecord::new();
    core.push_u8(0);
    core.push_u32(run_sequence);
    core.push_u16(0);
    push_u64(&mut core, plan.digest());
    core.push_u8(0);
    core.push_u16(plan.build_features);
    core.push_u32(BUILD_IDENTITY);
    core.push_u16(plan.schema_revision);
    core.push_u8(plan.authority.channel);
    push_u64(&mut core, plan.authority.acquisition.get());
    core.push_u16(plan.fixed_p);
    for membership in plan.authority.directional_memberships {
        core.push_u32(membership);
    }
    core.push_u32(plan.authority.proportional_joint_membership);
    core.push_u32(plan.authority.joint_membership);
    for validity in plan.authority.directional_validity {
        core.push_u8(validity as u8);
    }
    core.push_u8(u8::from(plan.authority.reduced_margin));
    for interval in plan.authority.directional_pooled_intervals {
        core.push_i32(interval[0]);
        core.push_i32(interval[1]);
    }
    core.push_u32(plan.planned_velocity_mrev_s);
    core.push_i32(plan.target_velocity_rpm);
    core.push_u32(plan.pwm_hz);
    core.push_u32(plan.encoder_counts_per_rev);
    core.push_u8(plan.positive_rung_count);
    core.push_u16(plan.family_size);
    core.push_u8(plan.total_rung_count);
    core.push_u16(plan.expected_observations);
    core.push_u16(plan.hard_torque_limit);
    core.push_u16(plan.usable_torque_limit);
    core.push_u8(plan.limit_evidence_capability as u8);
    if matches!(
        plan.schema_revision,
        super::integral_plan::INTEGRAL_EVIDENCE_SCHEMA_REVISION
            | super::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION
    ) {
        core.push_u16(plan.i_top_cap);
    }

    let mut schedule = VelocitySweepCanonicalRecord::new();
    schedule.push_u8(0);
    schedule.push_u32(run_sequence);
    schedule.push_u16(0);
    push_u64(&mut schedule, plan.digest());
    schedule.push_u8(1);
    schedule.push_u8(plan.positive_rung_count);
    for gain in plan.positive_rungs() {
        schedule.push_u16(*gain);
    }
    schedule.push_u32(plan.moving_stroke_us);
    schedule.push_u32(plan.zero_settle_us);
    schedule.push_u32(plan.analysis_budget_us);
    schedule.push_u32(plan.nominal_workflow_ms);
    schedule.push_u32(plan.maximum_workflow_ms);
    schedule.push_u32(plan.motion_request.requested_velocity_mrev_s);
    schedule.push_u32(plan.motion_request.machine_velocity_ceiling_mrev_s);
    schedule.push_u8(plan.motion_request.requested_velocity_source as u8);
    schedule.push_u32(plan.motion_request.max_stroke_travel_mrev);
    schedule.push_u32(plan.motion_request.settle_travel_reserve_mrev);
    schedule.push_u32(plan.motion_request.negative_position_headroom_mrev);
    schedule.push_u32(plan.motion_request.positive_position_headroom_mrev);
    schedule.push_u16(plan.motion_request.max_duration_ms);

    let mut recovery = VelocitySweepCanonicalRecord::new();
    recovery.push_u8(0);
    recovery.push_u32(run_sequence);
    recovery.push_u16(0);
    push_u64(&mut recovery, plan.digest());
    recovery.push_u8(2);
    recovery.push_u32(plan.recovery_origin_band_counts);
    recovery.push_u32(plan.recovery_nominal_slot_us);
    recovery.push_u32(plan.recovery_maximum_slot_us);
    recovery.push_u8(plan.recovery_slot_count);
    recovery.push_u8(plan_recovery_flags(&plan));
    [core, schedule, recovery]
}

pub(crate) const VELOCITY_INTEGRAL_TERMINAL_REPLY_SCHEMA_REVISION: u8 = 2;
pub(crate) const VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES: usize = 39;

#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct VelocityIntegralTerminalReproductionSummary {
    pub reproduced_masks: [u32; 2],
    pub divergent_masks: [u32; 2],
}

fn push_reply<const N: usize>(destination: &mut [u8; N], offset: &mut usize, value: &[u8]) {
    destination[*offset..*offset + value.len()].copy_from_slice(value);
    *offset += value.len();
}

pub(crate) fn encode_velocity_integral_terminal_reply(
    report: &VelocityIntegralTerminal,
    reproduction: Option<VelocityIntegralTerminalReproductionSummary>,
) -> [u8; VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES] {
    let mut bytes = [0_u8; VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES];
    let mut offset = 0;
    push_reply(
        &mut bytes,
        &mut offset,
        &[VELOCITY_INTEGRAL_TERMINAL_REPLY_SCHEMA_REVISION],
    );
    push_reply(&mut bytes, &mut offset, &report.run_sequence.to_le_bytes());
    push_reply(&mut bytes, &mut offset, &[outcome_code(report.outcome)]);
    push_reply(
        &mut bytes,
        &mut offset,
        &[report.cause.raw_value(), report.cause.namespace_code()],
    );
    let rest_rejection_flags = terminal_recovery_flags(report)
        & (TERMINAL_REST_REJECTION_AFTER_SUFFICIENCY | TERMINAL_REST_REJECTION_OWNER_MASK);
    push_reply(&mut bytes, &mut offset, &[rest_rejection_flags]);
    push_reply(
        &mut bytes,
        &mut offset,
        &report.evidence.eligible_masks[0].to_le_bytes(),
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &report.evidence.eligible_masks[1].to_le_bytes(),
    );
    let bookend_available_mask = u8::from(report.evidence.bookend[0].is_some())
        | (u8::from(report.evidence.bookend[1].is_some()) << 1);
    push_reply(&mut bytes, &mut offset, &[bookend_available_mask]);
    push_reply(
        &mut bytes,
        &mut offset,
        &[report.evidence.current_terminus.map_or(0, |rung| rung + 1)],
    );
    push_reply(&mut bytes, &mut offset, &[u8::from(reproduction.is_some())]);
    let reproduction = reproduction.unwrap_or_default();
    for direction in 0..2 {
        push_reply(
            &mut bytes,
            &mut offset,
            &reproduction.reproduced_masks[direction].to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &reproduction.divergent_masks[direction].to_le_bytes(),
        );
    }
    push_reply(
        &mut bytes,
        &mut offset,
        &[u8::from(report.candidate_i.is_some())],
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &report.candidate_i.unwrap_or(0).to_le_bytes(),
    );
    debug_assert_eq!(offset, VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES);
    bytes
}

pub(crate) fn encode_velocity_integral_terminal(
    report: &VelocityIntegralTerminal,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(4);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    push_u64(&mut record, report.digest);
    record.push_u8(outcome_code(report.outcome));
    record.push_u8(report.cause.raw_value());
    record.push_u8(terminal_recovery_flags(report));
    record.push_u16(report.expected_observations);
    record.push_u16(report.emitted_observations);
    record.push_u16(report.expected_rungs);
    record.push_u16(report.emitted_rungs);
    record.push_u32(report.evidence.eligible_masks[0]);
    record.push_u32(report.evidence.eligible_masks[1]);
    record.push_u8(
        u8::from(report.evidence.opening[0] != super::integral_evidence::ExactRateInterval::EMPTY)
            | (u8::from(
                report.evidence.opening[1] != super::integral_evidence::ExactRateInterval::EMPTY,
            ) << 1),
    );
    record.push_u8(
        u8::from(report.evidence.bookend[0].is_some())
            | (u8::from(report.evidence.bookend[1].is_some()) << 1),
    );
    record.push_u8(report.evidence.current_terminus.map_or(0, |rung| rung + 1));
    push_u64(&mut record, report.run_started_us);
    push_u64(&mut record, report.run_completed_us);
    record.push_u8(report.cause.namespace_code());
    record
}

pub(crate) fn encode_velocity_integral_rest_boundary(
    run_sequence: u32,
    evidence_sequence: u16,
    plan_digest: u64,
    boundary: PositiveIRestBoundary,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(5);
    record.push_u32(run_sequence);
    record.push_u16(evidence_sequence);
    push_u64(&mut record, plan_digest);
    record.push_u8(
        boundary
            .rung_index
            .checked_sub(1)
            .expect("positive-I rest boundary has a positive rung index"),
    );
    record.push_u8(boundary.slot);
    record.push_u16(boundary.i_raw);
    record.push_i8(boundary.direction);
    record.push_u16(boundary.active_settle.sample_count);
    record.push_u32(boundary.active_settle.duration_us);
    record.push_i32(boundary.active_settle.net_displacement_counts);
    record.push_i32(boundary.active_settle.minimum_displacement_counts);
    record.push_i32(boundary.active_settle.maximum_displacement_counts);
    record.push_u16(boundary.active_settle.transition_count);
    encode_transition(&mut record, boundary.active_settle.first_transition);
    encode_transition(&mut record, boundary.active_settle.last_transition);
    encode_stationarity(&mut record, boundary.active_stationarity);
    record.push_u16(boundary.zero_phase_current.clamp_limit);
    record.push_u16(boundary.zero_phase_current.moving_pid_output_peak);
    record.push_u16(boundary.zero_phase_current.zero_pid_output_peak);
    record.push_u32(boundary.zero_phase_current.moving_status_flags);
    record.push_u32(boundary.zero_phase_current.zero_status_flags);
    record.push_u8(boundary.zero_phase_current.capability as u8);
    record.push_u8(boundary.zero_phase_current.contact as u8);
    record.push_u8(u8::from(boundary.cleanup_confirmed));
    record
}

fn encode_velocity_integral_plan_marker(
    run_sequence: u32,
    plan: VelocityIntegralExecutionPlan,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(0);
    record.push_u32(run_sequence);
    push_u64(&mut record, plan.plan_digest);
    record.push_u16(plan.fixed_p);
    record.push_u32(plan.planned_velocity_mrev_s);
    record.push_i32(plan.target_velocity_rpm);
    record.push_u32(plan.pwm_hz);
    record.push_u16(plan.i_start);
    record.push_u8(plan.positive_rung_count);
    record.push_u16(plan.family_size);
    record.push_u32(plan.moving_stroke_us);
    record.push_u32(plan.zero_settle_us);
    record.push_u32(plan.analysis_budget_us);
    record.push_u32(plan.maximum_workflow_ms);
    record.push_u32(plan.recovery_origin_band_counts);
    record.push_u32(plan.recovery_nominal_slot_us);
    record.push_u32(plan.recovery_maximum_slot_us);
    record.push_u8(plan.recovery_slot_count);
    record.push_u16(plan.hard_torque_limit);
    record.push_u16(plan.usable_torque_limit);
    record
}

fn encode_observation(report: VelocityIntegralObservation) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(2);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.rung_index);
    record.push_u8(report.slot);
    record.push_u16(report.i_raw);
    record.push_u8(direction_code(report.summary.direction));
    record.push_u8(observation_class_code(integral_observation_class(report)));
    record.push_u8(u8::from(report.summary.initial_lead_in));
    let (rates, rate_shift) = encode_rate_values([
        report.summary.settled_rate_interval_q[0],
        report.summary.settled_rate_mean_q,
        report.summary.settled_rate_interval_q[1],
        report.summary.deficit_interval_q[0],
        report.summary.deficit_interval_q[1],
    ]);
    for value in rates {
        record.push_i32(value);
    }
    record.push_u8(rate_shift);
    record.push_u16(report.summary.predicted_torque_target_abs);
    record.push_u32(report.summary.drift_ratio_ppm);
    encode_stationarity(&mut record, report.stationarity);
    if matches!(report.stationarity, StationarityResult::Excluded(_)) {
        encode_rejection(&mut record, report.rejection);
    }
    record
}

#[cfg(test)]
pub(crate) const REJECTION_DIAGNOSTIC_BYTES: usize = 49;

fn encode_rejection(
    record: &mut VelocitySweepCanonicalRecord,
    rejection: Option<StationarityRejectionDiagnostic>,
) {
    let Some(rejection) = rejection else {
        record.push_u8(0);
        return;
    };
    record.push_u8(1);
    record.push_u16(rejection.suffix_len);
    record.push_u8(u8::from(rejection.usable_plateau));
    record.push_u8(u8::from(rejection.slope_passed));
    record.push_u8(u8::from(rejection.lag_one_passed));
    push_i64(record, rejection.settled_rate_interval_q[0]);
    push_i64(record, rejection.settled_rate_interval_q[1]);
    push_i64(record, rejection.slope_rejection_margin_q);
    record.push_i32(rejection.lag_one_rejection_margin_q);
    push_i64(record, rejection.slope_q);
    push_u64(record, rejection.slope_half_width_q);
}

fn encode_stationarity(record: &mut VelocitySweepCanonicalRecord, result: StationarityResult) {
    match result {
        StationarityResult::Usable(suffix) => {
            record.push_u8(0);
            record.push_u16(suffix.suffix_len);
            record.push_u8(suffix.selection.level);
            record.push_u32(suffix.selection.block_count);
            let (slope, slope_shift) = upper_bound_i64(suffix.slope_q);
            record.push_i32(slope);
            record.push_u8(slope_shift);
            let (half_width, half_width_shift) =
                upper_bound_u128(u128::from(suffix.slope_half_width_q));
            record.push_u32(half_width);
            record.push_u8(half_width_shift);
            record.push_i32(suffix.lag_one_q);
            record.push_u32(suffix.lag_one_half_width_q);
            let (residual, residual_shift) = upper_bound_u128(suffix.residual_magnitude_q2);
            record.push_u32(residual);
            record.push_u8(residual_shift);
            record.push_u8(suffix.tested_suffixes);
        }
        StationarityResult::Excluded(exclusion) => {
            record.push_u8(stationarity_exclusion_code(exclusion));
        }
    }
}

fn encode_transition(
    record: &mut VelocitySweepCanonicalRecord,
    transition: super::super::state::VelocitySettleTransition,
) {
    record.push_u16(transition.sample_index);
    record.push_u32(transition.elapsed_us);
    record.push_i32(transition.delta_counts);
    record.push_i32(transition.displacement_counts);
}

fn encode_rung(report: VelocityIntegralRungReport) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(3);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.rung_index);
    record.push_u16(report.i_raw);
    for direction in report.directions {
        record.push_u8(direction_code(direction.direction));
        record.push_u8(rung_kind_code(direction.kind));
        record.push_u8(rung_class_code(direction.classification));
        record.push_u8(direction.component_count);
        record.push_u8(direction.usable_mask);
        record.push_u8(direction.included_mask);
        for class in direction.observation_classes {
            record.push_u8(observation_class_code(class));
        }
        for component in direction.components {
            record.push_i32(component.low_q);
            record.push_i32(component.high_q);
        }
    }
    record.push_u8(report.target_terminus_plus_one);
    record
}

#[cfg(feature = "trace")]
pub(crate) struct IntegralPromotionDiagnostic {
    pub channel: u8,
    pub outcome: u8,
    pub acceptance_source_code: u8,
    pub phase_live: bool,
    pub breakaway_acceptance: bool,
    pub execution_plan_present: bool,
    pub authority_present: bool,
    pub rest_boundary_rung_plus_one: u8,
    pub rest_boundary_slot_plus_one: u8,
    pub installer_result: u8,
}

#[cfg(feature = "trace")]
pub(crate) fn encode_integral_promotion_diagnostic(
    diagnostic: IntegralPromotionDiagnostic,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(40);
    record.push_u8(diagnostic.channel);
    record.push_u8(diagnostic.outcome);
    record.push_u8(diagnostic.acceptance_source_code);
    record.push_u8(u8::from(diagnostic.phase_live));
    record.push_u8(u8::from(diagnostic.breakaway_acceptance));
    record.push_u8(u8::from(diagnostic.execution_plan_present));
    record.push_u8(u8::from(diagnostic.authority_present));
    record.push_u8(diagnostic.rest_boundary_rung_plus_one);
    record.push_u8(diagnostic.rest_boundary_slot_plus_one);
    record.push_u8(diagnostic.installer_result);
    record
}

pub(crate) fn integral_observation_class(
    report: VelocityIntegralObservation,
) -> IntegralObservationClass {
    use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;

    match report.summary.classification {
        VelocityObservationClass::Eligible => {
            if report.rung_index == 0 {
                let resolved = match report.summary.direction {
                    crate::autotune::velocity_measurement::VelocityDirection::Forward => {
                        report.summary.deficit_interval_q[0] > 0
                    }
                    crate::autotune::velocity_measurement::VelocityDirection::Reverse => {
                        report.summary.deficit_interval_q[1] < 0
                    }
                };
                if resolved {
                    IntegralObservationClass::Usable
                } else {
                    IntegralObservationClass::SignUnresolved
                }
            } else {
                IntegralObservationClass::Usable
            }
        }
        VelocityObservationClass::Inert => IntegralObservationClass::Inert,
        VelocityObservationClass::CurrentHeadroom => IntegralObservationClass::CurrentHeadroom,
        VelocityObservationClass::DeficitUnresolved => IntegralObservationClass::SignUnresolved,
        VelocityObservationClass::StationarityExcluded(_) => {
            IntegralObservationClass::StationarityExcluded
        }
        VelocityObservationClass::WrongDirectionFault
        | VelocityObservationClass::HardClipFault
        | VelocityObservationClass::InvalidEvidenceFault => IntegralObservationClass::Fault,
    }
}

pub(crate) const fn direction_code(
    direction: crate::autotune::velocity_measurement::VelocityDirection,
) -> u8 {
    match direction {
        crate::autotune::velocity_measurement::VelocityDirection::Forward => 0,
        crate::autotune::velocity_measurement::VelocityDirection::Reverse => 1,
    }
}

pub(crate) const fn observation_class_code(class: IntegralObservationClass) -> u8 {
    match class {
        IntegralObservationClass::Usable => 0,
        IntegralObservationClass::SignUnresolved => 1,
        IntegralObservationClass::Inert => 2,
        IntegralObservationClass::CurrentHeadroom => 3,
        IntegralObservationClass::StationarityExcluded => 4,
        IntegralObservationClass::Fault => 5,
    }
}

pub(crate) const fn rung_kind_code(kind: IntegralRungKind) -> u8 {
    match kind {
        IntegralRungKind::OpeningAnchor => 0,
        IntegralRungKind::Positive => 1,
        IntegralRungKind::ClosingBookend => 2,
    }
}

pub(crate) const fn rung_class_code(class: IntegralRungClass) -> u8 {
    match class {
        IntegralRungClass::EligibleConsensus => 0,
        IntegralRungClass::AmbiguousConsensus => 1,
        IntegralRungClass::InsufficientConsensus => 2,
        IntegralRungClass::CurrentHeadroom => 3,
        IntegralRungClass::Fault => 4,
    }
}

pub(crate) const fn stationarity_exclusion_code(exclusion: StationarityExclusion) -> u8 {
    match exclusion {
        StationarityExclusion::NoBlockingPlateau => 1,
        StationarityExclusion::TooFewDecorrelatedBlocks => 2,
        StationarityExclusion::ResolvedSlope => 3,
        StationarityExclusion::StructuredResidual => 4,
        StationarityExclusion::Arithmetic => 5,
        StationarityExclusion::ExcessiveDrift => 6,
    }
}

pub(crate) const fn outcome_code(outcome: VelocityIntegralOutcome) -> u8 {
    match outcome {
        VelocityIntegralOutcome::FirstRunRetained => 0,
        VelocityIntegralOutcome::RepeatabilityConfirmed => 1,
        VelocityIntegralOutcome::Inconclusive => 2,
        VelocityIntegralOutcome::Fault => 3,
        VelocityIntegralOutcome::RejectedPlanMismatch => 4,
        VelocityIntegralOutcome::Failed => 5,
    }
}

fn push_u64(record: &mut VelocitySweepCanonicalRecord, value: u64) {
    record.push_u32(value as u32);
    record.push_u32((value >> 32) as u32);
}

fn push_i64(record: &mut VelocitySweepCanonicalRecord, value: i64) {
    push_u64(record, value as u64);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ankyra::velocity_sweep_encoding::VELOCITY_SWEEP_CANONICAL_RECORD_MAX;
    use crate::autotune::velocity_measurement::{
        DyadicBlocker, PlateauSelection, RateBlock, RawVelocityObservation, StationarityWork,
        StationarityWorkPoll, StationarySuffix, SuffixDiagnostic, VelocityDirection,
    };
    use crate::commission_dispatch::test_support::breakaway_acquisition_for_test;
    use crate::commissioning::outer::state::{
        PositiveIRestBoundary, VelocitySettleMetrics, VelocitySettleTransition,
    };
    use crate::commissioning::outer::velocity::integral_evidence::{
        ExactRateInterval, IntegralRungEvidence,
    };
    use crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder;
    use crate::commissioning::outer::velocity::recovery::RecoveryStage;
    use crate::commissioning::outer::velocity::sweep_evidence::{
        VelocityObservationClass, VelocityObservationSummary,
    };
    use crate::commissioning::outer::velocity::velocity_clamp::{
        VelocityCurrentContact, VelocityCurrentPhaseEvidence, VelocityLimitEvidenceCapability,
    };
    use crate::commissioning::types::IntegralOutcomeCause;
    use crate::commissioning::types::RecoveryRestDiagnostic;
    use crate::commissioning::types::VelocityIntegralObservation;
    use foci_tmc4671::pid::PidPGain;

    const DRIFT_RATIO_PPM_OFFSET: usize = 45;

    fn read_u32_at(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    fn recovery_rest_diagnostic() -> RecoveryRestDiagnostic {
        RecoveryRestDiagnostic {
            plan_digest: 0x0123_4567_89ab_cdef,
            exact_velocity_interval_q: [-17, 29],
            run_sequence: 7,
            start_offset_counts: -1_212,
            closest_offset_counts: -947,
            final_offset_counts: -947,
            planned_return_distance_counts: 212,
            first_in_band_offset_counts: -1_000,
            displacement_to_band_counts: 212,
            entry_to_band_duration_us: 20_609,
            lead_in_net_displacement_counts: 42,
            lead_in_displacement_span_counts: 43,
            scored_net_displacement_counts: 0,
            scored_min_displacement_counts: 0,
            scored_max_displacement_counts: 0,
            p_raw: 1_024,
            evidence_sequence: 11,
            peak_torque_target_abs: 1_560,
            stage: RecoveryStage::Integral,
            rung_index: 4,
            stationarity_code: 0,
            interval_present: 1,
            current_contact_code: 0,
            disposition: 0,
        }
    }

    #[test]
    fn integral_recovery_rest_reuses_the_exact_event_150_record() {
        let report = recovery_rest_diagnostic();
        let encoded = encode_velocity_integral_event(VelocityIntegralEvent::RecoveryRest(report));
        let shared = super::super::sweep_reporting::encode_recovery_rest(report);

        assert_eq!(encoded.as_bytes(), shared.as_bytes());
        assert_eq!(encoded.as_bytes().len(), 89);
    }

    #[test]
    fn plan_trace_core_appends_i_top_cap_only_at_schema_seventeen_or_eighteen() {
        let mut positive_i = [0; super::super::integral_plan::INTEGRAL_MAX_POSITIVE_RUNGS];
        for (index, gain) in positive_i.iter_mut().enumerate() {
            *gain = u16::try_from(index + 1).unwrap();
        }
        let mut plan = super::super::integral_plan::VelocityIntegralPlan {
            slot_order: IntegralSlotOrder::ForwardFirst,
            build_features: 3,
            schema_revision: super::super::integral_plan::INTEGRAL_EVIDENCE_SCHEMA_REVISION,
            authority: super::super::integral_plan::IntegralAcquisitionAuthority {
                channel: 0,
                acquisition: breakaway_acquisition_for_test(),
                directional_memberships: [0x0038_0000, 0x0038_0000],
                proportional_joint_membership: 0x0038_0000,
                joint_membership: 0x0018_0000,
                directional_validity: [
                    super::super::reproduction::ProportionalDirectionalValidity::ReproducedRegionCoversFinalP;
                    2
                ],
                reduced_margin: false,
                directional_pooled_intervals: [
                    [17_000_000, 18_000_000],
                    [-19_000_000, -18_000_000],
                ],
                confidence_q: 0,
                band_position_q: 0,
            },
            fixed_p: 1_448,
            planned_velocity_mrev_s: 4_394,
            target_velocity_rpm: 264,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            positive_i,
            positive_rung_count: 30,
            i_top_cap: 0,
            family_size: 128,
            total_rung_count: 32,
            expected_observations: 256,
            moving_stroke_us: 512_000,
            zero_settle_us: 500_000,
            analysis_budget_us: 236_000,
            nominal_workflow_ms: 82_607,
            maximum_workflow_ms: 90_888,
            recovery_origin_band_counts: 1_000,
            recovery_nominal_slot_us: 1_816_958,
            recovery_maximum_slot_us: 3_000_000,
            recovery_slot_count: 7,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
            limit_evidence_capability:
                super::super::velocity_clamp::VelocityLimitEvidenceCapability::BestEffortPolling,
            recovery_quantization_exposed: true,
            probe_constrained_test_point: true,
            motion_request: super::super::sweep_plan::VelocitySweepRequest {
                requested_velocity_mrev_s: 4_394,
                machine_velocity_ceiling_mrev_s: 10_000,
                requested_velocity_source:
                    super::super::sweep_plan::RequestedVelocitySource::Explicit,
                max_stroke_travel_mrev: 10_000,
                settle_travel_reserve_mrev: 100,
                negative_position_headroom_mrev: 20_000,
                positive_position_headroom_mrev: 20_000,
                max_duration_ms: 60_000,
            },
        };

        plan.schema_revision = 3;
        plan.i_top_cap = 0;
        let [core_baseline, ..] = encode_velocity_integral_plan_trace(1, plan);
        let len_baseline = core_baseline.as_bytes().len();

        plan.schema_revision = 17;
        plan.i_top_cap = 1_310;
        let [core_17, ..] = encode_velocity_integral_plan_trace(1, plan);
        let len_17 = core_17.as_bytes().len();

        plan.schema_revision = 18;
        plan.i_top_cap = 1_310;
        let [core_18, ..] = encode_velocity_integral_plan_trace(1, plan);
        let len_18 = core_18.as_bytes().len();

        assert_eq!(
            len_17,
            len_baseline + 2,
            "schema 17 adds exactly one trailing u16"
        );
        assert_eq!(
            len_18,
            len_baseline + 2,
            "schema 18 adds exactly one trailing u16"
        );
        for core in [&core_17, &core_18] {
            let bytes = core.as_bytes();
            let tail = &bytes[bytes.len() - 2..];
            assert_eq!(u16::from_le_bytes([tail[0], tail[1]]), 1_310);
        }
    }

    fn eligible_rung(direction: VelocityDirection) -> IntegralRungEvidence {
        let interval = ExactRateInterval::new(10, 20).unwrap();
        IntegralRungEvidence {
            direction,
            kind: IntegralRungKind::Positive,
            classification: IntegralRungClass::EligibleConsensus,
            interval: Some(interval),
            components: [interval, ExactRateInterval::EMPTY],
            component_count: 1,
            observation_classes: [IntegralObservationClass::Usable; 4],
            observation_intervals: [Some(interval); 4],
            usable_mask: 0b1111,
            included_mask: 0b1111,
        }
    }

    #[test]
    fn rung_record_carries_no_response_horizon_fields() {
        let report = VelocityIntegralRungReport {
            run_sequence: 7,
            evidence_sequence: 9,
            plan_digest: 0x0123_4567_89ab_cdef,
            rung_index: 9,
            i_raw: 64,
            directions: [
                eligible_rung(VelocityDirection::Forward),
                eligible_rung(VelocityDirection::Reverse),
            ],
            target_terminus_plus_one: 0,
        };

        assert_eq!(
            encode_velocity_integral_event(VelocityIntegralEvent::Rung(report))
                .as_bytes()
                .len(),
            71
        );
    }

    #[test]
    fn integral_probe_constrained_flags_and_trace_plan_preserve_reconstruction_inputs() {
        let mut positive_i = [0; super::super::integral_plan::INTEGRAL_MAX_POSITIVE_RUNGS];
        for (index, gain) in positive_i.iter_mut().enumerate() {
            *gain = u16::try_from(index + 1).unwrap();
        }
        let plan = super::super::integral_plan::VelocityIntegralPlan {
            slot_order: IntegralSlotOrder::ForwardFirst,
            build_features: 3,
            schema_revision: super::super::integral_plan::INTEGRAL_EVIDENCE_SCHEMA_REVISION,
            authority: super::super::integral_plan::IntegralAcquisitionAuthority {
                channel: 0,
                acquisition: breakaway_acquisition_for_test(),
                directional_memberships: [0x0038_0000, 0x0038_0000],
                proportional_joint_membership: 0x0038_0000,
                joint_membership: 0x0018_0000,
                directional_validity: [
                    super::super::reproduction::ProportionalDirectionalValidity::ReproducedRegionCoversFinalP;
                    2
                ],
                reduced_margin: false,
                directional_pooled_intervals: [
                    [17_000_000, 18_000_000],
                    [-19_000_000, -18_000_000],
                ],
                confidence_q: 0,
                band_position_q: 0,
            },
            fixed_p: 1_448,
            planned_velocity_mrev_s: 4_394,
            target_velocity_rpm: 264,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            positive_i,
            positive_rung_count: 30,
            i_top_cap: 0,
            family_size: 128,
            total_rung_count: 32,
            expected_observations: 256,
            moving_stroke_us: 512_000,
            zero_settle_us: 500_000,
            analysis_budget_us: 236_000,
            nominal_workflow_ms: 82_607,
            maximum_workflow_ms: 90_888,
            recovery_origin_band_counts: 1_000,
            recovery_nominal_slot_us: 1_816_958,
            recovery_maximum_slot_us: 3_000_000,
            recovery_slot_count: 7,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
            limit_evidence_capability:
                super::super::velocity_clamp::VelocityLimitEvidenceCapability::BestEffortPolling,
            recovery_quantization_exposed: true,
            probe_constrained_test_point: true,
            motion_request: super::super::sweep_plan::VelocitySweepRequest {
                requested_velocity_mrev_s: 4_394,
                machine_velocity_ceiling_mrev_s: 10_000,
                requested_velocity_source:
                    super::super::sweep_plan::RequestedVelocitySource::Explicit,
                max_stroke_travel_mrev: 10_000,
                settle_travel_reserve_mrev: 100,
                negative_position_headroom_mrev: 20_000,
                positive_position_headroom_mrev: 20_000,
                max_duration_ms: 60_000,
            },
        };

        let records = encode_velocity_integral_plan_trace(7, plan);

        assert_eq!(records.len(), 3);
        assert_eq!(records[0].as_bytes()[0], 0);
        assert_eq!(records[1].as_bytes()[0], 0);
        assert_eq!(records[2].as_bytes()[0], 0);
        assert_eq!(&records[0].as_bytes()[1..5], &7_u32.to_le_bytes());
        assert_eq!(&records[0].as_bytes()[7..15], &plan.digest().to_le_bytes());
        assert_eq!(records[0].as_bytes()[15], 0);
        assert_eq!(records[1].as_bytes()[15], 1);
        assert_eq!(records[2].as_bytes()[15], 2);
        assert_eq!(records[0].as_bytes().len(), 99);
        assert_eq!(
            &records[0].as_bytes()[43..47],
            &plan.authority.proportional_joint_membership.to_le_bytes()
        );
        assert_eq!(
            &records[0].as_bytes()[47..51],
            &plan.authority.joint_membership.to_le_bytes()
        );
        assert_eq!(
            &records[0].as_bytes()[51..53],
            &[super::super::reproduction::ProportionalDirectionalValidity::ReproducedRegionCoversFinalP
                as u8; 2]
        );
        assert_eq!(records[0].as_bytes()[53], 0);
        assert_eq!(
            records[0].as_bytes()[96],
            super::super::velocity_clamp::VelocityLimitEvidenceCapability::BestEffortPolling as u8
        );
        assert_eq!(
            &records[0].as_bytes()[97..99],
            &plan.i_top_cap.to_le_bytes()
        );
        assert_eq!(
            records[2].as_bytes()[29],
            PLAN_RECOVERY_QUANTIZATION_EXPOSED | PLAN_PROBE_CONSTRAINED_TEST_POINT
        );
        assert!(
            records
                .iter()
                .all(|record| record.as_bytes().len() <= VELOCITY_SWEEP_CANONICAL_RECORD_MAX)
        );
        assert_eq!(records[1].as_bytes().len(), 124);
    }

    fn plan() -> VelocityIntegralExecutionPlan {
        VelocityIntegralExecutionPlan {
            slot_order: IntegralSlotOrder::ForwardFirst,
            plan_digest: 0x0123_4567_89ab_cdef,
            acceptance_plan_digest: 0,
            qualifier_bits: 0,
            fixed_p: 1_448,
            planned_velocity_mrev_s: 4_394,
            target_velocity_rpm: 264,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            i_start: 5,
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
            limit_evidence_capability:
                super::super::velocity_clamp::VelocityLimitEvidenceCapability::BestEffortPolling,
            amplitude_targets_rpm: [0; 5],
            selected_response_intervals_q: [[0; 2]; 2],
            recovery_lower_rate_q: [0; 2],
            recovery_quantization_exposed: true,
            probe_constrained_test_point: false,
        }
    }

    #[test]
    fn plan_marker_starts_sequence_one_and_has_stable_digest() {
        let integrity = VelocityIntegralIntegrity::new(7, plan());

        assert_eq!(integrity.run_sequence(), 7);
        assert_eq!(integrity.next_evidence_sequence(), 1);
        assert_eq!(integrity.emitted_observations(), 0);
        assert_eq!(integrity.emitted_rungs(), 0);
        assert_eq!(integrity.digest(), 0xabbe_8f32_faa4_a140);
    }

    #[test]
    fn production_integral_digest_matches_trace_baseline() {
        use crate::commissioning::types::{
            VelocityIntegralEvent, VelocityPrimitiveCurrentEvidence,
        };

        let mut integrity = VelocityIntegralIntegrity::new(7, plan());
        let current = VelocityPrimitiveCurrentEvidence {
            run_sequence: 7,
            evidence_sequence: 1,
            stage: 1,
            rung_index: 2,
            slot: 5,
            clamp_limit: 2_534,
            clamp_readback: 2_534,
            moving_pid_output_peak: 2_111,
            zero_pid_output_peak: 2_534,
            moving_status_flags: 0,
            zero_status_flags: 1 << 15,
            capability: 0,
            contact: 2,
        };

        integrity.record(VelocityIntegralEvent::Current(current));

        const EXPECTED_BASELINE_DIGEST: u64 = 0x3b7f_c863_4c84_7223;
        assert_eq!(integrity.digest(), EXPECTED_BASELINE_DIGEST);
    }

    #[test]
    fn primitive_current_evidence_is_hashed_and_count_neutral() {
        let mut integrity = VelocityIntegralIntegrity::new(7, plan());
        let before = integrity.digest();
        let evidence = crate::commissioning::types::VelocityPrimitiveCurrentEvidence {
            run_sequence: 7,
            evidence_sequence: 1,
            stage: 1,
            rung_index: 2,
            slot: 5,
            clamp_limit: 2_534,
            clamp_readback: 2_534,
            moving_pid_output_peak: 2_111,
            zero_pid_output_peak: 2_534,
            moving_status_flags: 0,
            zero_status_flags: 1 << 15,
            capability: 0,
            contact: 2,
        };

        integrity.record(VelocityIntegralEvent::Current(evidence));

        assert_ne!(integrity.digest(), before);
        assert_eq!(integrity.emitted_observations(), 0);
        assert_eq!(integrity.emitted_rungs(), 0);
        assert_eq!(integrity.next_evidence_sequence(), 2);
        assert_eq!(
            encode_velocity_integral_event(VelocityIntegralEvent::Current(evidence)).as_bytes()[0],
            31
        );
    }

    #[test]
    fn schema_five_rest_boundary_is_terminal_sequenced_and_digest_bearing() {
        let boundary = PositiveIRestBoundary {
            rung_index: 4,
            slot: 5,
            i_raw: 8,
            direction: -1,
            active_stationarity: StationarityResult::Usable(StationarySuffix {
                selection: PlateauSelection {
                    level: 2,
                    block_count: 16,
                    variance_of_mean_q2: 9,
                    exact_totals: RateBlock {
                        delta_counts: -3,
                        elapsed_us: 64_000,
                        observation_count: 64,
                    },
                },
                suffix_len: 64,
                slope_q: -7,
                slope_half_width_q: 11,
                lag_one_q: 13,
                lag_one_half_width_q: 17,
                residual_magnitude_q2: 19,
                span_x2_us: 0,
                drift_ratio_ppm: u32::MAX,
                tested_suffixes: 3,
            }),
            active_settle: VelocitySettleMetrics {
                sample_count: 433,
                duration_us: 433_000,
                net_displacement_counts: -31,
                minimum_displacement_counts: -33,
                maximum_displacement_counts: 2,
                transition_count: 7,
                first_transition: VelocitySettleTransition {
                    sample_index: 3,
                    elapsed_us: 3_000,
                    delta_counts: -1,
                    displacement_counts: -1,
                },
                last_transition: VelocitySettleTransition {
                    sample_index: 401,
                    elapsed_us: 401_000,
                    delta_counts: 2,
                    displacement_counts: -31,
                },
            },
            zero_phase_current: VelocityCurrentPhaseEvidence {
                clamp_limit: 2_534,
                moving_pid_output_peak: 2_111,
                zero_pid_output_peak: 2_400,
                moving_status_flags: 0x1122_3344,
                zero_status_flags: 0x5566_7788,
                capability: VelocityLimitEvidenceCapability::LatchedFlag,
                contact: VelocityCurrentContact::None,
            },
            cleanup_confirmed: true,
        };
        let record = encode_velocity_integral_rest_boundary(7, 19, 0x0123_4567_89ab_cdef, boundary);
        assert_eq!(record.as_bytes()[0], 5);
        assert_eq!(&record.as_bytes()[1..5], 7_u32.to_le_bytes());
        assert_eq!(&record.as_bytes()[5..7], 19_u16.to_le_bytes());
        assert_eq!(record.as_bytes().len(), 117);
        assert!(record.as_bytes().len() <= VELOCITY_SWEEP_CANONICAL_RECORD_MAX);

        let mut integrity = VelocityIntegralIntegrity::new(7, plan());
        let sequence = integrity.next_evidence_sequence();
        let observations = integrity.emitted_observations();
        let rungs = integrity.emitted_rungs();
        let digest = integrity.digest();
        integrity.record_terminal_boundary(&record);
        assert_eq!(integrity.next_evidence_sequence(), sequence);
        assert_eq!(integrity.emitted_observations(), observations);
        assert_eq!(integrity.emitted_rungs(), rungs);
        assert_ne!(integrity.digest(), digest);
    }

    #[test]
    fn exact_u64_terminal_words_survive_canonical_encoding() {
        let mut terminal = VelocityIntegralTerminal {
            run_sequence: 7,
            evidence_sequence: 9,
            plan_digest: 0x0123_4567_89ab_cdef,
            digest: 0xfedc_ba98_7654_3210,
            outcome: VelocityIntegralOutcome::RepeatabilityConfirmed,
            cause: IntegralOutcomeCause::Engine(0),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 5,
            rest_boundary_slot_plus_one: 6,
            evidence: super::super::integral_evidence::IntegralRunEvidence::new(
                [super::super::integral_evidence::ExactRateInterval::EMPTY; 2],
            ),
            expected_observations: 56,
            emitted_observations: 56,
            expected_rungs: 7,
            emitted_rungs: 7,
            run_started_us: 0x1111_2222_3333_4444,
            run_completed_us: 0x5555_6666_7777_8888,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };
        terminal.evidence.opening = [
            super::super::integral_evidence::ExactRateInterval::new(100, 200).unwrap(),
            super::super::integral_evidence::ExactRateInterval::new(-200, -100).unwrap(),
        ];
        terminal.evidence.bookend = [
            Some(super::super::integral_evidence::ExactRateInterval::new(110, 210).unwrap()),
            Some(super::super::integral_evidence::ExactRateInterval::new(-210, -110).unwrap()),
        ];

        let record = encode_velocity_integral_terminal(&terminal);
        assert!(record.as_bytes().len() <= 128);
        assert!(
            record
                .as_bytes()
                .windows(8)
                .any(|word| word == terminal.plan_digest.to_le_bytes())
        );
        assert!(
            record
                .as_bytes()
                .windows(8)
                .any(|word| word == terminal.digest.to_le_bytes())
        );
        assert_eq!(record.as_bytes()[42], 0b11);
        assert_eq!(record.as_bytes()[43], 0b11);
        assert_eq!(
            record.as_bytes(),
            &[
                0x04, 0x07, 0x00, 0x00, 0x00, 0x09, 0x00, 0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23,
                0x01, 0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, 0x01, 0x00, 0x00, 0x38, 0x00,
                0x38, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x03, 0x03, 0x00, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0x88, 0x88, 0x77,
                0x77, 0x66, 0x66, 0x55, 0x55, 0x00,
            ],
            "schema-4 terminal bytes, cause_namespace appended"
        );
    }

    #[test]
    fn encode_velocity_integral_terminal_reply_is_stable_and_within_wire_budget() {
        let mut terminal = VelocityIntegralTerminal {
            run_sequence: 0x1122_3344,
            evidence_sequence: 0,
            plan_digest: 0,
            digest: 0,
            outcome: VelocityIntegralOutcome::RepeatabilityConfirmed,
            cause: IntegralOutcomeCause::Engine(0x2A),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence: super::super::integral_evidence::IntegralRunEvidence::new(
                [super::super::integral_evidence::ExactRateInterval::EMPTY; 2],
            ),
            expected_observations: 0,
            emitted_observations: 0,
            expected_rungs: 0,
            emitted_rungs: 0,
            run_started_us: 0,
            run_completed_us: 0,
            safety_fault: None,
            rest_rejection_after_sufficiency: true,
            rest_rejection_owner: Some(RestSelectionOwner::IntegralRecovery),
        };
        terminal.evidence.eligible_masks = [0x0000_0007, 0x0000_0003];
        terminal.evidence.bookend = [
            Some(super::super::integral_evidence::ExactRateInterval::new(1, 2).unwrap()),
            None,
        ];
        terminal.evidence.current_terminus = Some(3);

        let reproduction = VelocityIntegralTerminalReproductionSummary {
            reproduced_masks: [0x0000_0005, 0x0000_0002],
            divergent_masks: [0x0000_0002, 0x0000_0001],
        };

        let bytes = encode_velocity_integral_terminal_reply(&terminal, Some(reproduction));
        assert_eq!(bytes.len(), VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES);

        #[rustfmt::skip]
        let expected: [u8; VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES] = [
            0x02,
            0x44, 0x33, 0x22, 0x11,
            0x01,
            0x2A, 0x00,
            0x50,
            0x07, 0x00, 0x00, 0x00,
            0x03, 0x00, 0x00, 0x00,
            0x01,
            0x04,
            0x01,
            0x05, 0x00, 0x00, 0x00,
            0x02, 0x00, 0x00, 0x00,
            0x02, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00,
        ];
        assert_eq!(bytes, expected);

        let bytes_no_reproduction = encode_velocity_integral_terminal_reply(&terminal, None);
        assert_eq!(
            bytes_no_reproduction[19], 0x00,
            "reproduction_available clear"
        );
        assert_eq!(
            &bytes_no_reproduction[20..36],
            &[0_u8; 16],
            "reproduction masks zeroed when absent"
        );
    }

    #[test]
    fn encode_velocity_integral_terminal_reply_carries_candidate_i_when_present() {
        let mut terminal = VelocityIntegralTerminal {
            run_sequence: 1,
            evidence_sequence: 0,
            plan_digest: 0,
            digest: 0,
            outcome: VelocityIntegralOutcome::RepeatabilityConfirmed,
            cause: IntegralOutcomeCause::Engine(0),
            breakaway_acceptance: true,
            fixed_p: 724,
            candidate_i: Some(512),
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence: super::super::integral_evidence::IntegralRunEvidence::new(
                [super::super::integral_evidence::ExactRateInterval::EMPTY; 2],
            ),
            expected_observations: 0,
            emitted_observations: 0,
            expected_rungs: 0,
            emitted_rungs: 0,
            run_started_us: 0,
            run_completed_us: 0,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };
        terminal.evidence.eligible_masks = [0, 0];

        let bytes = encode_velocity_integral_terminal_reply(&terminal, None);
        assert_eq!(bytes.len(), VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES);
        assert_eq!(bytes[0], 2, "schema revision bumped to 2");
        assert_eq!(bytes[36], 1, "candidate_i presence flag set");
        assert_eq!(&bytes[37..39], &512_u16.to_le_bytes(), "candidate_i value");

        terminal.candidate_i = None;
        let bytes_absent = encode_velocity_integral_terminal_reply(&terminal, None);
        assert_eq!(bytes_absent[36], 0, "candidate_i presence flag clear");
        assert_eq!(
            &bytes_absent[37..39],
            &[0, 0],
            "candidate_i value zeroed when absent"
        );
    }

    #[test]
    fn encode_velocity_integral_terminal_reply_masks_retired_recovery_state_bits() {
        let mut terminal = VelocityIntegralTerminal {
            run_sequence: 1,
            evidence_sequence: 0,
            plan_digest: 0,
            digest: 0,
            outcome: VelocityIntegralOutcome::RepeatabilityConfirmed,
            cause: IntegralOutcomeCause::Engine(0),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
            recovery_unavailable: true,
            recovered_with_current_headroom: true,
            recovery_quantization_exposed: true,
            probe_constrained_test_point: true,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence: super::super::integral_evidence::IntegralRunEvidence::new(
                [super::super::integral_evidence::ExactRateInterval::EMPTY; 2],
            ),
            expected_observations: 0,
            emitted_observations: 0,
            expected_rungs: 0,
            emitted_rungs: 0,
            run_started_us: 0,
            run_completed_us: 0,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };
        terminal.evidence.bookend = [None, None];

        let bytes = encode_velocity_integral_terminal_reply(&terminal, None);

        assert_eq!(
            bytes[8], 0x00,
            "retired recovery-state bits must not appear on the compact reply"
        );
    }

    #[test]
    fn integral_terminal_flags_are_packed_once() {
        let terminal = VelocityIntegralTerminal {
            run_sequence: 7,
            evidence_sequence: 9,
            plan_digest: 0,
            digest: 0,
            outcome: VelocityIntegralOutcome::Inconclusive,
            cause: IntegralOutcomeCause::Engine(8),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
            recovery_unavailable: false,
            recovered_with_current_headroom: true,
            recovery_quantization_exposed: true,
            probe_constrained_test_point: true,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence: super::super::integral_evidence::IntegralRunEvidence::new(
                [super::super::integral_evidence::ExactRateInterval::EMPTY; 2],
            ),
            expected_observations: 112,
            emitted_observations: 48,
            expected_rungs: 14,
            emitted_rungs: 6,
            run_started_us: 0,
            run_completed_us: 1,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };

        assert_eq!(
            terminal_recovery_flags(&terminal),
            TERMINAL_RECOVERED_WITH_CURRENT_HEADROOM
                | TERMINAL_RECOVERY_QUANTIZATION_EXPOSED
                | TERMINAL_PROBE_CONSTRAINED_TEST_POINT
        );
        assert_eq!(
            encode_velocity_integral_terminal(&terminal).as_bytes()[25],
            TERMINAL_RECOVERED_WITH_CURRENT_HEADROOM
                | TERMINAL_RECOVERY_QUANTIZATION_EXPOSED
                | TERMINAL_PROBE_CONSTRAINED_TEST_POINT
        );
        assert_eq!(terminal_recovery_flags(&terminal) & 0xf0, 0);
    }

    #[test]
    fn integral_terminal_flags_pack_the_rest_rejection_discriminator() {
        let mut terminal = VelocityIntegralTerminal {
            run_sequence: 7,
            evidence_sequence: 9,
            plan_digest: 0,
            digest: 0,
            outcome: VelocityIntegralOutcome::Inconclusive,
            cause: IntegralOutcomeCause::Error(53),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence: super::super::integral_evidence::IntegralRunEvidence::new(
                [super::super::integral_evidence::ExactRateInterval::EMPTY; 2],
            ),
            expected_observations: 112,
            emitted_observations: 48,
            expected_rungs: 14,
            emitted_rungs: 6,
            run_started_us: 0,
            run_completed_us: 1,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };
        assert_eq!(terminal_recovery_flags(&terminal) & 0xf0, 0);

        for (owner, code) in [
            (RestSelectionOwner::IntegralAnchor, 0b00),
            (RestSelectionOwner::IntegralPositiveObservation, 0b01),
            (RestSelectionOwner::IntegralRecovery, 0b10),
            (RestSelectionOwner::IntegralCleanup, 0b11),
        ] {
            terminal.rest_rejection_after_sufficiency = true;
            terminal.rest_rejection_owner = Some(owner);
            let flags = terminal_recovery_flags(&terminal);
            assert_eq!(
                flags & TERMINAL_REST_REJECTION_AFTER_SUFFICIENCY,
                TERMINAL_REST_REJECTION_AFTER_SUFFICIENCY
            );
            assert_eq!(
                (flags & TERMINAL_REST_REJECTION_OWNER_MASK) >> TERMINAL_REST_REJECTION_OWNER_SHIFT,
                code
            );
            assert_eq!(flags & (1 << 7), 0);
        }
    }

    #[test]
    fn breakaway_acceptance_terminal_exposes_target_status_and_terminus() {
        let mut terminal = VelocityIntegralTerminal {
            run_sequence: 7,
            evidence_sequence: 9,
            plan_digest: 0,
            digest: 0,
            outcome: VelocityIntegralOutcome::RepeatabilityConfirmed,
            cause: IntegralOutcomeCause::Engine(0),
            breakaway_acceptance: true,
            fixed_p: 512,
            candidate_i: None,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 3,
            rest_boundary_slot_plus_one: 0,
            evidence: super::super::integral_evidence::IntegralRunEvidence::new(
                [super::super::integral_evidence::ExactRateInterval::EMPTY; 2],
            ),
            expected_observations: 112,
            emitted_observations: 112,
            expected_rungs: 14,
            emitted_rungs: 14,
            run_started_us: 0,
            run_completed_us: 1,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };
        terminal.evidence.eligible_masks = [0b111, 0b111];

        assert_eq!(
            terminal_target_status(&terminal),
            Some(CombinedTargetStatus::TargetReached)
        );
        assert_eq!(terminal_target_terminus(&terminal), Some(2));
    }

    fn excluded_observation(
        exclusion: StationarityExclusion,
        rejection: Option<SuffixDiagnostic>,
    ) -> VelocityIntegralObservation {
        VelocityIntegralObservation {
            run_sequence: 3,
            evidence_sequence: 79,
            plan_digest: 0x0123_4567_89ab_cdef,
            rung_index: 3,
            slot: 6,
            i_raw: 512,
            summary: VelocityObservationSummary {
                direction: VelocityDirection::Forward,
                p: PidPGain::new(1_024).unwrap(),
                classification: VelocityObservationClass::StationarityExcluded(exclusion),
                settled_rate_interval_q: [0, 0],
                settled_rate_mean_q: 0,
                deficit_interval_q: [0, 0],
                disturbance: None,
                disturbance_interval: None,
                disturbance_variance_q2: 0,
                predicted_torque_target_abs: 1_023,
                drift_ratio_ppm: u32::MAX,
                initial_lead_in: false,
            },
            stationarity: StationarityResult::Excluded(exclusion),
            rejection: rejection.map(Into::into),
        }
    }

    fn rejection_diagnostic_fixture() -> SuffixDiagnostic {
        SuffixDiagnostic {
            suffix_len: 256,
            selection: Some(PlateauSelection {
                level: 4,
                block_count: 16,
                variance_of_mean_q2: 0,
                exact_totals: RateBlock {
                    delta_counts: -256,
                    elapsed_us: 256_000,
                    observation_count: 256,
                },
            }),
            usable_plateau: true,
            slope_passed: false,
            lag_one_passed: true,
            lag_one_q: 0,
            settled_rate_interval_q: [-8_773_356, -8_351_644],
            slope_rejection_margin_q: -2_389_706,
            lag_one_rejection_margin_q: 55_874,
            slope_q: -6_100_000,
            slope_half_width_q: 3_710_294,
        }
    }

    fn usable_suffix_fixture() -> StationarySuffix {
        StationarySuffix {
            selection: PlateauSelection {
                level: 4,
                block_count: 16,
                variance_of_mean_q2: 0,
                exact_totals: RateBlock {
                    delta_counts: 0,
                    elapsed_us: 0,
                    observation_count: 0,
                },
            },
            suffix_len: 256,
            slope_q: -2_389_706,
            slope_half_width_q: 8_628_554,
            lag_one_q: -680_061,
            lag_one_half_width_q: 673_435,
            residual_magnitude_q2: 0,
            span_x2_us: 0,
            drift_ratio_ppm: u32::MAX,
            tested_suffixes: 3,
        }
    }

    #[test]
    fn an_excluded_observation_records_why_the_gate_rejected_it() {
        let rejection = rejection_diagnostic_fixture();
        let record = encode_observation(excluded_observation(
            StationarityExclusion::ResolvedSlope,
            Some(rejection),
        ));
        let bytes = record.as_bytes();

        let code_offset = bytes.len() - 1 - REJECTION_DIAGNOSTIC_BYTES - 1;
        assert_eq!(bytes[code_offset], 3);
        assert_eq!(bytes[code_offset + 1], 1, "presence flag must be set");

        let tail = &bytes[code_offset + 2..];
        assert_eq!(&tail[0..2], 256_u16.to_le_bytes());
        assert_eq!(tail[2], 1, "usable_plateau");
        assert_eq!(tail[3], 0, "slope_passed");
        assert_eq!(tail[4], 1, "lag_one_passed");
        assert_eq!(&tail[5..13], (-8_773_356_i64).to_le_bytes());
        assert_eq!(&tail[13..21], (-8_351_644_i64).to_le_bytes());
        assert_eq!(&tail[21..29], (-2_389_706_i64).to_le_bytes());
        assert_eq!(&tail[29..33], 55_874_i32.to_le_bytes());
        assert_eq!(&tail[33..41], (-6_100_000_i64).to_le_bytes());
        assert_eq!(&tail[41..49], 3_710_294_u64.to_le_bytes());
        assert!(bytes.len() <= VELOCITY_SWEEP_CANONICAL_RECORD_MAX);
    }

    #[test]
    fn an_exclusion_before_the_gates_records_an_absent_diagnostic() {
        let record = encode_observation(excluded_observation(
            StationarityExclusion::NoBlockingPlateau,
            None,
        ));
        let bytes = record.as_bytes();

        assert_eq!(bytes[bytes.len() - 2], 1, "NoBlockingPlateau code");
        assert_eq!(bytes[bytes.len() - 1], 0, "presence flag must be clear");
    }

    #[test]
    fn a_usable_observation_keeps_its_established_layout() {
        let mut report = excluded_observation(StationarityExclusion::ResolvedSlope, None);
        report.summary.classification = VelocityObservationClass::Eligible;
        report.stationarity = StationarityResult::Usable(usable_suffix_fixture());

        let record = encode_observation(report);

        assert_eq!(record.as_bytes().len(), 81);
    }

    #[test]
    fn every_integral_observation_carries_the_drift_ratio_whatever_the_classification() {
        let mut excessive_report = excluded_observation(
            StationarityExclusion::ExcessiveDrift,
            Some(rejection_diagnostic_fixture()),
        );
        excessive_report.summary.drift_ratio_ppm = 412_009;
        let excessive = encode_observation(excessive_report);

        let mut pre_fit_report =
            excluded_observation(StationarityExclusion::NoBlockingPlateau, None);
        pre_fit_report.summary.drift_ratio_ppm = u32::MAX;
        let pre_fit = encode_observation(pre_fit_report);

        let mut usable_report = excluded_observation(StationarityExclusion::ResolvedSlope, None);
        usable_report.summary.classification = VelocityObservationClass::Eligible;
        usable_report.summary.drift_ratio_ppm = 38_503;
        usable_report.stationarity = StationarityResult::Usable(usable_suffix_fixture());
        let usable = encode_observation(usable_report);

        let offset = DRIFT_RATIO_PPM_OFFSET;
        assert_eq!(
            read_u32_at(usable.as_bytes(), offset),
            38_503,
            "a usable observation must carry the ratio"
        );
        assert_eq!(
            read_u32_at(excessive.as_bytes(), offset),
            412_009,
            "a post-fit exclusion must carry its evaluated ratio"
        );
        assert_eq!(
            read_u32_at(pre_fit.as_bytes(), offset),
            u32::MAX,
            "a pre-fit exclusion must report no finite ratio"
        );
    }

    #[test]
    fn excessive_drift_encodes_as_integral_exclusion_code_six() {
        let mut report = excluded_observation(
            StationarityExclusion::ExcessiveDrift,
            Some(rejection_diagnostic_fixture()),
        );
        report.summary.drift_ratio_ppm = 412_009;
        let record = encode_observation(report);
        let bytes = record.as_bytes();
        let code_offset = bytes.len() - 1 - REJECTION_DIAGNOSTIC_BYTES - 1;
        assert_eq!(bytes[code_offset], 6);
        assert_eq!(bytes[code_offset + 1], 1, "presence flag must be set");
    }

    fn drive_to_no_fit_exclusion<const LEVELS: usize>(
        observations: &[RawVelocityObservation],
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> (StationarityExclusion, u32) {
        let mut work = StationarityWork::new();
        loop {
            match work
                .poll(observations, blocker)
                .expect("bounded fixed-size observation feed stays in range")
            {
                StationarityWorkPoll::Pending => {}
                StationarityWorkPoll::Complete {
                    result,
                    drift_ratio_ppm,
                    ..
                } => {
                    let StationarityResult::Excluded(exclusion) = result else {
                        panic!("expected a no-fit exclusion, got a usable suffix");
                    };
                    return (exclusion, drift_ratio_ppm);
                }
                StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
                    panic!("moving mode returned a scored-rest result")
                }
            }
        }
    }

    fn observations_without_a_blocking_plateau() -> VelocityIntegralObservation {
        let observations: [RawVelocityObservation; 64] =
            core::array::from_fn(|_| RawVelocityObservation::new(1, 1_000).unwrap());
        let mut blocker = DyadicBlocker::<2>::new();
        let (exclusion, drift_ratio_ppm) = drive_to_no_fit_exclusion(&observations, &mut blocker);
        assert_eq!(exclusion, StationarityExclusion::NoBlockingPlateau);
        let mut report = excluded_observation(exclusion, None);
        report.summary.drift_ratio_ppm = drift_ratio_ppm;
        report
    }

    fn observations_with_two_blocks() -> VelocityIntegralObservation {
        let observations: [RawVelocityObservation; 2] =
            core::array::from_fn(|_| RawVelocityObservation::new(1, 1_000).unwrap());
        let mut blocker = DyadicBlocker::<9>::new();
        let (exclusion, drift_ratio_ppm) = drive_to_no_fit_exclusion(&observations, &mut blocker);
        assert_eq!(exclusion, StationarityExclusion::TooFewDecorrelatedBlocks);
        let mut report = excluded_observation(exclusion, None);
        report.summary.drift_ratio_ppm = drift_ratio_ppm;
        report
    }

    #[test]
    fn each_no_fit_path_encodes_the_sentinel_rather_than_zero() {
        for report in [
            observations_without_a_blocking_plateau(),
            observations_with_two_blocks(),
        ] {
            let record = encode_observation(report);
            assert_eq!(
                read_u32_at(record.as_bytes(), DRIFT_RATIO_PPM_OFFSET),
                u32::MAX,
                "a path with no fit reported a finite ratio"
            );
        }
    }
}
