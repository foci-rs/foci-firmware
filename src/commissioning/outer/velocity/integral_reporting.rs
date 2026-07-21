//! Canonical sequencing, integrity, and trace records for Stage C.

use crate::ankyra::velocity_sweep_encoding::{
    VelocitySweepCanonicalRecord, encode_rate_values, extend_velocity_sweep_digest,
    upper_bound_i64, upper_bound_u128,
};
use crate::autotune::velocity_measurement::{StationarityExclusion, StationarityResult};
use crate::commissioning::types::{
    VelocityIntegralEvent, VelocityIntegralObservation, VelocityIntegralOutcome,
    VelocityIntegralRungReport, VelocityIntegralTerminal,
};

use super::integral_evidence::{IntegralObservationClass, IntegralRungClass, IntegralRungKind};
use super::integral_plan::VelocityIntegralExecutionPlan;

const FNV1A64_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;

/// Fixed-memory sequencing and digest state for one Stage-C run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocityIntegralIntegrity {
    run_sequence: u32,
    next_evidence_sequence: u16,
    emitted_observations: u16,
    emitted_rungs: u16,
    digest: u64,
}

impl VelocityIntegralIntegrity {
    /// Start one run by hashing its compact immutable plan marker.
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

    /// Caller-assigned run sequence copied into every evidence group.
    pub(crate) const fn run_sequence(self) -> u32 {
        self.run_sequence
    }

    /// Sequence assigned to the next nonterminal evidence group.
    pub(crate) const fn next_evidence_sequence(self) -> u16 {
        self.next_evidence_sequence
    }

    /// Hash one already-sequenced event and advance exact counters.
    pub(crate) fn record(&mut self, event: VelocityIntegralEvent) -> VelocityIntegralEvent {
        match event {
            VelocityIntegralEvent::Observation(_) => {
                self.emitted_observations = self.emitted_observations.saturating_add(1);
            }
            VelocityIntegralEvent::Rung(_) => {
                self.emitted_rungs = self.emitted_rungs.saturating_add(1);
            }
            VelocityIntegralEvent::Planned(_) => {}
        }
        self.digest =
            extend_velocity_sweep_digest(self.digest, &encode_velocity_integral_event(event));
        self.next_evidence_sequence = self.next_evidence_sequence.saturating_add(1);
        event
    }

    /// Number of observation groups included in the current digest.
    pub(crate) const fn emitted_observations(self) -> u16 {
        self.emitted_observations
    }

    /// Number of rung groups included in the current digest.
    pub(crate) const fn emitted_rungs(self) -> u16 {
        self.emitted_rungs
    }

    /// Current exact FNV-1a digest.
    pub(crate) const fn digest(self) -> u64 {
        self.digest
    }
}

/// Encode one nonterminal Stage-C event in the canonical digest representation.
pub(crate) fn encode_velocity_integral_event(
    event: VelocityIntegralEvent,
) -> VelocitySweepCanonicalRecord {
    match event {
        VelocityIntegralEvent::Planned(report) => {
            let mut record = VelocitySweepCanonicalRecord::new();
            record.push_u8(1);
            record.push_u32(report.run_sequence);
            record.push_u16(report.evidence_sequence);
            push_u64(&mut record, report.plan_digest);
            record.push_u16(report.final_p);
            record.push_u8(report.positive_rung_count);
            record.push_u32(report.maximum_workflow_ms);
            record
        }
        VelocityIntegralEvent::Observation(report) => encode_observation(report),
        VelocityIntegralEvent::Rung(report) => encode_rung(report),
    }
}

/// Encode the detailed Stage-C terminal for trace-only parity.
pub(crate) fn encode_velocity_integral_terminal(
    report: VelocityIntegralTerminal,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(4);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    push_u64(&mut record, report.digest);
    record.push_u8(outcome_code(report.outcome));
    record.push_u8(report.cause);
    record.push_u16(report.expected_observations);
    record.push_u16(report.emitted_observations);
    record.push_u16(report.expected_rungs);
    record.push_u16(report.emitted_rungs);
    record.push_u32(report.evidence.eligible_masks[0]);
    record.push_u32(report.evidence.eligible_masks[1]);
    record.push_u8(report.evidence.current_terminus.map_or(0, |rung| rung + 1));
    push_u64(&mut record, report.run_started_us);
    push_u64(&mut record, report.run_completed_us);
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
    record.push_u16(plan.final_p);
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
    match report.stationarity {
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
    record
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
    }
}

pub(crate) const fn outcome_code(outcome: VelocityIntegralOutcome) -> u8 {
    match outcome {
        VelocityIntegralOutcome::CompleteCandidate => 0,
        VelocityIntegralOutcome::Complete => 1,
        VelocityIntegralOutcome::Inconclusive => 2,
        VelocityIntegralOutcome::Fault => 3,
        VelocityIntegralOutcome::RejectedPlanMismatch => 4,
    }
}

/// Rate-invert one Stage-B Q16.16 disturbance interval into Stage-C deficit-Q units.
pub(crate) fn stage_b_deficit_bounds_q(
    disturbance_q16: [i32; 2],
    p_raw: u16,
    encoder_counts_per_rev: u32,
) -> Option<[i32; 2]> {
    if disturbance_q16[0] > disturbance_q16[1] || p_raw == 0 || encoder_counts_per_rev == 0 {
        return None;
    }
    let scale = i128::from(256_u16)
        .checked_mul(i128::from(encoder_counts_per_rev))?
        .checked_mul(1_000)?;
    let denominator = i128::from(p_raw).checked_mul(65_536)?.checked_mul(60)?;
    let low = floor_ratio(
        i128::from(disturbance_q16[0]).checked_mul(scale)?,
        denominator,
    )?;
    let high = ceil_ratio(
        i128::from(disturbance_q16[1]).checked_mul(scale)?,
        denominator,
    )?;
    Some([i32::try_from(low).ok()?, i32::try_from(high).ok()?])
}

fn floor_ratio(numerator: i128, denominator: i128) -> Option<i128> {
    if denominator <= 0 {
        return None;
    }
    Some(numerator.div_euclid(denominator))
}

fn ceil_ratio(numerator: i128, denominator: i128) -> Option<i128> {
    let floor = floor_ratio(numerator, denominator)?;
    Some(if numerator.rem_euclid(denominator) == 0 {
        floor
    } else {
        floor.checked_add(1)?
    })
}

fn push_u64(record: &mut VelocitySweepCanonicalRecord, value: u64) {
    record.push_u32(value as u32);
    record.push_u32((value >> 32) as u32);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan() -> VelocityIntegralExecutionPlan {
        VelocityIntegralExecutionPlan {
            plan_digest: 0x0123_4567_89ab_cdef,
            final_p: 1_448,
            planned_velocity_mrev_s: 4_394,
            target_velocity_rpm: 264,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            i_start: 5,
            positive_rung_count: 5,
            family_size: 28,
            moving_stroke_us: 512_000,
            zero_settle_us: 500_000,
            analysis_budget_us: 236_000,
            maximum_workflow_ms: 69_888,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
        }
    }

    #[test]
    fn plan_marker_starts_sequence_one_and_has_stable_digest() {
        let integrity = VelocityIntegralIntegrity::new(7, plan());

        assert_eq!(integrity.run_sequence(), 7);
        assert_eq!(integrity.next_evidence_sequence(), 1);
        assert_eq!(integrity.emitted_observations(), 0);
        assert_eq!(integrity.emitted_rungs(), 0);
        assert_eq!(integrity.digest(), 0x271e_1a4f_8f00_23c7);
    }

    #[test]
    fn exact_u64_terminal_words_survive_canonical_encoding() {
        let terminal = VelocityIntegralTerminal {
            run_sequence: 7,
            evidence_sequence: 9,
            plan_digest: 0x0123_4567_89ab_cdef,
            digest: 0xfedc_ba98_7654_3210,
            outcome: VelocityIntegralOutcome::Complete,
            cause: 0,
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
        };

        let record = encode_velocity_integral_terminal(terminal);
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
    }

    #[test]
    fn stage_b_rate_inversion_is_outward_and_sign_symmetric() {
        assert_eq!(
            stage_b_deficit_bounds_q([65_536, 65_536], 256, 4_000),
            Some([66_666, 66_667])
        );
        assert_eq!(
            stage_b_deficit_bounds_q([-65_536, -65_536], 256, 4_000),
            Some([-66_667, -66_666])
        );
    }
}
