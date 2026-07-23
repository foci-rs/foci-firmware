//! Streamed velocity-sweep evidence sequencing and integrity.

use crate::autotune::velocity_measurement::StationarityResult;
use crate::commissioning::types::{
    RungOriginRecoveryEvidence, VelocitySweepEvent, VelocitySweepObservation, VelocitySweepPlanned,
    VelocitySweepRungReport, VelocitySweepTerminal,
};

use super::sweep_consensus::consensus_class_report_code;
use super::sweep_evidence::{common_rate_mantissas, observation_class_report_code};
use crate::ankyra::velocity_sweep_encoding::{
    VelocitySweepCanonicalRecord, encode_rate_values, extend_velocity_sweep_digest,
    upper_bound_i64, upper_bound_u128,
};

const FNV1A64_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;

/// Fixed-memory sequencing and digest state for one report-only sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocitySweepIntegrity {
    run_sequence: u32,
    next_evidence_sequence: u16,
    emitted_observations: u16,
    emitted_rungs: u16,
    directional_region_counts: [u8; 2],
    directional_fragment_counts: [u8; 2],
    digest: u64,
}

impl VelocitySweepIntegrity {
    /// Construct integrity state for one caller-assigned run sequence.
    pub(crate) const fn new(run_sequence: u32) -> Self {
        Self {
            run_sequence,
            next_evidence_sequence: 0,
            emitted_observations: 0,
            emitted_rungs: 0,
            directional_region_counts: [0; 2],
            directional_fragment_counts: [0; 2],
            digest: FNV1A64_OFFSET,
        }
    }

    /// Return the run sequence copied into every report fragment.
    pub(crate) const fn run_sequence(self) -> u32 {
        self.run_sequence
    }

    /// Return the sequence assigned to the next emitted evidence group.
    pub(crate) const fn next_evidence_sequence(self) -> u16 {
        self.next_evidence_sequence
    }

    /// Fold one event into the digest and advance its evidence sequence.
    pub(crate) fn record(&mut self, event: VelocitySweepEvent) -> VelocitySweepEvent {
        match event {
            VelocitySweepEvent::Planned(_) => {}
            VelocitySweepEvent::Observation(_) => {
                self.emitted_observations = self.emitted_observations.saturating_add(1);
            }
            VelocitySweepEvent::Rung(_) => {
                self.emitted_rungs = self.emitted_rungs.saturating_add(1);
            }
            VelocitySweepEvent::Recovery(_) => {}
            VelocitySweepEvent::DirectionalRegion(report) => {
                let direction = match report.region.direction {
                    crate::autotune::velocity_measurement::VelocityDirection::Forward => 0,
                    crate::autotune::velocity_measurement::VelocityDirection::Reverse => 1,
                };
                match report.region.kind {
                    super::operating_region::DirectionalRegionKind::Valid => {
                        self.directional_region_counts[direction] =
                            self.directional_region_counts[direction].saturating_add(1);
                    }
                    super::operating_region::DirectionalRegionKind::Fragment => {
                        self.directional_fragment_counts[direction] =
                            self.directional_fragment_counts[direction].saturating_add(1);
                    }
                }
            }
            VelocitySweepEvent::JointRegion(_) | VelocitySweepEvent::Handoff(_) => {}
        }
        self.digest =
            extend_velocity_sweep_digest(self.digest, &encode_velocity_sweep_event(event));
        if let VelocitySweepEvent::Observation(report) = event
            && let Some(record) = encode_velocity_sweep_structured_boundary(report)
        {
            self.digest = extend_velocity_sweep_digest(self.digest, &record);
        }
        self.next_evidence_sequence = self.next_evidence_sequence.saturating_add(1);
        event
    }

    /// Number of observation groups emitted so far.
    pub(crate) const fn emitted_observations(self) -> u16 {
        self.emitted_observations
    }

    /// Number of rung groups emitted so far.
    pub(crate) const fn emitted_rungs(self) -> u16 {
        self.emitted_rungs
    }

    /// Number of valid directional regions emitted for forward and reverse.
    pub(crate) const fn directional_region_counts(self) -> [u8; 2] {
        self.directional_region_counts
    }

    /// Number of directional fragments emitted for forward and reverse.
    pub(crate) const fn directional_fragment_counts(self) -> [u8; 2] {
        self.directional_fragment_counts
    }

    /// Current exact FNV-1a digest.
    pub(crate) const fn digest(self) -> u64 {
        self.digest
    }
}

/// Encode one nonterminal sweep event in the exact canonical digest representation.
pub(crate) fn encode_velocity_sweep_event(
    event: VelocitySweepEvent,
) -> VelocitySweepCanonicalRecord {
    match event {
        VelocitySweepEvent::Planned(report) => encode_plan(report),
        VelocitySweepEvent::Observation(report) => encode_observation(report),
        VelocitySweepEvent::Rung(report) => encode_rung(report),
        VelocitySweepEvent::Recovery(report) => encode_rung_origin_recovery(report),
        VelocitySweepEvent::DirectionalRegion(report) => {
            let mut record = VelocitySweepCanonicalRecord::new();
            record.push_u8(4);
            record.push_u32(report.run_sequence);
            record.push_u16(report.evidence_sequence);
            record.push_u32(report.region.member_mask);
            record.push_u8(super::sweep_evidence::direction_report_code(
                report.region.direction,
            ));
            record.push_u8(super::sweep_evidence::directional_region_kind_code(
                report.region.kind,
            ));
            record.push_u8(super::sweep_evidence::directional_region_closure_code(
                report.region.closure,
            ));
            record.push_u8(report.region.first_rung);
            record.push_u8(report.region.last_rung);
            record.push_u16(report.region.p_low.raw());
            record.push_u16(report.region.p_high.raw());
            record.push_u8(report.region.member_count);
            record.push_i32(report.region.common_intersection.low().raw());
            record.push_i32(report.region.common_intersection.high().raw());
            record.push_i32(report.region.pooled.estimate.raw());
            record.push_i32(report.region.pooled.interval.low().raw());
            record.push_i32(report.region.pooled.interval.high().raw());
            record.push_u8(report.region.pooled.variance_floor_observations);
            for value in [
                report.region.settled_rate_mean_min_q,
                report.region.settled_rate_mean_max_q,
                report.region.settled_rate_envelope_low_q,
                report.region.settled_rate_envelope_high_q,
            ] {
                let (mantissa, shift) = upper_bound_i64(value);
                record.push_i32(mantissa);
                record.push_u8(shift);
            }
            record.push_u8(report.region.boundary_rung.map_or(0, |rung| rung + 1));
            record.push_i32(report.region.tested_intersection.low().raw());
            record.push_i32(report.region.tested_intersection.high().raw());
            record
        }
        VelocitySweepEvent::JointRegion(report) => {
            let mut record = VelocitySweepCanonicalRecord::new();
            record.push_u8(5);
            record.push_u32(report.run_sequence);
            record.push_u16(report.evidence_sequence);
            record.push_u32(report.region.member_mask);
            record.push_u8(report.region.first_rung);
            record.push_u8(report.region.last_rung);
            record.push_u8(report.region.member_count);
            record.push_u16(report.p_low);
            record.push_u16(report.p_high);
            record.push_u8(super::sweep_evidence::joint_operable_closure_code(
                report.region.closure,
            ));
            record
        }
        VelocitySweepEvent::Handoff(report) => {
            let mut record = VelocitySweepCanonicalRecord::new();
            record.push_u8(6);
            record.push_u32(report.run_sequence);
            record.push_u16(report.evidence_sequence);
            record.push_u16(report.handoff.nomination.p.raw());
            record.push_u8(report.handoff.coverage_mask);
            record.push_u32(report.handoff.nomination.joint.member_mask);
            record.push_u8(report.handoff.nomination.rung_index);
            record.push_u32(report.handoff.nomination.distance_to_start_num);
            record.push_u32(report.handoff.nomination.distance_to_top_num);
            record.push_u8(u8::from(report.handoff.nomination.reduced_margin));
            for selected in report.handoff.directional {
                record.push_u32(selected.region.member_mask);
                record.push_i32(selected.region.pooled_interval[0]);
                record.push_i32(selected.region.pooled_interval[1]);
                record.push_u8(super::sweep_evidence::coverage_kind_code(selected.coverage));
                record.push_u8(selected.signed_rung_distance as u8);
                record.push_u16(selected.gain_ratio_num);
                record.push_u16(selected.gain_ratio_den);
                let (mantissa, shift) = upper_bound_i64(selected.settled_rate_difference_q);
                record.push_i32(mantissa);
                record.push_u8(shift);
            }
            record
        }
    }
}

/// Encode one stage-neutral rung-origin recovery in exact canonical wire order.
pub(crate) fn encode_rung_origin_recovery(
    evidence: RungOriginRecoveryEvidence,
) -> VelocitySweepCanonicalRecord {
    let report = evidence.report;
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(9);
    record.push_u32(evidence.run_sequence);
    record.push_u16(evidence.evidence_sequence);
    record.push_u8(report.stage as u8);
    record.push_u8(report.rung_index);
    record.push_u16(report.p_raw);
    record.push_u32(report.planned_velocity_mrev_s);
    record.push_i32(report.start_offset_counts);
    record.push_i32(report.closest_offset_counts);
    record.push_i32(report.final_offset_counts);
    record.push_u32(report.origin_band_counts);
    push_u64(&mut record, report.lower_rate_q.to_u64());
    record.push_u32(report.moving_duration_us);
    record.push_u32(report.settle_duration_us);
    record.push_u32(report.total_duration_us);
    record.push_u16(report.peak_torque_target_abs);
    record.push_u8(report.binding_source as u8);
    record.push_u8(report.outcome as u8);
    record
}

fn push_u64(record: &mut VelocitySweepCanonicalRecord, value: u64) {
    record.push_u32(value as u32);
    record.push_u32((value >> 32) as u32);
}

/// Encode an observation's optional structured-boundary evidence.
pub(crate) fn encode_velocity_sweep_structured_boundary(
    report: VelocitySweepObservation,
) -> Option<VelocitySweepCanonicalRecord> {
    let boundary = report.structured_boundary?;
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(7);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u8(report.rung_index);
    record.push_u8(report.slot);
    record.push_u8(boundary.passing_suffix_mask);
    for value in boundary.common_rate_interval_q {
        let (mantissa, shift) = upper_bound_i64(value);
        record.push_i32(mantissa);
        record.push_u8(shift);
    }
    let (mantissa, shift) = upper_bound_i64(boundary.binding_slope_margin_q);
    record.push_i32(mantissa);
    record.push_u8(shift);
    record.push_i32(boundary.binding_lag_one_margin_q);
    Some(record)
}

fn encode_plan(report: VelocitySweepPlanned) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(1);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u32(report.plan_digest as u32);
    record.push_u32((report.plan_digest >> 32) as u32);
    record.push_u32(report.requested_velocity_mrev_s);
    record.push_u8(report.requested_velocity_source);
    record.push_u32(report.planned_velocity_mrev_s);
    record.push_u32(report.effective_ceiling_mrev_s);
    record.push_u16(report.clamp_flags);
    record.push_u8(report.effective_ceiling_source);
    record.push_i32(report.target_velocity_rpm);
    record.push_u16(report.p_start);
    record.push_u16(report.p_top);
    record.push_u8(report.rung_count);
    record.push_u16(report.observations_per_direction);
    record.push_u32(report.moving_stroke_us);
    record.push_u32(report.zero_settle_us);
    record.push_u32(report.nominal_workflow_ms);
    record.push_u32(report.maximum_workflow_ms);
    record.push_u32(report.recovery_origin_band_counts);
    record.push_u32(report.recovery_nominal_slot_us);
    record.push_u32(report.recovery_maximum_slot_us);
    record.push_u8(report.recovery_slot_count);
    record.push_u32(report.max_stroke_travel_mrev);
    record.push_u32(report.settle_travel_reserve_mrev);
    record.push_u32(report.negative_position_headroom_mrev);
    record.push_u32(report.positive_position_headroom_mrev);
    record.push_u16(report.hard_torque_limit);
    record.push_u16(report.usable_torque_limit);
    record
}

fn encode_observation(report: VelocitySweepObservation) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(2);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u8(report.rung_index);
    record.push_u8(report.slot);
    record.push_i32(report.target_velocity_rpm);
    record.push_u8(observation_class_report_code(report.summary.classification));
    record.push_u8(u8::from(report.summary.initial_lead_in));
    record.push_u8(u8::from(report.selected_delta_counts < 0));
    let (delta_mantissa, delta_shift) =
        upper_bound_u128(report.selected_delta_counts.unsigned_abs() as u128);
    record.push_u32(delta_mantissa);
    record.push_u8(delta_shift);
    let (elapsed_mantissa, elapsed_shift) =
        upper_bound_u128(u128::from(report.selected_elapsed_us));
    record.push_u32(elapsed_mantissa);
    record.push_u8(elapsed_shift);
    let rate_values = [
        report.summary.settled_rate_interval_q[0],
        report.settled_rate_mean_q,
        report.summary.settled_rate_interval_q[1],
        report.summary.deficit_interval_q[0],
        report.summary.deficit_interval_q[1],
    ];
    let (rate_values, rate_shift) = encode_rate_values(rate_values);
    for value in rate_values {
        record.push_i32(value);
    }
    record.push_u8(rate_shift);
    let suffix_len = match report.stationarity {
        StationarityResult::Usable(suffix) => suffix.suffix_len,
        StationarityResult::Excluded(_) => 0,
    };
    record.push_u16(suffix_len);
    record.push_u8(report.selected_level);
    record.push_u32(report.selected_blocks);
    let (variance_mantissa, variance_shift) = upper_bound_u128(report.variance_of_mean_q2);
    record.push_u32(variance_mantissa);
    record.push_u8(variance_shift);
    let (
        slope_mantissa,
        slope_shift,
        slope_half_width_mantissa,
        slope_half_width_shift,
        lag_one_q,
        lag_one_half_width_q,
        residual_mantissa,
        residual_shift,
        tested_suffixes,
    ) = match report.stationarity {
        StationarityResult::Usable(suffix) => {
            let (slope_mantissa, slope_shift) = upper_bound_i64(suffix.slope_q);
            let (slope_half_width_mantissa, slope_half_width_shift) =
                upper_bound_u128(u128::from(suffix.slope_half_width_q));
            let (residual_mantissa, residual_shift) =
                upper_bound_u128(suffix.residual_magnitude_q2);
            (
                slope_mantissa,
                slope_shift,
                slope_half_width_mantissa,
                slope_half_width_shift,
                suffix.lag_one_q,
                suffix.lag_one_half_width_q,
                residual_mantissa,
                residual_shift,
                suffix.tested_suffixes,
            )
        }
        StationarityResult::Excluded(_) => (0, 0, 0, 0, 0, 0, 0, 0, 0),
    };
    record.push_i32(slope_mantissa);
    record.push_u8(slope_shift);
    record.push_u32(slope_half_width_mantissa);
    record.push_u8(slope_half_width_shift);
    record.push_i32(lag_one_q);
    record.push_u32(lag_one_half_width_q);
    record.push_u32(residual_mantissa);
    record.push_u8(residual_shift);
    record.push_u8(tested_suffixes);
    record.push_u16(report.summary.p.raw());
    let disturbance = report.summary.disturbance.map_or(0, |value| value.raw());
    let (disturbance_low_q16, disturbance_high_q16) = report
        .summary
        .disturbance_interval
        .map_or((0, 0), |interval| {
            (interval.low().raw(), interval.high().raw())
        });
    record.push_i32(disturbance);
    record.push_i32(disturbance_low_q16);
    record.push_i32(disturbance_high_q16);
    let (disturbance_variance_mantissa, disturbance_variance_shift) =
        upper_bound_u128(u128::from(report.summary.disturbance_variance_q2));
    record.push_u32(disturbance_variance_mantissa);
    record.push_u8(disturbance_variance_shift);
    record.push_u16(report.summary.predicted_torque_target_abs);
    record
}

fn encode_rung(report: VelocitySweepRungReport) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(3);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u8(report.rung_index);
    record.push_u16(report.forward.p.raw());
    for directional in [report.forward, report.reverse] {
        record.push_u8(consensus_class_report_code(directional.classification));
        record.push_u8(directional.component_count());
        record.push_u8(directional.collected_mask);
        record.push_u8(directional.eligible_mask);
        record.push_u8(directional.included_mask);
        record.push_u8(directional.operable_count);
        record.push_u8(directional.flags);
        for component in directional.components {
            record.push_i32(component[0]);
            record.push_i32(component[1]);
        }
        let (point, low, high, variance_floor) =
            directional.pooled.map_or((0, 0, 0, 0), |pooled| {
                (
                    pooled.estimate.raw(),
                    pooled.interval.low().raw(),
                    pooled.interval.high().raw(),
                    pooled.variance_floor_observations,
                )
            });
        record.push_i32(point);
        record.push_i32(low);
        record.push_i32(high);
        record.push_u8(variance_floor);
        let (rate_mantissas, rate_shift) = common_rate_mantissas(directional.settled_rate_bounds_q);
        for mantissa in rate_mantissas {
            record.push_i32(mantissa);
            record.push_u8(rate_shift);
        }
    }
    assert!(record.as_bytes().len() <= 128);
    record
}

/// Encode the detailed terminal in a trace-only canonical record.
pub(crate) fn encode_velocity_sweep_terminal(
    report: VelocitySweepTerminal,
    encoder_counts_per_rev: u32,
    f_pwm_hz: u32,
) -> VelocitySweepCanonicalRecord {
    use crate::commissioning::types::VelocitySweepOutcome;

    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(8);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u32(encoder_counts_per_rev);
    record.push_u32(f_pwm_hz);
    record.push_u8(report.model_direction_mask);
    for membership in report.selected_memberships {
        record.push_u32(membership);
    }
    record.push_u16(report.nominated_p);
    for interval in report.selected_intervals {
        record.push_i32(interval[0]);
        record.push_i32(interval[1]);
    }
    record.push_u8(report.coverage_mask);
    for count in report.directional_region_counts {
        record.push_u8(count);
    }
    for count in report.directional_fragment_counts {
        record.push_u8(count);
    }
    record.push_u16(report.expected_observations);
    record.push_u16(report.emitted_observations);
    record.push_u16(report.expected_rungs);
    record.push_u16(report.emitted_rungs);
    record.push_u32(report.digest as u32);
    record.push_u32((report.digest >> 32) as u32);
    record.push_u8(match report.outcome {
        VelocitySweepOutcome::CompleteCandidate => 0,
        VelocitySweepOutcome::Complete => 1,
        VelocitySweepOutcome::Inconclusive => 2,
        VelocitySweepOutcome::Fault => 3,
        VelocitySweepOutcome::RejectedPlanMismatch => 4,
    });
    record.push_u8(report.cause);
    record.push_u8(u8::from(report.recovery_unavailable));
    record.push_u32(report.run_started_us as u32);
    record.push_u32((report.run_started_us >> 32) as u32);
    record.push_u32(report.run_completed_us as u32);
    record.push_u32((report.run_completed_us >> 32) as u32);
    record
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::velocity_measurement::{
        StationarityExclusion, StationarityResult, VelocityDirection,
    };
    use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
    use crate::commissioning::outer::velocity::operating_region::{
        BoundRounding, CompactSignedRate,
    };
    use crate::commissioning::outer::velocity::recovery::{
        ExactU64Parts, RecoveryBindingSource, RecoveryStage, RungOriginRecoveryOutcome,
        RungOriginRecoveryReport,
    };
    use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
    use crate::commissioning::outer::velocity::sweep_evidence::{
        CompactDirectionalRungReport, DIRECTIONAL_RUNG_FLAG_INITIAL_LEAD_IN,
        DIRECTIONAL_RUNG_FLAG_OPERABLE, PooledDisturbance, VelocityObservationClass,
        VelocityObservationSummary,
    };
    use crate::commissioning::types::RungOriginRecoveryEvidence;
    use foci_tmc4671::pid::PidGain;

    fn fixture_plan() -> VelocitySweepPlanned {
        VelocitySweepPlanned {
            plan_digest: 0,
            run_sequence: 7,
            evidence_sequence: 0,
            requested_velocity_mrev_s: 5_000,
            requested_velocity_source: 1,
            planned_velocity_mrev_s: 4_800,
            effective_ceiling_mrev_s: 7_500,
            clamp_flags: 1,
            effective_ceiling_source: 0,
            target_velocity_rpm: 3_200,
            p_start: 8,
            p_top: 128,
            rung_count: 5,
            observations_per_direction: 2,
            moving_stroke_us: 256_000,
            zero_settle_us: 200_000,
            nominal_workflow_ms: 9_120,
            maximum_workflow_ms: 10_000,
            recovery_origin_band_counts: 1_000,
            recovery_nominal_slot_us: 1_816_958,
            recovery_maximum_slot_us: 3_000_000,
            recovery_slot_count: 5,
            max_stroke_travel_mrev: 1_000,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 1_250,
            positive_position_headroom_mrev: 1_250,
            hard_torque_limit: 1_000,
            usable_torque_limit: 900,
        }
    }

    fn fixture_recovery() -> RungOriginRecoveryEvidence {
        RungOriginRecoveryEvidence {
            run_sequence: 0x1234_5678,
            evidence_sequence: 0x9abc,
            report: RungOriginRecoveryReport {
                stage: RecoveryStage::Proportional,
                rung_index: u8::MAX,
                p_raw: u16::MAX,
                planned_velocity_mrev_s: u32::MAX,
                start_offset_counts: i32::MIN,
                closest_offset_counts: i32::MAX,
                final_offset_counts: -1,
                origin_band_counts: u32::MAX,
                lower_rate_q: ExactU64Parts::from_u64(0xfedc_ba98_7654_3210),
                moving_duration_us: u32::MAX,
                settle_duration_us: u32::MAX,
                total_duration_us: u32::MAX,
                peak_torque_target_abs: u16::MAX,
                binding_source: RecoveryBindingSource::SettledRate,
                outcome: RungOriginRecoveryOutcome::Recovered,
            },
        }
    }

    #[test]
    fn rung_origin_recovery_canonical_record_is_exact_and_count_neutral() {
        let recovery = fixture_recovery();
        let record = encode_rung_origin_recovery(recovery);
        let mut integrity = VelocitySweepIntegrity::new(recovery.run_sequence);

        integrity.record(VelocitySweepEvent::Recovery(recovery));

        assert_eq!(record.as_bytes().len(), 55);
        assert_eq!(
            &record.as_bytes()[1..5],
            &recovery.run_sequence.to_le_bytes()
        );
        assert_eq!(
            &record.as_bytes()[5..7],
            &recovery.evidence_sequence.to_le_bytes()
        );
        assert!(
            record
                .as_bytes()
                .windows(8)
                .any(|word| { word == 0xfedc_ba98_7654_3210_u64.to_le_bytes() })
        );
        assert_eq!(integrity.emitted_observations(), 0);
        assert_eq!(integrity.emitted_rungs(), 0);
        assert_eq!(integrity.next_evidence_sequence(), 1);
    }

    #[test]
    fn structured_recovery_source_round_trips_without_payload_growth() {
        let mut recovery = fixture_recovery();
        recovery.report.binding_source = RecoveryBindingSource::StructuredSuffix;
        let record = encode_rung_origin_recovery(recovery);

        assert_eq!(record.as_bytes().len(), 55);
        assert_eq!(
            record.as_bytes()[53],
            RecoveryBindingSource::StructuredSuffix as u8
        );
    }

    fn fixture_region(
        direction: VelocityDirection,
        kind: super::super::operating_region::DirectionalRegionKind,
    ) -> crate::commissioning::outer::velocity::operating_region::DirectionalRegionSummary {
        use crate::commissioning::outer::velocity::operating_region::{
            DirectionalRegionClosure, DirectionalRegionSummary,
        };
        let interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(100),
            EquivalentTorqueTargetQ16::from_raw(110),
        )
        .unwrap();
        DirectionalRegionSummary {
            direction,
            member_mask: 0b111,
            first_rung: 0,
            last_rung: 2,
            p_low: PidGain::new(256).unwrap(),
            p_high: PidGain::new(512).unwrap(),
            member_count: 3,
            kind,
            common_intersection: interval,
            pooled: PooledDisturbance {
                estimate: EquivalentTorqueTargetQ16::from_raw(105),
                interval,
                variance_floor_observations: 0,
            },
            settled_rate_mean_min_q: 1_000,
            settled_rate_mean_max_q: 1_100,
            settled_rate_envelope_low_q: 900,
            settled_rate_envelope_high_q: 1_200,
            closure: DirectionalRegionClosure::EndOfLadder,
            boundary_rung: None,
            tested_intersection: interval,
        }
    }

    fn maximal_directional_rung(p: PidGain) -> CompactDirectionalRungReport {
        let pooled_interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(i32::MIN + 1),
            EquivalentTorqueTargetQ16::from_raw(i32::MAX),
        )
        .unwrap();
        CompactDirectionalRungReport {
            p,
            classification: DirectionalConsensusClass::EligibleConsensus,
            components: [[i32::MIN + 1, i32::MAX], [0, 0]],
            pooled: Some(PooledDisturbance {
                estimate: EquivalentTorqueTargetQ16::from_raw(0),
                interval: pooled_interval,
                variance_floor_observations: 4,
            }),
            settled_rate_bounds_q: [
                CompactSignedRate::encode(i64::MIN / 2, BoundRounding::TowardNegativeInfinity),
                CompactSignedRate::encode(i64::MAX / 2, BoundRounding::TowardPositiveInfinity),
                CompactSignedRate::encode(i64::MIN / 3, BoundRounding::TowardNegativeInfinity),
                CompactSignedRate::encode(i64::MAX / 3, BoundRounding::TowardPositiveInfinity),
            ],
            collected_mask: 0b1111,
            eligible_mask: 0b1111,
            included_mask: 0b1111,
            operable_count: 4,
            flags: DIRECTIONAL_RUNG_FLAG_OPERABLE | DIRECTIONAL_RUNG_FLAG_INITIAL_LEAD_IN,
        }
    }

    #[test]
    fn velocity_sweep_reporting_maximal_consensus_rung_fits_audited_capacity() {
        let p = PidGain::new(PidGain::MAX.raw()).unwrap();
        let directional = maximal_directional_rung(p);
        let record = encode_rung(VelocitySweepRungReport {
            run_sequence: u32::MAX,
            evidence_sequence: u16::MAX,
            rung_index: u8::MAX,
            forward: directional,
            reverse: directional,
        });

        assert!(record.as_bytes().len() <= 128);
        assert_eq!(record.as_bytes().len(), 122);
    }

    #[test]
    fn velocity_sweep_reporting_uses_the_wire_common_rate_shift() {
        let p = PidGain::new(256).unwrap();
        let mut directional = maximal_directional_rung(p);
        directional.settled_rate_bounds_q = [
            CompactSignedRate::encode(-10, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(40_000, BoundRounding::TowardPositiveInfinity),
            CompactSignedRate::encode(-90_000, BoundRounding::TowardNegativeInfinity),
            CompactSignedRate::encode(120, BoundRounding::TowardPositiveInfinity),
        ];
        let expected_shift = directional
            .settled_rate_bounds_q
            .iter()
            .map(|rate| rate.shift)
            .max()
            .unwrap();

        let record = encode_rung(VelocitySweepRungReport {
            run_sequence: 7,
            evidence_sequence: 9,
            rung_index: 0,
            forward: directional,
            reverse: directional,
        });
        let bytes = record.as_bytes();

        assert_eq!(
            [bytes[50], bytes[55], bytes[60], bytes[65]],
            [expected_shift; 4]
        );
    }

    #[test]
    fn velocity_sweep_reporting_plan_digest_matches_host_wire_fixture() {
        let mut integrity = VelocitySweepIntegrity::new(7);
        integrity.record(VelocitySweepEvent::Planned(fixture_plan()));

        assert_eq!(integrity.digest(), 0x5df5_d108_3d3e_407d);
    }

    #[test]
    fn velocity_sweep_reporting_compressed_observation_digest_matches_host_wire_fixture() {
        let mut integrity = VelocitySweepIntegrity::new(7);
        integrity.record(VelocitySweepEvent::Planned(fixture_plan()));
        let disturbance = EquivalentTorqueTargetQ16::from_raw(123_456);
        let disturbance_interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(120_000),
            EquivalentTorqueTargetQ16::from_raw(130_000),
        )
        .unwrap();
        integrity.record(VelocitySweepEvent::Observation(VelocitySweepObservation {
            run_sequence: 7,
            evidence_sequence: 1,
            rung_index: 0,
            slot: 0,
            target_velocity_rpm: 3_200,
            summary: VelocityObservationSummary {
                direction: VelocityDirection::Forward,
                p: PidGain::new(350).unwrap(),
                classification: VelocityObservationClass::Eligible,
                settled_rate_interval_q: [-5_000_000_001, 5_000_000_001],
                settled_rate_mean_q: 1,
                deficit_interval_q: [-6_000_000_001, 6_000_000_001],
                disturbance: Some(disturbance),
                disturbance_interval: Some(disturbance_interval),
                disturbance_variance_q2: 1 << 40,
                predicted_torque_target_abs: 500,
                initial_lead_in: true,
            },
            stationarity: StationarityResult::Excluded(StationarityExclusion::NoBlockingPlateau),
            selected_delta_counts: 4_294_967_297,
            selected_elapsed_us: 5_000_000_001,
            selected_level: 3,
            selected_blocks: 16,
            variance_of_mean_q2: 1 << 80,
            settled_rate_mean_q: 1,
            structured_boundary: None,
        }));

        assert_eq!(integrity.digest(), 0xd22a_c49c_f269_f830);
    }

    #[test]
    fn velocity_sweep_reporting_digest_extension_matches_integrity() {
        let plan = fixture_plan();
        let record = encode_velocity_sweep_event(VelocitySweepEvent::Planned(plan));
        let extended = extend_velocity_sweep_digest(FNV1A64_OFFSET, &record);
        let mut integrity = VelocitySweepIntegrity::new(plan.run_sequence);

        integrity.record(VelocitySweepEvent::Planned(plan));

        assert_eq!(extended, integrity.digest());
    }

    #[test]
    fn velocity_sweep_reporting_counts_directional_regions_and_fragments() {
        use crate::commissioning::outer::velocity::operating_region::DirectionalRegionKind;
        use crate::commissioning::types::VelocityDirectionalRegionReport;
        let mut integrity = VelocitySweepIntegrity::new(7);
        integrity.record(VelocitySweepEvent::DirectionalRegion(
            VelocityDirectionalRegionReport {
                run_sequence: 7,
                evidence_sequence: 0,
                region: fixture_region(VelocityDirection::Forward, DirectionalRegionKind::Valid),
            },
        ));
        integrity.record(VelocitySweepEvent::DirectionalRegion(
            VelocityDirectionalRegionReport {
                run_sequence: 7,
                evidence_sequence: 1,
                region: fixture_region(VelocityDirection::Reverse, DirectionalRegionKind::Fragment),
            },
        ));

        assert_eq!(integrity.directional_region_counts(), [1, 0]);
        assert_eq!(integrity.directional_fragment_counts(), [0, 1]);
    }
}
