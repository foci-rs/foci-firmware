//! Streamed velocity-sweep evidence sequencing and integrity.

use crate::autotune::velocity_measurement::{
    RestSelectionDisposition, RestStructuralCode, StationarityResult, VELOCITY_SUFFIX_LENGTHS,
};
use crate::commissioning::types::{
    RecoveryRestDiagnostic, RestSelectionDiagnostic, RungOriginRecoveryEvidence,
    VelocitySweepEvent, VelocitySweepObservation, VelocitySweepPlanned, VelocitySweepRungReport,
    VelocitySweepTerminal,
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
            VelocitySweepEvent::Recovery(_)
            | VelocitySweepEvent::RecoveryRest(_)
            | VelocitySweepEvent::RestSelection(_)
            | VelocitySweepEvent::Current(_) => {}
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
        VelocitySweepEvent::RecoveryRest(report) => encode_recovery_rest(report),
        VelocitySweepEvent::RestSelection(report) => {
            encode_rest_selection(report).expect("completed scored-rest diagnostic is coherent")
        }
        VelocitySweepEvent::Current(report) => {
            crate::ankyra::velocity_sweep_encoding::encode_velocity_primitive_current_evidence(
                report,
            )
        }
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
            record.push_u8(report.model_direction_mask);
            record.push_u32(report.joint_union);
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

/// Encode recovery lead-in and scored-rest evidence in exact canonical order.
pub(crate) fn encode_recovery_rest(report: RecoveryRestDiagnostic) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(10);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u8(report.stage as u8);
    record.push_u8(report.rung_index);
    record.push_u16(report.p_raw);
    push_u64(&mut record, report.plan_digest);
    record.push_i32(report.start_offset_counts);
    record.push_i32(report.closest_offset_counts);
    record.push_i32(report.final_offset_counts);
    record.push_u32(report.planned_return_distance_counts);
    record.push_i32(report.first_in_band_offset_counts);
    record.push_i32(report.displacement_to_band_counts);
    record.push_u32(report.entry_to_band_duration_us);
    record.push_i32(report.lead_in_net_displacement_counts);
    record.push_u32(report.lead_in_displacement_span_counts);
    record.push_i32(report.scored_net_displacement_counts);
    record.push_i32(report.scored_min_displacement_counts);
    record.push_i32(report.scored_max_displacement_counts);
    record.push_u8(report.stationarity_code);
    record.push_u8(report.interval_present);
    push_u64(&mut record, report.exact_velocity_interval_q[0] as u64);
    push_u64(&mut record, report.exact_velocity_interval_q[1] as u64);
    record.push_u16(report.peak_torque_target_abs);
    record.push_u8(report.current_contact_code);
    record.push_u8(report.disposition);
    record
}

/// Integrity error in one completed scored-rest selector diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RestSelectionEncodingError {
    /// A mask names a suffix outside the declared family.
    MaskOutOfRange,
    /// Structural codes, masks, and interval presence disagree.
    StructuralMismatch,
    /// A present interval is reversed or an absent interval is nonzero.
    InvalidInterval,
    /// The zero-compatible mask disagrees with the exact closed intervals.
    ZeroCompatibilityMismatch,
    /// The selected suffix is not the end of the causal compatible prefix.
    InvalidSelection,
    /// Arithmetic cannot be serialized as a completed decision.
    ArithmeticResult,
}

/// Encode one completed scored-rest selector decision in exact canonical order.
pub(crate) fn encode_rest_selection(
    report: RestSelectionDiagnostic,
) -> Result<VelocitySweepCanonicalRecord, RestSelectionEncodingError> {
    validate_rest_selection(report)?;
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(11);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u8(report.stage as u8);
    record.push_u8(report.owner as u8);
    record.push_u8(report.rung_index);
    record.push_u8(report.direction);
    record.push_u8(report.slot);
    push_u64(&mut record, report.plan_digest);
    for code in report.structural_codes {
        record.push_u8(code as u8);
    }
    record.push_u8(report.interval_present_mask);
    record.push_u8(report.usable_mask);
    record.push_u8(report.zero_compatible_mask);
    record.push_u8(report.selected_mask);
    record.push_u16(report.selected_suffix_len);
    record.push_u8(report.disposition as u8);
    for interval in report.intervals_q {
        for endpoint in interval {
            push_u64(&mut record, endpoint as u64);
        }
    }
    Ok(record)
}

fn validate_rest_selection(
    report: RestSelectionDiagnostic,
) -> Result<(), RestSelectionEncodingError> {
    const VALID_MASK: u8 = 0b111;
    for mask in [
        report.interval_present_mask,
        report.usable_mask,
        report.zero_compatible_mask,
        report.selected_mask,
    ] {
        if mask & !VALID_MASK != 0 {
            return Err(RestSelectionEncodingError::MaskOutOfRange);
        }
    }

    let mut expected_interval_mask = 0_u8;
    let mut expected_usable_mask = 0_u8;
    let mut expected_zero_mask = 0_u8;
    for (index, code) in report.structural_codes.into_iter().enumerate() {
        let bit = 1_u8 << index;
        let interval_expected = match code {
            RestStructuralCode::Usable => {
                expected_usable_mask |= bit;
                true
            }
            RestStructuralCode::ResolvedSlope | RestStructuralCode::StructuredResidual => true,
            RestStructuralCode::NoBlockingPlateau
            | RestStructuralCode::TooFewDecorrelatedBlocks => false,
            RestStructuralCode::Arithmetic => {
                return Err(RestSelectionEncodingError::ArithmeticResult);
            }
        };
        if interval_expected {
            expected_interval_mask |= bit;
            let [low, high] = report.intervals_q[index];
            if low > high {
                return Err(RestSelectionEncodingError::InvalidInterval);
            }
            if code == RestStructuralCode::Usable && low <= 0 && 0 <= high {
                expected_zero_mask |= bit;
            }
        } else if report.intervals_q[index] != [0, 0] {
            return Err(RestSelectionEncodingError::InvalidInterval);
        }
    }
    if report.interval_present_mask != expected_interval_mask
        || report.usable_mask != expected_usable_mask
    {
        return Err(RestSelectionEncodingError::StructuralMismatch);
    }
    if report.zero_compatible_mask != expected_zero_mask {
        return Err(RestSelectionEncodingError::ZeroCompatibilityMismatch);
    }

    let mut expected_selected_mask = 0_u8;
    let mut expected_selected_len = 0_u16;
    for (index, suffix_len) in VELOCITY_SUFFIX_LENGTHS.into_iter().enumerate() {
        let bit = 1_u8 << index;
        if expected_zero_mask & bit == 0 {
            break;
        }
        expected_selected_mask = bit;
        expected_selected_len = suffix_len;
    }
    match report.disposition {
        RestSelectionDisposition::StatisticalAcceptance => {
            if expected_selected_mask == 0
                || report.selected_mask != expected_selected_mask
                || report.selected_suffix_len != expected_selected_len
            {
                return Err(RestSelectionEncodingError::InvalidSelection);
            }
        }
        RestSelectionDisposition::OneCountFallbackAcceptance
        | RestSelectionDisposition::Rejected => {
            if expected_selected_mask != 0
                || report.selected_mask != 0
                || report.selected_suffix_len != 0
            {
                return Err(RestSelectionEncodingError::InvalidSelection);
            }
        }
    }
    Ok(())
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
    record.push_u32(report.summary.drift_ratio_ppm);
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
        BoundRounding, CompactSignedRate, CoverageKind, DirectionalRegionKind,
        DirectionalRegionRef, JointOperableClosure, JointOperableRegion, ProvisionalPNomination,
        SelectedDirectionalRegion, StageBHandoff,
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
    use crate::commissioning::types::{
        RecoveryRestDiagnostic, RestSelectionDiagnostic, RestSelectionOwner,
        RungOriginRecoveryEvidence, VelocityPrimitiveCurrentEvidence,
    };
    use foci_tmc4671::pid::PidPGain;

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
    fn selected_recovery_rest_record_has_exact_sentinel_layout() {
        let report = RecoveryRestDiagnostic {
            plan_digest: 0x1122_3344_5566_7788,
            exact_velocity_interval_q: [-9, 11],
            run_sequence: 0x0102_0304,
            start_offset_counts: -1_834,
            closest_offset_counts: 998,
            final_offset_counts: 999,
            planned_return_distance_counts: 834,
            first_in_band_offset_counts: 998,
            displacement_to_band_counts: 2_832,
            entry_to_band_duration_us: 90_904,
            lead_in_net_displacement_counts: -3,
            lead_in_displacement_span_counts: 7,
            scored_net_displacement_counts: 0,
            scored_min_displacement_counts: -1,
            scored_max_displacement_counts: 1,
            p_raw: 1_024,
            evidence_sequence: 19,
            peak_torque_target_abs: 1_560,
            stage: RecoveryStage::Proportional,
            rung_index: 15,
            stationarity_code: 0,
            interval_present: 1,
            current_contact_code: 0,
            disposition: 0,
        };
        let mut integrity = VelocitySweepIntegrity::new(report.run_sequence);
        let record = encode_velocity_sweep_event(VelocitySweepEvent::RecoveryRest(report));

        integrity.record(VelocitySweepEvent::RecoveryRest(report));

        assert_eq!(record.as_bytes().len(), 89);
        assert_eq!(record.as_bytes()[0], 10);
        assert_eq!(&record.as_bytes()[1..5], &report.run_sequence.to_le_bytes());
        assert_eq!(
            &record.as_bytes()[11..19],
            &report.plan_digest.to_le_bytes()
        );
        assert_eq!(
            &record.as_bytes()[69..77],
            &report.exact_velocity_interval_q[0].to_le_bytes()
        );
        assert_eq!(
            &record.as_bytes()[77..85],
            &report.exact_velocity_interval_q[1].to_le_bytes()
        );
        assert_eq!(&record.as_bytes()[85..89], &[24, 6, 0, 0]);
        assert_eq!(integrity.emitted_observations(), 0);
        assert_eq!(integrity.emitted_rungs(), 0);
        assert_eq!(integrity.next_evidence_sequence(), 1);
    }

    fn rest_selection_diagnostic() -> RestSelectionDiagnostic {
        RestSelectionDiagnostic {
            intervals_q: [[-9, 0], [-17, 29], [31, 47]],
            plan_digest: 0x1122_3344_5566_7788,
            run_sequence: 0x0102_0304,
            evidence_sequence: 0x0506,
            selected_suffix_len: 128,
            structural_codes: [
                RestStructuralCode::Usable,
                RestStructuralCode::Usable,
                RestStructuralCode::ResolvedSlope,
            ],
            interval_present_mask: 0b111,
            usable_mask: 0b011,
            zero_compatible_mask: 0b011,
            selected_mask: 0b010,
            stage: RecoveryStage::Integral,
            owner: RestSelectionOwner::StageCPositiveObservation,
            rung_index: 7,
            direction: 1,
            slot: 0xff,
            disposition: RestSelectionDisposition::StatisticalAcceptance,
        }
    }

    #[test]
    fn rest_selection_record_has_exact_78_byte_layout() {
        let report = rest_selection_diagnostic();
        let record = encode_rest_selection(report).unwrap();
        let mut expected = std::vec![11];
        expected.extend_from_slice(&report.run_sequence.to_le_bytes());
        expected.extend_from_slice(&report.evidence_sequence.to_le_bytes());
        expected.extend_from_slice(&[1, 4, 7, 1, 0xff]);
        expected.extend_from_slice(&report.plan_digest.to_le_bytes());
        expected.extend_from_slice(&[0, 0, 3, 0b111, 0b011, 0b011, 0b010]);
        expected.extend_from_slice(&128_u16.to_le_bytes());
        expected.push(0);
        for interval in report.intervals_q {
            for endpoint in interval {
                expected.extend_from_slice(&endpoint.to_le_bytes());
            }
        }

        assert_eq!(record.as_bytes(), expected);
        assert_eq!(record.as_bytes().len(), 78);
    }

    #[test]
    fn rest_selection_codes_and_stage_encoders_are_shared() {
        assert_eq!(
            [
                RestSelectionOwner::StageBObservation as u8,
                RestSelectionOwner::StageBRecovery as u8,
                RestSelectionOwner::StageBSelectedRecovery as u8,
                RestSelectionOwner::StageCAnchor as u8,
                RestSelectionOwner::StageCPositiveObservation as u8,
                RestSelectionOwner::StageCRecovery as u8,
                RestSelectionOwner::StageCCleanup as u8,
                RestSelectionOwner::MatrixObservation as u8,
                RestSelectionOwner::MatrixRecovery as u8,
            ],
            [0, 1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(
            [
                RecoveryStage::Proportional as u8,
                RecoveryStage::Integral as u8
            ],
            [0, 1]
        );

        let report = rest_selection_diagnostic();
        let shared = encode_rest_selection(report).unwrap();
        let stage_c = super::super::integral_reporting::encode_velocity_integral_event(
            crate::commissioning::types::VelocityIntegralEvent::RestSelection(report),
        );
        let matrix = super::super::acceptance_reporting::encode_acceptance_event(
            crate::commissioning::types::AcceptanceMatrixEvent::RestSelection(report),
        );
        assert_eq!(stage_c.as_bytes(), shared.as_bytes());
        assert_eq!(matrix.as_bytes(), shared.as_bytes());
    }

    #[test]
    fn rest_selection_advances_one_sequence_without_counting_an_observation() {
        let report = rest_selection_diagnostic();
        let mut integrity = VelocitySweepIntegrity::new(report.run_sequence);
        let before = integrity.digest();

        integrity.record(VelocitySweepEvent::RestSelection(report));

        assert_eq!(integrity.next_evidence_sequence(), 1);
        assert_eq!(integrity.emitted_observations(), 0);
        assert_eq!(integrity.emitted_rungs(), 0);
        assert_ne!(integrity.digest(), before);
    }

    #[test]
    fn rest_selection_validation_rejects_incoherent_diagnostics() {
        let valid = rest_selection_diagnostic();
        assert!(encode_rest_selection(valid).is_ok());

        let mut interval_without_result = valid;
        interval_without_result.structural_codes[2] = RestStructuralCode::TooFewDecorrelatedBlocks;
        assert!(encode_rest_selection(interval_without_result).is_err());

        let mut selected_outside_prefix = valid;
        selected_outside_prefix.intervals_q[0] = [1, 9];
        selected_outside_prefix.zero_compatible_mask = 0b010;
        assert!(encode_rest_selection(selected_outside_prefix).is_err());

        let mut nonzero_unused_interval = valid;
        nonzero_unused_interval.interval_present_mask = 0b011;
        assert!(encode_rest_selection(nonzero_unused_interval).is_err());

        let mut completed_arithmetic = valid;
        completed_arithmetic.structural_codes[2] = RestStructuralCode::Arithmetic;
        assert!(encode_rest_selection(completed_arithmetic).is_err());
    }

    #[test]
    fn rest_selection_endpoint_touch_and_terminal_codes_are_exact() {
        let mut report = rest_selection_diagnostic();
        report.intervals_q[0] = [-9, 0];
        report.intervals_q[1] = [0, 11];
        assert!(encode_rest_selection(report).is_ok());

        report.selected_mask = 0;
        report.selected_suffix_len = 0;
        report.disposition = RestSelectionDisposition::OneCountFallbackAcceptance;
        report.zero_compatible_mask = 0;
        report.intervals_q[0] = [1, 9];
        report.intervals_q[1] = [1, 11];
        assert_eq!(encode_rest_selection(report).unwrap().as_bytes()[29], 1);

        report.disposition = RestSelectionDisposition::Rejected;
        assert_eq!(encode_rest_selection(report).unwrap().as_bytes()[29], 2);
    }

    #[test]
    fn primitive_current_evidence_is_hashed_and_count_neutral() {
        let mut integrity = VelocitySweepIntegrity::new(7);
        let before = integrity.digest();
        let evidence = VelocityPrimitiveCurrentEvidence {
            run_sequence: 7,
            evidence_sequence: 0,
            stage: 0,
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

        integrity.record(VelocitySweepEvent::Current(evidence));

        assert_ne!(integrity.digest(), before);
        assert_eq!(integrity.emitted_observations(), 0);
        assert_eq!(integrity.emitted_rungs(), 0);
        assert_eq!(integrity.next_evidence_sequence(), 1);
        assert_eq!(
            encode_velocity_sweep_event(VelocitySweepEvent::Current(evidence)).as_bytes()[0],
            31
        );
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
            p_low: PidPGain::new(256).unwrap(),
            p_high: PidPGain::new(512).unwrap(),
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

    fn maximal_directional_rung(p: PidPGain) -> CompactDirectionalRungReport {
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
        let p = PidPGain::new(PidPGain::MAX.raw()).unwrap();
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
        let p = PidPGain::new(256).unwrap();
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

    /// Byte offset of `drift_ratio_ppm` in an encoded Stage-B observation record.
    ///
    /// Counted from the fixed prefix `encode_observation` pushes ahead of the
    /// field: tag(1) + run_sequence(4) + evidence_sequence(2) + rung_index(1) +
    /// slot(1) + target_velocity_rpm(4) + class(1) + initial_lead_in(1) +
    /// delta_sign(1) + delta_mantissa(4) + delta_shift(1) + elapsed_mantissa(4) +
    /// elapsed_shift(1) + five encoded rates(20) + rate_shift(1) + suffix_len(2) +
    /// selected_level(1) + selected_blocks(4) + variance_mantissa(4) +
    /// variance_shift(1) + slope_mantissa(4) + slope_shift(1) +
    /// slope_half_width_mantissa(4) + slope_half_width_shift(1) + lag_one_q(4) +
    /// lag_one_half_width_q(4) + residual_mantissa(4) + residual_shift(1) +
    /// tested_suffixes(1) + p(2) + disturbance(4) + disturbance_low_q16(4) +
    /// disturbance_high_q16(4) + disturbance_variance_mantissa(4) +
    /// disturbance_variance_shift(1) + predicted_torque_target_abs(2) = 104.
    const SWEEP_DRIFT_RATIO_PPM_OFFSET: usize = 104;

    /// Read a little-endian `u32` out of an encoded record at a fixed offset.
    fn read_u32_at(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    fn fixture_observation() -> VelocitySweepObservation {
        let disturbance = EquivalentTorqueTargetQ16::from_raw(123_456);
        let disturbance_interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(120_000),
            EquivalentTorqueTargetQ16::from_raw(130_000),
        )
        .unwrap();
        VelocitySweepObservation {
            run_sequence: 7,
            evidence_sequence: 1,
            rung_index: 0,
            slot: 0,
            target_velocity_rpm: 3_200,
            summary: VelocityObservationSummary {
                direction: VelocityDirection::Forward,
                p: PidPGain::new(350).unwrap(),
                classification: VelocityObservationClass::Eligible,
                settled_rate_interval_q: [-5_000_000_001, 5_000_000_001],
                settled_rate_mean_q: 1,
                deficit_interval_q: [-6_000_000_001, 6_000_000_001],
                disturbance: Some(disturbance),
                disturbance_interval: Some(disturbance_interval),
                disturbance_variance_q2: 1 << 40,
                predicted_torque_target_abs: 500,
                drift_ratio_ppm: u32::MAX,
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
        }
    }

    #[test]
    fn velocity_sweep_reporting_compressed_observation_digest_matches_host_wire_fixture() {
        let mut integrity = VelocitySweepIntegrity::new(7);
        integrity.record(VelocitySweepEvent::Planned(fixture_plan()));
        integrity.record(VelocitySweepEvent::Observation(fixture_observation()));

        assert_eq!(integrity.digest(), 0x0b1d_04e6_d425_a08c);
    }

    #[test]
    fn every_sweep_observation_carries_the_drift_ratio() {
        // Stage B has no rejection diagnostic block at all, so a schema bump that
        // only touched that block would change nothing here while claiming to.
        let mut inert_report = fixture_observation();
        inert_report.summary.classification = VelocityObservationClass::Inert;
        inert_report.summary.drift_ratio_ppm = u32::MAX;
        let inert = encode_observation(inert_report);

        let mut eligible_report = fixture_observation();
        eligible_report.summary.classification = VelocityObservationClass::Eligible;
        eligible_report.summary.drift_ratio_ppm = 14_220;
        let eligible = encode_observation(eligible_report);

        let offset = SWEEP_DRIFT_RATIO_PPM_OFFSET;
        assert_eq!(read_u32_at(inert.as_bytes(), offset), u32::MAX);
        assert_eq!(read_u32_at(eligible.as_bytes(), offset), 14_220);
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

    #[test]
    fn schema8_handoff_distinguishes_joint_union_and_selected_component() {
        let region = fixture_region(VelocityDirection::Forward, DirectionalRegionKind::Valid);
        let reference = DirectionalRegionRef::from_summary(region, 2).unwrap();
        let selected = SelectedDirectionalRegion {
            region: reference,
            coverage: CoverageKind::FullCoverage,
            signed_rung_distance: 0,
            gain_ratio_num: 1,
            gain_ratio_den: 1,
            settled_rate_difference_q: 0,
        };
        let event =
            VelocitySweepEvent::Handoff(crate::commissioning::types::VelocityStageBHandoffReport {
                run_sequence: 7,
                evidence_sequence: 9,
                joint_union: 0b11_0011,
                model_direction_mask: 0b01,
                handoff: StageBHandoff {
                    nomination: ProvisionalPNomination {
                        joint: JointOperableRegion {
                            member_mask: 0b11_0000,
                            first_rung: 4,
                            last_rung: 5,
                            member_count: 2,
                            closure: JointOperableClosure::EndOfLadder,
                        },
                        rung_index: 4,
                        p: PidPGain::new(1_024).unwrap(),
                        distance_to_start_num: 10,
                        distance_to_top_num: 20,
                        reduced_margin: true,
                    },
                    directional: [selected; 2],
                    coverage_mask: 0b11,
                },
            });

        let record = encode_velocity_sweep_event(event);
        assert_eq!(&record.as_bytes()[11..15], &0b11_0011_u32.to_le_bytes());
        assert_eq!(&record.as_bytes()[15..19], &0b11_0000_u32.to_le_bytes());
        assert_eq!(record.as_bytes()[10], 0b01);
    }
}
