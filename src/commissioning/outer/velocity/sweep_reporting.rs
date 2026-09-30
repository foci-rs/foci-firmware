//! Streamed velocity-sweep evidence sequencing and integrity.

use crate::autotune::velocity_measurement::{
    RestSelectionDisposition, RestStructuralCode, RestSuffixEvaluation, select_causal_rest_suffixes,
};
use crate::commissioning::types::{
    RecoveryRestDiagnostic, RestSelectionDiagnostic, RungOriginRecoveryEvidence,
};

use crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord;

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
    record.push_u16(report.retained_sample_count);
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

    let mut suffixes = [RestSuffixEvaluation {
        structural_code: RestStructuralCode::TooFewDecorrelatedBlocks,
        interval_present: false,
        interval_q: [0; 2],
    }; 3];
    for (index, code) in report.structural_codes.into_iter().enumerate() {
        let interval_expected = match code {
            RestStructuralCode::Usable => true,
            RestStructuralCode::ResolvedSlope | RestStructuralCode::StructuredResidual => true,
            RestStructuralCode::NoBlockingPlateau
            | RestStructuralCode::TooFewDecorrelatedBlocks
            | RestStructuralCode::InsufficientHistory => false,
            RestStructuralCode::Arithmetic => {
                return Err(RestSelectionEncodingError::ArithmeticResult);
            }
        };
        if interval_expected {
            let [low, high] = report.intervals_q[index];
            if low > high {
                return Err(RestSelectionEncodingError::InvalidInterval);
            }
        } else if report.intervals_q[index] != [0, 0] {
            return Err(RestSelectionEncodingError::InvalidInterval);
        }
        suffixes[index] = RestSuffixEvaluation {
            structural_code: code,
            interval_present: interval_expected,
            interval_q: report.intervals_q[index],
        };
    }
    let expected = select_causal_rest_suffixes(
        suffixes,
        report.disposition == RestSelectionDisposition::OneCountFallbackAcceptance,
        report.retained_sample_count,
    );
    if report.structural_codes != expected.structural_codes
        || report.intervals_q != expected.intervals_q
        || report.interval_present_mask != expected.interval_present_mask
        || report.usable_mask != expected.usable_mask
    {
        return Err(RestSelectionEncodingError::StructuralMismatch);
    }
    if report.zero_compatible_mask != expected.zero_compatible_mask {
        return Err(RestSelectionEncodingError::ZeroCompatibilityMismatch);
    }
    if report.selected_mask != expected.selected_mask
        || report.selected_suffix_len != expected.selected_suffix_len
        || report.disposition != expected.disposition
    {
        return Err(RestSelectionEncodingError::InvalidSelection);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::outer::velocity::recovery::RecoveryStage;
    use crate::commissioning::types::{RestSelectionDiagnostic, RestSelectionOwner};

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
            owner: RestSelectionOwner::IntegralPositiveObservation,
            rung_index: 7,
            direction: 1,
            slot: 0xff,
            disposition: RestSelectionDisposition::StatisticalAcceptance,
            retained_sample_count: 256,
        }
    }

    #[test]
    fn rest_selection_record_has_exact_80_byte_layout() {
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
        expected.extend_from_slice(&report.retained_sample_count.to_le_bytes());
        for interval in report.intervals_q {
            for endpoint in interval {
                expected.extend_from_slice(&endpoint.to_le_bytes());
            }
        }

        assert_eq!(record.as_bytes(), expected);
        assert_eq!(record.as_bytes().len(), 80);
    }

    #[test]
    fn rest_selection_accepts_noncontiguous_causal_suffix_chain() {
        let mut report = rest_selection_diagnostic();
        report.structural_codes = [
            RestStructuralCode::Usable,
            RestStructuralCode::StructuredResidual,
            RestStructuralCode::Usable,
        ];
        report.intervals_q = [[-58_721, 90_287], [-29_401, 45_103], [-6_848, 22_512]];
        report.interval_present_mask = 0b111;
        report.usable_mask = 0b101;
        report.zero_compatible_mask = 0b101;
        report.selected_mask = 0b100;
        report.selected_suffix_len = 256;

        let record = encode_rest_selection(report).unwrap();

        assert_eq!(record.as_bytes()[26], 0b100);
        assert_eq!(&record.as_bytes()[27..29], &256_u16.to_le_bytes());
    }

    #[test]
    fn rest_selection_codes_and_stage_encoders_are_shared() {
        assert_eq!(
            [
                RestSelectionOwner::ProportionalObservation as u8,
                RestSelectionOwner::ProportionalRecovery as u8,
                RestSelectionOwner::ProportionalSelectedRecovery as u8,
                RestSelectionOwner::IntegralAnchor as u8,
                RestSelectionOwner::IntegralPositiveObservation as u8,
                RestSelectionOwner::IntegralRecovery as u8,
                RestSelectionOwner::IntegralCleanup as u8,
            ],
            [0, 1, 2, 3, 4, 5, 6]
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
        let integral = super::super::integral_reporting::encode_velocity_integral_event(
            crate::commissioning::types::VelocityIntegralEvent::RestSelection(report),
        );
        assert_eq!(integral.as_bytes(), shared.as_bytes());
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

        let mut selected_past_contradicting_middle = valid;
        selected_past_contradicting_middle.structural_codes = [
            RestStructuralCode::Usable,
            RestStructuralCode::ResolvedSlope,
            RestStructuralCode::Usable,
        ];
        selected_past_contradicting_middle.usable_mask = 0b101;
        selected_past_contradicting_middle.zero_compatible_mask = 0b101;
        selected_past_contradicting_middle.selected_mask = 0b100;
        selected_past_contradicting_middle.selected_suffix_len = 256;
        assert!(encode_rest_selection(selected_past_contradicting_middle).is_err());

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
}
