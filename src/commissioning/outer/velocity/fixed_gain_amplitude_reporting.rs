//! Canonical trace and compact wire reporting for fixed-gain amplitude validation runs.

use crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord;
use crate::commissioning::types::{
    FixedGainAmplitudeEvent, FixedGainAmplitudeTerminal, RungOriginRecoveryEvidence,
    VelocityIntegralEvent, VelocityIntegralObservation, VelocityPrimitiveCurrentEvidence,
};

use super::fixed_gain_amplitude_evidence::{
    FixedGainAmplitudeDirectionalEvidence, FixedGainAmplitudeReport,
};
use super::fixed_gain_amplitude_plan::{FixedGainAmplitudePlan, schedule_order_code};
use super::reproduction::BUILD_IDENTITY;

/// Bytes in the compact ordinary amplitude-plan reply payload.
pub(crate) const FIXED_GAIN_AMPLITUDE_PLAN_REPLY_BYTES: usize = 66;
/// Payload bytes carried by each amplitude-plan reply fragment.
///
/// The 66-byte plan cannot cross the transport in one message, so it ships as
/// two equal fragments that each fit the ordinary payload budget.
pub(crate) const FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES: usize = 33;
/// Number of amplitude-plan reply fragments.
#[cfg(test)]
pub(crate) const FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENTS: usize = 2;
/// Bytes in the compact ordinary amplitude-terminal reply payload.
pub(crate) const FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES: usize = 39;

/// Largest byte-array payload an ordinary reply can carry in one message.
///
/// A message is capped at 64 bytes, of which 2 header and 3 trailer bytes are
/// framing, leaving 59 for the encoded reply. Each reply then spends up to 3
/// bytes on its worst-case response id, up to 2 on the `oid` VLQ, and 1 on the
/// byte-array length prefix, so 53 bytes remain for payload data.
///
/// A release-build transport silently rolls back and drops any oversized frame,
/// so an over-budget payload never reaches the host and logs nothing anywhere.
#[cfg(test)]
pub(crate) const ORDINARY_REPLY_PAYLOAD_MAX_BYTES: usize = 53;

/// Encode the compact amplitude-plan ordinary reply payload.
pub(crate) fn encode_fixed_gain_amplitude_plan_reply(
    run_sequence: u32,
    plan: FixedGainAmplitudePlan,
) -> [u8; FIXED_GAIN_AMPLITUDE_PLAN_REPLY_BYTES] {
    let mut bytes = [0_u8; FIXED_GAIN_AMPLITUDE_PLAN_REPLY_BYTES];
    let mut offset = 0;
    push_reply(&mut bytes, &mut offset, &plan.schema_revision.to_le_bytes());
    push_reply(&mut bytes, &mut offset, &run_sequence.to_le_bytes());
    push_reply(
        &mut bytes,
        &mut offset,
        &[schedule_order_code(plan.order, plan.slot_order)],
    );
    push_reply(&mut bytes, &mut offset, &plan.digest().to_le_bytes());
    push_reply(
        &mut bytes,
        &mut offset,
        &plan.acceptance_identity.plan_digest.to_le_bytes(),
    );
    push_reply(&mut bytes, &mut offset, &plan.selected_p.to_le_bytes());
    push_reply(&mut bytes, &mut offset, &plan.selected_i.to_le_bytes());
    for target in plan.target_rpm {
        push_reply(&mut bytes, &mut offset, &target.to_le_bytes());
    }
    push_reply(&mut bytes, &mut offset, &plan.family_size.to_le_bytes());
    push_reply(
        &mut bytes,
        &mut offset,
        &plan.expected_observations.to_le_bytes(),
    );
    push_reply(&mut bytes, &mut offset, &[plan.amplitude_count]);
    push_reply(
        &mut bytes,
        &mut offset,
        &plan.nominal_workflow_ms.to_le_bytes(),
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &plan.maximum_workflow_ms.to_le_bytes(),
    );
    for lower_rate_q in plan.recovery_lower_rate_q {
        push_reply(&mut bytes, &mut offset, &lower_rate_q.to_le_bytes());
    }
    debug_assert_eq!(offset, FIXED_GAIN_AMPLITUDE_PLAN_REPLY_BYTES);
    bytes
}

/// Encode the compact amplitude-terminal ordinary reply payload.
pub(crate) fn encode_fixed_gain_amplitude_terminal_reply(
    terminal: FixedGainAmplitudeTerminal,
) -> [u8; FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES] {
    let mut bytes = [0_u8; FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES];
    let mut offset = 0;
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.schema_revision.to_le_bytes(),
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.run_sequence.to_le_bytes(),
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.evidence_sequence.to_le_bytes(),
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &[terminal.outcome as u8, terminal.cause as u8],
    );
    push_reply(&mut bytes, &mut offset, &terminal.plan_digest.to_le_bytes());
    push_reply(&mut bytes, &mut offset, &terminal.digest.to_le_bytes());
    for masks in [
        terminal.attempted_masks,
        terminal.eligible_masks,
        terminal.current_terminus_masks,
        terminal.unattempted_masks,
    ] {
        push_reply(&mut bytes, &mut offset, &masks);
    }
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.emitted_observations.to_le_bytes(),
    );
    push_reply(&mut bytes, &mut offset, &[terminal.emitted_amplitudes]);
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.qualifier_bits.to_le_bytes(),
    );
    debug_assert_eq!(offset, FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES);
    bytes
}

fn push_reply<const N: usize>(destination: &mut [u8; N], offset: &mut usize, value: &[u8]) {
    let end = offset
        .checked_add(value.len())
        .expect("fixed amplitude reply length fits usize");
    destination[*offset..end].copy_from_slice(value);
    *offset = end;
}

/// Encode the complete immutable amplitude plan as bounded canonical fragments.
#[allow(dead_code)]
pub(crate) fn encode_fixed_gain_amplitude_plan(
    run_sequence: u32,
    plan: FixedGainAmplitudePlan,
) -> [VelocitySweepCanonicalRecord; 3] {
    let digest = plan.digest();
    let mut core = amplitude_record_header(0, run_sequence, 0, digest);
    core.push_u8(0);
    core.push_u16(plan.schema_revision);
    core.push_u8(plan.acceptance_identity.channel);
    core.push_u16(plan.acceptance_identity.build_features);
    core.push_u32(BUILD_IDENTITY);
    core.push_u16(plan.acceptance_identity.schema_revision);
    push_u64(&mut core, plan.acceptance_identity.plan_digest);
    core.push_u8(schedule_order_code(plan.order, plan.slot_order));
    core.push_u16(plan.selected_p);
    core.push_u16(plan.selected_i);
    core.push_u16(plan.qualifier_bits);
    core.push_u32(plan.pwm_hz);
    core.push_u32(plan.encoder_counts_per_rev);
    for target in plan.target_rpm {
        core.push_i32(i32::from(target));
    }
    core.push_u8(plan.amplitude_count);
    core.push_u16(plan.expected_observations);
    core.push_u16(plan.family_size);

    let mut timing = amplitude_record_header(0, run_sequence, 0, digest);
    timing.push_u8(1);
    timing.push_u32(plan.moving_stroke_us);
    timing.push_u32(plan.zero_settle_us);
    timing.push_u32(plan.analysis_budget_us);
    timing.push_u32(plan.nominal_workflow_ms);
    timing.push_u32(plan.maximum_workflow_ms);
    timing.push_u32(plan.recovery_origin_band_counts);
    for lower_rate_q in plan.recovery_lower_rate_q {
        push_u64(&mut timing, lower_rate_q);
    }
    timing.push_u32(plan.recovery_nominal_slot_us);
    timing.push_u32(plan.recovery_maximum_slot_us);
    timing.push_u8(plan.recovery_slot_count);
    timing.push_u16(plan.hard_torque_limit);
    timing.push_u16(plan.usable_torque_limit);
    timing.push_u8(plan.limit_evidence_capability);
    timing.push_u8(u8::from(plan.recovery_quantization_exposed));
    timing.push_u8(u8::from(plan.probe_constrained_test_point));

    let mut motion = amplitude_record_header(0, run_sequence, 0, digest);
    motion.push_u8(2);
    motion.push_u32(plan.motion_request.requested_velocity_mrev_s);
    motion.push_u32(plan.motion_request.machine_velocity_ceiling_mrev_s);
    motion.push_u8(plan.motion_request.requested_velocity_source as u8);
    motion.push_u32(plan.motion_request.max_stroke_travel_mrev);
    motion.push_u32(plan.motion_request.settle_travel_reserve_mrev);
    motion.push_u32(plan.motion_request.negative_position_headroom_mrev);
    motion.push_u32(plan.motion_request.positive_position_headroom_mrev);
    motion.push_u16(plan.motion_request.max_duration_ms);
    [core, timing, motion]
}

/// Encode one intrinsic moving observation for the amplitude trace stream.
pub(crate) fn encode_fixed_gain_amplitude_observation(
    report: VelocityIntegralObservation,
) -> VelocitySweepCanonicalRecord {
    super::integral_reporting::encode_velocity_integral_event(VelocityIntegralEvent::Observation(
        report,
    ))
}

/// Encode one amplitude primitive's current and rest evidence.
pub(crate) fn encode_fixed_gain_amplitude_current(
    report: VelocityPrimitiveCurrentEvidence,
) -> VelocitySweepCanonicalRecord {
    crate::ankyra::velocity_sweep_encoding::encode_velocity_primitive_current_evidence(report)
}

/// Encode one evidence-excluded amplitude origin recovery.
pub(crate) fn encode_fixed_gain_amplitude_recovery(
    report: RungOriginRecoveryEvidence,
) -> VelocitySweepCanonicalRecord {
    super::sweep_reporting::encode_rung_origin_recovery(report)
}

/// Encode one trace-only amplitude event in its canonical representation.
pub(crate) fn encode_fixed_gain_amplitude_event(
    event: FixedGainAmplitudeEvent,
) -> VelocitySweepCanonicalRecord {
    match event {
        FixedGainAmplitudeEvent::Observation(report) => {
            encode_fixed_gain_amplitude_observation(report)
        }
        FixedGainAmplitudeEvent::Current(report) => encode_fixed_gain_amplitude_current(report),
        FixedGainAmplitudeEvent::Recovery(report) => encode_fixed_gain_amplitude_recovery(report),
        FixedGainAmplitudeEvent::RecoveryRest(report) => {
            super::sweep_reporting::encode_recovery_rest(report)
        }
        FixedGainAmplitudeEvent::RestSelection(report) => {
            super::sweep_reporting::encode_rest_selection(report)
                .expect("completed scored-rest diagnostic is coherent")
        }
        FixedGainAmplitudeEvent::Amplitude(report) => encode_fixed_gain_amplitude_amplitude(
            report.run_sequence,
            report.evidence_sequence,
            report.plan_digest,
            report.report,
        ),
    }
}

/// Encode one completed amplitude fold in the canonical amplitude representation.
pub(crate) fn encode_fixed_gain_amplitude_amplitude(
    run_sequence: u32,
    evidence_sequence: u16,
    plan_digest: u64,
    report: FixedGainAmplitudeReport,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(2);
    record.push_u32(run_sequence);
    record.push_u16(evidence_sequence);
    push_u64(&mut record, plan_digest);
    record.push_u8(report.amplitude_index);
    record.push_i32(i32::from(report.target_rpm));
    for direction in report.directions {
        encode_direction(&mut record, direction);
    }
    record
}

fn encode_direction(
    record: &mut VelocitySweepCanonicalRecord,
    direction: FixedGainAmplitudeDirectionalEvidence,
) {
    record.push_u8(direction.availability as u8);
    record.push_u8(direction.consensus.classification as u8);
    for component in direction.consensus.components {
        record.push_i32(component[0]);
        record.push_i32(component[1]);
    }
    record.push_u8(direction.consensus.component_count);
    record.push_u8(direction.consensus.collected_mask);
    record.push_u8(direction.consensus.eligible_mask);
    record.push_u8(direction.consensus.included_mask);
    record.push_i32(
        i32::try_from(direction.signed_target_rate_q)
            .expect("validated amplitude target rate fits i32"),
    );
    encode_optional_interval(record, direction.response_interval_q);
    encode_optional_i32(record, direction.robust_location_q);
    encode_optional_interval(
        record,
        direction.deficit_interval_q.map(|interval| {
            [
                i32::try_from(interval[0]).expect("validated amplitude deficit fits i32"),
                i32::try_from(interval[1]).expect("validated amplitude deficit fits i32"),
            ]
        }),
    );
    record.push_u8(direction.excluded_mask);
    let blocking_levels = direction
        .blocking_levels
        .into_iter()
        .enumerate()
        .fold(0_u16, |packed, (index, level)| {
            packed | (u16::from(level) << (index * 4))
        });
    record.push_u16(blocking_levels);
}

fn encode_optional_interval(record: &mut VelocitySweepCanonicalRecord, interval: Option<[i32; 2]>) {
    if let Some(interval) = interval {
        record.push_u8(1);
        record.push_i32(interval[0]);
        record.push_i32(interval[1]);
    } else {
        record.push_u8(0);
    }
}

fn encode_optional_i32(record: &mut VelocitySweepCanonicalRecord, value: Option<i32>) {
    if let Some(value) = value {
        record.push_u8(1);
        record.push_i32(value);
    } else {
        record.push_u8(0);
    }
}

/// Encode one amplitude terminal in the canonical amplitude representation.
pub(crate) fn encode_fixed_gain_amplitude_terminal(
    terminal: &FixedGainAmplitudeTerminal,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(3);
    record.push_u32(terminal.run_sequence);
    record.push_u16(terminal.evidence_sequence);
    record.push_u16(terminal.schema_revision);
    push_u64(&mut record, terminal.acceptance_plan_digest);
    push_u64(&mut record, terminal.plan_digest);
    push_u64(&mut record, terminal.digest);
    record.push_u8(terminal.outcome as u8);
    record.push_u8(terminal.cause as u8);
    for masks in [
        terminal.attempted_masks,
        terminal.eligible_masks,
        terminal.current_terminus_masks,
        terminal.unattempted_masks,
    ] {
        record.push_u8(masks[0]);
        record.push_u8(masks[1]);
    }
    record.push_u16(terminal.expected_observations);
    record.push_u16(terminal.emitted_observations);
    record.push_u8(terminal.expected_amplitudes);
    record.push_u8(terminal.emitted_amplitudes);
    record.push_u16(terminal.qualifier_bits);
    push_u64(&mut record, terminal.run_started_us);
    push_u64(&mut record, terminal.run_completed_us);
    record
}

fn push_u64(record: &mut VelocitySweepCanonicalRecord, value: u64) {
    record.push_u32(value as u32);
    record.push_u32((value >> 32) as u32);
}

#[allow(dead_code)]
fn amplitude_record_header(
    kind: u8,
    run_sequence: u32,
    evidence_sequence: u16,
    plan_digest: u64,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(kind);
    record.push_u32(run_sequence);
    record.push_u16(evidence_sequence);
    push_u64(&mut record, plan_digest);
    record
}

#[cfg(test)]
mod tests {
    use crate::ankyra::velocity_sweep_encoding::VELOCITY_SWEEP_CANONICAL_RECORD_MAX;
    use crate::commissioning::outer::velocity::fixed_gain_amplitude_evidence::{
        FixedGainAmplitudeAvailability, FixedGainAmplitudeDirectionalEvidence,
        FixedGainAmplitudeReport,
    };
    use crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::{
        FixedGainAmplitudeOrder, FixedGainAmplitudePlan,
    };
    use crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder;
    use crate::commissioning::outer::velocity::integral_reproduction::StageCPlanIdentity;
    use crate::commissioning::outer::velocity::recovery::RecoveryStage;
    use crate::commissioning::outer::velocity::sweep_consensus::{
        DirectionalConsensus, DirectionalConsensusClass,
    };
    use crate::commissioning::outer::velocity::sweep_plan::{
        RequestedVelocitySource, VelocitySweepRequest,
    };
    use crate::commissioning::types::{
        FixedGainAmplitudeCause, FixedGainAmplitudeEvent, FixedGainAmplitudeOutcome,
        FixedGainAmplitudeTerminal, RecoveryRestDiagnostic,
    };

    #[test]
    fn amplitude_recovery_rest_reuses_the_exact_event_150_record() {
        let report = RecoveryRestDiagnostic {
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
        };
        let encoded =
            super::encode_fixed_gain_amplitude_event(FixedGainAmplitudeEvent::RecoveryRest(report));
        let shared = super::super::sweep_reporting::encode_recovery_rest(report);

        assert_eq!(encoded.as_bytes(), shared.as_bytes());
        assert_eq!(encoded.as_bytes().len(), 89);
    }

    fn plan() -> FixedGainAmplitudePlan {
        FixedGainAmplitudePlan {
            slot_order: IntegralSlotOrder::ForwardFirst,
            schema_revision:
                crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION,
            acceptance_identity: StageCPlanIdentity {
                channel: 0,
                build_features: 0x1234,
                schema_revision:
                    crate::commissioning::outer::velocity::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION,
                plan_digest: 0x1111_2222_3333_4444,
            },
            order: FixedGainAmplitudeOrder::Ascending,
            target_rpm: [16, 33, 66, 132, 263],
            selected_p: 1_024,
            selected_i: 1_024,
            qualifier_bits: 0x0123,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            amplitude_count: 5,
            expected_observations: 40,
            family_size: 20,
            moving_stroke_us: 512_000,
            zero_settle_us: 500_000,
            analysis_budget_us: 236_000,
            nominal_workflow_ms: 63_041,
            maximum_workflow_ms: 66_456,
            recovery_origin_band_counts: 1_000,
            recovery_lower_rate_q: [13_217_792, 12_827_648],
            recovery_nominal_slot_us: 2_316_958,
            recovery_maximum_slot_us: 3_000_000,
            recovery_slot_count: 5,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_816,
            limit_evidence_capability: 1,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            motion_request: VelocitySweepRequest {
                requested_velocity_mrev_s: 4_394,
                machine_velocity_ceiling_mrev_s: 10_000,
                requested_velocity_source: RequestedVelocitySource::Defaulted,
                max_stroke_travel_mrev: 2_500,
                settle_travel_reserve_mrev: 250,
                negative_position_headroom_mrev: 7_025,
                positive_position_headroom_mrev: 7_025,
                max_duration_ms: 3_000,
            },
        }
    }

    /// Byte offset of `acceptance_identity.schema_revision` in the encoded
    /// amplitude-plan core record (`records[0]` returned by
    /// `encode_fixed_gain_amplitude_plan`).
    ///
    /// Counted from the fixed prefix `encode_fixed_gain_amplitude_plan` pushes ahead of
    /// the field: tag(1) + run_sequence(4) + evidence_sequence(2) +
    /// plan_digest(8) + subtype(1) + plan.schema_revision(2) +
    /// acceptance_identity.channel(1) + acceptance_identity.build_features(2)
    /// + BUILD_IDENTITY(4) = 25.
    const ACCEPTANCE_IDENTITY_SCHEMA_REVISION_OFFSET: usize = 25;

    /// Read a little-endian `u16` out of an encoded record at a fixed offset.
    fn read_u16_at(bytes: &[u8], offset: usize) -> u16 {
        u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap())
    }

    /// A breakaway-derived amplitude run must disclose the breakaway Stage-C
    /// schema (16), not the outer amplitude schema and not the combined
    /// constant, so the wire reply lets the host tell the two acceptance
    /// families apart.
    #[test]
    fn fixed_gain_amplitude_plan_discloses_breakaway_identity_schema() {
        let plan = plan();

        let records = super::encode_fixed_gain_amplitude_plan(7, plan);

        let disclosed = read_u16_at(
            records[0].as_bytes(),
            ACCEPTANCE_IDENTITY_SCHEMA_REVISION_OFFSET,
        );
        assert_eq!(
            disclosed,
            crate::commissioning::outer::velocity::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION
        );
    }

    #[test]
    fn plan_records_preserve_all_fragments_with_one_digest() {
        let plan = plan();

        let records = super::encode_fixed_gain_amplitude_plan(7, plan);

        assert_eq!(records.map(|record| record.as_bytes().len()), [75, 72, 43]);
        assert!(
            records
                .iter()
                .all(|record| record.as_bytes().len() <= VELOCITY_SWEEP_CANONICAL_RECORD_MAX)
        );
        assert_eq!(records.map(|record| record.as_bytes()[15]), [0, 1, 2]);
        let digest = plan.digest().to_le_bytes();
        assert!(
            records
                .iter()
                .all(|record| record.as_bytes()[7..15] == digest)
        );
    }

    #[test]
    fn compact_plan_reply_appends_exact_recovery_bounds() {
        let plan = plan();

        let payload = super::encode_fixed_gain_amplitude_plan_reply(7, plan);

        assert_eq!(payload.len(), 66);
        assert_eq!(&payload[50..58], &13_217_792_u64.to_le_bytes());
        assert_eq!(&payload[58..66], &12_827_648_u64.to_le_bytes());
    }

    fn direction() -> FixedGainAmplitudeDirectionalEvidence {
        FixedGainAmplitudeDirectionalEvidence {
            availability: FixedGainAmplitudeAvailability::Eligible,
            consensus: DirectionalConsensus {
                classification: DirectionalConsensusClass::EligibleConsensus,
                components: [[10, 20], [0, 0]],
                component_count: 1,
                collected_mask: 0b1111,
                eligible_mask: 0b1111,
                included_mask: 0b0111,
            },
            signed_target_rate_q: 17_533_333,
            response_interval_q: Some([17_300_000, 17_600_000]),
            robust_location_q: Some(17_450_000),
            deficit_interval_q: Some([-66_667, 233_333]),
            excluded_mask: 0b1000,
            blocking_levels: [2, 3, 4, 4],
        }
    }

    #[test]
    fn amplitude_record_is_exact_and_bounded() {
        let report = FixedGainAmplitudeReport {
            amplitude_index: 4,
            target_rpm: 263,
            directions: [direction(), direction()],
        };

        let record =
            super::encode_fixed_gain_amplitude_amplitude(7, 42, 0x0123_4567_89ab_cdef, report);

        assert!(record.as_bytes().len() <= VELOCITY_SWEEP_CANONICAL_RECORD_MAX);
        assert_eq!(&record.as_bytes()[..7], &[2, 7, 0, 0, 0, 42, 0]);
    }

    #[test]
    fn terminal_record_preserves_masks_counts_and_identity() {
        let terminal = FixedGainAmplitudeTerminal {
            schema_revision:
                crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION,
            run_sequence: 7,
            evidence_sequence: 43,
            acceptance_plan_digest: 0x1111_2222_3333_4444,
            plan_digest: 0x5555_6666_7777_8888,
            digest: 0x9999_aaaa_bbbb_cccc,
            outcome: FixedGainAmplitudeOutcome::Inconclusive,
            cause: FixedGainAmplitudeCause::VelocityRestNotConfirmed,
            attempted_masks: [0b11111; 2],
            eligible_masks: [0b11110, 0b11100],
            current_terminus_masks: [0, 0b10000],
            unattempted_masks: [0; 2],
            expected_observations: 40,
            emitted_observations: 40,
            expected_amplitudes: 5,
            emitted_amplitudes: 5,
            qualifier_bits: 0x1234,
            run_started_us: 100,
            run_completed_us: 200,
            safety_fault: None,
        };

        let record = super::encode_fixed_gain_amplitude_terminal(&terminal);

        assert!(record.as_bytes().len() <= VELOCITY_SWEEP_CANONICAL_RECORD_MAX);
        assert_eq!(&record.as_bytes()[..7], &[3, 7, 0, 0, 0, 43, 0]);
        assert_eq!(
            &record.as_bytes()[33..35],
            &[
                FixedGainAmplitudeOutcome::Inconclusive as u8,
                FixedGainAmplitudeCause::VelocityRestNotConfirmed as u8,
            ]
        );
        assert!(
            record
                .as_bytes()
                .windows(2)
                .any(|bytes| bytes == [0x34, 0x12])
        );
    }

    #[test]
    fn compact_terminal_reply_carries_evidence_sequence() {
        let terminal = FixedGainAmplitudeTerminal {
            schema_revision:
                crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION,
            run_sequence: 7,
            evidence_sequence: 43,
            acceptance_plan_digest: 0x1111_2222_3333_4444,
            plan_digest: 0x5555_6666_7777_8888,
            digest: 0x9999_aaaa_bbbb_cccc,
            outcome: FixedGainAmplitudeOutcome::Inconclusive,
            cause: FixedGainAmplitudeCause::VelocityRestNotConfirmed,
            attempted_masks: [0b11111; 2],
            eligible_masks: [0b11110, 0b11100],
            current_terminus_masks: [0, 0b10000],
            unattempted_masks: [0; 2],
            expected_observations: 40,
            emitted_observations: 40,
            expected_amplitudes: 5,
            emitted_amplitudes: 5,
            qualifier_bits: 0x1234,
            run_started_us: 100,
            run_completed_us: 200,
            safety_fault: None,
        };

        let payload = super::encode_fixed_gain_amplitude_terminal_reply(terminal);

        assert_eq!(payload.len(), 39);
        assert_eq!(&payload[6..8], &43_u16.to_le_bytes());
    }
}

#[cfg(test)]
mod reply_payload_budget_tests {
    use super::{
        FIXED_GAIN_AMPLITUDE_PLAN_REPLY_BYTES, FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES,
        FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENTS, FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES,
        ORDINARY_REPLY_PAYLOAD_MAX_BYTES,
    };

    /// An oversized payload is dropped silently in release builds, so the budget
    /// has to be enforced at compile-test time rather than discovered on hardware.
    #[test]
    fn every_ordinary_reply_payload_fits_one_transport_message() {
        for (name, bytes) in [
            (
                "fixed-gain amplitude plan fragment",
                FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES,
            ),
            (
                "fixed-gain amplitude terminal",
                FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES,
            ),
        ] {
            assert!(
                bytes <= ORDINARY_REPLY_PAYLOAD_MAX_BYTES,
                "{name} reply payload is {bytes} bytes, over the {ORDINARY_REPLY_PAYLOAD_MAX_BYTES}-byte transport budget"
            );
        }
    }

    /// Fragments must cover the whole plan payload with nothing dropped.
    #[test]
    fn plan_reply_fragments_span_the_entire_payload() {
        assert_eq!(
            FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES
                * FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENTS,
            FIXED_GAIN_AMPLITUDE_PLAN_REPLY_BYTES
        );
    }
}
