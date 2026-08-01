//! FTRC trace-stream encoders for breakaway-campaign evidence.
//!
//! Mirrors `sweep_reporting`'s per-event canonical-record encoders: each
//! function here takes one already-assembled `breakaway_reporting` wire
//! record and pushes its fields, in a fixed order, into a
//! [`VelocitySweepCanonicalRecord`] -- the same bounded byte-buffer type the
//! velocity-sweep and acceptance-matrix encoders already use, despite its
//! name (it is a generic canonical-record primitive, not sweep-specific; see
//! its own doc comment). `commissioning::mod::emit_trace_after_step` hands
//! each encoded record to `CommissioningBackend::record_velocity_sweep_trace_record`,
//! which writes it onto the dedicated FTRC bulk USB interface -- a physically
//! separate stream from the Klipper command-reply connection
//! `commission_dispatch::replies::emit_breakaway_campaign_event` feeds. Both
//! paths carry the same evidence; this module is the trace-stream half.
//!
//! Each record's leading byte is a local tag distinguishing record kinds
//! *within* the breakaway family -- it is only meaningful alongside the
//! `EVENT_COMMISSION_BREAKAWAY_*` id the surrounding `CommissionTraceEvent`
//! marker carries (mirroring how `sweep_reporting`'s own tags 0-11 are local
//! to the velocity-sweep family, and `acceptance_reporting`'s tags 0-3 are a
//! separate, independently-numbered family). The frame layouts here are the
//! contract `foci-trace`'s host-side decoder reconstructs breakaway evidence
//! from: every layout is pinned by a golden byte-sequence test below.
#![cfg_attr(not(test), allow(dead_code))]

use crate::commissioning::types::{
    AdditiveDiscoveryPlanned, AdditiveDiscoveryTerminal, BandConfirmationObservation,
    BandConfirmationPlanned, BandConfirmationTerminal, BreakawayCampaignTerminal,
    BreakawayCeilingSource, BreakawayDirectionalBreakaway, BreakawayDiscoveryRungMargin,
    BreakawayLadderRung, BreakawayProbePlanned, BreakawayProbeTerminal,
    BreakawayRungZeroDiagnostic,
};

use crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord;

fn push_u64(record: &mut VelocitySweepCanonicalRecord, value: u64) {
    record.push_u32(value as u32);
    record.push_u32((value >> 32) as u32);
}

/// Encode the immutable breakaway-probe plan (tag 0).
pub(crate) fn encode_probe_planned(report: BreakawayProbePlanned) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(0);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u16(report.family_size);
    record.push_u8(report.max_observations);
    record.push_u8(report.search_count);
    record.push_u16(report.p_start_raw);
    record.push_u16(report.p_top_raw);
    record
}

/// Encode one direction's resolved inert/moving breakaway bracket (tag 1).
pub(crate) fn encode_directional_breakaway(
    report: BreakawayDirectionalBreakaway,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(1);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.direction);
    record.push_u8(report.inert_present);
    record.push_u16(report.inert_p_raw);
    record.push_u16(report.moving_p_raw);
    record.push_u8(report.observations);
    record
}

/// Encode a probe-campaign terminal that did not resolve a breakaway (tag 2).
pub(crate) fn encode_probe_terminal(
    report: BreakawayProbeTerminal,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(2);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u16(report.family_size);
    record.push_u8(report.terminal_cause);
    record.push_u8(report.has_safety_fault);
    record
}

/// Encode the immutable additive discovery plan (tag 3).
pub(crate) fn encode_discovery_planned(
    report: AdditiveDiscoveryPlanned,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(3);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    push_u64(&mut record, report.prior_plan_digest);
    record.push_u16(report.family_size);
    record.push_u8(report.rung_count);
    record.push_u16(report.observations_per_direction);
    record.push_u16(report.floor_p_raw);
    record.push_u8(report.floor_origin);
    record.push_u16(report.breakaway_p_raw);
    record.push_u16(report.ceiling_p_raw);
    record.push_u32(report.first_additive_step_raw);
    record.push_u32(report.maximum_workflow_ms);
    record
}

/// Encode which `p_top` candidate bound the discovery authority ceiling (tag 4).
pub(crate) fn encode_ceiling_source(
    report: BreakawayCeilingSource,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(4);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.binding_source);
    record
}

/// Encode one rung of the complete resolved additive ladder (tag 5).
pub(crate) fn encode_ladder_rung(report: BreakawayLadderRung) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(5);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.rung_index);
    record.push_u16(report.p_raw);
    record
}

/// Encode a discovery-campaign terminal (tag 6).
pub(crate) fn encode_discovery_terminal(
    report: AdditiveDiscoveryTerminal,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(6);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    push_u64(&mut record, report.prior_plan_digest);
    record.push_u16(report.family_size);
    record.push_u8(report.terminal_cause);
    record.push_u8(report.collected_count);
    record.push_u8(report.has_safety_fault);
    record
}

/// Encode one in-band discovery rung's nomination bounds and margin (tag 7).
pub(crate) fn encode_discovery_rung_margin(
    report: BreakawayDiscoveryRungMargin,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(7);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.collected_index);
    record.push_u16(report.p_raw);
    record.push_i32(report.lower_percent_milli);
    record.push_i32(report.upper_percent_milli);
    record.push_i32(report.margin_percent_milli);
    record.push_i32(report.half_width_percent_milli);
    record.push_i32(report.adjacent_rate_step_permille);
    record
}

/// Encode whether the additive ladder's rung zero itself showed motion (tag 8).
pub(crate) fn encode_rung_zero_diagnostic(
    report: BreakawayRungZeroDiagnostic,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(8);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.forward_moved);
    record.push_u8(report.reverse_moved);
    record
}

/// Encode the immutable held-out confirmation plan (tag 9).
pub(crate) fn encode_confirmation_planned(
    report: BandConfirmationPlanned,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(9);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    push_u64(&mut record, report.prior_plan_digest);
    record.push_u16(report.family_size);
    record.push_u16(report.observations_per_direction);
    record.push_u16(report.candidate_p_raw);
    record.push_u8(report.band_lower_percent);
    record.push_u8(report.band_upper_percent);
    record.push_u8(report.capture_profile);
    record.push_u8(report.acceptance_rule);
    record.push_i32(report.nominated_margin_percent_milli);
    record
}

/// Encode a confirmation-block terminal (tag 10).
pub(crate) fn encode_confirmation_terminal(
    report: BandConfirmationTerminal,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(10);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    push_u64(&mut record, report.prior_plan_digest);
    record.push_u16(report.family_size);
    record.push_u8(report.terminal_cause);
    record.push_u8(report.forward_collected_mask);
    record.push_u8(report.forward_eligible_mask);
    record.push_u8(report.forward_included_mask);
    record.push_u8(report.reverse_collected_mask);
    record.push_u8(report.reverse_eligible_mask);
    record.push_u8(report.reverse_included_mask);
    record.push_u8(report.accepted);
    record.push_u16(report.confirmed_p_raw);
    record.push_u16(report.max_relative_se_permille);
    record.push_u16(report.required_relative_se_permille);
    record.push_u8(report.has_safety_fault);
    record
}

/// Encode one fresh held-out confirmation stroke's response interval (tag 11).
pub(crate) fn encode_confirmation_observation(
    report: BandConfirmationObservation,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(11);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.slot_index);
    record.push_u8(report.direction);
    record.push_i32(report.response_lower_percent_milli);
    record.push_i32(report.response_upper_percent_milli);
    record.push_u16(report.relative_standard_error_permille);
    record
}

/// Encode the canonical closure record for one complete breakaway campaign
/// (tag 12).
pub(crate) fn encode_campaign_terminal(
    report: BreakawayCampaignTerminal,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(12);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    record.push_u8(report.phase);
    record.push_u8(report.terminal_cause);
    record.push_u8(report.accepted);
    push_u64(&mut record, report.stage_c_plan_digest);
    record
}

#[cfg(test)]
#[path = "breakaway_trace_reporting_tests.rs"]
mod tests;
