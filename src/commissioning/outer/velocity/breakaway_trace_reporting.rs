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
use crate::commissioning::types::{
    AdditiveDiscoveryPlanned, AdditiveDiscoveryTerminal, BandConfirmationObservation,
    BandConfirmationPlanned, BandConfirmationTerminal, BreakawayCampaignTerminal,
    BreakawayCeilingSource, BreakawayDiscoveryRungMargin, BreakawayProbeDecision,
    BreakawayProbePlanned, BreakawayProbeResolved, BreakawayProbeTerminal, BreakawayRawObservation,
    BreakawayRecoveryOriginMismatch, BreakawayRecoveryRestRejected, BreakawayRungZeroDiagnostic,
};

use crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord;

fn push_u64(record: &mut VelocitySweepCanonicalRecord, value: u64) {
    record.push_u32(value as u32);
    record.push_u32((value >> 32) as u32);
}

fn push_i64(record: &mut VelocitySweepCanonicalRecord, value: i64) {
    push_u64(record, value as u64);
}

fn push_u128(record: &mut VelocitySweepCanonicalRecord, value: u128) {
    push_u64(record, value as u64);
    push_u64(record, (value >> 64) as u64);
}

/// Encode the immutable breakaway-probe plan (tag 0).
pub(crate) fn encode_probe_planned(report: BreakawayProbePlanned) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(0);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.max_observations);
    record.push_u8(report.search_count);
    record.push_u16(report.p_start_raw);
    record.push_u16(report.p_top_raw);
    push_u64(&mut record, report.workflow_digest);
    record.push_u32(report.target_velocity_rpm as u32);
    record.push_u16(report.hard_torque_limit);
    record.push_u16(report.usable_torque_limit);
    record.push_u32(report.pwm_hz);
    record.push_u32(report.encoder_counts_per_rev);
    record.push_u32(report.motion_threshold_counts);
    record.push_u32(report.max_capture_interval_us);
    record
}

/// Encode the resolved repeatable-motion rung (tag 1).
pub(crate) fn encode_probe_resolved(
    report: BreakawayProbeResolved,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(1);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.rung_index);
    record.push_u16(report.breakaway_p_raw);
    record.push_u32(report.motion_threshold_counts);
    record.push_u8(report.observation_count);
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
    record.push_u8(report.terminal_cause);
    record.push_u8(report.has_safety_fault);
    record.push_u8(report.fault_checkpoint);
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
    record.push_u16(report.schema_revision);
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

/// Encode one collected observation's raw settled-rate evidence (tag 5).
///
/// Tag 5 previously carried a per-rung resolved-ladder record; that fan-out was
/// dropped (its 34-frame burst could overflow the board trace queue, and the
/// discovery-plan summary already pins the whole ladder through `plan_digest`).
/// The tag is reused here for the raw-observation stream that lets an offline
/// decoder independently recompute the nomination margins and confirmation
/// containment.
pub(crate) fn encode_raw_observation(
    report: BreakawayRawObservation,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(5);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.phase);
    record.push_u8(report.rung_index);
    record.push_u8(report.slot_index);
    record.push_u8(report.direction);
    push_i64(&mut record, report.mean_rate_q);
    push_u128(&mut record, report.variance_of_mean_q2);
    push_i64(&mut record, report.target_rate_q);
    // Frozen classify/consensus inputs, so an offline decoder derives the
    // consensus masks itself. Order is the decode contract.
    record.push_u8(report.stationarity_code);
    push_i64(&mut record, report.settled_rate_low_q);
    push_i64(&mut record, report.settled_rate_high_q);
    push_i64(&mut record, report.deficit_low_q);
    push_i64(&mut record, report.deficit_high_q);
    record.push_u16(report.predicted_torque_target_abs);
    record.push_u16(report.usable_torque_limit);
    record.push_u16(report.hard_torque_limit);
    record.push_u8(report.disturbance_present);
    record.push_i32(report.disturbance_low_q16);
    record.push_i32(report.disturbance_high_q16);
    record.push_u32(report.drift_ratio_ppm);
    record.push_u8(report.post_fit_lag_one_state);
    record.push_i32(report.stroke_min_displacement_counts);
    record.push_i32(report.stroke_max_displacement_counts);
    record.push_u32(report.capture_max_interval_us);
    // Appended rather than placed beside `predicted_torque_target_abs`, whose
    // `max` it decomposes, so every earlier field keeps its byte offset and an
    // offline decoder reads it as one trailing conditional block -- the same
    // shape every prior addition to this record used.
    record.push_u16(report.moving_pid_output_peak);
    record
}

/// Encode one post-recovery probe decision (tag 13).
pub(crate) fn encode_probe_decision(
    report: BreakawayProbeDecision,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(13);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.direction);
    record.push_u8(report.rung_index);
    record.push_u8(report.disposition);
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
    record.push_u8(report.error_code);
    record.push_u8(report.stop_rung_index);
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
    record.push_i32(report.adjacent_rate_step_percent_milli);
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
    record.push_u8(report.error_code);
    record
}

/// Encode one rejected recovery-rest scoring window (tag 14).
///
/// Carries the scored-rest selector's own per-suffix verdicts alongside the
/// settle-window displacement bounds that gate the one-count fallback, so a
/// decoder can attribute the rejection to the statistical route, the fallback
/// route, or both without re-running the selector.
pub(crate) fn encode_recovery_rest_rejected(
    report: BreakawayRecoveryRestRejected,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(14);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.phase);
    record.push_u8(report.stage);
    record.push_u8(report.rung_index);
    record.push_u8(report.slot_index);
    record.push_u8(report.direction);
    record.push_u16(report.p_raw);
    for code in report.structural_codes {
        record.push_u8(code);
    }
    record.push_u8(report.interval_present_mask);
    record.push_u8(report.usable_mask);
    record.push_u8(report.zero_compatible_mask);
    record.push_u8(report.selected_mask);
    record.push_u16(report.selected_suffix_len);
    record.push_u8(report.disposition);
    for interval in report.intervals_q {
        push_i64(&mut record, interval[0]);
        push_i64(&mut record, interval[1]);
    }
    record.push_u8(report.scored_net_displacement_counts as u8);
    record.push_u8(report.scored_min_displacement_counts as u8);
    record.push_u8(report.scored_max_displacement_counts as u8);
    record
}

/// Encode one recovery refused because its origin moved (tag 15).
///
/// Carries both offsets and their exact difference, so a decoder can separate
/// a single-sample race from a real runaway without re-deriving either.
pub(crate) fn encode_recovery_origin_mismatch(
    report: BreakawayRecoveryOriginMismatch,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(15);
    record.push_u32(report.run_sequence);
    record.push_u16(report.evidence_sequence);
    push_u64(&mut record, report.plan_digest);
    record.push_u8(report.phase);
    record.push_u8(report.stage);
    record.push_u8(report.rung_index);
    record.push_u8(report.slot_index);
    record.push_u8(report.direction);
    record.push_u16(report.p_raw);
    record.push_i32(report.expected_start_offset_counts);
    record.push_i32(report.observed_position_counts);
    record.push_i32(report.delta_counts);
    record.push_u32(report.position_window_counts);
    record
}

#[cfg(test)]
#[path = "breakaway_trace_reporting_tests.rs"]
mod tests;
