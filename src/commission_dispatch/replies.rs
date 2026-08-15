//! Shared commissioning reply emission.
//!
//! Board crates keep transport-specific ankyra sender calls behind
//! [`CommissionReplySink`]. This module owns the semantic mapping from terminal
//! commissioning payloads to reply events so boards cannot drift by dropping
//! fields locally.

#[cfg(feature = "trace-velocity-sweep")]
use crate::ankyra::velocity_sweep_encoding::{
    encode_rate_values, upper_bound_i64, upper_bound_u128,
};
use crate::autotune::{CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE, PhaseId};
use crate::commission_dispatch::ReplyPayload;
use crate::commissioning::types::{
    AdcCalResult, AdcResidualEvidence, CommissionResult, CurrentValidationAxisEvidence,
    CurrentValidationSettledSampleEvidence, INDUCTANCE_SOURCE_NONE, InnerResult, OuterResult,
    OuterSafetyFault, VelocitySweepTerminal,
};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics, SelftestStageValue};
use crate::tmc_diagnostics::{
    AdcResidualReport, ClosedLoopEntryReport, CommissionTimingReport, CurrentLoopFiltersReport,
    CurrentLoopHoldReport, CurrentLoopRunReport, CurrentValidationAxisReport,
    CurrentValidationEnvelopeReport, CurrentValidationSettledSampleReport, EncoderAlignmentReport,
    InductanceEstimateReport, InductanceFrameReport, InductanceRunReport, ResistanceAxisReport,
    ResistanceRunReport,
};

/// TMC4671 external electrical angle for electrical axis 0 (PHASE0).
const RESISTANCE_AXIS0_PHI_E_EXT: u16 = 0;

/// TMC4671 external electrical angle for electrical axis 1 (PHASE90).
const RESISTANCE_AXIS1_PHI_E_EXT: u16 = 16_384;

/// Board-local sink for commissioning-family Klipper replies.
///
/// Implementations call board-generated ankyra reply macros. Methods return
/// `()` to preserve the existing fire-and-forget reply behavior.
pub trait CommissionReplySink {
    /// Emit one retained commissioning diagnostic detail event.
    fn foci_commission_detail(&mut self, oid: u8, event: CommissionDiagnostic);

    /// Emit a commissioning phase reply.
    fn foci_commission_phase(&mut self, oid: u8, phase: u8, status: u8);

    /// Emit the Stage 1 commissioning result.
    fn foci_commission_result(&mut self, oid: u8, status: u8, result: &CommissionResult);

    /// Emit timing-quality evidence for one commissioning electrical method.
    ///
    /// The default keeps the shared API additive until each board adopts the
    /// coordinated data dictionary in its board-integration task.
    fn foci_commission_timing(&mut self, _report: CommissionTimingReport) {}

    /// Emit the resistance-identification per-run evidence captured during
    /// Stage 1 commissioning. `FociCommissionResult` is size-capped, so this
    /// reuses the `foci_resistance_run` reply (Task 6) to carry the evidence
    /// that lives on `InnerResult` to the host.
    fn foci_resistance_run(&mut self, report: ResistanceRunReport);

    /// Emit one resistance-identification per-axis fit result captured
    /// during Stage 1 commissioning. Reuses the `foci_resistance_axis` reply
    /// (Task 6) for the same reason as [`Self::foci_resistance_run`].
    fn foci_resistance_axis(&mut self, report: ResistanceAxisReport);

    /// Emit encoder D-axis alignment evidence captured during Stage 1
    /// commissioning. This makes settled current-validation cross-axis tails
    /// correlatable with the alignment residual without expanding the
    /// size-capped terminal commission result.
    fn foci_encoder_alignment(&mut self, report: EncoderAlignmentReport);

    /// Emit post-ADC-calibration zero-current residual evidence captured
    /// before the terminal commissioning result.
    fn foci_adc_residual(&mut self, report: AdcResidualReport);

    /// Emit run-level production inductance evidence captured during Stage 1
    /// commissioning.
    fn foci_inductance_run(&mut self, report: InductanceRunReport);

    /// Emit rotating-frame production inductance current evidence captured
    /// during Stage 1 commissioning.
    fn foci_inductance_frame(&mut self, report: InductanceFrameReport);

    /// Emit accepted production inductance estimate evidence captured during
    /// Stage 1 commissioning.
    fn foci_inductance_estimate(&mut self, report: InductanceEstimateReport);

    /// Emit sustained-hold current-loop evidence captured during Stage 1
    /// commissioning before the run-level current-loop summary.
    fn foci_current_loop_hold(&mut self, report: CurrentLoopHoldReport);

    /// Emit fast closed-loop entry stability evidence captured before the
    /// sustained-hold sampler.
    fn foci_closed_loop_entry(&mut self, report: ClosedLoopEntryReport);

    /// Emit applied filter cutoffs for the commissioned closed-loop frame.
    fn foci_current_loop_filters(&mut self, report: CurrentLoopFiltersReport);

    /// Emit run-level current-loop commissioning evidence captured during
    /// Stage 1 commissioning. `FociCommissionResult` is size-capped, so this
    /// supplemental reply carries source, tier, split, validation coverage,
    /// and failure-reason evidence separately.
    fn foci_current_loop_run(&mut self, report: CurrentLoopRunReport);

    /// Emit one per-axis, per-delay current validation evidence sample
    /// captured during Stage 1 commissioning.
    fn foci_current_validation_axis(&mut self, report: CurrentValidationAxisReport);

    /// Emit one raw settled-flux current validation read captured during Stage 1
    /// commissioning.
    fn foci_current_validation_settled_sample(
        &mut self,
        report: CurrentValidationSettledSampleReport,
    );

    /// Emit the current-validation target envelope captured during Stage 1
    /// commissioning.
    fn foci_current_validation_envelope(&mut self, report: CurrentValidationEnvelopeReport);

    /// Emit the Stage 2 tune result.
    fn foci_tune_result(&mut self, oid: u8, status: u8, result: &OuterResult);

    /// Emit Stage 2 safety-envelope evidence before the terminal tune result.
    fn foci_outer_safety_fault(&mut self, oid: u8, fault: OuterSafetyFault);

    /// Emit one compact rung-origin recovery summary.
    fn foci_rung_origin_recovery_summary(
        &mut self,
        _report: crate::ankyra::replies::FociRungOriginRecoverySummary,
    ) {
    }

    /// Emit causal current-limit evidence for one velocity primitive.
    fn foci_velocity_primitive_current_evidence(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityPrimitiveCurrentEvidence,
    ) {
    }

    /// Emit the Stage B terminal class and completeness fragment.
    fn foci_velocity_stage_b_terminal_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBTerminalCore,
    ) {
    }

    /// Emit the exact Stage B terminal identity fragment.
    fn foci_velocity_stage_b_terminal_identity(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBTerminalIdentity,
    ) {
    }

    /// Emit one Stage B terminal directional interval fragment.
    fn foci_velocity_stage_b_terminal_interval(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBTerminalInterval,
    ) {
    }

    /// Emit the command-level workflow envelope.
    fn foci_commissioning_workflow_plan(
        &mut self,
        _report: crate::ankyra::replies::FociCommissioningWorkflowPlan,
    ) {
    }

    /// Emit the Stage-C plan identity.
    fn foci_velocity_integral_plan_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralPlanCore,
    ) {
    }

    /// Emit the Stage-C plan geometry.
    fn foci_velocity_integral_plan_geometry(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralPlanGeometry,
    ) {
    }

    /// Emit one direction of Stage-C authority.
    fn foci_velocity_integral_plan_authority(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralPlanAuthority,
    ) {
    }

    /// Emit Stage-C schedule timing.
    fn foci_velocity_integral_plan_timing(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralPlanTiming,
    ) {
    }

    /// Emit Stage-C travel identity.
    fn foci_velocity_integral_plan_travel(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralPlanTravel,
    ) {
    }

    /// Emit Stage-C recovery geometry and reserved schedule.
    fn foci_velocity_integral_plan_recovery(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralPlanRecovery,
    ) {
    }

    /// Emit one exact positive-I plan rung.
    fn foci_velocity_integral_plan_rung(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralPlanRung,
    ) {
    }

    /// Emit Stage-C observation identity.
    fn foci_velocity_integral_observation_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralObservationCore,
    ) {
    }

    /// Emit Stage-C observation rate evidence.
    fn foci_velocity_integral_observation_rate(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralObservationRate,
    ) {
    }

    /// Emit Stage-C observation stationarity quality.
    fn foci_velocity_integral_observation_quality(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralObservationQuality,
    ) {
    }

    /// Emit one Stage-C directional rung verdict.
    fn foci_velocity_integral_rung_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralRungCore,
    ) {
    }

    /// Emit one exact Stage-C consensus component.
    fn foci_velocity_integral_rung_component(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralRungComponent,
    ) {
    }

    /// Emit Stage-C run sufficiency and membership.
    fn foci_velocity_integral_run_summary(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralRunSummary,
    ) {
    }

    /// Emit one exact Stage-C curve interval.
    fn foci_velocity_integral_curve_interval(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralCurveInterval,
    ) {
    }

    /// Emit one direction's opening-to-bookend drift.
    fn foci_velocity_integral_drift(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralDrift,
    ) {
    }

    /// Emit fresh-anchor comparison with retained Stage-B disturbance.
    fn foci_velocity_integral_stage_b_comparison(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralStageBComparison,
    ) {
    }

    /// Emit the Stage-C reproduction class.
    fn foci_velocity_integral_reproduction_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralReproductionCore,
    ) {
    }

    /// Emit one direction's Stage-C reproduction masks.
    fn foci_velocity_integral_reproduction_mask(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralReproductionMask,
    ) {
    }

    /// Emit one reproduced or divergent Stage-C interval comparison.
    fn foci_velocity_integral_reproduction_interval(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralReproductionInterval,
    ) {
    }

    /// Emit exact Stage-C reproduction run digests.
    fn foci_velocity_integral_reproduction_digest(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralReproductionDigest,
    ) {
    }

    /// Emit the Stage-C terminal class and completeness.
    fn foci_velocity_integral_terminal_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralTerminalCore,
    ) {
    }

    /// Emit exact Stage-C terminal identity.
    fn foci_velocity_integral_terminal_identity(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralTerminalIdentity,
    ) {
    }

    /// Emit Stage-C terminal timing.
    fn foci_velocity_integral_terminal_timing(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityIntegralTerminalTiming,
    ) {
    }

    /// Emit one immutable fixed-I confidence-matrix plan.
    fn foci_acceptance_matrix_plan(
        &mut self,
        _report: crate::ankyra::replies::FociAcceptanceMatrixPlan<'_>,
    ) {
    }

    /// Emit one compact fixed-I confidence-matrix terminal.
    fn foci_acceptance_matrix_terminal(
        &mut self,
        _report: crate::ankyra::replies::FociAcceptanceMatrixTerminal<'_>,
    ) {
    }

    /// Emit the immutable breakaway-probe plan.
    fn foci_breakaway_probe_plan(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayProbePlan,
    ) {
    }

    /// Emit the resolved repeatable-motion rung.
    fn foci_breakaway_probe_result(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayProbeResult,
    ) {
    }

    /// Emit a probe-campaign terminal that did not resolve a breakaway.
    fn foci_breakaway_probe_terminal(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayProbeTerminal,
    ) {
    }

    /// Emit the additive discovery plan's identity fragment.
    fn foci_breakaway_discovery_plan_identity(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayDiscoveryPlanIdentity,
    ) {
    }

    /// Emit the additive discovery plan's geometry fragment.
    fn foci_breakaway_discovery_plan_geometry(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayDiscoveryPlanGeometry,
    ) {
    }

    /// Emit which `p_top` candidate bound the additive-discovery authority
    /// ceiling.
    fn foci_breakaway_ceiling_source(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayCeilingSource,
    ) {
    }

    /// Emit whether the additive ladder's rung zero itself showed motion.
    fn foci_breakaway_rung_zero_diagnostic(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayRungZeroDiagnostic,
    ) {
    }

    /// Emit a discovery-campaign terminal.
    fn foci_breakaway_discovery_terminal(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayDiscoveryTerminal,
    ) {
    }

    /// Emit the immutable held-out confirmation plan.
    fn foci_breakaway_confirmation_plan(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayConfirmationPlan,
    ) {
    }

    /// Emit the confirmation terminal's identity fragment.
    fn foci_breakaway_confirmation_terminal_identity(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayConfirmationTerminalIdentity,
    ) {
    }

    /// Emit the confirmation terminal's mask/acceptance fragment.
    fn foci_breakaway_confirmation_terminal_masks(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayConfirmationTerminalMasks,
    ) {
    }

    /// Emit the canonical closure record for one complete breakaway campaign.
    fn foci_breakaway_campaign_terminal(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayCampaignTerminal,
    ) {
    }

    /// Emit the ADC calibration result.
    fn foci_calibrate_result(&mut self, oid: u8, status: u8, result: &AdcCalResult);

    /// Emit one self-test stage result.
    fn foci_selftest_result(&mut self, oid: u8, stage: u8, value: SelftestStageValue);

    /// Emit the terminal self-test completion reply.
    fn foci_selftest_done(&mut self, oid: u8, status: u8);
}

/// Emit the command-level workflow envelope that precedes any Stage-B/C motion.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_commissioning_workflow_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    plan: crate::commissioning::outer::velocity::integral_plan::CommissioningWorkflowPlan,
) {
    let digest = plan.digest();
    sink.foci_commissioning_workflow_plan(crate::ankyra::replies::FociCommissioningWorkflowPlan {
        oid,
        run_sequence: plan.run_sequence,
        shape: plan.shape as u8,
        digest_low: digest as u32,
        digest_high: (digest >> 32) as u32,
        nominal_workflow_ms: plan.nominal_workflow_ms,
        maximum_workflow_ms: plan.maximum_workflow_ms,
    });
}

/// Emit the immutable breakaway-probe plan before the first probe stroke.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_probe_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BreakawayProbePlanned,
) {
    sink.foci_breakaway_probe_plan(crate::ankyra::replies::FociBreakawayProbePlan {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        plan_digest_low: report.plan_digest as u32,
        plan_digest_high: (report.plan_digest >> 32) as u32,
        max_observations: report.max_observations,
        search_count: report.search_count,
        p_start_raw: report.p_start_raw,
        p_top_raw: report.p_top_raw,
        motion_threshold_counts: report.motion_threshold_counts,
        max_capture_interval_us: report.max_capture_interval_us,
    });
}

/// Emit the resolved repeatable-motion rung.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_probe_result<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BreakawayProbeResolved,
) {
    sink.foci_breakaway_probe_result(crate::ankyra::replies::FociBreakawayProbeResult {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        plan_digest_low: report.plan_digest as u32,
        plan_digest_high: (report.plan_digest >> 32) as u32,
        rung_index: report.rung_index,
        breakaway_p_raw: report.breakaway_p_raw,
        motion_threshold_counts: report.motion_threshold_counts,
        observation_count: report.observation_count,
    });
}

/// Emit a probe-campaign terminal that did not resolve a breakaway.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_probe_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BreakawayProbeTerminal,
) {
    sink.foci_breakaway_probe_terminal(crate::ankyra::replies::FociBreakawayProbeTerminal {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        plan_digest_low: report.plan_digest as u32,
        plan_digest_high: (report.plan_digest >> 32) as u32,
        terminal_cause: report.terminal_cause,
        has_safety_fault: report.has_safety_fault,
    });
}

/// Emit the additive discovery plan before the first discovery stroke, split
/// into its identity and geometry fragments.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_additive_discovery_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::AdditiveDiscoveryPlanned,
) {
    sink.foci_breakaway_discovery_plan_identity(
        crate::ankyra::replies::FociBreakawayDiscoveryPlanIdentity {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            plan_digest_low: report.plan_digest as u32,
            plan_digest_high: (report.plan_digest >> 32) as u32,
            prior_plan_digest_low: report.prior_plan_digest as u32,
            prior_plan_digest_high: (report.prior_plan_digest >> 32) as u32,
            family_size: report.family_size,
        },
    );
    sink.foci_breakaway_discovery_plan_geometry(
        crate::ankyra::replies::FociBreakawayDiscoveryPlanGeometry {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            schema_revision: report.schema_revision,
            rung_count: report.rung_count,
            observations_per_direction: report.observations_per_direction,
            floor_p_raw: report.floor_p_raw,
            floor_origin: report.floor_origin,
            breakaway_p_raw: report.breakaway_p_raw,
            ceiling_p_raw: report.ceiling_p_raw,
            first_additive_step_raw: report.first_additive_step_raw,
            maximum_workflow_ms: report.maximum_workflow_ms,
        },
    );
}

/// Emit whether the additive ladder's rung zero itself showed motion.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_rung_zero_diagnostic<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BreakawayRungZeroDiagnostic,
) {
    sink.foci_breakaway_rung_zero_diagnostic(
        crate::ankyra::replies::FociBreakawayRungZeroDiagnostic {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            plan_digest_low: report.plan_digest as u32,
            plan_digest_high: (report.plan_digest >> 32) as u32,
            forward_moved: report.forward_moved,
            reverse_moved: report.reverse_moved,
        },
    );
}

/// Emit a discovery-campaign terminal.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_additive_discovery_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::AdditiveDiscoveryTerminal,
) {
    sink.foci_breakaway_discovery_terminal(
        crate::ankyra::replies::FociBreakawayDiscoveryTerminal {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            plan_digest_low: report.plan_digest as u32,
            plan_digest_high: (report.plan_digest >> 32) as u32,
            prior_plan_digest_low: report.prior_plan_digest as u32,
            prior_plan_digest_high: (report.prior_plan_digest >> 32) as u32,
            family_size: report.family_size,
            terminal_cause: report.terminal_cause,
            collected_count: report.collected_count,
            has_safety_fault: report.has_safety_fault,
        },
    );
}

/// Emit the immutable held-out confirmation plan before the first
/// confirmation stroke.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_band_confirmation_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BandConfirmationPlanned,
) {
    sink.foci_breakaway_confirmation_plan(crate::ankyra::replies::FociBreakawayConfirmationPlan {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        plan_digest_low: report.plan_digest as u32,
        plan_digest_high: (report.plan_digest >> 32) as u32,
        prior_plan_digest_low: report.prior_plan_digest as u32,
        prior_plan_digest_high: (report.prior_plan_digest >> 32) as u32,
        family_size: report.family_size,
        observations_per_direction: report.observations_per_direction,
        candidate_p_raw: report.candidate_p_raw,
        band_lower_percent: report.band_lower_percent,
        band_upper_percent: report.band_upper_percent,
        capture_profile: report.capture_profile,
        acceptance_rule: report.acceptance_rule,
        nominated_margin_percent_milli: report.nominated_margin_percent_milli,
    });
}

/// Emit a confirmation-block terminal, split into its identity and mask fragments.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_band_confirmation_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BandConfirmationTerminal,
) {
    sink.foci_breakaway_confirmation_terminal_identity(
        crate::ankyra::replies::FociBreakawayConfirmationTerminalIdentity {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            plan_digest_low: report.plan_digest as u32,
            plan_digest_high: (report.plan_digest >> 32) as u32,
            prior_plan_digest_low: report.prior_plan_digest as u32,
            prior_plan_digest_high: (report.prior_plan_digest >> 32) as u32,
            family_size: report.family_size,
            terminal_cause: report.terminal_cause,
        },
    );
    sink.foci_breakaway_confirmation_terminal_masks(
        crate::ankyra::replies::FociBreakawayConfirmationTerminalMasks {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            forward_collected_mask: report.forward_collected_mask,
            forward_eligible_mask: report.forward_eligible_mask,
            forward_included_mask: report.forward_included_mask,
            reverse_collected_mask: report.reverse_collected_mask,
            reverse_eligible_mask: report.reverse_eligible_mask,
            reverse_included_mask: report.reverse_included_mask,
            accepted: report.accepted,
            confirmed_p_raw: report.confirmed_p_raw,
            max_relative_se_permille: report.max_relative_se_permille,
            required_relative_se_permille: report.required_relative_se_permille,
            has_safety_fault: report.has_safety_fault,
        },
    );
}

/// Emit the canonical closure record for one complete breakaway campaign.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_campaign_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BreakawayCampaignTerminal,
) {
    sink.foci_breakaway_campaign_terminal(crate::ankyra::replies::FociBreakawayCampaignTerminal {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        phase: report.phase,
        terminal_cause: report.terminal_cause,
        accepted: report.accepted,
        stage_c_plan_digest_low: report.stage_c_plan_digest as u32,
        stage_c_plan_digest_high: (report.stage_c_plan_digest >> 32) as u32,
    });
}

/// Fan one campaign evidence event out to the Task-7 `emit_breakaway_*`
/// bridges, looping over any batch evidence a terminal variant carries (every
/// in-band rung's margin, every resolved ladder rung, every fresh held-out
/// stroke) the same way [`emit_acceptance_matrix_plan`] already loops one
/// Rust value out to several wire fragments.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_campaign_event<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    event: crate::commissioning::types::BreakawayCampaignEvent,
) {
    use crate::commissioning::types::BreakawayCampaignEvent;

    match event {
        BreakawayCampaignEvent::ProbePlanned(report) => {
            emit_breakaway_probe_plan(sink, oid, report)
        }
        BreakawayCampaignEvent::ProbeResolved(report) => {
            emit_breakaway_probe_result(sink, oid, report);
        }
        BreakawayCampaignEvent::ProbeTerminal(report) => {
            emit_breakaway_probe_terminal(sink, oid, report);
        }
        BreakawayCampaignEvent::DiscoveryPlanned {
            planned,
            ceiling_source,
        } => {
            emit_additive_discovery_plan(sink, oid, planned);
            emit_breakaway_ceiling_source(sink, oid, ceiling_source);
        }
        // Per-observation raw evidence and per-rung nomination margins are
        // detailed evidence the host does not consume: they ride the
        // canonical trace only, where foci-trace reconstructs the consensus and
        // nomination from them. No ordinary reply is emitted for either.
        BreakawayCampaignEvent::RawObservation(_) => {}
        BreakawayCampaignEvent::ProbeDecision(_) => {}
        BreakawayCampaignEvent::DiscoveryRungMargin(_) => {}
        BreakawayCampaignEvent::DiscoveryTerminal {
            terminal,
            rung_zero,
        } => {
            if let Some(diagnostic) = rung_zero {
                emit_breakaway_rung_zero_diagnostic(sink, oid, diagnostic);
            }
            emit_additive_discovery_terminal(sink, oid, terminal);
        }
        BreakawayCampaignEvent::ConfirmationPlanned(report) => {
            emit_band_confirmation_plan(sink, oid, report);
        }
        // The per-stroke confirmation observations are detailed trace-only
        // evidence; the ordinary reply carries the terminal (with its
        // mask structure) but not the individual strokes. `forward`/`reverse`
        // are consumed only by the trace encoder.
        BreakawayCampaignEvent::ConfirmationTerminal {
            terminal,
            forward: _,
            reverse: _,
        } => {
            emit_band_confirmation_terminal(sink, oid, terminal);
        }
        BreakawayCampaignEvent::CampaignTerminal(report) => {
            emit_breakaway_campaign_terminal(sink, oid, report);
        }
        // Diagnostic-only: the rejected-rest record exists to explain a
        // `VelocityRestNotConfirmed` terminal offline, and every field it
        // carries is already reconstructible from the FTRC trace stream.
        // Giving it a Klipper reply too would add a wire type to both board
        // sinks and the host extra for evidence no host-side decision reads,
        // so this path deliberately drops it. The campaign terminal the
        // rejection produces still reports over the reply channel as usual.
        BreakawayCampaignEvent::RecoveryRestRejected(_) => {}
        // Diagnostic-only, for the same reason as the rejected-rest record.
        BreakawayCampaignEvent::RecoveryOriginMismatch(_) => {}
    }
}

/// Emit which `p_top` candidate bound the additive-discovery authority
/// ceiling.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_ceiling_source<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BreakawayCeilingSource,
) {
    sink.foci_breakaway_ceiling_source(crate::ankyra::replies::FociBreakawayCeilingSource {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        plan_digest_low: report.plan_digest as u32,
        plan_digest_high: (report.plan_digest >> 32) as u32,
        binding_source: report.binding_source,
    });
}

/// Emit the single compact matrix-plan reply disclosed before motion.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_acceptance_matrix_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    run_sequence: u32,
    plan: crate::commissioning::outer::velocity::acceptance_plan::AcceptanceMatrixPlan,
) {
    use crate::commissioning::outer::velocity::acceptance_reporting::{
        ACCEPTANCE_PLAN_REPLY_FRAGMENT_BYTES, encode_acceptance_plan_reply,
    };
    let payload = encode_acceptance_plan_reply(run_sequence, plan);
    for (fragment, chunk) in payload
        .chunks(ACCEPTANCE_PLAN_REPLY_FRAGMENT_BYTES)
        .enumerate()
    {
        sink.foci_acceptance_matrix_plan(crate::ankyra::replies::FociAcceptanceMatrixPlan {
            oid,
            fragment: fragment as u8,
            payload: chunk,
        });
    }
}

/// Emit the single compact matrix terminal.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_acceptance_matrix_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    terminal: crate::commissioning::types::AcceptanceMatrixTerminal,
) {
    let payload = crate::commissioning::outer::velocity::acceptance_reporting::
        encode_acceptance_terminal_reply(terminal);
    sink.foci_acceptance_matrix_terminal(crate::ankyra::replies::FociAcceptanceMatrixTerminal {
        oid,
        payload: &payload,
    });
}

/// Emit the complete exact Stage-C plan as bounded causal fragments.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_velocity_integral_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    run_sequence: u32,
    plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
) {
    use crate::ankyra::replies::{
        FociVelocityIntegralPlanAuthority, FociVelocityIntegralPlanCore,
        FociVelocityIntegralPlanGeometry, FociVelocityIntegralPlanRecovery,
        FociVelocityIntegralPlanRung, FociVelocityIntegralPlanTiming,
        FociVelocityIntegralPlanTravel,
    };

    let digest = plan.digest();
    let sequence = 0;
    sink.foci_velocity_integral_plan_core(FociVelocityIntegralPlanCore {
        oid,
        run_sequence,
        evidence_sequence: sequence,
        fragment: 0,
        plan_digest_low: digest as u32,
        plan_digest_high: (digest >> 32) as u32,
        stage_b_digest_low: plan.authority.acquisition.digest() as u32,
        stage_b_digest_high: (plan.authority.acquisition.digest() >> 32) as u32,
        build_features: plan.build_features,
        build_identity: crate::commissioning::outer::velocity::reproduction::BUILD_IDENTITY,
        schema_revision: plan.schema_revision,
        channel: plan.authority.channel,
        final_p: plan.final_p,
    });
    sink.foci_velocity_integral_plan_geometry(FociVelocityIntegralPlanGeometry {
        oid,
        run_sequence,
        evidence_sequence: sequence,
        fragment: 1,
        planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
        target_velocity_rpm: plan.target_velocity_rpm,
        pwm_hz: plan.pwm_hz,
        encoder_counts_per_rev: plan.encoder_counts_per_rev,
        i_start: plan.positive_i[0],
        positive_rung_count: plan.positive_rung_count,
        family_size: plan.family_size,
        total_rung_count: plan.total_rung_count,
        expected_observations: plan.expected_observations,
        hard_torque_limit: plan.hard_torque_limit,
        usable_torque_limit: plan.usable_torque_limit,
    });
    for direction in 0..2 {
        let pooled_interval = plan.authority.directional_pooled_intervals[direction];
        sink.foci_velocity_integral_plan_authority(FociVelocityIntegralPlanAuthority {
            oid,
            run_sequence,
            evidence_sequence: sequence,
            direction: direction as u8,
            directional_membership: plan.authority.directional_memberships[direction],
            joint_membership: plan.authority.joint_membership,
            directional_validity: plan.authority.directional_validity[direction] as u8,
            reduced_margin: u8::from(plan.authority.reduced_margin),
            pooled_low_q16: pooled_interval[0],
            pooled_high_q16: pooled_interval[1],
        });
    }
    sink.foci_velocity_integral_plan_timing(FociVelocityIntegralPlanTiming {
        oid,
        run_sequence,
        evidence_sequence: sequence,
        fragment: 2,
        moving_stroke_us: plan.moving_stroke_us,
        zero_settle_us: plan.zero_settle_us,
        analysis_budget_us: plan.analysis_budget_us,
        nominal_workflow_ms: plan.nominal_workflow_ms,
        maximum_workflow_ms: plan.maximum_workflow_ms,
    });
    sink.foci_velocity_integral_plan_travel(FociVelocityIntegralPlanTravel {
        oid,
        run_sequence,
        evidence_sequence: sequence,
        fragment: 3,
        max_stroke_travel_mrev: plan.motion_request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: plan.motion_request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: plan.motion_request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: plan.motion_request.positive_position_headroom_mrev,
    });
    sink.foci_velocity_integral_plan_recovery(FociVelocityIntegralPlanRecovery {
        oid,
        run_sequence,
        evidence_sequence: sequence,
        origin_band_counts: plan.recovery_origin_band_counts,
        nominal_slot_us: plan.recovery_nominal_slot_us,
        maximum_slot_us: plan.recovery_maximum_slot_us,
        slot_count: plan.recovery_slot_count,
        flags: crate::commissioning::outer::velocity::integral_reporting::plan_recovery_flags(
            &plan,
        ),
    });
    for (rung_index, i_raw) in plan.positive_rungs().iter().copied().enumerate() {
        let tau_us = crate::commissioning::outer::velocity::integral_plan::velocity_i_tau_us(
            i_raw,
            plan.pwm_hz,
        )
        .and_then(|value| u32::try_from(value).ok())
        .unwrap_or(u32::MAX);
        sink.foci_velocity_integral_plan_rung(FociVelocityIntegralPlanRung {
            oid,
            run_sequence,
            evidence_sequence: sequence,
            rung_index: rung_index as u8,
            i_raw,
            tau_us,
        });
    }
}

/// Emit one Stage-C observation or rung group.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_velocity_integral_event<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    event: crate::commissioning::types::VelocityIntegralEvent,
) {
    match event {
        crate::commissioning::types::VelocityIntegralEvent::Planned(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::Observation(report) => {
            emit_velocity_integral_observation(sink, oid, report);
        }
        crate::commissioning::types::VelocityIntegralEvent::Rung(report) => {
            emit_velocity_integral_rung(sink, oid, report);
        }
        crate::commissioning::types::VelocityIntegralEvent::Recovery(report) => {
            emit_rung_origin_recovery(sink, oid, report);
        }
        crate::commissioning::types::VelocityIntegralEvent::RecoveryRest(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::RestSelection(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::Current(_) => {}
    }
}

#[cfg(feature = "trace-velocity-sweep")]
fn emit_velocity_integral_observation<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::VelocityIntegralObservation,
) {
    use crate::ankyra::replies::{
        FociVelocityIntegralObservationCore, FociVelocityIntegralObservationQuality,
        FociVelocityIntegralObservationRate,
    };
    use crate::autotune::velocity_measurement::StationarityResult;
    use crate::commissioning::outer::velocity::integral_reporting::{
        direction_code, integral_observation_class, observation_class_code,
        stationarity_exclusion_code,
    };

    sink.foci_velocity_integral_observation_core(FociVelocityIntegralObservationCore {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 0,
        plan_digest_low: report.plan_digest as u32,
        plan_digest_high: (report.plan_digest >> 32) as u32,
        rung_index: report.rung_index,
        slot: report.slot,
        i_raw: report.i_raw,
        direction: direction_code(report.summary.direction),
        classification: observation_class_code(integral_observation_class(report)),
        initial_lead_in: u8::from(report.summary.initial_lead_in),
        peak_torque_target_abs: report.summary.predicted_torque_target_abs,
    });
    let (rates, shift) = encode_rate_values([
        report.summary.settled_rate_interval_q[0],
        report.summary.settled_rate_mean_q,
        report.summary.settled_rate_interval_q[1],
        0,
        0,
    ]);
    sink.foci_velocity_integral_observation_rate(FociVelocityIntegralObservationRate {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 1,
        settled_low: rates[0],
        settled_mean: rates[1],
        settled_high: rates[2],
        settled_shift: shift,
        deficit_low_q: i32::try_from(report.summary.deficit_interval_q[0])
            .expect("Stage-C evidence gate narrows deficit bounds before reporting"),
        deficit_high_q: i32::try_from(report.summary.deficit_interval_q[1])
            .expect("Stage-C evidence gate narrows deficit bounds before reporting"),
    });
    let quality = match report.stationarity {
        StationarityResult::Usable(suffix) => {
            let (slope_mantissa, slope_shift) = upper_bound_i64(suffix.slope_q);
            let (slope_half_width_mantissa, slope_half_width_shift) =
                upper_bound_u128(u128::from(suffix.slope_half_width_q));
            let (residual_mantissa, residual_shift) =
                upper_bound_u128(suffix.residual_magnitude_q2);
            FociVelocityIntegralObservationQuality {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                fragment: 2,
                suffix_len: suffix.suffix_len,
                level_or_exclusion: suffix.selection.level,
                selected_blocks: suffix.selection.block_count,
                slope_mantissa,
                slope_shift,
                slope_half_width_mantissa,
                slope_half_width_shift,
                lag_one_q: suffix.lag_one_q,
                lag_one_half_width_q: suffix.lag_one_half_width_q,
                residual_mantissa,
                residual_shift,
                tested_suffixes: suffix.tested_suffixes,
            }
        }
        StationarityResult::Excluded(exclusion) => FociVelocityIntegralObservationQuality {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            fragment: 2,
            suffix_len: 0,
            level_or_exclusion: 0x80 | stationarity_exclusion_code(exclusion),
            selected_blocks: 0,
            slope_mantissa: 0,
            slope_shift: 0,
            slope_half_width_mantissa: 0,
            slope_half_width_shift: 0,
            lag_one_q: 0,
            lag_one_half_width_q: 0,
            residual_mantissa: 0,
            residual_shift: 0,
            tested_suffixes: 0,
        },
    };
    sink.foci_velocity_integral_observation_quality(quality);
}

#[cfg(feature = "trace-velocity-sweep")]
fn emit_velocity_integral_rung<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::VelocityIntegralRungReport,
) {
    use crate::commissioning::outer::velocity::integral_reporting::{
        direction_code, observation_class_code, rung_class_code, rung_kind_code,
    };

    for direction in report.directions {
        let packed_classes = direction.observation_classes.iter().enumerate().fold(
            0_u16,
            |packed, (index, class)| {
                packed | (u16::from(observation_class_code(*class)) << (index * 3))
            },
        );
        sink.foci_velocity_integral_rung_core(
            crate::ankyra::replies::FociVelocityIntegralRungCore {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                fragment: direction_code(direction.direction),
                rung_index: report.rung_index,
                i_raw: report.i_raw,
                direction: direction_code(direction.direction),
                kind: rung_kind_code(direction.kind),
                classification: rung_class_code(direction.classification),
                component_count: direction.component_count,
                usable_mask: direction.usable_mask,
                included_mask: direction.included_mask,
                observation_classes: packed_classes,
            },
        );
        for component in 0..usize::from(direction.component_count) {
            let interval = direction.components[component];
            sink.foci_velocity_integral_rung_component(
                crate::ankyra::replies::FociVelocityIntegralRungComponent {
                    oid,
                    run_sequence: report.run_sequence,
                    evidence_sequence: report.evidence_sequence,
                    rung_index: report.rung_index,
                    direction: direction_code(direction.direction),
                    component: component as u8,
                    low_q: interval.low_q,
                    high_q: interval.high_q,
                },
            );
        }
    }
}

/// Emit Stage-C derived evidence, optional reproduction, and terminal fragments.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_velocity_integral_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    terminal: crate::commissioning::types::VelocityIntegralTerminal,
    reproduction: Option<crate::commission_dispatch::stage_c::StageCReproductionReport>,
    authority: Option<
        crate::commissioning::outer::velocity::integral_reproduction::StageCAuthoritySummary,
    >,
) {
    use crate::ankyra::replies::{
        FociVelocityIntegralCurveInterval, FociVelocityIntegralDrift,
        FociVelocityIntegralReproductionCore, FociVelocityIntegralReproductionDigest,
        FociVelocityIntegralReproductionInterval, FociVelocityIntegralReproductionMask,
        FociVelocityIntegralRunSummary, FociVelocityIntegralStageBComparison,
    };
    use crate::commissioning::outer::velocity::integral_plan::STAGE_C_MAX_POSITIVE_RUNGS;
    use crate::commissioning::outer::velocity::integral_reporting::stage_b_deficit_bounds_q;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        closed_intersection, delta_e, signed_divergence_gap,
    };

    let sequence = terminal.evidence_sequence;
    let sufficient_direction_mask = terminal
        .evidence
        .eligible_masks
        .iter()
        .enumerate()
        .fold(0_u8, |mask, (direction, members)| {
            mask | (u8::from(members.count_ones() >= 3) << direction)
        });
    let bookend_available_mask = terminal
        .evidence
        .bookend
        .iter()
        .enumerate()
        .fold(0_u8, |mask, (direction, interval)| {
            mask | (u8::from(interval.is_some()) << direction)
        });
    let opening_available_mask =
        terminal
            .evidence
            .opening
            .iter()
            .enumerate()
            .fold(0_u8, |mask, (direction, interval)| {
                mask
                    | (u8::from(
                        *interval
                            != crate::commissioning::outer::velocity::integral_evidence::ExactRateInterval::EMPTY,
                    ) << direction)
            });
    sink.foci_velocity_integral_run_summary(FociVelocityIntegralRunSummary {
        oid,
        run_sequence: terminal.run_sequence,
        evidence_sequence: sequence,
        fragment: 0,
        forward_eligible_mask: terminal.evidence.eligible_masks[0],
        reverse_eligible_mask: terminal.evidence.eligible_masks[1],
        opening_available_mask,
        bookend_available_mask,
        current_terminus_plus_one: terminal
            .evidence
            .current_terminus
            .map_or(0, |rung| rung + 1),
        sufficient_direction_mask,
    });
    for direction in 0..2 {
        let opening = terminal.evidence.opening[direction];
        sink.foci_velocity_integral_curve_interval(FociVelocityIntegralCurveInterval {
            oid,
            run_sequence: terminal.run_sequence,
            evidence_sequence: sequence,
            kind: 0,
            direction: direction as u8,
            rung_index: 0,
            low_q: opening.low_q,
            high_q: opening.high_q,
        });
        for rung in 0..STAGE_C_MAX_POSITIVE_RUNGS {
            if terminal.evidence.eligible_masks[direction] & (1_u32 << rung) == 0 {
                continue;
            }
            let interval = terminal.evidence.positive[direction][rung];
            sink.foci_velocity_integral_curve_interval(FociVelocityIntegralCurveInterval {
                oid,
                run_sequence: terminal.run_sequence,
                evidence_sequence: sequence,
                kind: 1,
                direction: direction as u8,
                rung_index: rung as u8,
                low_q: interval.low_q,
                high_q: interval.high_q,
            });
        }
        let bookend = terminal.evidence.bookend[direction];
        if let Some(interval) = bookend {
            sink.foci_velocity_integral_curve_interval(FociVelocityIntegralCurveInterval {
                oid,
                run_sequence: terminal.run_sequence,
                evidence_sequence: sequence,
                kind: 2,
                direction: direction as u8,
                rung_index: 0,
                low_q: interval.low_q,
                high_q: interval.high_q,
            });
        }
        sink.foci_velocity_integral_drift(FociVelocityIntegralDrift {
            oid,
            run_sequence: terminal.run_sequence,
            evidence_sequence: sequence,
            direction: direction as u8,
            available: u8::from(bookend.is_some()),
            opening_low_q: opening.low_q,
            opening_high_q: opening.high_q,
            closing_low_q: bookend.map_or(0, |interval| interval.low_q),
            closing_high_q: bookend.map_or(0, |interval| interval.high_q),
        });
        let model_comparison = authority
            .filter(|authority| authority.model_interval_available(direction))
            .and_then(|authority| {
                stage_b_deficit_bounds_q(
                    authority.directional_pooled_intervals[direction],
                    authority.final_p,
                    authority.encoder_counts_per_rev,
                )
                .map(|expected| (authority, expected))
            });
        let (stage_b_interval, expected, available) =
            model_comparison.map_or(([0; 2], [0; 2], 0), |(authority, expected)| {
                (
                    authority.directional_pooled_intervals[direction],
                    expected,
                    1,
                )
            });
        sink.foci_velocity_integral_stage_b_comparison(FociVelocityIntegralStageBComparison {
            oid,
            run_sequence: terminal.run_sequence,
            evidence_sequence: sequence,
            direction: direction as u8,
            available,
            stage_b_low_q16: stage_b_interval[0],
            stage_b_high_q16: stage_b_interval[1],
            expected_low_q: expected[0],
            expected_high_q: expected[1],
            opening_low_q: opening.low_q,
            opening_high_q: opening.high_q,
        });
    }
    if let Some(reproduction) = reproduction {
        let direction_mask =
            reproduction.delta.reproduced_masks.iter().enumerate().fold(
                0_u8,
                |mask, (direction, members)| {
                    mask | (u8::from(members.count_ones() >= 3) << direction)
                },
            );
        sink.foci_velocity_integral_reproduction_core(FociVelocityIntegralReproductionCore {
            oid,
            run_sequence: terminal.run_sequence,
            evidence_sequence: sequence,
            outcome: reproduction.outcome,
            direction_mask,
        });
        for direction in 0..2 {
            sink.foci_velocity_integral_reproduction_mask(FociVelocityIntegralReproductionMask {
                oid,
                run_sequence: terminal.run_sequence,
                evidence_sequence: sequence,
                direction: direction as u8,
                previous_mask: reproduction.delta.previous_masks[direction],
                current_mask: reproduction.delta.current_masks[direction],
                shared_mask: reproduction.delta.shared_masks[direction],
                reproduced_mask: reproduction.delta.reproduced_masks[direction],
                divergent_mask: reproduction.delta.divergent_masks[direction],
                previous_only_mask: reproduction.delta.previous_only_masks[direction],
                current_only_mask: reproduction.delta.current_only_masks[direction],
            });
            for rung in 0..STAGE_C_MAX_POSITIVE_RUNGS {
                let bit = 1_u32 << rung;
                let kind = if reproduction.delta.reproduced_masks[direction] & bit != 0 {
                    1
                } else if reproduction.delta.divergent_masks[direction] & bit != 0 {
                    2
                } else {
                    continue;
                };
                let previous = delta_e(
                    reproduction.delta.previous_evidence.positive[direction][rung],
                    reproduction.delta.previous_evidence.opening[direction],
                )
                .expect("retained reproduction interval was validated before reporting");
                let current = delta_e(
                    terminal.evidence.positive[direction][rung],
                    terminal.evidence.opening[direction],
                )
                .expect("current reproduction interval was validated before reporting");
                let overlap = closed_intersection(previous, current);
                let signed_gap = signed_divergence_gap(previous, current)
                    .expect("reproduction gap was validated before reporting");
                sink.foci_velocity_integral_reproduction_interval(
                    FociVelocityIntegralReproductionInterval {
                        oid,
                        run_sequence: terminal.run_sequence,
                        evidence_sequence: sequence,
                        direction: direction as u8,
                        rung_index: rung as u8,
                        kind,
                        previous_low_q: previous.low_q,
                        previous_high_q: previous.high_q,
                        current_low_q: current.low_q,
                        current_high_q: current.high_q,
                        result_low_or_gap_q: if kind == 1 {
                            overlap
                                .expect("reproduced mask requires a closed overlap")
                                .low_q
                        } else {
                            signed_gap
                        },
                        result_high_q: overlap.map_or(0, |interval| interval.high_q),
                    },
                );
            }
        }
        sink.foci_velocity_integral_reproduction_digest(FociVelocityIntegralReproductionDigest {
            oid,
            run_sequence: terminal.run_sequence,
            evidence_sequence: sequence,
            previous_digest_low: reproduction.previous_run_digest as u32,
            previous_digest_high: (reproduction.previous_run_digest >> 32) as u32,
            current_digest_low: reproduction.current_run_digest as u32,
            current_digest_high: (reproduction.current_run_digest >> 32) as u32,
        });
    }
    if let Some(fault) = terminal.safety_fault {
        sink.foci_outer_safety_fault(oid, fault);
    }
    emit_velocity_integral_terminal_fragments(sink, oid, terminal);
}

/// Emit the three fragments that carry one Stage-C terminal itself.
///
/// Split from [`emit_velocity_integral_terminal`] so a request refused before
/// any exact plan can report its terminal without the derived evidence
/// fragments, which would otherwise describe a run that never happened.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_velocity_integral_terminal_fragments<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    terminal: crate::commissioning::types::VelocityIntegralTerminal,
) {
    use crate::ankyra::replies::{
        FociVelocityIntegralTerminalCore, FociVelocityIntegralTerminalIdentity,
        FociVelocityIntegralTerminalTiming,
    };
    use crate::commissioning::outer::velocity::integral_reporting::outcome_code;

    let sequence = terminal.evidence_sequence;
    sink.foci_velocity_integral_terminal_core(FociVelocityIntegralTerminalCore {
        oid,
        run_sequence: terminal.run_sequence,
        evidence_sequence: sequence,
        fragment: 0,
        outcome: outcome_code(terminal.outcome),
        cause: terminal.cause,
        recovery_flags:
            crate::commissioning::outer::velocity::integral_reporting::terminal_recovery_flags(
                &terminal,
            ),
        rest_boundary_rung_plus_one: terminal.rest_boundary_rung_plus_one,
        rest_boundary_slot_plus_one: terminal.rest_boundary_slot_plus_one,
        expected_observations: terminal.expected_observations,
        emitted_observations: terminal.emitted_observations,
        expected_rungs: terminal.expected_rungs,
        emitted_rungs: terminal.emitted_rungs,
    });
    sink.foci_velocity_integral_terminal_identity(FociVelocityIntegralTerminalIdentity {
        oid,
        run_sequence: terminal.run_sequence,
        evidence_sequence: sequence,
        fragment: 1,
        plan_digest_low: terminal.plan_digest as u32,
        plan_digest_high: (terminal.plan_digest >> 32) as u32,
        digest_low: terminal.digest as u32,
        digest_high: (terminal.digest >> 32) as u32,
    });
    sink.foci_velocity_integral_terminal_timing(FociVelocityIntegralTerminalTiming {
        oid,
        run_sequence: terminal.run_sequence,
        evidence_sequence: sequence,
        fragment: 2,
        started_low: terminal.run_started_us as u32,
        started_high: (terminal.run_started_us >> 32) as u32,
        completed_low: terminal.run_completed_us as u32,
        completed_high: (terminal.run_completed_us >> 32) as u32,
    });
}

#[cfg(feature = "trace-velocity-sweep")]
fn emit_rung_origin_recovery<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    evidence: crate::commissioning::types::RungOriginRecoveryEvidence,
) {
    use crate::ankyra::replies::FociRungOriginRecoverySummary;

    let report = evidence.report;
    sink.foci_rung_origin_recovery_summary(FociRungOriginRecoverySummary {
        oid,
        run_sequence: evidence.run_sequence,
        evidence_sequence: evidence.evidence_sequence,
        stage: report.stage as u8,
        rung_index: report.rung_index,
        p_raw: report.p_raw,
        binding_source: report.binding_source as u8,
        outcome: report.outcome as u8,
    });
}

fn emit_velocity_sweep_terminal_fragments<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: VelocitySweepTerminal,
) {
    use crate::ankyra::replies::{
        FociVelocityStageBTerminalCore, FociVelocityStageBTerminalIdentity,
        FociVelocityStageBTerminalInterval,
    };
    use crate::commissioning::types::VelocitySweepOutcome;

    let evidence_sequence = report.evidence_sequence;
    let outcome = match report.outcome {
        VelocitySweepOutcome::CompleteCandidate => 0,
        VelocitySweepOutcome::Complete => 1,
        VelocitySweepOutcome::Inconclusive => 2,
        VelocitySweepOutcome::Fault => 3,
        VelocitySweepOutcome::RejectedPlanMismatch => 4,
    };
    sink.foci_velocity_stage_b_terminal_core(FociVelocityStageBTerminalCore {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence,
        fragment: 0,
        outcome,
        cause: report.cause,
        recovery_unavailable: u8::from(report.recovery_unavailable),
        model_direction_mask: report.model_direction_mask,
        coverage_mask: report.coverage_mask,
        forward_region_count: report.directional_region_counts[0],
        reverse_region_count: report.directional_region_counts[1],
        forward_fragment_count: report.directional_fragment_counts[0],
        reverse_fragment_count: report.directional_fragment_counts[1],
        expected_observations: report.expected_observations,
        emitted_observations: report.emitted_observations,
        expected_rungs: report.expected_rungs,
        emitted_rungs: report.emitted_rungs,
    });
    sink.foci_velocity_stage_b_terminal_identity(FociVelocityStageBTerminalIdentity {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence,
        fragment: 1,
        forward_member_mask: report.selected_memberships[0],
        reverse_member_mask: report.selected_memberships[1],
        joint_member_mask: report.selected_memberships[2],
        nominated_p: report.nominated_p,
        plan_digest_low: report.plan_digest as u32,
        plan_digest_high: (report.plan_digest >> 32) as u32,
        digest_low: report.digest as u32,
        digest_high: (report.digest >> 32) as u32,
    });
    for direction in 0..2 {
        let interval = report.selected_intervals[usize::from(direction)];
        sink.foci_velocity_stage_b_terminal_interval(FociVelocityStageBTerminalInterval {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence,
            fragment: direction + 2,
            direction,
            pooled_low_q16: interval[0],
            pooled_high_q16: interval[1],
            started_low: report.run_started_us as u32,
            started_high: (report.run_started_us >> 32) as u32,
            completed_low: report.run_completed_us as u32,
            completed_high: (report.run_completed_us >> 32) as u32,
        });
    }
}

/// Emit all Klipper replies represented by one terminal commissioning payload.
pub fn emit_terminal_reply<S: CommissionReplySink>(sink: &mut S, payload: ReplyPayload) {
    match payload {
        ReplyPayload::CommissionPhase {
            oid,
            phase,
            status,
            diagnostics,
        } => {
            emit_commission_diagnostics(sink, oid, diagnostics);
            sink.foci_commission_phase(oid, phase, status);
        }
        ReplyPayload::CalibrateResult {
            oid,
            status,
            result,
            diagnostics,
        } => {
            emit_commission_diagnostics(sink, oid, diagnostics);
            sink.foci_calibrate_result(oid, status, &result);
        }
        ReplyPayload::CommissionResult {
            oid,
            status,
            result,
        } => {
            // FociCommissionResult is size-capped (23 fields, near the
            // 64-byte Klipper message limit) and cannot carry the evidence
            // that lives alongside `InnerResult`. Emit supplemental evidence
            // before the terminal commission result so the result stays the
            // true terminal/"done" signal for the whole commission flow.
            //
            // Resistance axes stay before the resistance run, mirroring the
            // diagnostic path's axes-then-run ordering (`emit_diagnostic_report`).
            let inner = &result.inner;
            emit_adc_residual_evidence(sink, oid, inner);
            sink.foci_resistance_axis(commission_resistance_axis_report(
                oid,
                0,
                RESISTANCE_AXIS0_PHI_E_EXT,
                inner,
            ));
            sink.foci_resistance_axis(commission_resistance_axis_report(
                oid,
                1,
                RESISTANCE_AXIS1_PHI_E_EXT,
                inner,
            ));
            sink.foci_resistance_run(commission_resistance_run_report(oid, inner));
            sink.foci_encoder_alignment(commission_encoder_alignment_report(oid, inner));
            emit_inductance_evidence(sink, oid, inner);
            emit_current_loop_evidence(sink, oid, status, &result);
            for (method, summary) in inner.timing.iter().copied().enumerate() {
                sink.foci_commission_timing(commission_timing_report(oid, method as u8, summary));
            }
            sink.foci_commission_result(oid, status, &result);
        }
        ReplyPayload::TuneResult {
            oid,
            status,
            result,
            safety_fault,
        } => {
            if let Some(fault) = safety_fault {
                sink.foci_outer_safety_fault(oid, fault);
            }
            sink.foci_tune_result(oid, status, &result);
        }
        ReplyPayload::Selftest {
            oid,
            results,
            diagnostics,
            done_status,
        } => {
            emit_commission_diagnostics(sink, oid, diagnostics);
            for (stage, value) in results.iter() {
                sink.foci_selftest_result(oid, stage, value);
            }
            sink.foci_selftest_done(oid, done_status);
        }
        ReplyPayload::VelocitySweepTerminal { oid, terminal } => {
            if let Some(fault) = terminal.safety_fault {
                sink.foci_outer_safety_fault(oid, fault);
            }
            emit_velocity_sweep_terminal_fragments(sink, oid, terminal);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::CommissioningWorkflowPlan { oid, plan } => {
            emit_commissioning_workflow_plan(sink, oid, plan);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::VelocityIntegralPlan {
            oid,
            run_sequence,
            plan,
        } => emit_velocity_integral_plan(sink, oid, run_sequence, plan),
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::VelocityIntegralEvent { oid, event } => {
            emit_velocity_integral_event(sink, oid, event);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::VelocityIntegralTerminal {
            oid,
            terminal,
            reproduction,
            authority,
        } => emit_velocity_integral_terminal(sink, oid, terminal, reproduction, authority),
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::VelocityIntegralRefusal { oid, terminal } => {
            emit_velocity_integral_terminal_fragments(sink, oid, terminal);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::AcceptanceMatrixPlan {
            oid,
            run_sequence,
            plan,
        } => emit_acceptance_matrix_plan(sink, oid, run_sequence, plan),
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::AcceptanceMatrixTerminal { oid, terminal } => {
            emit_acceptance_matrix_terminal(sink, oid, terminal);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::BreakawayCampaignEvent { oid, event } => {
            emit_breakaway_campaign_event(sink, oid, event);
        }
    }
}

fn commission_timing_report(
    oid: u8,
    method: u8,
    summary: crate::commissioning::types::CommissionTimingSummary,
) -> CommissionTimingReport {
    let evidence = summary.evidence;
    CommissionTimingReport {
        oid,
        method,
        status: summary.status,
        requested_period_us: evidence.requested_period_us,
        valid_samples: evidence.valid_samples,
        missed_samples: evidence.missed_samples,
        max_consecutive_misses: evidence.max_consecutive_misses,
        max_lateness_us: evidence.max_lateness_us,
        max_interval_us: evidence.max_interval_us,
        max_poll_wall_us: evidence.max_poll_wall_us,
        max_spi_wall_us: evidence.max_spi_wall_us,
    }
}

/// Emit a phase-transition reply with success status.
pub fn emit_phase_reply<S: CommissionReplySink>(sink: &mut S, oid: u8, phase_id: PhaseId) {
    sink.foci_commission_phase(oid, phase_id.to_u8(), 0);
}

fn emit_commission_diagnostics<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    diagnostics: CommissionDiagnostics,
) {
    for priority in [true, false] {
        for event in diagnostics.iter() {
            let is_electrical_timing =
                event.phase == PhaseId::ElectricalId.to_u8() && matches!(event.code, 34..=41);
            if is_electrical_timing == priority {
                sink.foci_commission_detail(oid, event);
            }
        }
    }
}

fn emit_adc_residual_evidence<S: CommissionReplySink>(sink: &mut S, oid: u8, inner: &InnerResult) {
    sink.foci_adc_residual(adc_residual_report(oid, inner.adc_residual_chop0));
    sink.foci_adc_residual(adc_residual_report(oid, inner.adc_residual_chop7));
}

fn adc_residual_report(oid: u8, evidence: AdcResidualEvidence) -> AdcResidualReport {
    AdcResidualReport {
        oid,
        stage: evidence.stage,
        sample_count: evidence.sample_count,
        pwm_sv_chop: evidence.pwm_sv_chop,
        pwm_bbm: evidence.pwm_bbm,
        adc_i0_scale_offset: evidence.adc_i0_scale_offset,
        adc_i1_scale_offset: evidence.adc_i1_scale_offset,
        adc_iux_mean_count: evidence.adc_iux_mean_count,
        adc_iwy_mean_count: evidence.adc_iwy_mean_count,
        pid_flux_mean_count: evidence.pid_flux_mean_count,
        pid_torque_mean_count: evidence.pid_torque_mean_count,
    }
}

/// Build the resistance-identification run report carried by the
/// commission flow's `foci_resistance_run` emission.
///
/// `status` is always 0: this report is only built once `InnerResult` has
/// already reached `CommissionDone`, which implies the resistance run
/// that produced it succeeded. Power-stage readback fields
/// (`pwm_maxcnt_readback`, `bbm_readback`, `dsadc_mdec_readback`,
/// `pwm_sv_chop_readback`) have no counterpart on `InnerResult` and are set
/// to 0; only the standalone diagnostic path (`foci_resistance_test`)
/// retains those readbacks.
fn commission_resistance_run_report(oid: u8, inner: &InnerResult) -> ResistanceRunReport {
    ResistanceRunReport {
        oid,
        status: 0,
        selected_r_count_slope_milli: inner.resistance_selected_count_slope_milli,
        gain_path_count_slope_milli: inner.resistance_gain_path_count_slope_milli,
        warning_flags: inner.resistance_warning_flags,
        status_flags_or: inner.resistance_status_flags_or,
        peak_abs_current_count: inner.resistance_peak_abs_current_count,
        max_abs_steady_mean_current_count: inner.resistance_max_abs_steady_mean_current_count,
        current_ceiling_count: inner.resistance_current_ceiling_count,
        power_stage_tripped: inner.resistance_power_stage_tripped,
        pwm_maxcnt_readback: 0,
        bbm_readback: 0,
        dsadc_mdec_readback: 0,
        pwm_sv_chop_readback: 0,
    }
}

/// Build one electrical-axis resistance fit report carried by the
/// commission flow's `foci_resistance_axis` emission.
///
/// `excluded_point_mask`, `selected_count`, and per-axis `warning_flags`
/// have no counterpart on `InnerResult` (only a run-level
/// `resistance_warning_flags` is retained) and are set to 0.
fn commission_resistance_axis_report(
    oid: u8,
    electrical_axis: u8,
    phi_e_ext: u16,
    inner: &InnerResult,
) -> ResistanceAxisReport {
    let (r_count_slope_milli, intercept_count, rmse_permille, selected_mask) = match electrical_axis
    {
        0 => (
            inner.resistance_axis0_count_slope_milli,
            inner.resistance_axis0_intercept_count,
            inner.resistance_axis0_rmse_permille,
            inner.resistance_selected_mask_axis0,
        ),
        _ => (
            inner.resistance_axis1_count_slope_milli,
            inner.resistance_axis1_intercept_count,
            inner.resistance_axis1_rmse_permille,
            inner.resistance_selected_mask_axis1,
        ),
    };
    let (signed_count_slope_milli, signed_asymmetry_permille, drift_permille) =
        match electrical_axis {
            0 => (
                inner.resistance_axis0_signed_count_slope_milli,
                inner.resistance_axis0_signed_asymmetry_permille,
                inner.resistance_axis0_drift_permille,
            ),
            _ => (
                inner.resistance_axis1_signed_count_slope_milli,
                inner.resistance_axis1_signed_asymmetry_permille,
                inner.resistance_axis1_drift_permille,
            ),
        };

    ResistanceAxisReport {
        oid,
        electrical_axis,
        phi_e_ext,
        r_count_slope_milli,
        intercept_count,
        rmse_permille,
        selected_mask,
        excluded_point_mask: 0,
        selected_count: 0,
        signed_count_slope_milli,
        signed_asymmetry_permille,
        drift_permille,
        warning_flags: 0,
    }
}

fn commission_encoder_alignment_report(oid: u8, inner: &InnerResult) -> EncoderAlignmentReport {
    EncoderAlignmentReport {
        oid,
        encoder_count: inner.encoder_alignment_count,
        electrical_residual_counts: inner.encoder_alignment_electrical_residual_counts,
        stability_counts: inner.encoder_alignment_stability_counts,
        movement_counts: inner.encoder_alignment_movement_counts,
        min_movement_counts: inner.encoder_alignment_min_movement_counts,
        counts_per_electrical_rev: inner.encoder_alignment_counts_per_electrical_rev,
    }
}

fn emit_inductance_evidence<S: CommissionReplySink>(sink: &mut S, oid: u8, inner: &InnerResult) {
    let evidence = inner.inductance;
    if evidence.run.source == INDUCTANCE_SOURCE_NONE || evidence.run.sample_count == 0 {
        return;
    }

    sink.foci_inductance_run(InductanceRunReport {
        oid,
        source: evidence.run.source,
        status: evidence.run.status,
        warning_flags: evidence.run.warning_flags,
        ud_count: evidence.run.ud_count,
        realized_frequency_millihz: evidence.run.realized_frequency_millihz,
        elapsed_us: evidence.run.elapsed_us,
        openloop_phi_delta_counts: evidence.run.openloop_phi_delta_counts,
        sample_count: evidence.run.sample_count,
        encoder_delta_counts: evidence.run.encoder_delta_counts,
        status_flags_or: evidence.run.status_flags_or,
    });
    sink.foci_inductance_frame(InductanceFrameReport {
        oid,
        id_mean_milli_count: evidence.frame.id_mean_milli_count,
        iq_mean_milli_count: evidence.frame.iq_mean_milli_count,
        id_rms_milli_count: evidence.frame.id_rms_milli_count,
        iq_rms_milli_count: evidence.frame.iq_rms_milli_count,
        drift_permille: evidence.frame.drift_permille,
        zero_id_mean_milli_count: evidence.frame.zero_id_mean_milli_count,
        zero_iq_mean_milli_count: evidence.frame.zero_iq_mean_milli_count,
    });
    sink.foci_inductance_estimate(InductanceEstimateReport {
        oid,
        x_average_count_ratio_milli: evidence.estimate.x_average_count_ratio_milli,
        x_d_count_ratio_milli: evidence.estimate.x_d_count_ratio_milli,
        x_q_count_ratio_milli: evidence.estimate.x_q_count_ratio_milli,
        saliency_status: evidence.estimate.saliency_status,
        saliency_permille: evidence.estimate.saliency_permille,
        x_mag_nominal_count_ratio_milli: evidence.estimate.x_mag_nominal_count_ratio_milli,
        x_mag_shift_minus_permille: evidence.estimate.x_mag_shift_minus_permille,
        x_mag_shift_plus_permille: evidence.estimate.x_mag_shift_plus_permille,
        x_mag_vs_quad_permille: evidence.estimate.x_mag_vs_quad_permille,
    });
}

fn emit_current_loop_evidence<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    status: u8,
    result: &CommissionResult,
) {
    let inner = &result.inner;
    sink.foci_current_validation_envelope(current_validation_envelope_report(oid, inner));
    for evidence in inner
        .current_settled_validation
        .iter()
        .take(inner.current_settled_validation_count as usize)
    {
        sink.foci_current_validation_settled_sample(current_validation_settled_sample_report(
            oid, evidence,
        ));
    }
    for (sample_index, evidence) in inner
        .current_flux_validation
        .iter()
        .take(inner.current_flux_validation_count as usize)
        .enumerate()
    {
        sink.foci_current_validation_axis(current_validation_axis_report(
            oid,
            CURRENT_AXIS_FLUX,
            sample_index as u8,
            evidence,
        ));
    }
    for (sample_index, evidence) in inner
        .current_torque_validation
        .iter()
        .take(inner.current_torque_validation_count as usize)
        .enumerate()
    {
        sink.foci_current_validation_axis(current_validation_axis_report(
            oid,
            CURRENT_AXIS_TORQUE,
            sample_index as u8,
            evidence,
        ));
    }
    sink.foci_current_loop_hold(commission_current_loop_hold_report(oid, inner));
    sink.foci_closed_loop_entry(commission_closed_loop_entry_report(oid, inner));
    sink.foci_current_loop_filters(commission_current_loop_filters_report(oid, result));
    sink.foci_current_loop_run(commission_current_loop_run_report(oid, status, inner));
}

fn current_validation_envelope_report(
    oid: u8,
    inner: &InnerResult,
) -> CurrentValidationEnvelopeReport {
    CurrentValidationEnvelopeReport {
        oid,
        step_amplitude: inner.current_validation_step_amplitude,
        flux_step_amplitude: inner.current_validation_flux_step_amplitude,
        torque_step_amplitude: inner.current_validation_torque_step_amplitude,
        pidout_limit: inner.current_validation_pidout_limit,
        current_limited: inner.current_validation_current_limited,
        voltage_limited: inner.current_validation_voltage_limited,
        max_p: inner.current_validation_max_p,
    }
}

fn current_validation_axis_report(
    oid: u8,
    axis: u8,
    sample_index: u8,
    evidence: &CurrentValidationAxisEvidence,
) -> CurrentValidationAxisReport {
    CurrentValidationAxisReport {
        oid,
        axis,
        sample_index,
        status: evidence.status,
        attempt: evidence.attempt,
        target: evidence.target,
        sample_delay_ms: evidence.sample_delay_ms,
        positive_response_permille: evidence.positive_response_permille,
        negative_response_permille: evidence.negative_response_permille,
        cross_axis_permille: evidence.cross_axis_permille,
        cross_axis_peak_permille: evidence.cross_axis_peak_permille,
        voltage_output_permille: evidence.voltage_output_permille,
        encoder_delta_counts: evidence.encoder_delta_counts,
        positive_encoder_delta_counts: evidence.positive_encoder_delta_counts,
        negative_encoder_delta_counts: evidence.negative_encoder_delta_counts,
        status_flags_or: evidence.status_flags_or,
    }
}

fn current_validation_settled_sample_report(
    oid: u8,
    evidence: &CurrentValidationSettledSampleEvidence,
) -> CurrentValidationSettledSampleReport {
    CurrentValidationSettledSampleReport {
        oid,
        axis: evidence.axis,
        sample_index: evidence.sample_index,
        direction: evidence.direction,
        raw_index: evidence.raw_index,
        attempt: evidence.attempt,
        target: evidence.target,
        sample_delay_ms: evidence.sample_delay_ms,
        same_axis_count: evidence.same_axis_count,
        cross_axis_count: evidence.cross_axis_count,
        cross_axis_permille: evidence.cross_axis_permille,
        voltage_output_permille: evidence.voltage_output_permille,
        encoder_delta_counts: evidence.encoder_delta_counts,
        status_flags: evidence.status_flags,
    }
}

fn commission_current_loop_run_report(
    oid: u8,
    status: u8,
    inner: &InnerResult,
) -> CurrentLoopRunReport {
    CurrentLoopRunReport {
        oid,
        status,
        gains_source: inner.current_gains_source,
        candidate_gains_source: inner.candidate_gains_source,
        axis_split_source: inner.axis_split_source,
        candidate_axis_split_source: inner.candidate_axis_split_source,
        gains_tier: inner.gains_tier,
        candidate_gains_tier: inner.candidate_gains_tier,
        measured_axis_split_permille: inner.measured_axis_split_permille,
        candidate_measured_axis_split_permille: inner.candidate_measured_axis_split_permille,
        applied_axis_split_permille: inner.applied_axis_split_permille,
        candidate_applied_axis_split_permille: inner.candidate_applied_axis_split_permille,
        axis_split_clamped: inner.axis_split_clamped,
        candidate_axis_split_clamped: inner.candidate_axis_split_clamped,
        candidate_flux_p: inner.candidate_flux_p,
        candidate_flux_i: inner.candidate_flux_i,
        candidate_torque_p: inner.candidate_torque_p,
        candidate_torque_i: inner.candidate_torque_i,
        candidate_attempt: inner.candidate_attempt,
        current_validation_axes: inner.current_validation_axes,
        flux_validation_sample_count: inner.current_flux_validation_count,
        torque_validation_sample_count: inner.current_torque_validation_count,
        retry_budget_exhausted: inner.retry_budget_exhausted,
        failure_reason: inner.current_loop_failure_reason,
    }
}

fn commission_current_loop_filters_report(
    oid: u8,
    result: &CommissionResult,
) -> CurrentLoopFiltersReport {
    CurrentLoopFiltersReport {
        oid,
        velocity_filter_hz: result.velocity_filter_hz,
        torque_filter_hz: result.torque_filter_hz,
        position_filter_hz: result.position_filter_hz,
        flux_filter_hz: result.flux_filter_hz,
    }
}

fn commission_current_loop_hold_report(oid: u8, inner: &InnerResult) -> CurrentLoopHoldReport {
    CurrentLoopHoldReport {
        oid,
        hold_status: inner.current_hold_status,
        warning_flags: inner.current_hold_warning_flags,
        elapsed_us: inner.current_hold_elapsed_us,
        requested_sample_period_us: inner.current_hold_requested_sample_period_us,
        sample_count: inner.current_hold_sample_count,
        position_span_count: inner.current_hold_position_span_count,
        position_drift_count: inner.current_hold_position_drift_count,
        torque_mean_count: inner.current_hold_torque_mean_count,
        torque_rms_count: inner.current_hold_torque_rms_count,
        torque_peak_to_peak_count: inner.current_hold_torque_peak_to_peak_count,
        torque_crossing_count: inner.current_hold_torque_crossing_count,
        flux_mean_count: inner.current_hold_flux_mean_count,
        flux_rms_count: inner.current_hold_flux_rms_count,
        flux_peak_to_peak_count: inner.current_hold_flux_peak_to_peak_count,
        flux_crossing_count: inner.current_hold_flux_crossing_count,
        status_flags_or: inner.current_hold_status_flags_or,
        actionable_status_count: inner.current_hold_actionable_status_count,
    }
}

fn commission_closed_loop_entry_report(oid: u8, inner: &InnerResult) -> ClosedLoopEntryReport {
    ClosedLoopEntryReport {
        oid,
        entry_status: inner.closed_loop_entry_status,
        position_1: inner.closed_loop_entry_position_1,
        position_2: inner.closed_loop_entry_position_2,
        drift_count: inner.closed_loop_entry_drift_count,
        threshold_count: inner.closed_loop_entry_threshold_count,
        runaway: inner.closed_loop_entry_runaway,
    }
}

#[cfg(test)]
mod tests {
    use crate::commissioning::outer::velocity::integral_plan::StageCAcquisition;
    use std::vec;
    use std::vec::Vec;

    use super::*;
    use crate::autotune::{
        AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE,
        CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED,
        CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, CURRENT_GAINS_TIER_NONE, CURRENT_LOOP_FAILURE_NONE,
        CURRENT_LOOP_FAILURE_SATURATION, CURRENT_VALIDATION_AXIS_FLUX,
        CURRENT_VALIDATION_AXIS_TORQUE, CommissionError, CommissionStatus, PhaseId,
    };
    use crate::commission_dispatch::ReplyPayload;
    use crate::commissioning::types::{
        AdcCalResult, CommissionResult, CurrentValidationAxisEvidence,
        CurrentValidationSettledSampleEvidence, InductanceEvidence, InnerResult, OuterResult,
    };
    use crate::selftest::{
        COMMISSION_DIAGNOSTIC_CAPACITY, CommissionDiagnostic, CommissionDiagnostics,
        SelftestResults, SelftestStageValue,
    };

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn wide_magnitude_encoding_rounds_outward_without_saturation() {
        for value in [0, u64::MAX as u128, u128::MAX] {
            let (mantissa, shift) = upper_bound_u128(value);
            let required_mantissa = value.div_ceil(1u128 << shift);
            assert!(u128::from(mantissa) >= required_mantissa);
        }

        let values = [i64::MIN, -3, -1, 1, i64::MAX];
        let (encoded, shift) = encode_rate_values(values);
        let decoded = encoded.map(|value| i128::from(value) << shift);
        assert!(decoded[0] <= i128::from(values[0]));
        assert!(decoded[1].unsigned_abs() >= i128::from(values[1]).unsigned_abs());
        assert!(decoded[2] >= i128::from(values[2]));
        assert!(decoded[3] <= i128::from(values[3]));
        assert!(decoded[4] >= i128::from(values[4]));

        for value in [i64::MIN, i64::MAX] {
            let (mantissa, shift) = upper_bound_i64(value);
            assert!((i128::from(mantissa) << shift).unsigned_abs() >= value.unsigned_abs() as u128);
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Event {
        Detail {
            oid: u8,
            phase: u8,
            code: u8,
            status: u8,
            value0: u32,
            value1: u32,
            value2: u32,
        },
        Phase {
            oid: u8,
            phase: u8,
            status: u8,
        },
        Calibrate {
            oid: u8,
            status: u8,
            adc_i0: u16,
            adc_i1: u16,
            encoder_count: u32,
        },
        Commission {
            oid: u8,
            status: u8,
            r_count_milli: u32,
            l_count_micro: u32,
            fallback_velocity_limit: u32,
        },
        CommissionTiming {
            oid: u8,
            method: u8,
            status: u8,
        },
        ResistanceRun {
            oid: u8,
            status: u8,
            selected_r_count_slope_milli: i32,
            gain_path_count_slope_milli: i32,
            warning_flags: u32,
            status_flags_or: u32,
            peak_abs_current_count: u16,
            max_abs_steady_mean_current_count: u16,
            current_ceiling_count: u16,
            power_stage_tripped: bool,
        },
        ResistanceAxis {
            oid: u8,
            electrical_axis: u8,
            phi_e_ext: u16,
            r_count_slope_milli: i32,
            intercept_count: i32,
            rmse_permille: u16,
            selected_mask: u16,
            signed_count_slope_milli: i32,
            signed_asymmetry_permille: u16,
            drift_permille: u16,
        },
        EncoderAlignment {
            oid: u8,
            encoder_count: u32,
            electrical_residual_counts: u16,
            stability_counts: u16,
            movement_counts: u16,
            min_movement_counts: u16,
            counts_per_electrical_rev: u16,
        },
        InductanceRun {
            oid: u8,
            source: u8,
            status: u8,
            warning_flags: u16,
            ud_count: i16,
            realized_frequency_millihz: u32,
            elapsed_us: u32,
            openloop_phi_delta_counts: u32,
            sample_count: u16,
            encoder_delta_counts: i32,
            status_flags_or: u32,
        },
        InductanceFrame {
            oid: u8,
            id_mean_milli_count: i32,
            iq_mean_milli_count: i32,
            id_rms_milli_count: u32,
            iq_rms_milli_count: u32,
            drift_permille: u16,
            zero_id_mean_milli_count: i32,
            zero_iq_mean_milli_count: i32,
        },
        InductanceEstimate {
            oid: u8,
            x_average_count_ratio_milli: u32,
            x_d_count_ratio_milli: u32,
            x_q_count_ratio_milli: u32,
            saliency_status: u8,
            saliency_permille: u16,
            x_mag_nominal_count_ratio_milli: u32,
            x_mag_shift_minus_permille: u16,
            x_mag_shift_plus_permille: u16,
            x_mag_vs_quad_permille: u16,
        },
        CurrentValidationAxis {
            oid: u8,
            axis: u8,
            sample_index: u8,
            status: u8,
            attempt: u8,
            target: i16,
            sample_delay_ms: u16,
            positive_response_permille: u16,
            negative_response_permille: u16,
            cross_axis_permille: u16,
            cross_axis_peak_permille: u16,
            voltage_output_permille: u16,
            encoder_delta_counts: u16,
            positive_encoder_delta_counts: i32,
            negative_encoder_delta_counts: i32,
            status_flags_or: u32,
        },
        CurrentValidationSettledSample {
            oid: u8,
            axis: u8,
            sample_index: u8,
            direction: u8,
            raw_index: u8,
            attempt: u8,
            target: i16,
            sample_delay_ms: u16,
            same_axis_count: i16,
            cross_axis_count: i16,
            cross_axis_permille: u16,
            voltage_output_permille: u16,
            encoder_delta_counts: i32,
            status_flags: u32,
        },
        CurrentValidationEnvelope {
            oid: u8,
            step_amplitude: u16,
            flux_step_amplitude: u16,
            torque_step_amplitude: u16,
            pidout_limit: u16,
            current_limited: u16,
            voltage_limited: u16,
            max_p: u16,
        },
        CurrentLoopHold {
            oid: u8,
            hold_status: u8,
            sample_count: u8,
            torque_rms_count: u16,
            flux_rms_count: u16,
        },
        ClosedLoopEntry {
            oid: u8,
            entry_status: u8,
            position_1: i32,
            position_2: i32,
            drift_count: u16,
            threshold_count: u16,
            runaway: u8,
        },
        CurrentLoopFilters {
            oid: u8,
            velocity_filter_hz: u16,
            torque_filter_hz: u16,
            position_filter_hz: u16,
            flux_filter_hz: u16,
        },
        CurrentLoopRun {
            oid: u8,
            status: u8,
            gains_source: u8,
            candidate_gains_source: u8,
            axis_split_source: u8,
            candidate_axis_split_source: u8,
            gains_tier: u8,
            candidate_gains_tier: u8,
            measured_axis_split_permille: u16,
            candidate_measured_axis_split_permille: u16,
            applied_axis_split_permille: u16,
            candidate_applied_axis_split_permille: u16,
            axis_split_clamped: u8,
            candidate_axis_split_clamped: u8,
            candidate_flux_p: u16,
            candidate_flux_i: u16,
            candidate_torque_p: u16,
            candidate_torque_i: u16,
            candidate_attempt: u8,
            current_validation_axes: u8,
            flux_validation_sample_count: u8,
            torque_validation_sample_count: u8,
            retry_budget_exhausted: u8,
            failure_reason: u8,
        },
        Tune {
            oid: u8,
            status: u8,
            velocity_p: u16,
            position_p: u16,
            warning_code: u8,
            outer_evidence_flags: u8,
            stiffness_timebase_ms: u16,
            velocity_search_stop_reason: u8,
            motion_budget_mrev: u16,
        },
        OuterSafetyFault {
            oid: u8,
            reason: u8,
            delta_counts: i32,
            velocity_counts_per_ms: i32,
            velocity_cap_counts_per_ms: i32,
        },
        StageBTerminal {
            fragment: u8,
            evidence_sequence: u16,
        },
        WorkflowPlan,
        IntegralPlan(u8),
        IntegralPlanRecovery,
        IntegralObservation(u8),
        IntegralRung(u8),
        PrimitiveCurrent,
        Recovery {
            oid: u8,
            run_sequence: u32,
            evidence_sequence: u16,
            stage: u8,
            rung_index: u8,
            p_raw: u16,
            binding_source: u8,
            outcome: u8,
        },
        IntegralDerived(u8),
        IntegralStageBComparison {
            direction: u8,
            available: u8,
        },
        IntegralTerminal(u8),
        SelftestResult {
            oid: u8,
            stage: u8,
            status: u8,
            value: u32,
        },
        SelftestDone {
            oid: u8,
            status: u8,
        },
    }

    #[derive(Default)]
    struct RecordingSink {
        events: Vec<Event>,
        integral_rest_boundary: Option<[u8; 2]>,
        integral_plan_joint_memberships: [u32; 2],
        integral_plan_intervals: [[i32; 2]; 2],
    }

    impl CommissionReplySink for RecordingSink {
        fn foci_commission_detail(&mut self, oid: u8, event: CommissionDiagnostic) {
            self.events.push(Event::Detail {
                oid,
                phase: event.phase,
                code: event.code,
                status: event.status,
                value0: event.value0,
                value1: event.value1,
                value2: event.value2,
            });
        }

        fn foci_commission_phase(&mut self, oid: u8, phase: u8, status: u8) {
            self.events.push(Event::Phase { oid, phase, status });
        }

        fn foci_commission_result(&mut self, oid: u8, status: u8, result: &CommissionResult) {
            self.events.push(Event::Commission {
                oid,
                status,
                r_count_milli: result.inner.r_count_milli,
                l_count_micro: result.inner.l_count_micro,
                fallback_velocity_limit: result.fallback_velocity_limit,
            });
        }

        fn foci_commission_timing(&mut self, report: CommissionTimingReport) {
            self.events.push(Event::CommissionTiming {
                oid: report.oid,
                method: report.method,
                status: report.status,
            });
        }

        fn foci_resistance_run(&mut self, report: crate::tmc_diagnostics::ResistanceRunReport) {
            self.events.push(Event::ResistanceRun {
                oid: report.oid,
                status: report.status,
                selected_r_count_slope_milli: report.selected_r_count_slope_milli,
                gain_path_count_slope_milli: report.gain_path_count_slope_milli,
                warning_flags: report.warning_flags,
                status_flags_or: report.status_flags_or,
                peak_abs_current_count: report.peak_abs_current_count,
                max_abs_steady_mean_current_count: report.max_abs_steady_mean_current_count,
                current_ceiling_count: report.current_ceiling_count,
                power_stage_tripped: report.power_stage_tripped,
            });
        }

        fn foci_resistance_axis(&mut self, report: crate::tmc_diagnostics::ResistanceAxisReport) {
            self.events.push(Event::ResistanceAxis {
                oid: report.oid,
                electrical_axis: report.electrical_axis,
                phi_e_ext: report.phi_e_ext,
                r_count_slope_milli: report.r_count_slope_milli,
                intercept_count: report.intercept_count,
                rmse_permille: report.rmse_permille,
                selected_mask: report.selected_mask,
                signed_count_slope_milli: report.signed_count_slope_milli,
                signed_asymmetry_permille: report.signed_asymmetry_permille,
                drift_permille: report.drift_permille,
            });
        }

        fn foci_encoder_alignment(
            &mut self,
            report: crate::tmc_diagnostics::EncoderAlignmentReport,
        ) {
            self.events.push(Event::EncoderAlignment {
                oid: report.oid,
                encoder_count: report.encoder_count,
                electrical_residual_counts: report.electrical_residual_counts,
                stability_counts: report.stability_counts,
                movement_counts: report.movement_counts,
                min_movement_counts: report.min_movement_counts,
                counts_per_electrical_rev: report.counts_per_electrical_rev,
            });
        }

        fn foci_adc_residual(&mut self, _report: crate::tmc_diagnostics::AdcResidualReport) {}

        fn foci_inductance_run(&mut self, report: crate::tmc_diagnostics::InductanceRunReport) {
            self.events.push(Event::InductanceRun {
                oid: report.oid,
                source: report.source,
                status: report.status,
                warning_flags: report.warning_flags,
                ud_count: report.ud_count,
                realized_frequency_millihz: report.realized_frequency_millihz,
                elapsed_us: report.elapsed_us,
                openloop_phi_delta_counts: report.openloop_phi_delta_counts,
                sample_count: report.sample_count,
                encoder_delta_counts: report.encoder_delta_counts,
                status_flags_or: report.status_flags_or,
            });
        }

        fn foci_inductance_frame(&mut self, report: crate::tmc_diagnostics::InductanceFrameReport) {
            self.events.push(Event::InductanceFrame {
                oid: report.oid,
                id_mean_milli_count: report.id_mean_milli_count,
                iq_mean_milli_count: report.iq_mean_milli_count,
                id_rms_milli_count: report.id_rms_milli_count,
                iq_rms_milli_count: report.iq_rms_milli_count,
                drift_permille: report.drift_permille,
                zero_id_mean_milli_count: report.zero_id_mean_milli_count,
                zero_iq_mean_milli_count: report.zero_iq_mean_milli_count,
            });
        }

        fn foci_inductance_estimate(
            &mut self,
            report: crate::tmc_diagnostics::InductanceEstimateReport,
        ) {
            self.events.push(Event::InductanceEstimate {
                oid: report.oid,
                x_average_count_ratio_milli: report.x_average_count_ratio_milli,
                x_d_count_ratio_milli: report.x_d_count_ratio_milli,
                x_q_count_ratio_milli: report.x_q_count_ratio_milli,
                saliency_status: report.saliency_status,
                saliency_permille: report.saliency_permille,
                x_mag_nominal_count_ratio_milli: report.x_mag_nominal_count_ratio_milli,
                x_mag_shift_minus_permille: report.x_mag_shift_minus_permille,
                x_mag_shift_plus_permille: report.x_mag_shift_plus_permille,
                x_mag_vs_quad_permille: report.x_mag_vs_quad_permille,
            });
        }

        fn foci_current_loop_hold(
            &mut self,
            report: crate::tmc_diagnostics::CurrentLoopHoldReport,
        ) {
            self.events.push(Event::CurrentLoopHold {
                oid: report.oid,
                hold_status: report.hold_status,
                sample_count: report.sample_count,
                torque_rms_count: report.torque_rms_count,
                flux_rms_count: report.flux_rms_count,
            });
        }

        fn foci_closed_loop_entry(
            &mut self,
            report: crate::tmc_diagnostics::ClosedLoopEntryReport,
        ) {
            self.events.push(Event::ClosedLoopEntry {
                oid: report.oid,
                entry_status: report.entry_status,
                position_1: report.position_1,
                position_2: report.position_2,
                drift_count: report.drift_count,
                threshold_count: report.threshold_count,
                runaway: report.runaway,
            });
        }

        fn foci_current_loop_filters(
            &mut self,
            report: crate::tmc_diagnostics::CurrentLoopFiltersReport,
        ) {
            self.events.push(Event::CurrentLoopFilters {
                oid: report.oid,
                velocity_filter_hz: report.velocity_filter_hz,
                torque_filter_hz: report.torque_filter_hz,
                position_filter_hz: report.position_filter_hz,
                flux_filter_hz: report.flux_filter_hz,
            });
        }

        fn foci_current_loop_run(&mut self, report: crate::tmc_diagnostics::CurrentLoopRunReport) {
            self.events.push(Event::CurrentLoopRun {
                oid: report.oid,
                status: report.status,
                gains_source: report.gains_source,
                candidate_gains_source: report.candidate_gains_source,
                axis_split_source: report.axis_split_source,
                candidate_axis_split_source: report.candidate_axis_split_source,
                gains_tier: report.gains_tier,
                candidate_gains_tier: report.candidate_gains_tier,
                measured_axis_split_permille: report.measured_axis_split_permille,
                candidate_measured_axis_split_permille: report
                    .candidate_measured_axis_split_permille,
                applied_axis_split_permille: report.applied_axis_split_permille,
                candidate_applied_axis_split_permille: report.candidate_applied_axis_split_permille,
                axis_split_clamped: report.axis_split_clamped,
                candidate_axis_split_clamped: report.candidate_axis_split_clamped,
                candidate_flux_p: report.candidate_flux_p,
                candidate_flux_i: report.candidate_flux_i,
                candidate_torque_p: report.candidate_torque_p,
                candidate_torque_i: report.candidate_torque_i,
                candidate_attempt: report.candidate_attempt,
                current_validation_axes: report.current_validation_axes,
                flux_validation_sample_count: report.flux_validation_sample_count,
                torque_validation_sample_count: report.torque_validation_sample_count,
                retry_budget_exhausted: report.retry_budget_exhausted,
                failure_reason: report.failure_reason,
            });
        }

        fn foci_current_validation_axis(
            &mut self,
            report: crate::tmc_diagnostics::CurrentValidationAxisReport,
        ) {
            self.events.push(Event::CurrentValidationAxis {
                oid: report.oid,
                axis: report.axis,
                sample_index: report.sample_index,
                status: report.status,
                attempt: report.attempt,
                target: report.target,
                sample_delay_ms: report.sample_delay_ms,
                positive_response_permille: report.positive_response_permille,
                negative_response_permille: report.negative_response_permille,
                cross_axis_permille: report.cross_axis_permille,
                cross_axis_peak_permille: report.cross_axis_peak_permille,
                voltage_output_permille: report.voltage_output_permille,
                encoder_delta_counts: report.encoder_delta_counts,
                positive_encoder_delta_counts: report.positive_encoder_delta_counts,
                negative_encoder_delta_counts: report.negative_encoder_delta_counts,
                status_flags_or: report.status_flags_or,
            });
        }

        fn foci_current_validation_settled_sample(
            &mut self,
            report: crate::tmc_diagnostics::CurrentValidationSettledSampleReport,
        ) {
            self.events.push(Event::CurrentValidationSettledSample {
                oid: report.oid,
                axis: report.axis,
                sample_index: report.sample_index,
                direction: report.direction,
                raw_index: report.raw_index,
                attempt: report.attempt,
                target: report.target,
                sample_delay_ms: report.sample_delay_ms,
                same_axis_count: report.same_axis_count,
                cross_axis_count: report.cross_axis_count,
                cross_axis_permille: report.cross_axis_permille,
                voltage_output_permille: report.voltage_output_permille,
                encoder_delta_counts: report.encoder_delta_counts,
                status_flags: report.status_flags,
            });
        }

        fn foci_current_validation_envelope(
            &mut self,
            report: crate::tmc_diagnostics::CurrentValidationEnvelopeReport,
        ) {
            self.events.push(Event::CurrentValidationEnvelope {
                oid: report.oid,
                step_amplitude: report.step_amplitude,
                flux_step_amplitude: report.flux_step_amplitude,
                torque_step_amplitude: report.torque_step_amplitude,
                pidout_limit: report.pidout_limit,
                current_limited: report.current_limited,
                voltage_limited: report.voltage_limited,
                max_p: report.max_p,
            });
        }

        fn foci_tune_result(&mut self, oid: u8, status: u8, result: &OuterResult) {
            self.events.push(Event::Tune {
                oid,
                status,
                velocity_p: result.velocity_p,
                position_p: result.position_p,
                warning_code: result.warning_code,
                outer_evidence_flags: result.outer_evidence_flags,
                stiffness_timebase_ms: result.stiffness_timebase_ms,
                velocity_search_stop_reason: result.velocity_search_stop_reason,
                motion_budget_mrev: result.motion_budget_mrev,
            });
        }

        fn foci_outer_safety_fault(&mut self, oid: u8, fault: OuterSafetyFault) {
            self.events.push(Event::OuterSafetyFault {
                oid,
                reason: fault.reason,
                delta_counts: fault.delta_counts,
                velocity_counts_per_ms: fault.velocity_counts_per_ms,
                velocity_cap_counts_per_ms: fault.velocity_cap_counts_per_ms,
            });
        }

        fn foci_rung_origin_recovery_summary(
            &mut self,
            report: crate::ankyra::replies::FociRungOriginRecoverySummary,
        ) {
            self.events.push(Event::Recovery {
                oid: report.oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                stage: report.stage,
                rung_index: report.rung_index,
                p_raw: report.p_raw,
                binding_source: report.binding_source,
                outcome: report.outcome,
            });
        }

        fn foci_velocity_primitive_current_evidence(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityPrimitiveCurrentEvidence,
        ) {
            self.events.push(Event::PrimitiveCurrent);
        }

        fn foci_velocity_stage_b_terminal_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBTerminalCore,
        ) {
            self.events.push(Event::StageBTerminal {
                fragment: report.fragment,
                evidence_sequence: report.evidence_sequence,
            });
        }

        fn foci_velocity_stage_b_terminal_identity(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBTerminalIdentity,
        ) {
            self.events.push(Event::StageBTerminal {
                fragment: report.fragment,
                evidence_sequence: report.evidence_sequence,
            });
        }

        fn foci_velocity_stage_b_terminal_interval(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBTerminalInterval,
        ) {
            self.events.push(Event::StageBTerminal {
                fragment: report.fragment,
                evidence_sequence: report.evidence_sequence,
            });
        }

        fn foci_commissioning_workflow_plan(
            &mut self,
            _report: crate::ankyra::replies::FociCommissioningWorkflowPlan,
        ) {
            self.events.push(Event::WorkflowPlan);
        }

        fn foci_velocity_integral_plan_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralPlanCore,
        ) {
            self.events.push(Event::IntegralPlan(report.fragment));
        }

        fn foci_velocity_integral_plan_geometry(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralPlanGeometry,
        ) {
            self.events.push(Event::IntegralPlan(report.fragment));
        }

        fn foci_velocity_integral_plan_authority(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralPlanAuthority,
        ) {
            self.integral_plan_joint_memberships[usize::from(report.direction)] =
                report.joint_membership;
            self.integral_plan_intervals[usize::from(report.direction)] =
                [report.pooled_low_q16, report.pooled_high_q16];
            self.events.push(Event::IntegralPlan(4 + report.direction));
        }

        fn foci_velocity_integral_plan_timing(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralPlanTiming,
        ) {
            self.events.push(Event::IntegralPlan(report.fragment));
        }

        fn foci_velocity_integral_plan_travel(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralPlanTravel,
        ) {
            self.events.push(Event::IntegralPlan(report.fragment));
        }

        fn foci_velocity_integral_plan_recovery(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralPlanRecovery,
        ) {
            self.events.push(Event::IntegralPlanRecovery);
        }

        fn foci_velocity_integral_plan_rung(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralPlanRung,
        ) {
            self.events.push(Event::IntegralPlan(6 + report.rung_index));
        }

        fn foci_velocity_integral_observation_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralObservationCore,
        ) {
            self.events
                .push(Event::IntegralObservation(report.fragment));
        }

        fn foci_velocity_integral_observation_rate(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralObservationRate,
        ) {
            self.events
                .push(Event::IntegralObservation(report.fragment));
        }

        fn foci_velocity_integral_observation_quality(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralObservationQuality,
        ) {
            self.events
                .push(Event::IntegralObservation(report.fragment));
        }

        fn foci_velocity_integral_rung_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralRungCore,
        ) {
            self.events.push(Event::IntegralRung(report.fragment));
        }

        fn foci_velocity_integral_rung_component(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralRungComponent,
        ) {
            self.events.push(Event::IntegralRung(2 + report.direction));
        }

        fn foci_velocity_integral_run_summary(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralRunSummary,
        ) {
            self.events.push(Event::IntegralDerived(0));
        }

        fn foci_velocity_integral_curve_interval(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralCurveInterval,
        ) {
            self.events.push(Event::IntegralDerived(1));
        }

        fn foci_velocity_integral_drift(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralDrift,
        ) {
            self.events.push(Event::IntegralDerived(2));
        }

        fn foci_velocity_integral_stage_b_comparison(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralStageBComparison,
        ) {
            self.events.push(Event::IntegralStageBComparison {
                direction: report.direction,
                available: report.available,
            });
        }

        fn foci_velocity_integral_reproduction_core(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralReproductionCore,
        ) {
            self.events.push(Event::IntegralDerived(4));
        }

        fn foci_velocity_integral_reproduction_mask(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralReproductionMask,
        ) {
            self.events.push(Event::IntegralDerived(5));
        }

        fn foci_velocity_integral_reproduction_interval(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralReproductionInterval,
        ) {
            self.events.push(Event::IntegralDerived(6));
        }

        fn foci_velocity_integral_reproduction_digest(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityIntegralReproductionDigest,
        ) {
            self.events.push(Event::IntegralDerived(7));
        }

        fn foci_velocity_integral_terminal_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralTerminalCore,
        ) {
            if report.fragment == 0 {
                self.integral_rest_boundary = Some([
                    report.rest_boundary_rung_plus_one,
                    report.rest_boundary_slot_plus_one,
                ]);
            }
            self.events.push(Event::IntegralTerminal(report.fragment));
        }

        fn foci_velocity_integral_terminal_identity(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralTerminalIdentity,
        ) {
            self.events.push(Event::IntegralTerminal(report.fragment));
        }

        fn foci_velocity_integral_terminal_timing(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralTerminalTiming,
        ) {
            self.events.push(Event::IntegralTerminal(report.fragment));
        }

        fn foci_calibrate_result(&mut self, oid: u8, status: u8, result: &AdcCalResult) {
            self.events.push(Event::Calibrate {
                oid,
                status,
                adc_i0: result.adc_i0,
                adc_i1: result.adc_i1,
                encoder_count: result.encoder_count,
            });
        }

        fn foci_selftest_result(&mut self, oid: u8, stage: u8, value: SelftestStageValue) {
            self.events.push(Event::SelftestResult {
                oid,
                stage,
                status: value.status,
                value: value.value,
            });
        }

        fn foci_selftest_done(&mut self, oid: u8, status: u8) {
            self.events.push(Event::SelftestDone { oid, status });
        }
    }

    fn diagnostic(phase: u8, code: u8) -> CommissionDiagnostic {
        CommissionDiagnostic {
            phase,
            code,
            status: code % 3,
            value0: 100 + u32::from(code),
            value1: 200 + u32::from(code),
            value2: 300 + u32::from(code),
        }
    }

    fn diagnostics<const N: usize>(items: [CommissionDiagnostic; N]) -> CommissionDiagnostics {
        let mut diagnostics = CommissionDiagnostics::default();
        for item in items {
            diagnostics.push(item);
        }
        diagnostics
    }

    fn commission_result() -> CommissionResult {
        let mut flux_validation = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        flux_validation[0] = CurrentValidationAxisEvidence {
            status: 0,
            attempt: 0,
            target: 250,
            sample_delay_ms: 2,
            positive_response_permille: 960,
            negative_response_permille: 940,
            cross_axis_permille: 32,
            cross_axis_peak_permille: 32,
            voltage_output_permille: 250,
            encoder_delta_counts: 1,
            positive_encoder_delta_counts: 1,
            negative_encoder_delta_counts: -1,
            status_flags_or: 0,
        };
        let mut torque_validation = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        torque_validation[0] = CurrentValidationAxisEvidence {
            status: 0,
            attempt: 0,
            target: 250,
            sample_delay_ms: 1,
            positive_response_permille: 900,
            negative_response_permille: 880,
            cross_axis_permille: 24,
            cross_axis_peak_permille: 24,
            voltage_output_permille: 280,
            encoder_delta_counts: 2,
            positive_encoder_delta_counts: 2,
            negative_encoder_delta_counts: -2,
            status_flags_or: 4,
        };
        let mut settled_validation = [CurrentValidationSettledSampleEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES];
        settled_validation[0] = CurrentValidationSettledSampleEvidence {
            axis: CURRENT_AXIS_FLUX,
            sample_index: 3,
            direction: 0,
            raw_index: 0,
            attempt: 0,
            target: 128,
            sample_delay_ms: 100,
            same_axis_count: 120,
            cross_axis_count: 45,
            cross_axis_permille: 351,
            voltage_output_permille: 410,
            encoder_delta_counts: 1,
            status_flags: 0x40,
        };

        CommissionResult {
            inner: InnerResult {
                status: CommissionStatus::Accepted,
                r_count_milli: 11,
                l_count_micro: 12,
                flux_p: 15,
                flux_i: 16,
                torque_p: 17,
                torque_i: 18,
                candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                candidate_measured_axis_split_permille: 1000,
                candidate_applied_axis_split_permille: 1000,
                candidate_axis_split_clamped: 0,
                candidate_flux_p: 15,
                candidate_flux_i: 16,
                candidate_torque_p: 17,
                candidate_torque_i: 18,
                candidate_attempt: 0,
                lambda_us: 19,
                theta_e_us: 20,
                ringing_count: 21,
                bandwidth_hz: 22,
                tau_e_us: 23,
                inductance: InductanceEvidence::test_fixture(),
                inner_warning_flags: 26,
                timing: [crate::commissioning::types::CommissionTimingSummary::NOT_RUN; 3],
                timing_summary: 0,
                resistance_selected_count_slope_milli: 1050,
                resistance_gain_path_count_slope_milli: 16,
                resistance_axis0_count_slope_milli: 1040,
                resistance_axis1_count_slope_milli: 1060,
                resistance_axis0_intercept_count: 5,
                resistance_axis1_intercept_count: 6,
                resistance_axis0_rmse_permille: 7,
                resistance_axis1_rmse_permille: 8,
                resistance_selected_mask_axis0: 0b1111_1000,
                resistance_selected_mask_axis1: 0b0111_1100,
                resistance_axis0_signed_count_slope_milli: 1045,
                resistance_axis1_signed_count_slope_milli: 1055,
                resistance_axis0_signed_asymmetry_permille: 9,
                resistance_axis1_signed_asymmetry_permille: 10,
                resistance_axis0_drift_permille: 11,
                resistance_axis1_drift_permille: 12,
                resistance_status_flags_or: 0,
                resistance_warning_flags: 0,
                resistance_peak_abs_current_count: 1_440,
                resistance_max_abs_steady_mean_current_count: 1_280,
                resistance_current_ceiling_count: 1_600,
                resistance_power_stage_tripped: false,
                encoder_alignment_count: 163,
                encoder_alignment_electrical_residual_counts: 3,
                encoder_alignment_stability_counts: 1,
                encoder_alignment_movement_counts: 37,
                encoder_alignment_min_movement_counts: 2,
                encoder_alignment_counts_per_electrical_rev: 80,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                current_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                measured_axis_split_permille: 1000,
                applied_axis_split_permille: 1000,
                axis_split_clamped: 0,
                current_validation_axes: CURRENT_VALIDATION_AXIS_FLUX
                    | CURRENT_VALIDATION_AXIS_TORQUE,
                retry_budget_exhausted: 0,
                current_loop_failure_reason: CURRENT_LOOP_FAILURE_NONE,
                closed_loop_entry_status: crate::autotune::CLOSED_LOOP_ENTRY_STATUS_NOT_RUN,
                closed_loop_entry_position_1: 0,
                closed_loop_entry_position_2: 0,
                closed_loop_entry_drift_count: 0,
                closed_loop_entry_threshold_count: 0,
                closed_loop_entry_runaway: 0,
                current_hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
                current_hold_warning_flags: 0,
                current_hold_elapsed_us: 0,
                current_hold_requested_sample_period_us: crate::autotune::HOLD_GATE_SAMPLE_PERIOD_US
                    as u16,
                current_hold_sample_count: 0,
                current_hold_position_span_count: 0,
                current_hold_position_drift_count: 0,
                current_hold_torque_mean_count: 0,
                current_hold_torque_rms_count: 0,
                current_hold_torque_peak_to_peak_count: 0,
                current_hold_torque_crossing_count: 0,
                current_hold_flux_mean_count: 0,
                current_hold_flux_rms_count: 0,
                current_hold_flux_peak_to_peak_count: 0,
                current_hold_flux_crossing_count: 0,
                current_hold_status_flags_or: 0,
                current_hold_actionable_status_count: 0,
                current_validation_step_amplitude: 128,
                current_validation_flux_step_amplitude: 128,
                current_validation_torque_step_amplitude: 128,
                current_validation_pidout_limit: 2400,
                current_validation_current_limited: 422,
                current_validation_voltage_limited: 128,
                current_validation_max_p: 3474,
                current_settled_validation_count: 1,
                current_settled_validation: settled_validation,
                current_flux_validation_count: 1,
                current_flux_validation: flux_validation,
                current_torque_validation_count: 1,
                current_torque_validation: torque_validation,
            },
            fallback_velocity_p: 31,
            fallback_velocity_i: 32,
            fallback_position_p: 33,
            fallback_position_i: 34,
            fallback_velocity_limit: 35,
            velocity_filter_hz: 0,
            torque_filter_hz: 3000,
            position_filter_hz: 0,
            flux_filter_hz: 3000,
        }
    }

    fn outer_result() -> OuterResult {
        OuterResult {
            status: CommissionStatus::AcceptedWithWarnings,
            velocity_p: 41,
            velocity_i: 42,
            velocity_limit: 43,
            position_p: 44,
            position_i: 45,
            velocity_filter_hz: 46,
            position_filter_hz: 47,
            flux_filter_hz: 48,
            torque_filter_hz: 49,
            j_eff: 50,
            b_eff: 51,
            warning_code: 52,
            mech_torque_step: 53,
            mech_fwd_accel: 54,
            mech_rev_accel: 55,
            mech_fwd_peak_velocity: 56,
            mech_rev_peak_velocity: 57,
            mech_fwd_travel: 58,
            mech_rev_travel: 59,
            synth_velocity_p: 60,
            synth_velocity_i: 61,
            synth_position_p: 62,
            synth_position_i: 63,
            outer_evidence_flags: 0x3f,
            stiffness_timebase_ms: 64,
            velocity_search_stop_reason: 65,
            motion_budget_mrev: 66,
        }
    }

    fn current_validation_event(
        oid: u8,
        axis: u8,
        sample_index: u8,
        evidence: CurrentValidationAxisEvidence,
    ) -> Event {
        Event::CurrentValidationAxis {
            oid,
            axis,
            sample_index,
            status: evidence.status,
            attempt: evidence.attempt,
            target: evidence.target,
            sample_delay_ms: evidence.sample_delay_ms,
            positive_response_permille: evidence.positive_response_permille,
            negative_response_permille: evidence.negative_response_permille,
            cross_axis_permille: evidence.cross_axis_permille,
            cross_axis_peak_permille: evidence.cross_axis_peak_permille,
            voltage_output_permille: evidence.voltage_output_permille,
            encoder_delta_counts: evidence.encoder_delta_counts,
            positive_encoder_delta_counts: evidence.positive_encoder_delta_counts,
            negative_encoder_delta_counts: evidence.negative_encoder_delta_counts,
            status_flags_or: evidence.status_flags_or,
        }
    }

    fn current_loop_events(oid: u8, status: u8) -> Vec<Event> {
        vec![
            Event::CurrentValidationEnvelope {
                oid,
                step_amplitude: 128,
                flux_step_amplitude: 128,
                torque_step_amplitude: 128,
                pidout_limit: 2400,
                current_limited: 422,
                voltage_limited: 128,
                max_p: 3474,
            },
            Event::CurrentValidationSettledSample {
                oid,
                axis: CURRENT_AXIS_FLUX,
                sample_index: 3,
                direction: 0,
                raw_index: 0,
                attempt: 0,
                target: 128,
                sample_delay_ms: 100,
                same_axis_count: 120,
                cross_axis_count: 45,
                cross_axis_permille: 351,
                voltage_output_permille: 410,
                encoder_delta_counts: 1,
                status_flags: 0x40,
            },
            current_validation_event(
                oid,
                CURRENT_AXIS_FLUX,
                0,
                CurrentValidationAxisEvidence {
                    status: 0,
                    attempt: 0,
                    target: 250,
                    sample_delay_ms: 2,
                    positive_response_permille: 960,
                    negative_response_permille: 940,
                    cross_axis_permille: 32,
                    cross_axis_peak_permille: 32,
                    voltage_output_permille: 250,
                    encoder_delta_counts: 1,
                    positive_encoder_delta_counts: 1,
                    negative_encoder_delta_counts: -1,
                    status_flags_or: 0,
                },
            ),
            current_validation_event(
                oid,
                CURRENT_AXIS_TORQUE,
                0,
                CurrentValidationAxisEvidence {
                    status: 0,
                    attempt: 0,
                    target: 250,
                    sample_delay_ms: 1,
                    positive_response_permille: 900,
                    negative_response_permille: 880,
                    cross_axis_permille: 24,
                    cross_axis_peak_permille: 24,
                    voltage_output_permille: 280,
                    encoder_delta_counts: 2,
                    positive_encoder_delta_counts: 2,
                    negative_encoder_delta_counts: -2,
                    status_flags_or: 4,
                },
            ),
            Event::CurrentLoopHold {
                oid,
                hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
                sample_count: 0,
                torque_rms_count: 0,
                flux_rms_count: 0,
            },
            Event::ClosedLoopEntry {
                oid,
                entry_status: crate::autotune::CLOSED_LOOP_ENTRY_STATUS_NOT_RUN,
                position_1: 0,
                position_2: 0,
                drift_count: 0,
                threshold_count: 0,
                runaway: 0,
            },
            Event::CurrentLoopFilters {
                oid,
                velocity_filter_hz: 0,
                torque_filter_hz: 3000,
                position_filter_hz: 0,
                flux_filter_hz: 3000,
            },
            Event::CurrentLoopRun {
                oid,
                status,
                gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                measured_axis_split_permille: 1000,
                candidate_measured_axis_split_permille: 1000,
                applied_axis_split_permille: 1000,
                candidate_applied_axis_split_permille: 1000,
                axis_split_clamped: 0,
                candidate_axis_split_clamped: 0,
                candidate_flux_p: 15,
                candidate_flux_i: 16,
                candidate_torque_p: 17,
                candidate_torque_i: 18,
                candidate_attempt: 0,
                current_validation_axes: CURRENT_VALIDATION_AXIS_FLUX
                    | CURRENT_VALIDATION_AXIS_TORQUE,
                flux_validation_sample_count: 1,
                torque_validation_sample_count: 1,
                retry_budget_exhausted: 0,
                failure_reason: CURRENT_LOOP_FAILURE_NONE,
            },
        ]
    }

    fn inductance_events(oid: u8) -> Vec<Event> {
        let evidence = InductanceEvidence::test_fixture();
        std::vec![
            Event::InductanceRun {
                oid,
                source: evidence.run.source,
                status: evidence.run.status,
                warning_flags: evidence.run.warning_flags,
                ud_count: evidence.run.ud_count,
                realized_frequency_millihz: evidence.run.realized_frequency_millihz,
                elapsed_us: evidence.run.elapsed_us,
                openloop_phi_delta_counts: evidence.run.openloop_phi_delta_counts,
                sample_count: evidence.run.sample_count,
                encoder_delta_counts: evidence.run.encoder_delta_counts,
                status_flags_or: evidence.run.status_flags_or,
            },
            Event::InductanceFrame {
                oid,
                id_mean_milli_count: evidence.frame.id_mean_milli_count,
                iq_mean_milli_count: evidence.frame.iq_mean_milli_count,
                id_rms_milli_count: evidence.frame.id_rms_milli_count,
                iq_rms_milli_count: evidence.frame.iq_rms_milli_count,
                drift_permille: evidence.frame.drift_permille,
                zero_id_mean_milli_count: evidence.frame.zero_id_mean_milli_count,
                zero_iq_mean_milli_count: evidence.frame.zero_iq_mean_milli_count,
            },
            Event::InductanceEstimate {
                oid,
                x_average_count_ratio_milli: evidence.estimate.x_average_count_ratio_milli,
                x_d_count_ratio_milli: evidence.estimate.x_d_count_ratio_milli,
                x_q_count_ratio_milli: evidence.estimate.x_q_count_ratio_milli,
                saliency_status: evidence.estimate.saliency_status,
                saliency_permille: evidence.estimate.saliency_permille,
                x_mag_nominal_count_ratio_milli: evidence.estimate.x_mag_nominal_count_ratio_milli,
                x_mag_shift_minus_permille: evidence.estimate.x_mag_shift_minus_permille,
                x_mag_shift_plus_permille: evidence.estimate.x_mag_shift_plus_permille,
                x_mag_vs_quad_permille: evidence.estimate.x_mag_vs_quad_permille,
            },
        ]
    }

    #[test]
    fn commission_result_emits_current_loop_evidence_before_terminal_result() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 7,
                result: commission_result(),
            },
        );

        let mut expected = std::vec![
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 0,
                phi_e_ext: 0,
                r_count_slope_milli: 1040,
                intercept_count: 5,
                rmse_permille: 7,
                selected_mask: 0b1111_1000,
                signed_count_slope_milli: 1045,
                signed_asymmetry_permille: 9,
                drift_permille: 11,
            },
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 1,
                phi_e_ext: 16_384,
                r_count_slope_milli: 1060,
                intercept_count: 6,
                rmse_permille: 8,
                selected_mask: 0b0111_1100,
                signed_count_slope_milli: 1055,
                signed_asymmetry_permille: 10,
                drift_permille: 12,
            },
            Event::ResistanceRun {
                oid: 6,
                status: 0,
                selected_r_count_slope_milli: 1050,
                gain_path_count_slope_milli: 16,
                warning_flags: 0,
                status_flags_or: 0,
                peak_abs_current_count: 1_440,
                max_abs_steady_mean_current_count: 1_280,
                current_ceiling_count: 1_600,
                power_stage_tripped: false,
            },
            Event::EncoderAlignment {
                oid: 6,
                encoder_count: 163,
                electrical_residual_counts: 3,
                stability_counts: 1,
                movement_counts: 37,
                min_movement_counts: 2,
                counts_per_electrical_rev: 80,
            },
        ];
        expected.extend(inductance_events(6));
        expected.extend(current_loop_events(6, 7));
        expected.extend([
            Event::CommissionTiming {
                oid: 6,
                method: 0,
                status: 0,
            },
            Event::CommissionTiming {
                oid: 6,
                method: 1,
                status: 0,
            },
            Event::CommissionTiming {
                oid: 6,
                method: 2,
                status: 0,
            },
        ]);
        expected.push(Event::Commission {
            oid: 6,
            status: 7,
            r_count_milli: 11,
            l_count_micro: 12,
            fallback_velocity_limit: 35,
        });

        assert_eq!(sink.events, expected);
    }

    #[test]
    fn current_loop_hold_emits_before_current_loop_run() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.inner.current_hold_status = crate::autotune::HOLD_GATE_STATUS_PASS;
        result.inner.current_hold_sample_count = 250;
        result.inner.current_hold_torque_rms_count = 12;
        result.inner.current_hold_flux_rms_count = 9;

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 0,
                result,
            },
        );

        let hold_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopHold { .. }))
            .expect("current loop hold event");
        let run_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopRun { .. }))
            .expect("current loop run event");
        assert!(hold_index < run_index);
    }

    #[test]
    fn current_loop_filters_emit_before_current_loop_run() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.velocity_filter_hz = 0;
        result.torque_filter_hz = 3000;
        result.position_filter_hz = 0;
        result.flux_filter_hz = 3000;

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 0,
                result,
            },
        );

        let filters_index = sink
            .events
            .iter()
            .position(|e| {
                matches!(
                    e,
                    Event::CurrentLoopFilters {
                        oid: 6,
                        velocity_filter_hz: 0,
                        torque_filter_hz: 3000,
                        position_filter_hz: 0,
                        flux_filter_hz: 3000,
                    }
                )
            })
            .expect("current loop filters event");
        let run_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopRun { .. }))
            .expect("current loop run event");
        assert!(filters_index < run_index);
    }

    #[test]
    fn commission_result_emits_inductance_evidence_before_terminal_result() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.inner.inductance = crate::commissioning::types::InductanceEvidence::test_fixture();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 7,
                status: 0,
                result,
            },
        );

        let alignment_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::EncoderAlignment { .. }))
            .expect("encoder alignment event");
        let run_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::InductanceRun { .. }))
            .expect("inductance run event");
        let frame_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::InductanceFrame { .. }))
            .expect("inductance frame event");
        let estimate_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::InductanceEstimate { .. }))
            .expect("inductance estimate event");
        let current_loop_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopRun { .. }))
            .expect("current loop run event");
        let terminal_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::Commission { .. }))
            .expect("terminal commission event");
        assert!(alignment_index < run_index);
        assert!(run_index < frame_index);
        assert!(frame_index < estimate_index);
        assert!(estimate_index < current_loop_index);
        assert!(current_loop_index < terminal_index);
    }

    #[test]
    fn commission_result_skips_empty_inductance_evidence() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.inner.inductance = InductanceEvidence::EMPTY;

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 7,
                status: 0,
                result,
            },
        );

        assert!(
            !sink
                .events
                .iter()
                .any(|e| matches!(e, Event::InductanceRun { .. }))
        );
        assert!(
            !sink
                .events
                .iter()
                .any(|e| matches!(e, Event::InductanceFrame { .. }))
        );
        assert!(
            !sink
                .events
                .iter()
                .any(|e| matches!(e, Event::InductanceEstimate { .. }))
        );
    }

    #[test]
    fn failed_commission_result_emits_current_loop_failure_before_terminal_result() {
        let mut result = commission_result();
        result.inner.status = CommissionStatus::Failed(CommissionError::CurrentValidationFailed);
        result.inner.current_gains_source = CURRENT_GAINS_SOURCE_FAILED;
        result.inner.gains_tier = CURRENT_GAINS_TIER_NONE;
        result.inner.candidate_gains_source = CURRENT_GAINS_SOURCE_MEASURED;
        result.inner.candidate_axis_split_source = AXIS_SPLIT_SOURCE_NONE;
        result.inner.candidate_gains_tier = CURRENT_GAINS_TIER_MEASURED_SYMMETRIC;
        result.inner.candidate_flux_p = 711;
        result.inner.candidate_flux_i = 26;
        result.inner.candidate_torque_p = 711;
        result.inner.candidate_torque_i = 26;
        result.inner.candidate_attempt = 1;
        result.inner.current_validation_axes = CURRENT_VALIDATION_AXIS_FLUX;
        result.inner.retry_budget_exhausted = 1;
        result.inner.current_loop_failure_reason = CURRENT_LOOP_FAILURE_SATURATION;
        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: CommissionError::CurrentValidationFailed.status_code(),
                result,
            },
        );

        let current_loop_index = sink.events.iter().position(|event| {
            matches!(
                event,
                Event::CurrentLoopRun {
                    status: 9,
                    gains_source: CURRENT_GAINS_SOURCE_FAILED,
                    candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                    candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                    gains_tier: CURRENT_GAINS_TIER_NONE,
                    candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                    candidate_flux_p: 711,
                    candidate_flux_i: 26,
                    candidate_torque_p: 711,
                    candidate_torque_i: 26,
                    candidate_attempt: 1,
                    current_validation_axes: CURRENT_VALIDATION_AXIS_FLUX,
                    retry_budget_exhausted: 1,
                    failure_reason: CURRENT_LOOP_FAILURE_SATURATION,
                    ..
                }
            )
        });
        let terminal_index = sink
            .events
            .iter()
            .position(|event| matches!(event, Event::Commission { status: 9, .. }));

        assert!(current_loop_index.is_some());
        assert!(terminal_index.is_some());
        assert!(current_loop_index < terminal_index);
    }

    #[test]
    fn commission_phase_emits_details_before_phase_reply() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionPhase {
                oid: 7,
                phase: 9,
                status: 3,
                diagnostics: diagnostics([diagnostic(4, 1), diagnostic(5, 2)]),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::Detail {
                    oid: 7,
                    phase: 4,
                    code: 1,
                    status: 1,
                    value0: 101,
                    value1: 201,
                    value2: 301,
                },
                Event::Detail {
                    oid: 7,
                    phase: 5,
                    code: 2,
                    status: 2,
                    value0: 102,
                    value1: 202,
                    value2: 302,
                },
                Event::Phase {
                    oid: 7,
                    phase: 9,
                    status: 3,
                },
            ]
        );
    }

    #[test]
    fn commission_phase_prioritizes_electrical_timing_details() {
        let electrical_id = PhaseId::ElectricalId.to_u8();
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionPhase {
                oid: 7,
                phase: electrical_id,
                status: 3,
                diagnostics: diagnostics([
                    diagnostic(electrical_id, 2),
                    diagnostic(electrical_id, 34),
                    diagnostic(electrical_id, 35),
                    diagnostic(electrical_id, 3),
                    diagnostic(electrical_id, 36),
                    diagnostic(electrical_id, 37),
                    diagnostic(electrical_id, 38),
                    diagnostic(electrical_id, 39),
                    diagnostic(electrical_id, 40),
                    diagnostic(electrical_id, 41),
                ]),
            },
        );

        let detail_keys: Vec<(u8, u8)> = sink
            .events
            .iter()
            .filter_map(|event| match event {
                Event::Detail { phase, code, .. } => Some((*phase, *code)),
                _ => None,
            })
            .collect();
        assert_eq!(
            detail_keys,
            [
                (electrical_id, 34),
                (electrical_id, 35),
                (electrical_id, 36),
                (electrical_id, 37),
                (electrical_id, 38),
                (electrical_id, 39),
                (electrical_id, 40),
                (electrical_id, 41),
                (electrical_id, 2),
                (electrical_id, 3),
            ]
        );
    }

    #[test]
    fn selftest_emits_details_results_then_done() {
        let mut results = SelftestResults::default();
        results.set(
            1,
            SelftestStageValue {
                status: 0,
                value: 111,
            },
        );
        results.set(
            3,
            SelftestStageValue {
                status: 2,
                value: 333,
            },
        );

        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::Selftest {
                oid: 8,
                results,
                diagnostics: diagnostics([diagnostic(6, 3)]),
                done_status: 5,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::Detail {
                    oid: 8,
                    phase: 6,
                    code: 3,
                    status: 0,
                    value0: 103,
                    value1: 203,
                    value2: 303,
                },
                Event::SelftestResult {
                    oid: 8,
                    stage: 1,
                    status: 0,
                    value: 111,
                },
                Event::SelftestResult {
                    oid: 8,
                    stage: 3,
                    status: 2,
                    value: 333,
                },
                Event::SelftestDone { oid: 8, status: 5 },
            ]
        );
    }

    #[test]
    fn diagnostics_emit_only_retained_capacity() {
        let mut diagnostics = CommissionDiagnostics::default();
        for code in 0..(COMMISSION_DIAGNOSTIC_CAPACITY as u8 + 2) {
            diagnostics.push(diagnostic(2, code));
        }

        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionPhase {
                oid: 9,
                phase: 2,
                status: 0,
                diagnostics,
            },
        );

        let detail_codes: Vec<u8> = sink
            .events
            .iter()
            .filter_map(|event| match event {
                Event::Detail { code, .. } => Some(*code),
                _ => None,
            })
            .collect();
        assert_eq!(detail_codes.len(), COMMISSION_DIAGNOSTIC_CAPACITY);
        assert_eq!(detail_codes[COMMISSION_DIAGNOSTIC_CAPACITY - 1], 15);
    }

    #[test]
    fn calibrate_result_emits_diagnostics_before_terminal_reply() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CalibrateResult {
                oid: 1,
                status: 8,
                result: AdcCalResult {
                    adc_i0: 0,
                    adc_i1: 0,
                    encoder_count: 0,
                    encoder_alignment_electrical_residual_counts: 0,
                    encoder_alignment_stability_counts: 0,
                    encoder_alignment_movement_counts: 0,
                    encoder_alignment_min_movement_counts: 0,
                    encoder_alignment_counts_per_electrical_rev: 0,
                    adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                    adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                },
                diagnostics: diagnostics([CommissionDiagnostic {
                    phase: PhaseId::EncoderCheck.to_u8(),
                    code: 2,
                    status: 1,
                    value0: 12,
                    value1: 12,
                    value2: 0,
                }]),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::Detail {
                    oid: 1,
                    phase: PhaseId::EncoderCheck.to_u8(),
                    code: 2,
                    status: 1,
                    value0: 12,
                    value1: 12,
                    value2: 0,
                },
                Event::Calibrate {
                    oid: 1,
                    status: 8,
                    adc_i0: 0,
                    adc_i1: 0,
                    encoder_count: 0,
                },
            ]
        );
    }

    #[test]
    fn empty_diagnostics_still_emit_terminal_replies() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::Selftest {
                oid: 10,
                results: SelftestResults::default(),
                diagnostics: CommissionDiagnostics::default(),
                done_status: 0,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::SelftestDone { oid: 10, status: 0 }]
        );
    }

    #[test]
    fn result_variants_emit_single_result_events() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CalibrateResult {
                oid: 1,
                status: 2,
                result: AdcCalResult {
                    adc_i0: 3,
                    adc_i1: 4,
                    encoder_count: 5,
                    encoder_alignment_electrical_residual_counts: 0,
                    encoder_alignment_stability_counts: 0,
                    encoder_alignment_movement_counts: 0,
                    encoder_alignment_min_movement_counts: 0,
                    encoder_alignment_counts_per_electrical_rev: 0,
                    adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                    adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                },
                diagnostics: CommissionDiagnostics::default(),
            },
        );
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 7,
                result: commission_result(),
            },
        );
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::TuneResult {
                oid: 8,
                status: 9,
                result: outer_result(),
                safety_fault: None,
            },
        );

        let mut expected = std::vec![
            Event::Calibrate {
                oid: 1,
                status: 2,
                adc_i0: 3,
                adc_i1: 4,
                encoder_count: 5,
            },
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 0,
                phi_e_ext: 0,
                r_count_slope_milli: 1040,
                intercept_count: 5,
                rmse_permille: 7,
                selected_mask: 0b1111_1000,
                signed_count_slope_milli: 1045,
                signed_asymmetry_permille: 9,
                drift_permille: 11,
            },
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 1,
                phi_e_ext: 16_384,
                r_count_slope_milli: 1060,
                intercept_count: 6,
                rmse_permille: 8,
                selected_mask: 0b0111_1100,
                signed_count_slope_milli: 1055,
                signed_asymmetry_permille: 10,
                drift_permille: 12,
            },
            Event::ResistanceRun {
                oid: 6,
                status: 0,
                selected_r_count_slope_milli: 1050,
                gain_path_count_slope_milli: 16,
                warning_flags: 0,
                status_flags_or: 0,
                peak_abs_current_count: 1_440,
                max_abs_steady_mean_current_count: 1_280,
                current_ceiling_count: 1_600,
                power_stage_tripped: false,
            },
            Event::EncoderAlignment {
                oid: 6,
                encoder_count: 163,
                electrical_residual_counts: 3,
                stability_counts: 1,
                movement_counts: 37,
                min_movement_counts: 2,
                counts_per_electrical_rev: 80,
            },
        ];
        expected.extend(inductance_events(6));
        expected.extend(current_loop_events(6, 7));
        expected.extend([
            Event::CommissionTiming {
                oid: 6,
                method: 0,
                status: 0,
            },
            Event::CommissionTiming {
                oid: 6,
                method: 1,
                status: 0,
            },
            Event::CommissionTiming {
                oid: 6,
                method: 2,
                status: 0,
            },
            Event::Commission {
                oid: 6,
                status: 7,
                r_count_milli: 11,
                l_count_micro: 12,
                fallback_velocity_limit: 35,
            },
            Event::Tune {
                oid: 8,
                status: 9,
                velocity_p: 41,
                position_p: 44,
                warning_code: 52,
                outer_evidence_flags: 0x3f,
                stiffness_timebase_ms: 64,
                velocity_search_stop_reason: 65,
                motion_budget_mrev: 66,
            },
        ]);

        assert_eq!(sink.events, expected);
    }

    #[test]
    fn tune_result_emits_outer_safety_fault_before_terminal_result() {
        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::TuneResult {
                oid: 3,
                status: CommissionError::SafetyEnvelopeViolation.status_code(),
                result: outer_result(),
                safety_fault: Some(OuterSafetyFault {
                    reason: crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY,
                    max_travel_mrev: 750,
                    max_velocity_mrev_s: 6000,
                    max_duration_ms: 3000,
                    direction_mask: 3,
                    delta_counts: -125,
                    dt_us: 4000,
                    velocity_counts_per_ms: -31,
                    velocity_cap_counts_per_ms: 24,
                    position_counts: -373,
                    position_window_counts: 3000,
                    elapsed_us: 120_000,
                    duration_cap_us: 3_000_000,
                }),
            },
        );

        assert_eq!(
            &sink.events[..2],
            &[
                Event::OuterSafetyFault {
                    oid: 3,
                    reason: crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY,
                    delta_counts: -125,
                    velocity_counts_per_ms: -31,
                    velocity_cap_counts_per_ms: 24,
                },
                Event::Tune {
                    oid: 3,
                    status: CommissionError::SafetyEnvelopeViolation.status_code(),
                    velocity_p: 41,
                    position_p: 44,
                    warning_code: 52,
                    outer_evidence_flags: 0x3f,
                    stiffness_timebase_ms: 64,
                    velocity_search_stop_reason: 65,
                    motion_budget_mrev: 66,
                },
            ]
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn sweep_terminal_order_separates_fault_and_inconclusive_paths() {
        use crate::commissioning::types::VelocitySweepOutcome;
        let safety_fault = OuterSafetyFault {
            reason: crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY,
            max_travel_mrev: 750,
            max_velocity_mrev_s: 6_000,
            max_duration_ms: 3_000,
            direction_mask: 3,
            delta_counts: -125,
            dt_us: 4_000,
            velocity_counts_per_ms: -31,
            velocity_cap_counts_per_ms: 24,
            position_counts: -373,
            position_window_counts: 3_000,
            elapsed_us: 120_000,
            duration_cap_us: 3_000_000,
        };
        let digest = 0x0123_4567_89ab_cdef;
        let mut terminal = VelocitySweepTerminal {
            run_sequence: 7,
            evidence_sequence: 11,
            plan_digest: 0x1111_2222_3333_4444,
            outcome: VelocitySweepOutcome::Fault,
            cause: CommissionError::SafetyEnvelopeViolation.status_code(),
            recovery_unavailable: false,
            model_direction_mask: 0,
            selected_memberships: [0; 3],
            nominated_p: 0,
            selected_intervals: [[0; 2]; 2],
            selected_common_intervals: [[0; 2]; 2],
            p_start: 256,
            p_top: 1_024,
            rung_count: 5,
            coverage_mask: 0,
            directional_region_counts: [1, 2],
            directional_fragment_counts: [3, 4],
            expected_observations: 20,
            emitted_observations: 4,
            expected_rungs: 5,
            emitted_rungs: 1,
            digest,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: Some(safety_fault),
        };
        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocitySweepTerminal { oid: 3, terminal },
        );

        assert!(matches!(sink.events[0], Event::OuterSafetyFault { .. }));
        assert_eq!(
            &sink.events[1..],
            &[
                Event::StageBTerminal {
                    fragment: 0,
                    evidence_sequence: 11,
                },
                Event::StageBTerminal {
                    fragment: 1,
                    evidence_sequence: 11,
                },
                Event::StageBTerminal {
                    fragment: 2,
                    evidence_sequence: 11,
                },
                Event::StageBTerminal {
                    fragment: 3,
                    evidence_sequence: 11,
                },
            ]
        );
        assert!(
            !sink
                .events
                .iter()
                .any(|event| matches!(event, Event::Tune { .. }))
        );

        terminal.outcome = VelocitySweepOutcome::Inconclusive;
        terminal.safety_fault = None;
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocitySweepTerminal { oid: 3, terminal },
        );
        assert_eq!(
            sink.events,
            [
                Event::StageBTerminal {
                    fragment: 0,
                    evidence_sequence: 11,
                },
                Event::StageBTerminal {
                    fragment: 1,
                    evidence_sequence: 11,
                },
                Event::StageBTerminal {
                    fragment: 2,
                    evidence_sequence: 11,
                },
                Event::StageBTerminal {
                    fragment: 3,
                    evidence_sequence: 11,
                },
            ]
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    fn velocity_integral_plan_fixture()
    -> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
        use crate::commissioning::outer::OuterMotionBudget;
        use crate::commissioning::outer::velocity::integral_plan::{
            StageCAuthority, plan_velocity_integral_response,
        };
        use crate::commissioning::outer::velocity::sweep_plan::{
            RequestedVelocitySource, VelocitySweepRequest, plan_velocity_sweep,
        };

        let request = VelocitySweepRequest {
            requested_velocity_mrev_s: 2_929,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 2_000,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 50_000,
            positive_position_headroom_mrev: 50_000,
            max_duration_ms: 60_000,
        };
        let stage_b = plan_velocity_sweep(
            request,
            4_000,
            2_816,
            crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        )
        .unwrap();
        let digest = stage_b.digest();
        plan_velocity_integral_response(
            StageCAuthority {
                channel: 0,
                acquisition: StageCAcquisition::ReproducedStageB(digest),
                directional_memberships: [0x001c_0000; 2],
                stage_b_joint_membership: 0x001c_0000,
                joint_membership: 0x001c_0000,
                directional_validity: [
                    crate::commissioning::outer::velocity::reproduction::StageBDirectionalValidity::ReproducedRegionCoversFinalP;
                    2
                ],
                reduced_margin: false,
                directional_pooled_intervals: [
                    [17_000_000, 18_000_000],
                    [-19_000_000, -18_000_000],
                ],
            },
            stage_b.execution_plan(digest),
            OuterMotionBudget {
                requested_velocity_mrev_s: request.requested_velocity_mrev_s,
                machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
                requested_velocity_source: request.requested_velocity_source,
                max_stroke_travel_mrev: request.max_stroke_travel_mrev,
                settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
                negative_position_headroom_mrev: request.negative_position_headroom_mrev,
                positive_position_headroom_mrev: request.positive_position_headroom_mrev,
                max_duration_ms: request.max_duration_ms,
            },
            25_000,
            4_000,
        )
        .unwrap()
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn combined_velocity_flow_reuses_the_existing_reply_inventory() {
        let source = include_str!("replies.rs");
        for method in [
            "fn foci_commissioning_workflow_plan(",
            "fn foci_velocity_sweep_plan_core(",
            "fn foci_velocity_integral_plan_core(",
            "fn foci_velocity_integral_terminal_core(",
        ] {
            assert!(
                source.contains(method),
                "missing existing reply method {method}"
            );
        }
        assert!(!source.contains(concat!("fn foci_", "combined_")));
        assert!(!source.contains(concat!("fn foci_velocity_", "combined_")));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn velocity_integral_reply_emits_derived_evidence_before_dedicated_terminal() {
        use crate::commissioning::outer::velocity::integral_evidence::{
            ExactRateInterval, IntegralRunEvidence,
        };
        use crate::commissioning::types::{VelocityIntegralOutcome, VelocityIntegralTerminal};

        let evidence = IntegralRunEvidence::new([
            ExactRateInterval::new(100, 120).unwrap(),
            ExactRateInterval::new(-120, -100).unwrap(),
        ]);
        let terminal = VelocityIntegralTerminal {
            run_sequence: 18,
            evidence_sequence: 57,
            plan_digest: 0x1111_2222_3333_4444,
            digest: 0x5555_6666_7777_8888,
            outcome: VelocityIntegralOutcome::Inconclusive,
            cause: 7,
            breakaway_acceptance: false,
            final_p: 0,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 5,
            rest_boundary_slot_plus_one: 6,
            evidence,
            expected_observations: 56,
            emitted_observations: 56,
            expected_rungs: 7,
            emitted_rungs: 7,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
        };
        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocityIntegralTerminal {
                oid: 3,
                terminal,
                reproduction: None,
                authority: None,
            },
        );

        assert!(matches!(
            sink.events.first(),
            Some(Event::IntegralDerived(0))
        ));
        assert_eq!(
            &sink.events[sink.events.len() - 3..],
            &[
                Event::IntegralTerminal(0),
                Event::IntegralTerminal(1),
                Event::IntegralTerminal(2),
            ]
        );
        assert!(
            !sink
                .events
                .iter()
                .any(|event| matches!(event, Event::Tune { .. }))
        );
        assert_eq!(sink.integral_rest_boundary, Some([5, 6]));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn a_refused_stage_c_admission_emits_no_evidence_ahead_of_its_terminal() {
        // The ordinary terminal derives its leading fragments from run evidence.
        // A refusal happens before any stroke, so those fragments would describe
        // a run that never took place, and the host reads anything ahead of the
        // terminal as motion evidence that arrived before its exact plan.
        use crate::commission_dispatch::stage_c::missing_authority_stage_c_terminal;

        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocityIntegralRefusal {
                oid: 3,
                terminal: missing_authority_stage_c_terminal(21),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::IntegralTerminal(0),
                Event::IntegralTerminal(1),
                Event::IntegralTerminal(2),
            ]
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn velocity_integral_reply_labels_unavailable_model_comparison() {
        use crate::commissioning::outer::velocity::integral_evidence::{
            ExactRateInterval, IntegralRunEvidence,
        };
        use crate::commissioning::outer::velocity::integral_plan::STAGE_C_EVIDENCE_SCHEMA_REVISION;
        use crate::commissioning::outer::velocity::integral_reproduction::StageCAuthoritySummary;
        use crate::commissioning::outer::velocity::reproduction::{
            BUILD_FEATURES, StageBDirectionalValidity,
        };
        use crate::commissioning::types::{VelocityIntegralOutcome, VelocityIntegralTerminal};

        let mut plan = velocity_integral_plan_fixture();
        plan.authority.directional_memberships[1] = 0x0018_0000;
        plan.authority.directional_validity[1] = StageBDirectionalValidity::Fragment;
        plan.authority.directional_pooled_intervals[1] = [0, 0];
        let authority = StageCAuthoritySummary::from_plan(
            plan.authority.channel,
            BUILD_FEATURES,
            STAGE_C_EVIDENCE_SCHEMA_REVISION,
            plan.authority.acquisition,
            plan.execution_plan(),
            plan.authority,
        );
        let evidence = IntegralRunEvidence::new([
            ExactRateInterval::new(100, 120).unwrap(),
            ExactRateInterval::new(-120, -100).unwrap(),
        ]);
        let terminal = VelocityIntegralTerminal {
            run_sequence: 18,
            evidence_sequence: 57,
            plan_digest: plan.digest(),
            digest: 0x5555_6666_7777_8888,
            outcome: VelocityIntegralOutcome::Inconclusive,
            cause: 7,
            breakaway_acceptance: false,
            final_p: 0,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: plan.recovery_quantization_exposed,
            probe_constrained_test_point: plan.probe_constrained_test_point,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence,
            expected_observations: 56,
            emitted_observations: 56,
            expected_rungs: 7,
            emitted_rungs: 7,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
        };
        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocityIntegralTerminal {
                oid: 3,
                terminal,
                reproduction: None,
                authority: Some(authority),
            },
        );

        assert!(sink.events.contains(&Event::IntegralStageBComparison {
            direction: 0,
            available: 1,
        }));
        assert!(sink.events.contains(&Event::IntegralStageBComparison {
            direction: 1,
            available: 0,
        }));
        assert_eq!(sink.integral_rest_boundary, Some([0, 0]));
    }

    #[test]
    fn phase_helper_emits_zero_status_phase_reply() {
        let mut sink = RecordingSink::default();
        emit_phase_reply(&mut sink, 12, PhaseId::ClosedLoopEntry);

        assert_eq!(
            sink.events,
            std::vec![Event::Phase {
                oid: 12,
                phase: 17,
                status: 0,
            }]
        );
    }
}
