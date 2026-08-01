//! Shared commissioning reply emission.
//!
//! Board crates keep transport-specific ankyra sender calls behind
//! [`CommissionReplySink`]. This module owns the semantic mapping from terminal
//! commissioning payloads to reply events so boards cannot drift by dropping
//! fields locally.

use crate::ankyra::velocity_sweep_encoding::{
    encode_rate_values, upper_bound_i64, upper_bound_u128,
};
use crate::autotune::{CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE, PhaseId};
use crate::commission_dispatch::ReplyPayload;
use crate::commissioning::types::{
    AdcCalResult, AdcResidualEvidence, CommissionResult, CurrentValidationAxisEvidence,
    CurrentValidationSettledSampleEvidence, INDUCTANCE_SOURCE_NONE, InnerResult, OuterResult,
    OuterSafetyFault, VelocitySweepEvent, VelocitySweepObservation, VelocitySweepPlanned,
    VelocitySweepRungReport, VelocitySweepTerminal,
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

    /// Emit one velocity-sweep plan-limits fragment.
    fn foci_velocity_sweep_plan_limits(
        &mut self,
        _report: crate::ankyra::replies::FociVelocitySweepPlanLimits,
    ) {
    }

    /// Emit one velocity-sweep plan-geometry fragment.
    fn foci_velocity_sweep_plan_geometry(
        &mut self,
        _report: crate::ankyra::replies::FociVelocitySweepPlanGeometry,
    ) {
    }

    /// Emit one velocity-sweep plan-timing fragment.
    fn foci_velocity_sweep_plan_timing(
        &mut self,
        _report: crate::ankyra::replies::FociVelocitySweepPlanTiming,
    ) {
    }

    /// Emit Stage-B recovery geometry and reserved schedule.
    fn foci_velocity_sweep_plan_recovery(
        &mut self,
        _report: crate::ankyra::replies::FociVelocitySweepPlanRecovery,
    ) {
    }

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

    /// Emit one velocity-observation core fragment.
    fn foci_velocity_observation_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityObservationCore,
    ) {
    }

    /// Emit one velocity-observation rate fragment.
    fn foci_velocity_observation_rate(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityObservationRate,
    ) {
    }

    /// Emit one velocity-observation stationarity fragment.
    fn foci_velocity_observation_stationarity(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityObservationStationarity,
    ) {
    }

    /// Emit one velocity-observation disturbance fragment.
    fn foci_velocity_observation_disturbance(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityObservationDisturbance,
    ) {
    }

    /// Emit one velocity-rung consensus core fragment.
    fn foci_velocity_rung_consensus_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityRungConsensusCore,
    ) {
    }

    /// Emit one exact velocity-rung consensus component.
    fn foci_velocity_rung_consensus_component(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityRungConsensusComponent,
    ) {
    }

    /// Emit one eligible direction's velocity-rung pool.
    fn foci_velocity_rung_consensus_pool(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityRungConsensusPool,
    ) {
    }

    /// Emit one structured-boundary diagnostic.
    fn foci_velocity_structured_boundary(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStructuredBoundary,
    ) {
    }

    /// Emit one directional-region identity fragment.
    fn foci_velocity_directional_region_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityDirectionalRegionCore,
    ) {
    }

    /// Emit one directional-region model fragment.
    fn foci_velocity_directional_region_model(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityDirectionalRegionModel,
    ) {
    }

    /// Emit one directional-region settled-rate fragment.
    fn foci_velocity_directional_region_rates(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityDirectionalRegionRates,
    ) {
    }

    /// Emit one directional-region boundary fragment.
    fn foci_velocity_directional_region_boundary(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityDirectionalRegionBoundary,
    ) {
    }

    /// Emit one joint-operability region.
    fn foci_velocity_joint_region(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityJointRegion,
    ) {
    }

    /// Emit one selected Stage B handoff.
    fn foci_velocity_stage_b_handoff_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBHandoffCore,
    ) {
    }

    /// Emit one Stage B nomination fragment.
    fn foci_velocity_stage_b_nomination(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBNomination,
    ) {
    }

    /// Emit one selected directional handoff fragment.
    fn foci_velocity_stage_b_directional_handoff(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBDirectionalHandoff,
    ) {
    }

    /// Emit the Stage B reproduction class and nominated gains.
    fn foci_velocity_stage_b_reproduction_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionCore,
    ) {
    }

    /// Emit the exact Stage B reproduction memberships.
    fn foci_velocity_stage_b_reproduction_membership(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionMembership,
    ) {
    }

    /// Emit one direction's Stage B reproduction intervals.
    fn foci_velocity_stage_b_reproduction_interval(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionInterval,
    ) {
    }

    /// Emit exact Stage B reproduction digest inputs.
    fn foci_velocity_stage_b_reproduction_digest(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionDigest,
    ) {
    }

    /// Emit the schema-3 Stage B reproduction outcome and final nomination.
    fn foci_velocity_stage_b_reproduction_v3_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV3Core,
    ) {
    }

    /// Emit one schema-3 membership/core comparison.
    fn foci_velocity_stage_b_reproduction_v3_membership(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV3Membership,
    ) {
    }

    /// Emit one schema-3 pooled-interval comparison.
    fn foci_velocity_stage_b_reproduction_v3_pooled(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV3Pooled,
    ) {
    }

    /// Emit one schema-3 common-interval comparison.
    fn foci_velocity_stage_b_reproduction_v3_common(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV3Common,
    ) {
    }

    /// Emit one schema-3 core-relative coverage result.
    fn foci_velocity_stage_b_reproduction_v3_coverage(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV3Coverage,
    ) {
    }

    /// Emit exact schema-3 Stage B reproduction digest inputs.
    fn foci_velocity_stage_b_reproduction_v3_digest(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV3Digest,
    ) {
    }

    /// Emit the schema-8 Stage B reproduction outcome and selected component.
    fn foci_velocity_stage_b_reproduction_v4_core(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Core,
    ) {
    }

    /// Emit one schema-8 membership or joint-union comparison.
    fn foci_velocity_stage_b_reproduction_v4_membership(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Membership,
    ) {
    }

    /// Emit one schema-8 pooled-interval comparison.
    fn foci_velocity_stage_b_reproduction_v4_pooled(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Pooled,
    ) {
    }

    /// Emit one schema-8 common-interval comparison.
    fn foci_velocity_stage_b_reproduction_v4_common(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Common,
    ) {
    }

    /// Emit one schema-8 selected-component-relative coverage result.
    fn foci_velocity_stage_b_reproduction_v4_coverage(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Coverage,
    ) {
    }

    /// Emit exact schema-8 Stage B reproduction digest inputs.
    fn foci_velocity_stage_b_reproduction_v4_digest(
        &mut self,
        _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Digest,
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

    /// Emit one directional velocity-sweep terminal fragment.
    fn foci_velocity_sweep_terminal_direction(
        &mut self,
        _report: crate::ankyra::replies::FociVelocitySweepTerminalDirection,
    ) {
    }

    /// Emit one velocity-sweep terminal-integrity fragment.
    fn foci_velocity_sweep_terminal_integrity(
        &mut self,
        _report: crate::ankyra::replies::FociVelocitySweepTerminalIntegrity,
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

    /// Emit the generic outer inconclusive terminal.
    fn foci_outer_inconclusive(&mut self, _report: crate::ankyra::replies::FociOuterInconclusive) {}

    /// Emit the immutable breakaway-probe plan.
    fn foci_breakaway_probe_plan(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayProbePlan,
    ) {
    }

    /// Emit one direction's resolved inert/moving breakaway bracket.
    fn foci_breakaway_directional_breakaway(
        &mut self,
        _report: crate::ankyra::replies::FociBreakawayDirectionalBreakaway,
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
        family_size: report.family_size,
        max_observations: report.max_observations,
        search_count: report.search_count,
        p_start_raw: report.p_start_raw,
        p_top_raw: report.p_top_raw,
    });
}

/// Emit one direction's resolved inert/moving breakaway bracket.
#[cfg(feature = "trace-velocity-sweep")]
pub fn emit_breakaway_directional_breakaway<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::BreakawayDirectionalBreakaway,
) {
    sink.foci_breakaway_directional_breakaway(
        crate::ankyra::replies::FociBreakawayDirectionalBreakaway {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            plan_digest_low: report.plan_digest as u32,
            plan_digest_high: (report.plan_digest >> 32) as u32,
            direction: report.direction,
            inert_present: report.inert_present,
            inert_p_raw: report.inert_p_raw,
            moving_p_raw: report.moving_p_raw,
            observations: report.observations,
        },
    );
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
        family_size: report.family_size,
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
        BreakawayCampaignEvent::DirectionalBreakaway { forward, reverse } => {
            emit_breakaway_directional_breakaway(sink, oid, forward);
            emit_breakaway_directional_breakaway(sink, oid, reverse);
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
        stage_b_digest_low: plan.authority.stage_b_plan_digest as u32,
        stage_b_digest_high: (plan.authority.stage_b_plan_digest >> 32) as u32,
        build_revision: plan.build_revision,
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
        let pooled_interval = plan.combined_selected_response.map_or(
            plan.authority.directional_pooled_intervals[direction],
            |selected| {
                selected.response_intervals_q()[direction].map(|endpoint| {
                    i32::try_from(endpoint)
                        .expect("validated combined response interval remains in i32")
                })
            },
        );
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
        FociVelocityIntegralTerminalCore, FociVelocityIntegralTerminalIdentity,
        FociVelocityIntegralTerminalTiming,
    };
    use crate::commissioning::outer::velocity::integral_plan::STAGE_C_MAX_POSITIVE_RUNGS;
    use crate::commissioning::outer::velocity::integral_reporting::{
        outcome_code, stage_b_deficit_bounds_q,
    };
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

/// Emit one streamed velocity-sweep event as bounded wire fragments.
pub fn emit_velocity_sweep_event<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    event: VelocitySweepEvent,
) {
    match event {
        VelocitySweepEvent::Planned(report) => emit_velocity_sweep_plan(sink, oid, report),
        VelocitySweepEvent::Observation(report) => {
            emit_velocity_sweep_observation(sink, oid, report);
        }
        VelocitySweepEvent::Rung(report) => emit_velocity_sweep_rung(sink, oid, report),
        #[cfg(feature = "trace-velocity-sweep")]
        VelocitySweepEvent::Recovery(report) => emit_rung_origin_recovery(sink, oid, report),
        #[cfg(feature = "trace-velocity-sweep")]
        VelocitySweepEvent::RecoveryRest(_) => {}
        #[cfg(feature = "trace-velocity-sweep")]
        VelocitySweepEvent::RestSelection(_) => {}
        VelocitySweepEvent::Current(_) => {}
        VelocitySweepEvent::DirectionalRegion(report) => {
            emit_velocity_directional_region(sink, oid, report)
        }
        VelocitySweepEvent::JointRegion(report) => emit_velocity_joint_region(sink, oid, report),
        VelocitySweepEvent::Handoff(report) => emit_velocity_stage_b_handoff(sink, oid, report),
    }
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

fn emit_velocity_sweep_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: VelocitySweepPlanned,
) {
    use crate::ankyra::replies::{
        FociVelocitySweepPlanGeometry, FociVelocitySweepPlanLimits, FociVelocitySweepPlanRecovery,
        FociVelocitySweepPlanTiming,
    };

    sink.foci_velocity_sweep_plan_limits(FociVelocitySweepPlanLimits {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 0,
        requested_velocity_mrev_s: report.requested_velocity_mrev_s,
        requested_velocity_source: report.requested_velocity_source,
        planned_velocity_mrev_s: report.planned_velocity_mrev_s,
        effective_ceiling_mrev_s: report.effective_ceiling_mrev_s,
        clamp_flags: report.clamp_flags,
        binding_source: report.effective_ceiling_source,
    });
    sink.foci_velocity_sweep_plan_geometry(FociVelocitySweepPlanGeometry {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 1,
        target_velocity_rpm: report.target_velocity_rpm,
        p_start: report.p_start,
        p_top: report.p_top,
        rung_count: report.rung_count,
        observations_per_direction: report.observations_per_direction,
        max_stroke_travel_mrev: report.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: report.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: report.negative_position_headroom_mrev,
        positive_position_headroom_mrev: report.positive_position_headroom_mrev,
        hard_torque_limit: report.hard_torque_limit,
        usable_torque_limit: report.usable_torque_limit,
    });
    sink.foci_velocity_sweep_plan_timing(FociVelocitySweepPlanTiming {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 2,
        moving_stroke_us: report.moving_stroke_us,
        zero_settle_us: report.zero_settle_us,
        nominal_workflow_ms: report.nominal_workflow_ms,
        maximum_workflow_ms: report.maximum_workflow_ms,
    });
    sink.foci_velocity_sweep_plan_recovery(FociVelocitySweepPlanRecovery {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        origin_band_counts: report.recovery_origin_band_counts,
        nominal_slot_us: report.recovery_nominal_slot_us,
        maximum_slot_us: report.recovery_maximum_slot_us,
        slot_count: report.recovery_slot_count,
    });
}

fn emit_velocity_sweep_observation<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: VelocitySweepObservation,
) {
    use crate::ankyra::replies::{
        FociVelocityObservationCore, FociVelocityObservationDisturbance,
        FociVelocityObservationRate, FociVelocityObservationStationarity,
    };
    use crate::autotune::velocity_measurement::StationarityResult;

    let (delta_mantissa, delta_shift) =
        upper_bound_u128(report.selected_delta_counts.unsigned_abs() as u128);
    let (elapsed_mantissa, elapsed_shift) =
        upper_bound_u128(u128::from(report.selected_elapsed_us));
    let rate_values = [
        report.summary.settled_rate_interval_q[0],
        report.settled_rate_mean_q,
        report.summary.settled_rate_interval_q[1],
        report.summary.deficit_interval_q[0],
        report.summary.deficit_interval_q[1],
    ];
    let (rate_values, rate_shift) = encode_rate_values(rate_values);
    let (variance_mantissa, variance_shift) = upper_bound_u128(report.variance_of_mean_q2);
    let initial_flag = u8::from(report.summary.initial_lead_in);
    sink.foci_velocity_observation_core(FociVelocityObservationCore {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 0,
        rung_index: report.rung_index,
        slot: report.slot,
        classification:
            crate::commissioning::outer::velocity::sweep_evidence::observation_class_report_code(
                report.summary.classification,
            ),
        flags: initial_flag,
        delta_sign: u8::from(report.selected_delta_counts < 0),
        delta_mantissa,
        delta_shift,
        elapsed_mantissa,
        elapsed_shift,
    });
    let suffix_len = match report.stationarity {
        StationarityResult::Usable(suffix) => suffix.suffix_len,
        StationarityResult::Excluded(_) => 0,
    };
    sink.foci_velocity_observation_rate(FociVelocityObservationRate {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 1,
        rate_low: rate_values[0],
        rate_mean: rate_values[1],
        rate_high: rate_values[2],
        deficit_low: rate_values[3],
        deficit_high: rate_values[4],
        rate_shift,
        suffix_len,
        selected_level: report.selected_level,
        selected_blocks: report.selected_blocks,
        variance_mantissa,
        variance_shift,
    });
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
    sink.foci_velocity_observation_stationarity(FociVelocityObservationStationarity {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 2,
        slope_mantissa,
        slope_shift,
        slope_half_width_mantissa,
        slope_half_width_shift,
        lag_one_q,
        lag_one_half_width_q,
        residual_mantissa,
        residual_shift,
        tested_suffixes,
    });
    let disturbance = report.summary.disturbance.map_or(0, |value| value.raw());
    let (disturbance_low_q16, disturbance_high_q16) = report
        .summary
        .disturbance_interval
        .map_or((0, 0), |interval| {
            (interval.low().raw(), interval.high().raw())
        });
    let (disturbance_variance_mantissa, disturbance_variance_shift) =
        upper_bound_u128(u128::from(report.summary.disturbance_variance_q2));
    sink.foci_velocity_observation_disturbance(FociVelocityObservationDisturbance {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 3,
        velocity_p: report.summary.p.raw(),
        target_velocity_rpm: report.target_velocity_rpm,
        disturbance_q16: disturbance,
        disturbance_low_q16,
        disturbance_high_q16,
        variance_mantissa: disturbance_variance_mantissa,
        variance_shift: disturbance_variance_shift,
        predicted_torque_target_abs: report.summary.predicted_torque_target_abs,
    });
    if let Some(boundary) = report.structured_boundary {
        let (rates, rate_shift) = crate::ankyra::velocity_sweep_encoding::encode_rate_values([
            boundary.common_rate_interval_q[0],
            0,
            boundary.common_rate_interval_q[1],
            0,
            0,
        ]);
        let (slope_margin_mantissa, slope_margin_shift) =
            upper_bound_i64(boundary.binding_slope_margin_q);
        sink.foci_velocity_structured_boundary(
            crate::ankyra::replies::FociVelocityStructuredBoundary {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                rung_index: report.rung_index,
                slot: report.slot,
                passing_suffix_mask: boundary.passing_suffix_mask,
                rate_low_mantissa: rates[0],
                rate_high_mantissa: rates[2],
                rate_shift,
                slope_margin_mantissa,
                slope_margin_shift,
                lag_one_margin_q: boundary.binding_lag_one_margin_q,
            },
        );
    }
}

fn emit_velocity_directional_region<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::VelocityDirectionalRegionReport,
) {
    use crate::autotune::velocity_measurement::VelocityDirection;
    use crate::commissioning::outer::velocity::sweep_evidence::{
        directional_region_closure_code, directional_region_kind_code,
    };
    let region = report.region;
    let direction = match region.direction {
        VelocityDirection::Forward => 0,
        VelocityDirection::Reverse => 1,
    };
    let kind = directional_region_kind_code(region.kind);
    let closure = directional_region_closure_code(region.closure);
    sink.foci_velocity_directional_region_core(
        crate::ankyra::replies::FociVelocityDirectionalRegionCore {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            direction_kind_closure: direction | (kind << 1) | (closure << 2),
            member_mask: region.member_mask,
            rung_bounds: u16::from(region.first_rung) | (u16::from(region.last_rung) << 8),
            p_low: region.p_low.raw(),
            p_high: region.p_high.raw(),
            member_count: region.member_count,
            boundary_rung_plus_one: region.boundary_rung.map_or(0, |rung| rung + 1),
        },
    );
    sink.foci_velocity_directional_region_model(
        crate::ankyra::replies::FociVelocityDirectionalRegionModel {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            member_mask: region.member_mask,
            common_low_q16: region.common_intersection.low().raw(),
            common_high_q16: region.common_intersection.high().raw(),
            pooled_q16: region.pooled.estimate.raw(),
            pooled_low_q16: region.pooled.interval.low().raw(),
            pooled_high_q16: region.pooled.interval.high().raw(),
            variance_floor_observations: region.pooled.variance_floor_observations,
        },
    );
    let (rates, rate_shift) = crate::ankyra::velocity_sweep_encoding::encode_rate_values([
        region.settled_rate_mean_min_q,
        0,
        region.settled_rate_mean_max_q,
        region.settled_rate_envelope_low_q,
        region.settled_rate_envelope_high_q,
    ]);
    sink.foci_velocity_directional_region_rates(
        crate::ankyra::replies::FociVelocityDirectionalRegionRates {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            member_mask: region.member_mask,
            mean_min_mantissa: rates[0],
            mean_max_mantissa: rates[2],
            envelope_low_mantissa: rates[3],
            envelope_high_mantissa: rates[4],
            rate_shift,
        },
    );
    sink.foci_velocity_directional_region_boundary(
        crate::ankyra::replies::FociVelocityDirectionalRegionBoundary {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            member_mask: region.member_mask,
            tested_low_q16: region.tested_intersection.low().raw(),
            tested_high_q16: region.tested_intersection.high().raw(),
        },
    );
}

fn emit_velocity_joint_region<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::VelocityJointRegionReport,
) {
    let closure =
        crate::commissioning::outer::velocity::sweep_evidence::joint_operable_closure_code(
            report.region.closure,
        );
    sink.foci_velocity_joint_region(crate::ankyra::replies::FociVelocityJointRegion {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        member_mask: report.region.member_mask,
        rung_bounds: u16::from(report.region.first_rung)
            | (u16::from(report.region.last_rung) << 8),
        member_count: report.region.member_count,
        p_low: report.p_low,
        p_high: report.p_high,
        closure,
    });
}

fn emit_velocity_stage_b_handoff<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commissioning::types::VelocityStageBHandoffReport,
) {
    sink.foci_velocity_stage_b_handoff_core(
        crate::ankyra::replies::FociVelocityStageBHandoffCore {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            nominated_p: report.handoff.nomination.p.raw(),
            joint_member_mask: report.joint_union,
            forward_member_mask: report.handoff.directional[0].region.member_mask,
            reverse_member_mask: report.handoff.directional[1].region.member_mask,
            flags: report.handoff.coverage_mask
                | (u8::from(report.handoff.nomination.reduced_margin) << 2),
        },
    );
    let nomination = report.handoff.nomination;
    sink.foci_velocity_stage_b_nomination(crate::ankyra::replies::FociVelocityStageBNomination {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        nominated_rung: nomination.rung_index,
        nominated_p: nomination.p.raw(),
        selected_joint_mask: nomination.joint.member_mask,
        distance_to_start_q16: nomination.distance_to_start_num,
        distance_to_top_q16: nomination.distance_to_top_num,
        flags: u8::from(nomination.reduced_margin),
    });
    for (direction, selected) in report.handoff.directional.into_iter().enumerate() {
        let (difference_mantissa, difference_shift) =
            upper_bound_i64(selected.settled_rate_difference_q);
        sink.foci_velocity_stage_b_directional_handoff(
            crate::ankyra::replies::FociVelocityStageBDirectionalHandoff {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                direction: direction as u8,
                member_mask: selected.region.member_mask,
                region_class: u8::from(report.model_direction_mask & (1 << direction) != 0),
                coverage: crate::commissioning::outer::velocity::sweep_evidence::coverage_kind_code(
                    selected.coverage,
                ),
                signed_rung_distance: i32::from(selected.signed_rung_distance),
                gain_ratio_num: selected.gain_ratio_num,
                gain_ratio_den: selected.gain_ratio_den,
                settled_rate_difference_mantissa: difference_mantissa,
                settled_rate_difference_shift: difference_shift,
            },
        );
    }
}

fn emit_velocity_sweep_rung<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: VelocitySweepRungReport,
) {
    use crate::ankyra::replies::{
        FociVelocityRungConsensusComponent, FociVelocityRungConsensusCore,
        FociVelocityRungConsensusPool,
    };
    use crate::commissioning::outer::velocity::sweep_consensus::consensus_class_report_code;

    let directions = [report.forward, report.reverse];
    let flags = u8::from(report.forward.operable())
        | (u8::from(report.reverse.operable()) << 1)
        | (u8::from(report.forward.initial_lead_in()) << 2)
        | (u8::from(report.reverse.initial_lead_in()) << 3);
    sink.foci_velocity_rung_consensus_core(FociVelocityRungConsensusCore {
        oid,
        run_sequence: report.run_sequence,
        evidence_sequence: report.evidence_sequence,
        fragment: 0,
        rung_index: report.rung_index,
        velocity_p: report.forward.p.raw(),
        forward_class: consensus_class_report_code(report.forward.classification),
        reverse_class: consensus_class_report_code(report.reverse.classification),
        forward_collected_mask: report.forward.collected_mask,
        reverse_collected_mask: report.reverse.collected_mask,
        forward_eligible_mask: report.forward.eligible_mask,
        reverse_eligible_mask: report.reverse.eligible_mask,
        forward_included_mask: report.forward.included_mask,
        reverse_included_mask: report.reverse.included_mask,
        forward_component_count: report.forward.component_count(),
        reverse_component_count: report.reverse.component_count(),
        forward_operable_count: report.forward.operable_count,
        reverse_operable_count: report.reverse.operable_count,
        flags,
    });

    for (direction, directional) in directions.into_iter().enumerate() {
        for component_index in 0..usize::from(directional.component_count()) {
            let component = directional.components[component_index];
            sink.foci_velocity_rung_consensus_component(FociVelocityRungConsensusComponent {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                fragment: 1 + direction as u8 * 2 + component_index as u8,
                rung_index: report.rung_index,
                direction: direction as u8,
                component_index: component_index as u8,
                low_q16: component[0],
                high_q16: component[1],
            });
        }
        let Some(pooled) = directional.pooled else {
            continue;
        };
        let (rate_mantissas, rate_shift) =
            crate::commissioning::outer::velocity::sweep_evidence::common_rate_mantissas(
                directional.settled_rate_bounds_q,
            );
        sink.foci_velocity_rung_consensus_pool(FociVelocityRungConsensusPool {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            fragment: 5 + direction as u8,
            rung_index: report.rung_index,
            direction: direction as u8,
            pooled_q16: pooled.estimate.raw(),
            pooled_low_q16: pooled.interval.low().raw(),
            pooled_high_q16: pooled.interval.high().raw(),
            variance_floor_observations: pooled.variance_floor_observations,
            mean_min_mantissa: rate_mantissas[0],
            mean_max_mantissa: rate_mantissas[1],
            envelope_low_mantissa: rate_mantissas[2],
            envelope_high_mantissa: rate_mantissas[3],
            rate_shift,
        });
    }
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

#[cfg(feature = "trace-velocity-sweep")]
fn emit_velocity_stage_b_reproduction<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    report: crate::commission_dispatch::stage_b::StageBReproductionReport,
) {
    let delta = report.delta;
    sink.foci_velocity_stage_b_reproduction_v4_core(
        crate::ankyra::replies::FociVelocityStageBReproductionV4Core {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            outcome: report.outcome,
            reason_mask: delta.reason_mask,
            previous_provisional_p: delta.previous_provisional_p,
            current_provisional_p: delta.current_provisional_p,
            final_p: delta.final_p,
            selected_joint_mask: delta.selected_joint.member_mask,
            selected_first_rung: delta.selected_joint.first_rung,
            selected_member_count: delta.selected_joint.member_count,
            forward_validity: delta.directional_validity[0] as u8,
            reverse_validity: delta.directional_validity[1] as u8,
            reduced_margin: u8::from(delta.reduced_margin),
            schema_revision:
                crate::commissioning::outer::velocity::reproduction::STAGE_B_EVIDENCE_SCHEMA_REVISION,
        },
    );
    for (object, membership) in delta.memberships.into_iter().enumerate() {
        sink.foci_velocity_stage_b_reproduction_v4_membership(
            crate::ankyra::replies::FociVelocityStageBReproductionV4Membership {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                object: object as u8,
                previous_mask: membership.previous,
                current_mask: membership.current,
                core_mask: membership.core,
                previous_only_mask: membership.previous_only,
                current_only_mask: membership.current_only,
                low_delta: i32::from(membership.low_delta),
                high_delta: i32::from(membership.high_delta),
            },
        );
    }
    for (direction, interval) in delta.intervals.into_iter().enumerate() {
        sink.foci_velocity_stage_b_reproduction_v4_pooled(
            crate::ankyra::replies::FociVelocityStageBReproductionV4Pooled {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                direction: direction as u8,
                previous_low_q16: interval.previous_pooled[0],
                previous_high_q16: interval.previous_pooled[1],
                current_low_q16: interval.current_pooled[0],
                current_high_q16: interval.current_pooled[1],
                overlap_low_q16: interval.pooled_overlap[0],
                overlap_high_q16: interval.pooled_overlap[1],
            },
        );
    }
    for (direction, interval) in delta.intervals.into_iter().enumerate() {
        sink.foci_velocity_stage_b_reproduction_v4_common(
            crate::ankyra::replies::FociVelocityStageBReproductionV4Common {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                direction: direction as u8,
                previous_low_q16: interval.previous_common[0],
                previous_high_q16: interval.previous_common[1],
                current_low_q16: interval.current_common[0],
                current_high_q16: interval.current_common[1],
                conservative_low_q16: interval.conservative_common[0],
                conservative_high_q16: interval.conservative_common[1],
                nonempty: u8::from(interval.conservative_common_nonempty),
            },
        );
    }
    for (direction, coverage) in delta.coverage.into_iter().enumerate() {
        sink.foci_velocity_stage_b_reproduction_v4_coverage(
            crate::ankyra::replies::FociVelocityStageBReproductionV4Coverage {
                oid,
                run_sequence: report.run_sequence,
                evidence_sequence: report.evidence_sequence,
                direction: direction as u8,
                coverage: crate::commissioning::outer::velocity::sweep_evidence::coverage_kind_code(
                    coverage.kind,
                ),
                signed_rung_distance: i32::from(coverage.signed_rung_distance),
                gain_ratio_num: coverage.gain_ratio_num,
                gain_ratio_den: coverage.gain_ratio_den,
            },
        );
    }
    sink.foci_velocity_stage_b_reproduction_v4_digest(
        crate::ankyra::replies::FociVelocityStageBReproductionV4Digest {
            oid,
            run_sequence: report.run_sequence,
            evidence_sequence: report.evidence_sequence,
            previous_digest_low: delta.previous_run_digest as u32,
            previous_digest_high: (delta.previous_run_digest >> 32) as u32,
            current_digest_low: delta.current_run_digest as u32,
            current_digest_high: (delta.current_run_digest >> 32) as u32,
        },
    );
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
        ReplyPayload::VelocitySweepEvent { oid, event } => {
            emit_velocity_sweep_event(sink, oid, event);
        }
        ReplyPayload::VelocitySweepTerminal {
            oid,
            terminal,
            #[cfg(feature = "trace-velocity-sweep")]
            reproduction,
        } => {
            if let Some(fault) = terminal.safety_fault {
                sink.foci_outer_safety_fault(oid, fault);
            }
            #[cfg(feature = "trace-velocity-sweep")]
            if let Some(report) = reproduction {
                emit_velocity_stage_b_reproduction(sink, oid, report);
            }
            emit_velocity_sweep_terminal_fragments(sink, oid, terminal);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::CommissioningWorkflowPlan { oid, plan } => {
            emit_commissioning_workflow_plan(sink, oid, plan);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::VelocityIntegralContinuation {
            oid,
            run_sequence,
            stage_b_terminal,
            stage_b_reproduction,
            stage_c_plan,
        } => {
            if let Some(report) = stage_b_reproduction {
                emit_velocity_stage_b_reproduction(sink, oid, report);
            }
            emit_velocity_sweep_terminal_fragments(sink, oid, stage_b_terminal);
            emit_velocity_integral_plan(sink, oid, run_sequence, stage_c_plan);
        }
        #[cfg(feature = "trace-velocity-sweep")]
        ReplyPayload::VelocityIntegralAdmissionFailure {
            oid,
            stage_b_terminal,
            stage_b_reproduction,
            stage_c_terminal,
        } => {
            if let Some(report) = stage_b_reproduction {
                emit_velocity_stage_b_reproduction(sink, oid, report);
            }
            emit_velocity_sweep_terminal_fragments(sink, oid, stage_b_terminal);
            emit_velocity_integral_terminal(sink, oid, stage_c_terminal, None, None);
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

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ConsensusEvent {
        Core {
            fragment: u8,
            classes: [u8; 2],
            collected: [u8; 2],
            eligible: [u8; 2],
            included: [u8; 2],
            components: [u8; 2],
            operable: [u8; 2],
            flags: u8,
        },
        Component {
            fragment: u8,
            direction: u8,
            index: u8,
            bounds: [i32; 2],
        },
        Pool {
            fragment: u8,
            direction: u8,
            point: i32,
            interval: [i32; 2],
            variance_floor: u8,
            rates: [i32; 4],
            rate_shift: u8,
        },
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
        SweepPlan(u8),
        SweepPlanRecovery,
        SweepObservation(u8),
        SweepRung(ConsensusEvent),
        SweepStructuredBoundary,
        SweepDirectionalRegionCore(u32),
        SweepDirectionalRegionModel(u32),
        SweepDirectionalRegionRates(u32),
        SweepDirectionalRegionBoundary(u32),
        SweepJointRegion(u32),
        SweepHandoff {
            nominated_p: u16,
            joint_union: u32,
        },
        SweepNomination {
            nominated_p: u16,
            selected_joint: u32,
        },
        SweepDirectionalHandoff {
            direction: u8,
            member_mask: u32,
            region_class: u8,
        },
        StageBReproduction(u8),
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
        OuterInconclusive,
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
        integral_target: Option<[u8; 2]>,
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

        fn foci_velocity_sweep_plan_limits(
            &mut self,
            report: crate::ankyra::replies::FociVelocitySweepPlanLimits,
        ) {
            self.events.push(Event::SweepPlan(report.fragment));
        }

        fn foci_velocity_sweep_plan_geometry(
            &mut self,
            report: crate::ankyra::replies::FociVelocitySweepPlanGeometry,
        ) {
            self.events.push(Event::SweepPlan(report.fragment));
        }

        fn foci_velocity_sweep_plan_timing(
            &mut self,
            report: crate::ankyra::replies::FociVelocitySweepPlanTiming,
        ) {
            self.events.push(Event::SweepPlan(report.fragment));
        }

        fn foci_velocity_sweep_plan_recovery(
            &mut self,
            _report: crate::ankyra::replies::FociVelocitySweepPlanRecovery,
        ) {
            self.events.push(Event::SweepPlanRecovery);
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

        fn foci_velocity_observation_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityObservationCore,
        ) {
            self.events.push(Event::SweepObservation(report.fragment));
        }

        fn foci_velocity_observation_rate(
            &mut self,
            report: crate::ankyra::replies::FociVelocityObservationRate,
        ) {
            self.events.push(Event::SweepObservation(report.fragment));
        }

        fn foci_velocity_observation_stationarity(
            &mut self,
            report: crate::ankyra::replies::FociVelocityObservationStationarity,
        ) {
            self.events.push(Event::SweepObservation(report.fragment));
        }

        fn foci_velocity_observation_disturbance(
            &mut self,
            report: crate::ankyra::replies::FociVelocityObservationDisturbance,
        ) {
            self.events.push(Event::SweepObservation(report.fragment));
        }

        fn foci_velocity_rung_consensus_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityRungConsensusCore,
        ) {
            self.events.push(Event::SweepRung(ConsensusEvent::Core {
                fragment: report.fragment,
                classes: [report.forward_class, report.reverse_class],
                collected: [report.forward_collected_mask, report.reverse_collected_mask],
                eligible: [report.forward_eligible_mask, report.reverse_eligible_mask],
                included: [report.forward_included_mask, report.reverse_included_mask],
                components: [
                    report.forward_component_count,
                    report.reverse_component_count,
                ],
                operable: [report.forward_operable_count, report.reverse_operable_count],
                flags: report.flags,
            }));
        }

        fn foci_velocity_rung_consensus_component(
            &mut self,
            report: crate::ankyra::replies::FociVelocityRungConsensusComponent,
        ) {
            self.events
                .push(Event::SweepRung(ConsensusEvent::Component {
                    fragment: report.fragment,
                    direction: report.direction,
                    index: report.component_index,
                    bounds: [report.low_q16, report.high_q16],
                }));
        }

        fn foci_velocity_rung_consensus_pool(
            &mut self,
            report: crate::ankyra::replies::FociVelocityRungConsensusPool,
        ) {
            self.events.push(Event::SweepRung(ConsensusEvent::Pool {
                fragment: report.fragment,
                direction: report.direction,
                point: report.pooled_q16,
                interval: [report.pooled_low_q16, report.pooled_high_q16],
                variance_floor: report.variance_floor_observations,
                rates: [
                    report.mean_min_mantissa,
                    report.mean_max_mantissa,
                    report.envelope_low_mantissa,
                    report.envelope_high_mantissa,
                ],
                rate_shift: report.rate_shift,
            }));
        }

        fn foci_velocity_structured_boundary(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityStructuredBoundary,
        ) {
            self.events.push(Event::SweepStructuredBoundary);
        }

        fn foci_velocity_directional_region_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityDirectionalRegionCore,
        ) {
            self.events
                .push(Event::SweepDirectionalRegionCore(report.member_mask));
        }

        fn foci_velocity_directional_region_model(
            &mut self,
            report: crate::ankyra::replies::FociVelocityDirectionalRegionModel,
        ) {
            self.events
                .push(Event::SweepDirectionalRegionModel(report.member_mask));
        }

        fn foci_velocity_directional_region_rates(
            &mut self,
            report: crate::ankyra::replies::FociVelocityDirectionalRegionRates,
        ) {
            self.events
                .push(Event::SweepDirectionalRegionRates(report.member_mask));
        }

        fn foci_velocity_directional_region_boundary(
            &mut self,
            report: crate::ankyra::replies::FociVelocityDirectionalRegionBoundary,
        ) {
            self.events
                .push(Event::SweepDirectionalRegionBoundary(report.member_mask));
        }

        fn foci_velocity_joint_region(
            &mut self,
            report: crate::ankyra::replies::FociVelocityJointRegion,
        ) {
            self.events
                .push(Event::SweepJointRegion(report.member_mask));
        }

        fn foci_velocity_stage_b_handoff_core(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBHandoffCore,
        ) {
            self.events.push(Event::SweepHandoff {
                nominated_p: report.nominated_p,
                joint_union: report.joint_member_mask,
            });
        }

        fn foci_velocity_stage_b_nomination(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBNomination,
        ) {
            self.events.push(Event::SweepNomination {
                nominated_p: report.nominated_p,
                selected_joint: report.selected_joint_mask,
            });
        }

        fn foci_velocity_stage_b_directional_handoff(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBDirectionalHandoff,
        ) {
            self.events.push(Event::SweepDirectionalHandoff {
                direction: report.direction,
                member_mask: report.member_mask,
                region_class: report.region_class,
            });
        }

        fn foci_velocity_stage_b_reproduction_core(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityStageBReproductionCore,
        ) {
            self.events.push(Event::StageBReproduction(0));
        }

        fn foci_velocity_stage_b_reproduction_membership(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityStageBReproductionMembership,
        ) {
            self.events.push(Event::StageBReproduction(1));
        }

        fn foci_velocity_stage_b_reproduction_interval(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBReproductionInterval,
        ) {
            self.events
                .push(Event::StageBReproduction(report.direction + 2));
        }

        fn foci_velocity_stage_b_reproduction_digest(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityStageBReproductionDigest,
        ) {
            self.events.push(Event::StageBReproduction(4));
        }

        fn foci_velocity_stage_b_reproduction_v4_core(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Core,
        ) {
            self.events.push(Event::StageBReproduction(10));
        }

        fn foci_velocity_stage_b_reproduction_v4_membership(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBReproductionV4Membership,
        ) {
            self.events
                .push(Event::StageBReproduction(11 + report.object));
        }

        fn foci_velocity_stage_b_reproduction_v4_pooled(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBReproductionV4Pooled,
        ) {
            self.events
                .push(Event::StageBReproduction(14 + report.direction));
        }

        fn foci_velocity_stage_b_reproduction_v4_common(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBReproductionV4Common,
        ) {
            self.events
                .push(Event::StageBReproduction(16 + report.direction));
        }

        fn foci_velocity_stage_b_reproduction_v4_coverage(
            &mut self,
            report: crate::ankyra::replies::FociVelocityStageBReproductionV4Coverage,
        ) {
            self.events
                .push(Event::StageBReproduction(18 + report.direction));
        }

        fn foci_velocity_stage_b_reproduction_v4_digest(
            &mut self,
            _report: crate::ankyra::replies::FociVelocityStageBReproductionV4Digest,
        ) {
            self.events.push(Event::StageBReproduction(20));
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
                if report.recovery_flags
                    & crate::commissioning::outer::velocity::combined_plan::TERMINAL_COMBINED_WORKFLOW
                    != 0
                {
                    let encoded = (report.recovery_flags
                        >> crate::commissioning::outer::velocity::combined_plan::TERMINAL_COMBINED_TARGET_SHIFT)
                        & 0x07;
                    self.integral_target = (encoded != 0)
                        .then_some([encoded - 1, report.rest_boundary_rung_plus_one]);
                }
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

        fn foci_outer_inconclusive(
            &mut self,
            _report: crate::ankyra::replies::FociOuterInconclusive,
        ) {
            self.events.push(Event::OuterInconclusive);
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
        let mut events = Vec::new();
        events.push(Event::CurrentValidationEnvelope {
            oid,
            step_amplitude: 128,
            flux_step_amplitude: 128,
            torque_step_amplitude: 128,
            pidout_limit: 2400,
            current_limited: 422,
            voltage_limited: 128,
            max_p: 3474,
        });
        events.push(Event::CurrentValidationSettledSample {
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
        });
        events.push(current_validation_event(
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
        ));
        events.push(current_validation_event(
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
        ));
        events.push(Event::CurrentLoopHold {
            oid,
            hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            sample_count: 0,
            torque_rms_count: 0,
            flux_rms_count: 0,
        });
        events.push(Event::ClosedLoopEntry {
            oid,
            entry_status: crate::autotune::CLOSED_LOOP_ENTRY_STATUS_NOT_RUN,
            position_1: 0,
            position_2: 0,
            drift_count: 0,
            threshold_count: 0,
            runaway: 0,
        });
        events.push(Event::CurrentLoopFilters {
            oid,
            velocity_filter_hz: 0,
            torque_filter_hz: 3000,
            position_filter_hz: 0,
            flux_filter_hz: 3000,
        });
        events.push(Event::CurrentLoopRun {
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
            current_validation_axes: CURRENT_VALIDATION_AXIS_FLUX | CURRENT_VALIDATION_AXIS_TORQUE,
            flux_validation_sample_count: 1,
            torque_validation_sample_count: 1,
            retry_budget_exhausted: 0,
            failure_reason: CURRENT_LOOP_FAILURE_NONE,
        });
        events
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

    #[test]
    fn stage_b_region_and_handoff_groups_emit_all_causal_fragments() {
        use crate::autotune::velocity_measurement::VelocityDirection;
        use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
        use crate::commissioning::outer::velocity::operating_region::{
            BoundRounding, CompactSignedRate, CoverageKind, DirectionalRegionClosure,
            DirectionalRegionKind, DirectionalRegionRef, DirectionalRegionSummary,
            JointOperableClosure, JointOperableRegion, ProvisionalPNomination,
            SelectedDirectionalRegion, StageBHandoff,
        };
        use crate::commissioning::outer::velocity::sweep_evidence::PooledDisturbance;
        use crate::commissioning::types::{
            VelocityDirectionalRegionReport, VelocityJointRegionReport,
            VelocityStageBHandoffReport, VelocitySweepEvent,
        };
        use foci_tmc4671::pid::PidPGain;

        let disturbance = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(100),
            EquivalentTorqueTargetQ16::from_raw(110),
        )
        .unwrap();
        let membership = 0b1_1100;
        let region = DirectionalRegionSummary {
            direction: VelocityDirection::Forward,
            member_mask: membership,
            first_rung: 2,
            last_rung: 4,
            p_low: PidPGain::new(512).unwrap(),
            p_high: PidPGain::new(1_024).unwrap(),
            member_count: 3,
            kind: DirectionalRegionKind::Valid,
            common_intersection: disturbance,
            pooled: PooledDisturbance {
                estimate: EquivalentTorqueTargetQ16::from_raw(105),
                interval: disturbance,
                variance_floor_observations: 0,
            },
            settled_rate_mean_min_q: 1_000,
            settled_rate_mean_max_q: 1_100,
            settled_rate_envelope_low_q: 900,
            settled_rate_envelope_high_q: 1_200,
            closure: DirectionalRegionClosure::EndOfLadder,
            boundary_rung: None,
            tested_intersection: disturbance,
        };
        let joint = JointOperableRegion {
            member_mask: membership,
            first_rung: 2,
            last_rung: 4,
            member_count: 3,
            closure: JointOperableClosure::EndOfLadder,
        };
        let region_ref = DirectionalRegionRef {
            member_mask: membership,
            pooled_interval: [100, 110],
            common_intersection: [102, 108],
            selected_edge_rate: CompactSignedRate::encode(
                1_000,
                BoundRounding::TowardNegativeInfinity,
            ),
        };
        let selected = SelectedDirectionalRegion {
            region: region_ref,
            coverage: CoverageKind::FullCoverage,
            signed_rung_distance: 0,
            gain_ratio_num: 1,
            gain_ratio_den: 1,
            settled_rate_difference_q: 0,
        };
        let handoff = StageBHandoff {
            nomination: ProvisionalPNomination {
                joint,
                rung_index: 3,
                p: PidPGain::new(724).unwrap(),
                distance_to_start_num: 2 << 16,
                distance_to_top_num: 3 << 16,
                reduced_margin: false,
            },
            directional: [selected; 2],
            coverage_mask: 3,
        };
        let mut sink = RecordingSink::default();

        emit_velocity_sweep_event(
            &mut sink,
            9,
            VelocitySweepEvent::DirectionalRegion(VelocityDirectionalRegionReport {
                run_sequence: 7,
                evidence_sequence: 4,
                region,
            }),
        );
        emit_velocity_sweep_event(
            &mut sink,
            9,
            VelocitySweepEvent::JointRegion(VelocityJointRegionReport {
                run_sequence: 7,
                evidence_sequence: 5,
                region: joint,
                p_low: 512,
                p_high: 1_024,
            }),
        );
        emit_velocity_sweep_event(
            &mut sink,
            9,
            VelocitySweepEvent::Handoff(VelocityStageBHandoffReport {
                run_sequence: 7,
                evidence_sequence: 6,
                joint_union: membership | 1,
                model_direction_mask: 0b11,
                handoff,
            }),
        );

        assert_eq!(
            sink.events,
            [
                Event::SweepDirectionalRegionCore(membership),
                Event::SweepDirectionalRegionModel(membership),
                Event::SweepDirectionalRegionRates(membership),
                Event::SweepDirectionalRegionBoundary(membership),
                Event::SweepJointRegion(membership),
                Event::SweepHandoff {
                    nominated_p: 724,
                    joint_union: membership | 1,
                },
                Event::SweepNomination {
                    nominated_p: 724,
                    selected_joint: membership,
                },
                Event::SweepDirectionalHandoff {
                    direction: 0,
                    member_mask: membership,
                    region_class: 1,
                },
                Event::SweepDirectionalHandoff {
                    direction: 1,
                    member_mask: membership,
                    region_class: 1,
                },
            ]
        );
    }

    fn consensus_report(
        class: crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass,
        components: [[i32; 2]; 2],
        masks: [u8; 3],
        operable_count: u8,
        flags: u8,
    ) -> crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport {
        use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
        use crate::commissioning::outer::velocity::operating_region::{
            BoundRounding, CompactSignedRate,
        };
        use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
        use crate::commissioning::outer::velocity::sweep_evidence::PooledDisturbance;

        let pooled = (class == DirectionalConsensusClass::EligibleConsensus).then(|| {
            let interval = EquivalentTorqueTargetInterval::new(
                EquivalentTorqueTargetQ16::from_raw(components[0][0]),
                EquivalentTorqueTargetQ16::from_raw(components[0][1]),
            )
            .unwrap();
            PooledDisturbance {
                estimate: EquivalentTorqueTargetQ16::from_raw(
                    (components[0][0] + components[0][1]) / 2,
                ),
                interval,
                variance_floor_observations: 1,
            }
        });
        crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport {
            p: foci_tmc4671::pid::PidPGain::new(724).unwrap(),
            classification: class,
            components,
            pooled,
            settled_rate_bounds_q: [
                CompactSignedRate::encode(100, BoundRounding::TowardNegativeInfinity),
                CompactSignedRate::encode(110, BoundRounding::TowardPositiveInfinity),
                CompactSignedRate::encode(90, BoundRounding::TowardNegativeInfinity),
                CompactSignedRate::encode(120, BoundRounding::TowardPositiveInfinity),
            ],
            collected_mask: masks[0],
            eligible_mask: masks[1],
            included_mask: masks[2],
            operable_count,
            flags,
        }
    }

    #[test]
    fn velocity_sweep_reply_tests_preserve_consensus_groups_exactly() {
        use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
        use crate::commissioning::outer::velocity::sweep_evidence::{
            DIRECTIONAL_RUNG_FLAG_INITIAL_LEAD_IN, DIRECTIONAL_RUNG_FLAG_OPERABLE,
        };

        let eligible = consensus_report(
            DirectionalConsensusClass::EligibleConsensus,
            [[100, 120], [0, 0]],
            [0b1111, 0b1111, 0b0111],
            4,
            DIRECTIONAL_RUNG_FLAG_OPERABLE | DIRECTIONAL_RUNG_FLAG_INITIAL_LEAD_IN,
        );
        let ambiguous = consensus_report(
            DirectionalConsensusClass::AmbiguousConsensus,
            [[-140, -130], [-110, -100]],
            [0b1111, 0b1111, 0],
            3,
            DIRECTIONAL_RUNG_FLAG_OPERABLE,
        );
        let report = VelocitySweepRungReport {
            run_sequence: 7,
            evidence_sequence: 9,
            rung_index: 4,
            forward: eligible,
            reverse: ambiguous,
        };
        let mut sink = RecordingSink::default();

        emit_velocity_sweep_rung(&mut sink, 3, report);

        assert_eq!(sink.events.len(), 5);
        assert_eq!(
            sink.events[0],
            Event::SweepRung(ConsensusEvent::Core {
                fragment: 0,
                classes: [0, 1],
                collected: [0b1111, 0b1111],
                eligible: [0b1111, 0b1111],
                included: [0b0111, 0],
                components: [1, 2],
                operable: [4, 3],
                flags: 0b0111,
            })
        );
        assert_eq!(
            &sink.events[1..],
            &[
                Event::SweepRung(ConsensusEvent::Component {
                    fragment: 1,
                    direction: 0,
                    index: 0,
                    bounds: [100, 120],
                }),
                Event::SweepRung(ConsensusEvent::Pool {
                    fragment: 5,
                    direction: 0,
                    point: 110,
                    interval: [100, 120],
                    variance_floor: 1,
                    rates: [100, 110, 90, 120],
                    rate_shift: 0,
                }),
                Event::SweepRung(ConsensusEvent::Component {
                    fragment: 3,
                    direction: 1,
                    index: 0,
                    bounds: [-140, -130],
                }),
                Event::SweepRung(ConsensusEvent::Component {
                    fragment: 4,
                    direction: 1,
                    index: 1,
                    bounds: [-110, -100],
                }),
            ]
        );

        sink.events.clear();
        let incomplete = consensus_report(
            DirectionalConsensusClass::IncompleteRung,
            [[0; 2]; 2],
            [0b0111, 0b0111, 0],
            2,
            0,
        );
        emit_velocity_sweep_rung(
            &mut sink,
            3,
            VelocitySweepRungReport {
                run_sequence: 7,
                evidence_sequence: 10,
                rung_index: 5,
                forward: incomplete,
                reverse: incomplete,
            },
        );
        assert_eq!(sink.events.len(), 1);
        assert!(matches!(
            sink.events[0],
            Event::SweepRung(ConsensusEvent::Core {
                classes: [3, 3],
                components: [0, 0],
                ..
            })
        ));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn rung_origin_recovery_reply_is_shared_and_causally_complete() {
        use crate::commissioning::outer::velocity::recovery::{
            ExactU64Parts, RecoveryBindingSource, RecoveryStage, RungOriginRecoveryOutcome,
            RungOriginRecoveryReport,
        };
        use crate::commissioning::types::{
            RungOriginRecoveryEvidence, VelocityIntegralEvent, VelocitySweepEvent,
        };

        let evidence = RungOriginRecoveryEvidence {
            run_sequence: 7,
            evidence_sequence: 10,
            report: RungOriginRecoveryReport {
                stage: RecoveryStage::Proportional,
                rung_index: 4,
                p_raw: 724,
                planned_velocity_mrev_s: 2_929,
                start_offset_counts: 8_458,
                closest_offset_counts: 900,
                final_offset_counts: 950,
                origin_band_counts: 1_000,
                lower_rate_q: ExactU64Parts::from_u64(0x0123_4567_89ab_cdef),
                moving_duration_us: 350_000,
                settle_duration_us: 500_000,
                total_duration_us: 850_000,
                peak_torque_target_abs: 2_533,
                binding_source: RecoveryBindingSource::SettledRate,
                outcome: RungOriginRecoveryOutcome::Recovered,
            },
        };
        let mut sink = RecordingSink::default();
        let canonical =
            crate::commissioning::outer::velocity::sweep_reporting::encode_rung_origin_recovery(
                evidence,
            );
        let integral_evidence = RungOriginRecoveryEvidence {
            report: RungOriginRecoveryReport {
                stage: RecoveryStage::Integral,
                ..evidence.report
            },
            ..evidence
        };

        emit_velocity_sweep_event(&mut sink, 3, VelocitySweepEvent::Recovery(evidence));
        emit_velocity_integral_event(
            &mut sink,
            3,
            VelocityIntegralEvent::Recovery(integral_evidence),
        );

        assert_eq!(canonical.as_bytes().len(), 55);
        assert!(
            canonical
                .as_bytes()
                .windows(8)
                .any(|word| word == 0x0123_4567_89ab_cdef_u64.to_le_bytes())
        );
        assert_eq!(
            sink.events,
            [
                Event::Recovery {
                    oid: 3,
                    run_sequence: 7,
                    evidence_sequence: 10,
                    stage: RecoveryStage::Proportional as u8,
                    rung_index: 4,
                    p_raw: 724,
                    binding_source: RecoveryBindingSource::SettledRate as u8,
                    outcome: RungOriginRecoveryOutcome::Recovered as u8,
                },
                Event::Recovery {
                    oid: 3,
                    run_sequence: 7,
                    evidence_sequence: 10,
                    stage: RecoveryStage::Integral as u8,
                    rung_index: 4,
                    p_raw: 724,
                    binding_source: RecoveryBindingSource::SettledRate as u8,
                    outcome: RungOriginRecoveryOutcome::Recovered as u8,
                },
            ]
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn primitive_current_events_are_trace_only() {
        use crate::commissioning::types::{
            VelocityIntegralEvent, VelocityPrimitiveCurrentEvidence, VelocitySweepEvent,
        };

        let evidence = VelocityPrimitiveCurrentEvidence {
            run_sequence: 7,
            evidence_sequence: 2,
            stage: 0,
            rung_index: 1,
            slot: 3,
            clamp_limit: 2_534,
            clamp_readback: 2_534,
            moving_pid_output_peak: 1_200,
            zero_pid_output_peak: 1_400,
            moving_status_flags: 0,
            zero_status_flags: 0,
            capability: 0,
            contact: 0,
        };
        let mut sweep_sink = RecordingSink::default();
        let mut integral_sink = RecordingSink::default();

        emit_velocity_sweep_event(&mut sweep_sink, 3, VelocitySweepEvent::Current(evidence));
        emit_velocity_integral_event(
            &mut integral_sink,
            3,
            VelocityIntegralEvent::Current(VelocityPrimitiveCurrentEvidence {
                stage: 1,
                ..evidence
            }),
        );

        assert!(sweep_sink.events.is_empty());
        assert!(integral_sink.events.is_empty());
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn velocity_sweep_plan_reports_recovery_geometry_before_motion() {
        let report = VelocitySweepPlanned {
            plan_digest: 0x0123_4567_89ab_cdef,
            run_sequence: 7,
            evidence_sequence: 0,
            requested_velocity_mrev_s: 4_394,
            requested_velocity_source: 1,
            planned_velocity_mrev_s: 2_929,
            effective_ceiling_mrev_s: 10_000,
            clamp_flags: 0,
            effective_ceiling_source: 0,
            target_velocity_rpm: 176,
            p_start: 2,
            p_top: 32_767,
            rung_count: 23,
            observations_per_direction: 4,
            moving_stroke_us: 512_000,
            zero_settle_us: 500_000,
            nominal_workflow_ms: 271_700,
            maximum_workflow_ms: 298_632,
            recovery_origin_band_counts: 1_000,
            recovery_nominal_slot_us: 1_816_958,
            recovery_maximum_slot_us: 3_000_000,
            recovery_slot_count: 23,
            max_stroke_travel_mrev: 2_500,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 50_000,
            positive_position_headroom_mrev: 50_000,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
        };
        let mut sink = RecordingSink::default();

        emit_velocity_sweep_event(&mut sink, 3, VelocitySweepEvent::Planned(report));

        assert_eq!(
            sink.events,
            [
                Event::SweepPlan(0),
                Event::SweepPlan(1),
                Event::SweepPlan(2),
                Event::SweepPlanRecovery,
            ]
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn reproduction_group_emits_before_the_authoritative_terminal() {
        use crate::commission_dispatch::stage_b::StageBReproductionReport;
        use crate::commissioning::outer::velocity::operating_region::CoverageKind;
        use crate::commissioning::outer::velocity::reproduction::{
            StageBCoreCoverage, StageBDirectionalValidity, StageBIntervalComparison,
            StageBMembershipComparison, StageBReproductionDelta, StageBSelectedJointComponent,
        };
        use crate::commissioning::types::VelocitySweepOutcome;

        let terminal = VelocitySweepTerminal {
            run_sequence: 8,
            evidence_sequence: 13,
            plan_digest: 0x1111_2222_3333_4444,
            outcome: VelocitySweepOutcome::Complete,
            cause: 0,
            recovery_unavailable: false,
            model_direction_mask: 3,
            selected_memberships: [0b11100; 3],
            nominated_p: 724,
            selected_intervals: [[10, 20], [-20, -10]],
            selected_common_intervals: [[12, 18], [-18, -12]],
            p_start: 256,
            p_top: 1_024,
            rung_count: 5,
            coverage_mask: 3,
            directional_region_counts: [1, 1],
            directional_fragment_counts: [0, 0],
            expected_observations: 20,
            emitted_observations: 20,
            expected_rungs: 5,
            emitted_rungs: 5,
            digest: 0x0123_4567_89ab_cdef,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
        };
        let reproduction = StageBReproductionReport {
            run_sequence: 8,
            evidence_sequence: 12,
            outcome: 1,
            delta: StageBReproductionDelta {
                memberships: [StageBMembershipComparison {
                    previous: 0b11100,
                    current: 0b11100,
                    core: 0b11100,
                    previous_only: 0,
                    current_only: 0,
                    low_delta: 0,
                    high_delta: 0,
                }; 3],
                intervals: [
                    StageBIntervalComparison {
                        previous_pooled: [9, 19],
                        current_pooled: [10, 20],
                        pooled_overlap: [10, 19],
                        previous_common: [11, 18],
                        current_common: [12, 18],
                        conservative_common: [12, 18],
                        conservative_common_nonempty: true,
                    },
                    StageBIntervalComparison {
                        previous_pooled: [-21, -11],
                        current_pooled: [-20, -10],
                        pooled_overlap: [-20, -11],
                        previous_common: [-19, -12],
                        current_common: [-18, -12],
                        conservative_common: [-18, -12],
                        conservative_common_nonempty: true,
                    },
                ],
                selected_joint: StageBSelectedJointComponent {
                    member_mask: 0b11100,
                    first_rung: 2,
                    member_count: 3,
                },
                directional_validity: [StageBDirectionalValidity::ReproducedRegionCoversFinalP; 2],
                previous_provisional_p: 724,
                current_provisional_p: 724,
                final_p: 724,
                coverage: [StageBCoreCoverage {
                    kind: CoverageKind::FullCoverage,
                    signed_rung_distance: 0,
                    gain_ratio_num: 1,
                    gain_ratio_den: 1,
                }; 2],
                reduced_margin: false,
                reason_mask: 0,
                previous_run_digest: 1,
                current_run_digest: 2,
            },
        };
        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocitySweepTerminal {
                oid: 3,
                terminal,
                reproduction: Some(reproduction),
            },
        );

        assert_eq!(
            sink.events,
            [
                Event::StageBReproduction(10),
                Event::StageBReproduction(11),
                Event::StageBReproduction(12),
                Event::StageBReproduction(13),
                Event::StageBReproduction(14),
                Event::StageBReproduction(15),
                Event::StageBReproduction(16),
                Event::StageBReproduction(17),
                Event::StageBReproduction(18),
                Event::StageBReproduction(19),
                Event::StageBReproduction(20),
                Event::StageBTerminal {
                    fragment: 0,
                    evidence_sequence: 13,
                },
                Event::StageBTerminal {
                    fragment: 1,
                    evidence_sequence: 13,
                },
                Event::StageBTerminal {
                    fragment: 2,
                    evidence_sequence: 13,
                },
                Event::StageBTerminal {
                    fragment: 3,
                    evidence_sequence: 13,
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
            ReplyPayload::VelocitySweepTerminal {
                oid: 3,
                terminal,
                reproduction: None,
            },
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
            ReplyPayload::VelocitySweepTerminal {
                oid: 3,
                terminal,
                reproduction: None,
            },
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
                stage_b_plan_digest: digest,
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
    fn combined_velocity_integral_plan_fixture()
    -> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
        use crate::commissioning::outer::OuterMotionBudget;
        use crate::commissioning::outer::velocity::combined_plan::select_proportional_response;
        use crate::commissioning::outer::velocity::integral_plan::plan_combined_velocity_integral_response;
        use crate::commissioning::outer::velocity::operating_region::{
            BoundRounding, CompactSignedRate,
        };
        use crate::commissioning::outer::velocity::sweep_consensus::DirectionalConsensusClass;
        use crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport;
        use crate::commissioning::outer::velocity::sweep_plan::{
            RequestedVelocitySource, VelocitySweepRequest, plan_combined_velocity_sweep,
        };
        use foci_tmc4671::pid::PidPGain;

        fn report(low_q: i64, high_q: i64) -> CompactDirectionalRungReport {
            CompactDirectionalRungReport {
                p: PidPGain::new(1_024).unwrap(),
                classification: DirectionalConsensusClass::EligibleConsensus,
                components: [[0; 2]; 2],
                pooled: None,
                settled_rate_bounds_q: [
                    CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
                    CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
                    CompactSignedRate::encode(low_q, BoundRounding::TowardNegativeInfinity),
                    CompactSignedRate::encode(high_q, BoundRounding::TowardPositiveInfinity),
                ],
                collected_mask: 0b1111,
                eligible_mask: 0b1111,
                included_mask: 0b1111,
                operable_count: 4,
                flags: 0,
            }
        }

        let request = VelocitySweepRequest {
            requested_velocity_mrev_s: 6_666,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 2_500,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 7_025,
            positive_position_headroom_mrev: 7_025,
            max_duration_ms: 3_000,
        };
        let stage_b = plan_combined_velocity_sweep(
            request,
            4_000,
            2_816,
            crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        )
        .unwrap();
        let selected = select_proportional_response(
            report(13_359_104, 13_492_224),
            report(-12_968_960, -12_921_856),
            264,
            4_000,
        )
        .unwrap()
        .unwrap();
        plan_combined_velocity_integral_response(
            selected,
            stage_b.execution_plan(stage_b.digest()),
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
    fn stage_b_terminal_fixture() -> VelocitySweepTerminal {
        use crate::commissioning::types::VelocitySweepOutcome;

        VelocitySweepTerminal {
            run_sequence: 17,
            evidence_sequence: 23,
            plan_digest: 0x1111_2222_3333_4444,
            outcome: VelocitySweepOutcome::Complete,
            cause: 0,
            recovery_unavailable: false,
            model_direction_mask: 3,
            selected_memberships: [0x0038_0000; 3],
            nominated_p: 1_448,
            selected_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
            selected_common_intervals: [[17_250_000, 17_750_000], [-18_750_000, -18_250_000]],
            p_start: 2,
            p_top: 32_767,
            rung_count: 23,
            coverage_mask: 3,
            directional_region_counts: [2; 2],
            directional_fragment_counts: [1; 2],
            expected_observations: 184,
            emitted_observations: 184,
            expected_rungs: 23,
            emitted_rungs: 23,
            digest: 0x5555_6666_7777_8888,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
        }
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
    fn combined_integral_plan_relabels_existing_interval_fragments() {
        let mut sink = RecordingSink::default();
        let plan = combined_velocity_integral_plan_fixture();

        emit_velocity_integral_plan(&mut sink, 3, 18, plan);

        assert_eq!(sink.integral_plan_joint_memberships, [0; 2]);
        assert_eq!(
            sink.integral_plan_intervals,
            plan.execution_plan().selected_response_intervals_q
        );
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn velocity_integral_reply_continuation_is_causal_and_plan_is_complete() {
        let mut sink = RecordingSink::default();
        let plan = velocity_integral_plan_fixture();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocityIntegralContinuation {
                oid: 3,
                run_sequence: 18,
                stage_b_terminal: stage_b_terminal_fixture(),
                stage_b_reproduction: None,
                stage_c_plan: plan,
            },
        );

        assert_eq!(
            &sink.events[..4],
            &[
                Event::StageBTerminal {
                    fragment: 0,
                    evidence_sequence: 23,
                },
                Event::StageBTerminal {
                    fragment: 1,
                    evidence_sequence: 23,
                },
                Event::StageBTerminal {
                    fragment: 2,
                    evidence_sequence: 23,
                },
                Event::StageBTerminal {
                    fragment: 3,
                    evidence_sequence: 23,
                },
            ]
        );
        assert_eq!(sink.integral_plan_joint_memberships, [0x0004_0000; 2]);
        assert_eq!(
            &sink.events[4..11],
            &[
                Event::IntegralPlan(0),
                Event::IntegralPlan(1),
                Event::IntegralPlan(4),
                Event::IntegralPlan(5),
                Event::IntegralPlan(2),
                Event::IntegralPlan(3),
                Event::IntegralPlanRecovery,
            ]
        );
        assert_eq!(
            sink.events.len(),
            11 + usize::from(plan.positive_rung_count)
        );
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

        let mut combined_terminal = terminal;
        combined_terminal.final_p = 1_448;
        combined_terminal.rest_boundary_rung_plus_one = 2;
        combined_terminal.rest_boundary_slot_plus_one = 0;
        let mut combined_sink = RecordingSink::default();
        emit_terminal_reply(
            &mut combined_sink,
            ReplyPayload::VelocityIntegralTerminal {
                oid: 3,
                terminal: combined_terminal,
                reproduction: None,
                authority: None,
            },
        );
        assert_eq!(combined_sink.integral_target, Some([1, 2]));
        assert_eq!(
            &combined_sink.events[combined_sink.events.len() - 3..],
            &[
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
            STAGE_B_BUILD_REVISION, StageBDirectionalValidity,
        };
        use crate::commissioning::types::{VelocityIntegralOutcome, VelocityIntegralTerminal};

        let mut plan = velocity_integral_plan_fixture();
        plan.authority.directional_memberships[1] = 0x0018_0000;
        plan.authority.directional_validity[1] = StageBDirectionalValidity::Fragment;
        plan.authority.directional_pooled_intervals[1] = [0, 0];
        let authority = StageCAuthoritySummary::from_plan(
            plan.authority.channel,
            STAGE_B_BUILD_REVISION,
            STAGE_C_EVIDENCE_SCHEMA_REVISION,
            plan.authority.stage_b_plan_digest,
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
