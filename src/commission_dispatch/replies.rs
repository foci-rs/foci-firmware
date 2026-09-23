//! Shared commissioning reply emission.
//!
//! Board crates keep transport-specific ankyra sender calls behind
//! [`CommissionReplySink`]. This module owns the semantic mapping from terminal
//! commissioning payloads to reply events so boards cannot drift by dropping
//! fields locally.

use crate::autotune::{CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE, CommissionPhase};
use crate::commission_dispatch::ReplyPayload;
use crate::commissioning::types::{
    AdcCalResult, CommissionResult, CurrentValidationAxisEvidence, INDUCTANCE_SOURCE_NONE,
    InnerResult, OuterResult, OuterSafetyFault,
};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics, SelftestStageValue};
use crate::tmc_diagnostics::{
    ClosedLoopActivationReport, CommissionTimingReport, CurrentLoopFiltersReport,
    CurrentLoopHoldReport, CurrentLoopRunReport, CurrentValidationAxisReport,
    InductanceEstimateReport, InductanceFrameReport, InductanceRunReport, ResistanceAxisReport,
    ResistanceRunReport,
};

/// TMC4671 external electrical angle for electrical axis 0 (PHASE0).
const RESISTANCE_AXIS0_PHI_E_EXT: u16 = 0;

/// TMC4671 external electrical angle for electrical axis 1 (PHASE90).
const RESISTANCE_AXIS1_PHI_E_EXT: u16 = 16_384;

/// Single source of truth for every `CommissionReplySink` reply that can be
/// generated mechanically: which trait method it becomes, its wire reply
/// type, and the fields forwarded into it. Consumed via the callback
/// pattern by `foci_firmware_macros::commission_reply_trait_methods!`
/// (inside this trait) and `foci_firmware_macros::commission_reply_forwarding_impls!`
/// (inside each board's `impl CommissionReplySink` block) so both surfaces
/// stay generated from exactly one list.
///
/// Entries use the crate-absolute path `foci_firmware::ankyra::replies::...`
/// for `reply`/`param`, not `crate::ankyra::replies::...`: this macro's
/// entries are expanded verbatim at each *invocation* site (once here, once
/// per board), and a board has no `crate::ankyra::replies` module of its
/// own. `foci_firmware::` resolves to the same crate everywhere, since
/// every invocation site (this trait's own crate and both boards) already
/// depends on `foci-firmware` -- including this crate's own trait
/// definition below, via the `extern crate self as foci_firmware;` alias
/// in `lib.rs` (a crate cannot otherwise refer to itself by its own crate
/// name).
#[macro_export]
macro_rules! __commission_reply_manifest {
    ($callback:path) => {
        $callback! {
            entry {
                name: foci_fixed_gain_amplitude_terminal,
                doc: "Emit one compact fixed-I fixed-gain amplitude terminal.",
                reply: foci_firmware::ankyra::replies::FociFixedGainAmplitudeTerminal<'_>,
                fields: { oid: u8, payload: &[u8] },
            }
            entry {
                name: foci_robustness_cycle_evidence,
                doc: "Emit one direction's per-cycle reversal-robustness evidence, ahead",
                doc: "of the compact terminal. No default: every board sink must decide",
                doc: "whether and how to forward this reply.",
                reply: foci_firmware::ankyra::replies::FociRobustnessCycleEvidence<'_>,
                fields: { oid: u8, payload: &[u8] },
                no_default,
            }
            entry {
                name: foci_commission_timing,
                doc: "Emit timing-quality evidence for one commissioning electrical method.",
                doc: "",
                doc: "The default keeps the shared API additive until each board adopts the",
                doc: "coordinated data dictionary in its board-integration task.",
                param: foci_firmware::tmc_diagnostics::CommissionTimingReport,
                reply: foci_firmware::ankyra::replies::FociCommissionTiming,
                fields: { oid: u8, method: u8, status: u8, requested_period_us: u16, valid_samples: u16, missed_samples: u16, max_consecutive_misses: u8, max_lateness_us: u32, max_interval_us: u32, max_poll_wall_us: u32, max_spi_wall_us: u32 },
            }
            entry {
                name: foci_commissioning_workflow_plan,
                doc: "Emit the command-level workflow envelope.",
                reply: foci_firmware::ankyra::replies::FociCommissioningWorkflowPlan,
                fields: { oid: u8, run_sequence: u32, shape: u8, digest_low: u32, digest_high: u32, nominal_workflow_ms: u32, maximum_workflow_ms: u32 },
            }
            entry {
                name: foci_velocity_integral_plan_core,
                doc: "Emit the velocity-integral plan identity.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralPlanCore,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, fragment: u8, plan_digest_low: u32, plan_digest_high: u32, proportional_digest_low: u32, proportional_digest_high: u32, build_features: u16, build_identity: u32, schema_revision: u16, channel: u8, fixed_p: u16 },
            }
            entry {
                name: foci_velocity_integral_plan_geometry,
                doc: "Emit the velocity-integral plan geometry.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralPlanGeometry,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, fragment: u8, planned_velocity_mrev_s: u32, target_velocity_rpm: i32, pwm_hz: u32, encoder_counts_per_rev: u32, i_start: u16, positive_rung_count: u8, family_size: u16, total_rung_count: u8, expected_observations: u16, hard_torque_limit: u16, usable_torque_limit: u16 },
            }
            entry {
                name: foci_velocity_integral_plan_authority,
                doc: "Emit one direction of velocity-integral authority.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralPlanAuthority,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, direction: u8, directional_membership: u32, joint_membership: u32, directional_validity: u8, reduced_margin: u8, pooled_low_q16: i32, pooled_high_q16: i32 },
            }
            entry {
                name: foci_velocity_integral_plan_timing,
                doc: "Emit velocity-integral schedule timing.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralPlanTiming,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, fragment: u8, moving_stroke_us: u32, zero_settle_us: u32, analysis_budget_us: u32, nominal_workflow_ms: u32, maximum_workflow_ms: u32 },
            }
            entry {
                name: foci_velocity_integral_plan_travel,
                doc: "Emit velocity-integral travel identity.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralPlanTravel,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, fragment: u8, max_stroke_travel_mrev: u32, settle_travel_reserve_mrev: u32, negative_position_headroom_mrev: u32, positive_position_headroom_mrev: u32 },
            }
            entry {
                name: foci_velocity_integral_plan_recovery,
                doc: "Emit velocity-integral recovery geometry and reserved schedule.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralPlanRecovery,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, origin_band_counts: u32, nominal_slot_us: u32, maximum_slot_us: u32, slot_count: u8, flags: u8 },
            }
            entry {
                name: foci_velocity_integral_plan_rung,
                doc: "Emit one exact positive-I plan rung.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralPlanRung,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, rung_index: u8, i_raw: u16, tau_us: u32 },
            }
            entry {
                name: foci_velocity_integral_terminal,
                doc: "Emit the single compact velocity-integral terminal reply.",
                reply: foci_firmware::ankyra::replies::FociVelocityIntegralTerminal<'_>,
                fields: { oid: u8, payload: &[u8] },
            }
            entry {
                name: foci_fixed_gain_amplitude_plan,
                doc: "Emit one immutable fixed-I fixed-gain amplitude plan.",
                reply: foci_firmware::ankyra::replies::FociFixedGainAmplitudePlan<'_>,
                fields: { oid: u8, fragment: u8, payload: &[u8] },
            }
            entry {
                name: foci_robustness_reversal_terminal,
                doc: "Emit one compact reversal-standstill robustness-gate terminal.",
                reply: foci_firmware::ankyra::replies::FociRobustnessReversalTerminal<'_>,
                fields: { oid: u8, payload: &[u8] },
            }
            entry {
                name: foci_breakaway_probe_plan,
                doc: "Emit the immutable breakaway-probe plan.",
                reply: foci_firmware::ankyra::replies::FociBreakawayProbePlan,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, max_observations: u8, search_count: u8, p_start_raw: u16, p_top_raw: u16, motion_threshold_counts: u32, max_capture_interval_us: u32 },
            }
            entry {
                name: foci_breakaway_probe_result,
                doc: "Emit the resolved repeatable-motion rung.",
                reply: foci_firmware::ankyra::replies::FociBreakawayProbeResult,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, rung_index: u8, breakaway_p_raw: u16, motion_threshold_counts: u32, observation_count: u8 },
            }
            entry {
                name: foci_breakaway_probe_terminal,
                doc: "Emit a probe-campaign terminal that did not resolve a breakaway.",
                reply: foci_firmware::ankyra::replies::FociBreakawayProbeTerminal,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, terminal_cause: u8, has_safety_fault: u8 },
            }
            entry {
                name: foci_breakaway_discovery_plan_identity,
                doc: "Emit the additive discovery plan's identity fragment.",
                reply: foci_firmware::ankyra::replies::FociBreakawayDiscoveryPlanIdentity,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, prior_plan_digest_low: u32, prior_plan_digest_high: u32, family_size: u16 },
            }
            entry {
                name: foci_breakaway_discovery_plan_geometry,
                doc: "Emit the additive discovery plan's geometry fragment.",
                reply: foci_firmware::ankyra::replies::FociBreakawayDiscoveryPlanGeometry,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, schema_revision: u16, rung_count: u8, observations_per_direction: u16, floor_p_raw: u16, floor_origin: u8, breakaway_p_raw: u16, ceiling_p_raw: u16, first_additive_step_raw: u32, maximum_workflow_ms: u32 },
            }
            entry {
                name: foci_breakaway_ceiling_source,
                doc: "Emit which `p_top` candidate bound the additive-discovery authority",
                doc: "ceiling.",
                reply: foci_firmware::ankyra::replies::FociBreakawayCeilingSource,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, binding_source: u8 },
            }
            entry {
                name: foci_breakaway_rung_zero_diagnostic,
                doc: "Emit whether the additive ladder's rung zero itself showed motion.",
                reply: foci_firmware::ankyra::replies::FociBreakawayRungZeroDiagnostic,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, forward_moved: u8, reverse_moved: u8 },
            }
            entry {
                name: foci_breakaway_discovery_terminal,
                doc: "Emit a discovery-campaign terminal.",
                reply: foci_firmware::ankyra::replies::FociBreakawayDiscoveryTerminal,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, prior_plan_digest_low: u32, prior_plan_digest_high: u32, family_size: u16, terminal_cause: u8, collected_count: u8, has_safety_fault: u8, stop_rung_index: u8 },
            }
            entry {
                name: foci_breakaway_confirmation_plan,
                doc: "Emit the immutable held-out confirmation plan.",
                reply: foci_firmware::ankyra::replies::FociBreakawayConfirmationPlan,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, prior_plan_digest_low: u32, prior_plan_digest_high: u32, family_size: u16, observations_per_direction: u16, nominated_p_raw: u16, band_lower_percent: u8, band_upper_percent: u8, capture_profile: u8, acceptance_rule: u8, nominated_margin_percent_milli: i32 },
            }
            entry {
                name: foci_breakaway_confirmation_terminal_identity,
                doc: "Emit the confirmation terminal's identity fragment.",
                reply: foci_firmware::ankyra::replies::FociBreakawayConfirmationTerminalIdentity,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, plan_digest_low: u32, plan_digest_high: u32, prior_plan_digest_low: u32, prior_plan_digest_high: u32, family_size: u16, terminal_cause: u8 },
            }
            entry {
                name: foci_breakaway_confirmation_terminal_masks,
                doc: "Emit the confirmation terminal's mask/acceptance fragment.",
                reply: foci_firmware::ankyra::replies::FociBreakawayConfirmationTerminalMasks,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, forward_collected_mask: u8, forward_eligible_mask: u8, forward_included_mask: u8, reverse_collected_mask: u8, reverse_eligible_mask: u8, reverse_included_mask: u8, accepted: u8, confirmed_p_raw: u16, max_relative_se_permille: u16, required_relative_se_permille: u16, has_safety_fault: u8 },
            }
            entry {
                name: foci_breakaway_campaign_terminal,
                doc: "Emit the canonical closure record for one complete breakaway campaign.",
                reply: foci_firmware::ankyra::replies::FociBreakawayCampaignTerminal,
                fields: { oid: u8, run_sequence: u32, evidence_sequence: u16, phase: u8, terminal_cause: u8, accepted: u8, integral_plan_digest_low: u32, integral_plan_digest_high: u32, error_code: u8 },
            }
        }
    };
}

/// Board-local sink for commissioning-family Klipper replies.
///
/// Implementations call board-generated ankyra reply macros. Methods return
/// `()` to preserve the existing fire-and-forget reply behavior.
pub trait CommissionReplySink {
    /// Emit one retained commissioning diagnostic detail event.
    fn foci_commission_detail(&mut self, oid: u8, event: CommissionDiagnostic);

    /// Emit a commissioning phase reply.
    fn foci_commission_phase(&mut self, oid: u8, phase: u8, status: u8);

    /// Emit the commissioning result.
    fn foci_commission_result(&mut self, oid: u8, status: u8, result: &CommissionResult);

    crate::__commission_reply_manifest!(foci_firmware_macros::commission_reply_trait_methods);

    /// Emit the resistance-identification per-run evidence captured during
    /// commissioning. `FociCommissionResult` is size-capped, so this
    /// reuses the `foci_resistance_run` reply (Task 6) to carry the evidence
    /// that lives on `InnerResult` to the host.
    fn foci_resistance_run(&mut self, report: ResistanceRunReport);

    /// Emit one resistance-identification per-axis fit result captured
    /// during commissioning. Reuses the `foci_resistance_axis` reply
    /// (Task 6) for the same reason as [`Self::foci_resistance_run`].
    fn foci_resistance_axis(&mut self, report: ResistanceAxisReport);

    /// Emit run-level production inductance evidence captured during
    /// commissioning.
    fn foci_inductance_run(&mut self, report: InductanceRunReport);

    /// Emit rotating-frame production inductance current evidence captured
    /// during commissioning.
    fn foci_inductance_frame(&mut self, report: InductanceFrameReport);

    /// Emit accepted production inductance estimate evidence captured during
    /// commissioning.
    fn foci_inductance_estimate(&mut self, report: InductanceEstimateReport);

    /// Emit sustained-hold current-loop evidence captured during
    /// commissioning before the run-level current-loop summary.
    fn foci_current_loop_hold(&mut self, report: CurrentLoopHoldReport);

    /// Emit fast closed-loop entry stability evidence captured before the
    /// sustained-hold sampler.
    fn foci_closed_loop_activation(&mut self, report: ClosedLoopActivationReport);

    /// Emit applied filter cutoffs for the commissioned closed-loop frame.
    fn foci_current_loop_filters(&mut self, report: CurrentLoopFiltersReport);

    /// Emit run-level current-loop evidence captured during commissioning.
    /// `FociCommissionResult` is size-capped, so this
    /// supplemental reply carries source, tier, split, validation coverage,
    /// and failure-reason evidence separately.
    fn foci_current_loop_run(&mut self, report: CurrentLoopRunReport);

    /// Emit one per-axis, per-delay current validation evidence sample
    /// captured during commissioning.
    fn foci_current_validation_axis(&mut self, report: CurrentValidationAxisReport);

    /// Emit the installed-tuning result.
    fn foci_tune_result(&mut self, oid: u8, status: u8, result: &OuterResult);

    /// Emit installed-tuning safety-envelope evidence before the terminal tune result.
    fn foci_outer_safety_fault(&mut self, oid: u8, fault: OuterSafetyFault);

    /// Emit the ADC calibration result.
    fn foci_calibrate_result(&mut self, oid: u8, status: u8, result: &AdcCalResult);

    /// Emit one self-test stage result.
    fn foci_selftest_result(&mut self, oid: u8, stage: u8, value: SelftestStageValue);

    /// Emit the terminal self-test completion reply.
    fn foci_selftest_done(&mut self, oid: u8, status: u8);
}

/// Emit the command-level workflow envelope that precedes any velocity-workflow motion.
pub fn emit_commissioning_workflow_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    plan: crate::commissioning::outer::velocity::workflow_plan::CommissioningWorkflowPlan,
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
pub fn emit_breakaway_discovery_plan<S: CommissionReplySink>(
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
pub fn emit_breakaway_discovery_terminal<S: CommissionReplySink>(
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
            stop_rung_index: report.stop_rung_index,
        },
    );
}

/// Emit the immutable held-out confirmation plan before the first
/// confirmation stroke.
pub fn emit_breakaway_confirmation_plan<S: CommissionReplySink>(
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
        nominated_p_raw: report.nominated_p_raw,
        band_lower_percent: report.band_lower_percent,
        band_upper_percent: report.band_upper_percent,
        capture_profile: report.capture_profile,
        acceptance_rule: report.acceptance_rule,
        nominated_margin_percent_milli: report.nominated_margin_percent_milli,
    });
}

/// Emit a confirmation-block terminal, split into its identity and mask fragments.
pub fn emit_breakaway_confirmation_terminal<S: CommissionReplySink>(
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
        integral_plan_digest_low: report.integral_plan_digest as u32,
        integral_plan_digest_high: (report.integral_plan_digest >> 32) as u32,
        error_code: report.error_code,
    });
}

/// Fan one campaign evidence event out to the Task-7 `emit_breakaway_*`
/// bridges, looping over any batch evidence a terminal variant carries (every
/// in-band rung's margin, every resolved ladder rung, every fresh held-out
/// stroke) the same way [`emit_fixed_gain_amplitude_plan`] already loops one
/// Rust value out to several wire fragments.
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
            emit_breakaway_discovery_plan(sink, oid, planned);
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
            emit_breakaway_discovery_terminal(sink, oid, terminal);
        }
        BreakawayCampaignEvent::ConfirmationPlanned(report) => {
            emit_breakaway_confirmation_plan(sink, oid, report);
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
            emit_breakaway_confirmation_terminal(sink, oid, terminal);
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

/// Emit the single compact amplitude-plan reply disclosed before motion.
pub fn emit_fixed_gain_amplitude_plan<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    run_sequence: u32,
    plan: crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::FixedGainAmplitudePlan,
) {
    use crate::commissioning::outer::velocity::fixed_gain_amplitude_reporting::{
        FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES, encode_fixed_gain_amplitude_plan_reply,
    };
    let payload = encode_fixed_gain_amplitude_plan_reply(run_sequence, plan);
    for (fragment, chunk) in payload
        .chunks(FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES)
        .enumerate()
    {
        sink.foci_fixed_gain_amplitude_plan(crate::ankyra::replies::FociFixedGainAmplitudePlan {
            oid,
            fragment: fragment as u8,
            payload: chunk,
        });
    }
}

/// Emit the single compact amplitude terminal.
pub fn emit_fixed_gain_amplitude_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    terminal: crate::commissioning::types::FixedGainAmplitudeTerminal,
) {
    let payload = crate::commissioning::outer::velocity::fixed_gain_amplitude_reporting::
        encode_fixed_gain_amplitude_terminal_reply(terminal);
    sink.foci_fixed_gain_amplitude_terminal(
        crate::ankyra::replies::FociFixedGainAmplitudeTerminal {
            oid,
            payload: &payload,
        },
    );
}

/// Emit the single compact reversal-standstill robustness-gate terminal.
pub fn emit_robustness_reversal_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    terminal: crate::commissioning::types::RobustnessReversalTerminal,
) {
    let payload = crate::commissioning::outer::velocity::robustness_reporting::
        encode_robustness_terminal_reply(&terminal);
    sink.foci_robustness_reversal_terminal(
        crate::ankyra::replies::FociRobustnessReversalTerminal {
            oid,
            payload: &payload,
        },
    );
}

/// Emit both directions' per-cycle robustness evidence, ahead of the
/// compact terminal -- mirrors the tune-result path's
/// outer-safety-fault-before-terminal-result ordering.
pub fn emit_robustness_cycle_evidence<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    evidence: &crate::commissioning::outer::velocity::robustness_evidence::RobustnessEvidence,
) {
    for (direction, cycles) in evidence.cycles.iter().enumerate() {
        let payload = crate::commissioning::outer::velocity::robustness_reporting::
            encode_robustness_cycle_evidence(direction as u8, evidence, cycles);
        sink.foci_robustness_cycle_evidence(crate::ankyra::replies::FociRobustnessCycleEvidence {
            oid,
            payload: &payload,
        });
    }
}

/// Emit the complete exact velocity-integral plan as bounded causal fragments.
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
        proportional_digest_low: plan.authority.acquisition.get() as u32,
        proportional_digest_high: (plan.authority.acquisition.get() >> 32) as u32,
        build_features: plan.build_features,
        build_identity: crate::commissioning::outer::velocity::reproduction::BUILD_IDENTITY,
        schema_revision: plan.schema_revision,
        channel: plan.authority.channel,
        fixed_p: plan.fixed_p,
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

/// Emit one velocity-integral observation or rung group.
pub fn emit_velocity_integral_event<S: CommissionReplySink>(
    _sink: &mut S,
    _oid: u8,
    event: crate::commissioning::types::VelocityIntegralEvent,
) {
    match event {
        crate::commissioning::types::VelocityIntegralEvent::Planned(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::Observation(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::Rung(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::Recovery(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::RecoveryRest(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::RestSelection(_) => {}
        crate::commissioning::types::VelocityIntegralEvent::Current(_) => {}
    }
}

/// Emit the single compact velocity-integral terminal reply.
///
/// Carries only the fields the live host reads for status text and
/// orchestration: outcome, cause, rest-rejection, eligibility/bookend
/// masks, and the reproduced/divergent mask summary. The detailed
/// observation/rung/curve/recovery evidence rides the canonical trace
/// only, decoded independently by `foci-trace` (see
/// `encode_velocity_integral_terminal` for that record).
pub fn emit_velocity_integral_terminal<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    terminal: crate::commissioning::types::VelocityIntegralTerminal,
    reproduction: Option<crate::commission_dispatch::integral::IntegralReproductionReport>,
    _authority: Option<
        crate::commissioning::outer::velocity::integral_reproduction::IntegralAuthoritySummary,
    >,
) {
    use crate::commissioning::outer::velocity::integral_reporting::{
        VelocityIntegralTerminalReproductionSummary, encode_velocity_integral_terminal_reply,
    };

    let reproduction_summary =
        reproduction.map(|reproduction| VelocityIntegralTerminalReproductionSummary {
            reproduced_masks: reproduction.delta.reproduced_masks,
            divergent_masks: reproduction.delta.divergent_masks,
        });
    let payload = encode_velocity_integral_terminal_reply(&terminal, reproduction_summary);
    if let Some(fault) = terminal.safety_fault {
        sink.foci_outer_safety_fault(oid, fault);
    }
    sink.foci_velocity_integral_terminal(crate::ankyra::replies::FociVelocityIntegralTerminal {
        oid,
        payload: &payload,
    });
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
            robustness_evidence,
        } => {
            if let Some(fault) = safety_fault {
                sink.foci_outer_safety_fault(oid, fault);
            }
            if let Some(evidence) = robustness_evidence {
                emit_robustness_cycle_evidence(sink, oid, &evidence);
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
        ReplyPayload::CommissioningWorkflowPlan { oid, plan } => {
            emit_commissioning_workflow_plan(sink, oid, plan);
        }
        ReplyPayload::VelocityIntegralPlan {
            oid,
            run_sequence,
            plan,
        } => emit_velocity_integral_plan(sink, oid, run_sequence, plan),
        ReplyPayload::VelocityIntegralEvent { oid, event } => {
            emit_velocity_integral_event(sink, oid, event);
        }
        ReplyPayload::VelocityIntegralTerminal {
            oid,
            terminal,
            reproduction,
            authority,
        } => emit_velocity_integral_terminal(sink, oid, terminal, reproduction, authority),
        ReplyPayload::VelocityIntegralRefusal { oid, terminal } => {
            emit_velocity_integral_terminal(sink, oid, terminal, None, None);
        }
        ReplyPayload::FixedGainAmplitudePlan {
            oid,
            run_sequence,
            plan,
        } => emit_fixed_gain_amplitude_plan(sink, oid, run_sequence, plan),
        ReplyPayload::FixedGainAmplitudeTerminal { oid, terminal } => {
            emit_fixed_gain_amplitude_terminal(sink, oid, terminal);
        }
        ReplyPayload::RobustnessReversalTerminal { oid, terminal } => {
            if let Some(fault) = terminal.safety_fault {
                sink.foci_outer_safety_fault(oid, fault);
            }
            emit_robustness_cycle_evidence(sink, oid, &terminal.evidence);
            emit_robustness_reversal_terminal(sink, oid, terminal);
        }
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
pub fn emit_phase_reply<S: CommissionReplySink>(sink: &mut S, oid: u8, phase_id: CommissionPhase) {
    sink.foci_commission_phase(oid, phase_id.to_u8(), 0);
}

fn emit_commission_diagnostics<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    diagnostics: CommissionDiagnostics,
) {
    for priority in [true, false] {
        for event in diagnostics.iter() {
            let is_electrical_timing = event.phase
                == CommissionPhase::ElectricalIdentification.to_u8()
                && matches!(event.code, 34..=41);
            if is_electrical_timing == priority {
                sink.foci_commission_detail(oid, event);
            }
        }
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
    sink.foci_closed_loop_activation(commission_closed_loop_activation_report(oid, inner));
    sink.foci_current_loop_filters(commission_current_loop_filters_report(oid, result));
    sink.foci_current_loop_run(commission_current_loop_run_report(oid, status, inner));
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

fn commission_closed_loop_activation_report(
    oid: u8,
    inner: &InnerResult,
) -> ClosedLoopActivationReport {
    ClosedLoopActivationReport {
        oid,
        entry_status: inner.closed_loop_activation_status,
        position_1: inner.closed_loop_activation_position_1,
        position_2: inner.closed_loop_activation_position_2,
        drift_count: inner.closed_loop_activation_drift_count,
        threshold_count: inner.closed_loop_activation_threshold_count,
        runaway: inner.closed_loop_activation_runaway,
    }
}

#[cfg(test)]
mod tests {
    use std::vec;
    use std::vec::Vec;

    use super::*;
    use crate::ankyra::velocity_sweep_encoding::{
        encode_rate_values, upper_bound_i64, upper_bound_u128,
    };
    use crate::autotune::{
        AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE,
        CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED,
        CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, CURRENT_GAINS_TIER_NONE, CURRENT_LOOP_FAILURE_NONE,
        CURRENT_LOOP_FAILURE_SATURATION, CURRENT_VALIDATION_AXIS_FLUX,
        CURRENT_VALIDATION_AXIS_TORQUE, CommissionError, CommissionPhase, CommissionStatus,
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
        RobustnessReversalTerminal {
            oid: u8,
            payload: Vec<u8>,
        },
        RobustnessCycleEvidence {
            oid: u8,
            payload: Vec<u8>,
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
        CurrentLoopHold {
            oid: u8,
            hold_status: u8,
            sample_count: u8,
            torque_rms_count: u16,
            flux_rms_count: u16,
        },
        ClosedLoopActivation {
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
        },
        OuterSafetyFault {
            oid: u8,
            reason: u8,
            delta_counts: i32,
            velocity_counts_per_ms: i32,
            velocity_cap_counts_per_ms: i32,
        },
        WorkflowPlan,
        IntegralPlan(u8),
        IntegralPlanRecovery,
        IntegralTerminal(usize),
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

        fn foci_robustness_reversal_terminal(
            &mut self,
            report: crate::ankyra::replies::FociRobustnessReversalTerminal<'_>,
        ) {
            self.events.push(Event::RobustnessReversalTerminal {
                oid: report.oid,
                payload: report.payload.to_vec(),
            });
        }

        fn foci_robustness_cycle_evidence(
            &mut self,
            report: crate::ankyra::replies::FociRobustnessCycleEvidence<'_>,
        ) {
            self.events.push(Event::RobustnessCycleEvidence {
                oid: report.oid,
                payload: report.payload.to_vec(),
            });
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

        fn foci_closed_loop_activation(
            &mut self,
            report: crate::tmc_diagnostics::ClosedLoopActivationReport,
        ) {
            self.events.push(Event::ClosedLoopActivation {
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

        fn foci_tune_result(&mut self, oid: u8, status: u8, result: &OuterResult) {
            self.events.push(Event::Tune {
                oid,
                status,
                velocity_p: result.velocity_p,
                position_p: result.position_p,
                warning_code: result.warning_code,
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

        fn foci_velocity_integral_terminal(
            &mut self,
            report: crate::ankyra::replies::FociVelocityIntegralTerminal<'_>,
        ) {
            self.events
                .push(Event::IntegralTerminal(report.payload.len()));
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
                theta_source: 0,
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
                closed_loop_activation_status:
                    crate::autotune::CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN,
                closed_loop_activation_position_1: 0,
                closed_loop_activation_position_2: 0,
                closed_loop_activation_drift_count: 0,
                closed_loop_activation_threshold_count: 0,
                closed_loop_activation_runaway: 0,
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
            warning_code: 52,
            provenance: crate::commissioning::types::TuneProvenance::default(),
            position_tune: crate::commissioning::types::PositionTuneProvenance::default(),
            position_tune_outcome_code: 0,
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
            Event::ClosedLoopActivation {
                oid,
                entry_status: crate::autotune::CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN,
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
        let electrical_id = CommissionPhase::ElectricalIdentification.to_u8();
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
                    phase: CommissionPhase::EncoderCheck.to_u8(),
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
                    phase: CommissionPhase::EncoderCheck.to_u8(),
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
                robustness_evidence: None,
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
                robustness_evidence: None,
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
                },
            ]
        );
    }

    #[test]
    fn combined_velocity_flow_reuses_the_existing_reply_inventory() {
        let source = include_str!("replies.rs");
        for method in [
            "fn foci_commissioning_workflow_plan(",
            "fn foci_velocity_sweep_plan_core(",
            "fn foci_velocity_integral_plan_core(",
            "fn foci_velocity_integral_terminal(",
        ] {
            assert!(
                source.contains(method),
                "missing existing reply method {method}"
            );
        }
        assert!(!source.contains(concat!("fn foci_", "combined_")));
        assert!(!source.contains(concat!("fn foci_velocity_", "combined_")));
    }

    #[test]
    fn velocity_integral_reply_emits_exactly_one_compact_terminal() {
        use crate::commissioning::outer::velocity::integral_evidence::{
            ExactRateInterval, IntegralRunEvidence,
        };
        use crate::commissioning::outer::velocity::integral_reporting::VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES;
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
            cause: crate::commissioning::types::IntegralOutcomeCause::Engine(7),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
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
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
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

        assert_eq!(
            sink.events,
            std::vec![Event::IntegralTerminal(
                VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES
            )]
        );
    }

    #[test]
    fn a_refused_integral_admission_emits_exactly_one_compact_terminal() {
        use crate::commission_dispatch::integral::missing_authority_integral_terminal;
        use crate::commissioning::outer::velocity::integral_reporting::VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES;

        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::VelocityIntegralRefusal {
                oid: 3,
                terminal: missing_authority_integral_terminal(21),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::IntegralTerminal(
                VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES
            )]
        );
    }

    #[test]
    fn phase_helper_emits_zero_status_phase_reply() {
        let mut sink = RecordingSink::default();
        emit_phase_reply(&mut sink, 12, CommissionPhase::ClosedLoopActivation);

        assert_eq!(
            sink.events,
            std::vec![Event::Phase {
                oid: 12,
                phase: 17,
                status: 0,
            }]
        );
    }

    #[test]
    fn emit_terminal_reply_routes_the_robustness_terminal_payload() {
        use crate::commissioning::outer::velocity::robustness_evidence::{
            RobustnessDirectionSummary, RobustnessEvidence,
        };
        use crate::commissioning::outer::velocity::robustness_reporting::{
            CycleWire, ROBUSTNESS_SCHEMA_REVISION, ROBUSTNESS_TERMINAL_REPLY_BYTES,
            encode_robustness_cycle_evidence, encode_robustness_terminal_reply,
        };
        use crate::commissioning::types::{RobustnessOutcome, RobustnessReversalTerminal};

        let direction = RobustnessDirectionSummary {
            median_reconvergence_ms: 4,
            max_reconvergence_ms: 4,
            median_forward_settle_ms: 3,
            overshoot_peak_counts: 0,
            valid_cycles: 1,
            trip_count: 0,
            retry_count: 1,
            inconclusive: false,
        };
        // Real per-cycle wire triples now ride the terminal's evidence and are
        // emitted verbatim (no single-cycle bridge).
        let example_cycles: [Option<CycleWire>; 5] = [
            Some(CycleWire {
                reconvergence_ms: 42,
                overshoot_counts: 7,
                forward_settle_ms: 12,
            }),
            Some(CycleWire {
                reconvergence_ms: 44,
                overshoot_counts: 0,
                forward_settle_ms: 13,
            }),
            None,
            None,
            None,
        ];
        let terminal = RobustnessReversalTerminal {
            schema_revision: ROBUSTNESS_SCHEMA_REVISION,
            run_sequence: 0x0000_abcd,
            outcome: RobustnessOutcome::Complete,
            cause: 0,
            selected_p: 800,
            selected_i: 400,
            target_velocity_rpm: 120,
            plant_rate_q: 60_000,
            iae_max_q: 120_000_000_000,
            evidence: RobustnessEvidence {
                directions: [direction, direction],
                residual_medians_q: [17, 17],
                iae_medians_qs: [0, 0],
                dac_rms_medians_q: [0, 0],
                cycles: [example_cycles, example_cycles],
                tail_counts: [1, 0],
                displaced_tails: [example_cycles[0], None],
            },
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
        };

        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::RobustnessReversalTerminal { oid: 3, terminal },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::RobustnessCycleEvidence {
                    oid: 3,
                    payload: encode_robustness_cycle_evidence(
                        0,
                        &terminal.evidence,
                        &terminal.evidence.cycles[0]
                    )
                    .to_vec(),
                },
                Event::RobustnessCycleEvidence {
                    oid: 3,
                    payload: encode_robustness_cycle_evidence(
                        1,
                        &terminal.evidence,
                        &terminal.evidence.cycles[1]
                    )
                    .to_vec(),
                },
                Event::RobustnessReversalTerminal {
                    oid: 3,
                    payload: encode_robustness_terminal_reply(&terminal).to_vec(),
                },
            ]
        );
        assert_eq!(
            encode_robustness_terminal_reply(&terminal).len(),
            ROBUSTNESS_TERMINAL_REPLY_BYTES
        );
    }

    #[test]
    fn emit_terminal_reply_emits_cycle_evidence_before_a_deployed_tune_result() {
        use crate::commission_dispatch::robustness::{
            RobustnessPreflightRejection, failed_robustness_terminal,
        };
        use crate::commissioning::outer::velocity::robustness_reporting::encode_robustness_cycle_evidence;

        let mut evidence =
            failed_robustness_terminal(1, RobustnessPreflightRejection::NoAcceptedCandidate)
                .evidence;
        evidence.iae_medians_qs = [9, -9];
        evidence.dac_rms_medians_q = [21, -21];
        let result = outer_result();

        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::TuneResult {
                oid: 8,
                status: 9,
                result: outer_result(),
                safety_fault: None,
                robustness_evidence: Some(evidence),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::RobustnessCycleEvidence {
                    oid: 8,
                    payload: encode_robustness_cycle_evidence(0, &evidence, &evidence.cycles[0])
                        .to_vec(),
                },
                Event::RobustnessCycleEvidence {
                    oid: 8,
                    payload: encode_robustness_cycle_evidence(1, &evidence, &evidence.cycles[1])
                        .to_vec(),
                },
                Event::Tune {
                    oid: 8,
                    status: 9,
                    velocity_p: result.velocity_p,
                    position_p: result.position_p,
                    warning_code: result.warning_code,
                },
            ]
        );
    }

    #[test]
    fn emit_terminal_reply_emits_outer_safety_fault_before_robustness_terminal() {
        use crate::commissioning::outer::velocity::robustness_evidence::{
            RobustnessDirectionSummary, RobustnessEvidence,
        };
        use crate::commissioning::outer::velocity::robustness_reporting::{
            ROBUSTNESS_SCHEMA_REVISION, encode_robustness_cycle_evidence,
            encode_robustness_terminal_reply,
        };
        use crate::commissioning::types::{
            OuterSafetyFault, RobustnessOutcome, RobustnessReversalTerminal,
        };

        let direction = RobustnessDirectionSummary {
            median_reconvergence_ms: 0,
            max_reconvergence_ms: 0,
            median_forward_settle_ms: 0,
            overshoot_peak_counts: 0,
            valid_cycles: 1,
            trip_count: 0,
            retry_count: 0,
            inconclusive: true,
        };
        let terminal = RobustnessReversalTerminal {
            schema_revision: ROBUSTNESS_SCHEMA_REVISION,
            run_sequence: 0x0000_abcd,
            outcome: RobustnessOutcome::Failed,
            cause: crate::commission_dispatch::robustness::ROBUSTNESS_CAUSE_SAFETY_FAULT,
            selected_p: 350,
            selected_i: 1_280,
            target_velocity_rpm: 322,
            plant_rate_q: 0,
            iae_max_q: 0,
            evidence: RobustnessEvidence {
                directions: [direction, direction],
                residual_medians_q: [0, 0],
                iae_medians_qs: [0, 0],
                dac_rms_medians_q: [0, 0],
                cycles: [[None; crate::commissioning::outer::velocity::robustness_cycles::ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
                tail_counts: [0, 0],
                displaced_tails: [None, None],
            },
            run_started_us: 1_000,
            run_completed_us: 2_000,
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
        };

        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::RobustnessReversalTerminal { oid: 3, terminal },
        );

        // A safety fault preempts the run before any cycle is measured, so the
        // cycle-evidence replies carry all-unfilled slots.
        assert_eq!(
            sink.events,
            std::vec![
                Event::OuterSafetyFault {
                    oid: 3,
                    reason: crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY,
                    delta_counts: -125,
                    velocity_counts_per_ms: -31,
                    velocity_cap_counts_per_ms: 24,
                },
                Event::RobustnessCycleEvidence {
                    oid: 3,
                    payload: encode_robustness_cycle_evidence(
                        0,
                        &terminal.evidence,
                        &terminal.evidence.cycles[0]
                    )
                    .to_vec(),
                },
                Event::RobustnessCycleEvidence {
                    oid: 3,
                    payload: encode_robustness_cycle_evidence(
                        1,
                        &terminal.evidence,
                        &terminal.evidence.cycles[1]
                    )
                    .to_vec(),
                },
                Event::RobustnessReversalTerminal {
                    oid: 3,
                    payload: encode_robustness_terminal_reply(&terminal).to_vec(),
                },
            ]
        );
    }
}
