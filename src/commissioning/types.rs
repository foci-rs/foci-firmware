//! Public data types for commissioning.

use crate::autotune::{CommissionError, CommissionPhase, CommissionStatus};

/// Compatibility re-export for existing `commissioning::types` backend users.
pub use crate::commissioning::backend::CommissioningBackend;

/// One current-axis validation evidence item for a single sample delay.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentValidationAxisEvidence {
    /// `CURRENT_AXIS_VALIDATION_*` status code for this delay.
    pub status: u8,
    pub attempt: u8,
    /// Signed current target used for each pulse.
    pub target: i16,
    pub sample_delay_ms: u16,
    /// Positive-pulse response normalized to target, in permille.
    pub positive_response_permille: u16,
    /// Negative-pulse response normalized to target, in permille.
    pub negative_response_permille: u16,
    pub cross_axis_permille: u16,
    pub cross_axis_peak_permille: u16,
    pub voltage_output_permille: u16,
    pub encoder_delta_counts: u16,
    /// Signed encoder movement observed during the positive pulse.
    pub positive_encoder_delta_counts: i32,
    /// Signed encoder movement observed during the negative pulse.
    pub negative_encoder_delta_counts: i32,
    /// OR of TMC4671 `STATUS_FLAGS` captured for the positive/negative pair.
    pub status_flags_or: u32,
}

impl CurrentValidationAxisEvidence {
    pub const EMPTY: Self = Self {
        status: crate::autotune::CURRENT_AXIS_VALIDATION_ZERO_TARGET,
        attempt: 0,
        target: 0,
        sample_delay_ms: 0,
        positive_response_permille: 0,
        negative_response_permille: 0,
        cross_axis_permille: 0,
        cross_axis_peak_permille: 0,
        voltage_output_permille: 0,
        encoder_delta_counts: 0,
        positive_encoder_delta_counts: 0,
        negative_encoder_delta_counts: 0,
        status_flags_or: 0,
    };
}

/// One raw read contributing to the settled-flux current-validation gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentValidationSettledSampleEvidence {
    /// Stable `CURRENT_AXIS_*` axis identifier.
    pub axis: u8,
    pub sample_index: u8,
    pub direction: u8,
    pub raw_index: u8,
    pub attempt: u8,
    /// Signed current target used for this pulse.
    pub target: i16,
    pub sample_delay_ms: u16,
    /// Same-axis current count from `PID_TORQUE_FLUX_ACTUAL`.
    pub same_axis_count: i16,
    /// Cross-axis current count from `PID_TORQUE_FLUX_ACTUAL`.
    pub cross_axis_count: i16,
    /// Cross-axis response normalized to target, in permille.
    pub cross_axis_permille: u16,
    /// Voltage output normalized to the configured limit, in permille.
    pub voltage_output_permille: u16,
    /// Signed encoder movement observed at this raw read.
    pub encoder_delta_counts: i32,
    /// TMC4671 `STATUS_FLAGS` captured at this raw read.
    pub status_flags: u32,
}

impl CurrentValidationSettledSampleEvidence {
    pub const EMPTY: Self = Self {
        axis: 0,
        sample_index: 0,
        direction: 0,
        raw_index: 0,
        attempt: 0,
        target: 0,
        sample_delay_ms: 0,
        same_axis_count: 0,
        cross_axis_count: 0,
        cross_axis_permille: 0,
        voltage_output_permille: 0,
        encoder_delta_counts: 0,
        status_flags: 0,
    };
}

/// No accepted production inductance source.
pub const INDUCTANCE_SOURCE_NONE: u8 = 0;
/// Accepted production inductance source: rotating-field estimator.
pub const INDUCTANCE_SOURCE_ROTATING_FIELD: u8 = 1;

/// High RMS saliency/ripple evidence; warning only.
pub const INDUCTANCE_WARN_HIGH_RMS: u16 = 1 << 0;
/// Half-window drift exceeded the warning threshold but not the hard failure line.
pub const INDUCTANCE_WARN_DRIFT: u16 = 1 << 1;
/// Magnitude branch disagreed with quadrature but stayed inside tolerance.
pub const INDUCTANCE_WARN_MAG_CROSSCHECK: u16 = 1 << 2;
/// TMC4671 status flags were observed during the AC window; warning only.
pub const INDUCTANCE_WARN_STATUS_FLAGS: u16 = 1 << 3;

/// Run-level production inductance evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceRunEvidence {
    /// Accepted inductance source code.
    pub source: u8,
    pub status: u8,
    pub warning_flags: u16,
    pub ud_count: i16,
    pub realized_frequency_millihz: u32,
    pub elapsed_us: u32,
    pub openloop_phi_delta_counts: u32,
    pub sample_count: u16,
    pub encoder_delta_counts: i32,
    /// OR of pre/post TMC status flags.
    pub status_flags_or: u32,
}

/// Rotating-frame current evidence for production inductance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceFrameEvidence {
    pub id_mean_milli_count: i32,
    pub iq_mean_milli_count: i32,
    pub id_rms_milli_count: u32,
    pub iq_rms_milli_count: u32,
    pub drift_permille: u16,
    pub zero_id_mean_milli_count: i32,
    pub zero_iq_mean_milli_count: i32,
}

/// Accepted count-space inductance estimate and cross-check evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceEstimateEvidence {
    pub x_average_count_ratio_milli: u32,
    pub x_d_count_ratio_milli: u32,
    pub x_q_count_ratio_milli: u32,
    /// `AC_SALIENCY_*` status code.
    pub saliency_status: u8,
    /// `abs(Ld - Lq) / average(Ld, Lq)` in permille.
    pub saliency_permille: u16,
    pub x_mag_nominal_count_ratio_milli: u32,
    pub x_mag_shift_minus_permille: u16,
    pub x_mag_shift_plus_permille: u16,
    pub x_mag_vs_quad_permille: u16,
}

/// Complete production inductance evidence carried from Phase 5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceEvidence {
    pub run: InductanceRunEvidence,
    pub frame: InductanceFrameEvidence,
    pub estimate: InductanceEstimateEvidence,
}

impl InductanceEvidence {
    pub const EMPTY: Self = Self {
        run: InductanceRunEvidence {
            source: INDUCTANCE_SOURCE_NONE,
            status: 0,
            warning_flags: 0,
            ud_count: 0,
            realized_frequency_millihz: 0,
            elapsed_us: 0,
            openloop_phi_delta_counts: 0,
            sample_count: 0,
            encoder_delta_counts: 0,
            status_flags_or: 0,
        },
        frame: InductanceFrameEvidence {
            id_mean_milli_count: 0,
            iq_mean_milli_count: 0,
            id_rms_milli_count: 0,
            iq_rms_milli_count: 0,
            drift_permille: 0,
            zero_id_mean_milli_count: 0,
            zero_iq_mean_milli_count: 0,
        },
        estimate: InductanceEstimateEvidence {
            x_average_count_ratio_milli: 0,
            x_d_count_ratio_milli: 0,
            x_q_count_ratio_milli: 0,
            saliency_status: crate::electrical_capture::AC_SALIENCY_NOT_DETERMINABLE,
            saliency_permille: 0,
            x_mag_nominal_count_ratio_milli: 0,
            x_mag_shift_minus_permille: 0,
            x_mag_shift_plus_permille: 0,
            x_mag_vs_quad_permille: 0,
        },
    };
}

#[cfg(test)]
impl InductanceEvidence {
    pub(crate) const fn test_fixture() -> Self {
        Self {
            run: InductanceRunEvidence {
                source: INDUCTANCE_SOURCE_ROTATING_FIELD,
                status: 0,
                warning_flags: 0,
                ud_count: 768,
                realized_frequency_millihz: 1_000_000,
                elapsed_us: 8_000,
                openloop_phi_delta_counts: 524_288,
                sample_count: 104,
                encoder_delta_counts: 0,
                status_flags_or: 0,
            },
            frame: InductanceFrameEvidence {
                id_mean_milli_count: 20_000,
                iq_mean_milli_count: -84_000,
                id_rms_milli_count: 5_000,
                iq_rms_milli_count: 21_000,
                drift_permille: 40,
                zero_id_mean_milli_count: 0,
                zero_iq_mean_milli_count: 0,
            },
            estimate: InductanceEstimateEvidence {
                x_average_count_ratio_milli: 8_600,
                x_d_count_ratio_milli: 9_200,
                x_q_count_ratio_milli: 8_000,
                saliency_status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                saliency_permille: 140,
                x_mag_nominal_count_ratio_milli: 8_780,
                x_mag_shift_minus_permille: 4,
                x_mag_shift_plus_permille: 4,
                x_mag_vs_quad_permille: 20,
            },
        }
    }
}

/// Which constructor family built a `CommissioningEngine`; selects the terminal reply payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EngineKind {
    Commission,
    Calibrate,
    Selftest,
    OuterTune,
}

/// Immutable velocity-sweep plan evidence emitted before the first moving stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepPlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of every frozen plan field and generated gain.
    pub plan_digest: u64,
    pub requested_velocity_mrev_s: u32,
    pub requested_velocity_source: u8,
    pub planned_velocity_mrev_s: u32,
    pub effective_ceiling_mrev_s: u32,
    pub clamp_flags: u16,
    /// Source of the binding effective ceiling.
    pub effective_ceiling_source: u8,
    pub target_velocity_rpm: i32,
    pub p_start: u16,
    pub p_top: u16,
    pub rung_count: u8,
    pub observations_per_direction: u16,
    pub moving_stroke_us: u32,
    pub zero_settle_us: u32,
    pub nominal_workflow_ms: u32,
    pub maximum_workflow_ms: u32,
    pub recovery_origin_band_counts: u32,
    pub recovery_nominal_slot_us: u32,
    pub recovery_maximum_slot_us: u32,
    pub recovery_slot_count: u8,
    pub max_stroke_travel_mrev: u32,
    pub settle_travel_reserve_mrev: u32,
    pub negative_position_headroom_mrev: u32,
    pub positive_position_headroom_mrev: u32,
    pub hard_torque_limit: u16,
    pub usable_torque_limit: u16,
}

/// One causally sequenced rung-origin recovery record shared by Proportional and Integral.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RungOriginRecoveryEvidence {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub report: crate::commissioning::outer::velocity::recovery::RungOriginRecoveryReport,
}

/// Recovery stabilization lead-in and scored-rest evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RecoveryRestDiagnostic {
    pub(crate) plan_digest: u64,
    pub(crate) exact_velocity_interval_q: [i64; 2],
    pub(crate) run_sequence: u32,
    pub(crate) start_offset_counts: i32,
    pub(crate) closest_offset_counts: i32,
    pub(crate) final_offset_counts: i32,
    pub(crate) planned_return_distance_counts: u32,
    pub(crate) first_in_band_offset_counts: i32,
    pub(crate) displacement_to_band_counts: i32,
    pub(crate) entry_to_band_duration_us: u32,
    pub(crate) lead_in_net_displacement_counts: i32,
    pub(crate) lead_in_displacement_span_counts: u32,
    pub(crate) scored_net_displacement_counts: i32,
    pub(crate) scored_min_displacement_counts: i32,
    pub(crate) scored_max_displacement_counts: i32,
    pub(crate) p_raw: u16,
    pub(crate) evidence_sequence: u16,
    pub(crate) peak_torque_target_abs: u16,
    pub(crate) stage: crate::commissioning::outer::velocity::recovery::RecoveryStage,
    pub(crate) rung_index: u8,
    pub(crate) stationarity_code: u8,
    pub(crate) interval_present: u8,
    pub(crate) current_contact_code: u8,
    pub(crate) disposition: u8,
}

/// Causal owner of one completed scored-rest selector decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RestSelectionOwner {
    ProportionalObservation = 0,
    ProportionalRecovery = 1,
    ProportionalSelectedRecovery = 2,
    IntegralAnchor = 3,
    IntegralPositiveObservation = 4,
    IntegralRecovery = 5,
    IntegralCleanup = 6,
}

/// Exact trace-only evidence for one completed scored-rest selector decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RestSelectionDiagnostic {
    pub(crate) intervals_q: [[i64; 2]; 3],
    pub(crate) plan_digest: u64,
    pub(crate) run_sequence: u32,
    pub(crate) evidence_sequence: u16,
    pub(crate) selected_suffix_len: u16,
    pub(crate) structural_codes: [crate::autotune::velocity_measurement::RestStructuralCode; 3],
    pub(crate) interval_present_mask: u8,
    pub(crate) usable_mask: u8,
    pub(crate) zero_compatible_mask: u8,
    pub(crate) selected_mask: u8,
    pub(crate) stage: crate::commissioning::outer::velocity::recovery::RecoveryStage,
    pub(crate) owner: RestSelectionOwner,
    pub(crate) rung_index: u8,
    pub(crate) direction: u8,
    pub(crate) slot: u8,
    pub(crate) disposition: crate::autotune::velocity_measurement::RestSelectionDisposition,
    pub(crate) retained_sample_count: u16,
}

/// Causal current-limit evidence for one signed-plus-zero velocity primitive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityPrimitiveCurrentEvidence {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub stage: u8,
    pub rung_index: u8,
    /// Fixed signed observation slot.
    pub slot: u8,
    pub clamp_limit: u16,
    /// Verified clamp readback that authorized the signed target.
    pub clamp_readback: u16,
    pub moving_pid_output_peak: u16,
    pub zero_pid_output_peak: u16,
    /// Complete status word observed at the signed-to-zero boundary.
    pub moving_status_flags: u32,
    pub zero_status_flags: u32,
    pub capability: u8,
    pub contact: u8,
}

/// Compact gate statistics retained at the Integral event boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StationarityRejectionDiagnostic {
    pub suffix_len: u16,
    pub usable_plateau: bool,
    pub slope_passed: bool,
    pub lag_one_passed: bool,
    pub settled_rate_interval_q: [i64; 2],
    pub slope_rejection_margin_q: i64,
    pub lag_one_rejection_margin_q: i32,
    pub slope_q: i64,
    pub slope_half_width_q: u64,
}

impl From<crate::autotune::velocity_measurement::SuffixDiagnostic>
    for StationarityRejectionDiagnostic
{
    fn from(value: crate::autotune::velocity_measurement::SuffixDiagnostic) -> Self {
        Self {
            suffix_len: value.suffix_len,
            usable_plateau: value.usable_plateau,
            slope_passed: value.slope_passed,
            lag_one_passed: value.lag_one_passed,
            settled_rate_interval_q: value.settled_rate_interval_q,
            slope_rejection_margin_q: value.slope_rejection_margin_q,
            lag_one_rejection_margin_q: value.lag_one_rejection_margin_q,
            slope_q: value.slope_q,
            slope_half_width_q: value.slope_half_width_q,
        }
    }
}

/// One analyzed Integral moving observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralObservation {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact Integral plan digest.
    pub plan_digest: u64,
    pub rung_index: u8,
    pub slot: u8,
    pub i_raw: u16,
    pub summary: crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationSummary,
    pub stationarity: crate::autotune::velocity_measurement::StationarityResult,
    pub rejection: Option<StationarityRejectionDiagnostic>,
}

/// One completed Integral rung after both directional consensus folds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralRungReport {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact Integral plan digest.
    pub plan_digest: u64,
    pub rung_index: u8,
    pub i_raw: u16,
    pub directions:
        [crate::commissioning::outer::velocity::integral_evidence::IntegralRungEvidence; 2],
    /// Recorded breakaway target terminus after this rung, one-based; zero when none is recorded.
    pub target_terminus_plus_one: u8,
}

/// Nonterminal Integral evidence produced by one cooperative poll.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityIntegralEvent {
    Observation(VelocityIntegralObservation),
    Rung(VelocityIntegralRungReport),
    Recovery(RungOriginRecoveryEvidence),
    RecoveryRest(RecoveryRestDiagnostic),
    RestSelection(RestSelectionDiagnostic),
    /// Current-limit evidence for one completed signed-plus-zero primitive.
    Current(VelocityPrimitiveCurrentEvidence),
}

/// Report-only Integral terminal class.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityIntegralOutcome {
    FirstRunRetained = 0,
    RepeatabilityConfirmed = 1,
    Inconclusive = 2,
    Fault = 3,
    RejectedPlanMismatch = 4,
    Failed = 5,
}

/// Firmware-owned terminal outcome for one reversal-robustness gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RobustnessOutcome {
    Complete = 0,
    Rejected = 1,
    Inconclusive = 2,
    Failed = 3,
}

/// Compact terminal for one reversal-standstill robustness gate.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct RobustnessReversalTerminal {
    pub schema_revision: u16,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub outcome: RobustnessOutcome,
    /// `ROBUSTNESS_CAUSE_*` gate-local terminal cause.
    pub cause: u8,
    pub selected_p: u16,
    pub selected_i: u16,
    /// Signed target velocity used for the probe, in RPM.
    pub target_velocity_rpm: i32,
    /// The plant-adaptive rate this run's gate thresholds were constructed from (see
    /// `reduce_plant_rate_q`), in rate-Q units.
    pub plant_rate_q: i64,
    /// The constructed IAE gate threshold for this run (see `robustness_thresholds_for`), in rate-Q
    /// microseconds.
    pub iae_max_q: i64,
    pub evidence: crate::commissioning::outer::velocity::robustness_evidence::RobustnessEvidence,
    pub run_started_us: u64,
    pub run_completed_us: u64,
    pub safety_fault: Option<OuterSafetyFault>,
}

/// Namespaced Integral terminal cause: engine (`INTEGRAL_CAUSE_*`), error
/// (`CommissionError::status_code()`), or dispatch (`INTEGRAL_DISPATCH_CAUSE_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum IntegralOutcomeCause {
    /// From `integral_sweep::INTEGRAL_CAUSE_*`.
    Engine(u8),
    /// From `CommissionError::status_code()`.
    Error(u8),
    /// From `commission_dispatch::integral::INTEGRAL_DISPATCH_CAUSE_*`.
    Dispatch(u8),
}

impl IntegralOutcomeCause {
    pub(crate) const fn namespace_code(self) -> u8 {
        match self {
            Self::Engine(_) => 0,
            Self::Error(_) => 1,
            Self::Dispatch(_) => 2,
        }
    }

    pub(crate) const fn raw_value(self) -> u8 {
        match self {
            Self::Engine(value) | Self::Error(value) | Self::Dispatch(value) => value,
        }
    }
}

/// Detailed Integral terminal retained for lifecycle and canonical reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact Integral plan digest.
    pub plan_digest: u64,
    /// Rolling FNV-1a digest over the plan marker and all nonterminal evidence.
    pub digest: u64,
    pub outcome: VelocityIntegralOutcome,
    /// Dedicated Integral phase-local cause, namespaced (see [`IntegralOutcomeCause`]).
    pub cause: IntegralOutcomeCause,
    pub breakaway_acceptance: bool,
    pub fixed_p: u16,
    /// The confirmed integral gain, once a breakaway acceptance point has been installed for this
    /// terminal (`RepeatabilityConfirmed` with a recorded target terminus).
    pub candidate_i: Option<u16>,
    pub recovery_unavailable: bool,
    pub recovered_with_current_headroom: bool,
    pub recovery_quantization_exposed: bool,
    pub probe_constrained_test_point: bool,
    pub rest_boundary_rung_plus_one: u8,
    pub rest_boundary_slot_plus_one: u8,
    pub evidence: crate::commissioning::outer::velocity::integral_evidence::IntegralRunEvidence,
    pub expected_observations: u16,
    pub emitted_observations: u16,
    pub expected_rungs: u16,
    pub emitted_rungs: u16,
    pub run_started_us: u64,
    pub run_completed_us: u64,
    pub safety_fault: Option<OuterSafetyFault>,
    /// Whether `evidence.assess(..)` was already `Sufficient` when a rest check rejected the run.
    pub rest_rejection_after_sufficiency: bool,
    /// The Integral owner responsible, only when `rest_rejection_after_sufficiency`.
    pub rest_rejection_owner: Option<RestSelectionOwner>,
}

/// Immutable breakaway-probe plan evidence emitted before the first probe stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbePlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the frozen probe plan.
    pub plan_digest: u64,
    pub max_observations: u8,
    pub search_count: u8,
    pub p_start_raw: u16,
    pub p_top_raw: u16,
    /// Command-level workflow envelope digest bound into `plan_digest`.
    pub workflow_digest: u64,
    /// Frozen signed-target magnitude in mechanical RPM register units.
    pub target_velocity_rpm: i32,
    pub hard_torque_limit: u16,
    pub usable_torque_limit: u16,
    pub pwm_hz: u32,
    pub encoder_counts_per_rev: u32,
    pub motion_threshold_counts: u32,
    pub max_capture_interval_us: u32,
}

/// Resolved repeatable-motion rung for the complete upward probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbeResolved {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe plan.
    pub plan_digest: u64,
    pub rung_index: u8,
    pub breakaway_p_raw: u16,
    pub motion_threshold_counts: u32,
    pub observation_count: u8,
}

/// Detailed terminal evidence for a probe campaign that did not resolve a breakaway (every
/// `crate::commissioning::outer::velocity::breakaway_campaign::BreakawayProbeOutcome` variant other
/// than `BreakawayFound`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbeTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the frozen probe plan.
    pub plan_digest: u64,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    pub has_safety_fault: u8,
    pub fault_checkpoint: u8,
}

/// Immutable additive-discovery plan evidence emitted before the first discovery stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdditiveDiscoveryPlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the frozen discovery plan.
    pub plan_digest: u64,
    /// Exact digest of the probe plan whose search located this ladder's breakaway.
    pub prior_plan_digest: u64,
    /// Breakaway-campaign schema/rule revision under which this ladder was resolved
    /// (`combined_plan::BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION`).
    pub schema_revision: u16,
    /// Discovery confidence family: `rung_count * observations_per_direction`.
    pub family_size: u16,
    pub rung_count: u8,
    pub observations_per_direction: u16,
    pub floor_p_raw: u16,
    /// Origin of the diagnostic floor: `0` predecessor grid rung, `1` clamped at breakaway.
    pub floor_origin: u8,
    pub breakaway_p_raw: u16,
    pub ceiling_p_raw: u16,
    pub first_additive_step_raw: u32,
    pub maximum_workflow_ms: u32,
}

/// One collected observation's raw settled-rate evidence, streamed on the tick its stroke completes
/// -- before the atom is folded into a discovery rung's nomination bounds or judged for
/// confirmation containment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRawObservation {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe, discovery, or confirmation plan.
    pub plan_digest: u64,
    pub phase: u8,
    /// Zero-based probe or discovery rung; `0` for confirmation.
    pub rung_index: u8,
    /// Probe direction-local slot (`0` initial, `1` repeat), discovery within-direction slot, or
    /// confirmation's fixed `0..=7` schedule slot.
    pub slot_index: u8,
    /// Direction wire code (see `sweep_evidence::direction_report_code`).
    pub direction: u8,
    /// Raw signed settled-rate mean in fixed-point rate units.
    pub mean_rate_q: i64,
    pub variance_of_mean_q2: u128,
    pub target_rate_q: i64,
    /// Frozen stationarity outcome: `0` usable, otherwise the exclusion reason code. Lets an
    /// offline decoder reproduce the `StationarityExcluded` class.
    pub stationarity_code: u8,
    pub settled_rate_low_q: i64,
    pub settled_rate_high_q: i64,
    /// Inclusive lower signed-deficit confidence edge (classify input).
    pub deficit_low_q: i64,
    /// Inclusive upper signed-deficit confidence edge (classify input).
    pub deficit_high_q: i64,
    pub predicted_torque_target_abs: u16,
    pub usable_torque_limit: u16,
    pub hard_torque_limit: u16,
    pub disturbance_present: u8,
    /// Lower disturbance-interval bound in equivalent torque-target Q16.16 counts (`0` when
    /// absent).
    pub disturbance_low_q16: i32,
    /// Upper disturbance-interval bound in the same units (`0` when absent).
    pub disturbance_high_q16: i32,
    /// Drift ratio in parts per million, or `u32::MAX` when unavailable.
    pub drift_ratio_ppm: u32,
    pub post_fit_lag_one_state: u8,
    pub stroke_min_displacement_counts: i32,
    pub stroke_max_displacement_counts: i32,
    pub capture_max_interval_us: u32,
    pub moving_pid_output_peak: u16,
}

/// One recovered probe observation committed to the adaptive cursor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbeDecision {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe plan.
    pub plan_digest: u64,
    /// Direction wire code.
    pub direction: u8,
    pub rung_index: u8,
    pub disposition: u8,
}

/// One in-band discovery rung's nomination bounds and pointwise margin `M_i`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayDiscoveryRungMargin {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning discovery plan.
    pub plan_digest: u64,
    pub collected_index: u8,
    pub p_raw: u16,
    /// Enveloped nomination-interval lower bound `L_i`, in percent * 1000.
    pub lower_percent_milli: i32,
    /// Enveloped nomination-interval upper bound `U_i`, in percent * 1000.
    pub upper_percent_milli: i32,
    /// `M_i = min(L_i - 70, 80 - U_i)`, in percent * 1000.
    pub margin_percent_milli: i32,
    /// `(U_i - L_i) / 2`, in percent * 1000.
    pub half_width_percent_milli: i32,
    /// Achieved rate step from the previous collected in-band rung's own midpoint response, in
    /// percent * 1000; zero for the first collected rung.
    pub adjacent_rate_step_percent_milli: i32,
}

/// Diagnostic evidence for whether the additive ladder's rung zero (the diagnostic floor) itself
/// showed motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRungZeroDiagnostic {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning discovery plan.
    pub plan_digest: u64,
    pub forward_moved: u8,
    pub reverse_moved: u8,
}

/// Detailed terminal evidence for an additive discovery campaign.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdditiveDiscoveryTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the frozen discovery plan.
    pub plan_digest: u64,
    /// Exact digest of the probe plan whose search located this ladder's breakaway.
    pub prior_plan_digest: u64,
    /// Discovery confidence family: `rung_count * observations_per_direction`.
    pub family_size: u16,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    pub collected_count: u8,
    pub has_safety_fault: u8,
    pub stop_rung_index: u8,
    /// `CommissionError::status_code()` for a faulted terminal, or `0` when the terminal carries no
    /// error.
    pub error_code: u8,
}

/// Immutable held-out confirmation plan evidence emitted before the first confirmation stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationPlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the frozen confirmation plan.
    pub plan_digest: u64,
    /// Exact digest of the discovery plan whose nomination screen selected the candidate.
    pub prior_plan_digest: u64,
    pub family_size: u16,
    pub observations_per_direction: u16,
    pub nominated_p_raw: u16,
    /// Inclusive lower edge of the confirmable response band, in percent.
    pub band_lower_percent: u8,
    /// Inclusive upper edge of the confirmable response band, in percent.
    pub band_upper_percent: u8,
    /// Stable wire code for the confirmation capture profile.
    pub capture_profile: u8,
    /// Stable wire code for the acceptance rule.
    pub acceptance_rule: u8,
    /// Nomination screen's pointwise margin `M_i` for the nominated candidate, in percent * 1000.
    pub nominated_margin_percent_milli: i32,
}

/// One fresh held-out confirmation stroke's response interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationObservation {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning confirmation plan.
    pub plan_digest: u64,
    /// Fixed schedule slot index (`0..=7`).
    pub slot_index: u8,
    /// Direction wire code (see `sweep_evidence::direction_report_code`).
    pub direction: u8,
    /// Fresh response-interval lower bound, in percent * 1000.
    pub response_lower_percent_milli: i32,
    /// Fresh response-interval upper bound, in percent * 1000.
    pub response_upper_percent_milli: i32,
    pub relative_standard_error_permille: u16,
}

/// Detailed terminal evidence for a held-out confirmation block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the frozen confirmation plan.
    pub plan_digest: u64,
    /// Exact digest of the discovery plan whose nomination screen selected the candidate.
    pub prior_plan_digest: u64,
    pub family_size: u16,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    /// Forward-direction collected mask (`collected_mask == eligible_mask == included_mask ==
    /// 0b1111` required for acceptance).
    pub forward_collected_mask: u8,
    pub forward_eligible_mask: u8,
    pub forward_included_mask: u8,
    pub reverse_collected_mask: u8,
    pub reverse_eligible_mask: u8,
    pub reverse_included_mask: u8,
    pub accepted: u8,
    pub confirmed_p_raw: u16,
    pub max_relative_se_permille: u16,
    pub required_relative_se_permille: u16,
    pub has_safety_fault: u8,
}

/// Canonical closure record for one complete breakaway campaign.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayCampaignTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub phase: u8,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    pub accepted: u8,
    /// Exact digest of the Integral plan bound through the confirmed candidate, or zero when
    /// `accepted == 0`.
    pub integral_plan_digest: u64,
    /// `CommissionError::status_code()` for a campaign terminal that carries a causally-relevant
    /// error beyond `terminal_cause` alone, or `0` when the terminal carries no error.
    pub error_code: u8,
}

/// Which `p_top` candidate bound the additive-discovery authority ceiling: the current-limit-
/// derived candidate, or the `PidPGain::MAX` representability clamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayCeilingSource {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest: u64,
    /// `sweep_plan::CEILING_BINDING_CURRENT_LIMIT` or
    /// `sweep_plan::CEILING_BINDING_REPRESENTABILITY_CLAMP`.
    pub binding_source: u8,
}

/// One rejected recovery-rest scoring window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRecoveryRestRejected {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe, discovery, or confirmation plan.
    pub plan_digest: u64,
    pub phase: u8,
    pub stage: u8,
    /// Zero-based probe or discovery rung; `0` for confirmation.
    pub rung_index: u8,
    pub slot_index: u8,
    pub direction: u8,
    pub p_raw: u16,
    /// Structural result codes in `VELOCITY_SUFFIX_LENGTHS` order.
    pub structural_codes: [u8; 3],
    /// Bit set when the matching suffix interval is present.
    pub interval_present_mask: u8,
    /// Bit set when every structural stationarity gate passed.
    pub usable_mask: u8,
    /// Bit set when a structurally usable closed interval contains zero.
    pub zero_compatible_mask: u8,
    /// Bit naming the selected suffix, or zero when none was selected.
    pub selected_mask: u8,
    /// Selected suffix length, or zero when none was selected.
    pub selected_suffix_len: u16,
    /// Final selector disposition; always `Rejected` for this record.
    pub disposition: u8,
    /// Exact simultaneous intervals in `VELOCITY_SUFFIX_LENGTHS` order.
    pub intervals_q: [[i64; 2]; 3],
    /// Net settle displacement over the scored window, clamped to `-2..=2`.
    pub scored_net_displacement_counts: i8,
    pub scored_min_displacement_counts: i8,
    pub scored_max_displacement_counts: i8,
}

/// One recovery attempt refused because the origin moved under it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRecoveryOriginMismatch {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe, discovery, or confirmation plan.
    pub plan_digest: u64,
    pub phase: u8,
    pub stage: u8,
    /// Zero-based probe or discovery rung; `0` for confirmation.
    pub rung_index: u8,
    pub slot_index: u8,
    pub direction: u8,
    pub p_raw: u16,
    pub expected_start_offset_counts: i32,
    pub observed_position_counts: i32,
    /// `observed - expected`; the exact discrepancy that raised the fault.
    pub delta_counts: i32,
    pub position_window_counts: u32,
}

/// One breakaway-campaign evidence record ready for wire emission.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub enum BreakawayCampaignEvent {
    ProbePlanned(BreakawayProbePlanned),
    ProbeResolved(BreakawayProbeResolved),
    ProbeTerminal(BreakawayProbeTerminal),
    /// The immutable additive discovery plan and the ceiling's binding source, emitted before
    /// discovery's first stroke.
    DiscoveryPlanned {
        planned: AdditiveDiscoveryPlanned,
        ceiling_source: BreakawayCeilingSource,
    },
    /// One collected observation's raw settled-rate evidence, streamed on the tick its stroke
    /// completes for independent offline recomputation of the nomination margins and confirmation
    /// containment.
    RawObservation(BreakawayRawObservation),
    ProbeDecision(BreakawayProbeDecision),
    DiscoveryRungMargin(BreakawayDiscoveryRungMargin),
    /// A discovery-campaign terminal: the rung-zero motion diagnostic (when available) and the
    /// terminal record.
    DiscoveryTerminal {
        terminal: AdditiveDiscoveryTerminal,
        rung_zero: Option<BreakawayRungZeroDiagnostic>,
    },
    ConfirmationPlanned(BandConfirmationPlanned),
    /// A confirmation-block terminal: every fresh stroke's response interval and the terminal
    /// itself.
    ConfirmationTerminal {
        terminal: BandConfirmationTerminal,
        forward:
            crate::commissioning::outer::velocity::band_confirmation::ConfirmationDirectionalInput,
        reverse:
            crate::commissioning::outer::velocity::band_confirmation::ConfirmationDirectionalInput,
    },
    CampaignTerminal(BreakawayCampaignTerminal),
    RecoveryRestRejected(BreakawayRecoveryRestRejected),
    RecoveryOriginMismatch(BreakawayRecoveryOriginMismatch),
}

/// Generic terminal tying an inconclusive outer phase to its detailed report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OuterInconclusive {
    pub phase: CommissionPhase,
    /// Bit zero is forward sufficiency and bit one is reverse sufficiency.
    pub sufficient_direction_mask: u8,
    pub cause: u8,
    /// Low 32 bits of the exact report digest.
    pub digest_low: u32,
    /// High 32 bits of the exact report digest.
    pub digest_high: u32,
}

/// Result of a `step()` call.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub enum StepResult {
    ReadyNow,
    At(u64),
    PhaseChanged(CommissionPhase),
    VelocityIntegralEvent(VelocityIntegralEvent),
    VelocityIntegralTerminal(VelocityIntegralTerminal),
    RobustnessReversalTerminal(RobustnessReversalTerminal),
    BreakawayCampaignEvent(BreakawayCampaignEvent),
    BreakawayIntegralPlanReady {
        run_sequence: u32,
        plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
    },
    InnerFailed(CommissionError),
    OuterDone(OuterResult),
    OuterFailed(CommissionError),
    CalibrateDone(AdcCalResult),
    SelftestDone,
    CommissionDone(CommissionResult),
}

/// No installed-tuning safety-envelope fault was reported.
pub const OUTER_SAFETY_FAULT_NONE: u8 = 0;
/// Host budget was structurally invalid before installed-tuning actuation.
pub const OUTER_SAFETY_FAULT_INVALID_BUDGET: u8 = 1;
/// Host budget was too small to collect useful installed-tuning evidence.
pub const OUTER_SAFETY_FAULT_UNUSABLE_BUDGET: u8 = 2;
/// Installed-tuning primitive exceeded its host-provided duration cap.
pub const OUTER_SAFETY_FAULT_DURATION: u8 = 3;
/// Installed-tuning encoder velocity exceeded its active safety cap.
pub const OUTER_SAFETY_FAULT_VELOCITY: u8 = 4;
/// Installed-tuning accumulated encoder position exceeded its active travel window.
pub const OUTER_SAFETY_FAULT_POSITION: u8 = 5;
/// Reserved, historical-only: the retired mechanical-ID position-to-torque
/// handoff did not settle before timeout. No live phase reports it; never
/// reassign.
pub const OUTER_SAFETY_FAULT_POST_SWITCH_SETTLE: u8 = 6;
/// Encoder observations were separated by too much time for wrap-safe interpretation.
pub const OUTER_SAFETY_FAULT_OBSERVATION_GAP: u8 = 7;
/// A single encoder delta reached the inclusive quarter-turn observability guard.
pub const OUTER_SAFETY_FAULT_QUARTER_TURN: u8 = 8;
/// Velocity sweep reached the configured hard torque-target clip.
pub const OUTER_SAFETY_FAULT_CURRENT: u8 = 9;
/// Rung-origin recovery increased absolute origin error beyond quantization allowance.
pub const OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY: u8 = 10;
/// Installed-tuning primitive-local travel exceeded its per-stroke geometry window.
pub const OUTER_SAFETY_FAULT_POSITION_LOCAL: u8 = 11;

/// Host-visible installed-tuning safety-envelope fault evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OuterSafetyFault {
    /// `OUTER_SAFETY_FAULT_*` reason code.
    pub reason: u8,
    pub max_travel_mrev: u16,
    pub max_velocity_mrev_s: u16,
    pub max_duration_ms: u16,
    pub direction_mask: u8,
    /// Signed encoder delta for the sample that tripped the gate.
    pub delta_counts: i32,
    /// Elapsed time for `delta_counts`, in microseconds.
    pub dt_us: u32,
    /// Signed sample velocity in encoder counts per millisecond.
    pub velocity_counts_per_ms: i32,
    pub velocity_cap_counts_per_ms: i32,
    /// Signed accumulated position from the installed-tuning start, in encoder counts.
    pub position_counts: i32,
    /// Active signed-position window in encoder counts.
    pub position_window_counts: i32,
    pub elapsed_us: u32,
    pub duration_cap_us: u32,
}

/// Timing capture did not run.
pub const COMMISSION_TIMING_NOT_RUN: u8 = 0;
/// Timing capture completed inside its method envelope.
pub const COMMISSION_TIMING_ACCEPTED: u8 = 1;
/// Timing capture was rejected by its method envelope.
pub const COMMISSION_TIMING_REJECTED: u8 = 2;

/// Electrical-identification timing method identifier for resistance.
pub const COMMISSION_TIMING_METHOD_RESISTANCE: u8 = 0;
/// Electrical-identification timing method identifier for inductance.
pub const COMMISSION_TIMING_METHOD_INDUCTANCE: u8 = 1;
/// Electrical-identification timing method slot reserved for the retired delay capture; always
/// reads `CommissionTimingSummary::NOT_RUN`.
pub const COMMISSION_TIMING_METHOD_DELAY: u8 = 2;

/// Compact retained timing result for one electrical-identification method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CommissionTimingSummary {
    /// `COMMISSION_TIMING_*` state.
    pub status: u8,
    pub overflowed: bool,
    pub evidence: crate::commissioning::capture_timing::CaptureTimingEvidence,
}

impl CommissionTimingSummary {
    /// Summary for a method that did not run.
    pub const NOT_RUN: Self = Self {
        status: COMMISSION_TIMING_NOT_RUN,
        overflowed: false,
        evidence: crate::commissioning::capture_timing::CaptureTimingEvidence::EMPTY,
    };

    /// Build an accepted summary.
    #[must_use]
    pub const fn accepted(
        evidence: crate::commissioning::capture_timing::CaptureTimingEvidence,
    ) -> Self {
        Self {
            status: COMMISSION_TIMING_ACCEPTED,
            overflowed: false,
            evidence,
        }
    }

    /// Build a rejected summary from the last observable timing snapshot.
    #[must_use]
    pub const fn rejected(
        evidence: crate::commissioning::capture_timing::CaptureTimingEvidence,
        overflowed: bool,
    ) -> Self {
        Self {
            status: COMMISSION_TIMING_REJECTED,
            overflowed,
            evidence,
        }
    }
}

/// Pack three R/L/delay timing summaries into the terminal wire bitfield.
#[must_use]
pub fn pack_commission_timing_summary(summaries: [CommissionTimingSummary; 3]) -> u32 {
    let states = u32::from(summaries[0].status & 0x03)
        | (u32::from(summaries[1].status & 0x03) << 2)
        | (u32::from(summaries[2].status & 0x03) << 4);
    let rejected = summaries
        .iter()
        .any(|summary| summary.status == COMMISSION_TIMING_REJECTED);
    let missed_total = summaries.iter().fold(0_u32, |total, summary| {
        total.saturating_add(u32::from(summary.evidence.missed_samples))
    });
    let max_lateness = summaries
        .iter()
        .map(|summary| summary.evidence.max_lateness_us)
        .max()
        .unwrap_or(0);
    let overflowed = summaries.iter().any(|summary| summary.overflowed)
        || missed_total > u32::from(u8::MAX)
        || max_lateness > u32::from(u16::MAX);

    states
        | (u32::from(rejected) << 6)
        | (u32::from(overflowed) << 7)
        | (missed_total.min(u32::from(u8::MAX)) << 8)
        | (max_lateness.min(u32::from(u16::MAX)) << 16)
}

/// Result of inner (electrical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct InnerResult {
    pub status: CommissionStatus,
    pub r_count_milli: u32,
    pub l_count_micro: u32,
    /// Computed flux P gain (Q8.8).
    pub flux_p: u16,
    /// Computed flux I gain (raw Q4.12).
    pub flux_i: u16,
    /// Computed torque P gain (Q8.8).
    pub torque_p: u16,
    /// Computed torque I gain (raw Q4.12).
    pub torque_i: u16,
    /// Candidate `CURRENT_GAINS_SOURCE_*` before validation verdict.
    pub candidate_gains_source: u8,
    /// Candidate `AXIS_SPLIT_SOURCE_*` before validation verdict.
    pub candidate_axis_split_source: u8,
    /// Candidate `CURRENT_GAINS_TIER_*` before validation verdict.
    pub candidate_gains_tier: u8,
    pub candidate_measured_axis_split_permille: u16,
    pub candidate_applied_axis_split_permille: u16,
    pub candidate_axis_split_clamped: u8,
    pub candidate_flux_p: u16,
    pub candidate_flux_i: u16,
    pub candidate_torque_p: u16,
    pub candidate_torque_i: u16,
    pub candidate_attempt: u8,
    pub lambda_us: u32,
    pub theta_e_us: u32,
    /// Source of `theta_e_us`: 0 = the accepted inductance run's phase-delay estimate, 1 = board-
    /// configured fallback.
    pub theta_source: u8,
    pub bandwidth_hz: u32,
    pub inductance: InductanceEvidence,
    pub inner_warning_flags: u8,
    pub timing: [CommissionTimingSummary; 3],
    /// Packed terminal timing status derived from [`Self::timing`].
    pub timing_summary: u32,

    pub resistance_selected_count_slope_milli: i32,
    pub resistance_gain_path_count_slope_milli: i32,
    pub resistance_axis0_count_slope_milli: i32,
    pub resistance_axis1_count_slope_milli: i32,
    pub resistance_axis0_intercept_count: i32,
    pub resistance_axis1_intercept_count: i32,
    pub resistance_axis0_rmse_permille: u16,
    pub resistance_axis1_rmse_permille: u16,
    pub resistance_selected_mask_axis0: u16,
    pub resistance_selected_mask_axis1: u16,
    /// Axis 0 signed-anchor count-slope in milli-units.
    pub resistance_axis0_signed_count_slope_milli: i32,
    /// Axis 1 signed-anchor count-slope in milli-units.
    pub resistance_axis1_signed_count_slope_milli: i32,
    /// Axis 0 signed-anchor asymmetry in permille.
    pub resistance_axis0_signed_asymmetry_permille: u16,
    /// Axis 1 signed-anchor asymmetry in permille.
    pub resistance_axis1_signed_asymmetry_permille: u16,
    /// Axis 0 signed-anchor thermal-drift in permille.
    pub resistance_axis0_drift_permille: u16,
    /// Axis 1 signed-anchor thermal-drift in permille.
    pub resistance_axis1_drift_permille: u16,
    /// OR of all sampled `STATUS_FLAGS` during the resistance run.
    pub resistance_status_flags_or: u32,
    /// Resistance run warning bitfield (`ResistanceWarningFlags`).
    pub resistance_warning_flags: u32,
    pub resistance_peak_abs_current_count: u16,
    pub resistance_max_abs_steady_mean_current_count: u16,
    pub resistance_current_ceiling_count: u16,
    pub resistance_power_stage_tripped: bool,

    pub encoder_alignment_count: u32,
    pub encoder_alignment_electrical_residual_counts: u16,
    pub encoder_alignment_stability_counts: u16,
    pub encoder_alignment_movement_counts: u16,
    pub encoder_alignment_min_movement_counts: u16,
    pub encoder_alignment_counts_per_electrical_rev: u16,
    pub adc_residual_chop0: AdcResidualEvidence,
    pub adc_residual_chop7: AdcResidualEvidence,

    /// `CURRENT_GAINS_SOURCE_*`.
    pub current_gains_source: u8,
    /// `AXIS_SPLIT_SOURCE_*`.
    pub axis_split_source: u8,
    /// `CURRENT_GAINS_TIER_*`.
    pub gains_tier: u8,
    /// Measured axis split ratio, permille where 1000 = 1.000x.
    pub measured_axis_split_permille: u16,
    pub applied_axis_split_permille: u16,
    pub axis_split_clamped: u8,
    /// Bitmask of `CURRENT_VALIDATION_AXIS_*` axes that passed validation.
    pub current_validation_axes: u8,
    pub retry_budget_exhausted: u8,
    /// `CURRENT_LOOP_FAILURE_*`.
    pub current_loop_failure_reason: u8,
    /// Closed-loop entry fast stability status (`CLOSED_LOOP_ACTIVATION_STATUS_*`).
    pub closed_loop_activation_status: u8,
    pub closed_loop_activation_position_1: i32,
    pub closed_loop_activation_position_2: i32,
    pub closed_loop_activation_drift_count: u16,
    pub closed_loop_activation_threshold_count: u16,
    pub closed_loop_activation_runaway: u8,
    /// Sustained-hold gate status (`HOLD_GATE_STATUS_*`).
    pub current_hold_status: u8,
    /// Sustained-hold warning bitfield (`HOLD_GATE_WARNING_*`).
    pub current_hold_warning_flags: u16,
    pub current_hold_elapsed_us: u32,
    pub current_hold_requested_sample_period_us: u16,
    pub current_hold_sample_count: u8,
    pub current_hold_position_span_count: u16,
    pub current_hold_position_drift_count: u16,
    pub current_hold_torque_mean_count: i16,
    pub current_hold_torque_rms_count: u16,
    pub current_hold_torque_peak_to_peak_count: u16,
    pub current_hold_torque_crossing_count: u8,
    pub current_hold_flux_mean_count: i16,
    pub current_hold_flux_rms_count: u16,
    pub current_hold_flux_peak_to_peak_count: u16,
    pub current_hold_flux_crossing_count: u8,
    /// OR of all sampled TMC4671 `STATUS_FLAGS`.
    pub current_hold_status_flags_or: u32,
    pub current_hold_actionable_status_count: u8,
    pub current_validation_step_amplitude: u16,
    pub current_validation_flux_step_amplitude: u16,
    pub current_validation_torque_step_amplitude: u16,
    pub current_validation_pidout_limit: u16,
    pub current_validation_current_limited: u16,
    pub current_validation_voltage_limited: u16,
    pub current_validation_max_p: u16,
    pub current_settled_validation_count: u8,
    pub current_settled_validation: [CurrentValidationSettledSampleEvidence;
        crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
    pub current_flux_validation_count: u8,
    pub current_flux_validation:
        [CurrentValidationAxisEvidence; crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
    pub current_torque_validation_count: u8,
    pub current_torque_validation:
        [CurrentValidationAxisEvidence; crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
}

/// Firmware-authored provenance for a deployed outer tune.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct TuneProvenance {
    /// Probed velocity, in milli-revolutions/second. Same domain as the request field
    /// `requested_velocity_mrev_s`.
    pub probed_velocity_mrev_s: u32,
    /// Settled-rate-Q, in the encoder-rate-Q domain of `selected_response_intervals_q`.
    pub d_eq_q: i32,
    /// Q0.16 normalized confidence fraction (0 = none, 65535 ~= 1.0).
    pub confidence_q: u16,
    /// Lower response-band edge, integer percent (e.g. 70).
    pub band_lower_percent: u8,
    /// Upper response-band edge, integer percent (e.g. 80).
    pub band_upper_percent: u8,
    /// Q0.16 position within `[band_lower_percent, band_upper_percent]` (0 = lower edge, 65535 =
    /// upper edge).
    pub band_position_q: u16,
}

/// `PositionTune` diagnostic provenance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct PositionTuneProvenance {
    pub predicted_start_p: u16,
    pub last_rung_p: u16,
    pub min_overshoot_p: u16,
    pub min_overshoot_units: u32,
    pub bound_units: u32,
    pub homing_speed_mrev_s: u32,
    pub motion_speed_mrev_s: u32,
    pub homing_peak_abs_units: u32,
    pub motion_cruise_mean_abs_units: u32,
    pub motion_overshoot_units: u32,
    pub motion_cruise_rms_units: u32,
    /// Nominal position-loop bandwidth in Hz for the last rung, from `bandwidth_hz =
    /// (position_p/256) * 65536 / (60*2*pi)` (an already-validated formula, position-only, no
    /// torque constant).
    pub nominal_bandwidth_hz: u32,
    pub dither_margin_milli: i32,
    pub rungs_measured: u8,
    /// Feedforward paths live on the last admitted motion stroke: bit 0 velocity, 2 acceleration.
    pub stimulus_feedforward_paths: u8,
}

/// Result of outer (mechanical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct OuterResult {
    pub status: CommissionStatus,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q4.12).
    pub velocity_i: u16,
    pub velocity_limit: u32,
    /// Position P gain (Q8.8).
    pub position_p: u16,
    /// Position I gain (Q4.12).
    pub position_i: u16,
    /// Velocity filter cutoff Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Position filter cutoff Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Flux filter cutoff Hz (0 = disabled).
    pub flux_filter_hz: u16,
    /// Torque filter cutoff Hz (0 = disabled).
    pub torque_filter_hz: u16,
    /// Warning reason for accepted-with-warning results (0 = none).
    pub warning_code: u8,
    pub provenance: TuneProvenance,
    /// `PositionTune` diagnostic provenance.
    pub position_tune: PositionTuneProvenance,
    /// Wire code of the specific `PositionTuneOutcome` behind a
    /// `CommissionError::PositionTuneFailed`.
    pub position_tune_outcome_code: u8,
}

impl Default for OuterResult {
    fn default() -> Self {
        Self {
            status: CommissionStatus::Failed(CommissionError::Spi),
            velocity_p: 0,
            velocity_i: 0,
            velocity_limit: 0,
            position_p: 0,
            position_i: 0,
            velocity_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            torque_filter_hz: 0,
            warning_code: 0,
            provenance: TuneProvenance::default(),
            position_tune: PositionTuneProvenance::default(),
            position_tune_outcome_code: 0,
        }
    }
}

/// ADC residual stage sampled with PWM chopping disabled after offset programming.
pub const ADC_RESIDUAL_STAGE_CHOP_OFF: u8 = 0;
/// ADC residual stage sampled with PWM chopping restored after offset programming.
pub const ADC_RESIDUAL_STAGE_CHOP_ON: u8 = 1;

/// Zero-current ADC residual evidence captured after programming ADC offsets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdcResidualEvidence {
    /// `ADC_RESIDUAL_STAGE_*` discriminator.
    pub stage: u8,
    pub sample_count: u8,
    /// `PWM_SV_CHOP` readback for this residual block.
    pub pwm_sv_chop: u8,
    /// `PWM_BBM_H_BBM_L` readback for this residual block.
    pub pwm_bbm: u32,
    /// `ADC_I0_SCALE_OFFSET` readback after programming.
    pub adc_i0_scale_offset: u32,
    /// `ADC_I1_SCALE_OFFSET` readback after programming.
    pub adc_i1_scale_offset: u32,
    /// Mean low-half raw phase-current monitor from `ADC_IWY_IUX`.
    pub adc_iux_mean_count: i16,
    /// Mean high-half raw phase-current monitor from `ADC_IWY_IUX`.
    pub adc_iwy_mean_count: i16,
    /// Mean low-half measured flux current from `PID_TORQUE_FLUX_ACTUAL`.
    pub pid_flux_mean_count: i16,
    /// Mean high-half measured torque current from `PID_TORQUE_FLUX_ACTUAL`.
    pub pid_torque_mean_count: i16,
}

impl AdcResidualEvidence {
    pub const EMPTY: Self = Self {
        stage: ADC_RESIDUAL_STAGE_CHOP_OFF,
        sample_count: 0,
        pwm_sv_chop: 0,
        pwm_bbm: 0,
        adc_i0_scale_offset: 0,
        adc_i1_scale_offset: 0,
        adc_iux_mean_count: 0,
        adc_iwy_mean_count: 0,
        pid_flux_mean_count: 0,
        pid_torque_mean_count: 0,
    };
}

/// Result of the calibration subset requested by a calibrate engine.
#[derive(Debug, Clone, defmt::Format)]
pub struct AdcCalResult {
    pub adc_i0: u16,
    pub adc_i1: u16,
    pub encoder_count: u32,
    /// Folded residual of `encoder_count` modulo one electrical revolution.
    pub encoder_alignment_electrical_residual_counts: u16,
    pub encoder_alignment_stability_counts: u16,
    pub encoder_alignment_movement_counts: u16,
    pub encoder_alignment_min_movement_counts: u16,
    pub encoder_alignment_counts_per_electrical_rev: u16,
    pub adc_residual_chop0: AdcResidualEvidence,
    pub adc_residual_chop7: AdcResidualEvidence,
}

/// Result of current-loop commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct CommissionResult {
    pub inner: InnerResult,
    /// Fallback velocity P gain (Q8.8) written during closed-loop entry.
    pub fallback_velocity_p: u16,
    /// Fallback velocity I gain (Q4.12) written during closed-loop entry.
    pub fallback_velocity_i: u16,
    /// Fallback position P gain (Q8.8) written during closed-loop entry.
    pub fallback_position_p: u16,
    /// Fallback position I gain (Q4.12) written during closed-loop entry.
    pub fallback_position_i: u16,
    pub fallback_velocity_limit: u32,
    /// Applied velocity-filter cutoff in Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Applied torque-filter cutoff in Hz (0 = disabled).
    pub torque_filter_hz: u16,
    /// Applied position-filter cutoff in Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Applied flux-filter cutoff in Hz (0 = disabled).
    pub flux_filter_hz: u16,
}

#[cfg(test)]
mod commission_timing_tests {
    use super::{
        COMMISSION_TIMING_ACCEPTED, COMMISSION_TIMING_NOT_RUN, COMMISSION_TIMING_REJECTED,
        CommissionTimingSummary, pack_commission_timing_summary,
    };
    use crate::commissioning::capture_timing::CaptureTimingEvidence;

    fn summary(status: u8, missed_samples: u16, max_lateness_us: u32) -> CommissionTimingSummary {
        CommissionTimingSummary {
            status,
            overflowed: false,
            evidence: CaptureTimingEvidence {
                requested_period_us: 100,
                valid_samples: 12,
                missed_samples,
                max_consecutive_misses: 1,
                max_lateness_us,
                max_interval_us: 125,
                max_poll_wall_us: 20,
                max_spi_wall_us: 8,
            },
        }
    }

    #[test]
    fn commission_timing_packs_states_and_worst_case_metrics() {
        let packed = pack_commission_timing_summary([
            summary(COMMISSION_TIMING_ACCEPTED, 2, 15),
            summary(COMMISSION_TIMING_REJECTED, 3, 40),
            CommissionTimingSummary::NOT_RUN,
        ]);

        assert_eq!(packed & 0x03, u32::from(COMMISSION_TIMING_ACCEPTED));
        assert_eq!((packed >> 2) & 0x03, u32::from(COMMISSION_TIMING_REJECTED));
        assert_eq!((packed >> 4) & 0x03, u32::from(COMMISSION_TIMING_NOT_RUN));
        assert_ne!(packed & (1 << 6), 0);
        assert_eq!((packed >> 8) & 0xff, 5);
        assert_eq!(packed >> 16, 40);
    }

    #[test]
    fn commission_timing_marks_metric_saturation_as_overflow() {
        let mut resistance = summary(COMMISSION_TIMING_ACCEPTED, 250, 70_000);
        resistance.overflowed = true;
        let packed = pack_commission_timing_summary([
            resistance,
            summary(COMMISSION_TIMING_ACCEPTED, 20, 1),
            CommissionTimingSummary::NOT_RUN,
        ]);

        assert_ne!(packed & (1 << 7), 0);
        assert_eq!((packed >> 8) & 0xff, 0xff);
        assert_eq!(packed >> 16, 0xffff);
    }
}

#[cfg(test)]
mod integral_cause_tests {
    use super::IntegralOutcomeCause;

    #[test]
    fn namespace_and_raw_value_round_trip_per_variant() {
        assert_eq!(IntegralOutcomeCause::Engine(4).namespace_code(), 0);
        assert_eq!(IntegralOutcomeCause::Engine(4).raw_value(), 4);

        assert_eq!(IntegralOutcomeCause::Error(4).namespace_code(), 1);
        assert_eq!(IntegralOutcomeCause::Error(4).raw_value(), 4);

        assert_eq!(IntegralOutcomeCause::Dispatch(4).namespace_code(), 2);
        assert_eq!(IntegralOutcomeCause::Dispatch(4).raw_value(), 4);
    }

    #[test]
    fn colliding_raw_values_remain_distinguishable_by_namespace() {
        let engine = IntegralOutcomeCause::Engine(4);
        let error = IntegralOutcomeCause::Error(4);
        let dispatch = IntegralOutcomeCause::Dispatch(4);

        assert_ne!(engine.namespace_code(), error.namespace_code());
        assert_ne!(engine.namespace_code(), dispatch.namespace_code());
        assert_ne!(error.namespace_code(), dispatch.namespace_code());
        assert_eq!(engine.raw_value(), error.raw_value());
        assert_eq!(engine.raw_value(), dispatch.raw_value());
    }
}

#[cfg(test)]
mod tests {
    use super::OuterResult;

    #[test]
    fn outer_result_provenance_defaults_to_zero() {
        let r = OuterResult::default();
        assert_eq!(r.provenance.probed_velocity_mrev_s, 0);
        assert_eq!(r.provenance.d_eq_q, 0);
        assert_eq!(r.provenance.confidence_q, 0);
        assert_eq!(r.provenance.band_lower_percent, 0);
        assert_eq!(r.provenance.band_upper_percent, 0);
        assert_eq!(r.provenance.band_position_q, 0);
    }
}
