//! Public data types for commissioning.
//!
//! These types form the cross-crate API surface of the commissioning
//! engine. They are referenced by board binaries for engine construction
//! and result dispatch.

use crate::autotune::{CommissionError, CommissionPhase, CommissionStatus};

/// Compatibility re-export for existing `commissioning::types` backend users.
pub use crate::commissioning::backend::CommissioningBackend;

/// One current-axis validation evidence item for a single sample delay.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentValidationAxisEvidence {
    /// `CURRENT_AXIS_VALIDATION_*` status code for this delay.
    pub status: u8,
    /// Validation attempt index that produced this evidence.
    pub attempt: u8,
    /// Signed current target used for each pulse.
    pub target: i16,
    /// Delay between target write and sample, in milliseconds.
    pub sample_delay_ms: u16,
    /// Positive-pulse response normalized to target, in permille.
    pub positive_response_permille: u16,
    /// Negative-pulse response normalized to target, in permille.
    pub negative_response_permille: u16,
    /// Maximum opposite-axis response for the positive/negative pair, in permille.
    pub cross_axis_permille: u16,
    /// Maximum instantaneous cross-axis response seen while building this evidence.
    ///
    /// Equals `cross_axis_permille` for single-read samples. For averaged
    /// settled flux samples, `cross_axis_permille` is the averaged gate value
    /// and this field preserves the raw ripple peak for audit.
    pub cross_axis_peak_permille: u16,
    /// Maximum voltage output for the positive/negative pair, in permille.
    pub voltage_output_permille: u16,
    /// Maximum encoder movement for the positive/negative pair.
    pub encoder_delta_counts: u16,
    /// Signed encoder movement observed during the positive pulse.
    pub positive_encoder_delta_counts: i32,
    /// Signed encoder movement observed during the negative pulse.
    pub negative_encoder_delta_counts: i32,
    /// OR of TMC4671 `STATUS_FLAGS` captured for the positive/negative pair.
    pub status_flags_or: u32,
}

impl CurrentValidationAxisEvidence {
    /// Empty evidence slot used for fixed-capacity arrays.
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
    /// Zero-based sample index for this axis.
    pub sample_index: u8,
    /// Direction index: 0 for positive pulse, 1 for negative pulse.
    pub direction: u8,
    /// Raw read index within the settled averaging window.
    pub raw_index: u8,
    /// Validation attempt that produced this raw read.
    pub attempt: u8,
    /// Signed current target used for this pulse.
    pub target: i16,
    /// Delay between target write and this settled gate, in milliseconds.
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
    /// Empty evidence slot used for fixed-capacity arrays.
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
    /// Run status code.
    pub status: u8,
    /// Warning bitfield.
    pub warning_flags: u16,
    /// Commanded d-axis voltage count.
    pub ud_count: i16,
    /// Realized electrical measurement frequency in millihertz.
    pub realized_frequency_millihz: u32,
    /// Elapsed AC measurement window in microseconds.
    pub elapsed_us: u32,
    /// Observed open-loop phi delta across the accepted window.
    pub openloop_phi_delta_counts: u32,
    /// Number of accepted AC samples.
    pub sample_count: u16,
    /// Encoder movement during the energized window.
    pub encoder_delta_counts: i32,
    /// OR of pre/post TMC status flags.
    pub status_flags_or: u32,
}

/// Rotating-frame current evidence for production inductance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceFrameEvidence {
    /// Mean d-axis current in milli-counts.
    pub id_mean_milli_count: i32,
    /// Mean q-axis current in milli-counts.
    pub iq_mean_milli_count: i32,
    /// RMS d-axis spread around the mean in milli-counts.
    pub id_rms_milli_count: u32,
    /// RMS q-axis spread around the mean in milli-counts.
    pub iq_rms_milli_count: u32,
    /// Half-window reactance drift in permille.
    pub drift_permille: u16,
    /// Pre-drive mean d-axis offset in milli-counts.
    pub zero_id_mean_milli_count: i32,
    /// Pre-drive mean q-axis offset in milli-counts.
    pub zero_iq_mean_milli_count: i32,
}

/// Accepted count-space inductance estimate and cross-check evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceEstimateEvidence {
    /// Accepted angle-averaged reactance ratio in milli count-ratio units.
    pub x_average_count_ratio_milli: u32,
    /// D-axis reactance ratio in milli count-ratio units.
    pub x_d_count_ratio_milli: u32,
    /// Q-axis reactance ratio in milli count-ratio units.
    pub x_q_count_ratio_milli: u32,
    /// `AC_SALIENCY_*` status code.
    pub saliency_status: u8,
    /// `abs(Ld - Lq) / average(Ld, Lq)` in permille.
    pub saliency_permille: u16,
    /// Magnitude-branch nominal reactance in milli count-ratio units.
    pub x_mag_nominal_count_ratio_milli: u32,
    /// Magnitude-branch shift with R scaled down by 20%, in permille.
    pub x_mag_shift_minus_permille: u16,
    /// Magnitude-branch shift with R scaled up by 20%, in permille.
    pub x_mag_shift_plus_permille: u16,
    /// Magnitude-vs-quadrature disagreement in permille.
    pub x_mag_vs_quad_permille: u16,
}

impl InductanceEstimateEvidence {
    /// Return true when determinable Ld/Lq bracket the accepted average.
    pub fn brackets_average(&self) -> bool {
        if self.saliency_status != crate::electrical_capture::AC_SALIENCY_DETERMINABLE {
            return false;
        }
        let low = self.x_d_count_ratio_milli.min(self.x_q_count_ratio_milli);
        let high = self.x_d_count_ratio_milli.max(self.x_q_count_ratio_milli);
        low <= self.x_average_count_ratio_milli && self.x_average_count_ratio_milli <= high
    }
}

/// Complete production inductance evidence carried from Phase 5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InductanceEvidence {
    /// Run-level evidence.
    pub run: InductanceRunEvidence,
    /// Rotating-frame current evidence.
    pub frame: InductanceFrameEvidence,
    /// Accepted estimate and cross-check evidence.
    pub estimate: InductanceEstimateEvidence,
}

impl InductanceEvidence {
    /// Empty evidence used before the production inductance estimator has run.
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

/// Discriminant for the four ways a `CommissioningEngine` can be constructed.
/// Used by the dispatch to choose the right reply payload at terminal arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EngineKind {
    /// Engine constructed via `new_commission` — Stage 1 commissioning.
    Commission,
    /// Engine constructed via `new_calibrate` — ADC calibration + alignment.
    Calibrate,
    /// Engine constructed via `new_selftest` — diagnostic selftest run.
    Selftest,
    /// Engine constructed via one of the live outer constructors
    /// (`new_outer_for_tmc_with_fixed_gain_amplitude_preflight`, `_with_integral_preflight`,
    /// `_with_breakaway_campaign`, `_with_robustness`) — Stage 2 mechanical/velocity tune.
    OuterTune,
}

/// Immutable velocity-sweep plan evidence emitted before the first moving stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepPlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of every frozen plan field and generated gain.
    pub plan_digest: u64,
    /// Host-requested mechanical velocity amplitude.
    pub requested_velocity_mrev_s: u32,
    /// Origin of the host-requested amplitude.
    pub requested_velocity_source: u8,
    /// Planned encoder-domain mechanical velocity amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Firmware-owned effective velocity ceiling.
    pub effective_ceiling_mrev_s: u32,
    /// Bitset describing every amplitude reduction.
    pub clamp_flags: u16,
    /// Source of the binding effective ceiling.
    pub effective_ceiling_source: u8,
    /// Frozen positive TMC mechanical-RPM target.
    pub target_velocity_rpm: i32,
    /// First structural velocity-P gain.
    pub p_start: u16,
    /// Last structural velocity-P gain.
    pub p_top: u16,
    /// Number of post-dedupe rungs.
    pub rung_count: u8,
    /// Independent observations per direction and rung.
    pub observations_per_direction: u16,
    /// Fixed moving-stroke duration.
    pub moving_stroke_us: u32,
    /// Fixed zero-target settle duration.
    pub zero_settle_us: u32,
    /// Nominal workflow duration.
    pub nominal_workflow_ms: u32,
    /// Maximum declared workflow duration.
    pub maximum_workflow_ms: u32,
    /// Inclusive immutable-origin recovery band.
    pub recovery_origin_band_counts: u32,
    /// Conservative nominal duration reserved for one recovery slot.
    pub recovery_nominal_slot_us: u32,
    /// Host-declared maximum duration reserved for one recovery slot.
    pub recovery_maximum_slot_us: u32,
    /// Maximum recovery-record count reserved by the plan.
    pub recovery_slot_count: u8,
    /// Complete per-primitive displacement window.
    pub max_stroke_travel_mrev: u32,
    /// Reserved braking and settle displacement.
    pub settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom.
    pub negative_position_headroom_mrev: u32,
    /// Positive absolute-position headroom.
    pub positive_position_headroom_mrev: u32,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target limit.
    pub usable_torque_limit: u16,
}

/// One analyzed moving observation emitted before the following signed stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepObservation {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Zero-based gain-rung index.
    pub rung_index: u8,
    /// Observation slot (`F1`, `R1`, `F2`, `R2`).
    pub slot: u8,
    /// Signed TMC mechanical-RPM target used for this observation.
    pub target_velocity_rpm: i32,
    /// Auditable observation classification and controller-domain evidence.
    pub summary: crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationSummary,
    /// Stationary-suffix analysis result used to construct `summary`.
    pub stationarity: crate::autotune::velocity_measurement::StationarityResult,
    /// Exact displacement represented by the selected suffix.
    pub selected_delta_counts: i64,
    /// Exact elapsed time represented by the selected suffix.
    pub selected_elapsed_us: u64,
    /// Selected blocking level, or zero when no suffix was usable.
    pub selected_level: u8,
    /// Selected completed-block population, or zero when unusable.
    pub selected_blocks: u32,
    /// Settled-rate variance of the mean, or zero when unusable.
    pub variance_of_mean_q2: u128,
    /// Time-weighted settled-rate mean, or zero when unusable.
    pub settled_rate_mean_q: i64,
    /// Non-binding evidence that short-scale structure remains bounded at longer scales.
    pub structured_boundary: Option<StructuredBoundaryEvidence>,
}

/// Diagnostic evidence for a structured but bounded moving tail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StructuredBoundaryEvidence {
    /// Bit set for each declared suffix that passes every stationarity gate.
    pub passing_suffix_mask: u8,
    /// Common settled-rate interval shared by every usable suffix.
    pub common_rate_interval_q: [i64; 2],
    /// Signed slope rejection margin from the binding 64-sample suffix.
    pub binding_slope_margin_q: i64,
    /// Signed lag-one rejection margin from the binding 64-sample suffix.
    pub binding_lag_one_margin_q: i32,
}

/// Completed eight-observation rung with firmware-owned directional consensus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepRungReport {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Zero-based gain-rung index.
    pub rung_index: u8,
    /// Forward compact consensus report.
    pub forward:
        crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport,
    /// Reverse compact consensus report.
    pub reverse:
        crate::commissioning::outer::velocity::sweep_evidence::CompactDirectionalRungReport,
}

/// One causally closed directional model region or fragment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityDirectionalRegionReport {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Closed directional evidence.
    pub region: crate::commissioning::outer::velocity::operating_region::DirectionalRegionSummary,
}

/// One causally closed jointly operable region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityJointRegionReport {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Closed joint-operability evidence.
    pub region: crate::commissioning::outer::velocity::operating_region::JointOperableRegion,
    /// First member gain.
    pub p_low: u16,
    /// Last member gain.
    pub p_high: u16,
}

/// Per-run Proportional handoff emitted after every region record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityProportionalHandoffReport {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Union of every per-run jointly operable component.
    pub joint_union: u32,
    /// Directions whose selected per-run evidence is a valid model region.
    pub model_direction_mask: u8,
    /// Sufficient report-only Proportional handoff.
    pub handoff: crate::commissioning::outer::velocity::operating_region::ProportionalHandoff,
}

/// One causally sequenced rung-origin recovery record shared by Proportional and Integral.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RungOriginRecoveryEvidence {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the owning stage.
    pub evidence_sequence: u16,
    /// Stage-neutral recovery measurements and classification.
    pub report: crate::commissioning::outer::velocity::recovery::RungOriginRecoveryReport,
}

/// Recovery stabilization lead-in and scored-rest evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RecoveryRestDiagnostic {
    /// Digest of the immutable owning-stage plan.
    pub(crate) plan_digest: u64,
    /// Exact zero-target velocity interval from the scored rest window.
    pub(crate) exact_velocity_interval_q: [i64; 2],
    /// Caller-assigned run sequence.
    pub(crate) run_sequence: u32,
    /// Offset from the immutable origin when recovery began.
    pub(crate) start_offset_counts: i32,
    /// Closest offset reached while returning toward the origin.
    pub(crate) closest_offset_counts: i32,
    /// Offset from the immutable origin after the scored rest window.
    pub(crate) final_offset_counts: i32,
    /// Planned driven return distance before entering the origin band.
    pub(crate) planned_return_distance_counts: u32,
    /// First sampled offset inside the origin band.
    pub(crate) first_in_band_offset_counts: i32,
    /// Signed displacement from recovery start to the first in-band sample.
    pub(crate) displacement_to_band_counts: i32,
    /// Elapsed time from recovery start to the first in-band sample.
    pub(crate) entry_to_band_duration_us: u32,
    /// Net displacement during the unscored lead-in window.
    pub(crate) lead_in_net_displacement_counts: i32,
    /// Full displacement span during the unscored lead-in window.
    pub(crate) lead_in_displacement_span_counts: u32,
    /// Net displacement during the scored rest window.
    pub(crate) scored_net_displacement_counts: i32,
    /// Minimum displacement during the scored rest window.
    pub(crate) scored_min_displacement_counts: i32,
    /// Maximum displacement during the scored rest window.
    pub(crate) scored_max_displacement_counts: i32,
    /// Proportional gain used by the selected response.
    pub(crate) p_raw: u16,
    /// Monotonic evidence sequence within the Proportional run.
    pub(crate) evidence_sequence: u16,
    /// Largest sampled absolute velocity-loop output during recovery.
    pub(crate) peak_torque_target_abs: u16,
    /// Recovery phase that owned the diagnostic.
    pub(crate) stage: crate::commissioning::outer::velocity::recovery::RecoveryStage,
    /// Zero-based rung index.
    pub(crate) rung_index: u8,
    /// Stationarity result code for the scored rest window.
    pub(crate) stationarity_code: u8,
    /// Whether the exact velocity interval is present.
    pub(crate) interval_present: u8,
    /// Current-contact classification for the selected recovery.
    pub(crate) current_contact_code: u8,
    /// Zero for confirmed rest and one for completed but unconfirmed rest.
    pub(crate) disposition: u8,
}

/// Causal owner of one completed scored-rest selector decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RestSelectionOwner {
    /// Ordinary Proportional observation.
    ProportionalObservation = 0,
    /// Ordinary Proportional origin recovery.
    ProportionalRecovery = 1,
    /// Selected final-P Proportional recovery.
    ProportionalSelectedRecovery = 2,
    /// Stage-C opening anchor or closing bookend.
    IntegralAnchor = 3,
    /// Stage-C positive-I observation.
    IntegralPositiveObservation = 4,
    /// Stage-C origin recovery.
    IntegralRecovery = 5,
    /// Positive-I I=0 cleanup.
    IntegralCleanup = 6,
    /// Amplitude observation.
    FixedGainAmplitudeObservation = 7,
    /// Amplitude origin recovery.
    FixedGainAmplitudeRecovery = 8,
}

/// Exact trace-only evidence for one completed scored-rest selector decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RestSelectionDiagnostic {
    /// Exact simultaneous intervals in 64-, 128-, and 256-sample order.
    pub(crate) intervals_q: [[i64; 2]; 3],
    /// Digest of the immutable owning-stage plan.
    pub(crate) plan_digest: u64,
    /// Caller-assigned run sequence.
    pub(crate) run_sequence: u32,
    /// Monotonic evidence sequence within the owning stage.
    pub(crate) evidence_sequence: u16,
    /// Selected suffix length, or zero for fallback and rejection.
    pub(crate) selected_suffix_len: u16,
    /// Structural result for each declared suffix.
    pub(crate) structural_codes: [crate::autotune::velocity_measurement::RestStructuralCode; 3],
    /// Bit set when the matching exact interval is present.
    pub(crate) interval_present_mask: u8,
    /// Bit set when every structural stationarity gate passed.
    pub(crate) usable_mask: u8,
    /// Bit set when a usable closed interval contains zero.
    pub(crate) zero_compatible_mask: u8,
    /// Bit naming the selected suffix, or zero.
    pub(crate) selected_mask: u8,
    /// Owning commissioning stage.
    pub(crate) stage: crate::commissioning::outer::velocity::recovery::RecoveryStage,
    /// Causal scored-rest owner.
    pub(crate) owner: RestSelectionOwner,
    /// Zero-based rung or amplitude index.
    pub(crate) rung_index: u8,
    /// Direction code, or `0xff` when inapplicable.
    pub(crate) direction: u8,
    /// Fixed observation slot, or `0xff` when inapplicable.
    pub(crate) slot: u8,
    /// Final selector disposition.
    pub(crate) disposition: crate::autotune::velocity_measurement::RestSelectionDisposition,
    /// Retained observation count at the moment the decision was finalized.
    pub(crate) retained_sample_count: u16,
}

/// Causal current-limit evidence for one signed-plus-zero velocity primitive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityPrimitiveCurrentEvidence {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the owning stage.
    pub evidence_sequence: u16,
    /// Owning stage: zero for Proportional and one for Integral.
    pub stage: u8,
    /// Zero-based rung index.
    pub rung_index: u8,
    /// Fixed signed observation slot.
    pub slot: u8,
    /// Temporary velocity-output clamp configured for the primitive.
    pub clamp_limit: u16,
    /// Verified clamp readback that authorized the signed target.
    pub clamp_readback: u16,
    /// Largest sampled absolute PID output during moving evidence.
    pub moving_pid_output_peak: u16,
    /// Largest sampled absolute PID output during braking and settle.
    pub zero_pid_output_peak: u16,
    /// Complete status word observed at the signed-to-zero boundary.
    pub moving_status_flags: u32,
    /// Complete status word observed after confirmed rest.
    pub zero_status_flags: u32,
    /// Limit-evidence capability declared by the immutable plan.
    pub capability: u8,
    /// First phase in which clamp contact was observed.
    pub contact: u8,
}

/// Exact Stage-C plan identity disclosed before the first integral stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralPlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact Stage-C stage-plan digest.
    pub plan_digest: u64,
    /// Firmware-authoritative fixed proportional gain.
    pub fixed_p: u16,
    /// Number of positive-I rungs between the two I=0 anchors.
    pub positive_rung_count: u8,
    /// Conservative complete Stage-C duration.
    pub maximum_workflow_ms: u32,
}

/// Compact gate statistics retained at the Stage-C event boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StationarityRejectionDiagnostic {
    /// Number of raw observations in the rejected suffix.
    pub suffix_len: u16,
    /// Whether a usable blocking plateau was available.
    pub usable_plateau: bool,
    /// Whether the slope confidence interval contained zero.
    pub slope_passed: bool,
    /// Whether the lag-one residual confidence interval contained its null.
    pub lag_one_passed: bool,
    /// Outward-rounded settled-rate confidence interval.
    pub settled_rate_interval_q: [i64; 2],
    /// Positive clearance or negative rejection beyond the slope interval.
    pub slope_rejection_margin_q: i64,
    /// Positive clearance or negative rejection beyond the lag-one interval.
    pub lag_one_rejection_margin_q: i32,
    /// Fitted rate slope in fixed-point rate units per second.
    pub slope_q: i64,
    /// Outward-rounded family-wise slope half-width in the same units.
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

/// One analyzed Stage-C moving observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralObservation {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact Stage-C plan digest.
    pub plan_digest: u64,
    /// Zero-based rung index including both anchors.
    pub rung_index: u8,
    /// Fixed F/R observation slot.
    pub slot: u8,
    /// Integral gain programmed for this rung.
    pub i_raw: u16,
    /// Direct encoder-rate-domain observation summary.
    pub summary: crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationSummary,
    /// Target-independent stationarity evidence for this observation.
    pub stationarity: crate::autotune::velocity_measurement::StationarityResult,
    /// Gate statistics that rejected an excluded tail.
    ///
    /// Present only when the analysis reached the slope and lag-one gates.
    /// Blocking-stage exclusions carry `None` because no statistics exist yet.
    pub rejection: Option<StationarityRejectionDiagnostic>,
}

/// Relation between one integral time constant and the fixed moving horizon.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum ResponseHorizonClass {
    /// I=0 has no finite integral time constant.
    NotApplicable = 0,
    /// The integral time constant exceeds the moving horizon.
    SubHorizon = 1,
    /// The moving horizon spans at least one integral time constant.
    HorizonResolved = 2,
}

/// One completed Stage-C rung after both directional consensus folds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralRungReport {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact Stage-C plan digest.
    pub plan_digest: u64,
    /// Zero-based rung index including both anchors.
    pub rung_index: u8,
    /// Integral gain programmed for this rung.
    pub i_raw: u16,
    /// Exact integral time constant in microseconds, or zero for I=0.
    pub tau_us: u64,
    /// Fixed moving-response horizon in microseconds.
    pub moving_horizon_us: u32,
    /// Relation between the time constant and moving horizon.
    pub horizon_class: ResponseHorizonClass,
    /// Forward and reverse exact consensus evidence.
    pub directions:
        [crate::commissioning::outer::velocity::integral_evidence::IntegralRungEvidence; 2],
    /// Recorded breakaway target terminus after this rung, one-based; zero when
    /// none is recorded. Diagnostic: exposes the recorder's per-rung state to the
    /// sweep trace so a missing-terminus resume can be localized to the rung.
    pub target_terminus_plus_one: u8,
}

/// Nonterminal Stage-C evidence produced by one cooperative poll.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityIntegralEvent {
    /// Exact Stage-C plan became authoritative before motion.
    Planned(VelocityIntegralPlanned),
    /// One moving observation was analyzed.
    Observation(VelocityIntegralObservation),
    /// One complete anchor, positive, or bookend rung was folded.
    Rung(VelocityIntegralRungReport),
    /// One bounded return to the immutable campaign origin.
    Recovery(RungOriginRecoveryEvidence),
    /// Recovery stabilization lead-in and scored-rest evidence.
    RecoveryRest(RecoveryRestDiagnostic),
    /// Exact per-suffix scored-rest selector evidence.
    RestSelection(RestSelectionDiagnostic),
    /// Current-limit evidence for one completed signed-plus-zero primitive.
    Current(VelocityPrimitiveCurrentEvidence),
}

/// Report-only Stage-C terminal class.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityIntegralOutcome {
    /// One sufficient run awaits a separately commanded repeat.
    FirstRunRetained = 0,
    /// A second run reproduced at least three response points per direction.
    RepeatabilityConfirmed = 1,
    /// Measurement was safe but evidence was insufficient.
    Inconclusive = 2,
    /// Safety or evidence integrity failed.
    Fault = 3,
    /// A changed exact plan was rejected before motion.
    RejectedPlanMismatch = 4,
    /// No reproduced operating rung can run the Stage-C acquisition.
    Failed = 5,
}

/// Firmware-owned terminal outcome for one fixed fixed-gain amplitude validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FixedGainAmplitudeOutcome {
    /// The declared schedule closed with at least three eligible amplitudes per direction.
    Complete = 0,
    /// Motion stayed safe but the directional floor was not met.
    Inconclusive = 1,
    /// A firmware safety or evidence-integrity invariant failed.
    Fault = 2,
    /// Admission rejected before motion.
    Failed = 3,
}

/// Amplitude-local terminal cause; never mirrored into generic commissioning status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FixedGainAmplitudeCause {
    /// No failure or insufficiency.
    None = 0,
    /// Fewer than three shared amplitudes remained in at least one direction.
    InsufficientSharedFloor = 1,
    /// A required return to the immutable origin could not be proved.
    RecoveryUnavailable = 2,
    /// No RAM-only acceptance point exists.
    MissingAcceptancePoint = 3,
    /// The retained point does not bind the reconstructed plan.
    AcceptancePointIdentityMismatch = 4,
    /// The requested order contradicts retained amplitude progress.
    AmplitudeOrderMismatch = 5,
    /// Fixed evidence storage was exhausted.
    EvidenceCapacity = 6,
    /// Canonical counts, masks, or identities violated their contract.
    EvidenceIntegrity = 7,
    /// The active-terminal matrix-progress transition (advance or close) was
    /// refused. Code 8.
    ///
    /// Not reachable via `apply_fixed_gain_amplitude_terminal`, its only
    /// production caller as of this writing: the preceding identity/progress
    /// check already guarantees the precondition
    /// `advance_matrix_progress`/`close_matrix_progress` require -- allocated
    /// because the underlying check is a real code path, not because current
    /// production input can reach it.
    ProgressTransitionRejected = 8,
    /// The disarm workflow did not complete during a staged terminal
    /// restore. Code 9.
    DisarmIncomplete = 9,
    /// The engine's amplitude order or plan digest was unavailable when an
    /// active terminal needed it. Code 10.
    MissingAmplitudeContext = 10,
    /// Operator-requested cancel reached GPIO-first safe-state gracefully
    /// during a torn-down restore. Code 11.
    Cancelled = 11,
    /// Hardware shutdown was requested (M112 / e-stop) during a torn-down
    /// restore. Code 12.
    Shutdown = 12,
    /// A completed run's attempted/eligible/current-terminus/unattempted
    /// masks failed bit-range validation. Code 13.
    InvalidRunMasks = 13,
    /// Rest analysis completed but did not confirm a stationary zero-target window.
    VelocityRestNotConfirmed = 53,
}

/// One completed amplitude report with canonical amplitude sequencing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FixedGainAmplitudeEvidence {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the amplitude run.
    pub evidence_sequence: u16,
    /// Exact immutable amplitude-plan digest.
    pub plan_digest: u64,
    /// Exact two-direction amplitude fold.
    pub report:
        crate::commissioning::outer::velocity::fixed_gain_amplitude_evidence::FixedGainAmplitudeReport,
}

/// Trace-only evidence produced by one fixed-I confidence amplitude.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FixedGainAmplitudeEvent {
    /// One intrinsic moving observation.
    Observation(VelocityIntegralObservation),
    /// Current and zero-target rest evidence for one primitive.
    Current(VelocityPrimitiveCurrentEvidence),
    /// One evidence-excluded return to the immutable origin.
    Recovery(RungOriginRecoveryEvidence),
    /// Recovery stabilization lead-in and scored-rest evidence.
    RecoveryRest(RecoveryRestDiagnostic),
    /// Exact per-suffix scored-rest selector evidence.
    RestSelection(RestSelectionDiagnostic),
    /// One complete four-observation fold per direction.
    Amplitude(FixedGainAmplitudeEvidence),
}

/// Compact terminal for one fixed-gain-amplitude acquisition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FixedGainAmplitudeTerminal {
    /// Independent amplitude evidence schema.
    pub schema_revision: u16,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact retained combined-plan identity.
    pub acceptance_plan_digest: u64,
    /// Exact amplitude-plan digest.
    pub plan_digest: u64,
    /// Rolling canonical amplitude evidence digest.
    pub digest: u64,
    /// Amplitude-local terminal class.
    pub outcome: FixedGainAmplitudeOutcome,
    /// Amplitude-local terminal cause.
    pub cause: FixedGainAmplitudeCause,
    /// Per-direction amplitudes whose schedules began.
    pub attempted_masks: [u8; 2],
    /// Per-direction amplitudes with unique depth-three consensus.
    pub eligible_masks: [u8; 2],
    /// Per-direction amplitudes ending on current headroom.
    pub current_terminus_masks: [u8; 2],
    /// Per-direction amplitudes not reached by the declared order.
    pub unattempted_masks: [u8; 2],
    /// Number of observation records required by the plan.
    pub expected_observations: u16,
    /// Number of observation records emitted.
    pub emitted_observations: u16,
    /// Number of amplitude records required by the plan.
    pub expected_amplitudes: u8,
    /// Number of amplitude records emitted.
    pub emitted_amplitudes: u8,
    /// Packed inherited validity and evidence-quality qualifiers.
    pub qualifier_bits: u16,
    /// Monotonic timestamp before the first amplitude motion.
    pub run_started_us: u64,
    /// Monotonic timestamp after the final amplitude action.
    pub run_completed_us: u64,
    /// Exact safety-fault context, when present.
    pub safety_fault: Option<OuterSafetyFault>,
}

/// Firmware-owned terminal outcome for one reversal-robustness gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RobustnessOutcome {
    /// Both reversal directions converged within every threshold budget.
    Complete = 0,
    /// Motion stayed safe but at least one threshold gate tripped.
    Rejected = 1,
    /// Motion stayed safe but evidence was insufficient to score the gate.
    Inconclusive = 2,
    /// A firmware safety or evidence-integrity invariant failed.
    Failed = 3,
}

/// Compact terminal for one reversal-standstill robustness gate.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct RobustnessReversalTerminal {
    /// Independent robustness evidence schema.
    ///
    /// The wire encoding packs this as a single `u8`; the struct field
    /// stays `u16` so schema comparisons elsewhere (the detailed reply,
    /// host tooling) do not need a second, narrower type.
    pub schema_revision: u16,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Gate-local terminal class.
    pub outcome: RobustnessOutcome,
    /// `ROBUSTNESS_CAUSE_*` gate-local terminal cause.
    pub cause: u8,
    /// Selected velocity-loop proportional gain used for the probe.
    pub selected_p: u16,
    /// Selected velocity-loop integral gain used for the probe.
    pub selected_i: u16,
    /// Signed target velocity used for the probe, in RPM.
    pub target_velocity_rpm: i32,
    /// The plant-adaptive rate this run's gate thresholds were constructed
    /// from (see `reduce_plant_rate_q`), in rate-Q units. Diagnostic-only:
    /// not consumed by gate comparison logic, carried so an on-target
    /// calibration pass can read back what the gate actually used.
    pub plant_rate_q: i64,
    /// The constructed IAE gate threshold for this run (see
    /// `robustness_thresholds_for`), in rate-Q microseconds. Diagnostic-only,
    /// same rationale as `plant_rate_q`.
    pub iae_max_q: i64,
    /// Both reversal directions' measured robustness evidence.
    pub evidence: crate::commissioning::outer::velocity::robustness_evidence::RobustnessEvidence,
    /// Monotonic timestamp before the first probe motion.
    pub run_started_us: u64,
    /// Monotonic timestamp after the final probe action.
    pub run_completed_us: u64,
    /// Exact safety-fault context, when present.
    pub safety_fault: Option<OuterSafetyFault>,
}

/// Namespaced Stage-C terminal cause: engine (`INTEGRAL_CAUSE_*`), error
/// (`CommissionError::status_code()`), or dispatch (`INTEGRAL_DISPATCH_CAUSE_*`).
/// The three source enumerations are independently numbered and can share a
/// raw value, so the namespace must travel with it.
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
    /// Wire discriminator identifying which namespace `raw_value` belongs
    /// to: 0 = engine, 1 = error, 2 = dispatch.
    pub(crate) const fn namespace_code(self) -> u8 {
        match self {
            Self::Engine(_) => 0,
            Self::Error(_) => 1,
            Self::Dispatch(_) => 2,
        }
    }

    /// The raw cause value within its namespace.
    pub(crate) const fn raw_value(self) -> u8 {
        match self {
            Self::Engine(value) | Self::Error(value) | Self::Dispatch(value) => value,
        }
    }
}

/// Detailed Stage-C terminal retained for lifecycle and canonical reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityIntegralTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact Stage-C plan digest.
    pub plan_digest: u64,
    /// Rolling FNV-1a digest over the plan marker and all nonterminal evidence.
    pub digest: u64,
    /// Terminal measurement-stage class.
    pub outcome: VelocityIntegralOutcome,
    /// Dedicated Stage-C phase-local cause, namespaced (see [`IntegralOutcomeCause`]).
    pub cause: IntegralOutcomeCause,
    /// Whether this terminal came from a breakaway-campaign acceptance run.
    ///
    /// Terminus-independent: a breakaway `RepeatabilityConfirmed` that finished on
    /// sufficiency without reaching target is still a breakaway acceptance
    /// terminal. Firmware-internal only; not on the host wire. Lets
    /// `terminal_target_terminus`/`terminal_target_status` distinguish
    /// "breakaway with no terminus recorded" from "not an acceptance
    /// terminal at all".
    pub breakaway_acceptance: bool,
    /// Fixed Stage-C proportional gain carried on the terminal.
    ///
    /// Carried verbatim from the plan for every Stage-C workflow. Not zeroed
    /// for a non-target-recording (e.g. reproduced Proportional) terminal.
    pub fixed_p: u16,
    /// Final bookend recovery could not establish the next-command origin.
    pub recovery_unavailable: bool,
    /// Final bookend recovery proved passive in-band rest after clamp contact.
    pub recovered_with_current_headroom: bool,
    /// The immutable plan permits one meter quantum to contact the probe clamp.
    pub recovery_quantization_exposed: bool,
    /// The commissioning probe clamp narrowed or removed the Stage-C test point.
    pub probe_constrained_test_point: bool,
    /// One-based positive-rung index of an active-I rest boundary, or zero.
    pub rest_boundary_rung_plus_one: u8,
    /// One-based observation slot of an active-I rest boundary, or zero.
    pub rest_boundary_slot_plus_one: u8,
    /// Exact raw anchor and positive-rung evidence.
    pub evidence: crate::commissioning::outer::velocity::integral_evidence::IntegralRunEvidence,
    /// Number of observations required by the immutable plan.
    pub expected_observations: u16,
    /// Number of observation records emitted.
    pub emitted_observations: u16,
    /// Number of anchor/positive/bookend rungs required by the plan.
    pub expected_rungs: u16,
    /// Number of rung records emitted.
    pub emitted_rungs: u16,
    /// Monotonic timestamp captured before Stage-C motion.
    pub run_started_us: u64,
    /// Monotonic timestamp captured at terminal construction.
    pub run_completed_us: u64,
    /// Safety evidence retained for ordered fault emission.
    pub safety_fault: Option<OuterSafetyFault>,
    /// Whether `evidence.assess(..)` was already `Sufficient` when a rest
    /// check rejected the run. Distinguishes a partial acquisition from
    /// evidence that was discarded after it was already good.
    pub rest_rejection_after_sufficiency: bool,
    /// The Stage-C owner responsible, only when `rest_rejection_after_sufficiency`.
    /// `None` whenever that flag is `false`, even if the run's rest check did
    /// fail for a different (case-(a)) reason.
    pub rest_rejection_owner: Option<RestSelectionOwner>,
}

// ============================================================================
// Breakaway-seeded campaign reporting
//
// These records carry the wire evidence for the breakaway probe, additive
// discovery, and held-out confirmation phases. Every plan-time record's
// `plan_digest` is the exact digest of the frozen phase plan it describes;
// every phase after the first also carries `prior_plan_digest`, the digest of
// the phase plan it was derived from, so the causal chain probe -> discovery
// -> confirmation is auditable end to end. Discovery and confirmation carry
// their predeclared confidence families; the physical-excursion probe does not.
// See
// `crate::commissioning::outer::velocity::breakaway_reporting` for the pure
// assembly functions that build these records from the campaign's own
// produced evidence types.
// ============================================================================

/// Immutable breakaway-probe plan evidence emitted before the first probe stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbePlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the frozen probe plan.
    pub plan_digest: u64,
    /// Maximum observations permitted across the complete upward walk.
    pub max_observations: u8,
    /// Number of valid entries in the post-dedupe search grid.
    pub search_count: u8,
    /// First structural search-grid gain.
    pub p_start_raw: u16,
    /// Last structural search-grid gain.
    pub p_top_raw: u16,
    /// Command-level workflow envelope digest bound into `plan_digest`.
    pub workflow_digest: u64,
    /// Frozen signed-target magnitude in mechanical RPM register units.
    pub target_velocity_rpm: i32,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
    /// PWM carrier frequency the probe was planned under, in Hz.
    pub pwm_hz: u32,
    /// Encoder counts per mechanical revolution used by the probe.
    pub encoder_counts_per_rev: u32,
    /// Minimum commanded-direction excursion that counts as motion.
    pub motion_threshold_counts: u32,
    /// Maximum accepted interval between consecutive encoder samples.
    pub max_capture_interval_us: u32,
}

/// Resolved repeatable-motion rung for the complete upward probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbeResolved {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe plan.
    pub plan_digest: u64,
    /// Zero-based rung in the realized probe grid.
    pub rung_index: u8,
    /// Proportional gain at the resolved rung.
    pub breakaway_p_raw: u16,
    /// Minimum commanded-direction excursion that counted as motion.
    pub motion_threshold_counts: u32,
    /// Total observations consumed across the upward walk.
    pub observation_count: u8,
}

/// Detailed terminal evidence for a probe campaign that did not resolve a
/// breakaway (every [`crate::commissioning::outer::velocity::breakaway_campaign::BreakawayProbeOutcome`]
/// variant other than `BreakawayFound`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbeTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact digest of the frozen probe plan.
    pub plan_digest: u64,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
    /// Active breakaway operation checkpoint when a safety fault was raised, or zero.
    pub fault_checkpoint: u8,
}

/// Immutable additive-discovery plan evidence emitted before the first
/// discovery stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdditiveDiscoveryPlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the frozen discovery plan.
    pub plan_digest: u64,
    /// Exact digest of the probe plan whose search located this ladder's breakaway.
    pub prior_plan_digest: u64,
    /// Breakaway-campaign schema/rule revision under which this ladder was
    /// resolved (`combined_plan::BREAKAWAY_PROPORTIONAL_SCHEMA_REVISION`). Carried in
    /// the plan summary so an offline decoder can regenerate the exact ladder
    /// under the same rules the firmware applied and cross-check its recomputed
    /// digest against `plan_digest`.
    pub schema_revision: u16,
    /// Discovery confidence family: `rung_count * observations_per_direction`.
    pub family_size: u16,
    /// Number of valid entries in the resolved additive ladder.
    pub rung_count: u8,
    /// Discovery per-rung, per-direction observation count.
    pub observations_per_direction: u16,
    /// Chosen diagnostic floor gain.
    pub floor_p_raw: u16,
    /// Origin of the diagnostic floor: `0` predecessor grid rung, `1` clamped at breakaway.
    pub floor_origin: u8,
    /// Resolved campaign breakaway gain.
    pub breakaway_p_raw: u16,
    /// Most restrictive applicable authority ceiling; the ladder's exact final anchor.
    pub ceiling_p_raw: u16,
    /// Fixed additive step from breakaway to the first rung strictly above it.
    pub first_additive_step_raw: u32,
    /// Reserved maximum campaign workflow duration.
    ///
    /// The campaign's own retained request only carries the reserved maximum
    /// (`BreakawayCampaignInputs::workflow_envelope_ms`); it never keeps a
    /// nominal figure downstream of the command-level
    /// `CommissioningWorkflowPlan` disclosure, so this record does not carry
    /// one either -- matching `VelocityIntegralPlanned::maximum_workflow_ms`,
    /// the existing combined-workflow wire field that also carries maximum
    /// only.
    pub maximum_workflow_ms: u32,
}

/// One collected observation's raw settled-rate evidence, streamed on the tick
/// its stroke completes -- before the atom is folded into a discovery rung's
/// nomination bounds or judged for confirmation containment.
///
/// This is the independent-audit primitive: the firmware still reduces these
/// atoms to the `_percent_milli` bounds it reports in
/// [`BreakawayDiscoveryRungMargin`] and [`BandConfirmationObservation`], but
/// those reduced bounds can only be trusted, not verified. Streaming the raw
/// `mean_rate_q`/`variance_of_mean_q2`/`target_rate_q`/`direction` inputs lets
/// an offline decoder recompute `r`, `SE`, the nomination interval, and the
/// [70, 80] containment itself and flag any disagreement with the firmware's
/// reported values. The raw atoms are transient (built, enqueued, dropped);
/// nothing here is retained, so this adds no RAM.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRawObservation {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe, discovery, or confirmation plan.
    pub plan_digest: u64,
    /// Owning phase: probe (0), discovery (1), or confirmation (2).
    pub phase: u8,
    /// Zero-based probe or discovery rung; `0` for confirmation.
    pub rung_index: u8,
    /// Probe direction-local slot (`0` initial, `1` repeat), discovery
    /// within-direction slot, or confirmation's fixed `0..=7` schedule slot.
    pub slot_index: u8,
    /// Direction wire code (see `sweep_evidence::direction_report_code`).
    pub direction: u8,
    /// Raw signed settled-rate mean in fixed-point rate units.
    pub mean_rate_q: i64,
    /// Family-free variance of the mean, in squared fixed-point rate units.
    pub variance_of_mean_q2: u128,
    /// Positive commanded-magnitude target rate shared by both directions.
    pub target_rate_q: i64,
    /// Frozen stationarity outcome: `0` usable, otherwise the exclusion reason
    /// code. Lets an offline decoder reproduce the `StationarityExcluded` class.
    pub stationarity_code: u8,
    /// Inclusive lower settled-rate confidence edge (classify input).
    pub settled_rate_low_q: i64,
    /// Inclusive upper settled-rate confidence edge (classify input).
    pub settled_rate_high_q: i64,
    /// Inclusive lower signed-deficit confidence edge (classify input).
    pub deficit_low_q: i64,
    /// Inclusive upper signed-deficit confidence edge (classify input).
    pub deficit_high_q: i64,
    /// Predicted absolute velocity-controller torque target (classify input).
    pub predicted_torque_target_abs: u16,
    /// Inclusive current-headroom terminus (classify input).
    pub usable_torque_limit: u16,
    /// Hard controller-output clip boundary (classify input).
    pub hard_torque_limit: u16,
    /// Whether a representable controller-domain disturbance interval exists.
    pub disturbance_present: u8,
    /// Lower disturbance-interval bound in equivalent torque-target Q16.16 counts
    /// (`0` when absent).
    pub disturbance_low_q16: i32,
    /// Upper disturbance-interval bound in the same units (`0` when absent).
    pub disturbance_high_q16: i32,
    /// Drift ratio in parts per million, or `u32::MAX` when unavailable.
    pub drift_ratio_ppm: u32,
    /// Post-fit lag-one state: 0 unavailable, 1 failed, 2 passed.
    pub post_fit_lag_one_state: u8,
    /// Minimum start-relative displacement observed during a probe stroke.
    pub stroke_min_displacement_counts: i32,
    /// Maximum start-relative displacement observed during a probe stroke.
    pub stroke_max_displacement_counts: i32,
    /// Maximum interval between consecutive probe encoder captures.
    pub capture_max_interval_us: u32,
    /// Measured peak velocity-controller output observed during this stroke's
    /// moving window, in torque-target counts.
    ///
    /// Streamed separately from `predicted_torque_target_abs` because that
    /// field is `max(P * rpm / 256, moving_pid_output_peak)`
    /// (`sweep_analysis.rs::predicted_torque_target_abs`), so a headroom
    /// terminus reported through it cannot be attributed to the open-loop model
    /// term or to measured demand without this value. Carrying both makes the
    /// `max` decomposable offline, which is needed to choose a
    /// commissioning clamp from evidence rather than from a fixed fraction.
    pub moving_pid_output_peak: u16,
}

/// One recovered probe observation committed to the adaptive cursor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayProbeDecision {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe plan.
    pub plan_digest: u64,
    /// Direction wire code.
    pub direction: u8,
    /// Zero-based index in the deduplicated probe grid.
    pub rung_index: u8,
    /// Decision: 0 no motion, 1 moved.
    pub disposition: u8,
}

/// One in-band discovery rung's nomination bounds and pointwise margin `M_i`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayDiscoveryRungMargin {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the owning discovery plan.
    pub plan_digest: u64,
    /// Zero-based index within the collected in-band rung set (collection order).
    pub collected_index: u8,
    /// Proportional gain at this rung.
    pub p_raw: u16,
    /// Enveloped nomination-interval lower bound `L_i`, in percent * 1000.
    pub lower_percent_milli: i32,
    /// Enveloped nomination-interval upper bound `U_i`, in percent * 1000.
    pub upper_percent_milli: i32,
    /// `M_i = min(L_i - 70, 80 - U_i)`, in percent * 1000.
    pub margin_percent_milli: i32,
    /// `(U_i - L_i) / 2`, in percent * 1000.
    pub half_width_percent_milli: i32,
    /// Achieved rate step from the previous collected in-band rung's own
    /// midpoint response, in percent * 1000; zero for the first collected rung.
    pub adjacent_rate_step_percent_milli: i32,
}

/// Diagnostic evidence for whether the additive ladder's rung zero (the
/// diagnostic floor) itself showed motion. Reported purely as evidence: rung
/// zero moving does not fail the discovery run (spec section 6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRungZeroDiagnostic {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the owning discovery plan.
    pub plan_digest: u64,
    /// Nonzero when the forward-direction rung-zero observation showed motion.
    pub forward_moved: u8,
    /// Nonzero when the reverse-direction rung-zero observation showed motion.
    pub reverse_moved: u8,
}

/// Detailed terminal evidence for an additive discovery campaign.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdditiveDiscoveryTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact digest of the frozen discovery plan.
    pub plan_digest: u64,
    /// Exact digest of the probe plan whose search located this ladder's breakaway.
    pub prior_plan_digest: u64,
    /// Discovery confidence family: `rung_count * observations_per_direction`.
    pub family_size: u16,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    /// Number of in-band rungs collected before the ladder stopped.
    pub collected_count: u8,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
    /// Rung index the discovery walk had reached when it stopped, whether by
    /// nominating, exhausting the ladder, or a discovery-owned fault.
    pub stop_rung_index: u8,
    /// `CommissionError::status_code()` for a faulted terminal, or `0` when the
    /// terminal carries no error.
    ///
    /// The terminal cause says which phase stopped and whether a safety fault
    /// was retained; it does not say why. Without this a `*_internal_fault`
    /// is unattributable, which is exactly the state the 2026-08-03
    /// intermittent left the campaign in.
    pub error_code: u8,
}

/// Immutable held-out confirmation plan evidence emitted before the first
/// confirmation stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationPlanned {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the frozen confirmation plan.
    pub plan_digest: u64,
    /// Exact digest of the discovery plan whose nomination screen selected the candidate.
    pub prior_plan_digest: u64,
    /// Predeclared confirmation family size.
    pub family_size: u16,
    /// Fresh observations collected per direction.
    pub observations_per_direction: u16,
    /// Nominated proportional gain under held-out test.
    pub nominated_p_raw: u16,
    /// Inclusive lower edge of the confirmable response band, in percent.
    pub band_lower_percent: u8,
    /// Inclusive upper edge of the confirmable response band, in percent.
    pub band_upper_percent: u8,
    /// Stable wire code for the confirmation capture profile.
    pub capture_profile: u8,
    /// Stable wire code for the acceptance rule.
    pub acceptance_rule: u8,
    /// Nomination screen's pointwise margin `M_i` for the nominated candidate,
    /// in percent * 1000.
    pub nominated_margin_percent_milli: i32,
}

/// One fresh held-out confirmation stroke's response interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationObservation {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
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
    /// Measured relative standard error for this stroke, in permille.
    pub relative_standard_error_permille: u16,
}

/// Detailed terminal evidence for a held-out confirmation block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact digest of the frozen confirmation plan.
    pub plan_digest: u64,
    /// Exact digest of the discovery plan whose nomination screen selected the candidate.
    pub prior_plan_digest: u64,
    /// Predeclared confirmation family size.
    pub family_size: u16,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    /// Forward-direction collected mask (`collected_mask == eligible_mask ==
    /// included_mask == 0b1111` required for acceptance).
    pub forward_collected_mask: u8,
    /// Forward-direction eligible mask.
    pub forward_eligible_mask: u8,
    /// Forward-direction included mask.
    pub forward_included_mask: u8,
    /// Reverse-direction collected mask.
    pub reverse_collected_mask: u8,
    /// Reverse-direction eligible mask.
    pub reverse_eligible_mask: u8,
    /// Reverse-direction included mask.
    pub reverse_included_mask: u8,
    /// Nonzero when the held-out block was accepted.
    pub accepted: u8,
    /// Confirmed proportional gain, or zero when not accepted.
    pub confirmed_p_raw: u16,
    /// Largest measured relative standard error across all eight strokes, in permille.
    pub max_relative_se_permille: u16,
    /// Relative standard error above which containment is unachievable, in permille.
    pub required_relative_se_permille: u16,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
}

/// Canonical closure record for one complete breakaway campaign.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayCampaignTerminal {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Phase that owned the campaign's terminal outcome.
    pub phase: u8,
    /// Unified `BreakawayTerminalCause` wire code.
    pub terminal_cause: u8,
    /// Nonzero when the confirmed candidate was accepted and admitted to Integral.
    pub accepted: u8,
    /// Exact digest of the Stage-C plan bound through the confirmed candidate,
    /// or zero when `accepted == 0`.
    pub integral_plan_digest: u64,
    /// `CommissionError::status_code()` for a campaign terminal that carries a
    /// causally-relevant error beyond `terminal_cause` alone, or `0` when the
    /// terminal carries no error.
    ///
    /// `terminal_cause` says which phase closed and, broadly, why; it does not
    /// carry a specific `CommissionError` identity. Without this, a confirmed
    /// acceptance whose Stage-C plan build was refused is unattributable.
    pub error_code: u8,
}

/// Which `p_top` candidate bound the additive-discovery authority ceiling:
/// the current-limit-derived candidate, or the `PidPGain::MAX`
/// representability clamp. See `sweep_plan::CEILING_BINDING_*`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayCeilingSource {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Evidence sequence shared with the discovery plan this ceiling anchors.
    pub evidence_sequence: u16,
    /// Digest of the additive discovery plan this ceiling bound.
    pub plan_digest: u64,
    /// `sweep_plan::CEILING_BINDING_CURRENT_LIMIT` or
    /// `sweep_plan::CEILING_BINDING_REPRESENTABILITY_CLAMP`.
    pub binding_source: u8,
}

/// One rejected recovery-rest scoring window.
///
/// Emitted only when the scored-rest selector returned
/// [`crate::autotune::velocity_measurement::RestSelectionDisposition::Rejected`].
/// Accepted recoveries emit nothing: a campaign passes roughly 160 recovery
/// rest gates, so streaming every one would bury the single rejection that
/// ends the campaign under two orders of magnitude of noise.
///
/// A rejection needs both acceptance routes to fail at once, and the two are
/// evidenced separately. The selector's per-suffix verdicts explain the
/// statistical route; `scored_min_displacement_counts` and
/// `scored_max_displacement_counts` explain the one-count fallback, which is
/// only offered when their span is at most one count. Pairing them in one
/// record lets an offline decoder tell a two-count displacement span, a
/// structural rejection, and a resolved non-zero interval apart -- none of
/// which is otherwise observable, because the settle displacement never
/// reaches the wire and `RecoveryRestDiagnostic` is only produced under the
/// stabilized rest schedule that the breakaway campaign never uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRecoveryRestRejected {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe, discovery, or confirmation plan.
    pub plan_digest: u64,
    /// Owning phase: probe (0), discovery (1), or confirmation (2).
    pub phase: u8,
    /// Recovery stage that scored the window.
    pub stage: u8,
    /// Zero-based probe or discovery rung; `0` for confirmation.
    pub rung_index: u8,
    /// Phase-local slot of the stroke whose recovery was scored.
    pub slot_index: u8,
    /// Direction of the stroke whose recovery was scored.
    pub direction: u8,
    /// Raw velocity P gain in force for the stroke.
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
    /// Minimum settle displacement over the scored window.
    pub scored_min_displacement_counts: i8,
    /// Maximum settle displacement over the scored window.
    pub scored_max_displacement_counts: i8,
}

/// One recovery attempt refused because the origin moved under it.
///
/// Rung-origin recovery captures `start_offset_counts` on the tick it is
/// constructed and re-reads the position on the following tick, faulting on
/// exact inequality. `unwrapped_position` advances on every outer-safety
/// encoder sample, so a single sample landing between those two ticks is
/// enough. The fault itself carries no measurement, which leaves a
/// one-count race and a genuine runaway indistinguishable.
///
/// This record supplies the missing magnitude and direction: `delta_counts` is
/// the exact discrepancy, and `position_window_counts` gives the containment
/// bound it should be judged against.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawayRecoveryOriginMismatch {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the run.
    pub evidence_sequence: u16,
    /// Exact digest of the owning probe, discovery, or confirmation plan.
    pub plan_digest: u64,
    /// Owning phase: probe (0), discovery (1), or confirmation (2).
    pub phase: u8,
    /// Recovery stage that refused to start.
    pub stage: u8,
    /// Zero-based probe or discovery rung; `0` for confirmation.
    pub rung_index: u8,
    /// Phase-local slot of the stroke whose recovery refused to start.
    pub slot_index: u8,
    /// Direction of that stroke.
    pub direction: u8,
    /// Raw velocity P gain in force for the stroke.
    pub p_raw: u16,
    /// Position captured when the recovery runtime was constructed.
    pub expected_start_offset_counts: i32,
    /// Position observed on the tick recovery actually began.
    pub observed_position_counts: i32,
    /// `observed - expected`; the exact discrepancy that raised the fault.
    pub delta_counts: i32,
    /// Outer-safety containment window the offsets are judged against.
    pub position_window_counts: u32,
}

/// One breakaway-campaign evidence record ready for wire emission.
///
/// The campaign driver
/// (`breakaway_campaign::step_breakaway_campaign_with_trace`) returns one of
/// these per streamed tick, and `commission_dispatch::replies` fans it out to
/// the Task-7 `emit_breakaway_*` wire bridges. The discovery and confirmation
/// terminal variants carry their full batch evidence (every in-band rung,
/// every held-out stroke) because that evidence is only ever available in
/// full at the terminal transition -- `breakaway_campaign`'s own
/// `CollectedInBandRungs`/`ConfirmationDirectionalInput` retain it across the
/// whole phase for exactly this reason. The dispatch bridge loops over it to
/// emit one wire record per item, the same way `emit_fixed_gain_amplitude_plan`
/// already fans one Rust value out to several wire fragments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub enum BreakawayCampaignEvent {
    /// Immutable probe plan, emitted before the probe's first stroke.
    ProbePlanned(BreakawayProbePlanned),
    /// The first rung with repeatable motion in both directions.
    ProbeResolved(BreakawayProbeResolved),
    /// A probe-campaign terminal that did not resolve a breakaway.
    ProbeTerminal(BreakawayProbeTerminal),
    /// The immutable additive discovery plan and the ceiling's binding source,
    /// emitted before discovery's first stroke. The plan summary
    /// ([`AdditiveDiscoveryPlanned`]) carries the complete ladder geometry
    /// (schema revision, floor, breakaway, ceiling, additive step, rung count,
    /// family, and digest), so a decoder regenerates the exact ladder from it
    /// and cross-checks its recomputed digest against `plan_digest` -- the
    /// per-rung ladder records this used to fan out bought nothing the digest
    /// does not already pin, and their 34-frame burst could overflow the board
    /// trace queue.
    DiscoveryPlanned {
        /// The frozen plan identity and geometry.
        planned: AdditiveDiscoveryPlanned,
        /// Which candidate bound the plan's authority ceiling.
        ceiling_source: BreakawayCeilingSource,
    },
    /// One collected observation's raw settled-rate evidence, streamed on the
    /// tick its stroke completes for independent offline recomputation of the
    /// nomination margins and confirmation containment. Emitted for both the
    /// discovery and confirmation phases (distinguished by the record's
    /// `phase` field), one per tick at collection -- never bundled into a
    /// terminal-time burst.
    RawObservation(BreakawayRawObservation),
    /// One recovered probe observation committed to the adaptive cursor.
    ProbeDecision(BreakawayProbeDecision),
    /// One collected in-band rung's nomination margin, streamed on its own tick
    /// as the discovery walk folds the rung (never bundled into the discovery-
    /// terminal tick, which would burst the trace queue). A non-nominatable
    /// in-band rung produces no such event.
    DiscoveryRungMargin(BreakawayDiscoveryRungMargin),
    /// A discovery-campaign terminal: the rung-zero motion diagnostic (when
    /// available) and the terminal record. Per-rung margins stream separately
    /// as [`Self::DiscoveryRungMargin`] before this event.
    DiscoveryTerminal {
        /// The discovery terminal record.
        terminal: AdditiveDiscoveryTerminal,
        /// The rung-zero motion diagnostic, when discovery progressed far
        /// enough to classify rung zero's own strokes.
        rung_zero: Option<BreakawayRungZeroDiagnostic>,
    },
    /// The immutable held-out confirmation plan, emitted before its first stroke.
    ConfirmationPlanned(BandConfirmationPlanned),
    /// A confirmation-block terminal: every fresh stroke's response interval
    /// and the terminal itself. `forward`/`reverse` carry no observations for
    /// a terminal reached before any confirmation stroke ran.
    ConfirmationTerminal {
        /// The confirmation terminal record.
        terminal: BandConfirmationTerminal,
        /// Forward-direction fresh evidence, fanned out to one observation
        /// record per collected stroke at dispatch.
        forward:
            crate::commissioning::outer::velocity::band_confirmation::ConfirmationDirectionalInput,
        /// Reverse-direction fresh evidence, fanned out the same way.
        reverse:
            crate::commissioning::outer::velocity::band_confirmation::ConfirmationDirectionalInput,
    },
    /// The canonical closure record for one complete breakaway campaign.
    CampaignTerminal(BreakawayCampaignTerminal),
    /// One rejected recovery-rest scoring window, streamed on the tick the
    /// selector rejected it. Accepted rest windows produce no event.
    RecoveryRestRejected(BreakawayRecoveryRestRejected),
    /// One recovery refused because the origin moved between the tick that
    /// captured it and the tick recovery began. Streamed immediately before
    /// the fault it explains.
    RecoveryOriginMismatch(BreakawayRecoveryOriginMismatch),
}

/// Generic terminal tying an inconclusive outer phase to its detailed report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OuterInconclusive {
    /// Phase that could not produce sufficient evidence.
    pub phase: CommissionPhase,
    /// Bit zero is forward sufficiency and bit one is reverse sufficiency.
    pub sufficient_direction_mask: u8,
    /// Phase-local inconclusive cause.
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
    /// Engine remains runnable without an artificial delay.
    ReadyNow,
    /// Engine remains runnable at the strictly future absolute timestamp.
    At(u64),
    /// A phase transition occurred. Report to host.
    PhaseChanged(CommissionPhase),
    /// Streamed Stage-C velocity-integral evidence became available.
    VelocityIntegralEvent(VelocityIntegralEvent),
    /// The report-only Stage-C response sweep reached a terminal result.
    VelocityIntegralTerminal(VelocityIntegralTerminal),
    /// Trace-only fixed-I velocity confidence evidence became available.
    FixedGainAmplitudeEvent(FixedGainAmplitudeEvent),
    /// One fixed-I fixed-gain amplitude validation reached its terminal.
    FixedGainAmplitudeTerminal(FixedGainAmplitudeTerminal),
    /// One reversal-standstill robustness gate reached its terminal.
    RobustnessReversalTerminal(RobustnessReversalTerminal),
    /// Streamed breakaway-campaign evidence became available.
    BreakawayCampaignEvent(BreakawayCampaignEvent),
    /// An accepted breakaway campaign handed off to Integral, carrying the exact
    /// plan that hand-off installed.
    ///
    /// The plan must be reported, not merely installed. This event carries the
    /// exact Stage-C plan alongside the hand-off transition so the host sees it,
    /// and the host assembler refuses every Stage-C observation until it has seen
    /// a complete plan. A hand-off that transitions silently strands the host
    /// with no plan and fails the run on its first observation.
    BreakawayIntegralPlanReady {
        /// Campaign run sequence the Stage-C evidence chains to.
        run_sequence: u32,
        /// Exact Stage-C plan installed by the hand-off.
        plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
    },
    /// Inner commissioning failed.
    InnerFailed(CommissionError),
    /// Outer commissioning completed successfully.
    OuterDone(OuterResult),
    /// Outer commissioning failed.
    OuterFailed(CommissionError),
    /// ADC calibration completed. Hardware remains active for encoder
    /// alignment.
    CalibrateDone(AdcCalResult),
    /// Selftest phases completed. Hardware restored to safe state.
    SelftestDone,
    /// Stage 1 commissioning complete. Motor is in closed-loop position mode.
    /// Carries inner result + fallback outer gains for the Python host to
    /// cache and persist.
    CommissionDone(CommissionResult),
}

/// No Stage 2 safety-envelope fault was reported.
pub const OUTER_SAFETY_FAULT_NONE: u8 = 0;
/// Host budget was structurally invalid before Stage 2 actuation.
pub const OUTER_SAFETY_FAULT_INVALID_BUDGET: u8 = 1;
/// Host budget was too small to collect useful Stage 2 evidence.
pub const OUTER_SAFETY_FAULT_UNUSABLE_BUDGET: u8 = 2;
/// Stage 2 primitive exceeded its host-provided duration cap.
pub const OUTER_SAFETY_FAULT_DURATION: u8 = 3;
/// Stage 2 encoder velocity exceeded its active safety cap.
pub const OUTER_SAFETY_FAULT_VELOCITY: u8 = 4;
/// Stage 2 accumulated encoder position exceeded its active travel window.
pub const OUTER_SAFETY_FAULT_POSITION: u8 = 5;
/// Mechanical-ID position-to-torque handoff did not settle before timeout.
pub const OUTER_SAFETY_FAULT_POST_SWITCH_SETTLE: u8 = 6;
/// Encoder observations were separated by too much time for wrap-safe interpretation.
pub const OUTER_SAFETY_FAULT_OBSERVATION_GAP: u8 = 7;
/// A single encoder delta reached the inclusive quarter-turn observability guard.
pub const OUTER_SAFETY_FAULT_QUARTER_TURN: u8 = 8;
/// Velocity sweep reached the configured hard torque-target clip.
pub const OUTER_SAFETY_FAULT_CURRENT: u8 = 9;
/// Rung-origin recovery increased absolute origin error beyond quantization allowance.
pub const OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY: u8 = 10;
/// Stage 2 primitive-local travel exceeded its per-stroke geometry window.
/// Distinct from [`OUTER_SAFETY_FAULT_POSITION`] (the absolute machine-headroom
/// branch): this is displacement from the primitive start, a per-stroke sizing
/// concern, whereas `POSITION` is absolute travel from Stage-2 entry approaching
/// a physical limit. The payload reports the actually-compared excursion and the
/// signed window it exceeded, so `|position_counts| > |position_window_counts|`.
pub const OUTER_SAFETY_FAULT_POSITION_LOCAL: u8 = 11;

/// Host-visible Stage 2 safety-envelope fault evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OuterSafetyFault {
    /// `OUTER_SAFETY_FAULT_*` reason code.
    pub reason: u8,
    /// Host-provided travel budget in milli-revolutions.
    pub max_travel_mrev: u16,
    /// Host-provided velocity budget in milli-revolutions per second.
    pub max_velocity_mrev_s: u16,
    /// Host-provided primitive duration budget in milliseconds.
    pub max_duration_ms: u16,
    /// Reserved direction mask carried from the host request.
    pub direction_mask: u8,
    /// Signed encoder delta for the sample that tripped the gate.
    pub delta_counts: i32,
    /// Elapsed time for `delta_counts`, in microseconds.
    pub dt_us: u32,
    /// Signed sample velocity in encoder counts per millisecond.
    pub velocity_counts_per_ms: i32,
    /// Active velocity cap in encoder counts per millisecond.
    pub velocity_cap_counts_per_ms: i32,
    /// Signed accumulated position from the Stage 2 start, in encoder counts.
    pub position_counts: i32,
    /// Active signed-position window in encoder counts.
    pub position_window_counts: i32,
    /// Elapsed time from Stage 2 safety arming, in microseconds.
    pub elapsed_us: u32,
    /// Active primitive duration cap in microseconds.
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
/// Electrical-identification timing method slot reserved for the retired
/// delay capture; always reads `CommissionTimingSummary::NOT_RUN`.
pub const COMMISSION_TIMING_METHOD_DELAY: u8 = 2;

/// Compact retained timing result for one electrical-identification method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CommissionTimingSummary {
    /// `COMMISSION_TIMING_*` state.
    pub status: u8,
    /// Whether timing telemetry saturated its retained integer representation.
    pub overflowed: bool,
    /// Saturating capture metrics. Zero-filled when the method did not run.
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
    /// Overall status.
    pub status: CommissionStatus,
    /// Resistance count slope in milli count-ratio units.
    pub r_count_milli: u32,
    /// Count-space inductance proxy in micro-time units.
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
    /// Candidate measured axis split ratio before validation verdict.
    pub candidate_measured_axis_split_permille: u16,
    /// Candidate applied axis split ratio before validation verdict.
    pub candidate_applied_axis_split_permille: u16,
    /// Candidate axis split clamp flag before validation verdict.
    pub candidate_axis_split_clamped: u8,
    /// Candidate flux P gain last written for validation.
    pub candidate_flux_p: u16,
    /// Candidate flux I gain last written for validation.
    pub candidate_flux_i: u16,
    /// Candidate torque P gain last written for validation.
    pub candidate_torque_p: u16,
    /// Candidate torque I gain last written for validation.
    pub candidate_torque_i: u16,
    /// Retry attempt associated with the candidate gains.
    pub candidate_attempt: u8,
    /// Inner-loop time scale in microseconds.
    ///
    /// Physical-P commissioning reports `1 / (2*pi*bandwidth)`. Older SIMC
    /// helper output used the same field for SIMC lambda.
    pub lambda_us: u32,
    /// Electrical transport delay in microseconds.
    pub theta_e_us: u32,
    /// Source of `theta_e_us`: 0 = high-rate measurement, 1 =
    /// board-configured fallback.
    pub theta_source: u8,
    /// Max ringing count from forward/reverse current validation.
    pub ringing_count: u8,
    /// Effective current-loop bandwidth in Hz.
    pub bandwidth_hz: u32,
    /// Electrical time constant L/R in microseconds (Phase 5 measurement).
    pub tau_e_us: u32,
    /// Accepted production inductance evidence.
    pub inductance: InductanceEvidence,
    /// Warning bitfield.
    ///
    /// Bits 0, 1 and 3 report model warnings; bit 2 is unused. Bit 5 reports
    /// fallback current gains, bit 6 reports host-default confidence, and bit 7 is
    /// reserved (older firmware used it for a skipped gain floor).
    pub inner_warning_flags: u8,
    /// Per-method timing summaries in resistance, inductance, delay order.
    pub timing: [CommissionTimingSummary; 3],
    /// Packed terminal timing status derived from [`Self::timing`].
    pub timing_summary: u32,

    // --- Shared multi-point resistance-identification evidence ---
    //
    // Persisted so the host can audit the count-space fit and the 64x
    // count-slope-milli mismatch directly from artifacts. All slope fields
    // are in the same count-slope-milli unit space (dV_count/dI_count * 1000)
    // as the `r_count_milli` resistance scale.
    /// Run-selected resistance count-slope in milli-units.
    pub resistance_selected_count_slope_milli: i32,
    /// Resistance count-slope (milli-units) actually consumed by the gain path.
    /// Same-run comparison counterpart for the 64x count-slope-milli audit.
    pub resistance_gain_path_count_slope_milli: i32,
    /// Axis 0 (PHASE0) fitted count-slope in milli-units.
    pub resistance_axis0_count_slope_milli: i32,
    /// Axis 1 (PHASE90) fitted count-slope in milli-units.
    pub resistance_axis1_count_slope_milli: i32,
    /// Axis 0 fit intercept in current counts.
    pub resistance_axis0_intercept_count: i32,
    /// Axis 1 fit intercept in current counts.
    pub resistance_axis1_intercept_count: i32,
    /// Axis 0 fit RMSE in permille.
    pub resistance_axis0_rmse_permille: u16,
    /// Axis 1 fit RMSE in permille.
    pub resistance_axis1_rmse_permille: u16,
    /// Axis 0 selected-point mask.
    pub resistance_selected_mask_axis0: u16,
    /// Axis 1 selected-point mask.
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
    /// Largest absolute individual current sample observed while energized.
    pub resistance_peak_abs_current_count: u16,
    /// Largest absolute mean from a completed energized acquisition window.
    pub resistance_max_abs_steady_mean_current_count: u16,
    /// Current ceiling applied to individual energized samples.
    pub resistance_current_ceiling_count: u16,
    /// Whether sample-level containment synchronously tripped the power stage.
    pub resistance_power_stage_tripped: bool,

    // --- Encoder alignment evidence ---
    /// Encoder count captured immediately before alignment zeroed the ABN count.
    pub encoder_alignment_count: u32,
    /// Folded residual of the alignment count modulo one electrical revolution.
    pub encoder_alignment_electrical_residual_counts: u16,
    /// Difference between the two post-settle alignment reads.
    pub encoder_alignment_stability_counts: u16,
    /// Largest movement from the pre-alignment count to a post-settle read.
    pub encoder_alignment_movement_counts: u16,
    /// Minimum movement required for the alignment to be accepted.
    pub encoder_alignment_min_movement_counts: u16,
    /// Encoder counts per electrical revolution.
    pub encoder_alignment_counts_per_electrical_rev: u16,
    /// Zero-current ADC residual with PWM chopping disabled.
    pub adc_residual_chop0: AdcResidualEvidence,
    /// Zero-current ADC residual with PWM chopping restored.
    pub adc_residual_chop7: AdcResidualEvidence,

    // --- Current-loop synthesis evidence ---
    /// `CURRENT_GAINS_SOURCE_*`.
    pub current_gains_source: u8,
    /// `AXIS_SPLIT_SOURCE_*`.
    pub axis_split_source: u8,
    /// `CURRENT_GAINS_TIER_*`.
    pub gains_tier: u8,
    /// Measured axis split ratio, permille where 1000 = 1.000x.
    pub measured_axis_split_permille: u16,
    /// Applied axis split ratio after profile clamp, permille.
    pub applied_axis_split_permille: u16,
    /// Non-zero when the applied axis split was clamped below measured.
    pub axis_split_clamped: u8,
    /// Bitmask of `CURRENT_VALIDATION_AXIS_*` axes that passed validation.
    pub current_validation_axes: u8,
    /// Non-zero when current-loop retry budget was exhausted.
    pub retry_budget_exhausted: u8,
    /// `CURRENT_LOOP_FAILURE_*`.
    pub current_loop_failure_reason: u8,
    /// Closed-loop entry fast stability status (`CLOSED_LOOP_ACTIVATION_STATUS_*`).
    pub closed_loop_activation_status: u8,
    /// First closed-loop entry position sample after entering position mode.
    pub closed_loop_activation_position_1: i32,
    /// Second closed-loop entry position sample after the stability delay.
    pub closed_loop_activation_position_2: i32,
    /// Absolute drift between the two closed-loop entry position samples.
    pub closed_loop_activation_drift_count: u16,
    /// Closed-loop entry drift threshold used for this run.
    pub closed_loop_activation_threshold_count: u16,
    /// Non-zero when the closed-loop entry rejection was runaway growth.
    pub closed_loop_activation_runaway: u8,
    /// Sustained-hold gate status (`HOLD_GATE_STATUS_*`).
    pub current_hold_status: u8,
    /// Sustained-hold warning bitfield (`HOLD_GATE_WARNING_*`).
    pub current_hold_warning_flags: u16,
    /// Sustained-hold elapsed time in microseconds.
    pub current_hold_elapsed_us: u32,
    /// Sustained-hold sample period requested by firmware.
    pub current_hold_requested_sample_period_us: u16,
    /// Sustained-hold samples collected.
    pub current_hold_sample_count: u8,
    /// Sustained-hold position span.
    pub current_hold_position_span_count: u16,
    /// Absolute difference between first and last sustained-hold position.
    pub current_hold_position_drift_count: u16,
    /// Mean torque current during sustained hold.
    pub current_hold_torque_mean_count: i16,
    /// RMS torque current about the mean.
    pub current_hold_torque_rms_count: u16,
    /// Peak-to-peak torque current span.
    pub current_hold_torque_peak_to_peak_count: u16,
    /// Raw torque-current zero-crossing count.
    pub current_hold_torque_crossing_count: u8,
    /// Mean flux current during sustained hold.
    pub current_hold_flux_mean_count: i16,
    /// RMS flux current about the mean.
    pub current_hold_flux_rms_count: u16,
    /// Peak-to-peak flux current span.
    pub current_hold_flux_peak_to_peak_count: u16,
    /// Raw flux-current zero-crossing count.
    pub current_hold_flux_crossing_count: u8,
    /// OR of all sampled TMC4671 `STATUS_FLAGS`.
    pub current_hold_status_flags_or: u32,
    /// Count of samples with actionable status bits.
    pub current_hold_actionable_status_count: u8,
    /// Primary current-validation target after all safety caps.
    ///
    /// This is retained for compatibility and mirrors the flux-axis target,
    /// because the settled flux gate is the production hard gate.
    pub current_validation_step_amplitude: u16,
    /// Flux-axis current-validation target after all safety caps.
    pub current_validation_flux_step_amplitude: u16,
    /// Torque-axis current-validation target after all safety caps.
    pub current_validation_torque_step_amplitude: u16,
    /// PIDOUT limit used while collecting current-validation samples.
    pub current_validation_pidout_limit: u16,
    /// Target cap derived from the configured current limit.
    pub current_validation_current_limited: u16,
    /// Target cap derived from PIDOUT headroom and the candidate P gain.
    pub current_validation_voltage_limited: u16,
    /// Maximum candidate P gain used for voltage-limited target sizing.
    pub current_validation_max_p: u16,
    /// Number of populated settled-flux raw validation reads.
    pub current_settled_validation_count: u8,
    /// Raw settled-flux validation reads retained for offline analysis.
    pub current_settled_validation: [CurrentValidationSettledSampleEvidence;
        crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
    /// Number of populated flux-axis validation evidence items.
    pub current_flux_validation_count: u8,
    /// Flux-axis validation evidence by sample delay.
    pub current_flux_validation:
        [CurrentValidationAxisEvidence; crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
    /// Number of populated torque-axis validation evidence items.
    pub current_torque_validation_count: u8,
    /// Torque-axis validation evidence by sample delay.
    pub current_torque_validation:
        [CurrentValidationAxisEvidence; crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
}

/// Firmware-authored provenance for a deployed outer tune.
///
/// All-zero (via `Default`) on legacy replies and failure paths, where no
/// robustness-gated candidate was evaluated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct TuneProvenance {
    /// Probed velocity, in milli-revolutions/second. Same domain as the
    /// request field `requested_velocity_mrev_s`.
    pub probed_velocity_mrev_s: u32,
    /// Settled-rate-Q, in the encoder-rate-Q domain of
    /// `selected_response_intervals_q`.
    pub d_eq_q: i32,
    /// Q0.16 normalized confidence fraction (0 = none, 65535 ~= 1.0).
    pub confidence_q: u16,
    /// Lower response-band edge, integer percent (e.g. 70).
    pub band_lower_percent: u8,
    /// Upper response-band edge, integer percent (e.g. 80).
    pub band_upper_percent: u8,
    /// Q0.16 position within `[band_lower_percent, band_upper_percent]`
    /// (0 = lower edge, 65535 = upper edge).
    pub band_position_q: u16,
}

/// Result of outer (mechanical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct OuterResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q4.12).
    pub velocity_i: u16,
    /// Velocity limit.
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
    /// Provenance of the deployed tune (probed velocity, D_eq, confidence,
    /// response band). All-zero on legacy replies and failure paths.
    pub provenance: TuneProvenance,
}

impl Default for OuterResult {
    /// Zero-filled result for legacy and failure paths. `status` defaults to
    /// `Failed(Spi)`, matching the existing `zero_outer_result` convention.
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
    /// Number of samples folded into the means.
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
    /// Empty evidence used before ADC residual capture has run.
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

/// Result of the calibration subset requested by `new_calibrate`.
#[derive(Debug, Clone, defmt::Format)]
pub struct AdcCalResult {
    /// ADC I0 offset.
    pub adc_i0: u16,
    /// ADC I1 offset.
    pub adc_i1: u16,
    /// Encoder count captured before the alignment phase zeroed it.
    pub encoder_count: u32,
    /// Folded residual of `encoder_count` modulo one electrical revolution.
    pub encoder_alignment_electrical_residual_counts: u16,
    /// Difference between the two post-settle alignment reads.
    pub encoder_alignment_stability_counts: u16,
    /// Largest movement from the pre-alignment count to a post-settle read.
    pub encoder_alignment_movement_counts: u16,
    /// Minimum movement required for the alignment to be accepted.
    pub encoder_alignment_min_movement_counts: u16,
    /// Encoder counts per electrical revolution.
    pub encoder_alignment_counts_per_electrical_rev: u16,
    /// Zero-current residual with PWM chopping disabled after offset programming.
    pub adc_residual_chop0: AdcResidualEvidence,
    /// Zero-current residual with PWM chopping restored after offset programming.
    pub adc_residual_chop7: AdcResidualEvidence,
}

/// Result of Stage 1 commissioning.
///
/// Combines the inner result (current-loop gains, electrical model) with
/// the fallback outer gains applied during closed-loop entry. The terminal
/// `foci_commission_result` wire reply carries the gains; supplemental
/// replies carry evidence fields such as the applied filters.
#[derive(Debug, Clone, defmt::Format)]
pub struct CommissionResult {
    /// Inner commissioning result (current-loop gains, electrical model).
    pub inner: InnerResult,
    /// Fallback velocity P gain (Q8.8) written during closed-loop entry.
    pub fallback_velocity_p: u16,
    /// Fallback velocity I gain (Q4.12) written during closed-loop entry.
    pub fallback_velocity_i: u16,
    /// Fallback position P gain (Q8.8) written during closed-loop entry.
    pub fallback_position_p: u16,
    /// Fallback position I gain (Q4.12) written during closed-loop entry.
    pub fallback_position_i: u16,
    /// Fallback PID velocity limit register value.
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
mod fixed_gain_amplitude_code_tests {
    use super::{FixedGainAmplitudeCause, FixedGainAmplitudeOutcome};

    #[test]
    fn fixed_gain_amplitude_wire_namespace_is_stable() {
        assert_eq!(FixedGainAmplitudeOutcome::Complete as u8, 0);
        assert_eq!(FixedGainAmplitudeOutcome::Inconclusive as u8, 1);
        assert_eq!(FixedGainAmplitudeOutcome::Fault as u8, 2);
        assert_eq!(FixedGainAmplitudeOutcome::Failed as u8, 3);

        assert_eq!(FixedGainAmplitudeCause::None as u8, 0);
        assert_eq!(FixedGainAmplitudeCause::InsufficientSharedFloor as u8, 1);
        assert_eq!(FixedGainAmplitudeCause::RecoveryUnavailable as u8, 2);
        assert_eq!(FixedGainAmplitudeCause::MissingAcceptancePoint as u8, 3);
        assert_eq!(
            FixedGainAmplitudeCause::AcceptancePointIdentityMismatch as u8,
            4
        );
        assert_eq!(FixedGainAmplitudeCause::AmplitudeOrderMismatch as u8, 5);
        assert_eq!(FixedGainAmplitudeCause::EvidenceCapacity as u8, 6);
        assert_eq!(FixedGainAmplitudeCause::EvidenceIntegrity as u8, 7);
        assert_eq!(FixedGainAmplitudeCause::VelocityRestNotConfirmed as u8, 53);
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

    /// The same raw value from each namespace must stay distinguishable by
    /// `namespace_code` alone -- two enumerations legitimately assigning the
    /// same number to different things is not itself a bug.
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
