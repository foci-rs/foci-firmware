//! Public data types for commissioning.
//!
//! These types form the cross-crate API surface of the commissioning
//! engine. They are referenced by board binaries for engine construction
//! and result dispatch.

use crate::autotune::{CommissionError, CommissionStatus, PhaseId};

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
    /// Engine constructed via `new_outer` — Stage 2 mechanical/velocity tune.
    OuterTune,
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
    PhaseChanged(PhaseId),
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
/// Electrical-identification timing method identifier for delay.
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
    /// Computed flux I gain (raw Q8.8; effective advanced-PI zero is I/65536).
    pub flux_i: u16,
    /// Computed torque P gain (Q8.8).
    pub torque_p: u16,
    /// Computed torque I gain (raw Q8.8; effective advanced-PI zero is I/65536).
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
    /// Bits 0 and 3 report model warnings; bits 1 and 2 are unused. Bit 5 reports
    /// fallback gains, bit 6 reports host-default confidence, and bit 7 is
    /// reserved.
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
    /// Closed-loop entry fast stability status (`CLOSED_LOOP_ENTRY_STATUS_*`).
    pub closed_loop_entry_status: u8,
    /// First closed-loop entry position sample after entering position mode.
    pub closed_loop_entry_position_1: i32,
    /// Second closed-loop entry position sample after the stability delay.
    pub closed_loop_entry_position_2: i32,
    /// Absolute drift between the two closed-loop entry position samples.
    pub closed_loop_entry_drift_count: u16,
    /// Closed-loop entry drift threshold used for this run.
    pub closed_loop_entry_threshold_count: u16,
    /// Non-zero when the closed-loop entry rejection was runaway growth.
    pub closed_loop_entry_runaway: u8,
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

/// Result of outer (mechanical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct OuterResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q8.8).
    pub velocity_i: u16,
    /// Velocity limit.
    pub velocity_limit: u32,
    /// Position P gain (Q8.8).
    pub position_p: u16,
    /// Position I gain (Q8.8).
    pub position_i: u16,
    /// Velocity filter cutoff Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Position filter cutoff Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Flux filter cutoff Hz (0 = disabled).
    pub flux_filter_hz: u16,
    /// Torque filter cutoff Hz (0 = disabled).
    pub torque_filter_hz: u16,
    /// Identified effective inertia from mechanical ID (internal units).
    pub j_eff: u32,
    /// Identified effective damping from mechanical ID (internal units).
    pub b_eff: u32,
    /// Warning reason for accepted-with-warning results (0 = none).
    pub warning_code: u8,
    /// Torque step used by the final mechanical-ID attempt.
    pub mech_torque_step: u16,
    /// Forward initial acceleration, absolute encoder counts/ms^2.
    pub mech_fwd_accel: u32,
    /// Reverse initial acceleration, absolute encoder counts/ms^2.
    pub mech_rev_accel: u32,
    /// Forward peak velocity, absolute encoder counts/ms.
    pub mech_fwd_peak_velocity: u32,
    /// Reverse peak velocity, absolute encoder counts/ms.
    pub mech_rev_peak_velocity: u32,
    /// Forward total travel during mechanical ID, encoder counts.
    pub mech_fwd_travel: u32,
    /// Reverse total travel during mechanical ID, encoder counts.
    pub mech_rev_travel: u32,
    /// Velocity P before safety fallback, if synthesis reached that far.
    pub synth_velocity_p: u16,
    /// Velocity I before safety fallback, if synthesis reached that far.
    pub synth_velocity_i: u16,
    /// Position P before safety fallback, if position tuning reached that far.
    pub synth_position_p: u16,
    /// Position I before safety fallback, if position tuning reached that far.
    pub synth_position_i: u16,
    /// Compact outer evidence bitfield.
    ///
    /// bit0: velocity model candidate was under-driven and probed upward.
    /// bit1: mechanical ID was low confidence.
    /// bit2: mechanical tau was defaulted instead of derived from J/B.
    /// bit3: host motion budget clamped a firmware hard cap.
    /// bit4: Conservative velocity policy was required.
    /// bit5: velocity lambda was widened from Stage 1 warning evidence.
    ///
    /// Bits 6-7 are reserved and must be zero.
    pub outer_evidence_flags: u8,
    /// Stiffness classification timebase in milliseconds.
    pub stiffness_timebase_ms: u16,
    /// Reason velocity search stopped.
    pub velocity_search_stop_reason: u8,
    /// Host-provided motion budget in milli-revolutions after clamping.
    pub motion_budget_mrev: u16,
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
    /// Fallback velocity I gain (Q8.8) written during closed-loop entry.
    pub fallback_velocity_i: u16,
    /// Fallback position P gain (Q8.8) written during closed-loop entry.
    pub fallback_position_p: u16,
    /// Fallback position I gain (Q8.8) written during closed-loop entry.
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
