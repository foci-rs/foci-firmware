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
    /// Maximum voltage output for the positive/negative pair, in permille.
    pub voltage_output_permille: u16,
    /// Maximum encoder movement for the positive/negative pair.
    pub encoder_delta_counts: u16,
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
        voltage_output_permille: 0,
        encoder_delta_counts: 0,
        status_flags_or: 0,
    };
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
    /// Engine is still working. Call `step()` again next tick.
    Continue,
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

/// Result of inner (electrical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct InnerResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Resistance in scaled internal ADC/DAC units.
    pub r_int: u32,
    /// Inductance in internal micro-units.
    pub l_int: u32,
    /// Resistance in milliohms (for host display/persistence).
    pub r_mohm: u32,
    /// Inductance in microhenries (for host display/persistence).
    pub l_uh: u32,
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
    /// Inner-loop closed-loop lambda in microseconds.
    pub lambda_us: u32,
    /// Electrical transport delay in microseconds.
    pub theta_e_us: u32,
    /// Max ringing count from forward/reverse current validation.
    pub ringing_count: u8,
    /// Effective current-loop bandwidth in Hz.
    pub bandwidth_hz: u32,
    /// Electrical time constant L/R in microseconds (Phase 5 measurement).
    pub tau_e_us: u32,
    /// Tau crosscheck from the transient onset (microseconds).
    pub tau_e_crosscheck_us: u32,
    /// L/R-vs-transient tau residual in permille.
    pub tau_residual_permille: u16,
    /// Warning bitfield.
    ///
    /// Bits 0, 1, and 3 report model warnings; bit 2 is reserved for tau
    /// residual telemetry and is not emitted as a warning. Bit 5 reports
    /// fallback gains, bit 6 reports host-default confidence, and bit 7 is
    /// reserved.
    pub inner_warning_flags: u8,

    // --- Shared multi-point resistance-identification evidence ---
    //
    // Persisted so the host can audit the count-space fit and the 64x
    // count-slope-milli mismatch directly from artifacts. All slope fields
    // are in the same count-slope-milli unit space (dV_count/dI_count * 1000),
    // identical to the legacy `r_int` resistance scale.
    /// Resistance profile version that produced the run.
    pub resistance_profile_version: u16,
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

    // --- Current-loop synthesis and impedance evidence ---
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
    /// `CURRENT_AXIS_*` high-inductance axis from accepted impedance evidence.
    pub impedance_high_inductance_axis: u8,
    /// Maximum fitted current response axis, count per UD millivolt-equivalent.
    pub impedance_selected_axis_max_count_per_ud_milli: u32,
    /// Minimum fitted current response axis, count per UD millivolt-equivalent.
    pub impedance_selected_axis_min_count_per_ud_milli: u32,
    /// Fitted impedance saliency in permille.
    pub impedance_saliency_permille: u16,
    /// Fitted low-inductance axis angle in 16-bit electrical turn counts.
    pub impedance_fitted_axis_electrical_counts: u16,
    /// Maximum per-angle impedance residual in permille.
    pub impedance_residual_permille: u16,
    /// Impedance warning bitfield (`ImpedanceWarningFlags`).
    pub impedance_warning_flags: u32,
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
}

/// Result of Stage 1 commissioning.
///
/// Combines the inner result (current-loop gains, electrical model) with
/// the fallback outer gains applied during closed-loop entry. Matches the
/// wire result `foci_commission_result`.
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
}
