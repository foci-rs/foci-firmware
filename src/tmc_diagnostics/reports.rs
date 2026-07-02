/// Diagnostic could not run because the channel was not ready.
pub const CURRENT_STEP_STATUS_NOT_READY: u8 = 1;

/// Diagnostic failed while communicating with or restoring the TMC4671.
pub const CURRENT_STEP_STATUS_SPI: u8 = 2;

/// High-rate capture completed successfully.
pub const HIGH_RATE_CAPTURE_STATUS_OK: u8 = 0;
/// High-rate capture could not run because the channel was not ready.
pub const HIGH_RATE_CAPTURE_STATUS_NOT_READY: u8 = 1;
/// High-rate capture failed while communicating with the TMC4671.
pub const HIGH_RATE_CAPTURE_STATUS_SPI: u8 = 2;
/// High-rate capture restored safe state but rejected the profile selector.
pub const HIGH_RATE_CAPTURE_STATUS_BAD_PROFILE: u8 = 3;
/// High-rate capture restored safe state but the estimator could not fit data.
pub const HIGH_RATE_CAPTURE_STATUS_ESTIMATOR: u8 = 4;
/// High-rate capture failed or could not confirm safe-state restore.
pub const HIGH_RATE_CAPTURE_STATUS_RESTORE: u8 = 5;

/// Collects readings captured during a current-step or current-vector-step test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentStepReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not ready, 2 SPI or restore failure.
    pub status: u8,
    /// Scalar current-step target, or torque target for current-vector-step.
    pub torque_target: i16,
    /// Flux target for current-vector-step, or for a flux-axis scalar step.
    pub flux_target: i16,
    /// Requested dwell duration in milliseconds.
    pub duration_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS value used during the diagnostic.
    pub voltage_limit: u16,
    /// Scalar current-step actual, or torque actual for current-vector-step, before the target.
    pub torque_before: i16,
    /// Scalar current-step actual, or torque actual for current-vector-step, during the target.
    pub torque_during: i16,
    /// Scalar current-step actual, or torque actual for current-vector-step, after target zero.
    pub torque_after: i16,
    /// Flux actual while the diagnostic target is applied.
    pub flux_during: i16,
    /// FOC IQ while the diagnostic target is applied.
    pub iq_during: i16,
    /// FOC ID while the diagnostic target is applied.
    pub id_during: i16,
    /// Limited UQ while the diagnostic target is applied.
    pub uq_limited: i16,
    /// Limited UD while the diagnostic target is applied.
    pub ud_limited: i16,
    /// ABN decoder count before applying the target.
    pub encoder_before: u32,
    /// ABN decoder count after returning target to zero.
    pub encoder_after: u32,
    /// Signed encoder delta from before to after.
    pub encoder_delta: i32,
    /// Raw TMC4671 supply-voltage ADC sample.
    pub adc_vm_raw: u16,
}

/// Collects readings captured during a current torque sample test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentTorqueSampleReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not ready, 2 SPI or restore failure.
    pub status: u8,
    /// Requested signed torque target in TMC4671 current counts.
    pub target: i16,
    /// Requested signed flux target in TMC4671 current counts.
    pub flux_target: i16,
    /// Requested sample delay in milliseconds.
    pub sample_delay_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS value used during the diagnostic.
    pub voltage_limit: u16,
    /// Torque actual before applying the diagnostic target.
    pub torque_before: i16,
    /// Torque actual at the sample point.
    pub torque_sample: i16,
    /// Torque actual after returning target to zero.
    pub torque_after: i16,
    /// Flux actual at the sample point.
    pub flux_sample: i16,
    /// FOC IQ at the sample point.
    pub iq_sample: i16,
    /// FOC ID at the sample point.
    pub id_sample: i16,
    /// Limited UQ at the sample point.
    pub uq_limited: i16,
    /// Limited UD at the sample point.
    pub ud_limited: i16,
    /// ABN decoder count before applying the target.
    pub encoder_before: u32,
    /// ABN decoder count at the sample point.
    pub encoder_sample: u32,
    /// ABN decoder count after returning target to zero.
    pub encoder_after: u32,
    /// Signed encoder delta from before to sample.
    pub encoder_delta_sample: i32,
    /// Signed encoder delta from before to after.
    pub encoder_delta_after: i32,
    /// Raw TMC4671 supply-voltage ADC sample.
    pub adc_vm_raw: u16,
    /// Raw PIDIN target torque monitor.
    pub pidin_target_torque: i32,
    /// Raw PIDIN target flux monitor.
    pub pidin_target_flux: i32,
    /// Raw PIDOUT target torque monitor.
    pub pidout_target_torque: i32,
    /// Raw PIDOUT target flux monitor.
    pub pidout_target_flux: i32,
    /// Signed PID torque target monitor.
    pub pid_torque_target_monitor: i16,
    /// PID torque error at the sample point.
    pub torque_error: i32,
    /// PID flux error at the sample point.
    pub flux_error: i32,
    /// PID torque error integrator at the sample point.
    pub torque_error_sum: i32,
    /// PID flux error integrator at the sample point.
    pub flux_error_sum: i32,
    /// UQ before the voltage limiter at the sample point.
    pub uq_prelimit: i16,
    /// UD before the voltage limiter at the sample point.
    pub ud_prelimit: i16,
    /// Velocity feedforward monitor at the sample point.
    pub ff_velocity: i32,
    /// Torque feedforward monitor at the sample point.
    pub ff_torque: i16,
    /// Raw TMC4671 status flags at the sample point.
    pub status_flags: u32,
}

/// Collects readings captured during an open-loop voltage step test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VoltageStepReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not ready, 2 SPI or restore failure.
    pub status: u8,
    /// Requested signed external UQ voltage.
    pub uq_ext: i16,
    /// Requested signed external UD voltage.
    pub ud_ext: i16,
    /// Requested sample delay in milliseconds.
    pub sample_delay_ms: u16,
    /// Torque actual before applying the voltage step.
    pub torque_before: i16,
    /// Torque actual at the sample point.
    pub torque_sample: i16,
    /// Torque actual after returning voltage to zero.
    pub torque_after: i16,
    /// Flux actual at the sample point.
    pub flux_sample: i16,
    /// FOC IQ at the sample point.
    pub iq_sample: i16,
    /// FOC ID at the sample point.
    pub id_sample: i16,
    /// Limited UQ at the sample point.
    pub uq_limited: i16,
    /// Limited UD at the sample point.
    pub ud_limited: i16,
    /// UUX voltage monitor at the sample point.
    pub uux_sample: i16,
    /// UWY voltage monitor at the sample point.
    pub uwy_sample: i16,
    /// PWM UX monitor at the sample point.
    pub pwm_ux_sample: i16,
    /// PWM WY monitor at the sample point.
    pub pwm_wy_sample: i16,
    /// PWM_SV_CHOP register value at the sample point.
    pub pwm_sv_chop: u32,
    /// PWM_BBM register value at the sample point.
    pub pwm_bbm: u32,
    /// PWM_MAXCNT register value at the sample point.
    pub pwm_maxcnt: u32,
    /// Electrical angle sample.
    pub phi_e_sample: i16,
    /// Mechanical angle sample.
    pub phi_m_sample: i16,
    /// ABN decoder count before applying the voltage step.
    pub encoder_before: u32,
    /// ABN decoder count at the sample point.
    pub encoder_sample: u32,
    /// ABN decoder count after returning voltage to zero.
    pub encoder_after: u32,
    /// Signed encoder delta from before to sample.
    pub encoder_delta_sample: i32,
    /// Signed encoder delta from before to after.
    pub encoder_delta_after: i32,
    /// Raw TMC4671 supply-voltage ADC sample.
    pub adc_vm_raw: u16,
    /// Raw TMC4671 status flags at the sample point.
    pub status_flags: u32,
}

/// Metadata for one high-rate capture validation profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HighRateCaptureProfileReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// High-rate capture profile schema version.
    pub profile_version: u16,
    /// Maximum raw sample rows the firmware capture buffer can hold.
    pub sample_capacity: u16,
    /// Number of raw samples requested by this profile.
    pub requested_samples: u16,
    /// Maximum capture window in microseconds.
    pub max_capture_us: u32,
    /// Signed q-axis external voltage component used by the profile.
    pub uq_ext: i16,
    /// Signed d-axis external voltage component used by the profile.
    pub ud_ext: i16,
    /// External electrical angle used by the profile.
    pub phi_e_ext: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS value used during the profile.
    pub voltage_limit: u16,
    /// Count-space resistance slope used for nominal R and +/-20% perturbation.
    pub r_count_slope_milli: i32,
    /// Profile-level flags.
    pub flags: u32,
}

/// Optional raw sample row for detail level 2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HighRateCaptureSampleReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Zero-based raw sample index.
    pub sample_index: u16,
    /// Monotonic timestamp relative to capture start, in microseconds.
    pub timestamp_us: u32,
    /// Signed q-axis external voltage component at this sample.
    pub uq_ext: i16,
    /// Signed d-axis external voltage component at this sample.
    pub ud_ext: i16,
    /// Primary current sample in TMC4671 current counts.
    pub current_primary: i16,
    /// Secondary current sample in TMC4671 current counts.
    pub current_secondary: i16,
    /// ABN decoder count at this sample.
    pub encoder_count: u32,
    /// Raw TMC4671 status flags at this sample.
    pub status_flags: u32,
}

/// Terminal high-rate capture result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HighRateCaptureRunReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, non-zero stable failure reason.
    pub status: u8,
    /// High-rate capture profile schema version.
    pub profile_version: u16,
    /// Number of samples captured during the run.
    pub sample_count: u16,
    /// Elapsed capture time in microseconds.
    pub elapsed_us: u32,
    /// Effective sample frequency in Hertz.
    pub effective_frequency_hz: u32,
    /// Nominal inductive time estimate in microseconds.
    pub l_nominal_us: u32,
    /// Negative perturbation shift from nominal, in permille.
    pub l_shift_minus_permille: u16,
    /// Positive perturbation shift from nominal, in permille.
    pub l_shift_plus_permille: u16,
    /// Estimated response onset time in microseconds.
    pub theta_onset_us: u32,
    /// Estimator residual RMS in current counts.
    pub residual_rms_count: u32,
    /// Maximum interval between adjacent samples in microseconds.
    pub max_sample_interval_us: u32,
    /// Signed encoder delta across the capture.
    pub encoder_delta: i32,
    /// TMC4671 status flags accumulated across the run.
    pub status_flags_or: u32,
    /// Run-level warning bitfield.
    pub warning_flags: u32,
}

/// Construct a zero-filled [`CurrentTorqueSampleReport`] with header fields.
pub fn empty_current_torque_sample_report(
    oid: u8,
    status: u8,
    target: i16,
    flux_target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) -> CurrentTorqueSampleReport {
    CurrentTorqueSampleReport {
        oid,
        status,
        target,
        flux_target,
        sample_delay_ms,
        voltage_limit,
        torque_before: 0,
        torque_sample: 0,
        torque_after: 0,
        flux_sample: 0,
        iq_sample: 0,
        id_sample: 0,
        uq_limited: 0,
        ud_limited: 0,
        encoder_before: 0,
        encoder_sample: 0,
        encoder_after: 0,
        encoder_delta_sample: 0,
        encoder_delta_after: 0,
        adc_vm_raw: 0,
        pidin_target_torque: 0,
        pidin_target_flux: 0,
        pidout_target_torque: 0,
        pidout_target_flux: 0,
        pid_torque_target_monitor: 0,
        torque_error: 0,
        flux_error: 0,
        torque_error_sum: 0,
        flux_error_sum: 0,
        uq_prelimit: 0,
        ud_prelimit: 0,
        ff_velocity: 0,
        ff_torque: 0,
        status_flags: 0,
    }
}

/// Construct a zero-filled [`VoltageStepReport`] with header fields.
pub fn empty_voltage_step_report(
    oid: u8,
    status: u8,
    uq_ext: i16,
    ud_ext: i16,
    sample_delay_ms: u16,
) -> VoltageStepReport {
    VoltageStepReport {
        oid,
        status,
        uq_ext,
        ud_ext,
        sample_delay_ms,
        torque_before: 0,
        torque_sample: 0,
        torque_after: 0,
        flux_sample: 0,
        iq_sample: 0,
        id_sample: 0,
        uq_limited: 0,
        ud_limited: 0,
        uux_sample: 0,
        uwy_sample: 0,
        pwm_ux_sample: 0,
        pwm_wy_sample: 0,
        pwm_sv_chop: 0,
        pwm_bbm: 0,
        pwm_maxcnt: 0,
        phi_e_sample: 0,
        phi_m_sample: 0,
        encoder_before: 0,
        encoder_sample: 0,
        encoder_after: 0,
        encoder_delta_sample: 0,
        encoder_delta_after: 0,
        adc_vm_raw: 0,
        status_flags: 0,
    }
}

/// Board/profile constants reported for display by the resistance-test
/// diagnostic. Carries no per-run status; always emitted once per request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResistanceProfileReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Resistance-identification profile schema version.
    pub profile_version: u16,
    /// PWM_MAXCNT register value used during the sweep.
    pub pwm_maxcnt: u16,
    /// Break-before-make high-side count.
    pub bbm_h: u8,
    /// Break-before-make low-side count.
    pub bbm_l: u8,
    /// DSADC_MDEC_A decimation/filter setting.
    pub dsadc_mdec_a: u16,
    /// DSADC_MDEC_B decimation/filter setting.
    pub dsadc_mdec_b: u16,
    /// Linear-region current threshold in counts.
    pub linear_current_threshold_count: u16,
    /// Encoder-move warning threshold in counts.
    pub encoder_move_warn_counts: u16,
    /// STATUS_FLAGS bitmask that raises the warning flag during a window.
    pub status_flags_warn_mask: u32,
    /// Whether the board's scale metadata has been validated.
    pub scale_metadata_validated: bool,
}

/// Per-run status, warnings, and power-stage readbacks for the
/// resistance-test diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResistanceRunReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, non-zero stable failure reason from
    /// [`crate::resistance_identification::ResistanceError::status_code`].
    pub status: u8,
    /// Resistance-identification profile schema version.
    pub profile_version: u16,
    /// Selected count-space resistance slope, scaled by 1000.
    pub selected_r_count_slope_milli: i32,
    /// Resistance count-slope (milli-units) actually consumed by the gain
    /// path, for the same-run 64x count-slope-milli audit. Commission-only;
    /// always 0 on the standalone diagnostic path.
    pub gain_path_count_slope_milli: i32,
    /// Run-level warning bitfield.
    pub warning_flags: u32,
    /// STATUS_FLAGS accumulated (OR) across the run.
    pub status_flags_or: u32,
    /// PWM_MAXCNT readback captured during the run.
    pub pwm_maxcnt_readback: u16,
    /// Packed PWM_BBM_H_BBM_L readback captured during the run.
    pub bbm_readback: u16,
    /// Packed DSADC_MDEC_B_MDEC_A readback captured during the run.
    pub dsadc_mdec_readback: u32,
    /// PWM_SV_CHOP readback captured during the run.
    pub pwm_sv_chop_readback: u32,
}

/// Per-electrical-axis fit result for the resistance-test diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResistanceAxisReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Stable per-result electrical-axis index.
    pub electrical_axis: u8,
    /// TMC4671 external electrical angle used for this axis.
    pub phi_e_ext: u16,
    /// Fitted count-space resistance slope, scaled by 1000.
    pub r_count_slope_milli: i32,
    /// Fitted intercept in current counts.
    pub intercept_count: i32,
    /// Fit residual in permille.
    pub rmse_permille: u16,
    /// Bitmask of points selected for the production linear-region fit.
    pub selected_mask: u16,
    /// Bitmask of points excluded from the fit.
    pub excluded_point_mask: u16,
    /// Count of points selected for the fit.
    pub selected_count: u8,
    /// Signed anchor slope across the +/-UD pair, scaled by 1000.
    pub signed_count_slope_milli: i32,
    /// Signed anchor current asymmetry in permille.
    pub signed_asymmetry_permille: u16,
    /// Repeated positive anchor drift in permille.
    pub drift_permille: u16,
    /// Per-axis warning bitfield.
    pub warning_flags: u32,
}

/// Encoder D-axis alignment evidence emitted before the terminal
/// `foci_commission_result` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderAlignmentReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Encoder count captured immediately before alignment zeroed the ABN count.
    pub encoder_count: u32,
    /// Folded residual of `encoder_count` modulo one electrical revolution.
    pub electrical_residual_counts: u16,
    /// Difference between the two post-settle alignment reads.
    pub stability_counts: u16,
    /// Largest movement from the pre-alignment count to a post-settle read.
    pub movement_counts: u16,
    /// Minimum movement required for the alignment to be accepted.
    pub min_movement_counts: u16,
    /// Encoder counts per electrical revolution for this motor.
    pub counts_per_electrical_rev: u16,
}

/// Run-level current-loop commissioning evidence emitted before the terminal
/// `foci_commission_result` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentLoopRunReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Terminal commissioning status code.
    pub status: u8,
    /// Stable `CURRENT_GAINS_SOURCE_*` code.
    pub gains_source: u8,
    /// Candidate `CURRENT_GAINS_SOURCE_*` code before validation verdict.
    pub candidate_gains_source: u8,
    /// Stable `AXIS_SPLIT_SOURCE_*` code.
    pub axis_split_source: u8,
    /// Candidate `AXIS_SPLIT_SOURCE_*` code before validation verdict.
    pub candidate_axis_split_source: u8,
    /// Stable `CURRENT_GAINS_TIER_*` code.
    pub gains_tier: u8,
    /// Candidate `CURRENT_GAINS_TIER_*` code before validation verdict.
    pub candidate_gains_tier: u8,
    /// Measured axis split ratio, permille where 1000 = 1.000x.
    pub measured_axis_split_permille: u16,
    /// Candidate measured axis split ratio before validation verdict.
    pub candidate_measured_axis_split_permille: u16,
    /// Applied axis split ratio after profile clamp, permille.
    pub applied_axis_split_permille: u16,
    /// Candidate applied axis split ratio before validation verdict.
    pub candidate_applied_axis_split_permille: u16,
    /// Non-zero when the applied split was clamped below the measured split.
    pub axis_split_clamped: u8,
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
    /// Bitmask of `CURRENT_VALIDATION_AXIS_*` axes that passed validation.
    pub current_validation_axes: u8,
    /// Number of flux-axis validation samples emitted for this run.
    pub flux_validation_sample_count: u8,
    /// Number of torque-axis validation samples emitted for this run.
    pub torque_validation_sample_count: u8,
    /// Non-zero when the section-level retry budget was exhausted.
    pub retry_budget_exhausted: u8,
    /// Stable `CURRENT_LOOP_FAILURE_*` code.
    pub failure_reason: u8,
}

/// Accepted per-coil corrected inductance fit evidence emitted before the
/// terminal `foci_commission_result` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InductanceFitReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Coil index within the two-phase motor model.
    pub coil: u8,
    /// Corrected accepted electrical time constant in microseconds.
    pub tau_us: u32,
    /// Fitted current delta slope per UD count, scaled by 1000.
    pub slope_delta_per_ud_milli: u32,
    /// Fitted current delta intercept, scaled by 1000.
    pub intercept_delta_milli: i32,
    /// Estimated voltage deadtime correction in UD counts.
    pub deadtime_ud: i16,
    /// Fit residual in permille.
    pub residual_permille: u16,
    /// Count of usable points selected for this fit.
    pub usable_points: u8,
    /// Bitmask of points selected for this fit.
    pub selected_mask: u16,
}

/// Per-amplitude dI/dt point used by a corrected inductance fit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InductancePointReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Coil index within the two-phase motor model.
    pub coil: u8,
    /// Zero-based amplitude point index.
    pub point: u8,
    /// Requested UD amplitude in TMC4671 voltage counts.
    pub ud: u16,
    /// Average current delta for this amplitude point.
    pub avg_delta: u32,
    /// Average current count sampled for this amplitude point.
    pub avg_current_count: i32,
    /// Estimated resistive voltage drop in UD counts.
    pub r_drop_ud: i32,
    /// Effective voltage after correction in UD counts.
    pub effective_ud: i32,
    /// Number of samples folded into this point.
    pub sample_count: u32,
    /// Elapsed sampling time in microseconds.
    pub elapsed_us: u32,
}

/// Per-axis, per-delay current validation evidence emitted before the terminal
/// `foci_commission_result` reply. One reply represents the positive and
/// negative samples at one delay for one axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentValidationAxisReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Stable `CURRENT_AXIS_*` axis identifier.
    pub axis: u8,
    /// Zero-based sample index for this axis.
    pub sample_index: u8,
    /// Validation status for this axis sample.
    pub status: u8,
    /// Retry attempt that produced this sample.
    pub attempt: u8,
    /// Signed current target used for the validation pulse.
    pub target: i16,
    /// Delay between applying the target and sampling, in milliseconds.
    pub sample_delay_ms: u16,
    /// Positive-pulse response normalized to target, in permille.
    pub positive_response_permille: u16,
    /// Negative-pulse response normalized to target, in permille.
    pub negative_response_permille: u16,
    /// Cross-axis response normalized to target, in permille.
    pub cross_axis_permille: u16,
    /// Peak instantaneous cross-axis response used as audit telemetry.
    pub cross_axis_peak_permille: u16,
    /// Voltage output normalized to the configured limit, in permille.
    pub voltage_output_permille: u16,
    /// Absolute encoder movement observed during the sample.
    pub encoder_delta_counts: u16,
    /// Signed encoder movement observed during the positive pulse.
    pub positive_encoder_delta_counts: i32,
    /// Signed encoder movement observed during the negative pulse.
    pub negative_encoder_delta_counts: i32,
    /// TMC4671 `STATUS_FLAGS` accumulated across the sample.
    pub status_flags_or: u32,
}

/// Construct a zero-filled [`ResistanceRunReport`] carrying only the header
/// status, used on entry-precheck rejection before any profile is read.
pub fn empty_resistance_run_report(oid: u8, status: u8) -> ResistanceRunReport {
    ResistanceRunReport {
        oid,
        status,
        profile_version: 0,
        selected_r_count_slope_milli: 0,
        gain_path_count_slope_milli: 0,
        warning_flags: 0,
        status_flags_or: 0,
        pwm_maxcnt_readback: 0,
        bbm_readback: 0,
        dsadc_mdec_readback: 0,
        pwm_sv_chop_readback: 0,
    }
}
