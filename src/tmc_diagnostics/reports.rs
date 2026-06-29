/// Diagnostic could not run because the channel was not ready.
pub const CURRENT_STEP_STATUS_NOT_READY: u8 = 1;

/// Diagnostic failed while communicating with or restoring the TMC4671.
pub const CURRENT_STEP_STATUS_SPI: u8 = 2;

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

/// Impedance diagnostic failed because the motor type is unsupported.
pub const IMPEDANCE_STATUS_UNSUPPORTED_MOTOR_TYPE: u8 = 10;
/// Impedance diagnostic failed because the voltage limit cannot cover max UD.
pub const IMPEDANCE_STATUS_INSUFFICIENT_VOLTAGE: u8 = 11;
/// Impedance diagnostic failed because the profile is invalid for firmware.
pub const IMPEDANCE_STATUS_UNSUPPORTED_PROFILE: u8 = 12;
/// Impedance diagnostic failed because a required readback mismatched.
pub const IMPEDANCE_STATUS_PROFILE_READBACK: u8 = 13;
/// Impedance diagnostic failed because current exceeded the profile limit.
pub const IMPEDANCE_STATUS_CURRENT_LIMIT: u8 = 14;
/// Impedance diagnostic failed because the encoder moved too far.
pub const IMPEDANCE_STATUS_ENCODER_MOVED: u8 = 15;
/// Impedance diagnostic failed because the captured signal was too small.
pub const IMPEDANCE_STATUS_LOW_SIGNAL: u8 = 16;
/// Impedance diagnostic failed because phase coverage was insufficient.
pub const IMPEDANCE_STATUS_POOR_COVERAGE: u8 = 17;
/// Impedance diagnostic failed because scalar 2f mismatch exceeded the hard gate.
pub const IMPEDANCE_STATUS_SCALAR_2F_MISMATCH: u8 = 18;
/// Impedance diagnostic failed because fit residual exceeded the hard gate.
pub const IMPEDANCE_STATUS_HIGH_RESIDUAL: u8 = 19;
/// Impedance diagnostic failed because cross-point repeatability exceeded the hard gate.
pub const IMPEDANCE_STATUS_REPEATABILITY: u8 = 20;
/// Impedance diagnostic failed because TMC4671 `STATUS_FLAGS` reported a hard fault.
pub const IMPEDANCE_STATUS_UNSAFE_STATUS_FLAGS: u8 = 21;
/// Impedance diagnostic failed because the fitted saliency axis deviated too far.
pub const IMPEDANCE_STATUS_AXIS_DEVIATION: u8 = 22;
/// Impedance diagnostic failed because torque/flux filter state could not be controlled.
///
/// Values 21 and 22 are stable legacy meanings, so filter-state failures use
/// the next available code to preserve host compatibility.
pub const IMPEDANCE_STATUS_FILTER_STATE: u8 = 23;

/// Board/profile constants reported for display by the impedance-test
/// diagnostic. Carries no per-run status; always emitted once per request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImpedanceProfileReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Number of injection points in this profile.
    pub point_count: u8,
    /// Number of samples captured per injection point.
    pub sample_count: u16,
    /// Sampling interval in microseconds.
    pub sample_interval_us: u16,
    /// Square-carrier frequency for held-angle sampling, in millihertz.
    pub carrier_frequency_millihz: u32,
    /// Number of samples captured for each carrier cycle.
    pub samples_per_cycle: u16,
    /// Number of carrier cycles captured at each fixed angle.
    pub cycles_per_angle: u16,
    /// Maximum absolute UD voltage request used by the profile.
    pub max_ud: u16,
    /// DC UD magnitude used to hold the rotor at each commanded electrical angle.
    pub hold_ud: u16,
    /// Square-carrier UD amplitude around the held angle.
    pub carrier_ud: u16,
    /// STATUS_FLAGS bitmask that hard-fails the run.
    pub status_flags_fail_mask: u32,
    /// STATUS_FLAGS bitmask that raises the warning flag during a window.
    pub status_flags_warn_mask: u32,
    /// Whether physical scale projection metadata has been validated.
    pub scale_metadata_validated: bool,
}

/// Per-run status, warnings, and aggregate fit result for the impedance-test
/// diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImpedanceRunReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, non-zero stable failure reason.
    pub status: u8,
    /// Selected major response axis normalized to counts per 1000 UD counts.
    pub selected_axis_max_count_per_ud_milli: u32,
    /// Selected minor response axis normalized to counts per 1000 UD counts.
    pub selected_axis_min_count_per_ud_milli: u32,
    /// Worse cross-point spread of normalized major/minor axes.
    pub repeatability_permille: u16,
    /// Run-level warning bitfield.
    pub warning_flags: u32,
    /// STATUS_FLAGS accumulated (OR) across the run.
    pub status_flags_or: u32,
    /// Register address for the last profile readback mismatch, or zero.
    pub readback_mismatch_addr: u8,
    /// Expected value for the last profile readback mismatch.
    pub readback_expected: u32,
    /// Actual value for the last profile readback mismatch.
    pub readback_actual: u32,
    /// Torque filter enable state before held-angle sampling.
    pub filter_torque_enable_before: u8,
    /// Flux filter enable state before held-angle sampling.
    pub filter_flux_enable_before: u8,
    /// Torque filter enable state during held-angle sampling.
    pub filter_torque_enable_during: u8,
    /// Flux filter enable state during held-angle sampling.
    pub filter_flux_enable_during: u8,
    /// Encoder drift between duplicate-zero observation starts.
    pub duplicate_zero_encoder_drift_counts: u32,
}

/// Per-injection-point fit result for the impedance-test diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImpedanceFitReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Stable point index within the profile.
    pub point_index: u8,
    /// Electrical injection frequency in millihertz.
    pub frequency_millihz: u32,
    /// Absolute UD magnitude applied during the window.
    pub ud_abs: u16,
    /// Major response axis normalized to counts per 1000 UD counts.
    pub axis_max_count_per_ud_milli: u32,
    /// Minor response axis normalized to counts per 1000 UD counts.
    pub axis_min_count_per_ud_milli: u32,
    /// Major-axis stationary-frame electrical angle in 16-bit turn counts.
    pub axis_angle_electrical_counts: u16,
    /// Fitted-model residual as permille of fitted signal RMS.
    pub residual_permille: u16,
    /// Phase coverage determinant score, where 1000 is ideal circular coverage.
    pub coverage_permille: u16,
    /// Scalar `2f` saliency mismatch as permille.
    pub scalar_2f_mismatch_permille: u16,
    /// Mean supplemental lock-in response normalized to counts per 1000 UD counts.
    pub mean_response_count_per_ud_milli: u32,
    /// Supplemental saliency relative to mean response, in permille.
    pub saliency_permille: u16,
    /// Non-zero when the supplemental saliency axis was determinable.
    pub saliency_determinable: u8,
    /// Supplemental valid fixed-angle coverage, in permille.
    pub angle_coverage_permille: u16,
    /// Supplemental half-turn folded deviation from expected electrical zero.
    pub axis_deviation_counts: u16,
    /// Per-angle supplemental lock-in inductive response normalized to counts per 1000 UD counts.
    pub inductive_response_count_per_ud_milli: u32,
    /// Per-angle supplemental in-phase response normalized to counts per 1000 UD counts.
    pub in_phase_response_count_per_ud_milli: u32,
    /// Secondary ADC supplemental lock-in response normalized to counts per 1000 UD counts.
    pub secondary_adc_response_count_per_ud_milli: u32,
    /// Mean transformed-current magnitude normalized to counts per 1000 UD counts.
    pub magnitude_mean_count_per_ud_milli: u32,
    /// `2f` transformed-current magnitude ripple normalized to counts per 1000 UD counts.
    pub magnitude_2f_count_per_ud_milli: u32,
    /// Transformed-current magnitude `2f` fit residual in permille.
    pub magnitude_2f_residual_permille: u16,
    /// Transformed-current magnitude `2f` coverage score.
    pub magnitude_2f_coverage_permille: u16,
    /// Per-point warning bitfield.
    pub warning_flags: u32,
}

/// Per-injection-point chip observation for the impedance-test diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImpedanceObservationReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Stable point index within the profile.
    pub point_index: u8,
    /// Electrical injection frequency in millihertz.
    pub frequency_millihz: u32,
    /// Absolute UD magnitude requested during the window.
    pub ud_abs: u16,
    /// Electrical angle for the fixed-angle supplemental observation.
    pub angle_electrical_counts: u16,
    /// STATUS_FLAGS observed during this point's DC and AC windows.
    pub status_flags_or: u32,
    /// OPENLOOP_PHI sampled while AC voltage is active.
    pub openloop_phi_sample: u16,
    /// OPENLOOP_VELOCITY_ACTUAL sampled while AC voltage is active.
    pub openloop_velocity_actual: i32,
    /// Direct ADC IUX sample from `ADC_IWY_IUX`.
    pub adc_iux_sample: i16,
    /// Direct ADC IWY sample from `ADC_IWY_IUX`.
    pub adc_iwy_sample: i16,
    /// Stationary-frame FOC IUX sample from `INTERIM_FOC_IWY_IUX`.
    pub foc_iux_sample: i16,
    /// Stationary-frame FOC IWY sample from `INTERIM_FOC_IWY_IUX`.
    pub foc_iwy_sample: i16,
    /// Pre-limit FOC UQ sample from `INTERIM_FOC_UQ_UD`.
    pub foc_uq_sample: i16,
    /// Pre-limit FOC UD sample from `INTERIM_FOC_UQ_UD`.
    pub foc_ud_sample: i16,
    /// Limited FOC UQ sample from `INTERIM_FOC_UQ_UD_LIMITED`.
    pub foc_uq_limited_sample: i16,
    /// Limited FOC UD sample from `INTERIM_FOC_UQ_UD_LIMITED`.
    pub foc_ud_limited_sample: i16,
    /// Park-transformed q-axis current sample from `INTERIM_FOC_IQ_ID`.
    pub foc_iq_sample: i16,
    /// Park-transformed d-axis current sample from `INTERIM_FOC_IQ_ID`.
    pub foc_id_sample: i16,
    /// Per-angle supplemental lock-in inductive response normalized to counts per 1000 UD counts.
    pub inductive_response_count_per_ud_milli: u32,
    /// Per-angle supplemental in-phase response normalized to counts per 1000 UD counts.
    pub in_phase_response_count_per_ud_milli: u32,
    /// Encoder count before the supplemental observation window.
    pub encoder_before: u32,
    /// Encoder count after the supplemental observation window.
    pub encoder_after: u32,
    /// Minimum encoder count observed during the supplemental observation window.
    pub encoder_min: u32,
    /// Maximum encoder count observed during the supplemental observation window.
    pub encoder_max: u32,
    /// Peak encoder movement observed during the supplemental observation window.
    pub encoder_peak_counts: u16,
    /// Non-zero when the supplemental fixed-angle observation participated in the fit.
    pub angle_valid: u8,
}

/// Per-injection-point DC baseline observation for the impedance diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImpedanceBaselineReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Stable point index within the profile.
    pub point_index: u8,
    /// Electrical injection frequency in millihertz.
    pub frequency_millihz: u32,
    /// Absolute UD magnitude requested for this point.
    pub ud_abs: u16,
    /// Direct ADC IUX sample from the static-UD DC baseline.
    pub dc_adc_iux_sample: i16,
    /// Direct ADC IWY sample from the static-UD DC baseline.
    pub dc_adc_iwy_sample: i16,
    /// Stationary-frame FOC IUX sample from the static-UD DC baseline.
    pub dc_foc_iux_sample: i16,
    /// Stationary-frame FOC IWY sample from the static-UD DC baseline.
    pub dc_foc_iwy_sample: i16,
    /// Park-transformed q-axis current sample from the static-UD DC baseline.
    pub dc_foc_iq_sample: i16,
    /// Park-transformed d-axis current sample from the static-UD DC baseline.
    pub dc_foc_id_sample: i16,
}

/// Construct a zero-filled [`ImpedanceRunReport`] carrying only the header
/// status, used on entry-precheck rejection or runner failure before a
/// successful aggregate fit exists.
pub fn empty_impedance_run_report(oid: u8, status: u8) -> ImpedanceRunReport {
    ImpedanceRunReport {
        oid,
        status,
        selected_axis_max_count_per_ud_milli: 0,
        selected_axis_min_count_per_ud_milli: 0,
        repeatability_permille: 0,
        warning_flags: 0,
        status_flags_or: 0,
        readback_mismatch_addr: 0,
        readback_expected: 0,
        readback_actual: 0,
        filter_torque_enable_before: 0,
        filter_flux_enable_before: 0,
        filter_torque_enable_during: 0,
        filter_flux_enable_during: 0,
        duplicate_zero_encoder_drift_counts: 0,
    }
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
