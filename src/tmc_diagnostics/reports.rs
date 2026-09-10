/// Diagnostic could not run because the channel was not ready.
pub const CURRENT_STEP_STATUS_NOT_READY: u8 = 1;

/// Diagnostic failed while communicating with or restoring the TMC4671.
pub const CURRENT_STEP_STATUS_SPI: u8 = 2;

/// The run aborted because the host-liveness lease expired mid-run, not
/// because of a genuine SPI transaction failure.
pub const CURRENT_STEP_STATUS_HOST_LIVENESS: u8 = 3;

/// Velocity-limit latch diagnostic could not run because the channel was not ready.
#[cfg(feature = "trace-velocity-limit-latch-test")]
pub const VELOCITY_LIMIT_LATCH_STATUS_NOT_READY: u8 = 1;

/// Velocity-limit latch diagnostic lost register or restoration integrity.
#[cfg(feature = "trace-velocity-limit-latch-test")]
pub const VELOCITY_LIMIT_LATCH_STATUS_SPI: u8 = 2;

/// Focused velocity-output-limit persistence diagnostic result.
#[cfg(feature = "trace-velocity-limit-latch-test")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VelocityLimitLatchReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: zero success, one not ready, two SPI or restoration failure.
    pub status: u8,
    /// Temporary velocity-output clamp used by the focused stimulus.
    pub test_limit: u16,
    /// Velocity proportional gain used by the focused stimulus.
    pub p_raw: u16,
    /// Velocity integral gain used by the focused stimulus.
    pub i_raw: u16,
    /// Signed velocity target in the TMC4671 RPM register domain.
    pub target_velocity_rpm: i32,
    /// Verified temporary clamp readback.
    pub limit_readback: u32,
    /// Verified velocity gain-register readback.
    pub gains_readback: u32,
    /// Verified signed velocity-target readback.
    pub target_readback: i32,
    /// Complete status word sampled before commanding zero.
    pub active_status_flags: u32,
    /// Complete status word sampled immediately after commanding zero.
    pub post_pulse_status_flags: u32,
    /// Complete status word sampled after the delayed persistence interval.
    pub delayed_status_flags: u32,
    /// Complete status word sampled after the explicit status reset.
    pub post_clear_status_flags: u32,
    /// Measured target-active interval in microseconds.
    pub pulse_elapsed_us: u32,
    /// Measured interval from the immediate to delayed status read.
    pub delayed_read_elapsed_us: u32,
    /// ABN encoder count before the stimulus.
    pub encoder_before: u32,
    /// ABN encoder count after the delayed status read.
    pub encoder_after: u32,
    /// Signed encoder displacement across the diagnostic.
    pub encoder_delta: i32,
    /// Saved operational current-limit register word.
    pub saved_limit: u32,
    /// Saved velocity gain-register word.
    pub saved_gains: u32,
    /// Saved signed velocity target.
    pub saved_target: i32,
    /// Saved mode-register word.
    pub saved_mode: u32,
    /// Readback after restoring the operational current limit.
    pub restored_limit_readback: u32,
    /// Readback after restoring the velocity gains.
    pub restored_gains_readback: u32,
    /// Readback after restoring the velocity target.
    pub restored_target_readback: i32,
    /// Readback after restoring the motion mode.
    pub restored_mode_readback: u32,
    /// Bit mask of target, gains, limit, and mode registers restored exactly.
    pub restore_verified_mask: u8,
    /// Whether the board containment hook physically disarmed the channel.
    pub power_stage_tripped: bool,
}

/// Collects readings captured during a current-step or current-vector-step test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentStepReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not ready, 2 SPI or restore failure, 3 host-liveness expiry.
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
    /// Status code: 0 success, 1 not ready, 2 SPI or restore failure, 3 host-liveness expiry.
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
    /// Status code: 0 success, 1 not ready, 2 SPI or restore failure, 3 host-liveness expiry.
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
    /// Largest absolute individual current sample observed while energized.
    pub peak_abs_current_count: u16,
    /// Largest absolute mean from a completed energized acquisition window.
    pub max_abs_steady_mean_current_count: u16,
    /// Current ceiling applied to individual energized samples.
    pub current_ceiling_count: u16,
    /// Whether sample-level containment synchronously tripped the power stage.
    pub power_stage_tripped: bool,
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

/// Post-ADC-calibration zero-current residual evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcResidualReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
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

/// Applied current-loop filter cutoffs emitted before the run report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentLoopFiltersReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Applied velocity-filter cutoff in Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Applied torque-filter cutoff in Hz (0 = disabled).
    pub torque_filter_hz: u16,
    /// Applied position-filter cutoff in Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Applied flux-filter cutoff in Hz (0 = disabled).
    pub flux_filter_hz: u16,
}

/// Fast closed-loop entry stability evidence emitted before the run report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClosedLoopActivationReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// `CLOSED_LOOP_ACTIVATION_STATUS_*`.
    pub entry_status: u8,
    /// First position sample after entering position mode.
    pub position_1: i32,
    /// Second position sample after the stability delay.
    pub position_2: i32,
    /// Absolute drift between the two position samples.
    pub drift_count: u16,
    /// Drift threshold used by the fast entry check.
    pub threshold_count: u16,
    /// Non-zero when the check rejected runaway growth.
    pub runaway: u8,
}

/// Sustained current-loop hold evidence emitted before the current-loop run report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentLoopHoldReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sustained-hold gate status (`HOLD_GATE_STATUS_*`).
    pub hold_status: u8,
    /// Sustained-hold warning bitfield (`HOLD_GATE_WARNING_*`).
    pub warning_flags: u16,
    /// Sustained-hold elapsed time in microseconds.
    pub elapsed_us: u32,
    /// Sustained-hold sample period requested by firmware.
    pub requested_sample_period_us: u16,
    /// Sustained-hold samples collected.
    pub sample_count: u8,
    /// Sustained-hold position span.
    pub position_span_count: u16,
    /// Absolute difference between first and last sustained-hold position.
    pub position_drift_count: u16,
    /// Mean torque current during sustained hold.
    pub torque_mean_count: i16,
    /// RMS torque current about the mean.
    pub torque_rms_count: u16,
    /// Peak-to-peak torque current span.
    pub torque_peak_to_peak_count: u16,
    /// Raw torque-current zero-crossing count.
    pub torque_crossing_count: u8,
    /// Mean flux current during sustained hold.
    pub flux_mean_count: i16,
    /// RMS flux current about the mean.
    pub flux_rms_count: u16,
    /// Peak-to-peak flux current span.
    pub flux_peak_to_peak_count: u16,
    /// Raw flux-current zero-crossing count.
    pub flux_crossing_count: u8,
    /// OR of all sampled TMC4671 `STATUS_FLAGS`.
    pub status_flags_or: u32,
    /// Count of samples with actionable status bits.
    pub actionable_status_count: u8,
}

/// Run-level production inductance evidence emitted before the terminal
/// `foci_commission_result` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InductanceRunReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InductanceFrameReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InductanceEstimateReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
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

/// One raw read contributing to the settled-flux current-validation gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentValidationSettledSampleReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
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

/// Current-validation target envelope emitted once per commissioning run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentValidationEnvelopeReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Primary validation target after all safety caps.
    ///
    /// Retained for compatibility and mirrors the flux-axis target.
    pub step_amplitude: u16,
    /// Flux-axis validation target after all safety caps.
    pub flux_step_amplitude: u16,
    /// Torque-axis validation target after all safety caps.
    pub torque_step_amplitude: u16,
    /// PIDOUT limit active while collecting validation samples.
    pub pidout_limit: u16,
    /// Target cap derived from the configured current limit.
    pub current_limited: u16,
    /// Target cap derived from PIDOUT headroom and candidate P gain.
    pub voltage_limited: u16,
    /// Maximum candidate P gain used for voltage-limited target sizing.
    pub max_p: u16,
}

/// Construct a zero-filled [`ResistanceRunReport`] carrying only the header
/// status, used on entry-precheck rejection before any profile is read.
pub fn empty_resistance_run_report(oid: u8, status: u8) -> ResistanceRunReport {
    ResistanceRunReport {
        oid,
        status,
        selected_r_count_slope_milli: 0,
        gain_path_count_slope_milli: 0,
        warning_flags: 0,
        status_flags_or: 0,
        peak_abs_current_count: 0,
        max_abs_steady_mean_current_count: 0,
        current_ceiling_count: 0,
        power_stage_tripped: false,
        pwm_maxcnt_readback: 0,
        bbm_readback: 0,
        dsadc_mdec_readback: 0,
        pwm_sv_chop_readback: 0,
    }
}
/// Timing-quality evidence for one commissioning electrical capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommissionTimingReport {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Electrical method: 0 resistance, 1 inductance, 2 delay.
    pub method: u8,
    /// Timing state: 0 not run, 1 accepted, 2 rejected.
    pub status: u8,
    /// Requested logical sample period in microseconds.
    pub requested_period_us: u16,
    /// Accepted logical samples.
    pub valid_samples: u16,
    /// Skipped logical sample slots.
    pub missed_samples: u16,
    /// Largest consecutive skipped-slot count.
    pub max_consecutive_misses: u8,
    /// Maximum sample-start lateness in microseconds.
    pub max_lateness_us: u32,
    /// Maximum accepted-observation interval in microseconds.
    pub max_interval_us: u32,
    /// Maximum complete poll wall time in microseconds.
    pub max_poll_wall_us: u32,
    /// Maximum SPI-await wall time in one poll in microseconds.
    pub max_spi_wall_us: u32,
}
