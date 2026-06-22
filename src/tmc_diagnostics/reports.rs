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
    /// Requested signed torque target in TMC4671 current counts.
    pub torque_target: i16,
    /// Requested signed flux target in TMC4671 current counts.
    pub flux_target: i16,
    /// Requested dwell duration in milliseconds.
    pub duration_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS value used during the diagnostic.
    pub voltage_limit: u16,
    /// Torque actual before applying the diagnostic target.
    pub torque_before: i16,
    /// Torque actual while the diagnostic target is applied.
    pub torque_during: i16,
    /// Torque actual after returning target to zero.
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

/// Construct a zero-filled [`ResistanceRunReport`] carrying only the header
/// status, used on entry-precheck rejection before any profile is read.
pub fn empty_resistance_run_report(oid: u8, status: u8) -> ResistanceRunReport {
    ResistanceRunReport {
        oid,
        status,
        profile_version: 0,
        selected_r_count_slope_milli: 0,
        warning_flags: 0,
        status_flags_or: 0,
        pwm_maxcnt_readback: 0,
        bbm_readback: 0,
        dsadc_mdec_readback: 0,
        pwm_sv_chop_readback: 0,
    }
}
