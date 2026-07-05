//! `#[klipper_reply]` struct declarations for openffboard-fw.
//!
//! Each struct in this module is a reply shape that a `#[klipper_command]`
//! handler (or Task 8's background reply emitters) sends back to the
//! Klipper host via `::ankyra::klipper_reply!(Path, field: ty = expr, ...)`.
//!
//! # Wire-name preservation
//!
//! Ankyra derives the protocol name from the struct identifier and
//! automatically converts PascalCase idents to `snake_case` on the wire
//! (e.g. `TrsyncState` → `trsync_state`). Struct names in this module
//! therefore use idiomatic Rust PascalCase while preserving the
//! on-the-wire protocol names expected by the Klipper host.
//!
//! Field declaration order is load-bearing: ankyra serializes the
//! response in struct declaration order, and the Klipper host parses
//! the data-dictionary format string in the same order. Each struct's
//! field list mirrors the pre-migration `klipper_reply!(name, field:
//! ty = expr, ...)` call-site order recorded in
//! `docs/notes/anchor-datadict-baseline.txt`.

use ankyra::prelude::klipper_reply;

/// `clock` reply — emitted by the `get_clock` handler.
///
/// Wire name: `clock`.
#[klipper_reply]
pub struct Clock {
    /// Low 32 bits of the MCU systick counter.
    pub clock: u32,
}

/// `config` reply — emitted by `get_config` and `finalize_config`.
///
/// Wire name: `config`.
#[klipper_reply]
pub struct Config {
    /// `true` once the host has sent `finalize_config`.
    pub is_config: bool,
    /// CRC of the applied configuration, as reported by the host.
    pub crc: u32,
    /// `true` if the MCU is in a shutdown state.
    pub is_shutdown: bool,
    /// Count of in-flight move queue entries.
    pub move_count: u16,
}

/// `endstop_state` reply — emitted by `endstop_home` / `endstop_query_state`.
///
/// Wire name: `endstop_state`.
#[klipper_reply]
pub struct EndstopState {
    /// Object id of the endstop reporting state.
    pub oid: u8,
    /// Non-zero while the endstop is still arming for a homing move.
    pub homing: u8,
    /// Next clock at which the endstop will be sampled.
    pub next_clock: u32,
    /// Current pin value sampled by the endstop.
    pub pin_value: u8,
}

/// `foci_calibrate_result` reply — emitted by the ADC-calibration flow.
///
/// Wire name: `foci_calibrate_result`.
#[klipper_reply]
pub struct FociCalibrateResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code (0 = success, non-zero = error code).
    pub status: u8,
    /// Calibrated ADC zero-current offset for current I0.
    pub adc_i0: u16,
    /// Calibrated ADC zero-current offset for current I1.
    pub adc_i1: u16,
    /// Encoder count captured at the end of calibration.
    pub encoder_count: u32,
}

/// `foci_adc_vm_offset_result` reply - cached ADC_VM mode-5 startup offset.
///
/// Wire name: `foci_adc_vm_offset_result`.
#[klipper_reply]
pub struct FociAdcVmOffsetResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Averaged raw ADC_VM offset, valid when `status == 0`.
    pub offset_raw: u16,
    /// Number of samples folded into `offset_raw`.
    pub sample_count: u8,
    /// Status code (0 = cached offset available, 1 = not ready).
    pub status: u8,
}

/// `foci_commission_phase` reply — emitted by the autotune cascade on
/// every phase transition.
///
/// Wire name: `foci_commission_phase`.
#[klipper_reply]
pub struct FociCommissionPhase {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Phase identifier within the commissioning cascade.
    pub phase: u8,
    /// Status code (0 = success, non-zero = error code).
    pub status: u8,
}

/// `foci_commission_detail` reply — structured commissioning diagnostic.
///
/// Wire name: `foci_commission_detail`.
#[klipper_reply]
pub struct FociCommissionDetail {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Commissioning phase identifier.
    pub phase: u8,
    /// Phase-local diagnostic code.
    pub code: u8,
    /// Diagnostic status (0 = measurement, non-zero = failure/rejection).
    pub status: u8,
    /// First code-specific value.
    pub value0: u32,
    /// Second code-specific value.
    pub value1: u32,
    /// Third code-specific value.
    pub value2: u32,
}

/// `foci_commission_result` reply — emitted at the end of inner
/// commissioning with measured electrical and fallback gain values.
///
/// Wire name: `foci_commission_result`.
#[klipper_reply]
pub struct FociCommissionResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code (0 = success, non-zero = error code).
    pub status: u8,
    /// Resistance in scaled internal ADC/DAC units.
    pub r_int: u32,
    /// Inductance in internal micro-units.
    pub l_int: u32,
    /// Phase resistance in milliohms.
    pub r_mohm: u32,
    /// Phase inductance in microhenries.
    pub l_uh: u32,
    /// Flux P gain.
    pub flux_p: u16,
    /// Flux I gain.
    pub flux_i: u16,
    /// Torque P gain.
    pub torque_p: u16,
    /// Torque I gain.
    pub torque_i: u16,
    /// Measured IMC lambda time constant in microseconds.
    pub lambda_us: u32,
    /// Measured electrical time constant in microseconds.
    pub theta_e_us: u32,
    /// Ringing count observed during the step response.
    pub ringing_count: u8,
    /// Closed-loop bandwidth in Hz.
    pub bandwidth_hz: u32,
    /// Fallback velocity P gain used by the outer loop.
    pub fallback_velocity_p: u16,
    /// Fallback velocity I gain used by the outer loop.
    pub fallback_velocity_i: u16,
    /// Fallback position P gain used by the outer loop.
    pub fallback_position_p: u16,
    /// Fallback position I gain used by the outer loop.
    pub fallback_position_i: u16,
    /// Fallback velocity limit used by the outer loop.
    pub fallback_velocity_limit: u32,
    /// Electrical time constant L/R in microseconds (Phase 5 measurement).
    pub tau_e_us: u32,
    /// Inner-confidence warning bitfield.
    ///
    /// Bits 0, 1, and 3 report model warnings; bit 2 is reserved for tau
    /// residual telemetry and is not emitted as a warning. Bit 5 reports
    /// fallback gains, bit 6 reports host-default confidence, and bit 7 is
    /// reserved.
    pub inner_warning_flags: u8,
}

/// `foci_encoder_alignment` reply - encoder D-axis alignment evidence emitted before the
/// terminal `foci_commission_result` reply.
///
/// Wire name: `foci_encoder_alignment`.
#[klipper_reply]
pub struct FociEncoderAlignment {
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

/// `foci_inductance_run` reply - run-level production inductance evidence.
///
/// Wire name: `foci_inductance_run`.
#[klipper_reply]
pub struct FociInductanceRun {
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

/// `foci_inductance_frame` reply - rotating-frame current evidence.
///
/// Wire name: `foci_inductance_frame`.
#[klipper_reply]
pub struct FociInductanceFrame {
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

/// `foci_inductance_estimate` reply - accepted count-space inductance evidence.
///
/// Wire name: `foci_inductance_estimate`.
#[klipper_reply]
pub struct FociInductanceEstimate {
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

/// `foci_current_loop_run` reply - run-level current-loop commissioning evidence.
///
/// Wire name: `foci_current_loop_run`.
#[klipper_reply]
pub struct FociCurrentLoopRun {
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

/// `foci_current_validation_axis` reply - per-axis, per-delay current validation evidence.
///
/// Wire name: `foci_current_validation_axis`.
#[klipper_reply]
pub struct FociCurrentValidationAxis {
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

/// `foci_dump_done` reply — emitted at the end of a register dump.
///
/// Wire name: `foci_dump_done`.
#[klipper_reply]
pub struct FociDumpDone {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
}

/// `foci_dump_value` reply — emitted once per register during a dump.
///
/// Wire name: `foci_dump_value`.
#[klipper_reply]
pub struct FociDumpValue {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Register address (or synthetic address for derived values).
    pub addr: u8,
    /// Register value.
    pub value: u32,
}

/// `foci_diagnostic` reply - best-effort detail before fatal shutdown.
///
/// Wire name: `foci_diagnostic`.
#[klipper_reply]
pub struct FociDiagnostic {
    /// Fault class from `foci_firmware::diagnostic::FaultClass`.
    pub fault: u8,
    /// Operation code from `foci_firmware::diagnostic::operation`.
    pub operation: u8,
    /// Stepper or FOCI OID when available, otherwise 255.
    pub oid: u8,
    /// Board-local channel when available, otherwise 255.
    pub channel: u8,
    /// First operation-specific value, such as queue depth.
    pub data0: u32,
    /// Second operation-specific value, such as queue capacity.
    pub data1: u32,
}

/// `foci_current_step_result` reply - bounded current-step diagnostic result.
///
/// Wire name: `foci_current_step_result`.
#[klipper_reply]
pub struct FociCurrentStepResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not enabled/calibrated, 2 SPI failure.
    pub status: u8,
    /// Requested signed torque target in TMC4671 current counts.
    pub target: i16,
    /// Hold duration in milliseconds.
    pub duration_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS used during the diagnostic.
    pub voltage_limit: u16,
    /// Torque actual before applying the step.
    pub torque_before: i16,
    /// Torque actual while the step is applied.
    pub torque_during: i16,
    /// Torque actual after returning target to zero.
    pub torque_after: i16,
    /// Flux actual while the step is applied.
    pub flux_during: i16,
    /// FOC IQ while the step is applied.
    pub iq_during: i16,
    /// FOC ID while the step is applied.
    pub id_during: i16,
    /// Limited UQ while the step is applied.
    pub uq_limited: i16,
    /// Limited UD while the step is applied.
    pub ud_limited: i16,
    /// ABN decoder count before applying the step.
    pub encoder_before: u32,
    /// ABN decoder count after returning target to zero.
    pub encoder_after: u32,
    /// Signed encoder delta from before to after.
    pub encoder_delta: i32,
    /// Raw TMC4671 supply-voltage ADC sample during the dwell.
    pub adc_vm_raw: u16,
}

/// `foci_current_vector_step_result` reply - bounded current-vector diagnostic.
///
/// Wire name: `foci_current_vector_step_result`.
#[klipper_reply]
pub struct FociCurrentVectorStepResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not enabled/calibrated, 2 SPI failure.
    pub status: u8,
    /// Requested signed torque-current target in TMC4671 current counts.
    pub torque_target: i16,
    /// Requested signed flux-current target in TMC4671 current counts.
    pub flux_target: i16,
    /// Hold duration in milliseconds.
    pub duration_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS used during the diagnostic.
    pub voltage_limit: u16,
    /// Torque actual before applying the step.
    pub torque_before: i16,
    /// Torque actual while the step is applied.
    pub torque_during: i16,
    /// Torque actual after returning target to zero.
    pub torque_after: i16,
    /// Flux actual while the step is applied.
    pub flux_during: i16,
    /// FOC IQ while the step is applied.
    pub iq_during: i16,
    /// FOC ID while the step is applied.
    pub id_during: i16,
    /// Limited UQ while the step is applied.
    pub uq_limited: i16,
    /// Limited UD while the step is applied.
    pub ud_limited: i16,
    /// ABN decoder count before applying the step.
    pub encoder_before: u32,
    /// ABN decoder count after returning target to zero.
    pub encoder_after: u32,
    /// Signed encoder delta from before to after.
    pub encoder_delta: i32,
    /// Raw TMC4671 supply-voltage ADC sample during the dwell.
    pub adc_vm_raw: u16,
}

/// `foci_current_torque_sample_result` reply - early torque-pulse sample result.
///
/// Wire name: `foci_current_torque_sample_result`.
#[klipper_reply]
pub struct FociCurrentTorqueSampleResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not enabled/calibrated, 2 SPI failure.
    pub status: u8,
    /// Requested signed torque target in TMC4671 current counts.
    pub target: i16,
    /// Requested signed flux target in TMC4671 current counts.
    pub flux_target: i16,
    /// Delay between applying the target and sampling, in milliseconds.
    pub sample_delay_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS used during the diagnostic.
    pub voltage_limit: u16,
    /// Torque actual before applying the pulse.
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
    /// ABN decoder count before applying the pulse.
    pub encoder_before: u32,
    /// ABN decoder count at the sample point.
    pub encoder_sample: u32,
    /// ABN decoder count after returning target to zero.
    pub encoder_after: u32,
    /// Signed encoder delta from before to sample.
    pub encoder_delta_sample: i32,
    /// Signed encoder delta from before to after.
    pub encoder_delta_after: i32,
    /// Raw TMC4671 supply-voltage ADC sample at the sample point.
    pub adc_vm_raw: u16,
    /// TMC clipped torque input target at the sample point.
    pub pidin_target_torque: i32,
    /// TMC clipped flux input target at the sample point.
    pub pidin_target_flux: i32,
    /// TMC torque-controller output target at the sample point.
    pub pidout_target_torque: i32,
    /// TMC flux-controller output target at the sample point.
    pub pidout_target_flux: i32,
    /// TMC packed monitor's torque-target half at the sample point.
    pub pid_torque_target_monitor: i16,
    /// TMC4671 status flags at the sample point.
    pub status_flags: u32,
}

/// `foci_current_torque_sample_detail_result` reply - extended early sample data.
///
/// Wire name: `foci_current_torque_sample_detail_result`.
#[klipper_reply]
pub struct FociCurrentTorqueSampleDetailResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Requested signed torque target in TMC4671 current counts.
    pub target: i16,
    /// Requested signed flux target in TMC4671 current counts.
    pub flux_target: i16,
    /// Delay between applying the target and sampling, in milliseconds.
    pub sample_delay_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS used during the diagnostic.
    pub voltage_limit: u16,
    /// TMC torque current-loop error at the sample point.
    pub torque_error: i32,
    /// TMC flux current-loop error at the sample point.
    pub flux_error: i32,
    /// TMC torque current-loop error sum at the sample point.
    pub torque_error_sum: i32,
    /// TMC flux current-loop error sum at the sample point.
    pub flux_error_sum: i32,
    /// Pre-limit UQ at the sample point.
    pub uq_prelimit: i16,
    /// Pre-limit UD at the sample point.
    pub ud_prelimit: i16,
    /// TMC velocity feedforward monitor at the sample point.
    pub ff_velocity: i32,
    /// TMC torque feedforward monitor at the sample point.
    pub ff_torque: i16,
}

/// `foci_voltage_step_result` reply - bounded open-loop voltage diagnostic.
///
/// Wire name: `foci_voltage_step_result`.
#[klipper_reply]
pub struct FociVoltageStepResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, 1 not enabled/calibrated, 2 SPI failure.
    pub status: u8,
    /// Requested signed q-axis voltage component for UQ_UD_EXT.
    pub uq_ext: i16,
    /// Requested signed d-axis voltage component for UQ_UD_EXT.
    pub ud_ext: i16,
    /// Delay between applying the voltage vector and sampling, in milliseconds.
    pub sample_delay_ms: u16,
    /// Torque actual before applying the pulse.
    pub torque_before: i16,
    /// Torque actual at the sample point.
    pub torque_sample: i16,
    /// Torque actual after returning to position mode.
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
    /// FOC2 X/UX phase-voltage command at the sample point.
    pub uux_sample: i16,
    /// FOC2 Y/WY phase-voltage command at the sample point.
    pub uwy_sample: i16,
    /// PWM UX command at the sample point.
    pub pwm_ux_sample: i16,
    /// PWM WY command at the sample point.
    pub pwm_wy_sample: i16,
    /// PWM_SV_CHOP register value at the sample point.
    pub pwm_sv_chop: u32,
    /// PWM_BBM_H_BBM_L register value at the sample point.
    pub pwm_bbm: u32,
    /// PWM_MAXCNT register value at the sample point.
    pub pwm_maxcnt: u32,
    /// ABN electrical angle at the sample point.
    pub phi_e_sample: i16,
    /// ABN mechanical angle at the sample point.
    pub phi_m_sample: i16,
    /// ABN decoder count before applying the pulse.
    pub encoder_before: u32,
    /// ABN decoder count at the sample point.
    pub encoder_sample: u32,
    /// ABN decoder count after returning to position mode.
    pub encoder_after: u32,
    /// Signed encoder delta from before to sample.
    pub encoder_delta_sample: i32,
    /// Signed encoder delta from before to after.
    pub encoder_delta_after: i32,
    /// Raw TMC4671 supply-voltage ADC sample at the sample point.
    pub adc_vm_raw: u16,
    /// TMC4671 status flags at the sample point.
    pub status_flags: u32,
}

/// `foci_resistance_profile` reply - board/profile constants for display.
///
/// Wire name: `foci_resistance_profile`.
#[klipper_reply]
pub struct FociResistanceProfile {
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
    /// Non-zero when the board's scale metadata has been validated.
    pub scale_metadata_validated: u8,
}

/// `foci_resistance_run` reply - per-run status, warnings, and readbacks.
///
/// Wire name: `foci_resistance_run`.
#[klipper_reply]
pub struct FociResistanceRun {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code: 0 success, non-zero stable failure reason.
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

/// `foci_resistance_axis` reply - per-electrical-axis fit result.
///
/// Wire name: `foci_resistance_axis`.
#[klipper_reply]
pub struct FociResistanceAxis {
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

#[cfg(test)]
mod tests {
    use super::{
        FociCurrentStepResult, FociCurrentTorqueSampleDetailResult, FociCurrentTorqueSampleResult,
        FociCurrentVectorStepResult, FociVoltageStepResult,
    };

    #[test]
    fn current_step_result_carries_motion_and_supply_observability() {
        let reply = FociCurrentStepResult {
            oid: 7,
            status: 0,
            target: 250,
            duration_ms: 80,
            voltage_limit: 12_000,
            torque_before: -3,
            torque_during: 240,
            torque_after: 18,
            flux_during: 4,
            iq_during: 239,
            id_during: -5,
            uq_limited: 1500,
            ud_limited: -20,
            encoder_before: 3900,
            encoder_after: 12,
            encoder_delta: 112,
            adc_vm_raw: 40099,
        };

        assert_eq!(reply.encoder_before, 3900);
        assert_eq!(reply.encoder_after, 12);
        assert_eq!(reply.encoder_delta, 112);
        assert_eq!(reply.adc_vm_raw, 40099);
    }

    #[test]
    fn current_vector_step_result_carries_axis_targets() {
        let reply = FociCurrentVectorStepResult {
            oid: 7,
            status: 0,
            torque_target: 0,
            flux_target: 250,
            duration_ms: 80,
            voltage_limit: 12_000,
            torque_before: -3,
            torque_during: 8,
            torque_after: 18,
            flux_during: 240,
            iq_during: 12,
            id_during: 238,
            uq_limited: 20,
            ud_limited: 1500,
            encoder_before: 3900,
            encoder_after: 3912,
            encoder_delta: 12,
            adc_vm_raw: 40099,
        };

        assert_eq!(reply.torque_target, 0);
        assert_eq!(reply.flux_target, 250);
        assert_eq!(reply.flux_during, 240);
        assert_eq!(reply.encoder_delta, 12);
    }

    #[test]
    fn current_torque_sample_result_carries_sample_observability() {
        let reply = FociCurrentTorqueSampleResult {
            oid: 7,
            status: 0,
            target: 500,
            flux_target: -125,
            sample_delay_ms: 5,
            voltage_limit: 29_000,
            torque_before: -3,
            torque_sample: 310,
            torque_after: 18,
            flux_sample: 4,
            iq_sample: 309,
            id_sample: -5,
            uq_limited: 3200,
            ud_limited: -20,
            encoder_before: 3900,
            encoder_sample: 3902,
            encoder_after: 3912,
            encoder_delta_sample: 2,
            encoder_delta_after: 12,
            adc_vm_raw: 40099,
            pidin_target_torque: 500,
            pidin_target_flux: -125,
            pidout_target_torque: 3199,
            pidout_target_flux: -25,
            pid_torque_target_monitor: 500,
            status_flags: 0x0000_8000,
        };

        assert_eq!(reply.target, 500);
        assert_eq!(reply.flux_target, -125);
        assert_eq!(reply.sample_delay_ms, 5);
        assert_eq!(reply.encoder_sample, 3902);
        assert_eq!(reply.encoder_delta_sample, 2);
        assert_eq!(reply.encoder_delta_after, 12);
        assert_eq!(reply.pidin_target_torque, 500);
        assert_eq!(reply.pidin_target_flux, -125);
        assert_eq!(reply.pidout_target_torque, 3199);
        assert_eq!(reply.pidout_target_flux, -25);
        assert_eq!(reply.pid_torque_target_monitor, 500);
        assert_eq!(reply.status_flags, 0x0000_8000);
    }

    #[test]
    fn current_torque_sample_detail_result_carries_extended_observability() {
        let reply = FociCurrentTorqueSampleDetailResult {
            oid: 7,
            target: 500,
            flux_target: -125,
            sample_delay_ms: 5,
            voltage_limit: 29_000,
            torque_error: 190,
            flux_error: -129,
            torque_error_sum: 12_345,
            flux_error_sum: -2_345,
            uq_prelimit: 3210,
            ud_prelimit: -30,
            ff_velocity: 17,
            ff_torque: -42,
        };

        assert_eq!(reply.target, 500);
        assert_eq!(reply.flux_target, -125);
        assert_eq!(reply.sample_delay_ms, 5);
        assert_eq!(reply.voltage_limit, 29_000);
        assert_eq!(reply.torque_error, 190);
        assert_eq!(reply.flux_error, -129);
        assert_eq!(reply.torque_error_sum, 12_345);
        assert_eq!(reply.flux_error_sum, -2_345);
        assert_eq!(reply.uq_prelimit, 3210);
        assert_eq!(reply.ud_prelimit, -30);
        assert_eq!(reply.ff_velocity, 17);
        assert_eq!(reply.ff_torque, -42);
    }

    #[test]
    fn voltage_step_result_carries_open_loop_observability() {
        let reply = FociVoltageStepResult {
            oid: 7,
            status: 0,
            uq_ext: 512,
            ud_ext: -256,
            sample_delay_ms: 2,
            torque_before: -3,
            torque_sample: 42,
            torque_after: 4,
            flux_sample: -21,
            iq_sample: 44,
            id_sample: -19,
            uq_limited: 500,
            ud_limited: -251,
            uux_sample: 123,
            uwy_sample: -456,
            pwm_ux_sample: 120,
            pwm_wy_sample: -450,
            pwm_sv_chop: 0x0000_0007,
            pwm_bbm: 0x0000_2828,
            pwm_maxcnt: 3999,
            phi_e_sample: 3000,
            phi_m_sample: -1200,
            encoder_before: 3900,
            encoder_sample: 3901,
            encoder_after: 3900,
            encoder_delta_sample: 1,
            encoder_delta_after: 0,
            adc_vm_raw: 40099,
            status_flags: 0x7000_0000,
        };

        assert_eq!(reply.uq_ext, 512);
        assert_eq!(reply.ud_ext, -256);
        assert_eq!(reply.sample_delay_ms, 2);
        assert_eq!(reply.iq_sample, 44);
        assert_eq!(reply.id_sample, -19);
        assert_eq!(reply.uux_sample, 123);
        assert_eq!(reply.uwy_sample, -456);
        assert_eq!(reply.pwm_ux_sample, 120);
        assert_eq!(reply.pwm_wy_sample, -450);
        assert_eq!(reply.pwm_sv_chop, 0x0000_0007);
        assert_eq!(reply.pwm_bbm, 0x0000_2828);
        assert_eq!(reply.pwm_maxcnt, 3999);
        assert_eq!(reply.phi_e_sample, 3000);
        assert_eq!(reply.phi_m_sample, -1200);
        assert_eq!(reply.encoder_delta_sample, 1);
        assert_eq!(reply.encoder_delta_after, 0);
        assert_eq!(reply.status_flags, 0x7000_0000);
    }
}

#[cfg(test)]
mod resistance_reply_size_tests {
    use super::{FociResistanceAxis, FociResistanceProfile, FociResistanceRun};
    use ankyra::ScratchOutput;
    use ankyra::encoding::Writable;

    /// Klipper's wire-frame payload budget: `MESSAGE_MAX` (64) minus the
    /// 2-byte header and 3-byte trailer. See
    /// `ankyra/ankyra/src/transport.rs` `max_valid_frame_emits_bytes`. This
    /// budget covers the full frame body that `encode_frame` writes, which is
    /// the reply-id `u16` followed by the reply payload (see
    /// `ankyra-assemble/src/senders.rs::emit_sender_impl`), so the asserted
    /// total below includes a worst-case id encoding, not payload alone.
    const KLIPPER_MESSAGE_PAYLOAD_MAX: usize = 59;

    /// Worst-case VLQ width, in bytes, of the `u16` reply id that
    /// `encode_frame` prepends to every reply payload. Klipper's VLQ encoder
    /// (`ankyra::encoding::encode_vlq_int`) reinterprets its `u32` input as
    /// `i32` and range-checks that signed value, so the worst case for a
    /// `u16`-typed value is `u16::MAX` (65535), which falls outside the
    /// 2-byte range `[-4096, 12288)` and encodes to 3 bytes.
    const REPLY_ID_WORST_CASE_BYTES: usize = 3;

    /// Encode `reply` and return the actual wire-payload byte count.
    fn encoded_len<T: Writable>(reply: &T) -> usize {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        scratch.result().len()
    }

    /// Assert that `reply`'s encoded payload, plus a worst-case reply-id
    /// prefix, fits Klipper's wire-frame budget.
    fn assert_fits_with_id<T: Writable>(reply: &T, reply_kind: &str) {
        let payload_len = encoded_len(reply);
        let total_len = payload_len + REPLY_ID_WORST_CASE_BYTES;
        assert!(
            total_len <= KLIPPER_MESSAGE_PAYLOAD_MAX,
            "{reply_kind} encodes to {payload_len} payload bytes + {REPLY_ID_WORST_CASE_BYTES} \
             worst-case id bytes = {total_len}, exceeds {KLIPPER_MESSAGE_PAYLOAD_MAX}"
        );
    }

    #[test]
    fn resistance_profile_reply_fits_klipper_message_limit() {
        // True worst-case VLQ encoding per field type. Klipper's VLQ encoder
        // reinterprets the `u32`/`i32` payload as `i32` and range-checks
        // that signed value (`encode_vlq_int`), so `u32::MAX as i32 == -1`
        // collapses to a 1-byte encoding rather than the genuine 5-byte
        // worst case. `0x7FFF_FFFF` (`i32::MAX`) is outside all four VLQ
        // range checks and genuinely encodes to 5 bytes. `u16::MAX` and
        // `u8::MAX` are already correct worst cases for their domains (3
        // and 2 bytes respectively) because those types' maximum values
        // cannot reach the wider per-length VLQ thresholds.
        let reply = FociResistanceProfile {
            oid: u8::MAX,
            profile_version: u16::MAX,
            pwm_maxcnt: u16::MAX,
            bbm_h: u8::MAX,
            bbm_l: u8::MAX,
            dsadc_mdec_a: u16::MAX,
            dsadc_mdec_b: u16::MAX,
            linear_current_threshold_count: u16::MAX,
            encoder_move_warn_counts: u16::MAX,
            status_flags_warn_mask: 0x7FFF_FFFF,
            scale_metadata_validated: u8::MAX,
        };

        assert_fits_with_id(&reply, "foci_resistance_profile");
    }

    #[test]
    fn resistance_run_reply_fits_klipper_message_limit() {
        // See `resistance_profile_reply_fits_klipper_message_limit` for why
        // `u32` fields use `0x7FFF_FFFF` instead of `u32::MAX`. `i32::MIN` is
        // already a genuine 5-byte worst case for `i32` fields (it sits
        // outside all four VLQ range checks on the signed side).
        let reply = FociResistanceRun {
            oid: u8::MAX,
            status: u8::MAX,
            profile_version: u16::MAX,
            selected_r_count_slope_milli: i32::MIN,
            gain_path_count_slope_milli: i32::MIN,
            warning_flags: 0x7FFF_FFFF,
            status_flags_or: 0x7FFF_FFFF,
            pwm_maxcnt_readback: u16::MAX,
            bbm_readback: u16::MAX,
            dsadc_mdec_readback: 0x7FFF_FFFF,
            pwm_sv_chop_readback: 0x7FFF_FFFF,
        };

        assert_fits_with_id(&reply, "foci_resistance_run");
    }

    #[test]
    fn resistance_axis_reply_fits_klipper_message_limit() {
        // See `resistance_profile_reply_fits_klipper_message_limit` for why
        // `u32` fields use `0x7FFF_FFFF` instead of `u32::MAX`.
        let reply = FociResistanceAxis {
            oid: u8::MAX,
            electrical_axis: u8::MAX,
            phi_e_ext: u16::MAX,
            r_count_slope_milli: i32::MIN,
            intercept_count: i32::MIN,
            rmse_permille: u16::MAX,
            selected_mask: u16::MAX,
            excluded_point_mask: u16::MAX,
            selected_count: u8::MAX,
            signed_count_slope_milli: i32::MIN,
            signed_asymmetry_permille: u16::MAX,
            drift_permille: u16::MAX,
            warning_flags: 0x7FFF_FFFF,
        };

        assert_fits_with_id(&reply, "foci_resistance_axis");
    }

    #[test]
    fn current_loop_run_reply_fits_klipper_message_limit() {
        let reply = super::FociCurrentLoopRun {
            oid: u8::MAX,
            status: u8::MAX,
            gains_source: u8::MAX,
            candidate_gains_source: u8::MAX,
            axis_split_source: u8::MAX,
            candidate_axis_split_source: u8::MAX,
            gains_tier: u8::MAX,
            candidate_gains_tier: u8::MAX,
            measured_axis_split_permille: u16::MAX,
            candidate_measured_axis_split_permille: u16::MAX,
            applied_axis_split_permille: u16::MAX,
            candidate_applied_axis_split_permille: u16::MAX,
            axis_split_clamped: u8::MAX,
            candidate_axis_split_clamped: u8::MAX,
            candidate_flux_p: u16::MAX,
            candidate_flux_i: u16::MAX,
            candidate_torque_p: u16::MAX,
            candidate_torque_i: u16::MAX,
            candidate_attempt: u8::MAX,
            current_validation_axes: u8::MAX,
            flux_validation_sample_count: u8::MAX,
            torque_validation_sample_count: u8::MAX,
            retry_budget_exhausted: u8::MAX,
            failure_reason: u8::MAX,
        };
        assert_fits_with_id(&reply, "foci_current_loop_run");
    }

    #[test]
    fn encoder_alignment_reply_fits_klipper_message_limit() {
        let reply = super::FociEncoderAlignment {
            oid: u8::MAX,
            encoder_count: 0x7FFF_FFFF,
            electrical_residual_counts: u16::MAX,
            stability_counts: u16::MAX,
            movement_counts: u16::MAX,
            min_movement_counts: u16::MAX,
            counts_per_electrical_rev: u16::MAX,
        };
        assert_fits_with_id(&reply, "foci_encoder_alignment");
    }

    #[test]
    fn inductance_run_reply_fits_klipper_message_limit() {
        let reply = super::FociInductanceRun {
            oid: u8::MAX,
            source: u8::MAX,
            status: u8::MAX,
            warning_flags: u16::MAX,
            ud_count: i16::MIN,
            realized_frequency_millihz: 0x7FFF_FFFF,
            elapsed_us: 0x7FFF_FFFF,
            openloop_phi_delta_counts: 0x7FFF_FFFF,
            sample_count: u16::MAX,
            encoder_delta_counts: i32::MIN,
            status_flags_or: 0x7FFF_FFFF,
        };
        assert_fits_with_id(&reply, "foci_inductance_run");
    }

    #[test]
    fn inductance_frame_reply_fits_klipper_message_limit() {
        let reply = super::FociInductanceFrame {
            oid: u8::MAX,
            id_mean_milli_count: i32::MIN,
            iq_mean_milli_count: i32::MIN,
            id_rms_milli_count: 0x7FFF_FFFF,
            iq_rms_milli_count: 0x7FFF_FFFF,
            drift_permille: u16::MAX,
            zero_id_mean_milli_count: i32::MIN,
            zero_iq_mean_milli_count: i32::MIN,
        };
        assert_fits_with_id(&reply, "foci_inductance_frame");
    }

    #[test]
    fn inductance_estimate_reply_fits_klipper_message_limit() {
        let reply = super::FociInductanceEstimate {
            oid: u8::MAX,
            x_average_count_ratio_milli: 0x7FFF_FFFF,
            x_d_count_ratio_milli: 0x7FFF_FFFF,
            x_q_count_ratio_milli: 0x7FFF_FFFF,
            saliency_status: u8::MAX,
            saliency_permille: u16::MAX,
            x_mag_nominal_count_ratio_milli: 0x7FFF_FFFF,
            x_mag_shift_minus_permille: u16::MAX,
            x_mag_shift_plus_permille: u16::MAX,
            x_mag_vs_quad_permille: u16::MAX,
        };
        assert_fits_with_id(&reply, "foci_inductance_estimate");
    }

    #[test]
    fn current_validation_axis_reply_fits_klipper_message_limit() {
        let reply = super::FociCurrentValidationAxis {
            oid: u8::MAX,
            axis: u8::MAX,
            sample_index: u8::MAX,
            status: u8::MAX,
            attempt: u8::MAX,
            target: i16::MAX,
            sample_delay_ms: u16::MAX,
            positive_response_permille: u16::MAX,
            negative_response_permille: u16::MAX,
            cross_axis_permille: u16::MAX,
            cross_axis_peak_permille: u16::MAX,
            voltage_output_permille: u16::MAX,
            encoder_delta_counts: u16::MAX,
            positive_encoder_delta_counts: i32::MAX,
            negative_encoder_delta_counts: i32::MIN,
            status_flags_or: 0x7FFF_FFFF,
        };
        assert_fits_with_id(&reply, "foci_current_validation_axis");
    }
}

/// `foci_stepper_event` reply - bounded stepper runtime diagnostic.
///
/// Wire name: `foci_stepper_event`.
#[klipper_reply]
pub struct FociStepperEvent {
    /// Reason code from `foci_firmware::diagnostic::StepperEventReason`.
    pub reason: u8,
    /// Board-local channel.
    pub channel: u8,
    /// Firmware step counter at event time.
    pub position: i32,
    /// TIM5/Klipper clock at event time.
    pub clock: u32,
    /// Non-zero when the board believes the step timer is active.
    pub timer_active: u8,
    /// Board-local queue length, or 65535 when unknown.
    pub queue_len: u16,
    /// Current direction state.
    pub direction: u8,
    /// Reason-specific value.
    pub data0: u32,
    /// Reason-specific value.
    pub data1: u32,
}

/// `foci_stepper_perf_event` reply - fatal dispatch performance snapshot.
///
/// Wire name: `foci_stepper_perf_event`.
#[klipper_reply]
pub struct FociStepperPerfEvent {
    /// Reason code from `foci_firmware::diagnostic::StepperEventReason`.
    pub reason: u8,
    /// Board-local channel.
    pub channel: u8,
    /// TIM5/Klipper clock at event time.
    pub clock: u32,
    /// Total profiler samples accumulated since the last reset.
    pub sample_count: u32,
    /// Trigger-domain critical sections measured since the last reset.
    pub crit_count: u32,
    /// Longest trigger-domain critical section in CPU cycles.
    pub crit_max_cycles: u32,
    /// Site code that produced `crit_max_cycles`.
    pub crit_max_site: u8,
    /// Critical sections longer than 10 us.
    pub crit_over_10us: u32,
    /// Critical sections longer than 50 us.
    pub crit_over_50us: u32,
    /// Critical sections longer than 100 us.
    pub crit_over_100us: u32,
    /// Critical sections longer than 1000 us.
    pub crit_over_1000us: u32,
    /// `queue_step` trigger-domain sections measured since the last reset.
    pub queue_step_count: u32,
    /// Longest `queue_step` trigger-domain section in CPU cycles.
    pub queue_step_max_cycles: u32,
    /// TIM5 IRQ dispatch activations since the last reset.
    pub tim5_activation_count: u32,
    /// Longest TIM5 IRQ handler duration in CPU cycles.
    pub tim5_irq_max_cycles: u32,
    /// Longest TIM5 scheduler dispatch duration in CPU cycles.
    pub tim5_dispatch_max_cycles: u32,
    /// Events processed in the activation that set `tim5_dispatch_max_cycles`.
    pub tim5_dispatch_max_cycles_events: u32,
    /// Largest number of due events processed by one TIM5 IRQ.
    pub tim5_events_max_per_irq: u32,
    /// Total scheduler events processed by TIM5 since the last reset.
    pub tim5_event_count_total: u32,
    /// TIM5 activations that hit the repeat guard and deferred.
    pub tim5_defer_count: u32,
    /// TIM5 activations that processed no scheduler event.
    pub tim5_empty_count: u32,
    /// Events processed in the most recent TIM5 activation.
    pub tim5_events_last_activation: u32,
    /// Max per-activation `dispatch_cycles / processed_events`.
    pub tim5_burst_cycles_per_event_max: u32,
    /// Raw dispatch cycles for the activation that set `tim5_burst_cycles_per_event_max`.
    pub tim5_burst_cycles_per_event_max_cycles: u32,
    /// Events processed by the activation that set `tim5_burst_cycles_per_event_max`.
    pub tim5_burst_cycles_per_event_max_events: u32,
    /// Max per-activation cycles/event for activations with at least three events.
    pub tim5_burst_cycles_per_event_floor3_max: u32,
    /// Max TIM5 entry lateness in TIM5 ticks.
    pub tim5_entry_latency_max_ticks: u32,
    /// Max positive scheduler pop lateness in TIM5 ticks.
    pub tim5_pop_lateness_max_ticks: u32,
    /// Longest TIM5-dispatch scheduler bookkeeping duration in CPU cycles.
    pub scheduler_cycles_max: u32,
    /// Events processed in the activation that set `scheduler_cycles_max`.
    pub scheduler_cycles_events_at_max: u32,
    /// Max per-activation `scheduler_cycles / processed_events`.
    pub scheduler_cycles_per_event_max: u32,
    /// Raw scheduler cycles for the activation that set the per-event max.
    pub scheduler_cycles_per_event_max_cycles: u32,
    /// Events processed by the activation that set the per-event max.
    pub scheduler_cycles_per_event_max_events: u32,
    /// Max scheduler cycles/event for activations with at least three events.
    pub scheduler_cycles_per_event_floor3_max: u32,
    /// Scheduler insertions dropped because the fixed table was full.
    pub scheduler_full_count: u32,
    /// Max positive stepper segment-load lateness in TIM5 ticks.
    pub stepper_load_lateness_max_ticks: u32,
    /// Last stepper segment-load lateness as signed TIM5 ticks.
    pub stepper_load_lateness_last_ticks: i32,
    /// Non-zero when firmware was built with trace instrumentation.
    pub build_trace_enabled: u8,
}

/// `foci_stepper_stats_result` reply - step queue/execution counters.
///
/// Wire name: `foci_stepper_stats_result`.
#[klipper_reply]
pub struct FociStepperStatsResult {
    /// Object id of the queried FOCI endpoint.
    pub oid: u8,
    /// Board-local channel.
    pub channel: u8,
    /// Firmware step counter at query time.
    pub position: i32,
    /// Accepted `queue_step` segments since boot.
    pub queued_segments: u32,
    /// Accepted motor steps since boot.
    pub queued_steps: u32,
    /// Segments loaded by the timer ISR since boot.
    pub loaded_segments: u32,
    /// Motor steps loaded by the timer ISR since boot.
    pub loaded_steps: u32,
    /// Segments discarded because the stepper awaited `reset_step_clock`.
    pub discarded_segments: u32,
    /// Motor steps discarded because the stepper awaited `reset_step_clock`.
    pub discarded_steps: u32,
    /// Non-zero when the board believes the step timer is active.
    pub timer_active: u8,
    /// Current board-local queue length.
    pub queue_len: u16,
}

/// `foci_stepper_exec_stats_result` reply - step execution counters.
///
/// Wire name: `foci_stepper_exec_stats_result`.
#[klipper_reply]
pub struct FociStepperExecStatsResult {
    /// Object id of the queried FOCI endpoint.
    pub oid: u8,
    /// Board-local channel.
    pub channel: u8,
    /// Executed positive-direction motor steps since boot.
    pub executed_pos_steps: u32,
    /// Executed negative-direction motor steps since boot.
    pub executed_neg_steps: u32,
    /// Times the step timer went idle because the queue was empty.
    pub queue_empty_count: u32,
    /// Combined missed-deadline count from segment-load and in-segment checks.
    pub missed_deadline_count: u32,
}

/// `foci_stepper_timing_stats_result` reply - first-step timing counters.
///
/// Wire name: `foci_stepper_timing_stats_result`.
#[klipper_reply]
pub struct FociStepperTimingStatsResult {
    /// Object id of the queried FOCI endpoint.
    pub oid: u8,
    /// Board-local channel.
    pub channel: u8,
    /// Number of times the board armed the step timer.
    pub activation_count: u32,
    /// TIM5 clock when the step timer was most recently armed.
    pub last_activation_clock: u32,
    /// TIM5 clock when the first segment of the last activation loaded.
    pub first_load_now: u32,
    /// Scheduled first-step clock from the first loaded segment.
    pub first_load_scheduled: u32,
    /// Compare clock selected for the first loaded segment.
    pub first_load_compare: u32,
    /// `first_load_scheduled - first_load_now`, interpreted as signed ticks.
    pub first_load_lead_ticks: i32,
    /// `first_load_compare - first_load_now`, interpreted as signed ticks.
    pub first_load_compare_delay_ticks: i32,
    /// TIM5 clock observed at the first rising edge of the last activation.
    pub first_step_clock: u32,
    /// `first_step_clock - last_activation_clock`, interpreted as signed ticks.
    pub first_step_delay_ticks: i32,
}

/// `foci_stepper_perf_stats_result` reply - dispatch performance counters.
///
/// Wire name: `foci_stepper_perf_stats_result`.
#[klipper_reply]
pub struct FociStepperPerfStatsResult {
    /// Object id of the queried FOCI endpoint.
    pub oid: u8,
    /// Board-local channel.
    pub channel: u8,
    /// Longest trigger-domain critical section in CPU cycles.
    pub crit_max_cycles: u32,
    /// Site code that produced `crit_max_cycles`.
    pub crit_max_site: u8,
    /// Critical sections longer than 10 us.
    pub crit_over_10us: u32,
    /// Critical sections longer than 50 us.
    pub crit_over_50us: u32,
    /// Critical sections longer than 100 us.
    pub crit_over_100us: u32,
    /// Critical sections longer than 1000 us.
    pub crit_over_1000us: u32,
    /// `queue_step` trigger-domain sections measured since the last reset.
    pub queue_step_count: u32,
    /// Longest `queue_step` trigger-domain section in CPU cycles.
    pub queue_step_max_cycles: u32,
    /// TIM5 IRQ dispatch activations since the last reset.
    pub tim5_activation_count: u32,
    /// Longest TIM5 IRQ handler duration in CPU cycles.
    pub tim5_irq_max_cycles: u32,
    /// Longest TIM5 scheduler dispatch duration in CPU cycles.
    pub tim5_dispatch_max_cycles: u32,
    /// Events processed in the activation that set `tim5_dispatch_max_cycles`.
    pub tim5_dispatch_max_cycles_events: u32,
    /// Largest number of due events processed by one TIM5 IRQ.
    pub tim5_events_max_per_irq: u32,
    /// Total scheduler events processed by TIM5 since the last reset.
    pub tim5_event_count_total: u32,
    /// TIM5 activations that hit the repeat guard and deferred.
    pub tim5_defer_count: u32,
    /// Raw dispatch cycles for the activation that set `tim5_burst_cycles_per_event_max`.
    pub tim5_burst_cycles_per_event_max_cycles: u32,
    /// Events processed by the activation that set `tim5_burst_cycles_per_event_max`.
    pub tim5_burst_cycles_per_event_max_events: u32,
    /// Max per-activation cycles/event for activations with at least three events.
    pub tim5_burst_cycles_per_event_floor3_max: u32,
    /// Max TIM5 entry lateness in TIM5 ticks.
    pub tim5_entry_latency_max_ticks: u32,
    /// Max positive scheduler pop lateness in TIM5 ticks.
    pub tim5_pop_lateness_max_ticks: u32,
    /// Longest TIM5-dispatch scheduler bookkeeping duration in CPU cycles.
    pub scheduler_cycles_max: u32,
    /// Events processed in the activation that set `scheduler_cycles_max`.
    pub scheduler_cycles_events_at_max: u32,
    /// Max per-activation `scheduler_cycles / processed_events`.
    pub scheduler_cycles_per_event_max: u32,
    /// Max scheduler cycles/event for activations with at least three events.
    pub scheduler_cycles_per_event_floor3_max: u32,
    /// Scheduler insertions dropped because the fixed table was full.
    pub scheduler_full_count: u32,
    /// Max positive stepper segment-load lateness in TIM5 ticks.
    pub stepper_load_lateness_max_ticks: u32,
    /// Last stepper segment-load lateness as signed TIM5 ticks.
    pub stepper_load_lateness_last_ticks: i32,
    /// Non-zero when firmware was built with trace instrumentation.
    pub build_trace_enabled: u8,
}

/// `foci_stepper_stop_stats_result` reply - physical stop/reset counters.
///
/// Wire name: `foci_stepper_stop_stats_result`.
#[klipper_reply]
pub struct FociStepperStopStatsResult {
    /// Object id of the queried FOCI endpoint.
    pub oid: u8,
    /// Board-local channel.
    pub channel: u8,
    /// Physical stepper stop count from TrSync or P1 stop paths.
    pub stop_count: u32,
    /// Queued segments drained by physical stop paths since boot.
    pub stop_drained_segments: u32,
    /// Queued motor steps drained by physical stop paths since boot.
    pub stop_drained_steps: u32,
    /// Accepted `reset_step_clock` count.
    pub reset_count: u32,
    /// Queued segments drained by `reset_step_clock` since boot.
    pub reset_drained_segments: u32,
    /// Queued motor steps drained by `reset_step_clock` since boot.
    pub reset_drained_steps: u32,
    /// `StepperEventReason` code for the most recent physical stop.
    pub last_stop_reason: u8,
    /// Active timer events remaining at the most recent physical stop.
    pub last_stop_remaining_events: u32,
    /// Queue length observed at the most recent physical stop.
    pub last_stop_queue_len: u16,
    /// Queued segments drained by the most recent physical stop.
    pub last_stop_drained_segments: u32,
    /// Queued motor steps drained by the most recent physical stop.
    pub last_stop_drained_steps: u32,
}

/// `foci_selftest_done` reply — emitted at the end of a self-test run.
///
/// Wire name: `foci_selftest_done`.
#[klipper_reply]
pub struct FociSelftestDone {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code (0 = success, non-zero = error code).
    pub status: u8,
}

/// `foci_selftest_result` reply — emitted once per self-test stage
/// carrying the per-stage measurement.
///
/// Wire name: `foci_selftest_result`.
#[klipper_reply]
pub struct FociSelftestResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Stage identifier within the self-test sequence.
    pub stage: u8,
    /// Status code for this stage (0 = success, non-zero = error code).
    pub status: u8,
    /// Stage-specific measurement.
    pub value: u32,
}

/// `foci_tune_result` reply — final outer-loop tune output.
///
/// Wire name: `foci_tune_result`.
#[klipper_reply]
pub struct FociTuneResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Status code (0 = success, non-zero = error code).
    pub status: u8,
    /// Velocity P gain.
    pub velocity_p: u16,
    /// Velocity I gain.
    pub velocity_i: u16,
    /// Velocity limit.
    pub velocity_limit: u32,
    /// Position P gain.
    pub position_p: u16,
    /// Position I gain.
    pub position_i: u16,
    /// Velocity filter cutoff in Hz.
    pub velocity_filter_hz: u16,
    /// Position filter cutoff in Hz.
    pub position_filter_hz: u16,
    /// Flux filter cutoff in Hz.
    pub flux_filter_hz: u16,
    /// Torque filter cutoff in Hz.
    pub torque_filter_hz: u16,
    /// Effective rotor moment of inertia.
    pub j_eff: u32,
    /// Effective viscous friction.
    pub b_eff: u32,
    /// Warning bitfield for non-fatal tune anomalies.
    pub warning_code: u8,
    /// Mechanical torque step used during the tune.
    pub mech_torque_step: u16,
    /// Forward acceleration measurement.
    pub mech_fwd_accel: u32,
    /// Reverse acceleration measurement.
    pub mech_rev_accel: u32,
    /// Forward peak velocity measurement.
    pub mech_fwd_peak_velocity: u32,
    /// Reverse peak velocity measurement.
    pub mech_rev_peak_velocity: u32,
    /// Forward travel measurement.
    pub mech_fwd_travel: u32,
    /// Reverse travel measurement.
    pub mech_rev_travel: u32,
    /// Synthesized velocity P gain.
    pub synth_velocity_p: u16,
    /// Synthesized velocity I gain.
    pub synth_velocity_i: u16,
    /// Synthesized position P gain.
    pub synth_position_p: u16,
    /// Synthesized position I gain.
    pub synth_position_i: u16,
}

/// `spi_transfer_response` reply — echo of a completed SPI transfer.
///
/// Carries an arbitrary `&[u8]` response, so the struct is generic
/// over the borrow lifetime.
///
/// Wire name: `spi_transfer_response`.
#[klipper_reply]
pub struct SpiTransferResponse<'a> {
    /// Object id of the SPI device.
    pub oid: u8,
    /// Bytes read from the SPI device during the transfer.
    pub response: &'a [u8],
}

/// `stats` reply — periodic MCU heartbeat consumed by Klipper to compute
/// `mcu_awake`, `mcu_task_avg`, and `mcu_task_stddev` for the host's
/// per-MCU summary line. Must be a reply (not an output) so klippy's
/// `register_response(self._handle_mcu_stats, 'stats')` callback fires;
/// outputs route to the `#output:` debug log instead.
///
/// Wire name: `stats`. Field semantics match upstream Klipper's
/// `sendf("stats count=%u sum=%u sumsq=%u", ...)` in
/// `src/basecmd.c::stats_update`. Note: today's accumulator in
/// `boards/openffboard-fw/src/heartbeat.rs` emits coarse values
/// (`count` always 1, `sum` is wall time, `sumsq` saturated); accurate
/// busy/total tick reporting is tracked in
/// `docs/specs/2026-03-29-foci-design.md` Phase 5.
#[klipper_reply]
pub struct Stats {
    /// Number of accumulator iterations since the last emission.
    pub count: u32,
    /// Sum of per-iteration busy clock-tick deltas.
    pub sum: u32,
    /// Sum of squared per-iteration deltas, scaled by `STATS_SUMSQ_BASE`.
    pub sumsq: u32,
}

/// `stepper_position` reply — current commanded stepper position.
///
/// Wire name: `stepper_position`.
#[klipper_reply]
pub struct StepperPosition {
    /// Object id of the stepper.
    pub oid: u8,
    /// Signed stepper position in steps.
    pub pos: i32,
}

/// `tmc_register_value` reply — value read from a TMC4671 register.
///
/// Wire name: `tmc_register_value`.
#[klipper_reply]
pub struct TmcRegisterValue {
    /// Object id of the TMC4671 instance.
    pub oid: u8,
    /// Register address.
    pub addr: u8,
    /// Register value.
    pub value: u32,
}

/// `trsync_state` reply — emitted by the trigger subsystem as trsync
/// snapshots are drained.
///
/// Ankyra emits the wire name `trsync_state` via the automatic
/// PascalCase-to-snake_case conversion.
#[klipper_reply]
pub struct TrsyncState {
    /// Object id of the trsync endpoint reporting the snapshot.
    pub oid: u8,
    /// Non-zero while the trsync can still fire its trigger.
    pub can_trigger: u8,
    /// Reason code captured when the trsync transitioned out of
    /// `can_trigger`.
    pub trigger_reason: u8,
    /// Clock tick at which the snapshot was taken.
    pub clock: u32,
}

/// `uptime` reply — high/low 64-bit systick reading.
///
/// Wire name: `uptime`.
#[klipper_reply]
pub struct Uptime {
    /// High 32 bits of the MCU systick counter.
    pub high: u32,
    /// Low 32 bits of the MCU systick counter.
    pub clock: u32,
}
