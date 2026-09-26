//! `#[klipper_reply]` struct declarations shared by every board.
//!
//! Each struct in this module is a reply shape that a `#[klipper_command]`
//! handler (or a background reply emitter) sends back to the
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
//! the data-dictionary format string in the same order, so reordering a
//! struct's fields changes the wire format.

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

/// `foci_motion_scale_rejected` reply emitted immediately before shutdown.
///
/// Wire name: `foci_motion_scale_rejected`.
#[klipper_reply]
pub struct FociMotionScaleRejected {
    /// Object id of the FOCI endpoint whose scale was rejected.
    pub oid: u8,
    /// Task stage that detected the rejection (`0` = P2, `1` = P1).
    pub stage: u8,
    /// Cause code from [`crate::motion_scale::MotionScaleRejection`].
    pub reason: u8,
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

/// `analog_in_state` reply — emitted by `query_analog_in`.
///
/// Wire name: `analog_in_state`.
#[klipper_reply]
pub struct AnalogInState {
    /// Object id of the analog-in channel reporting a sample.
    pub oid: u8,
    /// Next clock at which the channel will report again.
    pub next_clock: u32,
    /// Summed raw ADC value for the completed burst.
    pub value: u16,
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

/// `foci_commission_timing` reply — timing quality for one electrical method.
#[klipper_reply]
pub struct FociCommissionTiming {
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
    /// Resistance count slope in milli count-ratio units.
    pub r_count_milli: u32,
    /// Count-space inductance proxy in micro-time units.
    pub l_count_micro: u32,
    /// Flux P gain.
    pub flux_p: u16,
    /// Flux I gain.
    pub flux_i: u16,
    /// Torque P gain.
    pub torque_p: u16,
    /// Torque I gain.
    pub torque_i: u16,
    /// Inner-loop time scale in microseconds.
    pub lambda_us: u32,
    /// Measured electrical time constant in microseconds.
    pub theta_e_us: u32,
    /// Source of the accepted `theta_e_us`: 0 = high-rate measurement, 1 =
    /// board-configured fallback.
    pub theta_source: u8,
    /// Closed-loop bandwidth in Hz, capped at `MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ`.
    pub bandwidth_hz: u16,
    /// Fallback velocity P gain used by the outer loop.
    pub fallback_velocity_p: u16,
    /// Fallback velocity I gain used by the outer loop.
    pub fallback_velocity_i: u16,
    /// Fallback position P gain used by the outer loop.
    pub fallback_position_p: u16,
    /// Inner-confidence warning bitfield.
    ///
    /// Bits 0, 1, and 3 report model warnings; bit 2 is reserved for tau
    /// residual telemetry and is not emitted as a warning. Bit 5 reports
    /// fallback current gains, bit 6 reports host-default confidence, and bit 7 is
    /// reserved (older firmware used it for a skipped gain floor).
    pub inner_warning_flags: u8,
    /// Packed R/L/delay timing states and worst-case metrics.
    pub timing_summary: u32,
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

/// `foci_current_loop_filters` reply - applied filter cutoffs for the
/// commissioned closed-loop current frame.
///
/// Wire name: `foci_current_loop_filters`.
#[klipper_reply]
pub struct FociCurrentLoopFilters {
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

/// `foci_closed_loop_activation` reply - fast closed-loop entry stability evidence.
///
/// Wire name: `foci_closed_loop_activation`.
#[klipper_reply]
pub struct FociClosedLoopActivation {
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

/// `foci_current_loop_hold` reply - sustained current-loop hold evidence.
///
/// Wire name: `foci_current_loop_hold`.
#[klipper_reply]
pub struct FociCurrentLoopHold {
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

/// `foci_stack_watermark_result` reply - MSP stack high-water evidence.
///
/// Reports the surviving run of the boot-time sentinel paint described in
/// [`crate::stack_watermark`]. `stack_unused_bytes` is headroom the stack never
/// consumed since boot, so the value is only meaningful once the workload under
/// test has run.
///
/// `status` says how to read it: 0 means the scan reached the first overwritten
/// word and the figure is a measurement, while 2 means the scan window ran out
/// with every word still holding the sentinel, making the figure a lower bound.
/// A lower bound still settles a `>=` check it already clears; it cannot settle
/// one it fails. Status 3 means the scan never ran because motion was live, and
/// the counters carry nothing.
///
/// Wire name: `foci_stack_watermark_result`.
#[klipper_reply]
pub struct FociStackWatermarkResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Bytes at the bottom of the painted span the stack never reached.
    pub stack_unused_bytes: u32,
    /// Total span painted at startup.
    pub painted_bytes: u32,
    /// Status code (0 = measured, 1 = never painted, 2 = lower bound,
    /// 3 = refused because motion was active).
    pub status: u8,
}

/// `foci_last_boot_diagnostic` reply - recovered previous-boot fault context.
///
/// Emitted once during the connect-time register dump when the board recovered
/// a panic, HardFault, or interrupted breakaway campaign from no-init RAM after
/// a reset. Lets a probe-free host localize the failure. `fault_kind` follows
/// `foci_firmware::diagnostic::last_panic_kind`; `line` locates a Rust panic,
/// `pc` a HardFault, and the breakaway fields carry the last typed campaign
/// boundary. A completed breadcrumb accompanies a fault but is quiet alone.
///
/// Wire name: `foci_last_boot_diagnostic`.
#[klipper_reply]
pub struct FociLastBootDiagnostic {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Fault origin: 0 = none, 1 = Rust panic, 2 = HardFault.
    pub fault_kind: u8,
    /// Source line of a captured Rust panic, or 0.
    pub line: u32,
    /// Faulting program counter of a captured HardFault, or 0.
    pub pc: u32,
    /// FNV-1a hash of the panic source file path (0 for a HardFault); match
    /// against hashed candidate paths to name the file.
    pub file_hash: u32,
    /// Last validated [`crate::diagnostic::BreakawayCheckpoint`] code, or 0.
    pub breakaway_checkpoint: u8,
    /// Nonzero when reset interrupted an active breakaway campaign.
    pub breakaway_interrupted: u8,
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
    /// Driven axis: `CurrentStepAxis::as_u8()` of the axis this step targeted.
    pub axis: u8,
    /// Requested signed torque target in TMC4671 current counts.
    pub target: i16,
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
    /// Raw TMC4671 supply-voltage ADC sample during the dwell.
    pub adc_vm_raw: u16,
}

/// `foci_current_torque_sample_pid_result` reply - non-terminal PID monitor fragment.
///
/// Emitted before the detail and terminal fragments when the sample succeeded.
/// Wire name: `foci_current_torque_sample_pid_result`.
#[klipper_reply]
pub struct FociCurrentTorqueSamplePidResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sequence shared by every fragment and the terminal of this report.
    pub report_seq: u8,
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
    /// TMC torque current-loop error at the sample point.
    pub torque_error: i32,
    /// TMC flux current-loop error at the sample point.
    pub flux_error: i32,
    /// TMC torque current-loop error sum at the sample point.
    pub torque_error_sum: i32,
    /// TMC flux current-loop error sum at the sample point.
    pub flux_error_sum: i32,
}

/// `foci_current_torque_sample_result` reply - torque-pulse sample terminal result.
///
/// Wire name: `foci_current_torque_sample_result`.
#[klipper_reply]
pub struct FociCurrentTorqueSampleResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sequence shared by every fragment and the terminal of this report.
    pub report_seq: u8,
    /// Which torque-sample diagnostic produced this report: `TorqueSampleKind::as_u8()`.
    pub kind: u8,
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
    /// Raw TMC4671 supply-voltage ADC sample at the sample point.
    pub adc_vm_raw: u16,
    /// TMC4671 status flags at the sample point.
    pub status_flags: u32,
}

/// `foci_current_torque_sample_detail_result` reply - non-terminal
/// encoder/prelimit/feedforward fragment.
///
/// Wire name: `foci_current_torque_sample_detail_result`.
#[klipper_reply]
pub struct FociCurrentTorqueSampleDetailResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sequence shared by every fragment and the terminal of this report.
    pub report_seq: u8,
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
    /// Pre-limit UQ at the sample point.
    pub uq_prelimit: i16,
    /// Pre-limit UD at the sample point.
    pub ud_prelimit: i16,
    /// TMC velocity feedforward monitor at the sample point.
    pub ff_velocity: i32,
    /// TMC torque feedforward monitor at the sample point.
    pub ff_torque: i16,
}

/// `foci_voltage_step_result` reply - bounded open-loop voltage diagnostic terminal.
///
/// Wire name: `foci_voltage_step_result`.
#[klipper_reply]
pub struct FociVoltageStepResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sequence shared by the detail fragment and this terminal.
    pub report_seq: u8,
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
    /// Raw TMC4671 supply-voltage ADC sample at the sample point.
    pub adc_vm_raw: u16,
    /// TMC4671 status flags at the sample point.
    pub status_flags: u32,
}

/// `foci_voltage_step_detail_result` reply - non-terminal encoder fragment.
///
/// Emitted before the terminal, only when the step succeeded.
/// Wire name: `foci_voltage_step_detail_result`.
#[klipper_reply]
pub struct FociVoltageStepDetailResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sequence shared by this fragment and the terminal.
    pub report_seq: u8,
    /// ABN decoder count before applying the pulse.
    pub encoder_before: u32,
    /// ABN decoder count at the sample point.
    pub encoder_sample: u32,
    /// ABN decoder count after returning to position mode.
    pub encoder_after: u32,
}

/// `foci_resistance_profile` reply - board/profile constants for display.
///
/// Wire name: `foci_resistance_profile`.
#[klipper_reply]
pub struct FociResistanceProfile {
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
    /// Non-zero when sample-level containment synchronously tripped the power stage.
    pub power_stage_tripped: u8,
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
mod resistance_reply_size_tests {
    use super::{FociResistanceRun, FociRobustnessCycleEvidence};
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
    fn resistance_run_current_evidence_has_stable_wire_order() {
        let reply = FociResistanceRun {
            oid: 1,
            status: 2,
            selected_r_count_slope_milli: 3,
            gain_path_count_slope_milli: 4,
            warning_flags: 5,
            status_flags_or: 6,
            peak_abs_current_count: 7,
            max_abs_steady_mean_current_count: 8,
            current_ceiling_count: 9,
            power_stage_tripped: 10,
            pwm_maxcnt_readback: 11,
            bbm_readback: 12,
            dsadc_mdec_readback: 13,
            pwm_sv_chop_readback: 14,
        };
        let mut scratch = ScratchOutput::<64>::new();

        reply.write(&mut scratch);

        assert_eq!(
            scratch.result(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        );
    }

    #[test]
    fn robustness_cycle_evidence_reply_fits_klipper_message_limit() {
        let payload = [u8::MAX;
            crate::commissioning::outer::velocity::robustness_reporting::
                ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES];
        let reply = FociRobustnessCycleEvidence {
            oid: u8::MAX,
            payload: &payload,
        };
        assert_fits_with_id(&reply, "foci_robustness_cycle_evidence");
    }
}

#[cfg(test)]
mod breakaway_reply_wire_tests {
    //! REPLAY-COMPAT wire fixtures for the breakaway-campaign reply family.
    //! Each `*_has_stable_wire_order` test uses small ascending
    //! field values (every field type here VLQ-encodes values `1..=16` as a
    //! single raw byte, mirroring `resistance_run_current_evidence_has_stable_wire_order`
    //! above) so the expected byte array is exactly the field values in
    //! struct-declaration order -- a byte-for-byte pin on this schema's wire
    //! shape. None of these are new schema-13-and-older records: every field
    //! list here is additive to the reply table, so no existing reply's wire
    //! order changes.
    use super::{
        FociBreakawayCampaignTerminal, FociBreakawayConfirmationPlan,
        FociBreakawayConfirmationTerminalIdentity, FociBreakawayConfirmationTerminalMasks,
        FociBreakawayDiscoveryPlanGeometry, FociBreakawayDiscoveryPlanIdentity,
        FociBreakawayDiscoveryTerminal, FociBreakawayProbePlan, FociBreakawayProbeResult,
        FociBreakawayProbeTerminal, FociBreakawayRungZeroDiagnostic,
    };
    use ankyra::ScratchOutput;
    use ankyra::encoding::Writable;

    fn assert_wire_bytes<T: Writable>(reply: &T, expected: &[u8]) {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        assert_eq!(scratch.result(), expected);
    }

    #[test]
    fn breakaway_probe_plan_has_stable_wire_order() {
        let reply = FociBreakawayProbePlan {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            max_observations: 6,
            search_count: 7,
            p_start_raw: 8,
            p_top_raw: 9,
            motion_threshold_counts: 10,
            max_capture_interval_us: 11,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
    }

    #[test]
    fn breakaway_probe_result_has_stable_wire_order() {
        let reply = FociBreakawayProbeResult {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            rung_index: 6,
            breakaway_p_raw: 7,
            motion_threshold_counts: 8,
            observation_count: 9,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    }

    #[test]
    fn breakaway_probe_terminal_has_stable_wire_order() {
        let reply = FociBreakawayProbeTerminal {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            terminal_cause: 6,
            has_safety_fault: 7,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn breakaway_discovery_plan_identity_has_stable_wire_order() {
        let reply = FociBreakawayDiscoveryPlanIdentity {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn breakaway_discovery_plan_geometry_has_stable_wire_order() {
        let reply = FociBreakawayDiscoveryPlanGeometry {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            schema_revision: 4,
            rung_count: 5,
            observations_per_direction: 6,
            floor_p_raw: 7,
            floor_origin: 8,
            breakaway_p_raw: 9,
            ceiling_p_raw: 10,
            first_additive_step_raw: 11,
            maximum_workflow_ms: 12,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
    }

    #[test]
    fn breakaway_rung_zero_diagnostic_has_stable_wire_order() {
        let reply = FociBreakawayRungZeroDiagnostic {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            forward_moved: 6,
            reverse_moved: 7,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn breakaway_discovery_terminal_has_stable_wire_order() {
        let reply = FociBreakawayDiscoveryTerminal {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
            terminal_cause: 9,
            collected_count: 10,
            has_safety_fault: 11,
            stop_rung_index: 12,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
    }

    #[test]
    fn breakaway_confirmation_plan_has_stable_wire_order() {
        let reply = FociBreakawayConfirmationPlan {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
            observations_per_direction: 9,
            nominated_p_raw: 10,
            band_lower_percent: 11,
            band_upper_percent: 12,
            capture_profile: 13,
            acceptance_rule: 14,
            nominated_margin_percent_milli: 15,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
    }

    #[test]
    fn breakaway_confirmation_terminal_identity_has_stable_wire_order() {
        let reply = FociBreakawayConfirmationTerminalIdentity {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
            terminal_cause: 9,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    }

    #[test]
    fn breakaway_confirmation_terminal_masks_has_stable_wire_order() {
        let reply = FociBreakawayConfirmationTerminalMasks {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            forward_collected_mask: 4,
            forward_eligible_mask: 5,
            forward_included_mask: 6,
            reverse_collected_mask: 7,
            reverse_eligible_mask: 8,
            reverse_included_mask: 9,
            accepted: 10,
            confirmed_p_raw: 11,
            max_relative_se_permille: 12,
            required_relative_se_permille: 13,
            has_safety_fault: 14,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]);
    }

    #[test]
    fn breakaway_campaign_terminal_has_stable_wire_order() {
        let reply = FociBreakawayCampaignTerminal {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            phase: 4,
            terminal_cause: 5,
            accepted: 6,
            integral_plan_digest_low: 7,
            integral_plan_digest_high: 8,
            error_code: 9,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
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
    /// Frames dropped since boot for exceeding the 64-byte wire limit.
    pub oversize_frame_drops: u32,
}

/// `foci_stepper_exec_stats_result` reply - step execution counters.
///
/// Wire name: `foci_stepper_exec_stats_result`.
///
/// `channel`, `encoder_counts_per_rev`, `tmc_grid`, `physical_step_width`,
/// `motion_scale_configured`, `step_half_period_ticks`, `dir_setup_ticks`,
/// `waveform_worst_case_ticks` and `fatal_lateness_ticks` are dropped from the wire: the host
/// derives them from `channel` (already delivered by `foci_stepper_stats_result` in the same
/// sweep), `planner_steps_per_rev`, `encoder_ppr` and its own `CLOCK_FREQ` constant.
#[klipper_reply]
pub struct FociStepperExecStatsResult {
    /// Object id of the queried FOCI endpoint.
    pub oid: u8,
    /// Executed positive-direction logical motor steps since boot.
    pub executed_pos_steps: u32,
    /// Executed negative-direction logical motor steps since boot.
    pub executed_neg_steps: u32,
    /// Positive-direction physical STEP pulses emitted since boot, wrapping at `u32::MAX`.
    pub physical_pos_pulses: u32,
    /// Negative-direction physical STEP pulses emitted since boot, wrapping at `u32::MAX`.
    pub physical_neg_pulses: u32,
    /// Configured logical planner steps per mechanical revolution, or zero when unconfigured.
    pub planner_steps_per_rev: u32,
    /// Configured encoder pulses per mechanical revolution, or zero when unconfigured.
    pub encoder_ppr: u32,
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

// ============================================================================
// Breakaway-seeded campaign reporting
// ============================================================================

/// Immutable breakaway-probe plan evidence emitted before the first probe stroke.
#[klipper_reply]
pub struct FociBreakawayProbePlan {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low probe-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high probe-plan digest word.
    pub plan_digest_high: u32,
    /// Maximum observations permitted across the upward walk.
    pub max_observations: u8,
    /// Number of valid entries in the post-dedupe search grid.
    pub search_count: u8,
    /// First structural search-grid gain.
    pub p_start_raw: u16,
    /// Last structural search-grid gain.
    pub p_top_raw: u16,
    /// Minimum commanded-direction excursion that counts as motion.
    pub motion_threshold_counts: u32,
    /// Maximum accepted interval between encoder captures.
    pub max_capture_interval_us: u32,
}

/// Resolved repeatable-motion rung for the upward probe.
#[klipper_reply]
pub struct FociBreakawayProbeResult {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low probe-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high probe-plan digest word.
    pub plan_digest_high: u32,
    /// Zero-based rung in the realized grid.
    pub rung_index: u8,
    /// Proportional gain at the resolved rung.
    pub breakaway_p_raw: u16,
    /// Minimum commanded-direction excursion that counted as motion.
    pub motion_threshold_counts: u32,
    /// Total observations consumed across the walk.
    pub observation_count: u8,
}

/// Detailed terminal evidence for a probe campaign that did not resolve a breakaway.
#[klipper_reply]
pub struct FociBreakawayProbeTerminal {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low probe-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high probe-plan digest word.
    pub plan_digest_high: u32,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
}

/// Additive discovery plan identity: digests and confidence family.
#[klipper_reply]
pub struct FociBreakawayDiscoveryPlanIdentity {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low discovery-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high discovery-plan digest word.
    pub plan_digest_high: u32,
    /// Exact low probe-plan digest word.
    pub prior_plan_digest_low: u32,
    /// Exact high probe-plan digest word.
    pub prior_plan_digest_high: u32,
    /// Discovery confidence family: `rung_count * observations_per_direction`.
    pub family_size: u16,
}

/// Additive discovery plan geometry: floor, breakaway, ceiling, and step.
#[klipper_reply]
pub struct FociBreakawayDiscoveryPlanGeometry {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Breakaway-campaign schema/rule revision the ladder was resolved under.
    pub schema_revision: u16,
    /// Number of valid entries in the resolved additive ladder.
    pub rung_count: u8,
    /// Discovery per-rung, per-direction observation count.
    pub observations_per_direction: u16,
    /// Chosen diagnostic floor gain.
    pub floor_p_raw: u16,
    /// Origin of the diagnostic floor.
    pub floor_origin: u8,
    /// Resolved campaign breakaway gain.
    pub breakaway_p_raw: u16,
    /// Most restrictive applicable authority ceiling.
    pub ceiling_p_raw: u16,
    /// Fixed additive step from breakaway to the first rung strictly above it.
    pub first_additive_step_raw: u32,
    /// Reserved maximum campaign workflow duration.
    pub maximum_workflow_ms: u32,
}

// FociBreakawayRawObservationIdentity/Measurement were removed: the
// per-observation raw evidence is detailed evidence the host does not consume; it
// rides the canonical trace only (FTRC tag 5), where foci-trace reconstructs the
// consensus and nomination. It no longer rides the ordinary Klipper reply channel.

// FociBreakawayDiscoveryRungMargin was removed: the per-rung nomination
// margin is detailed evidence that foci-trace reconstructs from the streamed
// raw-observation trace records (marker 160), so it no longer rides the ordinary
// Klipper reply channel. The canonical trace record (tag 7) is retained.

/// Diagnostic evidence for whether the additive ladder's rung zero itself showed motion.
#[klipper_reply]
pub struct FociBreakawayRungZeroDiagnostic {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low discovery-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high discovery-plan digest word.
    pub plan_digest_high: u32,
    /// Nonzero when the forward-direction rung-zero observation showed motion.
    pub forward_moved: u8,
    /// Nonzero when the reverse-direction rung-zero observation showed motion.
    pub reverse_moved: u8,
}

/// Detailed terminal evidence for an additive discovery campaign.
#[klipper_reply]
pub struct FociBreakawayDiscoveryTerminal {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low discovery-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high discovery-plan digest word.
    pub plan_digest_high: u32,
    /// Exact low probe-plan digest word.
    pub prior_plan_digest_low: u32,
    /// Exact high probe-plan digest word.
    pub prior_plan_digest_high: u32,
    /// Discovery confidence family.
    pub family_size: u16,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
    /// Number of in-band rungs collected before the ladder stopped.
    pub collected_count: u8,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
    /// Rung index the discovery walk had reached when it stopped, whether by
    /// nominating, exhausting the ladder, or a discovery-owned fault.
    pub stop_rung_index: u8,
}

/// Immutable held-out confirmation plan evidence emitted before the first
/// confirmation stroke.
#[klipper_reply]
pub struct FociBreakawayConfirmationPlan {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low confirmation-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high confirmation-plan digest word.
    pub plan_digest_high: u32,
    /// Exact low discovery-plan digest word.
    pub prior_plan_digest_low: u32,
    /// Exact high discovery-plan digest word.
    pub prior_plan_digest_high: u32,
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
    /// Nomination screen's pointwise margin `M_i`, in percent * 1000.
    pub nominated_margin_percent_milli: i32,
}

// FociBreakawayConfirmationObservation was removed: the per-stroke
// confirmation response intervals are detailed trace-only evidence. The ordinary
// reply carries the confirmation terminal (with its mask structure), not the
// individual strokes; foci-trace reconstructs from the trace records.

/// Confirmation terminal identity: digests and confidence family.
#[klipper_reply]
pub struct FociBreakawayConfirmationTerminalIdentity {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Exact low confirmation-plan digest word.
    pub plan_digest_low: u32,
    /// Exact high confirmation-plan digest word.
    pub plan_digest_high: u32,
    /// Exact low discovery-plan digest word.
    pub prior_plan_digest_low: u32,
    /// Exact high discovery-plan digest word.
    pub prior_plan_digest_high: u32,
    /// Predeclared confirmation family size.
    pub family_size: u16,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
}

/// Confirmation terminal masks: collected/eligible/included per direction,
/// acceptance verdict, and measured/required standard error.
#[klipper_reply]
pub struct FociBreakawayConfirmationTerminalMasks {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Forward-direction collected mask.
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
#[klipper_reply]
pub struct FociBreakawayCampaignTerminal {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Phase that owned the campaign's terminal outcome.
    pub phase: u8,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
    /// Nonzero when the confirmed candidate was accepted and admitted to Integral.
    pub accepted: u8,
    /// Exact low Integral plan digest word.
    pub integral_plan_digest_low: u32,
    /// Exact high Integral plan digest word.
    pub integral_plan_digest_high: u32,
    /// `CommissionError::status_code()` for a campaign terminal that carries a
    /// causally-relevant error, or `0` when the terminal carries no error.
    pub error_code: u8,
}

/// Which `p_top` candidate bound the additive-discovery authority ceiling.
#[klipper_reply]
pub struct FociBreakawayCeilingSource {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Evidence sequence shared with the discovery plan this ceiling anchors.
    pub evidence_sequence: u16,
    /// Exact low discovery plan digest word.
    pub plan_digest_low: u32,
    /// Exact high discovery plan digest word.
    pub plan_digest_high: u32,
    /// `sweep_plan::CEILING_BINDING_CURRENT_LIMIT` or
    /// `sweep_plan::CEILING_BINDING_REPRESENTABILITY_CLAMP`.
    pub binding_source: u8,
}

/// Command-level commissioning workflow disclosed before any motion.
#[klipper_reply]
pub struct FociCommissioningWorkflowPlan {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned command sequence.
    pub run_sequence: u32,
    /// Proportional, Proportional-then-Integral, or Integral-resume discriminator.
    pub shape: u8,
    /// Exact low workflow-envelope digest word.
    pub digest_low: u32,
    /// Exact high workflow-envelope digest word.
    pub digest_high: u32,
    /// Expected duration under the fixed schedule.
    pub nominal_workflow_ms: u32,
    /// Conservative host-timeout authority.
    pub maximum_workflow_ms: u32,
}

/// Integral plan identity and reproduced proportional authority.
#[klipper_reply]
pub struct FociVelocityIntegralPlanCore {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared plan-group evidence sequence.
    pub evidence_sequence: u16,
    /// Plan fragment discriminator.
    pub fragment: u8,
    /// Exact low Integral plan digest word.
    pub plan_digest_low: u32,
    /// Exact high Integral plan digest word.
    pub plan_digest_high: u32,
    /// Exact low authorizing Proportional plan digest word.
    pub proportional_digest_low: u32,
    /// Exact high authorizing Proportional plan digest word.
    pub proportional_digest_high: u32,
    /// Analysis-rule contract this plan was built under.
    pub build_features: u16,
    /// Image identity of the firmware that produced this plan.
    pub build_identity: u32,
    /// Integral evidence schema revision.
    pub schema_revision: u16,
    /// Physical firmware channel.
    pub channel: u8,
    /// Reproduced fixed proportional gain.
    pub fixed_p: u16,
}

/// Integral plan geometry and controller-domain limits.
#[klipper_reply]
pub struct FociVelocityIntegralPlanGeometry {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared plan-group evidence sequence.
    pub evidence_sequence: u16,
    /// Plan fragment discriminator.
    pub fragment: u8,
    /// Frozen encoder-domain amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen mechanical-RPM target register.
    pub target_velocity_rpm: i32,
    /// Frozen PWM frequency.
    pub pwm_hz: u32,
    /// Mechanical encoder quadrature counts per revolution.
    pub encoder_counts_per_rev: u32,
    /// First exact positive-I rung.
    pub i_start: u16,
    /// Number of positive-I rungs.
    pub positive_rung_count: u8,
    /// Simultaneous observation family size per direction.
    pub family_size: u16,
    /// Total opening, positive, and closing rung count.
    pub total_rung_count: u8,
    /// Total planned observation count.
    pub expected_observations: u16,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
}

/// One direction of Proportional evidence authorizing Integral.
#[klipper_reply]
pub struct FociVelocityIntegralPlanAuthority {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared plan-group evidence sequence.
    pub evidence_sequence: u16,
    /// Direction code.
    pub direction: u8,
    /// Directional reproduced model membership.
    pub directional_membership: u32,
    /// Reproduced joint-operability membership.
    pub joint_membership: u32,
    /// Cross-run directional model-validity code.
    pub directional_validity: u8,
    /// Whether the selected operating component has reduced margin.
    pub reduced_margin: u8,
    /// Retained Proportional pooled lower edge, or schema-8 selected-rate lower edge.
    pub pooled_low_q16: i32,
    /// Retained Proportional pooled upper edge, or schema-8 selected-rate upper edge.
    pub pooled_high_q16: i32,
}

/// Integral fixed schedule and travel identity.
#[klipper_reply]
pub struct FociVelocityIntegralPlanTiming {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared plan-group evidence sequence.
    pub evidence_sequence: u16,
    /// Plan fragment discriminator.
    pub fragment: u8,
    /// Moving-stroke duration.
    pub moving_stroke_us: u32,
    /// Zero-target settle duration.
    pub zero_settle_us: u32,
    /// Cooperative analysis allowance.
    pub analysis_budget_us: u32,
    /// Expected fixed-schedule duration.
    pub nominal_workflow_ms: u32,
    /// Conservative fixed-schedule duration.
    pub maximum_workflow_ms: u32,
}

/// Integral travel and absolute-position identity.
#[klipper_reply]
pub struct FociVelocityIntegralPlanTravel {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared plan-group evidence sequence.
    pub evidence_sequence: u16,
    /// Plan fragment discriminator.
    pub fragment: u8,
    /// Per-stroke travel budget.
    pub max_stroke_travel_mrev: u32,
    /// Reserved settle/braking travel.
    pub settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom.
    pub negative_position_headroom_mrev: u32,
    /// Positive absolute-position headroom.
    pub positive_position_headroom_mrev: u32,
}

/// Integral rung-origin recovery geometry and reserved schedule.
#[klipper_reply]
pub struct FociVelocityIntegralPlanRecovery {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared plan-group evidence sequence.
    pub evidence_sequence: u16,
    /// Inclusive immutable-origin band.
    pub origin_band_counts: u32,
    /// Conservative nominal duration of one recovery slot.
    pub nominal_slot_us: u32,
    /// Host-declared maximum duration of one recovery slot.
    pub maximum_slot_us: u32,
    /// Maximum recovery-record count reserved by the plan.
    pub slot_count: u8,
    /// Schema-defined recovery policy flags.
    pub flags: u8,
}

/// One exact positive-I plan rung.
#[klipper_reply]
pub struct FociVelocityIntegralPlanRung {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared plan-group evidence sequence.
    pub evidence_sequence: u16,
    /// Zero-based positive-rung index.
    pub rung_index: u8,
    /// Exact raw velocity-I gain.
    pub i_raw: u16,
    /// Outward-ceiled advanced-PI time constant.
    pub tau_us: u32,
}

/// Immutable fixed-I fixed-gain amplitude plan.
#[klipper_reply]
pub struct FociFixedGainAmplitudePlan<'a> {
    /// Endpoint object id.
    pub oid: u8,
    /// Zero-based fragment index; the plan spans two fragments.
    pub fragment: u8,
    /// One fragment of the fixed-layout plan payload for its schema revision.
    pub payload: &'a [u8],
}

/// Compact fixed-I fixed-gain amplitude terminal.
#[klipper_reply]
pub struct FociFixedGainAmplitudeTerminal<'a> {
    /// Endpoint object id.
    pub oid: u8,
    /// Exact fixed-layout schema-1 payload.
    pub payload: &'a [u8],
}

/// Compact reversal-standstill robustness-gate terminal.
#[klipper_reply]
pub struct FociRobustnessReversalTerminal<'a> {
    /// Endpoint object id.
    pub oid: u8,
    /// Exact fixed-layout payload; length follows the current wire schema
    /// revision.
    pub payload: &'a [u8],
}

/// Compact single-reply Integral velocity-integral terminal.
#[klipper_reply]
pub struct FociVelocityIntegralTerminal<'a> {
    /// Endpoint object id.
    pub oid: u8,
    /// Exact fixed-layout payload; length follows the current wire schema
    /// revision.
    pub payload: &'a [u8],
}

/// Compact per-cycle reversal-standstill robustness-evidence reply.
///
/// Emitted once per direction, ahead of
/// [`FociRobustnessReversalTerminal`]: carries the off-terminal
/// residual/IAE medians plus one direction's K-cycle reconvergence,
/// overshoot, and forward-settle values.
#[klipper_reply]
pub struct FociRobustnessCycleEvidence<'a> {
    /// Endpoint object id.
    pub oid: u8,
    /// Exact fixed-layout payload; length follows the current wire schema
    /// revision.
    pub payload: &'a [u8],
}

#[cfg(test)]
mod velocity_sweep_reply_tests {
    use super::*;
    use ankyra::ScratchOutput;
    use ankyra::encoding::Writable;

    use crate::commissioning::outer::velocity::fixed_gain_amplitude_reporting::{
        FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES, FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES,
    };

    const PAYLOAD_WITH_ID_MAX: usize = 59;
    const FRAME_MAX: usize = 64;
    const REPLY_ID_WORST_CASE_BYTES: usize = 3;
    const FRAME_OVERHEAD_BYTES: usize = 5;

    fn assert_fits<T: Writable>(reply: &T, name: &str) {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let payload = scratch.result().len() + REPLY_ID_WORST_CASE_BYTES;
        assert!(
            payload <= PAYLOAD_WITH_ID_MAX,
            "{name} payload is {payload}"
        );
        assert!(
            payload + FRAME_OVERHEAD_BYTES <= FRAME_MAX,
            "{name} frame is {}",
            payload + FRAME_OVERHEAD_BYTES
        );
    }

    #[test]
    fn fixed_gain_amplitude_replies_fit_one_protocol_frame() {
        assert_fits(
            &FociFixedGainAmplitudePlan {
                oid: u8::MAX,
                fragment: u8::MAX,
                payload: &[u8::MAX; FIXED_GAIN_AMPLITUDE_PLAN_REPLY_FRAGMENT_BYTES],
            },
            "fixed-gain amplitude plan fragment",
        );
        assert_fits(
            &FociFixedGainAmplitudeTerminal {
                oid: u8::MAX,
                payload: &[u8::MAX; FIXED_GAIN_AMPLITUDE_TERMINAL_REPLY_BYTES],
            },
            "fixed-gain amplitude terminal",
        );
    }

    #[test]
    fn velocity_integral_terminal_fits_one_protocol_frame() {
        assert_fits(
            &FociVelocityIntegralTerminal {
                oid: u8::MAX,
                payload: &[u8::MAX;
                    crate::commissioning::outer::velocity::integral_reporting::VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES],
            },
            "integral_terminal",
        );
    }
}

/// `foci_outer_safety_fault` reply — installed-tuning safety-envelope evidence.
///
/// Wire name: `foci_outer_safety_fault`.
#[klipper_reply]
pub struct FociOuterSafetyFault {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
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
    /// Signed accumulated position from the installed-tuning start, in encoder counts.
    pub position_counts: i32,
    /// Active signed-position window in encoder counts.
    pub position_window_counts: i32,
    /// Elapsed time from installed-tuning safety arming, in microseconds.
    pub elapsed_us: u32,
    /// Active primitive duration cap in microseconds.
    pub duration_cap_us: u32,
}

/// `foci_stall_result` reply — outcome of the most recent homing mode.
///
/// Wire name: `foci_stall_result`.
#[klipper_reply]
pub struct FociStallResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// 1 if the stall latch set during the most recent homing mode.
    pub latched: u8,
    /// Peak following error in TMC position units during that homing mode.
    pub peak_error_units: u32,
    /// Detector tick at which the latch set, 0 if it never did.
    pub trigger_tick: u32,
    /// 1 while the homing clamp overrides the run-current limit.
    pub clamp_active: u8,
    /// Which check latched: 0 = none, 1 = ceiling, 2 = margin.
    pub trigger_path: u8,
    /// Largest error-minus-baseline observed while the baseline was valid,
    /// in TMC position units. 0 if the baseline never warmed up.
    pub peak_margin_delta_units: u32,
}

/// `foci_tune_result` reply — final outer-loop tune output.
///
/// Wire name: `foci_tune_result`.
#[klipper_reply]
pub struct FociTuneResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sequence shared by the position-evidence fragment and this terminal.
    pub report_seq: u8,
    /// Status code (0 = success, non-zero = error code).
    pub status: u8,
    /// Velocity P gain.
    pub velocity_p: u16,
    /// Velocity I gain.
    pub velocity_i: u16,
    /// Position P gain.
    pub position_p: u16,
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
    /// `PositionTune` closed-form start rung the sweep predicted.
    pub predicted_start_p: u16,
    /// `PositionTune` count of rungs measured before this terminal.
    pub rungs_measured: u8,
    /// `PositionTune` feedforward paths live on the last admitted
    /// motion stroke (bit 0 velocity, 1 transient, 2 acceleration,
    /// 3 decoupling, 4 phase advance).
    pub stimulus_feedforward_paths: u8,
    /// Wire code of the specific `PositionTuneOutcome` behind a
    /// `CommissionError::PositionTuneFailed` (0 = not applicable).
    pub position_tune_outcome_code: u8,
}

/// `foci_tune_position_evidence` reply - non-terminal `PositionTune` fragment.
///
/// Emitted before the terminal, only when `rungs_measured > 0`.
/// Wire name: `foci_tune_position_evidence`.
#[klipper_reply]
pub struct FociTunePositionEvidence {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Sequence shared by this fragment and the terminal.
    pub report_seq: u8,
    /// `PositionTune` position P of the last rung measured.
    pub last_rung_p: u16,
    /// `PositionTune` position P of the rung that set the
    /// running-minimum motion overshoot.
    pub min_overshoot_p: u16,
    /// `PositionTune` running-minimum motion overshoot, in
    /// position-error units.
    pub min_overshoot_units: u32,
    /// `PositionTune` config-derived tracking bound, in
    /// position-error units.
    pub bound_units: u32,
    /// `PositionTune` planned homing-leg cruise speed, in
    /// milli-revolutions/second.
    pub homing_speed_mrev_s: u32,
    /// `PositionTune` planned motion-leg cruise speed, in
    /// milli-revolutions/second.
    pub motion_speed_mrev_s: u32,
    /// `PositionTune` last rung's median homing-leg peak absolute
    /// tracking error.
    pub homing_peak_abs_units: u32,
    /// `PositionTune` last rung's median motion-leg mean absolute
    /// cruise tracking error.
    pub motion_cruise_mean_abs_units: u32,
    /// `PositionTune` last rung's median motion-leg overshoot.
    pub motion_overshoot_units: u32,
    /// `PositionTune` last rung's median motion-leg cruise RMS
    /// tracking error.
    pub motion_cruise_rms_units: u32,
    /// `PositionTune` last rung's median ceiling ratio, scaled by
    /// 1000.
    pub dither_margin_milli: i32,
}

#[cfg(test)]
mod tune_result_reply_wire_tests {
    //! Pins the `foci_tune_result` and `foci_tune_position_evidence` wire
    //! orders. Mirrors the ascending-small-value idiom in
    //! `breakaway_reply_wire_tests` above (every field here VLQ-encodes
    //! values `1..=16` as a single raw byte -- see `encode_vlq_int`'s
    //! single-byte range of `-32..96`).
    use super::{FociTunePositionEvidence, FociTuneResult};
    use ankyra::ScratchOutput;
    use ankyra::encoding::{Readable, Writable};

    fn reply_with_known_fields() -> FociTuneResult {
        FociTuneResult {
            oid: 1,
            report_seq: 2,
            status: 3,
            velocity_p: 4,
            velocity_i: 5,
            position_p: 6,
            probed_velocity_mrev_s: 7,
            d_eq_q: 8,
            confidence_q: 9,
            band_lower_percent: 10,
            band_upper_percent: 11,
            band_position_q: 12,
            predicted_start_p: 13,
            rungs_measured: 14,
            stimulus_feedforward_paths: 15,
            position_tune_outcome_code: 16,
        }
    }

    fn evidence_with_known_fields() -> FociTunePositionEvidence {
        FociTunePositionEvidence {
            oid: 1,
            report_seq: 2,
            last_rung_p: 3,
            min_overshoot_p: 4,
            min_overshoot_units: 5,
            bound_units: 6,
            homing_speed_mrev_s: 7,
            motion_speed_mrev_s: 8,
            homing_peak_abs_units: 9,
            motion_cruise_mean_abs_units: 10,
            motion_overshoot_units: 11,
            motion_cruise_rms_units: 12,
            dither_margin_milli: 13,
        }
    }

    #[test]
    fn terminal_fields_encode_in_declared_order() {
        let reply = reply_with_known_fields();
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let encoded = scratch.result();

        assert_eq!(
            encoded,
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
            "terminal fields must serialize in declared order"
        );
    }

    #[test]
    fn evidence_fields_encode_in_declared_order() {
        let evidence = evidence_with_known_fields();
        let mut scratch = ScratchOutput::<64>::new();
        evidence.write(&mut scratch);
        let encoded = scratch.result();

        assert_eq!(
            encoded,
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
            "evidence fields must serialize in declared order"
        );
    }

    #[test]
    fn position_tune_failure_reply_carries_the_last_rungs_evidence() {
        let reply = FociTuneResult {
            oid: 1,
            report_seq: 0,
            status: 12, // CommissionError::PositionTuneFailed
            velocity_p: 0,
            velocity_i: 0,
            position_p: 0,
            probed_velocity_mrev_s: 0,
            d_eq_q: 0,
            confidence_q: 0,
            band_lower_percent: 0,
            band_upper_percent: 0,
            band_position_q: 0,
            predicted_start_p: 0,
            rungs_measured: 3,
            stimulus_feedforward_paths: 0,
            position_tune_outcome_code: 8, // PositionTuneOutcome::Conflict
        };
        let evidence = FociTunePositionEvidence {
            oid: 1,
            report_seq: 0,
            last_rung_p: 388,
            min_overshoot_p: 195,
            min_overshoot_units: 40,
            bound_units: 0,
            homing_speed_mrev_s: 0,
            motion_speed_mrev_s: 0,
            homing_peak_abs_units: 0,
            motion_cruise_mean_abs_units: 0,
            motion_overshoot_units: 500,
            motion_cruise_rms_units: 0,
            dither_margin_milli: 0,
        };

        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let mut cursor = scratch.result();

        let _oid = u8::read(&mut cursor).expect("oid");
        let _report_seq = u8::read(&mut cursor).expect("report_seq");
        let status = u8::read(&mut cursor).expect("status");
        let _velocity_p = u16::read(&mut cursor).expect("velocity_p");
        let _velocity_i = u16::read(&mut cursor).expect("velocity_i");
        let _position_p = u16::read(&mut cursor).expect("position_p");
        let _probed_velocity_mrev_s = u32::read(&mut cursor).expect("probed_velocity_mrev_s");
        let _d_eq_q = i32::read(&mut cursor).expect("d_eq_q");
        let _confidence_q = u16::read(&mut cursor).expect("confidence_q");
        let _band_lower_percent = u8::read(&mut cursor).expect("band_lower_percent");
        let _band_upper_percent = u8::read(&mut cursor).expect("band_upper_percent");
        let _band_position_q = u16::read(&mut cursor).expect("band_position_q");
        let predicted_start_p = u16::read(&mut cursor).expect("predicted_start_p");
        let rungs_measured = u8::read(&mut cursor).expect("rungs_measured");
        let _stimulus_feedforward_paths =
            u8::read(&mut cursor).expect("stimulus_feedforward_paths");
        let outcome_code = u8::read(&mut cursor).expect("position_tune_outcome_code");

        assert_eq!(status, 12);
        assert_eq!(predicted_start_p, 0);
        assert_eq!(rungs_measured, 3);
        assert_eq!(outcome_code, 8);
        assert!(
            cursor.is_empty(),
            "every terminal field must have been decoded"
        );

        let mut evidence_scratch = ScratchOutput::<64>::new();
        evidence.write(&mut evidence_scratch);
        let mut evidence_cursor = evidence_scratch.result();

        let _oid = u8::read(&mut evidence_cursor).expect("oid");
        let _report_seq = u8::read(&mut evidence_cursor).expect("report_seq");
        let last_rung_p = u16::read(&mut evidence_cursor).expect("last_rung_p");
        let min_overshoot_p = u16::read(&mut evidence_cursor).expect("min_overshoot_p");
        let min_overshoot_units = u32::read(&mut evidence_cursor).expect("min_overshoot_units");
        let bound_units = u32::read(&mut evidence_cursor).expect("bound_units");
        let _homing_speed_mrev_s = u32::read(&mut evidence_cursor).expect("homing_speed_mrev_s");
        let _motion_speed_mrev_s = u32::read(&mut evidence_cursor).expect("motion_speed_mrev_s");
        let _homing_peak_abs_units =
            u32::read(&mut evidence_cursor).expect("homing_peak_abs_units");
        let _motion_cruise_mean_abs_units =
            u32::read(&mut evidence_cursor).expect("motion_cruise_mean_abs_units");
        let motion_overshoot_units =
            u32::read(&mut evidence_cursor).expect("motion_overshoot_units");
        let _motion_cruise_rms_units =
            u32::read(&mut evidence_cursor).expect("motion_cruise_rms_units");
        let _dither_margin_milli = i32::read(&mut evidence_cursor).expect("dither_margin_milli");

        assert_eq!(last_rung_p, 388);
        assert_eq!(min_overshoot_p, 195);
        assert_eq!(min_overshoot_units, 40);
        assert_eq!(bound_units, 0);
        assert_eq!(motion_overshoot_units, 500);
        assert!(
            evidence_cursor.is_empty(),
            "every evidence field must have been decoded"
        );
    }
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
/// `src/basecmd.c::stats_update`, filled from [`crate::cpu_load::take`].
#[klipper_reply]
pub struct Stats {
    /// Idle (`wfi`) laps since the last emission, at least 1.
    pub count: u32,
    /// Busy clock ticks since the last emission: wall time minus idle time.
    pub sum: u32,
    /// Always 0; per-iteration variance is not measured.
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
