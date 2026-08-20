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

/// `foci_adc_residual` reply - post-ADC-calibration zero-current residual evidence.
///
/// Wire name: `foci_adc_residual`.
#[klipper_reply]
pub struct FociAdcResidual {
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
    /// Packed R/L/delay timing states and worst-case metrics.
    pub timing_summary: u32,
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

/// `foci_closed_loop_entry` reply - fast closed-loop entry stability evidence.
///
/// Wire name: `foci_closed_loop_entry`.
#[klipper_reply]
pub struct FociClosedLoopEntry {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// `CLOSED_LOOP_ENTRY_STATUS_*`.
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

/// `foci_current_validation_settled_sample` reply - raw settled-flux validation read.
///
/// Wire name: `foci_current_validation_settled_sample`.
#[klipper_reply]
pub struct FociCurrentValidationSettledSample {
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

/// `foci_current_validation_envelope` reply - current-validation target limits.
///
/// Wire name: `foci_current_validation_envelope`.
#[klipper_reply]
pub struct FociCurrentValidationEnvelope {
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
    use super::{
        FociClosedLoopEntry, FociCommissionResult, FociCommissionTiming, FociCurrentLoopFilters,
        FociCurrentLoopHold, FociOuterSafetyFault, FociResistanceAxis, FociResistanceProfile,
        FociResistanceRun, FociRobustnessCycleEvidence,
    };
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

    #[test]
    fn commission_timing_reply_fits_klipper_message_limit() {
        let reply = FociCommissionTiming {
            oid: u8::MAX,
            method: u8::MAX,
            status: u8::MAX,
            requested_period_us: u16::MAX,
            valid_samples: u16::MAX,
            missed_samples: u16::MAX,
            max_consecutive_misses: u8::MAX,
            max_lateness_us: u32::MAX,
            max_interval_us: u32::MAX,
            max_poll_wall_us: u32::MAX,
            max_spi_wall_us: u32::MAX,
        };
        assert_fits_with_id(&reply, "foci_commission_timing");
    }

    #[test]
    fn commission_result_with_timing_summary_fits_klipper_message_limit() {
        let reply = FociCommissionResult {
            oid: u8::MAX,
            status: u8::MAX,
            r_count_milli: u32::MAX,
            l_count_micro: u32::MAX,
            flux_p: u16::MAX,
            flux_i: u16::MAX,
            torque_p: u16::MAX,
            torque_i: u16::MAX,
            lambda_us: u32::MAX,
            theta_e_us: u32::MAX,
            ringing_count: u8::MAX,
            bandwidth_hz: u32::MAX,
            fallback_velocity_p: u16::MAX,
            fallback_velocity_i: u16::MAX,
            fallback_position_p: u16::MAX,
            fallback_position_i: u16::MAX,
            fallback_velocity_limit: u32::MAX,
            tau_e_us: u32::MAX,
            inner_warning_flags: u8::MAX,
            timing_summary: u32::MAX,
        };
        assert_fits_with_id(&reply, "foci_commission_result");
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
            selected_r_count_slope_milli: i32::MIN,
            gain_path_count_slope_milli: i32::MIN,
            warning_flags: 0x7FFF_FFFF,
            status_flags_or: 0x7FFF_FFFF,
            peak_abs_current_count: u16::MAX,
            max_abs_steady_mean_current_count: u16::MAX,
            current_ceiling_count: u16::MAX,
            power_stage_tripped: u8::MAX,
            pwm_maxcnt_readback: u16::MAX,
            bbm_readback: u16::MAX,
            dsadc_mdec_readback: 0x7FFF_FFFF,
            pwm_sv_chop_readback: 0x7FFF_FFFF,
        };

        assert_fits_with_id(&reply, "foci_resistance_run");
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
    fn current_loop_filters_reply_fits_klipper_message_limit() {
        let reply = FociCurrentLoopFilters {
            oid: u8::MAX,
            velocity_filter_hz: u16::MAX,
            torque_filter_hz: u16::MAX,
            position_filter_hz: u16::MAX,
            flux_filter_hz: u16::MAX,
        };
        assert_fits_with_id(&reply, "foci_current_loop_filters");
    }

    #[test]
    fn closed_loop_entry_reply_fits_klipper_message_limit() {
        let reply = FociClosedLoopEntry {
            oid: u8::MAX,
            entry_status: u8::MAX,
            position_1: i32::MIN,
            position_2: i32::MIN,
            drift_count: u16::MAX,
            threshold_count: u16::MAX,
            runaway: u8::MAX,
        };
        assert_fits_with_id(&reply, "foci_closed_loop_entry");
    }

    #[test]
    fn current_loop_hold_reply_fits_klipper_message_limit() {
        let reply = FociCurrentLoopHold {
            oid: u8::MAX,
            hold_status: u8::MAX,
            warning_flags: u16::MAX,
            elapsed_us: 0x7FFF_FFFF,
            requested_sample_period_us: u16::MAX,
            sample_count: u8::MAX,
            position_span_count: u16::MAX,
            position_drift_count: u16::MAX,
            torque_mean_count: i16::MIN,
            torque_rms_count: u16::MAX,
            torque_peak_to_peak_count: u16::MAX,
            torque_crossing_count: u8::MAX,
            flux_mean_count: i16::MIN,
            flux_rms_count: u16::MAX,
            flux_peak_to_peak_count: u16::MAX,
            flux_crossing_count: u8::MAX,
            status_flags_or: 0x7FFF_FFFF,
            actionable_status_count: u8::MAX,
        };
        assert_fits_with_id(&reply, "foci_current_loop_hold");
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

    #[test]
    fn current_validation_settled_sample_reply_fits_klipper_message_limit() {
        let reply = super::FociCurrentValidationSettledSample {
            oid: u8::MAX,
            axis: u8::MAX,
            sample_index: u8::MAX,
            direction: u8::MAX,
            raw_index: u8::MAX,
            attempt: u8::MAX,
            target: i16::MAX,
            sample_delay_ms: u16::MAX,
            same_axis_count: i16::MAX,
            cross_axis_count: i16::MIN,
            cross_axis_permille: u16::MAX,
            voltage_output_permille: u16::MAX,
            encoder_delta_counts: i32::MAX,
            status_flags: 0x7FFF_FFFF,
        };
        assert_fits_with_id(&reply, "foci_current_validation_settled_sample");
    }

    #[test]
    fn current_validation_envelope_reply_fits_klipper_message_limit() {
        let reply = super::FociCurrentValidationEnvelope {
            oid: u8::MAX,
            step_amplitude: u16::MAX,
            flux_step_amplitude: u16::MAX,
            torque_step_amplitude: u16::MAX,
            pidout_limit: u16::MAX,
            current_limited: u16::MAX,
            voltage_limited: u16::MAX,
            max_p: u16::MAX,
        };
        assert_fits_with_id(&reply, "foci_current_validation_envelope");
    }

    #[test]
    fn adc_residual_reply_fits_klipper_message_limit() {
        let reply = super::FociAdcResidual {
            oid: u8::MAX,
            stage: u8::MAX,
            sample_count: u8::MAX,
            pwm_sv_chop: u8::MAX,
            pwm_bbm: u32::MAX,
            adc_i0_scale_offset: u32::MAX,
            adc_i1_scale_offset: u32::MAX,
            adc_iux_mean_count: i16::MAX,
            adc_iwy_mean_count: i16::MIN,
            pid_flux_mean_count: i16::MAX,
            pid_torque_mean_count: i16::MIN,
        };
        assert_fits_with_id(&reply, "foci_adc_residual");
    }

    #[test]
    fn outer_safety_fault_reply_fits_klipper_message_limit() {
        let reply = FociOuterSafetyFault {
            oid: u8::MAX,
            reason: u8::MAX,
            max_travel_mrev: u16::MAX,
            max_velocity_mrev_s: u16::MAX,
            max_duration_ms: u16::MAX,
            direction_mask: u8::MAX,
            delta_counts: i32::MIN,
            dt_us: 0x7FFF_FFFF,
            velocity_counts_per_ms: i32::MIN,
            velocity_cap_counts_per_ms: i32::MAX,
            position_counts: i32::MIN,
            position_window_counts: i32::MAX,
            elapsed_us: 0x7FFF_FFFF,
            duration_cap_us: 0x7FFF_FFFF,
        };
        assert_fits_with_id(&reply, "foci_outer_safety_fault");
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

    /// Klipper's wire-frame payload budget (see `resistance_reply_size_tests`
    /// above for the derivation).
    const KLIPPER_MESSAGE_PAYLOAD_MAX: usize = 59;
    const REPLY_ID_WORST_CASE_BYTES: usize = 3;

    fn encoded_len<T: Writable>(reply: &T) -> usize {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        scratch.result().len()
    }

    fn assert_fits_with_id<T: Writable>(reply: &T, reply_kind: &str) {
        let payload_len = encoded_len(reply);
        let total_len = payload_len + REPLY_ID_WORST_CASE_BYTES;
        assert!(
            total_len <= KLIPPER_MESSAGE_PAYLOAD_MAX,
            "{reply_kind} encodes to {payload_len} payload bytes + {REPLY_ID_WORST_CASE_BYTES} \
             worst-case id bytes = {total_len}, exceeds {KLIPPER_MESSAGE_PAYLOAD_MAX}"
        );
    }

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
        assert_fits_with_id(
            &FociBreakawayProbePlan {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                max_observations: u8::MAX,
                search_count: u8::MAX,
                p_start_raw: u16::MAX,
                p_top_raw: u16::MAX,
                motion_threshold_counts: u32::MAX,
                max_capture_interval_us: u32::MAX,
            },
            "foci_breakaway_probe_plan",
        );
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
        assert_fits_with_id(
            &FociBreakawayProbeResult {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                rung_index: u8::MAX,
                breakaway_p_raw: u16::MAX,
                motion_threshold_counts: u32::MAX,
                observation_count: u8::MAX,
            },
            "foci_breakaway_probe_result",
        );
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
        assert_fits_with_id(
            &FociBreakawayProbeTerminal {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                terminal_cause: u8::MAX,
                has_safety_fault: u8::MAX,
            },
            "foci_breakaway_probe_terminal",
        );
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
        assert_fits_with_id(
            &FociBreakawayDiscoveryPlanIdentity {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                prior_plan_digest_low: u32::MAX,
                prior_plan_digest_high: u32::MAX,
                family_size: u16::MAX,
            },
            "foci_breakaway_discovery_plan_identity",
        );
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
        assert_fits_with_id(
            &FociBreakawayDiscoveryPlanGeometry {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                schema_revision: u16::MAX,
                rung_count: u8::MAX,
                observations_per_direction: u16::MAX,
                floor_p_raw: u16::MAX,
                floor_origin: u8::MAX,
                breakaway_p_raw: u16::MAX,
                ceiling_p_raw: u16::MAX,
                first_additive_step_raw: u32::MAX,
                maximum_workflow_ms: u32::MAX,
            },
            "foci_breakaway_discovery_plan_geometry",
        );
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
        assert_fits_with_id(
            &FociBreakawayRungZeroDiagnostic {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                forward_moved: u8::MAX,
                reverse_moved: u8::MAX,
            },
            "foci_breakaway_rung_zero_diagnostic",
        );
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
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
        assert_fits_with_id(
            &FociBreakawayDiscoveryTerminal {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                prior_plan_digest_low: u32::MAX,
                prior_plan_digest_high: u32::MAX,
                family_size: u16::MAX,
                terminal_cause: u8::MAX,
                collected_count: u8::MAX,
                has_safety_fault: u8::MAX,
            },
            "foci_breakaway_discovery_terminal",
        );
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
            candidate_p_raw: 10,
            band_lower_percent: 11,
            band_upper_percent: 12,
            capture_profile: 13,
            acceptance_rule: 14,
            nominated_margin_percent_milli: 15,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
        assert_fits_with_id(
            &FociBreakawayConfirmationPlan {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                prior_plan_digest_low: u32::MAX,
                prior_plan_digest_high: u32::MAX,
                family_size: u16::MAX,
                observations_per_direction: u16::MAX,
                candidate_p_raw: u16::MAX,
                band_lower_percent: u8::MAX,
                band_upper_percent: u8::MAX,
                capture_profile: u8::MAX,
                acceptance_rule: u8::MAX,
                nominated_margin_percent_milli: i32::MIN,
            },
            "foci_breakaway_confirmation_plan",
        );
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
        assert_fits_with_id(
            &FociBreakawayConfirmationTerminalIdentity {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                plan_digest_low: u32::MAX,
                plan_digest_high: u32::MAX,
                prior_plan_digest_low: u32::MAX,
                prior_plan_digest_high: u32::MAX,
                family_size: u16::MAX,
                terminal_cause: u8::MAX,
            },
            "foci_breakaway_confirmation_terminal_identity",
        );
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
        assert_fits_with_id(
            &FociBreakawayConfirmationTerminalMasks {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                forward_collected_mask: u8::MAX,
                forward_eligible_mask: u8::MAX,
                forward_included_mask: u8::MAX,
                reverse_collected_mask: u8::MAX,
                reverse_eligible_mask: u8::MAX,
                reverse_included_mask: u8::MAX,
                accepted: u8::MAX,
                confirmed_p_raw: u16::MAX,
                max_relative_se_permille: u16::MAX,
                required_relative_se_permille: u16::MAX,
                has_safety_fault: u8::MAX,
            },
            "foci_breakaway_confirmation_terminal_masks",
        );
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
            stage_c_plan_digest_low: 7,
            stage_c_plan_digest_high: 8,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_fits_with_id(
            &FociBreakawayCampaignTerminal {
                oid: u8::MAX,
                run_sequence: u32::MAX,
                evidence_sequence: u16::MAX,
                phase: u8::MAX,
                terminal_cause: u8::MAX,
                accepted: u8::MAX,
                stage_c_plan_digest_low: u32::MAX,
                stage_c_plan_digest_high: u32::MAX,
            },
            "foci_breakaway_campaign_terminal",
        );
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
    /// Configured quadrature encoder counts per revolution, or zero when unconfigured.
    pub encoder_counts_per_rev: u32,
    /// Configured physical TMC STEP pulses per revolution, or zero when unconfigured.
    pub tmc_grid: u32,
    /// Configured TMC position units per physical STEP pulse, or zero when unconfigured.
    pub physical_step_width: u32,
    /// Non-zero when a complete motion scale is currently configured.
    pub motion_scale_configured: u8,
    /// Timer ticks in one physical STEP high or low phase.
    pub step_half_period_ticks: u32,
    /// Timer ticks reserved between a DIR change and the first STEP edge.
    pub dir_setup_ticks: u32,
    /// Waveform-only two-pulse cost including DIR setup.
    pub waveform_worst_case_ticks: u32,
    /// One millisecond of fatal scheduler lateness in timer ticks.
    pub fatal_lateness_ticks: u32,
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

/// Compact ordinary-channel projection of one canonical recovery record.
#[klipper_reply]
pub struct FociRungOriginRecoverySummary {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Owning commissioning stage.
    pub stage: u8,
    /// Zero-based rung index.
    pub rung_index: u8,
    /// Proportional gain used by the recovery controller.
    pub p_raw: u16,
    /// Binding recovery-authority or containment source.
    pub binding_source: u8,
    /// Recovery outcome.
    pub outcome: u8,
}

/// Causal current-limit evidence for one velocity primitive.
#[klipper_reply]
pub struct FociVelocityPrimitiveCurrentEvidence {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence within the owning stage.
    pub evidence_sequence: u16,
    /// Owning stage: zero for Stage B and one for Stage C.
    pub stage: u8,
    /// Zero-based rung index.
    pub rung_index: u8,
    /// Fixed signed observation slot.
    pub slot: u8,
    /// Temporary velocity-output clamp.
    pub clamp_limit: u16,
    /// Verified clamp readback.
    pub clamp_readback: u16,
    /// Largest sampled absolute PID output during moving evidence.
    pub moving_pid_output_peak: u16,
    /// Largest sampled absolute PID output during braking and settle.
    pub zero_pid_output_peak: u16,
    /// Complete status word at the signed-to-zero boundary.
    pub moving_status_flags: u32,
    /// Complete status word after confirmed rest.
    pub zero_status_flags: u32,
    /// Declared limit-evidence capability.
    pub capability: u8,
    /// First phase with observed clamp contact.
    pub contact: u8,
}

/// Fixed stimulus for the velocity-limit latch test.
#[klipper_reply]
pub struct FociVelocityLimitLatchCore {
    /// Endpoint object id.
    pub oid: u8,
    /// Diagnostic status code.
    pub status: u8,
    /// Temporary velocity-output clamp.
    pub test_limit: u16,
    /// Velocity proportional gain.
    pub p_raw: u16,
    /// Velocity integral gain.
    pub i_raw: u16,
    /// Signed velocity target in the TMC4671 RPM domain.
    pub target_velocity_rpm: i32,
    /// Verified clamp readback.
    pub limit_readback: u32,
    /// Verified gain-register readback.
    pub gains_readback: u32,
    /// Verified signed target readback.
    pub target_readback: i32,
    /// Whether the board containment hook physically disarmed the channel.
    pub power_stage_tripped: u8,
}

/// Timing and displacement for the velocity-limit latch test.
#[klipper_reply]
pub struct FociVelocityLimitLatchMotion {
    /// Endpoint object id.
    pub oid: u8,
    /// Measured target-active interval.
    pub pulse_elapsed_us: u32,
    /// Measured persistence interval.
    pub delayed_read_elapsed_us: u32,
    /// ABN count before the stimulus.
    pub encoder_before: u32,
    /// ABN count after the delayed read.
    pub encoder_after: u32,
    /// Signed diagnostic displacement.
    pub encoder_delta: i32,
}

/// Full status snapshots for the velocity-limit latch test.
#[klipper_reply]
pub struct FociVelocityLimitLatchFlags {
    /// Endpoint object id.
    pub oid: u8,
    /// Status sampled before commanding zero.
    pub active_status_flags: u32,
    /// Status sampled immediately after commanding zero.
    pub post_pulse_status_flags: u32,
    /// Status sampled after the persistence interval.
    pub delayed_status_flags: u32,
    /// Status sampled after explicit reset.
    pub post_clear_status_flags: u32,
}

/// Saved and restored register evidence for the velocity-limit latch test.
#[klipper_reply]
pub struct FociVelocityLimitLatchRestore {
    /// Endpoint object id.
    pub oid: u8,
    /// Saved operational current-limit word.
    pub saved_limit: u32,
    /// Saved velocity-gain word.
    pub saved_gains: u32,
    /// Saved signed velocity target.
    pub saved_target: i32,
    /// Saved motion-mode word.
    pub saved_mode: u32,
    /// Restored current-limit readback.
    pub restored_limit_readback: u32,
    /// Restored velocity-gain readback.
    pub restored_gains_readback: u32,
    /// Restored signed velocity-target readback.
    pub restored_target_readback: i32,
    /// Restored motion-mode readback.
    pub restored_mode_readback: u32,
    /// Exact restoration-success mask.
    pub restore_verified_mask: u8,
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
    pub candidate_p_raw: u16,
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
    /// Nonzero when the confirmed candidate was accepted and admitted to Stage C.
    pub accepted: u8,
    /// Exact low Stage-C plan digest word.
    pub stage_c_plan_digest_low: u32,
    /// Exact high Stage-C plan digest word.
    pub stage_c_plan_digest_high: u32,
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
    /// Stage B, Stage-B-then-C, or Stage-C-resume discriminator.
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

/// Stage-C plan identity and reproduced proportional authority.
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
    /// Exact low Stage-C plan digest word.
    pub plan_digest_low: u32,
    /// Exact high Stage-C plan digest word.
    pub plan_digest_high: u32,
    /// Exact low authorizing Stage-B plan digest word.
    pub stage_b_digest_low: u32,
    /// Exact high authorizing Stage-B plan digest word.
    pub stage_b_digest_high: u32,
    /// Analysis-rule contract this plan was built under.
    pub build_features: u16,
    /// Image identity of the firmware that produced this plan.
    pub build_identity: u32,
    /// Stage-C evidence schema revision.
    pub schema_revision: u16,
    /// Physical firmware channel.
    pub channel: u8,
    /// Reproduced fixed proportional gain.
    pub final_p: u16,
}

/// Stage-C plan geometry and controller-domain limits.
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

/// One direction of Stage-B evidence authorizing Stage C.
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
    /// Retained Stage-B pooled lower edge, or schema-8 selected-rate lower edge.
    pub pooled_low_q16: i32,
    /// Retained Stage-B pooled upper edge, or schema-8 selected-rate upper edge.
    pub pooled_high_q16: i32,
}

/// Stage-C fixed schedule and travel identity.
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

/// Stage-C travel and absolute-position identity.
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

/// Stage-C rung-origin recovery geometry and reserved schedule.
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

/// Stage-C observation identity and classification.
#[klipper_reply]
pub struct FociVelocityIntegralObservationCore {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Observation fragment discriminator.
    pub fragment: u8,
    /// Exact low plan digest word.
    pub plan_digest_low: u32,
    /// Exact high plan digest word.
    pub plan_digest_high: u32,
    /// Rung index including anchors.
    pub rung_index: u8,
    /// Fixed F/R observation slot.
    pub slot: u8,
    /// Integral gain programmed for the rung.
    pub i_raw: u16,
    /// Direction code.
    pub direction: u8,
    /// Intrinsic Stage-C observation class.
    pub classification: u8,
    /// Whether this was the campaign's initial lead-in.
    pub initial_lead_in: u8,
    /// Peak absolute torque-target count.
    pub peak_torque_target_abs: u16,
}

/// Stage-C observation rate and exact deficit interval.
#[klipper_reply]
pub struct FociVelocityIntegralObservationRate {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Observation fragment discriminator.
    pub fragment: u8,
    /// Compact settled-rate lower edge.
    pub settled_low: i32,
    /// Compact settled-rate point estimate.
    pub settled_mean: i32,
    /// Compact settled-rate upper edge.
    pub settled_high: i32,
    /// Common settled-rate right shift.
    pub settled_shift: u8,
    /// Exact signed deficit lower edge.
    pub deficit_low_q: i32,
    /// Exact signed deficit upper edge.
    pub deficit_high_q: i32,
}

/// Stage-C observation stationarity quality.
#[klipper_reply]
pub struct FociVelocityIntegralObservationQuality {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Observation fragment discriminator.
    pub fragment: u8,
    /// Selected suffix length or zero when excluded.
    pub suffix_len: u16,
    /// Selected blocking level or exclusion code.
    pub level_or_exclusion: u8,
    /// Selected decorrelated block count.
    pub selected_blocks: u32,
    /// Outward compact slope estimate.
    pub slope_mantissa: i32,
    /// Slope estimate right shift.
    pub slope_shift: u8,
    /// Outward compact slope half-width.
    pub slope_half_width_mantissa: u32,
    /// Slope half-width right shift.
    pub slope_half_width_shift: u8,
    /// Lag-one residual correlation in millionths.
    pub lag_one_q: i32,
    /// Lag-one half-width in millionths.
    pub lag_one_half_width_q: u32,
    /// Outward compact residual magnitude.
    pub residual_mantissa: u32,
    /// Residual magnitude right shift.
    pub residual_shift: u8,
    /// Number of declared suffixes tested.
    pub tested_suffixes: u8,
}

/// Stage-C directional rung verdict and consensus masks.
#[klipper_reply]
pub struct FociVelocityIntegralRungCore {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Rung fragment discriminator.
    pub fragment: u8,
    /// Rung index including anchors.
    pub rung_index: u8,
    /// Integral gain programmed for the rung.
    pub i_raw: u16,
    /// Direction code.
    pub direction: u8,
    /// Opening, positive, or closing role.
    pub kind: u8,
    /// Directional consensus class.
    pub classification: u8,
    /// Number of exact depth-three components.
    pub component_count: u8,
    /// Four-bit intrinsically usable observation mask.
    pub usable_mask: u8,
    /// Four-bit unique-component support mask.
    pub included_mask: u8,
    /// Four packed three-bit observation classes.
    pub observation_classes: u16,
}

/// One exact Stage-C consensus component.
#[klipper_reply]
pub struct FociVelocityIntegralRungComponent {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Monotonic evidence sequence.
    pub evidence_sequence: u16,
    /// Rung index including anchors.
    pub rung_index: u8,
    /// Direction code.
    pub direction: u8,
    /// Zero-based component index.
    pub component: u8,
    /// Exact lower deficit edge.
    pub low_q: i32,
    /// Exact upper deficit edge.
    pub high_q: i32,
}

/// Stage-C run sufficiency, masks, and current terminus.
#[klipper_reply]
pub struct FociVelocityIntegralRunSummary {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Derived fragment discriminator.
    pub fragment: u8,
    /// Forward positive-rung eligibility mask.
    pub forward_eligible_mask: u32,
    /// Reverse positive-rung eligibility mask.
    pub reverse_eligible_mask: u32,
    /// Directions with an eligible opening anchor.
    pub opening_available_mask: u8,
    /// Directions with an eligible closing bookend.
    pub bookend_available_mask: u8,
    /// One plus the current-headroom terminus, or zero.
    pub current_terminus_plus_one: u8,
    /// Directions meeting the per-run three-rung floor.
    pub sufficient_direction_mask: u8,
}

/// One exact Stage-C anchor, curve, or bookend interval.
#[klipper_reply]
pub struct FociVelocityIntegralCurveInterval {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Opening, positive, or closing interval kind.
    pub kind: u8,
    /// Direction code.
    pub direction: u8,
    /// Positive-rung index, or zero for an anchor.
    pub rung_index: u8,
    /// Exact lower deficit edge.
    pub low_q: i32,
    /// Exact upper deficit edge.
    pub high_q: i32,
}

/// One direction's opening-to-bookend drift evidence.
#[klipper_reply]
pub struct FociVelocityIntegralDrift {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Direction code.
    pub direction: u8,
    /// Whether the closing bookend is available.
    pub available: u8,
    /// Exact opening lower edge.
    pub opening_low_q: i32,
    /// Exact opening upper edge.
    pub opening_high_q: i32,
    /// Exact closing lower edge, or zero.
    pub closing_low_q: i32,
    /// Exact closing upper edge, or zero.
    pub closing_high_q: i32,
}

/// Fresh Stage-C opening anchor compared with retained Stage-B disturbance.
#[klipper_reply]
pub struct FociVelocityIntegralStageBComparison {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Direction code.
    pub direction: u8,
    /// Whether model-relative comparison evidence is available.
    pub available: u8,
    /// Retained Stage-B pooled lower disturbance edge.
    pub stage_b_low_q16: i32,
    /// Retained Stage-B pooled upper disturbance edge.
    pub stage_b_high_q16: i32,
    /// Rate-inverted Stage-B deficit lower edge.
    pub expected_low_q: i32,
    /// Rate-inverted Stage-B deficit upper edge.
    pub expected_high_q: i32,
    /// Fresh Stage-C opening-anchor lower deficit edge.
    pub opening_low_q: i32,
    /// Fresh Stage-C opening-anchor upper deficit edge.
    pub opening_high_q: i32,
}

/// Stage-C reproduction outcome and run-level identity.
#[klipper_reply]
pub struct FociVelocityIntegralReproductionCore {
    /// Endpoint object id.
    pub oid: u8,
    /// Current caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared reproduction-group evidence sequence.
    pub evidence_sequence: u16,
    /// Reproduced or mismatched outcome code.
    pub outcome: u8,
    /// Per-direction reproduced sufficiency mask.
    pub direction_mask: u8,
}

/// One direction's Stage-C reproduction masks.
#[klipper_reply]
pub struct FociVelocityIntegralReproductionMask {
    /// Endpoint object id.
    pub oid: u8,
    /// Current caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared reproduction-group evidence sequence.
    pub evidence_sequence: u16,
    /// Direction code.
    pub direction: u8,
    /// Retained-run eligibility mask.
    pub previous_mask: u32,
    /// Current-run eligibility mask.
    pub current_mask: u32,
    /// Eligibility shared by both runs.
    pub shared_mask: u32,
    /// Shared rungs with overlapping anchor-relative response.
    pub reproduced_mask: u32,
    /// Shared rungs with divergent anchor-relative response.
    pub divergent_mask: u32,
    /// Rungs present only in the retained run.
    pub previous_only_mask: u32,
    /// Rungs present only in the current run.
    pub current_only_mask: u32,
}

/// One exact reproduced or divergent Stage-C response interval pair.
#[klipper_reply]
pub struct FociVelocityIntegralReproductionInterval {
    /// Endpoint object id.
    pub oid: u8,
    /// Current caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared reproduction-group evidence sequence.
    pub evidence_sequence: u16,
    /// Direction code.
    pub direction: u8,
    /// Positive-rung index.
    pub rung_index: u8,
    /// One for reproduced overlap, two for divergence.
    pub kind: u8,
    /// Retained anchor-relative lower edge.
    pub previous_low_q: i32,
    /// Retained anchor-relative upper edge.
    pub previous_high_q: i32,
    /// Current anchor-relative lower edge.
    pub current_low_q: i32,
    /// Current anchor-relative upper edge.
    pub current_high_q: i32,
    /// Exact overlap lower edge, or signed gap for divergence.
    pub result_low_or_gap_q: i32,
    /// Exact overlap upper edge, or zero for divergence.
    pub result_high_q: i32,
}

/// Exact previous and current Stage-C run digests.
#[klipper_reply]
pub struct FociVelocityIntegralReproductionDigest {
    /// Endpoint object id.
    pub oid: u8,
    /// Current caller-assigned run sequence.
    pub run_sequence: u32,
    /// Shared reproduction-group evidence sequence.
    pub evidence_sequence: u16,
    /// Retained digest low word.
    pub previous_digest_low: u32,
    /// Retained digest high word.
    pub previous_digest_high: u32,
    /// Current digest low word.
    pub current_digest_low: u32,
    /// Current digest high word.
    pub current_digest_high: u32,
}

/// Stage-C terminal class, cause, and report completeness.
#[klipper_reply]
pub struct FociVelocityIntegralTerminalCore {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Terminal fragment discriminator.
    pub fragment: u8,
    /// Stage-C terminal outcome.
    pub outcome: u8,
    /// Dedicated Stage-C cause.
    pub cause: u8,
    /// Recovery flags; schema 8 also carries combined target status.
    pub recovery_flags: u8,
    /// One-based rest-boundary rung, or schema-8 target rung when no boundary exists.
    pub rest_boundary_rung_plus_one: u8,
    /// One-based observation slot of an active-I rest boundary, or zero.
    pub rest_boundary_slot_plus_one: u8,
    /// Planned observation count.
    pub expected_observations: u16,
    /// Emitted observation count.
    pub emitted_observations: u16,
    /// Planned rung count.
    pub expected_rungs: u16,
    /// Emitted rung count.
    pub emitted_rungs: u16,
}

/// Stage-C terminal plan and evidence digests.
#[klipper_reply]
pub struct FociVelocityIntegralTerminalIdentity {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Terminal fragment discriminator.
    pub fragment: u8,
    /// Exact low plan digest word.
    pub plan_digest_low: u32,
    /// Exact high plan digest word.
    pub plan_digest_high: u32,
    /// Exact low evidence digest word.
    pub digest_low: u32,
    /// Exact high evidence digest word.
    pub digest_high: u32,
}

/// Stage-C terminal monotonic timing.
#[klipper_reply]
pub struct FociVelocityIntegralTerminalTiming {
    /// Endpoint object id.
    pub oid: u8,
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Terminal evidence sequence.
    pub evidence_sequence: u16,
    /// Terminal fragment discriminator.
    pub fragment: u8,
    /// Low start timestamp word.
    pub started_low: u32,
    /// High start timestamp word.
    pub started_high: u32,
    /// Low completion timestamp word.
    pub completed_low: u32,
    /// High completion timestamp word.
    pub completed_high: u32,
}

/// Immutable fixed-I velocity confidence-matrix plan.
#[klipper_reply]
pub struct FociAcceptanceMatrixPlan<'a> {
    /// Endpoint object id.
    pub oid: u8,
    /// Zero-based fragment index; the plan spans two fragments.
    pub fragment: u8,
    /// One fragment of the fixed-layout plan payload for its schema revision.
    pub payload: &'a [u8],
}

/// Compact fixed-I velocity confidence-matrix terminal.
#[klipper_reply]
pub struct FociAcceptanceMatrixTerminal<'a> {
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

    use crate::commissioning::outer::velocity::acceptance_reporting::{
        ACCEPTANCE_PLAN_REPLY_FRAGMENT_BYTES, ACCEPTANCE_TERMINAL_REPLY_BYTES,
    };

    const PAYLOAD_WITH_ID_MAX: usize = 59;
    const FRAME_MAX: usize = 64;
    const REPLY_ID_WORST_CASE_BYTES: usize = 3;
    const FRAME_OVERHEAD_BYTES: usize = 5;
    const WIDE_U32: u32 = 0x7fff_ffff;

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
    fn acceptance_matrix_replies_fit_one_protocol_frame() {
        assert_fits(
            &FociAcceptanceMatrixPlan {
                oid: u8::MAX,
                fragment: u8::MAX,
                payload: &[u8::MAX; ACCEPTANCE_PLAN_REPLY_FRAGMENT_BYTES],
            },
            "acceptance matrix plan fragment",
        );
        assert_fits(
            &FociAcceptanceMatrixTerminal {
                oid: u8::MAX,
                payload: &[u8::MAX; ACCEPTANCE_TERMINAL_REPLY_BYTES],
            },
            "acceptance matrix terminal",
        );
    }

    #[test]
    fn rung_origin_recovery_summary_is_one_bounded_frame() {
        let summary = FociRungOriginRecoverySummary {
            oid: u8::MAX,
            run_sequence: WIDE_U32,
            evidence_sequence: u16::MAX,
            stage: u8::MAX,
            rung_index: u8::MAX,
            p_raw: u16::MAX,
            binding_source: u8::MAX,
            outcome: u8::MAX,
        };
        let integral_plan = FociVelocityIntegralPlanRecovery {
            oid: u8::MAX,
            run_sequence: WIDE_U32,
            evidence_sequence: u16::MAX,
            origin_band_counts: WIDE_U32,
            nominal_slot_us: WIDE_U32,
            maximum_slot_us: WIDE_U32,
            slot_count: u8::MAX,
            flags: u8::MAX,
        };

        for (name, fits) in [
            ("recovery_summary", encoded_reply_fits(&summary)),
            ("integral_plan_recovery", encoded_reply_fits(&integral_plan)),
        ] {
            assert!(fits, "{name} exceeds the transport limit");
        }
    }

    #[test]
    fn velocity_primitive_current_evidence_fits_transport() {
        assert_fits(
            &FociVelocityPrimitiveCurrentEvidence {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                stage: u8::MAX,
                rung_index: u8::MAX,
                slot: u8::MAX,
                clamp_limit: u16::MAX,
                clamp_readback: u16::MAX,
                moving_pid_output_peak: u16::MAX,
                zero_pid_output_peak: u16::MAX,
                moving_status_flags: WIDE_U32,
                zero_status_flags: WIDE_U32,
                capability: u8::MAX,
                contact: u8::MAX,
            },
            "velocity_primitive_current_evidence",
        );
    }

    #[test]
    fn velocity_limit_latch_fragments_fit_transport() {
        assert_fits(
            &FociVelocityLimitLatchCore {
                oid: u8::MAX,
                status: u8::MAX,
                test_limit: u16::MAX,
                p_raw: u16::MAX,
                i_raw: u16::MAX,
                target_velocity_rpm: i32::MAX,
                limit_readback: WIDE_U32,
                gains_readback: WIDE_U32,
                target_readback: i32::MIN,
                power_stage_tripped: u8::MAX,
            },
            "velocity_limit_latch_core",
        );
        assert_fits(
            &FociVelocityLimitLatchMotion {
                oid: u8::MAX,
                pulse_elapsed_us: WIDE_U32,
                delayed_read_elapsed_us: WIDE_U32,
                encoder_before: WIDE_U32,
                encoder_after: WIDE_U32,
                encoder_delta: i32::MIN,
            },
            "velocity_limit_latch_motion",
        );
        assert_fits(
            &FociVelocityLimitLatchFlags {
                oid: u8::MAX,
                active_status_flags: WIDE_U32,
                post_pulse_status_flags: WIDE_U32,
                delayed_status_flags: WIDE_U32,
                post_clear_status_flags: WIDE_U32,
            },
            "velocity_limit_latch_flags",
        );
        assert_fits(
            &FociVelocityLimitLatchRestore {
                oid: u8::MAX,
                saved_limit: WIDE_U32,
                saved_gains: WIDE_U32,
                saved_target: i32::MIN,
                saved_mode: WIDE_U32,
                restored_limit_readback: WIDE_U32,
                restored_gains_readback: WIDE_U32,
                restored_target_readback: i32::MIN,
                restored_mode_readback: WIDE_U32,
                restore_verified_mask: u8::MAX,
            },
            "velocity_limit_latch_restore",
        );
    }

    fn encoded_reply_fits<T: Writable>(reply: &T) -> bool {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let payload = scratch.result().len() + REPLY_ID_WORST_CASE_BYTES;
        payload <= PAYLOAD_WITH_ID_MAX && payload + FRAME_OVERHEAD_BYTES <= FRAME_MAX
    }

    #[test]
    fn worst_case_velocity_integral_fragments_fit_transport() {
        let workflow = FociCommissioningWorkflowPlan {
            oid: u8::MAX,
            run_sequence: WIDE_U32,
            shape: u8::MAX,
            digest_low: WIDE_U32,
            digest_high: WIDE_U32,
            nominal_workflow_ms: WIDE_U32,
            maximum_workflow_ms: WIDE_U32,
        };
        assert_fits(&workflow, "integral_workflow");
        assert_fits(
            &FociVelocityIntegralPlanCore {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                plan_digest_low: WIDE_U32,
                plan_digest_high: WIDE_U32,
                stage_b_digest_low: WIDE_U32,
                stage_b_digest_high: WIDE_U32,
                build_features: u16::MAX,
                build_identity: WIDE_U32,
                schema_revision: u16::MAX,
                channel: u8::MAX,
                final_p: u16::MAX,
            },
            "integral_plan_core",
        );
        assert_fits(
            &FociVelocityIntegralPlanGeometry {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                planned_velocity_mrev_s: WIDE_U32,
                target_velocity_rpm: i32::MAX,
                pwm_hz: WIDE_U32,
                encoder_counts_per_rev: WIDE_U32,
                i_start: u16::MAX,
                positive_rung_count: u8::MAX,
                family_size: u16::MAX,
                total_rung_count: u8::MAX,
                expected_observations: u16::MAX,
                hard_torque_limit: u16::MAX,
                usable_torque_limit: u16::MAX,
            },
            "integral_plan_geometry",
        );
        assert_fits(
            &FociVelocityIntegralPlanAuthority {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                direction: u8::MAX,
                directional_membership: WIDE_U32,
                joint_membership: WIDE_U32,
                directional_validity: u8::MAX,
                reduced_margin: u8::MAX,
                pooled_low_q16: i32::MIN,
                pooled_high_q16: i32::MAX,
            },
            "integral_plan_authority",
        );
        assert_fits(
            &FociVelocityIntegralPlanTiming {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                moving_stroke_us: WIDE_U32,
                zero_settle_us: WIDE_U32,
                analysis_budget_us: WIDE_U32,
                nominal_workflow_ms: WIDE_U32,
                maximum_workflow_ms: WIDE_U32,
            },
            "integral_plan_timing",
        );
        assert_fits(
            &FociVelocityIntegralPlanTravel {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                max_stroke_travel_mrev: WIDE_U32,
                settle_travel_reserve_mrev: WIDE_U32,
                negative_position_headroom_mrev: WIDE_U32,
                positive_position_headroom_mrev: WIDE_U32,
            },
            "integral_plan_travel",
        );
        assert_fits(
            &FociVelocityIntegralPlanRung {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                rung_index: u8::MAX,
                i_raw: u16::MAX,
                tau_us: WIDE_U32,
            },
            "integral_plan_rung",
        );
        assert_fits(
            &FociVelocityIntegralObservationCore {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                plan_digest_low: WIDE_U32,
                plan_digest_high: WIDE_U32,
                rung_index: u8::MAX,
                slot: u8::MAX,
                i_raw: u16::MAX,
                direction: u8::MAX,
                classification: u8::MAX,
                initial_lead_in: u8::MAX,
                peak_torque_target_abs: u16::MAX,
            },
            "integral_observation_core",
        );
        assert_fits(
            &FociVelocityIntegralObservationRate {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                settled_low: i32::MIN,
                settled_mean: i32::MAX,
                settled_high: i32::MAX,
                settled_shift: u8::MAX,
                deficit_low_q: i32::MIN,
                deficit_high_q: i32::MAX,
            },
            "integral_observation_rate",
        );
        assert_fits(
            &FociVelocityIntegralObservationQuality {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                suffix_len: u16::MAX,
                level_or_exclusion: u8::MAX,
                selected_blocks: WIDE_U32,
                slope_mantissa: i32::MIN,
                slope_shift: u8::MAX,
                slope_half_width_mantissa: WIDE_U32,
                slope_half_width_shift: u8::MAX,
                lag_one_q: i32::MIN,
                lag_one_half_width_q: WIDE_U32,
                residual_mantissa: WIDE_U32,
                residual_shift: u8::MAX,
                tested_suffixes: u8::MAX,
            },
            "integral_observation_quality",
        );
        assert_fits(
            &FociVelocityIntegralRungCore {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                rung_index: u8::MAX,
                i_raw: u16::MAX,
                direction: u8::MAX,
                kind: u8::MAX,
                classification: u8::MAX,
                component_count: u8::MAX,
                usable_mask: u8::MAX,
                included_mask: u8::MAX,
                observation_classes: u16::MAX,
            },
            "integral_rung_core",
        );
        assert_fits(
            &FociVelocityIntegralRungComponent {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                rung_index: u8::MAX,
                direction: u8::MAX,
                component: u8::MAX,
                low_q: i32::MIN,
                high_q: i32::MAX,
            },
            "integral_rung_component",
        );
        assert_fits(
            &FociVelocityIntegralRunSummary {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                forward_eligible_mask: WIDE_U32,
                reverse_eligible_mask: WIDE_U32,
                opening_available_mask: u8::MAX,
                bookend_available_mask: u8::MAX,
                current_terminus_plus_one: u8::MAX,
                sufficient_direction_mask: u8::MAX,
            },
            "integral_run_summary",
        );
        assert_fits(
            &FociVelocityIntegralCurveInterval {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                kind: u8::MAX,
                direction: u8::MAX,
                rung_index: u8::MAX,
                low_q: i32::MIN,
                high_q: i32::MAX,
            },
            "integral_curve_interval",
        );
        assert_fits(
            &FociVelocityIntegralDrift {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                direction: u8::MAX,
                available: u8::MAX,
                opening_low_q: i32::MIN,
                opening_high_q: i32::MAX,
                closing_low_q: i32::MIN,
                closing_high_q: i32::MAX,
            },
            "integral_drift",
        );
        assert_fits(
            &FociVelocityIntegralStageBComparison {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                direction: u8::MAX,
                available: u8::MAX,
                stage_b_low_q16: i32::MIN,
                stage_b_high_q16: i32::MAX,
                expected_low_q: i32::MIN,
                expected_high_q: i32::MAX,
                opening_low_q: i32::MIN,
                opening_high_q: i32::MAX,
            },
            "integral_stage_b_comparison",
        );
        assert_fits(
            &FociVelocityIntegralReproductionCore {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                outcome: u8::MAX,
                direction_mask: u8::MAX,
            },
            "integral_reproduction_core",
        );
        assert_fits(
            &FociVelocityIntegralReproductionMask {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                direction: u8::MAX,
                previous_mask: WIDE_U32,
                current_mask: WIDE_U32,
                shared_mask: WIDE_U32,
                reproduced_mask: WIDE_U32,
                divergent_mask: WIDE_U32,
                previous_only_mask: WIDE_U32,
                current_only_mask: WIDE_U32,
            },
            "integral_reproduction_mask",
        );
        assert_fits(
            &FociVelocityIntegralReproductionInterval {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                direction: u8::MAX,
                rung_index: u8::MAX,
                kind: u8::MAX,
                previous_low_q: i32::MIN,
                previous_high_q: i32::MAX,
                current_low_q: i32::MIN,
                current_high_q: i32::MAX,
                result_low_or_gap_q: i32::MIN,
                result_high_q: i32::MAX,
            },
            "integral_reproduction_interval",
        );
        assert_fits(
            &FociVelocityIntegralReproductionDigest {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                previous_digest_low: WIDE_U32,
                previous_digest_high: WIDE_U32,
                current_digest_low: WIDE_U32,
                current_digest_high: WIDE_U32,
            },
            "integral_reproduction_digest",
        );
        assert_fits(
            &FociVelocityIntegralTerminalCore {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                outcome: u8::MAX,
                cause: u8::MAX,
                recovery_flags: u8::MAX,
                rest_boundary_rung_plus_one: u8::MAX,
                rest_boundary_slot_plus_one: u8::MAX,
                expected_observations: u16::MAX,
                emitted_observations: u16::MAX,
                expected_rungs: u16::MAX,
                emitted_rungs: u16::MAX,
            },
            "integral_terminal_core",
        );
        assert_fits(
            &FociVelocityIntegralTerminalIdentity {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                plan_digest_low: WIDE_U32,
                plan_digest_high: WIDE_U32,
                digest_low: WIDE_U32,
                digest_high: WIDE_U32,
            },
            "integral_terminal_identity",
        );
        assert_fits(
            &FociVelocityIntegralTerminalTiming {
                oid: u8::MAX,
                run_sequence: WIDE_U32,
                evidence_sequence: u16::MAX,
                fragment: u8::MAX,
                started_low: WIDE_U32,
                started_high: WIDE_U32,
                completed_low: WIDE_U32,
                completed_high: WIDE_U32,
            },
            "integral_terminal_timing",
        );
    }

    #[test]
    fn velocity_integral_digest_halves_round_trip_exactly() {
        let digest = 0x0123_4567_89ab_cdef_u64;
        let terminal = FociVelocityIntegralTerminalIdentity {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            fragment: 2,
            plan_digest_low: digest as u32,
            plan_digest_high: (digest >> 32) as u32,
            digest_low: (!digest) as u32,
            digest_high: ((!digest) >> 32) as u32,
        };
        assert_eq!(
            (u64::from(terminal.plan_digest_high) << 32) | u64::from(terminal.plan_digest_low),
            digest
        );
        assert_eq!(
            (u64::from(terminal.digest_high) << 32) | u64::from(terminal.digest_low),
            !digest
        );
    }
}

/// `foci_outer_safety_fault` reply — Stage 2 safety-envelope evidence.
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
    /// Signed accumulated position from the Stage 2 start, in encoder counts.
    pub position_counts: i32,
    /// Active signed-position window in encoder counts.
    pub position_window_counts: i32,
    /// Elapsed time from Stage 2 safety arming, in microseconds.
    pub elapsed_us: u32,
    /// Active primitive duration cap in microseconds.
    pub duration_cap_us: u32,
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
    /// Warning bitfield for non-fatal tune anomalies.
    pub warning_code: u8,
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

#[cfg(test)]
mod tune_result_reply_wire_tests {
    //! Pins the `foci_tune_result` wire order: the provenance block must
    //! serialize after every pre-existing field, and must not disturb their
    //! encoding. Mirrors the ascending-small-value idiom
    //! in `breakaway_reply_wire_tests` above (every field here VLQ-encodes
    //! values `1..=18` as a single raw byte -- see `encode_vlq_int`'s
    //! single-byte range of `-32..96`).
    use super::FociTuneResult;
    use ankyra::ScratchOutput;
    use ankyra::encoding::Writable;

    /// Pre-provenance field values, in struct-declaration order.
    const LEGACY_FIELD_BYTES: [u8; 12] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];

    fn reply_with_known_provenance() -> FociTuneResult {
        FociTuneResult {
            oid: 1,
            status: 2,
            velocity_p: 3,
            velocity_i: 4,
            velocity_limit: 5,
            position_p: 6,
            position_i: 7,
            velocity_filter_hz: 8,
            position_filter_hz: 9,
            flux_filter_hz: 10,
            torque_filter_hz: 11,
            warning_code: 12,
            probed_velocity_mrev_s: 13,
            d_eq_q: 14,
            confidence_q: 15,
            band_lower_percent: 16,
            band_upper_percent: 17,
            band_position_q: 18,
        }
    }

    #[test]
    fn provenance_fields_encode_after_existing_fields_in_declared_order() {
        let reply = reply_with_known_provenance();
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let encoded = scratch.result();

        assert_eq!(
            &encoded[..LEGACY_FIELD_BYTES.len()],
            &LEGACY_FIELD_BYTES[..],
            "existing fields must keep their wire order and values"
        );
        assert_eq!(
            encoded.len(),
            LEGACY_FIELD_BYTES.len() + 6,
            "provenance must append exactly six bytes to the existing payload"
        );
        assert_eq!(
            &encoded[LEGACY_FIELD_BYTES.len()..],
            &[13, 14, 15, 16, 17, 18],
            "known provenance values must round-trip through the encoder unchanged"
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
