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

/// `foci_trace_data` reply — chunked trace payload delivery.
///
/// Carries an arbitrary `&[u8]` chunk, so the struct is generic over
/// the borrow lifetime.
///
/// Wire name: `foci_trace_data`.
#[klipper_reply]
pub struct FociTraceData<'a> {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Byte offset of this chunk within the trace buffer.
    pub offset: u16,
    /// Status code (0 = more chunks follow, 1/2 = terminal status).
    pub status: u8,
    /// Raw trace-chunk bytes (empty on terminal-status replies).
    pub data: &'a [u8],
}

/// `foci_trace_info_result` reply — metadata for the active trace.
///
/// Wire name: `foci_trace_info_result`.
#[klipper_reply]
pub struct FociTraceInfoResult {
    /// Object id of the FOCI endpoint.
    pub oid: u8,
    /// Trace state (IDLE / RUNNING / COMPLETE).
    pub state: u8,
    /// Generation number for the current trace buffer.
    pub generation: u8,
    /// Trace preset selector.
    pub preset: u8,
    /// Number of samples captured.
    pub count: u16,
    /// Commissioning phase identifier at the time of the trace.
    pub phase: u8,
    /// Sampling period in microseconds.
    pub sample_period_us: u16,
    /// Number of samples that were dropped due to buffer pressure.
    pub dropped: u16,
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
