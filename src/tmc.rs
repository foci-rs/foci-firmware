//! TMC4671 command types and shared logic.
//!
//! These functions produce [`TmcRequest`] values that firmware crates
//! send to their board-specific SPI task. The actual SPI communication
//! and motor control is handled by the firmware, not this crate.

use core::sync::atomic::Ordering;

/// Request type for TMC4671 task communication.
///
/// Firmware crates send these to their board-specific SPI task via a
/// channel (e.g., `heapless::spsc` or `rtic_sync::channel`).
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub struct TmcRequest {
    /// Board-local TMC4671 channel index.
    pub channel: u8,
    /// Requested TMC operation.
    pub kind: TmcRequestKind,
}

impl TmcRequest {
    /// Build a TMC request for `channel`.
    pub const fn new(channel: u8, kind: TmcRequestKind) -> Self {
        Self { channel, kind }
    }
}

/// Runtime acceleration feedforward gains in permille.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AccelFeedforwardGains {
    /// Gain applied while step intervals shrink.
    pub accel_permille: u16,
    /// Gain applied while step intervals grow.
    pub decel_permille: u16,
}

/// Command-acceleration inputs for velocity-surface transient feedforward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityTransientFeedforwardInput {
    /// Precomputed velocity scale factor.
    pub scale_factor: u32,
    /// Step-timer clock frequency in Hz.
    pub clock_freq: u32,
    /// Current step interval in timer ticks.
    pub step_interval: u32,
    /// Klipper interval delta in timer ticks per step.
    pub interval_add: i16,
    /// True when the TMC4671 DIR input is high.
    pub dir_high: bool,
}

/// Runtime configuration for velocity-surface transient feedforward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityTransientFeedforwardConfig {
    /// Commanded lead time in microseconds.
    pub lead_time_us: u16,
    /// Gain in permille applied to the predicted velocity delta.
    pub gain_permille: u16,
    /// Maximum absolute offset in raw `PID_VELOCITY_OFFSET` counts.
    pub max_offset: u16,
}

/// Runtime configuration for velocity-proportional commutation phase advance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PhaseAdvanceConfig {
    /// Whether runtime phase advance is enabled.
    pub enable: bool,
    /// Signed gain in parts per million of TMC velocity-reference counts.
    pub gain_ppm: i32,
    /// Maximum absolute electrical-angle advance in ABN PHI_E counts.
    pub max_counts: u16,
    /// Absolute TMC velocity-reference counts below which output is zero.
    pub deadband_velocity: u16,
}

/// Computed runtime phase-advance output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PhaseAdvanceOutput {
    /// Signed electrical-angle advance in ABN PHI_E counts.
    pub counts: i16,
    /// Feature was configured enabled.
    pub enabled: bool,
    /// Output was zeroed by the velocity deadband.
    pub deadbanded: bool,
    /// Output reached `max_counts`.
    pub clamped: bool,
}

/// Maximum phase-advance cap accepted by firmware for live diagnostics.
pub const PHASE_ADVANCE_MAX_SAFE_COUNTS: u16 = 512;

/// Commanded-velocity and current inputs for q/d decoupling proxy feedforward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DecouplingFeedforwardInput {
    /// Command velocity reference in TMC position counts per PWM cycle.
    pub target_velocity: i32,
    /// Actual q-axis current from `INTERIM_FOC_IQ_ID`.
    pub foc_iq: i16,
    /// Actual d-axis current from `INTERIM_FOC_IQ_ID`.
    pub foc_id: i16,
}

/// Runtime configuration for q/d decoupling proxy feedforward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DecouplingFeedforwardConfig {
    /// Identified count-space resistance, scaled by 1000.
    pub r_int: u32,
    /// Identified count-space inductance in microseconds.
    pub l_int: u32,
    /// Motor pole-pair count.
    pub pole_pairs: u16,
    /// Position units per mechanical revolution for `target_velocity`.
    pub position_units_per_rev: u32,
    /// PWM/current-loop update rate in Hz.
    pub f_pwm_hz: u32,
    /// Maximum absolute output current-target offset.
    pub max_offset: u16,
}

/// Default velocity-transient update rate in Hz.
pub const VELOCITY_TRANSIENT_DEFAULT_RATE_HZ: u16 = 1_000;
/// Highest supported velocity-transient update rate in Hz.
pub const VELOCITY_TRANSIENT_MAX_RATE_HZ: u16 = 10_000;

/// Normalize a requested velocity-transient update rate.
pub fn normalize_velocity_transient_rate_hz(rate_hz: u16) -> u16 {
    if rate_hz == 0 {
        VELOCITY_TRANSIENT_DEFAULT_RATE_HZ
    } else {
        rate_hz.clamp(
            VELOCITY_TRANSIENT_DEFAULT_RATE_HZ,
            VELOCITY_TRANSIENT_MAX_RATE_HZ,
        )
    }
}

/// Return the control period in microseconds for a transient update rate.
pub fn velocity_transient_period_us(rate_hz: u16) -> u32 {
    1_000_000 / u32::from(normalize_velocity_transient_rate_hz(rate_hz))
}

/// Operation carried by a [`TmcRequest`].
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum TmcRequestKind {
    /// Set motor enable/disable.
    SetEnable(bool),
    /// Run ADC offset calibration, encoder alignment, closed-loop entry.
    Calibrate { oid: u8 },
    /// Set motor run current (milliamps to register counts).
    SetCurrent { run_ma: u32 },
    /// Configure ABN encoder resolution.
    SetEncoder { ppr: u32 },
    /// Set encoder count direction.
    SetEncoderDir { invert: bool },
    /// Trigger TMC4671 self-test diagnostic.
    SelfTest { oid: u8 },
    /// Dump all registers via klipper_output protocol.
    DumpRegisters { oid: u8 },
    /// Host sends pre-computed PI gains for flux/torque controllers.
    SetPidGains {
        flux_p: u16,
        flux_i: u16,
        torque_p: u16,
        torque_i: u16,
    },
    /// Start Stage 1 commissioning (diagnostics + current tune + closed-loop entry).
    /// Profile: 0=Conservative, 1=Balanced, 2=Stiff.
    Commission { oid: u8, profile: u8 },
    /// Start Stage 2 installed tuning (mechanical ID + velocity/position + filters).
    /// Profile: 0=Conservative, 1=Balanced, 2=Stiff.
    /// Mode: 0=Unloaded, 1=Nominal, 2=HighInertia.
    Tune {
        oid: u8,
        profile: u8,
        mode: u8,
        inner_lambda_us: u32,
        theta_e_us: u32,
        /// Current-loop ringing count from inner validation (>1 = ringing).
        current_ringing_count: u8,
        /// Current-loop effective bandwidth Hz from inner commissioning.
        current_bandwidth_hz: u32,
        /// Phase 5 tau_e_us measurement (or persisted default).
        tau_e_us: u32,
        /// Phase 5 tau crosscheck (or persisted default).
        tau_e_crosscheck_us: u32,
        /// Phase 5 tau residual in permille.
        tau_residual_permille: u16,
        /// Inner-confidence warning bitfield carried from Stage 1.
        inner_warning_flags: u8,
    },
    /// Set velocity biquad low-pass filter cutoff frequency.
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetVelocityFilter { filter_hz: u16 },
    /// Set torque biquad low-pass filter cutoff frequency.
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetTorqueFilter { filter_hz: u16 },
    /// Set position target biquad low-pass filter cutoff frequency.
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetPositionFilter { filter_hz: u16 },
    /// Set flux biquad low-pass filter cutoff frequency.
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetFluxFilter { filter_hz: u16 },
    /// Host sends pre-computed PI gains for position/velocity controllers.
    /// Range: 0..32767 (Q8.8 format, max 0x7FFF per datasheet page 120).
    SetPositionGains {
        position_p: u16,
        position_i: u16,
        velocity_p: u16,
        velocity_i: u16,
    },
    /// Enable or disable velocity feedforward in the 1ms tick loop.
    SetVelocityFeedforward { enable: bool, multiplier: u16 },
    /// Enable or disable velocity-surface transient feedforward.
    SetVelocityTransientFeedforward {
        enable: bool,
        /// Lead time in microseconds applied to command acceleration.
        lead_time_us: u16,
        /// Gain in permille applied to the predicted velocity delta.
        gain_permille: u16,
        /// Maximum absolute raw velocity-offset contribution.
        max_offset: u16,
        /// Requested update rate in Hz.
        rate_hz: u16,
    },
    /// Enable or disable acceleration feedforward in the 1ms tick loop.
    SetAccelFeedforward {
        enable: bool,
        /// Gain in permille applied while step intervals shrink.
        accel_gain_permille: u16,
        /// Gain in permille applied while step intervals grow.
        decel_gain_permille: u16,
    },
    /// Enable or disable q/d decoupling proxy feedforward.
    SetDecouplingFeedforward {
        enable: bool,
        /// Identified count-space resistance, scaled by 1000.
        r_int: u32,
        /// Identified count-space inductance in microseconds.
        l_int: u32,
        /// Motor pole-pair count.
        pole_pairs: u16,
        /// Position units per mechanical revolution.
        position_units_per_rev: u32,
        /// PWM/current-loop update rate in Hz.
        f_pwm_hz: u32,
        /// Maximum absolute output current-target offset.
        max_offset: u16,
    },
    /// Enable or disable diagnostic position-target lead in the 1ms tick loop.
    SetPositionLead {
        enable: bool,
        /// Lead gain in permille of the unmultiplied velocity reference.
        gain_permille: u16,
        /// Maximum absolute lead in logical step counts.
        max_counts: u16,
    },
    /// Enable or disable runtime commutation phase advance in the 1 ms tick loop.
    SetPhaseAdvance {
        enable: bool,
        /// Signed gain in ppm of the command velocity reference.
        gain_ppm: i32,
        /// Maximum absolute advance in ABN PHI_E counts.
        max_counts: u16,
        /// Absolute TMC velocity-reference deadband in counts.
        deadband: u16,
    },
    /// Set operational PIDOUT_UQ_UD_LIMITS for authority diagnostics.
    SetVoltageLimit { voltage_limit: u32 },
    /// Run a bounded current-loop step diagnostic while already enabled.
    CurrentStepTest {
        /// Object id used by the reply.
        oid: u8,
        /// Signed torque-current target in TMC4671 current counts.
        target: i16,
        /// Hold duration for the applied target.
        duration_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded current-vector step diagnostic while already enabled.
    CurrentVectorStepTest {
        /// Object id used by the reply.
        oid: u8,
        /// Signed torque-current target in TMC4671 current counts.
        torque_target: i16,
        /// Signed flux-current target in TMC4671 current counts.
        flux_target: i16,
        /// Hold duration for the applied target.
        duration_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded torque-current pulse and sample it before the old dwell floor.
    CurrentTorqueSampleTest {
        /// Object id used by the reply.
        oid: u8,
        /// Signed torque-current target in TMC4671 current counts.
        target: i16,
        /// Signed flux-current target in TMC4671 current counts.
        flux_target: i16,
        /// Delay between applying the target and sampling, in milliseconds.
        sample_delay_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded torque-offset pulse in position mode and sample it.
    PositionTorqueOffsetSampleTest {
        /// Object id used by the reply.
        oid: u8,
        /// Signed torque offset in TMC4671 current counts.
        target: i16,
        /// Delay between applying the offset and sampling, in milliseconds.
        sample_delay_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded open-loop voltage pulse and sample it.
    VoltageStepTest {
        /// Object id used by the reply.
        oid: u8,
        /// Signed q-axis voltage component for UQ_UD_EXT.
        uq_ext: i16,
        /// Signed d-axis voltage component for UQ_UD_EXT.
        ud_ext: i16,
        /// Delay between applying the voltage vector and sampling, in milliseconds.
        sample_delay_ms: u16,
    },
    /// Start the per-tick trace buffer with the selected preset.
    TraceStart { oid: u8, preset: u8 },
    /// Stop the per-tick trace buffer, preserving samples for fetch.
    TraceStop { oid: u8 },
    /// Query trace buffer metadata.
    TraceInfo { oid: u8 },
    /// Fetch a trace sample by offset. generation must match current generation.
    TraceFetch {
        oid: u8,
        offset: u16,
        generation: u8,
    },
    /// Write a TMC4671 register via SPI (dev-only).
    #[cfg(feature = "dev")]
    Write { addr: u8, value: u32 },
    /// Read a TMC4671 register via SPI (dev-only).
    #[cfg(feature = "dev")]
    Read { oid: u8, addr: u8 },
    /// Trigger TMC4671 initialization sequence (dev-only).
    #[cfg(feature = "dev")]
    Init,
    /// Test motor rotation by sweeping PHI_E_EXT (dev-only).
    #[cfg(feature = "dev")]
    TestSpin { revolutions: u8 },
}

/// Build a `SetCurrent` request.
pub fn tmc_set_current(channel: u8, run_ma: u32) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetCurrent { run_ma })
}

/// Compute step width from encoder PPR and pole pairs for hardware step/dir.
///
/// Returns `(TmcRequest, step_width)` where `step_width` is the value to
/// write to TMC4671 STEP_WIDTH register (0x78). The board crate writes
/// the step width directly to the TMC4671 register.
///
/// Step width formula: `65536 * pole_pairs / (ppr * 4)`
pub fn tmc_set_encoder_hw(channel: u8, ppr: u32, pole_pairs: u32) -> (TmcRequest, i32) {
    let step_width = if ppr > 0 {
        let quad_ppr = (ppr as u64) * 4;
        (65536u64 * pole_pairs as u64 / quad_ppr) as i32
    } else {
        0
    };
    (
        TmcRequest::new(channel, TmcRequestKind::SetEncoder { ppr }),
        step_width,
    )
}

/// Build a `SetEncoderDir` request.
pub fn tmc_set_encoder_dir(channel: u8, invert: bool) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetEncoderDir { invert })
}

/// Build a `SelfTest` request.
pub fn tmc_selftest(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SelfTest { oid })
}

/// Build a `Calibrate` request.
pub fn tmc_calibrate(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Calibrate { oid })
}

/// Build a `DumpRegisters` request.
pub fn tmc_dump_registers(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::DumpRegisters { oid })
}

/// Build a `SetPidGains` request.
pub fn tmc_set_pid_gains(
    channel: u8,
    flux_p: u16,
    flux_i: u16,
    torque_p: u16,
    torque_i: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetPidGains {
            flux_p,
            flux_i,
            torque_p,
            torque_i,
        },
    )
}

/// Build a `Commission` request (Stage 1: diagnostics + current tune + closed-loop entry).
pub fn tmc_commission(channel: u8, oid: u8, profile: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Commission { oid, profile })
}

/// Build a `Tune` request (Stage 2: mechanical ID + velocity/position + filters).
#[allow(clippy::too_many_arguments)]
pub fn tmc_tune(
    channel: u8,
    oid: u8,
    profile: u8,
    mode: u8,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing_count: u8,
    current_bandwidth_hz: u32,
    tau_e_us: u32,
    tau_e_crosscheck_us: u32,
    tau_residual_permille: u16,
    inner_warning_flags: u8,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::Tune {
            oid,
            profile,
            mode,
            inner_lambda_us,
            theta_e_us,
            current_ringing_count,
            current_bandwidth_hz,
            tau_e_us,
            tau_e_crosscheck_us,
            tau_residual_permille,
            inner_warning_flags,
        },
    )
}

/// Build a `SetVelocityFilter` request.
pub fn tmc_set_velocity_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetVelocityFilter { filter_hz })
}

/// Build a `SetTorqueFilter` request.
pub fn tmc_set_torque_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetTorqueFilter { filter_hz })
}

/// Build a `SetPositionFilter` request.
pub fn tmc_set_position_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetPositionFilter { filter_hz })
}

/// Build a `SetFluxFilter` request.
pub fn tmc_set_flux_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetFluxFilter { filter_hz })
}

/// Build a `SetPositionGains` request.
pub fn tmc_set_position_gains(
    channel: u8,
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetPositionGains {
            position_p,
            position_i,
            velocity_p,
            velocity_i,
        },
    )
}

/// Build a `SetVelocityFeedforward` request.
pub fn tmc_set_velocity_feedforward(channel: u8, enable: bool, multiplier: u16) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetVelocityFeedforward { enable, multiplier },
    )
}

/// Build a `SetVelocityTransientFeedforward` request.
pub fn tmc_set_velocity_transient_feedforward(
    channel: u8,
    enable: bool,
    lead_time_us: u16,
    gain_permille: u16,
    max_offset: u16,
    rate_hz: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetVelocityTransientFeedforward {
            enable,
            lead_time_us,
            gain_permille,
            max_offset,
            rate_hz: normalize_velocity_transient_rate_hz(rate_hz),
        },
    )
}

/// Build a `SetAccelFeedforward` request.
pub fn tmc_set_accel_feedforward(
    channel: u8,
    enable: bool,
    accel_gain_permille: u16,
    decel_gain_permille: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetAccelFeedforward {
            enable,
            accel_gain_permille,
            decel_gain_permille,
        },
    )
}

/// Build a `SetDecouplingFeedforward` request.
#[allow(clippy::too_many_arguments)]
pub fn tmc_set_decoupling_feedforward(
    channel: u8,
    enable: bool,
    r_int: u32,
    l_int: u32,
    pole_pairs: u16,
    position_units_per_rev: u32,
    f_pwm_hz: u32,
    max_offset: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetDecouplingFeedforward {
            enable,
            r_int,
            l_int,
            pole_pairs,
            position_units_per_rev,
            f_pwm_hz,
            max_offset,
        },
    )
}

/// Build a `SetPositionLead` request.
pub fn tmc_set_position_lead(
    channel: u8,
    enable: bool,
    gain_permille: u16,
    max_counts: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetPositionLead {
            enable,
            gain_permille,
            max_counts,
        },
    )
}

/// Build a `SetPhaseAdvance` request.
pub fn tmc_set_phase_advance(
    channel: u8,
    enable: bool,
    gain_ppm: i32,
    max_counts: u16,
    deadband: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetPhaseAdvance {
            enable,
            gain_ppm,
            max_counts,
            deadband,
        },
    )
}

/// Build a `SetVoltageLimit` request.
pub fn tmc_set_voltage_limit(channel: u8, voltage_limit: u32) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetVoltageLimit { voltage_limit })
}

/// Build a `CurrentStepTest` request.
pub fn tmc_current_step_test(
    channel: u8,
    oid: u8,
    target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::CurrentStepTest {
            oid,
            target,
            duration_ms,
            voltage_limit,
        },
    )
}

/// Build a `CurrentVectorStepTest` request.
pub fn tmc_current_vector_step_test(
    channel: u8,
    oid: u8,
    torque_target: i16,
    flux_target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::CurrentVectorStepTest {
            oid,
            torque_target,
            flux_target,
            duration_ms,
            voltage_limit,
        },
    )
}

/// Build a `CurrentTorqueSampleTest` request.
pub fn tmc_current_torque_sample_test(
    channel: u8,
    oid: u8,
    target: i16,
    flux_target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::CurrentTorqueSampleTest {
            oid,
            target,
            flux_target,
            sample_delay_ms,
            voltage_limit,
        },
    )
}

/// Build a `PositionTorqueOffsetSampleTest` request.
pub fn tmc_position_torque_offset_sample_test(
    channel: u8,
    oid: u8,
    target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::PositionTorqueOffsetSampleTest {
            oid,
            target,
            sample_delay_ms,
            voltage_limit,
        },
    )
}

/// Build a `VoltageStepTest` request.
pub fn tmc_voltage_step_test(
    channel: u8,
    oid: u8,
    uq_ext: i16,
    ud_ext: i16,
    sample_delay_ms: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::VoltageStepTest {
            oid,
            uq_ext,
            ud_ext,
            sample_delay_ms,
        },
    )
}

/// Build a `SetEnable` request.
pub fn tmc_set_enable(channel: u8, enable: bool) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetEnable(enable))
}

/// Set whether a raw enable may trigger firmware-side auto-calibration.
pub fn set_auto_calibrate_on_enable_allowed(channel: u8, enable: bool) -> bool {
    let Some(state) = crate::state::tmc::tmc_channel(channel) else {
        return false;
    };
    state
        .auto_calibrate_on_enable_allowed
        .store(enable, Ordering::Relaxed);
    true
}

/// Return whether raw enable auto-calibration is currently allowed.
pub fn auto_calibrate_on_enable_allowed(channel: u8) -> bool {
    crate::state::tmc::tmc_channel(channel)
        .map(|state| {
            state
                .auto_calibrate_on_enable_allowed
                .load(Ordering::Relaxed)
        })
        .unwrap_or(false)
}

/// Build a `TraceStart` request.
pub fn tmc_trace_start(channel: u8, oid: u8, preset: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::TraceStart { oid, preset })
}

/// Build a `TraceStop` request.
pub fn tmc_trace_stop(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::TraceStop { oid })
}

/// Build a `TraceInfo` request.
pub fn tmc_trace_info(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::TraceInfo { oid })
}

/// Build a `TraceFetch` request.
pub fn tmc_trace_fetch(channel: u8, oid: u8, offset: u16, generation: u8) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::TraceFetch {
            oid,
            offset,
            generation,
        },
    )
}

/// Precompute the velocity feedforward scale factor.
///
/// The result encodes `step_width * clock_freq / f_pwm` so that the
/// per-tick feedforward computation is a single division:
/// `velocity_tmc = scale_factor / step_interval`.
///
/// Uses `u64` intermediate to avoid overflow on the multiplication
/// (`step_width * clock_freq` can exceed `u32::MAX`).
///
/// Returns 0 if `step_width == 0` or `f_pwm == 0` (feedforward disabled).
pub fn compute_velocity_scale_factor(step_width: u32, clock_freq: u32, f_pwm: u32) -> u32 {
    if step_width == 0 || f_pwm == 0 {
        return 0;
    }
    let numerator = step_width as u64 * clock_freq as u64;
    (numerator / f_pwm as u64) as u32
}

/// Compute the velocity feedforward offset value for PID_VELOCITY_OFFSET.
///
/// # Parameters
/// - `scale_factor`: precomputed by [`compute_velocity_scale_factor`]
/// - `step_interval`: current step interval in TIM5 clock ticks (from atomic)
/// - `dir_high`: true if the TMC4671 DIR input is high
///
/// Returns 0 if `step_interval == 0` (motor idle) or `scale_factor == 0`
/// (feedforward not configured).
///
/// The result is in TMC4671 velocity units (position change per PWM period)
/// and is written directly to PID_VELOCITY_OFFSET (0x67, s32).
///
/// With a positive STEP_WIDTH, the TMC4671 STEP/DIR input reports a
/// negative velocity for DIR high and a positive velocity for DIR low.
pub fn compute_velocity_offset(scale_factor: u32, step_interval: u32, dir_high: bool) -> i32 {
    compute_velocity_offset_scaled(scale_factor, step_interval, dir_high, 1)
}

/// Compute the signed command-velocity reference used by trace diagnostics.
///
/// This uses the same TMC4671 sign convention as `PID_VELOCITY_OFFSET`, but it
/// intentionally does not apply the runtime feedforward multiplier. The value
/// is a software reference derived from Klipper STEP/DIR timing, not a TMC
/// register read.
pub fn compute_command_velocity_reference(
    scale_factor: u32,
    step_interval: u32,
    dir_high: bool,
) -> i32 {
    compute_velocity_offset_scaled(scale_factor, step_interval, dir_high, 1)
}

/// Compute the velocity feedforward offset with a runtime multiplier.
///
/// `multiplier == 0` disables the offset. Positive products saturate to
/// `i32::MAX`; negative products use the same magnitude limit so the value
/// remains symmetric around zero.
pub fn compute_velocity_offset_scaled(
    scale_factor: u32,
    step_interval: u32,
    dir_high: bool,
    multiplier: u16,
) -> i32 {
    if step_interval == 0 || scale_factor == 0 {
        return 0;
    }
    let magnitude = (scale_factor / step_interval) as u64 * multiplier as u64;
    let magnitude = magnitude.min(i32::MAX as u64) as i32;
    if dir_high { -magnitude } else { magnitude }
}

/// Compute a bounded logical position-target lead from the command velocity.
///
/// The returned value is in firmware logical step counts, not TMC4671
/// electrical-position counts. With OpenFFBoard's current hardware STEP/DIR
/// path, DIR high increments the firmware step counter even though the TMC
/// velocity monitor reports a negative velocity for that same direction.
pub fn compute_position_lead_counts(
    scale_factor: u32,
    step_interval: u32,
    dir_high: bool,
    gain_permille: u16,
    max_counts: u16,
) -> i16 {
    if scale_factor == 0 || step_interval == 0 || gain_permille == 0 || max_counts == 0 {
        return 0;
    }

    let velocity = compute_command_velocity_reference(scale_factor, step_interval, dir_high);
    let magnitude = (velocity.unsigned_abs() as u64 * gain_permille as u64 / 1000)
        .min(max_counts as u64)
        .min(i16::MAX as u64) as i16;
    if magnitude == 0 {
        return 0;
    }

    if dir_high { magnitude } else { -magnitude }
}

/// Compute a relative `PID_POSITION_TARGET` with diagnostic lead applied.
///
/// `current_target` is the live TMC4671 target register value. The function
/// removes the previously applied diagnostic delta before applying the new
/// lead, so repeated calls do not accumulate lead into the TMC's target frame.
/// It returns the register value to write and the new applied delta in raw
/// TMC4671 position counts.
pub fn compute_position_lead_target(
    current_target: u32,
    previous_lead_scaled: i32,
    lead_counts: i16,
    step_width: u32,
) -> (u32, i32) {
    if step_width == 0 {
        return (current_target, 0);
    }

    let new_lead_scaled = i32::from(lead_counts).wrapping_mul(step_width as i32);
    let base_target = (current_target as i32).wrapping_sub(previous_lead_scaled);
    let target = base_target.wrapping_add(new_lead_scaled);
    (target as u32, new_lead_scaled)
}

/// Compute a bounded velocity-loop transient offset from command acceleration.
///
/// The value predicts how much the command velocity will change over
/// `lead_time_us`, scales it by `gain_permille`, and applies it through the
/// same sign convention as `PID_VELOCITY_OFFSET`. It is intended to be added to
/// the existing velocity feedforward offset and written as one combined
/// register value.
pub fn compute_velocity_transient_offset(
    input: VelocityTransientFeedforwardInput,
    config: VelocityTransientFeedforwardConfig,
) -> i32 {
    if input.scale_factor == 0
        || input.clock_freq == 0
        || input.step_interval == 0
        || input.interval_add == 0
        || config.lead_time_us == 0
        || config.gain_permille == 0
        || config.max_offset == 0
    {
        return 0;
    }

    let interval = input.step_interval as u128;
    let denominator = interval
        .saturating_mul(interval)
        .saturating_mul(interval)
        .saturating_mul(1_000_000)
        .saturating_mul(1000);
    if denominator == 0 {
        return 0;
    }

    let numerator = input.interval_add.unsigned_abs() as u128
        * input.scale_factor as u128
        * input.clock_freq as u128
        * config.lead_time_us as u128
        * config.gain_permille as u128;
    let magnitude = (numerator / denominator)
        .min(config.max_offset as u128)
        .min(i32::MAX as u128) as i32;
    if magnitude == 0 {
        return 0;
    }

    let accel_sign = if input.interval_add < 0 { 1 } else { -1 };
    let direction_sign = if input.dir_high { -1 } else { 1 };
    if accel_sign * direction_sign < 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// Compute a bounded runtime commutation phase advance.
///
/// `target_velocity` is the command-derived TMC velocity reference in
/// electrical-angle counts per PWM period, using the same sign convention as
/// `PID_VELOCITY_OFFSET`.
pub fn compute_phase_advance_counts(
    target_velocity: i32,
    config: PhaseAdvanceConfig,
) -> PhaseAdvanceOutput {
    if !config.enable || config.gain_ppm == 0 || config.max_counts == 0 {
        return PhaseAdvanceOutput {
            counts: 0,
            enabled: config.enable,
            deadbanded: false,
            clamped: false,
        };
    }

    if target_velocity.unsigned_abs() <= u32::from(config.deadband_velocity) {
        return PhaseAdvanceOutput {
            counts: 0,
            enabled: true,
            deadbanded: true,
            clamped: false,
        };
    }

    let raw = i64::from(target_velocity) * i64::from(config.gain_ppm);
    let scaled = raw / 1_000_000;
    let limit = i64::from(
        config
            .max_counts
            .min(PHASE_ADVANCE_MAX_SAFE_COUNTS)
            .min(i16::MAX as u16),
    );
    let clamped = scaled.clamp(-limit, limit);
    let limit_ppm = limit.saturating_mul(1_000_000);

    PhaseAdvanceOutput {
        counts: clamped as i16,
        enabled: true,
        deadbanded: false,
        clamped: raw.unsigned_abs() > limit_ppm as u64,
    }
}

/// Pack a runtime phase advance into ABN_DECODER_PHI_E_PHI_M_OFFSET.
///
/// The lower halfword is PHI_M and is preserved. The upper halfword is PHI_E
/// and receives the signed wrapping advance.
pub fn pack_abn_phi_offset(base_register: u32, phase_advance_counts: i16) -> u32 {
    let phi_m = base_register & 0x0000_FFFF;
    let base_phi_e = (base_register >> 16) as u16 as i16;
    let next_phi_e = base_phi_e.wrapping_add(phase_advance_counts);
    phi_m | (u32::from(next_phi_e as u16) << 16)
}

fn scale_decoupling_axis(
    target_velocity: i32,
    current: i16,
    config: DecouplingFeedforwardConfig,
) -> i16 {
    if target_velocity == 0
        || current == 0
        || config.r_int == 0
        || config.l_int == 0
        || config.pole_pairs == 0
        || config.position_units_per_rev == 0
        || config.f_pwm_hz == 0
        || config.max_offset == 0
    {
        return 0;
    }

    const TAU_MICRO: u128 = 6_283_185;
    let magnitude = target_velocity.unsigned_abs() as u128
        * current.unsigned_abs() as u128
        * config.l_int as u128
        * config.pole_pairs as u128
        * config.f_pwm_hz as u128
        * TAU_MICRO
        * 1000;
    let denominator = config.position_units_per_rev as u128
        * 1_000_000u128
        * 1_000_000u128
        * config.r_int as u128;
    if denominator == 0 {
        return 0;
    }

    let clamped = (magnitude / denominator)
        .min(config.max_offset as u128)
        .min(i16::MAX as u128) as i16;
    if (target_velocity < 0) ^ (current < 0) {
        -clamped
    } else {
        clamped
    }
}

/// Compute a bounded current-target proxy for q/d voltage decoupling.
///
/// The returned tuple is `(torque_offset, flux_offset)` for
/// `PID_TORQUE_FLUX_OFFSET`. This intentionally stays in count space:
/// `L_int` maps current-count and electrical-speed terms into voltage counts,
/// then `R_int` maps the voltage-count estimate back into current counts.
pub fn compute_decoupling_current_offsets(
    input: DecouplingFeedforwardInput,
    config: DecouplingFeedforwardConfig,
) -> (i16, i16) {
    let torque = scale_decoupling_axis(input.target_velocity, input.foc_id, config);
    let flux = -scale_decoupling_axis(input.target_velocity, input.foc_iq, config);
    (torque, flux)
}

/// Compute the diagnostic acceleration feedforward torque offset.
///
/// The input `interval_add` is Klipper's `queue_step add`: the interval change
/// in TIM5 ticks per step. A negative add means the step interval is shrinking,
/// so velocity is increasing along the current TMC STEP/DIR velocity direction.
///
/// `gain_permille` is a live diagnostic gain. `1000` applies the raw
/// acceleration estimate, `500` applies half, and `0` disables the offset.
/// The result is clamped to `torque_limit` and is intended for the torque field
/// of `PID_TORQUE_FLUX_OFFSET`.
pub fn compute_accel_torque_offset(
    scale_factor: u32,
    clock_freq: u32,
    step_interval: u32,
    interval_add: i16,
    dir_high: bool,
    gain_permille: u16,
    torque_limit: u32,
) -> i16 {
    if scale_factor == 0
        || clock_freq == 0
        || step_interval == 0
        || interval_add == 0
        || gain_permille == 0
        || torque_limit == 0
    {
        return 0;
    }

    let interval = step_interval as u128;
    let denominator = interval
        .saturating_mul(interval)
        .saturating_mul(interval)
        .saturating_mul(1000);
    if denominator == 0 {
        return 0;
    }

    let numerator = interval_add.unsigned_abs() as u128
        * scale_factor as u128
        * clock_freq as u128
        * gain_permille as u128;
    let current_limit = torque_limit.min(i16::MAX as u32) as u128;
    let magnitude = (numerator / denominator).min(current_limit) as i16;
    if magnitude == 0 {
        return 0;
    }

    let accel_sign = if interval_add < 0 { 1 } else { -1 };
    let direction_sign = if dir_high { -1 } else { 1 };
    if accel_sign * direction_sign < 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// Compute acceleration feedforward with independent accel/decel gains.
pub fn compute_accel_torque_offset_split(
    scale_factor: u32,
    clock_freq: u32,
    step_interval: u32,
    interval_add: i16,
    dir_high: bool,
    gains: AccelFeedforwardGains,
    torque_limit: u32,
) -> i16 {
    let gain_permille = if interval_add < 0 {
        gains.accel_permille
    } else if interval_add > 0 {
        gains.decel_permille
    } else {
        0
    };
    compute_accel_torque_offset(
        scale_factor,
        clock_freq,
        step_interval,
        interval_add,
        dir_high,
        gain_permille,
        torque_limit,
    )
}

/// Pack signed torque and flux targets for PID_TORQUE_FLUX_TARGET.
pub fn pack_torque_flux_target(torque: i16, flux: i16) -> u32 {
    ((torque as u16 as u32) << 16) | flux as u16 as u32
}

/// Decode the signed upper 16 bits of a TMC4671 packed register.
pub fn decode_signed_upper_half(value: u32) -> i16 {
    ((value >> 16) & 0xffff) as u16 as i16
}

/// Decode the signed lower 16 bits of a TMC4671 packed register.
pub fn decode_signed_lower_half(value: u32) -> i16 {
    (value & 0xffff) as u16 as i16
}

/// Build a `Write` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_write_register(channel: u8, addr: u8, value: u32) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Write { addr, value })
}

/// Build a `Read` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_read_register(channel: u8, oid: u8, addr: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Read { oid, addr })
}

/// Build an `Init` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_init(channel: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Init)
}

/// Build a `TestSpin` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_test_spin(channel: u8, revolutions: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::TestSpin { revolutions })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tmc_set_current_includes_channel() {
        let req = tmc_set_current(1, 800);

        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            TmcRequestKind::SetCurrent { run_ma: 800 }
        ));
    }

    #[test]
    fn tmc_set_current_builds_request() {
        assert_eq!(
            tmc_set_current(0, 500),
            TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 500 })
        );
    }

    #[test]
    fn tmc_set_encoder_hw_computes_step_width() {
        let (req, step_width) = tmc_set_encoder_hw(0, 1000, 50);
        assert_eq!(
            req,
            TmcRequest::new(0, TmcRequestKind::SetEncoder { ppr: 1000 })
        );
        assert_eq!(step_width, 819);
    }

    #[test]
    fn tmc_set_encoder_hw_computes_mechanical_step_width() {
        let (req, step_width) = tmc_set_encoder_hw(0, 1000, 1);
        assert_eq!(
            req,
            TmcRequest::new(0, TmcRequestKind::SetEncoder { ppr: 1000 })
        );
        assert_eq!(step_width, 16);
    }

    #[test]
    fn tmc_set_encoder_hw_zero_ppr() {
        let (req, step_width) = tmc_set_encoder_hw(0, 0, 50);
        assert_eq!(
            req,
            TmcRequest::new(0, TmcRequestKind::SetEncoder { ppr: 0 })
        );
        assert_eq!(step_width, 0);
    }

    #[test]
    fn tmc_set_encoder_dir_builds_request() {
        assert_eq!(
            tmc_set_encoder_dir(0, true),
            TmcRequest::new(0, TmcRequestKind::SetEncoderDir { invert: true })
        );
    }

    #[test]
    fn tmc_selftest_builds_request() {
        assert_eq!(
            tmc_selftest(0, 5),
            TmcRequest::new(0, TmcRequestKind::SelfTest { oid: 5 })
        );
    }

    #[test]
    fn tmc_calibrate_builds_request() {
        assert_eq!(
            tmc_calibrate(0, 7),
            TmcRequest::new(0, TmcRequestKind::Calibrate { oid: 7 })
        );
    }

    #[test]
    fn tmc_dump_registers_builds_request() {
        assert_eq!(
            tmc_dump_registers(0, 1),
            TmcRequest::new(0, TmcRequestKind::DumpRegisters { oid: 1 })
        );
    }

    #[test]
    fn tmc_trace_start_builds_request() {
        assert_eq!(
            tmc_trace_start(0, 7, crate::trace::PRESET_FULL),
            TmcRequest::new(
                0,
                TmcRequestKind::TraceStart {
                    oid: 7,
                    preset: crate::trace::PRESET_FULL,
                }
            )
        );
    }

    #[test]
    fn tmc_trace_stop_builds_request() {
        assert_eq!(
            tmc_trace_stop(0, 7),
            TmcRequest::new(0, TmcRequestKind::TraceStop { oid: 7 })
        );
    }

    #[test]
    fn tmc_set_pid_gains_builds_request() {
        assert_eq!(
            tmc_set_pid_gains(0, 100, 200, 300, 400),
            TmcRequest::new(
                0,
                TmcRequestKind::SetPidGains {
                    flux_p: 100,
                    flux_i: 200,
                    torque_p: 300,
                    torque_i: 400,
                }
            )
        );
    }

    #[test]
    fn tmc_current_step_test_builds_request() {
        assert_eq!(
            tmc_current_step_test(0, 7, 250, 80, 12_000),
            TmcRequest::new(
                0,
                TmcRequestKind::CurrentStepTest {
                    oid: 7,
                    target: 250,
                    duration_ms: 80,
                    voltage_limit: 12_000,
                }
            )
        );
    }

    #[test]
    fn tmc_current_vector_step_test_builds_request() {
        assert_eq!(
            tmc_current_vector_step_test(0, 7, 0, 250, 80, 12_000),
            TmcRequest::new(
                0,
                TmcRequestKind::CurrentVectorStepTest {
                    oid: 7,
                    torque_target: 0,
                    flux_target: 250,
                    duration_ms: 80,
                    voltage_limit: 12_000,
                }
            )
        );
    }

    #[test]
    fn tmc_current_torque_sample_test_builds_vector_request() {
        assert_eq!(
            tmc_current_torque_sample_test(0, 7, 500, -125, 5, 29_000),
            TmcRequest::new(
                0,
                TmcRequestKind::CurrentTorqueSampleTest {
                    oid: 7,
                    target: 500,
                    flux_target: -125,
                    sample_delay_ms: 5,
                    voltage_limit: 29_000,
                }
            )
        );
    }

    #[test]
    fn tmc_position_torque_offset_sample_test_builds_request() {
        assert_eq!(
            tmc_position_torque_offset_sample_test(0, 7, 500, 2, 29_000),
            TmcRequest::new(
                0,
                TmcRequestKind::PositionTorqueOffsetSampleTest {
                    oid: 7,
                    target: 500,
                    sample_delay_ms: 2,
                    voltage_limit: 29_000,
                }
            )
        );
    }

    #[test]
    fn tmc_voltage_step_test_builds_request() {
        assert_eq!(
            tmc_voltage_step_test(0, 7, 512, -256, 2),
            TmcRequest::new(
                0,
                TmcRequestKind::VoltageStepTest {
                    oid: 7,
                    uq_ext: 512,
                    ud_ext: -256,
                    sample_delay_ms: 2,
                }
            )
        );
    }

    #[test]
    fn pack_torque_target_places_signed_torque_in_upper_half() {
        assert_eq!(pack_torque_flux_target(250, 0), 0x00fa_0000);
        assert_eq!(pack_torque_flux_target(-250, 0), 0xff06_0000);
        assert_eq!(pack_torque_flux_target(0, -3), 0x0000_fffd);
    }

    #[test]
    fn signed_half_decoders_extract_torque_and_flux() {
        let value = 0xff06_0005;

        assert_eq!(decode_signed_upper_half(value), -250);
        assert_eq!(decode_signed_lower_half(value), 5);
    }

    #[test]
    fn tmc_set_velocity_filter_builds_request() {
        assert_eq!(
            tmc_set_velocity_filter(0, 200),
            TmcRequest::new(0, TmcRequestKind::SetVelocityFilter { filter_hz: 200 })
        );
    }

    #[test]
    fn tmc_set_torque_filter_builds_request() {
        assert_eq!(
            tmc_set_torque_filter(0, 500),
            TmcRequest::new(0, TmcRequestKind::SetTorqueFilter { filter_hz: 500 })
        );
    }

    #[test]
    fn tmc_set_position_filter_builds_request() {
        assert_eq!(
            tmc_set_position_filter(0, 100),
            TmcRequest::new(0, TmcRequestKind::SetPositionFilter { filter_hz: 100 })
        );
    }

    #[test]
    fn tmc_set_flux_filter_builds_request() {
        assert_eq!(
            tmc_set_flux_filter(0, 300),
            TmcRequest::new(0, TmcRequestKind::SetFluxFilter { filter_hz: 300 })
        );
    }

    #[test]
    fn tmc_set_position_gains_builds_request() {
        assert_eq!(
            tmc_set_position_gains(0, 640, 5, 1152, 0),
            TmcRequest::new(
                0,
                TmcRequestKind::SetPositionGains {
                    position_p: 640,
                    position_i: 5,
                    velocity_p: 1152,
                    velocity_i: 0,
                }
            )
        );
    }

    #[test]
    fn tmc_set_velocity_feedforward_builds_request() {
        assert_eq!(
            tmc_set_velocity_feedforward(0, true, 4),
            TmcRequest::new(
                0,
                TmcRequestKind::SetVelocityFeedforward {
                    enable: true,
                    multiplier: 4,
                }
            )
        );
    }

    #[test]
    fn tmc_set_velocity_transient_feedforward_builds_request() {
        assert_eq!(
            tmc_set_velocity_transient_feedforward(0, true, 750, 1000, 2000, 10_000),
            TmcRequest::new(
                0,
                TmcRequestKind::SetVelocityTransientFeedforward {
                    enable: true,
                    lead_time_us: 750,
                    gain_permille: 1000,
                    max_offset: 2000,
                    rate_hz: 10_000,
                }
            )
        );
    }

    #[test]
    fn velocity_transient_rate_is_normalized_to_supported_periods() {
        assert_eq!(normalize_velocity_transient_rate_hz(0), 1_000);
        assert_eq!(normalize_velocity_transient_rate_hz(500), 1_000);
        assert_eq!(normalize_velocity_transient_rate_hz(10_000), 10_000);
        assert_eq!(normalize_velocity_transient_rate_hz(25_000), 10_000);

        assert_eq!(velocity_transient_period_us(1_000), 1_000);
        assert_eq!(velocity_transient_period_us(10_000), 100);
        assert_eq!(velocity_transient_period_us(25_000), 100);
    }

    #[test]
    fn tmc_set_accel_feedforward_builds_request() {
        assert_eq!(
            tmc_set_accel_feedforward(0, true, 1000, 500),
            TmcRequest::new(
                0,
                TmcRequestKind::SetAccelFeedforward {
                    enable: true,
                    accel_gain_permille: 1000,
                    decel_gain_permille: 500,
                }
            )
        );
    }

    #[test]
    fn tmc_set_decoupling_feedforward_builds_request() {
        assert_eq!(
            tmc_set_decoupling_feedforward(0, true, 3000, 4095, 50, 65_536, 25_000, 500),
            TmcRequest::new(
                0,
                TmcRequestKind::SetDecouplingFeedforward {
                    enable: true,
                    r_int: 3000,
                    l_int: 4095,
                    pole_pairs: 50,
                    position_units_per_rev: 65_536,
                    f_pwm_hz: 25_000,
                    max_offset: 500,
                }
            )
        );
    }

    #[test]
    fn tmc_set_position_lead_builds_request() {
        assert_eq!(
            tmc_set_position_lead(0, true, 10, 20),
            TmcRequest::new(
                0,
                TmcRequestKind::SetPositionLead {
                    enable: true,
                    gain_permille: 10,
                    max_counts: 20,
                }
            )
        );
    }

    #[test]
    fn tmc_set_phase_advance_builds_request() {
        assert_eq!(
            tmc_set_phase_advance(0, true, -60_000, 64, 16),
            TmcRequest::new(
                0,
                TmcRequestKind::SetPhaseAdvance {
                    enable: true,
                    gain_ppm: -60_000,
                    max_counts: 64,
                    deadband: 16,
                },
            )
        );
    }

    #[test]
    fn tmc_set_voltage_limit_builds_request() {
        assert_eq!(
            tmc_set_voltage_limit(0, 20_000),
            TmcRequest::new(
                0,
                TmcRequestKind::SetVoltageLimit {
                    voltage_limit: 20_000,
                },
            )
        );
    }

    #[test]
    fn auto_calibrate_on_enable_policy_can_be_toggled() {
        set_auto_calibrate_on_enable_allowed(0, false);
        assert!(!auto_calibrate_on_enable_allowed(0));

        set_auto_calibrate_on_enable_allowed(0, true);
        assert!(auto_calibrate_on_enable_allowed(0));

        set_auto_calibrate_on_enable_allowed(0, false);
        assert!(!auto_calibrate_on_enable_allowed(0));
    }

    #[test]
    fn tmc_commission_builds_request() {
        assert_eq!(
            tmc_commission(0, 7, 1),
            TmcRequest::new(0, TmcRequestKind::Commission { oid: 7, profile: 1 })
        );
    }

    #[test]
    fn tmc_tune_builds_request() {
        assert_eq!(
            tmc_tune(0, 7, 2, 1, 1400, 700, 3, 500, 730, 720, 50, 0x40),
            TmcRequest::new(
                0,
                TmcRequestKind::Tune {
                    oid: 7,
                    profile: 2,
                    mode: 1,
                    inner_lambda_us: 1400,
                    theta_e_us: 700,
                    current_ringing_count: 3,
                    current_bandwidth_hz: 500,
                    tau_e_us: 730,
                    tau_e_crosscheck_us: 720,
                    tau_residual_permille: 50,
                    inner_warning_flags: 0x40,
                }
            )
        );
    }

    #[test]
    fn velocity_scale_factor_typical() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        assert_eq!(scale, 272_160);
    }

    #[test]
    fn velocity_scale_factor_zero_step_width() {
        let scale = compute_velocity_scale_factor(0, 84_000_000, 25_000);
        assert_eq!(scale, 0);
    }

    #[test]
    fn velocity_scale_factor_large_step_width() {
        let scale = compute_velocity_scale_factor(4096, 84_000_000, 25_000);
        assert_eq!(scale, 13_762_560);
    }

    #[test]
    fn velocity_offset_dir_high_matches_tmc_negative_velocity() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 420, true);
        assert_eq!(offset, -648);
    }

    #[test]
    fn velocity_offset_dir_low_matches_tmc_positive_velocity() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 420, false);
        assert_eq!(offset, 648);
    }

    #[test]
    fn velocity_offset_zero_interval() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 0, true);
        assert_eq!(offset, 0);
    }

    #[test]
    fn velocity_offset_zero_scale() {
        let offset = compute_velocity_offset(0, 420, true);
        assert_eq!(offset, 0);
    }

    #[test]
    fn velocity_offset_high_speed() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 84, true);
        assert_eq!(offset, -3240);
    }

    #[test]
    fn velocity_offset_low_speed() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 84_000, true);
        assert_eq!(offset, -3);
    }

    #[test]
    fn velocity_offset_applies_multiplier() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset_scaled(scale, 84_000, false, 8);
        assert_eq!(offset, 24);
    }

    #[test]
    fn velocity_offset_multiplier_zero_disables_offset() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset_scaled(scale, 84_000, false, 0);
        assert_eq!(offset, 0);
    }

    #[test]
    fn velocity_offset_multiplier_saturates_to_i32_range() {
        let offset = compute_velocity_offset_scaled(u32::MAX, 1, false, u16::MAX);
        assert_eq!(offset, i32::MAX);

        let offset = compute_velocity_offset_scaled(u32::MAX, 1, true, u16::MAX);
        assert_eq!(offset, i32::MIN + 1);
    }

    #[test]
    fn command_velocity_reference_uses_step_direction_sign() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);

        let forward = compute_command_velocity_reference(scale, 2800, false);
        let reversed = compute_command_velocity_reference(scale, 2800, true);

        assert_eq!(forward, 19);
        assert_eq!(reversed, -19);
    }

    #[test]
    fn command_velocity_reference_zero_cases_disable_reference() {
        assert_eq!(compute_command_velocity_reference(0, 2800, false), 0);
        assert_eq!(compute_command_velocity_reference(19, 0, false), 0);
    }

    #[test]
    fn command_velocity_reference_is_unmultiplied_velocity_offset() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);
        let reference = compute_command_velocity_reference(scale, 2800, false);
        let offset = compute_velocity_offset_scaled(scale, 2800, false, 40);

        assert_eq!(offset, reference * 40);
    }

    #[test]
    fn position_lead_zero_cases_disable_lead() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);

        assert_eq!(compute_position_lead_counts(0, 2800, true, 10, 20), 0);
        assert_eq!(compute_position_lead_counts(scale, 0, true, 10, 20), 0);
        assert_eq!(compute_position_lead_counts(scale, 2800, true, 0, 20), 0);
        assert_eq!(compute_position_lead_counts(scale, 2800, true, 10, 0), 0);
    }

    #[test]
    fn position_lead_uses_logical_step_direction() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);

        let dir_high = compute_position_lead_counts(scale, 2800, true, 1000, 20);
        let dir_low = compute_position_lead_counts(scale, 2800, false, 1000, 20);

        assert_eq!(dir_high, 19);
        assert_eq!(dir_low, -19);
    }

    #[test]
    fn position_lead_caps_magnitude() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);

        assert_eq!(
            compute_position_lead_counts(scale, 2800, true, 4000, 20),
            20
        );
        assert_eq!(
            compute_position_lead_counts(scale, 2800, false, 4000, 20),
            -20
        );
    }

    #[test]
    fn position_lead_target_does_not_jump_at_rest() {
        let (target, applied) = compute_position_lead_target(12_345, 0, 0, 16);

        assert_eq!(target, 12_345);
        assert_eq!(applied, 0);
    }

    #[test]
    fn position_lead_target_is_relative_to_current_tmc_target() {
        let (target, applied) = compute_position_lead_target(1_000, 0, 2, 16);

        assert_eq!(target, 1_032);
        assert_eq!(applied, 32);
    }

    #[test]
    fn position_lead_target_does_not_accumulate_existing_lead() {
        let previous_applied = 32;

        let (unchanged, still_applied) =
            compute_position_lead_target(1_192, previous_applied, 2, 16);
        let (changed, changed_applied) =
            compute_position_lead_target(1_192, previous_applied, 3, 16);

        assert_eq!(unchanged, 1_192);
        assert_eq!(still_applied, 32);
        assert_eq!(changed, 1_208);
        assert_eq!(changed_applied, 48);
    }

    #[test]
    fn position_lead_target_clears_previous_relative_lead() {
        let (target, applied) = compute_position_lead_target(1_192, 32, 0, 16);

        assert_eq!(target, 1_160);
        assert_eq!(applied, 0);
    }

    #[test]
    fn velocity_transient_zero_cases_disable_offset() {
        let config = VelocityTransientFeedforwardConfig {
            lead_time_us: 1_000,
            gain_permille: 1_000,
            max_offset: 1_000,
        };
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(0, 1_000, 1_000, -10, false),
                config,
            ),
            0
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000, 0, 1_000, -10, false),
                config,
            ),
            0
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000, 1_000, 0, -10, false),
                config,
            ),
            0
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000, 1_000, 1_000, 0, false),
                config,
            ),
            0
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000, 1_000, 1_000, -10, false),
                VelocityTransientFeedforwardConfig {
                    lead_time_us: 0,
                    gain_permille: 1_000,
                    max_offset: 1_000,
                },
            ),
            0
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000, 1_000, 1_000, -10, false),
                VelocityTransientFeedforwardConfig {
                    lead_time_us: 1_000,
                    gain_permille: 0,
                    max_offset: 1_000,
                },
            ),
            0
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000, 1_000, 1_000, -10, false),
                VelocityTransientFeedforwardConfig {
                    lead_time_us: 1_000,
                    gain_permille: 1_000,
                    max_offset: 0,
                },
            ),
            0
        );
    }

    #[test]
    fn velocity_transient_uses_accel_direction_and_dir_sign() {
        let config = VelocityTransientFeedforwardConfig {
            lead_time_us: 1_000,
            gain_permille: 1_000,
            max_offset: 10_000,
        };
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000_000, 1_000_000, 1_000, -1_000, false),
                config,
            ),
            1_000
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000_000, 1_000_000, 1_000, 1_000, false),
                config,
            ),
            -1_000
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000_000, 1_000_000, 1_000, -1_000, true),
                config,
            ),
            -1_000
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000_000, 1_000_000, 1_000, 1_000, true),
                config,
            ),
            1_000
        );
    }

    #[test]
    fn velocity_transient_applies_gain_and_clamp() {
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000_000, 1_000_000, 1_000, -1_000, false),
                VelocityTransientFeedforwardConfig {
                    lead_time_us: 1_000,
                    gain_permille: 500,
                    max_offset: 10_000,
                },
            ),
            500
        );
        assert_eq!(
            compute_velocity_transient_offset(
                velocity_transient_input(1_000_000, 1_000_000, 1_000, -1_000, false),
                VelocityTransientFeedforwardConfig {
                    lead_time_us: 1_000,
                    gain_permille: 2_000,
                    max_offset: 750,
                },
            ),
            750
        );
    }

    fn velocity_transient_input(
        scale_factor: u32,
        clock_freq: u32,
        step_interval: u32,
        interval_add: i16,
        dir_high: bool,
    ) -> VelocityTransientFeedforwardInput {
        VelocityTransientFeedforwardInput {
            scale_factor,
            clock_freq,
            step_interval,
            interval_add,
            dir_high,
        }
    }

    fn decoupling_input(
        target_velocity: i32,
        foc_iq: i16,
        foc_id: i16,
    ) -> DecouplingFeedforwardInput {
        DecouplingFeedforwardInput {
            target_velocity,
            foc_iq,
            foc_id,
        }
    }

    fn decoupling_config(max_offset: u16) -> DecouplingFeedforwardConfig {
        DecouplingFeedforwardConfig {
            r_int: 3000,
            l_int: 4095,
            pole_pairs: 50,
            position_units_per_rev: 65_536,
            f_pwm_hz: 25_000,
            max_offset,
        }
    }

    #[test]
    fn decoupling_feedforward_zero_cases_disable_offsets() {
        assert_eq!(
            compute_decoupling_current_offsets(
                decoupling_input(0, 400, 800),
                decoupling_config(5000)
            ),
            (0, 0)
        );
        assert_eq!(
            compute_decoupling_current_offsets(
                decoupling_input(1080, 400, 800),
                DecouplingFeedforwardConfig {
                    r_int: 0,
                    ..decoupling_config(5000)
                },
            ),
            (0, 0)
        );
        assert_eq!(
            compute_decoupling_current_offsets(
                decoupling_input(1080, 400, 800),
                DecouplingFeedforwardConfig {
                    l_int: 0,
                    ..decoupling_config(5000)
                },
            ),
            (0, 0)
        );
    }

    #[test]
    fn decoupling_feedforward_uses_velocity_and_dq_current_signs() {
        assert_eq!(
            compute_decoupling_current_offsets(
                decoupling_input(10, 400, 800),
                decoupling_config(5000),
            ),
            (1308, -654)
        );
        assert_eq!(
            compute_decoupling_current_offsets(
                decoupling_input(-10, 400, 800),
                decoupling_config(5000),
            ),
            (-1308, 654)
        );
    }

    #[test]
    fn decoupling_feedforward_clamps_both_axes() {
        assert_eq!(
            compute_decoupling_current_offsets(
                decoupling_input(1080, 400, 800),
                decoupling_config(500),
            ),
            (500, -500)
        );
    }

    #[test]
    fn phase_advance_is_zero_when_disabled_or_idle() {
        let config = PhaseAdvanceConfig {
            enable: false,
            gain_ppm: 60_000,
            max_counts: 64,
            deadband_velocity: 16,
        };
        let output = compute_phase_advance_counts(1080, config);
        assert_eq!(output.counts, 0);
        assert!(!output.enabled);

        let config = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: 60_000,
            max_counts: 64,
            deadband_velocity: 16,
        };
        let output = compute_phase_advance_counts(0, config);
        assert_eq!(output.counts, 0);
        assert!(output.enabled);
        assert!(output.deadbanded);
    }

    #[test]
    fn phase_advance_applies_deadband() {
        let config = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: 60_000,
            max_counts: 64,
            deadband_velocity: 16,
        };
        let output = compute_phase_advance_counts(15, config);
        assert_eq!(output.counts, 0);
        assert!(output.deadbanded);
    }

    #[test]
    fn phase_advance_scales_and_clamps_signed_velocity() {
        let config = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: 60_000,
            max_counts: 64,
            deadband_velocity: 16,
        };
        let positive = compute_phase_advance_counts(1080, config);
        assert_eq!(positive.counts, 64);
        assert!(positive.clamped);

        let negative = compute_phase_advance_counts(-1080, config);
        assert_eq!(negative.counts, -64);
        assert!(negative.clamped);
    }

    #[test]
    fn phase_advance_negative_gain_flips_sign() {
        let config = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: -60_000,
            max_counts: 64,
            deadband_velocity: 16,
        };
        let output = compute_phase_advance_counts(1080, config);
        assert_eq!(output.counts, -64);
        assert!(output.clamped);
    }

    #[test]
    fn phase_advance_zero_gain_or_cap_is_safe_noop() {
        let zero_gain = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: 0,
            max_counts: 64,
            deadband_velocity: 16,
        };
        let output = compute_phase_advance_counts(1080, zero_gain);
        assert_eq!(output.counts, 0);
        assert!(output.enabled);
        assert!(!output.clamped);

        let zero_cap = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: 60_000,
            max_counts: 0,
            deadband_velocity: 16,
        };
        let output = compute_phase_advance_counts(1080, zero_cap);
        assert_eq!(output.counts, 0);
        assert!(output.enabled);
        assert!(!output.clamped);
    }

    #[test]
    fn phase_advance_caps_to_firmware_safety_limit() {
        let config = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: 1_000_000,
            max_counts: 10_000,
            deadband_velocity: 0,
        };
        let output = compute_phase_advance_counts(10_000, config);
        assert_eq!(output.counts, PHASE_ADVANCE_MAX_SAFE_COUNTS as i16);
        assert!(output.clamped);
    }

    #[test]
    fn phase_advance_deadband_boundary_is_inclusive() {
        let config = PhaseAdvanceConfig {
            enable: true,
            gain_ppm: 60_000,
            max_counts: 64,
            deadband_velocity: 16,
        };
        let output = compute_phase_advance_counts(16, config);
        assert_eq!(output.counts, 0);
        assert!(output.deadbanded);
    }

    #[test]
    fn pack_abn_phi_offset_preserves_phi_m_and_wraps_phi_e() {
        assert_eq!(pack_abn_phi_offset(0x0000_0000, 64), 0x0040_0000);
        assert_eq!(pack_abn_phi_offset(0xABCD_1234, 0), 0xABCD_1234);
        assert_eq!(pack_abn_phi_offset(0x0000_ABCD, 64), 0x0040_ABCD);
        assert_eq!(pack_abn_phi_offset(0xFFFF_0000, 10), 0x0009_0000);
        assert_eq!(pack_abn_phi_offset(0x0000_5000, -1), 0xFFFF_5000);
    }

    #[test]
    fn acceleration_feedforward_zero_cases_disable_offset() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);
        assert_eq!(
            compute_accel_torque_offset(scale, 84_000_000, 2800, 0, false, 1000, 2816),
            0
        );
        assert_eq!(
            compute_accel_torque_offset(scale, 84_000_000, 0, -9, false, 1000, 2816),
            0
        );
        assert_eq!(
            compute_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 0, 2816),
            0
        );
    }

    #[test]
    fn acceleration_feedforward_uses_add_direction_and_gain() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);

        let forward = compute_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 1000, 2816);
        assert_eq!(forward, 1851);

        let half = compute_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 500, 2816);
        assert_eq!(half, 925);

        let reversed_dir =
            compute_accel_torque_offset(scale, 84_000_000, 2800, -9, true, 1000, 2816);
        assert_eq!(reversed_dir, -1851);

        let decel = compute_accel_torque_offset(scale, 84_000_000, 2800, 9, false, 1000, 2816);
        assert_eq!(decel, -1851);
    }

    #[test]
    fn acceleration_feedforward_split_uses_accel_gain_for_shrinking_interval() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);
        assert_eq!(
            compute_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                -9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 1000,
                    decel_permille: 0,
                },
                2816
            ),
            compute_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 1000, 2816)
        );
        assert_eq!(
            compute_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                -9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 0,
                    decel_permille: 500,
                },
                2816
            ),
            0
        );
    }

    #[test]
    fn acceleration_feedforward_split_uses_decel_gain_for_growing_interval() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);
        assert_eq!(
            compute_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 0,
                    decel_permille: 500,
                },
                2816
            ),
            compute_accel_torque_offset(scale, 84_000_000, 2800, 9, false, 500, 2816)
        );
        assert_eq!(
            compute_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 1000,
                    decel_permille: 0,
                },
                2816
            ),
            0
        );
    }

    #[test]
    fn acceleration_feedforward_clamps_to_current_limit() {
        let scale = compute_velocity_scale_factor(16, 84_000_000, 25_000);
        let offset = compute_accel_torque_offset(scale, 84_000_000, 2800, -90, false, 1000, 1200);
        assert_eq!(offset, 1200);
    }
}
