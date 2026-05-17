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
    SetVelocityFeedforward { enable: bool },
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
pub fn tmc_set_velocity_feedforward(channel: u8, enable: bool) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetVelocityFeedforward { enable })
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
/// - `forward`: true if stepping in the forward direction
///
/// Returns 0 if `step_interval == 0` (motor idle) or `scale_factor == 0`
/// (feedforward not configured).
///
/// The result is in TMC4671 velocity units (position change per PWM period)
/// and is written directly to PID_VELOCITY_OFFSET (0x67, s32).
pub fn compute_velocity_offset(scale_factor: u32, step_interval: u32, forward: bool) -> i32 {
    if step_interval == 0 || scale_factor == 0 {
        return 0;
    }
    let magnitude = scale_factor / step_interval;
    if forward {
        magnitude as i32
    } else {
        -(magnitude as i32)
    }
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
            tmc_set_velocity_feedforward(0, true),
            TmcRequest::new(0, TmcRequestKind::SetVelocityFeedforward { enable: true })
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
    fn velocity_offset_worked_example() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 420, true);
        assert_eq!(offset, 648);
    }

    #[test]
    fn velocity_offset_reverse_direction() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 420, false);
        assert_eq!(offset, -648);
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
        assert_eq!(offset, 3240);
    }

    #[test]
    fn velocity_offset_low_speed() {
        let scale = compute_velocity_scale_factor(81, 84_000_000, 25_000);
        let offset = compute_velocity_offset(scale, 84_000, true);
        assert_eq!(offset, 3);
    }
}
