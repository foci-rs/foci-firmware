//! TMC4671 command types and shared logic.
//!
//! These functions produce [`TmcRequest`] values that firmware crates
//! send to their board-specific SPI task. The actual SPI communication
//! and motor control is handled by the firmware, not this crate.

/// Request type for TMC4671 task communication.
///
/// Firmware crates send these to their board-specific SPI task via a
/// channel (e.g., `heapless::spsc` or `rtic_sync::channel`).
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TmcRequest {
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
    /// Trigger R/L measurement and gain computation (FOCI_AUTOTUNE).
    Autotune { oid: u8 },
    /// Start inner commissioning (diagnostics + electrical ID + current tune).
    /// Profile: 0=Conservative, 1=Balanced, 2=Stiff.
    CommissionInner { oid: u8, profile: u8 },
    /// Start outer commissioning (mechanical ID + velocity/position + filters).
    /// Profile: 0=Conservative, 1=Balanced, 2=Stiff.
    /// Mode: 0=Unloaded, 1=Nominal, 2=HighInertia.
    CommissionOuter { oid: u8, profile: u8, mode: u8 },
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
pub fn tmc_set_current(run_ma: u32) -> TmcRequest {
    TmcRequest::SetCurrent { run_ma }
}

/// Compute step width from encoder PPR and pole pairs for hardware step/dir.
///
/// Returns `(TmcRequest, step_width)` where `step_width` is the value to
/// write to TMC4671 STEP_WIDTH register (0x78). The board crate writes
/// the step width directly to the TMC4671 register.
///
/// Step width formula: `65536 * pole_pairs / (ppr * 4)`
pub fn tmc_set_encoder_hw(ppr: u32, pole_pairs: u32) -> (TmcRequest, i32) {
    let step_width = if ppr > 0 {
        let quad_ppr = (ppr as u64) * 4;
        (65536u64 * pole_pairs as u64 / quad_ppr) as i32
    } else {
        0
    };
    (TmcRequest::SetEncoder { ppr }, step_width)
}

/// Build a `SetEncoderDir` request.
pub fn tmc_set_encoder_dir(invert: bool) -> TmcRequest {
    TmcRequest::SetEncoderDir { invert }
}

/// Build a `SelfTest` request.
pub fn tmc_selftest(oid: u8) -> TmcRequest {
    TmcRequest::SelfTest { oid }
}

/// Build a `Calibrate` request.
pub fn tmc_calibrate(oid: u8) -> TmcRequest {
    TmcRequest::Calibrate { oid }
}

/// Build a `DumpRegisters` request.
pub fn tmc_dump_registers(oid: u8) -> TmcRequest {
    TmcRequest::DumpRegisters { oid }
}

/// Build a `SetPidGains` request.
pub fn tmc_set_pid_gains(flux_p: u16, flux_i: u16, torque_p: u16, torque_i: u16) -> TmcRequest {
    TmcRequest::SetPidGains {
        flux_p,
        flux_i,
        torque_p,
        torque_i,
    }
}

/// Build an `Autotune` request.
pub fn tmc_autotune(oid: u8) -> TmcRequest {
    TmcRequest::Autotune { oid }
}

/// Build a `CommissionInner` request.
pub fn tmc_commission_inner(oid: u8, profile: u8) -> TmcRequest {
    TmcRequest::CommissionInner { oid, profile }
}

/// Build a `CommissionOuter` request.
pub fn tmc_commission_outer(oid: u8, profile: u8, mode: u8) -> TmcRequest {
    TmcRequest::CommissionOuter { oid, profile, mode }
}

/// Build a `SetVelocityFilter` request.
pub fn tmc_set_velocity_filter(filter_hz: u16) -> TmcRequest {
    TmcRequest::SetVelocityFilter { filter_hz }
}

/// Build a `SetTorqueFilter` request.
pub fn tmc_set_torque_filter(filter_hz: u16) -> TmcRequest {
    TmcRequest::SetTorqueFilter { filter_hz }
}

/// Build a `SetPositionFilter` request.
pub fn tmc_set_position_filter(filter_hz: u16) -> TmcRequest {
    TmcRequest::SetPositionFilter { filter_hz }
}

/// Build a `SetFluxFilter` request.
pub fn tmc_set_flux_filter(filter_hz: u16) -> TmcRequest {
    TmcRequest::SetFluxFilter { filter_hz }
}

/// Build a `SetPositionGains` request.
pub fn tmc_set_position_gains(
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) -> TmcRequest {
    TmcRequest::SetPositionGains {
        position_p,
        position_i,
        velocity_p,
        velocity_i,
    }
}

/// Build a `SetVelocityFeedforward` request.
pub fn tmc_set_velocity_feedforward(enable: bool) -> TmcRequest {
    TmcRequest::SetVelocityFeedforward { enable }
}

/// Build a `Write` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_write_register(addr: u8, value: u32) -> TmcRequest {
    TmcRequest::Write { addr, value }
}

/// Build a `Read` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_read_register(oid: u8, addr: u8) -> TmcRequest {
    TmcRequest::Read { oid, addr }
}

/// Build an `Init` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_init() -> TmcRequest {
    TmcRequest::Init
}

/// Build a `TestSpin` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_test_spin(revolutions: u8) -> TmcRequest {
    TmcRequest::TestSpin { revolutions }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tmc_set_current_builds_request() {
        assert_eq!(tmc_set_current(500), TmcRequest::SetCurrent { run_ma: 500 });
    }

    #[test]
    fn tmc_set_encoder_hw_computes_step_width() {
        let (req, step_width) = tmc_set_encoder_hw(1000, 50);
        assert_eq!(req, TmcRequest::SetEncoder { ppr: 1000 });
        assert_eq!(step_width, 819);
    }

    #[test]
    fn tmc_set_encoder_hw_zero_ppr() {
        let (req, step_width) = tmc_set_encoder_hw(0, 50);
        assert_eq!(req, TmcRequest::SetEncoder { ppr: 0 });
        assert_eq!(step_width, 0);
    }

    #[test]
    fn tmc_set_encoder_dir_builds_request() {
        assert_eq!(
            tmc_set_encoder_dir(true),
            TmcRequest::SetEncoderDir { invert: true }
        );
    }

    #[test]
    fn tmc_selftest_builds_request() {
        assert_eq!(tmc_selftest(5), TmcRequest::SelfTest { oid: 5 });
    }

    #[test]
    fn tmc_calibrate_builds_request() {
        assert_eq!(tmc_calibrate(0), TmcRequest::Calibrate { oid: 0 });
    }

    #[test]
    fn tmc_dump_registers_builds_request() {
        assert_eq!(tmc_dump_registers(1), TmcRequest::DumpRegisters { oid: 1 });
    }

    #[test]
    fn tmc_set_pid_gains_builds_request() {
        assert_eq!(
            tmc_set_pid_gains(100, 200, 300, 400),
            TmcRequest::SetPidGains {
                flux_p: 100,
                flux_i: 200,
                torque_p: 300,
                torque_i: 400,
            }
        );
    }

    #[test]
    fn tmc_autotune_builds_request() {
        assert_eq!(tmc_autotune(0), TmcRequest::Autotune { oid: 0 });
    }

    #[test]
    fn tmc_set_velocity_filter_builds_request() {
        assert_eq!(
            tmc_set_velocity_filter(200),
            TmcRequest::SetVelocityFilter { filter_hz: 200 }
        );
    }

    #[test]
    fn tmc_set_torque_filter_builds_request() {
        assert_eq!(
            tmc_set_torque_filter(500),
            TmcRequest::SetTorqueFilter { filter_hz: 500 }
        );
    }

    #[test]
    fn tmc_set_position_filter_builds_request() {
        assert_eq!(
            tmc_set_position_filter(100),
            TmcRequest::SetPositionFilter { filter_hz: 100 }
        );
    }

    #[test]
    fn tmc_set_flux_filter_builds_request() {
        assert_eq!(
            tmc_set_flux_filter(300),
            TmcRequest::SetFluxFilter { filter_hz: 300 }
        );
    }

    #[test]
    fn tmc_set_position_gains_builds_request() {
        assert_eq!(
            tmc_set_position_gains(640, 5, 1152, 0),
            TmcRequest::SetPositionGains {
                position_p: 640,
                position_i: 5,
                velocity_p: 1152,
                velocity_i: 0,
            }
        );
    }

    #[test]
    fn tmc_set_velocity_feedforward_builds_request() {
        assert_eq!(
            tmc_set_velocity_feedforward(true),
            TmcRequest::SetVelocityFeedforward { enable: true }
        );
    }

    #[test]
    fn tmc_commission_inner_builds_request() {
        assert_eq!(
            tmc_commission_inner(0, 1),
            TmcRequest::CommissionInner { oid: 0, profile: 1 }
        );
    }

    #[test]
    fn tmc_commission_outer_builds_request() {
        assert_eq!(
            tmc_commission_outer(0, 2, 1),
            TmcRequest::CommissionOuter {
                oid: 0,
                profile: 2,
                mode: 1
            }
        );
    }
}
