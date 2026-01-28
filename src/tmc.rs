//! TMC4671 command types and shared logic.
//!
//! These functions produce [`TmcRequest`] values that firmware crates
//! send to their board-specific SPI task. The actual SPI communication
//! and motor control is handled by the firmware, not this crate.

use klipper_mcu::state::{State, NUM_STEPPERS};

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
    SelfTest,
    /// Dump all registers via klipper_output protocol.
    DumpRegisters { oid: u8 },
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

/// Compute step width from encoder PPR and pole pairs, update stepper
/// state, and build a `SetEncoder` request.
///
/// `pole_pairs` is motor-type-specific: 50 for 2-phase stepper
/// (MOTOR_TYPE=2), different for BLDC. The firmware crate provides
/// this value.
///
/// Step width formula: `65536 * pole_pairs / (ppr * 4)`
pub fn tmc_set_encoder(state: &mut State, channel: u8, ppr: u32, pole_pairs: u32) -> TmcRequest {
    if ppr > 0 {
        let quad_ppr = ppr * 4;
        let step_width = (65536u64 * pole_pairs as u64 / quad_ppr as u64) as i32;
        if (channel as usize) < NUM_STEPPERS {
            state.steppers[channel as usize].set_step_width(step_width);
        }
    }
    TmcRequest::SetEncoder { ppr }
}

/// Build a `SetEncoderDir` request.
pub fn tmc_set_encoder_dir(invert: bool) -> TmcRequest {
    TmcRequest::SetEncoderDir { invert }
}

/// Build a `SelfTest` request.
pub fn tmc_selftest() -> TmcRequest {
    TmcRequest::SelfTest
}

/// Build a `Calibrate` request.
pub fn tmc_calibrate(oid: u8) -> TmcRequest {
    TmcRequest::Calibrate { oid }
}

/// Build a `DumpRegisters` request.
pub fn tmc_dump_registers(oid: u8) -> TmcRequest {
    TmcRequest::DumpRegisters { oid }
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
    fn tmc_set_encoder_computes_step_width() {
        let mut state = State::new();
        state.steppers_by_oid.insert(0, 0).unwrap();

        // 1000 PPR encoder, 50 pole pairs:
        // quad_ppr = 4000, step_width = 65536 * 50 / 4000 = 819
        let req = tmc_set_encoder(&mut state, 0, 1000, 50);
        assert_eq!(req, TmcRequest::SetEncoder { ppr: 1000 });
        assert_eq!(state.steppers[0].step_width(), 819);
    }

    #[test]
    fn tmc_set_encoder_channel_out_of_range() {
        let mut state = State::new();
        // Channel 5 is out of range (NUM_STEPPERS=2), should not panic
        let req = tmc_set_encoder(&mut state, 5, 1000, 50);
        assert_eq!(req, TmcRequest::SetEncoder { ppr: 1000 });
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
        assert_eq!(tmc_selftest(), TmcRequest::SelfTest);
    }

    #[test]
    fn tmc_calibrate_builds_request() {
        assert_eq!(tmc_calibrate(0), TmcRequest::Calibrate { oid: 0 });
    }

    #[test]
    fn tmc_dump_registers_builds_request() {
        assert_eq!(tmc_dump_registers(1), TmcRequest::DumpRegisters { oid: 1 });
    }
}
