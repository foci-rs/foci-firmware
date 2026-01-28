//! TMC4671 register dump list.
//!
//! The firmware reads these registers via SPI and sends each value
//! back as a `foci_dump_value` reply. The Python host collects them
//! and pretty-prints using field definitions from `tmc4671.py`.

/// Registers to dump, in address order. Must include every register
/// referenced by `DUMP_GROUPS` in `klipper-foci/tmc4671.py`.
///
/// Register addresses verified against TMC4671 datasheet rev 2.08 and
/// `tmc4671/src/registers.rs`.
pub const DUMP_REGISTERS: &[u8] = &[
    // ADC calibration
    0x08, // ADC_I1_SCALE_OFFSET
    0x09, // ADC_I0_SCALE_OFFSET
    // PWM
    0x1A, // PWM_SV_CHOP
    // Motor config
    0x1B, // MOTOR_TYPE_N_POLE_PAIRS
    // Encoder
    0x25, // ABN_DECODER_MODE
    0x26, // ABN_DECODER_PPR
    0x27, // ABN_DECODER_COUNT
    0x2A, // ABN_DECODER_PHI_E_PHI_M
    // PHI_E selection
    0x52, // PHI_E_SELECTION
    // PID gains
    0x54, // PID_FLUX_P_FLUX_I
    0x56, // PID_TORQUE_P_TORQUE_I
    0x58, // PID_VELOCITY_P_VELOCITY_I
    0x5A, // PID_POSITION_P_POSITION_I
    // PID limits
    0x5D, // PIDOUT_UQ_UD_LIMITS
    0x5E, // PID_TORQUE_FLUX_LIMITS
    // Motion mode
    0x63, // MODE_RAMP_MODE_MOTION
    // PID targets and actuals
    0x64, // PID_TORQUE_FLUX_TARGET
    0x68, // PID_POSITION_TARGET
    0x69, // PID_TORQUE_FLUX_ACTUAL
    0x6B, // PID_POSITION_ACTUAL
    // Status
    0x7C, // STATUS_FLAGS
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dump_registers_not_empty() {
        assert!(!DUMP_REGISTERS.is_empty());
    }

    #[test]
    fn dump_registers_all_valid_addresses() {
        for &addr in DUMP_REGISTERS {
            assert!(addr <= 0x7F, "addr {:#04X} exceeds 7-bit range", addr);
        }
    }
}
