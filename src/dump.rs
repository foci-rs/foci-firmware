//! TMC4671 register dump list.
//!
//! The firmware reads these registers via SPI and sends each value
//! back as a `foci_dump_value` reply. The Python host collects them
//! and pretty-prints using field definitions from `foci.py`.

use foci_tmc4671::registers::SubRegister;

/// Registers to dump, in address order. Must include every register
/// referenced by `DUMP_GROUPS` in `klipper-foci/foci.py`.
///
/// Register addresses verified against TMC4671 datasheet rev 2.08 and
/// `foci-tmc4671/src/registers.rs`.
pub const DUMP_REGISTERS: &[u8] = &[
    // ADC calibration
    0x08, // ADC_I1_SCALE_OFFSET
    0x09, // ADC_I0_SCALE_OFFSET
    0x0A, // ADC_I_SELECT
    // PWM
    0x19, // PWM_BBM_H_BBM_L
    0x1A, // PWM_SV_CHOP
    // Motor config
    0x1B, // MOTOR_TYPE_N_POLE_PAIRS
    // Encoder
    0x25, // ABN_DECODER_MODE
    0x26, // ABN_DECODER_PPR
    0x27, // ABN_DECODER_COUNT
    0x29, // ABN_DECODER_PHI_E_PHI_M_OFFSET
    0x2A, // ABN_DECODER_PHI_E_PHI_M
    // Selection registers
    0x50, // VELOCITY_SELECTION
    0x51, // POSITION_SELECTION
    0x52, // PHI_E_SELECTION
    // PID gains
    0x54, // PID_FLUX_P_FLUX_I
    0x56, // PID_TORQUE_P_TORQUE_I
    0x58, // PID_VELOCITY_P_VELOCITY_I
    0x5A, // PID_POSITION_P_POSITION_I
    // PID limits
    0x5D, // PIDOUT_UQ_UD_LIMITS
    0x5E, // PID_TORQUE_FLUX_LIMITS
    // PID velocity limit
    0x60, // PID_VELOCITY_LIMIT
    // Motion mode
    0x63, // MODE_RAMP_MODE_MOTION
    // PID targets and actuals
    0x64, // PID_TORQUE_FLUX_TARGET
    0x65, // PID_TORQUE_FLUX_OFFSET
    0x67, // PID_VELOCITY_OFFSET
    0x68, // PID_POSITION_TARGET
    0x69, // PID_TORQUE_FLUX_ACTUAL
    0x6A, // PID_VELOCITY_ACTUAL
    0x6B, // PID_POSITION_ACTUAL
    // ADC limits
    0x75, // ADC_VM_LIMITS
    // Status
    0x7C, // STATUS_FLAGS
];

/// Sub-registers to dump after regular registers.
///
/// Each entry is a (synthetic_addr, sub_register) pair. The synthetic
/// address (0x80+) is sent as `addr` in `foci_dump_value` replies so
/// the Python host can map them to display names. Regular register
/// addresses are 0x00-0x7F, so 0x80+ is an unambiguous sub-register
/// indicator.
pub const DUMP_SUB_REGISTERS: &[(u8, SubRegister)] = &[
    // INTERIM: PID cascade internal targets (datasheet page 122)
    (0x80, foci_tmc4671::registers::INTERIM_PIDIN_TARGET_VELOCITY),
    (
        0x81,
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_VELOCITY,
    ),
    // PID_ERROR: integrator state (datasheet page 122)
    (
        0x82,
        foci_tmc4671::registers::PID_ERROR_PID_POSITION_ERROR_SUM,
    ),
    (
        0x83,
        foci_tmc4671::registers::PID_ERROR_PID_TORQUE_ERROR_SUM,
    ),
    (0x84, foci_tmc4671::registers::PID_ERROR_PID_FLUX_ERROR_SUM),
    (
        0x85,
        foci_tmc4671::registers::PID_ERROR_PID_VELOCITY_ERROR_SUM,
    ),
    // CONFIG: advanced PI gain representation bits (datasheet section 7.2)
    (0x86, foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT),
    // ADC_RAW selector 1: low half is ADC_VM_RAW, high half is ADC_AGPI_A_RAW.
    (0x87, foci_tmc4671::registers::ADC_AGPI_A_RAW_ADC_VM_RAW),
    // CONFIG: position (X) biquad filter enable.
    (0x8A, foci_tmc4671::registers::CONFIG_BIQUAD_X_ENABLE),
];

/// Synthetic dump address for the latched velocity-clamp flag.
pub const DUMP_CLAMP_LATCHED_ADDR: u8 = 0x88;

/// Synthetic dump address for the velocity-clamp activation count.
pub const DUMP_CLAMP_COUNT_ADDR: u8 = 0x89;

/// Maps velocity-clamp runtime state to synthetic dump `(addr, value)` pairs.
pub fn clamp_dump_values(latched: bool, count: u32) -> [(u8, u32); 2] {
    [
        (DUMP_CLAMP_LATCHED_ADDR, u32::from(latched)),
        (DUMP_CLAMP_COUNT_ADDR, count),
    ]
}

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

    #[test]
    fn dump_registers_include_selection_registers() {
        assert!(DUMP_REGISTERS.contains(&0x0A), "missing ADC_I_SELECT");
        assert!(DUMP_REGISTERS.contains(&0x50), "missing VELOCITY_SELECTION");
        assert!(DUMP_REGISTERS.contains(&0x51), "missing POSITION_SELECTION");
        assert!(DUMP_REGISTERS.contains(&0x52), "missing PHI_E_SELECTION");
    }

    #[test]
    fn dump_registers_include_abn_phase_offset() {
        assert!(
            DUMP_REGISTERS.contains(&0x29),
            "missing ABN_DECODER_PHI_E_PHI_M_OFFSET"
        );
    }

    #[test]
    fn dump_registers_include_pwm_bbm() {
        assert!(DUMP_REGISTERS.contains(&0x19), "missing PWM_BBM_H_BBM_L");
    }

    #[test]
    fn dump_sub_registers_synthetic_addrs_above_0x80() {
        for &(addr, _) in DUMP_SUB_REGISTERS {
            assert!(
                addr >= 0x80,
                "sub-register synthetic addr {:#04X} must be >= 0x80",
                addr
            );
        }
    }

    #[test]
    fn synthetic_addresses_do_not_collide_across_lists() {
        let mut seen = heapless::index_set::FnvIndexSet::<u8, 32>::new();
        for &(addr, _) in DUMP_SUB_REGISTERS {
            assert!(seen.insert(addr).unwrap(), "dup {:#04X}", addr);
        }
        for addr in [DUMP_CLAMP_LATCHED_ADDR, DUMP_CLAMP_COUNT_ADDR] {
            assert!(
                seen.insert(addr).unwrap(),
                "clamp addr {:#04X} collides",
                addr
            );
        }
    }

    #[test]
    fn clamp_dump_values_map_addresses() {
        assert_eq!(clamp_dump_values(true, 5), [(0x88u8, 1u32), (0x89u8, 5u32)]);
        assert_eq!(
            clamp_dump_values(false, 0),
            [(0x88u8, 0u32), (0x89u8, 0u32)]
        );
    }

    #[test]
    fn dump_sub_registers_include_advanced_pi_representation() {
        let found = DUMP_SUB_REGISTERS.iter().any(|&(addr, sub)| {
            addr == 0x86 && sub == foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
        });

        assert!(found, "missing CONFIG_ADVANCED_PI_REPRESENT dump field");
    }

    #[test]
    fn dump_sub_registers_include_adc_vm_raw() {
        let found = DUMP_SUB_REGISTERS.iter().any(|&(addr, sub)| {
            addr == 0x87 && sub == foci_tmc4671::registers::ADC_AGPI_A_RAW_ADC_VM_RAW
        });

        assert!(found, "missing ADC_VM_RAW dump field");
    }

    #[test]
    fn dump_sub_registers_include_position_filter_enable() {
        let found = DUMP_SUB_REGISTERS.iter().any(|&(addr, sub)| {
            addr == 0x8A && sub == foci_tmc4671::registers::CONFIG_BIQUAD_X_ENABLE
        });
        assert!(found, "missing CONFIG_BIQUAD_X_ENABLE dump field");
    }
}
