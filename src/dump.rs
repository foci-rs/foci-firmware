//! TMC4671 register dump list.

use foci_tmc4671::registers::SubRegister;

/// Registers dumped in address order; must cover every register in the host's `DUMP_GROUPS`.
pub const DUMP_REGISTERS: &[u8] = &[
    0x08, 0x09, 0x0A, 0x19, 0x1A, 0x1B, 0x25, 0x26, 0x27, 0x29, 0x2A, 0x50, 0x51, 0x52, 0x54, 0x56,
    0x58, 0x5A, 0x5D, 0x5E, 0x60, 0x63, 0x64, 0x65, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x75, 0x7C,
];

/// `(synthetic_addr, sub_register)` pairs dumped after the regular registers. Synthetic
/// addresses are 0x80+, disjoint from real register addresses (0x00-0x7F).
pub const DUMP_SUB_REGISTERS: &[(u8, SubRegister)] = &[
    (0x80, foci_tmc4671::registers::INTERIM_PIDIN_TARGET_VELOCITY),
    (
        0x81,
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_VELOCITY,
    ),
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
    (0x86, foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT),
    (0x87, foci_tmc4671::registers::ADC_AGPI_A_RAW_ADC_VM_RAW),
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
