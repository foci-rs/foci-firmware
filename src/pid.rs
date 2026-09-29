//! PID gain register helpers for TMC4671 control loops.

use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};

/// Computed PI gains for TMC4671 flux and torque controllers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InnerLoopGains {
    pub flux_p: PidPGain,
    pub flux_i: PidIGain,
    pub torque_p: PidPGain,
    pub torque_i: PidIGain,
}

/// Computed PI gains for TMC4671 position and velocity outer loops.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OuterLoopGains {
    pub position_p: PidPGain,
    pub position_i: PidIGain,
    pub velocity_p: PidPGain,
    pub velocity_i: PidIGain,
}

/// Conservative default gains for when no R/L data is available: stable but sluggish.
pub const DEFAULT_INNER_GAINS: InnerLoopGains = InnerLoopGains {
    flux_p: match PidPGain::new(256) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    flux_i: match PidIGain::new(416) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    torque_p: match PidPGain::new(256) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    torque_i: match PidIGain::new(416) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
};

#[cfg(test)]
pub const DEFAULT_OUTER_GAINS: OuterLoopGains = OuterLoopGains {
    position_p: match PidPGain::new(640) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    position_i: PidIGain::ZERO,
    velocity_p: match PidPGain::new(1152) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    velocity_i: PidIGain::ZERO,
};

/// Inert pre-commissioning gains written on config reset; arming is refused until commissioning
/// produces real gains.
pub const CONFIG_RESET_PLACEHOLDER_GAINS: OuterLoopGains = OuterLoopGains {
    position_p: PidPGain::ZERO,
    position_i: PidIGain::ZERO,
    velocity_p: PidPGain::ZERO,
    velocity_i: PidIGain::ZERO,
};

#[cfg(test)]
pub(crate) const fn p_gain(raw: u16) -> PidPGain {
    match PidPGain::new(raw) {
        Ok(gain) => gain,
        Err(_) => panic!("test P gain must be in range"),
    }
}

#[cfg(test)]
pub(crate) const fn i_gain(raw: u16) -> PidIGain {
    match PidIGain::new(raw) {
        Ok(gain) => gain,
        Err(_) => panic!("test I gain must be in range"),
    }
}

#[cfg(test)]
pub(crate) const fn test_outer_gains(
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) -> OuterLoopGains {
    OuterLoopGains {
        position_p: p_gain(position_p),
        position_i: i_gain(position_i),
        velocity_p: p_gain(velocity_p),
        velocity_i: i_gain(velocity_i),
    }
}

#[cfg(test)]
pub(crate) const fn test_inner_gains(
    flux_p: u16,
    flux_i: u16,
    torque_p: u16,
    torque_i: u16,
) -> InnerLoopGains {
    InnerLoopGains {
        flux_p: p_gain(flux_p),
        flux_i: i_gain(flux_i),
        torque_p: p_gain(torque_p),
        torque_i: i_gain(torque_i),
    }
}

impl InnerLoopGains {
    /// Packs into `(PID_FLUX_P_FLUX_I, PID_TORQUE_P_TORQUE_I)` register values.
    pub fn to_registers(self) -> (u32, u32) {
        let flux = PidGainPair::new(self.flux_p, self.flux_i).register_value();
        let torque = PidGainPair::new(self.torque_p, self.torque_i).register_value();
        (flux, torque)
    }
}

impl OuterLoopGains {
    /// Packs into `(PID_VELOCITY_P_VELOCITY_I, PID_POSITION_P_POSITION_I)` register values.
    pub fn to_registers(self) -> (u32, u32) {
        let velocity = PidGainPair::new(self.velocity_p, self.velocity_i).register_value();
        let position = PidGainPair::new(self.position_p, self.position_i).register_value();
        (velocity, position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_tmc4671::pid::{PidIGain, PidPGain};

    #[test]
    fn q4_12_migration_preserves_p_and_scales_i_exactly() {
        let gains = InnerLoopGains {
            flux_p: PidPGain::new(707).unwrap(),
            flux_i: PidIGain::new(2_544).unwrap(),
            torque_p: PidPGain::new(707).unwrap(),
            torque_i: PidIGain::new(2_544).unwrap(),
        };

        let (flux_reg, torque_reg) = gains.to_registers();

        assert_eq!(flux_reg, 0x02c3_09f0);
        assert_eq!(torque_reg, 0x02c3_09f0);
        assert_eq!(DEFAULT_INNER_GAINS.flux_i.raw(), 416);
        assert_eq!(DEFAULT_INNER_GAINS.torque_i.raw(), 416);
    }

    #[test]
    fn default_gains_are_valid() {
        const {
            assert!(DEFAULT_INNER_GAINS.flux_p.raw() > 0);
            assert!(DEFAULT_INNER_GAINS.flux_i.raw() > 0);
            assert!(DEFAULT_INNER_GAINS.torque_p.raw() > 0);
            assert!(DEFAULT_INNER_GAINS.torque_i.raw() > 0);
            assert!(DEFAULT_OUTER_GAINS.position_p.raw() > 0);
            assert!(DEFAULT_OUTER_GAINS.velocity_p.raw() > 0);
        }
    }

    #[test]
    fn inner_loop_gains_to_registers_places_p_in_upper_half() {
        let gains = InnerLoopGains {
            flux_p: PidPGain::new(707).unwrap(),
            flux_i: PidIGain::new(2_592).unwrap(),
            torque_p: PidPGain::new(707).unwrap(),
            torque_i: PidIGain::new(2_592).unwrap(),
        };

        let (flux_reg, torque_reg) = gains.to_registers();

        assert_eq!(flux_reg, 0x02c3_0a20);
        assert_eq!(torque_reg, 0x02c3_0a20);
    }

    #[test]
    fn config_reset_placeholder_gains_are_obviously_inert() {
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.velocity_p, PidPGain::ZERO);
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.position_p, PidPGain::ZERO);
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.velocity_i, PidIGain::ZERO);
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.position_i, PidIGain::ZERO);
    }

    #[test]
    fn default_outer_gains_value_is_unchanged() {
        assert_eq!(DEFAULT_OUTER_GAINS.velocity_p.raw(), 1152);
        assert_eq!(DEFAULT_OUTER_GAINS.position_p.raw(), 640);
    }

    #[test]
    fn outer_loop_gains_to_registers_places_p_in_upper_half() {
        let gains = OuterLoopGains {
            position_p: p_gain(256),
            position_i: i_gain(48),
            velocity_p: p_gain(6656),
            velocity_i: i_gain(64),
        };

        let (velocity_reg, position_reg) = gains.to_registers();

        assert_eq!(velocity_reg, 0x1a00_0040);
        assert_eq!(position_reg, 0x0100_0030);
    }
}
