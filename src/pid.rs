//! PID gain register helpers for TMC4671 control loops.

use foci_tmc4671::pid::{PidGainOutOfRange, PidGainPair, PidIGain, PidPGain};

/// Computed PI gains for TMC4671 flux and torque controllers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InnerLoopGains {
    /// Flux P gain — raw TMC4671 register value (Q8.8 format).
    pub flux_p: PidPGain,
    /// Flux I gain — raw TMC4671 register value (Q4.12 format).
    pub flux_i: PidIGain,
    /// Torque P gain — raw TMC4671 register value (Q8.8 format).
    pub torque_p: PidPGain,
    /// Torque I gain — raw TMC4671 register value (Q4.12 format).
    pub torque_i: PidIGain,
}

/// Computed PI gains for TMC4671 position and velocity outer loops.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OuterLoopGains {
    /// Position P gain — raw TMC4671 register value (Q8.8 format).
    pub position_p: PidPGain,
    /// Position I gain — raw TMC4671 register value (Q4.12 format).
    pub position_i: PidIGain,
    /// Velocity P gain — raw TMC4671 register value (Q8.8 format).
    pub velocity_p: PidPGain,
    /// Velocity I gain — raw TMC4671 register value (Q4.12 format).
    pub velocity_i: PidIGain,
}

/// Conservative default gains for when no R/L data is available.
/// These are intentionally low — stable but sluggish.
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

/// Fresh-commission fallback outer-loop gains for boards whose configured
/// current-loop bandwidth (`TmcConfig::bandwidth_hz`) falls outside the
/// gain-floor measurement's supported `[95.5 Hz, 796 Hz]` range (Task 7).
/// Not tuned to be "conservative" for any specific motor — historical value,
/// left unchanged so boards outside that range keep their current behavior.
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

/// Pre-commissioning placeholder written by `TmcChannelState::reset()` after
/// a config reset. Never applied to hardware while armed:
/// `arm_motor_electrical`'s calibration assert refuses to arm before a
/// commissioning result exists, and by the time any fresh commission reaches
/// `ClosedLoopActivation`, the gains actually written come from either the
/// gain-floor measurement (Task 7) or `DEFAULT_OUTER_GAINS` (below) — never
/// this value.
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
    /// Pack gains into TMC4671 register values.
    ///
    /// Returns (flux_register, torque_register) ready for
    /// `PID_FLUX_P_FLUX_I` (0x54) and `PID_TORQUE_P_TORQUE_I` (0x56).
    pub fn to_registers(self) -> (u32, u32) {
        let flux = PidGainPair::new(self.flux_p, self.flux_i).register_value();
        let torque = PidGainPair::new(self.torque_p, self.torque_i).register_value();
        (flux, torque)
    }

    /// Unpack gains from TMC4671 register values.
    pub fn from_registers(flux_reg: u32, torque_reg: u32) -> Result<Self, PidGainOutOfRange> {
        let flux = PidGainPair::try_from_register(flux_reg)?;
        let torque = PidGainPair::try_from_register(torque_reg)?;
        Ok(Self {
            flux_p: PidPGain::new(flux.p.raw())?,
            flux_i: PidIGain::new(flux.i.raw())?,
            torque_p: PidPGain::new(torque.p.raw())?,
            torque_i: PidIGain::new(torque.i.raw())?,
        })
    }
}

impl OuterLoopGains {
    /// Pack gains into TMC4671 register values.
    ///
    /// Returns `(velocity_register, position_register)` ready for
    /// `PID_VELOCITY_P_VELOCITY_I` and `PID_POSITION_P_POSITION_I`.
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
    fn maximum_inner_gains_round_trip() {
        let gains = InnerLoopGains {
            flux_p: PidPGain::MAX,
            flux_i: PidIGain::MAX,
            torque_p: PidPGain::MAX,
            torque_i: PidIGain::MAX,
        };
        let (flux_reg, torque_reg) = gains.to_registers();
        let restored = InnerLoopGains::from_registers(flux_reg, torque_reg).unwrap();
        assert_eq!(gains, restored);
    }

    #[test]
    fn inner_loop_gains_reject_invalid_register_half() {
        assert!(InnerLoopGains::from_registers(0x8000_0001, 0).is_err());
        assert!(InnerLoopGains::from_registers(0, 0x0001_8000).is_err());
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
        // "Obviously inert": far below any value that could produce meaningful
        // motion if ever written armed. Zero P gain is the clearest such value.
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.velocity_p, PidPGain::ZERO);
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.position_p, PidPGain::ZERO);
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.velocity_i, PidIGain::ZERO);
        assert_eq!(CONFIG_RESET_PLACEHOLDER_GAINS.position_i, PidIGain::ZERO);
    }

    #[test]
    fn default_outer_gains_value_is_unchanged() {
        // DEFAULT_OUTER_GAINS keeps its historical value: it is now the
        // fresh-commission fallback for boards outside the gain-floor
        // measurement's supported bandwidth range, not an inert placeholder.
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
