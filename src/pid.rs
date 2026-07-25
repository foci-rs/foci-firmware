//! PID gain register helpers for TMC4671 control loops.

use foci_tmc4671::pid::{PidGain, PidGainOutOfRange, PidGainPair};

/// Computed PI gains for TMC4671 flux and torque controllers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InnerLoopGains {
    /// Flux P gain — raw TMC4671 register value (Q8.8 format).
    pub flux_p: PidGain,
    /// Flux I gain — raw TMC4671 Q8.8 register value.
    pub flux_i: PidGain,
    /// Torque P gain — raw TMC4671 register value (Q8.8 format).
    pub torque_p: PidGain,
    /// Torque I gain — raw TMC4671 Q8.8 register value.
    pub torque_i: PidGain,
}

/// Computed PI gains for TMC4671 position and velocity outer loops.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct OuterLoopGains {
    /// Position P gain — raw TMC4671 register value (Q8.8 format).
    pub position_p: PidGain,
    /// Position I gain — raw TMC4671 register value (Q8.8 format).
    pub position_i: PidGain,
    /// Velocity P gain — raw TMC4671 register value (Q8.8 format).
    pub velocity_p: PidGain,
    /// Velocity I gain — raw TMC4671 register value (Q8.8 format).
    pub velocity_i: PidGain,
}

/// Conservative default gains for when no R/L data is available.
/// These are intentionally low — stable but sluggish.
pub const DEFAULT_INNER_GAINS: InnerLoopGains = InnerLoopGains {
    flux_p: match PidGain::new(256) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    flux_i: match PidGain::new(26) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    torque_p: match PidGain::new(256) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    torque_i: match PidGain::new(26) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
};

/// Conservative default gains for the position and velocity loops.
pub const DEFAULT_OUTER_GAINS: OuterLoopGains = OuterLoopGains {
    position_p: match PidGain::new(640) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    position_i: PidGain::ZERO,
    velocity_p: match PidGain::new(1152) {
        Ok(gain) => gain,
        Err(_) => panic!("valid default"),
    },
    velocity_i: PidGain::ZERO,
};

#[cfg(test)]
pub(crate) const fn test_gain(raw: u16) -> PidGain {
    match PidGain::new(raw) {
        Ok(gain) => gain,
        Err(_) => panic!("test gain must be in range"),
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
        position_p: test_gain(position_p),
        position_i: test_gain(position_i),
        velocity_p: test_gain(velocity_p),
        velocity_i: test_gain(velocity_i),
    }
}

impl InnerLoopGains {
    /// Pack gains into TMC4671 register values.
    ///
    /// Returns (flux_register, torque_register) ready for
    /// `PID_FLUX_P_FLUX_I` (0x54) and `PID_TORQUE_P_TORQUE_I` (0x56).
    pub fn to_registers(self) -> (u32, u32) {
        let flux = PidGainPair::new_legacy(self.flux_p, self.flux_i).register_value();
        let torque = PidGainPair::new_legacy(self.torque_p, self.torque_i).register_value();
        (flux, torque)
    }

    /// Unpack gains from TMC4671 register values.
    pub fn from_registers(flux_reg: u32, torque_reg: u32) -> Result<Self, PidGainOutOfRange> {
        let flux = PidGainPair::try_from_register(flux_reg)?;
        let torque = PidGainPair::try_from_register(torque_reg)?;
        Ok(Self {
            flux_p: PidGain::new(flux.p.raw())?,
            flux_i: PidGain::new(flux.i.raw())?,
            torque_p: PidGain::new(torque.p.raw())?,
            torque_i: PidGain::new(torque.i.raw())?,
        })
    }
}

impl OuterLoopGains {
    /// Pack gains into TMC4671 register values.
    ///
    /// Returns `(velocity_register, position_register)` ready for
    /// `PID_VELOCITY_P_VELOCITY_I` and `PID_POSITION_P_POSITION_I`.
    pub fn to_registers(self) -> (u32, u32) {
        let velocity = PidGainPair::new_legacy(self.velocity_p, self.velocity_i).register_value();
        let position = PidGainPair::new_legacy(self.position_p, self.position_i).register_value();
        (velocity, position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
            flux_p: PidGain::MAX,
            flux_i: PidGain::MAX,
            torque_p: PidGain::MAX,
            torque_i: PidGain::MAX,
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
            flux_p: PidGain::new(707).unwrap(),
            flux_i: PidGain::new(162).unwrap(),
            torque_p: PidGain::new(707).unwrap(),
            torque_i: PidGain::new(162).unwrap(),
        };

        let (flux_reg, torque_reg) = gains.to_registers();

        assert_eq!(flux_reg, 0x02c3_00a2);
        assert_eq!(torque_reg, 0x02c3_00a2);
    }

    #[test]
    fn outer_loop_gains_to_registers_places_p_in_upper_half() {
        let gains = OuterLoopGains {
            position_p: test_gain(256),
            position_i: test_gain(3),
            velocity_p: test_gain(6656),
            velocity_i: test_gain(4),
        };

        let (velocity_reg, position_reg) = gains.to_registers();

        assert_eq!(velocity_reg, 0x1a00_0004);
        assert_eq!(position_reg, 0x0100_0003);
    }
}
