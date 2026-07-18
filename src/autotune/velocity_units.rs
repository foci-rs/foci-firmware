/// Failure to convert or sign a TMC4671 mechanical-RPM value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityUnitError {
    /// The requested mechanical rate rounds to a zero controller target.
    ZeroTarget,
    /// A signed direction other than positive or negative was requested.
    InvalidDirection,
    /// Checked conversion exceeded the signed register domain.
    ArithmeticOverflow,
}

/// Signed mechanical RPM used by the TMC4671 velocity controller.
///
/// With `VELOCITY_SELECTION = phi_m_abn`, both `PID_VELOCITY_TARGET` and
/// `PID_VELOCITY_ACTUAL` use this domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcMechanicalRpm(i32);

impl TmcMechanicalRpm {
    /// Convert positive mechanical millirevolutions per second to nearest RPM.
    pub fn from_mrev_s_nearest(value: u32) -> Result<Self, VelocityUnitError> {
        let rounded = u64::from(value)
            .checked_mul(60)
            .and_then(|product| product.checked_add(500))
            .ok_or(VelocityUnitError::ArithmeticOverflow)?
            / 1_000;
        if rounded == 0 {
            return Err(VelocityUnitError::ZeroTarget);
        }
        let raw = i32::try_from(rounded).map_err(|_| VelocityUnitError::ArithmeticOverflow)?;
        Ok(Self(raw))
    }

    /// Construct a value at an explicit TMC register or wire boundary.
    pub const fn from_register(raw: i32) -> Self {
        Self(raw)
    }

    /// Return the exact signed register representation.
    pub const fn raw(self) -> i32 {
        self.0
    }

    /// Apply a positive or negative direction to a nonzero magnitude.
    pub fn with_direction(self, direction: i32) -> Result<Self, VelocityUnitError> {
        let magnitude = self
            .0
            .checked_abs()
            .ok_or(VelocityUnitError::ArithmeticOverflow)?;
        if magnitude == 0 {
            return Err(VelocityUnitError::ZeroTarget);
        }
        match direction {
            1 => Ok(Self(magnitude)),
            -1 => Ok(Self(-magnitude)),
            _ => Err(VelocityUnitError::InvalidDirection),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{TmcMechanicalRpm, VelocityUnitError};

    #[test]
    fn planned_mrev_per_second_rounds_to_nearest_mechanical_rpm() {
        assert_eq!(
            TmcMechanicalRpm::from_mrev_s_nearest(2_929).unwrap().raw(),
            176
        );
        assert_eq!(
            TmcMechanicalRpm::from_mrev_s_nearest(2_925).unwrap().raw(),
            176
        );
    }

    #[test]
    fn zero_target_is_rejected_before_motion() {
        assert_eq!(
            TmcMechanicalRpm::from_mrev_s_nearest(1),
            Err(VelocityUnitError::ZeroTarget)
        );
    }

    #[test]
    fn direction_is_applied_without_exposing_the_field() {
        let magnitude = TmcMechanicalRpm::from_register(176);

        assert_eq!(magnitude.with_direction(1).unwrap().raw(), 176);
        assert_eq!(magnitude.with_direction(-1).unwrap().raw(), -176);
        assert_eq!(
            magnitude.with_direction(0),
            Err(VelocityUnitError::InvalidDirection)
        );
    }
}
