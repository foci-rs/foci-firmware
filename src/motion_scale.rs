//! Independent encoder-feedback and planner-motion scale definitions.

use foci_klipper::pulse_mapper::{PulseMapperConfig, PulseMapperConfigError};

const QUADRATURE_COUNTS_PER_PULSE: u32 = 4;
const POSITION_UNITS_PER_REV: u64 = 65_536;

/// Errors returned when constructing a [`MotionScale`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MotionScaleError {
    /// The encoder pulses-per-revolution value was zero.
    ZeroEncoderPpr,
    /// The quadrature encoder count overflowed `u32`.
    EncoderCountsPerRevOverflow,
    /// The planner steps-per-revolution value was zero.
    ZeroPlannerStepsPerRev,
    /// The planner steps-per-revolution value exceeded the supported bound.
    PlannerStepsPerRevTooLarge,
}

/// Complete validated feedback, planner, and physical-pulse scale for one motor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MotionScale {
    encoder_ppr: u32,
    encoder_counts_per_rev: u32,
    pulse_mapper: PulseMapperConfig,
}

impl MotionScale {
    /// Constructs independent encoder and planner scales.
    ///
    /// # Errors
    ///
    /// Returns an error for zero or overflowing encoder resolution and for a
    /// planner resolution outside the bounds enforced by [`PulseMapperConfig`].
    pub fn new(encoder_ppr: u32, planner_steps_per_rev: u32) -> Result<Self, MotionScaleError> {
        if encoder_ppr == 0 {
            return Err(MotionScaleError::ZeroEncoderPpr);
        }
        let encoder_counts_per_rev = encoder_ppr
            .checked_mul(QUADRATURE_COUNTS_PER_PULSE)
            .ok_or(MotionScaleError::EncoderCountsPerRevOverflow)?;
        let pulse_mapper =
            PulseMapperConfig::new(planner_steps_per_rev).map_err(|error| match error {
                PulseMapperConfigError::ZeroPlannerStepsPerRev => {
                    MotionScaleError::ZeroPlannerStepsPerRev
                }
                PulseMapperConfigError::PlannerStepsPerRevTooLarge => {
                    MotionScaleError::PlannerStepsPerRevTooLarge
                }
            })?;

        Ok(Self {
            encoder_ppr,
            encoder_counts_per_rev,
            pulse_mapper,
        })
    }

    /// Returns encoder pulses per mechanical revolution before quadrature.
    pub const fn encoder_ppr(&self) -> u32 {
        self.encoder_ppr
    }

    /// Returns quadrature encoder counts per mechanical revolution.
    pub const fn encoder_counts_per_rev(&self) -> u32 {
        self.encoder_counts_per_rev
    }

    /// Returns logical planner steps per mechanical revolution.
    pub const fn planner_steps_per_rev(&self) -> u32 {
        self.pulse_mapper.planner_steps_per_rev()
    }

    /// Returns physical TMC pulses per mechanical revolution.
    pub const fn tmc_grid(&self) -> u32 {
        self.pulse_mapper.tmc_grid()
    }

    /// Returns TMC4671 position units added by each physical pulse.
    pub const fn step_width(&self) -> u32 {
        self.pulse_mapper.step_width()
    }

    /// Returns twice the physical mapper's instantaneous position-error bound.
    pub const fn position_error_bound_half_units(&self) -> u32 {
        self.pulse_mapper.position_error_bound_half_units()
    }

    /// Converts signed encoder counts to TMC4671 mechanical-position units.
    ///
    /// Exact half-unit ties round away from zero, symmetrically by sign.
    pub fn encoder_count_to_position_units(&self, count: i32) -> i64 {
        round_signed_ratio(count, POSITION_UNITS_PER_REV, self.encoder_counts_per_rev)
    }

    /// Converts signed logical planner steps to TMC4671 mechanical-position units.
    ///
    /// Exact half-unit ties round away from zero, symmetrically by sign.
    pub fn planner_steps_to_position_units(&self, steps: i32) -> i64 {
        round_signed_ratio(steps, POSITION_UNITS_PER_REV, self.planner_steps_per_rev())
    }

    /// Builds the exact planner-timing velocity ratio for a control loop.
    ///
    /// Returns `None` when either supplied frequency is zero.
    pub fn planner_velocity_scale(
        &self,
        step_timer_hz: u32,
        f_pwm_hz: u32,
    ) -> Option<PlannerVelocityScale> {
        PlannerVelocityScale::new(self.planner_steps_per_rev(), step_timer_hz, f_pwm_hz)
    }
}

/// Reduced exact ratio for converting planner step timing to TMC velocity units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PlannerVelocityScale {
    numerator: u64,
    denominator: u64,
}

impl PlannerVelocityScale {
    fn new(planner_steps_per_rev: u32, step_timer_hz: u32, f_pwm_hz: u32) -> Option<Self> {
        if step_timer_hz == 0 || f_pwm_hz == 0 {
            return None;
        }
        let numerator = POSITION_UNITS_PER_REV * u64::from(step_timer_hz);
        let denominator = u64::from(planner_steps_per_rev) * u64::from(f_pwm_hz);
        let divisor = greatest_common_divisor(numerator, denominator);
        Some(Self {
            numerator: numerator / divisor,
            denominator: denominator / divisor,
        })
    }

    #[cfg(test)]
    pub(crate) const fn from_reduced(numerator: u64, denominator: u64) -> Self {
        Self {
            numerator,
            denominator,
        }
    }

    /// Returns the reduced `65536 * step_timer_hz` factor.
    pub const fn numerator(&self) -> u64 {
        self.numerator
    }

    /// Returns the reduced `planner_steps_per_rev * f_pwm_hz` factor.
    pub const fn denominator(&self) -> u64 {
        self.denominator
    }
}

pub(crate) fn greatest_common_divisor(mut left: u64, mut right: u64) -> u64 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn round_signed_ratio(value: i32, numerator: u64, denominator: u32) -> i64 {
    let scaled = i128::from(value) * i128::from(numerator);
    let denominator = i128::from(denominator);
    let rounding = denominator / 2;
    let rounded = if scaled < 0 {
        (scaled - rounding) / denominator
    } else {
        (scaled + rounding) / denominator
    };
    rounded as i64
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_klipper::pulse_mapper::MAX_PLANNER_STEPS_PER_REV;

    #[test]
    fn rejects_invalid_encoder_and_planner_scales() {
        assert_eq!(
            MotionScale::new(0, 3_200),
            Err(MotionScaleError::ZeroEncoderPpr)
        );
        assert_eq!(
            MotionScale::new(u32::MAX / 4 + 1, 3_200),
            Err(MotionScaleError::EncoderCountsPerRevOverflow)
        );
        assert_eq!(
            MotionScale::new(1_000, 0),
            Err(MotionScaleError::ZeroPlannerStepsPerRev)
        );
        assert_eq!(
            MotionScale::new(1_000, MAX_PLANNER_STEPS_PER_REV + 1),
            Err(MotionScaleError::PlannerStepsPerRevTooLarge)
        );
    }

    #[test]
    fn encoder_resolution_does_not_change_planner_grid() {
        let low_resolution = MotionScale::new(500, 3_200).unwrap();
        let high_resolution = MotionScale::new(4_000, 3_200).unwrap();

        assert_eq!(low_resolution.tmc_grid(), high_resolution.tmc_grid());
        assert_eq!(low_resolution.step_width(), high_resolution.step_width());
    }

    #[test]
    fn signed_position_conversions_round_ties_away_from_zero() {
        let scale = MotionScale::new(8_192, 32_768).unwrap();

        assert_eq!(scale.encoder_count_to_position_units(1), 2);
        assert_eq!(scale.encoder_count_to_position_units(-1), -2);
        assert_eq!(scale.planner_steps_to_position_units(1), 2);
        assert_eq!(scale.planner_steps_to_position_units(-1), -2);

        let half_unit_scale = MotionScale::new(32_768, 131_072).unwrap();
        assert_eq!(half_unit_scale.encoder_count_to_position_units(1), 1);
        assert_eq!(half_unit_scale.encoder_count_to_position_units(-1), -1);
        assert_eq!(half_unit_scale.planner_steps_to_position_units(1), 1);
        assert_eq!(half_unit_scale.planner_steps_to_position_units(-1), -1);
    }

    #[test]
    fn position_conversions_use_their_independent_exact_rationals() {
        let scale = MotionScale::new(1_000, 3_200).unwrap();

        assert_eq!(scale.encoder_count_to_position_units(125), 2_048);
        assert_eq!(scale.planner_steps_to_position_units(125), 2_560);
        assert_eq!(scale.encoder_count_to_position_units(-125), -2_048);
        assert_eq!(scale.planner_steps_to_position_units(-125), -2_560);
    }

    #[test]
    fn planner_velocity_scale_is_reduced() {
        let scale = MotionScale::new(1_000, 3_200).unwrap();
        let velocity = scale.planner_velocity_scale(84_000_000, 25_000).unwrap();

        assert_eq!(velocity.numerator(), 344_064);
        assert_eq!(velocity.denominator(), 5);
    }

    #[test]
    fn ldo_baseline_has_independent_encoder_and_planner_scales() {
        let scale = MotionScale::new(1_000, 3_200).unwrap();

        assert_eq!(scale.encoder_counts_per_rev(), 4_000);
        assert_eq!(scale.planner_steps_per_rev(), 3_200);
        assert_eq!(scale.tmc_grid(), 4_096);
        assert_eq!(scale.step_width(), 16);
        assert_eq!(scale.position_error_bound_half_units() / 2, 8);
    }
}
