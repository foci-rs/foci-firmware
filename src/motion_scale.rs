//! Independent encoder-feedback and planner-motion scale definitions.

use foci_klipper::pulse_mapper::{PulseMapperConfig, PulseMapperConfigError};

const QUADRATURE_COUNTS_PER_PULSE: u32 = 4;
/// `PID_POSITION_ACTUAL` units per mechanical revolution (phi_m format).
pub(crate) const POSITION_UNITS_PER_REV: u64 = 65_536;

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

/// Bounded host-visible reasons for rejecting motion-scale configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum MotionScaleRejection {
    /// Encoder pulses per revolution was zero.
    EncoderPprZero = 1,
    /// Encoder quadrature counts would overflow `u32`.
    EncoderPprTooLarge = 2,
    /// Planner steps per revolution was zero.
    PlannerStepsPerRevZero = 3,
    /// Planner steps per revolution exceeded the mapper bound.
    PlannerStepsPerRevTooLarge = 4,
    /// The P1 motor runtime still considered the motor enabled.
    RuntimeEnabled = 5,
    /// The shared armed-state mirror was set.
    MotorArmed = 6,
    /// The physical step queue was not empty.
    QueueNotEmpty = 7,
    /// The physical step timer was active.
    TimerActive = 8,
    /// The board could not install the validated mapper.
    MapperInstallFailed = 9,
    /// The encoder-count register write failed.
    EncoderRegisterWriteFailed = 10,
    /// The physical step-width register write failed.
    StepWidthRegisterWriteFailed = 11,
    /// The P2-to-P1 request queue was full.
    RequestQueueFull = 12,
    /// The board-local channel was unavailable.
    InvalidChannel = 13,
}

impl From<MotionScaleError> for MotionScaleRejection {
    fn from(error: MotionScaleError) -> Self {
        match error {
            MotionScaleError::ZeroEncoderPpr => Self::EncoderPprZero,
            MotionScaleError::EncoderCountsPerRevOverflow => Self::EncoderPprTooLarge,
            MotionScaleError::ZeroPlannerStepsPerRev => Self::PlannerStepsPerRevZero,
            MotionScaleError::PlannerStepsPerRevTooLarge => Self::PlannerStepsPerRevTooLarge,
        }
    }
}

impl MotionScaleRejection {
    /// Stable wire code used by `foci_motion_scale_rejected`.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Rejection was detected by the high-priority protocol task.
pub const MOTION_SCALE_REJECTION_STAGE_P2: u8 = 0;
/// Rejection was detected by the slow-control transaction task.
pub const MOTION_SCALE_REJECTION_STAGE_P1: u8 = 1;

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
    pub const fn physical_step_width(&self) -> u32 {
        self.pulse_mapper.step_width()
    }

    /// Returns twice the physical mapper's instantaneous position-error bound.
    pub const fn position_error_bound_half_units(&self) -> u32 {
        self.pulse_mapper.position_error_bound_half_units()
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

    /// Builds the exact planner-timing mechanical-RPM ratio for velocity feedforward.
    ///
    /// Returns `None` when `step_timer_hz` is zero.
    pub fn planner_rpm_scale(&self, step_timer_hz: u32) -> Option<PlannerRpmScale> {
        PlannerRpmScale::new(self.planner_steps_per_rev(), step_timer_hz)
    }
}

/// Reduced exact ratio for converting planner step timing to TMC velocity units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PlannerVelocityScale {
    numerator: u64,
    denominator: u64,
}

impl PlannerVelocityScale {
    pub(crate) fn new(
        planner_steps_per_rev: u32,
        step_timer_hz: u32,
        f_pwm_hz: u32,
    ) -> Option<Self> {
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

/// Reduced exact ratio for converting planner step timing to mechanical RPM.
///
/// A planner step every `denominator` step-timer ticks corresponds to one
/// full mechanical revolution every `denominator * planner_steps_per_rev`
/// ticks, so RPM is `60 * step_timer_hz / planner_steps_per_rev` — independent
/// of the PWM frequency and of position units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PlannerRpmScale {
    numerator: u64,
    denominator: u64,
}

impl PlannerRpmScale {
    pub(crate) fn new(planner_steps_per_rev: u32, step_timer_hz: u32) -> Option<Self> {
        if step_timer_hz == 0 || planner_steps_per_rev == 0 {
            return None;
        }
        let numerator = 60 * u64::from(step_timer_hz);
        let denominator = u64::from(planner_steps_per_rev);
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

    /// Returns the reduced `60 * step_timer_hz` factor.
    pub const fn numerator(&self) -> u64 {
        self.numerator
    }

    /// Returns the reduced `planner_steps_per_rev` factor.
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
    fn rejection_wire_codes_are_stable_and_distinct() {
        let reasons = [
            MotionScaleRejection::EncoderPprZero,
            MotionScaleRejection::EncoderPprTooLarge,
            MotionScaleRejection::PlannerStepsPerRevZero,
            MotionScaleRejection::PlannerStepsPerRevTooLarge,
            MotionScaleRejection::RuntimeEnabled,
            MotionScaleRejection::MotorArmed,
            MotionScaleRejection::QueueNotEmpty,
            MotionScaleRejection::TimerActive,
            MotionScaleRejection::MapperInstallFailed,
            MotionScaleRejection::EncoderRegisterWriteFailed,
            MotionScaleRejection::StepWidthRegisterWriteFailed,
            MotionScaleRejection::RequestQueueFull,
            MotionScaleRejection::InvalidChannel,
        ];

        for (index, reason) in reasons.into_iter().enumerate() {
            assert_eq!(reason.code(), index as u8 + 1);
        }
    }

    #[test]
    fn encoder_resolution_does_not_change_planner_grid() {
        let low_resolution = MotionScale::new(500, 3_200).unwrap();
        let high_resolution = MotionScale::new(4_000, 3_200).unwrap();

        assert_eq!(low_resolution.tmc_grid(), high_resolution.tmc_grid());
        assert_eq!(
            low_resolution.physical_step_width(),
            high_resolution.physical_step_width()
        );
    }

    #[test]
    fn planner_velocity_scale_is_reduced() {
        let scale = MotionScale::new(1_000, 3_200).unwrap();
        let velocity = scale.planner_velocity_scale(84_000_000, 25_000).unwrap();

        assert_eq!(velocity.numerator(), 344_064);
        assert_eq!(velocity.denominator(), 5);
    }

    #[test]
    fn planner_rpm_scale_is_reduced_and_coprime() {
        let scale = MotionScale::new(1_000, 3_200).unwrap();
        // 60*84_000_000 = 5_040_000_000; gcd(5_040_000_000, 3_200) = 3_200.
        let rpm = scale.planner_rpm_scale(84_000_000).unwrap();
        assert_eq!(rpm.numerator(), 1_575_000);
        assert_eq!(rpm.denominator(), 1);
        assert_eq!(
            greatest_common_divisor(rpm.numerator(), rpm.denominator()),
            1
        );
    }

    #[test]
    fn planner_rpm_scale_rejects_zero_step_timer() {
        let scale = MotionScale::new(1_000, 3_200).unwrap();
        assert!(scale.planner_rpm_scale(0).is_none());
    }

    #[test]
    fn planner_rpm_equals_pos_per_pwm_times_exact_factor() {
        // Exact-rational cross-check (no truncation):
        // rpm.num * ppwm.den * 65536 == ppwm.num * rpm.den * 60 * 25000.
        let scale = MotionScale::new(1_000, 3_200).unwrap();
        let rpm = scale.planner_rpm_scale(84_000_000).unwrap();
        let ppwm = scale.planner_velocity_scale(84_000_000, 25_000).unwrap();
        let lhs = (rpm.numerator() as u128) * (ppwm.denominator() as u128) * 65_536;
        let rhs = (ppwm.numerator() as u128) * (rpm.denominator() as u128) * 60 * 25_000;
        assert_eq!(lhs, rhs);
    }

    #[test]
    fn ldo_baseline_has_independent_encoder_and_planner_scales() {
        let scale = MotionScale::new(1_000, 3_200).unwrap();

        assert_eq!(scale.encoder_counts_per_rev(), 4_000);
        assert_eq!(scale.planner_steps_per_rev(), 3_200);
        assert_eq!(scale.tmc_grid(), 4_096);
        assert_eq!(scale.physical_step_width(), 16);
        assert_eq!(scale.position_error_bound_half_units() / 2, 8);
    }

    /// Drift guard (green on arrival, not a red TDD test): pins the host-side
    /// `derived_exec_stats` formulas in `klipper-foci/diagnostics/stepper_scale.py` to the
    /// firmware constants they mirror. It exists to fail if `StepPulseTiming` or
    /// `PulseMapperConfig` ever change a formula the host duplicates, since the host cannot see
    /// those firmware types directly.
    #[test]
    fn host_exec_stats_derivations_match_firmware_formulas() {
        use crate::step_pulse::StepPulseTiming;
        use foci_klipper::pulse_mapper::PulseMapperConfig;

        fn expected_tmc_grid(planner_steps_per_rev: u32) -> u32 {
            if planner_steps_per_rev == 0 {
                0
            } else {
                planner_steps_per_rev.next_power_of_two().min(65_536)
            }
        }

        for clock_freq in [84_000_000_u32, 260_000_000_u32] {
            let timing = StepPulseTiming::new(clock_freq).unwrap();
            let step_half_period_ticks = clock_freq.div_ceil(25_000_000);

            assert_eq!(timing.half_period_ticks(), step_half_period_ticks);
            assert_eq!(timing.dir_setup_ticks(), 2 * step_half_period_ticks);
            assert_eq!(
                timing.waveform_worst_case_ticks(),
                6 * step_half_period_ticks
            );
            assert_eq!(timing.fatal_lateness_ticks(), clock_freq / 1_000);
        }

        for planner_steps_per_rev in [0_u32, 200, 6_400, 65_536, 70_000] {
            let expected_grid = expected_tmc_grid(planner_steps_per_rev);

            if planner_steps_per_rev == 0 {
                assert_eq!(expected_grid, 0);
                continue;
            }

            let config = PulseMapperConfig::new(planner_steps_per_rev).unwrap();
            assert_eq!(config.tmc_grid(), expected_grid);
            assert_eq!(config.step_width(), 65_536 / expected_grid);
        }
    }
}
