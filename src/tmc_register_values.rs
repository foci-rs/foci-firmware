//! TMC4671 register words the firmware writes, packed through the typed layouts.

use foci_tmc4671::registers::{
    AdcI0ScaleOffset, AdcI1ScaleOffset, ModeRampModeMotion, MotionMode, MotorType,
    MotorTypeNPolePairs, PwmSvChop,
};

/// `MODE_RAMP_MODE_MOTION`: stopped, no output drive.
pub const MOTION_STOPPED: u32 = motion(MotionMode::Stopped, false);
/// `MODE_RAMP_MODE_MOTION`: external UQ/UD voltage drive.
pub const MOTION_UQ_UD_EXT: u32 = motion(MotionMode::UqUdExt, false);
/// `MODE_RAMP_MODE_MOTION`: torque mode with the advanced PI structure.
pub const MOTION_TORQUE_ADVANCED_PI: u32 = motion(MotionMode::Torque, true);
/// `MODE_RAMP_MODE_MOTION`: velocity mode with the advanced PI structure.
pub const MOTION_VELOCITY_ADVANCED_PI: u32 = motion(MotionMode::Velocity, true);
/// `MODE_RAMP_MODE_MOTION`: position mode with the advanced PI structure.
pub const MOTION_POSITION_ADVANCED_PI: u32 = motion(MotionMode::Position, true);

/// `PWM_SV_CHOP`: PWM off, free running. Required during ADC offset calibration.
pub const PWM_CHOP_OFF: u32 = PwmSvChop::new().with_pwm_chop(0).into_bits();
/// `PWM_SV_CHOP`: centered PWM for FOC, space-vector PWM disabled.
pub const PWM_CHOP_CENTERED_FOC: u32 = PwmSvChop::new().with_pwm_chop(7).into_bits();

/// `MOTOR_TYPE_N_POLE_PAIRS` for a two-phase 1.8 degree stepper (50 pole pairs).
///
/// Motor type 2 is the only safe value for the steppers FOCI drives: type 3
/// (BLDC) computes wrong current vectors and destroys them.
pub const STEPPER_MOTOR_TYPE_N_POLE_PAIRS: u32 = MotorTypeNPolePairs::new()
    .with_motor_type(MotorType::TwoPhase)
    .with_n_pole_pairs(50)
    .into_bits();

/// Unity ADC gain in the signed Q8.8 `ADC_Ix_SCALE` field.
const ADC_SCALE_UNITY: i16 = 256;

/// `ADC_I0_SCALE_OFFSET` with unity gain and the given zero-current offset.
pub const fn adc_i0_scale_offset(offset: u16) -> u32 {
    AdcI0ScaleOffset::new()
        .with_scale(ADC_SCALE_UNITY)
        .with_offset(offset)
        .into_bits()
}

/// `ADC_I1_SCALE_OFFSET` with unity gain and the given zero-current offset.
pub const fn adc_i1_scale_offset(offset: u16) -> u32 {
    AdcI1ScaleOffset::new()
        .with_scale(ADC_SCALE_UNITY)
        .with_offset(offset)
        .into_bits()
}

const fn motion(mode: MotionMode, advanced_pi: bool) -> u32 {
    ModeRampModeMotion::new()
        .with_mode_motion(mode)
        .with_mode_pid_type(advanced_pi)
        .into_bits()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_words_match_the_datasheet_encoding() {
        assert_eq!(MOTION_STOPPED, 0);
        assert_eq!(MOTION_UQ_UD_EXT, 8);
        assert_eq!(MOTION_TORQUE_ADVANCED_PI, 0x8000_0001);
        assert_eq!(MOTION_VELOCITY_ADVANCED_PI, 0x8000_0002);
        assert_eq!(MOTION_POSITION_ADVANCED_PI, 0x8000_0003);
        assert_eq!(PWM_CHOP_OFF, 0);
        assert_eq!(PWM_CHOP_CENTERED_FOC, 7);
        assert_eq!(STEPPER_MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50);
        assert_eq!(adc_i0_scale_offset(33_000), (256 << 16) | 33_000);
        assert_eq!(adc_i1_scale_offset(0), 256 << 16);
    }
}
