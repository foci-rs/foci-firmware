//! Public data types for the impedance-identification engine.

/// Fixed number of injection windows in the v1 impedance profile.
pub const IMPEDANCE_PROFILE_POINTS: usize = 2;

/// Fixed capture size exposed by the standard host-visible profile.
pub const IMPEDANCE_SAMPLE_COUNT: usize = 128;

/// One open-loop voltage injection window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceInjectionPoint {
    /// Absolute UD magnitude applied during the window.
    pub ud_abs: u16,
    /// Electrical injection frequency in millihertz.
    pub frequency_millihz: u32,
    /// Open-loop electrical velocity in rpm derived from the injection frequency.
    pub openloop_velocity_rpm: i32,
    /// Bounded open-loop acceleration used when ramping into the injection point.
    pub openloop_acceleration_rpm_per_s: u32,
    /// Settling time before the sample capture starts.
    pub settle_ms: u16,
}

/// One captured current sample from the impedance window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceSample {
    /// Relative sample timestamp in microseconds.
    pub t_us: u32,
    /// Raw TMC current count for channel 0.
    pub i_ch0: i16,
    /// Raw TMC current count for channel 1.
    pub i_ch1: i16,
}

/// Power-stage setup read back before impedance identification energizes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedancePowerStageReadback {
    /// Raw `PWM_MAXCNT` register readback.
    pub pwm_maxcnt_readback: u32,
    /// Raw `PWM_BBM_H_BBM_L` register readback.
    pub bbm_readback: u32,
    /// Raw `DSADC_MDEC_B_MDEC_A` register readback.
    pub dsadc_mdec_readback: u32,
    /// Raw `PWM_SV_CHOP` register readback.
    pub pwm_sv_chop_readback: u32,
}

impl ImpedancePowerStageReadback {
    /// Empty readback evidence used before power-stage verification runs.
    pub const EMPTY: Self = Self {
        pwm_maxcnt_readback: 0,
        bbm_readback: 0,
        dsadc_mdec_readback: 0,
        pwm_sv_chop_readback: 0,
    };
}

/// Warning bitfield returned by impedance identification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceWarningFlags(pub u32);

impl ImpedanceWarningFlags {
    pub const NONE: Self = Self(0);
    pub const PHYSICAL_PROJECTION_UNAVAILABLE: Self = Self(1 << 0);
    pub const HIGH_RESIDUAL: Self = Self(1 << 1);
    pub const REPEATABILITY: Self = Self(1 << 2);
    pub const SCALAR_2F_MISMATCH: Self = Self(1 << 3);
    pub const HIGH_NOISE: Self = Self(1 << 4);
    pub const ENCODER_MOVED: Self = Self(1 << 5);
    pub const STATUS_FLAGS: Self = Self(1 << 6);

    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    pub const fn intersects(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }

    pub fn insert(&mut self, other: Self) {
        self.0 |= other.0;
    }
}

/// Hard-failure reason for impedance identification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ImpedanceError {
    NotArmed,
    NotReady,
    UnsupportedMotorType,
    MissingCountSubstrate,
    InsufficientVoltageAuthority,
    UnsupportedProfile,
    ProfileReadbackMismatch,
    CurrentLimitExceeded,
    EncoderMoved,
    UnsafeStatusFlags,
    LowSignal,
    PoorCoverage,
    Scalar2fMismatch,
    HighResidual,
    RepeatabilityMismatch,
    BackendIo,
    RestoreFailed,
}
