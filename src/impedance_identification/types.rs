//! Public data types for the impedance-identification engine.

/// Fixed number of injection windows in the v1 impedance profile.
pub const IMPEDANCE_PROFILE_POINTS: usize = 2;

/// Fixed capture size exposed by the standard host-visible profile.
pub const IMPEDANCE_SAMPLE_COUNT: usize = 128;

/// Fixed number of angle windows in the v2 impedance profile.
pub const PULSATING_IMPEDANCE_PROFILE_POINTS: usize = 9;

/// Fixed carrier sample count exposed by the standard host-visible profile.
pub const PULSATING_IMPEDANCE_SAMPLE_COUNT: usize = 160;

/// One open-loop voltage injection window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceInjectionPoint {
    /// Absolute UD magnitude applied during the window.
    pub ud_abs: u16,
    /// Electrical injection frequency in millihertz.
    pub frequency_millihz: u32,
    /// Bounded open-loop acceleration used when ramping into the injection point.
    pub openloop_acceleration_rpm_per_s: u32,
    /// Settling time before the sample capture starts.
    pub settle_ms: u16,
}

/// One fixed-angle pulsating voltage window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PulsatingImpedancePoint {
    /// Electrical angle for `PHI_E_EXT`, in 16-bit turn counts.
    pub angle_electrical_counts: u16,
    /// Absolute UD magnitude applied by the square carrier.
    pub ud_abs: u16,
}

/// One captured current sample from the impedance window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceSample {
    /// Relative sample timestamp in microseconds.
    pub t_us: u32,
    /// `OPENLOOP_PHI` sampled with this current sample, in 16-bit turn counts.
    pub openloop_phi: u16,
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

/// Last profile readback mismatch observed before a profile-readback failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceReadbackMismatch {
    /// TMC4671 register address that failed readback, or zero if none.
    pub addr: u8,
    /// Expected raw register value.
    pub expected: u32,
    /// Actual raw register value read from the chip.
    pub actual: u32,
}

impl ImpedanceReadbackMismatch {
    /// Empty mismatch evidence used when no profile readback failed.
    pub const EMPTY: Self = Self {
        addr: 0,
        expected: 0,
        actual: 0,
    };
}

/// DC baseline current observation sampled during the static-UD precheck.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceBaselineObservation {
    /// Direct ADC IUX sample from `ADC_IWY_IUX`.
    pub dc_adc_iux_sample: i16,
    /// Direct ADC IWY sample from `ADC_IWY_IUX`.
    pub dc_adc_iwy_sample: i16,
    /// Stationary-frame FOC IUX sample from `INTERIM_FOC_IWY_IUX`.
    pub dc_foc_iux_sample: i16,
    /// Stationary-frame FOC IWY sample from `INTERIM_FOC_IWY_IUX`.
    pub dc_foc_iwy_sample: i16,
    /// Park-transformed q-axis current sample from `INTERIM_FOC_IQ_ID`.
    pub dc_foc_iq_sample: i16,
    /// Park-transformed d-axis current sample from `INTERIM_FOC_IQ_ID`.
    pub dc_foc_id_sample: i16,
}

impl ImpedanceBaselineObservation {
    /// Empty observation used before a baseline has been sampled.
    pub const EMPTY: Self = Self {
        dc_adc_iux_sample: 0,
        dc_adc_iwy_sample: 0,
        dc_foc_iux_sample: 0,
        dc_foc_iwy_sample: 0,
        dc_foc_iq_sample: 0,
        dc_foc_id_sample: 0,
    };
}

/// Per-injection-point chip observability sampled while AC voltage is active.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedancePointObservation {
    /// STATUS_FLAGS observed during this point's DC and AC windows.
    pub status_flags_or: u32,
    /// OPENLOOP_PHI sampled while AC voltage is active.
    pub openloop_phi_sample: u16,
    /// OPENLOOP_VELOCITY_ACTUAL sampled while AC voltage is active.
    pub openloop_velocity_actual: i32,
    /// Direct ADC IUX sample from `ADC_IWY_IUX`.
    pub adc_iux_sample: i16,
    /// Direct ADC IWY sample from `ADC_IWY_IUX`.
    pub adc_iwy_sample: i16,
    /// Stationary-frame FOC IUX sample from `INTERIM_FOC_IWY_IUX`.
    pub foc_iux_sample: i16,
    /// Stationary-frame FOC IWY sample from `INTERIM_FOC_IWY_IUX`.
    pub foc_iwy_sample: i16,
    /// Pre-limit FOC UQ sample from `INTERIM_FOC_UQ_UD`.
    pub foc_uq_sample: i16,
    /// Pre-limit FOC UD sample from `INTERIM_FOC_UQ_UD`.
    pub foc_ud_sample: i16,
    /// Limited FOC UQ sample from `INTERIM_FOC_UQ_UD_LIMITED`.
    pub foc_uq_limited_sample: i16,
    /// Limited FOC UD sample from `INTERIM_FOC_UQ_UD_LIMITED`.
    pub foc_ud_limited_sample: i16,
    /// Park-transformed q-axis current sample from `INTERIM_FOC_IQ_ID`.
    pub foc_iq_sample: i16,
    /// Park-transformed d-axis current sample from `INTERIM_FOC_IQ_ID`.
    pub foc_id_sample: i16,
}

impl ImpedancePointObservation {
    /// Empty observation used before a point has been sampled.
    pub const EMPTY: Self = Self {
        status_flags_or: 0,
        openloop_phi_sample: 0,
        openloop_velocity_actual: 0,
        adc_iux_sample: 0,
        adc_iwy_sample: 0,
        foc_iux_sample: 0,
        foc_iwy_sample: 0,
        foc_uq_sample: 0,
        foc_ud_sample: 0,
        foc_uq_limited_sample: 0,
        foc_ud_limited_sample: 0,
        foc_iq_sample: 0,
        foc_id_sample: 0,
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
