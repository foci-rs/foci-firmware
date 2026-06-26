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

/// One reduced fixed-angle observation produced by the pulsating lock-in stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PulsatingAngleObservation {
    /// Electrical angle for this fixed-angle observation, in 16-bit turn counts.
    pub angle_electrical_counts: u16,
    /// Absolute UD magnitude used during the observation.
    pub ud_abs: u16,
    /// Lock-in inductive response normalized to count per UD millivolt-equivalent.
    pub inductive_response_count_per_ud_milli: u32,
    /// Lock-in in-phase response normalized to count per UD millivolt-equivalent.
    pub in_phase_response_count_per_ud_milli: u32,
    /// Secondary ADC lock-in response normalized to count per UD millivolt-equivalent.
    pub secondary_adc_response_count_per_ud_milli: u32,
    /// Baseline d-axis current count sampled before injection.
    pub baseline_d_count: i16,
    /// Baseline q-axis current count sampled before injection.
    pub baseline_q_count: i16,
    /// Baseline ADC channel 0 count sampled before injection.
    pub baseline_adc_ch0_count: i16,
    /// Baseline ADC channel 1 count sampled before injection.
    pub baseline_adc_ch1_count: i16,
    /// Fit residual for this angle, in permille.
    pub residual_permille: u16,
    /// Lock-in sample coverage for this angle, in permille.
    pub coverage_permille: u16,
    /// Encoder count before the observation window.
    pub encoder_before: u32,
    /// Encoder count after the observation window.
    pub encoder_after: u32,
    /// Minimum encoder count observed during the window.
    pub encoder_min: u32,
    /// Maximum encoder count observed during the window.
    pub encoder_max: u32,
    /// Peak encoder motion observed during the window.
    pub encoder_peak_counts: u16,
    /// Whether this observation should participate in the fit.
    pub valid: bool,
    /// Warning evidence collected while sampling this angle.
    pub warning_flags: ImpedanceWarningFlags,
}

impl PulsatingAngleObservation {
    /// Empty observation used before an angle has been sampled.
    pub const EMPTY: Self = Self {
        angle_electrical_counts: 0,
        ud_abs: 0,
        inductive_response_count_per_ud_milli: 0,
        in_phase_response_count_per_ud_milli: 0,
        secondary_adc_response_count_per_ud_milli: 0,
        baseline_d_count: 0,
        baseline_q_count: 0,
        baseline_adc_ch0_count: 0,
        baseline_adc_ch1_count: 0,
        residual_permille: 0,
        coverage_permille: 0,
        encoder_before: 0,
        encoder_after: 0,
        encoder_min: 0,
        encoder_max: 0,
        encoder_peak_counts: 0,
        valid: false,
        warning_flags: ImpedanceWarningFlags::NONE,
    };
}

/// Static baseline observation used by the pulsating lock-in path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PulsatingBaselineObservation {
    /// Baseline d-axis current count.
    pub d_count: i16,
    /// Baseline q-axis current count.
    pub q_count: i16,
    /// Baseline ADC channel 0 count.
    pub adc_ch0_count: i16,
    /// Baseline ADC channel 1 count.
    pub adc_ch1_count: i16,
}

/// Pure fixed-angle pulsating impedance fit result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PulsatingFitSummary {
    /// Mean response normalized to count per UD millivolt-equivalent.
    pub mean_response_count_per_ud_milli: u32,
    /// Maximum fitted response axis normalized to count per UD millivolt-equivalent.
    pub axis_max_count_per_ud_milli: u32,
    /// Minimum fitted response axis normalized to count per UD millivolt-equivalent.
    pub axis_min_count_per_ud_milli: u32,
    /// Fitted saliency relative to the mean response, in permille.
    pub saliency_permille: u16,
    /// Fitted saliency axis angle, in 16-bit electrical turn counts.
    pub axis_angle_electrical_counts: u16,
    /// Half-turn folded deviation from the expected zero electrical axis.
    pub axis_deviation_counts: u16,
    /// Whether saliency is strong enough to determine the axis.
    pub saliency_determinable: bool,
    /// Angular coverage of valid observations, in permille.
    pub angle_coverage_permille: u16,
    /// Duplicate zero-angle repeatability error, in permille.
    pub repeatability_permille: u16,
    /// Warning flags accumulated during sampling and fitting.
    pub warning_flags: ImpedanceWarningFlags,
}

/// TMC4671 filter state observed around pulsating lock-in sampling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PulsatingFilterState {
    /// Torque filter enable state before sampling.
    pub torque_enable_before: u8,
    /// Flux filter enable state before sampling.
    pub flux_enable_before: u8,
    /// Torque filter enable state during sampling.
    pub torque_enable_during: u8,
    /// Flux filter enable state during sampling.
    pub flux_enable_during: u8,
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
    pub const AXIS_UNDETERMINED: Self = Self(1 << 7);
    pub const ANGLE_LOCAL_MOTION: Self = Self(1 << 8);
    pub const FILTER_STATE: Self = Self(1 << 9);

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
    AxisDeviation,
    Scalar2fMismatch,
    HighResidual,
    RepeatabilityMismatch,
    BackendIo,
    RestoreFailed,
}
