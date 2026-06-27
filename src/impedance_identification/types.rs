//! Public data types for the impedance-identification engine.

/// Fixed number of angle windows in the impedance profile.
pub const IMPEDANCE_PROFILE_POINTS: usize = 10;

/// Zero-angle window used only to let the rotor return after the sweep.
pub const IMPEDANCE_RETURN_ZERO_SETTLE_INDEX: usize = 7;

/// First settled post-sweep zero-angle measurement window.
pub const IMPEDANCE_POST_SWEEP_ZERO_INDEX: usize = 8;

/// Second settled post-sweep zero-angle window used for repeatability.
pub const IMPEDANCE_STABILITY_ZERO_INDEX: usize = 9;

/// Fixed carrier sample count exposed by the standard host-visible profile.
pub const IMPEDANCE_SAMPLE_COUNT: usize = 160;

/// One held-angle rotating-carrier probe window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedancePoint {
    /// Carrier probe angle relative to the fixed held field, in 16-bit turn counts.
    pub angle_electrical_counts: u16,
    /// Absolute carrier-vector magnitude.
    pub ud_abs: u16,
}

/// One reduced held-angle observation produced by the rotating-carrier lock-in stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceAngleObservation {
    /// Carrier probe angle relative to the fixed held field, in 16-bit turn counts.
    pub angle_electrical_counts: u16,
    /// Absolute carrier-vector magnitude used during the observation.
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

impl ImpedanceAngleObservation {
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

/// Static baseline observation used by the rotating-carrier lock-in path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceBaselineObservation {
    /// Baseline d-axis current count.
    pub d_count: i16,
    /// Baseline q-axis current count.
    pub q_count: i16,
    /// Baseline ADC channel 0 count.
    pub adc_ch0_count: i16,
    /// Baseline ADC channel 1 count.
    pub adc_ch1_count: i16,
}

/// Pure held-angle rotating-carrier impedance fit result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceFitSummary {
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

/// TMC4671 filter state observed around rotating-carrier lock-in sampling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceFilterState {
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
    FilterState,
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
