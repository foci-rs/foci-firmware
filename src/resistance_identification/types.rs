//! Public data types for the resistance-identification engine.

/// Electrical axis sampled by the resistance-identification profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ElectricalAxis {
    /// Stable per-result index.
    pub index: u8,
    /// TMC4671 external electrical angle.
    pub phi_e_ext: u16,
    /// Human-facing phase label used by Klipper output.
    pub label: u8,
}

impl ElectricalAxis {
    /// Electrical-axis index 0: PHI_E_EXT=0, human label phase0.
    pub const PHASE0: Self = Self {
        index: 0,
        phi_e_ext: 0,
        label: 0,
    };

    /// Electrical-axis index 1: PHI_E_EXT=16384, human label phase90.
    pub const PHASE90: Self = Self {
        index: 1,
        phi_e_ext: 16_384,
        label: 90,
    };
}

/// One zero-subtracted resistance point in count space.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ResistancePoint {
    /// Requested UD_EXT count.
    pub requested_ud: i16,
    /// Mean limited UD count minus adjacent zero-window UD count.
    pub voltage_delta_count: i32,
    /// Mean physical phase current count minus adjacent zero-window count.
    pub current_delta_count: i32,
    /// True when selected for the production linear-region fit.
    pub selected: bool,
}

/// Warning bitfield returned by the engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ResistanceWarningFlags(pub u32);

impl ResistanceWarningFlags {
    pub const NONE: Self = Self(0);
    pub const SIGNED_CURRENT_ASYMMETRY: Self = Self(1 << 0);
    pub const THERMAL_DRIFT: Self = Self(1 << 1);
    pub const HIGH_RESIDUAL: Self = Self(1 << 2);
    pub const SCALE_METADATA_UNVALIDATED: Self = Self(1 << 3);
    pub const ENCODER_MOVED: Self = Self(1 << 4);
    pub const STATUS_FLAGS: Self = Self(1 << 5);

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }

    pub fn insert(&mut self, other: Self) {
        self.0 |= other.0;
    }
}

/// Signed anchor evidence for one electrical axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct SignedAnchorEvidence {
    pub positive_current_count: i32,
    pub negative_current_count: i32,
    pub signed_count_slope_milli: i32,
    pub asymmetry_permille: u16,
    pub repeated_positive_current_count: i32,
    pub drift_permille: u16,
}

/// Fitted result for one electrical axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ResistanceAxisResult {
    pub axis: ElectricalAxis,
    pub r_count_slope_milli: i32,
    pub slope_num: i64,
    pub slope_den: i64,
    pub intercept_count: i32,
    pub rmse_permille: u16,
    pub selected_mask: u16,
    pub excluded_point_mask: u16,
    pub selected_count: u8,
    pub signed_anchor: Option<SignedAnchorEvidence>,
    pub warning_flags: ResistanceWarningFlags,
}

/// Complete two-axis resistance run result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ResistanceRunResult {
    pub profile_version: u16,
    pub selected_r_count_slope_milli: i32,
    pub status_flags_or: u32,
    pub axis0: ResistanceAxisResult,
    pub axis1: ResistanceAxisResult,
    pub warning_flags: ResistanceWarningFlags,
}

/// Hard-failure reason for resistance identification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ResistanceError {
    WrongPowerStage,
    InsufficientLinearPoints,
    ThermalDrift,
    ElectricalAxisMismatch,
    HighResidual,
    BackendIo,
    InvalidMotorType,
    NotAdcReady,
    NotArmed,
    PeakCurrentExceeded,
    UnsupportedProfile,
    OutOfBoundsParameter,
    MeasurementEnvelopeUnsupported,
}

impl ResistanceError {
    /// Non-zero status code for the Klipper protocol message.
    ///
    /// Reuses [`crate::autotune::CommissionError`]'s stable numbering. A
    /// backend I/O failure maps to the shared SPI code, and every resistance
    /// precondition, quality-gate, or configuration failure gets its own
    /// resistance-specific code so diagnostics can report the actionable cause.
    pub fn status_code(self) -> u8 {
        crate::autotune::CommissionError::from(self).status_code()
    }
}

impl From<ResistanceError> for crate::autotune::CommissionError {
    fn from(value: ResistanceError) -> Self {
        match value {
            ResistanceError::BackendIo => Self::Spi,
            ResistanceError::NotAdcReady => Self::ResistanceAdcNotReady,
            ResistanceError::NotArmed => Self::ResistanceNotArmed,
            ResistanceError::WrongPowerStage => Self::ResistanceWrongPowerStage,
            ResistanceError::InsufficientLinearPoints => Self::ResistanceInsufficientLinearPoints,
            ResistanceError::ThermalDrift => Self::ResistanceThermalDrift,
            ResistanceError::ElectricalAxisMismatch => Self::ResistanceElectricalAxisMismatch,
            ResistanceError::HighResidual => Self::ResistanceHighResidual,
            ResistanceError::InvalidMotorType => Self::ResistanceInvalidMotorType,
            ResistanceError::PeakCurrentExceeded => Self::ResistancePeakCurrentExceeded,
            ResistanceError::UnsupportedProfile => Self::ResistanceUnsupportedProfile,
            ResistanceError::OutOfBoundsParameter => Self::ResistanceParameterOutOfBounds,
            ResistanceError::MeasurementEnvelopeUnsupported => {
                Self::ResistanceMeasurementEnvelopeUnsupported
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::ResistanceError;

    #[test]
    fn backend_io_reuses_commissioning_spi_code() {
        assert_eq!(
            ResistanceError::BackendIo.status_code(),
            crate::autotune::CommissionError::Spi.status_code()
        );
    }

    #[test]
    fn resistance_errors_get_distinct_actionable_status_codes() {
        let expected = [
            (ResistanceError::NotAdcReady, 20),
            (ResistanceError::NotArmed, 21),
            (ResistanceError::WrongPowerStage, 22),
            (ResistanceError::InsufficientLinearPoints, 23),
            (ResistanceError::ThermalDrift, 24),
            (ResistanceError::ElectricalAxisMismatch, 25),
            (ResistanceError::HighResidual, 26),
            (ResistanceError::InvalidMotorType, 27),
            (ResistanceError::PeakCurrentExceeded, 28),
            (ResistanceError::UnsupportedProfile, 29),
            (ResistanceError::OutOfBoundsParameter, 30),
            (ResistanceError::MeasurementEnvelopeUnsupported, 31),
        ];
        let mut seen = [false; 256];
        for (error, code) in expected {
            assert_eq!(error.status_code(), code, "{error:?}");
            assert!(!seen[code as usize], "duplicate status code {code}");
            seen[code as usize] = true;
        }
    }
}
