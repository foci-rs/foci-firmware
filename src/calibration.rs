//! Calibration-sequence error type.

/// Calibration failure; `status_code` gives the wire code.
#[derive(Debug, defmt::Format)]
pub enum CalibrationError {
    /// SPI communication or register verification failed. Code 1.
    SpiError,
    /// Sentinel mismatch at arm time; channel must be re-commissioned. Code 2.
    ChipResetDetected,
    AlreadyEnabled,
    InternalError,
    /// Host has not issued required configuration (e.g. `tmc_set_motion_scale`).
    ConfigFault,
    EncoderFault,
    /// Closed-loop entry position hold unstable after the drift retry budget. Code 9.
    ClosedLoopActivationUnstable,
    /// Operator cancel reached safe-state; MCU stays running. Code 10.
    Cancelled,
    /// ADC calibration produced offsets outside the expected range. Code 11.
    AdcFault,
    /// Coil connectivity check failed (open circuit or short). Code 12.
    CoilFault,
    /// Phase wiring check: current appeared on the wrong ADC channel. Code 13.
    PhaseFault,
    /// No current was detected in the motor windings during the coil check. Code 14.
    NoCurrent,
    /// Retained gain storage was corrupt at closed-loop entry. Code 15.
    CurrentValidationFailed,
    /// A calibration state returned an invalid or unrepresentable deadline. Code 16.
    InvalidSchedule,
    /// Current-loop hold validation failed at closed-loop activation. Code 17.
    CurrentHoldFailed,
    /// Latched fault or asserted STATUS pin refused a `STATUS_FLAGS` clear or enable. Code 18.
    FaultInterlock,
    /// Hardware shutdown was requested (M112 / e-stop) while calibrating. Code 19.
    Shutdown,
    /// Last safe-state restore had write failures; arming refuses until one passes cleanly. Code 20.
    SafeStateIncomplete,
}

impl CalibrationError {
    /// Numeric status code for `foci_calibrate_result`.
    pub fn status_code(&self) -> u8 {
        match self {
            Self::SpiError => 1,
            Self::ChipResetDetected => 2,
            Self::AlreadyEnabled => 5,
            Self::InternalError => 6,
            Self::ConfigFault => 7,
            Self::EncoderFault => 8,
            Self::ClosedLoopActivationUnstable => 9,
            Self::Cancelled => 10,
            Self::AdcFault => 11,
            Self::CoilFault => 12,
            Self::PhaseFault => 13,
            Self::NoCurrent => 14,
            Self::CurrentValidationFailed => 15,
            Self::InvalidSchedule => 16,
            Self::CurrentHoldFailed => 17,
            Self::FaultInterlock => 18,
            Self::Shutdown => 19,
            Self::SafeStateIncomplete => 20,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_chip_reset_detected_has_status_code_2() {
        assert_eq!(CalibrationError::ChipResetDetected.status_code(), 2);
    }

    #[test]
    fn calibration_closed_loop_activation_unstable_has_status_code_9() {
        assert_eq!(
            CalibrationError::ClosedLoopActivationUnstable.status_code(),
            9
        );
    }

    #[test]
    fn calibration_cancelled_has_status_code_10() {
        assert_eq!(CalibrationError::Cancelled.status_code(), 10);
    }
}
