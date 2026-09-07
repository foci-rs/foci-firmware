//! Calibration-sequence error type shared between the board binaries
//! and the foci-firmware commissioning engine.

/// Errors that can occur during the TMC4671 calibration sequence.
#[derive(Debug, defmt::Format)]
pub enum CalibrationError {
    /// SPI communication or register verification failed. Code 1.
    SpiError,
    /// Sentinel register mismatch at arm time. Channel must be
    /// re-commissioned. Code 2.
    ChipResetDetected,
    /// Motor already enabled (calibration not needed).
    AlreadyEnabled,
    /// Internal firmware error (queue full, channel not initialized,
    /// etc.).
    InternalError,
    /// Required configuration (e.g., `tmc_set_motion_scale`) has not been
    /// issued by the host before calibration. Most commonly this means
    /// STEP_WIDTH has not been computed, so the TMC4671 hardware step
    /// input would not advance the position target — the motor would
    /// calibrate successfully but silently not respond to step pulses.
    ConfigFault,
    /// Encoder presence, movement, or alignment validation failed.
    EncoderFault,
    /// Closed-loop entry position hold was unstable (runaway or excess
    /// drift) after exhausting the bounded-drift retry budget. Code 9.
    ClosedLoopActivationUnstable,
    /// Operator-requested cancel reached GPIO-first safe-state gracefully
    /// during calibration; the MCU stays running. Code 10.
    Cancelled,
}

impl CalibrationError {
    /// Map to a numeric status code for `foci_calibrate_result`.
    pub fn status_code(&self) -> u8 {
        match self {
            Self::SpiError => 1,
            Self::ChipResetDetected => 2,
            // Code 4 was PidFault (removed with tmc_enter_closedloop).
            Self::AlreadyEnabled => 5,
            Self::InternalError => 6,
            Self::ConfigFault => 7,
            Self::EncoderFault => 8,
            Self::ClosedLoopActivationUnstable => 9,
            Self::Cancelled => 10,
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
