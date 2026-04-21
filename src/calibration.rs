//! Calibration-sequence error type shared between the board binaries
//! and the foci-firmware commissioning engine.

/// Errors that can occur during the TMC4671 calibration sequence.
#[derive(Debug, defmt::Format)]
pub enum CalibrationError {
    /// SPI communication or register verification failed.
    SpiError,
    /// Motor already enabled (calibration not needed).
    AlreadyEnabled,
    /// Internal firmware error (queue full, channel not initialized,
    /// etc.).
    InternalError,
    /// Required configuration (e.g., `tmc_set_encoder`) has not been
    /// issued by the host before calibration. Most commonly this means
    /// STEP_WIDTH has not been computed, so the TMC4671 hardware step
    /// input would not advance the position target — the motor would
    /// calibrate successfully but silently not respond to step pulses.
    ConfigFault,
}

impl CalibrationError {
    /// Map to a numeric status code for `foci_calibrate_result`.
    pub fn status_code(&self) -> u8 {
        match self {
            Self::SpiError => 1,
            // Code 4 was PidFault (removed with tmc_enter_closedloop).
            Self::AlreadyEnabled => 5,
            Self::InternalError => 6,
            Self::ConfigFault => 7,
        }
    }
}
