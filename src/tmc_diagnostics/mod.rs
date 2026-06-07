//! Shared active TMC4671 diagnostic runners.
//!
//! The runners own chip-level diagnostic sequencing and report construction.
//! Board crates keep channel selection, reply emission, shutdown glue, RTIC
//! timing, and hardware validation.

mod helpers;
mod reports;

#[cfg(test)]
mod tests;

pub use helpers::{current_step_encoder_delta, pack_voltage_ext};
pub use reports::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, CurrentStepReport,
    CurrentTorqueSampleReport, VoltageStepReport, empty_current_torque_sample_report,
    empty_voltage_step_report,
};
