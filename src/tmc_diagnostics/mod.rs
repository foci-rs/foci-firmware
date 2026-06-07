//! Shared active TMC4671 diagnostic runners.
//!
//! The runners own chip-level diagnostic sequencing and report construction.
//! Board crates keep channel selection, reply emission, shutdown glue, RTIC
//! timing, and hardware validation.

mod current_step;
mod helpers;
mod reports;
mod torque_sample;

#[cfg(test)]
mod tests;

pub use current_step::run_current_step_request;
pub use helpers::{current_step_encoder_delta, pack_voltage_ext};
pub use reports::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, CurrentStepReport,
    CurrentTorqueSampleReport, VoltageStepReport, empty_current_torque_sample_report,
    empty_voltage_step_report,
};
pub use torque_sample::{
    run_current_torque_sample_request, run_position_torque_offset_sample_request,
};

use foci_tmc4671::spi::Tmc4671Spi;

/// Board-supplied readiness snapshot for one diagnostic request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticReadiness {
    /// Whether board-global homing is active.
    pub homing_active: bool,
    /// Whether this request channel has active step pulses.
    pub stepper_active: bool,
    /// Saved operational PIDOUT_UQ_UD_LIMITS value for this request channel.
    pub saved_voltage_limit: u32,
}

/// Delay abstraction used by shared diagnostic runners.
#[allow(async_fn_in_trait)]
pub trait DiagnosticDelay {
    /// Delay by `ms` milliseconds.
    async fn delay_ms(&mut self, ms: u64);
}

/// Mutable resources borrowed by one active diagnostic request.
pub struct DiagnosticContext<'a, B, D> {
    /// TMC4671 SPI transport for the selected request channel.
    pub tmc: &'a mut Tmc4671Spi<B>,
    /// Shared TMC runtime for the selected request channel.
    pub runtime: &'a mut crate::tmc_control::TmcControlRuntime,
    /// Board delay provider.
    pub delay: &'a mut D,
    /// Board readiness snapshot for the selected request channel.
    pub readiness: DiagnosticReadiness,
}

/// Report produced by a handled active diagnostic request.
#[allow(clippy::large_enum_variant)]
pub enum DiagnosticReport {
    /// Current-step or current-vector-step diagnostic.
    CurrentStep {
        /// Emit the vector reply if true; emit scalar current-step reply if false.
        vector_reply: bool,
        /// Captured current-step report fields.
        report: CurrentStepReport,
    },
    /// Current-torque-sample or position-offset sample diagnostic.
    CurrentTorqueSample {
        /// Captured torque-sample report fields.
        report: CurrentTorqueSampleReport,
        /// Emit the detail reply before the summary reply.
        emit_detail: bool,
    },
    /// Open-loop voltage-step diagnostic.
    VoltageStep(VoltageStepReport),
}

/// Result of shared diagnostic request dispatch.
#[allow(clippy::large_enum_variant)]
pub enum DiagnosticOutcome {
    /// The request was an active diagnostic and produced a report.
    Handled(DiagnosticReport),
    /// The request was not one of the active diagnostic request kinds.
    NotDiagnostic(crate::tmc::TmcRequest),
}
