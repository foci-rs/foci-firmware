//! Shared active TMC4671 diagnostic runners.
//!
//! The runners own chip-level diagnostic sequencing and report construction.
//! Board crates keep channel selection, reply emission, shutdown glue, RTIC
//! timing, and hardware validation.

mod current_step;
mod helpers;
mod impedance;
mod replies;
mod reports;
mod resistance;
mod torque_sample;
mod voltage_step;

#[cfg(test)]
mod tests;

pub use current_step::run_current_step_request;
pub use helpers::{current_step_encoder_delta, pack_voltage_ext};
pub use impedance::run_impedance_test_request;
pub use replies::{DiagnosticReplySink, emit_diagnostic_report};
pub use reports::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, CurrentStepReport,
    CurrentTorqueSampleReport, IMPEDANCE_STATUS_CURRENT_LIMIT, IMPEDANCE_STATUS_ENCODER_MOVED,
    IMPEDANCE_STATUS_HIGH_RESIDUAL, IMPEDANCE_STATUS_INSUFFICIENT_VOLTAGE,
    IMPEDANCE_STATUS_LOW_SIGNAL, IMPEDANCE_STATUS_POOR_COVERAGE,
    IMPEDANCE_STATUS_PROFILE_READBACK, IMPEDANCE_STATUS_REPEATABILITY,
    IMPEDANCE_STATUS_SCALAR_2F_MISMATCH, IMPEDANCE_STATUS_UNSUPPORTED_MOTOR_TYPE,
    IMPEDANCE_STATUS_UNSUPPORTED_PROFILE, ImpedanceFitReport, ImpedanceProfileReport,
    ImpedanceRunReport, ResistanceAxisReport, ResistanceProfileReport, ResistanceRunReport,
    VoltageStepReport, empty_current_torque_sample_report, empty_impedance_run_report,
    empty_resistance_run_report, empty_voltage_step_report,
};
pub use resistance::run_resistance_test_request;
pub use torque_sample::{
    run_current_torque_sample_request, run_position_torque_offset_sample_request,
};
pub use voltage_step::run_voltage_step_request;

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

    /// Delay by `us` microseconds.
    async fn delay_us(&mut self, us: u32) {
        self.delay_ms(u64::from(us).div_ceil(1_000)).await;
    }
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
    /// Board-specific TMC4671 hardware configuration, used by the
    /// resistance-test diagnostic to build the board's resistance profile.
    pub tmc_config: &'static crate::config::TmcConfig,
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
    /// Shared resistance-identification diagnostic. Axis reports are absent
    /// when the run failed before producing per-axis fit results (entry
    /// pre-check rejection or a mid-run backend/quality-gate failure).
    ResistanceTest {
        /// Board/profile metadata, always present.
        profile: ResistanceProfileReport,
        /// Per-run status, warnings, and readbacks.
        run: ResistanceRunReport,
        /// Per-axis fit results, present only on a successful run.
        axes: Option<[ResistanceAxisReport; 2]>,
    },
    /// Shared impedance-identification diagnostic. Fit reports are absent
    /// when the run failed before producing per-point fit results.
    ImpedanceTest {
        /// Board/profile metadata, always present.
        profile: ImpedanceProfileReport,
        /// Per-run status, warnings, and aggregate selected axes.
        run: ImpedanceRunReport,
        /// Per-point fit results, present only on a successful run.
        fits: Option<[ImpedanceFitReport; crate::impedance_identification::IMPEDANCE_PROFILE_POINTS]>,
        /// Normalized detail level from the command request.
        detail: u8,
    },
}

/// Result of shared diagnostic request dispatch.
#[allow(clippy::large_enum_variant)]
pub enum DiagnosticOutcome {
    /// The request was an active diagnostic and produced a report.
    Handled(DiagnosticReport),
    /// The request was not one of the active diagnostic request kinds.
    NotDiagnostic(crate::tmc::TmcRequest),
}

/// Dispatch one active diagnostic request through the shared runner set.
pub async fn handle_diagnostic_request<B, D>(
    req: crate::tmc::TmcRequest,
    ctx: DiagnosticContext<'_, B, D>,
) -> DiagnosticOutcome
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    match &req.kind {
        crate::tmc::TmcRequestKind::CurrentStepTest { .. }
        | crate::tmc::TmcRequestKind::CurrentVectorStepTest { .. } => {
            current_step::run_current_step_request(req, ctx).await
        }
        crate::tmc::TmcRequestKind::CurrentTorqueSampleTest { .. } => {
            torque_sample::run_current_torque_sample_request(req, ctx).await
        }
        crate::tmc::TmcRequestKind::PositionTorqueOffsetSampleTest { .. } => {
            torque_sample::run_position_torque_offset_sample_request(req, ctx).await
        }
        crate::tmc::TmcRequestKind::VoltageStepTest { .. } => {
            voltage_step::run_voltage_step_request(req, ctx).await
        }
        crate::tmc::TmcRequestKind::ResistanceTest { .. } => {
            resistance::run_resistance_test_request(req, ctx).await
        }
        crate::tmc::TmcRequestKind::ImpedanceTest { .. } => {
            impedance::run_impedance_test_request(req, ctx).await
        }
        _ => DiagnosticOutcome::NotDiagnostic(req),
    }
}
