//! Shared active TMC4671 diagnostic runners.
//!
//! The runners own chip-level diagnostic sequencing and report construction.
//! Board crates keep channel selection, reply emission, shutdown glue, RTIC
//! timing, and hardware validation.

mod current_step;
mod helpers;
mod replies;
mod reports;
mod resistance;
mod torque_sample;
mod voltage_step;

#[cfg(test)]
mod tests;

pub use current_step::run_current_step_request;
pub use helpers::{current_step_encoder_delta, pack_voltage_ext};
pub use replies::{DiagnosticReplySink, emit_diagnostic_report};
pub use reports::{
    AdcResidualReport, CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI,
    ClosedLoopEntryReport, CommissionTimingReport, CurrentLoopFiltersReport, CurrentLoopHoldReport,
    CurrentLoopRunReport, CurrentStepReport, CurrentTorqueSampleReport,
    CurrentValidationAxisReport, CurrentValidationEnvelopeReport,
    CurrentValidationSettledSampleReport, EncoderAlignmentReport, InductanceEstimateReport,
    InductanceFrameReport, InductanceRunReport, ResistanceAxisReport, ResistanceProfileReport,
    ResistanceRunReport, VoltageStepReport, empty_current_torque_sample_report,
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

    /// Monotonic microsecond timestamp from the board clock.
    ///
    /// Diagnostics use this to stamp samples with the real time each measurement
    /// was taken, including register-read and gate-check overhead between
    /// requested delays. Boards back it with their microsecond monotonic.
    fn now_us(&self) -> u64;
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
}

/// Result of shared diagnostic request dispatch.
#[allow(clippy::large_enum_variant)]
pub enum DiagnosticOutcome {
    /// The request was an active diagnostic and produced a report.
    Handled(DiagnosticReport),
    /// The request was not one of the active diagnostic request kinds.
    NotDiagnostic(crate::tmc::TmcRequest),
}

/// Build the established not-ready reply for a diagnostic rejected by commissioning.
pub fn busy_diagnostic_report(
    req: crate::tmc::TmcRequest,
    tmc_config: &crate::config::TmcConfig,
) -> DiagnosticOutcome {
    use crate::tmc::{CurrentStepAxis, TmcRequestKind};

    let report = match req.kind {
        TmcRequestKind::CurrentStepTest {
            oid,
            axis,
            target,
            duration_ms,
            voltage_limit,
        } => {
            let flux_target = if matches!(axis, CurrentStepAxis::Flux) {
                target
            } else {
                0
            };
            DiagnosticReport::CurrentStep {
                vector_reply: false,
                report: current_step::empty_current_step_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    target,
                    flux_target,
                    duration_ms,
                    voltage_limit,
                ),
            }
        }
        TmcRequestKind::CurrentVectorStepTest {
            oid,
            torque_target,
            flux_target,
            duration_ms,
            voltage_limit,
        } => DiagnosticReport::CurrentStep {
            vector_reply: true,
            report: current_step::empty_current_step_report(
                oid,
                CURRENT_STEP_STATUS_NOT_READY,
                torque_target,
                flux_target,
                duration_ms,
                voltage_limit,
            ),
        },
        TmcRequestKind::CurrentTorqueSampleTest {
            oid,
            target,
            flux_target,
            sample_delay_ms,
            voltage_limit,
        } => DiagnosticReport::CurrentTorqueSample {
            report: empty_current_torque_sample_report(
                oid,
                CURRENT_STEP_STATUS_NOT_READY,
                target,
                flux_target,
                sample_delay_ms,
                voltage_limit,
            ),
            emit_detail: false,
        },
        TmcRequestKind::PositionTorqueOffsetSampleTest {
            oid,
            target,
            sample_delay_ms,
            voltage_limit,
        } => DiagnosticReport::CurrentTorqueSample {
            report: empty_current_torque_sample_report(
                oid,
                CURRENT_STEP_STATUS_NOT_READY,
                target,
                0,
                sample_delay_ms,
                voltage_limit,
            ),
            emit_detail: false,
        },
        TmcRequestKind::VoltageStepTest {
            oid,
            uq_ext,
            ud_ext,
            sample_delay_ms,
        } => DiagnosticReport::VoltageStep(empty_voltage_step_report(
            oid,
            CURRENT_STEP_STATUS_NOT_READY,
            uq_ext,
            ud_ext,
            sample_delay_ms,
        )),
        TmcRequestKind::ResistanceTest { oid, .. } => {
            let profile =
                tmc_config.resistance_profile(tmc_config.resistance_scale_metadata_validated);
            DiagnosticReport::ResistanceTest {
                profile: resistance::profile_report(oid, &profile),
                run: empty_resistance_run_report(oid, CURRENT_STEP_STATUS_NOT_READY),
                axes: None,
            }
        }
        _ => return DiagnosticOutcome::NotDiagnostic(req),
    };
    DiagnosticOutcome::Handled(report)
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
        _ => DiagnosticOutcome::NotDiagnostic(req),
    }
}
