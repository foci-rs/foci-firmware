//! `FOCI_RESISTANCE_TEST` diagnostic: runs the shared resistance-identification
//! engine over an already-armed motor without touching commissioning state.

use core::sync::atomic::Ordering::Relaxed;

use foci_tmc4671::registers::Register;

use crate::autotune::Permille;
use crate::resistance_identification::{
    ResistanceAxisResult, ResistanceBackend, ResistanceBackendError, ResistanceCurrentEvidence,
    ResistanceError, ResistanceProfile, ResistanceRunMode, ResistanceRunResult,
    run_resistance_identification,
};
use crate::tmc::{TmcRequest, TmcRequestKind};

use super::helpers::checked_delay_ms;
use super::{
    CURRENT_STEP_STATUS_NOT_READY, DiagnosticContext, DiagnosticDelay, DiagnosticOutcome,
    DiagnosticReport, ResistanceAxisReport, ResistanceProfileReport, ResistanceRunReport,
    empty_resistance_run_report,
};

pub(super) struct DiagnosticResistanceBackend<'ctx, 'a, B, D> {
    ctx: &'ctx mut DiagnosticContext<'a, B, D>,
    pending_liveness_injection: bool,
    host_liveness_aborted: bool,
}

impl<'ctx, 'a, B, D> DiagnosticResistanceBackend<'ctx, 'a, B, D> {
    pub(super) fn new(ctx: &'ctx mut DiagnosticContext<'a, B, D>) -> Self {
        Self {
            ctx,
            pending_liveness_injection: false,
            host_liveness_aborted: false,
        }
    }

    pub(super) fn host_liveness_aborted(&self) -> bool {
        self.host_liveness_aborted
    }
}

impl<B, D> ResistanceBackend for DiagnosticResistanceBackend<'_, '_, B, D>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    async fn read_register(&mut self, addr: u8) -> Result<u32, ResistanceBackendError> {
        if core::mem::take(&mut self.pending_liveness_injection) {
            return Err(ResistanceBackendError::Spi);
        }
        self.ctx
            .tmc
            .read_register(Register::new(addr))
            .await
            .map_err(|_| ResistanceBackendError::Spi)
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ResistanceBackendError> {
        if core::mem::take(&mut self.pending_liveness_injection) {
            return Err(ResistanceBackendError::Spi);
        }
        self.ctx
            .tmc
            .write_register(Register::new(addr), value)
            .await
            .map_err(|_| ResistanceBackendError::Spi)
    }

    async fn delay_ms(&mut self, ms: u64) {
        if checked_delay_ms(
            self.ctx.delay,
            reborrow_liveness_check(&mut self.ctx.host_liveness_expired),
            ms,
        )
        .await
        .is_err()
        {
            self.pending_liveness_injection = true;
            self.host_liveness_aborted = true;
        }
    }

    fn elapsed_us(&self) -> u64 {
        self.ctx.delay.now_us()
    }

    fn set_enable_pin(&mut self, _enabled: bool) {}

    fn interlock_tripped(&self) -> bool {
        crate::state::tmc::tmc_channel(self.ctx.runtime.channel())
            .is_none_or(crate::tmc_control::interlock::fault_interlock_tripped)
    }

    fn trip_power_stage(&mut self) {
        self.ctx.power_stage.trip_power_stage();
        self.ctx.runtime.contain_diagnostic_trip();
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        self.ctx.runtime.is_motor_enabled() && self.ctx.runtime.is_calibrated()
    }

    fn profile(&self) -> ResistanceProfile {
        self.ctx
            .tmc_config
            .resistance_profile(self.ctx.tmc_config.resistance_scale_metadata_validated)
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        crate::state::tmc::tmc_channel(self.ctx.runtime.channel())
            .map(|state| state.encoder_ppr.load(Relaxed).saturating_mul(4))
            .unwrap_or(0)
    }

    fn feedback_config(&self) -> crate::config::FeedbackConfig {
        self.ctx.feedback_config
    }

    fn adc_offsets_ready(&self) -> bool {
        crate::state::tmc::tmc_channel(self.ctx.runtime.channel())
            .is_some_and(|state| state.motor_sentinel.load(Relaxed) != 0)
    }
}

// `Option::as_deref_mut()` reborrowed inline at the `checked_delay_ms` call
// site, across an intervening `.await` in the same async fn, hits a rustc
// borrow-checker limitation with `&mut dyn Trait` reborrows spanning
// suspension points (same class already worked around in
// `current_step.rs`/`torque_sample.rs`/`voltage_step.rs`). An explicit,
// non-generic reborrow function sidesteps it.
fn reborrow_liveness_check<'a>(
    check: &'a mut Option<&mut dyn FnMut() -> bool>,
) -> Option<&'a mut dyn FnMut() -> bool> {
    match check {
        Some(f) => Some(&mut **f),
        None => None,
    }
}

pub(super) fn profile_report(oid: u8, profile: &ResistanceProfile) -> ResistanceProfileReport {
    ResistanceProfileReport {
        oid,
        pwm_maxcnt: profile.pwm_maxcnt,
        bbm_h: profile.bbm_h,
        bbm_l: profile.bbm_l,
        dsadc_mdec_a: profile.dsadc_mdec_a,
        dsadc_mdec_b: profile.dsadc_mdec_b,
        linear_current_threshold_count: profile.linear_current_threshold_count,
        encoder_move_warn_counts: profile.encoder_move_warn_counts,
        status_flags_warn_mask: profile.status_flags_warn_mask,
        scale_metadata_validated: profile.scale_metadata_validated,
    }
}

fn axis_report(oid: u8, axis: &ResistanceAxisResult) -> ResistanceAxisReport {
    let (signed_count_slope_milli, signed_asymmetry_permille, drift_permille) = axis
        .signed_anchor
        .map(|anchor| {
            (
                anchor.signed_count_slope_milli,
                anchor.asymmetry_permille,
                anchor.drift_permille,
            )
        })
        .unwrap_or((0, Permille::ZERO, Permille::ZERO));

    ResistanceAxisReport {
        oid,
        electrical_axis: axis.axis.index,
        phi_e_ext: axis.axis.phi_e_ext,
        r_count_slope_milli: axis.r_count_slope_milli,
        intercept_count: axis.intercept_count,
        rmse_permille: axis.rmse_permille.raw(),
        selected_mask: axis.selected_mask,
        excluded_point_mask: axis.excluded_point_mask,
        selected_count: axis.selected_count,
        signed_count_slope_milli,
        signed_asymmetry_permille: signed_asymmetry_permille.raw(),
        drift_permille: drift_permille.raw(),
        warning_flags: axis.warning_flags.0,
    }
}

fn success_run_report(
    oid: u8,
    profile: &ResistanceProfile,
    result: &ResistanceRunResult,
) -> ResistanceRunReport {
    let pwm_maxcnt_readback = profile.pwm_maxcnt;
    let bbm_readback = (u16::from(profile.bbm_h) << 8) | u16::from(profile.bbm_l);
    let dsadc_mdec_readback =
        (u32::from(profile.dsadc_mdec_b) << 16) | u32::from(profile.dsadc_mdec_a);

    ResistanceRunReport {
        oid,
        status: 0,
        selected_r_count_slope_milli: result.selected_r_count_slope_milli,
        gain_path_count_slope_milli: 0,
        warning_flags: result.warning_flags.0,
        status_flags_or: result.status_flags_or,
        peak_abs_current_count: result.current.peak_abs_current_count(),
        max_abs_steady_mean_current_count: result.current.max_abs_steady_mean_current_count(),
        current_ceiling_count: result.current.current_ceiling_count(),
        power_stage_tripped: result.current.power_stage_tripped(),
        pwm_maxcnt_readback,
        bbm_readback,
        dsadc_mdec_readback,
        pwm_sv_chop_readback: profile.pwm_sv_chop,
    }
}

pub(super) fn failed_run_report(
    oid: u8,
    status: u8,
    current: ResistanceCurrentEvidence,
) -> ResistanceRunReport {
    ResistanceRunReport {
        peak_abs_current_count: current.peak_abs_current_count(),
        max_abs_steady_mean_current_count: current.max_abs_steady_mean_current_count(),
        current_ceiling_count: current.current_ceiling_count(),
        power_stage_tripped: current.power_stage_tripped(),
        ..empty_resistance_run_report(oid, status)
    }
}

pub(super) fn terminal_status(error: ResistanceError, host_liveness_aborted: bool) -> u8 {
    if host_liveness_aborted {
        crate::autotune::CommissionError::HostLivenessExpired.status_code()
    } else {
        error.status_code()
    }
}

fn handled(
    profile: ResistanceProfileReport,
    run: ResistanceRunReport,
    axes: Option<[ResistanceAxisReport; 2]>,
) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { profile, run, axes })
}

/// Run a resistance-identification diagnostic request.
///
/// Runs the shared engine in [`ResistanceRunMode::ExternallyArmed`] over an
/// already armed motor. Rejects with the not-ready status when
/// `readiness.homing_active` or `readiness.stepper_active` is set.
pub async fn run_resistance_test_request<B, D>(
    req: TmcRequest,
    mut ctx: DiagnosticContext<'_, B, D>,
) -> DiagnosticOutcome
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    let TmcRequest {
        kind: TmcRequestKind::ResistanceTest { oid, detail: _ },
        ..
    } = req
    else {
        return DiagnosticOutcome::NotDiagnostic(req);
    };

    let profile = ctx
        .tmc_config
        .resistance_profile(ctx.tmc_config.resistance_scale_metadata_validated);
    let profile_report = profile_report(oid, &profile);

    if ctx.readiness.homing_active || ctx.readiness.stepper_active {
        return handled(
            profile_report,
            empty_resistance_run_report(oid, CURRENT_STEP_STATUS_NOT_READY),
            None,
        );
    }

    let mut backend = DiagnosticResistanceBackend::new(&mut ctx);
    let terminal =
        run_resistance_identification(&mut backend, ResistanceRunMode::ExternallyArmed).await;
    let host_liveness_aborted = backend.host_liveness_aborted();
    let current = terminal.current();

    match terminal.result() {
        Ok(result) => {
            let run_report = success_run_report(oid, &profile, &result);
            let axes = [
                axis_report(oid, &result.axis0),
                axis_report(oid, &result.axis1),
            ];
            handled(profile_report, run_report, Some(axes))
        }
        Err(error) => handled(
            profile_report,
            failed_run_report(oid, terminal_status(error, host_liveness_aborted), current),
            None,
        ),
    }
}
