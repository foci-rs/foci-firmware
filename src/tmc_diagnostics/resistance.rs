//! `FOCI_RESISTANCE_TEST` diagnostic: runs the shared resistance-identification
//! engine over an already-armed motor without touching commissioning state.

use core::sync::atomic::Ordering::Relaxed;

use foci_tmc4671::registers::Register;

use crate::resistance_identification::{
    ResistanceAxisResult, ResistanceBackend, ResistanceBackendError, ResistanceCurrentEvidence,
    ResistanceProfile, ResistanceRunMode, ResistanceRunResult, run_resistance_identification,
};
use crate::tmc::{TmcRequest, TmcRequestKind};

use super::{
    CURRENT_STEP_STATUS_NOT_READY, DiagnosticContext, DiagnosticDelay, DiagnosticOutcome,
    DiagnosticReport, ResistanceAxisReport, ResistanceProfileReport, ResistanceRunReport,
    empty_resistance_run_report,
};

/// Adapter exposing the board-agnostic [`DiagnosticContext`] as a
/// [`ResistanceBackend`] for the shared resistance-identification runner.
///
/// `set_enable_pin` is a no-op: the diagnostic runs in
/// [`ResistanceRunMode::ExternallyArmed`], so the runner never asserts or
/// clears the enable pin. `is_armed_for_open_loop_voltage` checks the same
/// runtime/lifecycle state every other diagnostic-test handler in this module
/// uses to gate readiness: the enable pin asserted
/// ([`crate::tmc_control::TmcControlRuntime::is_motor_enabled`]) and
/// calibration established
/// ([`crate::tmc_control::TmcControlRuntime::is_calibrated`]).
/// `adc_offsets_ready` checks the channel's `motor_sentinel`, the same flag
/// [`crate::commissioning::backend::CommissionBoardConfig::adc_offsets_ready`]
/// uses: it is captured by the ADC-calibration phase before any later
/// commissioning phase energizes, so a non-zero sentinel proves ADC offsets
/// were established at some commission/arm cycle, independent of whether full
/// commissioning ever ran afterward.
pub(super) struct DiagnosticResistanceBackend<'ctx, 'a, B, D> {
    ctx: &'ctx mut DiagnosticContext<'a, B, D>,
}

impl<'ctx, 'a, B, D> DiagnosticResistanceBackend<'ctx, 'a, B, D> {
    pub(super) fn new(ctx: &'ctx mut DiagnosticContext<'a, B, D>) -> Self {
        Self { ctx }
    }
}

impl<B, D> ResistanceBackend for DiagnosticResistanceBackend<'_, '_, B, D>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    async fn read_register(&mut self, addr: u8) -> Result<u32, ResistanceBackendError> {
        self.ctx
            .tmc
            .read_register(Register::new(addr))
            .await
            .map_err(|_| ResistanceBackendError::Spi)
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ResistanceBackendError> {
        self.ctx
            .tmc
            .write_register(Register::new(addr), value)
            .await
            .map_err(|_| ResistanceBackendError::Spi)
    }

    async fn delay_ms(&mut self, ms: u64) {
        self.ctx.delay.delay_ms(ms).await;
    }

    fn elapsed_us(&self) -> u64 {
        self.ctx.delay.now_us()
    }

    fn set_enable_pin(&mut self, _enabled: bool) {
        // No-op: `ExternallyArmed` mode never asserts/clears the enable pin.
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

    fn adc_offsets_ready(&self) -> bool {
        crate::state::tmc::tmc_channel(self.ctx.runtime.channel())
            .is_some_and(|state| state.motor_sentinel.load(Relaxed) != 0)
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
        .unwrap_or((0, 0, 0));

    ResistanceAxisReport {
        oid,
        electrical_axis: axis.axis.index,
        phi_e_ext: axis.axis.phi_e_ext,
        r_count_slope_milli: axis.r_count_slope_milli,
        intercept_count: axis.intercept_count,
        rmse_permille: axis.rmse_permille,
        selected_mask: axis.selected_mask,
        excluded_point_mask: axis.excluded_point_mask,
        selected_count: axis.selected_count,
        signed_count_slope_milli,
        signed_asymmetry_permille,
        drift_permille,
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
        // The diagnostic path's `ResistanceRunResult` has no gain-path
        // concept; only the commission path populates this field.
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

fn handled(
    profile: ResistanceProfileReport,
    run: ResistanceRunReport,
    axes: Option<[ResistanceAxisReport; 2]>,
) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { profile, run, axes })
}

/// Run a resistance-identification diagnostic request.
///
/// Runs the shared engine in [`ResistanceRunMode::ExternallyArmed`] over the
/// motor the normal lifecycle path has already armed and enabled. Never
/// touches commissioning state.
///
/// Rejects with the same not-ready status the sibling
/// [`voltage_step`](super::run_voltage_step_request) and
/// [`current_step`](super::run_current_step_request) diagnostics use when
/// `readiness.homing_active` or `readiness.stepper_active` is set, before
/// invoking the shared runner. The runner's own entry pre-checks only see
/// enabled/calibrated/ADC-ready state through
/// [`ResistanceBackend::is_armed_for_open_loop_voltage`]; without this
/// dispatch-level gate a resistance test could switch the motor to open-loop
/// voltage while homing or active stepping is in progress.
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
            failed_run_report(oid, error.status_code(), current),
            None,
        ),
    }
}
