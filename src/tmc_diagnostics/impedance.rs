//! `FOCI_IMPEDANCE_TEST` diagnostic: runs the shared impedance-identification
//! engine over an already-armed motor without touching commissioning state.

use foci_tmc4671::registers::{
    DSADC_MDEC_B_MDEC_A, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, Register, STATUS_FLAGS,
};

use crate::impedance_identification::{
    IMPEDANCE_PROFILE_POINTS, ImpedanceBackend, ImpedanceBackendError, ImpedanceError,
    ImpedanceFitPoint, ImpedancePowerStageReadback, ImpedanceProfile, ImpedanceRunResult,
    run_impedance_identification,
};
use crate::tmc::{TmcRequest, TmcRequestKind, normalize_diagnostic_detail};

use super::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, DiagnosticContext, DiagnosticDelay,
    DiagnosticOutcome, DiagnosticReport, IMPEDANCE_STATUS_CURRENT_LIMIT,
    IMPEDANCE_STATUS_ENCODER_MOVED, IMPEDANCE_STATUS_HIGH_RESIDUAL,
    IMPEDANCE_STATUS_INSUFFICIENT_VOLTAGE, IMPEDANCE_STATUS_LOW_SIGNAL,
    IMPEDANCE_STATUS_POOR_COVERAGE, IMPEDANCE_STATUS_PROFILE_READBACK,
    IMPEDANCE_STATUS_REPEATABILITY, IMPEDANCE_STATUS_SCALAR_2F_MISMATCH,
    IMPEDANCE_STATUS_UNSAFE_STATUS_FLAGS, IMPEDANCE_STATUS_UNSUPPORTED_MOTOR_TYPE,
    IMPEDANCE_STATUS_UNSUPPORTED_PROFILE, ImpedanceFitReport, ImpedanceProfileReport,
    ImpedanceRunReport, empty_impedance_run_report,
};

struct DiagnosticImpedanceBackend<'ctx, 'a, B, D> {
    ctx: &'ctx mut DiagnosticContext<'a, B, D>,
    elapsed_us: u32,
    status_flags_or: u32,
    power_stage_readback: ImpedancePowerStageReadback,
}

impl<B, D> ImpedanceBackend for DiagnosticImpedanceBackend<'_, '_, B, D>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    async fn read_register(&mut self, addr: u8) -> Result<u32, ImpedanceBackendError> {
        let value = self
            .ctx
            .tmc
            .read_register(Register::new(addr))
            .await
            .map_err(|_| ImpedanceBackendError::Spi)?;
        if addr == STATUS_FLAGS.addr() {
            self.status_flags_or |= value;
        } else if addr == PWM_MAXCNT.addr() {
            self.power_stage_readback.pwm_maxcnt_readback = value;
        } else if addr == PWM_BBM_H_BBM_L.addr() {
            self.power_stage_readback.bbm_readback = value;
        } else if addr == DSADC_MDEC_B_MDEC_A.addr() {
            self.power_stage_readback.dsadc_mdec_readback = value;
        } else if addr == PWM_SV_CHOP.addr() {
            self.power_stage_readback.pwm_sv_chop_readback = value;
        }
        Ok(value)
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ImpedanceBackendError> {
        self.ctx
            .tmc
            .write_register(Register::new(addr), value)
            .await
            .map_err(|_| ImpedanceBackendError::Spi)
    }

    async fn delay_us(&mut self, us: u32) {
        self.ctx.delay.delay_us(us).await;
        self.elapsed_us = self.elapsed_us.wrapping_add(us);
    }

    async fn delay_ms(&mut self, ms: u64) {
        self.ctx.delay.delay_ms(ms).await;
        let us = ms.saturating_mul(1_000).min(u64::from(u32::MAX)) as u32;
        self.elapsed_us = self.elapsed_us.wrapping_add(us);
    }

    fn timestamp_us(&self) -> u32 {
        self.elapsed_us
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        self.ctx.runtime.is_motor_enabled() && self.ctx.runtime.is_calibrated()
    }

    fn profile(&self) -> ImpedanceProfile {
        ImpedanceProfile::standard_openffboard()
    }
}

fn profile_report(oid: u8, profile: &ImpedanceProfile) -> ImpedanceProfileReport {
    ImpedanceProfileReport {
        oid,
        profile_version: profile.profile_version,
        point_count: profile.injection_points.len() as u8,
        sample_count: profile.sample_count,
        sample_interval_us: profile.sample_interval_us,
        max_ud: profile.max_abs_ud(),
        status_flags_fail_mask: profile.status_flags_fail_mask,
        status_flags_warn_mask: profile.status_flags_warn_mask,
        scale_metadata_validated: profile.scale_metadata_validated,
    }
}

fn run_report(oid: u8, result: &ImpedanceRunResult) -> ImpedanceRunReport {
    ImpedanceRunReport {
        oid,
        status: 0,
        profile_version: result.profile.profile_version,
        selected_axis_max_count_per_ud_milli: result.summary.selected_axis_max_count_per_ud_milli,
        selected_axis_min_count_per_ud_milli: result.summary.selected_axis_min_count_per_ud_milli,
        repeatability_permille: result.summary.repeatability_permille,
        warning_flags: result.warning_flags.0,
        status_flags_or: result.status_flags_or,
    }
}

fn failed_run_report(
    oid: u8,
    status: u8,
    status_flags_or: u32,
    _power_stage_readback: ImpedancePowerStageReadback,
) -> ImpedanceRunReport {
    let mut report = empty_impedance_run_report(oid, status);
    report.status_flags_or = status_flags_or;
    report
}

fn fit_report(
    oid: u8,
    point_index: usize,
    profile: &ImpedanceProfile,
    fit: ImpedanceFitPoint,
) -> ImpedanceFitReport {
    let point = profile.injection_points[point_index];
    ImpedanceFitReport {
        oid,
        point_index: point_index as u8,
        frequency_millihz: point.frequency_millihz,
        ud_abs: point.ud_abs,
        axis_max_count_per_ud_milli: fit.axis_max_count_per_ud_milli,
        axis_min_count_per_ud_milli: fit.axis_min_count_per_ud_milli,
        axis_angle_electrical_counts: fit.axis_angle_electrical_counts,
        residual_permille: fit.residual_permille,
        coverage_permille: fit.coverage_permille,
        scalar_2f_mismatch_permille: fit.scalar_2f_mismatch_permille,
        warning_flags: fit.warning_flags.0,
    }
}

fn fit_reports(
    oid: u8,
    result: &ImpedanceRunResult,
) -> [ImpedanceFitReport; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| fit_report(oid, index, &result.profile, result.point_fits[index]))
}

fn status_code(error: ImpedanceError) -> u8 {
    match error {
        ImpedanceError::NotArmed | ImpedanceError::NotReady => CURRENT_STEP_STATUS_NOT_READY,
        ImpedanceError::BackendIo | ImpedanceError::RestoreFailed => CURRENT_STEP_STATUS_SPI,
        ImpedanceError::UnsupportedMotorType => IMPEDANCE_STATUS_UNSUPPORTED_MOTOR_TYPE,
        ImpedanceError::MissingCountSubstrate => IMPEDANCE_STATUS_UNSUPPORTED_PROFILE,
        ImpedanceError::InsufficientVoltageAuthority => IMPEDANCE_STATUS_INSUFFICIENT_VOLTAGE,
        ImpedanceError::UnsupportedProfile => IMPEDANCE_STATUS_UNSUPPORTED_PROFILE,
        ImpedanceError::ProfileReadbackMismatch => IMPEDANCE_STATUS_PROFILE_READBACK,
        ImpedanceError::CurrentLimitExceeded => IMPEDANCE_STATUS_CURRENT_LIMIT,
        ImpedanceError::EncoderMoved => IMPEDANCE_STATUS_ENCODER_MOVED,
        ImpedanceError::UnsafeStatusFlags => IMPEDANCE_STATUS_UNSAFE_STATUS_FLAGS,
        ImpedanceError::LowSignal => IMPEDANCE_STATUS_LOW_SIGNAL,
        ImpedanceError::PoorCoverage => IMPEDANCE_STATUS_POOR_COVERAGE,
        ImpedanceError::Scalar2fMismatch => IMPEDANCE_STATUS_SCALAR_2F_MISMATCH,
        ImpedanceError::HighResidual => IMPEDANCE_STATUS_HIGH_RESIDUAL,
        ImpedanceError::RepeatabilityMismatch => IMPEDANCE_STATUS_REPEATABILITY,
    }
}

fn handled(
    profile: ImpedanceProfileReport,
    run: ImpedanceRunReport,
    fits: Option<[ImpedanceFitReport; IMPEDANCE_PROFILE_POINTS]>,
    detail: u8,
) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::ImpedanceTest {
        profile,
        run,
        fits,
        detail,
    })
}

/// Run an impedance-identification diagnostic request.
///
/// Rejects homing, active stepping, and not-armed/not-calibrated channels
/// before constructing the backend or invoking the runner, so these busy/not
/// ready cases perform no SPI writes and no restore sequence.
pub async fn run_impedance_test_request<B, D>(
    req: TmcRequest,
    mut ctx: DiagnosticContext<'_, B, D>,
) -> DiagnosticOutcome
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    let TmcRequest {
        kind: TmcRequestKind::ImpedanceTest { oid, detail },
        ..
    } = req
    else {
        return DiagnosticOutcome::NotDiagnostic(req);
    };
    let detail = normalize_diagnostic_detail(detail);
    let profile = ImpedanceProfile::standard_openffboard();
    let profile_report = profile_report(oid, &profile);

    if ctx.readiness.homing_active
        || ctx.readiness.stepper_active
        || !ctx.runtime.is_motor_enabled()
        || !ctx.runtime.is_calibrated()
    {
        return handled(
            profile_report,
            empty_impedance_run_report(oid, CURRENT_STEP_STATUS_NOT_READY),
            None,
            detail,
        );
    }

    let mut backend = DiagnosticImpedanceBackend {
        ctx: &mut ctx,
        elapsed_us: 0,
        status_flags_or: 0,
        power_stage_readback: ImpedancePowerStageReadback::EMPTY,
    };

    match run_impedance_identification(&mut backend).await {
        Ok(result) => {
            let run = run_report(oid, &result);
            let fits = fit_reports(oid, &result);
            handled(profile_report, run, Some(fits), detail)
        }
        Err(error) => handled(
            profile_report,
            failed_run_report(
                oid,
                status_code(error),
                backend.status_flags_or,
                backend.power_stage_readback,
            ),
            None,
            detail,
        ),
    }
}
