//! `FOCI_IMPEDANCE_TEST` diagnostic: runs the shared impedance-identification
//! engine over an already-armed motor without touching commissioning state.

use foci_tmc4671::registers::{
    DSADC_MDEC_B_MDEC_A, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, Register, STATUS_FLAGS,
};

use crate::impedance_identification::{
    IMPEDANCE_PROFILE_POINTS, ImpedanceBackend, ImpedanceBackendError, ImpedanceError,
    ImpedanceFitPoint, ImpedancePointObservation, ImpedancePowerStageReadback, ImpedanceProfile,
    ImpedanceReadbackMismatch, ImpedanceRunResult, run_impedance_identification,
};
use crate::tmc::{TmcRequest, TmcRequestKind, normalize_diagnostic_detail};

use super::reports::IMPEDANCE_STATUS_AXIS_DEVIATION;
use super::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, DiagnosticContext, DiagnosticDelay,
    DiagnosticOutcome, DiagnosticReport, IMPEDANCE_STATUS_CURRENT_LIMIT,
    IMPEDANCE_STATUS_ENCODER_MOVED, IMPEDANCE_STATUS_HIGH_RESIDUAL,
    IMPEDANCE_STATUS_INSUFFICIENT_VOLTAGE, IMPEDANCE_STATUS_LOW_SIGNAL,
    IMPEDANCE_STATUS_POOR_COVERAGE, IMPEDANCE_STATUS_PROFILE_READBACK,
    IMPEDANCE_STATUS_REPEATABILITY, IMPEDANCE_STATUS_SCALAR_2F_MISMATCH,
    IMPEDANCE_STATUS_UNSAFE_STATUS_FLAGS, IMPEDANCE_STATUS_UNSUPPORTED_MOTOR_TYPE,
    IMPEDANCE_STATUS_UNSUPPORTED_PROFILE, ImpedanceBaselineReport, ImpedanceFitReport,
    ImpedanceObservationReport, ImpedanceProfileReport, ImpedanceRunReport,
    empty_impedance_run_report,
};

pub(crate) struct DiagnosticImpedanceBackend<'ctx, 'a, B, D> {
    ctx: &'ctx mut DiagnosticContext<'a, B, D>,
    status_flags_or: u32,
    power_stage_readback: ImpedancePowerStageReadback,
    fit_attempts: [Option<ImpedanceFitPoint>; IMPEDANCE_PROFILE_POINTS],
    baselines: [Option<crate::impedance_identification::ImpedanceBaselineObservation>;
        IMPEDANCE_PROFILE_POINTS],
    observations: [Option<ImpedancePointObservation>; IMPEDANCE_PROFILE_POINTS],
    readback_mismatch: ImpedanceReadbackMismatch,
}

impl<'ctx, 'a, B, D> DiagnosticImpedanceBackend<'ctx, 'a, B, D> {
    pub(crate) fn new(ctx: &'ctx mut DiagnosticContext<'a, B, D>) -> Self {
        Self {
            ctx,
            status_flags_or: 0,
            power_stage_readback: ImpedancePowerStageReadback::EMPTY,
            fit_attempts: [None; IMPEDANCE_PROFILE_POINTS],
            baselines: [None; IMPEDANCE_PROFILE_POINTS],
            observations: [None; IMPEDANCE_PROFILE_POINTS],
            readback_mismatch: ImpedanceReadbackMismatch::EMPTY,
        }
    }
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
    }

    async fn delay_ms(&mut self, ms: u64) {
        self.ctx.delay.delay_ms(ms).await;
    }

    fn timestamp_us(&self) -> u32 {
        // Low 32 bits of the board monotonic. The runner only takes start-relative
        // wrapping differences within a capture window far shorter than the u32 us
        // wrap (~71 min), so truncation is safe and captures real read overhead.
        self.ctx.delay.now_us() as u32
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        self.ctx.runtime.is_motor_enabled() && self.ctx.runtime.is_calibrated()
    }

    fn profile(&self) -> ImpedanceProfile {
        ImpedanceProfile::standard_openffboard()
    }

    fn record_point_observation(
        &mut self,
        point_index: usize,
        observation: ImpedancePointObservation,
    ) {
        if let Some(slot) = self.observations.get_mut(point_index) {
            *slot = Some(observation);
        }
    }

    fn record_baseline_observation(
        &mut self,
        point_index: usize,
        observation: crate::impedance_identification::ImpedanceBaselineObservation,
    ) {
        if let Some(slot) = self.baselines.get_mut(point_index) {
            *slot = Some(observation);
        }
    }

    fn record_fit_attempt(&mut self, point_index: usize, fit: ImpedanceFitPoint) {
        if let Some(slot) = self.fit_attempts.get_mut(point_index) {
            *slot = Some(fit);
        }
    }

    fn record_profile_readback_mismatch(&mut self, addr: u8, expected: u32, actual: u32) {
        self.readback_mismatch = ImpedanceReadbackMismatch {
            addr,
            expected,
            actual,
        };
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
        readback_mismatch_addr: result.readback_mismatch.addr,
        readback_expected: result.readback_mismatch.expected,
        readback_actual: result.readback_mismatch.actual,
    }
}

fn failed_run_report(
    oid: u8,
    status: u8,
    status_flags_or: u32,
    readback_mismatch: ImpedanceReadbackMismatch,
) -> ImpedanceRunReport {
    let mut report = empty_impedance_run_report(oid, status);
    report.status_flags_or = status_flags_or;
    report.readback_mismatch_addr = readback_mismatch.addr;
    report.readback_expected = readback_mismatch.expected;
    report.readback_actual = readback_mismatch.actual;
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
        magnitude_mean_count_per_ud_milli: fit.magnitude_mean_count_per_ud_milli,
        magnitude_2f_count_per_ud_milli: fit.magnitude_2f_count_per_ud_milli,
        magnitude_2f_residual_permille: fit.magnitude_2f_residual_permille,
        magnitude_2f_coverage_permille: fit.magnitude_2f_coverage_permille,
        warning_flags: fit.warning_flags.0,
    }
}

fn fit_reports(
    oid: u8,
    result: &ImpedanceRunResult,
) -> [Option<ImpedanceFitReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        Some(fit_report(
            oid,
            index,
            &result.profile,
            result.point_fits[index],
        ))
    })
}

fn fit_attempt_reports(
    oid: u8,
    profile: &ImpedanceProfile,
    attempts: &[Option<ImpedanceFitPoint>; IMPEDANCE_PROFILE_POINTS],
) -> [Option<ImpedanceFitReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| attempts[index].map(|fit| fit_report(oid, index, profile, fit)))
}

fn observation_report(
    oid: u8,
    point_index: usize,
    profile: &ImpedanceProfile,
    observation: ImpedancePointObservation,
) -> ImpedanceObservationReport {
    let point = profile.injection_points[point_index];
    ImpedanceObservationReport {
        oid,
        point_index: point_index as u8,
        frequency_millihz: point.frequency_millihz,
        ud_abs: point.ud_abs,
        status_flags_or: observation.status_flags_or,
        openloop_phi_sample: observation.openloop_phi_sample,
        openloop_velocity_actual: observation.openloop_velocity_actual,
        adc_iux_sample: observation.adc_iux_sample,
        adc_iwy_sample: observation.adc_iwy_sample,
        foc_iux_sample: observation.foc_iux_sample,
        foc_iwy_sample: observation.foc_iwy_sample,
        foc_uq_sample: observation.foc_uq_sample,
        foc_ud_sample: observation.foc_ud_sample,
        foc_uq_limited_sample: observation.foc_uq_limited_sample,
        foc_ud_limited_sample: observation.foc_ud_limited_sample,
        foc_iq_sample: observation.foc_iq_sample,
        foc_id_sample: observation.foc_id_sample,
    }
}

fn observation_reports(
    oid: u8,
    result: &ImpedanceRunResult,
) -> [Option<ImpedanceObservationReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        Some(observation_report(
            oid,
            index,
            &result.profile,
            result.point_observations[index],
        ))
    })
}

fn baseline_report(
    oid: u8,
    point_index: usize,
    profile: &ImpedanceProfile,
    baseline: crate::impedance_identification::ImpedanceBaselineObservation,
) -> ImpedanceBaselineReport {
    let point = profile.injection_points[point_index];
    ImpedanceBaselineReport {
        oid,
        point_index: point_index as u8,
        frequency_millihz: point.frequency_millihz,
        ud_abs: point.ud_abs,
        dc_adc_iux_sample: baseline.dc_adc_iux_sample,
        dc_adc_iwy_sample: baseline.dc_adc_iwy_sample,
        dc_foc_iux_sample: baseline.dc_foc_iux_sample,
        dc_foc_iwy_sample: baseline.dc_foc_iwy_sample,
        dc_foc_iq_sample: baseline.dc_foc_iq_sample,
        dc_foc_id_sample: baseline.dc_foc_id_sample,
    }
}

fn baseline_reports(
    oid: u8,
    result: &ImpedanceRunResult,
) -> [Option<ImpedanceBaselineReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        Some(baseline_report(
            oid,
            index,
            &result.profile,
            result.point_baselines[index],
        ))
    })
}

fn baseline_attempt_reports(
    oid: u8,
    profile: &ImpedanceProfile,
    baselines: &[Option<crate::impedance_identification::ImpedanceBaselineObservation>;
         IMPEDANCE_PROFILE_POINTS],
) -> [Option<ImpedanceBaselineReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        baselines[index].map(|baseline| baseline_report(oid, index, profile, baseline))
    })
}

fn observation_attempt_reports(
    oid: u8,
    profile: &ImpedanceProfile,
    observations: &[Option<ImpedancePointObservation>; IMPEDANCE_PROFILE_POINTS],
) -> [Option<ImpedanceObservationReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        observations[index].map(|observation| observation_report(oid, index, profile, observation))
    })
}

fn empty_fit_reports() -> [Option<ImpedanceFitReport>; IMPEDANCE_PROFILE_POINTS] {
    [None; IMPEDANCE_PROFILE_POINTS]
}

fn empty_baseline_reports() -> [Option<ImpedanceBaselineReport>; IMPEDANCE_PROFILE_POINTS] {
    [None; IMPEDANCE_PROFILE_POINTS]
}

fn empty_observation_reports() -> [Option<ImpedanceObservationReport>; IMPEDANCE_PROFILE_POINTS] {
    [None; IMPEDANCE_PROFILE_POINTS]
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
        ImpedanceError::AxisDeviation => IMPEDANCE_STATUS_AXIS_DEVIATION,
        ImpedanceError::Scalar2fMismatch => IMPEDANCE_STATUS_SCALAR_2F_MISMATCH,
        ImpedanceError::HighResidual => IMPEDANCE_STATUS_HIGH_RESIDUAL,
        ImpedanceError::RepeatabilityMismatch => IMPEDANCE_STATUS_REPEATABILITY,
    }
}

fn handled(
    profile: ImpedanceProfileReport,
    run: ImpedanceRunReport,
    fits: [Option<ImpedanceFitReport>; IMPEDANCE_PROFILE_POINTS],
    baselines: [Option<ImpedanceBaselineReport>; IMPEDANCE_PROFILE_POINTS],
    observations: [Option<ImpedanceObservationReport>; IMPEDANCE_PROFILE_POINTS],
    detail: u8,
) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::ImpedanceTest {
        profile,
        run,
        fits,
        baselines,
        observations,
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
            empty_fit_reports(),
            empty_baseline_reports(),
            empty_observation_reports(),
            detail,
        );
    }

    let mut backend = DiagnosticImpedanceBackend::new(&mut ctx);

    match run_impedance_identification(&mut backend).await {
        Ok(result) => {
            let run = run_report(oid, &result);
            let fits = fit_reports(oid, &result);
            let baselines = baseline_reports(oid, &result);
            let observations = observation_reports(oid, &result);
            handled(profile_report, run, fits, baselines, observations, detail)
        }
        Err(error) => handled(
            profile_report,
            failed_run_report(
                oid,
                status_code(error),
                backend.status_flags_or,
                backend.readback_mismatch,
            ),
            fit_attempt_reports(oid, &profile, &backend.fit_attempts),
            baseline_attempt_reports(oid, &profile, &backend.baselines),
            observation_attempt_reports(oid, &profile, &backend.observations),
            detail,
        ),
    }
}
