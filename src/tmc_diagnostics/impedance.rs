//! `FOCI_IMPEDANCE_TEST` diagnostic: runs the shared impedance-identification
//! engine over an already-armed motor without touching commissioning state.

use core::sync::atomic::Ordering::Relaxed;

use foci_tmc4671::registers::{
    DSADC_MDEC_B_MDEC_A, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, Register, STATUS_FLAGS,
};

use crate::impedance_identification::{
    IMPEDANCE_POST_SWEEP_ZERO_INDEX, IMPEDANCE_PROFILE_POINTS, IMPEDANCE_STABILITY_ZERO_INDEX,
    ImpedanceAngleObservation, ImpedanceBackend, ImpedanceBackendError, ImpedanceError,
    ImpedancePowerStageReadback, ImpedanceProfile, ImpedanceReadbackMismatch, ImpedanceRunResult,
    run_impedance_identification,
};
use crate::tmc::{TmcRequest, TmcRequestKind, normalize_diagnostic_detail};

use super::reports::{IMPEDANCE_STATUS_AXIS_DEVIATION, IMPEDANCE_STATUS_FILTER_STATE};
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
    observations: [Option<ImpedanceAngleObservation>; IMPEDANCE_PROFILE_POINTS],
    readback_mismatch: ImpedanceReadbackMismatch,
}

impl<'ctx, 'a, B, D> DiagnosticImpedanceBackend<'ctx, 'a, B, D> {
    pub(crate) fn new(ctx: &'ctx mut DiagnosticContext<'a, B, D>) -> Self {
        Self {
            ctx,
            status_flags_or: 0,
            power_stage_readback: ImpedancePowerStageReadback::EMPTY,
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

    fn encoder_counts_per_rev(&self) -> u32 {
        crate::state::tmc::tmc_channel(self.ctx.runtime.channel())
            .map(|state| state.encoder_ppr.load(Relaxed).saturating_mul(4))
            .unwrap_or(0)
    }

    fn record_angle_observation(
        &mut self,
        point_index: usize,
        observation: ImpedanceAngleObservation,
    ) {
        if let Some(slot) = self.observations.get_mut(point_index) {
            *slot = Some(observation);
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
        point_count: profile.angle_points.len() as u8,
        sample_count: profile.sample_count,
        sample_interval_us: profile.sample_interval_us,
        carrier_frequency_millihz: profile.carrier_frequency_millihz(),
        samples_per_cycle: profile.samples_per_cycle,
        cycles_per_angle: profile.cycles_per_angle,
        max_ud: profile.max_abs_ud(),
        hold_ud: profile.hold_ud_abs,
        carrier_ud: profile
            .angle_points
            .iter()
            .map(|point| point.ud_abs)
            .max()
            .unwrap_or(0),
        status_flags_fail_mask: profile.status_flags_fail_mask,
        status_flags_warn_mask: profile.status_flags_warn_mask,
        scale_metadata_validated: profile.scale_metadata_validated,
    }
}

fn run_report(oid: u8, result: &ImpedanceRunResult) -> ImpedanceRunReport {
    let observations = result.angle_observations.map(Some);
    let duplicate_zero = duplicate_zero_evidence(&observations);

    ImpedanceRunReport {
        oid,
        status: 0,
        selected_axis_max_count_per_ud_milli: result.summary.axis_max_count_per_ud_milli,
        selected_axis_min_count_per_ud_milli: result.summary.axis_min_count_per_ud_milli,
        repeatability_permille: result.summary.repeatability_permille,
        warning_flags: result.warning_flags.0,
        status_flags_or: result.status_flags_or,
        readback_mismatch_addr: result.readback_mismatch.addr,
        readback_expected: result.readback_mismatch.expected,
        readback_actual: result.readback_mismatch.actual,
        filter_torque_enable_before: result.filter_state.torque_enable_before,
        filter_flux_enable_before: result.filter_state.flux_enable_before,
        filter_torque_enable_during: result.filter_state.torque_enable_during,
        filter_flux_enable_during: result.filter_state.flux_enable_during,
        duplicate_zero_encoder_drift_counts: duplicate_zero.encoder_drift,
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

fn failed_observed_run_report(
    oid: u8,
    status: u8,
    status_flags_or: u32,
    readback_mismatch: ImpedanceReadbackMismatch,
    observations: &[Option<ImpedanceAngleObservation>; IMPEDANCE_PROFILE_POINTS],
) -> ImpedanceRunReport {
    let mut report = failed_run_report(oid, status, status_flags_or, readback_mismatch);
    let duplicate_zero = duplicate_zero_evidence(observations);
    report.repeatability_permille = duplicate_zero.repeatability_permille;
    report.duplicate_zero_encoder_drift_counts = duplicate_zero.encoder_drift;
    report
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DuplicateZeroEvidence {
    repeatability_permille: u16,
    encoder_drift: u32,
}

impl DuplicateZeroEvidence {
    const EMPTY: Self = Self {
        repeatability_permille: 0,
        encoder_drift: 0,
    };
}

fn duplicate_zero_evidence(
    observations: &[Option<ImpedanceAngleObservation>; IMPEDANCE_PROFILE_POINTS],
) -> DuplicateZeroEvidence {
    indexed_duplicate_zero_evidence(
        observations,
        IMPEDANCE_POST_SWEEP_ZERO_INDEX,
        IMPEDANCE_STABILITY_ZERO_INDEX,
    )
}

fn indexed_duplicate_zero_evidence(
    observations: &[Option<ImpedanceAngleObservation>; IMPEDANCE_PROFILE_POINTS],
    first_index: usize,
    second_index: usize,
) -> DuplicateZeroEvidence {
    let Some(Some(first)) = observations.get(first_index) else {
        return DuplicateZeroEvidence::EMPTY;
    };
    let Some(Some(second)) = observations.get(second_index) else {
        return DuplicateZeroEvidence::EMPTY;
    };
    if first.angle_electrical_counts != second.angle_electrical_counts {
        return DuplicateZeroEvidence::EMPTY;
    }

    DuplicateZeroEvidence {
        repeatability_permille: repeatability_permille(
            first.inductive_response_count_per_ud_milli,
            second.inductive_response_count_per_ud_milli,
        ),
        encoder_drift: first.encoder_before.abs_diff(second.encoder_before),
    }
}

fn repeatability_permille(first: u32, second: u32) -> u16 {
    let mean = (u64::from(first) + u64::from(second)) / 2;
    if mean == 0 {
        return 0;
    }

    let diff = u64::from(first.abs_diff(second));
    ((diff * 1_000 + mean / 2) / mean).min(u64::from(u16::MAX)) as u16
}

fn fit_report(oid: u8, point_index: usize, result: &ImpedanceRunResult) -> ImpedanceFitReport {
    let point = result.profile.angle_points[point_index];
    let observation = result.angle_observations[point_index];
    ImpedanceFitReport {
        oid,
        point_index: point_index as u8,
        frequency_millihz: result.profile.carrier_frequency_millihz(),
        ud_abs: point.ud_abs,
        axis_max_count_per_ud_milli: result.summary.axis_max_count_per_ud_milli,
        axis_min_count_per_ud_milli: result.summary.axis_min_count_per_ud_milli,
        axis_angle_electrical_counts: result.summary.axis_angle_electrical_counts,
        residual_permille: observation.residual_permille,
        coverage_permille: observation.coverage_permille,
        scalar_2f_mismatch_permille: 0,
        mean_response_count_per_ud_milli: result.summary.mean_response_count_per_ud_milli,
        saliency_permille: result.summary.saliency_permille,
        saliency_determinable: u8::from(result.summary.saliency_determinable),
        angle_coverage_permille: result.summary.angle_coverage_permille,
        axis_deviation_counts: result.summary.axis_deviation_counts,
        inductive_response_count_per_ud_milli: observation.inductive_response_count_per_ud_milli,
        in_phase_response_count_per_ud_milli: observation.in_phase_response_count_per_ud_milli,
        secondary_adc_response_count_per_ud_milli: observation
            .secondary_adc_response_count_per_ud_milli,
        magnitude_mean_count_per_ud_milli: 0,
        magnitude_2f_count_per_ud_milli: 0,
        magnitude_2f_residual_permille: 0,
        magnitude_2f_coverage_permille: 0,
        warning_flags: observation.warning_flags.0,
    }
}

fn fit_reports(
    oid: u8,
    result: &ImpedanceRunResult,
) -> [Option<ImpedanceFitReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| Some(fit_report(oid, index, result)))
}

fn observation_report(
    oid: u8,
    point_index: usize,
    profile: &ImpedanceProfile,
    observation: ImpedanceAngleObservation,
) -> ImpedanceObservationReport {
    let point = profile.angle_points[point_index];
    ImpedanceObservationReport {
        oid,
        point_index: point_index as u8,
        frequency_millihz: profile.carrier_frequency_millihz(),
        ud_abs: point.ud_abs,
        angle_electrical_counts: observation.angle_electrical_counts,
        status_flags_or: 0,
        openloop_phi_sample: observation.angle_electrical_counts,
        openloop_velocity_actual: 0,
        adc_iux_sample: observation.baseline_adc_ch0_count,
        adc_iwy_sample: observation.baseline_adc_ch1_count,
        foc_iux_sample: 0,
        foc_iwy_sample: 0,
        foc_uq_sample: 0,
        foc_ud_sample: point.ud_abs as i16,
        foc_uq_limited_sample: 0,
        foc_ud_limited_sample: point.ud_abs as i16,
        foc_iq_sample: observation.baseline_q_count,
        foc_id_sample: observation.baseline_d_count,
        inductive_response_count_per_ud_milli: observation.inductive_response_count_per_ud_milli,
        in_phase_response_count_per_ud_milli: observation.in_phase_response_count_per_ud_milli,
        encoder_before: observation.encoder_before,
        encoder_after: observation.encoder_after,
        encoder_min: observation.encoder_min,
        encoder_max: observation.encoder_max,
        encoder_peak_counts: observation.encoder_peak_counts,
        angle_valid: u8::from(observation.valid),
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
            result.angle_observations[index],
        ))
    })
}

fn observation_attempt_reports(
    oid: u8,
    profile: &ImpedanceProfile,
    observations: &[Option<ImpedanceAngleObservation>; IMPEDANCE_PROFILE_POINTS],
) -> [Option<ImpedanceObservationReport>; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        observations[index].map(|observation| observation_report(oid, index, profile, observation))
    })
}

fn baseline_reports(
    oid: u8,
    result: &ImpedanceRunResult,
) -> [Option<ImpedanceBaselineReport>; IMPEDANCE_PROFILE_POINTS] {
    let _ = (oid, result);
    [None; IMPEDANCE_PROFILE_POINTS]
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
        ImpedanceError::FilterState => IMPEDANCE_STATUS_FILTER_STATE,
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
            failed_observed_run_report(
                oid,
                status_code(error),
                backend.status_flags_or,
                backend.readback_mismatch,
                &backend.observations,
            ),
            empty_fit_reports(),
            empty_baseline_reports(),
            observation_attempt_reports(oid, &profile, &backend.observations),
            detail,
        ),
    }
}

#[cfg(test)]
mod tests {
    use crate::impedance_identification::{
        IMPEDANCE_RETURN_ZERO_SETTLE_INDEX, ImpedanceAngleObservation, ImpedanceFilterState,
        ImpedanceFitSummary, ImpedancePowerStageReadback, ImpedanceReadbackMismatch,
        ImpedanceWarningFlags,
    };

    use super::*;

    #[test]
    fn result_maps_held_angle_evidence_into_reports() {
        let profile = ImpedanceProfile::standard_openffboard();
        let mut observations = [ImpedanceAngleObservation::EMPTY; IMPEDANCE_PROFILE_POINTS];
        observations[0] = ImpedanceAngleObservation {
            angle_electrical_counts: 0,
            ud_abs: 640,
            inductive_response_count_per_ud_milli: 1_300,
            in_phase_response_count_per_ud_milli: 40,
            secondary_adc_response_count_per_ud_milli: 7,
            baseline_d_count: 11,
            baseline_q_count: 12,
            baseline_adc_ch0_count: -13,
            baseline_adc_ch1_count: 14,
            residual_permille: 15,
            coverage_permille: 1000,
            encoder_before: 1_000,
            encoder_after: 1_004,
            encoder_min: 998,
            encoder_max: 1_010,
            encoder_peak_counts: 12,
            valid: true,
            warning_flags: ImpedanceWarningFlags::ANGLE_LOCAL_MOTION,
        };
        let result = ImpedanceRunResult {
            profile,
            angle_observations: observations,
            summary: ImpedanceFitSummary {
                mean_response_count_per_ud_milli: 1_000,
                axis_max_count_per_ud_milli: 1_300,
                axis_min_count_per_ud_milli: 700,
                saliency_permille: 300,
                axis_angle_electrical_counts: 0,
                axis_deviation_counts: 0,
                saliency_determinable: true,
                angle_coverage_permille: 1000,
                repeatability_permille: 0,
                warning_flags: ImpedanceWarningFlags::NONE,
            },
            warning_flags: ImpedanceWarningFlags::NONE,
            status_flags_or: 0,
            power_stage_readback: ImpedancePowerStageReadback::EMPTY,
            readback_mismatch: ImpedanceReadbackMismatch::EMPTY,
            filter_state: ImpedanceFilterState {
                torque_enable_before: 1,
                flux_enable_before: 1,
                torque_enable_during: 0,
                flux_enable_during: 0,
            },
        };

        let profile = profile_report(7, &result.profile);
        let fit = fit_reports(7, &result)[0].expect("first fit report");
        let observation = observation_reports(7, &result)[0].expect("first observation report");
        let run = run_report(7, &result);

        assert_eq!(profile.carrier_frequency_millihz, 25_000);
        assert_eq!(profile.samples_per_cycle, 20);
        assert_eq!(profile.cycles_per_angle, 8);
        assert_eq!(fit.mean_response_count_per_ud_milli, 1_000);
        assert_eq!(fit.inductive_response_count_per_ud_milli, 1_300);
        assert_eq!(fit.in_phase_response_count_per_ud_milli, 40);
        assert_eq!(fit.saliency_permille, 300);
        assert_eq!(fit.saliency_determinable, 1);
        assert_eq!(fit.angle_coverage_permille, 1000);
        assert_eq!(fit.secondary_adc_response_count_per_ud_milli, 7);
        assert_eq!(observation.angle_electrical_counts, 0);
        assert_eq!(observation.status_flags_or, 0);
        assert_eq!(observation.encoder_peak_counts, 12);
        assert_eq!(observation.angle_valid, 1);
        assert_eq!(run.filter_torque_enable_before, 1);
        assert_eq!(run.filter_flux_enable_before, 1);
    }

    #[test]
    fn failure_attempt_reports_preserve_captured_observations() {
        let profile = ImpedanceProfile::standard_openffboard();
        let mut observations = [None; IMPEDANCE_PROFILE_POINTS];
        observations[0] = Some(ImpedanceAngleObservation {
            angle_electrical_counts: profile.angle_points[0].angle_electrical_counts,
            ud_abs: profile.angle_points[0].ud_abs,
            inductive_response_count_per_ud_milli: 0,
            in_phase_response_count_per_ud_milli: 0,
            secondary_adc_response_count_per_ud_milli: 0,
            baseline_d_count: 11,
            baseline_q_count: 12,
            baseline_adc_ch0_count: -13,
            baseline_adc_ch1_count: 14,
            residual_permille: 0,
            coverage_permille: 1000,
            encoder_before: 1_000,
            encoder_after: 1_004,
            encoder_min: 998,
            encoder_max: 1_010,
            encoder_peak_counts: 12,
            valid: true,
            warning_flags: ImpedanceWarningFlags::NONE,
        });

        let reports = observation_attempt_reports(7, &profile, &observations);

        let observation = reports[0].expect("first observation report");
        assert_eq!(observation.oid, 7);
        assert_eq!(observation.point_index, 0);
        assert_eq!(
            observation.angle_electrical_counts,
            profile.angle_points[0].angle_electrical_counts
        );
        assert_eq!(observation.encoder_peak_counts, 12);
        assert_eq!(observation.angle_valid, 1);
        assert!(reports[1..].iter().all(Option::is_none));
    }

    #[test]
    fn failure_run_report_preserves_final_duplicate_zero_evidence() {
        let profile = ImpedanceProfile::standard_openffboard();
        let mut observations = [None; IMPEDANCE_PROFILE_POINTS];
        observations[IMPEDANCE_POST_SWEEP_ZERO_INDEX] = Some(ImpedanceAngleObservation {
            angle_electrical_counts: profile.angle_points[IMPEDANCE_POST_SWEEP_ZERO_INDEX]
                .angle_electrical_counts,
            ud_abs: profile.angle_points[IMPEDANCE_POST_SWEEP_ZERO_INDEX].ud_abs,
            inductive_response_count_per_ud_milli: 1_000,
            in_phase_response_count_per_ud_milli: 25,
            secondary_adc_response_count_per_ud_milli: 9,
            baseline_d_count: 11,
            baseline_q_count: 12,
            baseline_adc_ch0_count: -13,
            baseline_adc_ch1_count: 14,
            residual_permille: 0,
            coverage_permille: 1000,
            encoder_before: 100,
            encoder_after: 104,
            encoder_min: 98,
            encoder_max: 110,
            encoder_peak_counts: 12,
            valid: true,
            warning_flags: ImpedanceWarningFlags::NONE,
        });
        observations[IMPEDANCE_STABILITY_ZERO_INDEX] = Some(ImpedanceAngleObservation {
            angle_electrical_counts: profile.angle_points[IMPEDANCE_STABILITY_ZERO_INDEX]
                .angle_electrical_counts,
            ud_abs: profile.angle_points[IMPEDANCE_STABILITY_ZERO_INDEX].ud_abs,
            inductive_response_count_per_ud_milli: 700,
            in_phase_response_count_per_ud_milli: 20,
            secondary_adc_response_count_per_ud_milli: 8,
            baseline_d_count: 11,
            baseline_q_count: 12,
            baseline_adc_ch0_count: -13,
            baseline_adc_ch1_count: 14,
            residual_permille: 0,
            coverage_permille: 1000,
            encoder_before: 172,
            encoder_after: 169,
            encoder_min: 160,
            encoder_max: 172,
            encoder_peak_counts: 12,
            valid: true,
            warning_flags: ImpedanceWarningFlags::NONE,
        });

        let report = failed_observed_run_report(
            7,
            IMPEDANCE_STATUS_REPEATABILITY,
            0x20,
            ImpedanceReadbackMismatch::EMPTY,
            &observations,
        );

        assert_eq!(report.status, IMPEDANCE_STATUS_REPEATABILITY);
        assert_eq!(report.repeatability_permille, 353);
        assert_eq!(report.duplicate_zero_encoder_drift_counts, 72);
        assert_eq!(report.status_flags_or, 0x20);
    }

    #[test]
    fn failure_run_report_ignores_return_zero_settle_evidence() {
        let profile = ImpedanceProfile::standard_openffboard();
        let mut observations = [None; IMPEDANCE_PROFILE_POINTS];
        observations[IMPEDANCE_RETURN_ZERO_SETTLE_INDEX] = Some(ImpedanceAngleObservation {
            angle_electrical_counts: profile.angle_points[IMPEDANCE_RETURN_ZERO_SETTLE_INDEX]
                .angle_electrical_counts,
            ud_abs: profile.angle_points[IMPEDANCE_RETURN_ZERO_SETTLE_INDEX].ud_abs,
            inductive_response_count_per_ud_milli: 124,
            in_phase_response_count_per_ud_milli: 977,
            secondary_adc_response_count_per_ud_milli: 0,
            baseline_d_count: 0,
            baseline_q_count: 0,
            baseline_adc_ch0_count: 0,
            baseline_adc_ch1_count: 0,
            residual_permille: 0,
            coverage_permille: 1000,
            encoder_before: 362,
            encoder_after: 395,
            encoder_min: 362,
            encoder_max: 397,
            encoder_peak_counts: profile.encoder_angle_invalid_counts + 3,
            valid: false,
            warning_flags: ImpedanceWarningFlags::ANGLE_LOCAL_MOTION,
        });
        observations[IMPEDANCE_POST_SWEEP_ZERO_INDEX] = Some(ImpedanceAngleObservation {
            angle_electrical_counts: profile.angle_points[IMPEDANCE_POST_SWEEP_ZERO_INDEX]
                .angle_electrical_counts,
            ud_abs: profile.angle_points[IMPEDANCE_POST_SWEEP_ZERO_INDEX].ud_abs,
            inductive_response_count_per_ud_milli: 92,
            in_phase_response_count_per_ud_milli: 1025,
            secondary_adc_response_count_per_ud_milli: 0,
            baseline_d_count: 0,
            baseline_q_count: 0,
            baseline_adc_ch0_count: 0,
            baseline_adc_ch1_count: 0,
            residual_permille: 0,
            coverage_permille: 1000,
            encoder_before: 396,
            encoder_after: 395,
            encoder_min: 395,
            encoder_max: 397,
            encoder_peak_counts: 2,
            valid: true,
            warning_flags: ImpedanceWarningFlags::NONE,
        });
        observations[IMPEDANCE_STABILITY_ZERO_INDEX] = Some(ImpedanceAngleObservation {
            angle_electrical_counts: profile.angle_points[IMPEDANCE_STABILITY_ZERO_INDEX]
                .angle_electrical_counts,
            ud_abs: profile.angle_points[IMPEDANCE_STABILITY_ZERO_INDEX].ud_abs,
            inductive_response_count_per_ud_milli: 90,
            in_phase_response_count_per_ud_milli: 1030,
            secondary_adc_response_count_per_ud_milli: 0,
            baseline_d_count: 0,
            baseline_q_count: 0,
            baseline_adc_ch0_count: 0,
            baseline_adc_ch1_count: 0,
            residual_permille: 0,
            coverage_permille: 1000,
            encoder_before: 397,
            encoder_after: 396,
            encoder_min: 396,
            encoder_max: 397,
            encoder_peak_counts: 1,
            valid: true,
            warning_flags: ImpedanceWarningFlags::NONE,
        });

        let report = failed_observed_run_report(
            7,
            IMPEDANCE_STATUS_REPEATABILITY,
            0x20,
            ImpedanceReadbackMismatch::EMPTY,
            &observations,
        );

        assert_eq!(report.repeatability_permille, 22);
        assert_eq!(report.duplicate_zero_encoder_drift_counts, 1);
    }
}
