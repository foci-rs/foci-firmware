use crate::electrical_capture::{
    ElectricalCaptureBuffer, ElectricalCaptureSample, FitWindow, HIGH_RATE_CAPTURE_SAMPLE_CAPACITY,
    OnsetThreshold, RDropModel, detect_theta_onset_us, effective_frequency_hz, fit_current_slope,
    inductance_sensitivity,
};
use crate::tmc::{TmcRequest, TmcRequestKind, normalize_diagnostic_detail};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ABN_DECODER_PPR, MODE_RAMP_MODE_MOTION, PHI_E_EXT, PHI_E_SELECTION,
    PID_TORQUE_FLUX_ACTUAL, PIDOUT_UQ_UD_LIMITS, Register, STATUS_FLAGS, UQ_UD_EXT,
};

use super::helpers::{current_step_encoder_delta, pack_voltage_ext, split_signed_halves};
use super::{
    DiagnosticContext, DiagnosticDelay, DiagnosticReadiness, DiagnosticReplySink, DiagnosticReport,
    HIGH_RATE_CAPTURE_STATUS_BAD_PROFILE, HIGH_RATE_CAPTURE_STATUS_ESTIMATOR,
    HIGH_RATE_CAPTURE_STATUS_NOT_READY, HIGH_RATE_CAPTURE_STATUS_OK,
    HIGH_RATE_CAPTURE_STATUS_RESTORE, HIGH_RATE_CAPTURE_STATUS_SPI, HighRateCaptureProfileReport,
    HighRateCaptureRunReport, HighRateCaptureSampleReport, emit_diagnostic_report,
};

/// High-rate capture profile schema version.
pub const PROFILE_VERSION: u16 = 1;

const VALIDATION_PROFILE_ID: u8 = 0;
const VALIDATION_REQUESTED_SAMPLES: u16 = 256;
const VALIDATION_MAX_CAPTURE_US: u32 = 12_000;
const VALIDATION_UQ_EXT: i16 = 0;
const VALIDATION_UD_EXT: i16 = 1024;
const VALIDATION_PHI_E_EXT: u16 = 0;
const VALIDATION_VOLTAGE_LIMIT: u16 = 2048;
const CURRENT_NOISE_FLOOR_COUNT: u16 = 5;
const THETA_ONSET_DELTA_COUNT: u16 = 8;

/// TMC4671-LA Rev 1, 02/2024, MODE_RAMP_MODE_MOTION bits `[7:0]`:
/// `MODE_MOTION = 8` selects `UQ_UD_EXT` voltage-vector mode.
const OPEN_LOOP_VOLTAGE_MODE: u32 = 8;

/// TMC4671-LA Rev 1, 02/2024, PHI_E_SELECTION register:
/// value `1` selects external `PHI_E_EXT`.
const PHI_E_SELECTION_EXT: u32 = 1;

type CaptureBuffer = ElectricalCaptureBuffer<HIGH_RATE_CAPTURE_SAMPLE_CAPACITY>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HighRateCaptureProfile {
    profile_id: u8,
    profile_version: u16,
    sample_capacity: u16,
    requested_samples: u16,
    max_capture_us: u32,
    uq_ext: i16,
    ud_ext: i16,
    phi_e_ext: u16,
    voltage_limit: u16,
    r_count_slope_milli: i32,
}

impl HighRateCaptureProfile {
    fn validation_initial_slope() -> Self {
        Self {
            profile_id: VALIDATION_PROFILE_ID,
            profile_version: PROFILE_VERSION,
            sample_capacity: HIGH_RATE_CAPTURE_SAMPLE_CAPACITY as u16,
            requested_samples: VALIDATION_REQUESTED_SAMPLES,
            max_capture_us: VALIDATION_MAX_CAPTURE_US,
            uq_ext: VALIDATION_UQ_EXT,
            ud_ext: VALIDATION_UD_EXT,
            phi_e_ext: VALIDATION_PHI_E_EXT,
            voltage_limit: VALIDATION_VOLTAGE_LIMIT,
            r_count_slope_milli: 0,
        }
    }

    fn with_r_count_slope_milli(mut self, r_count_slope_milli: i32) -> Self {
        self.r_count_slope_milli = r_count_slope_milli;
        self
    }

    fn report(self, oid: u8) -> HighRateCaptureProfileReport {
        HighRateCaptureProfileReport {
            oid,
            profile_version: self.profile_version,
            sample_capacity: self.sample_capacity,
            requested_samples: self.requested_samples,
            max_capture_us: self.max_capture_us,
            uq_ext: self.uq_ext,
            ud_ext: self.ud_ext,
            phi_e_ext: self.phi_e_ext,
            voltage_limit: self.voltage_limit,
            r_count_slope_milli: self.r_count_slope_milli,
            flags: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SavedRegisters {
    mode_ramp_mode_motion: u32,
    phi_e_selection: u32,
    phi_e_ext: u32,
    uq_ud_ext: u32,
    pidout_uq_ud_limits: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RegisterWrite {
    register: Register,
    value: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CaptureResult {
    status: u8,
    samples: CaptureBuffer,
    elapsed_us: u32,
    max_sample_interval_us: u32,
    encoder_before: u32,
    encoder_after: u32,
    encoder_counts_per_rev: u32,
    status_flags_or: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct EstimateSummary {
    effective_frequency_hz: u32,
    l_nominal_us: u32,
    l_shift_minus_permille: u16,
    l_shift_plus_permille: u16,
    theta_onset_us: u32,
    residual_rms_count: u32,
    max_sample_interval_us: u32,
}

fn readiness_status(readiness: DiagnosticReadiness) -> Option<u8> {
    if readiness.homing_active || readiness.stepper_active || readiness.saved_voltage_limit == 0 {
        Some(HIGH_RATE_CAPTURE_STATUS_NOT_READY)
    } else {
        None
    }
}

fn rejected_report(
    oid: u8,
    status: u8,
    _profile: HighRateCaptureProfile,
    r_count_slope_milli: i32,
) -> DiagnosticReport {
    let profile = HighRateCaptureProfile::validation_initial_slope()
        .with_r_count_slope_milli(r_count_slope_milli);
    DiagnosticReport::HighRateCapture {
        profile: profile.report(oid),
        run: zero_run_report(oid, status, profile.profile_version),
    }
}

/// Return the temporary voltage limit used by the high-rate validation profile.
pub const fn high_rate_capture_voltage_limit() -> u16 {
    VALIDATION_VOLTAGE_LIMIT
}

/// Emit a terminal high-rate capture rejection for board-side readiness gates.
pub fn emit_high_rate_capture_not_ready<S: DiagnosticReplySink>(req: TmcRequest, sink: &mut S) {
    let TmcRequest {
        kind:
            TmcRequestKind::HighRateCaptureTest {
                oid,
                r_count_slope_milli,
                ..
            },
        ..
    } = req
    else {
        return;
    };

    emit_diagnostic_report(
        sink,
        rejected_report(
            oid,
            HIGH_RATE_CAPTURE_STATUS_NOT_READY,
            HighRateCaptureProfile::validation_initial_slope(),
            r_count_slope_milli,
        ),
    );
}

/// Run and emit a high-rate capture diagnostic request.
pub async fn run_high_rate_capture_request<B, D, S>(
    req: TmcRequest,
    mut ctx: DiagnosticContext<'_, B, D>,
    sink: &mut S,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
    S: DiagnosticReplySink,
{
    let TmcRequest {
        kind:
            TmcRequestKind::HighRateCaptureTest {
                oid,
                profile,
                detail,
                r_count_slope_milli,
            },
        ..
    } = req
    else {
        return;
    };

    let profile_cfg = HighRateCaptureProfile::validation_initial_slope()
        .with_r_count_slope_milli(r_count_slope_milli);

    if profile != VALIDATION_PROFILE_ID {
        emit_diagnostic_report(
            sink,
            rejected_report(
                oid,
                HIGH_RATE_CAPTURE_STATUS_BAD_PROFILE,
                profile_cfg,
                r_count_slope_milli,
            ),
        );
        return;
    }

    if !ctx.runtime.is_motor_enabled() || !ctx.runtime.is_calibrated() {
        emit_diagnostic_report(
            sink,
            rejected_report(
                oid,
                HIGH_RATE_CAPTURE_STATUS_NOT_READY,
                profile_cfg,
                r_count_slope_milli,
            ),
        );
        return;
    }

    if let Some(status) = readiness_status(ctx.readiness) {
        emit_diagnostic_report(
            sink,
            rejected_report(oid, status, profile_cfg, r_count_slope_milli),
        );
        return;
    }

    let capture = capture_profile(&mut ctx, profile_cfg).await;
    emit_capture_report(
        sink,
        oid,
        profile_cfg,
        normalize_diagnostic_detail(detail),
        &capture,
    );
}

async fn save_registers<B>(tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>) -> Result<SavedRegisters, ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    Ok(SavedRegisters {
        mode_ramp_mode_motion: tmc
            .read_register(MODE_RAMP_MODE_MOTION)
            .await
            .map_err(|_| ())?,
        phi_e_selection: tmc.read_register(PHI_E_SELECTION).await.map_err(|_| ())?,
        phi_e_ext: tmc.read_register(PHI_E_EXT).await.map_err(|_| ())?,
        uq_ud_ext: tmc.read_register(UQ_UD_EXT).await.map_err(|_| ())?,
        pidout_uq_ud_limits: tmc
            .read_register(PIDOUT_UQ_UD_LIMITS)
            .await
            .map_err(|_| ())?,
    })
}

async fn restore_registers<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    saved: SavedRegisters,
) -> Result<(), ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    let mut ok = true;
    for write in restore_register_sequence(saved) {
        ok &= tmc
            .write_register(write.register, write.value)
            .await
            .is_ok();
    }

    if ok { Ok(()) } else { Err(()) }
}

fn restore_register_sequence(saved: SavedRegisters) -> [RegisterWrite; 7] {
    [
        RegisterWrite {
            register: UQ_UD_EXT,
            value: 0,
        },
        RegisterWrite {
            register: MODE_RAMP_MODE_MOTION,
            value: 0,
        },
        RegisterWrite {
            register: PHI_E_EXT,
            value: saved.phi_e_ext,
        },
        RegisterWrite {
            register: PHI_E_SELECTION,
            value: saved.phi_e_selection,
        },
        RegisterWrite {
            register: PIDOUT_UQ_UD_LIMITS,
            value: saved.pidout_uq_ud_limits,
        },
        RegisterWrite {
            register: UQ_UD_EXT,
            value: saved.uq_ud_ext,
        },
        RegisterWrite {
            register: MODE_RAMP_MODE_MOTION,
            value: saved.mode_ramp_mode_motion,
        },
    ]
}

async fn read_capture_sample<B, D>(
    ctx: &mut DiagnosticContext<'_, B, D>,
    start_us: u64,
    uq_ext: i16,
    ud_ext: i16,
    primary_is_flux: bool,
) -> Result<ElectricalCaptureSample, ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    let actual = ctx
        .tmc
        .read_register(PID_TORQUE_FLUX_ACTUAL)
        .await
        .map_err(|_| ())?;
    let read_end_us = ctx.delay.now_us();
    let timestamp_us = read_end_us.checked_sub(start_us).ok_or(())?;
    let (torque, flux) = split_signed_halves(actual);
    let (current_primary, current_secondary) = if primary_is_flux {
        (flux, torque)
    } else {
        (torque, flux)
    };

    Ok(ElectricalCaptureSample::new(
        timestamp_us,
        uq_ext,
        ud_ext,
        current_primary,
        current_secondary,
        0,
        0,
    ))
}

async fn capture_profile<B, D>(
    ctx: &mut DiagnosticContext<'_, B, D>,
    profile: HighRateCaptureProfile,
) -> CaptureResult
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    let mut result = CaptureResult {
        status: HIGH_RATE_CAPTURE_STATUS_SPI,
        samples: CaptureBuffer::new(),
        elapsed_us: 0,
        max_sample_interval_us: 0,
        encoder_before: 0,
        encoder_after: 0,
        encoder_counts_per_rev: 0,
        status_flags_or: 0,
    };

    let saved = match save_registers(ctx.tmc).await {
        Ok(saved) => saved,
        Err(()) => return result,
    };

    let capture_status = match run_capture_window(ctx, profile, &mut result).await {
        Ok(()) => HIGH_RATE_CAPTURE_STATUS_OK,
        Err(()) => HIGH_RATE_CAPTURE_STATUS_SPI,
    };
    finalize_capture_timing(&mut result);

    let zero_result = ctx.tmc.write_register(UQ_UD_EXT, 0).await;
    let restore_result = restore_registers(ctx.tmc, saved).await;
    result.status = if zero_result.is_err() || restore_result.is_err() {
        HIGH_RATE_CAPTURE_STATUS_RESTORE
    } else {
        capture_status
    };
    result
}

async fn run_capture_window<B, D>(
    ctx: &mut DiagnosticContext<'_, B, D>,
    profile: HighRateCaptureProfile,
    result: &mut CaptureResult,
) -> Result<(), ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    if !ctx.runtime.restore_phase_advance_now(ctx.tmc).await {
        return Err(());
    }

    result.encoder_before = ctx
        .tmc
        .read_register(ABN_DECODER_COUNT)
        .await
        .map_err(|_| ())?;
    result.encoder_counts_per_rev = ctx
        .tmc
        .read_register(ABN_DECODER_PPR)
        .await
        .map_err(|_| ())?;
    let status_before = ctx.tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
    ctx.tmc
        .write_register(MODE_RAMP_MODE_MOTION, 0)
        .await
        .map_err(|_| ())?;
    ctx.tmc.write_register(UQ_UD_EXT, 0).await.map_err(|_| ())?;
    ctx.tmc
        .write_register(PIDOUT_UQ_UD_LIMITS, u32::from(profile.voltage_limit))
        .await
        .map_err(|_| ())?;
    ctx.tmc
        .write_register(PHI_E_EXT, u32::from(profile.phi_e_ext))
        .await
        .map_err(|_| ())?;
    ctx.tmc
        .write_register(PHI_E_SELECTION, PHI_E_SELECTION_EXT)
        .await
        .map_err(|_| ())?;
    ctx.tmc
        .write_register(MODE_RAMP_MODE_MOTION, OPEN_LOOP_VOLTAGE_MODE)
        .await
        .map_err(|_| ())?;
    ctx.tmc
        .write_register(PIDOUT_UQ_UD_LIMITS, u32::from(profile.voltage_limit))
        .await
        .map_err(|_| ())?;
    ctx.tmc
        .write_register(STATUS_FLAGS, 0)
        .await
        .map_err(|_| ())?;

    let start_us = ctx.delay.now_us();
    let primary_is_flux = profile.ud_ext != 0;
    let baseline = read_capture_sample(ctx, start_us, 0, 0, primary_is_flux).await?;
    push_capture_sample(result, baseline)?;

    ctx.tmc
        .write_register(UQ_UD_EXT, pack_voltage_ext(profile.uq_ext, profile.ud_ext))
        .await
        .map_err(|_| ())?;

    while result.samples.len() < usize::from(profile.requested_samples) {
        let now_us = ctx.delay.now_us();
        if elapsed_since_start_us(start_us, now_us)? >= profile.max_capture_us {
            break;
        }
        let sample = read_capture_sample(
            ctx,
            start_us,
            profile.uq_ext,
            profile.ud_ext,
            primary_is_flux,
        )
        .await?;
        let timestamp_us = u32::try_from(sample.timestamp_us).map_err(|_| ())?;
        push_capture_sample(result, sample)?;
        if timestamp_us >= profile.max_capture_us {
            break;
        }
    }

    result.encoder_after = ctx
        .tmc
        .read_register(ABN_DECODER_COUNT)
        .await
        .map_err(|_| ())?;
    let status_after = ctx.tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
    result.status_flags_or = status_before | status_after;
    Ok(())
}

fn push_capture_sample(
    result: &mut CaptureResult,
    sample: ElectricalCaptureSample,
) -> Result<(), ()> {
    if let Some(previous) = result.samples.as_slice().last() {
        let interval = sample
            .timestamp_us
            .checked_sub(previous.timestamp_us)
            .ok_or(())?;
        let interval = u32::try_from(interval).map_err(|_| ())?;
        result.max_sample_interval_us = result.max_sample_interval_us.max(interval);
    }

    result.elapsed_us = u32::try_from(sample.timestamp_us).map_err(|_| ())?;
    result.samples.push(sample).map_err(|_| ())
}

fn finalize_capture_timing(capture: &mut CaptureResult) {
    if let Some(last) = capture.samples.as_slice().last() {
        capture.elapsed_us = u32::try_from(last.timestamp_us).unwrap_or(u32::MAX);
    }
}

fn elapsed_since_start_us(start_us: u64, now_us: u64) -> Result<u32, ()> {
    u32::try_from(now_us.checked_sub(start_us).ok_or(())?).map_err(|_| ())
}

fn build_estimate(
    samples: &[ElectricalCaptureSample],
    profile: HighRateCaptureProfile,
) -> Option<EstimateSummary> {
    let fit = fit_current_slope(
        samples,
        FitWindow {
            start_index: 1,
            end_index: samples.len(),
        },
    )?;
    let frequency = effective_frequency_hz(
        fit.slope_count_per_s,
        fit.mean_current_count,
        CURRENT_NOISE_FLOOR_COUNT,
    )?;
    let inductance = inductance_sensitivity(
        fit,
        RDropModel {
            r_count_slope_milli: profile.r_count_slope_milli,
            ud_count: profile.ud_ext,
            perturb_permille: crate::electrical_capture::R_SENSITIVITY_PERTURB_PERMILLE,
        },
    )?;
    let theta_onset_us = u32::try_from(detect_theta_onset_us(
        samples,
        OnsetThreshold {
            delta_count: THETA_ONSET_DELTA_COUNT,
        },
    )?)
    .ok()?;
    let max_sample_interval_us = max_sample_interval_us(samples)?;

    Some(EstimateSummary {
        effective_frequency_hz: frequency,
        l_nominal_us: inductance.l_nominal_us,
        l_shift_minus_permille: inductance.l_shift_minus_permille,
        l_shift_plus_permille: inductance.l_shift_plus_permille,
        theta_onset_us,
        residual_rms_count: fit.residual_rms_count,
        max_sample_interval_us,
    })
}

fn max_sample_interval_us(samples: &[ElectricalCaptureSample]) -> Option<u32> {
    let mut max_interval_us = 0_u32;
    for pair in samples.windows(2) {
        let interval = pair[1].timestamp_us.checked_sub(pair[0].timestamp_us)?;
        let interval = u32::try_from(interval).ok()?;
        max_interval_us = max_interval_us.max(interval);
    }
    Some(max_interval_us)
}

fn emit_sample_reports<S: DiagnosticReplySink>(
    sink: &mut S,
    oid: u8,
    samples: &[ElectricalCaptureSample],
    detail: u8,
) {
    if detail < 2 {
        return;
    }

    for (index, sample) in samples.iter().enumerate() {
        let Ok(sample_index) = u16::try_from(index) else {
            break;
        };
        sink.foci_high_rate_capture_sample(HighRateCaptureSampleReport {
            oid,
            sample_index,
            timestamp_us: u32::try_from(sample.timestamp_us).unwrap_or(u32::MAX),
            uq_ext: sample.uq_ext,
            ud_ext: sample.ud_ext,
            current_primary: sample.current_primary,
            current_secondary: sample.current_secondary,
            encoder_count: sample.encoder_count,
            status_flags: sample.status_flags,
        });
    }
}

fn build_report(
    oid: u8,
    profile: HighRateCaptureProfile,
    capture: &CaptureResult,
) -> DiagnosticReport {
    DiagnosticReport::HighRateCapture {
        profile: profile.report(oid),
        run: build_run_report(oid, profile, capture),
    }
}

fn build_run_report(
    oid: u8,
    profile: HighRateCaptureProfile,
    capture: &CaptureResult,
) -> HighRateCaptureRunReport {
    let samples = capture.samples.as_slice();
    let mut run = zero_run_report(oid, capture.status, profile.profile_version);
    run.sample_count = u16::try_from(samples.len()).unwrap_or(u16::MAX);
    run.elapsed_us = capture.elapsed_us;
    run.encoder_delta = current_step_encoder_delta(
        capture.encoder_before,
        capture.encoder_after,
        capture.encoder_counts_per_rev,
    );
    run.status_flags_or = capture.status_flags_or
        | samples
            .iter()
            .fold(0_u32, |flags, sample| flags | sample.status_flags);
    run.max_sample_interval_us = capture.max_sample_interval_us;

    if capture.status != HIGH_RATE_CAPTURE_STATUS_OK {
        return run;
    }

    match build_estimate(samples, profile) {
        Some(estimate) => {
            run.effective_frequency_hz = estimate.effective_frequency_hz;
            run.l_nominal_us = estimate.l_nominal_us;
            run.l_shift_minus_permille = estimate.l_shift_minus_permille;
            run.l_shift_plus_permille = estimate.l_shift_plus_permille;
            run.theta_onset_us = estimate.theta_onset_us;
            run.residual_rms_count = estimate.residual_rms_count;
            run.max_sample_interval_us = estimate.max_sample_interval_us;
        }
        None => {
            run.status = HIGH_RATE_CAPTURE_STATUS_ESTIMATOR;
        }
    }

    run
}

fn zero_run_report(oid: u8, status: u8, profile_version: u16) -> HighRateCaptureRunReport {
    HighRateCaptureRunReport {
        oid,
        status,
        profile_version,
        sample_count: 0,
        elapsed_us: 0,
        effective_frequency_hz: 0,
        l_nominal_us: 0,
        l_shift_minus_permille: 0,
        l_shift_plus_permille: 0,
        theta_onset_us: 0,
        residual_rms_count: 0,
        max_sample_interval_us: 0,
        encoder_delta: 0,
        status_flags_or: 0,
        warning_flags: 0,
    }
}

fn emit_capture_report<S: DiagnosticReplySink>(
    sink: &mut S,
    oid: u8,
    profile: HighRateCaptureProfile,
    detail: u8,
    capture: &CaptureResult,
) {
    let report = build_report(oid, profile, capture);
    let DiagnosticReport::HighRateCapture { profile, run } = report else {
        unreachable!();
    };
    sink.foci_high_rate_capture_profile(profile);
    emit_sample_reports(sink, oid, capture.samples.as_slice(), detail);
    sink.foci_high_rate_capture_run(run);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::electrical_capture::{ElectricalCaptureBuffer, ElectricalCaptureSample};
    use crate::tmc::TmcRequestKind;
    use crate::tmc_diagnostics::{
        CurrentStepReport, CurrentTorqueSampleReport, DiagnosticReadiness, DiagnosticReplySink,
        HIGH_RATE_CAPTURE_STATUS_BAD_PROFILE, HIGH_RATE_CAPTURE_STATUS_NOT_READY,
        HIGH_RATE_CAPTURE_STATUS_OK, HIGH_RATE_CAPTURE_STATUS_SPI, HighRateCaptureProfileReport,
        HighRateCaptureRunReport, HighRateCaptureSampleReport, ResistanceAxisReport,
        ResistanceProfileReport, ResistanceRunReport, VoltageStepReport,
    };
    use std::vec::Vec;

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Event {
        Profile {
            oid: u8,
            requested_samples: u16,
            max_capture_us: u32,
        },
        Sample {
            oid: u8,
            sample_index: u16,
            timestamp_us: u32,
            current_primary: i16,
        },
        Run {
            oid: u8,
            status: u8,
            sample_count: u16,
        },
    }

    #[derive(Default)]
    struct RecordingSink {
        events: Vec<Event>,
    }

    impl DiagnosticReplySink for RecordingSink {
        fn foci_current_step_result(&mut self, _report: CurrentStepReport) {}

        fn foci_current_vector_step_result(&mut self, _report: CurrentStepReport) {}

        fn foci_current_torque_sample_result(&mut self, _report: CurrentTorqueSampleReport) {}

        fn foci_current_torque_sample_detail_result(
            &mut self,
            _report: &CurrentTorqueSampleReport,
        ) {
        }

        fn foci_voltage_step_result(&mut self, _report: VoltageStepReport) {}

        fn foci_resistance_profile(&mut self, _report: ResistanceProfileReport) {}

        fn foci_resistance_axis(&mut self, _report: ResistanceAxisReport) {}

        fn foci_resistance_run(&mut self, _report: ResistanceRunReport) {}

        fn foci_high_rate_capture_profile(&mut self, report: HighRateCaptureProfileReport) {
            self.events.push(Event::Profile {
                oid: report.oid,
                requested_samples: report.requested_samples,
                max_capture_us: report.max_capture_us,
            });
        }

        fn foci_high_rate_capture_sample(&mut self, report: HighRateCaptureSampleReport) {
            self.events.push(Event::Sample {
                oid: report.oid,
                sample_index: report.sample_index,
                timestamp_us: report.timestamp_us,
                current_primary: report.current_primary,
            });
        }

        fn foci_high_rate_capture_run(&mut self, report: HighRateCaptureRunReport) {
            self.events.push(Event::Run {
                oid: report.oid,
                status: report.status,
                sample_count: report.sample_count,
            });
        }
    }

    #[test]
    fn validation_profile_has_bounded_exposure_and_capacity() {
        let profile = HighRateCaptureProfile::validation_initial_slope();
        let report = profile.report(9);

        assert_eq!(profile.profile_id, VALIDATION_PROFILE_ID);
        assert_eq!(profile.profile_version, PROFILE_VERSION);
        assert_eq!(
            profile.sample_capacity,
            HIGH_RATE_CAPTURE_SAMPLE_CAPACITY as u16
        );
        assert_eq!(profile.requested_samples, 256);
        assert_eq!(profile.max_capture_us, 12_000);
        assert!(usize::from(profile.requested_samples) <= HIGH_RATE_CAPTURE_SAMPLE_CAPACITY);
        assert_eq!(report.oid, 9);
        assert_eq!(report.ud_ext, 1024);
        assert_eq!(report.voltage_limit, high_rate_capture_voltage_limit());
    }

    #[test]
    fn bad_profile_returns_terminal_report() {
        let req = crate::tmc::TmcRequest::new(
            0,
            TmcRequestKind::HighRateCaptureTest {
                oid: 9,
                profile: VALIDATION_PROFILE_ID + 1,
                detail: 2,
                r_count_slope_milli: 1_706,
            },
        );
        let TmcRequestKind::HighRateCaptureTest {
            oid,
            profile,
            r_count_slope_milli,
            ..
        } = req.kind
        else {
            unreachable!();
        };
        let profile_cfg = HighRateCaptureProfile::validation_initial_slope()
            .with_r_count_slope_milli(r_count_slope_milli);

        assert_ne!(profile, profile_cfg.profile_id);
        let report = rejected_report(
            oid,
            HIGH_RATE_CAPTURE_STATUS_BAD_PROFILE,
            profile_cfg,
            r_count_slope_milli,
        );

        let super::DiagnosticReport::HighRateCapture { profile, run } = report else {
            panic!("expected high-rate capture report");
        };
        assert_eq!(profile.oid, 9);
        assert_eq!(profile.r_count_slope_milli, r_count_slope_milli);
        assert_eq!(run.oid, 9);
        assert_eq!(run.status, HIGH_RATE_CAPTURE_STATUS_BAD_PROFILE);
        assert_eq!(run.sample_count, 0);
    }

    #[test]
    fn readiness_rejects_homing_or_stepper_activity() {
        let ready = DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 1,
        };
        assert_eq!(readiness_status(ready), None);

        assert_eq!(
            readiness_status(DiagnosticReadiness {
                homing_active: true,
                ..ready
            }),
            Some(HIGH_RATE_CAPTURE_STATUS_NOT_READY)
        );
        assert_eq!(
            readiness_status(DiagnosticReadiness {
                stepper_active: true,
                ..ready
            }),
            Some(HIGH_RATE_CAPTURE_STATUS_NOT_READY)
        );
        assert_eq!(
            readiness_status(DiagnosticReadiness {
                saved_voltage_limit: 0,
                ..ready
            }),
            Some(HIGH_RATE_CAPTURE_STATUS_NOT_READY)
        );
    }

    #[test]
    fn restore_sequence_stops_mode_and_restores_saved_mode_last() {
        let saved = SavedRegisters {
            mode_ramp_mode_motion: 0x8000_0003,
            phi_e_selection: 2,
            phi_e_ext: 0x1234,
            uq_ud_ext: 0x5678_9abc,
            pidout_uq_ud_limits: 0x3456,
        };

        let sequence = restore_register_sequence(saved);
        let writes: Vec<(u8, u32)> = sequence
            .iter()
            .map(|write| (write.register.addr(), write.value))
            .collect();

        assert_eq!(
            writes,
            std::vec![
                (UQ_UD_EXT.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (PHI_E_EXT.addr(), saved.phi_e_ext),
                (PHI_E_SELECTION.addr(), saved.phi_e_selection),
                (PIDOUT_UQ_UD_LIMITS.addr(), saved.pidout_uq_ud_limits),
                (UQ_UD_EXT.addr(), saved.uq_ud_ext),
                (MODE_RAMP_MODE_MOTION.addr(), saved.mode_ramp_mode_motion),
            ]
        );
    }

    #[test]
    fn failure_report_preserves_sample_timing_evidence() {
        let profile = HighRateCaptureProfile::validation_initial_slope();
        let mut samples = ElectricalCaptureBuffer::new();
        for (timestamp_us, current) in [(0_u64, 0_i16), (80, 9), (250, 18)] {
            samples
                .push(ElectricalCaptureSample::new(
                    timestamp_us,
                    profile.uq_ext,
                    profile.ud_ext,
                    current,
                    0,
                    0,
                    0,
                ))
                .expect("sample should fit in test buffer");
        }
        let capture = CaptureResult {
            status: HIGH_RATE_CAPTURE_STATUS_SPI,
            samples,
            elapsed_us: 250,
            max_sample_interval_us: 170,
            encoder_before: 0,
            encoder_after: 0,
            encoder_counts_per_rev: 0,
            status_flags_or: 0,
        };

        let run = build_run_report(9, profile, &capture);

        assert_eq!(run.status, HIGH_RATE_CAPTURE_STATUS_SPI);
        assert_eq!(run.sample_count, 3);
        assert_eq!(run.elapsed_us, 250);
        assert_eq!(run.max_sample_interval_us, 170);
    }

    #[test]
    fn detail_two_streams_samples_between_profile_and_run() {
        let profile = HighRateCaptureProfile::validation_initial_slope();
        let mut samples = ElectricalCaptureBuffer::new();
        for (index, current) in [0_i16, 10, 20, 30].into_iter().enumerate() {
            samples
                .push(ElectricalCaptureSample::new(
                    (index as u64) * 100,
                    profile.uq_ext,
                    profile.ud_ext,
                    current,
                    0,
                    0,
                    0,
                ))
                .expect("sample should fit in test buffer");
        }
        let capture = CaptureResult {
            status: HIGH_RATE_CAPTURE_STATUS_OK,
            samples,
            elapsed_us: 300,
            max_sample_interval_us: 100,
            encoder_before: 0,
            encoder_after: 0,
            encoder_counts_per_rev: 0,
            status_flags_or: 0,
        };
        let mut sink = RecordingSink::default();

        emit_capture_report(&mut sink, 9, profile, 2, &capture);

        assert_eq!(
            sink.events,
            std::vec![
                Event::Profile {
                    oid: 9,
                    requested_samples: 256,
                    max_capture_us: 12_000,
                },
                Event::Sample {
                    oid: 9,
                    sample_index: 0,
                    timestamp_us: 0,
                    current_primary: 0,
                },
                Event::Sample {
                    oid: 9,
                    sample_index: 1,
                    timestamp_us: 100,
                    current_primary: 10,
                },
                Event::Sample {
                    oid: 9,
                    sample_index: 2,
                    timestamp_us: 200,
                    current_primary: 20,
                },
                Event::Sample {
                    oid: 9,
                    sample_index: 3,
                    timestamp_us: 300,
                    current_primary: 30,
                },
                Event::Run {
                    oid: 9,
                    status: HIGH_RATE_CAPTURE_STATUS_OK,
                    sample_count: 4,
                },
            ]
        );
    }
}
