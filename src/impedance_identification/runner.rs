//! Safe register-sequenced impedance-identification runner.
//!
//! This module provides the shared firmware logic used by the active impedance
//! diagnostic dispatch. It centralizes the register sequencing and safety
//! gates, while hardware validation remains a separate workflow.

use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, DSADC_MDEC_B_MDEC_A, MODE_RAMP_MODE_MOTION,
    MOTOR_TYPE_N_POLE_PAIRS, MotionMode, MotorType, MotorTypeNPolePairs, OPENLOOP_ACCELERATION,
    OPENLOOP_MODE, OPENLOOP_PHI, OPENLOOP_VELOCITY_ACTUAL, OPENLOOP_VELOCITY_TARGET, PHI_E_EXT,
    PHI_E_SELECTION, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, STATUS_FLAGS,
    UQ_UD_EXT, UqUdExt,
};

use super::fit::{ImpedanceFitPoint, ImpedanceFitSummary};
use super::profile::ImpedanceProfile;
use super::types::{
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceError, ImpedancePowerStageReadback,
    ImpedanceSample, ImpedanceWarningFlags,
};
use crate::impedance_identification::fit::{evaluate_impedance_result, fit_impedance_point};

const PHI_E_EXTERNAL: u32 = 1;
const PHI_E_OPEN_LOOP: u32 = 2;
const OPENLOOP_PHI_DIRECTION_POSITIVE: u32 = 0;
const MIN_SAMPLE_INTERVAL_US: u16 = 1_000;

const EMPTY_FIT: ImpedanceFitPoint = ImpedanceFitPoint {
    axis_max_count_per_ud_milli: 0,
    axis_min_count_per_ud_milli: 0,
    axis_angle_electrical_counts: 0,
    residual_permille: 0,
    coverage_permille: 0,
    scalar_2f_mismatch_permille: 0,
    warning_flags: ImpedanceWarningFlags::NONE,
};

/// Error returned by backend register I/O during impedance identification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ImpedanceBackendError {
    /// SPI transaction failed.
    Spi,
    /// Backend refused the operation because the motor is not armed.
    NotArmed,
}

/// Board glue required by the impedance runner.
#[allow(async_fn_in_trait)]
pub trait ImpedanceBackend {
    /// Read a raw TMC4671 register by 7-bit address.
    async fn read_register(&mut self, addr: u8) -> Result<u32, ImpedanceBackendError>;
    /// Write a raw TMC4671 register by 7-bit address.
    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ImpedanceBackendError>;
    /// Delay for the requested number of microseconds.
    async fn delay_us(&mut self, us: u32);
    /// Delay for the requested number of milliseconds.
    async fn delay_ms(&mut self, ms: u64);
    /// Monotonic microsecond timestamp.
    fn timestamp_us(&self) -> u32;
    /// Whether the motor is already armed for externally managed open-loop voltage.
    fn is_armed_for_open_loop_voltage(&self) -> bool;
    /// Board-specific impedance-identification profile.
    fn profile(&self) -> ImpedanceProfile;
}

/// Complete runner output returned through the active diagnostic surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceRunResult {
    /// Profile used for the run.
    pub profile: ImpedanceProfile,
    /// Per-injection fitted impedance points.
    pub point_fits: [ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS],
    /// Aggregate impedance summary.
    pub summary: ImpedanceFitSummary,
    /// Aggregate warning flags from fit, status, and encoder gates.
    pub warning_flags: ImpedanceWarningFlags,
    /// OR of every sampled `STATUS_FLAGS` value.
    pub status_flags_or: u32,
    /// Power-stage readbacks verified before the first nonzero voltage command.
    pub power_stage_readback: ImpedancePowerStageReadback,
}

/// Run impedance identification over an already armed motor.
///
/// All pre-energize validation failures return before any register write and
/// before safe-state restore. Every failure after the energized setup begins
/// runs the shared restore sequence before returning.
pub async fn run_impedance_identification<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<ImpedanceRunResult, ImpedanceError> {
    if !backend.is_armed_for_open_loop_voltage() {
        return Err(ImpedanceError::NotArmed);
    }

    let saved_voltage_limit = match backend.read_register(PIDOUT_UQ_UD_LIMITS.addr()).await {
        Ok(value) => value,
        Err(_) => return Err(ImpedanceError::BackendIo),
    };

    let profile = backend.profile();
    if pidout_ud_limit(saved_voltage_limit) < profile.max_abs_ud() {
        return Err(ImpedanceError::InsufficientVoltageAuthority);
    }

    let motor_type_raw = match backend.read_register(MOTOR_TYPE_N_POLE_PAIRS.addr()).await {
        Ok(value) => value,
        Err(_) => return Err(ImpedanceError::BackendIo),
    };
    let motor_type = MotorTypeNPolePairs::from(motor_type_raw).motor_type();
    if motor_type != MotorType::TwoPhase as u8 {
        return Err(ImpedanceError::UnsupportedMotorType);
    }

    validate_profile(&profile)?;

    let body_result = run_energized_body(backend, profile).await;
    let restore_result = restore_safe_state(backend, saved_voltage_limit).await;

    match (body_result, restore_result) {
        (Ok(result), Ok(())) => Ok(result),
        (_, Err(())) => Err(ImpedanceError::RestoreFailed),
        (Err(error), Ok(())) => Err(error),
    }
}

fn validate_profile(profile: &ImpedanceProfile) -> Result<(), ImpedanceError> {
    if profile.sample_count as usize != IMPEDANCE_SAMPLE_COUNT
        || profile.sample_count > profile.max_sample_count
        || profile.sample_interval_us < MIN_SAMPLE_INTERVAL_US
        || profile.sample_interval_us % MIN_SAMPLE_INTERVAL_US != 0
        || profile.max_expected_current_count == 0
        || profile.encoder_move_fail_counts == 0
        || profile.encoder_move_warn_counts > profile.encoder_move_fail_counts
        || profile.max_abs_ud() == 0
        || profile.max_abs_ud() > i16::MAX as u16
        || profile.max_window_exposure() > profile.max_window_exposure_budget
        || profile.max_total_exposure() > profile.max_total_exposure_budget
    {
        return Err(ImpedanceError::UnsupportedProfile);
    }

    for point in profile.injection_points {
        if point.ud_abs == 0
            || point.ud_abs > i16::MAX as u16
            || point.frequency_millihz == 0
            || point.openloop_velocity_rpm <= 0
            || point.openloop_acceleration_rpm_per_s == 0
        {
            return Err(ImpedanceError::UnsupportedProfile);
        }
    }

    Ok(())
}

async fn run_energized_body<B: ImpedanceBackend>(
    backend: &mut B,
    profile: ImpedanceProfile,
) -> Result<ImpedanceRunResult, ImpedanceError> {
    let power_stage_readback = prepare_zero_voltage_setup(backend, &profile).await?;

    let mut warning_flags = ImpedanceWarningFlags::NONE;
    let mut status_flags_or = 0u32;
    let mut point_fits = [EMPTY_FIT; IMPEDANCE_PROFILE_POINTS];

    for (index, point) in profile.injection_points.iter().copied().enumerate() {
        run_dc_baseline_check(
            backend,
            &profile,
            point,
            &mut warning_flags,
            &mut status_flags_or,
        )
        .await?;

        let samples = capture_ac_window(
            backend,
            &profile,
            point,
            &mut warning_flags,
            &mut status_flags_or,
        )
        .await?;
        write(backend, UQ_UD_EXT.addr(), 0).await?;
        write(
            backend,
            MODE_RAMP_MODE_MOTION.addr(),
            MotionMode::Stopped as u32,
        )
        .await?;

        point_fits[index] = fit_impedance_point(&profile, point, &samples)?;
        warning_flags.insert(point_fits[index].warning_flags);
    }

    let mut summary = evaluate_impedance_result(&profile, &point_fits)?;
    summary.warning_flags.insert(warning_flags);
    warning_flags = summary.warning_flags;

    Ok(ImpedanceRunResult {
        profile,
        point_fits,
        summary,
        warning_flags,
        status_flags_or,
        power_stage_readback,
    })
}

async fn prepare_zero_voltage_setup<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
) -> Result<ImpedancePowerStageReadback, ImpedanceError> {
    write(
        backend,
        PIDOUT_UQ_UD_LIMITS.addr(),
        pack_pidout_limits(profile.max_abs_ud()),
    )
    .await?;
    write(backend, UQ_UD_EXT.addr(), 0).await?;
    stop_openloop_generator(backend).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    write(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    write(backend, PHI_E_EXT.addr(), 0).await?;

    verify_readback(
        backend,
        PIDOUT_UQ_UD_LIMITS.addr(),
        pack_pidout_limits(profile.max_abs_ud()),
    )
    .await?;
    verify_readback(backend, UQ_UD_EXT.addr(), 0).await?;
    verify_readback(
        backend,
        OPENLOOP_MODE.addr(),
        OPENLOOP_PHI_DIRECTION_POSITIVE,
    )
    .await?;
    verify_readback(backend, OPENLOOP_ACCELERATION.addr(), 0).await?;
    verify_readback(backend, OPENLOOP_VELOCITY_TARGET.addr(), 0).await?;
    verify_readback(backend, OPENLOOP_VELOCITY_ACTUAL.addr(), 0).await?;
    verify_readback(backend, OPENLOOP_PHI.addr(), 0).await?;
    verify_readback(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    verify_readback(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    verify_readback(backend, PHI_E_EXT.addr(), 0).await?;

    write(backend, PWM_SV_CHOP.addr(), profile.pwm_sv_chop).await?;
    let power_stage_readback = read_power_stage_setup(backend).await?;
    if power_stage_readback.pwm_sv_chop_readback != profile.pwm_sv_chop
        || power_stage_readback.pwm_maxcnt_readback != u32::from(profile.pwm_maxcnt)
        || power_stage_readback.bbm_readback != expected_bbm_readback(profile)
        || power_stage_readback.dsadc_mdec_readback != expected_dsadc_readback(profile)
    {
        Err(ImpedanceError::ProfileReadbackMismatch)
    } else {
        Ok(power_stage_readback)
    }
}

async fn read_power_stage_setup<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<ImpedancePowerStageReadback, ImpedanceError> {
    Ok(ImpedancePowerStageReadback {
        pwm_maxcnt_readback: read(backend, PWM_MAXCNT.addr()).await?,
        bbm_readback: read(backend, PWM_BBM_H_BBM_L.addr()).await?,
        dsadc_mdec_readback: read(backend, DSADC_MDEC_B_MDEC_A.addr()).await?,
        pwm_sv_chop_readback: read(backend, PWM_SV_CHOP.addr()).await?,
    })
}

fn expected_bbm_readback(profile: &ImpedanceProfile) -> u32 {
    (u32::from(profile.bbm_h) << 8) | u32::from(profile.bbm_l)
}

fn expected_dsadc_readback(profile: &ImpedanceProfile) -> u32 {
    (u32::from(profile.dsadc_mdec_b) << 16) | u32::from(profile.dsadc_mdec_a)
}

async fn run_dc_baseline_check<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
    point: super::types::ImpedanceInjectionPoint,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<(), ImpedanceError> {
    let encoder_before = read(backend, ABN_DECODER_COUNT.addr()).await?;

    write(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    write(backend, PHI_E_EXT.addr(), 0).await?;
    verify_readback(backend, UQ_UD_EXT.addr(), 0).await?;
    verify_readback(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    verify_readback(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    verify_readback(backend, PHI_E_EXT.addr(), 0).await?;
    sample_status(backend, profile, warning_flags, status_flags_or).await?;
    write(backend, UQ_UD_EXT.addr(), pack_ud(point.ud_abs as i16)).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::UqUdExt as u32,
    )
    .await?;

    backend.delay_ms(u64::from(point.settle_ms)).await;

    let packed = read(backend, ADC_IWY_IUX.addr()).await?;
    let sample = decode_adc_sample(0, packed);
    check_current_limit(profile, sample)?;
    sample_status(backend, profile, warning_flags, status_flags_or).await?;

    let encoder_after = read(backend, ABN_DECODER_COUNT.addr()).await?;
    check_encoder_delta(profile, encoder_before, encoder_after, warning_flags)
}

async fn capture_ac_window<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
    point: super::types::ImpedanceInjectionPoint,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT], ImpedanceError> {
    let encoder_before = read(backend, ABN_DECODER_COUNT.addr()).await?;

    write(backend, UQ_UD_EXT.addr(), 0).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    write(backend, OPENLOOP_PHI.addr(), 0).await?;
    write(
        backend,
        OPENLOOP_ACCELERATION.addr(),
        point.openloop_acceleration_rpm_per_s,
    )
    .await?;
    write(
        backend,
        OPENLOOP_VELOCITY_TARGET.addr(),
        point.openloop_velocity_rpm as u32,
    )
    .await?;
    write(
        backend,
        OPENLOOP_MODE.addr(),
        OPENLOOP_PHI_DIRECTION_POSITIVE,
    )
    .await?;
    write(backend, PHI_E_SELECTION.addr(), PHI_E_OPEN_LOOP).await?;
    verify_readback(backend, UQ_UD_EXT.addr(), 0).await?;
    verify_readback(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    verify_readback(backend, PHI_E_SELECTION.addr(), PHI_E_OPEN_LOOP).await?;
    verify_readback(
        backend,
        OPENLOOP_ACCELERATION.addr(),
        point.openloop_acceleration_rpm_per_s,
    )
    .await?;
    verify_readback(
        backend,
        OPENLOOP_VELOCITY_TARGET.addr(),
        point.openloop_velocity_rpm as u32,
    )
    .await?;
    verify_readback(
        backend,
        OPENLOOP_MODE.addr(),
        OPENLOOP_PHI_DIRECTION_POSITIVE,
    )
    .await?;
    sample_status(backend, profile, warning_flags, status_flags_or).await?;
    write(backend, UQ_UD_EXT.addr(), pack_ud(point.ud_abs as i16)).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::UqUdExt as u32,
    )
    .await?;

    backend.delay_ms(u64::from(point.settle_ms)).await;

    let start_us = backend.timestamp_us();
    let mut samples = [ImpedanceSample {
        t_us: 0,
        i_ch0: 0,
        i_ch1: 0,
    }; IMPEDANCE_SAMPLE_COUNT];
    let sample_count = profile.sample_count as usize;
    for (index, sample) in samples.iter_mut().take(sample_count).enumerate() {
        let packed = read(backend, ADC_IWY_IUX.addr()).await?;
        let decoded = decode_adc_sample(backend.timestamp_us().wrapping_sub(start_us), packed);
        check_current_limit(profile, decoded)?;
        *sample = decoded;
        sample_status(backend, profile, warning_flags, status_flags_or).await?;
        if index + 1 < sample_count {
            backend
                .delay_us(u32::from(profile.sample_interval_us))
                .await;
        }
    }

    let encoder_after = read(backend, ABN_DECODER_COUNT.addr()).await?;
    check_encoder_delta(profile, encoder_before, encoder_after, warning_flags)?;

    Ok(samples)
}

async fn sample_status<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<(), ImpedanceError> {
    let status_flags = read(backend, STATUS_FLAGS.addr()).await?;
    *status_flags_or |= status_flags;
    if status_flags & profile.status_flags_warn_mask != 0 {
        warning_flags.insert(ImpedanceWarningFlags::STATUS_FLAGS);
    }
    if status_flags & profile.status_flags_fail_mask != 0 {
        return Err(ImpedanceError::UnsafeStatusFlags);
    }
    Ok(())
}

fn check_current_limit(
    profile: &ImpedanceProfile,
    sample: ImpedanceSample,
) -> Result<(), ImpedanceError> {
    let max_abs = i32::from(sample.i_ch0)
        .unsigned_abs()
        .max(i32::from(sample.i_ch1).unsigned_abs());
    if max_abs > u32::from(profile.max_expected_current_count) {
        Err(ImpedanceError::CurrentLimitExceeded)
    } else {
        Ok(())
    }
}

fn check_encoder_delta(
    profile: &ImpedanceProfile,
    before: u32,
    after: u32,
    warning_flags: &mut ImpedanceWarningFlags,
) -> Result<(), ImpedanceError> {
    let delta = after.wrapping_sub(before) as i32;
    let delta_abs = delta.unsigned_abs();
    if delta_abs > u32::from(profile.encoder_move_fail_counts) {
        return Err(ImpedanceError::EncoderMoved);
    }
    if delta_abs > u32::from(profile.encoder_move_warn_counts) {
        warning_flags.insert(ImpedanceWarningFlags::ENCODER_MOVED);
    }
    Ok(())
}

async fn restore_safe_state<B: ImpedanceBackend>(
    backend: &mut B,
    saved_voltage_limit: u32,
) -> Result<(), ()> {
    let mut failed = false;
    let writes = [
        (UQ_UD_EXT.addr(), 0),
        (OPENLOOP_VELOCITY_TARGET.addr(), 0),
        (OPENLOOP_VELOCITY_ACTUAL.addr(), 0),
        (OPENLOOP_ACCELERATION.addr(), 0),
        (OPENLOOP_PHI.addr(), 0),
        (OPENLOOP_MODE.addr(), OPENLOOP_PHI_DIRECTION_POSITIVE),
        (MODE_RAMP_MODE_MOTION.addr(), MotionMode::Stopped as u32),
        (PHI_E_SELECTION.addr(), 0),
        (PWM_SV_CHOP.addr(), 0),
        (PIDOUT_UQ_UD_LIMITS.addr(), saved_voltage_limit),
    ];

    for (addr, value) in writes {
        if backend.write_register(addr, value).await.is_err() {
            failed = true;
        }
    }

    if failed { Err(()) } else { Ok(()) }
}

async fn stop_openloop_generator<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<(), ImpedanceError> {
    write(backend, OPENLOOP_VELOCITY_TARGET.addr(), 0).await?;
    write(backend, OPENLOOP_VELOCITY_ACTUAL.addr(), 0).await?;
    write(backend, OPENLOOP_ACCELERATION.addr(), 0).await?;
    write(backend, OPENLOOP_PHI.addr(), 0).await?;
    write(
        backend,
        OPENLOOP_MODE.addr(),
        OPENLOOP_PHI_DIRECTION_POSITIVE,
    )
    .await
}

async fn verify_readback<B: ImpedanceBackend>(
    backend: &mut B,
    addr: u8,
    expected: u32,
) -> Result<(), ImpedanceError> {
    let actual = read(backend, addr).await?;
    if actual == expected {
        Ok(())
    } else {
        Err(ImpedanceError::ProfileReadbackMismatch)
    }
}

async fn read<B: ImpedanceBackend>(backend: &mut B, addr: u8) -> Result<u32, ImpedanceError> {
    backend
        .read_register(addr)
        .await
        .map_err(|_| ImpedanceError::BackendIo)
}

async fn write<B: ImpedanceBackend>(
    backend: &mut B,
    addr: u8,
    value: u32,
) -> Result<(), ImpedanceError> {
    backend
        .write_register(addr, value)
        .await
        .map_err(|_| ImpedanceError::BackendIo)
}

fn pack_ud(ud: i16) -> u32 {
    UqUdExt::new().with_ud_ext(ud).with_uq_ext(0).into()
}

fn pack_pidout_limits(limit: u16) -> u32 {
    (u32::from(limit) << 16) | u32::from(limit)
}

fn pidout_ud_limit(value: u32) -> u16 {
    (value & 0xffff) as u16
}

fn decode_adc_sample(t_us: u32, packed: u32) -> ImpedanceSample {
    ImpedanceSample {
        t_us,
        i_ch0: (packed & 0xffff) as u16 as i16,
        i_ch1: (packed >> 16) as u16 as i16,
    }
}
